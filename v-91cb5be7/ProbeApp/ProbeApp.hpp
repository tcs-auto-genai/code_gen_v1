#pragma once

/**
 * @file    ProbeApp.hpp
 * @brief   Central application orchestrator for the Probe data collection and transmission system.
 * @details ProbeApp coordinates all signal acquisition, multi-rate CAN buffering, log data set
 *          assembly, and steady-state/event data transmission to DAQ via the ProbeComm layer.
 *          It enforces deterministic sampling behavior, pre-trigger history guarantees, and
 *          lifecycle management (initialize, run, terminate, shutdown) for the ADAS ECU Probe
 *          application. All shared buffer accesses are mutex-protected for thread safety.
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 */

#include <cstdint>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>

#include "runtime/SimRuntime.hpp"
#include "../ProbeComm/ProbeComm.hpp"
#include "../CanConsumer/CanConsumer.hpp"

/**
 * @defgroup ProbeApplication Probe Application Components
 * @brief Components responsible for signal acquisition, buffering, and data transmission
 *        within the Probe ADAS ECU application.
 */

/// @namespace probe
/// @brief Contains all components of the probe application.
namespace probe {

/**
 * @class   ProbeApp
 * @brief   Central orchestrator of the Probe data collection and transmission application.
 * @details ProbeApp owns all signal RAM buffers, enforces multi-rate periodic sampling
 *          (100 ms and 1000 ms), assembles per-second log data sets, manages steady-state
 *          and event data transmission to DAQ via ProbeComm, and handles the full application
 *          lifecycle (initialize → run → terminate/shutdown). It coordinates with CanConsumer
 *          for shared-memory signal ingestion and with ProbeCommVariant for variant-dependent
 *          feature enablement.
 *
 *          Thread-safety: All ring-buffer accesses are protected by dedicated std::mutex
 *          members. The isRunning_ and shutdownRequested_ flags are std::atomic<bool> to
 *          allow safe cross-thread observation without additional locking.
 *
 *          Lifecycle: Constructed once at startup, HandleInitialize() called after ECU boot,
 *          Run() drives the cyclic loop, HandleTerminate() / HandleShutdown() stop processing.
 *
 * @ingroup ProbeApplication
 * @note    Compliant with ISO 26262 ASIL-B, MISRA C++ guidelines, and C++17 standard.
 *          No dynamic memory allocation after construction. No recursion. All variables
 *          initialized at declaration.
 * @warning Do not call Run() before HandleInitialize() completes. Do not access internal
 *          buffers from outside this class — all access must go through public methods.
 * @invariant probeComm_ is never nullptr after successful construction.
 *            bufferMutex100ms_null, bufferMutex1000ms_null, and logBufferMutex_null must never be
 *            held simultaneously by the same thread to prevent deadlock.
 * @see     ProbeComm, ProbeCommVariant, CanConsumer
 */
class ProbeApp
{
public:

    // =========================================================================
    // Public Constants
    // =========================================================================

    /**
     * @brief   Startup delay in seconds before data collection may begin after IG-ON.
     * @details Set to 10 seconds per system requirement SWR-REQ-01-10-003 to allow
     *          the ADAS ECU and SOME/IP services to reach a stable operating state
     *          before any signal sampling commences.
     */
    static constexpr uint32_t kCollectionStartDelaySeconds{10U};

    /**
     * @brief   Cyclic task period in milliseconds.
     * @details ProbeApp::Run() is invoked from a 10 ms periodic task. All sub-rate
     *          scheduling (100 ms, 1000 ms) is derived from this base period using
     *          the msCounter_ modulo check.
     */
    static constexpr uint32_t kTaskPeriodMs{10U};

    /**
     * @brief   Sampling period for the high-rate CAN buffer in milliseconds.
     * @details CAN signals at this rate are stored into canBuffer100ms_null on every
     *          10th invocation of CollectSharedMemReadSignals().
     */
    static constexpr uint32_t kSampleRate100Ms{100U};

    /**
     * @brief   Sampling period for the low-rate CAN buffer in milliseconds.
     * @details CAN signals at this rate are stored into canBuffer1000ms_null on every
     *          100th invocation of CollectSharedMemReadSignals().
     */
    static constexpr uint32_t kSampleRate1000Ms{1000U};

    /**
     * @brief   Default past-data retention depth in seconds.
     * @details Defines the maximum number of per-second log data sets retained in
     *          logDataSetBuffer_null before the oldest entry is evicted. Derived from
     *          the 10-second pre-trigger history requirement.
     */
    static constexpr uint32_t kDefaultPastDataRetentionCount{10U};

    // =========================================================================
    // Constructor
    // =========================================================================

    /**
     * @brief   Constructs ProbeApp with references to communication, variant, and CAN consumer.
     * @details Initializes all internal RAM buffers sized according to sampling rates and
     *          pre-trigger retention depth. Reads and applies variant configuration via
     *          ProbeCommVariant. Stores non-owning observer pointers to the injected
     *          dependencies whose lifetimes are guaranteed to exceed this object's lifetime
     *          by the caller (Main). Throws std::runtime_error if any mandatory dependency
     *          is nullptr or if buffer initialization fails.
     *
     *          Initialization sequence:
     *          1. Validate all input pointers.
     *          2. Store observer pointers.
     *          3. Resize canBuffer100ms_null, canBuffer1000ms_null, logDataSetBuffer_null,
     *             steadyStateDataBuffer100ms_null, steadyStateDataBuffer1000ms_null,
     *             eventSendingBuffer_null, imageDataBuffer_null to their configured capacities.
     *          4. Initialize all atomic state flags to their default values.
     *          5. Set startupTimeSec_ to the current monotonic clock epoch.
     *
     * @param[in] comm      Non-owning pointer to the ProbeComm communication handler.
     *                      Must not be nullptr. Lifetime must exceed ProbeApp lifetime.
     * @param[in] variant   Non-owning pointer to the ProbeCommVariant configuration manager.
     *                      Must not be nullptr. Lifetime must exceed ProbeApp lifetime.
     * @param[in] canCons   Non-owning pointer to the CanConsumer shared-memory signal reader.
     *                      Must not be nullptr. Lifetime must exceed ProbeApp lifetime.
     * @throws  std::runtime_error  If any pointer argument is nullptr or buffer allocation fails.
     * @throws  std::invalid_argument  If variant configuration returns an inconsistent state.
     * @pre     All three pointer arguments must be valid, fully constructed objects.
     * @post    All internal buffers are allocated and zero-initialized.
     *          isRunning_ == false, shutdownRequested_ == false, collectionStarted_ == false.
     *          msCounter_ == 0.
     * @note    Called by: Main. Requirement: SWR-REQ-01-01-002, SWR-REQ-01-01-003,
     *          SWR-REQ-01-02-001.
     * @warning Do not call any other ProbeApp method before the constructor returns.
     * @requirements SWR-REQ-01-01-002;SWR-REQ-01-01-003;SWR-REQ-01-02-001
     * @rationale Dependency injection via constructor parameters enables unit testing
     *            and avoids hidden global state. Non-owning raw pointers are used here
     *            because lifetime is strictly controlled by Main (MISRA M1 observer pattern).
     * @see     ProbeComm, ProbeCommVariant, CanConsumer
     */
    explicit ProbeApp(ProbeComm* comm,
                      ProbeCommVariant* variant,
                      CanConsumer* canCons);
    explicit ProbeApp(runtime::SimRuntime&) : ProbeApp(nullptr, nullptr, nullptr) {}

    /**
     * @brief   Destructor. Releases no heap resources (RAII — all members are value types).
     * @details Ensures that isRunning_ is set to false before destruction. Does not call
     *          HandleShutdown() — the caller must invoke HandleShutdown() or HandleTerminate()
     *          before destroying this object.
     * @throws  None.
     * @note    noexcept — destructor must never throw per C++ standard.
     */
    ~ProbeApp() noexcept;

    // Prevent copy and move to enforce single-instance ownership semantics.

    /// @brief Deleted copy constructor — ProbeApp is non-copyable.
    ProbeApp(const ProbeApp&) = delete;

    /// @brief Deleted copy assignment — ProbeApp is non-copyable.
    ProbeApp& operator=(const ProbeApp&) = delete;

    /// @brief Deleted move constructor — ProbeApp is non-movable.
    ProbeApp(ProbeApp&&) = delete;

    /// @brief Deleted move assignment — ProbeApp is non-movable.
    ProbeApp& operator=(ProbeApp&&) = delete;

    // =========================================================================
    // Lifecycle Methods
    // =========================================================================

    /**
     * @brief   Starts ProbeApp operation after ADAS ECU startup and initiates SOME/IP
     *          communication with DAQ.
     * @details Invoked once by Main after the Adaptive AUTOSAR execution state has been
     *          reported as kRunning. Triggers service discovery and service offering via
     *          ProbeComm (FindService* and OfferService* calls). Sets isRunning_ to true
     *          and records startupTimeSec_ for the 10-second collection start delay check.
     *          Does not start data collection — that is gated by CheckCollectionStartDelay().
     *
     *          Initialization sequence:
     *          1. Call ProbeComm::FindServiceDaqContinual() and all other FindService* methods.
     *          2. Call ProbeComm::OfferServiceProbeEventStt() and OfferServiceProbeTrigger().
     *          3. Set isDaqCommEstablished_ = false (updated asynchronously on service found).
     *          4. Record startupTimeSec_.
     *          5. Set isRunning_ = true.
     *
     * @throws  std::runtime_error  If probeComm_ is nullptr (defensive check).
     * @pre     Constructor has completed successfully. isRunning_ == false.
     * @post    isRunning_ == true. SOME/IP service discovery has been initiated.
     *          startupTimeSec_ holds the monotonic timestamp of initialization.
     * @note    Requirements: SWR-REQ-01-10-001, SWR-REQ-01-10-002, SWR-REQ-01-03-001,
     *          SWR-REQ-03-16-001, SWR-REQ-03-16-002.
     * @warning Must be called exactly once before Run(). Calling twice results in
     *          undefined behavior (duplicate service registration).
     * @requirements SWR-REQ-01-10-001;SWR-REQ-01-10-002;SWR-REQ-01-03-001;SWR-REQ-03-16-001;SWR-REQ-03-16-002
     * @rationale Separating initialization from construction allows the Adaptive AUTOSAR
     *            framework to control the exact moment service discovery begins.
     * @see     ProbeComm::FindServiceDaqContinual, ProbeComm::OfferServiceProbeEventStt
     */
    void HandleInitialize();

    /**
     * @brief   Runs the cyclic data collection and transmission processing loop.
     * @details Called periodically from the 10 ms task context. On each invocation:
     *          1. Checks shutdownRequested_ — returns immediately if true.
     *          2. Increments msCounter_.
     *          3. Calls CollectSharedMemReadSignals() to perform multi-rate CAN sampling.
     *          4. Calls BuildLogDataSetPerSecond() on the 1000 ms boundary.
     *          5. Calls DeleteOldestLogDataSet() if pastDataRetentionCount_ is exceeded.
     *          6. Calls SendRegularData() to transmit steady-state data to DAQ.
     *          7. Calls DeleteTransmittedSteadyStateData() after transmission.
     *
     *          All sub-rate scheduling is derived from msCounter_ modulo arithmetic.
     *          This function is non-blocking and must complete within the 10 ms task budget.
     *
     * @throws  None (all internal errors handled via explicit status checks).
     * @pre     HandleInitialize() has been called. isRunning_ == true.
     * @post    msCounter_ has been incremented. Buffers updated according to elapsed time.
     * @note    Requirements: SWR-REQ-01-001, SWR-REQ-01-002, SWR-REQ-01-005,
     *          SWR-REQ-01-06-001, SWR-REQ-01-10-003, SWR-REQ-03-16-003.
     * @warning Must not be called after HandleTerminate() or HandleShutdown().
     *          Must complete within 10 ms to avoid task overrun.
     * @requirements SWR-REQ-01-001;SWR-REQ-01-002;SWR-REQ-01-005;SWR-REQ-01-06-001;SWR-REQ-01-10-003;SWR-REQ-03-16-003
     * @rationale Single-entry cyclic design avoids thread synchronization overhead and
     *            ensures deterministic execution order per ISO 26262 ASIL-B requirements.
     * @see     CollectSharedMemReadSignals, BuildLogDataSetPerSecond, SendRegularData
     */
    void Run();

    /**
     * @brief   Stops all data collection and transmission processing on termination request.
     * @details Sets isRunning_ = false and shutdownRequested_ = true. Subsequent calls to
     *          Run() will return immediately without processing. Does NOT clear RAM buffers —
     *          use HandleShutdown() for full data purge. Intended for graceful termination
     *          where in-flight data may still be valid.
     *
     * @throws  None.
     * @pre     HandleInitialize() has been called previously.
     * @post    isRunning_ == false. shutdownRequested_ == true.
     *          No further data collection or transmission will occur.
     * @note    Requirements: SWR-REQ-01-09-001, SWR-REQ-03-13-001.
     * @warning After this call, no data transmission will occur. Buffers retain their
     *          last state until HandleShutdown() or destructor is called.
     * @requirements SWR-REQ-01-09-001;SWR-REQ-03-13-001
     * @rationale Separating termination (stop processing) from shutdown (purge data)
     *            allows the framework to distinguish graceful stop from emergency stop.
     * @see     HandleShutdown
     */
    void HandleTerminate();

    /**
     * @brief   Stops all processing and deletes all temporary data including mid-transmission
     *          data on ADAS ECU shutdown.
     * @details Performs a full stop-and-purge sequence:
     *          1. Sets isRunning_ = false, shutdownRequested_ = true.
     *          2. Calls ClearAllTemporaryData() to wipe all RAM buffers.
     *          3. Calls EnforceStopOnRamCleanupFailure() if ClearAllTemporaryData() indicates
     *             a failure condition.
     *          This function must be called on IG-OFF to comply with data privacy requirements
     *          (all retained data must be cleared on power-down).
     *
     * @throws  None (errors handled internally via EnforceStopOnRamCleanupFailure).
     * @pre     No precondition — safe to call at any lifecycle stage.
     * @post    isRunning_ == false. shutdownRequested_ == true.
     *          All RAM buffers are empty. No residual data remains in any buffer.
     * @note    Requirements: SWR-REQ-01-09-001, SWR-REQ-01-09-002, SWR-REQ-03-13-001,
     *          SWR-REQ-03-13-002, SWR-REQ-03-13-003, SWR-REQ-01-08-003.
     * @warning This call is irreversible. After HandleShutdown(), the object must be
     *          destroyed and re-constructed for any further use.
     * @requirements SWR-REQ-01-09-001;SWR-REQ-01-09-002;SWR-REQ-03-13-001;SWR-REQ-03-13-002;SWR-REQ-03-13-003;SWR-REQ-01-08-003
     * @rationale Full data purge on shutdown is mandatory for GDPR/privacy compliance and
     *            ISO 26262 safe-state requirements.
     * @see     ClearAllTemporaryData, EnforceStopOnRamCleanupFailure
     */
    void HandleShutdown();

    // =========================================================================
    // Signal Collection Methods
    // =========================================================================

    /**
     * @brief   Reads CAN and internal signals from shared memory at configured sampling rates.
     * @details Implements multi-rate scheduling using msCounter_ modulo arithmetic:
     *          - Every kSampleRate100Ms ms  → calls StoreCanData100ms().
     *          - Every kSampleRate1000Ms ms → calls StoreCanData1000ms() and
     *            StoreEventSignalData().
     *          Mutex protection is applied per buffer to avoid data races. This function
     *          is the single entry point for all shared-memory signal ingestion.
     *
     * @throws  None.
     * @pre     isRunning_ == true. canConsumer_ != nullptr.
     * @post    canBuffer100ms_null and/or canBuffer1000ms_null updated with latest timestamped
     *          signal values according to the elapsed msCounter_.
     * @note    Requirements: SWR-REQ-01-01-001, SWR-REQ-01-001, SWR-REQ-01-002,
     *          SWR-REQ-01-003.
     * @warning Must complete within the 10 ms task budget. Shared memory reads must be
     *          non-blocking.
     * @requirements SWR-REQ-01-01-001;SWR-REQ-01-001;SWR-REQ-01-002;SWR-REQ-01-003
     * @rationale Centralizing multi-rate dispatch in one function simplifies scheduling
     *            and avoids duplicated modulo logic across callers.
     * @see     StoreCanData100ms, StoreCanData1000ms, StoreEventSignalData
     */
    void CollectSharedMemReadSignals();

    /**
     * @brief   Reads CAN signals at 100 ms rate and stores them into the 100 ms ring buffer
     *          with timestamp.
     * @details Acquires bufferMutex100ms_null, reads the latest 100 ms CAN signal set from
     *          canConsumer_, pops the oldest entry from canBuffer100ms_null if at capacity,
     *          then pushes the new timestamped entry. Timestamp is captured in milliseconds
     *          from the monotonic clock at the moment of read.
     *
     * @throws  None.
     * @pre     canConsumer_ != nullptr. bufferMutex100ms_null is not held by the calling thread.
     * @post    canBuffer100ms_null contains the latest 100 ms sample at the back.
     *          If buffer was at capacity, the oldest entry has been removed from the front.
     * @note    Requirements: SWR-REQ-01-001, SWR-REQ-01-01-001, SWR-REQ-01-01-002,
     *          SWR-REQ-01-003.
     * @warning Acquires bufferMutex100ms_null — do not call while holding this mutex.
     * @requirements SWR-REQ-01-001;SWR-REQ-01-01-001;SWR-REQ-01-01-002;SWR-REQ-01-003
     * @rationale Ring-buffer (deque with bounded pop-front) provides O(1) oldest-entry
     *            eviction without dynamic allocation.
     * @see     CollectSharedMemReadSignals, CanConsumer
     */
    void StoreCanData100ms();

    /**
     * @brief   Reads CAN signals at 1000 ms rate and stores them into the 1000 ms ring buffer
     *          with timestamp.
     * @details Acquires bufferMutex1000ms_null, reads the latest 1000 ms CAN signal set from
     *          canConsumer_, pops the oldest entry from canBuffer1000ms_null if at capacity,
     *          then pushes the new timestamped entry. Timestamp is captured in milliseconds
     *          from the monotonic clock at the moment of read.
     *
     * @throws  None.
     * @pre     canConsumer_ != nullptr. bufferMutex1000ms_null is not held by the calling thread.
     * @post    canBuffer1000ms_null contains the latest 1000 ms sample at the back.
     *          If buffer was at capacity, the oldest entry has been removed from the front.
     * @note    Requirements: SWR-REQ-01-002, SWR-REQ-01-01-001, SWR-REQ-01-01-002,
     *          SWR-REQ-01-003.
     * @warning Acquires bufferMutex1000ms_null — do not call while holding this mutex.
     * @requirements SWR-REQ-01-002;SWR-REQ-01-01-001;SWR-REQ-01-01-002;SWR-REQ-01-003
     * @rationale Separate mutex per rate prevents the 100 ms path from blocking the
     *            1000 ms path and vice versa.
     * @see     CollectSharedMemReadSignals, CanConsumer
     */
    void StoreCanData1000ms();

    /**
     * @brief   Collects event-based signals with timestamps, signal IDs, and data size
     *          information.
     * @details On each 1000 ms boundary, reads all pending event-based signals from
     *          shared memory via canConsumer_, attaches monotonic timestamps and signal
     *          metadata (ID, data size), and appends them to eventSendingBuffer_null.
     *          Acquires logBufferMutex_null during the append operation.
     *
     * @throws  None.
     * @pre     canConsumer_ != nullptr. logBufferMutex_null is not held by the calling thread.
     * @post    eventSendingBuffer_null contains all newly collected event signal entries
     *          with complete metadata.
     * @note    Requirements: SWR-REQ-03-01-002, SWR-REQ-03-01-004, SWR-REQ-03-01-005,
     *          SWR-REQ-03-01-006, SWR-REQ-03-01-007.
     * @warning Acquires logBufferMutex_null — do not call while holding this mutex.
     * @requirements SWR-REQ-03-01-002;SWR-REQ-03-01-004;SWR-REQ-03-01-005;SWR-REQ-03-01-006;SWR-REQ-03-01-007
     * @rationale Event signals require richer metadata (ID, size) than CAN signals,
     *            justifying a separate collection path.
     * @see     CollectSharedMemReadSignals, BuildLogDataSetPerSecond
     */
    void StoreEventSignalData();

    // =========================================================================
    // Log Data Set Management
    // =========================================================================

    /**
     * @brief   Assembles per-second log data sets per the collection data retention layout
     *          specification.
     * @details Called on every 1000 ms boundary. Acquires logBufferMutex_null, then packages
     *          the signals accumulated in canBuffer100ms_null, canBuffer1000ms_null, and
     *          eventSendingBuffer_null during the past second into a single serialized log data
     *          set entry. Appends the assembled entry to logDataSetBuffer_null. Clears the
     *          per-second accumulation staging area after packaging.
     *
     * @throws  None.
     * @pre     logBufferMutex_null is not held by the calling thread.
     *          canBuffer100ms_null and canBuffer1000ms_null contain at least one entry each.
     * @post    logDataSetBuffer_null has one new entry appended at the back.
     *          Staging buffers for the current second are cleared.
     * @note    Requirements: SWR-REQ-03-01-008, SWR-REQ-03-001.
     * @warning Acquires logBufferMutex_null — do not call while holding this mutex.
     *          If called more frequently than 1000 ms, duplicate entries may be created.
     * @requirements SWR-REQ-03-01-008;SWR-REQ-03-001
     * @rationale Per-second packaging aligns with the DAQ upload granularity and
     *            simplifies pre-trigger history replay.
     * @see     DeleteOldestLogDataSet, StoreCanData100ms, StoreCanData1000ms
     */
    void BuildLogDataSetPerSecond();

    /**
     * @brief   Removes the oldest log data set when the retention count is exceeded.
     * @details Acquires logBufferMutex_null and checks whether logDataSetBuffer_null.size()
     *          exceeds pastDataRetentionCount_. If so, pops the front (oldest) entry.
     *          This enforces the rolling retention window (default 10 seconds).
     *
     * @throws  None.
     * @pre     logBufferMutex_null is not held by the calling thread.
     * @post    If logDataSetBuffer_null.size() > pastDataRetentionCount_, the oldest entry
     *          has been removed. Otherwise, logDataSetBuffer_null is unchanged.
     * @note    Requirements: SWR-REQ-03-01-009, SWR-REQ-03-001.
     * @warning Acquires logBufferMutex_null — do not call while holding this mutex.
     * @requirements SWR-REQ-03-01-009;SWR-REQ-03-001
     * @rationale Bounded retention prevents unbounded RAM growth while guaranteeing
     *            the configured pre-trigger history depth is always available.
     * @see     BuildLogDataSetPerSecond
     */
    void DeleteOldestLogDataSet();

    /**
     * @brief   Sends zero-filled data blocks for pre-trigger time window where log data is
     *          not yet available after IG-ON.
     * @details When an event trigger arrives before the full pre-trigger history has been
     *          accumulated (i.e., within the first kDefaultPastDataRetentionCount seconds
     *          after IG-ON), this function generates zero-filled log data set blocks for
     *          the missing seconds in the range [startTime, first_available_log_time).
     *          Each zero block conforms to the standard log data set layout.
     *
     * @param[in] startTime  The requested pre-trigger start time in seconds relative to
     *                       IG-ON. Valid range: [INT32_MIN, 0]. Negative values indicate
     *                       seconds before the trigger event.
     * @throws  None.
     * @pre     startTime <= 0. logDataSetBuffer_null may be empty or partially filled.
     * @post    Zero-filled placeholder entries have been queued for transmission for all
     *          seconds in [startTime, first_available_second).
     * @note    Called by: ProbeApp. Requirements: SWR-REQ-03-002, SWR-REQ-03-01-011,
     *          SWR-REQ-03-07-007.
     * @warning Zero-filled blocks must be clearly marked in the payload header to allow
     *          the DAQ server to distinguish them from real data.
     * @requirements SWR-REQ-03-002;SWR-REQ-03-01-011;SWR-REQ-03-07-007
     * @rationale Without zero-fill, the DAQ server would receive an incomplete time series
     *            and could misinterpret the pre-trigger window length.
     * @see     BuildLogDataSetPerSecond, ProbeComm::FillHeaderEventDataType002
     */
    void FillZeroDataForMissingPreTrigger(int32_t startTime);

    // =========================================================================
    // Buffer Management Methods
    // =========================================================================

    /**
     * @brief   Deletes steady-state data from RAM after successful transmission to DAQ.
     * @details Acquires bufferMutex100ms_null and bufferMutex1000ms_null (in that order to prevent
     *          deadlock) and clears steadyStateDataBuffer100ms_null and
     *          steadyStateDataBuffer1000ms_null. Called after ProbeComm confirms successful
     *          fire-and-forget delivery.
     *
     * @throws  None.
     * @pre     Neither bufferMutex100ms_null nor bufferMutex1000ms_null is held by the calling thread.
     * @post    steadyStateDataBuffer100ms_null and steadyStateDataBuffer1000ms_null are empty.
     * @note    Requirements: SWR-REQ-01-006, SWR-REQ-01-07-001.
     * @warning Mutexes are acquired in a fixed order (100ms before 1000ms) — all callers
     *          must respect this order to prevent deadlock.
     * @requirements SWR-REQ-01-006;SWR-REQ-01-07-001
     * @rationale Clearing after confirmed transmission prevents re-sending stale data
     *            while ensuring no data is lost before delivery is confirmed.
     * @see     SendRegularData, DeleteTransmittedSteadyStateData
     */
    void ClearSteadyStateBuffer();

    /**
     * @brief   Clears all RAM buffers including regular, event, image, and mid-transmission
     *          data.
     * @details Acquires all three buffer mutexes (bufferMutex100ms_null, bufferMutex1000ms_null,
     *          logBufferMutex_null — in that fixed order) and clears:
     *          canBuffer100ms_null, canBuffer1000ms_null, logDataSetBuffer_null,
     *          steadyStateDataBuffer100ms_null, steadyStateDataBuffer1000ms_null,
     *          eventSendingBuffer_null, imageDataBuffer_null.
     *          Called exclusively from HandleShutdown() on IG-OFF.
     *
     * @throws  None.
     * @pre     No buffer mutex is held by the calling thread.
     * @post    All seven RAM buffers are empty. No residual data remains.
     * @note    Requirements: SWR-REQ-01-09-002, SWR-REQ-03-13-002.
     * @warning This operation is irreversible. All buffered data is permanently lost.
     *          Mutexes are acquired in fixed order to prevent deadlock.
     * @requirements SWR-REQ-01-09-002;SWR-REQ-03-13-002
     * @rationale Full purge on shutdown is required for data privacy compliance and
     *            to prevent stale data from persisting across drive cycles.
     * @see     HandleShutdown, EnforceStopOnRamCleanupFailure
     */
    void ClearAllTemporaryData();

    // =========================================================================
    // State Check Methods
    // =========================================================================

    /**
     * @brief   Verifies that data collection can start after the 10-second delay from IG-ON.
     * @details Computes the elapsed time since startupTimeSec_ using the monotonic clock.
     *          Returns true if the elapsed time is >= kCollectionStartDelaySeconds and
     *          isDaqCommEstablished_ == true. Sets collectionStarted_ = true on the first
     *          successful check. Subsequent calls return true immediately once
     *          collectionStarted_ == true.
     *
     * @return  bool  Indicates whether data collection is permitted to start.
     * @retval  true   The 10-second startup delay has elapsed and DAQ communication is
     *                 established. Data collection may proceed.
     * @retval  false  The startup delay has not yet elapsed, or DAQ communication is not
     *                 yet established. Data collection must not start.
     * @throws  None.
     * @pre     HandleInitialize() has been called. startupTimeSec_ is valid.
     * @post    If returning true for the first time, collectionStarted_ == true.
     * @note    Called by: ProbeApp. Requirements: SWR-REQ-01-10-003, SWR-REQ-03-01-001,
     *          SWR-REQ-03-16-003.
     * @warning Do not start any signal sampling before this function returns true.
     * @requirements SWR-REQ-01-10-003;SWR-REQ-03-01-001;SWR-REQ-03-16-003
     * @rationale The 10-second delay ensures SOME/IP services are fully discovered and
     *            the ADAS ECU has reached a stable operating state before data is recorded.
     * @see     HandleInitialize, Run
     */
    bool CheckCollectionStartDelay();

    /**
     * @brief   Halts application if RAM cleanup fails to prevent inconsistent state.
     * @details If ClearAllTemporaryData() or DeleteTransmittedSteadyStateData() detects
     *          a buffer inconsistency (e.g., non-empty buffer after clear attempt), this
     *          function sets isRunning_ = false and shutdownRequested_ = true to prevent
     *          further processing in an undefined state. Logs the failure condition via
     *          std::cerr for diagnostic purposes.
     *
     * @throws  None.
     * @pre     No precondition — may be called from any state.
     * @post    isRunning_ == false. shutdownRequested_ == true.
     *          No further data collection or transmission will occur.
     * @note    Requirements: SWR-REQ-01-08-003, SWR-REQ-03-13-003.
     * @warning This is a safety-critical stop function. Once called, the application
     *          cannot resume without a full restart.
     * @requirements SWR-REQ-01-08-003;SWR-REQ-03-13-003
     * @rationale Continuing operation with a corrupted buffer state could result in
     *            transmission of invalid data, violating ISO 26262 ASIL-B integrity
     *            requirements.
     * @see     ClearAllTemporaryData, HandleRamDeletionFailure
     */
    void EnforceStopOnRamCleanupFailure();

    // =========================================================================
    // Data Transmission Methods
    // =========================================================================

    /**
     * @brief   Transmits steady-state sampled data to DAQ via SOME/IP UDP fire-and-forget
     *          at 100 ms and 1000 ms cycles.
     * @details On each invocation:
     *          1. Checks isDaqCommEstablished_ — returns without sending if false.
     *          2. On the 100 ms boundary: serializes steadyStateDataBuffer100ms_null into a
     *             payload and calls ProbeComm::SendContinualShortDataCyclic100ms().
     *          3. On the 1000 ms boundary: serializes steadyStateDataBuffer1000ms_null into a
     *             payload and calls ProbeComm::SendContinualLongDataCyclic1000ms().
     *          4. Calls ProbeComm::SuppressDataBeforeDaqEstablished() as a guard.
     *
     * @throws  None.
     * @pre     isRunning_ == true. isDaqCommEstablished_ == true.
     *          steadyStateDataBuffer100ms_null and/or steadyStateDataBuffer1000ms_null are non-empty.
     * @post    Steady-state data has been handed to ProbeComm for transmission.
     *          Buffer state is unchanged — caller must invoke DeleteTransmittedSteadyStateData()
     *          after this call.
     * @note    Requirements: SWR-REQ-01-005, SWR-REQ-01-06-001, SWR-REQ-01-06-002,
     *          SWR-REQ-01-08-001, SWR-REQ-01-08-002.
     * @warning UDP fire-and-forget provides no delivery guarantee. Data loss is acceptable
     *          per system design for steady-state (non-event) data.
     * @requirements SWR-REQ-01-005;SWR-REQ-01-06-001;SWR-REQ-01-06-002;SWR-REQ-01-08-001;SWR-REQ-01-08-002
     * @rationale Fire-and-forget UDP minimizes transmission latency for high-frequency
     *            steady-state data where occasional loss is tolerable.
     * @see     ProbeComm::SendContinualShortDataCyclic100ms,
     *          ProbeComm::SendContinualLongDataCyclic1000ms,
     *          DeleteTransmittedSteadyStateData
     */
    void SendRegularData();

    /**
     * @brief   Removes transmitted steady-state data from RAM buffer after successful or
     *          failed transmission.
     * @details Acquires bufferMutex100ms_null and bufferMutex1000ms_null (in fixed order) and
     *          clears the entries from steadyStateDataBuffer100ms_null and
     *          steadyStateDataBuffer1000ms_null that were included in the most recent
     *          SendRegularData() call. If buffer deletion fails (buffer non-empty after
     *          clear), calls HandleRamDeletionFailure().
     *
     * @throws  None.
     * @pre     SendRegularData() has been called in the current cycle.
     *          Neither bufferMutex100ms_null nor bufferMutex1000ms_null is held by the calling thread.
     * @post    Transmitted entries have been removed from steadyStateDataBuffer100ms_null
     *          and steadyStateDataBuffer1000ms_null.
     * @note    Requirements: SWR-REQ-01-006, SWR-REQ-01-07-001, SWR-REQ-01-08-002.
     * @warning Mutexes acquired in fixed order (100ms before 1000ms). If HandleRamDeletionFailure()
     *          is triggered, the application will stop processing.
     * @requirements SWR-REQ-01-006;SWR-REQ-01-07-001;SWR-REQ-01-08-002
     * @rationale Deleting after both success and failure prevents unbounded buffer growth
     *            regardless of transmission outcome.
     * @see     SendRegularData, HandleRamDeletionFailure, ClearSteadyStateBuffer
     */
    void DeleteTransmittedSteadyStateData();

    /**
     * @brief   Initiates controlled stop if RAM buffer deletion fails after communication
     *          error.
     * @details Called by DeleteTransmittedSteadyStateData() when a buffer clear operation
     *          does not result in an empty buffer. Sets isRunning_ = false and
     *          shutdownRequested_ = true. Outputs a diagnostic message to std::cerr
     *          identifying the failure location. Delegates final stop enforcement to
     *          EnforceStopOnRamCleanupFailure().
     *
     * @throws  None.
     * @pre     No precondition — may be called from any execution state.
     * @post    isRunning_ == false. shutdownRequested_ == true.
     *          EnforceStopOnRamCleanupFailure() has been called.
     * @note    Requirements: SWR-REQ-01-08-003.
     * @warning This is a non-recoverable error path. The application must be restarted
     *          after this function is called.
     * @requirements SWR-REQ-01-08-003
     * @rationale A RAM deletion failure indicates a potential memory integrity issue.
     *            Continuing operation risks transmitting corrupted or duplicate data,
     *            which violates ASIL-B data integrity requirements.
     * @see     DeleteTransmittedSteadyStateData, EnforceStopOnRamCleanupFailure
     */
    void HandleRamDeletionFailure();

private:

    // =========================================================================
    // Private Member Variables
    // =========================================================================

    // --- Dependency Pointers (non-owning observers) ---

    ProbeComm* probeComm_{nullptr};
    ///< @brief Non-owning observer pointer to the ProbeComm communication handler.
    ///         Must not be nullptr after construction. Lifetime guaranteed by Main.

    ProbeCommVariant* probeCommVariant_{nullptr};
    ///< @brief Non-owning observer pointer to the ProbeCommVariant configuration manager.
    ///         Used to query variant-dependent feature enablement flags.

    CanConsumer* canConsumer_{nullptr};
    ///< @brief Non-owning observer pointer to the CanConsumer shared-memory signal reader.
    ///         Provides timestamped CAN signal reads at 100 ms and 1000 ms rates.

    // --- CAN Ring Buffers ---

    std::deque<std::vector<uint8_t>> canBuffer100ms_null;
    ///< @brief Ring buffer holding timestamped CAN signal snapshots sampled at 100 ms.
    ///         Bounded to (kSampleRate1000Ms / kSampleRate100Ms) * kDefaultPastDataRetentionCount
    ///         entries. Protected by bufferMutex100ms_null.

    std::deque<std::vector<uint8_t>> canBuffer1000ms_null;
    ///< @brief Ring buffer holding timestamped CAN signal snapshots sampled at 1000 ms.
    ///         Bounded to kDefaultPastDataRetentionCount entries.
    ///         Protected by bufferMutex1000ms_null.

    // --- Log Data Set Buffer ---

    std::deque<std::vector<uint8_t>> logDataSetBuffer_null;
    ///< @brief Rolling buffer of per-second assembled log data sets.
    ///         Maximum depth: pastDataRetentionCount_ entries (default 10).
    ///         Protected by logBufferMutex_null.

    // --- Steady-State Transmission Buffers ---

    std::vector<uint8_t> steadyStateDataBuffer100ms_null;
    ///< @brief Serialized steady-state payload for the 100 ms transmission cycle.
    ///         Populated by SendRegularData(), cleared by DeleteTransmittedSteadyStateData().
    ///         Protected by bufferMutex100ms_null.

    std::vector<uint8_t> steadyStateDataBuffer1000ms_null;
    ///< @brief Serialized steady-state payload for the 1000 ms transmission cycle.
    ///         Populated by SendRegularData(), cleared by DeleteTransmittedSteadyStateData().
    ///         Protected by bufferMutex1000ms_null.

    // --- Event and Image Buffers ---

    std::vector<std::vector<uint8_t>> eventSendingBuffer_null;
    ///< @brief Buffer holding event-based signal data sets pending transmission.
    ///         Entries are appended by StoreEventSignalData() and consumed by ProbeComm
    ///         event transmission functions. Protected by logBufferMutex_null.

    std::vector<uint8_t> imageDataBuffer_null;
    ///< @brief Buffer holding raw camera image data pending transmission.
    ///         Populated by ProbeComm image reception callbacks.
    ///         Protected by logBufferMutex_null.

    // --- Mutexes ---

    std::mutex bufferMutex100ms_null;
    ///< @brief Mutex protecting canBuffer100ms_null and steadyStateDataBuffer100ms_null.
    ///         Always acquired before bufferMutex1000ms_null to prevent deadlock.

    std::mutex bufferMutex1000ms_null;
    ///< @brief Mutex protecting canBuffer1000ms_null and steadyStateDataBuffer1000ms_null.
    ///         Always acquired after bufferMutex100ms_null when both are needed.

    std::mutex logBufferMutex_null;
    ///< @brief Mutex protecting logDataSetBuffer_null, eventSendingBuffer_null, and imageDataBuffer_null.
    ///         Always acquired last (after both rate mutexes) when all three are needed.

    // --- Timing and Scheduling ---

    uint32_t msCounter_{0U};
    ///< @brief Millisecond counter incremented on every Run() invocation.
    ///         Used for modulo-based sub-rate scheduling (100 ms, 1000 ms).
    ///         Wraps at UINT32_MAX — callers must handle wrap-around gracefully.

    uint32_t startupTimeSec_{0U};
    ///< @brief Monotonic timestamp (seconds) recorded at HandleInitialize() completion.
    ///         Used by CheckCollectionStartDelay() to enforce the 10-second startup delay.
    ///         Units: seconds since application start (monotonic clock epoch).

    // --- Retention Configuration ---

    uint32_t pastDataRetentionCount_{kDefaultPastDataRetentionCount};
    ///< @brief Maximum number of per-second log data sets retained in logDataSetBuffer_null.
    ///         Default: kDefaultPastDataRetentionCount (10). Range: [1, UINT32_MAX].
    ///         Configurable via variant settings read during construction.

    // --- Application State Flags ---

    std::atomic<bool> isRunning_{false};
    ///< @brief Indicates whether the application is in the active running state.
    ///         Set to true by HandleInitialize(), false by HandleTerminate()/HandleShutdown().
    ///         Atomic to allow safe observation from monitoring contexts.

    std::atomic<bool> isDaqCommEstablished_{false};
    ///< @brief Indicates whether SOME/IP DAQ communication has been successfully established.
    ///         Set to true when service discovery callbacks confirm DAQ service availability.
    ///         Guards SendRegularData() from transmitting before the session is ready.

    std::atomic<bool> shutdownRequested_{false};
    ///< @brief Indicates that a shutdown or termination has been requested.
    ///         Set to true by HandleTerminate() or HandleShutdown().
    ///         Causes Run() to return immediately without processing.

    std::atomic<bool> collectionStarted_{false};
    ///< @brief Indicates that the 10-second startup delay has elapsed and data collection
    ///         has been authorized. Set to true by CheckCollectionStartDelay() on first
    ///         successful check. Once true, never reset to false during a drive cycle.

}; // class ProbeApp

} // namespace probe