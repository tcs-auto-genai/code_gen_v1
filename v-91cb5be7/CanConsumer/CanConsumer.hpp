#pragma once

/**
 * @file    CanConsumer.hpp
 * @brief   Declares the CanConsumer class for reading CAN RX/TX signals from shared memory.
 * @details CanConsumer is a pure data-acquisition layer responsible for ingesting CAN signals
 *          from a POSIX shared memory region at multiple periodicities (100 ms and 1000 ms).
 *          It provides timestamped raw signal vectors to the ProbeApp for downstream buffering
 *          and transmission. The class is designed as a single-responsibility, non-allocating
 *          component compliant with ISO 26262 ASIL-B and MISRA C++ guidelines.
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 */

#include <cstdint>
#include <cstddef>
#include <vector>
#include <unordered_map>
#include <string>
#include <chrono>

#include <map>
#include <string>


/**
 * @defgroup CanAcquisition CAN Data Acquisition Components
 * @brief Components responsible for reading and buffering CAN signals from shared memory.
 */

/// @namespace probe
/// @brief Contains all components of the probe application.
namespace probe {

/**
 * @class   CanConsumer
 * @brief   Reads CAN RX/TX signals from shared memory at configurable periodicities.
 * @details CanConsumer acts as the sole data-acquisition boundary between the shared memory
 *          IPC region (written by the CAN writer process) and the ProbeApp signal buffers.
 *          It exposes periodic read functions for 100 ms and 1000 ms cycles, on-request
 *          per-signal readers for RX and TX directions, a monotonic timestamp collector,
 *          and a signal-data validator. All heap allocations use std::vector with value
 *          semantics; no dynamic memory is retained between calls. The class is not
 *          thread-safe by itself — the caller (ProbeApp) is responsible for serialising
 *          access from a single periodic task context.
 *
 * @ingroup CanAcquisition
 *
 * @note    Compliant with ISO 26262 ASIL-B, MISRA C++ 2008, and C++14/17 coding standards.
 *          No raw owning pointers, no recursion, no dynamic memory retained as state.
 *          All fixed-width integer types are used where bit-width matters.
 *
 * @warning Do NOT call any method before a valid non-null shared memory handle has been
 *          supplied to the constructor. Passing a null handle results in all read
 *          operations returning empty vectors.
 *
 * @invariant sharedMemHandle_ is either nullptr (invalid/uninitialised) or points to a
 *            mapped shared memory region whose lifetime exceeds that of this object.
 *            timestampSec_ and timestampMsec_ always reflect the most recent successful
 *            call to CollectTimestamp().
 *
 * @see     ProbeApp
 */
class CanConsumer
{
public:

    // =========================================================================
    // Constants
    // =========================================================================

    /**
     * @brief   Maximum number of bytes expected in a single CAN signal payload.
     * @details CAN frames carry at most 8 data bytes (classic CAN). This constant
     *          is used as an upper-bound guard inside ValidateSignalData() to reject
     *          oversized payloads that would indicate memory corruption.
     *          Value: 8 bytes — matches the ISO 11898 CAN frame data-length limit.
     */
    static constexpr std::size_t kMaxCanSignalBytes{8U};

    /**
     * @brief   Sentinel value returned by CollectTimestamp() on failure.
     * @details A return value of zero indicates that the system clock could not be
     *          read (e.g. the handle to the time source is unavailable). Callers
     *          MUST check for this sentinel before using the timestamp.
     */
    static constexpr uint64_t kInvalidTimestamp{0ULL};

    // =========================================================================
    // Constructor / Destructor
    // =========================================================================

    /**
     * @brief   Constructs a CanConsumer and binds it to the provided shared memory handle.
     * @details Stores the raw non-owning pointer to the shared memory region that was
     *          mapped by the caller (ProbeApp) prior to construction. Initialises all
     *          internal state to safe defaults. No I/O or system calls are performed
     *          during construction; the handle is only dereferenced during subsequent
     *          read operations.
     *
     *          If @p memHandle is nullptr the object is constructed in a degraded state:
     *          all read operations will return empty vectors and ValidateSignalData()
     *          will return false.
     *
     * @param[in] memHandle  Non-owning raw pointer to the mapped shared memory region
     *                       that contains the CAN signal data structure. Valid range:
     *                       non-null pointer to a region of at least sizeof(CANDATA)
     *                       bytes; nullptr is accepted but results in degraded operation.
     *
     * @pre   The shared memory region pointed to by @p memHandle has been successfully
     *        created, sized, and mapped by the caller before this constructor is invoked.
     * @post  sharedMemHandle_ == memHandle. timestampSec_ == 0. timestampMsec_ == 0.
     *        signalIdMap_ is empty.
     *
     * @throws None — constructor is noexcept.
     *
     * @note  Called by: ProbeApp at application startup (Startup call condition).
     * @note  MISRA C++ Rule 8-4-4: all members are explicitly initialised in the
     *        member-initialiser list.
     *
     * @warning Passing a stale or already-unmapped pointer as @p memHandle leads to
     *          undefined behaviour when any read method is subsequently called.
     *
     * @requirements SWR-REQ-01-01-001
     * @rationale    Accepting a raw non-owning pointer (observer) is intentional:
     *               the shared memory lifetime is managed by ProbeApp, not by this class.
     *               Using std::shared_ptr<void> would impose unnecessary overhead and
     *               coupling for a safety-critical embedded context.
     */
    explicit CanConsumer(void* memHandle) noexcept;

    /**
     * @brief   Destroys the CanConsumer instance.
     * @details Releases no resources — the shared memory handle is non-owning and must
     *          be unmapped by the caller (ProbeApp) after this destructor returns.
     *          All std containers (signalIdMap_) are destroyed automatically via RAII.
     *
     * @post  All member containers are destroyed. sharedMemHandle_ is not dereferenced.
     *
     * @throws None — destructor is noexcept.
     */
    ~CanConsumer() noexcept = default;

    /// @brief Deleted copy constructor — CanConsumer is non-copyable (owns a handle context).
    CanConsumer(const CanConsumer&) = delete;

    /// @brief Deleted copy-assignment operator — CanConsumer is non-copyable.
    CanConsumer& operator=(const CanConsumer&) = delete;

    /// @brief Deleted move constructor — pointer ownership semantics are explicit.
    CanConsumer(CanConsumer&&) = delete;

    /// @brief Deleted move-assignment operator — pointer ownership semantics are explicit.
    CanConsumer& operator=(CanConsumer&&) = delete;

    // =========================================================================
    // Public Methods
    // =========================================================================

    /**
     * @brief   Reads all CAN RX/TX signals designated for the 100 ms periodicity cycle.
     * @details Iterates over the set of signal IDs registered in signalIdMap_ that are
     *          tagged with the 100 ms periodicity. For each signal ID the method calls
     *          the appropriate internal reader (ReadCanRxData() or ReadCanTxData()),
     *          appends the returned bytes to the output vector, and captures a timestamp
     *          via CollectTimestamp(). The resulting byte vector is a flat concatenation
     *          of all 100 ms signal payloads in signal-ID ascending order.
     *
     *          If sharedMemHandle_ is nullptr or no 100 ms signals are registered, an
     *          empty vector is returned.
     *
     * @return  std::vector<uint8_t> containing the concatenated raw signal bytes for all
     *          100 ms CAN signals. Returns an empty vector on error or if no signals are
     *          registered for this periodicity.
     *
     * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
     * @pre   signalIdMap_ has been populated (or is empty, in which case an empty vector
     *        is returned without error).
     * @post  timestampSec_ and timestampMsec_ reflect the time of this read cycle.
     *
     * @throws None — all internal errors are handled by returning an empty vector.
     *
     * @note  Called by: ProbeApp. Call condition: Cyclic_100ms (every 100 ms).
     * @note  The returned vector is allocated on the heap by std::vector; the caller
     *        takes ownership and is responsible for its lifetime.
     *
     * @warning Do not call from an interrupt context. This function performs shared
     *          memory reads which may involve memory barriers.
     *
     * @requirements SWR-REQ-01-001; SWR-REQ-01-01-001
     * @rationale    Returning std::vector<uint8_t> by value provides a clean, owning
     *               interface that avoids output-parameter aliasing and is compatible
     *               with RAII-based buffer management in ProbeApp.
     */
    std::vector<uint8_t> ReadCanData100ms();

    /**
     * @brief   Reads all CAN RX/TX signals designated for the 1000 ms periodicity cycle.
     * @details Iterates over the set of signal IDs registered in signalIdMap_ that are
     *          tagged with the 1000 ms periodicity. For each signal ID the method calls
     *          the appropriate internal reader (ReadCanRxData() or ReadCanTxData()),
     *          appends the returned bytes to the output vector, and captures a timestamp
     *          via CollectTimestamp(). The resulting byte vector is a flat concatenation
     *          of all 1000 ms signal payloads in signal-ID ascending order.
     *
     *          If sharedMemHandle_ is nullptr or no 1000 ms signals are registered, an
     *          empty vector is returned.
     *
     * @return  std::vector<uint8_t> containing the concatenated raw signal bytes for all
     *          1000 ms CAN signals. Returns an empty vector on error or if no signals are
     *          registered for this periodicity.
     *
     * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
     * @pre   signalIdMap_ has been populated (or is empty, in which case an empty vector
     *        is returned without error).
     * @post  timestampSec_ and timestampMsec_ reflect the time of this read cycle.
     *
     * @throws None — all internal errors are handled by returning an empty vector.
     *
     * @note  Called by: ProbeApp. Call condition: Cyclic_1000ms (every 1000 ms).
     * @note  The returned vector is allocated on the heap by std::vector; the caller
     *        takes ownership and is responsible for its lifetime.
     *
     * @warning Do not call from an interrupt context. This function performs shared
     *          memory reads which may involve memory barriers.
     *
     * @requirements SWR-REQ-01-002; SWR-REQ-01-01-001
     * @rationale    Mirrors ReadCanData100ms() in structure to maintain a consistent,
     *               auditable interface pattern across all periodicity readers.
     */
    std::vector<uint8_t> ReadCanData1000ms();

    /**
     * @brief   Reads the raw byte payload of a specific CAN RX signal from shared memory.
     * @details Looks up the signal identified by @p signalId in the shared memory region
     *          referenced by sharedMemHandle_. The lookup maps @p signalId to the
     *          corresponding byte offset and length within the CANDATA structure. The
     *          raw bytes are copied into a newly constructed std::vector<uint8_t> and
     *          returned to the caller.
     *
     *          Returns an empty vector if:
     *          - sharedMemHandle_ is nullptr,
     *          - @p signalId is not found in signalIdMap_,
     *          - the byte range derived from the signal descriptor would exceed the
     *            mapped memory bounds.
     *
     * @param[in] signalId  Unique numeric identifier of the CAN RX signal to read.
     *                      Valid range: any uint32_t value present in signalIdMap_;
     *                      unregistered IDs cause an empty-vector return.
     *
     * @return  std::vector<uint8_t> containing the raw bytes of the requested RX signal.
     *          Returns an empty vector if the signal is not found or the handle is invalid.
     *
     * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
     * @pre   @p signalId is a valid RX signal identifier known to the system.
     * @post  The returned vector contains a copy of the signal bytes at the time of the call.
     *        The shared memory region is not modified.
     *
     * @throws None — all internal errors are handled by returning an empty vector.
     *
     * @note  Called by: CanConsumer (internally from ReadCanData100ms / ReadCanData1000ms).
     *        Call condition: On_Request.
     * @note  This function performs a read-only access to shared memory; no locking is
     *        required provided the caller guarantees single-threaded access per the
     *        component design assumptions.
     *
     * @warning Passing an unregistered @p signalId silently returns an empty vector.
     *          Callers must check the returned vector's size before use.
     *
     * @requirements SWR-REQ-01-01-001
     * @rationale    Separating RX and TX readers allows independent unit testing and
     *               clear traceability to the Icmem_ReadCanRxData / Icmem_ReadCanTxData
     *               shared memory API contract.
     */
    std::vector<uint8_t> ReadCanRxData(uint32_t signalId);

    /**
     * @brief   Reads the raw byte payload of a specific CAN TX signal from shared memory.
     * @details Looks up the signal identified by @p signalId in the shared memory region
     *          referenced by sharedMemHandle_. The lookup maps @p signalId to the
     *          corresponding byte offset and length within the CANDATA structure. The
     *          raw bytes are copied into a newly constructed std::vector<uint8_t> and
     *          returned to the caller.
     *
     *          Returns an empty vector if:
     *          - sharedMemHandle_ is nullptr,
     *          - @p signalId is not found in signalIdMap_,
     *          - the byte range derived from the signal descriptor would exceed the
     *            mapped memory bounds.
     *
     * @param[in] signalId  Unique numeric identifier of the CAN TX signal to read.
     *                      Valid range: any uint32_t value present in signalIdMap_;
     *                      unregistered IDs cause an empty-vector return.
     *
     * @return  std::vector<uint8_t> containing the raw bytes of the requested TX signal.
     *          Returns an empty vector if the signal is not found or the handle is invalid.
     *
     * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
     * @pre   @p signalId is a valid TX signal identifier known to the system.
     * @post  The returned vector contains a copy of the signal bytes at the time of the call.
     *        The shared memory region is not modified.
     *
     * @throws None — all internal errors are handled by returning an empty vector.
     *
     * @note  Called by: CanConsumer (internally from ReadCanData100ms / ReadCanData1000ms).
     *        Call condition: On_Request.
     * @note  This function performs a read-only access to shared memory; no locking is
     *        required provided the caller guarantees single-threaded access per the
     *        component design assumptions.
     *
     * @warning Passing an unregistered @p signalId silently returns an empty vector.
     *          Callers must check the returned vector's size before use.
     *
     * @requirements SWR-REQ-01-01-001
     * @rationale    Mirrors ReadCanRxData() in structure; TX signals may reside in a
     *               different region of the CANDATA structure, hence the separation.
     */
    std::vector<uint8_t> ReadCanTxData(uint32_t signalId);

    /**
     * @brief   Captures the current wall-clock time as a combined seconds/milliseconds timestamp.
     * @details Queries the system's steady clock (std::chrono::steady_clock) to obtain a
     *          monotonically increasing timestamp. The seconds component is stored in
     *          timestampSec_ and the sub-second milliseconds component is stored in
     *          timestampMsec_. The combined 64-bit return value encodes both fields as:
     *
     *              returnValue = (timestampSec_ * 1000ULL) + timestampMsec_
     *
     *          This encoding yields milliseconds since the clock epoch, which is
     *          sufficient for signal-acquisition time-tagging at the rates used by
     *          this component (100 ms / 1000 ms).
     *
     *          Returns kInvalidTimestamp (0) if the clock query fails.
     *
     * @return  uint64_t  Combined timestamp in milliseconds since the steady-clock epoch.
     *                    Range: [1, UINT64_MAX]. Returns kInvalidTimestamp (0) on failure.
     *
     * @retval  kInvalidTimestamp  Clock query failed; timestampSec_ and timestampMsec_
     *                             are unchanged.
     * @retval  >0                 Valid timestamp; timestampSec_ and timestampMsec_ have
     *                             been updated.
     *
     * @pre   None — this function has no preconditions on object state.
     * @post  On success: timestampSec_ holds the seconds component and timestampMsec_
     *        holds the milliseconds remainder of the current time.
     *        On failure: timestampSec_ and timestampMsec_ are unchanged.
     *
     * @throws None — noexcept; clock failures are indicated via kInvalidTimestamp.
     *
     * @note  Called by: CanConsumer (internally). Call condition: On_Request.
     * @note  std::chrono::steady_clock is used (not system_clock) to guarantee
     *        monotonicity and avoid discontinuities from NTP adjustments.
     *
     * @warning The epoch of steady_clock is implementation-defined; the returned value
     *          is meaningful only for relative time differences within a single process
     *          lifetime, not for absolute UTC time.
     *
     * @requirements SWR-REQ-01-003; SWR-REQ-03-01-005
     * @rationale    Encoding seconds and milliseconds into a single uint64_t avoids
     *               returning a struct and keeps the interface minimal for safety-critical
     *               contexts where struct-return ABI differences can be a concern.
     */
    uint64_t CollectTimestamp() noexcept;

    /**
     * @brief   Validates that a collected signal data vector is non-empty and well-formed.
     * @details Performs the following checks on @p data in order:
     *          1. The vector is non-empty (size > 0).
     *          2. The vector size does not exceed kMaxCanSignalBytes (≤ 8 bytes).
     *          3. At least one byte in the vector is non-zero (rejects all-zero payloads
     *             which may indicate an uninitialised shared memory region).
     *
     *          All three conditions must hold for the function to return true.
     *
     * @param[in] data  Byte vector containing the raw signal payload to validate.
     *                  Valid range: 1 to kMaxCanSignalBytes bytes; empty or oversized
     *                  vectors cause a false return.
     *
     * @return  bool  Validation result.
     *
     * @retval  true   @p data is non-empty, within size bounds, and contains at least
     *                 one non-zero byte — the payload is considered valid.
     * @retval  false  @p data is empty, exceeds kMaxCanSignalBytes, or contains only
     *                 zero bytes — the payload is considered invalid.
     *
     * @pre   None — this function has no preconditions on object state.
     * @post  Object state is unchanged; this function is a pure predicate.
     *
     * @throws None — noexcept; no exceptions are thrown or propagated.
     *
     * @note  Called by: CanConsumer (internally). Call condition: On_Request.
     * @note  The all-zero rejection heuristic (check 3) is a best-effort guard against
     *        uninitialised shared memory. It does not guarantee semantic correctness of
     *        the signal value.
     *
     * @warning This function does not verify signal-level semantics (e.g. physical range,
     *          signal checksum, or message counter). Higher-level validation is the
     *          responsibility of the consuming layer (ProbeApp).
     *
     * @requirements SWR-REQ-01-01-001
     * @rationale    Keeping validation logic inside CanConsumer ensures that every signal
     *               read path (100 ms, 1000 ms, on-request) benefits from a single,
     *               auditable validation gate without duplicating logic in ProbeApp.
     */
    bool ValidateSignalData(const std::vector<uint8_t>& data) const noexcept;

private:

    // =========================================================================
    // Private Member Variables
    // =========================================================================
    std::map<uint32_t, uint32_t> signalIdMap_{};
    /**
     * @brief Non-owning raw pointer to the POSIX shared memory region containing CAN signal data.
     *        Valid range: non-null pointer to a mapped CANDATA region, or nullptr (degraded mode).
     *        Lifetime: must exceed the lifetime of this CanConsumer instance.
     */
    void* sharedMemHandle_{nullptr};  ///< @brief Handle to the shared memory region. Non-owning observer pointer.

    /**
     * @brief Seconds component of the most recently captured signal-acquisition timestamp.
     *        Updated by CollectTimestamp(). Units: seconds since steady_clock epoch.
     *        Range: [0, UINT32_MAX].
     */
    uint32_t timestampSec_{0U};  ///< @brief Seconds part of the last collected timestamp. Range: [0, UINT32_MAX].

    /**
     * @brief Milliseconds component (sub-second remainder) of the most recently captured
     *        signal-acquisition timestamp. Updated by CollectTimestamp().
     *        Units: milliseconds. Range: [0, 999].
     */
    uint32_t timestampMsec_{0U};  ///< @brief Milliseconds part of the last collected timestamp. Range: [0, 999].

    /**
     * @brief Map from signal ID to signal metadata string (e.g. direction tag "RX"/"TX"
     *        and periodicity tag "100ms"/"1000ms"). Used by ReadCanData100ms(),
     *        ReadCanData1000ms(), ReadCanRxData(), and ReadCanTxData() to resolve
     *        signal descriptors at runtime.
     *        Key:   uint32_t signal identifier.
     *        Value: std::string encoding direction and periodicity, e.g. "RX_100ms".
     */
    std::unordered_map<uint32_t, std::string> signalIdMap_null;  ///< @brief Signal ID to descriptor map. Key: signal ID; Value: direction+periodicity tag string.

    // =========================================================================
    // Private Helper Methods
    // =========================================================================

    /**
     * @brief   Reads raw signal bytes from the shared memory region at a given byte offset.
     * @details Performs a bounds-checked byte copy from the shared memory region starting
     *          at @p byteOffset for @p length bytes. This is the lowest-level shared
     *          memory accessor used by both ReadCanRxData() and ReadCanTxData().
     *
     *          Returns an empty vector if:
     *          - sharedMemHandle_ is nullptr,
     *          - @p length is zero,
     *          - @p length exceeds kMaxCanSignalBytes,
     *          - @p byteOffset + @p length would overflow or exceed the mapped region.
     *
     * @param[in] byteOffset  Byte offset from the start of the shared memory region to
     *                        the first byte of the signal payload.
     *                        Valid range: [0, mapped_region_size - length].
     * @param[in] length      Number of bytes to read. Valid range: [1, kMaxCanSignalBytes].
     *
     * @return  std::vector<uint8_t> containing the copied signal bytes.
     *          Returns an empty vector on any error condition.
     *
     * @pre   sharedMemHandle_ is non-null.
     * @pre   @p byteOffset + @p length does not exceed the mapped region size.
     * @post  The shared memory region is not modified.
     *
     * @throws None — all errors result in an empty-vector return.
     *
     * @note  This private helper centralises all raw memory access, making it the single
     *        point for bounds-checking and enabling straightforward unit testing via mock
     *        shared memory buffers.
     *
     * @warning No synchronisation is performed inside this function. The caller must
     *          ensure that the shared memory region is not concurrently written during
     *          the copy operation (guaranteed by the single-threaded execution assumption).
     *
     * @requirements SWR-REQ-01-01-001
     * @rationale    Extracting raw memory access into a private helper avoids code
     *               duplication between ReadCanRxData() and ReadCanTxData() and
     *               concentrates the safety-critical bounds check in one auditable location.
     */
    std::vector<uint8_t> ReadSignalBytes(std::size_t byteOffset, std::size_t length) const;

};  // class CanConsumer

}  // namespace probe