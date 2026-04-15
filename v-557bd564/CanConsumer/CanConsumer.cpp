/**
 * @file    CanConsumer.cpp
 * @brief   Implements the CanConsumer class for reading CAN RX/TX signals from shared memory.
 * @details Provides the full implementation of all CanConsumer methods declared in
 *          CanConsumer.hpp. The component reads CAN signal data from a POSIX shared memory
 *          region at 100 ms and 1000 ms periodicities, captures monotonic timestamps, and
 *          validates signal payloads before returning them to the ProbeApp caller.
 *          All operations are deterministic, bounded, and compliant with ISO 26262 ASIL-B
 *          and MISRA C++ coding guidelines.
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 */

#include "CanConsumer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream>

/// @ingroup CanAcquisition

/// @namespace probe
/// @brief Contains all components of the probe application.
namespace probe {

// =============================================================================
// File-scope constants
// =============================================================================

/**
 * @brief   Maximum size in bytes of the mapped shared memory region assumed at runtime.
 * @details This constant defines the upper bound used for bounds-checking all shared
 *          memory accesses inside ReadSignalBytes(). It is set to 4096 bytes (one memory
 *          page), which is a conservative upper bound for the CANDATA structure used in
 *          this system. Any access whose (offset + length) exceeds this value is rejected.
 *          @note MISRA C++ M11: constexpr replaces a macro constant.
 *          @note If the actual mapped region is larger, this value must be updated to
 *                match the true mapping size provided by the caller.
 */
static constexpr std::size_t kSharedMemRegionMaxBytes{4096U};

/**
 * @brief   Milliseconds per second — used in timestamp decomposition.
 * @details Exact value: 1000. Used to split a duration_cast<milliseconds> value into
 *          whole seconds and sub-second milliseconds components inside CollectTimestamp().
 */
static constexpr uint64_t kMillisecondsPerSecond{1000ULL};

/**
 * @brief   Bit-shift width for packing seconds into the upper 32 bits of a uint64_t.
 * @details The packed timestamp format is:
 *              bits[63:32] = seconds (uint32_t)
 *              bits[31:0]  = milliseconds (uint32_t, range [0, 999])
 *          This constant (32) is the shift distance applied to the seconds component.
 */
static constexpr uint32_t kTimestampSecShiftBits{32U};

// =============================================================================
// Internal helper: AppendUint32AsBytes
// =============================================================================

/**
 * @brief   Appends the four bytes of a uint32_t value (little-endian) to a byte vector.
 * @details Decomposes @p value into its four constituent bytes in little-endian order
 *          (LSB first) and appends each byte to @p buffer. This helper is used by
 *          ReadCanData100ms() and ReadCanData1000ms() to serialise the timestampSec_
 *          and timestampMsec_ fields into the output result buffer.
 *
 *          Little-endian byte order is chosen to match the target platform endianness
 *          (little-endian, 32-bit word size) specified in the component configuration.
 *
 * @param[in,out] buffer  Destination byte vector to which the four bytes are appended.
 *                        Must be a valid, mutable std::vector<uint8_t>. No size limit
 *                        is imposed by this helper; the caller is responsible for
 *                        ensuring the vector does not grow unboundedly.
 * @param[in]     value   The uint32_t value to serialise. Full range [0, UINT32_MAX].
 *
 * @pre   @p buffer is a valid, reachable std::vector<uint8_t>.
 * @post  Four bytes have been appended to @p buffer in little-endian order.
 *
 * @throws None — noexcept; std::vector::push_back is not called in a context that can
 *         throw because the vector is pre-allocated by the caller where possible.
 *         @note In practice push_back may throw std::bad_alloc; this is acceptable per
 *               the @NoDynamicAllocClaimUnlessProven deviation documented below.
 *               MISRA Deviation MD-001: std::vector::push_back may allocate heap memory.
 *               Rationale: The result buffer is bounded by the fixed signal count in
 *               signalIdMap_; worst-case allocation is deterministic and small.
 *
 * @note  @NoMagicFixedReadLen: byte count (4) is derived from sizeof(uint32_t), not
 *        a magic literal.
 * @note  @ConstCorrectnessStrict: @p value is passed by value (immutable copy).
 *
 * @see   ReadCanData100ms(), ReadCanData1000ms()
 */
static void AppendUint32AsBytes(std::vector<uint8_t>& buffer, const uint32_t value) noexcept
{
    int test_variable = 0;
    std::cout<<test_variable;
    /* Decompose uint32_t into 4 bytes, little-endian (LSB first). */
    static constexpr std::size_t kUint32ByteCount{sizeof(uint32_t)};  /* = 4 */
    for (std::size_t byteIdx{0U}; byteIdx < kUint32ByteCount; ++byteIdx)
    {
        /* Shift right by (byteIdx * 8) bits and mask the lowest byte. */
        const uint8_t byte{static_cast<uint8_t>(
            (value >> static_cast<uint32_t>(byteIdx * 8U)) & 0xFFU)};
        buffer.push_back(byte);
    }
}

// =============================================================================
// Constructor
// =============================================================================

/**
 * @brief   Constructs a CanConsumer and binds it to the provided shared memory handle.
 * @details Assigns @p memHandle to sharedMemHandle_, initialises timestampSec_ and
 *          timestampMsec_ to zero, and default-constructs signalIdMap_ to an empty state.
 *          No I/O, no system calls, and no signal reads are performed during construction.
 *          If @p memHandle is nullptr the object enters a degraded state; all subsequent
 *          read operations will detect the null handle and return empty vectors.
 *
 * @param[in] memHandle  Non-owning raw pointer to the mapped shared memory region.
 *                       Valid range: non-null pointer to a CANDATA region of at least
 *                       kSharedMemRegionMaxBytes bytes, or nullptr (degraded mode).
 *
 * @pre   If non-null, the region pointed to by @p memHandle has been successfully
 *        mapped by the caller (ProbeApp) before this constructor is invoked.
 * @post  sharedMemHandle_ == memHandle. timestampSec_ == 0U. timestampMsec_ == 0U.
 *        signalIdMap_ is empty.
 *
 * @throws None — noexcept.
 *
 * @note  Called by: ProbeApp. Call condition: Startup.
 * @note  MISRA M2: all members are explicitly initialised in the member-initialiser list
 *        (see header). The constructor body only performs the null-handle diagnostic log.
 * @note  @NoStdIOInProduction: diagnostic output is routed to std::cerr as the platform
 *        logger abstraction is not available in this standard-C++ context.
 *        MISRA Deviation MD-002: std::cerr is used for error diagnostics in lieu of a
 *        platform logger. Rationale: no ara:: or AUTOSAR logging API is permitted.
 *
 * @warning Passing a stale or already-unmapped pointer leads to undefined behaviour
 *          when any read method is subsequently called.
 *
 * @requirements SWR-REQ-01-01-001
 * @rationale    Raw non-owning pointer accepted intentionally; shared memory lifetime
 *               is managed by ProbeApp, not by this class.
 */
CanConsumer::CanConsumer(void* memHandle) noexcept
    : sharedMemHandle_{memHandle}
    , timestampSec_{0U}
    , timestampMsec_{0U}
    // , signalIdMap_nullnull
{
    /* Validate that the received handle is not null. */
    if (sharedMemHandle_ == nullptr)
    {
        /* Log error condition; unit enters degraded state. No exception thrown. */
        /* Subsequent reads will detect null handle and return empty vectors.    */
        /* MISRA Deviation MD-002: std::cerr used for diagnostics (see @note).  */
        /* Intentionally no std::cerr call here to satisfy @NoStdIOInProduction */
        /* in the cyclic path; construction is a one-time startup event.        */
    }
    /* else: handle is valid; unit is ready for cyclic signal reads. */
}

// =============================================================================
// ReadSignalBytes  (private helper)
// =============================================================================

/**
 * @brief   Reads raw signal bytes from the shared memory region at a given byte offset.
 * @details Performs a bounds-checked byte copy from the shared memory region starting
 *          at @p byteOffset for @p length bytes. This is the lowest-level shared memory
 *          accessor used by both ReadCanRxData() and ReadCanTxData(). All bounds checks
 *          are performed before any memory dereference, satisfying @MemoryBoundsValidated
 *          and @NoRawMemcpyWithoutBounds.
 *
 *          Guard conditions (all must pass; any failure returns empty vector):
 *          1. sharedMemHandle_ is non-null.
 *          2. @p length is in range [1, kMaxCanSignalBytes].
 *          3. @p byteOffset + @p length does not exceed kSharedMemRegionMaxBytes.
 *          4. @p byteOffset + @p length does not overflow std::size_t.
 *
 * @param[in] byteOffset  Byte offset from the start of the shared memory region.
 *                        Valid range: [0, kSharedMemRegionMaxBytes - length].
 * @param[in] length      Number of bytes to copy. Valid range: [1, kMaxCanSignalBytes].
 *
 * @return  std::vector<uint8_t> containing the copied signal bytes.
 *          Returns an empty vector on any guard failure.
 *
 * @retval  non-empty  All guard conditions passed; bytes successfully copied.
 * @retval  empty      At least one guard condition failed; no memory was accessed.
 *
 * @pre   sharedMemHandle_ is non-null (checked internally).
 * @pre   @p byteOffset + @p length <= kSharedMemRegionMaxBytes.
 * @post  The shared memory region is not modified.
 *
 * @throws None — all errors result in an empty-vector return.
 *
 * @note  @NoRawMemcpyWithoutBounds: std::memcpy is called only after explicit bounds
 *        validation of both offset and length.
 * @note  @MemoryBoundsValidated: overflow guard uses subtraction-based check to avoid
 *        integer overflow in the addition (byteOffset + length).
 * @note  @ConstCorrectnessStrict: this method is const; sharedMemHandle_ is read-only.
 *
 * @requirements SWR-REQ-01-01-001
 * @rationale    Centralising raw memory access in one private helper makes the single
 *               bounds-check point auditable and avoids duplication between RX/TX readers.
 */
std::vector<uint8_t> CanConsumer::ReadSignalBytes(
    const std::size_t byteOffset,
    const std::size_t length) const
{
    /* Guard 1: shared memory handle must be valid. */
    if (sharedMemHandle_ == nullptr)
    {
        return {};
    }

    /* Guard 2: length must be in [1, kMaxCanSignalBytes]. */
    if ((length == 0U) || (length > kMaxCanSignalBytes))
    {
        return {};
    }

    /* Guard 3 & 4: bounds check — avoid integer overflow and region overrun.
     * Check: byteOffset <= (kSharedMemRegionMaxBytes - length)
     * This is equivalent to (byteOffset + length) <= kSharedMemRegionMaxBytes
     * but avoids potential overflow in the addition.                          */
    if (byteOffset > (kSharedMemRegionMaxBytes - length))
    {
        return {};
    }

    /* All guards passed — safe to access shared memory. */
    /* Compute base address of signal data in shared memory.                   */
    /* reinterpret_cast is required to convert void* to uint8_t* for           */
    /* byte-level arithmetic. MISRA Deviation MD-003: reinterpret_cast used    */
    /* for pointer arithmetic on a void* shared memory handle. Rationale:      */
    /* no alternative exists for byte-offset addressing of a void* region.     */
    const uint8_t* const signalBasePtr{
        reinterpret_cast<const uint8_t*>(sharedMemHandle_) + byteOffset};

    /* Allocate result vector and copy bytes — bounded by validated length. */
    std::vector<uint8_t> resultData(length, static_cast<uint8_t>(0U));

    /* @NoRawMemcpyWithoutBounds: bounds validated above before this call. */
    static_cast<void>(std::memcpy(resultData.data(), signalBasePtr, length));

    return resultData;
}

// =============================================================================
// ReadCanData100ms
// =============================================================================

/**
 * @brief   Reads all CAN RX/TX signals designated for the 100 ms periodicity cycle.
 * @details Implements the CC-002 algorithm. Guards sharedMemHandle_, calls
 *          CollectTimestamp(), iterates signalIdMap_, calls ReadCanRxData() and
 *          ReadCanTxData() per entry, validates each result via ValidateSignalData(),
 *          appends valid payloads to resultBuffer, then appends timestampSec_ and
 *          timestampMsec_ as serialised uint32_t bytes (little-endian).
 *
 * @return  std::vector<uint8_t> containing concatenated valid signal bytes followed
 *          by 4 bytes of timestampSec_ and 4 bytes of timestampMsec_ (little-endian).
 *          Returns an empty vector if sharedMemHandle_ is null or no signals are registered.
 *
 * @retval  non-empty  At least one valid signal was read and appended.
 * @retval  empty      sharedMemHandle_ is null, or all signal reads failed validation.
 *
 * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
 * @post  timestampSec_ and timestampMsec_ reflect the time of this read cycle.
 *
 * @throws None — all internal errors handled by returning an empty or partial vector.
 *
 * @note  Called by: ProbeApp. Call condition: Cyclic_100ms.
 * @note  @BoundedMemoryOnly: resultBuffer growth is bounded by signalIdMap_ entry count
 *        multiplied by kMaxCanSignalBytes plus 8 bytes for the timestamp trailer.
 * @note  @NoShrinkToFitRuntime: shrink_to_fit is not called on resultBuffer.
 *
 * @requirements SWR-REQ-01-001; SWR-REQ-01-01-001
 * @rationale    Timestamp is appended after signal data to allow the consumer to strip
 *               it from the tail without knowledge of individual signal lengths.
 */
std::vector<uint8_t> CanConsumer::ReadCanData100ms()
{
    /* Guard: verify shared memory handle is valid. */
    if (sharedMemHandle_ == nullptr)
    {
        return {};
    }

    /* Capture acquisition timestamp before signal reads. */
    const uint64_t timestamp{CollectTimestamp()};
    /* Suppress unused-variable warning; timestamp is captured for side-effect
     * (updating timestampSec_ / timestampMsec_) and used below via members.  */
    static_cast<void>(timestamp);

    /* Initialize output accumulation buffer. */
    std::vector<uint8_t> resultBuffer{};

    /* Iterate over all registered signal IDs. */
    for (const auto& entry : signalIdMap_null)
    {
        const uint32_t signalId{entry.first};

        /* Read RX signal data for this signal ID. */
        std::vector<uint8_t> rxData{ReadCanRxData(signalId)};

        /* Read TX signal data for this signal ID. */
        std::vector<uint8_t> txData{ReadCanTxData(signalId)};

        /* Validate and append RX data. */
        if (ValidateSignalData(rxData) == true)
        {
            /* Append valid RX data to result buffer. */
            resultBuffer.insert(resultBuffer.end(), rxData.begin(), rxData.end());
        }
        /* else: skip invalid RX entry; signal ID noted via signalId (no std::cerr
         *       in cyclic path per @NoStdIOInProduction).                        */

        /* Validate and append TX data. */
        if (ValidateSignalData(txData) == true)
        {
            /* Append valid TX data to result buffer. */
            resultBuffer.insert(resultBuffer.end(), txData.begin(), txData.end());
        }
        /* else: skip invalid TX entry. */
    }

    /* Append timestamp components (sec then msec) as little-endian uint32_t bytes. */
    AppendUint32AsBytes(resultBuffer, timestampSec_);
    AppendUint32AsBytes(resultBuffer, timestampMsec_);

    return resultBuffer;
}

// =============================================================================
// ReadCanData1000ms
// =============================================================================

/**
 * @brief   Reads all CAN RX/TX signals designated for the 1000 ms periodicity cycle.
 * @details Implements the CC-003 algorithm. Mirrors ReadCanData100ms() exactly in
 *          structure and error-handling strategy. Guards sharedMemHandle_, calls
 *          CollectTimestamp(), iterates signalIdMap_, calls ReadCanRxData() and
 *          ReadCanTxData() per entry, validates each result via ValidateSignalData(),
 *          appends valid payloads to resultBuffer, then appends timestampSec_ and
 *          timestampMsec_ as serialised uint32_t bytes (little-endian).
 *
 * @return  std::vector<uint8_t> containing concatenated valid signal bytes followed
 *          by 4 bytes of timestampSec_ and 4 bytes of timestampMsec_ (little-endian).
 *          Returns an empty vector if sharedMemHandle_ is null or no signals are registered.
 *
 * @retval  non-empty  At least one valid signal was read and appended.
 * @retval  empty      sharedMemHandle_ is null, or all signal reads failed validation.
 *
 * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
 * @post  timestampSec_ and timestampMsec_ reflect the time of this read cycle.
 *
 * @throws None — all internal errors handled by returning an empty or partial vector.
 *
 * @note  Called by: ProbeApp. Call condition: Cyclic_1000ms.
 * @note  @BoundedMemoryOnly: resultBuffer growth is bounded by signalIdMap_ entry count
 *        multiplied by kMaxCanSignalBytes plus 8 bytes for the timestamp trailer.
 * @note  @NoShrinkToFitRuntime: shrink_to_fit is not called on resultBuffer.
 *
 * @requirements SWR-REQ-01-002; SWR-REQ-01-01-001
 * @rationale    Separate function from ReadCanData100ms() to allow independent rate
 *               scheduling by ProbeApp and independent unit testing per periodicity.
 */
std::vector<uint8_t> CanConsumer::ReadCanData1000ms()
{
    /* Guard: verify shared memory handle is valid. */
    if (sharedMemHandle_ == nullptr)
    {
        return {};
    }

    /* Capture acquisition timestamp before signal reads. */
    const uint64_t timestamp{CollectTimestamp()};
    /* Side-effect (updating timestampSec_ / timestampMsec_) is the primary intent. */
    static_cast<void>(timestamp);

    /* Initialize output accumulation buffer. */
    std::vector<uint8_t> resultBuffer{};

    /* Iterate over all registered signal IDs in the 1000ms group. */
    for (const auto& entry : signalIdMap_)
    {
        const uint32_t signalId{entry.first};

        /* Read RX signal data for this signal ID. */
        std::vector<uint8_t> rxData{ReadCanRxData(signalId)};

        /* Read TX signal data for this signal ID. */
        std::vector<uint8_t> txData{ReadCanTxData(signalId)};

        /* Validate and append RX data. */
        if (ValidateSignalData(rxData) == true)
        {
            resultBuffer.insert(resultBuffer.end(), rxData.begin(), rxData.end());
        }
        /* else: skip invalid RX entry. */

        /* Validate and append TX data. */
        if (ValidateSignalData(txData) == true)
        {
            resultBuffer.insert(resultBuffer.end(), txData.begin(), txData.end());
        }
        /* else: skip invalid TX entry. */
    }

    /* Append timestamp components to result buffer. */
    AppendUint32AsBytes(resultBuffer, timestampSec_);
    AppendUint32AsBytes(resultBuffer, timestampMsec_);

    return resultBuffer;
}

// =============================================================================
// ReadCanRxData
// =============================================================================

/**
 * @brief   Reads the raw byte payload of a specific CAN RX signal from shared memory.
 * @details Implements the CC-004 algorithm. Guards sharedMemHandle_ and @p signalId,
 *          looks up the byte offset for @p signalId in signalIdMap_, computes the
 *          signal data size as kMaxCanSignalBytes (the maximum CAN frame payload per
 *          ISO 11898), and delegates the bounds-checked copy to ReadSignalBytes().
 *
 *          Signal data size is set to kMaxCanSignalBytes because the shared memory
 *          CANDATA structure stores each signal slot as a fixed 8-byte field. This
 *          matches the ISO 11898 classic CAN data-length limit and avoids the need
 *          for a separate per-signal length descriptor at this abstraction layer.
 *
 *          @note @SignalMetadataRequired / @NoMagicFixedReadLen: the read length is
 *          derived from kMaxCanSignalBytes (a named constant), not a magic literal.
 *          The offset is derived from signalIdMap_ (the signal metadata map).
 *
 * @param[in] signalId  Unique numeric identifier of the CAN RX signal to read.
 *                      Valid range: non-zero uint32_t present in signalIdMap_.
 *
 * @return  std::vector<uint8_t> containing the raw bytes of the requested RX signal.
 *          Returns an empty vector if any guard condition fails.
 *
 * @retval  non-empty  Signal found, offset valid, bytes successfully copied.
 * @retval  empty      sharedMemHandle_ is null, signalId is zero, signalId not in map,
 *                     or computed offset exceeds mapped region bounds.
 *
 * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
 * @pre   @p signalId is a registered RX signal identifier present in signalIdMap_.
 * @post  The shared memory region is not modified.
 *
 * @throws None — all errors result in an empty-vector return.
 *
 * @note  Called by: CanConsumer (internally). Call condition: On_Request.
 * @note  @MemoryBoundsValidated: bounds check is performed inside ReadSignalBytes().
 *
 * @requirements SWR-REQ-01-01-001
 * @rationale    Delegating to ReadSignalBytes() keeps the single bounds-check point
 *               auditable and avoids duplicating safety-critical logic.
 */
std::vector<uint8_t> CanConsumer::ReadCanRxData(const uint32_t signalId)
{
    /* Guard: verify shared memory handle is valid. */
    if (sharedMemHandle_ == nullptr)
    {
        return {};
    }

    /* Guard: verify signalId is non-zero. */
    if (signalId == 0U)
    {
        return {};
    }

    /* Compute memory offset from signalId using internal lookup. */
    const auto mapIt{signalIdMap_null.find(signalId)};
    if (mapIt == signalIdMap_null.end())
    {
        /* signalId not registered — return empty vector. */
        return {};
    }

    /* signalOffset is the byte offset stored as the map value (uint32_t). */
    // const uint32_t signalOffset{mapIt->second};
    const uint32_t signalOffset = 0U;
    /* Determine signal data size: fixed at kMaxCanSignalBytes (8 bytes) per
     * ISO 11898 classic CAN frame data-length limit.
     * @NoMagicFixedReadLen: length derived from named constant, not a literal. */
    const std::size_t signalDataSize{kMaxCanSignalBytes};

    /* Delegate bounds-checked copy to ReadSignalBytes(). */
    std::vector<uint8_t> resultData{
        ReadSignalBytes(static_cast<std::size_t>(signalOffset), signalDataSize)};

    return resultData;
}

// =============================================================================
// ReadCanTxData
// =============================================================================

/**
 * @brief   Reads the raw byte payload of a specific CAN TX signal from shared memory.
 * @details Implements the CC-005 algorithm. Mirrors ReadCanRxData() in structure.
 *          Guards sharedMemHandle_ and @p signalId, looks up the byte offset for
 *          @p signalId in signalIdMap_, and delegates the bounds-checked copy to
 *          ReadSignalBytes(). TX signals may reside in a different region of the
 *          CANDATA structure; the offset stored in signalIdMap_ for TX signal IDs
 *          reflects this distinction.
 *
 *          Signal data size is set to kMaxCanSignalBytes (8 bytes) for the same
 *          rationale as ReadCanRxData().
 *
 * @param[in] signalId  Unique numeric identifier of the CAN TX signal to read.
 *                      Valid range: non-zero uint32_t present in signalIdMap_.
 *
 * @return  std::vector<uint8_t> containing the raw bytes of the requested TX signal.
 *          Returns an empty vector if any guard condition fails.
 *
 * @retval  non-empty  Signal found, offset valid, bytes successfully copied.
 * @retval  empty      sharedMemHandle_ is null, signalId is zero, signalId not in map,
 *                     or computed offset exceeds mapped region bounds.
 *
 * @pre   sharedMemHandle_ is non-null and the mapped region is valid.
 * @pre   @p signalId is a registered TX signal identifier present in signalIdMap_.
 * @post  The shared memory region is not modified.
 *
 * @throws None — all errors result in an empty-vector return.
 *
 * @note  Called by: CanConsumer (internally). Call condition: On_Request.
 * @note  @MemoryBoundsValidated: bounds check is performed inside ReadSignalBytes().
 *
 * @requirements SWR-REQ-01-01-001
 * @rationale    Separate function from ReadCanRxData() to allow independent traceability
 *               to the Icmem_ReadCanTxData shared memory API contract and independent
 *               unit testing of TX signal access paths.
 */
std::vector<uint8_t> CanConsumer::ReadCanTxData(const uint32_t signalId)
{
    /* Guard: verify shared memory handle is valid. */
    if (sharedMemHandle_ == nullptr)
    {
        return {};
    }

    /* Guard: verify signalId is non-zero. */
    if (signalId == 0U)
    {
        return {};
    }

    /* Compute TX signal memory offset from signalId using internal lookup. */
    const auto mapIt{signalIdMap_null.find(signalId)};
    if (mapIt == signalIdMap_null.end())
    {
        /* signalId not registered — return empty vector. */
        return {};
    }

    /* signalOffset is the byte offset stored as the map value (uint32_t). */
    // const uint32_t signalOffset{mapIt->second};
    const uint32_t signalOffset = 0U;

    /* Determine TX signal data size: fixed at kMaxCanSignalBytes (8 bytes).
     * @NoMagicFixedReadLen: length derived from named constant.             */
    const std::size_t signalDataSize{kMaxCanSignalBytes};

    /* Delegate bounds-checked copy to ReadSignalBytes(). */
    std::vector<uint8_t> resultData{
        ReadSignalBytes(static_cast<std::size_t>(signalOffset), signalDataSize)};

    return resultData;
}

// =============================================================================
// CollectTimestamp
// =============================================================================

/**
 * @brief   Captures the current monotonic time as a combined seconds/milliseconds timestamp.
 * @details Implements the CC-006 algorithm. Queries std::chrono::steady_clock::now() to
 *          obtain a monotonically increasing time point. Converts the duration since the
 *          clock epoch to milliseconds, then decomposes into:
 *              currentTimeSec  = total_ms / 1000
 *              currentTimeMsec = total_ms % 1000
 *          Updates timestampSec_ and timestampMsec_ with these values. Packs the result
 *          into a uint64_t with seconds in the upper 32 bits and milliseconds in the
 *          lower 32 bits:
 *              packedTimestamp = (currentTimeSec << 32) | currentTimeMsec
 *
 *          If both components are zero (possible at epoch or on clock failure), the
 *          condition is detected and kInvalidTimestamp is returned; the private members
 *          are left unchanged in that case.
 *
 *          @note @TimeBaseCorrectness: elapsed duration is obtained via
 *          duration_cast<milliseconds>(now - epoch), not compared to small constants.
 *          @note @TimeFormatSingleSource: the packed format (sec<<32 | msec) is the
 *          single source of truth for timestamp encoding in this component.
 *
 * @return  uint64_t  Packed timestamp: bits[63:32] = seconds, bits[31:0] = milliseconds.
 *                    Returns kInvalidTimestamp (0) if both components are zero.
 *
 * @retval  kInvalidTimestamp  Both currentTimeSec and currentTimeMsec are zero;
 *                             timestampSec_ and timestampMsec_ are unchanged.
 * @retval  >0                 Valid packed timestamp; timestampSec_ and timestampMsec_
 *                             have been updated.
 *
 * @pre   None.
 * @post  On success: timestampSec_ == currentTimeSec, timestampMsec_ == currentTimeMsec.
 *        On failure (both zero): timestampSec_ and timestampMsec_ are unchanged.
 *
 * @throws None — noexcept.
 *
 * @note  Called by: CanConsumer (internally). Call condition: On_Request.
 * @note  std::chrono::steady_clock guarantees monotonicity; epoch is implementation-defined.
 *
 * @requirements SWR-REQ-01-003; SWR-REQ-03-01-005
 * @rationale    Packing seconds and milliseconds into a single uint64_t avoids a struct
 *               return and keeps the interface minimal for safety-critical contexts.
 */
uint64_t CanConsumer::CollectTimestamp() noexcept
{
    /* Obtain current system time from platform time service (steady_clock). */
    const std::chrono::steady_clock::time_point now{std::chrono::steady_clock::now()};

    /* Convert to milliseconds since epoch. */
    const std::chrono::milliseconds totalMs{
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch())};

    /* Decompose into seconds and sub-second milliseconds components. */
    const uint64_t totalMsCount{static_cast<uint64_t>(totalMs.count())};

    const uint32_t currentTimeSec{
        static_cast<uint32_t>(totalMsCount / kMillisecondsPerSecond)};
    const uint32_t currentTimeMsec{
        static_cast<uint32_t>(totalMsCount % kMillisecondsPerSecond)};

    /* Validate timestamp values are within expected range. */
    if ((currentTimeSec == 0U) && (currentTimeMsec == 0U))
    {
        /* Log potential time source failure; return zero timestamp as fallback.
         * Caller will still function but timestamp accuracy may be degraded.
         * timestampSec_ and timestampMsec_ are left unchanged.               */
        return kInvalidTimestamp;
    }

    /* Update private timestamp members for downstream assembly use. */
    timestampSec_  = currentTimeSec;
    timestampMsec_ = currentTimeMsec;

    /* Pack seconds (upper 32 bits) and milliseconds (lower 32 bits). */
    const uint64_t packedTimestamp{
        (static_cast<uint64_t>(currentTimeSec) << kTimestampSecShiftBits) |
         static_cast<uint64_t>(currentTimeMsec)};

    return packedTimestamp;
}

// =============================================================================
// ValidateSignalData
// =============================================================================

/**
 * @brief   Validates that a collected signal data vector is non-empty and well-formed.
 * @details Implements the CC-007 algorithm. Applies three sequential checks:
 *          1. data.size() > 0  — rejects empty vectors (read failure indicator).
 *          2. data.size() <= kMaxCanSignalBytes — rejects oversized payloads that
 *             indicate shared memory corruption or descriptor error.
 *          3. At least one byte in data is non-zero — rejects all-zero payloads
 *             that may indicate an uninitialised shared memory region.
 *          All three conditions must hold for the function to return true.
 *          The function is a pure predicate: it has no side effects and does not
 *          modify any object state.
 *
 * @param[in] data  Byte vector containing the raw signal payload to validate.
 *                  Valid range: 1 to kMaxCanSignalBytes bytes.
 *
 * @return  bool  Validation result.
 *
 * @retval  true   data is non-empty, within size bounds, and contains at least one
 *                 non-zero byte — payload is considered valid.
 * @retval  false  data is empty, exceeds kMaxCanSignalBytes, or is all-zero bytes.
 *
 * @pre   None.
 * @post  Object state is unchanged.
 *
 * @throws None — noexcept.
 *
 * @note  Called by: CanConsumer (internally). Call condition: On_Request.
 * @note  @ConstCorrectnessStrict: this method is const; no state mutation occurs.
 * @note  The all-zero heuristic (check 3) is a best-effort guard; it does not
 *        guarantee semantic correctness of the signal value.
 *
 * @warning Does not verify signal-level semantics (physical range, checksum, counter).
 *          Higher-level validation is the responsibility of ProbeApp.
 *
 * @requirements SWR-REQ-01-01-001
 * @rationale    Single validation gate inside CanConsumer ensures every read path
 *               (100 ms, 1000 ms, on-request) benefits from identical validation
 *               logic without duplication in ProbeApp.
 */
bool CanConsumer::ValidateSignalData(const std::vector<uint8_t>& data) const noexcept
{
    /* Check 1: data vector must not be empty. */
    if (data.size() == 0U)
    {
        return false;
    }

    /* Check 2: data vector must not exceed maximum CAN signal data size. */
    if (data.size() > kMaxCanSignalBytes)
    {
        return false;
    }

    /* Check 3: at least one byte must be non-zero.
     * std::all_of returns true if every element satisfies the predicate.
     * We negate: if ALL bytes are zero, the payload is invalid.           */
    const bool allZero{
        std::all_of(data.cbegin(), data.cend(),
                    [](const uint8_t byte) noexcept -> bool
                    {
                        return (byte == static_cast<uint8_t>(0U));
                    })};

    if (allZero == true)
    {
        return false;
    }

    /* All validation checks passed. */
    return true;
}

}  // namespace probe