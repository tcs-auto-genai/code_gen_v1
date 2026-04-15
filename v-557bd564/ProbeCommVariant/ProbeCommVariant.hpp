#pragma once

/**
 * @file    ProbeCommVariant.hpp
 * @brief   Variant configuration manager for the Probe communication component.
 * @details Manages vehicle variant-dependent feature enablement for the Probe
 *          application running on the ADAS ECU. Reads a 5-byte variant code from
 *          shared memory during startup, matches it against a variant dictionary
 *          (variant_table.json), and sets internal boolean flags that control
 *          whether data collection and transmission paths (Regular Probe, Event
 *          Probe, GEDR, DAQ) are active. Falls back to a default variant code
 *          when shared memory read fails, and disables all transmission when the
 *          variant code is not found in the dictionary.
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/**
 * @defgroup ProbeVariantManagement Probe Variant Management
 * @brief Components responsible for variant-based feature flag control in the Probe application.
 */

/// @namespace probe
/// @brief Contains all components of the probe application.
namespace probe {

/**
 * @class   ProbeCommVariant
 * @brief   Variant configuration manager that controls feature enablement based on vehicle variant code.
 * @details ProbeCommVariant is responsible for reading the 5-byte vehicle variant code
 *          from shared memory at startup, resolving it against a variant dictionary file
 *          (variant_table.json), and setting internal boolean flags that govern which
 *          data collection and transmission paths are active at runtime. If the shared
 *          memory read fails, a default variant code is used. If the variant code is not
 *          found in the dictionary, all transmission is disabled via
 *          DisableAllTransmissionOnInvalidVariant(). This class is designed for
 *          single-threaded use, called from a periodic 10 ms task context managed by
 *          ProbeApp. No dynamic memory allocation is performed after construction.
 *
 * @ingroup ProbeVariantManagement
 *
 * @note    Compliant with ISO 26262 ASIL-B, MISRA C++:2008, and ISO/IEC 14882 (C++17).
 *          No dynamic memory allocation (no new/delete/malloc). All members are
 *          initialized at declaration. No recursion. No undefined behaviour.
 *
 * @warning This class is NOT thread-safe. All methods must be called from the same
 *          execution context (ProbeApp periodic task). Do not share instances across
 *          threads without external synchronization.
 *
 * @invariant After construction, all boolean feature flags are initialized to false.
 *            After a successful SetVariant() call, flags reflect the matched variant
 *            entry. daqTransmissionEnabled_ is false whenever variantReadSuccess_ is
 *            false or the variant code is absent from the dictionary.
 *
 * @see     SharedMem
 */
class ProbeCommVariant
{
public:

    // =========================================================================
    // Lifecycle
    // =========================================================================

    /**
     * @brief   Constructs the variant configuration manager with safe default values.
     * @details Initializes all internal state to deterministic defaults. All boolean
     *          feature flags are set to false. The variant code is set to the compile-time
     *          default. No shared memory access or file I/O is performed in the constructor;
     *          those operations are deferred to ReadVariantCode() and SetVariant() which
     *          are called by ProbeApp during startup sequencing. This design ensures
     *          exception-safe construction and avoids blocking the constructor.
     *
     * @pre     None. The object may be constructed at any point before startup sequencing.
     * @post    All boolean feature flags are false. variantCode_ equals kDefaultVariantCode_.
     *          variantReadSuccess_ is false.
     *
     * @throws  None. This constructor is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp during application initialization.
     * @note    MISRA C++:2008 compliant — all members initialized at declaration.
     *
     * @requirements SWR-REQ-01-03-001;SWR-REQ-03-14-001
     * @rationale Deferred I/O from constructor prevents blocking during object graph
     *            construction and simplifies unit testing.
     */
    ProbeCommVariant() noexcept;

    /**
     * @brief   Destroys the variant configuration manager and releases all resources.
     * @details The destructor performs orderly cleanup of any resources held by the
     *          variant manager. Since no heap allocations or OS handles are acquired
     *          by this class, the destructor body is effectively a no-op; however it
     *          is explicitly declared virtual to support potential future subclassing
     *          and to satisfy MISRA C++ Rule 12-1-1 regarding base class destructors.
     *
     * @pre     The object must have been fully constructed.
     * @post    All resources owned by this instance are released.
     *
     * @throws  None. The destructor is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp during application shutdown.
     */
    ~ProbeCommVariant() noexcept;

    // =========================================================================
    // Startup Interface — called by ProbeApp at startup
    // =========================================================================

    /**
     * @brief   Reads the 5-byte variant code from shared memory during initialization.
     * @details Attempts to read the vehicle variant code from the shared memory region
     *          exposed by the SharedMem component via Icmem_ReadVariantCode(). The
     *          variant code is a fixed-length 5-character ASCII string identifying the
     *          vehicle configuration. If the shared memory read succeeds, variantReadSuccess_
     *          is set to true and the read code is stored in variantCode_. If the read
     *          fails (e.g., shared memory not yet available at startup), the function
     *          falls back to kDefaultVariantCode_ and sets variantReadSuccess_ to false.
     *          The returned string is always a valid, non-empty 5-character code.
     *
     *          This function provides the SharedMemory_VariantCode interface.
     *
     * @return  std::string containing the resolved variant code (either read from shared
     *          memory or the compile-time default). Always a non-empty string of exactly
     *          kVariantCodeLength_ characters.
     *
     * @retval  kDefaultVariantCode_  Returned when shared memory read fails or returns
     *                                an invalid/empty result.
     * @retval  <variant_string>      The 5-character variant code successfully read from
     *                                shared memory.
     *
     * @pre     The SharedMem component must be initialized before this function is called.
     *          This function is intended to be called once during startup sequencing.
     * @post    variantCode_ contains the resolved variant code.
     *          variantReadSuccess_ reflects whether shared memory was read successfully.
     *
     * @throws  None. All error conditions are handled internally with fallback logic.
     *
     * @note    Called by: ProbeApp. Call condition: Startup.
     * @note    Provides interface: SharedMemory_VariantCode.
     * @warning Do not call this function after startup sequencing is complete. Repeated
     *          calls may overwrite a previously valid variant code with a stale read.
     *
     * @requirements SWR-REQ-01-03-001;SWR-REQ-03-14-001
     * @rationale Fallback to default variant prevents a hard failure when shared memory
     *            is temporarily unavailable during early startup, improving system
     *            robustness per ISO 26262 ASIL-B fault tolerance requirements.
     * @see     SetVariant()
     */
    std::string ReadVariantCode();

    /**
     * @brief   Reads the variant code from shared memory only (no dictionary apply).
     * @details Updates variantCode_ and variantReadSuccess_. Does not call SetVariant().
     *          Use with ApplyDefaultVariantCode() / ApplyVariantFeatureFlags() for the
     *          same sequencing as legacy ProbeApp startup.
     */
    std::string ReadVariantCodeFromSharedMemory();

    bool IsVariantReadSuccessful() const noexcept { return variantReadSuccess_; }

    /**
     * @brief   Forces the compile-time default variant code and applies it via SetVariant().
     */
    void ApplyDefaultVariantCode() noexcept;

    /**
     * @brief   Applies dictionary feature flags for the current variantCode_.
     * @return  true if the code matched a dictionary entry; false on load/miss/invalid code.
     */
    bool ApplyVariantFeatureFlags() noexcept;

    /**
     * @brief   Enables or disables collection features based on the variant code matched against the variant dictionary.
     * @details Accepts a variant code string, parses the variant dictionary file located
     *          at variantDictionaryPath_ (variant_table.json), and attempts to find a
     *          matching entry. If a match is found, the internal boolean feature flags
     *          (regularProbeEnabled_, eventProbeEnabled_, eventProbeWithoutPictureEnabled_,
     *          gedrEnabled_, gedrWithoutPictureEnabled_, daqTransmissionEnabled_) are set
     *          according to the matched entry's configuration. If no match is found,
     *          HandleMissingVariantCode() is called, which in turn calls
     *          DisableAllTransmissionOnInvalidVariant() to set all flags to false.
     *          Input validation is performed: an empty or oversized code string causes
     *          immediate invocation of HandleMissingVariantCode().
     *
     * @param[in] code  The variant code string to look up in the variant dictionary.
     *                  Must be a non-empty string of exactly kVariantCodeLength_ characters.
     *                  Valid range: printable ASCII characters, length == kVariantCodeLength_.
     *
     * @pre     ReadVariantCode() must have been called prior to this function so that
     *          variantCode_ is populated. The variant dictionary file must be accessible
     *          at variantDictionaryPath_.
     * @post    All boolean feature flags reflect the variant entry matched by @p code,
     *          or all flags are false if no match was found.
     *
     * @throws  None. File I/O errors and parse failures are handled internally.
     *
     * @note    Called by: ProbeApp. Call condition: Startup.
     * @warning Calling this function with an empty string will disable all transmission.
     *          Ensure ReadVariantCode() has been called and returned a valid code before
     *          passing its result to this function.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @rationale Dictionary-based variant control allows one binary to support multiple
     *            vehicle configurations without recompilation, reducing integration risk.
     * @see     ReadVariantCode(), HandleMissingVariantCode(), DisableAllTransmissionOnInvalidVariant()
     */
    void SetVariant(const std::string& code);

    // =========================================================================
    // Feature Query Interface — called by ProbeApp on request
    // =========================================================================

    /**
     * @brief   Returns whether regular probe data collection is enabled based on the active variant.
     * @details Provides a read-only query of the regularProbeEnabled_ flag, which is set
     *          by SetVariant() after matching the variant code against the dictionary.
     *          Regular probe data collection refers to the cyclic (100 ms / 1000 ms)
     *          CAN signal sampling and transmission to DAQ. This flag must be checked
     *          before initiating any regular data collection cycle.
     *
     * @return  bool indicating whether regular probe data collection is active.
     *
     * @retval  true   Regular probe data collection is enabled for the active variant.
     * @retval  false  Regular probe data collection is disabled, either because the
     *                 variant does not support it or because SetVariant() has not been
     *                 called successfully.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     * @note    This function is safe to call from any point after startup sequencing.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @see     SetVariant(), CheckEventProbeEnabled()
     */
    [[nodiscard]] bool CheckRegularProbeEnabled() const noexcept;

    /**
     * @brief   Returns whether event probe data collection is enabled based on the active variant.
     * @details Provides a read-only query of the eventProbeEnabled_ flag, which is set
     *          by SetVariant() after matching the variant code against the dictionary.
     *          Event probe data collection refers to trigger-based data capture (ZAT
     *          trigger) including camera image acquisition and transmission to DAQ/GEDR.
     *          This flag must be checked before initiating any event-based data collection.
     *
     * @return  bool indicating whether event probe data collection is active.
     *
     * @retval  true   Event probe data collection (with picture) is enabled for the active variant.
     * @retval  false  Event probe data collection is disabled, either because the variant
     *                 does not support it or because SetVariant() has not been called successfully.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @see     SetVariant(), CheckEventProbeWithoutPictureEnabled()
     */
    [[nodiscard]] bool CheckEventProbeEnabled() const noexcept;

    /**
     * @brief   Returns whether event probe without picture is enabled based on the active variant.
     * @details Provides a read-only query of the eventProbeWithoutPictureEnabled_ flag.
     *          This flag distinguishes between full event probe (with camera image capture)
     *          and a reduced event probe mode that transmits signal data only, without
     *          requesting or transmitting JPEG image frames. This allows variants that
     *          lack camera hardware or camera licensing to still participate in event
     *          data collection.
     *
     * @return  bool indicating whether event probe without picture is active.
     *
     * @retval  true   Event probe without picture is enabled for the active variant.
     * @retval  false  Event probe without picture is disabled for the active variant.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @see     SetVariant(), CheckEventProbeEnabled()
     */
    [[nodiscard]] bool CheckEventProbeWithoutPictureEnabled() const noexcept;

    /**
     * @brief   Returns whether GEDR data collection is enabled based on the active variant.
     * @details Provides a read-only query of the gedrEnabled_ flag, which is set by
     *          SetVariant() after matching the variant code against the dictionary.
     *          GEDR (General Event Data Recorder) data collection refers to event-triggered
     *          data transmission to the external GEDR storage device, including camera
     *          image frames. This flag must be checked before initiating any GEDR write
     *          operations.
     *
     * @return  bool indicating whether GEDR data collection (with picture) is active.
     *
     * @retval  true   GEDR data collection with picture is enabled for the active variant.
     * @retval  false  GEDR data collection is disabled for the active variant.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @see     SetVariant(), CheckGedrWithoutPictureEnabled()
     */
    [[nodiscard]] bool CheckGedrEnabled() const noexcept;

    /**
     * @brief   Returns whether GEDR without picture is enabled based on the active variant.
     * @details Provides a read-only query of the gedrWithoutPictureEnabled_ flag.
     *          This flag distinguishes between full GEDR mode (with camera image frames)
     *          and a reduced GEDR mode that writes signal data only, without image payloads.
     *          Variants without camera capability may still support GEDR signal recording
     *          via this mode.
     *
     * @return  bool indicating whether GEDR without picture is active.
     *
     * @retval  true   GEDR without picture is enabled for the active variant.
     * @retval  false  GEDR without picture is disabled for the active variant.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-03-14-002
     * @see     SetVariant(), CheckGedrEnabled()
     */
    [[nodiscard]] bool CheckGedrWithoutPictureEnabled() const noexcept;

    /**
     * @brief   Returns whether DAQ transmission is enabled based on the active variant.
     * @details Provides a read-only query of the daqTransmissionEnabled_ flag. DAQ
     *          transmission covers both regular (continual) and event-based data uploads
     *          to the DAQ service on the Core ECU via SOME/IP. This flag is set to false
     *          when the variant code is not found in the dictionary (per SWR-REQ-01-03-003),
     *          ensuring that no data is transmitted to DAQ when the vehicle configuration
     *          is unrecognised. Callers must check this flag before initiating any DAQ
     *          send operation.
     *
     * @return  bool indicating whether DAQ transmission is permitted for the active variant.
     *
     * @retval  true   DAQ transmission is enabled for the active variant.
     * @retval  false  DAQ transmission is disabled — either the variant code was not found
     *                 in the dictionary, or SetVariant() has not been called successfully.
     *
     * @pre     SetVariant() must have been called during startup sequencing.
     * @post    Internal state is not modified.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeApp. Call condition: On_Request.
     * @warning Per SWR-REQ-01-03-003, this flag is explicitly false when the variant
     *          code is absent from the dictionary. Do not bypass this check.
     *
     * @requirements SWR-REQ-01-03-002;SWR-REQ-01-03-003;SWR-REQ-03-14-002
     * @see     SetVariant(), HandleMissingVariantCode(), DisableAllTransmissionOnInvalidVariant()
     */
    [[nodiscard]] bool CheckDaqTransmissionEnabled() const noexcept;

    /**
     * @brief Stub: DAQ BDP upload request/response (desktop build).
     * @return Status code; 0x0000 means accepted.
     */
    uint16_t REQUEST_BDP_UPLOAD_REQ_RES(uint16_t clientId,
                                        uint32_t eventValue,
                                        uint32_t dataSize,
                                        uint32_t requestNum) noexcept;

    /**
     * @brief Stub: DAQ BDP transmit request/response (desktop build).
     * @return Reply code; 0x0000 means success.
     */
    uint16_t TRANSMIT_BDP_REQ_RES(uint16_t clientId,
                                  uint32_t requestNum,
                                  const std::vector<uint8_t>& contents) noexcept;

private:

    // =========================================================================
    // Private Methods
    // =========================================================================

    /**
     * @brief   Logs an error and prevents data transmission when the variant code is not found in the dictionary.
     * @details Called internally by SetVariant() when the provided variant code string
     *          does not match any entry in the variant dictionary (variant_table.json).
     *          This function logs the unrecognised variant code to the system error output
     *          (std::cerr) and delegates to DisableAllTransmissionOnInvalidVariant() to
     *          set all feature flags to false, ensuring no data is transmitted to DAQ or
     *          GEDR for an unrecognised vehicle configuration. This implements the
     *          fail-safe behavior required by SWR-REQ-01-03-003.
     *
     * @pre     variantCode_ must contain the unrecognised code that triggered this call.
     * @post    All boolean feature flags are false. daqTransmissionEnabled_ is false.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeCommVariant (internal). Call condition: Startup.
     * @note    This is a private safety-critical function. Its behavior must not be
     *          altered without a corresponding change to the safety analysis.
     * @warning Do not call this function directly from outside the class. It is invoked
     *          exclusively by SetVariant() as part of the missing-variant handling path.
     *
     * @requirements SWR-REQ-01-03-003;SWR-REQ-03-14-003
     * @rationale Centralising the missing-variant error path ensures consistent
     *            fail-safe behavior and simplifies safety analysis traceability.
     * @see     DisableAllTransmissionOnInvalidVariant(), SetVariant()
     */
    void HandleMissingVariantCode() noexcept;

    /**
     * @brief   Disables all data sending to DAQ and GEDR when the variant is invalid.
     * @details Sets all boolean feature flags to false:
     *          regularProbeEnabled_, eventProbeEnabled_, eventProbeWithoutPictureEnabled_,
     *          gedrEnabled_, gedrWithoutPictureEnabled_, and daqTransmissionEnabled_.
     *          This function implements the fail-safe transmission lockout required when
     *          the vehicle variant code cannot be resolved to a known configuration.
     *          It is called exclusively by HandleMissingVariantCode() and ensures that
     *          no data is transmitted to any downstream system (DAQ, GEDR) in an
     *          undefined variant state.
     *
     * @pre     None. This function may be called at any point during startup sequencing.
     * @post    All boolean feature flags are false. No transmission will be initiated
     *          by any Check*Enabled() query until SetVariant() is called with a valid code.
     *
     * @throws  None. This function is guaranteed not to throw.
     *
     * @note    Called by: ProbeCommVariant (internal via HandleMissingVariantCode()).
     *          Call condition: Startup.
     * @note    This is a private safety-critical function implementing ASIL-B fail-safe
     *          behavior. All flag assignments are explicit and deterministic.
     * @warning This function unconditionally disables all transmission. Ensure it is
     *          only invoked on confirmed invalid variant conditions.
     *
     * @requirements SWR-REQ-01-03-003;SWR-REQ-03-14-003
     * @rationale A single dedicated function for disabling all transmission flags
     *            provides a single point of control for the fail-safe path, reducing
     *            the risk of partial flag updates and simplifying code review.
     * @see     HandleMissingVariantCode()
     */
    void DisableAllTransmissionOnInvalidVariant() noexcept;

    // =========================================================================
    // Private Constants
    // =========================================================================

    /**
     * @brief   Expected fixed length of the variant code string in bytes.
     * @details The vehicle variant code is always exactly 5 ASCII characters as defined
     *          by the shared memory interface specification (Icmem_ReadVariantCode).
     *          This constant is used for input validation in ReadVariantCode() and
     *          SetVariant(). Range: fixed value 5.
     */
    static constexpr std::size_t kVariantCodeLength_{5U};

    /**
     * @brief   Default variant code used as fallback when shared memory read fails.
     * @details Applied when Icmem_ReadVariantCode() returns an error or an empty result
     *          during startup. The default code "00000" represents an unspecified/unknown
     *          variant and will typically not match any dictionary entry, causing
     *          HandleMissingVariantCode() to be invoked and all transmission to be disabled.
     *          This ensures deterministic, fail-safe behavior on shared memory failure.
     */
    static constexpr const char* kDefaultVariantCode_{"00000"};

    /**
     * @brief   Filesystem path to the variant dictionary JSON file.
     * @details Points to the variant_table.json file that maps variant code strings to
     *          feature flag configurations. This path is resolved relative to the
     *          application working directory. The file must be present and readable at
     *          startup for SetVariant() to function correctly.
     */
    static constexpr const char* kVariantDictionaryPath_{"/etc/probe/variant_table.json"};

    // =========================================================================
    // Private Member Variables
    // =========================================================================

    /**
     * @brief Active variant code string resolved during startup.
     *        Contains either the value read from shared memory or kDefaultVariantCode_.
     *        Length is always kVariantCodeLength_ characters after ReadVariantCode() completes.
     *        Valid range: printable ASCII string of exactly 5 characters.
     */
    std::string variantCode_{kDefaultVariantCode_};  ///< @brief Active variant code. Range: 5-char ASCII string.

    /**
     * @brief Default variant code string used as the fallback value.
     *        Mirrors kDefaultVariantCode_ as an instance-level copy for runtime use.
     *        Immutable after construction.
     */
    std::string defaultVariantCode_{kDefaultVariantCode_};  ///< @brief Default fallback variant code. Value: "00000".

    /**
     * @brief Flag indicating whether the variant code was successfully read from shared memory.
     *        Set to true by ReadVariantCode() on a successful Icmem_ReadVariantCode() call.
     *        Set to false on shared memory read failure or when the default code is used.
     *        Used by CheckDaqTransmissionEnabled() to enforce transmission lockout on read failure.
     */
    bool variantReadSuccess_{false};  ///< @brief True if variant code was read from shared memory successfully.

    /**
     * @brief True after the last SetVariant() completed with a dictionary match (not missing-variant path).
     */
    bool variantDictionaryMatched_{false};

    /**
     * @brief Flag indicating whether regular (cyclic) probe data collection is enabled.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Controls 100 ms / 1000 ms CAN signal sampling and DAQ transmission.
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool regularProbeEnabled_{false};  ///< @brief True if regular probe data collection is enabled for this variant.

    /**
     * @brief Flag indicating whether event probe data collection (with picture) is enabled.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Controls ZAT-triggered event data capture including camera image acquisition.
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool eventProbeEnabled_{false};  ///< @brief True if event probe with picture is enabled for this variant.

    /**
     * @brief Flag indicating whether event probe without picture is enabled.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Controls ZAT-triggered event data capture excluding camera image frames.
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool eventProbeWithoutPictureEnabled_{false};  ///< @brief True if event probe without picture is enabled for this variant.

    /**
     * @brief Flag indicating whether GEDR data collection (with picture) is enabled.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Controls event-triggered data writes to the external GEDR device including images.
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool gedrEnabled_{false};  ///< @brief True if GEDR data collection with picture is enabled for this variant.

    /**
     * @brief Flag indicating whether GEDR without picture is enabled.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Controls event-triggered data writes to GEDR excluding camera image frames.
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool gedrWithoutPictureEnabled_{false};  ///< @brief True if GEDR without picture is enabled for this variant.

    /**
     * @brief Flag indicating whether DAQ transmission is permitted for the active variant.
     *        Set by SetVariant() based on the matched variant dictionary entry.
     *        Explicitly set to false by DisableAllTransmissionOnInvalidVariant() when the
     *        variant code is not found in the dictionary (per SWR-REQ-01-03-003).
     *        Initialized to false (disabled) until a valid variant is applied.
     */
    bool daqTransmissionEnabled_{false};  ///< @brief True if DAQ transmission is enabled for this variant. False on invalid/missing variant.

    /**
     * @brief Filesystem path to the variant dictionary JSON file used by SetVariant().
     *        Defaults to kVariantDictionaryPath_. Stored as an instance member to allow
     *        path override in test environments without modifying the constant.
     */
    std::string variantDictionaryPath_{kVariantDictionaryPath_};  ///< @brief Path to variant_table.json. Default: /etc/probe/variant_table.json.
};

}  // namespace probe