#pragma once

/**
 * @file    ProbeComm.hpp
 * @brief   Communication layer for the ProbeComm component handling DAQ, ZAT, CameraHost, GEDR, and SDMAP services.
 * @details ProbeComm manages all SOME/IP service discovery, event handling, payload construction,
 *          segmentation, and transmission for probe data to DAQ and GEDR destinations.
 *          It enforces trigger acceptance limits, retry logic, image acquisition timeouts,
 *          and dual-target transmission ordering in compliance with ISO 26262 ASIL-B requirements.
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

/**
 * @defgroup ProbeCommModule ProbeComm Communication Module
 * @brief Components responsible for probe data communication, service discovery, and event handling.
 * @details This module covers all outbound and inbound SOME/IP communication for the probe
 *          application, including DAQ continual data, BDP event upload, camera image acquisition,
 *          GEDR write operations, and ZAT trigger arbitration status reporting.
 */

/// @namespace probe
/// @brief Contains all components of the probe application.
namespace probe {

/**
 * @brief Opaque variant configuration type for ProbeComm initialization.
 * @details Holds variant-specific configuration parameters required during
 *          ProbeComm construction. Lifetime must exceed that of the ProbeComm instance.
 */
struct ProbeCommVariant;

/**
 * @class ProbeComm
 * @brief Communication manager for the probe application, handling all service interactions.
 * @details ProbeComm is responsible for:
 *          - Discovering and subscribing to SOME/IP services (DAQ, ZAT, CameraHost, TSR, HDF, SDMAP).
 *          - Offering SOME/IP services to ZAT and CameraHost.
 *          - Receiving and dispatching inbound event data from all subscribed services.
 *          - Constructing, segmenting, and transmitting event data payloads to DAQ via BDP protocol.
 *          - Writing event data to GEDR API with retry and error handling.
 *          - Enforcing trigger acceptance limits, overlap limits, and per-category counters.
 *          - Managing image acquisition triggers, timeouts, and answer-back validation.
 *          - Reporting trigger acceptance and rejection status to ZAT.
 *          - Supporting development log file output for sent and received data.
 *
 *          Thread safety: sendBufferMutex_ protects DAQ send buffer access.
 *          cameraMutex_ protects camera image state. All atomic members are
 *          individually thread-safe. Non-atomic members must only be accessed
 *          from the single periodic task context unless otherwise noted.
 *
 * @ingroup ProbeCommModule
 * @note    Complies with MISRA C++ guidelines and ISO 26262 ASIL-B requirements.
 *          No dynamic memory allocation after construction. No recursion.
 * @warning Do not call public methods from multiple threads without external synchronization
 *          except where std::mutex or std::atomic protection is explicitly documented.
 * @invariant daqCommunicationEstablished_ reflects the current SOME/IP DAQ link state.
 *            activeEventProcessCount_ must never exceed allowedOverlapTriggerNum_.
 *            bdpRequestRetryCounter_ <= bdpMaxRetryCount_.
 *            bdpTransmitRetryCounter_ <= bdpMaxRetryCount_.
 *            gedrWriteRetryCounter_ <= gedrMaxRetryCount_.
 * @see ProbeCommVariant
 */
class ProbeComm {
public:

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * @brief Constructs ProbeComm with a variant reference for communication setup.
     * @details Initializes all internal state, counters, flags, and mutexes.
     *          Stores the variant pointer for configuration access during service
     *          discovery and event handling. No heap allocation is performed after
     *          this constructor returns.
     * @param[in] variant Pointer to the ProbeCommVariant configuration structure.
     *                    Must not be nullptr. Lifetime must exceed this object's lifetime.
     * @pre  variant != nullptr and points to a valid, fully initialized ProbeCommVariant.
     * @post All internal counters are zero. All flags are false. Mutexes are unlocked.
     * @throws None — constructor is noexcept after parameter validation.
     * @note Called by: ProbeApp at startup.
     * @warning Passing a nullptr variant results in undefined behavior.
     * @requirements SWR-REQ-01-04-001;SWR-REQ-03-16-002
     * @rationale Variant-based initialization allows compile-time variant selection
     *            without runtime branching in the communication hot path.
     */
    explicit ProbeComm(ProbeCommVariant* variant) noexcept;

    /**
     * @brief Destructor. Releases all owned resources.
     * @details Stops any active timers, unlocks mutexes if held, and resets all state.
     * @pre  None.
     * @post All resources are released. No dangling references remain.
     * @throws None.
     */
    ~ProbeComm() noexcept;

    /// @brief Deleted copy constructor — ProbeComm is non-copyable.
    ProbeComm(const ProbeComm&) = delete;

    /// @brief Deleted copy assignment — ProbeComm is non-copyable.
    ProbeComm& operator=(const ProbeComm&) = delete;

    /// @brief Deleted move constructor — ProbeComm is non-movable.
    ProbeComm(ProbeComm&&) = delete;

    /// @brief Deleted move assignment — ProbeComm is non-movable.
    ProbeComm& operator=(ProbeComm&&) = delete;

    // =========================================================================
    // Service Discovery — Find Services (PC-002 to PC-012)
    // =========================================================================

    /**
     * @brief Locates the DAQ SOME/IP continual data service for steady-state data transmission.
     * @details Initiates SOME/IP service discovery for the DAQ continual data service.
     *          When the service is found, daqCommunicationEstablished_ is set to true
     *          and data suppression is lifted. Called once at startup by ProbeApp.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post daqCommunicationEstablished_ may transition to true upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: CORE_DAQ_continual_Service.
     * @warning Blocks until service is found or a platform-defined timeout expires.
     * @requirements SWR-REQ-01-04-001;SWR-REQ-01-06-003;SWR-REQ-03-16-002
     * @rationale Separate find functions per service allow independent retry and
     *            fault isolation per SOME/IP service endpoint.
     */
    void FindServiceDaqContinual() noexcept;

    /**
     * @brief Locates the DAQ SOME/IP development event data service for BDP event transmission.
     * @details Initiates SOME/IP service discovery for the DAQ development event service.
     *          Required before any BDP upload request or transmit method can be called.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post DAQ dev event service handle is available for BDP operations upon success.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: CORE_DAQ_devEvent_Service.
     * @warning Must be called before SendBdpUploadRequestToDAQ or TransmitBdpDataToDAQ.
     * @requirements SWR-REQ-01-04-001;SWR-REQ-03-16-002
     * @rationale Separate discovery for dev event service isolates BDP channel from
     *            continual data channel for independent fault handling.
     */
    void FindServiceDaqDevEvent() noexcept;

    /**
     * @brief Locates the ZAT trigger service for receiving event triggers at 100ms periodic rate.
     * @details Initiates SOME/IP service discovery for the ZAT hdfProbeBusOut trigger service.
     *          Subscribes to the trigger event upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post ZAT trigger event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: ZAT_hdfProbeBusOut_Service.
     * @requirements SWR-REQ-03-02-001;SWR-REQ-03-003
     * @rationale ZAT trigger drives the 100ms event detection cycle.
     */
    void FindServiceZatTrigger() noexcept;

    /**
     * @brief Locates ZAT regular upload data service for receiving regular upload data.
     * @details Initiates SOME/IP service discovery for the ZAT regular upload data service.
     *          Subscribes to the regular upload data event upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post ZAT regular upload event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: ZAT_hdfProbeBusOut_Service.
     * @requirements SWR-REQ-01-01-001
     * @rationale Regular upload data provides steady-state signal collection independent
     *            of event triggers.
     */
    void FindServiceZatRegularUpload() noexcept;

    /**
     * @brief Locates the CameraHost image service for receiving camera images.
     * @details Initiates SOME/IP service discovery for the CameraHost image service.
     *          Subscribes to image and answer-back events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post CameraHost image event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: CameraHost_IMG_Service.
     * @requirements SWR-REQ-03-06-002;SWR-REQ-03-06-006
     * @rationale Camera image service must be discovered before any image acquisition
     *            trigger is sent to CameraHost.
     */
    void FindServiceCameraImg() noexcept;

    /**
     * @brief Locates the CameraHost A-HDF/G-HDF data service for receiving camera object data.
     * @details Initiates SOME/IP service discovery for the CameraHost AGHDF service.
     *          Subscribes to FC camera bus out events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post CameraHost AGHDF event subscriptions are active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: CameraHost_AGhdf_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale AGHDF data provides camera object signals for event log data sets.
     */
    void FindServiceCameraAghdf() noexcept;

    /**
     * @brief Locates the TSR2GCoreHDF service for receiving TSR data.
     * @details Initiates SOME/IP service discovery for the TSR to GCore HDF service.
     *          Subscribes to TSR GCore HDF events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post TSR2GCoreHDF event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: TSR2GCoreHDF_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale TSR GCore data is required for event signal collection in log data sets.
     */
    void FindServiceTsr2GCoreHdf() noexcept;

    /**
     * @brief Locates the TSR2HDF service for receiving TSR ACore data.
     * @details Initiates SOME/IP service discovery for the TSR to HDF service.
     *          Subscribes to TSR HDF events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post TSR2HDF event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: TSR2HDF_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale TSR ACore data supplements GCore data for complete TSR signal coverage.
     */
    void FindServiceTsr2Hdf() noexcept;

    /**
     * @brief Locates the hdfAp2CpBusOut service for receiving HDF AP to CP data.
     * @details Initiates SOME/IP service discovery for the HDF AP to CP bus out service.
     *          Subscribes to AP-to-CP bus out events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post hdfAp2CpBusOut event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: hdfAp2CpBusOut_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale AP-to-CP bus data is required for cross-core signal collection.
     */
    void FindServiceHdfAp2CpBusOut() noexcept;

    /**
     * @brief Locates the hdfCp2ApBusOut service for receiving ICCOMMW CP to AP data.
     * @details Initiates SOME/IP service discovery for the HDF CP to AP bus out service.
     *          Subscribes to CP-to-AP bus out events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post hdfCp2ApBusOut event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: hdfCp2ApBusOut_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale CP-to-AP bus data completes the bidirectional inter-core signal set.
     */
    void FindServiceHdfCp2ApBusOut() noexcept;

    /**
     * @brief Locates the SDMAP data service for receiving map data.
     * @details Initiates SOME/IP service discovery for the SDMAP service.
     *          Subscribes to SDMAP data events upon service availability.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post SDMAP event subscription is active upon service availability.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: SDMAP_Service.
     * @requirements SWR-REQ-03-01-002
     * @rationale Map data enriches event log data sets with localization context.
     */
    void FindServiceSdmap() noexcept;

    // =========================================================================
    // Service Offering (PC-013 to PC-014)
    // =========================================================================

    /**
     * @brief Offers the probeEventStt service to allow ZAT to subscribe for trigger acceptance status.
     * @details Registers and offers the probeEventStt SOME/IP service so that ZAT can
     *          subscribe to receive trigger acceptance status events including accepted
     *          transmission numbers, category identifiers, and allowed overlap counts.
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post probeEventStt_Service is offered and ZAT can subscribe.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: probeEventStt_Service.
     * @requirements SWR-REQ-03-05-001;SWR-REQ-03-05-003
     * @rationale ZAT requires real-time feedback on trigger acceptance to manage
     *            its own trigger arbitration logic.
     */
    void OfferServiceProbeEventStt() noexcept;

    /**
     * @brief Offers the PROBE_TRG_Service to allow CameraHost to subscribe for image acquisition triggers.
     * @details Registers and offers the PROBE_TRG SOME/IP service so that CameraHost can
     *          subscribe to receive image acquisition trigger events (ADAS_PROBE2CAM_EVT).
     * @pre  ProbeComm has been constructed with a valid variant.
     * @post PROBE_TRG_Service is offered and CameraHost can subscribe.
     * @throws None.
     * @note Called by: ProbeApp at startup. Provides: PROBE_TRG_Service.
     * @requirements SWR-REQ-03-06-001
     * @rationale CameraHost must subscribe to the trigger service before any event
     *            can initiate image acquisition.
     */
    void OfferServiceProbeTrigger() noexcept;

    // =========================================================================
    // Inbound Event Handlers (PC-015 to PC-028)
    // =========================================================================

    /**
     * @brief Receives and processes development event triggers from ZAT at 100ms periodic cycle.
     * @details Deserializes the trigger data vector, validates the upload request flag,
     *          checks category and overlap limits, and initiates event data generation
     *          if all acceptance criteria are met. Updates zatTriggerReceived_ flag.
     * @param[in] triggerData Raw serialized trigger data from ZAT. Must not be empty.
     *                        Expected format defined by ZAT_hdfProbeBusOut_Service interface.
     * @pre  FindServiceZatTrigger() has been called and subscription is active.
     * @post zatTriggerReceived_ is updated. Event processing may be initiated.
     * @throws None.
     * @note Called by: ProbeApp. Provides: ZAT_hdfProbeBusOut_Service. Cyclic: 100ms.
     * @warning Must complete within the 100ms task budget.
     * @requirements SWR-REQ-03-02-001;SWR-REQ-03-02-002;SWR-REQ-03-003
     * @rationale 100ms cycle aligns with ZAT trigger generation rate.
     */
    void HandleAdasAcoreTriggerEvt(std::vector<uint8_t> triggerData) noexcept;

    /**
     * @brief Receives ZAT regular upload data for signal collection.
     * @details Deserializes and stores the regular upload data payload for inclusion
     *          in the continual data transmission to DAQ.
     * @param[in] data Raw serialized regular upload data from ZAT. Must not be empty.
     *                 Expected format defined by ZAT_hdfProbeBusOut_Service interface.
     * @pre  FindServiceZatRegularUpload() has been called and subscription is active.
     * @post Regular upload data is buffered for next continual data transmission.
     * @throws None.
     * @note Called by: ProbeApp. Provides: ZAT_hdfProbeBusOut_Service. Cyclic: 100ms.
     * @requirements SWR-REQ-01-01-001
     * @rationale Regular upload data provides steady-state signal values independent
     *            of event triggers.
     */
    void HandleZatRegularUploadData1Evt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives and evaluates the answer back from CameraHost after image acquisition trigger.
     * @details Deserializes the answer back message, calls ValidateCameraAnswerBack(),
     *          and updates cameraAnswerBackReceived_. If rejected, calls
     *          AbortImageProcessingBeforeTransmitPhase().
     * @param[in] answerData Raw serialized answer back data from CameraHost. Must not be empty.
     *                       Expected format defined by CameraHost_IMG_Service interface.
     * @pre  SendImageAcquisitionTriggerToCameraHost() has been called.
     * @post cameraAnswerBackReceived_ is updated. Image processing proceeds or is aborted.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_IMG_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-002;SWR-REQ-03-06-003;SWR-REQ-03-06-004
     * @rationale Answer back validation gates image data reception to avoid processing
     *            images from a rejected trigger.
     */
    void HandleCam2ProbeAnswerBackEvt(std::vector<uint8_t> answerData) noexcept;

    /**
     * @brief Receives camera images from CameraHost with timestamp and frame counter.
     * @details Deserializes image data, validates frame counter and timestamp,
     *          aligns image size to 4-byte boundary, and stores the image for
     *          payload construction. Increments receivedImageCount_.
     * @param[in] imageData Raw serialized image data from CameraHost. Must not be empty.
     *                      Contains JPEG image bytes, timestamp, and frame counter.
     * @pre  HandleCam2ProbeAnswerBackEvt() has been called and answer back was accepted.
     * @post receivedImageCount_ is incremented. Image data is buffered.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_IMG_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-006;SWR-REQ-03-06-007;SWR-REQ-03-06-008;SWR-REQ-03-06-009
     * @rationale 14 images covering -5 to +8 seconds provide the required temporal
     *            context around the trigger event.
     */
    void HandleCam2ProbePictureEvt(std::vector<uint8_t> imageData) noexcept;

    /**
     * @brief Receives camera FC object data from CameraHost.
     * @details Deserializes and buffers FC camera bus out data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized FC camera bus out data. Must not be empty.
     *                 Expected format defined by CameraHost_AGhdf_Service interface.
     * @pre  FindServiceCameraAghdf() has been called and subscription is active.
     * @post FC camera data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_AGhdf_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale FC camera object data is a required signal in the ADAS event log.
     */
    void HandleCam2ProbeFcCamBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives fused camera object data from CameraHost.
     * @details Deserializes and buffers fused camera CMBS FCV bus out data for
     *          inclusion in the event log data set payload.
     * @param[in] data Raw serialized fused camera object data. Must not be empty.
     *                 Expected format defined by CameraHost_AGhdf_Service interface.
     * @pre  FindServiceCameraAghdf() has been called and subscription is active.
     * @post Fused camera FCV data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_AGhdf_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale Fused object data provides combined sensor fusion output for ADAS analysis.
     */
    void HandleCam2ProbeFcCamCmbsFcvBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives lane camera data from CameraHost.
     * @details Deserializes and buffers lane camera CMBS LN bus out data for
     *          inclusion in the event log data set payload.
     * @param[in] data Raw serialized lane camera data. Must not be empty.
     *                 Expected format defined by CameraHost_AGhdf_Service interface.
     * @pre  FindServiceCameraAghdf() has been called and subscription is active.
     * @post Lane camera data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_AGhdf_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale Lane detection data is required for lane departure and keeping ADAS events.
     */
    void HandleCam2ProbeFcCamCmbsLnBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives object camera data from CameraHost.
     * @details Deserializes and buffers object camera CMBS OBJ bus out data for
     *          inclusion in the event log data set payload.
     * @param[in] data Raw serialized object camera data. Must not be empty.
     *                 Expected format defined by CameraHost_AGhdf_Service interface.
     * @pre  FindServiceCameraAghdf() has been called and subscription is active.
     * @post Object camera data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CameraHost_AGhdf_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale Object detection data is required for collision avoidance ADAS events.
     */
    void HandleCam2ProbeFcCamCmbsObjBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives TSR GCore data for event signal collection.
     * @details Deserializes and buffers TSR GCore HDF data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized TSR GCore HDF data. Must not be empty.
     *                 Expected format defined by TSR2GCoreHDF_Service interface.
     * @pre  FindServiceTsr2GCoreHdf() has been called and subscription is active.
     * @post TSR GCore data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: TSR2GCoreHDF_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale TSR GCore data provides traffic sign recognition results from the GCore processor.
     */
    void HandleTsr2GCoreHdfEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives TSR ACore data for event signal collection.
     * @details Deserializes and buffers TSR HDF data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized TSR HDF data. Must not be empty.
     *                 Expected format defined by TSR2HDF_Service interface.
     * @pre  FindServiceTsr2Hdf() has been called and subscription is active.
     * @post TSR ACore data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: TSR2HDF_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale TSR ACore data provides traffic sign recognition results from the ACore processor.
     */
    void HandleTsr2HdfEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives HDF AP to CP bus output data.
     * @details Deserializes and buffers HDF AP-to-CP bus out data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized HDF AP-to-CP bus out data. Must not be empty.
     *                 Expected format defined by hdfAp2CpBusOut_Service interface.
     * @pre  FindServiceHdfAp2CpBusOut() has been called and subscription is active.
     * @post HDF AP-to-CP data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: hdfAp2CpBusOut_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale AP-to-CP bus data captures inter-core communication signals for ADAS analysis.
     */
    void HandleHdfAp2CpBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives ICCOMMW CP to AP communication data.
     * @details Deserializes and buffers HDF CP-to-AP bus out data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized HDF CP-to-AP bus out data. Must not be empty.
     *                 Expected format defined by hdfCp2ApBusOut_Service interface.
     * @pre  FindServiceHdfCp2ApBusOut() has been called and subscription is active.
     * @post HDF CP-to-AP data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: hdfCp2ApBusOut_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale CP-to-AP bus data completes the bidirectional inter-core signal capture.
     */
    void HandleHdfCp2ApBusOutEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives SDMAP map data for event signal collection.
     * @details Deserializes and buffers SDMAP data for inclusion in
     *          the event log data set payload.
     * @param[in] data Raw serialized SDMAP data. Must not be empty.
     *                 Expected format defined by SDMAP_Service interface.
     * @pre  FindServiceSdmap() has been called and subscription is active.
     * @post SDMAP data is buffered for log data set construction.
     * @throws None.
     * @note Called by: ProbeApp. Provides: SDMAP_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-01-002
     * @rationale Map data provides localization context for event classification.
     */
    void HandleSdmapDataEvt(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Receives and processes the BDP upload completion result code from DAQ.
     * @details Evaluates the result code from DAQ after BDP data transmission.
     *          On success, triggers DeleteEventDataAfterDaqSuccess(). On failure,
     *          triggers retry or abort logic.
     * @param[in] resultCode BDP upload result code from DAQ.
     *                       Valid range: [0x0000, 0xFFFF]. 0x0000 indicates success.
     * @pre  TransmitBdpDataToDAQ() has been called and a response is pending.
     * @post Event data is deleted on success, or retry/abort is initiated on failure.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-10-001
     * @rationale Result code processing ensures reliable delivery confirmation before
     *            event data is released from memory.
     */
    void HandleResultBdpEvt(uint16_t resultCode) noexcept;

    // =========================================================================
    // Validation and Limit Checking (PC-029 to PC-033)
    // =========================================================================

    /**
     * @brief Checks if the data upload request flag is non-zero to determine valid trigger.
     * @details Returns true if the flag value is non-zero, indicating a valid upload request.
     *          Returns false if the flag is zero, indicating the trigger should be ignored.
     * @param[in] flag Upload request flag value from ZAT trigger data.
     *                 Valid range: [0x00, 0xFF]. Non-zero indicates valid request.
     * @return bool Indicates whether the upload request flag is valid.
     * @retval true  Flag is non-zero; trigger is a valid upload request.
     * @retval false Flag is zero; trigger should be ignored.
     * @pre  flag has been extracted from a valid ZAT trigger data vector.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Cyclic: 100ms.
     * @requirements SWR-REQ-03-02-002;SWR-REQ-03-003
     * @rationale Flag validation prevents spurious event processing from zero-value triggers.
     */
    [[nodiscard]] bool ValidateTriggerUploadRequestFlag(uint8_t flag) const noexcept;

    /**
     * @brief Validates whether the acceptance counter for a category has reached the configurable limit.
     * @details Looks up the category in categoryTriggerCountMap_ and compares the count
     *          against sameCategoryLimitPerDC_. Returns true if the limit has been reached.
     * @param[in] categoryId Category identifier to check.
     *                       Valid range: [0x00000000, 0xFFFFFFFF].
     * @return bool Indicates whether the per-category limit has been reached.
     * @retval true  Category acceptance count has reached or exceeded sameCategoryLimitPerDC_.
     * @retval false Category acceptance count is below the limit; trigger may be accepted.
     * @pre  categoryTriggerCountMap_ is initialized. sameCategoryLimitPerDC_ > 0.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Cyclic: 100ms.
     * @requirements SWR-REQ-03-12-001;SWR-REQ-03-12-002;SWR-REQ-03-12-005;SWR-REQ-03-12-006
     * @rationale Per-category limits prevent over-collection of similar events within a drive cycle.
     */
    [[nodiscard]] bool CheckSameCategoryLimit(uint32_t categoryId) const noexcept;

    /**
     * @brief Validates whether the active event process count exceeds the allowed overlap limit.
     * @details Compares activeEventProcessCount_ against allowedOverlapTriggerNum_.
     *          Returns true if the overlap limit has been reached or exceeded.
     * @return bool Indicates whether the overlap trigger limit has been reached.
     * @retval true  activeEventProcessCount_ >= allowedOverlapTriggerNum_; new trigger rejected.
     * @retval false activeEventProcessCount_ < allowedOverlapTriggerNum_; new trigger may proceed.
     * @pre  allowedOverlapTriggerNum_ > 0.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-04-003;SWR-REQ-03-04-004
     * @rationale Overlap limit prevents resource exhaustion from simultaneous event processing.
     */
    [[nodiscard]] bool CheckOverlapTriggerLimit() const noexcept;

    /**
     * @brief Clamps log data start time to the negative of past data retention setting when exceeded.
     * @details If startTime is more negative than -retentionLimit, returns -static_cast<int32_t>(retentionLimit).
     *          Otherwise returns startTime unchanged.
     * @param[in] startTime    Log data start time in seconds relative to trigger. Range: [INT32_MIN, 0].
     * @param[in] retentionLimit Past data retention limit in seconds. Range: [0, INT32_MAX].
     * @return int32_t Clamped log data start time in seconds.
     * @retval -static_cast<int32_t>(retentionLimit) When startTime exceeds the retention limit.
     * @retval startTime When startTime is within the retention limit.
     * @pre  retentionLimit > 0. startTime <= 0.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-02-003;SWR-REQ-03-17-007
     * @rationale Clamping prevents requesting data older than the retention buffer can provide.
     */
    [[nodiscard]] int32_t ValidateLogStartTimeClamping(int32_t startTime, uint32_t retentionLimit) noexcept;

    /**
     * @brief Clamps log data end time to +83 when the received value exceeds the limit.
     * @details If endTime > 83, returns 83. Otherwise returns endTime unchanged.
     *          The value +83 seconds is the maximum future data window defined by the system specification.
     * @param[in] endTime Log data end time in seconds relative to trigger. Range: [INT32_MIN, INT32_MAX].
     * @return int32_t Clamped log data end time in seconds.
     * @retval 83        When endTime > 83.
     * @retval endTime   When endTime <= 83.
     * @pre  None.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-02-004
     * @rationale +83 seconds is the maximum future data window; values beyond this are invalid.
     */
    [[nodiscard]] int32_t ValidateLogEndTimeClamping(int32_t endTime) noexcept;

    // =========================================================================
    // Continual Data Transmission (PC-034 to PC-037)
    // =========================================================================

    /**
     * @brief Transmits short-cycle steady-state data to DAQ via SOME/IP UDP at 100ms intervals.
     * @details Checks SuppressDataBeforeDaqEstablished() before transmitting.
     *          If DAQ communication is established, sends the payload via SendDaqFireForgetMethod().
     *          Protected by sendBufferMutex_.
     * @param[in] payload Serialized short-cycle steady-state data. Must not be empty.
     *                    Maximum size defined by DAQ SOME/IP UDP MTU.
     * @pre  FindServiceDaqContinual() has been called. daqCommunicationEstablished_ is true.
     * @post Payload is transmitted to DAQ if communication is established.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CORE_DAQ_continual_Service. Cyclic: 100ms.
     * @warning Must complete within the 100ms task budget.
     * @requirements SWR-REQ-01-001;SWR-REQ-01-06-001;SWR-REQ-01-05-001
     * @rationale 100ms cycle matches the short-cycle steady-state data update rate.
     */
    void SendContinualShortDataCyclic100ms(std::vector<uint8_t> payload) noexcept;

    /**
     * @brief Transmits long-cycle steady-state data to DAQ via SOME/IP UDP at 1000ms intervals.
     * @details Checks SuppressDataBeforeDaqEstablished() before transmitting.
     *          If DAQ communication is established, sends the payload via SendDaqFireForgetMethod().
     *          Protected by sendBufferMutex_.
     * @param[in] payload Serialized long-cycle steady-state data. Must not be empty.
     *                    Maximum size defined by DAQ SOME/IP UDP MTU.
     * @pre  FindServiceDaqContinual() has been called. daqCommunicationEstablished_ is true.
     * @post Payload is transmitted to DAQ if communication is established.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CORE_DAQ_continual_Service. Cyclic: 1000ms.
     * @requirements SWR-REQ-01-002;SWR-REQ-01-06-001;SWR-REQ-01-05-001
     * @rationale 1000ms cycle matches the long-cycle steady-state data update rate.
     */
    void SendContinualLongDataCyclic1000ms(std::vector<uint8_t> payload) noexcept;

    /**
     * @brief Transmits DAQ_UNIVERSAL_ETH_DATA_FIR_FGT message with client ID, PDU ID, and data vector.
     * @details Constructs and sends the fire-and-forget UDP message to DAQ.
     *          This is the low-level send primitive for continual data.
     *          Protected by sendBufferMutex_.
     * @param[in] clientId   DAQ client identifier. Valid range: [0x0000, 0xFFFF].
     * @param[in] pduId      PDU identifier for the data type. Valid range: [0x00000000, 0xFFFFFFFF].
     * @param[in] dataVector Serialized data payload. Must not be empty.
     * @pre  daqCommunicationEstablished_ is true. sendBufferMutex_ is not held by caller.
     * @post Message is enqueued for UDP transmission to DAQ.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_continual_Service. Cyclic: 100ms.
     * @requirements SWR-REQ-01-005;SWR-REQ-01-06-002;SWR-REQ-01-05-002
     * @rationale Fire-and-forget semantics are appropriate for continual data where
     *            occasional loss is acceptable and retransmission is not required.
     */
    void SendDaqFireForgetMethod(uint16_t clientId, uint32_t pduId,
                                 std::vector<uint8_t> dataVector) noexcept;

    /**
     * @brief Returns true to suppress data transmission when SOME/IP communication with DAQ is not yet established.
     * @details Reads daqCommunicationEstablished_ atomically and returns its negation.
     * @return bool Indicates whether data transmission should be suppressed.
     * @retval true  DAQ communication is not established; suppress transmission.
     * @retval false DAQ communication is established; transmission is allowed.
     * @pre  None.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: On_Request.
     * @requirements SWR-REQ-01-04-001;SWR-REQ-01-04-002
     * @rationale Suppression prevents data loss from transmitting before the DAQ
     *            service endpoint is ready to receive.
     */
    [[nodiscard]] bool SuppressDataBeforeDaqEstablished() const noexcept;

    // =========================================================================
    // Camera Image Acquisition (PC-038 to PC-045)
    // =========================================================================

    /**
     * @brief Sends trigger message to CameraHost when event data generation starts.
     * @details Constructs and sends the ADAS_PROBE2CAM_EVT trigger message to CameraHost
     *          via the PROBE_TRG_Service. Starts the image acquisition timeout timer.
     * @param[in] eventInfo  Event information field for the trigger message.
     *                       Valid range: [0x00000000, 0xFFFFFFFF].
     * @param[in] numEvt     Number of events field for the trigger message.
     *                       Valid range: [0x00, 0xFF].
     * @pre  OfferServiceProbeTrigger() has been called. CameraHost is subscribed.
     * @post ADAS_PROBE2CAM_EVT is sent. Image acquisition timeout timer is started.
     * @throws None.
     * @note Called by: ProbeApp. Provides: PROBE_TRG_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-001;SWR-REQ-03-18-001
     * @rationale Trigger must be sent at event start to maximize the pre-event image window.
     */
    void SendImageAcquisitionTriggerToCameraHost(uint32_t eventInfo, uint8_t numEvt) noexcept;

    /**
     * @brief Receives 14 JPEG images covering -5 to +8 seconds around the event with timeout enforcement.
     * @details Waits for receivedImageCount_ to reach 14 while enforcing the
     *          imageAcquisitionTimeoutSec_ timeout. Calls AbortImageAcquisitionOnTimeout()
     *          if the timeout expires before all images are received.
     * @pre  SendImageAcquisitionTriggerToCameraHost() has been called.
     *       cameraAnswerBackReceived_ is true.
     * @post imageAcquisitionComplete_ is true if all 14 images received.
     *       imageAcquisitionComplete_ remains false if timeout occurred.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CameraHost_IMG_Service. Condition: Event_Trigger.
     * @warning This function may block for up to imageAcquisitionTimeoutSec_ seconds.
     * @requirements SWR-REQ-03-06-006;SWR-REQ-03-06-010
     * @rationale 14 images at 1 fps cover the -5 to +8 second window around the event.
     */
    void ReceiveCameraImages() noexcept;

    /**
     * @brief Evaluates acceptance status from CameraHost answer back message.
     * @details Parses the answerBack byte and returns true if CameraHost accepted
     *          the trigger, false if rejected.
     * @param[in] answerBack Answer back status byte from CameraHost.
     *                       Valid range: [0x00, 0xFF]. 0x01 = accepted, others = rejected.
     * @return bool Indicates whether CameraHost accepted the trigger.
     * @retval true  CameraHost accepted the image acquisition trigger.
     * @retval false CameraHost rejected the image acquisition trigger.
     * @pre  answerBack has been extracted from a valid CameraHost answer back message.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-002;SWR-REQ-03-06-003
     * @rationale Answer back validation gates image reception to avoid processing
     *            images from a rejected trigger.
     */
    [[nodiscard]] bool ValidateCameraAnswerBack(uint8_t answerBack) noexcept;

    /**
     * @brief Processes trigger-stage and per-image error codes from CameraHost.
     * @details Logs the error code and updates internal error state. Trigger-stage
     *          errors (received in answer back) abort image acquisition. Per-image
     *          errors are recorded but do not abort the acquisition.
     * @param[in] errorCode Error code from CameraHost.
     *                      Valid range: [0x0000, 0xFFFF]. 0x0000 = no error.
     * @pre  errorCode has been extracted from a valid CameraHost message.
     * @post Internal error state is updated. Acquisition may be aborted on trigger-stage error.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-004;SWR-REQ-03-06-005
     * @rationale Distinguishing trigger-stage from per-image errors allows partial
     *            image sets to be transmitted when only some images fail.
     */
    void HandleCameraErrorCode(uint16_t errorCode) noexcept;

    /**
     * @brief Enforces configurable 55-second timeout for image acquisition from CameraHost.
     * @details Checks whether the elapsed time since imageTimeoutTimerStarted_ was set
     *          exceeds imageAcquisitionTimeoutSec_. Returns true if timeout has expired.
     * @return bool Indicates whether the image acquisition timeout has expired.
     * @retval true  Timeout has expired; image acquisition should be aborted.
     * @retval false Timeout has not expired; image acquisition may continue.
     * @pre  imageTimeoutTimerStarted_ is true.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-010
     * @rationale 55-second timeout is configurable to accommodate varying camera response times.
     */
    [[nodiscard]] bool EnforceImageAcquisitionTimeout() const noexcept;

    /**
     * @brief Abandons image acquisition and image transmission processing when timeout expires.
     * @details Sets imageAcquisitionComplete_ to false, clears image buffers, and
     *          resets imageTimeoutTimerStarted_. Triggers BuildDummyImageDataForGedr()
     *          if GEDR transmission is required.
     * @pre  EnforceImageAcquisitionTimeout() returned true.
     * @post imageAcquisitionComplete_ is false. Image buffers are cleared.
     *       imageTimeoutTimerStarted_ is false.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-010
     * @rationale Aborting on timeout prevents indefinite blocking of the event processing pipeline.
     */
    void AbortImageAcquisitionOnTimeout() noexcept;

    /**
     * @brief Abandons image acquisition if answer back not received before image transmission phase.
     * @details If cameraAnswerBackReceived_ is false when the image transmission phase
     *          begins, clears image state and proceeds without images.
     * @pre  Image transmission phase is about to begin.
     * @post cameraAnswerBackReceived_ and imageAcquisitionComplete_ are false.
     *       Image buffers are cleared.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-003
     * @rationale Prevents the transmission phase from waiting indefinitely for an
     *            answer back that will never arrive.
     */
    void AbortImageProcessingBeforeTransmitPhase() noexcept;

    /**
     * @brief Rounds up PictureSize to multiple of 4 bytes for payload assembly.
     * @details Computes the aligned size as ((pictureSize + 3U) / 4U) * 4U and
     *          returns a new vector with the image data zero-padded to that size.
     * @param[in] pictureSize Original picture size in bytes. Valid range: [0, 65535].
     * @param[in] imageData   Raw image data bytes. Size must be >= pictureSize.
     * @return std::vector<uint8_t> Image data zero-padded to the next 4-byte boundary.
     * @pre  imageData.size() >= static_cast<size_t>(pictureSize).
     * @post Returned vector size is a multiple of 4 and >= pictureSize.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-06-007
     * @rationale 4-byte alignment is required by the DAQ payload format specification.
     */
    [[nodiscard]] std::vector<uint8_t> CopyImageDataAligned4Bytes(
        uint16_t pictureSize,
        std::vector<uint8_t> imageData) const noexcept;

    // =========================================================================
    // Payload Construction (PC-046 to PC-053)
    // =========================================================================

    /**
     * @brief Builds header with timestamp, category optional data, and event list payload.
     * @details Constructs the type-001 event data header including system timestamp,
     *          category identifier, optional category data, and the event list.
     *          Sets eventDataFrameCounter_ for this transmission sequence.
     * @param[in] categoryId Category identifier for this event. Valid range: [0x00000000, 0xFFFFFFFF].
     * @param[in] transNum   Data transmission number for this event. Valid range: [0x00, 0xFF].
     * @return std::vector<uint8_t> Serialized type-001 header payload.
     * @pre  currentCategoryIdentifier_ and currentDataTransmissionNum_ are set.
     * @post eventDataFrameCounter_ is initialized for this event sequence.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-003;SWR-REQ-03-07-010
     * @rationale Type-001 header is the first payload in the mandatory transmission order.
     */
    [[nodiscard]] std::vector<uint8_t> FillHeaderEventDataType001(
        uint32_t categoryId,
        uint8_t transNum) const noexcept;

    /**
     * @brief Builds header with timestamp and log data set payload.
     * @details Constructs the type-002 event data header including system timestamp
     *          and the provided log data set.
     * @param[in] logDataSet Serialized log data set payload. Must not be empty.
     * @return std::vector<uint8_t> Serialized type-002 header payload.
     * @pre  logDataSet is a valid serialized log data set.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-004
     * @rationale Type-002 header wraps each 1-second log data unit for DAQ ingestion.
     */
    [[nodiscard]] std::vector<uint8_t> FillHeaderEventDataType002(
        std::vector<uint8_t> logDataSet) const noexcept;

    /**
     * @brief Builds header with timestamp and 14 image frames payload.
     * @details Constructs the type-003 event data header including system timestamp
     *          and the provided image data set containing 14 frames.
     * @param[in] imageData Serialized image data set payload. Must not be empty.
     *                      Must contain exactly 14 image frames.
     * @return std::vector<uint8_t> Serialized type-003 header payload.
     * @pre  imageData contains exactly 14 aligned image frames.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-005
     * @rationale Type-003 header wraps the complete image data set for DAQ ingestion.
     */
    [[nodiscard]] std::vector<uint8_t> FillHeaderEventDataType003(
        std::vector<uint8_t> imageData) const noexcept;

    /**
     * @brief Constructs the first event payload with category data and event list.
     * @details Serializes category optional data and the event list from triggerData
     *          into the first event payload structure.
     * @param[in] triggerData Raw trigger data from ZAT containing category and event list fields.
     *                        Must not be empty.
     * @return std::vector<uint8_t> Serialized category optional data and event list payload.
     * @pre  triggerData is a valid ZAT trigger data vector.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-001;SWR-REQ-03-07-003
     * @rationale Category and event list are the first required data element in the
     *            mandatory event data transmission order.
     */
    [[nodiscard]] std::vector<uint8_t> BuildPayloadCategoryOptionalAndEventList(
        std::vector<uint8_t> triggerData) const noexcept;

    /**
     * @brief Constructs the second event payload with log data set in chronological order.
     * @details Serializes the log data into chronological 1-second units for the
     *          second event payload structure.
     * @param[in] logData Raw log data to be serialized. Must not be empty.
     * @return std::vector<uint8_t> Serialized log data set payload in chronological order.
     * @pre  logData is a valid log data buffer.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-004;SWR-REQ-03-07-006
     * @rationale Chronological ordering is required by the DAQ log data ingestion format.
     */
    [[nodiscard]] std::vector<uint8_t> BuildPayloadLogDataSet(
        std::vector<uint8_t> logData) const noexcept;

    /**
     * @brief Constructs the third event payload with 14 image frames and timestamps.
     * @details Serializes 14 image frames with their associated timestamps into
     *          the third event payload structure.
     * @param[in] imageData Raw image data containing 14 frames with timestamps. Must not be empty.
     * @return std::vector<uint8_t> Serialized image data set payload with 14 frames.
     * @pre  imageData contains exactly 14 4-byte-aligned image frames with timestamps.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-005
     * @rationale Image data set is the third and final required payload in the
     *            mandatory event data transmission order.
     */
    [[nodiscard]] std::vector<uint8_t> BuildPayloadImageDataSet(
        std::vector<uint8_t> imageData) const noexcept;

    /**
     * @brief Divides event data into numbered segments when total size exceeds max packet size.
     * @details If payload.size() exceeds the maximum packet size, splits the payload
     *          into numbered segments. Each segment includes a data information flag
     *          set by SetDataInformationFlag(). Updates segmentNumber_ and totalSegments_.
     * @param[in] payload Complete event data payload to be segmented. Must not be empty.
     * @return std::vector<std::vector<uint8_t>> Vector of payload segments.
     *         Contains exactly one element if no segmentation is needed.
     * @pre  payload is a valid serialized event data payload.
     * @post segmentNumber_ and totalSegments_ are updated.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-08-001;SWR-REQ-03-08-002;SWR-REQ-03-08-003;SWR-REQ-03-07-008
     * @rationale Segmentation allows large event payloads to be transmitted within
     *            the SOME/IP TCP maximum message size constraint.
     */
    [[nodiscard]] std::vector<std::vector<uint8_t>> SegmentPayloadIfExceedsMaxSize(
        std::vector<uint8_t> payload) noexcept;

    /**
     * @brief Marks each segment with start, middle, end, or single-packet indicator.
     * @details Returns a data information flag byte encoding the segment position:
     *          - Single packet: isSinglePacket = true
     *          - Start segment: isStart = true, isEnd = false
     *          - Middle segment: isStart = false, isEnd = false
     *          - End segment: isStart = false, isEnd = true
     * @param[in] isSinglePacket True if the entire payload fits in one packet.
     * @param[in] isStart        True if this is the first segment of a multi-segment payload.
     * @param[in] isEnd          True if this is the last segment of a multi-segment payload.
     * @return uint8_t Data information flag byte encoding the segment position.
     * @pre  At most one of isSinglePacket, isStart, isEnd is true for multi-segment payloads.
     *       isSinglePacket implies !isStart && !isEnd.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-08-002;SWR-REQ-03-08-003
     * @rationale Data information flags allow the receiver to reassemble segmented payloads.
     */
    [[nodiscard]] uint8_t SetDataInformationFlag(
        bool isSinglePacket,
        bool isStart,
        bool isEnd) const noexcept;

    // =========================================================================
    // BDP Upload and Transmission (PC-054 to PC-066)
    // =========================================================================

    /**
     * @brief Sends transmission permission request to DAQ before sending event data.
     * @details Constructs and sends the REQUEST_BDP_UPLOAD_REQ message to DAQ.
     *          Waits for the response and returns the acceptance status.
     * @param[in] clientId   DAQ client identifier. Valid range: [0x0000, 0xFFFF].
     * @param[in] eventValue Event value field for the upload request. Valid range: [0x00000000, 0xFFFFFFFF].
     * @param[in] dataSize   Total data size in bytes. Valid range: [0, UINT32_MAX].
     * @param[in] requestNum Request sequence number. Valid range: [0x00000000, 0xFFFFFFFF].
     * @return bool Indicates whether DAQ granted the upload request.
     * @retval true  DAQ accepted the upload request; transmission may proceed.
     * @retval false DAQ rejected the upload request; retry or abort logic applies.
     * @pre  FindServiceDaqDevEvent() has been called. daqCommunicationEstablished_ is true.
     * @post bdpRequestRetryCounter_ may be incremented on failure.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-001;SWR-REQ-03-09-002
     * @rationale Upload request ensures DAQ has capacity before data is transmitted,
     *            preventing buffer overflow at the DAQ endpoint.
     */
    [[nodiscard]] bool SendBdpUploadRequestToDAQ(
        uint16_t clientId,
        uint32_t eventValue,
        uint32_t dataSize,
        uint32_t requestNum) noexcept;

    /**
     * @brief Transmits event data payload to DAQ via SOME/IP TCP BDP protocol.
     * @details Constructs and sends the TRANSMIT_BDP_REQ message with the provided
     *          contents to DAQ. Returns the transmission acceptance status.
     * @param[in] clientId   DAQ client identifier. Valid range: [0x0000, 0xFFFF].
     * @param[in] requestNum Request sequence number matching the upload request. Valid range: [0x00000000, 0xFFFFFFFF].
     * @param[in] contents   Serialized event data payload. Must not be empty.
     * @return bool Indicates whether DAQ accepted the transmitted data.
     * @retval true  DAQ accepted the transmitted data.
     * @retval false DAQ rejected the transmitted data; retry or abort logic applies.
     * @pre  SendBdpUploadRequestToDAQ() returned true for the same requestNum.
     * @post bdpTransmitRetryCounter_ may be incremented on failure.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-011;SWR-REQ-03-09-002
     * @rationale TCP-based BDP protocol provides reliable delivery for event data
     *            which cannot be retransmitted from source after the event window closes.
     */
    [[nodiscard]] bool TransmitBdpDataToDAQ(
        uint16_t clientId,
        uint32_t requestNum,
        std::vector<uint8_t> contents) noexcept;

    /**
     * @brief Evaluates DAQ response to upload request and triggers retry or proceed.
     * @details Parses the status code from the DAQ upload request response.
     *          Returns true if the status indicates acceptance, false otherwise.
     * @param[in] statusCode DAQ upload request response status code.
     *                       Valid range: [0x0000, 0xFFFF]. 0x0000 = accepted.
     * @return bool Indicates whether the upload request was accepted.
     * @retval true  Status code indicates DAQ accepted the upload request.
     * @retval false Status code indicates DAQ rejected the upload request.
     * @pre  statusCode has been received from a valid DAQ upload request response.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-002;SWR-REQ-03-09-003
     * @rationale Status code evaluation decouples protocol parsing from retry logic.
     */
    [[nodiscard]] bool HandleBdpRequestResponseStatusCode(uint16_t statusCode) const noexcept;

    /**
     * @brief Evaluates DAQ reply after data transmission.
     * @details Parses the status code from the DAQ transmit reply.
     *          Returns true if the status indicates successful reception, false otherwise.
     * @param[in] statusCode DAQ transmit reply status code.
     *                       Valid range: [0x0000, 0xFFFF]. 0x0000 = success.
     * @return bool Indicates whether the data transmission was successful.
     * @retval true  Status code indicates DAQ successfully received the data.
     * @retval false Status code indicates DAQ failed to receive the data.
     * @pre  statusCode has been received from a valid DAQ transmit reply.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-002;SWR-REQ-03-11-002
     * @rationale Transmit reply evaluation decouples protocol parsing from retry logic.
     */
    [[nodiscard]] bool HandleBdpTransmitReplyStatusCode(uint16_t statusCode) const noexcept;

    /**
     * @brief Retries upload request after configurable wait time up to maximum retry count.
     * @details Waits bdpRetryIntervalMs_ milliseconds and retries the upload request.
     *          Increments bdpRequestRetryCounter_. Returns false when bdpMaxRetryCount_
     *          is reached.
     * @return bool Indicates whether a retry was performed.
     * @retval true  Retry was performed; bdpRequestRetryCounter_ < bdpMaxRetryCount_.
     * @retval false Maximum retry count reached; caller should abort.
     * @pre  bdpRequestRetryCounter_ < bdpMaxRetryCount_.
     * @post bdpRequestRetryCounter_ is incremented.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-003;SWR-REQ-03-11-001
     * @rationale Bounded retry with configurable interval prevents infinite loops
     *            while tolerating transient DAQ unavailability.
     */
    [[nodiscard]] bool EnforceBdpRequestRetry() noexcept;

    /**
     * @brief Retries data transmission after configurable wait time up to maximum retransmission count.
     * @details Waits bdpRetryIntervalMs_ milliseconds and retries the data transmission.
     *          Increments bdpTransmitRetryCounter_. Returns false when bdpMaxRetryCount_
     *          is reached.
     * @return bool Indicates whether a retry was performed.
     * @retval true  Retry was performed; bdpTransmitRetryCounter_ < bdpMaxRetryCount_.
     * @retval false Maximum retry count reached; caller should abort.
     * @pre  bdpTransmitRetryCounter_ < bdpMaxRetryCount_.
     * @post bdpTransmitRetryCounter_ is incremented.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-11-002;SWR-REQ-03-11-005
     * @rationale Bounded retransmission with configurable interval handles transient
     *            TCP transmission failures without indefinite blocking.
     */
    [[nodiscard]] bool EnforceBdpTransmitRetry() noexcept;

    /**
     * @brief Resets all retry counters to zero after successful transmission cycle.
     * @details Sets bdpRequestRetryCounter_, bdpTransmitRetryCounter_, and
     *          gedrWriteRetryCounter_ to zero.
     * @pre  A complete transmission cycle has succeeded.
     * @post bdpRequestRetryCounter_ == 0. bdpTransmitRetryCounter_ == 0.
     *       gedrWriteRetryCounter_ == 0.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-11-006
     * @rationale Counter reset ensures each new event starts with a fresh retry budget.
     */
    void ResetRetryCountersOnSuccess() noexcept;

    /**
     * @brief Stops all DAQ transmission and discards data when max retries are exhausted.
     * @details Calls DeleteEventDataOnRetryExhaustion(), resets retry counters,
     *          and decrements activeEventProcessCount_.
     * @pre  bdpRequestRetryCounter_ >= bdpMaxRetryCount_ or
     *       bdpTransmitRetryCounter_ >= bdpMaxRetryCount_.
     * @post Event data is discarded. activeEventProcessCount_ is decremented.
     *       All retry counters are reset.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-004;SWR-REQ-03-11-003;SWR-REQ-03-11-004
     * @rationale Aborting on retry exhaustion prevents indefinite resource consumption
     *            and allows the system to accept new triggers.
     */
    void AbortEventDataTransmissionOnRetryExhaustion() noexcept;

    /**
     * @brief Ensures event data is transmitted in strict order: header+category, log data, image data.
     * @details Enforces the mandatory transmission sequence by gating each phase
     *          on completion of the previous phase. Calls the appropriate send
     *          functions in order.
     * @pre  All event data payloads have been constructed.
     * @post Event data is transmitted in the required order or aborted on failure.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-001;SWR-REQ-03-04-002
     * @rationale Strict ordering is required by the DAQ event data ingestion format.
     */
    void EnforceEventDataTransmissionOrder() noexcept;

    /**
     * @brief Transmits event data payloads at 1000ms pacing intervals.
     * @details Called from the 1000ms periodic task. Checks for pending event data
     *          and transmits the next payload in the transmission sequence.
     * @pre  Event data payloads are available for transmission.
     * @post One event data payload is transmitted per call if available.
     * @throws None.
     * @note Called by: ProbeApp. Provides: CORE_DAQ_devEvent_Service. Cyclic: 1000ms.
     * @requirements SWR-REQ-03-07-002;SWR-REQ-03-09-005
     * @rationale 1000ms pacing prevents DAQ buffer overflow from rapid successive transmissions.
     */
    void SendEventDataAtCyclic1000ms() noexcept;

    /**
     * @brief Transmits header with category optional data and event list to DAQ.
     * @details Calls SendBdpUploadRequestToDAQ() and TransmitBdpDataToDAQ() for
     *          the type-001 category data payload.
     * @param[in] payload Serialized type-001 category data payload. Must not be empty.
     * @pre  SendBdpUploadRequestToDAQ() has been called and accepted.
     * @post Category data is transmitted to DAQ or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-18-002;SWR-REQ-03-07-003
     * @rationale Category data is the first required transmission in the event data sequence.
     */
    void SendAdasCategoryDataToDAQ(std::vector<uint8_t> payload) noexcept;

    /**
     * @brief Transmits log data sets to DAQ in chronological order without waiting for images.
     * @details Iterates over logPayloads and transmits each via BDP protocol.
     *          Does not wait for image data before starting log data transmission.
     * @param[in] logPayloads Vector of serialized log data set payloads in chronological order.
     *                        Must not be empty.
     * @pre  SendAdasCategoryDataToDAQ() has completed successfully.
     * @post All log data payloads are transmitted to DAQ or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-004;SWR-REQ-03-07-006;SWR-REQ-03-07-012;SWR-REQ-03-18-004
     * @rationale Log data transmission proceeds independently of image acquisition
     *            to minimize total event data transmission latency.
     */
    void SendLogDataSetsToDAQ(std::vector<std::vector<uint8_t>> logPayloads) noexcept;

    /**
     * @brief Transmits 14 image frames with timestamps to DAQ as soon as image data is received.
     * @details Transmits the image data set payload via BDP protocol as soon as
     *          imageAcquisitionComplete_ is true.
     * @param[in] imagePayload Serialized type-003 image data set payload. Must not be empty.
     * @pre  imageAcquisitionComplete_ is true or AbortImageAcquisitionOnTimeout() was called.
     * @post Image data is transmitted to DAQ or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: CORE_DAQ_devEvent_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-07-005;SWR-REQ-03-18-006
     * @rationale Image data is transmitted as soon as available to minimize end-to-end latency.
     */
    void SendImageDataSetToDAQ(std::vector<uint8_t> imagePayload) noexcept;

    // =========================================================================
    // GEDR Transmission (PC-067 to PC-076)
    // =========================================================================

    /**
     * @brief Transmits category data to GEDR API after DAQ transmission.
     * @details Calls UploadGedrWrite() with the category data payload after
     *          DAQ category data transmission is complete.
     * @param[in] payload Serialized category data payload for GEDR. Must not be empty.
     * @pre  SendAdasCategoryDataToDAQ() has completed.
     * @post Category data is written to GEDR or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: GEDR_API_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-001;SWR-REQ-03-18-003
     * @rationale GEDR transmission follows DAQ transmission per the dual-target ordering requirement.
     */
    void SendAdasCategoryDataToGEDR(std::vector<uint8_t> payload) noexcept;

    /**
     * @brief Transmits log data in 1-second units to GEDR API.
     * @details Iterates over logPayloads and calls UploadGedrWrite() for each
     *          1-second log data unit.
     * @param[in] logPayloads Vector of serialized 1-second log data payloads. Must not be empty.
     * @pre  SendAdasCategoryDataToGEDR() has completed.
     * @post All log data payloads are written to GEDR or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: GEDR_API_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-002;SWR-REQ-03-18-005
     * @rationale 1-second granularity matches the GEDR write block size requirement.
     */
    void SendLogDataSetsToGEDR(std::vector<std::vector<uint8_t>> logPayloads) noexcept;

    /**
     * @brief Transmits 14 image data set to GEDR API.
     * @details Calls UploadGedrWrite() with the image data set payload.
     *          If image acquisition failed, uses BuildDummyImageDataForGedr().
     * @param[in] imagePayload Serialized image data set payload. Must not be empty.
     * @pre  SendLogDataSetsToGEDR() has completed.
     * @post Image data is written to GEDR or retry/abort is initiated.
     * @throws None.
     * @note Called by: ProbeComm. Provides: GEDR_API_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-002;SWR-REQ-03-18-007
     * @rationale Image data is the final required GEDR write in the event data sequence.
     */
    void SendImageDataSetToGEDR(std::vector<uint8_t> imagePayload) noexcept;

    /**
     * @brief Calls GEDR_Write with offset, data, data_size, start_block, and end_block parameters.
     * @details Invokes the GEDR write API with the provided parameters.
     *          Returns the GEDR_Write return code for error handling.
     * @param[in] offset     Byte offset within the GEDR storage block. Valid range: [0, UINT32_MAX].
     * @param[in] data       Data bytes to write. Must not be empty.
     * @param[in] dataSize   Number of bytes to write. Must equal data.size(). Valid range: [0, UINT32_MAX].
     * @param[in] startBlock True if this is the first write of a new GEDR block.
     * @param[in] endBlock   True if this is the last write of the current GEDR block.
     * @return int8_t GEDR_Write return code.
     * @retval 0   Write succeeded.
     * @retval >0  Retryable error; see HandleGedrRetryOnError().
     * @retval <0  Fatal error; see HandleGedrFatalError().
     * @pre  data.size() == static_cast<size_t>(dataSize). dataSize > 0.
     * @post GEDR storage is updated on success.
     * @throws None.
     * @note Called by: ProbeComm. Provides: GEDR_API_Service. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-001;SWR-REQ-03-17-003;SWR-REQ-03-17-004;SWR-REQ-03-17-005
     * @rationale Direct GEDR_Write call encapsulates the GEDR API boundary for testability.
     */
    [[nodiscard]] int8_t UploadGedrWrite(
        uint32_t offset,
        std::vector<uint8_t> data,
        uint32_t dataSize,
        bool startBlock,
        bool endBlock) noexcept;

    /**
     * @brief Retries GEDR write on error with bounded retry count.
     * @details If returnCode indicates a retryable error and gedrWriteRetryCounter_
     *          < gedrMaxRetryCount_, increments gedrWriteRetryCounter_ and returns true.
     * @param[in] returnCode GEDR_Write return code. Positive values indicate retryable errors.
     * @return bool Indicates whether a retry should be attempted.
     * @retval true  Retry should be attempted; gedrWriteRetryCounter_ < gedrMaxRetryCount_.
     * @retval false Maximum retry count reached or returnCode is not retryable.
     * @pre  returnCode > 0 (retryable error).
     * @post gedrWriteRetryCounter_ is incremented if retry is allowed.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-001
     * @rationale Bounded retry prevents infinite loops on persistent GEDR errors.
     */
    [[nodiscard]] bool HandleGedrRetryOnError(int8_t returnCode) noexcept;

    /**
     * @brief Acts on fatal GEDR API error codes by aborting GEDR transmission.
     * @details If returnCode indicates a fatal error (negative value), aborts
     *          GEDR transmission and resets gedrWriteRetryCounter_.
     * @param[in] returnCode GEDR_Write return code. Negative values indicate fatal errors.
     * @pre  returnCode < 0 (fatal error).
     * @post GEDR transmission is aborted. gedrWriteRetryCounter_ is reset.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-001
     * @rationale Fatal errors cannot be recovered by retry; aborting prevents
     *            data corruption in GEDR storage.
     */
    void HandleGedrFatalError(int8_t returnCode) noexcept;

    /**
     * @brief Creates header-only dummy data with length 0 when camera image collection fails.
     * @details Constructs a minimal type-003 header with image data length set to 0
     *          to satisfy the GEDR write requirement when no images are available.
     * @return std::vector<uint8_t> Serialized dummy image data header with length 0.
     * @pre  imageAcquisitionComplete_ is false (image collection failed or timed out).
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-006
     * @rationale GEDR requires a complete event record even when images are unavailable;
     *            dummy data satisfies the format requirement without fabricating image content.
     */
    [[nodiscard]] std::vector<uint8_t> BuildDummyImageDataForGedr() const noexcept;

    /**
     * @brief Clamps GEDR log start time to past data retention limit.
     * @details If startTime is more negative than the configured past data retention limit,
     *          returns the clamped value. Otherwise returns startTime unchanged.
     * @param[in] startTime Log data start time in seconds relative to trigger. Range: [INT32_MIN, 0].
     * @return int32_t Clamped GEDR log start time in seconds.
     * @pre  startTime <= 0.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-007
     * @rationale GEDR log start time must not exceed the available past data retention window.
     */
    [[nodiscard]] int32_t ValidateGedrLogStartTime(int32_t startTime) const noexcept;

    /**
     * @brief Clamps GEDR log data end time to -1 when greater than -1.
     * @details If endTime > -1, returns -1. Otherwise returns endTime unchanged.
     *          GEDR log data end time must be -1 or less.
     * @param[in] endTime Log data end time in seconds relative to trigger. Range: [INT32_MIN, INT32_MAX].
     * @return int32_t Clamped GEDR log end time in seconds.
     * @pre  None.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-008
     * @rationale GEDR only stores past data; end time must be at or before the trigger time.
     */
    [[nodiscard]] int32_t ValidateGedrLogEndTime(int32_t endTime) const noexcept;

    /**
     * @brief Does not send log data to GEDR API when log data start time is greater than -1.
     * @details Returns true (suppress) if startTime > -1, indicating that no past data
     *          is available for GEDR transmission.
     * @param[in] startTime Log data start time in seconds relative to trigger. Range: [INT32_MIN, INT32_MAX].
     * @return bool Indicates whether GEDR log data transmission should be suppressed.
     * @retval true  startTime > -1; suppress GEDR log data transmission.
     * @retval false startTime <= -1; GEDR log data transmission is allowed.
     * @pre  None.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-17-009
     * @rationale Suppressing transmission when start time is invalid prevents writing
     *            empty or future-referenced log data to GEDR storage.
     */
    [[nodiscard]] bool SuppressGedrDataWhenStartTimeInvalid(int32_t startTime) const noexcept;

    // =========================================================================
    // Dual Trigger Transmission Ordering (PC-077)
    // =========================================================================

    /**
     * @brief Ensures correct DAQ-before-GEDR ordering for both probe and GEDR triggers.
     * @details Enforces the transmission sequence:
     *          1. Send category data to DAQ
     *          2. Send category data to GEDR
     *          3. Send log data to DAQ
     *          4. Send log data to GEDR
     *          5. Send image data to DAQ
     *          6. Send image data to GEDR
     *          DAQ transmission always precedes GEDR transmission for each data type.
     * @pre  All event data payloads have been constructed.
     * @post All event data is transmitted to both DAQ and GEDR in the required order.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-18-001;SWR-REQ-03-18-002;SWR-REQ-03-18-003;SWR-REQ-03-18-004;SWR-REQ-03-18-005
     * @rationale DAQ-before-GEDR ordering ensures that the primary data store receives
     *            data before the secondary store, consistent with data priority requirements.
     */
    void EnforceDualTriggerTransmissionOrder() noexcept;

    // =========================================================================
    // ZAT Status Reporting (PC-078 to PC-080)
    // =========================================================================

    /**
     * @brief Periodically sends accepted data transmission numbers and category identifiers to ZAT.
     * @details Constructs and sends the ADAS_ACore_PROBE2ZAT_EVT message containing
     *          acceptedDataTransmissionNum_ and acceptedCategoryIdentifier_ to ZAT
     *          via the probeEventStt_Service.
     * @pre  OfferServiceProbeEventStt() has been called. ZAT is subscribed.
     * @post ADAS_ACore_PROBE2ZAT_EVT is sent with current acceptance status.
     * @throws None.
     * @note Called by: ProbeApp. Provides: probeEventStt_Service. Cyclic: 100ms.
     * @requirements SWR-REQ-03-05-001;SWR-REQ-03-05-004
     * @rationale Periodic status reporting allows ZAT to track which triggers were accepted
     *            and update its own trigger arbitration state.
     */
    void SendTriggerAcceptanceStatusToZAT() noexcept;

    /**
     * @brief Indicates trigger rejection when maximum duplicates exceeded.
     * @details Constructs and sends the ADAS_ACore_PROBE2ZAT_EVT rejection message
     *          to ZAT when a trigger is rejected due to category limit or overlap limit.
     * @pre  OfferServiceProbeEventStt() has been called. ZAT is subscribed.
     * @post ADAS_ACore_PROBE2ZAT_EVT rejection message is sent to ZAT.
     * @throws None.
     * @note Called by: ProbeComm. Provides: probeEventStt_Service. Cyclic: 100ms.
     * @requirements SWR-REQ-03-05-002
     * @rationale Rejection notification allows ZAT to immediately update its trigger
     *            arbitration state without waiting for the next acceptance status cycle.
     */
    void SendTriggerRejectionToZAT() noexcept;

    /**
     * @brief Periodically reports the allowed overlap trigger number to trigger arbitration function.
     * @details Constructs and sends the ADAS_ACore_PROBE2ZAT_EVT message containing
     *          allowedOverlapTriggerNum_ to ZAT via the probeEventStt_Service.
     * @pre  OfferServiceProbeEventStt() has been called. ZAT is subscribed.
     * @post ADAS_ACore_PROBE2ZAT_EVT is sent with current allowed overlap count.
     * @throws None.
     * @note Called by: ProbeApp. Provides: probeEventStt_Service. Cyclic: 100ms.
     * @requirements SWR-REQ-03-05-003;SWR-REQ-03-04-004
     * @rationale ZAT uses the allowed overlap count to gate new trigger generation
     *            and prevent exceeding the ProbeComm processing capacity.
     */
    void SendAllowedOverlapCountToZAT() noexcept;

    // =========================================================================
    // Event Data Lifecycle (PC-081 to PC-084)
    // =========================================================================

    /**
     * @brief Removes development event data from memory after successful DAQ transmission confirmation.
     * @details Clears all event data buffers associated with the current event after
     *          HandleResultBdpEvt() confirms successful DAQ reception.
     *          Decrements activeEventProcessCount_ if GEDR transmission is also complete.
     * @pre  HandleResultBdpEvt() has confirmed successful DAQ transmission.
     * @post Event data buffers are cleared. activeEventProcessCount_ may be decremented.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-10-001
     * @rationale Releasing event data after DAQ confirmation prevents memory exhaustion
     *            from accumulating unconfirmed event data.
     */
    void DeleteEventDataAfterDaqSuccess() noexcept;

    /**
     * @brief Removes development event data from memory after successful GEDR transmission.
     * @details Clears all event data buffers associated with the current event after
     *          GEDR write completion is confirmed.
     *          Decrements activeEventProcessCount_ if DAQ transmission is also complete.
     * @pre  UploadGedrWrite() has completed successfully for all event data.
     * @post Event data buffers are cleared. activeEventProcessCount_ may be decremented.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-10-002
     * @rationale Releasing event data after GEDR confirmation ensures both destinations
     *            have received the data before memory is freed.
     */
    void DeleteEventDataAfterGedrSuccess() noexcept;

    /**
     * @brief Discards event data when maximum retries are exhausted.
     * @details Clears all event data buffers for the current event and decrements
     *          activeEventProcessCount_.
     * @pre  EnforceBdpRequestRetry() or EnforceBdpTransmitRetry() returned false.
     * @post Event data buffers are cleared. activeEventProcessCount_ is decremented.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-09-004;SWR-REQ-03-11-004
     * @rationale Discarding on retry exhaustion prevents stale event data from
     *            blocking new event processing indefinitely.
     */
    void DeleteEventDataOnRetryExhaustion() noexcept;

    /**
     * @brief Retains event data until transmission to all designated destinations is confirmed successful.
     * @details Returns true if event data should be retained because at least one
     *          designated destination (DAQ or GEDR) has not yet confirmed successful reception.
     * @return bool Indicates whether event data should be retained.
     * @retval true  At least one destination has not confirmed reception; retain data.
     * @retval false All destinations have confirmed reception; data may be released.
     * @pre  None.
     * @post No state is modified.
     * @throws None.
     * @note Called by: ProbeComm. Condition: Event_Trigger.
     * @requirements SWR-REQ-03-10-003
     * @rationale Data retention until all targets confirm prevents premature release
     *            that could cause data loss if one destination fails after another succeeds.
     */
    [[nodiscard]] bool CheckRetainDataUntilAllTargetsComplete() const noexcept;

    // =========================================================================
    // Drive Cycle Management (PC-085)
    // =========================================================================

    /**
     * @brief Resets all per-category acceptance counters to zero when a new driving cycle begins.
     * @details Clears all entries in categoryTriggerCountMap_ and resets
     *          activeEventProcessCount_ to zero.
     * @pre  A new driving cycle has been detected.
     * @post categoryTriggerCountMap_ is empty. activeEventProcessCount_ == 0.
     * @throws None.
     * @note Called by: ProbeApp at startup.
     * @requirements SWR-REQ-03-12-004
     * @rationale Per-category limits are defined per drive cycle; resetting at cycle
     *            start ensures fair trigger acceptance across drive cycles.
     */
    void ResetCategoryCounterOnNewDriveCycle() noexcept;

    // =========================================================================
    // Development Log File Output (PC-086 to PC-089)
    // =========================================================================

    /**
     * @brief Enables DAQ data logging in development builds only.
     * @details Sets devLogEnabled_ to true. Log file output is only active when
     *          this function has been called and the build is a development build.
     * @pre  None.
     * @post devLogEnabled_ is true.
     * @throws None.
     * @note Called by: ProbeApp at startup.
     * @requirements SWR-REQ-01-11-001;SWR-REQ-03-15-001
     * @rationale Development logging must be explicitly enabled to prevent accidental
     *            log file generation in production builds.
     */
    void EnableDevLogFileOutput() noexcept;

    /**
     * @brief Disables DAQ data logging for production builds.
     * @details Sets devLogEnabled_ to false. No log files are written after this call.
     * @pre  None.
     * @post devLogEnabled_ is false.
     * @throws None.
     * @note Called by: ProbeApp at startup.
     * @requirements SWR-REQ-01-11-003;SWR-REQ-03-15-003
     * @rationale Explicit disable ensures production builds do not generate log files
     *            that could consume storage or expose sensitive data.
     */
    void DisableDevLogFileOutput() noexcept;

    /**
     * @brief Writes data sent to DAQ into log file with _snd_YYYYMMDD-HHMMSS.log naming.
     * @details If devLogEnabled_ is true, appends the data bytes to a log file
     *          named with the _snd_YYYYMMDD-HHMMSS.log convention.
     *          Does nothing if devLogEnabled_ is false.
     * @param[in] data Data bytes that were sent to DAQ. Must not be empty.
     * @pre  devLogEnabled_ is true for any output to occur.
     * @post Data is appended to the send log file if logging is enabled.
     * @throws None.
     * @note Called by: ProbeComm. Condition: On_Request.
     * @requirements SWR-REQ-01-11-001;SWR-REQ-01-11-002;SWR-REQ-03-15-001
     * @rationale Sent data logging enables post-hoc verification of DAQ transmission content.
     */
    void WriteDevLogSentData(std::vector<uint8_t> data) noexcept;

    /**
     * @brief Writes data received from DAQ into log file with _rcv_YYYYMMDD-HHMMSS.log naming.
     * @details If devLogEnabled_ is true, appends the data bytes to a log file
     *          named with the _rcv_YYYYMMDD-HHMMSS.log convention.
     *          Does nothing if devLogEnabled_ is false.
     * @param[in] data Data bytes received from DAQ. Must not be empty.
     * @pre  devLogEnabled_ is true for any output to occur.
     * @post Data is appended to the receive log file if logging is enabled.
     * @throws None.
     * @note Called by: ProbeComm. Condition: On_Request.
     * @requirements SWR-REQ-03-15-002
     * @rationale Received data logging enables post-hoc verification of DAQ response content.
     */
    void WriteDevLogReceivedData(std::vector<uint8_t> data) noexcept;

private:

    // =========================================================================
    // Constants
    // =========================================================================

    /**
     * @brief Maximum log data end time in seconds relative to trigger.
     * @details Defined as +83 seconds per system specification. Values exceeding
     *          this limit are clamped by ValidateLogEndTimeClamping().
     */
    static constexpr int32_t kMaxLogEndTimeSec{83};

    /**
     * @brief Maximum GEDR log data end time in seconds relative to trigger.
     * @details GEDR only stores past data; end time must be at or before -1 second.
     */
    static constexpr int32_t kMaxGedrLogEndTimeSec{-1};

    /**
     * @brief Number of camera images expected per event.
     * @details 14 images covering -5 to +8 seconds at 1 fps around the trigger event.
     */
    static constexpr uint8_t kExpectedImageCount{14U};

    /**
     * @brief Default image acquisition timeout in seconds.
     * @details Configurable; default value is 55 seconds per system specification.
     */
    static constexpr uint32_t kDefaultImageAcquisitionTimeoutSec{55U};

    /**
     * @brief Byte alignment requirement for image data payloads.
     * @details Image data must be padded to a multiple of 4 bytes per DAQ format specification.
     */
    static constexpr uint8_t kImageDataAlignment{4U};

    // =========================================================================
    // Private Member Variables
    // =========================================================================

    ProbeCommVariant* variant_{nullptr};  ///< @brief Non-owning pointer to variant configuration. Must not be nullptr after construction.

    std::atomic<bool> daqCommunicationEstablished_{false};  ///< @brief True when SOME/IP DAQ service is discovered and communication is established. Thread-safe via std::atomic.

    bool zatTriggerReceived_{false};  ///< @brief True when a ZAT trigger event has been received in the current 100ms cycle.

    bool cameraAnswerBackReceived_{false};  ///< @brief True when CameraHost has sent an accepted answer back for the current trigger.

    bool imageAcquisitionComplete_{false};  ///< @brief True when all 14 expected camera images have been received.

    uint32_t bdpRequestRetryCounter_{0U};  ///< @brief Current BDP upload request retry count. Range: [0, bdpMaxRetryCount_].

    uint32_t bdpTransmitRetryCounter_{0U};  ///< @brief Current BDP data transmit retry count. Range: [0, bdpMaxRetryCount_].

    uint32_t gedrWriteRetryCounter_{0U};  ///< @brief Current GEDR write retry count. Range: [0, gedrMaxRetryCount_].

    uint32_t activeEventProcessCount_{0U};  ///< @brief Number of events currently being processed. Must not exceed allowedOverlapTriggerNum_.

    std::map<uint32_t, uint32_t> categoryTriggerCountMap_;  ///< @brief Maps category ID to acceptance count within the current drive cycle. Key: categoryId, Value: acceptance count.

    uint8_t currentDataTransmissionNum_{0U};  ///< @brief Data transmission sequence number for the current event. Range: [0x00, 0xFF].

    uint32_t currentCategoryIdentifier_{0U};  ///< @brief Category identifier for the current event being processed. Valid range: [0x00000000, 0xFFFFFFFF].

    uint8_t dataUploadRequestFlg_{0U};  ///< @brief Upload request flag extracted from the latest ZAT trigger. Non-zero indicates a valid upload request.

    int32_t effectiveLogStartTime_{0};  ///< @brief Clamped log data start time in seconds relative to trigger. Range: [-retentionLimit, 0].

    int32_t effectiveLogEndTime_{0};  ///< @brief Clamped log data end time in seconds relative to trigger. Range: [INT32_MIN, kMaxLogEndTimeSec].

    uint8_t imageFrameCounter_{0U};  ///< @brief Frame counter for the current image being processed. Range: [0, kExpectedImageCount - 1].

    uint8_t receivedImageCount_{0U};  ///< @brief Number of camera images received for the current event. Range: [0, kExpectedImageCount].

    uint32_t eventDataFrameCounter_{0U};  ///< @brief Frame counter for event data payload sequencing. Incremented per transmitted payload.

    uint16_t segmentNumber_{0U};  ///< @brief Current segment number within a segmented payload. Range: [0, totalSegments_ - 1].

    uint16_t totalSegments_{0U};  ///< @brief Total number of segments for the current payload. Range: [1, UINT16_MAX].

    uint8_t acceptedDataTransmissionNum_{0U};  ///< @brief Data transmission number of the most recently accepted trigger. Reported to ZAT.

    uint32_t acceptedCategoryIdentifier_{0U};  ///< @brief Category identifier of the most recently accepted trigger. Reported to ZAT.

    uint32_t allowedOverlapTriggerNum_{0U};  ///< @brief Maximum number of simultaneously active event processes allowed. Reported to ZAT.

    uint32_t imageAcquisitionTimeoutSec_{kDefaultImageAcquisitionTimeoutSec};  ///< @brief Configurable image acquisition timeout in seconds. Default: 55. Range: [1, UINT32_MAX].

    uint32_t bdpRetryIntervalMs_{0U};  ///< @brief Wait interval in milliseconds between BDP retry attempts. Range: [0, UINT32_MAX].

    uint32_t bdpMaxRetryCount_{0U};  ///< @brief Maximum number of BDP request and transmit retry attempts. Range: [0, UINT32_MAX].

    uint32_t gedrMaxRetryCount_{0U};  ///< @brief Maximum number of GEDR write retry attempts. Range: [0, UINT32_MAX].

    uint32_t sameCategoryLimitPerDC_{0U};  ///< @brief Maximum number of accepted triggers per category per drive cycle. Range: [0, UINT32_MAX].

    bool devLogEnabled_{false};  ///< @brief True when development log file output is enabled. Must be false in production builds.

    bool imageTimeoutTimerStarted_{false};  ///< @brief True when the image acquisition timeout timer has been started.

    std::chrono::steady_clock::time_point imageTimeoutStartPoint_{};  ///< @brief Time point when the image acquisition timeout timer was started.

    mutable std::mutex sendBufferMutex_;  ///< @brief Mutex protecting DAQ send buffer access. Must be held when calling SendDaqFireForgetMethod().

    mutable std::mutex cameraMutex_;  ///< @brief Mutex protecting camera image state (receivedImageCount_, imageAcquisitionComplete_, imageTimeoutTimerStarted_).
};

}  // namespace probe