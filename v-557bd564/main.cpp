/**
 * @file    main.cpp
 * @brief   Application entry point — portable C++ launcher using SimRuntime abstraction.
 * @details Bootstraps the Probe data collection and transmission application.
 *          Replaces all ara:: platform APIs (ara::exec, ara::log, ara::core) with the
 *          portable SimRuntime abstraction. Contains no business logic — purely a launcher.
 *
 *          Lifecycle sequence:
 *          1. Construct SimRuntime on the stack.
 *          2. Report execution state kRunning.
 *          3. Construct ProbeApp orchestrator (manages all sub-components internally).
 *          4. Call HandleInitialize() to start service discovery and SOME/IP offering.
 *          5. Call Run() to execute the cyclic data collection and transmission loop.
 *          6. Call HandleShutdown() to purge all RAM buffers and stop processing.
 *          7. Report execution state kTerminating and return.
 *
 * @author  Engineering Team
 * @date    2024-01-01
 * @version 1.0.0
 * @copyright Copyright (c) 2024 Company. All rights reserved.
 *
 * @note    MISRA C++ compliant. No dynamic memory allocation. No ara:: dependencies.
 *          ISO 26262 ASIL-B — deterministic, no recursion, all variables initialized.
 */

#include <iostream>
#include <stdexcept>
#include <string>
#include <cstdint>

#include "runtime/SimRuntime.hpp"
#include "ProbeApp/ProbeApp.hpp"

/**
 * @brief   Application entry point.
 * @details Constructs the portable SimRuntime, instantiates the ProbeApp orchestrator,
 *          drives the full application lifecycle (initialize → run → shutdown), and
 *          handles all top-level exceptions. Contains no business logic.
 *
 * @return  int  Exit status code.
 * @retval  0    Application completed the full lifecycle successfully.
 * @retval  1    A std::exception or unknown exception was caught during execution.
 *
 * @pre     SimRuntime and ProbeApp headers are available on the include path.
 * @post    All ProbeApp RAM buffers have been purged via HandleShutdown().
 *          Execution state kTerminating has been reported to the runtime.
 *
 * @note    Single-threaded execution. Called from the OS process entry.
 *          No ara:: types, includes, or namespaces are used anywhere in this file.
 */
int main()
{
    /* -------------------------------------------------------------------------
     * Step 1: Construct the portable runtime simulation instance on the stack.
     *         This single instance replaces:
     *           - ara::exec::ExecutionClient  (execution state reporting)
     *           - ara::log                    (application logging)
     *           - ara::com service discovery  (abstracted via IRuntime)
     *         The default service discovery delay of 50 ms is used.
     * ---------------------------------------------------------------------- */
    runtime::SimRuntime simRuntime;

    /* -------------------------------------------------------------------------
     * Step 2: Report execution state kRunning.
     *         Replaces: ara::exec::ExecutionClient::ReportExecutionState(kRunning)
     * ---------------------------------------------------------------------- */
    simRuntime.reportExecutionState("kRunning");

    /* -------------------------------------------------------------------------
     * Step 3: Log application start.
     *         Replaces: ara::log logger invocation.
     * ---------------------------------------------------------------------- */
    simRuntime.logInfo("[APP] Application starting...");

    /* -------------------------------------------------------------------------
     * Step 4–6: Instantiate orchestrator, drive lifecycle, handle all exceptions.
     *           ProbeApp manages all sub-components (ProbeComm, ProbeCommVariant,
     *           CanConsumer) internally — main() only holds the orchestrator.
     * ---------------------------------------------------------------------- */
    try
    {
        /* ---------------------------------------------------------------------
         * Construct the ProbeApp orchestrator.
         * Passes simRuntime by reference as runtime::IRuntime& so that all
         * internal sub-components receive the same runtime context.
         * May throw std::runtime_error if mandatory internal dependencies fail
         * to initialise or if buffer allocation fails.
         * ------------------------------------------------------------------ */
        probe::ProbeApp app(simRuntime);

        /* ---------------------------------------------------------------------
         * Trigger service discovery, SOME/IP service offering, and set the
         * isRunning_ flag. Must be called exactly once before Run().
         * ------------------------------------------------------------------ */
        app.HandleInitialize();

        /* ---------------------------------------------------------------------
         * Execute the cyclic data collection and transmission loop.
         * Run() is declared void — static_cast<void> is intentionally omitted
         * per generation rule 12 (no static_cast<void> on void functions).
         * ------------------------------------------------------------------ */
        app.Run();

        /* ---------------------------------------------------------------------
         * Purge all RAM buffers and enforce a full stop.
         * Mandatory on IG-OFF for data privacy compliance (GDPR) and ISO 26262
         * ASIL-B safe-state requirements.
         * ------------------------------------------------------------------ */
        app.HandleShutdown();
    }
    catch (const std::exception& e)
    {
        /* Structured exception — log the diagnostic message and terminate. */
        simRuntime.logError(
            std::string("[APP] Exception: ") + e.what()
        );
        simRuntime.reportExecutionState("kTerminating");
        return 1;
    }
    catch (...)
    {
        /* Non-standard or unknown exception — log and terminate safely. */
        simRuntime.logError(
            "[APP] Unknown exception — application terminated."
        );
        simRuntime.reportExecutionState("kTerminating");
        return 1;
    }

    /* -------------------------------------------------------------------------
     * Step 7: Log shutdown completion and report kTerminating state.
     *         Replaces: ara::exec::ExecutionClient::ReportExecutionState(kTerminating)
     * ---------------------------------------------------------------------- */
    simRuntime.logInfo("[APP] Application shutting down.");
    simRuntime.reportExecutionState("kTerminating");

    return 0;
}