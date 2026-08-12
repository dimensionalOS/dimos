package com.dimensional.mini4pro.command

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The DJI half of M2, driven through a fake [ActionPort] — every decision `MsdkFlightActions`
 * makes about a real aircraft, made here against a recorded one.
 *
 * The suite is written to fail loudly for the mistakes that would hurt someone at half a metre:
 *
 *  - auto-confirming a landing this bridge did not start (the RC pilot's confirmation dialog,
 *    answered by software behind their back)
 *  - never confirming a landing it *did* start (the aircraft hovers at 0.5 m while QGC's mode
 *    reads "Land" — this bridge's characteristic failure, from the DJI side)
 *  - confirming more than once, or retrying a refused confirm (a command loop aimed at an
 *    aircraft)
 *  - touching KeyManager while the SDK is unregistered or no product is connected
 *  - paraphrasing DJI's error name on its way to the operator
 *
 * Mutation-checked 2026-07-26 — each breakage was made deliberately, the failing tests counted,
 * and the code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | initiated-by-us check inverted (`!ourLanding` in onConfirmationNeeded) | 10 |
 *  | confirm call dropped (gate evaluated, `port.confirmLanding` never called) | 7 |
 *  | wrong key: `returnToHome()` performs the landing start | 2 |
 *  | wrong key: the confirm performs a landing start | 7 |
 *  | `ourLanding` set at call time instead of in startAutoLanding's onSuccess | 2 |
 *  | `confirmSent` guard removed | 3 |
 *  | `sawLandingMode` gate removed (a re-delivered false clears the claim) | 1 |
 *  | availability check removed from `land()` | 2 |
 *  | go-home error name prettified (`lowercase().replace('_',' ')`) | 1 |
 *  | landing-start error name prettified the same way | 2 |
 *  | episode end stops clearing `ourLanding` | 2 |
 *
 * The capability pre-check added later the same day was mutation-checked the same way:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | capability check inverted on `returnToHome` | 5 |
 *  | capability check removed from `returnToHome` | 3 |
 *  | capability check removed from `land` | 2 |
 *  | capability refusal reported as `Refused` rather than `Unavailable` | 2 |
 *  | capability answer paraphrased instead of DJI's own word | 1 |
 *  | an unperformable landing still subscribes to the confirmation keys | 1 |
 *
 * Takeoff was added on 2026-07-26 (`docs/decisions/2026-07-26-takeoff.md`) and brings the one
 * decision in this class that is not about the SDK — `REQUIRE_SIMULATOR`. Mutation-checked the
 * same way:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | simulator gate removed | 3 |
 *  | simulator gate inverted | 9 |
 *  | `simulatedFlight` default flipped to fail **open** | 1 |
 *  | simulator refusal reported as `Refused` rather than `Unavailable` | 3 |
 *  | wrong key: `takeoff()` performs go-home | 6 |
 *  | capability check removed from `takeoff()` | 2 |
 *  | availability check removed from `takeoff()` | 2 |
 *  | DJI's takeoff error name prettified | 1 |
 *  | `takeoff()` subscribes to the landing keys | 1 |
 *
 * The prettification mutation first read **0** and that was a hole, not an equivalent mutant: the
 * test fired only `-7`, the measured flaky-takeoff code, and `"-7".lowercase()` is `"-7"`. The
 * fixture was invariant under the very transformation the test existed to catch. It now fires a
 * lettered name as well.
 *
 * The Stage C guided-confirm scope (2026-07-28, re-measured after the commit-to-DJI pivot) was
 * mutation-checked the same way, against the 2424-test suite; the cross-layer twins that also
 * kill several of these live in `guided/GuidedAutolandTest`, whose KDoc holds the full Stage C
 * table:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | confirm gate: cone check dropped | 1 |
 *  | confirm gate: freshness check dropped | 2 |
 *  | confirm gate: interlock check dropped | 2 |
 *  | confirm gate: single-confirm discipline dropped (`confirmSent` unset) | 3 |
 *
 * Read the capability tests knowing what the flag is: `DJIKey.canPerformAction()` is a constant
 * baked into the key's static initialiser, **not** a pre-flight check, and it is `true` for both
 * of these keys on MSDK 5.18.0 (bytecode in `ActionPort.canStartGoHome`'s KDoc). DJI ships no
 * synchronous way to ask whether an action will be honoured now. So every `false` below is a
 * hypothetical the guard exists to fail closed on — an SDK upgrade demoting the key, or a
 * miswiring — and none of it could have prevented the silence measured on 2026-07-26.
 */
class MsdkFlightActionsTest {

    // ------------------------------------------------------------------- rig

    private val port = FakeActionPort()
    private val djiErrors = mutableListOf<String>()
    private var announced = 0
    private val logs = mutableListOf<String>()

    /**
     * The injected monotonic clock, in milliseconds. Nothing in this suite sleeps: time is a
     * variable a test assigns, which is the only way the [MsdkFlightActions.LANDING_MODE_GRACE_MS]
     * window is testable at all — the alternative is a 5-second unit test that is still a race.
     */
    private var nowMs = 0L

    /**
     * What `Bridge.simulatorReportsRunning` would answer — DJI's observed
     * `KeyIsSimulatorStarted`, not our own request for one.
     *
     * Starts `true` so the pre-existing suite and every non-takeoff test are unaffected by
     * `MsdkFlightActions.REQUIRE_SIMULATOR`; the gate's own tests set it explicitly in both
     * directions.
     */
    private var simulated = true

    /**
     * The Stage C rig half. [clearance] null (the default) means no guided landing is live —
     * every pre-Stage-C test runs with the guided scope structurally inert, which is itself
     * the property those tests now also pin: a null clearance changes nothing.
     */
    private var interlock = true
    private var clearance: AutolandClearance? = null
    private var autolandConfirms = 0
    private val recorded = mutableListOf<Triple<String, String, Boolean>>()

    private val actions = MsdkFlightActions(
        port = port,
        reportAsyncDjiError = { djiErrors.add(it) },
        announceLandingConfirmed = { announced++ },
        simulatedFlight = { simulated },
        interlockEnabled = { interlock },
        autolandClearance = { clearance },
        onAutolandConfirmed = { autolandConfirms++ },
        recordEvent = { code, message, warn -> recorded += Triple(code, message, warn) },
        log = { logs.add(it) },
        nowMs = { nowMs },
    )

    /** Move the injected clock forward. */
    private fun elapse(ms: Long) {
        nowMs += ms
    }

    /**
     * A recorded MSDK. Callbacks are captured, never resolved on their own: each test plays
     * DJI's side explicitly, in the order it wants, which is what makes the asynchronous races
     * (`onSuccess` after return, a re-delivered `false` before the descent engages) writable as
     * straight-line code.
     */
    private class FakeActionPort : ActionPort {
        var reason: String? = null

        /**
         * What `DJIKey.canPerformAction()` reports for the two action keys. True in production on
         * MSDK 5.18.0 — it is a static flag, not a pre-flight check — so `false` here is the
         * SDK-upgrade / miswired-key case the guard exists for.
         */
        var canGoHome = true
        var canAutoLand = true
        var canStopLand = true
        var canTakeoff = true

        /** Every action call, in order — the audit trail of what would have reached DJI. */
        val performed = mutableListOf<String>()

        /** Capability checks, in order, so a test can assert one happened *before* the action. */
        val asked = mutableListOf<String>()

        val goHomeFailures = mutableListOf<(String) -> Unit>()
        val takeoffFailures = mutableListOf<(String) -> Unit>()
        val landStarts = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()
        val stopFailures = mutableListOf<(String) -> Unit>()
        val confirms = mutableListOf<Pair<() -> Unit, (String) -> Unit>>()

        var confirmationListener: ((Boolean?) -> Unit)? = null
        var landingModeListener: ((Boolean?) -> Unit)? = null
        var cancelled = false

        override fun unavailableReason(): String? = reason

        override fun canStartGoHome(): Boolean {
            asked.add("canStartGoHome")
            return canGoHome
        }

        override fun canStartAutoLanding(): Boolean {
            asked.add("canStartAutoLanding")
            return canAutoLand
        }

        override fun canStopAutoLanding(): Boolean {
            asked.add("canStopAutoLanding")
            return canStopLand
        }

        override fun canStartTakeoff(): Boolean {
            asked.add("canStartTakeoff")
            return canTakeoff
        }

        override fun startGoHome(onFailure: (String) -> Unit) {
            performed.add("startGoHome")
            goHomeFailures.add(onFailure)
        }

        override fun startTakeoff(onFailure: (String) -> Unit) {
            performed.add("startTakeoff")
            takeoffFailures.add(onFailure)
        }

        override fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            performed.add("startAutoLanding")
            landStarts.add(onSuccess to onFailure)
        }

        override fun stopAutoLanding(onFailure: (String) -> Unit) {
            performed.add("stopAutoLanding")
            stopFailures.add(onFailure)
        }

        override fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            performed.add("confirmLanding")
            confirms.add(onSuccess to onFailure)
        }

        override fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit) {
            confirmationListener = onDelivery
        }

        override fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit) {
            landingModeListener = onDelivery
        }

        override fun cancelListens() {
            cancelled = true
            confirmationListener = null
            landingModeListener = null
        }
    }

    /** DJI's side of a clean landing start: the performAction was accepted. */
    private fun djiAcceptsLandingStart(index: Int = 0) = port.landStarts[index].first()

    private fun confirmationNeeded() = port.confirmationListener!!.invoke(true)

    private val confirmCount get() = port.performed.count { it == "confirmLanding" }

    // ------------------------------------------------- the calls themselves

    @Test
    fun `returnToHome performs the go-home action and claims only that it asked`() {
        val outcome = actions.returnToHome()

        assertEquals(listOf("startGoHome"), port.performed)
        // Requested, not success: DJI has not even answered yet — the callback is still
        // unresolved when this returns, which is the non-blocking contract on mavlink-rx.
        assertTrue(outcome is ActionOutcome.Requested)
    }

    @Test
    fun `land performs the auto-landing action and claims only that it asked`() {
        val outcome = actions.land()

        assertEquals(listOf("startAutoLanding"), port.performed)
        assertTrue(outcome is ActionOutcome.Requested)
    }

    @Test
    fun `land subscribes to the confirmation keys before asking DJI to descend`() {
        actions.land()

        // Before, not after: if DJI accepted the start and stalled at 0.5 m in the gap between
        // performAction and listen, the confirmation window would open unobserved.
        assertTrue(port.confirmationListener != null)
        assertTrue(port.landingModeListener != null)
    }

    // ------------------------------------------------------------------ takeoff

    @Test
    fun `takeoff performs the takeoff action and claims only that it asked`() {
        val outcome = actions.takeoff()

        assertEquals(listOf("startTakeoff"), port.performed)
        // Requested, never Success: DJI has not answered, and on this airframe it may never
        // (2026-07-26). The aircraft leaving the ground is observed through the telemetry keys,
        // not inferred from this return value.
        assertTrue(outcome is ActionOutcome.Requested)
    }

    @Test
    fun `takeoff performs the takeoff key and not the go-home or landing key`() {
        actions.takeoff()

        // The mutation this catches is a copy-paste of returnToHome's body. It would still
        // return Requested, still announce, still look right in a log — and command the wrong
        // action on a real aircraft.
        assertEquals(listOf("startTakeoff"), port.performed)
    }

    @Test
    fun `takeoff subscribes to nothing and leaves no landing claim behind`() {
        actions.takeoff()
        // A takeoff acquires no standing permission to send anything later, unlike an accepted
        // landing start. Proven by the observable consequence: DJI raising the landing
        // confirmation question afterwards must not be answered.
        assertNull(port.confirmationListener)
        assertNull(port.landingModeListener)

        // And if a *foreign* landing later reaches the stall — via a land() that subscribes —
        // the takeoff must not have authorised its confirm.
        port.canAutoLand = false // keep land() from starting one of ours
        actions.land()
        assertEquals(listOf("startTakeoff"), port.performed)
    }

    @Test
    fun `takeoff passes DJI's error name to the operator verbatim`() {
        // -7 is the documented flaky-takeoff code on this exact airframe (DJI #783), so it goes
        // first. It is also *unmutatable* — lowercasing or re-spacing "-7" leaves "-7" — so a
        // lettered name is fired too, and both must survive untouched. Without the second one a
        // paraphrasing bug would pass this test while turning FC_AUTH_STATE into something the
        // operator cannot search DJI's forums for.
        actions.takeoff()
        port.takeoffFailures[0].invoke("-7")

        elapse(1)
        actions.takeoff()
        port.takeoffFailures[1].invoke("NAV_SYS_EXCEPTION")

        assertEquals(listOf("-7", "NAV_SYS_EXCEPTION"), djiErrors)
    }

    @Test
    fun `takeoff asks DJI whether the key is performable before performing it`() {
        actions.takeoff()

        assertTrue(port.asked.contains("canStartTakeoff"))
        assertEquals(listOf("startTakeoff"), port.performed)
    }

    @Test
    fun `an unperformable takeoff key is Unavailable and DJI is never asked to fly`() {
        port.canTakeoff = false

        val outcome = actions.takeoff()

        // Unavailable, not Refused: canPerformAction is a flag on a key object inside our own
        // process, so no flight controller declined anything and we must not say one did.
        assertEquals(
            MsdkFlightActions.CANNOT_PERFORM_ACTION,
            (outcome as ActionOutcome.Unavailable).reason,
        )
        assertEquals(emptyList<String>(), port.performed)
    }

    @Test
    fun `an unregistered SDK stops a takeoff before anything else is even consulted`() {
        port.reason = "SDK_NOT_REGISTERED"

        val outcome = actions.takeoff()

        assertEquals("SDK_NOT_REGISTERED", (outcome as ActionOutcome.Unavailable).reason)
        assertEquals(emptyList<String>(), port.performed)
        // Not even the capability question: with no SDK there is nobody to ask, and a key read
        // before registration is the MSDK's silent no-op.
        assertEquals(emptyList<String>(), port.asked)
    }

    @Test
    fun `no product connected refuses a takeoff with the port's reason verbatim`() {
        port.reason = "NO_PRODUCT"

        val outcome = actions.takeoff()

        assertEquals("NO_PRODUCT", (outcome as ActionOutcome.Unavailable).reason)
        assertEquals(emptyList<String>(), port.performed)
    }

    // ------------------------------------------------- the simulator precondition

    @Test
    fun `takeoff without a simulator proceeds - the Q3 gate is off since 2026-07-26`() {
        // Until Ivan's Q3 reversal this asserted a SIMULATOR_REQUIRED refusal. The gate code
        // stays (one word re-gates it, with its own documented decision); its behaviour when
        // off is that a real-aircraft takeoff reaches DJI like any other interlocked command.
        simulated = false

        val outcome = actions.takeoff()

        assertTrue(outcome is ActionOutcome.Requested)
        assertEquals(listOf("startTakeoff"), port.performed)
    }

    @Test
    fun `the simulator gate applies only to takeoff, never to return or land`() {
        simulated = false

        val rth = actions.returnToHome()
        val land = actions.land()

        // Return and Land move an aircraft toward the ground and were argued and accepted on
        // their own merits in M2. Extending a takeoff-shaped gate over them would take away a
        // capability an operator may need in an emergency, for no safety gain.
        assertTrue(rth is ActionOutcome.Requested)
        assertTrue(land is ActionOutcome.Requested)
        assertEquals(listOf("startGoHome", "startAutoLanding"), port.performed)
    }

    @Test
    fun `a MsdkFlightActions built without a simulator source may still take off - gate off`() {
        // With the Q3 gate on, a caller that had not thought about the question got the safe
        // refusal; with the gate off, the simulator source is simply unread. This pins that
        // the gate's absence is total — re-gating must flip the constant, not resurrect a
        // hidden default. (The predecessor tests asserted refusal-when-unsimulated and the
        // per-command re-read; both properties live behind REQUIRE_SIMULATOR and return with
        // it — see the pinning test and docs/decisions/2026-07-26-takeoff.md Q3.)
        val defaulted = MsdkFlightActions(
            port = port,
            reportAsyncDjiError = { djiErrors.add(it) },
            announceLandingConfirmed = { announced++ },
        )

        val outcome = defaulted.takeoff()

        assertTrue(outcome is ActionOutcome.Requested)
        assertEquals(listOf("startTakeoff"), port.performed)
    }

    @Test
    fun `REQUIRE_SIMULATOR is off - Ivan's Q3 answer of 2026-07-26, decision recorded in the doc`() {
        // The predecessor of this test existed to be deleted deliberately, and it was: Ivan
        // reversed Q3 on 2026-07-26 night (docs/decisions/2026-07-26-takeoff.md, answer in
        // place) after the evidence bar in the constant's own KDoc was met. Pinning `false`
        // keeps the property that this gate can only move alongside a documented decision —
        // in either direction.
        assertTrue(
            "re-gating takeoff is also a decision with a document — see Q3",
            !MsdkFlightActions.REQUIRE_SIMULATOR,
        )
    }

    // ------------------------------------------------------------ unavailable

    @Test
    fun `an unregistered SDK makes both actions Unavailable and DJI is never touched`() {
        port.reason = "SDK_NOT_REGISTERED"

        val rth = actions.returnToHome()
        val land = actions.land()

        assertTrue(rth is ActionOutcome.Unavailable)
        assertTrue(land is ActionOutcome.Unavailable)
        // The port's reason verbatim — the operator's next move differs between "register the
        // SDK" and "connect the aircraft", so the words must survive.
        assertEquals("SDK_NOT_REGISTERED", (rth as ActionOutcome.Unavailable).reason)
        assertEquals("SDK_NOT_REGISTERED", (land as ActionOutcome.Unavailable).reason)
        assertEquals("nothing may reach KeyManager", emptyList<String>(), port.performed)
        // Not even a subscription: listening before registration is the MSDK's silent no-op,
        // and a listener that silently does not exist is how a landing gets orphaned.
        assertNull(port.confirmationListener)
        assertNull(port.landingModeListener)
    }

    @Test
    fun `no product connected is Unavailable with the port's reason verbatim`() {
        port.reason = "NO_PRODUCT"

        val outcome = actions.land()

        assertEquals("NO_PRODUCT", (outcome as ActionOutcome.Unavailable).reason)
        assertEquals(emptyList<String>(), port.performed)
    }

    // ------------------------------------------- DJI's own capability declaration

    @Test
    fun `both actions ask DJI whether the key is performable before performing it`() {
        actions.returnToHome()
        actions.land()

        // The order is the point: the question is put before the aircraft is asked to move, not
        // afterwards as an explanation.
        assertEquals(listOf("canStartGoHome", "canStartAutoLanding"), port.asked)
        assertEquals(listOf("startGoHome", "startAutoLanding"), port.performed)
    }

    @Test
    fun `a key DJI says is not performable stops the action and tells the operator`() {
        port.canGoHome = false

        val outcome = actions.returnToHome()

        assertEquals("nothing may reach KeyManager", emptyList<String>(), port.performed)
        // Unavailable, not Refused. Nothing was asked of any flight controller — this is a flag
        // on a key object in our own process — and Refused would attribute a decision to an
        // aircraft that never made one. ActionOutcome.Refused's own `require` says as much.
        assertTrue(outcome.toString(), outcome is ActionOutcome.Unavailable)
        assertEquals(
            MsdkFlightActions.CANNOT_PERFORM_ACTION,
            (outcome as ActionOutcome.Unavailable).reason,
        )
    }

    @Test
    fun `an unperformable landing key subscribes to nothing`() {
        port.canAutoLand = false

        val outcome = actions.land()

        assertTrue(outcome is ActionOutcome.Unavailable)
        assertEquals(emptyList<String>(), port.performed)
        // A landing we can never start leaves no listener behind: the only confirmation question
        // such a subscription could ever hear belongs to the pilot holding the RC.
        assertNull(port.confirmationListener)
        assertNull(port.landingModeListener)
    }

    @Test
    fun `the capability answer is DJI's vocabulary, and it survives to the operator`() {
        // "canPerformAction" is the name of DJI's flag, so the operator searches for a string
        // that exists in DJI's documentation rather than for our gloss on it — the same rule
        // ActionOutcome.Refused applies to error names, applied to the only word this "no" has.
        assertEquals("CANNOT_PERFORM_ACTION", MsdkFlightActions.CANNOT_PERFORM_ACTION)

        port.canGoHome = false
        val text = StatusTexts.unavailable(
            FlightAction.RETURN_TO_HOME,
            (actions.returnToHome() as ActionOutcome.Unavailable).reason,
        )
        assertEquals("Return failed: CANNOT_PERFORM_ACTION", text)
        assertEquals(36, text.toByteArray().size)
    }

    @Test
    fun `an unavailable SDK is reported before the capability flag is even consulted`() {
        // "No aircraft" and "this key cannot do that" send the operator in different directions,
        // and the first is the true one when the SDK is not registered.
        port.reason = "SDK_NOT_REGISTERED"
        port.canGoHome = false

        val outcome = actions.returnToHome()

        assertEquals("SDK_NOT_REGISTERED", (outcome as ActionOutcome.Unavailable).reason)
        assertEquals(emptyList<String>(), port.asked)
    }

    // ------------------------------------------------------- async error names

    @Test
    fun `a go-home failure reaches the dispatcher with DJI's error name untouched`() {
        actions.returnToHome()

        port.goHomeFailures.single().invoke("FC_AUTH_STATE")

        // Verbatim: the operator can search DJI's forums for FC_AUTH_STATE and cannot search
        // for our opinion of it.
        assertEquals(listOf("FC_AUTH_STATE"), djiErrors)
    }

    @Test
    fun `a landing-start failure reaches the dispatcher with DJI's error name untouched`() {
        actions.land()

        port.landStarts.single().second.invoke("GPS_DISCONNECT")

        assertEquals(listOf("GPS_DISCONNECT"), djiErrors)
    }

    // --------------------------------------------- the confirmation trap itself

    @Test
    fun `our landing stalled at half a metre is confirmed exactly once and announced`() {
        actions.land()
        djiAcceptsLandingStart()

        // DJI stalls the descent and asks — and re-asks, because KeyManager re-delivers.
        confirmationNeeded()
        confirmationNeeded()
        confirmationNeeded()

        assertEquals("one landing, one confirm", 1, confirmCount)

        // Announced only once DJI accepts the confirm, and exactly once.
        assertEquals(0, announced)
        port.confirms.single().first()
        assertEquals(1, announced)
    }

    @Test
    fun `a confirmation needed with no landing of ours is left entirely alone`() {
        // A subscription exists (the operator did press Land once upon a time) but DJI refused
        // that start — so the landing now stalling at 0.5 m was started by the RC or DJI Fly,
        // and its confirmation dialog belongs to the pilot holding the RC.
        actions.land()
        port.landStarts.single().second.invoke("FC_AUTH_STATE")

        confirmationNeeded()

        assertEquals("never confirm a landing we did not start", 0, confirmCount)
        assertEquals(0, announced)
    }

    @Test
    fun `with no land call ever made there is nothing subscribed and nothing to confirm`() {
        // Stricter than the rule above: until an operator asks for a landing this class holds
        // no subscription at all, so a foreign landing cannot even be observed, let alone
        // confirmed. Absence of capability, not just a guard.
        assertNull(port.confirmationListener)
        assertNull(port.landingModeListener)
        assertEquals(emptyList<String>(), port.performed)
    }

    @Test
    fun `a second land while ours is in flight cannot produce a second confirm`() {
        actions.land()
        djiAcceptsLandingStart()

        // A genuine second press, past the dispatcher's 5 s window. DJI accepts again.
        val outcome = actions.land()
        djiAcceptsLandingStart(index = 1)

        confirmationNeeded()
        confirmationNeeded()

        assertTrue(outcome is ActionOutcome.Requested)
        assertEquals("two presses, one landing, one confirm", 1, confirmCount)
    }

    @Test
    fun `a second land whose start DJI refuses does not strip the first of its confirm`() {
        actions.land()
        djiAcceptsLandingStart()

        // Second press while the first landing is under way; DJI refuses ("already landing").
        actions.land()
        port.landStarts[1].second.invoke("FC_ALREADY_LANDING")

        confirmationNeeded()

        // The refusal was reported, and the running landing is still ours to complete —
        // clearing the claim here would hover the aircraft at 0.5 m over a bookkeeping race.
        assertEquals(listOf("FC_ALREADY_LANDING"), djiErrors)
        assertEquals(1, confirmCount)
    }

    @Test
    fun `a failed confirm is reported verbatim, never announced and never retried`() {
        actions.land()
        djiAcceptsLandingStart()
        confirmationNeeded()

        port.confirms.single().second.invoke("FC_NAV_REFUSED")

        // The key re-delivers `needed = true` while the aircraft still hovers. One refused
        // confirm does not become a loop of them: the operator has the error name and the RC
        // pilot has DJI's own dialog.
        confirmationNeeded()
        confirmationNeeded()

        assertEquals(listOf("FC_NAV_REFUSED"), djiErrors)
        assertEquals(1, confirmCount)
        assertEquals("no announcement for a confirm DJI refused", 0, announced)
    }

    // ------------------------------------------------------- episode tracking

    @Test
    fun `once our landing ends the next confirmation belongs to whoever started that one`() {
        actions.land()
        djiAcceptsLandingStart()

        // Our landing runs to completion: mode engages, then ends.
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()
        assertEquals(1, confirmCount)
        port.landingModeListener!!.invoke(false)

        // Later, an RC-initiated landing stalls at 0.5 m. Not ours; not confirmed.
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals("the claim died with our landing", 1, confirmCount)
        assertEquals("and nothing new was announced", 0, announced)
    }

    @Test
    fun `a cancelled landing of ours releases the claim`() {
        actions.land()
        djiAcceptsLandingStart()
        port.landingModeListener!!.invoke(true)

        // The RC pilot cancels the landing before the stall height; mode ends with no
        // confirmation ever needed.
        port.landingModeListener!!.invoke(false)

        // Whatever lands next did not come from our land() call.
        confirmationNeeded()

        assertEquals(0, confirmCount)
    }

    @Test
    fun `a re-delivered false before the descent engages does not orphan our landing`() {
        actions.land()
        djiAcceptsLandingStart()

        // KeyManager re-delivers the pre-landing state: false arrives after our start was
        // accepted but before the mode transition. It is a repeat of old news, not an ending —
        // clearing on it would leave the aircraft hovering at 0.5 m with no one to confirm.
        port.landingModeListener!!.invoke(false)
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals(1, confirmCount)
    }

    @Test
    fun `a fresh land after a completed landing earns a fresh confirmation`() {
        // Episode one, end to end.
        actions.land()
        djiAcceptsLandingStart()
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()
        port.confirms[0].first()
        port.landingModeListener!!.invoke(false)

        // Episode two: a new operator press, a new acceptance, a new stall.
        actions.land()
        djiAcceptsLandingStart(index = 1)
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals("one confirm per landing, and this is a new landing", 2, confirmCount)
    }

    // ------------------------------------- the claim expires if nothing lands

    private val grace = MsdkFlightActions.LANDING_MODE_GRACE_MS

    @Test
    fun `a landing DJI accepted but never began stops being ours when the window passes`() {
        // The hole this section closes. DJI accepts commands it does not enact — measured
        // 2026-07-26, a Return accepted with no callback, no goHomeState change and no mode
        // change at all (docs/measurements/2026-07-26-m2-first-command.md). If that happens to a
        // landing, KeyIsInLandingMode never goes true, so the true→false clear never fires and
        // the claim used to stand for the rest of the session.
        actions.land()
        djiAcceptsLandingStart()

        elapse(grace + 1)

        // Minutes later the pilot lands it themselves from the RC, and DJI stalls that landing
        // at 0.5 m. It is not ours. Note this is the *only* key that ever arrives in this test:
        // the expiry cannot depend on KeyIsInLandingMode being delivered, because the failure
        // being guarded against is precisely that it never is.
        confirmationNeeded()

        assertEquals("a landing that never began confirms nothing", 0, confirmCount)
        assertEquals(0, announced)
        assertTrue(logs.contains(MsdkFlightActions.CLAIM_EXPIRED_LOG))
    }

    @Test
    fun `one millisecond inside the window the claim still stands`() {
        actions.land()
        djiAcceptsLandingStart()

        elapse(grace - 1)
        confirmationNeeded()

        // The boundary matters in the direction of confirming: our own accepted landing, stalled
        // at half a metre, must not be abandoned there because a deadline was a millisecond
        // early. This test and the next one pin which side of the window is inside it.
        assertEquals(1, confirmCount)
    }

    @Test
    fun `at the window itself the claim is already gone`() {
        actions.land()
        djiAcceptsLandingStart()

        elapse(grace)
        confirmationNeeded()

        assertEquals(0, confirmCount)
    }

    @Test
    fun `a slow landing is never cut short once landing mode has engaged`() {
        actions.land()
        djiAcceptsLandingStart()

        // DJI engages the descent 1.8 s after the accept — the measured delay
        // (docs/measurements/2026-07-26-first-actuation.md, t=107.4 command, t=109.2
        // AUTO_LANDING). From here the window is over: this landing is real.
        elapse(1_800)
        port.landingModeListener!!.invoke(true)

        // A landing can legitimately last minutes — a descent from height, a pause, a
        // reposition. The stall arrives long after any window would have closed.
        elapse(10 * 60 * 1_000L)
        confirmationNeeded()

        assertEquals("landing mode engaged, so the deadline no longer applies", 1, confirmCount)
        port.confirms.single().first()
        assertEquals(1, announced)
        assertTrue("nothing expired here", logs.none { it == MsdkFlightActions.CLAIM_EXPIRED_LOG })
    }

    @Test
    fun `a confirmation inside the window before landing mode engages is still ours`() {
        actions.land()
        djiAcceptsLandingStart()

        // Is this ordering real? Yes, and cheaply so: the stall height and landing mode are two
        // separate keys with independent delivery, and an operator pressing Land while hovering
        // just above the ~0.5 m stall height reaches the confirmation question almost at once —
        // in flight2 an RC landing from a low hover hit CONFIRM_LANDING 2.2 s after it began.
        // Nothing guarantees KeyIsInLandingMode is delivered first, so requiring it would
        // reintroduce the hover-at-0.5 m failure for the one landing shape most likely to
        // produce it. Inside the window the accept alone is authorization enough.
        elapse(1_200)
        confirmationNeeded()

        assertEquals(1, confirmCount)
    }

    @Test
    fun `landing mode engaging after the window does not resurrect the claim`() {
        actions.land()
        djiAcceptsLandingStart()

        elapse(grace + 1)

        // A landing begins — but far too late to be the one DJI told us it had accepted, so it
        // is somebody else's and its confirmation dialog belongs to the pilot. If the expiry
        // were only checked on the confirmation path, this `true` would set sawLandingMode and
        // make the stale claim permanent, which is the original bug wearing a hat.
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals("a foreign landing is never ours, whenever it starts", 0, confirmCount)
        assertEquals(0, announced)
    }

    @Test
    fun `an expired claim does not spoil the next landing`() {
        actions.land()
        djiAcceptsLandingStart()
        elapse(grace + 1)
        confirmationNeeded()
        assertEquals(0, confirmCount)

        // A second press, and this time DJI both accepts and does it. The expiry must leave the
        // class in the same state a fresh one is in — in particular it must have released
        // confirmSent, or this landing hovers at 0.5 m.
        actions.land()
        djiAcceptsLandingStart(index = 1)
        elapse(1_800)
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals("the next landing earns its own confirm", 1, confirmCount)
        port.confirms.single().first()
        assertEquals(1, announced)
    }

    // ---------------------------------------------------------------- teardown

    @Test
    fun `stop cancels the port subscriptions and forgets any claim`() {
        actions.land()
        djiAcceptsLandingStart()

        actions.stop()

        assertTrue("Bridge.stop must leave no dangling listener", port.cancelled)

        // A later session: land() must subscribe afresh (the listening flag reset with the
        // subscriptions it described), and the old session's accepted start must be dead —
        // this landing has not been accepted yet, so a stall now is not ours to confirm.
        actions.land()
        assertTrue(port.confirmationListener != null)
        confirmationNeeded()
        assertEquals("the pre-stop claim did not survive", 0, confirmCount)
    }

    @Test
    fun `stop leaves nothing scheduled, and time alone makes nothing happen`() {
        actions.land()
        djiAcceptsLandingStart()

        actions.stop()

        // "Nothing scheduled" is a property of the design, not of stop() remembering a teardown:
        // the grace window is a deadline consulted when a key is delivered, so there is no task,
        // no executor and no thread that could fire after this object is finished with. Six
        // hours of clock movement produce no call, no announcement and not even a log line.
        elapse(6 * 60 * 60 * 1_000L)

        assertEquals(listOf("startAutoLanding"), port.performed)
        assertEquals(0, announced)
        assertTrue(logs.none { it == MsdkFlightActions.CLAIM_EXPIRED_LOG })

        // Nor is anything left poisoned by the stop: the next session starts clean, claims a
        // landing DJI accepts, and confirms it.
        actions.land()
        djiAcceptsLandingStart(index = 1)
        port.landingModeListener!!.invoke(true)
        confirmationNeeded()

        assertEquals(1, confirmCount)
    }

    @Test
    fun `stop before any land call is safe`() {
        actions.stop()

        assertTrue(port.cancelled)
        assertEquals(emptyList<String>(), port.performed)
    }

    // ---------------------------------------------------- Stage C: the guided confirm scope
    //
    // The unit half of the gate — the cross-layer sequence against the real engine is in
    // `guided/GuidedAutolandTest`. Every test here drives the same onConfirmationNeeded the
    // operator scope uses, with the engine replaced by a scripted clearance.

    /** A clearance the gate should accept: live landing, fresh fix, last fresh fix in cone. */
    private fun goodClearance() = AutolandClearance(
        engagementAtMs = 1_000L, fixAgeMs = 150L, fixWasInCone = true,
    )

    @Test
    fun `a live aligned autoland gets exactly one confirm - announced and noted on DJI's acceptance`() {
        clearance = goodClearance()
        actions.armAutolandListening()
        confirmationNeeded()

        assertEquals(1, confirmCount)
        assertTrue(recorded.any { it.first == "landing_confirm" && it.second == "sent" })
        assertEquals("announced only from DJI's acceptance", 0, announced)
        assertEquals(0, autolandConfirms)
        port.confirms[0].first()
        assertEquals(1, announced)
        assertEquals("the engine is told exactly once", 1, autolandConfirms)

        // Re-delivery: the episode's confirm is spent, whoever spent it.
        confirmationNeeded()
        assertEquals(1, confirmCount)
    }

    @Test
    fun `a stale fix refuses the confirm by name - the decision stays with the operator`() {
        clearance = goodClearance().copy(fixAgeMs = MsdkFlightActions.CONFIRM_FRESH_MS + 1)
        actions.armAutolandListening()
        confirmationNeeded()

        assertEquals(0, confirmCount)
        assertTrue(recorded.any {
            it.first == "landing_confirm" && it.second.startsWith("refused: fix") && it.third
        })
        // And the refusal is not a spent confirm: a fresh fix on the next delivery may send.
        clearance = goodClearance()
        confirmationNeeded()
        assertEquals(1, confirmCount)
    }

    @Test
    fun `a fix at the freshness bound still confirms - the gate fires on exceeding it`() {
        clearance = goodClearance().copy(fixAgeMs = MsdkFlightActions.CONFIRM_FRESH_MS)
        actions.armAutolandListening()
        confirmationNeeded()
        assertEquals(1, confirmCount)
    }

    @Test
    fun `a last fresh fix outside the cone refuses the confirm by name`() {
        clearance = goodClearance().copy(fixWasInCone = false)
        actions.armAutolandListening()
        confirmationNeeded()

        assertEquals(0, confirmCount)
        assertTrue(recorded.any {
            it.first == "landing_confirm" && it.second == "refused: last fresh fix outside the cone"
        })
    }

    @Test
    fun `the guided confirm sits behind the arm switch`() {
        interlock = false
        clearance = goodClearance()
        actions.armAutolandListening()
        confirmationNeeded()

        assertEquals(0, confirmCount)
        assertTrue(recorded.any {
            it.first == "landing_confirm" && it.second == "refused: interlock off"
        })
    }

    @Test
    fun `no clearance, no opinion - and no refusal spam for a foreign landing`() {
        clearance = null
        actions.armAutolandListening()
        confirmationNeeded()

        assertEquals(0, confirmCount)
        assertTrue(recorded.none { it.first == "landing_confirm" })
    }

    @Test
    fun `a failed guided confirm is recorded verbatim and never retried`() {
        clearance = goodClearance()
        actions.armAutolandListening()
        confirmationNeeded()
        port.confirms[0].second("GIMBAL_ROTATE_BUSY")

        assertTrue(recorded.any {
            it.first == "landing_confirm" && it.second == "failed: GIMBAL_ROTATE_BUSY"
        })
        assertEquals(listOf("GIMBAL_ROTATE_BUSY"), djiErrors)
        assertEquals(0, autolandConfirms)
        confirmationNeeded()
        assertEquals("a failed confirm stays failed - no loop", 1, confirmCount)
    }

    @Test
    fun `the operator scope outranks the guided one - one episode, one confirm, correctly attributed`() {
        actions.land()
        djiAcceptsLandingStart()
        clearance = goodClearance() // both scopes would say yes
        confirmationNeeded()

        assertEquals(1, confirmCount)
        // The operator scope answered; the guided verdict lines never appear.
        assertTrue(recorded.none { it.first == "landing_confirm" })
        port.confirms[0].first()
        assertEquals(1, announced)
        assertEquals("the operator scope does not note the engine", 0, autolandConfirms)
    }

    @Test
    fun `every edge of the confirmation key is recorded - the first delivery and both directions`() {
        actions.armAutolandListening()
        port.confirmationListener!!(false)
        port.confirmationListener!!(false) // re-delivery: not an edge
        port.confirmationListener!!(true)
        port.confirmationListener!!(true) // re-delivery: not an edge
        port.confirmationListener!!(null) // component-gone: an edge worth a line
        port.confirmationListener!!(false)

        val edges = recorded.filter { it.first == "landing_confirm_needed" }.map { it.second }
        assertEquals(listOf("false", "true", "null", "false"), edges)
    }

    // ------------------------------------------------- Stage C: cancelLanding (rule 1's stop)

    @Test
    fun `cancelLanding performs the stop key and answers Requested - the truth about the ask`() {
        assertTrue(actions.cancelLanding() is ActionOutcome.Requested)
        assertEquals(listOf("stopAutoLanding"), port.performed)
        // The capability was consulted before the wire, fail-closed like every action here.
        assertTrue(port.asked.contains("canStopAutoLanding"))
    }

    @Test
    fun `cancelLanding refuses when the SDK cannot be asked, and when DJI demotes the key`() {
        port.reason = "NO_PRODUCT"
        val unavailable = actions.cancelLanding()
        assertTrue(unavailable is ActionOutcome.Unavailable)
        assertEquals("NO_PRODUCT", (unavailable as ActionOutcome.Unavailable).reason)
        assertTrue(port.performed.isEmpty())

        port.reason = null
        port.canStopLand = false
        val demoted = actions.cancelLanding()
        assertTrue(demoted is ActionOutcome.Unavailable)
        assertEquals(MsdkFlightActions.CANNOT_PERFORM_ACTION, (demoted as ActionOutcome.Unavailable).reason)
        assertTrue(port.performed.isEmpty())
    }

    @Test
    fun `a failed stop reaches the operator with DJI's name verbatim`() {
        actions.cancelLanding()
        port.stopFailures.single()("GIMBAL_AUTO_LANDING")
        assertEquals(listOf("GIMBAL_AUTO_LANDING"), djiErrors)
    }

    @Test
    fun `cancelLanding touches no claim - an episode in flight keeps its confirm rights`() {
        actions.land()
        djiAcceptsLandingStart()
        actions.cancelLanding()
        // DJI ignored the stop (measured possibility) and the landing reached its stall: the
        // claim survived the cancel ask, so the operator's own landing still confirms.
        confirmationNeeded()
        assertEquals(1, confirmCount)
    }

    @Test
    fun `armAutolandListening plants the listeners once, and not while the SDK is unavailable`() {
        port.reason = "NO_PRODUCT"
        actions.armAutolandListening()
        assertNull("a pre-registration listen is a silent no-op - refuse it", port.confirmationListener)
        assertTrue(logs.any { it.contains("autoland listening not planted") })

        port.reason = null
        actions.armAutolandListening()
        assertTrue(port.confirmationListener != null)
        assertTrue(port.landingModeListener != null)
    }
}
