package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.record.EventCode

/**
 * The DJI half of M2: [FlightActions] made real over an [ActionPort], carrying every decision
 * that layer is allowed to make — and the one automatic command this project permits itself.
 *
 * No DJI imports, on purpose: the port is the seam and `KeyManagerActionPort` is the only class
 * below it. Everything here — the outcome mapping, the availability refusals, and above all the
 * landing-confirmation policy — runs under `MsdkFlightActionsTest` with a fake port, because
 * these are the decisions that decide whether an aircraft half a metre off the ground descends
 * or hovers.
 *
 * ## The landing-confirmation policy: scoped auto-confirm
 *
 * `docs/decisions/2026-07-26-landing-confirmation.md`. DJI stalls an auto-landing at ~0.5 m and
 * raises `KeyIsLandingConfirmationNeeded`; nothing descends further until `KeyConfirmLanding` is
 * performed. QGC has no confirm UI to forward that question to — its Land button *was* the
 * operator's whole decision — so a landing this class started is confirmed automatically, once,
 * and announced. A landing anyone else started (RC, DJI Fly) is **never** confirmed from here:
 * that confirmation dialog belongs to DJI's own UX in the pilot's hands, and answering it on
 * their behalf would be this bridge sending a command nobody asked it for.
 *
 * "Ours" is a claim with teeth, not a mood:
 *
 *  - It is set only in [ActionPort.startAutoLanding]'s `onSuccess` — i.e. only once DJI has
 *    **accepted our start**. Setting it at call time would mark a landing as ours while DJI was
 *    still free to refuse the start, and a refusal racing an RC-initiated landing would then
 *    auto-confirm someone else's descent. A rejected ask leaves no standing authorization.
 *  - It is cleared when the landing episode ends: `KeyIsInLandingMode` observed going
 *    true → false, whether that is touchdown or a cancel from the RC. A cancelled landing of
 *    ours must not leave a stale claim that confirms the *next* landing, whoever starts it.
 *  - The true → false **transition** is required, not a bare false: `KeyManager` re-delivers
 *    unchanged values (`StateCache`'s whole staleness model rests on that), so a `false` arriving
 *    in the gap between our start being accepted and the descent engaging is a repeat of the old
 *    state, not news. Clearing on it would orphan the landing at 0.5 m — the exact failure this
 *    class exists to prevent.
 *  - **It expires if the landing never begins.** An accepted start whose `KeyIsInLandingMode`
 *    never goes true would otherwise hold the claim for the rest of the session — the true→false
 *    clear needs a true first — and the next landing, RC-initiated, would be auto-confirmed.
 *    Measured on 2026-07-26: DJI accepts commands it does not enact (`docs/measurements/
 *    2026-07-26-m2-first-command.md`), so that is a real state, not a hypothetical one. The claim
 *    therefore stands for [LANDING_MODE_GRACE_MS] after acceptance *unless* landing mode engages;
 *    once it has, the episode-end rule above governs and a slow landing is never cut short.
 *
 * The expiry is evaluated where it is read — [expireStaleClaim] runs from both key callbacks — so
 * there is no timer, nothing to cancel in [stop] and nothing that can fire after the object is
 * done with. See [LANDING_MODE_GRACE_MS] for the window and the measurements behind it.
 *
 * The confirm itself is sent **at most once per landing episode**. A confirm that fails is
 * reported through [reportAsyncDjiError] and deliberately not retried on the next re-delivery of
 * `needed = true`: a command loop aimed at an aircraft is not a recovery strategy, and the
 * operator holding the RC still has DJI's own confirmation UX. The announcement to QGC goes out
 * only from the confirm's `onSuccess`, so it reports what DJI accepted rather than what we hoped.
 *
 * ## The second scoped authority: a Stage C full autoland (2026-07-28, pivoted the same day)
 *
 * The guided engine's tag autoland commits to **DJI's own landing** — landing04 measured the FC
 * flooring a virtual-stick descent at ~1.4 m and honouring the stick-down confirmation gesture
 * only from the physical RC, so the engine's commit IS a [land] call, made through `Bridge`'s
 * `DjiLanding` seam. The primary confirm authority for that landing is therefore the operator
 * scope above: DJI accepting the commit's start makes the landing "ours to confirm" by exactly
 * the machinery QGC's Land button uses, and nothing about that scope changed. What remains for
 * the second scope is the case where the claim never stood or has expired (the measured
 * silent-accept failure, a listener race) while the engine still vouches for a live committed
 * landing — [onConfirmationNeeded] then consults it, with its own teeth:
 *
 *  - **The engine must vouch, at the moment of the question.** [autolandClearance] is read
 *    fresh; null — no live committed landing — leaves the delivery entirely alone, exactly as
 *    before this scope existed. Rule 1 makes null the common case: any manual stick kills the
 *    engagement (and now also sends `KeyStopAutoLanding` — [cancelLanding]), and with it the
 *    clearance, before this callback could act on it.
 *  - **The fix must be recent ([CONFIRM_FRESH_MS]) and its last fresh reading inside the
 *    cone** — a confirm is the claim "descend the last half-metre here", and only the tag
 *    sensor can make it. A blind landing (the *expected* ending — DJI recenters the camera
 *    before touchdown) therefore refuses the confirm, with the reason recorded and the
 *    decision left to the operator.
 *  - **The same arm switch as everything else**: [interlockEnabled] is consulted at the moment
 *    of the confirm, not remembered from the arm.
 *  - **The same single-confirm-per-episode discipline**: both scopes share [confirmSent], so
 *    the operator path's invariants are unchanged and the two scopes cannot double-fire one
 *    episode between them.
 *
 * Subscriptions for this scope are planted by [armAutolandListening] — called by `Bridge` when a
 * full autoland arms, before any commit exists — which amends the lazy-subscription rule below
 * to: until an operator asks for a landing **or arms a full autoland**, this class holds no
 * subscriptions and can confirm nothing. Every delivery edge of `KeyIsLandingConfirmationNeeded`
 * is recorded (`landing_confirm_needed`): landing04 proved the key silent during a virtual-stick
 * descent, and whether it fires during an SDK-started landing is the next flight's question —
 * an absence must be a measured absence.
 *
 * ## The takeoff preconditions
 *
 * `docs/decisions/2026-07-26-takeoff.md`. [takeoff] is the only method here that can put an
 * aircraft into the air, and it is the only one with a precondition that is not about the SDK:
 * [REQUIRE_SIMULATOR]. Read that constant before changing it — it is one boolean, deliberately,
 * and it is pinned by a test that names the decision doc, so removing the gate is a visible act
 * rather than a quiet one.
 *
 * The three refusals are ordered cheapest-and-broadest first, and all three are
 * [ActionOutcome.Unavailable] rather than [ActionOutcome.Refused]: no flight controller was
 * consulted for any of them, and attributing a decision to an aircraft that never made one is
 * the thing `Refused`'s KDoc forbids.
 *
 * ## What this class does not do
 *
 * No retries, no watchdogs, no command DJI did not ask for and the operator did not press —
 * `KeyConfirmLanding` is the *sole* automatic call, and it completes an act the operator
 * themselves took: their own Land press (the first scope), or the full-autoland arm they made
 * on the phone's explicit toggle (the second — the engagement it completes exists only while
 * that armed descent is live and the interlock is on). Everything else here is a translation of
 * one button press into one `performAction`.
 *
 * ## Threads
 *
 * [returnToHome] and [land] run on `mavlink-rx`; port callbacks arrive on the MSDK's callback
 * thread. All flag transitions are under [gate], which is cheap at this call rate and removes
 * every ordering question the two threads could pose. The port's listen calls are made lazily on
 * the first [land] — after [ActionPort.unavailableReason] has passed, so `KeyManager` is
 * registered and the subscription cannot be the silent pre-registration no-op
 * (`docs/architecture.md`). Until an operator asks for a landing, this class holds no
 * subscriptions and can confirm nothing.
 */
class MsdkFlightActions(
    private val port: ActionPort,
    /** [CommandDispatcher.reportAsyncDjiError] in the app: DJI's error name, verbatim, to QGC. */
    private val reportAsyncDjiError: (String) -> Unit,
    /** [CommandDispatcher.reportLandingConfirmed] in the app: the auto-confirm announcement. */
    private val announceLandingConfirmed: () -> Unit,
    /**
     * Whether **DJI reports a simulator running on this aircraft** — the precondition
     * [REQUIRE_SIMULATOR] tests, and nothing else in this class reads it.
     *
     * A function rather than a value because the answer changes underneath us: a simulator can be
     * started or stopped from the app, from DJI Assistant, or by another process entirely, and
     * the only honest reading is the one taken at the moment of the command.
     *
     * **Defaults to `false`, i.e. closed.** A `MsdkFlightActions` constructed without this
     * argument refuses every takeoff, which is the correct behaviour for a caller that has not
     * thought about the question. `Bridge` supplies `Bridge.simulatorReportsRunning`, which maps
     * `SimulatorPhase` with an exhaustive `when` so a new phase cannot silently default to
     * "simulated" — and maps exactly the phases `SimulatorControl.noticeIfDue` warns QGC about,
     * so **takeoff is permitted precisely when the ground station is being told the telemetry is
     * not real flight.** That equivalence is the point: the gate and the warning cannot disagree.
     */
    private val simulatedFlight: () -> Boolean = { false },
    /**
     * Whether the single [CommandInterlock] is armed **right now** — the gate the Stage C
     * auto-confirm sits behind, read at the moment of the confirm rather than remembered from
     * the arm. Defaults to `false`, i.e. closed: a `MsdkFlightActions` constructed without it
     * can never confirm a guided landing, which is correct for a caller that has not thought
     * about the question. The operator-Land scope deliberately does *not* consult this — its
     * gate is `CommandDispatcher`'s, taken before [land] was ever called, and a landing the
     * interlock already admitted must not be orphaned at 0.5 m by the switch flicking off
     * mid-descent (the descent is DJI's by then; only the RC can stop it).
     */
    private val interlockEnabled: () -> Boolean = { false },
    /**
     * The guided engine's live full-autoland facts, or null when nothing this bridge flies
     * wants a confirmation — see [AutolandClearance]. Read fresh on every `needed = true`,
     * outside [gate] (the engine takes its own lock; this class must never hold [gate] across
     * that boundary).
     */
    private val autolandClearance: () -> AutolandClearance? = { null },
    /**
     * Told once per accepted guided confirm, from the confirm's `onSuccess` — the engine notes
     * that DJI has confirmed the landing, so a later rule-1 cancel can record that DJI may
     * finish the descent on its own (one of the facts the Stage C flight measures).
     */
    private val onAutolandConfirmed: () -> Unit = {},
    /**
     * The flight record — `Recorder.event` in the app, a no-op in callers without one. Carries
     * the Stage C measurement lines (`landing_confirm_needed` edges, `landing_confirm`
     * verdicts) that `log` alone would keep off the flight timeline. `(code, message, warn)`.
     */
    private val recordEvent: (String, String, Boolean) -> Unit = { _, _, _ -> },
    private val log: (String) -> Unit = {},
    /**
     * Monotonic clock, injected so [LANDING_MODE_GRACE_MS] is testable without sleeping — the
     * same reason `HandshakeResponder` and `CommandDispatcher` take theirs, and used for nothing
     * else here.
     *
     * **Monotonic, not wall clock**: a claim on a descending aircraft must not be lengthened or
     * cut short by an NTP step, which is `StateCache`'s rule for the same reason.
     * `System.nanoTime()` rather than `SystemClock.elapsedRealtime()` keeps this file free of
     * Android imports and therefore runnable in a plain JVM test, which is the whole reason the
     * landing policy lives here and not in the port. The one difference that matters —
     * `nanoTime` does not advance in deep sleep — cannot apply to a window measured in seconds
     * on a foregrounded bridge holding a live UDP link.
     */
    private val nowMs: () -> Long = { System.nanoTime() / 1_000_000L },
) : FlightActions {

    companion object {
        /**
         * What the operator reads when DJI's own key declaration says the action is not
         * performable — `Return failed: CANNOT_PERFORM_ACTION`, 36 bytes.
         *
         * The wording is DJI's: `canPerformAction` is the name of the flag, so the operator is
         * searching for a string that exists in DJI's documentation rather than for our gloss on
         * it. That is the same rule [ActionOutcome.Refused] applies to error names, applied to the
         * only vocabulary this particular "no" has — the flag is a bare boolean and carries no
         * `IDJIError`, no code and no hint.
         *
         * And that is exactly why this is an [ActionOutcome.Unavailable] rather than a
         * [ActionOutcome.Refused], despite being a "no". `Refused` means *the flight controller
         * was consulted and declined*; its KDoc requires DJI's own error name and its `require`
         * says in as many words to "use Unavailable if there is no name". A false capability flag
         * is a statement about the key sitting in our own process — nothing was asked of any
         * aircraft — and `Unavailable`'s KDoc already names this case: *"the airframe has no such
         * capability"*. Reporting it as a refusal would attribute a decision to a flight
         * controller that never made one.
         */
        const val CANNOT_PERFORM_ACTION = "CANNOT_PERFORM_ACTION"

        /**
         * **Whether a takeoff may only be commanded against a simulated aircraft.** `false`
         * since 2026-07-26 night: Ivan reversed Q3 during the first real-flight session
         * (`docs/decisions/2026-07-26-takeoff.md` — answer recorded in place), with the KDoc's
         * own evidence bar met: five end-to-end simulated takeoffs observed through the DJI
         * callback, the mode trail and the recorder, plus an evening of Stage A/B sessions on
         * the same path. Takeoff is now gated by exactly what gates Return and Land: the
         * single [CommandInterlock] plus QGC's slide-to-confirm.
         *
         * The original argument for `true`, kept because its reasoning still governs any
         * future gate of this kind:
         *
         * The recommendation, in full, because a gate is only as good as the argument for it:
         *
         *  - **The feature is entirely unproven.** No takeoff has ever been commanded through this
         *    bridge. Everything below this line is desk work checked against a jar and a test
         *    suite; not one byte of it has reached a flight controller. Something unproven whose
         *    failure mode is an aircraft leaving the ground should first run somewhere it cannot.
         *  - **It costs nothing today.** The aircraft is indoors and the only place a takeoff can
         *    be exercised right now *is* the simulator — the thing that unblocked this feature at
         *    all (`docs/simulator.md`). The gate forbids nothing that was going to happen this
         *    week.
         *  - **It is a precondition, not a second switch.** This matters, because Ivan answered
         *    the "how many switches" question in `docs/decisions/2026-07-26-m3-guided-control.md`
         *    Q1 — *"I think it's ok to have a single toggle"* — and that answer is respected: the
         *    operator still arms one thing, [CommandInterlock]. This is observed state, in the
         *    same family as `SDK_NOT_REGISTERED` and `NO_PRODUCT`, and it adds nothing to
         *    remember.
         *  - **Against it, and it is a real cost:** a gate that must later be removed is a gate
         *    that will be removed carelessly, and takeoff-only-in-simulation is not the feature
         *    Ivan asked for (*"takeoff I like to have"*). The mitigation is that removal is not a
         *    deletion but an edit to a named constant with a test asserting its value, so it
         *    cannot rot away — and the doc names the evidence that should precede it: a takeoff
         *    commanded end to end against the simulator, with the DJI callback, the flight mode
         *    and the recorder all observed.
         *
         * The **honest limit** of what it buys, stated so nobody over-trusts it: a simulated
         * takeoff proves the chain, not the flight. `docs/simulator.md` §7 is explicit that a
         * simulator settles nothing about units, failsafes or this airframe's physical behaviour.
         * The only accepted evidence that a simulator is running is `KeyIsSimulatorStarted`
         * observed through `SimulatorControl` — never `ISimulatorManager.isSimulatorEnabled()`,
         * whose value we have no jar-based reading of at all (`docs/simulator.md` §1.2 reports it
         * as stubbed to `return false`; that reading is a **linking artefact**, see
         * [ActionPort.READING_THE_JAR], and the method's real body is the dead code behind the
         * stub — so §1.2's conclusion is unsupported and its recommendation is right anyway,
         * because an observed key beats an unread method either way).
         */
        const val REQUIRE_SIMULATOR = false

        /**
         * What the operator reads when [REQUIRE_SIMULATOR] refuses a takeoff —
         * `Takeoff failed: SIMULATOR_REQUIRED`, 34 bytes.
         *
         * Named for the condition rather than for our policy ("BRIDGE_POLICY", "NOT_YET") on the
         * same rule that keeps DJI's error names verbatim: the operator's next act should be
         * suggested by the word. This one tells them exactly what would make the command work.
         */
        const val SIMULATOR_REQUIRED = "SIMULATOR_REQUIRED"

        /**
         * How long an accepted landing start stays ours while `KeyIsInLandingMode` is still
         * false. Past it, with no landing mode ever observed, the claim is dropped.
         *
         * **The number comes from the only measurement of this delay we have.** On 2026-07-26
         * (`docs/measurements/2026-07-26-first-actuation.md`, `tmp/session-logs/flight2.jsonl`)
         * the aircraft entered `AUTO_LANDING` **1.8 s** after the `SET_MODE` arrived — t=107.4
         * inbound, t=109.208 mode change — and that 1.8 s contains the whole chain: QGC to
         * bridge, bridge to `KeyManager`, flight controller, and the key delivery back. 5 s is
         * ~2.8× that, so a legitimate accept→landing-mode delay has room to be three times
         * slower than the one time we watched it and still be honoured. It is also
         * `CommandDispatcher.ACTION_REPEAT_MS`, i.e. exactly the span this project already calls
         * "one press": QGC's 3× retry burst (1.34 s apart) collapses into one `land()` inside
         * it, so the retries can neither renew the claim nor open a second one.
         *
         * The upper bound is measured too, and it is why this is seconds rather than tens of
         * them: in the same log an RC-initiated landing from a low hover reached the ~0.5 m
         * stall **2.2 s** after it began (t=124.0 stick override, t=126.208 `CONFIRM_LANDING`).
         * A foreign landing can therefore arrive at the confirmation question quickly, and every
         * second the claim outlives its landing is a second in which one could be caught. This
         * does not remove that overlap — a window shorter than the legitimate delay would hover
         * our own landings at 0.5 m, which is worse — it bounds it, from "the rest of the
         * session" to the few seconds after a Land press the operator themselves just made.
         */
        const val LANDING_MODE_GRACE_MS = 5_000L

        /**
         * The one trace the expiry leaves. Not a `STATUSTEXT`: nothing happened to the aircraft
         * and nothing the operator can act on changed — what changed is that this bridge went
         * back to being unable to confirm anything, which is the state it is in for all but a
         * few seconds of any flight. It is here as a named constant so the tests can assert the
         * expiry announced itself at all, rather than matching on prose.
         */
        const val CLAIM_EXPIRED_LOG =
            "landing mode never engaged within ${LANDING_MODE_GRACE_MS}ms — claim dropped"

        /**
         * How recent the newest believed tag fix must be for the Stage C auto-confirm,
         * milliseconds. **Two seconds — `TagLatch.DEFAULT_WINDOW_NANOS`' and
         * `TagDescentGuidance.T_CLIMB_MS`' twin, on their shared measurement**: at the 10 Hz
         * detection cap with ≥92 % per-frame in-band, two seconds of nothing is a tag that has
         * genuinely left the frame, and the descent law itself stops believing the fix at
         * exactly this bound. A confirmation is the claim "the aircraft is over the pad"; a
         * fix the law would no longer steer on cannot carry it, and the refusal costs nothing
         * the landing needs — the sustained stick-down (the primary mechanism) keeps flowing.
         */
        const val CONFIRM_FRESH_MS = 2_000L
    }

    private val gate = Any()

    /** DJI accepted a landing start *we* asked for, and its episode has not ended. */
    private var ourLanding = false

    /**
     * [nowMs] when the start behind [ourLanding] was accepted. Read only while [sawLandingMode]
     * is false — once the landing is real the grace window stops applying.
     */
    private var claimAcceptedAtMs = 0L

    /** The one confirm this episode is allowed has been sent (whether or not it succeeded). */
    private var confirmSent = false

    /** `KeyIsInLandingMode` has reported true this episode — arms the true→false clear. */
    private var sawLandingMode = false

    private var listening = false

    /**
     * The last delivered value of `KeyIsLandingConfirmationNeeded`, for edge recording only —
     * never a gate. Boxed-with-a-sentinel via [neededEverDelivered] so the very first delivery
     * (whatever its value, null included) is recorded as an edge too: the Stage C flight must
     * be able to distinguish "the key reported false all descent" from "the key never spoke".
     */
    private var lastNeeded: Boolean? = null
    private var neededEverDelivered = false

    /**
     * `performAction(FC.KeyStartTakeoff)`, behind three refusals that all fail closed.
     *
     * Nothing here is stateful and nothing here subscribes: unlike [land], a takeoff acquires no
     * standing authorisation to send anything later, so there is no claim to set, nothing to
     * expire, and nothing for [stop] to unwind. That is deliberate and worth preserving — the
     * riskiest action in the project is also the one that leaves the least behind it.
     *
     * The ordering of the three checks is the ordering of what they are statements *about*: the
     * SDK and the product first (we cannot ask anyone anything), then the aircraft's simulated
     * state (we will not ask *this* aircraft), then DJI's declaration about the key itself. Only
     * the first is reached before the object is even usable, and none of them touches the flight
     * controller, which is why all three are [ActionOutcome.Unavailable].
     */
    override fun takeoff(): ActionOutcome {
        port.unavailableReason()?.let { return ActionOutcome.Unavailable(it) }
        // Read at the moment of the command, never cached: a simulator can be stopped from DJI
        // Assistant or another process between one command and the next, and a stale "yes" here
        // is a real aircraft taking off behind a gate that thinks it is guarding a simulated one.
        if (REQUIRE_SIMULATOR && !simulatedFlight()) {
            log("takeoff refused: no simulator observed and REQUIRE_SIMULATOR is set")
            return ActionOutcome.Unavailable(SIMULATOR_REQUIRED)
        }
        // Same fail-closed guard as the other two, and the same caveat: canPerformAction is a
        // compile-time constant (true for this key on 5.18.0, bytecode in
        // ActionPort.canStartTakeoff), so this catches an SDK demotion or a miswired key and
        // nothing about the aircraft's readiness. DJI ships no pre-flight check.
        if (!port.canStartTakeoff()) {
            log("takeoff not performable: DJI's key declares canPerformAction false")
            return ActionOutcome.Unavailable(CANNOT_PERFORM_ACTION)
        }
        port.startTakeoff(
            onFailure = { djiError ->
                // Verbatim, including DJI's bare `-7` — the documented flaky-takeoff code on this
                // airframe (#783). An operator who can search for "-7 KeyStartTakeoff" is better
                // served than one reading our guess at what it meant. Note DJI's own widget would
                // swallow this failure when motors are already on; we do not, because QGC offers
                // the button only when the aircraft is not flying, so that case cannot arise from
                // the one caller this interface has.
                log("takeoff start failed: $djiError")
                reportAsyncDjiError(djiError)
            },
        )
        // Asked, not refused on the spot, nothing more known — and specifically not a claim that
        // the aircraft is leaving the ground. The measured 2026-07-26 silence applies here as
        // much as to go-home: DJI may never call back either way.
        return ActionOutcome.Requested
    }

    override fun returnToHome(): ActionOutcome {
        port.unavailableReason()?.let { return ActionOutcome.Unavailable(it) }
        // DJI's own declaration about the key, checked before we perform it. It is a static flag
        // and will be true here on 5.18.0 (see ActionPort.canStartGoHome for the bytecode), so
        // this is a fail-closed guard against an SDK upgrade or a miswired key, not a pre-flight
        // check — MSDK offers no pre-flight check for these actions at all.
        if (!port.canStartGoHome()) {
            log("go-home not performable: DJI's key declares canPerformAction false")
            return ActionOutcome.Unavailable(CANNOT_PERFORM_ACTION)
        }
        port.startGoHome(
            onFailure = { djiError ->
                log("go-home start failed: $djiError")
                reportAsyncDjiError(djiError)
            },
        )
        // The truth about the call we made: asked, not refused on the spot, nothing more known.
        return ActionOutcome.Requested
    }

    override fun land(): ActionOutcome {
        port.unavailableReason()?.let { return ActionOutcome.Unavailable(it) }
        // Before ensureListening, so an unperformable landing leaves no subscription behind: a
        // listener whose landing can never be started by us could only ever observe someone
        // else's confirmation question, which is not ours to hear.
        if (!port.canStartAutoLanding()) {
            log("auto-landing not performable: DJI's key declares canPerformAction false")
            return ActionOutcome.Unavailable(CANNOT_PERFORM_ACTION)
        }
        // Listeners first, so the confirmation window cannot open unobserved between DJI
        // accepting the start and our subscription landing.
        ensureListening()
        port.startAutoLanding(
            onSuccess = {
                // Only here. This is the entire basis of the auto-confirm scope: DJI accepted a
                // start that came from an operator's Land press through this method.
                synchronized(gate) {
                    ourLanding = true
                    claimAcceptedAtMs = nowMs()
                }
                log("landing start accepted — this landing is ours to confirm")
            },
            onFailure = { djiError ->
                // ourLanding is deliberately not touched: if unset, a refused start never set
                // it; if set, it was set by an *earlier accepted* start whose landing is still
                // running, and a second press failing with "already landing" must not strip
                // that one of its confirmation.
                log("landing start failed: $djiError")
                reportAsyncDjiError(djiError)
            },
        )
        return ActionOutcome.Requested
    }

    /**
     * **Withdraw a landing mid-descent** — `KeyStopAutoLanding`, wired 2026-07-28 for exactly
     * one caller: the guided engine's rule-1 (and operator-withdrawal) handling of a committed
     * Stage C autoland, through `Bridge`'s `DjiLanding` seam. It is deliberately **not** on
     * [FlightActions]: the decision to offer no operator-facing Cancel stands untouched
     * (a Cancel that sometimes silently does nothing is worse than none — that file's KDoc),
     * and this is a *withdrawal completing a manual takeover*, not a button.
     *
     * No interlock read, on the withdrawal rule ([CommandDispatcher.pause]'s family): there is
     * no circumstance in which this bridge should decline to stop a descent. No claim
     * bookkeeping is touched — the episode-end and grace-expiry rules already govern
     * [ourLanding], and a cancelled landing's `KeyIsInLandingMode` true→false clears it on the
     * measured path. Whether DJI honours the stop at all is a measurement
     * (`docs/msdk/actions.md` §2: possibly uncancellable during forced landings), which is why
     * the ask rides the record (`dji_call op=stop_landing`) and the caller treats
     * [ActionOutcome.Requested] as "asked", never "stopped".
     */
    fun cancelLanding(): ActionOutcome {
        port.unavailableReason()?.let { return ActionOutcome.Unavailable(it) }
        if (!port.canStopAutoLanding()) {
            log("stop-landing not performable: DJI's key declares canPerformAction false")
            return ActionOutcome.Unavailable(CANNOT_PERFORM_ACTION)
        }
        port.stopAutoLanding(
            onFailure = { djiError ->
                log("stop-landing failed: $djiError")
                reportAsyncDjiError(djiError)
            },
        )
        return ActionOutcome.Requested
    }

    /** `Bridge.stop()`. Cancels the subscriptions and forgets any claim on a landing. */
    fun stop() {
        port.cancelListens()
        synchronized(gate) {
            listening = false
            ourLanding = false
            claimAcceptedAtMs = 0L
            confirmSent = false
            sawLandingMode = false
            lastNeeded = null
            neededEverDelivered = false
        }
    }

    /**
     * Drop a claim whose landing never began. Must be called with [gate] held; returns whether a
     * claim was actually dropped, so the caller can log it *outside* the lock — this class never
     * calls the caller's [log] with [gate] held.
     *
     * Evaluated on read rather than on a timer, which is why this class still owns no executor
     * and [stop] has nothing extra to cancel: a deadline that is only consulted when it matters
     * cannot fire late, cannot outlive the object, and cannot leak a thread into a process that
     * holds a live link to an aircraft. The one thing a timer would add is the log line arriving
     * on time rather than at the next key delivery — and a claim nobody ever reads has no effect
     * to be timely about.
     *
     * Correct even if `KeyIsLandingConfirmationNeeded` is the only key that ever arrives: the
     * deadline is checked from [onConfirmationNeeded] itself, so the expiry does not depend on
     * `KeyIsInLandingMode` being delivered at all — which is exactly the case it exists for.
     *
     * **This is not the watchdog `docs/decisions/2026-07-25-m2-command-safety.md` §Q4 forbids.**
     * §Q4 bans sending the aircraft commands nobody asked for; expiring our own bookkeeping sends
     * nothing — no `performAction`, no key write, nothing on any wire. Its only effect is that we
     * auto-confirm *less*, never more: it fails in the safe direction by construction, which is
     * the opposite of a watchdog that acts.
     */
    private fun expireStaleClaim(): Boolean {
        if (!ourLanding || sawLandingMode) return false
        // Elapsed-since form, like CommandDispatcher's repeat window: correct across the clock's
        // wrap, and unambiguous about which side of the window is inside it.
        if (nowMs() - claimAcceptedAtMs < LANDING_MODE_GRACE_MS) return false
        ourLanding = false
        confirmSent = false
        claimAcceptedAtMs = 0L
        return true
    }

    /**
     * Plant the confirmation subscriptions for a **Stage C full autoland** — `Bridge` calls
     * this when the guided engine accepts a full-autoland arm, because that descent never goes
     * through [land] (it flies virtual sticks to touchdown, `landingdata.md` §4 Option 1) and
     * the lazy first-`land()` subscription would otherwise leave the confirmation window
     * unobserved and unrecorded for exactly the flight that exists to measure it.
     *
     * Behind the same fail-closed availability gate as [land]'s subscription: before
     * `KeyManager` registration a listen is a silent no-op (`docs/architecture.md`), and a
     * subscription that pretends to exist is worse than a refusal the log names. Idempotent.
     */
    fun armAutolandListening() {
        port.unavailableReason()?.let {
            log("autoland listening not planted: $it")
            return
        }
        ensureListening()
    }

    private fun ensureListening() {
        synchronized(gate) {
            if (listening) return
            listening = true
        }
        port.listenIsLandingConfirmationNeeded(::onConfirmationNeeded)
        port.listenIsInLandingMode(::onIsInLandingMode)
    }

    /**
     * The ~0.5 m stall, and the two scopes that may answer it. The operator-Land scope is
     * unchanged: confirm if and only if the landing is ours and this episode's confirm has not
     * been sent. The Stage C scope runs only when that one declined and a live full-autoland
     * LANDING vouches for the moment — see the class KDoc. In every other case the delivery is
     * left entirely alone: a foreign landing's confirmation question is DJI's to ask and the
     * RC pilot's to answer.
     *
     * Every **edge** of the key is recorded first, unconditionally — the Stage C flight's
     * question "does this key fire during a virtual-stick descent at all?" is answered by
     * these lines, and an absence must be a measured absence.
     */
    private fun onConfirmationNeeded(needed: Boolean?) {
        val edge = synchronized(gate) {
            val changed = !neededEverDelivered || needed != lastNeeded
            neededEverDelivered = true
            lastNeeded = needed
            changed
        }
        if (edge) {
            log("KeyIsLandingConfirmationNeeded -> $needed")
            recordEvent(EventCode.LANDING_CONFIRM_NEEDED, needed.toString(), needed == true)
        }
        if (needed != true) return
        var expired = false
        val confirmNow = synchronized(gate) {
            // Before the decision, never after: this is the read that would otherwise act on a
            // claim left by a landing DJI accepted and never started.
            expired = expireStaleClaim()
            val allowed = ourLanding && !confirmSent
            // Marked sent before the call, so a re-delivery racing the callback cannot
            // double-fire, and a failed confirm stays failed rather than becoming a loop.
            if (allowed) confirmSent = true
            allowed
        }
        if (expired) log(CLAIM_EXPIRED_LOG)
        if (confirmNow) {
            log("landing stalled at confirmation height — confirming our own landing")
            performConfirm(guided = false)
            return
        }
        confirmForAutoland()
    }

    /**
     * The Stage C scope of a `needed = true` — the operator-Land scope has already declined.
     *
     * The clearance is read **outside [gate]** (the engine takes its own lock behind it, and
     * this class must never hold [gate] across that boundary), and read at all only now, at
     * the moment of the question: rule 1 can have killed the engagement between deliveries,
     * and a dead engagement answers null, which ends this method exactly as the pre-Stage-C
     * code ended — the delivery left alone, silently, because nothing of ours wants it.
     *
     * A live clearance whose gates fail is different: our own committed landing is being asked
     * a question we decline to answer, and that refusal is *news* — logged and recorded with
     * the reason, decision left to the operator, while the engine's sustained stick-down (the
     * primary mechanism, Ivan's gesture) keeps flowing underneath.
     */
    private fun confirmForAutoland() {
        val clearance = autolandClearance() ?: return
        val refusal = when {
            !interlockEnabled() -> "interlock off"
            clearance.fixAgeMs > CONFIRM_FRESH_MS ->
                "fix ${clearance.fixAgeMs}ms old, bound ${CONFIRM_FRESH_MS}ms"
            !clearance.fixWasInCone -> "last fresh fix outside the cone"
            else -> null
        }
        if (refusal != null) {
            log("autoland confirm refused: $refusal — leaving the decision to the operator")
            recordEvent(EventCode.LANDING_CONFIRM, "refused: $refusal", true)
            return
        }
        val send = synchronized(gate) {
            // The shared per-episode single confirm — the same flag the operator scope sets,
            // so the two scopes cannot double-fire one episode between them.
            if (confirmSent) {
                false
            } else {
                confirmSent = true
                true
            }
        }
        if (!send) return
        log("autoland stalled at confirmation height — confirming (engagement ${clearance.engagementAtMs})")
        recordEvent(EventCode.LANDING_CONFIRM, "sent", false)
        performConfirm(guided = true)
    }

    /** The one `KeyConfirmLanding` call site, both scopes. [guided] routes the success note. */
    private fun performConfirm(guided: Boolean) {
        port.confirmLanding(
            onSuccess = {
                // Announced from DJI's acceptance, not from our attempt: the operator reads
                // what happened, never what we hoped.
                announceLandingConfirmed()
                if (guided) onAutolandConfirmed()
            },
            onFailure = { djiError ->
                log("landing confirm failed: $djiError")
                if (guided) recordEvent(EventCode.LANDING_CONFIRM, "failed: $djiError", true)
                reportAsyncDjiError(djiError)
            },
        )
    }

    /**
     * Episode tracking. `true` arms the clear; the first delivery at or below `true → false`
     * ends the episode and releases every claim, so the next landing — whoever starts it —
     * begins with no right to an automatic confirm.
     */
    private fun onIsInLandingMode(inLandingMode: Boolean?) {
        var expired = false
        var ended = false
        synchronized(gate) {
            // First, on **every** delivery, and the ordering is the whole point. A `true`
            // arriving inside the window is our own landing beginning, and expiry is a no-op
            // that leaves the claim to be armed below. A `true` arriving *after* the window is
            // someone else's landing beginning — and if the claim were armed before being
            // expired, `sawLandingMode` would make it permanent (expiry only applies while
            // landing mode has never been seen) and we would auto-confirm a foreign landing.
            // That is precisely the failure this expiry exists to prevent, so it must not
            // depend on a `false` happening to arrive first.
            expired = expireStaleClaim()
            if (inLandingMode == true) {
                sawLandingMode = true
            } else if (sawLandingMode) {
                sawLandingMode = false
                ourLanding = false
                confirmSent = false
                ended = true
            }
            // A false or null *without* a preceding true falls through both branches untouched:
            // it is a re-delivered old state (KeyManager repeats unchanged values), not the end
            // of anything, and ignoring it is what keeps a just-accepted landing from being
            // orphaned before its descent engages.
        }
        if (expired) log(CLAIM_EXPIRED_LOG)
        if (ended) log("landing mode ended — claim and confirmation state cleared")
    }
}
