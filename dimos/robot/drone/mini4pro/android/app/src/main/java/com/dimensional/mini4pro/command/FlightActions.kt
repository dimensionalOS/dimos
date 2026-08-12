package com.dimensional.mini4pro.command

/**
 * The three things a QGroundControl operator may ask this bridge to make the aircraft do, and the
 * contract the DJI layer must satisfy to make any of them real.
 *
 * **This file contains no DJI code and must never contain any.** It is the seam: everything on
 * the MAVLink side of it ([CommandDispatcher], [CommandInterlock]) is unit-testable with no SDK,
 * no aircraft and no Android, and everything on the other side is one class implementing
 * [FlightActions]. The seam exists because the MAVLink half of M2 was built before the MSDK
 * action surface was known; the surface is now recorded in `docs/msdk/actions.md` and
 * `docs/msdk/actions-rth-and-arming.md`.
 *
 * ## Why these three, and what changed
 *
 * `docs/decisions/2026-07-25-m2-command-safety.md` §Q1 asked for three — return, land and
 * emergency stop. The action-surface research removed one and Ivan later added a different one,
 * so the count is a coincidence and the reasoning is not:
 *
 *  - **Takeoff was out by decision, and is now in — `docs/decisions/2026-07-26-takeoff.md`.**
 *    The original reasoning is preserved rather than deleted, because it is still every bit as
 *    true and it is what the new gates answer to. It said: takeoff is the one command whose
 *    failure mode is an aircraft leaving the ground when nobody expected it; and it is not merely
 *    unimplemented, because our `MAV_RESULT_UNSUPPORTED` for `MAV_CMD_NAV_TAKEOFF` is what stops
 *    QGC following up with `MAV_CMD_COMPONENT_ARM_DISARM param1=1` on its own initiative
 *    (`HandshakeResponder.kt:132`, measured). Both halves survive intact.
 *
 *    What changed is not the risk, it is what can be done about it. Ivan said of takeoff *"I like
 *    to have, but we can do it later"* and M2 recorded it as M2.5, blocked on two things: the
 *    failure mode, and an aircraft that was indoors so the code would ship untested. **The MSDK
 *    simulator now works on this airframe** (`docs/simulator.md`), so a simulated aircraft can
 *    take off on a desk and the second objection is gone. The first is answered by gates, not by
 *    optimism: [takeoff] is behind the same [CommandInterlock] as the other two *and* behind
 *    `MsdkFlightActions.REQUIRE_SIMULATOR`, and the arm follow-up is answered explicitly rather
 *    than allowed to happen — see [CommandDispatcher.onArmDisarm]'s KDoc and this file's
 *    [takeoff].
 *  - **Emergency stop is out because MSDK 5.18.0 cannot do it.** There is no callable in-flight
 *    motor cut at all: `KeyAreMotorsOn` is telemetry-only, `KeyLockMotors` is set-only and
 *    pre-flight, and `KeyEmergencyStopMotorEnable` / `KeyFCUrgentStopMotorMode` merely configure
 *    whether the RC's physical stick gesture is armed — `canPerformAction` is false on both
 *    (`docs/msdk/actions-rth-and-arming.md`). The only thing that takes motors to zero is a
 *    several-second controlled landing, and mapping "stop now" onto "start a slow descent" would
 *    be the most dangerous lie in this project. [CommandDispatcher] therefore keeps refusing the
 *    command and instead tells the operator where the real motor cut is: the RC stick gesture.
 *
 * There is deliberately no cancel, either. DJI's own reference widgets grey out their Cancel
 * buttons off `KeyAutoRTHReason` rather than trusting a cancel call to refuse cleanly, and the
 * MSDK offers no way to ask in advance whether a cancel will be honoured
 * (`docs/msdk/actions-rth-and-arming.md`). An operator-facing Cancel that sometimes silently
 * does nothing is worse than no Cancel: QGC's mode dropdown already fails silently
 * (`HandshakeResponder.kt:186`) and adding a second such control is not an improvement.
 *
 * ## What a correct implementation must do
 *
 * Read this as a specification, because the author of the implementation will not be the author
 * of this file.
 *
 *  1. **Return the truth about the call you made, not about what you hope happens.**
 *     [ActionOutcome.Requested] means *"I called the MSDK and it did not refuse on the spot"*. It
 *     is deliberately not named `Success`, `Ok` or `Accepted`: nothing on this side of the seam
 *     can know whether the aircraft complied, and the only honest evidence is the flight mode the
 *     aircraft subsequently reports through `Px4Mode`.
 *  2. **Do not block waiting for the outcome, and do not expect the callback to explain it.**
 *     These are called on the MAVLink receive thread (`mavlink-rx`). Start the MSDK action and
 *     return. `CompletionCallback.onFailure(IDJIError)` arrives later and carries only an
 *     error code, a hint and a description — the *useful* reason lives in the telemetry keys we
 *     already subscribe to (`KeyMotorStartFailureError` and friends, the source of the
 *     `FC_AUTH_STATE`, `GPS_DISCONNECT` and `NAV_SYS_EXCEPTION` the recorder caught live on
 *     2026-07-25). So the common case is [ActionOutcome.Requested] followed later by a reason
 *     arriving from a *different* channel — push it to the operator with
 *     [CommandDispatcher.reportAsyncDjiError], which exists precisely because the reason does
 *     not come back down this return path.
 *  3. **When you do have DJI's own word, pass it through untouched.** [ActionOutcome.Refused]
 *     carries the error name verbatim. Do not paraphrase, do not sentence-case, do not
 *     translate. The operator is better served by a string they can search DJI's forums for than
 *     by our guess at what it means. The dispatcher puts it on the wire inside a 50-byte
 *     `STATUSTEXT` and shortens *our* wording rather than DJI's if the two do not fit — see
 *     [StatusTexts].
 *  4. **Never invent an outcome for a situation you did not handle.** There is no default
 *     implementation of any method here and no catch-all: a stub that throws, a `TODO()`, or an
 *     `UnsupportedOperationException` all reach the operator as a failure with the thrown
 *     message attached ([CommandDispatcher] catches and reports rather than accepting). The
 *     unwritten implementation therefore fails closed. Only a deliberate lie — returning
 *     [ActionOutcome.Requested] without calling anything — can report success falsely, and even
 *     that lie is contained: it cannot move the flight mode in our heartbeat, which is the field
 *     QGC actually polls to decide the change worked (`FirmwarePlugin.cc:246-274`, cited at
 *     `HandshakeResponder.kt:583-587`).
 *
 * ## What the implementation must *not* do
 *
 * Nothing here is a failsafe. `docs/decisions/2026-07-25-m2-command-safety.md` §Q4: no automatic
 * go-home on link loss, no automatic landing on low battery, no watchdog. DJI's own failsafes
 * already do all of that. Every call into this interface originates in an operator pressing a
 * button in QGroundControl, and there must be no other caller.
 */
interface FlightActions {

    /**
     * Leave the ground. **The only call in this project that puts an aircraft into the air.**
     *
     * Arrives as `MAV_CMD_NAV_TAKEOFF` (22) in a `COMMAND_LONG` — QGC's **Takeoff** button, and
     * unlike Return and Land it *is* acknowledged, so this call's outcome reaches QGC directly as
     * a `COMMAND_ACK`. That acknowledgement is load-bearing in a way no other one in this project
     * is: `PX4FirmwarePlugin::_mavCommandResult` sends `MAV_CMD_COMPONENT_ARM_DISARM param1=1`
     * within milliseconds of an `ACCEPTED` on this command id (`PX4FirmwarePlugin.cc:307-315`,
     * measured). Answering anything else stops that chain dead.
     *
     * ## It takes no altitude, and that is a fact about DJI, not an omission
     *
     * QGC asks for one: `param7` is an **AMSL** altitude composed as
     * `requestedHeightAboveVehicle + vehicle->altitudeAMSL()` (`PX4FirmwarePlugin.cc:317-338`),
     * and `altitudeAMSL` is whatever we last published in `GLOBAL_POSITION_INT.alt`
     * (`Vehicle.cc:870`). [CommandDispatcher] inverts that arithmetic to recover the height the
     * operator actually asked for.
     *
     * **It then does not pass it here, because there is nowhere for it to go.** The MSDK call is
     * `FC.KeyStartTakeoff`, and in `dji-sdk-v5-aircraft-provided-5.18.0.jar` it is
     * `DJIActionKeyInfo<EmptyMsg, EmptyMsg>` — both converters are
     * `EmptyValueConverter`, so the action carries no parameter at all. Verified by `javap -c` on
     * `DJIFlightControllerKey.<clinit>`, the `"StartTakeoff"` entry — a **static initialiser**,
     * which is the one part of that jar that is not an ABI stub
     * ([ActionPort.READING_THE_JAR]); the bytecode is quoted in [ActionPort.canStartTakeoff].
     * DJI's own documentation for it: *"When the aircraft hovers at
     * an altitude of **1.2 meters (4 feet)** above the ground, taking off is completed."* A sweep
     * of every `Takeoff`-named key in the whole 5.18.0 key surface found exactly one settable
     * height — `FlightAssistantKey.KeyFlyingOnShipTakeoffHeight`, a ship-deck mode setting, not
     * this — so there is no altitude to command anywhere.
     *
     * A signature of `takeoff(altitudeM: Double)` would therefore be an invitation to a future
     * implementer to believe the number is honoured. **The operator's requested height is not
     * dropped**: [CommandDispatcher] puts both numbers in front of them
     * (`StatusTexts.takeoffHeight`) before the aircraft moves. What must never happen is this
     * layer silently accepting a 30 m takeoff and delivering 1.2 m with nobody told.
     *
     * ## What an implementation owes
     *
     * Everything the other two owe, plus one thing they do not: **fail closed on anything you
     * cannot see.** Return and Land move the aircraft toward the ground; this one does the
     * opposite, so an implementation must refuse when it cannot establish a precondition rather
     * than proceed on the assumption that silence means fine. `MsdkFlightActions` does this three
     * ways — availability, DJI's own capability flag, and the simulator requirement — and each is
     * an [ActionOutcome.Unavailable] with a name the operator can read.
     *
     * `KeyStartTakeoff` is **flaky on this exact airframe**: intermittent `-7`, which DJI blames
     * on GPS or the link (DJI [#783](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/783),
     * `docs/mini4pro-constraints.md`). Kill DJI Fly before launching. And note DJI's own reference
     * widget treats a `KeyStartTakeoff` failure as a *success* when motors are already on
     * (`TakeOffWidgetModel.kt:127-138`) — we do not copy that, because reporting DJI's word
     * verbatim is this project's rule and QGC only offers the button when the aircraft is not
     * flying, so the case is unreachable from the one caller that exists.
     */
    fun takeoff(): ActionOutcome

    /**
     * Fly to the recorded home point and land there.
     *
     * Arrives as `SET_MODE` (#11) with PX4 `custom_mode` `AUTO.RTL` — QGC's **Return** button.
     * It is unacknowledged, so this call's return value never reaches QGC directly: a refusal
     * becomes a `STATUSTEXT`, and success is only ever visible as the aircraft reporting
     * `GO_HOME` (`Px4Mode.kt:219`) in a later heartbeat.
     *
     * Called at most once per operator press — QGC sends the same request three times about
     * 1.34 s apart and [CommandDispatcher] collapses the burst. An implementation may still be
     * called again for a genuine second press, and should be safe to call while a return is
     * already under way.
     *
     * The MSDK call is `FC.KeyStartGoHome` (`performAction`, documented, and
     * `KeyIsGoHomePathSupport` reads true on this airframe — `docs/msdk/actions.md:87`). Progress
     * shows up on `KeyGoHomeStatus` and as `FCFlightMode.GO_HOME`, which is already the
     * heartbeat's source.
     */
    fun returnToHome(): ActionOutcome

    /**
     * Land here, now.
     *
     * Arrives as `SET_MODE` (#11) with PX4 `custom_mode` `AUTO.LAND` — QGC's **Land** button.
     * Same unacknowledged path, same burst collapsing, same rule about evidence: success shows
     * up as one of the DJI landing modes (`Px4Mode.kt:240-244`) in a later heartbeat.
     *
     * Note this is a landing, not a stop. It takes several seconds and the aircraft is still
     * flying for all of them. It must never be offered as a substitute for a motor cut — see the
     * emergency-stop paragraph in this file's KDoc.
     *
     * The MSDK call is `FC.KeyStartAutoLanding` (`performAction`, documented,
     * `docs/msdk/actions.md:84`). **It is not the whole job.** DJI stalls the descent at roughly
     * 0.5 m and raises `FC.KeyIsLandingConfirmationNeeded`; the landing only completes once
     * `FC.KeyConfirmLanding` is sent (`docs/msdk/actions.md:167-169`). An implementation that
     * calls `KeyStartAutoLanding` alone leaves the aircraft hovering half a metre up while QGC's
     * flight mode says "Land" — which is this bridge's characteristic failure, arrived at from
     * the DJI side. Whether the confirmation is automatic or needs the operator is a decision for
     * whoever writes that class; leaving it unhandled is not one of the options.
     */
    fun land(): ActionOutcome
}

/**
 * Which of [FlightActions]' calls a request resolved to.
 *
 * Exists so [CommandDispatcher] can key its de-duplication and its log lines on the *action*
 * rather than on the `SET_MODE` custom mode the actions arrive in.
 */
enum class FlightAction(
    /**
     * How this action is named to the operator inside a `STATUSTEXT`. Short on purpose: the
     * field is 50 bytes and DJI's error name has first claim on them.
     */
    val label: String,
) {
    /**
     * `MAV_CMD_NAV_TAKEOFF` (22). The only member that arrives as an **acknowledged command**
     * rather than a `SET_MODE`, which is why [CommandDispatcher.perform] has a `MavResult` to
     * return at all.
     */
    TAKEOFF("Takeoff"),
    RETURN_TO_HOME("Return"),
    LAND("Land"),
}

/**
 * What came back from one call into [FlightActions].
 *
 * Three cases, and the naming is load-bearing. There is no `Success`: this type describes what
 * happened to *the request*, never what happened to the aircraft. Whoever implements
 * [FlightActions] reads these names before they read anything else, and a case called `Success`
 * invites exactly the claim this bridge must never make.
 */
sealed interface ActionOutcome {

    /**
     * The MSDK was asked and did not refuse on the spot. **Not a claim that the aircraft is
     * doing it**, and not even a claim that the action will not fail — MSDK failures arrive
     * asynchronously on `CompletionCallback.onFailure`, after this has already been returned.
     *
     * This is therefore the *expected* outcome even for a request that ultimately fails. The
     * operator's confirmation is the flight mode changing in a later heartbeat, derived from the
     * aircraft alone; the operator's explanation, when it fails, comes from the telemetry keys
     * via [CommandDispatcher.reportAsyncDjiError].
     *
     * **Amended by measurement, 2026-07-26** — `docs/measurements/2026-07-26-m2-first-command.md`.
     * This case used to reach the operator as *silence*, on the reasoning above: the heartbeat's
     * flight mode is the honest confirmation, so anything we said here would be our own
     * uncorroborated claim. The first real press proved the reasoning incomplete. An operator
     * pressed Return on a connected, healthy, grounded aircraft; `performAction(KeyStartGoHome)`
     * was called; DJI invoked **neither** `onSuccess` nor `onFailure`, ever; `goHomeState` stayed
     * `IDLE`; and not one `STATUSTEXT` left the bridge all session. All three channels the
     * argument depends on — the mode, the callback, the telemetry keys — stayed silent together,
     * and the operator could not distinguish "sent" from "the bridge is broken".
     *
     * A later press with GNSS locked settled where the silence comes from: **the RC beeped.** The
     * flight controller does receive these requests and does refuse them; it simply answers the
     * pilot on the remote and not the SDK. So an implementation must never read "no callback" as
     * "no decision" — the decision happened, on a channel we cannot hear.
     *
     * So [CommandDispatcher] now announces the dispatch itself (`StatusTexts.dispatched`, "Return
     * sent to DJI"). The naming discipline in this file is unchanged and governs that sentence
     * too: it reports what *we* did, never what the aircraft is doing. Silence is not honesty
     * when there is something true to say.
     */
    object Requested : ActionOutcome

    /**
     * DJI said no *before we let go of the call*, and this is DJI's own word for why.
     *
     * Rarer than you would expect: the MSDK's own refusals are mostly asynchronous, so this
     * covers the checks an implementation can make synchronously and any `IDJIError` it already
     * holds. [djiError] must be the error name as the MSDK spells it — `FC_AUTH_STATE`,
     * `GPS_DISCONNECT`, `NAV_SYS_EXCEPTION`. It reaches the operator verbatim.
     *
     * A blank reason is rejected at construction rather than tolerated: a refusal the operator
     * cannot act on is barely better than silence, and the whole point of
     * `docs/decisions/2026-07-25-m2-command-safety.md` §Q3 is that DJI names its refusals
     * precisely and we should not lose that. If there is genuinely no name, the honest move is
     * [Requested] followed by the real reason on
     * [CommandDispatcher.reportAsyncDjiError] when telemetry supplies it.
     */
    class Refused(val djiError: String) : ActionOutcome {
        init {
            require(djiError.isNotBlank()) {
                "a refusal must carry DJI's own error name; use Unavailable if there is no name"
            }
        }

        override fun toString(): String = "Refused($djiError)"
    }

    /**
     * We could not ask at all — no aircraft connected, the SDK not registered, the airframe has
     * no such capability. Distinct from [Refused] because it is a statement about *us*, not a
     * verdict from the flight controller, and the operator's next move is different.
     */
    class Unavailable(val reason: String) : ActionOutcome {
        init {
            require(reason.isNotBlank()) { "an unavailable action must say why" }
        }

        override fun toString(): String = "Unavailable($reason)"
    }
}
