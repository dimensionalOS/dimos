package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.ClimbArm
import com.dimensional.mini4pro.command.PendingClimb
import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.SetpointFrame
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.vision.RangeSource
import io.dronefleet.mavlink.common.ManualControl
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sqrt

/**
 * M3 Stage A: the operator's hand — QGC's on-screen sticks or a gamepad, arriving as
 * `MANUAL_CONTROL` (#69) — passed near-directly to DJI virtual-stick velocity setpoints.
 * Every decision Stage A makes lives in this class, above the [VirtualStickPort] seam, so all
 * of it runs under `GuidedStickEngineTest` with a fake port, a fake clock and no aircraft.
 *
 * Design authority: `docs/m3-guided-control.md` §1.3/§3; decisions closed in
 * `docs/decisions/2026-07-26-m3-guided-control.md` (Q1 envelope + single interlock, Q3 abort
 * gestures, Q4 swappable link-loss policy, Q5 never claim a mode, Q6 stage order). What is
 * built and what the bench must still prove: `docs/m3-stage-a.md`.
 *
 * ## Two setpoint sources, one owner — Stage B lives inside this engine
 *
 * Stage A made this class a **mapping with an envelope and a conscience**: axes in,
 * envelope-bounded velocities out, everything else deciding *whether* the mapping may run and
 * how it lets go. Stage B (`DO_REPOSITION` → the [RepositionGuidance] law → arrival →
 * keep-station) is a **second source of velocities inside the same tick**, not a second
 * controller: exactly one 10 Hz loop, one abort ladder, one watchdog, one path to the
 * [VirtualStickPort], whatever the setpoint's origin. The engaged tick branches once — an
 * active [reposition] target computes velocities from the guidance law; otherwise the Stage A
 * passthrough runs — and every safety check sits **above** that branch, so no manoeuvre type
 * can be reached without passing all of them.
 *
 * **Stage C — orbit — is a *third* source inside the same tick**, on exactly the same terms:
 * [orbit] takes a `DO_ORBIT`, the engaged tick's one branch grows a third arm, and the abort
 * ladder still sits above all of it. No second engine, no second timer, no origin-parameterised
 * envelope. An orbit is two phases — a **join** flown as an ordinary resting leg by the M3 law
 * and ended by the M3 arrival test, both *called* rather than restated, and then a **circle** whose
 * radial axis is that same law in one dimension while its tangential speed is bounded by the
 * curvature cap `sqrt(a_max·R)`. `docs/m4-mission-execution.md` §8 is the specification.
 *
 * The one M3 rule Stage C breaks is *"yaw rate is always zero"*, and it breaks it in one place
 * only: [tickOrbitCircleLocked], while our own executor holds authority and is circling. The
 * passthrough branch, the plain-reposition branch and the orbit's own resting legs all still
 * command exactly zero, and `GuidedOrbitTest` fails if any of them stops doing so. The permission
 * and its bounds are `docs/decisions/2026-07-27-orbit-yaw.md`, which is a decision rather than a
 * suggestion.
 *
 * ## The takeoff's second phase is not a fourth source
 *
 * A takeoff is two phases (`docs/m4-mission-execution.md` §3.6): DJI's own ~1.2 m hop, and then a
 * **vertical-only climb to the height the operator asked for**. This engine flies the second one,
 * and it flies it as *an ordinary Change Altitude* — [acceptTarget] with no explicit coordinate,
 * so the target is the current fix at a new height, through the same gates, the same envelope, the
 * same ceiling, the same arrival test and the same abort ladder. There is no takeoff branch in the
 * tick, no takeoff velocity law and no takeoff manoeuvre state, because there is nothing about the
 * climb that differs from the climb this engine already flew on 2026-07-27 to 0.09 m of error.
 *
 * What *is* new is a **pending intention** between the phases: armed by
 * [armTakeoffClimb] when M2.5 dispatches a takeoff to DJI, held while DJI flies it, and turned
 * into that Change Altitude by [TakeoffClimb] the moment DJI reports its own takeoff finished.
 * It lives here, rather than in `command/`, for one reason: **an armed climb that survived an
 * abort would fire later into a situation nobody remembers creating**, and every rung that could
 * abort is in this class. So [tickTakeoffClimbLocked] re-runs the interlock and RC-stick rungs
 * against the intention on every tick — the engaged tick is not running, because this engine holds
 * no authority while DJI flies — [abort] clears it *above* its own IDLE early-return, a deliberate
 * GCS deflection clears it in [onFrame], and it expires on its own after
 * [TakeoffClimb.WAIT_LIMIT_MS]. `GuidedTakeoffClimbTest` exists mostly to fail if any one of those
 * stops being true.
 *
 * The rejected alternative, recorded because the choice was delegated
 * (`docs/m3-stage-b.md`): a sibling `RepositionController` composing with this engine through
 * a setpoint-supplier interface. Rejected because every one of its needs — the envelope, the
 * aborts, the watchdogs, the recorder, the announcements, the engagement machine — already
 * lives here, so the sibling would either duplicate them (two places for one safety property)
 * or call back into this class for each (an interface as wide as the class), and its own
 * executor would be precisely the second 10 Hz stream the design forbids.
 *
 * ## Engagement never latches on "we asked" — landmine 5
 *
 * DJI can accept a call and never fire either callback (measured,
 * `docs/measurements/2026-07-26-m2-first-command.md`), so [GuidedPhase.ENGAGING] is entered on
 * our request and [GuidedPhase.ENGAGED] **only** on DJI's own `VirtualStickState` reporting
 * enabled + advanced + authority `MSDK` — `GcsMirror.authorityCode == 111`, *"the only value
 * under which a stick command means anything"*. No confirmation inside [ENGAGE_CONFIRM_MS]
 * cleans up (a disable that is safe when nothing enabled) and tells the operator
 * ([GuidedStatusTexts.NO_CONFIRM]). The same observe-don't-assume discipline as
 * `SimulatorControl`'s phase machine, and the same reason: a claim comes from an observation,
 * never from a request.
 *
 * ## The engagement gate, in full
 *
 * QGC streams `MANUAL_CONTROL` at 25 Hz whenever its virtual-joystick setting is on — touched
 * or not, flying or not. Receipt is therefore **not** operator intent, and treating it as
 * intent would hand the aircraft to this bridge for entire sessions. Engagement requires, at
 * the instant a *deliberate* deflection ([StickMapping.DELIBERATE]) arrives:
 *
 *  1. **The interlock is on** — Q1: the same single `Bridge.commandInterlock` that gates
 *     Return and Land, off at every start, unreachable from MAVLink.
 *  2. **The SDK can be asked** — `unavailableReason() == null`, per-call fresh.
 *  3. **The stream was recently seen at rest** — a fully neutral frame (throttle at its 500
 *     centre included) within [NEUTRAL_RECENT_MS]. This is the guard on landmine 2: a stream
 *     whose resting throttle is not 500 (QGC's opt-in centre-zero regime, or any
 *     −1000..1000-convention sender) can never engage, because its idle state never reads
 *     neutral. See [StickMapping] for the source-verified conventions.
 *  4. **The RC stick feed is alive** — all four `KeyStick*` values delivered non-null. Abort
 *     gesture 1 is *ours*, built on that feed; engaging blind to it would fly without the
 *     safety story Q3 was answered with.
 *
 * Deliberately **not** required: the aircraft flying. The bench block
 * (`docs/m3-guided-control.md` §7, items 1–4) needs ground engagement with motors off, and
 * DJI's own refusals — not our guess at them — are the honest answer to "what happens on the
 * ground". Least-authority is not served by a gate that also blocks the measurement sessions
 * this feature exists to enable.
 *
 * ## The three abort gestures — Q3, verbatim
 *
 * All idempotent, all safe from any thread ([abort]), all ending in zero setpoint → disable →
 * `STATUSTEXT` with the reason:
 *
 *  1. **RC sticks**: deflection past [RC_ABORT_DEFLECTION] sustained [RC_ABORT_SUSTAIN_MS],
 *     detected by **us** from the `KeyStick*` feed this engine subscribes to itself — never
 *     trusting DJI to arbitrate, because DJI documents RC and virtual-stick input as *mixed*,
 *     not overridden (`docs/flight-recording.md:270`).
 *  2. **The interlock switch** — checked every tick; off means gone.
 *  3. **Anything DJI says**: any `FlightControlAuthorityChangeReason` other than our own
 *     `MSDK_REQUEST`, any `VirtualStickState` in which conjunct-A fails, the product
 *     disappearing, and — landmine 6 — a **null** on any subscribed RC stick key, which is
 *     DJI's component-gone signal (measured: FC blackouts deliver null across the board for
 *     45–120 s) and is treated as authority lost, never as "sticks centred".
 *
 * ## The watchdog and the two ends of a silent stream
 *
 * DJI wants 5–25 Hz and documents nothing about silence, so silence is never relied on
 * ([TICK_MS] keeps a 10 Hz stream flowing whenever engaged). Inbound, two windows:
 * [GuidedEnvelope.INPUT_STALE_MS] of missing `MANUAL_CONTROL` starts a ramp to zero (engaged,
 * recoverable — the measured worst gap on this rig is 1.01 s, so this fires benignly and
 * un-fires on the next frame); [GuidedEnvelope.LINK_LOST_MS] ends the stream. Which *terminal
 * sequence* runs depends on whether the rest of the link is also silent: other traffic still
 * arriving means the operator turned the sticks off — reason `released`, the standard
 * wind-down — while total silence is a dead link and runs the armed [LinkLossPolicy] (Q4),
 * whose name was logged at engage time. Input returning before the release step has run
 * resumes passthrough in either case.
 *
 * ## What this class never does
 *
 * Claim a flight mode (Q5: the heartbeat reports what DJI reports, throughout — this class has
 * no route to the heartbeat, and keeping that absence is the guarantee, during repositions
 * exactly as during passthrough); persist anything; retry a refusal on its own; or send any
 * command the operator did not ask for — every send is the operator's current deflection under
 * the envelope, the progress of a reposition the operator commanded and this engine
 * acknowledged, or the bounded withdrawal of one of those already in progress.
 */
class GuidedStickEngine(
    private val port: VirtualStickPort,
    /** `Bridge.commandInterlock::enabled` — Q1's single switch. Read fresh on every decision. */
    private val interlockEnabled: () -> Boolean,
    /** The same snapshot the encoder reads; used only for the ceiling's height-above-datum. */
    private val aircraftState: () -> AircraftState,
    /**
     * Where an operator-facing sentence goes — every attached interface, not one link. Replaced
     * the `send: (Any) -> Unit` that built a `STATUSTEXT` here; [ANNOUNCE_REPEAT_MS] and the
     * suppression it drives are untouched, and the 50-byte clamp moved with the message it
     * belongs to (`mavlink/StatusTextSink`).
     */
    private val announcer: Announcer,
    /**
     * **Heading follows course — the one flag, and the only way back to the flight-verified
     * behaviour.** Read fresh on every tick, exactly as [interlockEnabled] is, so a switch thrown in
     * the air takes effect on the next setpoint rather than at the next engagement.
     *
     * Defaults to **on**, which is `docs/decisions/2026-07-27-heading-follows-course.md`'s
     * recommendation and what Ivan asked for after watching a leg flown with the nose fixed: *"the
     * drone is still not turning towards the destination when it's flying a waypoint. No yaw
     * change."* The switch exists because the *current* behaviour is the flight-verified one, and
     * keeping a way back to it costs one boolean — a frame surprise or a yaw-mixing surprise in the
     * air can then be turned off from the phone rather than requiring a new build.
     *
     * Scope, stated because it is narrower than "every commanded manoeuvre": this governs a plain
     * `DO_REPOSITION` goto and every mission leg. It deliberately does **not** touch the orbit,
     * whose join leg has flown and whose circling law is already a heading command
     * (`docs/decisions/2026-07-27-orbit-yaw.md`), and it cannot touch stick passthrough, where the
     * operator's own `r` axis is being relayed and ours would be fighting it. The rule that matters
     * is *generate versus relay*.
     */
    private val headingFollowsCourse: () -> Boolean = { true },
    /**
     * Stage C's one call into the camera: an absolute pitch, open loop. Null in every
     * configuration that has no gimbal — including every Stage A/B test — and an orbit flies
     * perfectly well without one; the aircraft circles and the camera stays wherever it was.
     * See [ManoeuvreGimbal] for why this interface cannot express an attitude age.
     */
    private val manoeuvreGimbal: ManoeuvreGimbal? = null,
    /**
     * **The takeoff seam** — phase one of a mission's `NAV_TAKEOFF` item, and the *only* way this
     * engine can start a motor.
     *
     * Null in every configuration that has no `FlightActions` behind it, including every unit test
     * that does not exercise takeoff, and a mission whose first item is a takeoff is refused at
     * Start when it is null rather than silently skipped.
     *
     * It is one call and nothing else on purpose: `KeyStartTakeoff` is `EmptyMsg → EmptyMsg` and
     * the aircraft stops at its own ~1.2 m, so there is no altitude to pass and nothing to
     * acknowledge. **Phase two — the commanded climb to the item's altitude — is deliberately not
     * built here**; see [tickMissionTakeoffLocked] for exactly where it attaches.
     */
    private val missionTakeoff: MissionTakeoff? = null,
    /**
     * **Stage C's commit seam** — DJI's own landing, started on the commit edge and stopped by
     * rule 1 / the operator's withdrawals; see [DjiLanding]. Null in every configuration with
     * no command layer behind it, and a full-autoland commit is then refused by name
     * (`NO_LANDING_PATH`) rather than silently hovering at the FC floor.
     */
    private val djiLanding: DjiLanding? = null,
    /**
     * **M3 Stage D's sensor seam**: what the tag pipeline can say right now, flattened to plain
     * values ([TagDescentSense]) at the `Bridge` wiring. Read per decision, never cached — the
     * recogniser starts and stops independently of this engine. Null in every configuration
     * with no on-board detector, including every earlier-stage test, and arming a descent is
     * then refused by name rather than attempted blind.
     */
    private val tagSense: (() -> TagDescentSense?)? = null,
    /**
     * The **believed** camera pitch, degrees, −90 at nadir, or null when neither belief exists —
     * `gimbal/PitchBelief`'s resolution at the `Bridge` wiring: the commanded angle when this
     * bridge has aimed the camera (exact, never stale), the last-*reported* attitude when it has
     * not (the RC-wheel session; believed by value and never by age, because
     * `KeyGimbalAttitude` is change-driven and silence means unchanged — the trap this project
     * has hit seven times, stated at [ManoeuvreGimbal] and `vision/CameraPose`). The descent's
     * arm gate and its mid-flight consistency check both read this, and it is deliberately the
     * same resolution `TagWorld.fix` judges, so the gate and the fix ladder cannot disagree
     * about where the camera points.
     */
    private val cameraPitchDeg: () -> Double? = { null },
    /**
     * The **reported** gimbal pitch — `KeyGimbalAttitude` as last delivered, `GimbalManager`'s
     * reading — or null when there has never been one. **The one consumer of a gimbal
     * measurement this engine has, and its scope is exactly the landing's gimbal watchdog.**
     *
     * Why a measurement is admissible here when [ManoeuvreGimbal] forbids feedback: the
     * change-driven trap is about *ages* — silence cannot distinguish "not moved" from "feed
     * dead", so no gate may rest on how old the reading is, and none does (this supplier
     * carries no age, and the watchdog reads only the value). The event the watchdog watches
     * for is a **movement DJI itself makes** — the measured landing recenter slews at
     * 250–300 °/s and delivered a fresh burst of samples on all four landing01/02 landings
     * (`landingdata.md` §2.4) — and a movement on a change-driven key is precisely the case
     * the key delivers. The failure mode of a stale value is a watchdog that stays quiet
     * (the reading still says −90), which fails toward not fighting DJI: the honest direction.
     */
    private val gimbalReportedPitchDeg: () -> Double? = { null },
    private val record: GuidedRecord = GuidedRecord.None,
    /** Q4's armed policy. A constructor value, not a setter: swapping it is a build. */
    private val policy: LinkLossPolicy = LinkLossPolicy.SHIPPED,
    private val log: (String) -> Unit = {},
    /** Monotonic clock — `SystemClock.elapsedRealtime` in the app, hand-cranked in tests. */
    private val nowMs: () -> Long = { System.nanoTime() / 1_000_000L },
) : PendingClimb {

    companion object {
        /**
         * The setpoint cadence, 10 Hz: inside DJI's documented 5–25 Hz band with margin on
         * both sides, matching the ~12 Hz feedback rate the aircraft was measured to deliver
         * in flight, and 2.5× QGC's 25 Hz input so at most one fresh frame is ever waiting.
         * Deliberately not `Bridge`'s 200 ms tick: `architecture.md` requires the guided
         * controller to own its own fixed-rate executor, because this loop is control, not
         * telemetry.
         */
        const val TICK_MS = 100L

        /**
         * DJI's own name for the mode it flies while virtual stick has authority —
         * `AircraftState.flightMode` carried exactly this string for the whole first
         * engagement (record `20260726-204721.001`, t=34.0, 0.3 s after ENGAGED).
         */
        const val JOYSTICK_MODE = "JOYSTICK"

        /**
         * How long after ENGAGED the flight mode may still read as something other than
         * [JOYSTICK_MODE] before that reads as DJI seizing the aircraft. The measured flip
         * was ≤0.3 s; 1.5 s covers a slow key delivery without meaningfully extending the
         * window in which a real takeover goes unnoticed.
         *
         * Why the check exists at all: the 2026-07-26 overheat `GO_HOME` flew the aircraft
         * for 40 s while `VirtualStickState` still reported enabled+advanced+MSDK — DJI can
         * seize the aircraft without touching the authority owner, so the flight mode is an
         * authority signal in its own right (Q3's third gesture, by another wire).
         */
        const val MODE_SEIZE_GRACE_MS = 1500L

        /**
         * How long an engagement may sit unconfirmed before it is abandoned. The simulator
         * bench measured DJI round-trips at ~1 s for comparable manager calls
         * (`docs/measurements/2026-07-26-simulator-on-hardware.md`: start confirmed in
         * 957 ms); 3 s is three of those, and past it the likeliest truths are the measured
         * swallowed-callback case or an FC blackout — both of which mean "not now".
         */
        const val ENGAGE_CONFIRM_MS = 3_000L

        /**
         * Minimum spacing between engage attempts, so a failing enable is not hammered at
         * 25 Hz by a held stick. The project's one-press window (`ACTION_REPEAT_MS` family).
         */
        const val ENGAGE_RETRY_MS = 5_000L

        /**
         * How recently the stream must have been seen fully at rest for a deflection to
         * engage. At QGC's 25 Hz a resting stream proves itself within 40 ms, so 2 s is
         * generous for the honest case and permanently unsatisfiable for a stream that never
         * rests at our neutral — which is the point (gate 3 above).
         */
        const val NEUTRAL_RECENT_MS = 2_000L

        /**
         * Abort gesture 1's deadband, as a fraction of full RC stick travel (±660). The
         * recorder treats 0.05 as "pilot input" for *analysis*; doubling it for an *abort*
         * keeps transmitter slop and a nudged table from ending an engagement, while a real
         * grab — tens of percent in the first moment — clears it instantly.
         */
        const val RC_ABORT_DEFLECTION = 0.10

        /** Q3's "sustained briefly": two consecutive engine ticks, debouncing a knocked stick. */
        const val RC_ABORT_SUSTAIN_MS = 200L

        /** Identical `STATUSTEXT` suppression window — the same 5 s every announcer here uses. */
        const val ANNOUNCE_REPEAT_MS = 5_000L

        /**
         * Identical *logcat* suppression window for the per-frame refusal lines. The gates are
         * consulted at QGC's 25 Hz, and before this window existed a held stick against a
         * refusing gate printed 25 lines a second (measured, first-engagement session). The
         * first occurrence goes out verbatim; a changed reason logs immediately.
         */
        const val LOG_REPEAT_MS = 2_000L

        /**
         * How stale the last mission setpoint may be before [missionFlying] stops being true —
         * i.e. before the heartbeat stops claiming `AUTO_MISSION`.
         *
         * Five ticks. Long enough that one late tick on a busy phone does not flicker the mode QGC
         * is polling, short enough that a loop which has genuinely stopped sending is telling the
         * truth again well inside QGC's own 1.3 s validation window. The claim is a *report* on the
         * setpoint stream, so its window is the setpoint stream's cadence and nothing else.
         */
        const val MISSION_CLAIM_STALE_MS = 500L

        /** `stick_cmd.src` message name for guidance-law sends — the command that asked for them. */
        const val REPOSITION_SOURCE = "DO_REPOSITION"

        /** `stick_cmd.src` for a mission leg's sends. The plan is what asked for them. */
        const val MISSION_SOURCE = "MISSION"

        /** `stick_cmd.src` for the orbit's sends, in both its phases. Stage C's own name. */
        const val ORBIT_SOURCE = "DO_ORBIT"

        /**
         * `stick_cmd.src` for the **precision `NAV_LAND` sequence's** sends, in all three of its
         * phases. Distinct from [MISSION_SOURCE] deliberately: it is still the plan that asked, but a
         * post-flight reader separating "flying the plan" from "the plan's landing approach" should not
         * have to reconstruct that from the phase events beside it. Everything below the arm reverts to
         * [TAG_DESCENT_SOURCE], because from there it genuinely is the descent that is asking.
         */
        const val LAND_TAG_SOURCE = "MISSION_LAND_TAG"

        /**
         * `stick_cmd.src` for the tag-tracked descent's sends. Not a MAVLink command name,
         * because no MAVLink command starts one today — the phone's arm control does; the name
         * says what asked for the setpoint, which is the descent itself.
         */
        const val TAG_DESCENT_SOURCE = "TAG_DESCENT"

        /**
         * `stick_cmd.src` for the shadow's **would-be** commands — never sent, only recorded.
         * The entry is doubly marked: this source name, and `accepted = null` (the value that
         * means "no SDK call was made at all" — a real send always carries true or false), so
         * a post-flight diff can never mistake a shadow line for an actuated one.
         */
        const val TAG_DESCENT_SHADOW_SOURCE = "TAG_DESCENT_SHADOW"

        /**
         * How old the shadow's newest would-be command may be before the comparison view must
         * blank its arrow rather than draw it — a frozen arrow reads as a live opinion. Five
         * ticks, the [MISSION_CLAIM_STALE_MS] argument: one late tick on a busy phone must not
         * flicker the display, and a loop that has genuinely stopped computing must stop being
         * drawn well inside a human glance.
         */
        const val SHADOW_CMD_STALE_MS = 500L

        /** Our MAVLink system id, mirrored from `MavlinkLink` so this file imports no transport. */
        const val SYSTEM_ID = 1

        /** DJI's reason name for the authority change *we* cause by engaging — the one that is not an abort. */
        const val OUR_AUTHORITY_REASON = "MSDK_REQUEST"

        /**
         * The wind-down used when the *sticks* stop but the link is alive (reason `released`):
         * the shipped Q4 shape, fixed — the policy choice governs link loss only.
         */
        val STANDARD_RELEASE = LinkLossPlan(
            rampToZeroMs = GuidedEnvelope.RAMP_TO_ZERO_MS,
            holdZeroMs = DecelerateThenHandback.HOLD_MS,
            release = true,
        )

        /** Released immediately: the idle disengage arrives already at zero velocity. */
        val IMMEDIATE_RELEASE = LinkLossPlan(rampToZeroMs = 0L, holdZeroMs = 0L, release = true)

        /** The Q1 envelope as the recorder's `StickRange`, written once per engagement. */
        val ENVELOPE_RANGE = StickRange(
            rollPitchMax = GuidedEnvelope.HORIZONTAL_MAX_MS,
            verticalMax = GuidedEnvelope.VERTICAL_MAX_MS,
            yawMax = GuidedEnvelope.YAW_RATE_MAX_DEGS,
        )
    }

    /** Why an engagement ended. [wire] is the word the Q5 announcement carries. */
    enum class DisengageReason(val wire: String) {
        /** The GCS stopped sending sticks while the link stayed up. */
        RELEASED("released"),

        /** Abort gesture 1 — the pilot's hands. */
        RC_STICKS("sticks"),

        /** Abort gesture 2 — the app switch. */
        INTERLOCK("interlock"),

        /** Abort gesture 3 — DJI's word, a failed conjunct, a vanished component, a send that threw. */
        AUTHORITY("authority"),

        /** The whole link died; the armed [LinkLossPolicy] ran. */
        LINK_LOST("link-lost"),

        /** Q1: five minutes with every axis neutral. */
        IDLE("idle"),

        /** Q1: one deflection held for sixty seconds — a stuck axis, not a hand. */
        TIMEOUT("timeout"),

        /**
         * Stage B: the position feed stayed stale past [RepositionGuidance.POSITION_LOST_MS]
         * mid-reposition. The loop held zero the whole time (never a cached fix); past the
         * window the honest act is handing the aircraft back to controllers that have their
         * own GPS.
         */
        NO_POSITION("no-fix"),

        /**
         * Stage D: the tag's newest world fix aged past [TagDescentGuidance.T_ABORT_MS]
         * mid-descent. The ladder held and then climbed the whole time — never a descent on a
         * stale fix — and past the bound the honest act is the same one [NO_POSITION] takes,
         * for the same reason: a descent without its sensor has no business holding authority.
         */
        TAG_LOST("tag-lost"),

        /**
         * Stage C: motors-off observed during a full autoland's LANDING — the wheels are down
         * and the engagement is complete. The one disengage in this enum that is a *success*.
         * Detected from `KeyAreMotorsOn`/`KeyIsFlying`, never from altitude: the barometric
         * floor sits below zero at touchdown (`landingdata.md` §4 — `relalt = −0.1 m` at rest,
         * measured), so height is the one signal that cannot say "landed".
         */
        TOUCHDOWN("touchdown"),

        /** `Bridge.stop()` — the session ended. */
        STOPPED("stopped"),
    }

    private val lock = Any()

    private var phaseLocked = GuidedPhase.IDLE

    // The GCS stream.
    private var lastFrame: GcsStickFrame? = null
    private var lastFrameAtMs: Long? = null
    private var lastFrameSeq: Int? = null

    /**
     * When each *transport* controller was last heard from at all — not just its sticks.
     *
     * **A map rather than a field**: the watchdogs must ask "was *this* controller alive?",
     * never "was anyone?", because a live-but-idle second ground station keeping a dead
     * controller's manoeuvre flying would make the aircraft less safe the more interfaces are
     * attached (`docs/zenoh-dimos-transport.md` §4.3). The map is written by inbound traffic
     * and read only through [controllerSeenAtLocked], the origin-semantics owner — which never
     * consults it for [ControlOrigin.PHONE], whose liveness is identity rather than traffic
     * (the phone sends no inbound stream at all; landing08 is the flight where that difference
     * mattered). So in practice the map holds [ControlOrigin.MAVLINK] alone until a second
     * *transport* (`ZENOH`) exists.
     *
     * Guarded by [lock], like every other stream field here.
     */
    private val gcsSeenAtMs = mutableMapOf<ControlOrigin, Long>()

    /**
     * The controller whose command or deflection put the engine into its current engagement, and
     * therefore the one whose liveness the watchdogs read.
     *
     * Set wherever authority is taken or a manoeuvre is (re-)established, including when GCS
     * sticks take a reposition over — the sticks' sender owns the engagement from that moment.
     * Never read outside an engagement; its value while [GuidedPhase.IDLE] is meaningless and is
     * not cleared, which is why every read falls back to [ControlOrigin.MAVLINK] — the
     * fail-safe origin, whose liveness demands heartbeat evidence rather than being granted by
     * identity as [ControlOrigin.PHONE]'s is.
     */
    private var engagementOrigin: ControlOrigin? = null
    private var neutralSeenAtMs: Long? = null
    private var lastNonNeutralAtMs: Long? = null
    private var nonNeutralSinceMs: Long? = null

    // Engagement.
    private var engageStartedAtMs: Long? = null
    private var lastEngageAttemptAtMs: Long? = null

    // What DJI and the RC report.
    private var vs: VirtualStickSnapshot? = null
    private var rc: RcSticks? = null
    private var rcDeflectedSinceMs: Long? = null

    // A terminal sequence in progress.
    private var releaseReason: DisengageReason? = null
    private var releaseDetail: String? = null
    private var releasePlan: LinkLossPlan? = null
    private var releaseStartedAtMs: Long? = null
    private var rampFrom: StickVelocities = StickVelocities.ZERO

    /** When [GuidedPhase.ENGAGED] last began — the basis for [MODE_SEIZE_GRACE_MS]. */
    private var engagedAtMs: Long? = null

    // Send bookkeeping.
    private var lastCommanded: StickVelocities = StickVelocities.ZERO
    private var rangeRecorded = false

    /**
     * Stage B: the accepted target, alive from the `ACCEPTED` ack until arrival's hold ends,
     * the manoeuvre is cancelled, or the engagement dies. Mutable fields are touched only
     * under [lock].
     */
    private class RepositionState(
        val latDeg: Double,
        val lonDeg: Double,
        /** Metres above this bridge's own takeoff datum — already ceiling-capped at accept. */
        val relAltM: Double,
        val acceptedAtMs: Long,
        /**
         * How long this manoeuvre may take, from [acceptedAtMs] —
         * [GuidedEnvelope.manoeuvreDeadlineMs] read at the distance and the height change actually
         * commanded, computed **once, at accept** rather than per tick.
         *
         * Once, because the deadline must be a fact about the command rather than about how the
         * flight is going: recomputing it from the shrinking error every tick would extend it
         * forever, and recomputing it from a *growing* error (wind, a mistaken fix) would shorten
         * the deadline of exactly the manoeuvre that needed the time. The flat
         * [GuidedEnvelope.MANOEUVRE_TIMEOUT_MS] floor is inside the derivation, so a short goto
         * keeps the bound it had before 2026-07-30 to the millisecond.
         */
        val deadlineMs: Long,
    ) {
        var arrived = false
        var arrivedAtMs: Long? = null

        /** Consecutive ticks with both arrival conjuncts true. Reset to 0 by any false tick. */
        var arriveTicks = 0

        /** When the position feed first went stale mid-manoeuvre, or null while it is fresh. */
        var positionStaleSinceMs: Long? = null

        companion object {
            /**
             * **The arrival hold** — keep station here, already arrived: the state a paused goto, a
             * paused mission, a cancelled descent and [keepStationLocked] all leave behind, in one
             * place so those four cannot drift apart.
             *
             * Its [deadlineMs] is the flat [GuidedEnvelope.MANOEUVRE_TIMEOUT_MS] and is **never
             * read**, because the deadline rung is gated on `!arrived` and this state is born
             * arrived. It carries the floor rather than a derived number because there is no
             * commanded distance to derive one from — a hold's target *is* the current fix — and
             * because a zero passed into the owner would read as a measured distance of zero. What
             * bounds a hold is [GuidedEnvelope.IDLE_DISENGAGE_MS], from [arrivedAtMs].
             */
            fun holdAt(latDeg: Double, lonDeg: Double, relAltM: Double, nowMs: Long) =
                RepositionState(
                    latDeg, lonDeg, relAltM,
                    acceptedAtMs = nowMs,
                    deadlineMs = GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
                ).apply {
                    arrived = true
                    arrivedAtMs = nowMs
                }
        }
    }

    private var reposition: RepositionState? = null

    /** Which half of an orbit is flying. Two phases plus the two ways one ends. */
    private enum class OrbitPhase {
        /**
         * Flying to the nearest point on the circle as an **ordinary resting leg** — the M3 law
         * and the M3 arrival test, unchanged, toward a fixed lat/lon. Yaw is exactly zero here,
         * because this is not yet circling and the yaw exception is scoped to circling.
         */
        JOIN,

        /** From rest: the tangential ramp, the radial hold, the swept-angle counter, and the yaw. */
        CIRCLE,

        /** The circle ended; coming to rest at the completion point, again under the M3 law. */
        FINISH,

        /** At rest, keeping station at zero commanded velocity. Q1's idle window runs from here. */
        HOLD,
    }

    /**
     * Stage C: the accepted circle, alive from the `ACCEPTED` ack until the hold's idle window
     * ends, the orbit is cancelled or replaced, or the engagement dies. Mutable fields are touched
     * only under [lock].
     *
     * [requiredSweepDeg] is `+∞` for `param4 = 0` ("orbit forever"), which is then bounded by
     * [OrbitGuidance.ORBIT_MAX_S] alone and announced as such.
     */
    private class OrbitState(
        val centreLatDeg: Double,
        val centreLonDeg: Double,
        /** Metres above this bridge's own takeoff datum — already ceiling-capped at accept. */
        val relAltM: Double,
        /** Always positive; the sign of `param1` lives in [direction]. */
        val radiusM: Double,
        /** `sign(param1)`: +1 clockwise seen from above, −1 counter-clockwise. */
        val direction: Int,
        val requiredSweepDeg: Double,
        val acceptedAtMs: Long,
        joinLatDeg: Double,
        joinLonDeg: Double,
        /**
         * How long the **join leg** may take, from [joinStartedAtMs] — the ordinary derived
         * manoeuvre deadline ([GuidedEnvelope.manoeuvreDeadlineMs]) read at the join leg's own
         * length, computed once at accept for the reasons `RepositionState.deadlineMs` gives.
         *
         * The circle itself is bounded by [OrbitGuidance.ORBIT_MAX_S] instead, and the FINISH leg by
         * nothing but that: those are unchanged by 2026-07-30, because a circle's duration is not a
         * distance problem and `R_MAX_M` did not move.
         */
        val joinDeadlineMs: Long,
    ) {
        var phase = OrbitPhase.JOIN

        /** The resting leg's target — the join point, then the completion point. */
        var legLatDeg = joinLatDeg
        var legLonDeg = joinLonDeg

        /** Consecutive ticks with the M3 arrival test true. Reset to 0 by any false tick. */
        var arriveTicks = 0

        /** When the join leg began, for the ordinary-leg manoeuvre timeout. */
        var joinStartedAtMs: Long? = null

        /** When the position feed first went stale mid-manoeuvre, or null while it is fresh. */
        var positionStaleSinceMs: Long? = null

        /** The ramped tangential speed, m/s. Starts at rest and is never stepped. */
        var tangentialMs = 0.0

        /** The previous circling tick, for the ramp's `dt`. */
        var lastCircleTickAtMs: Long? = null

        /** Degrees swept since the circle began. Reset to zero whenever the circle (re-)begins. */
        var sweptDeg = 0.0

        /** The previous bearing from the centre, or null before the first circling tick. */
        var lastBearingDeg: Double? = null

        /** When [OrbitPhase.HOLD] began — the basis for Q1's idle disengage. */
        var holdingSinceMs: Long? = null
    }

    private var orbit: OrbitState? = null

    /**
     * M4: the route being flown, alive from an accepted Start until the last item completes, a row
     * of §6.2 fires, or the engagement dies. Mutable fields are touched only under [lock].
     *
     * The route is **immutable and held by value**, which is what makes M4-12 free: an upload
     * arriving mid-flight replaces the *store*, and this run carries on with the plan it began with.
     */
    private class MissionRun(
        val route: MissionRoute,
        val sink: MissionRunSink,
        val startedAtMs: Long,
        /**
         * The door this run came in through, carried for one reason: the tag descent a
         * [MissionStepKind.PRECISION_LAND] item arms is armed **as this controller's manoeuvre**, so
         * the descent's inherited Q4 link watchdog judges liveness on the same origin's semantics the
         * mission has been judged on all along (a MAVLINK mission needs QGC's traffic; a PHONE-origin
         * one is alive by identity — landing08's fix, and the `flyTakeoffClimb` pattern of carrying the
         * origin on the intention rather than re-deciding it at the far end).
         */
        val origin: ControlOrigin,
        /** Index into [MissionRoute.steps] — **not** a wire `seq`; a plan holds non-navigable items. */
        var index: Int,
        /**
         * True until the first step of a **resumed** run completes.
         *
         * §6.3: a resume is not "continue", it is *"fly a new leg to the cursor, then continue"*,
         * and that leg was never drawn on anybody's map. So it gets the **conservative** completion
         * test — the aircraft comes to rest at the cursor under M3's arrival predicate — rather than
         * the fast one, whatever the step itself says.
         */
        var rejoining: Boolean,
    ) {
        /** When the current leg began, for [MissionGuidance.legTimeoutMs]. */
        var legStartedAtMs: Long? = null

        /**
         * `P` — where the current leg began, for the half-plane term. The position at which the
         * previous step completed, or the position at Start for the first leg. Deliberately the
         * *aircraft's* position rather than the previous waypoint's: the plane the half-plane test
         * uses is the one perpendicular to the leg actually being flown.
         */
        var legOriginLatDeg: Double? = null
        var legOriginLonDeg: Double? = null

        /** Consecutive ticks with the M3 arrival test true. Reset to 0 by any false tick. */
        var arriveTicks = 0

        /** When the position feed first went stale mid-mission, or null while it is fresh. */
        var positionStaleSinceMs: Long? = null

        /** Set once DJI's own takeoff has been asked for, so it is asked for exactly once. */
        var takeoffAsked = false

        /**
         * **The takeoff item's handback detector — the same machine the operator's Takeoff button
         * uses**, armed as a bare watch ([TakeoffClimb.armWatch]) because a mission's height lives
         * in its step.
         *
         * One instance *per run* rather than the engine's own, so that a live `MAV_CMD_NAV_TAKEOFF`
         * and a mission's takeoff item can never be mistaken for one another: each holds its own
         * pending intention, each is cancelled by the thing that owns it, and this one dies with
         * the run it belongs to.
         */
        val takeoff = TakeoffClimb()

        /**
         * Phase two: true once DJI has handed back and the vertical-only leg to the takeoff item's
         * height is being flown. Set under the engine's lock on the tick the handback is observed —
         * which is what makes the observation impossible to lose, since [TakeoffClimb.observe]
         * reports it exactly once.
         */
        var climbing = false

        /**
         * The altitude DJI's own hop actually left the aircraft at, or null if it was unknown at
         * the handback — the basis for the climb's leg timeout, so that a 100 m climb is not held
         * to the same deadline as a 5 m one.
         *
         * Measured rather than assumed: `docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md`
         * read 1.0–1.1 m off a completed DJI takeoff, against the ~1.2 m every document had
         * inherited from DJI's own.
         */
        var climbOriginAltM: Double? = null

        /** The last item completed: the aircraft is holding, in the air, waiting for a human. */
        var finished = false

        /** When [finished] began — the basis for Q1's idle disengage. */
        var holdingSinceMs: Long? = null

        /**
         * The tag-landing sequence, alive from the tick [PrecisionLand.gate] cleared at a
         * [MissionStepKind.PRECISION_LAND] item until the arm ends the run (or a refusal does).
         *
         * Null while the sequence has not begun, which is what makes "the gates are measured at the
         * moment the item begins, exactly once" structural: the first tick on that step builds this,
         * every later tick reads it, and there is no path on which the gates are re-run with the
         * aircraft already committed to flying the sequence.
         */
        var land: LandTagRun? = null

        /**
         * **The plan's ROI as this run last applied it** — the value most recently pushed through the
         * engine's one ROI door, or null when the run has pushed a clear (or has pushed nothing).
         *
         * It exists so the ROI is applied **on change** rather than on every cursor move. Re-taking an
         * unchanged target at each corner would reset the camera's rate limiter and re-announce
         * `ROI accepted` on every leg of a five-leg mission — noise on the one channel that is 50 bytes
         * at severity ERROR.
         *
         * Deliberately *not* a read of the engine's live [RoiState]: they differ exactly when the
         * operator has clicked their own ROI mid-mission, and that difference is the behaviour
         * `docs/m4-mission-transport.md` §6.3 asks for — **the live hand outranks the plan for as long
         * as the plan says nothing new.** The run remembers what it asked for; the next plan item that
         * asks for something *different* speaks again, and one that repeats itself stays quiet.
         */
        var roiApplied: RoiCommand? = null

        val step: MissionStep get() = route[index]

        /** The wire `seq` the cursor points at — what `MISSION_CURRENT` carries. */
        fun cursorSeq(): Int = route[index.coerceIn(0, route.size - 1)].seq
    }

    private var mission: MissionRun? = null

    /**
     * **The tag-landing sequence a precision `NAV_LAND` item flies**, one per item and dropped with
     * the run it belongs to. Mutable fields are touched only under [lock].
     *
     * Everything that decides *where* and *how low* is frozen at construction, on the same principle
     * [TagDescentRun] pins its own tag id and its full-autoland option at the arm: the gates were
     * measured once, against the world at the moment the item began, and a target that could be
     * recomputed later is a target that can move under an aircraft already descending toward it. In
     * particular [takeoffLatDeg]/[takeoffLonDeg] are the recorded takeoff point **as it stood at the
     * item's begin**, not a live read of DJI's home key — its ~1 m of session wander (landing09) must
     * not walk the target while the aircraft flies to it.
     */
    private class LandTagRun(
        /** [PrecisionLandMode] as authored — on the record, so 1 and 2 are distinguishable there. */
        val mode: Int,
        /** The recorded takeoff point: the XY of every phase. See the class KDoc for why it is frozen. */
        val takeoffLatDeg: Double,
        val takeoffLonDeg: Double,
        /** The item's own altitude — [PrecisionLand.Phase.TRANSIT]'s height. */
        val transitRelAltM: Double,
        val startedAtMs: Long,
    ) {
        var phase = PrecisionLand.Phase.TRANSIT

        var phaseStartedAtMs: Long = startedAtMs

        /**
         * The bound on the phase now flying, milliseconds — a leg timeout scaled to the leg for the
         * two flying phases, [PrecisionLand.NADIR_AIM_LIMIT_MS] for the aim. Written at every phase
         * entry so a phase can never inherit its predecessor's deadline.
         */
        var phaseLimitMs: Long = MissionGuidance.legTimeoutMs(0.0)

        /**
         * `min(`[PrecisionLand.LAND_TAG_ARM_HEIGHT_M]`, the altitude when [PrecisionLand.Phase.LOWER]
         * began)` — the never-climb clamp, frozen. Null until that phase begins.
         */
        var armHeightM: Double? = null

        /** The altitude the clamp above was taken from, for the record's own account of the decision. */
        var armHeightFromM: Double? = null

        /**
         * `min(`[PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M]`, `[armHeightM]`)` — the bottom of the
         * acquisition band, frozen when [PrecisionLand.Phase.ACQUIRE] begins. Null until then.
         */
        var acquireFloorM: Double? = null

        /**
         * The **last** thing the arm gate said while the acquisition descent was running, so the
         * refusal at the floor can name the conjunct that was still failing instead of saying
         * "no tag" about a camera problem. Null until the first poll.
         */
        var lastArmBlock: Pair<String, String>? = null

        /** True once the camera has been commanded to nadir, so it is commanded exactly once. */
        var nadirAsked = false
    }

    /**
     * M3 Stage D: an armed tag-tracked descent, alive from [armTagDescent]'s take until it
     * completes-and-idles, is disarmed, is cancelled by any of its named paths, or the
     * engagement dies. Mutable fields are touched only under [lock].
     *
     * The sighting-side state machine — the staleness ladder, the alignment cone, the terminal
     * latch — is [TagDescent], **one instance per arm**, built here and dropped with this
     * object. That construction is what makes rule 1's second half structural: nothing can
     * resume a dead descent, because the machine that was descending no longer exists and a
     * fresh arm builds a fresh one at [TagDescentPhase.TRACKING].
     */
    private class TagDescentRun(
        /** The latched tag's id, pinned at arm: a fix decoding any other id is not evidence. */
        val tagId: Int,
        val acceptedAtMs: Long,
        /** The newest believed fix, metres from home — [TagDescentSense]'s frame. */
        var fixNorthM: Double,
        var fixEastM: Double,
        /** When that fix was made, on the engine's clock (now − reported age at ingest). */
        var fixAtMs: Long,
        /**
         * The newest believed fix's tag-derived range and its provenance — ingested beside
         * [fixNorthM] on the same id-matched monotonic terms, so all the run's fix facts
         * always describe one frame. This is the number that scaled that fix's lateral, and
         * therefore the number the height law must fly when fresh
         * ([TagDescentGuidance.descentHeight] — landing07's consistency requirement).
         */
        var fixTagRangeM: Double? = null,
        var fixRangeSource: RangeSource? = null,
        /**
         * Stage C: whether this arm carried the operator's explicit full-autoland option.
         * Pinned at arm and handed to the machine — the toggle read once, at the operator's
         * act, never re-read mid-flight where a flicked switch could commit a descent nobody
         * armed for landing.
         */
        val fullAutoland: Boolean = false,
        /**
         * Whether the arm was taken above [TagDescentGuidance.ARM_CEILING_M] — the machine
         * starts in [TagDescentPhase.APPROACH] and flies down into the band before the
         * ordinary ladder begins. Pinned at arm exactly as the option flag is: the gate
         * measured the height once, at the operator's act, and a run built without it is
         * byte-identical to yesterday's.
         */
        val approach: Boolean = false,
    ) {
        val machine = TagDescent(fullAutoland, approach)

        /** When [TagDescentPhase.TERMINAL] began — the basis for Q1's idle disengage. */
        var terminalAtMs: Long? = null

        /** When the position feed first went stale mid-descent, or null while it is fresh. */
        var positionStaleSinceMs: Long? = null

        /**
         * Whether the last **fresh** fix sat inside the cone at a known height — evaluated on
         * every steering tick, the newest statement the sensor made about "over the tag", and
         * the cone half of the auto-confirm gate ([autolandClearance]).
         */
        var lastFreshFixInCone = false

        /** DJI accepted our guided landing confirm — [noteAutolandConfirmed]. */
        var djiConfirmed = false

        /** When the machine committed to [TagDescentPhase.DJI_LANDING], or null before it. */
        var committedAtMs: Long? = null

        /**
         * The newest believed fixes, appended by [ingestDescentFixLocked] on the same
         * id-matched monotonic terms as [fixNorthM] and pruned to
         * [TagDescentGuidance.LAND_TARGET_WINDOW_MS] behind the newest — the
         * [TagDescentGuidance.landingTarget] window's feed, so the blind-final steering
         * target is a robust average of one height's worth of samples rather than one frame's
         * jitter. **Frozen at the commit edge** — the committed tick does not ingest, because
         * post-commit fixes ride a camera DJI is already recentering while the pitch belief
         * still says nadir (measured entering the window on landing07, both landings — the
         * committed branch carries the numbers). **The live tracking loop never reads this**
         * (it steers on [fixNorthM] — newest, unaveraged — deliberately: see
         * [TagDescentGuidance.landingTarget]).
         */
        val recentFixes = ArrayList<TagDescentGuidance.FixSample>()

        /**
         * Whether the committed landing's steering is currently dead (position stale/absent or
         * no believed target) — the edge detector for the record's `landing steering` lines,
         * so a reader can tell steered-blind ticks from dead-stick ticks without inferring
         * from zeros, and the record is not written at 10 Hz.
         */
        var landSteerDead = false

        /**
         * The previous tick's commanded descent rate, down-positive — the floor detector's
         * "descent was being commanded" fact, read one tick in arrears because a stall is a
         * claim about a command that already went out.
         */
        var lastDownCmd = 0.0

        // The landing gimbal watchdog's own bookkeeping (LANDING only): last re-command time,
        // attempts spent, and whether the give-up line has been said.
        var gimbalNudgeAtMs: Long? = null
        var gimbalNudges = 0
        var gimbalGaveUp = false

        // The landing stall detector (LANDING only): the reference altitude progress is
        // measured against, when it was set, and whether this stall episode is recorded.
        var stallRefAltitudeM: Double? = null
        var stallRefAtMs: Long? = null
        var stallRecorded = false

        /**
         * Which rung the descent's height law flew on last tick
         * ([TagDescentGuidance.descentHeight]'s verdict), or null before the first tick / when
         * no instrument could vouch — the edge detector for the record's `height_source`
         * lines, so a reader can see which instrument the descent flew on and when it
         * switched, with both instruments' numbers at every switch, without the record being
         * written at 10 Hz.
         */
        var heightSource: RangeSource? = null

        /**
         * Whether the run is inside a tag/baro divergence episode
         * ([TagDescentGuidance.rangeDivergence] non-null last tick) — the edge detector for
         * the record's `range_baro_divergence` lines: one line per episode, with the first
         * diverging tick's numbers, not one per 100 ms tick. A measurement, never a gate —
         * the ladder already chose the tag.
         */
        var rangeDiverged = false
    }

    private var tagDescent: TagDescentRun? = null

    /**
     * **Shadow mode** — the validation gate before Stage D's first real engagement. While
     * enabled, the whole controller runs — the same [descentGateLocked] gates, the same
     * [TagDescent] machine, the same ladder — and actuates **nothing**: its would-be commands
     * go only to the flight record (`stick_cmd.src = TAG_DESCENT_SHADOW`, `accepted = null`),
     * so the operator can hand-fly a tag landing while the controller shadows, and the two
     * timelines can be diffed afterwards.
     *
     * Three deliberate differences from live, all in this wrapper and none in the law:
     *
     *  1. **Stick input never cancels a shadow** — the operator is flying, so a deflection is
     *     the flight, not a takeover; the would-cancel edge is *recorded* (with the deflection
     *     magnitude that crossed the dead-band) and the shadow keeps running.
     *  2. **Shadow re-arms automatically** whenever the arm gates hold again, so one manual
     *     landing produces a continuous timeline rather than one truncated run.
     *  3. Segment-ending facts (latch lost, nadir left, tag gone past the abort bound, link
     *     loss, position loss, timeouts) end the **segment**, recorded with the same words a
     *     live run would use, and never release anything — there is nothing to release.
     *
     * The law computes identical transitions in both modes — the machine is the same class fed
     * the same inputs — and `GuidedTagDescentTest` asserts the transition-sequence identity
     * directly, which is what makes shadow evidence transferable to live.
     */
    private class ShadowRun {
        /** The current shadow segment, or null while the gates block one. */
        var run: TagDescentRun? = null

        /** The last recorded blocker, so blocked-edges are recorded on change, not at 10 Hz. */
        var lastBlocker: String? = null

        /** True while the RC-deflection verdict holds — the would-cancel edge detector. */
        var rcWouldCancel = false

        /** Whether this segment's terminal sentence has been said. */
        var saidComplete = false

        /** The newest would-be command and when it was computed — the comparison view's feed. */
        var lastCmd: StickVelocities? = null
        var lastCmdAtMs: Long? = null
    }

    private var shadowDescent: ShadowRun? = null

    /**
     * When a setpoint last went out **for a mission**, or null since the last run ended.
     *
     * Conjunct 3 of [missionFlying], and the reason the heartbeat's `AUTO_MISSION` claim is a report
     * rather than an echo: the mode appears only *after* the aircraft has actually been commanded,
     * never because a `SET_MODE` arrived. Never latched — it is compared against the clock on every
     * read, so a loop that stops sending stops claiming.
     */
    private var missionSetpointAtMs: Long? = null

    /**
     * The camera's own rate limiter and deadband — **engine-level, because there is exactly one
     * camera and exactly one thing it may be pointed at at a time.**
     *
     * These were per-orbit while an orbit was the only thing that aimed. An ROI outlives any one
     * manoeuvre (it survives a goto ending, and it is tracked while the RC pilot flies), so the
     * limiter cannot live inside a manoeuvre's state without either being lost or being duplicated.
     * Reset whenever a new thing to point at is taken, so the first command of a new target is never
     * swallowed by the previous target's deadband.
     *
     * Neither field is ever a gimbal *reading*: they are what **we** last asked for and when. See
     * [ManoeuvreGimbal] for why nothing here may be a measurement.
     */
    private var gimbalLastAtMs: Long? = null
    private var gimbalLastPitchDeg: Double? = null

    /**
     * The region of interest: **a modifier, not a manoeuvre.**
     *
     * It changes where the camera looks, and — only while our own executor is flying — where the
     * nose points. It changes the flight path not at all: a goto with an ROI flies exactly the goto,
     * and an orbit with an explicit ROI flies exactly the circle. That is why this is a field beside
     * [reposition] and [orbit] rather than a third arm of the tick's branch, and why setting one
     * neither engages nor requires an engagement.
     *
     * Alive from the `ACCEPTED` ack until `DO_SET_ROI_NONE`, a replacement, or `stop()`. **An abort
     * does not clear it** — it [suspends][RoiState.suspended] the driving of the camera, because the
     * target is still the place the operator asked about and a resumed manoeuvre should re-acquire
     * it, while a camera slewing during an abort is noise at the moment the operator least needs it
     * (`docs/m4-mission-execution.md` §9.5).
     */
    private class RoiState(
        val latDeg: Double,
        val lonDeg: Double,
        val acceptedAtMs: Long,
        /**
         * The target's own height, **metres above our takeoff datum**, or null when the command's
         * frame carried none this bridge can read ([RoiCommand.relativeAltMOrNull] is the whole
         * decision, and the ground-level assumption is what null means).
         *
         * Null is therefore *not* the same as 0.0 even though the arithmetic coincides: 0.0 is a
         * target the operator placed at takeoff level, null is a target nobody gave a height for, and
         * only the second announces [GuidedStatusTexts.ROI_GROUND_LEVEL].
         */
        val relAltM: Double? = null,
        /**
         * True for the ROI an **orbit implies at its own centre** (M4-6: *"orbit implies ROI at the
         * circle's centre, at takeoff height"*), false for one the operator asked for by name.
         *
         * The distinction is precedence, and only precedence: an explicit ROI outranks an implied
         * one, and `DO_SET_ROI_NONE` clears the explicit one and lets the implied one back rather
         * than leaving an orbiting aircraft pointing nowhere.
         */
        val implied: Boolean = false,
        /**
         * The **wire `seq` of the plan item that set this**, or null when it came from a live command
         * (QGC's Fly-view click, the phone) or from an orbit's centre.
         *
         * Provenance on the one ROI state, not a second state — and it is load-bearing in exactly one
         * decision: **what a mission's ending does to the camera.** A plan's ROI dies with the plan
         * that asked for it (`docs/m4-mission-transport.md` §6.3: *"on mission end, abort, or handback,
         * the ROI is cleared — but the gimbal is not slewed back"*), while an operator's own click
         * outlives every manoeuvre and must not be taken away by a mission finishing
         * (`abort`'s ROI paragraph, unchanged since 2026-07-27). Without this field those two cases are
         * indistinguishable and one of them has to be got wrong.
         *
         * Also on the flight record: `roi_accepted`/`roi_cleared` carry `seq=N` when it is set, which is
         * how a post-flight reader ties the aiming to the plan item that asked for it.
         */
        val missionSeq: Int? = null,
    ) {
        /**
         * Set by any abort, cleared when an engagement is next confirmed or a fresh ROI arrives.
         * While true the camera is **not driven at all** — no recentring, no stowing, no tracking.
         */
        var suspended = false

        // Each sentence is said when its condition begins and re-armed when the condition clears,
        // so an ROI that spends a minute out of gimbal travel says so once rather than twelve times,
        // and an ROI that leaves and re-enters the condition says so again — which is news.
        var saidPitchOnly = false
        var saidTooClose = false
        var saidGimbalRange = false
    }

    private var roi: RoiState? = null

    /**
     * **The ROI an orbit implies at its own centre**, or null when no orbit has been accepted this
     * engagement. Read only through [roiTrackingLocked], which gates it on an orbit actually
     * running — so it needs no clearing at the eleven places an orbit can end, and cannot outlive
     * the manoeuvre that created it.
     *
     * It exists as a *state object* rather than being derived per tick because [RoiState] carries
     * the once-per-condition announcement flags: a fresh instance every tick would say
     * "ROI: out of gimbal range" ten times a second.
     */
    private var orbitRoi: RoiState? = null

    /**
     * The takeoff's pending second phase. Touched only under [lock] — which is also the lock every
     * abort takes, so an abort and a firing tick cannot interleave, and a climb can never both be
     * cancelled and fired.
     */
    private val takeoffClimb = TakeoffClimb()

    // Announcement de-duplication.
    private var lastAnnouncement: String? = null
    private var lastAnnouncedAtMs: Long? = null

    // Logcat de-duplication for the 25 Hz refusal gates (see LOG_REPEAT_MS).
    private var lastThrottledLog: String? = null
    private var lastThrottledLogAtMs: Long? = null

    private var ticker: ScheduledExecutorService? = null

    val phase: GuidedPhase get() = synchronized(lock) { phaseLocked }

    /**
     * A flat immutable copy of what is being flown, for anything that **draws** rather than
     * commands — see [GuidedSituation].
     *
     * One synchronized read producing plain data. It is `@Synchronized`-equivalent and cheap,
     * so a 4 Hz UI can call it without contending with the 10 Hz control tick; it holds the lock
     * for the duration of a handful of field reads and no allocation-free path is worth the
     * risk of an unlocked read of a mid-update `OrbitState`.
     *
     * **This is a one-way door on purpose.** Nothing returned here can be handed back in: there
     * is no method on [GuidedSituation] that reaches this engine, so a caller that draws a
     * circle cannot become a caller that changes one.
     */
    fun situation(): GuidedSituation = synchronized(lock) {
        val repo = reposition
        val orb = orbit
        val dsc = tagDescent
        val shadowRun = shadowDescent?.run
        val tracked = roiTrackingLocked()
        val remembered = roi
        GuidedSituation(
            descent = dsc?.let {
                val landing = it.machine.phase == TagDescentPhase.DJI_LANDING
                GuidedDescent(
                    it.tagId,
                    terminal = it.machine.phase == TagDescentPhase.TERMINAL,
                    landing = landing,
                    blind = landing && nowMs() - it.fixAtMs > TagDescentGuidance.LAND_FRESH_MS,
                    approach = it.machine.phase == TagDescentPhase.APPROACH,
                )
            } ?: shadowRun?.let {
                GuidedDescent(
                    it.tagId,
                    terminal = it.machine.phase == TagDescentPhase.TERMINAL,
                    shadow = true,
                    approach = it.machine.phase == TagDescentPhase.APPROACH,
                )
            },
            phase = phaseLocked,
            goto = repo?.let {
                GuidedGoto(it.latDeg, it.lonDeg, it.relAltM, it.arrived)
            },
            orbit = orb?.let {
                GuidedOrbit(
                    centreLatDeg = it.centreLatDeg,
                    centreLonDeg = it.centreLonDeg,
                    radiusM = it.radiusM,
                    direction = it.direction,
                    relAltM = it.relAltM,
                    circling = it.phase == OrbitPhase.CIRCLE,
                )
            },
            roi = remembered?.let {
                GuidedRoi(it.latDeg, it.lonDeg, tracking = tracked != null)
            },
        )
    }

    // ------------------------------------------------------------------ lifecycle

    /**
     * Subscribes the port's listeners. Split from [start] so tests can attach the observation
     * paths and then drive [tick] by hand with a fake clock — the ticker thread is cadence,
     * not correctness.
     */
    fun attach() {
        port.listenState(::onVsState, ::onAuthorityReason)
        port.listenRcSticks(::onRcSticks)
    }

    /**
     * Pass-through to [VirtualStickPort.ensureRcFeed]; `Bridge.tick` (200 ms) calls it so a
     * subscription deferred behind MSDK registration gets planted as soon as it can be. No
     * decision lives here — the precondition and the idempotence are the port's.
     */
    fun ensureRcFeed() = port.ensureRcFeed()

    /** [attach] plus the 10 Hz tick thread. `Bridge.start` only. */
    fun start() {
        attach()
        ticker = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "guided-tick").apply { isDaemon = true }
        }.also { exec ->
            exec.scheduleAtFixedRate({
                try {
                    tick()
                } catch (t: Throwable) {
                    // The loop must never die silently: a dead guided tick while engaged is an
                    // unrefreshed setpoint stream with no watchdog. Log and keep ticking; the
                    // per-tick guards fail closed on whatever broke.
                    log("guided tick failed: $t")
                }
            }, TICK_MS, TICK_MS, TimeUnit.MILLISECONDS)
        }
        log("guided stick engine up — link-loss policy=${policy.name}")
    }

    /**
     * `Bridge.stop()`. Aborts any engagement (zero setpoint, disable, announce `stopped`),
     * cancels the DJI subscriptions and stops the tick thread. Safe to call twice.
     */
    fun stop() {
        abort(DisengageReason.STOPPED)
        ticker?.shutdownNow()
        ticker = null
        port.cancelListens()
    }

    // ------------------------------------------------------------------- inbound

    /**
     * Every inbound MAVLink payload, from `Bridge.onInbound` — not only `MANUAL_CONTROL`,
     * because *any* traffic is evidence the GCS is alive, and that evidence is what separates
     * `released` from `link-lost` when the sticks go quiet.
     *
     * [origin] says *whose* liveness has just been evidenced. It defaults to
     * [ControlOrigin.MAVLINK] because that is the only sender that *has* inbound traffic —
     * [ControlOrigin.PHONE] is this very process and never arrives on a wire, so a PHONE-keyed
     * stamp would be an entry nothing reads ([controllerSeenAtLocked] answers for PHONE by
     * identity, never from the map) — and `Bridge`, the class whose documented job is routing
     * inbound, is the only caller.
     */
    fun onInbound(
        payload: Any?,
        sequence: Int? = null,
        origin: ControlOrigin = ControlOrigin.MAVLINK,
    ) {
        val now = nowMs()
        synchronized(lock) { gcsSeenAtMs[origin] = now }
        val mc = payload as? ManualControl ?: return
        if (mc.target() != 0 && mc.target() != SYSTEM_ID) return
        when (val reading = StickMapping.read(mc.x(), mc.y(), mc.z(), mc.r())) {
            is StickMapping.Reading.Unreadable -> {
                // Never guessed at (landmine 2): a convention this bridge has not measured is
                // refused with a sentence, and — while engaged — the frame is simply not
                // stored, so a persistent stream of garbage ramps to zero on the watchdog.
                logThrottled("MANUAL_CONTROL refused: ${reading.reason}")
                announce(GuidedStatusTexts.BAD_AXES)
            }

            is StickMapping.Reading.Valid -> onFrame(reading.frame, sequence, now, origin)
        }
    }

    private fun onFrame(
        frame: GcsStickFrame,
        sequence: Int?,
        now: Long,
        origin: ControlOrigin,
    ) {
        var announceText: String? = null
        var engage = false
        var gotoInterrupted = false
        var orbitInterrupted = false
        var descentInterrupted = false
        var descentCommittedDropped = false
        var descentConfirmedDropped = false
        var gotoUninterruptible = false
        var climbInterrupted = false
        var missionInterrupted: Triple<MissionRunSink, MissionPauseCause, Int>? = null
        synchronized(lock) {
            val neutralAtBefore = neutralSeenAtMs
            lastFrame = frame
            lastFrameAtMs = now
            lastFrameSeq = sequence
            if (StickMapping.isNeutral(frame)) {
                neutralSeenAtMs = now
                nonNeutralSinceMs = null
            } else {
                lastNonNeutralAtMs = now
                if (nonNeutralSinceMs == null) nonNeutralSinceMs = now
            }
            if (phaseLocked == GuidedPhase.IDLE && StickMapping.isDeliberate(frame)) {
                val unavailable = port.unavailableReason()
                val neutralAt = neutralSeenAtMs
                val retryAt = lastEngageAttemptAtMs
                when {
                    !interlockEnabled() -> announceText = GuidedStatusTexts.IGNORED_INTERLOCK
                    unavailable != null -> announceText = GuidedStatusTexts.ignored(unavailable)
                    neutralAt == null || now - neutralAt > NEUTRAL_RECENT_MS ->
                        announceText = GuidedStatusTexts.CENTER_FIRST
                    rc?.allPresent() != true -> announceText = GuidedStatusTexts.NO_RC_FEED
                    retryAt != null && now - retryAt < ENGAGE_RETRY_MS ->
                        logThrottled("deflection inside the engage retry window — not re-asking DJI")
                    else -> {
                        phaseLocked = GuidedPhase.ENGAGING
                        engageStartedAtMs = now
                        lastEngageAttemptAtMs = now
                        // This controller's deflection took the authority, so this controller's
                        // silence is what ends it.
                        engagementOrigin = origin
                        engage = true
                    }
                }
            }
            // Sticks outrank the controller. A *deliberate* deflection during a reposition
            // cancels the target and degrades to Stage A passthrough — the operator chose the
            // GCS-stick channel, so they get GCS-stick control, not an authority handover to an
            // RC across the field (that gesture exists too, and aborts: gesture 1).
            //
            // Gated on the stream having recently been seen at rest, exactly like engagement
            // (gate 3), and for the same landmine: a centre-zero stream's *idle* frame reads
            // deliberate (z = 0 is 500 off centre), so without this gate merely enabling that
            // regime would cancel every reposition and command a full-scale descent.
            if ((reposition != null || orbit != null || mission != null || tagDescent != null) &&
                (phaseLocked == GuidedPhase.ENGAGED || phaseLocked == GuidedPhase.ENGAGING) &&
                StickMapping.isDeliberate(frame)
            ) {
                if (neutralAtBefore != null && now - neutralAtBefore <= NEUTRAL_RECENT_MS) {
                    orbitInterrupted = orbit != null
                    gotoInterrupted = reposition != null
                    // Rule 1's GCS-channel half: a deliberate ground-station deflection kills the
                    // descent as completely as the RC gesture does — the state is dropped here,
                    // so nothing exists to resume, and a fresh arm is the only way back. This
                    // includes a committed LANDING and its blind final: rule 1 is absolute.
                    descentInterrupted = tagDescent != null
                    descentCommittedDropped =
                        tagDescent?.let { it.machine.phase == TagDescentPhase.DJI_LANDING } == true
                    descentConfirmedDropped = tagDescent?.let {
                        it.machine.phase == TagDescentPhase.DJI_LANDING && it.djiConfirmed
                    } == true
                    tagDescent = null
                    // §6.2 row 3: the operator chose the GCS-stick channel, so they get GCS-stick
                    // control. The plan waits — paused, resumably, at the item it was flying.
                    mission?.let { run ->
                        missionInterrupted = Triple(
                            run.sink, MissionPauseCause.GCS_STICK_DEFLECTION, run.cursorSeq(),
                        )
                    }
                    mission = null
                    missionSetpointAtMs = null
                    reposition = null
                    orbit = null
                    // The sticks took the manoeuvre over, so their sender now owns the
                    // engagement and it is their silence the watchdogs must watch for.
                    engagementOrigin = origin
                } else {
                    gotoUninterruptible = true
                }
            }
            // The same gesture applied to the takeoff's pending second phase, which is armed while
            // this engine is IDLE and so is not covered by the block above. Same gate — the stream
            // must have been seen at rest recently — for the same landmine: a centre-zero stream's
            // *idle* frame reads deliberate, and without the gate merely enabling that regime would
            // silently cancel the climb of every takeoff.
            if (takeoffClimb.armed &&
                StickMapping.isDeliberate(frame) &&
                neutralAtBefore != null &&
                now - neutralAtBefore <= NEUTRAL_RECENT_MS
            ) {
                takeoffClimb.cancel()
                climbInterrupted = true
            }
        }
        announceText?.let {
            logThrottled("deliberate deflection not engaging: $it")
            announce(it)
        }
        if (gotoInterrupted) {
            log("reposition cancelled by deliberate GCS deflection — passthrough has the aircraft")
            record.event(EventCode.GOTO_ENDED, "sticks")
            announce(GuidedStatusTexts.GOTO_STICKS)
        }
        if (orbitInterrupted) {
            log("orbit cancelled by deliberate GCS deflection — passthrough has the aircraft")
            record.event(EventCode.ORBIT_ENDED, "sticks")
            announce(GuidedStatusTexts.ORBIT_STICKS)
        }
        if (descentInterrupted) {
            log("tag descent cancelled by deliberate GCS deflection — passthrough has the aircraft")
            record.event(
                EventCode.TAG_DESCENT_ENDED,
                "sticks" + if (descentConfirmedDropped) " dji-confirmed" else "",
                warn = true,
            )
            // Rule 1's GCS-channel half of the same action: manual sticks during a committed
            // DJI landing withdraw the landing too, whichever channel the hand chose.
            if (descentCommittedDropped) stopDjiLanding("gcs-sticks")
            announce(GuidedStatusTexts.DESCENT_STICKS)
        }
        if (climbInterrupted) {
            log("takeoff climb cancelled by deliberate GCS deflection")
            announceClimbCancelled(DisengageReason.RC_STICKS.wire)
        }
        missionInterrupted?.let { (sink, cause, seq) ->
            log("mission paused by deliberate GCS deflection at item $seq — passthrough has the aircraft")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause", warn = true)
            announce(GuidedStatusTexts.MISSION_STICKS)
            sink.onPaused(cause, seq)
        }
        if (gotoUninterruptible) {
            // The deflection cannot be read as intent (stream never seen at rest — the
            // centre-zero regime, or a first-ever frame already deflected). The reposition
            // continues; the sentence tells the operator why their sticks are dead.
            logThrottled("deliberate deflection ignored during reposition: stream not seen at rest")
            announce(GuidedStatusTexts.CENTER_FIRST)
        }
        if (engage) beginEngage()
    }

    /**
     * Ask DJI for virtual stick. Nothing about the engagement rests on these callbacks firing
     * — [GuidedPhase.ENGAGED] comes from the state listener alone, and the [ENGAGE_CONFIRM_MS]
     * deadline covers a swallowed call. The advanced-mode setter rides `onSuccess` because
     * DJI's own contract is enable-then-advanced, and it is a void setter whose only
     * observable effect is the same state listener.
     */
    private fun beginEngage() {
        log("engaging virtual stick — policy=${policy.name}")
        record.event(EventCode.VS_ENABLE_REQUEST, "policy=${policy.name}")
        port.enable(
            onSuccess = {
                record.event(EventCode.VS_ENABLE_RESULT, "ok")
                log("enableVirtualStick accepted — setting advanced mode, awaiting DJI's state")
                port.setAdvancedMode(true)
            },
            onFailure = { djiError ->
                record.event(EventCode.VS_ENABLE_RESULT, djiError, warn = true)
                var pendingGotoDropped = false
                var pendingOrbitDropped = false
                var pendingDescentDropped = false
                val cancelled = synchronized(lock) {
                    if (phaseLocked == GuidedPhase.ENGAGING) {
                        phaseLocked = GuidedPhase.IDLE
                        // A target waiting on this engagement dies with it: a refused enable
                        // already ACCEPTED on the wire is repaired by the STATUSTEXT below,
                        // never by a target lying in ambush for the next engagement.
                        pendingGotoDropped = reposition != null
                        pendingOrbitDropped = orbit != null
                        pendingDescentDropped = tagDescent != null
                        reposition = null
                        orbit = null
                        tagDescent = null
                        true
                    } else {
                        false
                    }
                }
                if (cancelled) {
                    log("enableVirtualStick refused by DJI: $djiError")
                    if (pendingGotoDropped) record.event(EventCode.GOTO_ENDED, "engage refused", warn = true)
                    if (pendingOrbitDropped) record.event(EventCode.ORBIT_ENDED, "engage refused", warn = true)
                    if (pendingDescentDropped) {
                        record.event(EventCode.TAG_DESCENT_ENDED, "engage refused", warn = true)
                    }
                    announce(GuidedStatusTexts.refused(djiError))
                }
            },
        )
    }

    // ------------------------------------------------------------ DO_REPOSITION

    /**
     * M3 Stage B: QGC's Go-to (`COMMAND_INT 192`) and its Pause (`COMMAND_LONG` 192, NaN
     * coordinates). Runs on the `mavlink-rx` thread; the return value **is** the `COMMAND_ACK`,
     * so the ordering rule lives here:
     *
     * > **`ACCEPTED` if and only if the target was actually taken** — every gate below passed
     * > and the authority path started (or was already engaged/reused). Everything else is
     * > `DENIED` with a `STATUSTEXT` naming the reason, which QGC shows as a modal naming the
     * > command — the honest channel this command has that Return and Land never did. Never
     * > ack-then-silently-drop.
     *
     * The one exception is the interlock: **off answers `UNSUPPORTED`**, not `DENIED` — the
     * `CommandDispatcher.onTakeoff` precedent, which is M2 §Q2 taken literally: with commands
     * off the reply must be byte-for-byte the reply that existed before the feature (command
     * 192 was unregistered → `UNSUPPORTED`), and it lets an operator tell "commands are
     * switched off" from "commands are on and this reposition was refused" without looking at
     * the phone. No `STATUSTEXT` either, for the reason given there: QGC raises its own modal
     * for an unsupported command, so there is no silence to fill. *(This deliberately reads
     * the Stage B brief's "DENIED (interlock off …)" against the older, argued rule — recorded
     * as JC-1 in `docs/m3-stage-b.md`.)*
     *
     * ## The altitude datum — landmine 1, the whole of the altitude handling
     *
     * Inbound `z` is AMSL **composed by QGC from our own last published AMSL** (measured:
     * `PX4FirmwarePlugin.cc:415` sends `vehicle->altitudeAMSL()`, which QGC rebuilt from our
     * wire at 5 Hz). Our published AMSL is `takeoffAltitudeAmsl + relativeAltitude`, and
     * `takeoffAltitudeAmsl` is pressure altitude — it moved **41.5 m between sessions** and
     * drifts 2.3 m in twelve minutes. So the absolute number is never trusted: the target's
     * relative altitude is recovered as `z − takeoffAltitudeAmsl`, the same datum entering
     * with the opposite sign and cancelling exactly, whatever the weather. With no datum
     * published this link there is nothing to subtract and the reposition is **denied**, never
     * guessed at — the same `NO_ALT_DATUM` refusal takeoff makes, for the same reason.
     * Freshness deliberately does not gate the subtraction; `CommandDispatcher
     * .relativeTakeoffAltitude` carries the argument (both sides of the round trip age
     * together and cancel just as exactly).
     *
     * A recovered relative altitude **below the takeoff datum is refused** ([GuidedStatusTexts
     * .REASON_BELOW_DATUM]): plain Go-to arrives at ≈ the current altitude by construction, so
     * a below-datum target is either a launch-from-a-hill descent an operator can fly manually
     * or — the case worth guarding — a `z` composed against a datum that is not ours, landing
     * tens of metres off in the one direction that ends in the ground. One **above the Q1
     * ceiling is capped to it and announced** ([GuidedStatusTexts.GOTO_CAPPED]) rather than
     * refused — the JC-5 reading of "refused, never silently clamped": the lateral intent is
     * still honoured, nothing exceeds the envelope, and the operator is told exactly what was
     * not obeyed.
     *
     * ## What a reposition does *not* require, that a stick engagement does
     *
     * The stream-at-rest gate is stick-specific (it proves the z convention of a stream this
     * command does not use) and is skipped. The engage retry window is skipped too: it exists
     * so a held stick cannot hammer DJI at 25 Hz, and an acknowledged command is one press —
     * the `CommandDispatcher` rule that a repeat after a refusal must reach DJI again applies.
     * The RC-feed gate stays: abort gesture 1 is built on that feed, and a manoeuvre flown
     * blind to it would drop Q3's safety story.
     */
    fun reposition(cmd: RepositionCommand, origin: ControlOrigin = ControlOrigin.MAVLINK): Verdict {
        // The command is itself inbound traffic — evidence *this* controller is alive. Bridge
        // routes it to onInbound too, but only *after* this handler has returned its ack, and the
        // link watchdog must never start a reposition already half-expired.
        synchronized(lock) { gcsSeenAtMs[origin] = nowMs() }
        if (!interlockEnabled()) {
            log("DO_REPOSITION with interlock off — refused as before (UNSUPPORTED)")
            return Verdict.UNSUPPORTED
        }
        if (!cmd.isCommandInt) return pauseOrRefuseLong(cmd, origin)

        // Static shape checks — anything off the measured Go-to wire shape is refused,
        // never interpreted (the StickMapping.read rule, applied to a command).
        if (cmd.frame != RepositionCommand.FRAME_GLOBAL) {
            return deny("FRAME_${cmd.frame}", "frame ${cmd.frame} is not the measured MAV_FRAME_GLOBAL")
        }
        if (cmd.yawRad.isFinite()) {
            // QGC's Go-to sends NaN (keep heading, measured). A finite param4 is Change
            // Heading's shape — accepting it and not yawing would be a command the operator
            // believes was obeyed.
            return deny(GuidedStatusTexts.REASON_YAW, "param4=${cmd.yawRad} — Stage B does not yaw")
        }
        if (cmd.groundSpeedMs.isFinite() && cmd.groundSpeedMs > 0f) {
            // The Q1 envelope is the only speed there is; a speed request silently outvoted
            // by it would be a clamped command. QGC sends -1 ("default").
            return deny(GuidedStatusTexts.REASON_SPEED, "param1=${cmd.groundSpeedMs} — envelope is not negotiable")
        }
        if (!cmd.zAmslM.isFinite()) {
            return deny("ALT_NOT_A_NUMBER", "z=${cmd.zAmslM}")
        }
        val targetLat = cmd.latE7 / 1e7
        val targetLon = cmd.lonE7 / 1e7
        if (Geo.coordinateOrNull(targetLat, targetLon) == null) {
            return deny(GuidedStatusTexts.REASON_BAD_TARGET, "x=${cmd.latE7} y=${cmd.lonE7} is not a coordinate")
        }
        return acceptTarget(Pair(targetLat, targetLon), CommandedAltitude.Amsl(cmd.zAmslM.toDouble()), origin)
    }

    /**
     * How high an accepted target is, and in whose frame — the two ways a height reaches
     * [acceptTarget], kept apart by the type so that neither can be read as the other.
     *
     * The distinction is landmine 1 made structural. A number that arrived over MAVLink is AMSL in
     * a datum QGC built from *our own* published one, and must have that datum subtracted back out
     * or DJI's pressure-altitude error (+14 m one day, −28 m the next) stays in it. A number this
     * engine armed itself never entered QGC's sum at all and must **not** be put through the
     * subtraction — doing so would take tens of metres off a takeoff climb.
     */
    private sealed interface CommandedAltitude {

        /**
         * `COMMAND_INT.z` / `COMMAND_LONG.param7` — metres AMSL in this bridge's own published
         * datum, exactly as QGC composed it. Recovered by subtracting the datum we published.
         */
        data class Amsl(val zM: Double) : CommandedAltitude {
            override fun toString(): String = "z=%.1f AMSL".format(zM)
        }

        /**
         * Metres above this bridge's own takeoff datum, already in our frame — the takeoff's
         * second phase, whose height was recovered from `param7` once, at
         * `CommandDispatcher.relativeTakeoffAltitude`, and has been relative ever since.
         */
        data class Relative(val relM: Double) : CommandedAltitude {
            override fun toString(): String = "%.1fm above the takeoff datum".format(relM)
        }
    }

    /**
     * The dynamic gates every accepted target passes, and the take itself — shared by the
     * measured `COMMAND_INT` Go-to (explicit target), the measured `COMMAND_LONG` Change
     * Altitude ([explicit] null: the target is the **current fix**, resolved only after the
     * fix gate has passed, so "keep position" can never mean a stale position) and the takeoff's
     * second phase, which is a Change Altitude this engine commands itself. Only this function may
     * return ACCEPTED.
     */
    private fun acceptTarget(
        explicit: Pair<Double, Double>?,
        altitude: CommandedAltitude,
        origin: ControlOrigin,
    ): Verdict {
        // Dynamic gates, freshest last.
        port.unavailableReason()?.let { reason ->
            return deny(reason, "SDK unavailable: $reason")
        }
        if (rc?.allPresent() != true) {
            return deny("NO_RC_FEED", "RC stick feed not delivering — abort gesture 1 would be blind")
        }
        val state = aircraftState()
        val datum = state.takeoffAltitudeAmsl
        if (datum == null || !datum.isFinite() || state.relativeAltitude == null) {
            return deny(
                GuidedStatusTexts.REASON_NO_DATUM,
                "no published AMSL datum this link to interpret $altitude against",
            )
        }
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            return deny(GuidedStatusTexts.REASON_NO_FIX, "no fresh position fix to fly from")
        }
        val targetLat = explicit?.first ?: fix.first
        val targetLon = explicit?.second ?: fix.second

        // The geometry, against the datum and the Q1 caps. Landmine 1 lives on this line: the
        // datum is subtracted from an AMSL and never from a height that is already ours.
        var targetRelAlt = when (altitude) {
            is CommandedAltitude.Amsl -> altitude.zM - datum
            is CommandedAltitude.Relative -> altitude.relM
        }
        // The datum gate above is still required for a Relative, and is not redundant: the
        // vertical loop reads `state.relativeAltitude` every tick, and a manoeuvre accepted while
        // that is unknown is one whose ceiling cannot be enforced.
        if (targetRelAlt < 0.0) {
            return deny(
                GuidedStatusTexts.REASON_BELOW_DATUM,
                "$altitude is %.1f m below the takeoff datum %.1f".format(-targetRelAlt, datum),
            )
        }
        var capped = false
        if (targetRelAlt > GuidedEnvelope.CEILING_M) {
            targetRelAlt = GuidedEnvelope.CEILING_M
            capped = true
        }
        val distance = RepositionGuidance.horizontalMetres(fix.first, fix.second, targetLat, targetLon)
        if (distance > GuidedEnvelope.MAX_REPOSITION_DISTANCE_M) {
            return deny(GuidedStatusTexts.REASON_TOO_FAR, "target %.0f m away".format(distance))
        }

        // Every gate passed: take the target. Only now may ACCEPTED be returned.
        val now = nowMs()
        var engage = false
        var resumed = false
        var replaced = false
        var replacedOrbit = false
        var replacedDescent = false
        var pausedMission: Triple<MissionRunSink, MissionPauseCause, Int>? = null
        // The plan's ROI ending, captured under the lock and said after it — one owner
        // ([endPlanRoiLocked]) for all five paths that drop a run, because "a plan's ROI dies with
        // the plan" remembered at five sites is a rule forgotten at one of them.
        val roiEffects = mutableListOf<() -> Unit>()
        synchronized(lock) {
            replaced = reposition != null
            // A `DO_REPOSITION` is one of §8.4's terminators: a new destination ends the circle.
            replacedOrbit = orbit != null
            orbit = null
            // ...and it ends a tag descent, on the same reasoning: an explicit, validated,
            // acknowledged operator command outranks it, and the descent — unlike a mission —
            // has no cursor to pause at, so it simply dies and a fresh arm is the way back.
            replacedDescent = tagDescent != null
            tagDescent = null
            // **A new destination releases the circle's implied target** (Ivan, 2026-07-27): the
            // operator is going somewhere else now, so the nose goes back to following the course.
            // An *explicit* ROI is untouched — that one is theirs and outlives every manoeuvre,
            // which is the whole point of being able to fly somewhere while watching something.
            orbitRoi = null
            // ...and it ends a mission, on §6.2 row 3's reasoning generalised: an explicit,
            // validated, acknowledged operator command outranks the plan. The plan is not deleted —
            // it pauses, resumably, at the item it was flying, exactly as a stick grab leaves it.
            mission?.let { run ->
                pausedMission = Triple(run.sink, MissionPauseCause.GCS_NEW_DESTINATION, run.cursorSeq())
                endPlanRoiLocked(run, roiEffects)
            }
            mission = null
            missionSetpointAtMs = null
            reposition = RepositionState(
                targetLat, targetLon, targetRelAlt, acceptedAtMs = now,
                // The commanded distance on both axes, straight into the one owner of "how long may
                // this take". `state.relativeAltitude` is non-null here — the datum gate above
                // refuses without it — so the vertical leg is measured rather than assumed; if that
                // ever changes, the ceiling is the honest substitute for an unknown height change
                // (never zero, which would shorten the deadline of an unmeasurable climb).
                deadlineMs = GuidedEnvelope.manoeuvreDeadlineMs(
                    horizontalM = distance,
                    verticalM = state.relativeAltitude
                        ?.let { abs(targetRelAlt - it) }
                        ?: GuidedEnvelope.CEILING_M,
                ),
            )
            // The controller that commanded the manoeuvre owns it, in every phase below: a
            // reposition taken while already engaged, or resumed out of a wind-down, is still
            // this controller's manoeuvre and it is this controller's silence that must end it.
            engagementOrigin = origin
            when (phaseLocked) {
                GuidedPhase.IDLE -> {
                    phaseLocked = GuidedPhase.ENGAGING
                    engageStartedAtMs = now
                    lastEngageAttemptAtMs = now
                    engage = true
                }

                GuidedPhase.ENGAGING -> Unit // target replaced; the engage in flight serves it

                GuidedPhase.ENGAGED -> Unit // authority reused

                GuidedPhase.RELEASING -> {
                    // Authority is still held during a wind-down. An explicit, validated,
                    // acknowledged command outranks it — the JC-8 principle, strengthened by
                    // the command being a deliberate single act rather than a returning stream.
                    phaseLocked = GuidedPhase.ENGAGED
                    engagedAtMs = now
                    clearReleaseLocked()
                    resumed = true
                }
            }
        }
        log(
            "DO_REPOSITION accepted: lat=%.7f lon=%.7f relAlt=%.1f dist=%.1fm%s%s%s%s".format(
                targetLat, targetLon, targetRelAlt, distance,
                if (explicit == null) " (altitude change at current position)" else "",
                if (capped) " (altitude capped)" else "",
                if (replaced) " (replaces previous target)" else "",
                if (resumed) " (resumed from wind-down)" else "",
            )
        )
        record.event(
            EventCode.GOTO_ACCEPTED,
            "lat=%.7f lon=%.7f relAlt=%.1f dist=%.1f capped=%s".format(
                targetLat, targetLon, targetRelAlt, distance, capped,
            ),
        )
        if (replaced) record.event(EventCode.GOTO_ENDED, "replaced")
        if (replacedOrbit) record.event(EventCode.ORBIT_ENDED, "replaced by goto")
        if (replacedDescent) record.event(EventCode.TAG_DESCENT_ENDED, "replaced by goto", warn = true)
        pausedMission?.let { (sink, cause, seq) ->
            log("mission paused by a goto at item $seq")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause", warn = true)
            sink.onPaused(cause, seq)
        }
        // The plan's ROI, if the plan had one and it was still set — said after the pause it belongs to.
        roiEffects.forEach { it() }
        announce(GuidedStatusTexts.GOTO_STARTED)
        // The substitution, once per manoeuvre: QGC's `param4 = NaN` used to mean "keep heading"
        // here, and that reading is flight-verified. Changing what it means is a substitution, and
        // this project announces substitutions.
        if (headingFollowsCourse()) announce(GuidedStatusTexts.HEADING_FOLLOWS)
        if (capped) announce(GuidedStatusTexts.GOTO_CAPPED)
        if (resumed) announce(GuidedStatusTexts.RESUMED)
        if (engage) beginEngage()
        return Verdict.ACCEPTED
    }

    /**
     * The `COMMAND_LONG` 192 forms, both measured off QGC's wire:
     *
     *  - **Pause** — NaN lat/lon/alt (`PX4FirmwarePlugin.cc:272-284`, and measured): "drop the
     *    target, keep station" — a withdrawal, not a new intent, so it is the one QGC-side
     *    control that may end a manoeuvre (Q3 names Pause the natural convenience abort).
     *  - **Change Altitude** — NaN lat/lon, **finite `param7`** (measured 2026-07-26 22:19,
     *    record `20260726-221915.001`: `param1=-1, param2=1, param4/5/6=NaN, param7=AMSL`).
     *    "Keep position, new altitude": the current fix becomes the target and every Go-to
     *    gate — datum, below-datum guard, ceiling cap, fix freshness — applies unchanged.
     *
     * Anything else in `COMMAND_LONG` clothing carries float coordinates this project has
     * never measured and is refused rather than interpreted.
     */
    private fun pauseOrRefuseLong(cmd: RepositionCommand, origin: ControlOrigin): Verdict {
        val nanLatLon = cmd.param5.isNaN() && cmd.param6.isNaN()
        if (nanLatLon && cmd.zAmslM.isFinite()) {
            if (cmd.yawRad.isFinite()) {
                return deny(GuidedStatusTexts.REASON_YAW, "param4=${cmd.yawRad} — Stage B does not yaw")
            }
            if (cmd.groundSpeedMs.isFinite() && cmd.groundSpeedMs > 0f) {
                return deny(GuidedStatusTexts.REASON_SPEED, "param1=${cmd.groundSpeedMs} — envelope is not negotiable")
            }
            return acceptTarget(null, CommandedAltitude.Amsl(cmd.zAmslM.toDouble()), origin)
        }
        val pauseShaped = nanLatLon && !cmd.zAmslM.isFinite()
        if (!pauseShaped) {
            return deny(GuidedStatusTexts.REASON_LONG_FORM, "COMMAND_LONG with float coordinates — unmeasured shape")
        }
        return pause()
    }

    /**
     * **Pause — a withdrawal, and therefore takes no origin.**
     *
     * The body of QGC's pause-shaped `COMMAND_LONG` 192, lifted out so the phone can reach the
     * *same* code rather than a second implementation of it. Extracted 2026-07-27 for the
     * on-screen withdrawal controls; [pauseOrRefuseLong] now recognises the shape and delegates,
     * so QGC's pause and the phone's Pause are byte-for-byte the same act on the aircraft, the
     * same flight-record event and the same `STATUSTEXT`.
     *
     * No [ControlOrigin] parameter, and that is the decision rather than an omission.
     * `docs/decisions/2026-07-26-m3-guided-control.md` Q3 makes withdrawals honoured from any
     * origin: asking *who* wants a manoeuvre stopped could only ever be a way to refuse one, and
     * there is no circumstance in which this bridge should decline to stop flying. (The origin
     * map exists for *liveness* — whether the controller that started something is still there —
     * which is a question about continuing, not about stopping.)
     *
     * Safe from any thread and idempotent: a second pause finds an already-held orbit or an
     * already-arrived goto and answers `DENIED` with "nothing to pause", which is the truth.
     */
    fun pause(): Verdict {
        val now = nowMs()
        var outcome: String? = null
        var pausedOrbit = false
        var pausedDescent = false
        var pausedCommittedLanding = false
        var pausedMission: Triple<MissionRunSink, MissionPauseCause, Int>? = null
        // The plan's ROI ending, captured under the lock and said after it — one owner
        // ([endPlanRoiLocked]) for all five paths that drop a run, because "a plan's ROI dies with
        // the plan" remembered at five sites is a rule forgotten at one of them.
        val roiEffects = mutableListOf<() -> Unit>()
        synchronized(lock) {
            // The descent's withdrawal, first: a descent and the other manoeuvres are mutually
            // exclusive, so at most one of these blocks finds anything. "Stop and hold" is the
            // goto's own arrival hold, reused exactly as the mission's pause reuses it — the
            // descent itself is **dropped, not suspended**: there is deliberately no resume, and
            // a fresh arm through every gate is the only way back down (rule 1's shape, applied
            // to every cancel path uniformly).
            tagDescent?.let {
                pausedCommittedLanding = it.machine.phase == TagDescentPhase.DJI_LANDING
                tagDescent = null
                when (phaseLocked) {
                    GuidedPhase.ENGAGED -> {
                        val state = aircraftState()
                        val here = Geo.coordinateOrNull(state.latitude, state.longitude)
                        if (here != null && state.isFresh(Signal.POSITION)) {
                            reposition = RepositionState.holdAt(
                                here.first, here.second,
                                relAltM = state.relativeAltitude ?: 0.0,
                                nowMs = now,
                            )
                        }
                        outcome = "paused"
                        pausedDescent = true
                    }

                    GuidedPhase.ENGAGING -> {
                        phaseLocked = GuidedPhase.IDLE
                        outcome = "cancelled before engagement"
                        pausedDescent = true
                    }

                    else -> Unit // RELEASING: already stopping; nothing extra to promise
                }
            }
            // §6.2 row 19, and the mission answer to it. Q3 already named Pause the natural GCS-side
            // convenience abort — a withdrawal, not a new intent — so the plan pauses **resumably**
            // and the aircraft does exactly what a paused goto does: drops the target and keeps
            // station, still engaged, with Q1's idle clock running from now.
            //
            // The hold is the goto's own arrival hold, reused rather than restated: a mission
            // paused mid-air and a goto paused mid-air are the same aircraft in the same state, and
            // there is no second implementation of "stop and hold" to get wrong. It is anchored at
            // the **current fix**, taken here, so "keep position" can never mean a stale one.
            mission?.let { run ->
                val state = aircraftState()
                val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
                pausedMission = Triple(run.sink, MissionPauseCause.GCS_PAUSE, run.cursorSeq())
                endPlanRoiLocked(run, roiEffects)
                mission = null
                missionSetpointAtMs = null
                if (phaseLocked == GuidedPhase.ENGAGED && fix != null && state.isFresh(Signal.POSITION)) {
                    reposition = RepositionState.holdAt(
                        fix.first, fix.second,
                        relAltM = state.relativeAltitude ?: 0.0,
                        nowMs = now,
                    )
                    outcome = "paused"
                } else {
                    // Nothing to hold with — no authority, or no fix to hold at. The engagement's
                    // own machinery takes it from here; the plan is paused either way.
                    outcome = "paused"
                }
            }
            val orb = orbit
            if (orb != null) {
                // §8.4: an explicit Pause is one of the orbit's terminators. "Stop and hold" is
                // made the hold the orbit already has an ending for — zero setpoints, still
                // engaged, Q1's idle clock running from now.
                //
                // **The swept progress dies with it**, and that is Ivan's M4-8 answer rather than
                // an implementation shortcut: swept angle is a claim about where the aircraft has
                // been, and after a pause of unknown length with unknown drift it is a claim we
                // cannot re-verify. There is deliberately no resume — a circle restarts only on a
                // fresh, acknowledged `DO_ORBIT`, which begins at the join phase with the counter
                // at zero, so the property holds structurally rather than by remembering to reset.
                when (phaseLocked) {
                    GuidedPhase.ENGAGED -> {
                        orb.phase = OrbitPhase.HOLD
                        orb.holdingSinceMs = now
                        orb.arriveTicks = 0
                        orb.tangentialMs = 0.0
                        outcome = "paused"
                        pausedOrbit = true
                    }

                    GuidedPhase.ENGAGING -> {
                        orbit = null
                        phaseLocked = GuidedPhase.IDLE
                        outcome = "cancelled before engagement"
                        pausedOrbit = true
                    }

                    else -> Unit // RELEASING: already stopping; nothing extra to promise
                }
            }
            val repo = reposition
            when {
                pausedMission != null -> Unit // handled above, hold and all
                pausedOrbit -> Unit // handled above; an orbit and a goto are never both live
                pausedDescent -> Unit // handled above; the hold it synthesised is already arrived
                repo == null -> Unit // nothing of ours to pause

                phaseLocked == GuidedPhase.ENGAGED -> {
                    // "Stop and hold", by making the hold the arrival hold: zero setpoints,
                    // still engaged, idle disengage clock running from now.
                    repo.arrived = true
                    repo.arrivedAtMs = now
                    repo.arriveTicks = 0
                    outcome = "paused"
                }

                phaseLocked == GuidedPhase.ENGAGING -> {
                    // Never had authority; nothing is moving. Pausing an unstarted manoeuvre
                    // is cancelling it — withdraw the engage request like the confirm timeout.
                    reposition = null
                    phaseLocked = GuidedPhase.IDLE
                    outcome = "cancelled before engagement"
                }

                else -> Unit // RELEASING: already stopping; nothing extra to promise
            }
        }
        val done = outcome ?: return deny(GuidedStatusTexts.REASON_NOTHING_TO_PAUSE, "pause with no active goto")
        pausedMission?.let { (sink, cause, seq) ->
            log("mission paused by Pause at item $seq")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause")
            announce(GuidedStatusTexts.MISSION_PAUSED)
            sink.onPaused(cause, seq)
            // Inside the `let`, before its `return`: this branch leaves the function here.
            roiEffects.forEach { it() }
            return Verdict.ACCEPTED
        }
        if (pausedDescent) {
            log("tag descent pause: $done")
            record.event(EventCode.TAG_DESCENT_ENDED, "paused")
            // A pause during a committed landing withdraws DJI's landing too — same argument
            // as the disarm: a stop that left the aircraft autonomously descending is no stop.
            if (pausedCommittedLanding) stopDjiLanding("paused")
            if (done == "cancelled before engagement") requestDisable()
            announce(GuidedStatusTexts.DESCENT_DISARMED)
            return Verdict.ACCEPTED
        }
        log("DO_REPOSITION pause: $done${if (pausedOrbit) " (orbit)" else ""}")
        record.event(if (pausedOrbit) EventCode.ORBIT_ENDED else EventCode.GOTO_ENDED, "paused")
        if (done == "cancelled before engagement") requestDisable()
        announce(if (pausedOrbit) GuidedStatusTexts.ORBIT_PAUSED else GuidedStatusTexts.GOTO_PAUSED)
        return Verdict.ACCEPTED
    }

    // ------------------------------------------------- the takeoff's second phase

    /**
     * [PendingClimb.armTakeoffClimb] — M2.5 has just handed a takeoff to DJI and this is the climb
     * that follows it. Runs on the `mavlink-rx` thread, inside `CommandDispatcher.onTakeoff`'s
     * accepted branch and nowhere else.
     *
     * **Arming actuates nothing.** No enable, no engagement, no setpoint: the aircraft is on the
     * ground and DJI is about to fly it, and the whole of what happens here is that a number and a
     * timestamp are written down. Every gate that decides whether the climb may *fly* — the
     * interlock, the RC feed, the SDK, the fix, the datum, the ceiling — is re-read later, by
     * [tickTakeoffClimbLocked] and then by [acceptTarget], on the tick that would actually start
     * it. Checking them here as well would only mean refusing a climb for a condition that has
     * half a minute to change.
     *
     * The returned [ClimbArm] is what the operator's sentence is composed from, which is why the
     * ceiling cap is applied inside [TakeoffClimb.arm] rather than left to [acceptTarget]: both
     * would cap, but only this one lets the substitution be read before the aircraft moves.
     */
    override fun armTakeoffClimb(requestedRelAltM: Double, aimCameraNadir: Boolean, origin: ControlOrigin): ClimbArm {
        val now = nowMs()
        val outcome = synchronized(lock) { takeoffClimb.arm(requestedRelAltM, now, aimCameraNadir, origin) }
        when (outcome) {
            ClimbArm.NothingToDo ->
                log("takeoff climb not armed: ${requestedRelAltM}m is inside DJI's own takeoff")

            is ClimbArm.Armed -> {
                // The origin is on the record because landing08's diagnosis needed it and did
                // not have it: which controller's liveness the climb will be judged by is a
                // fact of the arm, readable before the engagement exists.
                log(
                    "takeoff climb armed: %.1fm capped=%s origin=%s%s"
                        .format(outcome.relAltM, outcome.capped, origin.name.lowercase(), if (aimCameraNadir) " nadir" else "")
                )
                record.event(
                    EventCode.TAKEOFF_CLIMB_ARMED,
                    "relAlt=%.1f capped=%s origin=%s%s"
                        .format(outcome.relAltM, outcome.capped, origin.name.lowercase(), if (aimCameraNadir) " nadir" else ""),
                )
            }
        }
        return outcome
    }

    /**
     * [PendingClimb.cancelTakeoffClimb] — the abort ladder's outside edge. Called by
     * `CommandDispatcher.reportAsyncDjiError`, because a DJI error arriving behind a takeoff we
     * started is evidence the takeoff is in doubt. Idempotent, safe from any thread, and silent
     * when nothing is armed.
     */
    override fun cancelTakeoffClimb(reason: String) {
        if (!synchronized(lock) { takeoffClimb.cancel() }) return
        announceClimbCancelled(reason)
    }

    /**
     * One armed climb ending because something cancelled it. Never called with nothing armed —
     * both callers check first — so this always has something to say.
     */
    private fun announceClimbCancelled(reason: String) {
        log("takeoff climb cancelled: $reason")
        record.event(EventCode.TAKEOFF_CLIMB_ENDED, "cancelled $reason", warn = true)
        announce(GuidedStatusTexts.takeoffClimbCancelled(reason))
    }

    /**
     * The pending climb, one tick. Must hold [lock].
     *
     * **This is the abort ladder applied to an intention instead of to an engagement**, and it
     * exists because the ladder's usual home — [tickEngagedLocked] — is not running: while DJI
     * flies its own takeoff this engine holds no authority at all and its phase is
     * [GuidedPhase.IDLE], so [abort] is never called and nothing else would look. The two rungs
     * that can be evaluated without authority are re-read here every tick, in the same order and
     * for the same reasons:
     *
     *  - **the interlock**, which is the operator's "this bridge may not move my aircraft" switch,
     *    and which must revoke a pending climb as completely as it revokes a flying one;
     *  - **the RC sticks**, gesture 1, which is exactly the hand an operator puts on the
     *    controller when a takeoff is not going the way they expected.
     *
     * The other rungs need no row here. DJI's authority reasons and the `VirtualStickState`
     * conjuncts are statements about an engagement that does not exist yet; a send cannot fail
     * because nothing is being sent; and the link watchdog is deliberately absent, because a
     * ground station that goes quiet for three seconds during a takeoff has not withdrawn the
     * takeoff — the climb will meet the watchdog the moment it becomes a reposition, which is
     * where it belongs.
     *
     * Ordered cancel-before-decide, so a tick that both revokes and completes revokes: the
     * conditions above are checked before [TakeoffClimb.observe] is asked anything.
     */
    private fun tickTakeoffClimbLocked(now: Long, effects: MutableList<() -> Unit>) {
        if (!takeoffClimb.armed) return
        if (!interlockEnabled()) {
            takeoffClimb.cancel()
            effects += { announceClimbCancelled(DisengageReason.INTERLOCK.wire) }
            return
        }
        if (rcAbortDueLocked(now)) {
            takeoffClimb.cancel()
            effects += { announceClimbCancelled(DisengageReason.RC_STICKS.wire) }
            return
        }
        val state = aircraftState()
        when (val decision = takeoffClimb.observe(state.isFlying, state.flightMode, now)) {
            TakeoffClimb.Decision.Idle, TakeoffClimb.Decision.Waiting -> Unit

            TakeoffClimb.Decision.Expired -> effects += {
                log("takeoff climb expired after ${TakeoffClimb.WAIT_LIMIT_MS}ms — the aircraft never flew")
                record.event(EventCode.TAKEOFF_CLIMB_ENDED, "expired", warn = true)
                announce(GuidedStatusTexts.TAKEOFF_CLIMB_EXPIRED)
            }

            // Never null on this path — either of them: [armTakeoffClimb] is the only thing
            // that arms *this* instance, and it always arms a target and its door's origin
            // together. The null forms belong to the mission's own instance
            // ([TakeoffClimb.armWatch]), which this branch never sees; treating them as
            // nothing to fly keeps an impossible case inert rather than fatal.
            is TakeoffClimb.Decision.HandedBack -> {
                val target = decision.relAltM
                val origin = decision.origin
                if (target != null && origin != null) {
                    effects += { flyTakeoffClimb(target, decision.aimCameraNadir, origin) }
                }
            }
        }
    }

    /**
     * DJI's takeoff is finished; fly the climb. Runs **outside** [lock], from [tick]'s effect
     * list, because [acceptTarget] takes the lock itself.
     *
     * The climb is an ordinary Change Altitude and is fed through the ordinary path: no explicit
     * coordinate, so the target is the current fix at [relAltM] metres above our own takeoff
     * datum, and every gate [acceptTarget] applies — SDK, RC feed, datum, fresh fix, ceiling —
     * applies here unchanged. A refusal is therefore possible and is *not* smoothed over: it goes
     * out as the ordinary `Goto refused: <reason>` sentence, immediately after this one, so the
     * operator reads why. There is no retry, and the pending climb is already gone
     * ([TakeoffClimb.Decision.HandedBack] consumes it) — an intention that re-armed itself after
     * failing is the thing this whole design is arranged to prevent.
     *
     * [origin] is the door the takeoff came in through, carried on the armed intention since
     * [armTakeoffClimb] (the [aimCameraNadir] pattern): the climb is that controller's manoeuvre
     * and the engagement's liveness is evaluated within it — QGC's takeoff keeps the MAVLink
     * heartbeat watchdog; the phone's climb watches the process it runs in (landing08,
     * `datasets/landing08/20260729-112216.001.jsonl`, is the flight where the missing label
     * killed the climb at t=33.93). Note this deliberately does **not** stamp any controller's
     * liveness the way [reposition] does: nothing has just been heard from anyone, and the
     * reposition tick's own watchdog is the right place for that question.
     *
     * ## The camera, and why it is commanded exactly here
     *
     * [aimCameraNadir] — the takeoff sequence's camera half, since 2026-07-29 passed true by
     * both doors (`CommandDispatcher.takeoffFromPhone` has the supersession story) and
     * independent of [origin] — fires at this point and no other, and the *when* was chosen
     * against evidence rather than convenience:
     *
     *  - **Not at dispatch.** The aircraft is on the ground with DJI's own hop about to run,
     *    and nothing has measured whether an auto-takeoff disturbs a pre-pointed gimbal (the
     *    *landing* recenter is measured — `record/GimbalRecenter`, ~300 °/s to level on every
     *    landing — the takeoff direction has simply never been observed with the camera down).
     *    A commanded −90° that DJI then silently moved would make `CommandedGimbalPort.pitchDeg`
     *    — the number `TagWorld.fix` does trigonometry with — a lie manufactured by our own
     *    sequencing. Commanding after DJI has let go is never exposed to that unmeasured window.
     *  - **The measured-good shape is exactly this.** landing06 (`datasets/landing06/
     *    20260728-205913.001.jsonl`): takeoff at t=16.0, `gimbal_rotate pitchDeg=-90 absolute`
     *    at t=19.77 while airborne, and every descent afterwards armed clean.
     *  - **The abort ladder comes free.** The flag rides the armed climb, so every rung that
     *    kills the climb — interlock, RC grab, GCS deflection, async DJI error, expiry — kills
     *    the camera move with it: a takeoff that never happened never aims a camera.
     *
     * Aimed **before** the climb's own accept, deliberately: DJI has flown the takeoff and the
     * aircraft is airborne at ~1.2 m, so the camera half of the operator's ask is due even if
     * the climb is then refused by the ordinary gates — a refusal the operator reads next to it.
     * Through [manoeuvreGimbal] (one path to DJI, one rate window, one recorded ask —
     * `dji_call op=gimbal_rotate` comes free from `RecordedGimbalPort`), and open loop like
     * every camera command here. Null gimbal is logged, not absorbed silently.
     */
    private fun flyTakeoffClimb(relAltM: Double, aimCameraNadir: Boolean = false, origin: ControlOrigin) {
        log("DJI reports takeoff finished — starting the commanded climb to %.1fm".format(relAltM))
        record.event(
            EventCode.TAKEOFF_CLIMB_ENDED,
            "fired relAlt=%.1f origin=%s".format(relAltM, origin.name.lowercase()),
        )
        if (aimCameraNadir) aimTakeoffCameraNadir()
        announce(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING)
        val verdict = acceptTarget(null, CommandedAltitude.Relative(relAltM), origin)
        if (verdict != Verdict.ACCEPTED) log("takeoff climb not taken: $verdict")
    }

    /**
     * **The takeoff sequence's camera half — one implementation, all three takeoff doors.**
     * Runs outside [lock], from an effect list, at DJI's handback and at no other moment (the
     * *when* is argued in full at [flyTakeoffClimb]).
     *
     * Extracted 2026-07-30 rather than copied into the mission path, and the reason is the
     * failure that made it necessary: until landing16 the phone and QGC doors passed
     * `aimCameraNadir = true` and the mission's `NAV_TAKEOFF` did not, so the aircraft's camera
     * depended on which door the takeoff came in through. That is the two-places-for-one-property
     * shape with one place missing — the worst version, because nothing disagreed, one door was
     * simply silent. One function, called from both handbacks, is what makes "every takeoff points
     * the camera down" a property of the code.
     *
     * Ivan's standing preference, 2026-07-29: camera down on every takeoff. Through
     * [manoeuvreGimbal] (one path to DJI, one rate window, one recorded ask —
     * `dji_call op=gimbal_rotate` comes free from `RecordedGimbalPort`, which is also what puts
     * the angle into `gimbal/PitchBelief`'s commanded half), open loop, and with a null gimbal
     * logged rather than absorbed.
     */
    private fun aimTakeoffCameraNadir() {
        val gimbal = manoeuvreGimbal
        if (gimbal == null) {
            log("takeoff sequence wanted the camera at nadir but no gimbal path is wired")
            return
        }
        log("takeoff sequence: pointing the camera at nadir (%.0f)".format(TagDescentGuidance.NADIR_PITCH_DEG))
        gimbal.aimPitch(TagDescentGuidance.NADIR_PITCH_DEG)
    }

    /** One refusal: log, flight record, `STATUSTEXT`, and the `DENIED` the modal is built from. */
    private fun deny(reason: String, detail: String): Verdict {
        log("DO_REPOSITION denied: $detail")
        record.event(EventCode.GOTO_DENIED, reason, warn = true)
        announce(GuidedStatusTexts.gotoRefused(reason))
        return Verdict.DENIED
    }

    // ------------------------------------------------------------------ MISSION

    /**
     * **M4: fly a plan.** The engine's whole half of the mission executor — a route in, the same
     * engagement path a `DO_REPOSITION` takes, and a [MissionRunSink] out.
     *
     * Called from the executor, which has already run the launch check (§7.2). The gates here are
     * the *engagement's* gates and nothing more, deliberately: duplicating the launch check would
     * be a second place for the same property to be got wrong, and the checks that belong to this
     * side are exactly the ones the reposition path also makes because they are about whether an
     * engagement can exist at all.
     *
     * `ACCEPTED` if and only if the route was taken and the authority path started (or was already
     * engaged). Never ack-then-silently-drop.
     *
     * @param startIndex which step of [route] to begin at — 0 for a fresh start, the paused cursor
     *   for a resume.
     * @param rejoining true for a resume, which makes the **first** leg a resting one whatever the
     *   step says (§6.3: a rejoin leg is one nobody drew, so it gets the conservative test).
     */
    fun missionStart(
        route: MissionRoute,
        startIndex: Int,
        rejoining: Boolean,
        sink: MissionRunSink,
        origin: ControlOrigin = ControlOrigin.MAVLINK,
    ): Verdict {
        synchronized(lock) { gcsSeenAtMs[origin] = nowMs() }
        if (!interlockEnabled()) {
            log("mission start with interlock off — refused")
            return Verdict.DENIED
        }
        if (startIndex !in 0 until route.size) {
            log("mission start with cursor $startIndex outside the route (${route.size} steps)")
            return Verdict.DENIED
        }
        port.unavailableReason()?.let { reason ->
            log("mission start refused: SDK unavailable: $reason")
            return Verdict.DENIED
        }
        if (rc?.allPresent() != true) {
            log("mission start refused: RC stick feed not delivering — abort gesture 1 would be blind")
            return Verdict.DENIED
        }
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        // The same split `MissionLaunch.positionUsable` makes at the gate, for the same measured
        // reason: on the ground the position key publishes nothing because nothing is moving, so a
        // flat freshness rule refuses every takeoff plan there is. A run that starts *on the
        // ground* is one whose first step is a takeoff — DJI flies that phase and we command zero —
        // so a position is not needed until the aircraft is airborne, by which time it is moving
        // and the ordinary rule has teeth again. Anything starting in the air is unchanged.
        val startsOnTheGround =
            route[startIndex].kind == MissionStepKind.TAKEOFF && state.isFlying == false
        if (fix == null || !(state.isFresh(Signal.POSITION) || (startsOnTheGround && state.fcConnected))) {
            log("mission start refused: no fresh position fix to fly from")
            return Verdict.DENIED
        }
        if (route[startIndex].kind == MissionStepKind.TAKEOFF && missionTakeoff == null) {
            log("mission start refused: the plan begins with a takeoff and nothing can start one")
            return Verdict.DENIED
        }

        val now = nowMs()
        var engage = false
        var resumed = false
        var replacedDescent = false
        synchronized(lock) {
            // One setpoint source at a time. A route replaces whatever was flying, and the
            // live manoeuvres end the way an operator command ends them.
            reposition = null
            orbit = null
            replacedDescent = tagDescent != null
            tagDescent = null
            // A plan is a sequence of destinations, so it releases the circle's implied target for
            // the same reason a single goto does.
            orbitRoi = null
            mission = MissionRun(
                route = route,
                sink = sink,
                startedAtMs = now,
                origin = origin,
                index = startIndex,
                rejoining = rejoining,
            ).also {
                it.legStartedAtMs = now
                it.legOriginLatDeg = fix.first
                it.legOriginLonDeg = fix.second
                // Seeded, not applied-and-forgotten: the cursor's own step already carries the plan's
                // sticky ROI, so the run starts knowing what it is about to ask for and the *next*
                // step's comparison is against the truth rather than against null.
                it.roiApplied = route[startIndex].roi
            }
            missionSetpointAtMs = null
            engagementOrigin = origin
            when (phaseLocked) {
                GuidedPhase.IDLE -> {
                    phaseLocked = GuidedPhase.ENGAGING
                    engageStartedAtMs = now
                    lastEngageAttemptAtMs = now
                    engage = true
                }

                GuidedPhase.ENGAGING, GuidedPhase.ENGAGED -> Unit

                GuidedPhase.RELEASING -> {
                    phaseLocked = GuidedPhase.ENGAGED
                    engagedAtMs = now
                    clearReleaseLocked()
                    resumed = true
                }
            }
        }
        log(
            "mission started: ${route.size} steps, cursor $startIndex (seq ${route[startIndex].seq})" +
                if (rejoining) " — rejoining, resting leg" else ""
        )
        record.event(
            EventCode.GOTO_ACCEPTED,
            "mission planId=${route.planId} steps=${route.size} cursor=$startIndex rejoin=$rejoining",
        )
        if (replacedDescent) record.event(EventCode.TAG_DESCENT_ENDED, "replaced by mission", warn = true)
        announce(GuidedStatusTexts.MISSION_STARTED)
        if (headingFollowsCourse()) announce(GuidedStatusTexts.HEADING_FOLLOWS)
        if (rejoining) announce(GuidedStatusTexts.MISSION_REJOINING)
        if (resumed) announce(GuidedStatusTexts.RESUMED)
        // **The plan's ROI at the cursor, taken now** — through the live door, outside the lock, like
        // every other announcement here. A **set** only: a route whose first step names no ROI names
        // nothing, and clearing there would be a plan taking away a target the operator clicked before
        // pressing Start. It is a *resume onto a step under an ROI* that this line exists for as much as
        // a Start at item 0: §9.5's "a resumed mission re-acquires it" needs no separate machinery,
        // because the cursor's step already knows which ROI the plan had in force there.
        route[startIndex].roi?.let { applyRoi(it, missionSeq = route[startIndex].seq) }
        if (engage) beginEngage()
        sink.onCursor(route[startIndex].seq)
        return Verdict.ACCEPTED
    }

    /**
     * **Conjunct 3 of the heartbeat's `AUTO_MISSION` claim**: a route is being flown, DJI itself
     * reports we hold virtual-stick authority, and a setpoint has actually gone out recently.
     *
     * The other two conjuncts — a plan is committed, and the executor's own state is `RUNNING` — are
     * the executor's to check, because they are facts about the plan rather than about the aircraft.
     *
     * Never latched, and that is the whole argument for making the claim at all
     * (`docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-1). It is re-earned from an observation
     * on the tick it is read: [GuidedPhase.ENGAGED] comes from `VirtualStickState` and not from our
     * request, and [missionSetpointAtMs] is compared against the clock rather than being a boolean
     * that was once set. **It cannot be non-null while the engine is not engaged**, which is the
     * mutation this design most needs to survive.
     */
    fun missionFlying(now: Long = nowMs()): Boolean = synchronized(lock) {
        if (mission == null) return false
        if (phaseLocked != GuidedPhase.ENGAGED) return false
        val sentAt = missionSetpointAtMs ?: return false
        now - sentAt <= MISSION_CLAIM_STALE_MS
    }

    /** The wire `seq` the engine's cursor points at, or null when no route is being flown. */
    fun missionCursorSeq(): Int? = synchronized(lock) { mission?.cursorSeq() }

    // ----------------------------------------------------------------- DO_ORBIT

    /**
     * M3 Stage C: QGC's Orbit button (`COMMAND_INT` 34, `MAV_CMD_DO_ORBIT`) — **a live Fly-mode
     * capability and a peer of the goto, not a mission feature.** Runs on the `mavlink-rx` thread
     * and the return value **is** the `COMMAND_ACK`, under Stage B's ordering rule verbatim:
     * `ACCEPTED` if and only if the circle was actually taken, everything else `DENIED` **with a
     * sentence**.
     *
     * The sentence is not decoration here. On 2026-07-27 this command was refused with a bare
     * `MAV_RESULT_UNSUPPORTED` and **no `STATUSTEXT` at all** — the only refusal in the project
     * that never said why — so the operator saw QGC's generic dialog and had no way to tell "the
     * bridge cannot do this" from "the bridge refused this circle". Every path out of this function
     * except the interlock carries a reason.
     *
     * The interlock is the one exception, and it is the `CommandDispatcher.onTakeoff` /
     * `reposition` precedent rather than an oversight: **off answers `UNSUPPORTED` with no
     * sentence**, byte-for-byte the reply that existed before this feature, because that lets an
     * operator tell "commands are switched off" from "commands are on and this circle was refused"
     * without looking at the phone — and QGC raises its own modal for an unsupported command, so
     * there is no silence to fill.
     *
     * ## What is refused, and the one thing that is capped instead
     *
     * A radius outside [OrbitGuidance.R_MIN_M]..[OrbitGuidance.R_MAX_M] is **refused, never
     * clamped** — a clamped circle is a *different* circle, drawn somewhere the operator did not
     * click, and the map would go on showing the one they drew. A finite `param2` (tangential
     * velocity) is refused exactly as Stage B refuses a ground-speed request: the Q1 envelope is
     * the only speed there is. A circle whose far side leaves the Q1 leg bound is refused. The
     * altitude, alone, is **capped and announced** rather than refused — the JC-5 reading Stage B
     * already applies to a goto: the lateral intent is still honoured and the operator is told
     * exactly what was not.
     *
     * ## The datum, and why it is safer here than anywhere else
     *
     * `z` is AMSL, and QGC composed it as `homePosition.altitude + sliderMetres` from **our own**
     * `HOME_POSITION` — so our pressure-altitude datum enters QGC's sum with one sign and leaves
     * ours with the other and cancels exactly, whatever the weather. The Orbit button does not even
     * appear without a valid home altitude. Everything else follows Stage B unchanged: no datum →
     * refused, below the datum → refused, above the ceiling → capped.
     *
     * ## `param3`, and the substitution we announce
     *
     * QGC sends `ORBIT_YAW_BEHAVIOUR_UNCHANGED` — the value [OrbitCommand.YAW_BEHAVIOUR_UNCHANGED]
     * documents as absent from our own dialect. **We fly nose-to-centre regardless**, because on
     * this airframe the gimbal cannot yaw (DJI issue #527) and pointing the camera at the centre is
     * achievable only by turning the aircraft. That is a substitution, so it is announced
     * ([GuidedStatusTexts.ORBIT_NOSE_TO_CENTRE]) whenever the request was not already
     * nose-to-centre — nothing exceeds the envelope, and the operator is never left believing
     * something else was obeyed.
     */
    fun orbit(cmd: OrbitCommand, origin: ControlOrigin = ControlOrigin.MAVLINK): Verdict {
        // As `reposition`: the command is itself evidence this controller is alive, stamped before
        // any gate so the link watchdog never starts a manoeuvre already half-expired.
        synchronized(lock) { gcsSeenAtMs[origin] = nowMs() }
        if (!interlockEnabled()) {
            log("DO_ORBIT with interlock off — refused as before (UNSUPPORTED)")
            return Verdict.UNSUPPORTED
        }

        // Static shape checks — anything off the measured wire shape is refused, never interpreted.
        if (!cmd.isCommandInt) {
            return denyOrbit(
                GuidedStatusTexts.REASON_ORBIT_LONG_FORM,
                "COMMAND_LONG 34 — QGC sends COMMAND_INT; the float-coordinate shape is unmeasured",
            )
        }
        if (cmd.frame != OrbitCommand.FRAME_GLOBAL) {
            return denyOrbit("FRAME_${cmd.frame}", "frame ${cmd.frame} is not the measured MAV_FRAME_GLOBAL")
        }
        if (cmd.velocityMs.isFinite() && cmd.velocityMs > 0f) {
            return denyOrbit(
                GuidedStatusTexts.REASON_SPEED,
                "param2=${cmd.velocityMs} — the envelope is not negotiable (Stage B's rule, verbatim)",
            )
        }
        val signedRadius = cmd.radiusM.toDouble()
        val radius = abs(signedRadius)
        if (!radius.isFinite() || radius < OrbitGuidance.R_MIN_M || radius > OrbitGuidance.R_MAX_M) {
            return denyOrbit(
                GuidedStatusTexts.REASON_ORBIT_RADIUS,
                "param1=${cmd.radiusM} — outside ${OrbitGuidance.R_MIN_M}..${OrbitGuidance.R_MAX_M} m, refused not clamped",
            )
        }
        if (!cmd.zAmslM.isFinite()) {
            return denyOrbit("ALT_NOT_A_NUMBER", "z=${cmd.zAmslM}")
        }
        val centreLat = cmd.latE7 / 1e7
        val centreLon = cmd.lonE7 / 1e7
        if (Geo.coordinateOrNull(centreLat, centreLon) == null) {
            return denyOrbit(GuidedStatusTexts.REASON_BAD_TARGET, "x=${cmd.latE7} y=${cmd.lonE7} is not a coordinate")
        }
        // QGC encodes direction in the sign of the radius (`GuidedActionsController.qml:641`).
        // Positive is clockwise; the counter-clockwise case is source-derived and still unmeasured.
        val direction = if (signedRadius < 0.0) -1 else 1

        // Dynamic gates, freshest last — the same ladder, in the same order, as `acceptTarget`.
        port.unavailableReason()?.let { reason ->
            return denyOrbit(reason, "SDK unavailable: $reason")
        }
        if (rc?.allPresent() != true) {
            return denyOrbit("NO_RC_FEED", "RC stick feed not delivering — abort gesture 1 would be blind")
        }
        val state = aircraftState()
        val datum = state.takeoffAltitudeAmsl
        if (datum == null || !datum.isFinite() || state.relativeAltitude == null) {
            return denyOrbit(
                GuidedStatusTexts.REASON_NO_DATUM,
                "no published AMSL datum this link to interpret z=${cmd.zAmslM} against",
            )
        }
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            return denyOrbit(GuidedStatusTexts.REASON_NO_FIX, "no fresh position fix to join the circle from")
        }

        var targetRelAlt = cmd.zAmslM - datum
        if (targetRelAlt < 0.0) {
            return denyOrbit(
                GuidedStatusTexts.REASON_BELOW_DATUM,
                "z=${cmd.zAmslM} is %.1f m below the takeoff datum %.1f".format(-targetRelAlt, datum),
            )
        }
        var capped = false
        if (targetRelAlt > GuidedEnvelope.CEILING_M) {
            targetRelAlt = GuidedEnvelope.CEILING_M
            capped = true
        }

        // The **whole circle** must fit the Q1 leg bound, not just its centre: the far side of the
        // ring is the farthest the aircraft will actually go, and bounding the centre alone would
        // let a 50 m circle 100 m away fly to 150 m.
        val toCentre = RepositionGuidance.horizontalMetres(fix.first, fix.second, centreLat, centreLon)
        if (toCentre + radius > GuidedEnvelope.MAX_REPOSITION_DISTANCE_M) {
            return denyOrbit(
                GuidedStatusTexts.REASON_ORBIT_TOO_FAR,
                "centre %.0f m away + radius %.0f m leaves the %.0f m bound"
                    .format(toCentre, radius, GuidedEnvelope.MAX_REPOSITION_DISTANCE_M),
            )
        }

        // `param4`: NaN is "vehicle default" (one turn), 0 is MAVLink's "forever" (bounded by the
        // time cap alone, and said so), anything else is a turn count truncated to what fits.
        val maxSweep = OrbitGuidance.maxSweepDeg(radius)
        var unbounded = false
        var turnsCapped = false
        val requested = when {
            !cmd.turns.isFinite() -> OrbitGuidance.DEFAULT_TURNS * OrbitGuidance.DEGREES_PER_TURN
            cmd.turns <= 0f -> {
                unbounded = true
                Double.POSITIVE_INFINITY
            }

            else -> cmd.turns * OrbitGuidance.DEGREES_PER_TURN
        }
        val requiredSweep = if (requested.isFinite() && requested > maxSweep) {
            turnsCapped = true
            maxSweep
        } else {
            requested
        }

        // The join target: the point on the circle nearest the aircraft *now*, fixed here so the
        // join is an ordinary resting leg toward a lat/lon rather than a moving carrot.
        val (joinLat, joinLon) = OrbitGuidance.joinPoint(centreLat, centreLon, radius, fix.first, fix.second)

        // Every gate passed. Only now may ACCEPTED be returned.
        val now = nowMs()
        var engage = false
        var resumed = false
        var replaced = false
        var replacedGoto = false
        var replacedDescent = false
        var pausedMission: Triple<MissionRunSink, MissionPauseCause, Int>? = null
        // The plan's ROI ending, captured under the lock and said after it — one owner
        // ([endPlanRoiLocked]) for all five paths that drop a run, because "a plan's ROI dies with
        // the plan" remembered at five sites is a rule forgotten at one of them.
        val roiEffects = mutableListOf<() -> Unit>()
        synchronized(lock) {
            replaced = orbit != null
            replacedGoto = reposition != null
            replacedDescent = tagDescent != null
            tagDescent = null
            // One setpoint source at a time: a new circle ends whatever was flying — including a
            // mission, which pauses resumably at the item it was on rather than being deleted.
            mission?.let { run ->
                pausedMission = Triple(run.sink, MissionPauseCause.GCS_NEW_DESTINATION, run.cursorSeq())
                endPlanRoiLocked(run, roiEffects)
            }
            mission = null
            missionSetpointAtMs = null
            reposition = null
            orbit = OrbitState(
                centreLatDeg = centreLat,
                centreLonDeg = centreLon,
                relAltM = targetRelAlt,
                radiusM = radius,
                direction = direction,
                requiredSweepDeg = requiredSweep,
                acceptedAtMs = now,
                joinLatDeg = joinLat,
                joinLonDeg = joinLon,
                // The join is an ordinary resting leg, so it gets the ordinary derived deadline
                // (2026-07-30). Its length is the aircraft's distance to the nearest point on the
                // circle — `|toCentre − radius|`, which is exactly what `joinPoint` returned — plus
                // whatever height change the circle's altitude asks for on the way. At a 2 km leg
                // bound a circle can now be commanded from 1.9 km away, where the flat 150 s would
                // have aborted the join at about 450 m out, every time.
                joinDeadlineMs = GuidedEnvelope.manoeuvreDeadlineMs(
                    horizontalM = abs(toCentre - radius),
                    verticalM = state.relativeAltitude
                        ?.let { abs(targetRelAlt - it) }
                        ?: GuidedEnvelope.CEILING_M,
                ),
            )
            // M4-6, built rather than described: the circle's centre becomes an ROI, so the nose
            // and the camera are pointed by the one system that points things — **from the moment
            // the orbit is accepted, which includes the join leg**. Before this the join flew to the
            // circle sideways with the nose wherever it happened to be, and the nose-to-centre law
            // only began on arrival, saturating the yaw clamp catching up an angle it could have
            // been closing all the way in (`docs/measurements/2026-07-27-first-mission-flown.md`).
            orbitRoi = RoiState(centreLat, centreLon, acceptedAtMs = now, implied = true)
            engagementOrigin = origin
            when (phaseLocked) {
                GuidedPhase.IDLE -> {
                    phaseLocked = GuidedPhase.ENGAGING
                    engageStartedAtMs = now
                    lastEngageAttemptAtMs = now
                    engage = true
                }

                GuidedPhase.ENGAGING -> Unit // the engage in flight serves this circle instead
                GuidedPhase.ENGAGED -> Unit // authority reused

                GuidedPhase.RELEASING -> {
                    phaseLocked = GuidedPhase.ENGAGED
                    engagedAtMs = now
                    clearReleaseLocked()
                    resumed = true
                }
            }
        }
        log(
            "DO_ORBIT accepted: centre=%.7f,%.7f R=%.1f dir=%+d relAlt=%.1f sweep=%s join=%.7f,%.7f%s%s%s"
                .format(
                    centreLat, centreLon, radius, direction, targetRelAlt,
                    if (requiredSweep.isFinite()) "%.0f°".format(requiredSweep) else "unbounded",
                    joinLat, joinLon,
                    if (capped) " (altitude capped)" else "",
                    if (replaced) " (replaces previous orbit)" else "",
                    if (replacedGoto) " (replaces a goto)" else "",
                )
        )
        record.event(
            EventCode.ORBIT_ACCEPTED,
            "centre=%.7f,%.7f R=%.1f dir=%+d relAlt=%.1f sweep=%.0f capped=%s"
                .format(centreLat, centreLon, radius, direction, targetRelAlt, requiredSweep, capped),
        )
        if (replaced) record.event(EventCode.ORBIT_ENDED, "replaced")
        if (replacedGoto) record.event(EventCode.GOTO_ENDED, "replaced by orbit")
        if (replacedDescent) record.event(EventCode.TAG_DESCENT_ENDED, "replaced by orbit", warn = true)
        pausedMission?.let { (sink, cause, seq) ->
            log("mission paused by an orbit at item $seq")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause", warn = true)
            sink.onPaused(cause, seq)
        }
        // The plan's ROI, if the plan had one and it was still set — said after the pause it belongs to.
        roiEffects.forEach { it() }
        announce(GuidedStatusTexts.ORBIT_STARTED)
        if (cmd.yawBehaviour.toInt() != OrbitCommand.YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER) {
            announce(GuidedStatusTexts.ORBIT_NOSE_TO_CENTRE)
        }
        if (capped) announce(GuidedStatusTexts.ORBIT_CAPPED)
        if (unbounded) announce(GuidedStatusTexts.ORBIT_UNBOUNDED)
        if (turnsCapped) announce(GuidedStatusTexts.ORBIT_TURNS_CAPPED)
        if (resumed) announce(GuidedStatusTexts.RESUMED)
        if (engage) beginEngage()
        return Verdict.ACCEPTED
    }

    /** One orbit refusal: log, flight record, `STATUSTEXT`, and the `DENIED` the modal is built from. */
    private fun denyOrbit(reason: String, detail: String): Verdict {
        log("DO_ORBIT denied: $detail")
        record.event(EventCode.ORBIT_DENIED, reason, warn = true)
        announce(GuidedStatusTexts.orbitRefused(reason))
        return Verdict.DENIED
    }

    // ------------------------------------------------------------- TAG DESCENT

    /**
     * **M3 Stage D: arm the tag-tracked descent** — centre over the latched AprilTag, descend
     * inside the alignment cone, end holding at [TagDescentGuidance.TARGET_HEIGHT_M] above the
     * tag. The arm surface is the phone's own screen (`MainActivity`, behind the same interlock
     * pattern as Return and Land), which enters through [armTagDescentFromPhone] — the door that
     * names its origin; QGC-side triggering is deliberately out of scope, and this
     * transport-neutral method with its [Verdict] return **is the seam it would attach at** — a
     * future command handler calls exactly this, exactly as `Bridge` routes `DO_REPOSITION`,
     * passing [ControlOrigin.MAVLINK] (the default) so its arms keep the heartbeat watchdog.
     *
     * Runs under Stage B's ordering rule verbatim: `ACCEPTED` if and only if every gate below
     * passed and the descent was actually taken. Every refusal is **named** — recorded, spoken,
     * and returned as `DENIED` — because an operator whose arm press does nothing is owed the
     * reason, and the arm gates are the safety case:
     *
     *  1. **the latch** — the camera has seen this flight's tag enough times to believe
     *     (`vision/TagLatch`: three sightings in two seconds, chosen against the two measured
     *     false ids in 1978 frames);
     *  2. **a fresh world fix of the latched id** — the tag is in view *now*, within
     *     [TagDescentGuidance.ARM_FRESH_MS], not merely remembered from the climb-out;
     *  3. **the detection ceiling, two-tiered** — below [TagDescentGuidance.ARM_CEILING_M]
     *     (the measured edge of the 100 % per-frame band) is today's arm, unchanged; between
     *     it and [TagDescentGuidance.APPROACH_CEILING_M] the arm is accepted into the
     *     APPROACH segment, which descends into the band and hands off to the ordinary
     *     ladder; above the approach ceiling no measured decode has ever existed and the arm
     *     is refused by name. The ceiling is the ONLY conjunct the approach relaxes — every
     *     other gate on this list binds identically at 10 m and at 5 m;
     *  4. **the camera at nadir** — the *commanded* pitch within
     *     [TagDescentGuidance.NADIR_TOLERANCE_DEG] of −90°, because `TagWorld.fix` refuses fixes
     *     beyond the same tolerance and the geometry this stage flies on assumes a plumb ray;
     *  5. **the engine available** — SDK reachable, RC feed alive (abort gesture 1 must not be
     *     blind), no other manoeuvre flying (a descent does not silently cancel a mission —
     *     refused, where the other manoeuvres replace, because arming comes from a different
     *     surface than the thing it would cancel), and the arming controller alive **on its own
     *     origin's semantics**, since the descent inherits the engine's Q4 link watchdog rather
     *     than inventing its own — a MAVLINK arm needs QGC's traffic; a phone arm is alive by
     *     identity and the rung never refuses it (landing08's six `LINK_DOWN` denials,
     *     t=51.4–76.9, were this rung reading the wrong origin);
     *  6. **the ordinary flying gates** — fresh position fix, a home point (the fix's
     *     north/east frame has no origin without one), usable altitude.
     *
     * The interlock is the one exception to "every refusal has a sentence", on the
     * `CommandDispatcher.onTakeoff` precedent: off answers `UNSUPPORTED` silently. The phone's
     * arm control is itself gated on the interlock, so reaching this state takes a race, and
     * the reply is then byte-for-byte the no-such-feature reply.
     *
     * ## Rule 1, stated where the arming happens
     *
     * **Manual stick takeover cancels the descent completely.** Physical RC deflection past
     * [RC_ABORT_DEFLECTION] (10 % of travel — transmitter slop and a nudged table sit well
     * under it, a real grab is tens of percent in the first moment) sustained
     * [RC_ABORT_SUSTAIN_MS] fires the engine's existing abort ladder, which sits **above** the
     * descent's tick branch and [abort] nulls the descent state before any side effect. There
     * is no resume path anywhere in this feature: every cancel drops [TagDescentRun], whose
     * [TagDescent] machine dies with it, and the only way back down is this method, through
     * every gate, on a fresh operator act.
     */
    fun armTagDescent(
        /**
         * The door the arm came in through, within which the engagement's liveness is judged.
         * Defaults to [ControlOrigin.MAVLINK] because that is the fail-safe label (its liveness
         * demands heartbeat evidence; [ControlOrigin.PHONE]'s is granted by identity), so the
         * future QGC-side handler this seam exists for gets the watchdog even if it forgets to
         * say — the phone door is [armTagDescentFromPhone], which names itself.
         */
        origin: ControlOrigin = ControlOrigin.MAVLINK,
        /**
         * Stage C: the operator's explicit full-autoland toggle, read at the moment of their
         * arm press and pinned into the run. Off by default — an arm that does not say
         * otherwise is exactly yesterday's Stage B and stops at the terminal hold. Design
         * authority: Ivan, 2026-07-28 (*"let's implement first pass on full landing"*), the
         * deliberate decision `landingdata.md` §4 Option 1 requires.
         */
        fullAutoland: Boolean = false,
    ): Verdict {
        if (!interlockEnabled()) {
            log("tag descent arm with interlock off — refused as before (UNSUPPORTED)")
            return Verdict.UNSUPPORTED
        }
        val now = nowMs()
        val gate: DescentGate
        var engage = false
        var resumed = false
        var shadowModeEnded = false
        synchronized(lock) {
            gate = descentGateLocked(tagSense?.invoke(), origin, now)
            if (gate is DescentGate.Clear) {
                // A live arm supersedes shadow mode, recorded: two recorders of the same tag
                // would interleave confusingly, and the operator has escalated past watching.
                shadowModeEnded = shadowDescent != null
                shadowDescent = null
                tagDescent = TagDescentRun(
                    tagId = gate.tagId,
                    acceptedAtMs = now,
                    fixNorthM = gate.fixNorthM,
                    fixEastM = gate.fixEastM,
                    fixAtMs = now - gate.fixAgeMs,
                    fixTagRangeM = gate.fixTagRangeM,
                    fixRangeSource = gate.fixRangeSource,
                    fullAutoland = fullAutoland,
                    approach = gate.approach,
                )
                engagementOrigin = origin
                when (phaseLocked) {
                    GuidedPhase.IDLE -> {
                        phaseLocked = GuidedPhase.ENGAGING
                        engageStartedAtMs = now
                        lastEngageAttemptAtMs = now
                        engage = true
                    }

                    GuidedPhase.ENGAGING -> Unit // the engage in flight serves this descent instead
                    GuidedPhase.ENGAGED -> Unit // authority reused

                    GuidedPhase.RELEASING -> {
                        phaseLocked = GuidedPhase.ENGAGED
                        engagedAtMs = now
                        clearReleaseLocked()
                        resumed = true
                    }
                }
            }
        }
        return when (gate) {
            is DescentGate.Blocked -> denyDescent(gate.reason, gate.detail)

            is DescentGate.Clear -> {
                if (shadowModeEnded) {
                    log("shadow descent mode ended: replaced by a live arm")
                    record.event(EventCode.TAG_DESCENT_PHASE, "shadow mode off (live arm)")
                }
                log(
                    "tag descent armed: id=%d height=%.1fm fix N%+.2f E%+.2f age=%dms%s%s%s".format(
                        gate.tagId, gate.altitudeM, gate.fixNorthM, gate.fixEastM, gate.fixAgeMs,
                        if (fullAutoland) " AUTOLAND" else "",
                        if (gate.approach) " APPROACH (above the band)" else "",
                        if (resumed) " (resumed from wind-down)" else "",
                    )
                )
                record.event(
                    EventCode.TAG_DESCENT_ARMED,
                    // The origin is on the record for the same reason the climb's arm carries
                    // it: landing08's diagnosis needed the label and did not have it. The
                    // approach marker rides beside the entry height for the same reason: the
                    // next flight's record must show where the approach began without a join.
                    "id=%d height=%.1f fixAge=%d origin=%s%s%s".format(
                        gate.tagId, gate.altitudeM, gate.fixAgeMs, origin.name.lowercase(),
                        if (fullAutoland) " autoland" else "",
                        if (gate.approach) " approach" else "",
                    ),
                )
                announce(
                    if (fullAutoland) {
                        GuidedStatusTexts.DESCENT_ARMED_AUTOLAND
                    } else {
                        GuidedStatusTexts.DESCENT_ARMED
                    }
                )
                // The approach says its own word beside the regime's (the RESUMED pattern):
                // an operator arming at 10 m must hear that the machine took the height and
                // is flying down to the band, not wonder whether the press half-applied.
                if (gate.approach) announce(GuidedStatusTexts.DESCENT_APPROACH)
                if (resumed) announce(GuidedStatusTexts.RESUMED)
                if (engage) beginEngage()
                Verdict.ACCEPTED
            }
        }
    }

    /**
     * **The phone's descent arm door** — [armTagDescent] with the origin this door *is*, named
     * here in engine-tested code rather than in `Bridge`'s Android-side routing so the label is
     * pinned by `GuidedTagDescentTest`'s mutation table (the `CommandDispatcher.takeoffFromPhone`
     * pattern: the corridor is shared, the door names itself). The decision is entirely
     * [armTagDescent]'s; nothing else differs.
     *
     * The origin is the whole reason this method exists: a phone arm judged as MAVLINK demands a
     * QGC heartbeat the phone-only flight never has — landing08's six `tag_descent_denied
     * LINK_DOWN` refusals (`datasets/landing08/20260729-112216.001.jsonl`, t=51.4–76.9) are that
     * mislabel, measured.
     */
    fun armTagDescentFromPhone(fullAutoland: Boolean = false): Verdict =
        armTagDescent(ControlOrigin.PHONE, fullAutoland)

    /** What the arm gates decided — one implementation, two callers ([armTagDescent], shadow). */
    private sealed interface DescentGate {
        data class Blocked(val reason: String, val detail: String) : DescentGate

        data class Clear(
            val tagId: Int,
            val fixNorthM: Double,
            val fixEastM: Double,
            val fixAgeMs: Long,
            val altitudeM: Double,
            /** The arming fix's range facts, seeded into the run beside its north/east — one
             * fix, all its facts together, so the first tick's height ladder reads the same
             * frame the arm believed rather than nulls until the next ingest. */
            val fixTagRangeM: Double? = null,
            val fixRangeSource: RangeSource? = null,
            /** The arm is above the band (and under the approach ceiling): begin in APPROACH. */
            val approach: Boolean = false,
        ) : DescentGate
    }

    /**
     * **The descent's arm gates, in one place.** Must hold [lock].
     *
     * Shared verbatim by [armTagDescent] and the shadow's auto-arm ([tickShadowLocked]) — the
     * sharing is what makes shadow evidence transferable to live: a shadow segment arms at
     * exactly the moment a live arm would have been accepted, gate for gate, number for
     * number, because there is one implementation for both to disagree with.
     *
     * The interlock is deliberately **not** here: the live arm answers `UNSUPPORTED` for it
     * (the pre-feature reply, no sentence) while the shadow records it as a named blocker, and
     * those are different acts on the same fact, taken by the caller that owns each.
     *
     * @param ignoringOwnMission **read-only lookahead, and never a path that arms anything.** True
     *   only for the precision-land sequence's per-tick poll ([tickMissionLandTagLocked]'s
     *   `ACQUIRE` phase), which is a mission asking *"would the arm be accepted the instant I
     *   end?"* — a question it cannot ask otherwise, because [armLandTagLocked] must null `mission`
     *   under the lock **before** arming and that step is destructive. It removes exactly one
     *   disjunct of the busy conjunct, the caller's own mission; every other manoeuvre still
     *   blocks, and the arm itself goes through [armTagDescent] with the default, so the gate the
     *   aircraft is actually armed by is byte-for-byte the one the phone door uses. Weakening the
     *   busy conjunct on a path that *arms* is the mutation this design most needs to survive, and
     *   this parameter deliberately does not create one.
     */
    private fun descentGateLocked(
        sense: TagDescentSense?,
        origin: ControlOrigin,
        now: Long,
        ignoringOwnMission: Boolean = false,
    ): DescentGate {
        if (sense == null) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_NO_DETECTOR, "no tag pipeline wired this session",
            )
        }
        val latchedId = if (sense.latched) sense.latchedTagId else null
        if (latchedId == null) {
            return DescentGate.Blocked(GuidedStatusTexts.REASON_NO_TAG, "no tag latched this flight")
        }
        // **The camera before the fix, deliberately — cause before symptom.** `TagWorld.fix`
        // refuses every sighting whose believed pitch is absent or off nadir, so a bad camera
        // *manufactures* a missing fix: judging fix freshness first names the symptom, and on
        // 2026-07-28 that cost real flights — two sessions of `tag_descent_denied
        // TAG_NOT_IN_VIEW` while sightings flowed at 10 Hz and the last was 20 ms old, because
        // the pitch had never been commanded and nothing said so (records 20260728-213858,
        // -214210). Reordered 2026-07-28: a camera problem now answers CAMERA_NOT_NADIR, and
        // TAG_NOT_IN_VIEW is reserved for the case its words describe — camera at nadir, tag
        // genuinely not believed in view now. Shared verbatim with the shadow path like every
        // gate here, so shadow blockers name the same truth.
        val pitch = cameraPitchDeg()
        if (pitch == null || !pitch.isFinite() ||
            abs(pitch - TagDescentGuidance.NADIR_PITCH_DEG) > TagDescentGuidance.NADIR_TOLERANCE_DEG
        ) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_NOT_NADIR,
                "believed camera pitch ${pitch ?: "unknown - never commanded or reported"} " +
                    "is not within ${TagDescentGuidance.NADIR_TOLERANCE_DEG}deg of nadir",
            )
        }
        val fixNorth = sense.fixNorthM
        val fixEast = sense.fixEastM
        val fixAge = sense.fixAgeMs
        if (fixNorth == null || fixEast == null || fixAge == null || sense.fixTagId != latchedId ||
            fixAge > TagDescentGuidance.ARM_FRESH_MS
        ) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_TAG_STALE,
                "newest fix ${fixAge?.let { "${it}ms old" } ?: "absent"} " +
                    "(id ${sense.fixTagId}) against bound ${TagDescentGuidance.ARM_FRESH_MS}ms",
            )
        }

        // Dynamic gates, freshest last — the same ladder, in the same order, as `acceptTarget`.
        port.unavailableReason()?.let { reason ->
            return DescentGate.Blocked(reason, "SDK unavailable: $reason")
        }
        if (rc?.allPresent() != true) {
            return DescentGate.Blocked(
                "NO_RC_FEED", "RC stick feed not delivering — abort gesture 1 would be blind",
            )
        }
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_NO_FIX, "no fresh position fix to centre from",
            )
        }
        if (Geo.coordinateOrNull(state.homeLatitude, state.homeLongitude) == null) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_NO_HOME,
                "no home point — the fix's north/east frame has no origin",
            )
        }
        val altitude = usableAltitude(state)
            ?: return DescentGate.Blocked(
                GuidedStatusTexts.REASON_NO_DATUM,
                "no usable height above the takeoff datum — neither the cone nor the ceiling can be evaluated",
            )
        // The ceiling conjunct, two-tiered since the approach (Ivan's brief, 2026-07-29,
        // after landing13 t=41.8 measured the friction): below ARM_CEILING_M is today's arm,
        // unchanged; between it and APPROACH_CEILING_M the arm is accepted into the APPROACH
        // segment — every other gate above and below this line still binding, because the
        // approach weakens the ceiling and nothing else; above APPROACH_CEILING_M no
        // measured decode has ever existed and a fresh fix claimed from there means an
        // instrument is lying (the constant's KDoc carries the decode-reach evidence).
        if (altitude > TagDescentGuidance.APPROACH_CEILING_M) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_APPROACH_CEILING,
                "%.1fm is above the %.0fm ceiling every measured decode sits under"
                    .format(altitude, TagDescentGuidance.APPROACH_CEILING_M),
            )
        }
        if (reposition != null || orbit != null ||
            (mission != null && !ignoringOwnMission) || takeoffClimb.armed ||
            tagDescent != null
        ) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_DESCENT_BUSY,
                "another manoeuvre is flying — a descent does not silently cancel it",
            )
        }
        // The descent inherits the Q4 link watchdog (no second watchdog), so it must not be
        // born already half-expired: the *arming* controller has to be alive on its own origin's
        // semantics — the same owner (controllerSeenAtLocked) the mid-descent watchdog reads, so
        // the gate and the tick cannot disagree about what alive means. A MAVLINK arm needs
        // QGC's traffic within the bound; a PHONE arm is alive by identity and this rung never
        // refuses it — landing08 measured the alternative: six phone arms refused LINK_DOWN
        // (t=51.4–76.9) on a flight whose commander was the operator's own hand.
        val seenAt = controllerSeenAtLocked(origin, now)
        if (seenAt == null || now - seenAt > GuidedEnvelope.LINK_LOST_MS) {
            return DescentGate.Blocked(
                GuidedStatusTexts.REASON_LINK_DOWN,
                "commanding controller not heard from within ${GuidedEnvelope.LINK_LOST_MS}ms",
            )
        }
        return DescentGate.Clear(
            latchedId, fixNorth, fixEast, fixAge, altitude,
            fixTagRangeM = sense.fixTagRangeM,
            fixRangeSource = sense.fixRangeSource,
            approach = altitude > TagDescentGuidance.ARM_CEILING_M,
        )
    }

    /**
     * **Disarm, from the phone — a withdrawal, and therefore takes no origin** (the [pause]
     * argument verbatim: there is no circumstance in which this bridge should decline to stop
     * descending). The descent is dropped — never suspended — and the aircraft keeps station
     * exactly as a paused goto does, still engaged, with Q1's idle clock running. Safe from any
     * thread and idempotent: with nothing armed it answers `DENIED`, which is the truth.
     */
    fun disarmTagDescent(): Verdict {
        val now = nowMs()
        var found = false
        var wasCommitted = false
        var withdrawnBeforeEngage = false
        synchronized(lock) {
            val d = tagDescent ?: return@synchronized
            tagDescent = null
            found = true
            wasCommitted = d.machine.phase == TagDescentPhase.DJI_LANDING
            when (phaseLocked) {
                GuidedPhase.ENGAGED -> keepStationLocked(now)

                GuidedPhase.ENGAGING -> {
                    phaseLocked = GuidedPhase.IDLE
                    withdrawnBeforeEngage = true
                }

                else -> Unit // IDLE cannot hold a descent; RELEASING is already stopping
            }
        }
        if (!found) {
            log("tag descent disarm with nothing armed")
            return Verdict.DENIED
        }
        log("tag descent disarmed by the operator")
        record.event(EventCode.TAG_DESCENT_ENDED, "disarmed")
        // The operator's own stop of a committed landing withdraws DJI's landing too — a
        // disarm that left the aircraft autonomously descending would be a stop that did not
        // stop, the project's characteristic failure shape.
        if (wasCommitted) stopDjiLanding("disarmed")
        if (withdrawnBeforeEngage) requestDisable()
        announce(GuidedStatusTexts.DESCENT_DISARMED)
        return Verdict.ACCEPTED
    }

    /**
     * **"Is this bridge flying a tag landing right now?"** — read by `Bridge`'s [FlightView]
     * projection and by nothing else, so that `vision/TagArming` can run the detector for a
     * landing this engine has committed to *even with nothing latched* and *even after the
     * acquisition band has closed*. Safe from any thread; a flat boolean out, nothing back in
     * (the `situation` discipline).
     *
     * ## Why it is derived and never latched
     *
     * The requirement is that it **ends when the landing run ends** — refused, aborted, or touched
     * down — and never one second later: a flag that latched on would leave 0.68 cores running for
     * the rest of a flight that had already given up on landing, which is precisely the failure
     * this rule exists to prevent, wearing the other face. A latch would need clearing at every
     * one of the *thirteen* places [tagDescent] is nulled plus every mission ending, and a
     * property with thirteen owners has none. So there is no field: this reads the two objects
     * whose lifetimes already *are* the landing, and both are dropped by every ending they have.
     *
     *  - **[LandTagRun.nadirAsked]** — true from the tick the precision-`NAV_LAND` sequence
     *    commands the camera to nadir (`PrecisionLand.Phase.AIMING`), which is the moment the
     *    aircraft is committed to going down onto the marker and, deliberately, well above the
     *    arm height: the detector wants the whole descent from the item's altitude to look, which
     *    is what makes the arm's own fresh-fix conjunct satisfiable when it arrives. The run is
     *    nulled with `mission` by every refusal, phase timeout and abort.
     *  - **[tagDescent] non-null** — the armed descent itself, from either door. For the *phone*
     *    door this changes nothing that a latch was not already granting below the ceiling, and it
     *    closes one real hole above it: an arm taken into `TagDescentPhase.APPROACH` between the
     *    detector's ceiling and `TagDescentGuidance.APPROACH_CEILING_M` used to descend with the
     *    detector off until it fell back under the ceiling.
     *
     * **The one gap, measured in microseconds and named rather than papered over:** at the
     * hand-off [armLandTagLocked] nulls `mission` under the lock and calls [armTagDescent] from
     * the effect list, so between the lock release and the arm this answers false. The detector
     * worker re-evaluates on a 250 ms idle wait, so the worst case is one arming decision computed
     * in that window — a disarm/re-arm pair in the log, no frames lost that the tick after does not
     * recover. Closing it would mean a flag spanning the two objects, which is the latch this
     * design refuses.
     */
    fun landingOnTag(): Boolean = synchronized(lock) {
        tagDescent != null || mission?.land?.nadirAsked == true
    }

    /**
     * **Stage C's answer to "may the landing be confirmed right now?"** — read by
     * `MsdkFlightActions` at the instant DJI raises `KeyIsLandingConfirmationNeeded`, and by
     * nothing else. Null unless a **live full-autoland engagement is committed to LANDING at
     * this moment**, which makes null the strong statement: nothing this engine flies wants a
     * confirmation, so a question that arrives anyway is someone else's landing. Rule 1 keeps
     * this honest without a subscription — any stick grab nulls [tagDescent] under [lock]
     * before this method could read it.
     *
     * Safe from any thread; a flat copy out, nothing back in (the [situation] discipline).
     */
    fun autolandClearance(): com.dimensional.mini4pro.command.AutolandClearance? =
        synchronized(lock) {
            val d = tagDescent ?: return null
            if (!d.fullAutoland || d.machine.phase != TagDescentPhase.DJI_LANDING) return null
            com.dimensional.mini4pro.command.AutolandClearance(
                engagementAtMs = d.acceptedAtMs,
                fixAgeMs = nowMs() - d.fixAtMs,
                fixWasInCone = d.lastFreshFixInCone,
            )
        }

    /**
     * DJI accepted the guided landing confirm — `MsdkFlightActions` reports it from the
     * confirm's own `onSuccess`, and the fact rides the run so a rule-1 cancel after this
     * point can say what the operator must know: **DJI may complete the landing on its own**
     * (whether it does is one of the facts the Stage C flight measures). Recorded on the
     * `tag_descent_phase` timeline; a note on a run that has already died is dropped, because
     * a dead engagement has no timeline to amend.
     */
    fun noteAutolandConfirmed() {
        val noted = synchronized(lock) {
            tagDescent?.takeIf { it.machine.phase == TagDescentPhase.DJI_LANDING }
                ?.also { it.djiConfirmed = true } != null
        }
        if (noted) {
            log("DJI accepted the autoland confirm — the FC may now finish the landing itself")
            record.event(EventCode.TAG_DESCENT_PHASE, "landing dji-confirmed")
        }
    }

    /**
     * **"Stop where you are and stay there"** — a station-keeping hold installed at the aircraft's own
     * current position, still engaged, with Q1's idle clock running from now. Must hold [lock]. Returns
     * false when there was nothing to hold *with*: not engaged, or no fresh fix to hold onto.
     *
     * One implementation for every caller that ends a manoeuvre **without** ending the engagement, so
     * "the aircraft holds where it is" means one thing: [disarmTagDescent]'s withdrawal (whose code
     * this was), and every refusal of the precision `NAV_LAND` sequence. A hold expressed as an
     * *arrived reposition* rather than as a special phase is what makes it inherit the reposition tick's
     * own watchdogs unchanged — the link watchdog, the position feed as eyes, and the idle window that
     * eventually gives the aircraft back.
     *
     * The false return is not absorbed by its callers: with no manoeuvre and no stick stream the engaged
     * tick's own fail-closed branch winds authority down, and DJI holds position on its own — which is
     * the right outcome for a bridge that has just discovered it cannot see where the aircraft is.
     */
    private fun keepStationLocked(now: Long): Boolean {
        if (phaseLocked != GuidedPhase.ENGAGED) return false
        val state = aircraftState()
        val here = Geo.coordinateOrNull(state.latitude, state.longitude) ?: return false
        if (!state.isFresh(Signal.POSITION)) return false
        reposition = RepositionState.holdAt(
            here.first, here.second,
            relAltM = state.relativeAltitude ?: 0.0,
            nowMs = now,
        )
        return true
    }

    /** One descent refusal: log, flight record, sentence, and the `DENIED` behind them. */
    private fun denyDescent(reason: String, detail: String): Verdict {
        log("tag descent denied: $detail")
        record.event(EventCode.TAG_DESCENT_DENIED, reason, warn = true)
        announce(GuidedStatusTexts.descentRefused(reason))
        return Verdict.DENIED
    }

    // --------------------------------------------------------------- DO_SET_ROI

    /**
     * **The operator clicks a point on QGC's map and the camera holds it.** `DO_SET_ROI_LOCATION`
     * (195) sets it, `DO_SET_ROI_NONE` (197) clears it, and the legacy `DO_SET_ROI` (201) carries
     * either intent in its own spelling. Runs on the `mavlink-rx` thread; the return value **is**
     * the `COMMAND_ACK`, under the ordering rule Stage B wrote and Stage C repeated: `ACCEPTED` if
     * and only if the target was actually taken, everything else `DENIED` **with a sentence**.
     *
     * The ack is not only an ack here. `Vehicle::_handleCommandAck` flips QGC's own `isROIEnabled`
     * on our `ACCEPTED` and **draws the ROI marker on its map**, so accepting an ROI we cannot point
     * at would put a marker on the operator's screen for a camera aimed somewhere else. That is why
     * a bridge with no camera at all refuses rather than accepting politely.
     *
     * ## An ROI is a modifier, not a manoeuvre — and that decides the gates
     *
     * Nothing about the flight path changes. A goto with an ROI flies the goto; an orbit with an
     * explicit ROI flies the circle and points at the ROI instead of at its own centre (they agree
     * when the ROI *is* the centre, and then there is nothing to say). Setting one does not engage,
     * does not require an engagement, and cannot start the aircraft moving.
     *
     * So the manoeuvre gates are deliberately **absent**: no fix gate (the target is a place, not a
     * leg — whether we can currently see it is a per-tick question and the honest answer to "no fix"
     * is to stop moving the camera, not to refuse the click), no RC-feed gate (abort gesture 1
     * guards *flying*, and this is not flying), no datum gate (the commanded `z` is discarded, so
     * there is nothing to interpret against a datum), and **no interlock gate**.
     *
     * ### The interlock, stated in full because it is the one that looks wrong
     *
     * Aiming a camera is not flying an aircraft, and manual gimbal aiming is deliberately not
     * interlock-gated in this project today. An ROI's *camera* half is exactly that — an absolute
     * pitch, sent to a component that cannot move the aircraft — so gating it would be a new
     * restriction argued for by nothing.
     *
     * An ROI's *yaw* half **is** flight control, and it is behind the interlock **by construction
     * rather than by a second gate**: yaw is only ever commanded from inside the two manoeuvre
     * ticks, which can only run while this engine holds virtual-stick authority, which can only be
     * obtained with the interlock on and is dropped the tick it goes off. A second gate here would
     * be a second place for the same property to be got wrong, and the first one to rot.
     *
     * ### Clearing is always allowed
     *
     * `DO_SET_ROI_NONE` is accepted in **every** state, ahead of every check including the shape
     * checks: turning something off is always allowed, and a refusal to stop pointing would be the
     * one refusal an operator can do nothing about.
     *
     * ## Two doors, one owner (2026-07-30)
     *
     * This function is the **wire** door. A **plan's** `DO_SET_ROI_LOCATION` / `DO_SET_ROI_NONE` item is
     * the second one, and it enters at [applyRoi] — everything below the liveness stamp, which is to say
     * every gate, every state write, every record line and every sentence. The route carries the item as
     * a [RoiCommand] ([MissionStep.roi]) precisely so that there is nothing else to share; the mission
     * grows no ROI state of its own, no second aiming path and no second clearing path, and
     * [applyMissionRoiLocked] is thirty lines of *when*, not of *what*.
     */
    fun roi(cmd: RoiCommand, origin: ControlOrigin = ControlOrigin.MAVLINK): Verdict {
        // As `reposition` and `orbit`: the command is itself evidence this controller is alive.
        //
        // **This line is the whole difference between the two doors, and it is why the plan's door
        // cannot be this function.** A stamp here is a statement that traffic arrived from [origin] —
        // the Q4 watchdog's only evidence. A mission item calling this would refresh the commanding
        // controller's liveness *from inside the aircraft*, so a mission whose GCS had died would be
        // kept alive by its own ROI legs: the watchdog would be watching itself. Everything below,
        // which is every gate, every state write, every record line and every sentence, is shared.
        synchronized(lock) { gcsSeenAtMs[origin] = nowMs() }
        return applyRoi(cmd, missionSeq = null)
    }

    /**
     * **The ROI itself — one implementation, two doors.** Called by [roi] for a live command and by
     * [applyMissionRoiLocked]'s effect for a plan's `DO_SET_ROI_*` item. Must **not** hold [lock]
     * (it takes it itself, and it announces).
     *
     * @param missionSeq the plan item's wire `seq` when this came from a route, null for a live
     *   command. It reaches the flight record and [RoiState.missionSeq]; it changes no gate, because a
     *   plan's ROI is not more or less trusted than an operator's — the same target, the same camera.
     *
     * The gates, the ack semantics and the whole interlock argument are [roi]'s KDoc, which is the
     * design authority for both doors.
     */
    private fun applyRoi(cmd: RoiCommand, missionSeq: Int?): Verdict {
        val from = missionSeq?.let { " (plan item $it)" } ?: ""
        val seqTail = missionSeq?.let { " seq=$it" } ?: ""

        if (cmd.isClear()) {
            // **Cancel means cancel.** Clears the operator's own target *and* the one a circle
            // implies at its centre, rather than peeling off one layer and leaving the other
            // pointing. Ivan, 2026-07-27, and it supersedes the earlier rule that a clear during an
            // orbit fell back to the centre: two layers you have to press through twice is the kind
            // of cleverness that surprises a pilot mid-flight.
            val had = synchronized(lock) { clearRoiTargetsLocked() }
            log("DO_SET_ROI_NONE$from: ${if (had) "target cleared" else "nothing was set"}")
            record.event(
                EventCode.ROI_CLEARED,
                (if (had) "commanded" else "commanded (none set)") + seqTail,
            )
            // Announced either way. "There was nothing to clear" is not worth a different sentence
            // — the operator asked for the camera to stop tracking and it is not tracking.
            announce(GuidedStatusTexts.ROI_CLEARED)
            return Verdict.ACCEPTED
        }

        // One refusal shape for both doors: the same reason word, the same sentence, the same
        // `roi_denied` line — with the plan item named when there is one. A plan whose ROI this bridge
        // cannot point at is refused **by name on the record and on the screen**, and the mission flies
        // on: an ROI is a modifier, so failing to take one is not a reason to stop flying the route.
        val deny = { reason: String, detail: String -> denyRoi(reason, detail + from, seqTail) }

        // Static shape checks — anything off the measured wire shape is refused, never interpreted.
        if (cmd.command == RoiCommand.MAV_CMD_DO_SET_ROI_LOCATION && !cmd.isCommandInt) {
            return deny(
                GuidedStatusTexts.REASON_ROI_LONG_FORM,
                "COMMAND_LONG 195 — QGC sends COMMAND_INT; the float-coordinate shape is unmeasured",
            )
        }
        if (cmd.command == RoiCommand.MAV_CMD_DO_SET_ROI &&
            cmd.param1.isFinite() &&
            cmd.param1.toInt() != RoiCommand.MAV_ROI_LOCATION
        ) {
            // MAV_ROI_WPNEXT, _WPINDEX and _TARGET are different intents — a moving target, or one
            // named by a mission index — and none of them is a place on the ground. Refused rather
            // than pointed approximately at, which is what interpreting them as a location would be.
            return deny(GuidedStatusTexts.REASON_ROI_MODE, "param1=${cmd.param1} is not MAV_ROI_LOCATION")
        }
        if (cmd.isCommandInt && cmd.frame !in RoiCommand.GLOBAL_FRAMES) {
            return deny("FRAME_${cmd.frame}", "frame ${cmd.frame} does not carry 1e7-degree x/y")
        }
        val target = cmd.targetDegrees()
        val point = target?.let { Geo.coordinateOrNull(it.first, it.second) }
        if (point == null) {
            return deny(GuidedStatusTexts.REASON_BAD_TARGET, "x=${cmd.latE7} y=${cmd.lonE7} is not a coordinate")
        }
        if (manoeuvreGimbal == null) {
            // Accepting would draw QGC's ROI marker for a camera that does not exist.
            return deny(GuidedStatusTexts.REASON_ROI_NO_GIMBAL, "no camera attached to point anywhere")
        }

        val now = nowMs()
        // The height, when the frame carries one we can read — see `RoiCommand.relativeAltMOrNull`.
        val targetAltM = cmd.relativeAltMOrNull()
        val replaced = synchronized(lock) {
            val had = roiTrackingLocked() != null
            roi = RoiState(
                point.first, point.second, acceptedAtMs = now, relAltM = targetAltM,
                missionSeq = missionSeq,
            )
            // The explicit one **replaces** the circle's rather than sitting on top of it, so that
            // clearing it later means nothing is tracked rather than silently reverting to a centre
            // the operator stopped thinking about several manoeuvres ago.
            orbitRoi = null
            // A new place to look at starts the limiter afresh, so the first command of the new
            // target is never swallowed by the last target's deadband.
            gimbalLastAtMs = null
            gimbalLastPitchDeg = null
            had
        }
        log(
            "DO_SET_ROI accepted$from: lat=%.7f lon=%.7f %s%s".format(
                point.first, point.second,
                targetAltM?.let { "target %.1fm above the takeoff datum (frame %d)".format(it, cmd.frame) }
                    ?: "(z discarded — target assumed at the takeoff datum)",
                if (replaced) " (replaces previous ROI)" else "",
            )
        )
        record.event(
            EventCode.ROI_ACCEPTED,
            "lat=%.7f lon=%.7f %s%s".format(
                point.first, point.second,
                targetAltM?.let { "relAlt=%.1f".format(it) } ?: "relAlt=none",
                seqTail,
            ),
        )
        announce(GuidedStatusTexts.ROI_STARTED)
        // Once per ROI, and the whole reason it is once per ROI rather than once per tick: the
        // operator has to know **which** of the two things happened to the height they clicked on.
        // A frame we can read is honoured and said so; anything else is the measured discard.
        announce(
            if (targetAltM != null) GuidedStatusTexts.roiTargetHeight(targetAltM)
            else GuidedStatusTexts.ROI_GROUND_LEVEL
        )
        return Verdict.ACCEPTED
    }

    /**
     * **Stop pointing at anything** — the operator's own target *and* the one a circle implies at its
     * centre. Must hold [lock]. Returns whether anything was actually being tracked.
     *
     * The single owner of "clear the ROI", shared by `DO_SET_ROI_NONE` (whose code this was) and the
     * precision `NAV_LAND` sequence, which must clear it before it can point the camera at nadir —
     * [updateRoiCameraLocked] runs on every engaged tick and would otherwise re-aim the gimbal at the
     * ROI forever. Both peel off **both** layers, because Ivan's 2026-07-27 rule is that cancel means
     * cancel: two layers you have to press through twice is the kind of cleverness that surprises a
     * pilot mid-flight.
     *
     * **The camera is left exactly where it is**, in both cases and deliberately: no recentring, no
     * stowing. `docs/gimbal.md`'s argument is that a silent slew is the wrong default, and it is why the
     * landing sequence has to command nadir explicitly rather than expecting a clear to have done it.
     */
    private fun clearRoiTargetsLocked(): Boolean =
        (roiTrackingLocked() != null).also { roi = null; orbitRoi = null }

    /**
     * **A plan's ROI items, executed** — the mission's door onto [applyRoi]. Must hold [lock].
     *
     * Called at Start and on every cursor advance, and it does exactly one thing: compare the ROI the
     * route says is in force at the cursor ([MissionStep.roi], the store's sticky resolution) with what
     * this run last applied, and push the difference through the live ROI door as an effect. Nothing
     * else in the mission path touches [roi], [orbitRoi] or the gimbal.
     *
     * ## The sequencing, stated once
     *
     * A `DO_` item acts **when the sequence reaches it** — the same rule `DO_CHANGE_SPEED` rides
     * through the same resolution walk. So an ROI item sitting between waypoint N and waypoint N+1 is
     * in force for the leg **to** N+1 and for every leg after it, until a `DO_SET_ROI_NONE`, a
     * different ROI item, or the plan ending. On `big1.plan` (item 6 the ROI, item 8 the clear) that is
     * Ivan's expectation verbatim: *"after wp6 it looks at roi, at wp8 it should stop"* — the camera
     * comes on as the leg to item 7 begins, and comes off when item 7 is reached and the cursor moves
     * to the item-9 landing.
     *
     * **Off-by-one is the failure this shape exists to prevent**: applying the ROI at the item's own
     * `seq` would need a step for a non-navigable item, and applying it one leg late would point the
     * camera at the target only after the aircraft had flown past it. Neither is expressible here,
     * because the only thing that can be applied is *the ROI the route says belongs to the step the
     * cursor is on*.
     *
     * ## The clear is unconditional, and that is a decision
     *
     * A plan's `DO_SET_ROI_NONE` clears whatever is in force — including an ROI the operator clicked
     * mid-mission. `docs/m4-mission-transport.md` §6.3 originally said the operator's live ROI would be
     * *restored* by such an item; that bullet is **superseded** by Ivan's 2026-07-27 rule that cancel
     * means cancel (which is why [clearRoiTargetsLocked] peels off both layers and why there is no
     * third one to stack a plan's ROI onto). Two layers of ROI that have to be pressed through twice is
     * the kind of cleverness that surprises a pilot mid-flight; one target, one clear.
     */
    private fun applyMissionRoiLocked(run: MissionRun, effects: MutableList<() -> Unit>) {
        val wanted = run.step.roi
        if (wanted == run.roiApplied) return
        val seq = run.step.seq
        run.roiApplied = wanted
        // `clearing()` rather than a bare call to the clear owner, so that a plan's "stop pointing"
        // travels the *same* path as QGC's ROI-off button: one record line, one sentence, one owner.
        val cmd = wanted ?: RoiCommand.clearing()
        effects += { applyRoi(cmd, missionSeq = seq) }
    }

    /**
     * **What a mission's ending does to the camera.** Must hold [lock].
     *
     * Called from every path that drops a [MissionRun] — the plan completing, [endMissionLocked]'s
     * pauses, the tag landing's refusals, and [abort]'s ladder — because *"the mission must not end
     * with the gimbal wherever an ROI left it, silently"* is a property of endings, and a rule that has
     * to be remembered at five sites is a rule that will be forgotten at one of them.
     *
     * Two cases, and the distinction is [RoiState.missionSeq]:
     *
     *  - **the ROI in force is the plan's** — it dies with the plan that asked for it
     *    (`docs/m4-mission-transport.md` §6.3), and the ending is recorded and announced. A paused
     *    mission that is later resumed re-acquires it from the cursor's own step, so nothing is lost by
     *    letting go here, and a plan target the operator can no longer see is not left driving the
     *    camera;
     *  - **the ROI in force is the operator's own click** (or an orbit's centre) — **untouched**. It
     *    outlives every manoeuvre by the rule [abort] has honoured since 2026-07-27, and a mission
     *    ending is not a reason to take away something they asked for by hand.
     *
     * **The gimbal is never moved here**: no recentring, no stowing, no nadir. `docs/gimbal.md`'s rule
     * is that a silent slew is the wrong default and this project's rule is that a camera is never swung
     * to a default nobody asked for — so the camera stays exactly where the plan left it and the
     * operator is *told* that, which is the honest half of the two.
     */
    private fun endPlanRoiLocked(run: MissionRun, effects: MutableList<() -> Unit>) {
        val seq = roi?.takeIf { !it.implied }?.missionSeq
        run.roiApplied = null
        if (seq == null) return
        clearRoiTargetsLocked()
        effects += {
            log("mission ended with the plan's ROI (item $seq) still set — tracking stopped, camera left as it is")
            record.event(EventCode.ROI_CLEARED, "plan ended with the ROI set seq=$seq", warn = true)
            announce(GuidedStatusTexts.MISSION_ROI_ENDED)
        }
    }

    /**
     * One ROI refusal: log, flight record, `STATUSTEXT`, and the `DENIED` the modal is built from.
     *
     * [seqTail] is `" seq=N"` for a plan item and empty for a live command, so a post-flight reader can
     * tell which of a plan's ROI items this bridge refused to point at. The reason word itself is
     * unchanged either way: the operator reads why, not who asked.
     */
    private fun denyRoi(reason: String, detail: String, seqTail: String = ""): Verdict {
        log("DO_SET_ROI denied: $detail")
        record.event(EventCode.ROI_DENIED, reason + seqTail, warn = true)
        announce(GuidedStatusTexts.roiRefused(reason))
        return Verdict.DENIED
    }

    // ----------------------------------------------------------- DJI's own reports

    /** `VirtualStickState`, on DJI's thread. Confirmation is read by [tick]; loss aborts now. */
    private fun onVsState(snapshot: VirtualStickSnapshot) {
        val lost = synchronized(lock) {
            vs = snapshot
            (phaseLocked == GuidedPhase.ENGAGED || phaseLocked == GuidedPhase.RELEASING) &&
                !authorityOk(snapshot)
        }
        if (lost) abort(DisengageReason.AUTHORITY, snapshot.authority ?: "none")
    }

    /**
     * `FlightControlAuthorityChangeReason`, verbatim. Everything except our own
     * [OUR_AUTHORITY_REASON] ends the engagement — geofence takebacks, every `RC_*`, both
     * battery failsafes — because each one means someone with more standing than this bridge
     * is flying the aircraft now (Q3 gesture 3).
     */
    private fun onAuthorityReason(reason: String) {
        val active = synchronized(lock) { phaseLocked != GuidedPhase.IDLE }
        if (active && reason != OUR_AUTHORITY_REASON) {
            abort(DisengageReason.AUTHORITY, reason)
        }
    }

    /**
     * The RC stick feed. A null anywhere while we hold authority is DJI's component-gone
     * signal — the measured FC-blackout shape — and is treated as authority lost (landmine 6),
     * never as centred sticks. Deflection starts the debounce clock; [tick] reads it.
     */
    private fun onRcSticks(sticks: RcSticks) {
        var componentGone = false
        synchronized(lock) {
            rc = sticks
            val deflection = sticks.maxDeflection()
            when {
                !sticks.allPresent() -> {
                    rcDeflectedSinceMs = null
                    componentGone = phaseLocked != GuidedPhase.IDLE
                }

                deflection != null && deflection > RC_ABORT_DEFLECTION ->
                    if (rcDeflectedSinceMs == null) rcDeflectedSinceMs = nowMs()

                else -> rcDeflectedSinceMs = null
            }
        }
        if (componentGone) abort(DisengageReason.AUTHORITY, "RC_STICK_NULL")
    }

    // ---------------------------------------------------------------------- tick

    /**
     * One pass of the 10 Hz loop. All transitions and every setpoint originate here, from one
     * thread's point of view, which is what makes the state machine testable by hand-cranking
     * this method with a fake clock.
     */
    fun tick(now: Long = nowMs()) {
        val effects = ArrayList<() -> Unit>(3)
        synchronized(lock) {
            // The takeoff's pending second phase, **first and in every phase**. First, because a
            // tick that both revokes the intention and would have completed it must revoke it.
            // Every phase, because the intention outlives none of them and belongs to none of
            // them: it is armed while IDLE (DJI is flying its own takeoff), and it is the abort
            // ladder's business in all four. Costs one null check when nothing is armed, which is
            // almost always.
            tickTakeoffClimbLocked(now, effects)
            // Shadow mode, second and also in every phase, for the same structural reason: it
            // belongs to no engagement — its whole point is to run while the operator
            // hand-flies (IDLE) or flies the GCS sticks (ENGAGED passthrough) — and it
            // actuates nothing, so no phase has authority over it.
            shadowDescent?.let { tickShadowLocked(now, it, effects) }
            when (phaseLocked) {
                GuidedPhase.IDLE -> Unit
                GuidedPhase.ENGAGING -> tickEngagingLocked(now, effects)
                GuidedPhase.ENGAGED -> tickEngagedLocked(now, effects)
                GuidedPhase.RELEASING -> tickReleasingLocked(now, effects)
            }
            // The camera, in **every** phase including IDLE, and last.
            //
            // Every phase, because pointing a camera is not flying an aircraft: an ROI tracks
            // whoever has the aircraft, and when that is the RC pilot the tracking is pitch-only and
            // says so. Last, so that a tick which also aborts has queued the abort's effect first —
            // and the aim re-checks the suspension at run time, so the abort wins the race and no
            // gimbal command goes out on the tick the aircraft was handed back.
            updateRoiCameraLocked(now, effects)
        }
        for (effect in effects) effect()
    }

    private fun tickEngagingLocked(now: Long, effects: MutableList<() -> Unit>) {
        if (!interlockEnabled()) {
            effects += { abort(DisengageReason.INTERLOCK) }
            return
        }
        if (rcAbortDueLocked(now)) {
            effects += { abort(DisengageReason.RC_STICKS) }
            return
        }
        val snapshot = vs
        if (snapshot != null && authorityOk(snapshot)) {
            phaseLocked = GuidedPhase.ENGAGED
            engagedAtMs = now
            rangeRecorded = false
            // A remembered ROI is re-acquired by the manoeuvre that engages next — §9.5's "the ROI
            // target is remembered" — which is the moment authority is confirmed, not the moment it
            // was asked for.
            roi?.suspended = false
            orbitRoi?.suspended = false
            lastNonNeutralAtMs = now
            nonNeutralSinceMs = if (lastFrame?.let { !StickMapping.isNeutral(it) } == true) now else null
            val forOrbit = orbit != null
            val forGoto = reposition != null
            val forDescent = tagDescent != null
            effects += {
                log("ENGAGED — DJI reports enabled+advanced+MSDK; policy=${policy.name}")
                record.event(EventCode.GUIDED_ENGAGED, "policy=${policy.name}")
                announce(
                    when {
                        forDescent -> GuidedStatusTexts.ENGAGED_DESCENT
                        forOrbit -> GuidedStatusTexts.ENGAGED_ORBIT
                        forGoto -> GuidedStatusTexts.ENGAGED_GOTO
                        else -> GuidedStatusTexts.ENGAGED
                    }
                )
            }
            return
        }
        val started = engageStartedAtMs
        if (started != null && now - started > ENGAGE_CONFIRM_MS) {
            // A swallowed enable, an FC blackout, or authority that never became MSDK. The
            // request is withdrawn: engagement must not sit armed on hope (landmine 5).
            phaseLocked = GuidedPhase.IDLE
            val pendingGotoDropped = reposition != null
            val pendingOrbitDropped = orbit != null
            val pendingDescentDropped = tagDescent != null
            reposition = null
            orbit = null
            tagDescent = null
            effects += {
                log("engage unconfirmed after ${ENGAGE_CONFIRM_MS}ms — withdrawing")
                if (pendingGotoDropped) record.event(EventCode.GOTO_ENDED, "engage unconfirmed", warn = true)
                if (pendingOrbitDropped) record.event(EventCode.ORBIT_ENDED, "engage unconfirmed", warn = true)
                if (pendingDescentDropped) {
                    record.event(EventCode.TAG_DESCENT_ENDED, "engage unconfirmed", warn = true)
                }
                record.event(EventCode.VS_DISABLE_REQUEST, "engage timeout", warn = true)
                requestDisable()
                announce(GuidedStatusTexts.refused(GuidedStatusTexts.NO_CONFIRM))
            }
        }
    }

    /**
     * The mode DJI is flying if it has seized the aircraft out from under an engagement, or
     * null. DJI's forced manoeuvres (the measured one: overheat `GO_HOME`) do NOT change the
     * virtual-stick authority owner, so `VirtualStickState` stays silent while our setpoints
     * are ignored — the flight mode is the only wire that says so. Null mode stays with the
     * component-gone paths ([onRcSticks], `unavailableReason`), never read as seized.
     */
    private fun modeSeizedLocked(now: Long): String? {
        val mode = aircraftState().flightMode ?: return null
        if (mode == JOYSTICK_MODE) return null
        // **A mission's takeoff item, phase one.** DJI is flying the aircraft *because we asked it
        // to*, and `MOTOR_START`/`AUTO_TAKE_OFF` are the evidence that it is — the same readings
        // [TakeoffClimb] waits for. Read as a seizure they would abort the takeoff we just
        // commanded, 1.5 s in, on every mission that begins with one; and the mode is not
        // `JOYSTICK` for the whole of DJI's ~4.4 s hop even though our engagement is real.
        //
        // What is given up is bounded and small: for this phase alone the mode stops being an
        // authority signal. Everything else that would catch a genuine seizure is still watched —
        // the RC gesture, the interlock, `VirtualStickState`, and the leg timeout that ends the
        // phase after 30 s — and what we command throughout is a zero setpoint, so a DJI that had
        // truly taken over is not being fought. Phase two is deliberately **not** covered: once
        // DJI has let go, `JOYSTICK` is what the mode must become, and [MissionRun.climbing]
        // restamps [engagedAtMs] so the ordinary [MODE_SEIZE_GRACE_MS] window applies from there.
        if (missionAwaitingTakeoffLocked()) return null
        // **Stage C's twin of the takeoff exemption, in the other direction.** During a
        // committed LANDING the FC is EXPECTED to flip into its own landing mode — every
        // measured landing went straight to CONFIRM_LANDING while the descent command was the
        // pilot's held stick (`landingdata.md` §2.2), and the sustained virtual-stick down
        // this engine flies is that gesture mimicked. Reading the flip as a seizure would
        // abort the landing at the exact moment it is being confirmed. Scope, stated: only
        // while a full autoland is in LANDING, and only for the landing-mode family — any
        // *other* mode (GO_HOME, a failsafe) is still a seizure, and every other rung
        // (authority, interlock, RC sticks, link) still watches this phase untouched.
        if (tagDescent?.machine?.phase == TagDescentPhase.DJI_LANDING &&
            com.dimensional.mini4pro.telemetry.Px4Mode.isDjiLandingMode(mode)
        ) {
            return null
        }
        val engagedAt = engagedAtMs ?: return null
        if (now - engagedAt <= MODE_SEIZE_GRACE_MS) return null
        return mode
    }

    /** Must hold [lock]. True while a run sits in phase one of a takeoff item — DJI's own hop. */
    private fun missionAwaitingTakeoffLocked(): Boolean {
        val run = mission ?: return false
        return !run.finished && !run.climbing && run.step.kind == MissionStepKind.TAKEOFF
    }

    private fun tickEngagedLocked(now: Long, effects: MutableList<() -> Unit>) {
        // The abort ladder, cheapest and most authoritative first. Each is also watched on its
        // own event path; re-checking per tick means no abort ever depends on a callback
        // having been delivered.
        if (!interlockEnabled()) {
            effects += { abort(DisengageReason.INTERLOCK) }
            return
        }
        val snapshot = vs
        if (snapshot == null || !authorityOk(snapshot)) {
            effects += { abort(DisengageReason.AUTHORITY, snapshot?.authority ?: "none") }
            return
        }
        val seizedMode = modeSeizedLocked(now)
        if (seizedMode != null) {
            effects += { abort(DisengageReason.AUTHORITY, "MODE_" + seizedMode) }
            return
        }
        port.unavailableReason()?.let { reason ->
            effects += { abort(DisengageReason.AUTHORITY, reason) }
            return
        }
        if (rc?.allPresent() != true) {
            effects += { abort(DisengageReason.AUTHORITY, "RC_STICK_NULL") }
            return
        }
        if (rcAbortDueLocked(now)) {
            effects += { abort(DisengageReason.RC_STICKS) }
            return
        }

        // The branch between the **three** setpoint sources. Everything above it — the whole abort
        // ladder — applies to all of them; nothing below the branch can be reached without it, and
        // there is still exactly one 10 Hz loop and one path to the port. Stage C is a third
        // source, not a second engine.
        val orb = orbit
        if (orb != null) {
            tickOrbitLocked(now, orb, effects)
            return
        }
        val repo = reposition
        if (repo != null) {
            tickRepositionLocked(now, repo, effects)
            return
        }
        // M4 is a **fourth** setpoint source on exactly the same terms as the third: same tick, same
        // abort ladder above the branch, same single path to the port. A mission is not a new
        // authority — it is a sequence of the authority we already have, with a gate in front of it
        // and a cursor that only ever advances on an observation.
        val run = mission
        if (run != null) {
            tickMissionLocked(now, run, effects)
            return
        }
        // M3 Stage D is a **fifth**, on the same terms again — which is the point of the terms.
        // In particular the RC-stick rung above this branch is rule 1's enforcement: a physical
        // stick grab never reaches this line, because the ladder fired first and [abort] has
        // already nulled the descent.
        val dsc = tagDescent
        if (dsc != null) {
            tickTagDescentLocked(now, dsc, effects)
            return
        }

        // The stuck-axis rung — the flat [GuidedEnvelope.MANOEUVRE_TIMEOUT_MS] in its original
        // meaning, and the one site that must never become distance-derived: a held stick commands
        // a velocity, so there is no destination to derive a deadline from.
        val heldSince = nonNeutralSinceMs
        if (heldSince != null && now - heldSince > GuidedEnvelope.MANOEUVRE_TIMEOUT_MS) {
            effects += { abort(DisengageReason.TIMEOUT) }
            return
        }
        val quietSince = lastNonNeutralAtMs
        if (quietSince != null && now - quietSince > GuidedEnvelope.IDLE_DISENGAGE_MS) {
            beginReleaseLocked(now, DisengageReason.IDLE, null, IMMEDIATE_RELEASE, effects)
            return
        }

        val frameAt = lastFrameAtMs
        val frame = lastFrame
        if (frameAt == null || frame == null) {
            // Unreachable by construction — engagement requires frames — kept as a fail-closed
            // branch: no input on record means nothing may be commanded.
            beginReleaseLocked(now, DisengageReason.RELEASED, null, STANDARD_RELEASE, effects)
            return
        }
        val inputAge = now - frameAt
        if (inputAge > GuidedEnvelope.LINK_LOST_MS) {
            val gcsAlive = commandingControllerSeenAtLocked(now)
                ?.let { now - it <= GuidedEnvelope.LINK_LOST_MS } == true
            if (gcsAlive) {
                beginReleaseLocked(now, DisengageReason.RELEASED, null, STANDARD_RELEASE, effects)
            } else {
                // The policy's name is deliberately NOT in the STATUSTEXT (it would overflow
                // the 50-byte field and the operator cannot act on it); it is in the log and
                // in the guided_engaged event, which is where Q4 requires it.
                beginReleaseLocked(now, DisengageReason.LINK_LOST, null, policy.plan(), effects)
            }
            return
        }

        // The mapping itself, with the §3.1 watchdog: a stale frame is not a command, so its
        // velocity is ramped monotonically to zero and comes back the moment input does.
        var velocities = StickMapping.velocities(frame)
        var passthrough = true
        if (inputAge > GuidedEnvelope.INPUT_STALE_MS) {
            val factor = 1.0 -
                (inputAge - GuidedEnvelope.INPUT_STALE_MS).toDouble() / GuidedEnvelope.RAMP_TO_ZERO_MS
            velocities = velocities.scaled(factor.coerceIn(0.0, 1.0))
            passthrough = false
        }

        velocities = climbGatedLocked(velocities, effects)

        val v = velocities
        val source = if (passthrough) CommandSource("MANUAL_CONTROL", lastFrameSeq, null) else null
        effects += { performSend(v, source) }
    }

    /**
     * Stage B's engaged tick: the guidance law between the abort ladder (already passed) and
     * the send. Must hold [lock].
     *
     * The [RepositionGuidance] law flies toward the accepted target; arrival needs **both
     * conjuncts** — distance inside `R_ACCEPT_M` *and* measured speed under `V_SETTLE_MS` —
     * for `ARRIVE_TICKS` consecutive ticks, so a fly-through (inside the circle at speed) can
     * never declare arrival. On arrival the loop keeps station at zero commanded velocity —
     * DJI's own position hold — stays engaged, and announces; disengaging remains an explicit
     * act (Q1's idle window, an abort, or `stop`).
     *
     * The reposition needs no `MANUAL_CONTROL` stream, so the stick watchdogs do not apply
     * here; the link watchdog does — *any* inbound MAVLink counts, and total silence past
     * [GuidedEnvelope.LINK_LOST_MS] runs the armed Q4 policy exactly as it does for sticks.
     * A wind-down ends the manoeuvre for good: nothing resumes a reposition without a fresh,
     * acknowledged command.
     *
     * The position feed is the loop's eyes, and staleness is absence, not zero (§3.2): a
     * stale or invalid fix commands **zero velocity immediately** — never a cached fix — and
     * a feed still dead after [RepositionGuidance.POSITION_LOST_MS] releases entirely,
     * because DJI's own failsafes have GPS of their own and this bridge no longer does.
     */
    private fun tickRepositionLocked(now: Long, repo: RepositionState, effects: MutableList<() -> Unit>) {
        // The link watchdog — Q4, per controller: the liveness of the origin that commanded the
        // manoeuvre, on that origin's own semantics (a MAVLINK goto needs QGC's traffic; a
        // PHONE-origin climb — the phone takeoff's second phase rides this very tick — is alive
        // by identity, the landing08 fix). One owner: controllerSeenAtLocked.
        val gcsAt = commandingControllerSeenAtLocked(now)
        if (gcsAt == null || now - gcsAt > GuidedEnvelope.LINK_LOST_MS) {
            reposition = null
            beginReleaseLocked(now, DisengageReason.LINK_LOST, null, policy.plan(), effects)
            return
        }

        // The position feed — landmine 7. A cached fix is never flown on.
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            val since = repo.positionStaleSinceMs ?: now.also { repo.positionStaleSinceMs = it }
            repo.arriveTicks = 0
            if (now - since > RepositionGuidance.POSITION_LOST_MS) {
                reposition = null
                beginReleaseLocked(now, DisengageReason.NO_POSITION, null, STANDARD_RELEASE, effects)
                return
            }
            effects += {
                announce(GuidedStatusTexts.NO_POSITION_HOLD)
                performSend(StickVelocities.ZERO, null)
            }
            return
        }
        repo.positionStaleSinceMs = null

        // Q1's manoeuvre deadline, from the command's arrival — **derived from the distance this
        // command asked for** (`RepositionState.deadlineMs`), not flat. A reposition that has not
        // finished in more than double the time the guidance law needs for its own commanded
        // distance is not going to; a flat 150 s would instead have ended every leg longer than
        // ~450 m mid-flight once the envelope reached 2 km (2026-07-30).
        if (!repo.arrived && now - repo.acceptedAtMs > repo.deadlineMs) {
            reposition = null
            beginReleaseLocked(now, DisengageReason.TIMEOUT, null, STANDARD_RELEASE, effects)
            return
        }

        if (repo.arrived) {
            // Keep station. Q1's idle disengage runs from the arrival (or pause): authority
            // is not held indefinitely by a controller nobody is commanding.
            val arrivedAt = repo.arrivedAtMs
            if (arrivedAt != null && now - arrivedAt > GuidedEnvelope.IDLE_DISENGAGE_MS) {
                reposition = null
                beginReleaseLocked(now, DisengageReason.IDLE, null, IMMEDIATE_RELEASE, effects)
                return
            }
            // Zero translation, but the nose still swings onto an ROI if one is set: "fly there and
            // look at the thing" is the case this exists for, and the aircraft is still ours.
            val held = roiYawLocked(StickVelocities.ZERO, fix, state, effects)
            effects += { performSend(held, null) }
            return
        }

        // The law.
        val (errorNorth, errorEast) = RepositionGuidance.nedMetres(fix.first, fix.second, repo.latDeg, repo.lonDeg)
        val altitude = usableAltitude(state)
        val errorDown: Double? = if (altitude != null) altitude - repo.relAltM else null
        if (errorDown == null) {
            // The vertical error is unknowable, so the vertical axis is inert — the same
            // blind-climb rule as Stage A's gate, announced through the same sentence.
            effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        }
        var v = RepositionGuidance.velocity(errorNorth, errorEast, errorDown)
        v = climbGatedLocked(v, effects)
        // The nose. An ROI outranks heading-follows-course — if the operator said "look at that",
        // the camera is the point of the flight and the nose serves the camera — and with neither in
        // force the setpoint comes back untouched, yaw included.
        v = targetYawLocked(v, fix, state, errorNorth, errorEast, effects)

        // Arrival — both conjuncts, k consecutive ticks (never on a fly-through).
        val horizontalError = hypot(errorNorth, errorEast)
        val speed = measuredSpeedOrNull(state)
        val settled = RepositionGuidance.settled(horizontalError, errorDown, speed)
        repo.arriveTicks = if (settled) repo.arriveTicks + 1 else 0
        if (repo.arriveTicks >= RepositionGuidance.ARRIVE_TICKS) {
            repo.arrived = true
            repo.arrivedAtMs = now
            effects += {
                log(
                    "goto arrived: horizontal error %.2f m, speed %.2f m/s, held %d ticks"
                        .format(horizontalError, speed, RepositionGuidance.ARRIVE_TICKS)
                )
                record.event(EventCode.GOTO_ARRIVED, "err=%.2f speed=%.2f".format(horizontalError, speed))
                announce(GuidedStatusTexts.GOTO_ARRIVED)
                performSend(StickVelocities.ZERO, null)
            }
            return
        }

        val commanded = v
        effects += { performSend(commanded, CommandSource(REPOSITION_SOURCE, null, null)) }
    }

    /**
     * **M4's engaged tick: the mission**, between the abort ladder (already passed) and the send.
     * Must hold [lock].
     *
     * Everything the reposition tick guards is guarded here too, by the same code in the same order
     * and for the same reasons — the Q4 link watchdog, the position feed as *eyes* whose staleness
     * is absence rather than zero, and the release when the feed stays dead. What a mission adds is
     * three bounds of its own ([MissionGuidance.MISSION_MAX_S] on the whole run,
     * [MissionGuidance.legTimeoutMs] on each leg) and one branch the other two ticks do not have:
     * the cursor.
     *
     * ## The cursor advances on an observation, never on a timer, and never on a stale fix
     *
     * §3.4, and it is structural here rather than remembered: the stale-fix branch **returns** above
     * every completion test, having zeroed [MissionRun.arriveTicks] and commanded zero velocity, so
     * there is no path on which a cursor moves without a fresh position having been read on the same
     * tick. There is no "assume we got there by now" branch and no dead-reckoned cursor.
     */
    private fun tickMissionLocked(now: Long, run: MissionRun, effects: MutableList<() -> Unit>) {
        // The link watchdog — Q4, per controller, exactly as the other two ticks read it.
        val gcsAt = commandingControllerSeenAtLocked(now)
        if (gcsAt == null || now - gcsAt > GuidedEnvelope.LINK_LOST_MS) {
            endMissionLocked(run, DisengageReason.LINK_LOST, null, effects)
            beginReleaseLocked(now, DisengageReason.LINK_LOST, null, policy.plan(), effects)
            return
        }

        // The whole-mission cap, read here rather than after the position block because phase one of
        // a takeoff runs above that block and must still be bounded by it. Not applied once the last
        // item is done: the hold that follows is the ending, not more mission.
        if (!run.finished && now - run.startedAtMs > MissionGuidance.MISSION_MAX_S * 1_000L) {
            endMissionLocked(run, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_MISSION_TIMEOUT, effects)
            beginReleaseLocked(
                now, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_MISSION_TIMEOUT,
                STANDARD_RELEASE, effects,
            )
            return
        }

        // **Phase one of a takeoff item sits above the position block, and the placement is the
        // whole of it.** The aircraft is on the ground and not moving, so `KeyAircraftLocation` —
        // change-driven like everything else on this airframe — publishes nothing, and `POSITION`
        // reads stale by construction. Measured 2026-07-27: 38 consecutive `GLOBAL_POSITION_INT`
        // messages while parked, every one carrying a good coordinate, **one distinct value**.
        //
        // Below the block, that stale reading would return before DJI was ever asked to take off:
        // the mission would sit commanding zero at a motionless aircraft and release itself after
        // `POSITION_LOST_MS`, having done nothing at all. Nothing is given up by being above it,
        // because this phase needs no position — DJI is flying, we command zero, and the cursor
        // moves on the handback rather than on any coordinate. `MissionLaunch.positionUsable` is
        // the same argument made at the gate.
        val step = run.step
        if (step.kind == MissionStepKind.TAKEOFF && !run.climbing) {
            tickMissionTakeoffLocked(now, run, step, effects)
            return
        }

        // The position feed — landmine 7. A cached fix is never flown on, and never advanced on.
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            val since = run.positionStaleSinceMs ?: now.also { run.positionStaleSinceMs = it }
            run.arriveTicks = 0
            if (now - since > RepositionGuidance.POSITION_LOST_MS) {
                endMissionLocked(run, DisengageReason.NO_POSITION, null, effects)
                beginReleaseLocked(now, DisengageReason.NO_POSITION, null, STANDARD_RELEASE, effects)
                return
            }
            effects += {
                announce(GuidedStatusTexts.NO_POSITION_HOLD)
                performSend(StickVelocities.ZERO, null)
            }
            return
        }
        run.positionStaleSinceMs = null

        if (run.finished) {
            // Arrived at the last item and holding — **in the air**, waiting for a human on the
            // sticks (M4-5). Q1's idle disengage runs from the hold, exactly as it does from a
            // goto's arrival: authority is not held indefinitely by a controller nobody is
            // commanding, and the hold is therefore bounded rather than permanent.
            val since = run.holdingSinceMs
            if (since != null && now - since > GuidedEnvelope.IDLE_DISENGAGE_MS) {
                endMissionLocked(run, DisengageReason.IDLE, null, effects)
                beginReleaseLocked(now, DisengageReason.IDLE, null, IMMEDIATE_RELEASE, effects)
                return
            }
            val held = roiYawLocked(StickVelocities.ZERO, fix, state, effects)
            effects += { performSend(held, null) }
            return
        }

        // Phase two of a takeoff — the commanded climb. Below the position block rather than above
        // it, unlike phase one: the aircraft is airborne and ours, so a position feed that has gone
        // quiet means the same thing here it means on any other leg.
        if (step.kind == MissionStepKind.TAKEOFF) {
            tickMissionClimbLocked(now, run, step, fix, effects)
            return
        }

        // The precision `NAV_LAND` — its own three-phase sequence, and the one item that ends with
        // the aircraft on the ground. Below the position block for the takeoff-climb reason and one
        // of its own: every gate this sequence has is a statement about where the aircraft *is*.
        if (step.kind == MissionStepKind.PRECISION_LAND) {
            tickMissionLandTagLocked(now, run, step, fix, effects)
            return
        }

        // The per-leg timeout, from the moment the leg began and scaled to the leg it bounds. Q1's
        // flat manoeuvre timeout is the wrong shape here: a 100 m leg and a 5 m one should not share
        // a deadline.
        val startedAt = run.legStartedAtMs ?: now.also { run.legStartedAtMs = it }
        val legOriginLat = run.legOriginLatDeg ?: fix.first
        val legOriginLon = run.legOriginLonDeg ?: fix.second
        val (legNorth, legEast) = RepositionGuidance.nedMetres(
            legOriginLat, legOriginLon, step.latDeg, step.lonDeg,
        )
        if (now - startedAt > MissionGuidance.legTimeoutMs(hypot(legNorth, legEast))) {
            endMissionLocked(run, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT, effects)
            beginReleaseLocked(
                now, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT,
                STANDARD_RELEASE, effects,
            )
            return
        }

        // The law. The P-term reads this leg's error; the braking term reads the distance to the
        // next point at which the mission requires zero speed (§3.3).
        val (errorNorth, errorEast) = RepositionGuidance.nedMetres(
            fix.first, fix.second, step.latDeg, step.lonDeg,
        )
        val horizontalError = hypot(errorNorth, errorEast)
        val altitude = usableAltitude(state)
        val errorDown: Double? = when {
            step.relAltM == null -> null // a step that commands no height: RTL and land do not descend
            altitude == null -> null
            else -> altitude - step.relAltM
        }
        if (altitude == null && step.relAltM != null) {
            effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        }
        // A **resting** step is the one point at which the aircraft must be stationary, so its own
        // rest-ahead is zero and the law reduces literally to M3's. A rejoin leg is treated as
        // resting whatever the step says, which is why the resting question is asked once, here.
        val resting = step.rest || run.rejoining
        val restAhead = if (resting) 0.0 else run.route.restAheadM(run.index)
        var v = MissionGuidance.velocity(
            errorNorth, errorEast, errorDown,
            MissionGuidance.stopDistanceM(horizontalError, restAhead),
        )
        v = climbGatedLocked(v, effects)
        v = targetYawLocked(v, fix, state, errorNorth, errorEast, effects)

        // Completion. Two tests, and which one applies is the whole of §3.1.
        val complete = if (resting) {
            // **The flight-verified M3 predicate, called rather than restated.** A test fails if
            // this stops being the same function the reposition path uses.
            val speed = measuredSpeedOrNull(state)
            val settled = RepositionGuidance.settled(horizontalError, errorDown, speed)
            run.arriveTicks = if (settled) run.arriveTicks + 1 else 0
            run.arriveTicks >= RepositionGuidance.ARRIVE_TICKS
        } else {
            run.arriveTicks = 0
            MissionGuidance.passed(
                distanceM = horizontalError,
                alongTrackRemainingM = MissionGuidance.alongTrackRemainingM(
                    errorNorth, errorEast, legNorth, legEast,
                ),
                switchRadiusM = step.switchRadiusM,
            )
        }

        val commanded = v
        if (complete) {
            // The setpoint still goes out on the tick that advances the cursor. Dropping it would
            // put a 200 ms hole in a 10 Hz stream at the one moment the aircraft is *between* legs,
            // which is exactly when the watchdogs at both ends are least forgiving; and the velocity
            // computed for the leg just completed points within a few degrees of the next one, which
            // is the whole reason the corner is rounded rather than cornered.
            advanceMissionLocked(now, run, fix, horizontalError, commanded, effects)
            return
        }
        effects += { performSend(commanded, CommandSource(MISSION_SOURCE, null, null)) }
    }

    /**
     * **`NAV_TAKEOFF`, phase one: DJI's own hop, and the wait for it to end.** Must hold [lock].
     *
     * DJI's own takeoff is asked for **once**, through [missionTakeoff], and this engine commands
     * **zero velocity** for the whole of it: DJI is flying this phase and a setpoint from us would
     * be fighting it. A takeoff DJI refuses ends the mission with the operator getting DJI's own
     * error verbatim.
     *
     * ## The cursor does not move on `isFlying`, and that is the whole of this function
     *
     * It moves on [TakeoffClimb]'s **four conjuncts** — the aircraft observed not flying, one of
     * DJI's takeoff modes observed, then flying, and the mode out of that family — because
     * `isFlying` goes true **1.4 s and 0.6 s into `AUTO_TAKE_OFF`** with 2.0 s and 2.4 s of DJI's
     * climb still to run (both 2026-07-27 records; a third on the same afternoon measured 1.4 s
     * again, this time with the takeoff commanded by us). A mission that advanced there would begin
     * flying the **next leg horizontally** while DJI was still climbing — two controllers on one
     * aircraft a metre off the ground, which is the failure that class exists to prevent.
     *
     * That machine is **called, not restated**: [MissionRun.takeoff] is a [TakeoffClimb] armed as a
     * bare watch ([TakeoffClimb.armWatch]), so the conjuncts, their evidence and their bound have
     * exactly one implementation and the live `MAV_CMD_NAV_TAKEOFF` path shares it.
     *
     * ## Phase two
     *
     * The handback sets [MissionRun.climbing] and [tickMissionClimbLocked] flies the vertical-only
     * leg to [MissionStep.relAltM] — §3.6, and the height comes from the route unmodified. The
     * transition is recorded under the lock on the tick it is observed, because
     * [TakeoffClimb.observe] reports a handback exactly once and losing it would strand the run.
     *
     * ## The camera, added 2026-07-30, and the flight that demanded it
     *
     * The handback also fires [aimTakeoffCameraNadir] — the identical call the phone and QGC
     * takeoff doors have made since 2026-07-29, on the identical seam ([TakeoffClimb.armWatch]'s
     * `aimCameraNadir`, consumed once). Until then this was the **only** takeoff door that left
     * the camera wherever the RC wheel had it, and landing16
     * (`datasets/landing16/20260730-161329.001.jsonl`) is what that cost: mission takeoff at
     * t=51.9, handback at t=56.5 with **no** `dji_call gimbal_rotate` — the gimbal held pitch 0°
     * through the whole climb — so the 1–2 m acquisition pass `vision/TagArming` opens the
     * detector for (100 % detection, measured) looked at the horizon, nothing latched, the climb
     * crossed the detector's ceiling at t=62.0 and closed the one-way band, and the flight's
     * precision landing was refused `NO_TAG_LATCHED` at t=188.8 over a pad the camera was by then
     * pointing straight at. **Zero `tag` lines in 241 s**, against 617 on the very next manual
     * flight (`20260730-101110`). The record now says which door aimed: `takeoff_climb_ended
     * mission seq=N nadir`.
     *
     * One interaction, named rather than discovered: an ROI live at the takeoff would be re-aimed
     * by `updateRoiCameraLocked` on the next engaged tick and would win. That is true of all three
     * takeoff doors identically, it is not new here, and the precision-land sequence — the one
     * place it would matter — clears the ROI when it begins.
     */
    private fun tickMissionTakeoffLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        effects: MutableList<() -> Unit>,
    ) {
        val takeoff = missionTakeoff
        if (takeoff == null) {
            // Refused at Start, so this is unreachable; kept as a fail-closed branch because a
            // takeoff item with nothing to start it must never become a silently skipped item.
            endMissionLocked(run, DisengageReason.AUTHORITY, "NO_TAKEOFF", effects)
            effects += { abort(DisengageReason.AUTHORITY, "NO_TAKEOFF") }
            return
        }

        if (!run.takeoffAsked) {
            run.takeoffAsked = true
            run.legStartedAtMs = now
            // Armed on the same tick DJI is asked, so the watch's window and the leg timeout below
            // start together and neither can outlive the other by a tick. `aimCameraNadir` rides
            // the watch for the same reason it rides the command doors' pending climb: it is
            // consumed exactly once, at the handback, and every rung that kills the watch —
            // timeout, expiry, RC grab, abort — kills the camera move with it.
            run.takeoff.armWatch(now, aimCameraNadir = true)
            effects += {
                log("mission takeoff: asking DJI (item ${step.seq})")
                announce(GuidedStatusTexts.MISSION_TAKEOFF)
                takeoff.startTakeoff { djiError ->
                    // DJI's own words, never our paraphrase — `SYSTEM_ERROR` shortly after a landing
                    // is a measured refusal and the operator needs to see which one it was.
                    log("mission takeoff refused by DJI: $djiError")
                    abort(DisengageReason.AUTHORITY, "TAKEOFF_$djiError")
                }
            }
        }

        // The bounded wait for DJI to report flight, on the leg timeout the item already has. Kept
        // beside [TakeoffClimb.WAIT_LIMIT_MS] rather than replaced by it: this one ends the *run*
        // with §6.2's leg-timeout consequence, which the watch — a machine that knows nothing about
        // missions — cannot do.
        val askedAt = run.legStartedAtMs ?: now
        if (now - askedAt > MissionGuidance.legTimeoutMs(0.0)) {
            endMissionLocked(run, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT, effects)
            beginReleaseLocked(
                now, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT,
                STANDARD_RELEASE, effects,
            )
            return
        }

        val state = aircraftState()
        when (val decision = run.takeoff.observe(state.isFlying, state.flightMode, now)) {
            // Idle is unreachable — the watch was armed above, on this tick or an earlier one, and
            // only a handback or the expiry clears it — and reads identically to Waiting anyway.
            TakeoffClimb.Decision.Idle, TakeoffClimb.Decision.Waiting -> Unit

            // The two bounds are equal (both 30 s from the same tick), so whichever fires first is
            // the leg timeout above; this is the same ending by the other road, and it must exist
            // rather than be reasoned away, because the equality is a fact about two constants.
            TakeoffClimb.Decision.Expired -> {
                endMissionLocked(run, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT, effects)
                beginReleaseLocked(
                    now, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT,
                    STANDARD_RELEASE, effects,
                )
                return
            }

            is TakeoffClimb.Decision.HandedBack -> {
                // `relAltM` is null by construction — a watch carries no target — and the height is
                // read from the step by the climb tick, so the mission has one source for it.
                // `aimCameraNadir` is *not* null-by-construction and is read rather than assumed:
                // it is the same fact the phone and QGC doors carry, on the same seam.
                val aimNadir = decision.aimCameraNadir
                run.climbing = true
                run.legStartedAtMs = now
                run.climbOriginAltM = usableAltitude(state)
                run.arriveTicks = 0
                // Phase two is the first moment this engagement actually commands anything, so the
                // mode-seize grace runs from here: DJI's mode is whatever its takeoff left behind
                // and has [MODE_SEIZE_GRACE_MS] to become `JOYSTICK`, exactly as any other
                // engagement does. See [modeSeizedLocked].
                engagedAtMs = now
                effects += {
                    log("mission takeoff: DJI has let go — climbing to ${step.relAltM ?: "no height"}")
                    record.event(
                        EventCode.TAKEOFF_CLIMB_ENDED,
                        "mission seq=${step.seq}${if (aimNadir) " nadir" else ""}",
                    )
                    // Before the climb's own first setpoint, exactly as the command doors aim
                    // before their accept: the aircraft is airborne at DJI's ~1.2 m and the
                    // camera half of the takeoff is due whatever the climb then does.
                    if (aimNadir) aimTakeoffCameraNadir()
                    announce(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING)
                }
            }
        }
        // DJI is flying this phase. Zero, so nothing of ours is fighting it — and on the handback
        // tick too, because the climb's own first setpoint belongs to the climb's own tick, after
        // a fresh look at the altitude.
        effects += { performSend(StickVelocities.ZERO, null) }
    }

    /**
     * **`NAV_TAKEOFF`, phase two: the commanded climb**, `docs/m4-mission-execution.md` §3.6. Must
     * hold [lock].
     *
     * A **vertical-only** leg to [MissionStep.relAltM]: DJI's hop leaves the aircraft over its
     * launch point and the takeoff item's coordinate is that same point, so there is nothing
     * horizontal to fly and a horizontal term here could only be error. The zeros are passed to
     * [MissionGuidance.velocity] explicitly rather than the leg path being reused with a coordinate,
     * which is what makes "a takeoff never translates" a property of the code instead of a property
     * of the numbers in the plan.
     *
     * Everything else is the ordinary leg: the same law (which with no horizontal error and nothing
     * ahead to rest for is [RepositionGuidance.clampedSpeed] on the vertical axis, digit for digit
     * — the same arithmetic the operator's Takeoff button flies), the same [climbGatedLocked]
     * ceiling, the same [targetYawLocked] so an ROI still outranks, the same leg timeout scaled to
     * the climb's own height, and the same arrival predicate [RepositionGuidance.settled] the
     * resting waypoint uses.
     *
     * The horizontal argument to that predicate is a literal `0.0`, and that is §3.6's *"completing
     * on M3's vertical conjunct alone"* stated in code: we command no horizontal correction, so
     * holding the climb open on horizontal drift would be waiting for something nothing is flying.
     */
    private fun tickMissionClimbLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        fix: Pair<Double, Double>,
        effects: MutableList<() -> Unit>,
    ) {
        val startedAt = run.legStartedAtMs ?: now.also { run.legStartedAtMs = it }
        val target = step.relAltM
        val state = aircraftState()
        val altitude = usableAltitude(state)
        // The climb's own height, measured from where DJI actually left the aircraft rather than
        // assumed to be its documented hop — the two differ, and 1.0–1.1 m is what was measured.
        val origin = run.climbOriginAltM
        val climbM = if (target != null && origin != null) abs(target - origin) else 0.0
        if (now - startedAt > MissionGuidance.legTimeoutMs(climbM)) {
            endMissionLocked(run, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT, effects)
            beginReleaseLocked(
                now, DisengageReason.TIMEOUT, MissionAbortPolicy.DETAIL_LEG_TIMEOUT,
                STANDARD_RELEASE, effects,
            )
            return
        }

        val errorDown: Double? = when {
            target == null -> null // a takeoff item that names no height: DJI's hop is the whole of it
            altitude == null -> null
            else -> altitude - target
        }
        if (altitude == null && target != null) effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        var v = MissionGuidance.velocity(0.0, 0.0, errorDown, stopDistanceM = 0.0)
        v = climbGatedLocked(v, effects)
        v = targetYawLocked(v, fix, state, 0.0, 0.0, effects)

        val settled = RepositionGuidance.settled(0.0, errorDown, measuredSpeedOrNull(state))
        run.arriveTicks = if (settled) run.arriveTicks + 1 else 0
        if (run.arriveTicks >= RepositionGuidance.ARRIVE_TICKS) {
            run.climbing = false
            advanceMissionLocked(now, run, fix, abs(errorDown ?: 0.0), StickVelocities.ZERO, effects)
            return
        }
        val commanded = v
        effects += { performSend(commanded, CommandSource(MISSION_SOURCE, null, null)) }
    }

    // ------------------------------------------- the precision NAV_LAND sequence

    /**
     * **A plan's `NAV_LAND` with "Precision Land" set: the mission's own half of a tag landing.**
     * Must hold [lock].
     *
     * The sequencing decision — the gates, the two numbers, the phase vocabulary, and every argument
     * about why the reference point is the *recorded* takeoff position — is [PrecisionLand], and this
     * function is what flies it. **No descent law and no landing law is implemented anywhere in this
     * file's precision-land path**: the last act is [armTagDescent], through the door
     * [armTagDescentFromPhone] uses, with every gate that door applies binding unchanged, and from
     * there the aircraft is flown by [tickTagDescentLocked] — approach, band entry, tracking, terminal,
     * commit, `KeyStartAutoLanding`, touchdown at motors-off.
     *
     * ## The shape, and the precedent for it
     *
     * A leg-shape per function, each **calling** the shared law rather than restating it — which is
     * [tickMissionClimbLocked]'s pattern exactly (phase two of a takeoff is its own tick for its own
     * shape, and it is the same [MissionGuidance.velocity], the same [climbGatedLocked], the same
     * [targetYawLocked], the same [RepositionGuidance.settled] and the same
     * [RepositionGuidance.ARRIVE_TICKS] as everything else). Every *number* here has exactly one
     * owner; what is repeated is the order the owners are called in, and the alternative — threading
     * three targets and two completion tests through one tick function — is how a `when` gets
     * discovered at item 14.
     *
     * ## Every ending, in one list
     *
     * Ivan's rule is *"just error out"*, and there is no path here that lands, returns, or descends by
     * any other means:
     *
     *  - **a gate refuses** → the run ends, the aircraft keeps station, the reason is named
     *    ([endMissionForLandTagLocked]);
     *  - **a phase times out** → the same ending, naming the phase;
     *  - **the camera never reaches believed nadir** → the same ending;
     *  - **the acquisition reaches [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M] and then holds there
     *    for [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] with the arm gate still refusing** → the
     *    same ending, carrying the wait it spent and the gate's own last words. **Reaching the
     *    floor is not itself an ending** — landing17 refused 267 ms before the tag decoded, and
     *    [PrecisionLand.Phase.HOLD] is what that flight bought;
     *  - **the arm is refused** → the same ending, and the descent's own door has already said which
     *    of its gates said no. Reachable only when the gate's answer changes between the lookahead
     *    poll and the arm one tick later — the sensor conjuncts are the ones that move, so this is
     *    a fix that went stale in the gap, not a design that guesses;
     *  - **the arm is accepted** → the run ends *successfully*: the item is reported reached, the
     *    mission is reported finished, and the descent owns the aircraft.
     *
     * Everything above this branch still applies unchanged — the whole abort ladder, the Q4 link
     * watchdog, the whole-mission cap, and the position block whose staleness is absence rather than
     * zero. An RC stick grab during any phase is the ladder's, exactly as it is on any other leg.
     */
    private fun tickMissionLandTagLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        fix: Pair<Double, Double>,
        effects: MutableList<() -> Unit>,
    ) {
        val state = aircraftState()
        val land = run.land ?: beginLandTagLocked(now, run, step, fix, state, effects) ?: return

        // The phase's own bound, from the phase's own start. A sequence that has not finished its
        // current phase in time is not going to, and the aircraft is over the pad rather than
        // somewhere nobody planned — so this ends the run and holds rather than releasing.
        //
        // **[PrecisionLand.Phase.HOLD]'s bound is not a timeout, it is the answer**, and it gets
        // its own sentence for that reason: the other four phases expiring means the flying did
        // not finish, this one expiring means the tag never became decodable over the pad, which
        // is a different thing to tell an operator and a different thing to find on a record. The
        // branch is here rather than in the `when` below so that a hold can never *also* be
        // refused as a phase that overran — one deadline, one meaning, one place it is read.
        if (now - land.phaseStartedAtMs > land.phaseLimitMs) {
            if (land.phase == PrecisionLand.Phase.HOLD) {
                refuseLandTagAcquisitionLocked(now, run, step, land, effects)
            } else {
                endMissionForLandTagLocked(
                    now, run, step,
                    GuidedStatusTexts.REASON_LAND_PHASE_TIMEOUT,
                    "the ${land.phase.name.lowercase()} phase did not finish inside " +
                        "${land.phaseLimitMs}ms",
                    effects,
                )
            }
            return
        }

        when (land.phase) {
            // The camera. Commanded once, then the bounded wait for the *belief* — the same number
            // `descentGateLocked` and `TagWorld.fix` judge, so a sequence that proceeds is one whose
            // arm will not be refused for the camera.
            PrecisionLand.Phase.AIMING -> {
                if (!land.nadirAsked) {
                    land.nadirAsked = true
                    val gimbal = manoeuvreGimbal
                    effects += {
                        if (gimbal == null) {
                            // Logged, never absorbed: the wait below will refuse by name, which is
                            // the honest ending for a landing that needs a camera nobody wired.
                            log("tag landing wanted the camera at nadir but no gimbal path is wired")
                        } else {
                            log(
                                "tag landing: pointing the camera at nadir (%.0f)"
                                    .format(TagDescentGuidance.NADIR_PITCH_DEG)
                            )
                            gimbal.aimPitch(TagDescentGuidance.NADIR_PITCH_DEG)
                        }
                    }
                }
                val pitch = cameraPitchDeg()
                val nadir = pitch != null && pitch.isFinite() &&
                    abs(pitch - TagDescentGuidance.NADIR_PITCH_DEG) <=
                    TagDescentGuidance.NADIR_TOLERANCE_DEG
                if (nadir) {
                    beginLandTagPhaseLocked(now, run, land, PrecisionLand.Phase.LOWER, state, effects)
                }
                // Station-kept, at the item's altitude, while the camera moves: the sequence has
                // arrived and the next phase is a descent that must not start blind.
                val held = roiYawLocked(StickVelocities.ZERO, fix, state, effects)
                effects += { performSend(held, CommandSource(LAND_TAG_SOURCE, null, null)) }
            }

            PrecisionLand.Phase.TRANSIT -> {
                val settled = flyLandTagLegLocked(
                    now, run, land, fix, state,
                    targetRelAltM = land.transitRelAltM, effects = effects,
                )
                if (settled) {
                    beginLandTagPhaseLocked(now, run, land, PrecisionLand.Phase.AIMING, state, effects)
                }
            }

            PrecisionLand.Phase.LOWER -> {
                val settled = flyLandTagLegLocked(
                    now, run, land, fix, state,
                    targetRelAltM = land.armHeightM, effects = effects,
                )
                // Arrival no longer arms — it starts *looking*. See [PrecisionLand.Phase.ACQUIRE]
                // and landing16's 8.8 m arrival for why one attempt here is a coin flip.
                if (settled) {
                    beginLandTagPhaseLocked(now, run, land, PrecisionLand.Phase.ACQUIRE, state, effects)
                }
            }

            // **The acquisition descent.** The gate is asked *before* the leg is flown, because a
            // gate that clears on this tick means the aircraft is about to stop being the mission's
            // — and sending one more of the mission's setpoints into that instant is the ordering
            // this whole hand-off is arranged to avoid.
            PrecisionLand.Phase.ACQUIRE -> {
                if (askLandTagArmGateLocked(now, run, step, land, state, effects)) return
                val settled = flyLandTagLegLocked(
                    now, run, land, fix, state,
                    targetRelAltM = land.acquireFloorM, effects = effects,
                )
                // Arrival at the floor is no longer the refusal — it is the beginning of the wait.
                // landing17 measured why: the refusal fired 267 ms before the tag first decoded.
                if (settled) {
                    beginLandTagPhaseLocked(now, run, land, PrecisionLand.Phase.HOLD, state, effects)
                }
            }

            // **The wait at the floor**, [PrecisionLand.Phase.HOLD]. The same gate, asked the same
            // way, on the same tick order — the only differences are that the aircraft has nowhere
            // lower to go and that the deadline above now means "the tag never became decodable"
            // rather than "the leg did not finish". The leg is still flown, so the last of the
            // descent completes here (landing17's arrival test fired 0.8 m high, still at 0.4 m/s).
            PrecisionLand.Phase.HOLD -> {
                if (askLandTagArmGateLocked(now, run, step, land, state, effects)) return
                flyLandTagLegLocked(
                    now, run, land, fix, state,
                    targetRelAltM = land.acquireFloorM, effects = effects,
                )
            }
        }
    }

    /**
     * **The one place the acquisition asks whether it may arm** — shared by
     * [PrecisionLand.Phase.ACQUIRE] and [PrecisionLand.Phase.HOLD], which is the point: the descent
     * and the wait must ask the *same* gate the *same* way, or the hold becomes a second, weaker
     * door into a landing. Must hold [lock].
     *
     * Returns true when the arm was taken and the caller must return without commanding anything
     * else on this tick — the run is over, [armLandTagLocked] has nulled [mission], and the
     * descent owns the aircraft from the effect list onward.
     *
     * A blocked gate is **remembered rather than announced**: at 8 m over a pad it says "no fix"
     * many times a second and that is the feature working, not a fault. Only the *last* one is
     * spoken, when the hold expires, where it is the reason the landing did not happen.
     */
    private fun askLandTagArmGateLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        land: LandTagRun,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ): Boolean {
        val gate = descentGateLocked(tagSense?.invoke(), run.origin, now, ignoringOwnMission = true)
        if (gate is DescentGate.Clear) {
            armLandTagLocked(now, run, step, land, usableAltitude(state), effects)
            return true
        }
        land.lastArmBlock = (gate as DescentGate.Blocked).let { it.reason to it.detail }
        return false
    }

    /**
     * **The acquisition's own ending**: the band was flown, the floor was held for
     * [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS], and the gate never cleared. Must hold [lock].
     *
     * Reached from exactly one place — the phase deadline at the top of [tickMissionLandTagLocked],
     * with [PrecisionLand.Phase.HOLD] in force — so "the window expired" and "the acquisition
     * failed" are the same event and cannot drift apart.
     *
     * The sentence is the one landing17's record carried, kept verbatim in shape because it was
     * the good part of that flight's evidence: it names the band it flew, **the wait it spent**,
     * and the descent gate's own last words, so a post-mortem reads which conjunct was still
     * failing at the floor without opening anything else. The wait is on the line for the same
     * reason the arm height is on the arming line — it is the measurement this feature exists to
     * accumulate, from the failing side.
     */
    private fun refuseLandTagAcquisitionLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        land: LandTagRun,
        effects: MutableList<() -> Unit>,
    ) {
        val last = land.lastArmBlock
        val heldMs = now - land.phaseStartedAtMs
        endMissionForLandTagLocked(
            now, run, step,
            GuidedStatusTexts.REASON_LAND_NO_ACQUIRE,
            ("descended to %.1fm from %.1fm and held %dms without an armable fix — " +
                "the descent's gate %s").format(
                land.acquireFloorM ?: Double.NaN,
                land.armHeightM ?: Double.NaN,
                heldMs,
                last?.let { "still said ${it.first}: ${it.second}" } ?: "was never asked",
            ),
            effects,
        )
    }

    /**
     * The sequence's first tick: resolve the reference point, run [PrecisionLand.gate], clear any ROI,
     * and begin [PrecisionLand.Phase.TRANSIT]. Must hold [lock].
     *
     * Returns the new [LandTagRun], or **null when the sequence was refused** — in which case the run
     * has already been ended and the aircraft put into a station hold, and the caller must return
     * without commanding anything else on this tick.
     *
     * ## The ROI, cleared here and nowhere else
     *
     * [updateRoiCameraLocked] runs on **every** engaged tick, above the setpoint branch, so an ROI
     * still in force would re-aim the gimbal at its own depression angle on every tick — fighting the
     * nadir command forever, and ending the sequence in a phase timeout with the camera pointed at the
     * operator's subject. `DO_SET_ROI_NONE` deliberately leaves the camera where it is (a default is a
     * lie about where the target is), which is why the clear is not enough on its own and the nadir
     * command has to be explicit.
     *
     * **Which ROIs can actually be in force here, stated exactly** — and since 2026-07-30 the list is
     * three rather than two, because a plan's own ROI items now fly:
     *
     *  - a **plan's** `DO_SET_ROI_LOCATION`, sticky from its own item to the `DO_SET_ROI_NONE` that ends
     *    it ([MissionStep.roi]). A plan that never clears it — big1.plan does, at item 8 — arrives at
     *    its Land item with the camera on the operator's subject, and **this clear is the thing that
     *    keeps the sequence from deadlocking**: [updateRoiCameraLocked] runs above the setpoint branch,
     *    so an ROI left in force would fight the nadir command on every tick until the aim phase timed
     *    out. That is why the row exists in this feature's mutation table with a count of its own;
     *  - a Fly-view `DO_SET_ROI_LOCATION` the operator sent mid-mission;
     *  - an orbit's implied centre.
     *
     * The clear is written for all three, because [clearRoiTargetsLocked] does not care who set the
     * target — one owner, one clear.
     */
    private fun beginLandTagLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        fix: Pair<Double, Double>,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ): LandTagRun? {
        val takeoff = recordedTakeoffPoint(state)
        val currentAlt = usableAltitude(state)
        val planTakeoffAlt = run.route.takeoffRelAltM
        val decision = PrecisionLand.gate(
            aircraft = fix,
            takeoff = takeoff,
            itemLatDeg = step.latDeg,
            itemLonDeg = step.lonDeg,
            itemRelAltM = step.relAltM,
            currentRelAltM = currentAlt,
            planTakeoffRelAltM = planTakeoffAlt,
        )
        if (decision is PrecisionLand.Decision.Refused) {
            endMissionForLandTagLocked(now, run, step, decision.reason, decision.detail, effects)
            return null
        }
        val clear = decision as PrecisionLand.Decision.Clear
        val land = LandTagRun(
            mode = step.precisionLandMode,
            takeoffLatDeg = clear.takeoffLatDeg,
            takeoffLonDeg = clear.takeoffLonDeg,
            transitRelAltM = clear.transitRelAltM,
            startedAtMs = now,
        )
        run.land = land
        run.arriveTicks = 0
        val hadRoi = clearRoiTargetsLocked()
        val legM = landTagLegMetres(fix, state, land, clear.transitRelAltM)
        land.phaseLimitMs = MissionGuidance.legTimeoutMs(legM)
        val acDist = RepositionGuidance.horizontalMetres(
            fix.first, fix.second, clear.takeoffLatDeg, clear.takeoffLonDeg,
        )
        val itemDist = RepositionGuidance.horizontalMetres(
            clear.takeoffLatDeg, clear.takeoffLonDeg, step.latDeg, step.lonDeg,
        )
        effects += {
            log(
                ("tag landing begun at item %d: mode=%d takeoff %.7f/%.7f, aircraft %.1fm from it, " +
                    "item drawn %.1fm from it, %.1fm up against the plan's %.1fm takeoff, " +
                    "transit to %.1fm").format(
                    step.seq, land.mode, clear.takeoffLatDeg, clear.takeoffLonDeg,
                    acDist, itemDist, currentAlt ?: Double.NaN, planTakeoffAlt ?: Double.NaN,
                    clear.transitRelAltM,
                )
            )
            record.event(
                EventCode.LAND_TAG_BEGUN,
                ("seq=%d mode=%d takeoff=%.7f/%.7f acDist=%.1f itemDist=%.1f alt=%.1f " +
                    "planTakeoff=%.1f transit=%.1f").format(
                    step.seq, land.mode, clear.takeoffLatDeg, clear.takeoffLonDeg,
                    acDist, itemDist, currentAlt ?: Double.NaN, planTakeoffAlt ?: Double.NaN,
                    clear.transitRelAltM,
                ),
            )
            if (hadRoi) {
                log("tag landing: the ROI was cleared — the camera is the landing's now")
                record.event(EventCode.ROI_CLEARED, "cleared by the tag landing sequence")
                announce(GuidedStatusTexts.ROI_CLEARED)
            }
            announce(GuidedStatusTexts.LAND_TAG_TRANSIT)
        }
        return land
    }

    /**
     * One phase transition: the clock, the bound, the frozen numbers the next phase needs, and the
     * sentence. Must hold [lock] — **the one place a phase changes**, so the arrival counter can never
     * be carried across a transition and a phase can never inherit its predecessor's deadline.
     */
    private fun beginLandTagPhaseLocked(
        now: Long,
        run: MissionRun,
        land: LandTagRun,
        next: PrecisionLand.Phase,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ) {
        land.phase = next
        land.phaseStartedAtMs = now
        run.arriveTicks = 0
        when (next) {
            PrecisionLand.Phase.TRANSIT -> Unit // built into the run; never re-entered

            PrecisionLand.Phase.AIMING -> {
                land.phaseLimitMs = PrecisionLand.NADIR_AIM_LIMIT_MS
                effects += {
                    log("tag landing: at the item's height — camera to nadir before descending")
                    record.event(EventCode.LAND_TAG_PHASE, "aiming")
                    announce(GuidedStatusTexts.LAND_TAG_AIMING)
                }
            }

            PrecisionLand.Phase.LOWER -> {
                // The never-climb clamp, frozen here: `min(ARM_HEIGHT, current)`, so a Land item
                // authored below the arm height arms where it is instead of climbing back up to it.
                val from = usableAltitude(state)
                val target = PrecisionLand.armHeightTargetM(from ?: PrecisionLand.LAND_TAG_ARM_HEIGHT_M)
                land.armHeightM = target
                land.armHeightFromM = from
                land.phaseLimitMs = MissionGuidance.legTimeoutMs(abs((from ?: target) - target))
                effects += {
                    log(
                        "tag landing: lowering to %.1fm (min of %.1f and the %.1fm we are at)"
                            .format(target, PrecisionLand.LAND_TAG_ARM_HEIGHT_M, from ?: Double.NaN)
                    )
                    record.event(
                        EventCode.LAND_TAG_PHASE,
                        "lowering to=%.1f from=%.1f".format(target, from ?: Double.NaN),
                    )
                    announce(GuidedStatusTexts.LAND_TAG_LOWERING)
                }
            }

            PrecisionLand.Phase.ACQUIRE -> {
                // The second never-climb clamp, frozen for the same reason the first one is: a Land
                // item authored below the floor holds and tries where it is rather than climbing a
                // metre to go looking. `armHeightM` is written by the LOWER entry above and is the
                // only way into this phase, so the elvis is a fail-closed branch, not a case.
                val arm = land.armHeightM ?: PrecisionLand.LAND_TAG_ARM_HEIGHT_M
                val floor = PrecisionLand.acquireFloorTargetM(arm)
                land.acquireFloorM = floor
                land.phaseLimitMs = MissionGuidance.legTimeoutMs(abs(arm - floor))
                effects += {
                    log(
                        ("tag landing: at %.1fm with nothing to arm on — descending to %.1fm, " +
                            "asking the arm gate every tick").format(arm, floor)
                    )
                    record.event(
                        EventCode.LAND_TAG_PHASE,
                        "acquiring from=%.1f to=%.1f".format(arm, floor),
                    )
                    announce(GuidedStatusTexts.LAND_TAG_ACQUIRING)
                }
            }

            PrecisionLand.Phase.HOLD -> {
                // The window is the phase's whole deadline — see the top of `tickMissionLandTagLocked`,
                // where its expiry is the acquisition's refusal rather than a leg that overran.
                land.phaseLimitMs = PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS
                val floor = land.acquireFloorM
                val at = usableAltitude(state)
                effects += {
                    log(
                        ("tag landing: at the floor (%.1fm indicated, target %.1fm) — holding and " +
                            "asking the arm gate for %dms before refusing")
                            .format(at ?: Double.NaN, floor ?: Double.NaN,
                                PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS)
                    )
                    // `at` and `for` on one line: the height the wait actually happened at (the
                    // barometer's, lies and all — `LAND_TAG_ACQUIRE_FLOOR_M` has the measurement)
                    // and the window it is being given. A post-flight reader times the acquisition
                    // from this line to `armed` or to `land_tag_refused`.
                    record.event(
                        EventCode.LAND_TAG_PHASE,
                        "holding at=%.1f floor=%.1f for=%d".format(
                            at ?: Double.NaN, floor ?: Double.NaN,
                            PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS,
                        ),
                    )
                    announce(GuidedStatusTexts.LAND_TAG_HOLDING)
                }
            }
        }
    }

    /**
     * One flying phase of the sequence, one tick: the law, the ceiling gate, the nose, the send, and
     * M3's arrival predicate. Must hold [lock]. Returns true when the arrival test has held for
     * [RepositionGuidance.ARRIVE_TICKS] consecutive ticks.
     *
     * **The lateral target is always the recorded takeoff point** ([LandTagRun.takeoffLatDeg]) — the
     * transit translates to it and the lowering holds station over it, which is the whole of Ivan's
     * *"plan's lat/lng will slightly differ from executed one"*: the drawn coordinate is never flown
     * to. The vertical target is the phase's ([LandTagRun.transitRelAltM], then
     * [LandTagRun.armHeightM], then [LandTagRun.acquireFloorM]).
     *
     * `stopDistanceM` is the leg's own error and nothing more, because the sequence's every phase
     * **rests**: there is no leg after this one to carry speed into. That makes the law reduce literally
     * to [RepositionGuidance.clampedSpeed], which is the arithmetic every arrival this project has
     * measured was flown by.
     *
     * A null [targetRelAltM] would make the vertical axis inert; it cannot be null in any of the three
     * flying phases (the transit's comes from the gate, the lowering's and the acquisition's are
     * written at their phase transitions), and the parameter is nullable only so the shared
     * [MissionGuidance.velocity] contract is honoured verbatim rather than re-expressed.
     */
    private fun flyLandTagLegLocked(
        now: Long,
        run: MissionRun,
        land: LandTagRun,
        fix: Pair<Double, Double>,
        state: AircraftState,
        targetRelAltM: Double?,
        effects: MutableList<() -> Unit>,
    ): Boolean {
        val (errorNorth, errorEast) = RepositionGuidance.nedMetres(
            fix.first, fix.second, land.takeoffLatDeg, land.takeoffLonDeg,
        )
        val horizontalError = hypot(errorNorth, errorEast)
        val altitude = usableAltitude(state)
        val errorDown: Double? = when {
            targetRelAltM == null -> null
            altitude == null -> null
            else -> altitude - targetRelAltM
        }
        if (altitude == null && targetRelAltM != null) {
            effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        }
        var v = MissionGuidance.velocity(
            errorNorth, errorEast, errorDown,
            MissionGuidance.stopDistanceM(horizontalError, 0.0),
        )
        v = climbGatedLocked(v, effects)
        v = targetYawLocked(v, fix, state, errorNorth, errorEast, effects)

        val settled = RepositionGuidance.settled(
            horizontalError, errorDown, measuredSpeedOrNull(state),
        )
        run.arriveTicks = if (settled) run.arriveTicks + 1 else 0
        val commanded = v
        effects += { performSend(commanded, CommandSource(LAND_TAG_SOURCE, null, null)) }
        return run.arriveTicks >= RepositionGuidance.ARRIVE_TICKS
    }

    /**
     * **The hand-off**: the mission's last item is done, so the run ends and the tag descent is armed
     * for full autoland through the door the phone's own arm uses. Must hold [lock].
     *
     * Called from [PrecisionLand.Phase.ACQUIRE] and [PrecisionLand.Phase.HOLD] — through the one
     * gate call [askLandTagArmGateLocked] — on the first tick the gate clears, never on a schedule
     * and never on arrival at a height. Until 2026-07-30 it was called from [LOWER]'s arrival
     * instead, one shot at the nominal arm height; landing16 refuted that
     * (`datasets/landing16/20260730-161329.001.jsonl`: arrival at 8.8 m, ~11 px of marker, 2.7 %
     * per-frame decode in that band, a 14 s hold and no arm), and landing17 refuted its successor
     * by refusing 267 ms before the tag decoded. [PrecisionLand.Phase.ACQUIRE] and
     * [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] carry the two arguments.
     *
     * ## Why the run ends *before* the arm, rather than after it
     *
     * [descentGateLocked] refuses to arm while any manoeuvre of ours is flying — *"a descent does not
     * silently cancel it"* — and a mission is one of them. That gate is not weakened, relaxed or
     * special-cased for this caller (weakening it is the mutation this design most needs to survive);
     * instead the ordering makes it true: [mission] is nulled **under the lock**, and only then, from
     * the effect list, is [armTagDescent] called. So at every instant exactly one setpoint source
     * exists, which is the property the whole engine is arranged around.
     *
     * The run object is kept in this closure rather than in the field, which is what lets the *report*
     * to the lifecycle half depend on the arm's answer:
     *
     *  - **arm accepted** → `MISSION_ITEM_REACHED` for the land item and `onFinished`, so QGC reads the
     *    plan as complete and the descent's own sentences take over. The item genuinely is reached: the
     *    aircraft is over the recorded pad at the arm height with a believed-nadir camera, which is
     *    everything the sequence promised. What happens below that height is the descent's, on the
     *    descent's own record events, and reporting it here would be this layer claiming a touchdown it
     *    does not observe.
     *  - **arm refused** → `onPaused(`[MissionPauseCause.LAND_TAG_REFUSED]`)` and a station hold, so
     *    the plan is *not* reported complete and a second, deliberate Start may rejoin the land item
     *    and try again. The descent's own door has already announced which gate said no.
     */
    private fun armLandTagLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        land: LandTagRun,
        /**
         * **The height the aircraft was actually at when the gate cleared** — `usableAltitude` at
         * this tick, not [LandTagRun.armHeightM], which is where the sequence *started* looking.
         * The two differ by however far down the acquisition band the marker first decoded, and
         * that difference is the measurement this feature exists to accumulate: over flights it
         * says where this marker's real acquisition band is, and it is the number that would
         * justify moving either end of [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M]'s. Null only if
         * the altitude is unknown, which the gate has already refused — a fail-closed carry, not
         * a case.
         */
        armedAtAltM: Double?,
        effects: MutableList<() -> Unit>,
    ) {
        val seq = step.seq
        val sink = run.sink
        val origin = run.origin
        val armHeight = land.armHeightM
        val floor = land.acquireFloorM
        // **How long the wait at the floor had run when the gate cleared**, milliseconds — zero
        // when the arm came on the way down, which is the ordinary case and the one worth telling
        // apart from the other. It is the second half of the measurement `armedAtAltM` starts:
        // "the marker decoded at 5.4 m, 2.3 s after we stopped" is a statement about this pad and
        // this camera that accumulates across flights into the real acquisition band. landing17
        // could not say it, because the sequence refused 267 ms before the tag appeared.
        val heldMs = if (land.phase == PrecisionLand.Phase.HOLD) now - land.phaseStartedAtMs else 0L
        mission = null
        missionSetpointAtMs = null
        effects += {
            log(
                ("tag landing: the arm gate cleared at %.1fm (band %.1f→%.1f, held %dms) — arming " +
                    "the tag descent for full autoland")
                    .format(
                        armedAtAltM ?: Double.NaN, armHeight ?: Double.NaN, floor ?: Double.NaN,
                        heldMs,
                    )
            )
            record.event(
                EventCode.LAND_TAG_PHASE,
                "armed at=%.1f band=%.1f-%.1f held=%d mode=%d".format(
                    armedAtAltM ?: Double.NaN, armHeight ?: Double.NaN, floor ?: Double.NaN,
                    heldMs, land.mode,
                ),
            )
            // The same door, the same gates, the same ladder. `fullAutoland` is the whole point of
            // the item: Ivan's *"trigger a full landing including the last half a metre"*.
            val verdict = armTagDescent(origin, fullAutoland = true)
            if (verdict == Verdict.ACCEPTED) {
                sink.onItemReached(seq)
                sink.onFinished(seq)
                announce(GuidedStatusTexts.LAND_TAG_ARMED)
            } else {
                log("tag landing: the descent arm was refused ($verdict) — holding at the arm height")
                record.event(EventCode.LAND_TAG_REFUSED, "arm $verdict", warn = true)
                if (!synchronized(lock) { keepStationLocked(now) }) {
                    log("tag landing: nothing to hold with — the release path has it")
                }
                announce(GuidedStatusTexts.tagLandRefused(GuidedStatusTexts.REASON_LAND_ARM))
                sink.onPaused(MissionPauseCause.LAND_TAG_REFUSED, seq)
            }
        }
    }

    /**
     * **Every refusal of the sequence, in one place**: the run ends, the aircraft keeps station where
     * it is, and the reason is named on the record, in the log and on the operator's screen. Must hold
     * [lock].
     *
     * Ivan's *"just error out"*, and the three things it deliberately is not: not a landing by another
     * means, not DJI's go-home, and not a release of authority. The aircraft is left **hovering** at
     * whatever height the sequence reached — which is the ending M4-5 already chose for every other way
     * a mission finishes, bounded by Q1's idle window exactly as that one is.
     *
     * Reported as [MissionPauseCause.LAND_TAG_REFUSED] rather than through
     * [MissionAbortPolicy.causeOf]: nothing disengaged, so there is no `DisengageReason` to translate,
     * and inventing one would put a gesture nobody made on the record.
     */
    private fun endMissionForLandTagLocked(
        now: Long,
        run: MissionRun,
        step: MissionStep,
        reason: String,
        detail: String,
        effects: MutableList<() -> Unit>,
    ) {
        val seq = step.seq
        val sink = run.sink
        mission = null
        missionSetpointAtMs = null
        // Same rule as every other ending. In practice the landing sequence has already cleared the ROI
        // by the time it can refuse anything (`beginLandTagLocked`), so this is the gate-refusal case —
        // the sequence refused before it began, and the plan's ROI must not outlive the plan.
        endPlanRoiLocked(run, effects)
        val held = keepStationLocked(now)
        effects += {
            log("tag landing refused at item $seq: $detail — holding where we are")
            // **The detail rides the record, not only the log** (`PrecisionLand.Decision.Refused`
            // always said it should: *"the sentence for the log and the flight record, where there
            // is room to say which numbers failed"*). The short reason is the wire's, sized for the
            // `STATUSTEXT` framing; a post-flight reader needs the numbers, and for the acquisition
            // floor the detail is where the descent gate's own last word lives.
            record.event(EventCode.LAND_TAG_REFUSED, "seq=$seq $reason — $detail", warn = true)
            announce(GuidedStatusTexts.tagLandRefused(reason))
            if (!held) log("tag landing: nothing to hold with — the release path has it")
            sink.onPaused(MissionPauseCause.LAND_TAG_REFUSED, seq)
        }
    }

    /**
     * **Where this bridge believes the aircraft actually took off** — DJI's own home point, and the
     * single owner of that fact for the tag-landing sequence. See [PrecisionLand]'s KDoc for the whole
     * argument; the two conjuncts are here because both are load-bearing:
     *
     *  - `homeLocationSet == true`, DJI's own answer, because before a home exists `KeyHomeLocation`
     *    returns a *populated* `4.583662361046586E7` (measured 2026-07-26,
     *    `2026-07-26-home-position-sentinel.md`) — the coordinate cannot answer the question alone;
     *  - [Geo.coordinateOrNull], which is the bridge's one definition of a coordinate we will act on.
     *
     * Deliberately **not** shared with [descentGateLocked]'s own home read, which asks a different
     * question — *does the fix's north/east frame have an origin at all* — and answers it on the
     * coordinate alone. This asks whether that origin is the pad the aircraft left, and only DJI's flag
     * speaks to that.
     */
    private fun recordedTakeoffPoint(state: AircraftState): Pair<Double, Double>? {
        if (state.homeLocationSet != true) return null
        return Geo.coordinateOrNull(state.homeLatitude, state.homeLongitude)
    }

    /**
     * The transit leg's length for its timeout — **the 3-D distance**, metres.
     *
     * The two existing forms are the degenerate cases of this one: an ordinary leg scales its timeout
     * on the horizontal distance (a leg that barely changes height) and a takeoff climb scales it on
     * the vertical (a leg that does not translate). This is the first leg in the project that does both
     * substantially — big1.plan's is 45 m down while moving 8 m across — and the horizontal form alone
     * would give it 38 s for a descent that takes ~35 s at [GuidedEnvelope.VERTICAL_MAX_MS] before the
     * approach's own deceleration, i.e. a timeout the aircraft would lose a race with.
     */
    private fun landTagLegMetres(
        fix: Pair<Double, Double>,
        state: AircraftState,
        land: LandTagRun,
        targetRelAltM: Double,
    ): Double {
        val lateral = RepositionGuidance.horizontalMetres(
            fix.first, fix.second, land.takeoffLatDeg, land.takeoffLonDeg,
        )
        val vertical = usableAltitude(state)?.let { abs(it - targetRelAltM) } ?: 0.0
        return hypot(lateral, vertical)
    }

    /**
     * The cursor's one advance, and therefore the one place `MISSION_ITEM_REACHED` is emitted. Must
     * hold [lock].
     *
     * Everything §3.4 requires happens exactly once here: the reached edge, the new cursor, the
     * flight-record event with the measured error at the crossing, and — deliberately — **no
     * `STATUSTEXT`**, because a five-waypoint mission would produce five announcements and the
     * announcement channel is 50 bytes at severity ERROR and exists for things the operator must act
     * on. Start, finish and every refusal are announced; leg progress lives in `MISSION_CURRENT`,
     * which is what QGC actually draws.
     */
    private fun advanceMissionLocked(
        now: Long,
        run: MissionRun,
        fix: Pair<Double, Double>,
        errorM: Double,
        /** What to send on this tick — the leg's own velocity mid-plan, zero at the end. */
        commanded: StickVelocities,
        effects: MutableList<() -> Unit>,
    ) {
        val reachedSeq = run.step.seq
        val sink = run.sink
        val last = run.index >= run.route.size - 1
        // A rejoin ends at the cursor it was flying to, whatever comes next.
        run.rejoining = false
        run.arriveTicks = 0
        if (last) {
            run.finished = true
            run.holdingSinceMs = now
            effects += {
                log("mission complete at item $reachedSeq (error %.2f m) — holding, in the air".format(errorM))
                record.event(EventCode.GOTO_ARRIVED, "mission complete seq=$reachedSeq err=%.2f".format(errorM))
                sink.onItemReached(reachedSeq)
                sink.onFinished(reachedSeq)
                // The operator must be told, unmistakably, that the aircraft is holding and waiting
                // for them: this mission does not land (M4-5).
                announce(GuidedStatusTexts.MISSION_DONE_HOLDING)
                performSend(StickVelocities.ZERO, null)
            }
            // The plan is over. If its own ROI is still set — no `DO_SET_ROI_NONE` before the last item
            // — the camera stops being driven and the operator is told, rather than the hold continuing
            // to slave a camera to a finished plan. The precision `NAV_LAND` path never reaches here
            // (it ends the run from its own arm), and it clears the ROI at the item's begin anyway.
            endPlanRoiLocked(run, effects)
            return
        }
        run.index += 1
        run.legStartedAtMs = now
        // `P` for the next leg's half-plane is where the aircraft actually is, which is the origin
        // of the leg actually being flown rather than of the one that was drawn.
        run.legOriginLatDeg = fix.first
        run.legOriginLonDeg = fix.second
        val nextSeq = run.step.seq
        // The plan's ROI for the leg now beginning, applied **on the cursor's move and nowhere else** —
        // which is what makes "a `DO_` item acts when the sequence reaches it" structural rather than
        // remembered. See [applyMissionRoiLocked] for the off-by-one argument.
        applyMissionRoiLocked(run, effects)
        effects += {
            log("mission item $reachedSeq reached (error %.2f m) — cursor now $nextSeq".format(errorM))
            record.event(EventCode.GOTO_ARRIVED, "mission item=$reachedSeq err=%.2f".format(errorM))
            sink.onItemReached(reachedSeq)
            sink.onCursor(nextSeq)
            performSend(commanded, CommandSource(MISSION_SOURCE, null, null))
        }
    }

    /**
     * Drops the route and tells the executor which row of §6.2 stopped it. Must hold [lock].
     *
     * Called from every path that ends a run, so that "the engine has let go before the sink hears
     * about it" is true by construction — the field is nulled under the lock, and the notification
     * is queued as an effect that runs after it.
     */
    private fun endMissionLocked(
        run: MissionRun,
        reason: DisengageReason,
        detail: String?,
        effects: MutableList<() -> Unit>,
    ) {
        val cause = MissionAbortPolicy.causeOf(reason, detail)
        val seq = run.cursorSeq()
        val sink = run.sink
        mission = null
        missionSetpointAtMs = null
        // The plan's ROI goes with the plan — a suspended-but-remembered plan target would be
        // re-acquired by the *next* engagement's confirmation, which is a dead plan's camera command
        // arriving after a handback. A resume re-takes it from the cursor's own step.
        endPlanRoiLocked(run, effects)
        effects += {
            log("mission paused at item $seq: $cause")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause", warn = true)
            sink.onPaused(cause, seq)
        }
    }

    /**
     * Stage C's engaged tick: the orbit, between the abort ladder (already passed) and the send.
     * Must hold [lock].
     *
     * Everything the reposition tick guards is guarded here too, by the same code in the same
     * order and for the same reasons — the Q4 link watchdog, the position feed as *eyes* whose
     * staleness is absence rather than zero, and the release when the feed stays dead. What is
     * added is the circle's own two bounds: an ordinary [GuidedEnvelope.MANOEUVRE_TIMEOUT_MS] on
     * the **join leg** (an ordinary leg that has not finished in a minute is not going to) and
     * [OrbitGuidance.ORBIT_MAX_S] on the orbit as a whole, because an orbit is the first thing
     * this project flies with no natural completion.
     *
     * The gimbal is commanded from here, above the phase branch, so the camera holds the centre
     * through the join and the circle alike.
     */
    private fun tickOrbitLocked(now: Long, orb: OrbitState, effects: MutableList<() -> Unit>) {
        // The link watchdog — Q4, per controller, exactly as the reposition tick reads it.
        val gcsAt = commandingControllerSeenAtLocked(now)
        if (gcsAt == null || now - gcsAt > GuidedEnvelope.LINK_LOST_MS) {
            orbit = null
            beginReleaseLocked(now, DisengageReason.LINK_LOST, null, policy.plan(), effects)
            return
        }

        // The position feed — landmine 7. A cached fix is never flown on, and never circled on.
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) {
            val since = orb.positionStaleSinceMs ?: now.also { orb.positionStaleSinceMs = it }
            orb.arriveTicks = 0
            if (now - since > RepositionGuidance.POSITION_LOST_MS) {
                orbit = null
                beginReleaseLocked(now, DisengageReason.NO_POSITION, null, STANDARD_RELEASE, effects)
                return
            }
            effects += {
                announce(GuidedStatusTexts.NO_POSITION_HOLD)
                performSend(StickVelocities.ZERO, null)
            }
            return
        }
        orb.positionStaleSinceMs = null

        // The whole-orbit time cap. Not applied once the circle has ended: the coming-to-rest leg
        // and the hold that follows it are the ending, not more circling.
        if ((orb.phase == OrbitPhase.JOIN || orb.phase == OrbitPhase.CIRCLE) &&
            now - orb.acceptedAtMs > OrbitGuidance.ORBIT_MAX_S * 1_000L
        ) {
            beginOrbitFinishLocked(now, orb, fix, "time limit", GuidedStatusTexts.ORBIT_TIME_LIMIT, effects)
            return
        }

        // The camera, above the phase branch: open loop, rate-limited, deadbanded, and holding the
        // centre at the takeoff datum's ground level.
        //
        // **An explicit ROI outranks the orbit's own centre** (`docs/m4-mission-execution.md` §9.3):
        // the operator's live click is a statement about what they want to see, and an orbit that
        // ignored it would be circling one thing while being told to look at another. When the ROI
        // *is* the centre the two agree and there is nothing to say. The tick's own ROI pass drives
        // the camera in that case, so this call is skipped rather than duplicated — one camera, one
        // limiter, one command per tick at most.
        // The camera needs no branch here either: the orbit's centre is a target like any other,
        // and `updateRoiCameraLocked` aims at whatever `roiTrackingLocked` says — with the same
        // pitch solution the orbit's own camera path used, the same rate limiter, the same clamp.

        when (orb.phase) {
            OrbitPhase.JOIN, OrbitPhase.FINISH -> tickOrbitLegLocked(now, orb, fix, state, effects)
            OrbitPhase.CIRCLE -> tickOrbitCircleLocked(now, orb, fix, state, effects)
            OrbitPhase.HOLD -> {
                // Keep station. Q1's idle disengage runs from the hold, exactly as it does from a
                // goto's arrival: authority is not held indefinitely by a controller nobody is
                // commanding.
                val since = orb.holdingSinceMs
                if (since != null && now - since > GuidedEnvelope.IDLE_DISENGAGE_MS) {
                    orbit = null
                    beginReleaseLocked(now, DisengageReason.IDLE, null, IMMEDIATE_RELEASE, effects)
                    return
                }
                val held = roiYawLocked(StickVelocities.ZERO, fix, state, effects)
                effects += { performSend(held, null) }
            }
        }
    }

    /**
     * The **join** and the **coming-to-rest**: an ordinary resting leg toward a fixed lat/lon,
     * flown by [RepositionGuidance.velocity] and ended by [RepositionGuidance.settled] — the M3
     * law and the M3 arrival test, called rather than restated, which is what makes
     * `docs/m4-mission-execution.md` §8.2's claim checkable.
     *
     * **Yaw is exactly zero on this leg unless an ROI is set**, in both phases. The *nose-to-centre*
     * exception is scoped to circling, and a resting leg is not circling; that loop takes over at
     * the moment the tangential ramp does, with the feed-forward starting from zero speed. An
     * explicit ROI is a different permission with the same justification — the operator asked to
     * look at a place and the gimbal cannot yaw — and it is in force wherever this engine is the one
     * flying, which includes a join leg.
     */
    private fun tickOrbitLegLocked(
        now: Long,
        orb: OrbitState,
        fix: Pair<Double, Double>,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ) {
        val startedAt = orb.joinStartedAtMs ?: now.also { orb.joinStartedAtMs = it }
        if (orb.phase == OrbitPhase.JOIN && now - startedAt > orb.joinDeadlineMs) {
            orbit = null
            beginReleaseLocked(now, DisengageReason.TIMEOUT, null, STANDARD_RELEASE, effects)
            return
        }

        val (errorNorth, errorEast) = RepositionGuidance.nedMetres(
            fix.first, fix.second, orb.legLatDeg, orb.legLonDeg,
        )
        val altitude = usableAltitude(state)
        val errorDown: Double? = if (altitude != null) altitude - orb.relAltM else null
        if (errorDown == null) effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        var v = RepositionGuidance.velocity(errorNorth, errorEast, errorDown)
        v = climbGatedLocked(v, effects)
        v = roiYawLocked(v, fix, state, effects)

        val horizontalError = hypot(errorNorth, errorEast)
        val speed = measuredSpeedOrNull(state)
        val settled = RepositionGuidance.settled(horizontalError, errorDown, speed)
        orb.arriveTicks = if (settled) orb.arriveTicks + 1 else 0
        if (orb.arriveTicks >= RepositionGuidance.ARRIVE_TICKS) {
            if (orb.phase == OrbitPhase.JOIN) {
                // From rest onto the circle. The swept counter starts here and starts at zero.
                orb.phase = OrbitPhase.CIRCLE
                orb.sweptDeg = 0.0
                orb.tangentialMs = 0.0
                orb.lastCircleTickAtMs = now
                val (n, e) = RepositionGuidance.nedMetres(
                    orb.centreLatDeg, orb.centreLonDeg, fix.first, fix.second,
                )
                orb.lastBearingDeg = OrbitGuidance.bearingDeg(n, e)
                effects += {
                    log("orbit join arrived: error %.2f m, speed %.2f m/s — circling".format(horizontalError, speed))
                    record.event(EventCode.ORBIT_CIRCLING, "err=%.2f R=%.1f".format(horizontalError, orb.radiusM))
                    announce(GuidedStatusTexts.ORBIT_CIRCLING)
                    performSend(StickVelocities.ZERO, null)
                }
            } else {
                orb.phase = OrbitPhase.HOLD
                orb.holdingSinceMs = now
                effects += {
                    log("orbit at rest: error %.2f m, speed %.2f m/s — holding".format(horizontalError, speed))
                    performSend(StickVelocities.ZERO, null)
                }
            }
            return
        }

        val commanded = v
        effects += { performSend(commanded, CommandSource(ORBIT_SOURCE, null, null)) }
    }

    /**
     * The circle proper: the radial/tangential decomposition, the swept-angle counter, and the one
     * place in this bridge that may command a non-zero yaw rate.
     *
     * The order is deliberate. The swept angle is accumulated from the bearing change **before**
     * anything is commanded, so a tick that completes the circle stops it on that tick rather than
     * flying one more. The tangential speed is then ramped — `previous + a_max·dt`, never stepped —
     * and capped by [OrbitGuidance.tangentialCap], which is the curvature bound and the whole of
     * Stage C's new safety property.
     */
    private fun tickOrbitCircleLocked(
        now: Long,
        orb: OrbitState,
        fix: Pair<Double, Double>,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ) {
        val (fromCentreNorth, fromCentreEast) = RepositionGuidance.nedMetres(
            orb.centreLatDeg, orb.centreLonDeg, fix.first, fix.second,
        )
        val bearing = OrbitGuidance.bearingDeg(fromCentreNorth, fromCentreEast)
        val previousBearing = orb.lastBearingDeg
        if (previousBearing != null) {
            orb.sweptDeg += OrbitGuidance.sweptDeltaDeg(previousBearing, bearing, orb.direction)
        }
        orb.lastBearingDeg = bearing
        if (orb.sweptDeg >= orb.requiredSweepDeg) {
            beginOrbitFinishLocked(now, orb, fix, "complete", GuidedStatusTexts.ORBIT_COMPLETE, effects)
            return
        }

        val dt = orb.lastCircleTickAtMs?.let { now - it } ?: 0L
        orb.lastCircleTickAtMs = now
        orb.tangentialMs = OrbitGuidance.rampedTangential(orb.tangentialMs, orb.radiusM, dt)

        // The vertical axis is the M3 law on its own, exactly as it is during a reposition.
        val altitude = usableAltitude(state)
        val errorDown: Double? = if (altitude != null) altitude - orb.relAltM else null
        if (errorDown == null) effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        val down = RepositionGuidance.velocity(0.0, 0.0, errorDown).down

        var v = OrbitGuidance.circleVelocity(
            northFromCentreM = fromCentreNorth,
            eastFromCentreM = fromCentreEast,
            radiusM = orb.radiusM,
            direction = orb.direction,
            tangentialMs = orb.tangentialMs,
            downMs = down,
        )

        // **The yaw, from the one pointing system.** A circling aircraft always has a target —
        // the operator's explicit ROI if they set one, otherwise the circle's own centre — so this
        // is `roiYawLocked` unconditionally, and `OrbitGuidance.yawRate` is no longer a second
        // implementation of the same idea. The equivalence is not assumed: with the target at the
        // centre, `RoiGuidance.bearingRateDegPerS` reduces exactly to the orbital feed-forward, and
        // `THE ROI LAW IS THE ORBIT'S OWN LAW` asserts it across radii and speeds rather than
        // stating it in a comment. A stale heading commands zero and says so, as it did before.
        v = roiYawLocked(v, fix, state, effects)
        v = climbGatedLocked(v, effects)

        val commanded = v
        effects += { performSend(commanded, CommandSource(ORBIT_SOURCE, null, null)) }
    }

    /**
     * The circle has ended — swept out or timed out. Must hold [lock].
     *
     * The aircraft **comes to rest at the completion point** under the M3 arrival test and then
     * holds, which is the same ending a goto has; the reason is announced at the moment the circle
     * stops rather than when the deceleration finishes, because the operator's question at that
     * instant is "why did it stop circling", not "has it settled yet".
     */
    private fun beginOrbitFinishLocked(
        now: Long,
        orb: OrbitState,
        fix: Pair<Double, Double>,
        reason: String,
        text: String,
        effects: MutableList<() -> Unit>,
    ) {
        orb.phase = OrbitPhase.FINISH
        orb.legLatDeg = fix.first
        orb.legLonDeg = fix.second
        orb.arriveTicks = 0
        orb.tangentialMs = 0.0
        effects += {
            log("orbit ended ($reason): swept %.0f° of %.0f°".format(orb.sweptDeg, orb.requiredSweepDeg))
            record.event(EventCode.ORBIT_ENDED, "%s swept=%.0f".format(reason, orb.sweptDeg))
            announce(text)
            performSend(StickVelocities.ZERO, null)
        }
    }

    /**
     * **Stage D's engaged tick: the tag-tracked descent**, between the abort ladder (already
     * passed — the RC-stick rung included, which is rule 1's teeth) and the send. Must hold
     * [lock].
     *
     * Everything the reposition tick guards is guarded here too, by the same code shapes in the
     * same order — the Q4 link watchdog, the position feed as eyes whose staleness is absence
     * rather than zero, the release when the feed stays dead. What is added is the descent's
     * own consistency checks (the latch and the commanded nadir — either failing is
     * refusal-grade and ends the manoeuvre, announced) and the sighting-side machine
     * ([TagDescent]): the staleness ladder, the alignment cone, and the terminal latch.
     *
     * ## Yaw is exactly zero throughout, and the ROI does not reach in
     *
     * The descent generates no yaw (the machine's setpoints carry 0.0, structurally) and
     * deliberately does not call `roiYawLocked`/`targetYawLocked`: the tag's world fix was
     * computed under the heading the aircraft *had*, and swinging the nose mid-descent buys
     * nothing the world-frame error does not already have. An **explicit ROI set during a
     * descent ends the descent through the nadir gate below** rather than through a special
     * case: the ROI's camera half aims the gimbal at its target, the commanded pitch leaves
     * the nadir tolerance, and this tick cancels with [GuidedStatusTexts.DESCENT_GIMBAL] —
     * one property (the camera must be plumb), one place it is checked, whatever moved it.
     *
     * ## Bounds
     *
     * The Q1 [GuidedEnvelope.MANOEUVRE_TIMEOUT_MS] runs from the arm until the terminal hold —
     * a descent from 7 m at [TagDescentGuidance.V_DESCENT_MAX_MS] is ~18 s of vertical plus the
     * ladder's excursions, so 150 s is the "not going to finish" bound, reused rather than
     * re-invented. From terminal, Q1's [GuidedEnvelope.IDLE_DISENGAGE_MS] runs instead —
     * authority is not held indefinitely over a completed stage.
     *
     * **Deliberately still the flat constant after 2026-07-30**, when the goto and the orbit join
     * moved to [GuidedEnvelope.manoeuvreDeadlineMs]. A descent's commanded distance is metres, so
     * every derivation lands under the floor anyway; and its axis speeds are the descent law's
     * ([TagDescentGuidance.V_DESCENT_MAX_MS], 0.4 m/s), not the envelope's, so feeding its height
     * to a deadline built on `clampedSpeed` would be quoting the wrong law for no change in
     * behaviour. The floor is the honest answer here — see that constant's table.
     */
    private fun tickTagDescentLocked(now: Long, d: TagDescentRun, effects: MutableList<() -> Unit>) {
        // Stage C: whether this run has committed to DJI's own landing. Read once per tick —
        // several rungs below change shape once committed, and each says why at its own line.
        val committed = d.machine.phase == TagDescentPhase.DJI_LANDING

        // The link watchdog — Q4, per controller, exactly as the other manoeuvre ticks read it.
        // For a PHONE-armed descent this rung can never fire (the arm surface is this process —
        // see controllerSeenAtLocked), which is landing08's second half repaired: the descent no
        // longer inherits a heartbeat demand its own commander cannot satisfy. A MAVLINK-armed
        // descent keeps the watchdog byte-for-byte.
        val gcsAt = commandingControllerSeenAtLocked(now)
        if (gcsAt == null || now - gcsAt > GuidedEnvelope.LINK_LOST_MS) {
            tagDescent = null
            effects += { record.event(EventCode.TAG_DESCENT_ENDED, "link-lost", warn = true) }
            beginReleaseLocked(now, DisengageReason.LINK_LOST, null, policy.plan(), effects)
            return
        }

        val state = aircraftState()

        // **Touchdown — the landing's own ending, checked before every sensor rung.** Motors
        // off (or DJI reporting not-flying) during a committed landing means the wheels are
        // down and the engagement is complete: recorded, announced, released. Never altitude —
        // the barometric floor sits below zero at touchdown (`landingdata.md` §4, measured
        // relalt = −0.1 m at rest), so height is the one signal that cannot say "landed".
        // Nulls do not trigger: an unknown motor state is not evidence of anything.
        if (committed && (state.motorsOn == false || state.isFlying == false)) {
            tagDescent = null
            effects += {
                log("autoland touchdown: motorsOn=${state.motorsOn} isFlying=${state.isFlying}")
                record.event(EventCode.TAG_DESCENT_ENDED, "touchdown")
                announce(GuidedStatusTexts.DESCENT_TOUCHDOWN)
            }
            beginReleaseLocked(now, DisengageReason.TOUCHDOWN, null, IMMEDIATE_RELEASE, effects)
            return
        }

        // **The never-engaged bound.** DJI accepts commands it does not enact (measured), so a
        // commit whose flight mode never becomes a landing mode within the window is a landing
        // that is not happening: give the engagement back to a hold, named. Once the FC is
        // measurably landing the bound stops applying — a slow landing is never cut short, and
        // touchdown above is the ending.
        val committedAt = d.committedAtMs
        if (committed && committedAt != null &&
            now - committedAt > TagDescentGuidance.DJI_LAND_TIMEOUT_MS &&
            !com.dimensional.mini4pro.telemetry.Px4Mode.isDjiLandingMode(state.flightMode)
        ) {
            endDescentToHoldLocked(
                now,
                "dji landing never engaged (${now - committedAt}ms)",
                GuidedStatusTexts.DESCENT_DJI_TIMEOUT,
                effects,
            )
            return
        }

        // The landing's gimbal contingency — before everything positional, because a DJI
        // recenter does not wait for our GPS, and strictly additive: nothing here delays,
        // gates or touches anything else this tick does.
        if (committed) tickLandingGimbalLocked(now, d, effects)

        // The sensor's standing facts, re-read every tick because both can change under us —
        // **except once committed, when their loss is the landing's expected shape.** DJI
        // recenters the camera off the pad ~3 s before touchdown on every measured landing
        // (`landingdata.md` §2, landing04 §), which takes the latch and the nadir claim with
        // it; a commit that cancelled on either would abandon the landing it just asked for.
        val sense = tagSense?.invoke()
        if (!committed && (sense == null || !sense.latched || sense.latchedTagId != d.tagId)) {
            endDescentToHoldLocked(now, "latch lost", GuidedStatusTexts.DESCENT_LATCH_LOST, effects)
            return
        }
        val pitch = cameraPitchDeg()
        if (!committed && (pitch == null || !pitch.isFinite() ||
                abs(pitch - TagDescentGuidance.NADIR_PITCH_DEG) > TagDescentGuidance.NADIR_TOLERANCE_DEG)
        ) {
            endDescentToHoldLocked(
                now,
                "camera left nadir (commanded pitch $pitch)",
                GuidedStatusTexts.DESCENT_GIMBAL,
                effects,
            )
            return
        }

        // Q1's manoeuvre timeout, until the terminal hold begins. Not once committed — the
        // never-engaged bound above is the committed phase's own, tighter clock.
        if (!committed && d.terminalAtMs == null &&
            now - d.acceptedAtMs > GuidedEnvelope.MANOEUVRE_TIMEOUT_MS
        ) {
            tagDescent = null
            effects += { record.event(EventCode.TAG_DESCENT_ENDED, "timeout", warn = true) }
            beginReleaseLocked(now, DisengageReason.TIMEOUT, null, STANDARD_RELEASE, effects)
            return
        }

        if (committed) {
            // DJI is flying its own landing; we fly the odometric lateral (the phase's KDoc
            // carries the lineage and the central unknown). **The target window is FROZEN at
            // the commit edge — no ingest here, deliberately.** The previous design ("the fix
            // is still ingested — while the tag survives the first ~0.2 s after commit the
            // target window keeps rolling") lost to landing07's measurement
            // (`datasets/landing07/20260729-095413.001.jsonl`, both landings): DJI is already
            // recentering the camera within ~0.2 s of the commit (the watchdog read −76.7° at
            // commit+0.2 s in landing A, −71.1° in landing B) while `PitchBelief` —
            // commanded-wins, by design — still says −90°, so a tilted camera's pixels are
            // computed as nadir and enter the record as plausible-looking fixes: landing A's
            // believed fix stepped N +0.115 → +0.033 across the commit, landing B's continued
            // as garbage, and the newest-anchored median ingested them. Post-commit fixes are
            // structurally untrustworthy — the camera is DJI's, the belief is ours — so the
            // steering target is the median over the window ending at the newest PRE-commit
            // sample, a world-frame constant from the commit edge on. The frozen fixAtMs also
            // keeps the screen's blind flag honest: it turns blind LAND_FRESH_MS after the
            // commit, which is when the sensor's last trustworthy statement expired.

            // Landmine 7, in its landing shape: an error against a stale aircraft position is
            // not steering, it is a random walk — so a position the feed cannot vouch for
            // this tick means a dead stick, NEVER an abort (DJI keeps landing; we just stop
            // helping) and never the non-committed rung's hold-then-release ladder. Height is
            // deliberately not consulted at all: the machine's committed branch has no
            // vertical to compute.
            val here = Geo.coordinateOrNull(state.latitude, state.longitude)
            val home = Geo.coordinateOrNull(state.homeLatitude, state.homeLongitude)
            val target = TagDescentGuidance.landingTarget(d.recentFixes)
            var errorNorth = 0.0
            var errorEast = 0.0
            var steering = false
            if (here != null && home != null && target != null && state.isFresh(Signal.POSITION)) {
                steering = true
                val (aircraftNorth, aircraftEast) = RepositionGuidance.nedMetres(
                    home.first, home.second, here.first, here.second,
                )
                errorNorth = target.first - aircraftNorth
                errorEast = target.second - aircraftEast
            }
            // The steered/dead edge, recorded on change so a reader can tell a steered-blind
            // tick from a dead-stick tick without inferring it from near-zero setpoints.
            if (!steering != d.landSteerDead) {
                d.landSteerDead = !steering
                val dead = d.landSteerDead
                effects += {
                    record.event(
                        EventCode.TAG_DESCENT_PHASE,
                        if (dead) "landing steering dead (position)" else "landing steering live",
                        warn = dead,
                    )
                }
            }
            val step = d.machine.step(
                heightM = null,
                errorNorthM = errorNorth,
                errorEastM = errorEast,
                fixAgeMs = now - d.fixAtMs,
            )
            if (step is TagDescent.Step.Fly) {
                effects += { performSend(step.velocities, CommandSource(TAG_DESCENT_SOURCE, null, null)) }
            }
            return
        }

        // The position feed — landmine 7. A cached fix is never centred on. The home point is
        // required on the same terms: without it the tag's north/east has no origin.
        val here = Geo.coordinateOrNull(state.latitude, state.longitude)
        val home = Geo.coordinateOrNull(state.homeLatitude, state.homeLongitude)
        if (here == null || home == null || !state.isFresh(Signal.POSITION)) {
            val since = d.positionStaleSinceMs ?: now.also { d.positionStaleSinceMs = it }
            if (now - since > RepositionGuidance.POSITION_LOST_MS) {
                tagDescent = null
                effects += { record.event(EventCode.TAG_DESCENT_ENDED, "no position fix", warn = true) }
                beginReleaseLocked(now, DisengageReason.NO_POSITION, null, STANDARD_RELEASE, effects)
                return
            }
            effects += {
                announce(GuidedStatusTexts.NO_POSITION_HOLD)
                performSend(StickVelocities.ZERO, null)
            }
            return
        }
        d.positionStaleSinceMs = null

        sense?.let { ingestDescentFixLocked(d, it, now) }

        // Q1's idle disengage, from the terminal hold: the stage is complete and authority is
        // not held indefinitely by a controller nobody is commanding.
        val terminalAt = d.terminalAtMs
        if (terminalAt != null && now - terminalAt > GuidedEnvelope.IDLE_DISENGAGE_MS) {
            tagDescent = null
            effects += { record.event(EventCode.TAG_DESCENT_ENDED, "idle after terminal") }
            beginReleaseLocked(now, DisengageReason.IDLE, null, IMMEDIATE_RELEASE, effects)
            return
        }

        // The aircraft in the fix's own frame — the same nedMetres from the same home origin the
        // detector's CameraPose was built with, so the frame's datum cancels exactly.
        val (aircraftNorth, aircraftEast) = RepositionGuidance.nedMetres(
            home.first, home.second, here.first, here.second,
        )
        val fixAge = now - d.fixAtMs
        // **The descent's height — landing07's range ladder.** A fresh believed fix's own
        // tag-derived range (solve or size, the fix already chose — `TagFix.rangeSource`)
        // outranks the baro; the baro is the fallback (no usable tag range, stale fix). The
        // ladder and its whole argument live in one place — `TagDescentGuidance.descentHeight`
        // — because a height law is a law, and the number it answers is the SAME number that
        // scaled the fix's lateral: one owner for "the range this sighting rests on", which is
        // what dissolves landing B's cascade. The raw baro is kept beside it: the floor
        // detector flies on it alone (its own KDoc has the seam argument) and the divergence
        // record needs the instrument, not the verdict.
        val baroAltitude = usableAltitude(state)
        val descentHeight = TagDescentGuidance.descentHeight(
            baroAltitude, d.fixTagRangeM, d.fixRangeSource, fixAge,
        )
        val altitude = descentHeight?.heightM
        if (altitude == null) effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
        // The height-source edge, recorded with both instruments' numbers — the record must
        // let a reader see which instrument the descent flew on and when it switched
        // (`height_source`), including the first tick's choice.
        val source = descentHeight?.source
        if (source != d.heightSource) {
            d.heightSource = source
            val tagRange = d.fixTagRangeM
            val baro = baroAltitude
            effects += {
                val detail = when (source) {
                    RangeSource.SOLVE, RangeSource.SIZE ->
                        "%s range=%.2f baro=%s".format(
                            source.name.lowercase(), tagRange ?: Double.NaN,
                            baro?.let { "%.2f".format(it) } ?: "none",
                        )

                    RangeSource.BARO ->
                        "baro height=%.2f tag=%s".format(
                            baro ?: Double.NaN,
                            tagRange?.let { "%.2f".format(it) } ?: "none",
                        )

                    null -> "none"
                }
                log("descent height source: $detail")
                record.event(EventCode.HEIGHT_SOURCE, detail, warn = source == null)
            }
        }
        // The tag/baro divergence — **a measurement, never a gate** (the ladder already chose
        // the tag; landing07's redesign). One line per episode, both numbers on it, so a
        // post-flight read shows the barometer's lies against the instrument the descent
        // actually flew — landing B's ~1.2 m drift would have been three greps instead of a
        // root-cause session.
        val divergence = TagDescentGuidance.rangeDivergence(d.fixTagRangeM, baroAltitude)
        if (divergence != null && !d.rangeDiverged) {
            d.rangeDiverged = true
            effects += {
                log("tag/baro divergence: $divergence")
                record.event(EventCode.RANGE_BARO_DIVERGENCE, divergence, warn = true)
            }
        } else if (divergence == null) {
            d.rangeDiverged = false
        }

        val errorNorth = d.fixNorthM - aircraftNorth
        val errorEast = d.fixEastM - aircraftEast
        // The cone half of the auto-confirm gate, evaluated on every tick whose fix could
        // steer: the newest statement the sensor made about "actually over the tag", frozen —
        // never re-derived — once the fix goes stale.
        if (fixAge <= TagDescentGuidance.LAND_FRESH_MS && altitude != null) {
            d.lastFreshFixInCone =
                hypot(errorNorth, errorEast) <= TagDescentGuidance.coneRadiusM(altitude)
        }

        // The FC-floor detector runs for every descent — the `landing_stall` line is a
        // measurement whatever the arm said — and its verdict goes to the machine as a plain
        // fact. What the fact may DO is the machine's alone: the `fullAutoland` gate lives in
        // exactly one place (the law's commit condition), because a second copy here survived
        // the 2026-07-28 mutation campaign as dead code — the two-places-for-one-property
        // failure this engine's own KDoc names, caught by the protocol and removed. A plain
        // Stage B descent at the floor stalls, records, and keeps asking, as landing04 flew.
        // **Fed the BARO, never the ladder's height, deliberately**: the detector compares
        // successive altitudes, and the ladder steps at every rung transition (BARO→SIZE at
        // ~20 px, SIZE→SOLVE at the 60 px gate, and back on any loss), all inside the descent
        // — a step at any seam would fake a 0.1 m drop or phantom progress. One instrument
        // per detector; `descentHeight`'s KDoc carries the full seam argument,
        // `floorStalledLocked`'s the detector's half.
        val floorStalled = floorStalledLocked(now, d, baroAltitude, effects)
        // The commit gate's speed fact — read here because the engine owns the sensor, handed
        // to the machine because the law owns what the fact may do (floorStalled's own split).
        val lateralSpeed = lateralSpeedOrNull(state)

        // Read before the step so the band-entry handoff can say its name: an entered edge
        // out of APPROACH is the approach ending, and the record line must carry that rather
        // than leaving a reader to join it against the armed line.
        val wasApproach = d.machine.phase == TagDescentPhase.APPROACH

        when (val step = d.machine.step(
            heightM = altitude,
            errorNorthM = errorNorth,
            errorEastM = errorEast,
            fixAgeMs = fixAge,
            floorStalled = floorStalled,
            lateralSpeedM = lateralSpeed,
        )) {
            is TagDescent.Step.HandBack -> {
                val blindMs = now - d.fixAtMs
                tagDescent = null
                effects += {
                    log("tag descent: fix ${blindMs}ms old — past the abort bound, handing back")
                    record.event(EventCode.TAG_DESCENT_ENDED, "tag gone ${blindMs}ms", warn = true)
                }
                beginReleaseLocked(now, DisengageReason.TAG_LOST, null, STANDARD_RELEASE, effects)
            }

            is TagDescent.Step.Fly -> {
                d.lastDownCmd = step.velocities.down
                step.entered?.let { entered ->
                    if (entered == TagDescentPhase.TERMINAL) d.terminalAtMs = now
                    val age = now - d.fixAtMs
                    effects += {
                        log(
                            "tag descent phase: $entered (fix ${age}ms old)" +
                                if (wasApproach) " — band entry, approach over" else ""
                        )
                        record.event(
                            EventCode.TAG_DESCENT_PHASE,
                            // The band-entry marker: the one edge that ends the approach,
                            // named on its own line (the rung the ladder lands on rides
                            // beside it — a stale entry honestly reads "holding band_entry").
                            "${entered.name.lowercase()} fixAge=$age" +
                                if (wasApproach) " band_entry" else "",
                            warn = entered == TagDescentPhase.CLIMBING,
                        )
                        announce(
                            when {
                                // The band entry says its own word: TRACKING's sentence claims
                                // a *re*acquisition, and the approach's first entry is not one.
                                wasApproach && entered == TagDescentPhase.TRACKING ->
                                    GuidedStatusTexts.DESCENT_BAND_ENTRY

                                else -> when (entered) {
                                    // Structurally unreachable — nothing transitions INTO the
                                    // approach (it is a birth state) — but the enum is
                                    // exhaustive and a silent arm here would be the
                                    // silent-decline bug.
                                    TagDescentPhase.APPROACH -> GuidedStatusTexts.DESCENT_APPROACH
                                    TagDescentPhase.TRACKING -> GuidedStatusTexts.DESCENT_TRACKING
                                    TagDescentPhase.HOLDING -> GuidedStatusTexts.DESCENT_HOLDING
                                    TagDescentPhase.CLIMBING -> GuidedStatusTexts.DESCENT_CLIMBING
                                    TagDescentPhase.TERMINAL -> GuidedStatusTexts.DESCENT_COMPLETE
                                    TagDescentPhase.DJI_LANDING -> GuidedStatusTexts.DESCENT_DJI_LANDING
                                }
                            }
                        )
                    }
                    // **The commit — the one-shot land().** On the entry edge and nowhere
                    // else, which is what makes double-commit structural rather than
                    // remembered: DJI_LANDING has no exit inside the law, so this edge fires
                    // at most once per machine, and a fresh machine needs a fresh arm.
                    if (entered == TagDescentPhase.DJI_LANDING) {
                        d.committedAtMs = now
                        val height = altitude
                        // Non-null at every commit — the speed conjunct just held — but the
                        // record never asserts what the code can simply carry.
                        val vlat = lateralSpeed
                        // Which instrument the committed height came from (landing07): the
                        // post-flight reader must not have to infer whether "height=0.6" was
                        // the barometer's word or the solve's.
                        val hsrc = source?.name?.lowercase() ?: "none"
                        effects += {
                            log(
                                "autoland commit: FC floor%s at %.1fm (%s), fix %dms old, vlat %.2fm/s — asking DJI to land"
                                    .format(
                                        if (floorStalled) "" else " (terminal)",
                                        height ?: Double.NaN, hsrc, age, vlat ?: Double.NaN,
                                    )
                            )
                            record.event(
                                EventCode.LANDING_COMMIT,
                                "height=%.1f fixAge=%d floor=%b vlat=%.2f hsrc=%s"
                                    .format(height ?: Double.NaN, age, floorStalled, vlat ?: Double.NaN, hsrc),
                            )
                            val seam = djiLanding
                            val refusal = if (seam == null) "NO_LANDING_PATH" else seam.start()
                            if (refusal != null) failDjiLandingCommit(refusal)
                        }
                    }
                }
                val commanded = climbGatedLocked(step.velocities, effects)
                effects += { performSend(commanded, CommandSource(TAG_DESCENT_SOURCE, null, null)) }
            }
        }
    }

    /**
     * **Rule 1's one action, and the operator withdrawals' — `KeyStopAutoLanding`.** A manual
     * stick grab (either channel) or an explicit disarm/pause during a committed DJI landing
     * cancels our engagement like every other cancel AND asks DJI to stop the landing we
     * started: a pilot grabbing the sticks at two metres wants the aircraft, not a race with
     * an autonomous descent. Whether DJI honours the stop is a **measurement, not a promise**
     * (`docs/msdk/actions.md` §2: DJI's own UI treats forced landings as uncancellable), which
     * is why the whole exchange rides the record — the `landing_stop` verdict here, the
     * `dji_call op=stop_landing` ask/answer pair from the port, and the flight modes around
     * them. Runs outside [lock], from the abort/interrupt effects.
     */
    private fun stopDjiLanding(cause: String) {
        log("withdrawing DJI's landing ($cause) — asking KeyStopAutoLanding")
        announce(GuidedStatusTexts.DESCENT_STOP_SENT)
        val seam = djiLanding
        val refusal = if (seam == null) "NO_LANDING_PATH" else seam.stop()
        record.event(
            EventCode.LANDING_STOP,
            if (refusal == null) "asked ($cause)" else "not asked: $refusal ($cause)",
            warn = true,
        )
        if (refusal != null) log("stop-landing never left: $refusal")
    }

    /**
     * **A commit whose `land()` never left** — the seam missing, the SDK unavailable, DJI's
     * capability flag false. The machine is already committed (no exit inside the law), so the
     * honest recovery is the run dying to a hold: the aircraft keeps station where it was, the
     * refusal is named, and the only way to try again is a fresh arm through every gate.
     * Deliberately **no retry**: a command loop aimed at an aircraft is not a recovery
     * strategy (`MsdkFlightActions`' rule, applied here).
     */
    private fun failDjiLandingCommit(reason: String) {
        val effects = ArrayList<() -> Unit>(2)
        synchronized(lock) {
            val d = tagDescent ?: return
            if (d.machine.phase != TagDescentPhase.DJI_LANDING) return
            endDescentToHoldLocked(
                nowMs(),
                "commit failed: $reason",
                GuidedStatusTexts.autolandCommitFailed(reason),
                effects,
            )
        }
        for (effect in effects) effect()
    }

    /**
     * **The landing's gimbal contingency — `landingdata.md` §4 Option 2, run as the experiment
     * it is.** During a committed LANDING, if the *reported* gimbal pitch has left nadir by
     * more than [TagDescentGuidance.GIMBAL_WATCHDOG_DEG] — the measured DJI recenter, which
     * slews at 250–300 °/s and crosses the threshold within ~20 ms — re-command −90° through
     * the one existing camera path ([manoeuvreGimbal] → `GimbalManager`, the recorded,
     * deliberately-uninterlocked seam `docs/gimbal.md` argues for), at most once per
     * [TagDescentGuidance.GIMBAL_WATCHDOG_MIN_MS] and
     * [TagDescentGuidance.GIMBAL_WATCHDOG_MAX] times in all. Whether the FC honours a nadir
     * command in `CONFIRM_LANDING` — or refuses it, or honours and immediately re-centres —
     * is precisely the open unknown the flight measures; every attempt is a
     * `gimbal_watchdog` event beside its `dji_call op=gimbal_rotate` pair, a refusal comes
     * back as DJI's error name verbatim on the gimbal's own announce path, and past the bound
     * the landing simply continues blind, logged once. What it buys if it sticks is the
     * *recording* of the touchdown; it buys no control, and it must never delay the descent
     * command — strictly additive effects, nothing else touched. Must hold [lock].
     */
    private fun tickLandingGimbalLocked(now: Long, d: TagDescentRun, effects: MutableList<() -> Unit>) {
        val gimbal = manoeuvreGimbal ?: return
        val reported = gimbalReportedPitchDeg() ?: return
        if (!reported.isFinite() ||
            abs(reported - TagDescentGuidance.NADIR_PITCH_DEG) <= TagDescentGuidance.GIMBAL_WATCHDOG_DEG
        ) {
            return
        }
        if (d.gimbalNudges >= TagDescentGuidance.GIMBAL_WATCHDOG_MAX) {
            if (!d.gimbalGaveUp) {
                d.gimbalGaveUp = true
                effects += {
                    log(
                        "landing gimbal watchdog: DJI keeps the camera off nadir after " +
                            "${TagDescentGuidance.GIMBAL_WATCHDOG_MAX} re-commands — continuing blind"
                    )
                    record.event(
                        EventCode.GIMBAL_WATCHDOG,
                        "gave up after ${TagDescentGuidance.GIMBAL_WATCHDOG_MAX} - continuing blind",
                        warn = true,
                    )
                }
            }
            return
        }
        val last = d.gimbalNudgeAtMs
        if (last != null && now - last < TagDescentGuidance.GIMBAL_WATCHDOG_MIN_MS) return
        d.gimbalNudgeAtMs = now
        d.gimbalNudges++
        val attempt = d.gimbalNudges
        effects += {
            log("landing gimbal watchdog: reported pitch %.1f — re-commanding nadir (attempt %d)"
                .format(reported, attempt))
            record.event(
                EventCode.GIMBAL_WATCHDOG,
                "re-command nadir attempt=%d reported=%.1f".format(attempt, reported),
                warn = true,
            )
            gimbal.aimPitch(TagDescentGuidance.NADIR_PITCH_DEG)
        }
    }

    /**
     * **The FC-floor detector — landing04's measurement, promoted to the commit trigger.**
     * True when the altitude has refused to fall by even one quantum
     * ([TagDescentGuidance.LAND_STALL_DROP_M]) for [TagDescentGuidance.LAND_STALL_MS] while
     * descent was being commanded ([TagDescentRun.lastDownCmd] — the previous tick's command,
     * because a stall is a claim about what already happened to a command that already went
     * out). The measured shape it names: downward obstacle sensing pinning a virtual-stick
     * descent at 1.4 m for 12 s under a continuous `vd = +0.4` (landing04, t≈81–94 s).
     *
     * **[altitudeM] must be the barometric altitude, never the range ladder's height** —
     * since landing07 the descent laws fly the ladder, but this detector compares successive
     * readings, and only a single instrument's deltas mean "the aircraft moved": the ladder
     * steps at every rung transition (all of which live inside the descent band), and a step
     * across the reference would fake a quantum of drop — resetting the window and deferring
     * the very commit this detector exists to fire — or a phantom stall. The baro's *deltas*
     * stay honest under its measured drift (~1.2 m over ~40 s on landing07 B ≈ 0.03 m/s:
     * 0.06 m across the 2 s window, under one quantum), which is all a progress detector
     * consumes; its absolute lies are the ladder's problem and are handled there.
     *
     * Ticks that command no descent — the cone's own gating wobbles `vd` 0/0.4 at the floor,
     * measured in the same record — neither advance nor reset the window; only altitude
     * *progress* resets it, so the detector reads through the wobble. Each stall episode still
     * writes its `landing_stall` line once: the verdict became load-bearing, the measurement
     * stayed. Must hold [lock].
     */
    private fun floorStalledLocked(
        now: Long,
        d: TagDescentRun,
        altitudeM: Double?,
        effects: MutableList<() -> Unit>,
    ): Boolean {
        if (altitudeM == null || d.lastDownCmd <= 0.0) return false
        val ref = d.stallRefAltitudeM
        val refAt = d.stallRefAtMs
        if (ref == null || refAt == null || altitudeM <= ref - TagDescentGuidance.LAND_STALL_DROP_M) {
            // Progress (or the first look): new reference, episode over.
            d.stallRefAltitudeM = altitudeM
            d.stallRefAtMs = now
            d.stallRecorded = false
            return false
        }
        val heldMs = now - refAt
        if (heldMs <= TagDescentGuidance.LAND_STALL_MS) return false
        if (!d.stallRecorded) {
            d.stallRecorded = true
            effects += {
                log("landing stall: altitude %.1fm unchanged for %dms under a down command"
                    .format(altitudeM, heldMs))
                record.event(
                    EventCode.LANDING_STALL,
                    "altitude %.1f held %dms".format(altitudeM, heldMs),
                    warn = true,
                )
            }
        }
        return true
    }

    /**
     * A descent ending for a reason that leaves the aircraft **ours and holding** — latch lost,
     * camera off nadir, or any future refusal-grade inconsistency. Must hold [lock].
     *
     * The hold is the goto's own arrival hold, synthesised at the current fix exactly as
     * [pause] and a mission pause synthesise it — one implementation of "stop and hold". The
     * descent state is dropped, never suspended: re-arming is a fresh [armTagDescent] through
     * every gate.
     */
    private fun endDescentToHoldLocked(
        now: Long,
        detail: String,
        text: String,
        effects: MutableList<() -> Unit>,
    ) {
        tagDescent = null
        if (phaseLocked == GuidedPhase.ENGAGED) {
            val state = aircraftState()
            val here = Geo.coordinateOrNull(state.latitude, state.longitude)
            if (here != null && state.isFresh(Signal.POSITION)) {
                reposition = RepositionState.holdAt(
                    here.first, here.second,
                    relAltM = state.relativeAltitude ?: 0.0,
                    nowMs = now,
                )
            }
        }
        effects += {
            log("tag descent ended: $detail")
            record.event(EventCode.TAG_DESCENT_ENDED, detail, warn = true)
            announce(text)
            performSend(StickVelocities.ZERO, null)
        }
    }

    /**
     * Ingest the newest believed fix: **id-matched against the latch** — a fix decoding any
     * other id is a measured failure mode (2 false ids in 1978 frames), not evidence — and
     * **monotonic**, so a re-read of the same sighting never rejuvenates the age. Must hold
     * [lock]. One implementation, fed by the live tick and the shadow tick alike.
     */
    private fun ingestDescentFixLocked(d: TagDescentRun, sense: TagDescentSense, now: Long) {
        val fixAge = sense.fixAgeMs
        if (sense.fixTagId == d.tagId && sense.fixNorthM != null && sense.fixEastM != null &&
            fixAge != null
        ) {
            val atMs = now - fixAge
            if (atMs > d.fixAtMs) {
                d.fixNorthM = sense.fixNorthM
                d.fixEastM = sense.fixEastM
                d.fixAtMs = atMs
                // The newest fix's own range facts, nulls included: a baro-scaled fix
                // superseding a tag-scaled one takes the tag range with it — the run's fix
                // facts always describe one frame, never a composite of two.
                d.fixTagRangeM = sense.fixTagRangeM
                d.fixRangeSource = sense.fixRangeSource
                // The landing-target window rides the same acceptance: only believed fixes
                // enter, in time order (monotonic by construction here), pruned to the window
                // behind the newest so the buffer is bounded by the 10 Hz cap (≤ ~6 entries).
                d.recentFixes += TagDescentGuidance.FixSample(atMs, sense.fixNorthM, sense.fixEastM)
                d.recentFixes.removeAll { it.atMs < atMs - TagDescentGuidance.LAND_TARGET_WINDOW_MS }
            }
        }
    }

    // ------------------------------------------------------------- shadow mode

    /**
     * Enable or disable **shadow mode** — see [ShadowRun] for what it is and why. Safe from
     * any thread, idempotent; returns whether anything changed.
     *
     * Enabling takes no gates at all: the mode is a recorder, and the gates decide per tick
     * whether a *segment* may arm, with each blocker recorded on its change. Disabling ends
     * any running segment, recorded and announced.
     */
    fun setShadowDescent(on: Boolean): Boolean {
        val hadSegment: Boolean
        synchronized(lock) {
            if (on == (shadowDescent != null)) return false
            hadSegment = shadowDescent?.run != null
            shadowDescent = if (on) ShadowRun() else null
        }
        if (on) {
            log("shadow descent mode on — computing everything, actuating nothing")
            record.event(EventCode.TAG_DESCENT_PHASE, "shadow mode on")
            announce(GuidedStatusTexts.DESCENT_SHADOW_ON)
        } else {
            log("shadow descent mode off (operator)")
            if (hadSegment) record.event(EventCode.TAG_DESCENT_ENDED, "shadow disarmed")
            record.event(EventCode.TAG_DESCENT_PHASE, "shadow mode off")
            announce(GuidedStatusTexts.shadowOff("disarmed"))
        }
        return true
    }

    /**
     * The comparison view's feed: the shadow's newest would-be command (null when older than
     * [SHADOW_CMD_STALE_MS] — a frozen arrow reads as a live opinion) beside the operator's
     * current RC sticks mapped through [StickMapping.rcVelocities] — **the** mapping, not a
     * second one — and rotated from the sticks' body frame into the same north/east frame the
     * controller commands in, using the same heading-freshness rule every other consumer of
     * `yawDeg` applies (no usable heading → no operator arrow, never a wrong-frame one).
     *
     * Null when shadow mode is off. **Display-only, one-way**: flat values out, nothing the
     * caller can hand back in, and nothing in the law knows this exists.
     */
    fun shadowComparison(now: Long = nowMs()): ShadowComparison? {
        val state = aircraftState()
        return synchronized(lock) {
            val shadow = shadowDescent ?: return null
            val cmd = shadow.lastCmd?.takeIf {
                val at = shadow.lastCmdAtMs
                at != null && now - at <= SHADOW_CMD_STALE_MS
            }
            val operator = rc?.let { StickMapping.rcVelocities(it) }?.let { body ->
                val heading = usableHeading(state) ?: return@let null
                val yawRad = Math.toRadians(heading)
                StickVelocities(
                    north = body.north * kotlin.math.cos(yawRad) - body.east * kotlin.math.sin(yawRad),
                    east = body.north * kotlin.math.sin(yawRad) + body.east * kotlin.math.cos(yawRad),
                    down = body.down,
                    yawRateDegPerS = body.yawRateDegPerS,
                )
            }
            ShadowComparison(
                segmentArmed = shadow.run != null,
                blocker = if (shadow.run == null) shadow.lastBlocker else null,
                shadow = cmd,
                operator = operator,
            )
        }
    }

    /** One frame of the live comparison view. Flat, immutable, no way back into the engine. */
    data class ShadowComparison(
        /** True while a shadow segment is armed; false between segments (gates blocking). */
        val segmentArmed: Boolean,
        /**
         * The gate currently refusing a segment, or null while one is armed. **On the screen
         * because its absence flew**: on 2026-07-28 (flight 152922) Ivan enabled shadow above
         * the band with the tag out of view, no segment could arm, and he stared at empty
         * screen space unable to tell shadow-broken from shadow-blocked. The blocker text is
         * that distinction, where the arrows will appear.
         */
        val blocker: String?,
        /** The would-be command, or null when none has been computed recently. */
        val shadow: StickVelocities?,
        /** The operator's sticks as envelope-scaled NE velocities, or null without feed/heading. */
        val operator: StickVelocities?,
    )

    /**
     * The shadow's tick — every phase, every 100 ms, from [tick]'s top exactly as the pending
     * takeoff climb is, because shadow belongs to no engagement phase: its whole point is to
     * run while the operator hand-flies (engine IDLE) or flies the GCS sticks (ENGAGED
     * passthrough). Must hold [lock].
     *
     * The structure mirrors [tickTagDescentLocked] rung for rung, with two substitutions that
     * are the whole of shadow: a cancel-class fact **ends the segment** (recorded, in the live
     * exit's own words) instead of releasing an authority that was never held, and a stick
     * deflection is **recorded as a would-cancel edge** instead of ending anything, because in
     * shadow the operator is flying and a deflection is the flight, not a takeover.
     */
    private fun tickShadowLocked(now: Long, shadow: ShadowRun, effects: MutableList<() -> Unit>) {
        val sense = tagSense?.invoke()
        val d = shadow.run
        if (d == null) {
            // Between segments: auto-arm the moment a live arm would be accepted — the same
            // gate function, so "the shadow armed here" is evidence about the live gates. The
            // interlock is a named blocker rather than the live arm's silent UNSUPPORTED.
            val gate = if (!interlockEnabled()) {
                DescentGate.Blocked("interlock off", "the command interlock is off")
            } else {
                // The origin a *live* arm would carry: the descent's one arm surface is the
                // phone ([armTagDescentFromPhone]), so shadow judges the gate at PHONE — that is
                // what makes shadow evidence transferable to live, gate for gate. The old
                // spelling (`engagementOrigin ?: MAVLINK`) read as LINK_DOWN for the whole of a
                // hand-flown phone-only flight — precisely the flight shadow mode exists for
                // (landing08 measured the same mislabel refusing the live arms).
                descentGateLocked(sense, ControlOrigin.PHONE, now)
            }
            when (gate) {
                is DescentGate.Blocked -> if (gate.reason != shadow.lastBlocker) {
                    shadow.lastBlocker = gate.reason
                    effects += {
                        log("shadow descent blocked: ${gate.detail}")
                        record.event(EventCode.TAG_DESCENT_PHASE, "shadow blocked ${gate.reason}")
                    }
                }

                is DescentGate.Clear -> {
                    shadow.lastBlocker = null
                    shadow.rcWouldCancel = false
                    shadow.saidComplete = false
                    shadow.run = TagDescentRun(
                        tagId = gate.tagId,
                        acceptedAtMs = now,
                        fixNorthM = gate.fixNorthM,
                        fixEastM = gate.fixEastM,
                        fixAtMs = now - gate.fixAgeMs,
                        fixTagRangeM = gate.fixTagRangeM,
                        fixRangeSource = gate.fixRangeSource,
                        // The same gate verdict a live arm would take — an above-band shadow
                        // segment shadows the approach, or its evidence transfers nothing.
                        approach = gate.approach,
                    )
                    effects += {
                        log(
                            "shadow descent armed: id=%d height=%.1fm fixAge=%dms%s"
                                .format(
                                    gate.tagId, gate.altitudeM, gate.fixAgeMs,
                                    if (gate.approach) " approach" else "",
                                )
                        )
                        record.event(
                            EventCode.TAG_DESCENT_ARMED,
                            "shadow id=%d height=%.1f fixAge=%d%s"
                                .format(
                                    gate.tagId, gate.altitudeM, gate.fixAgeMs,
                                    if (gate.approach) " approach" else "",
                                ),
                        )
                    }
                }
            }
            return
        }

        // The would-cancel edge, first and **never acted on**: the same debounced verdict the
        // live abort ladder uses, recorded with the magnitude that crossed the dead-band.
        if (rcAbortDueLocked(now)) {
            if (!shadow.rcWouldCancel) {
                shadow.rcWouldCancel = true
                val deflection = rc?.maxDeflection()
                effects += {
                    record.event(
                        EventCode.TAG_DESCENT_PHASE,
                        "shadow would-cancel sticks %.2f".format(deflection ?: Double.NaN),
                    )
                }
            }
        } else {
            shadow.rcWouldCancel = false
        }

        // Segment-ending facts — the live tick's rungs, in its order, ending only the segment.
        if (!interlockEnabled()) {
            endShadowSegmentLocked(shadow, "interlock", effects)
            return
        }
        // No link-lost rung here, deliberately — it existed until 2026-07-29 and was deleted
        // rather than left dead: the live descent this segment mirrors would be a PHONE-origin
        // arm, whose liveness is identity (controllerSeenAtLocked), so the mirrored rung can
        // structurally never fire and keeping it would be code that claims a check it cannot
        // make. Under the old MAVLINK-only spelling this rung ended every shadow segment of a
        // QGC-less flight at birth — the same landing08 mislabel, on the recording side.
        if (sense == null || !sense.latched || sense.latchedTagId != d.tagId) {
            endShadowSegmentLocked(shadow, "latch lost", effects)
            return
        }
        val pitch = cameraPitchDeg()
        if (pitch == null || !pitch.isFinite() ||
            abs(pitch - TagDescentGuidance.NADIR_PITCH_DEG) > TagDescentGuidance.NADIR_TOLERANCE_DEG
        ) {
            endShadowSegmentLocked(shadow, "camera left nadir (commanded pitch $pitch)", effects)
            return
        }
        if (d.terminalAtMs == null && now - d.acceptedAtMs > GuidedEnvelope.MANOEUVRE_TIMEOUT_MS) {
            endShadowSegmentLocked(shadow, "timeout", effects)
            return
        }
        val state = aircraftState()
        val here = Geo.coordinateOrNull(state.latitude, state.longitude)
        val home = Geo.coordinateOrNull(state.homeLatitude, state.homeLongitude)
        if (here == null || home == null || !state.isFresh(Signal.POSITION)) {
            val since = d.positionStaleSinceMs ?: now.also { d.positionStaleSinceMs = it }
            if (now - since > RepositionGuidance.POSITION_LOST_MS) {
                endShadowSegmentLocked(shadow, "no position fix", effects)
                return
            }
            recordShadowCmd(shadow, StickVelocities.ZERO, now, effects)
            return
        }
        d.positionStaleSinceMs = null
        ingestDescentFixLocked(d, sense, now)
        val terminalAt = d.terminalAtMs
        if (terminalAt != null && now - terminalAt > GuidedEnvelope.IDLE_DISENGAGE_MS) {
            endShadowSegmentLocked(shadow, "idle after terminal", effects)
            return
        }

        val (aircraftNorth, aircraftEast) = RepositionGuidance.nedMetres(
            home.first, home.second, here.first, here.second,
        )
        // The same range-ladder height the live tick flies (rung-for-rung mirroring is what
        // makes shadow evidence transferable); the shadow writes no height_source or
        // divergence lines of its own — its record is the stick_cmd stream.
        val shadowHeight = TagDescentGuidance.descentHeight(
            usableAltitude(state), d.fixTagRangeM, d.fixRangeSource, now - d.fixAtMs,
        )
        when (val step = d.machine.step(
            heightM = shadowHeight?.heightM,
            errorNorthM = d.fixNorthM - aircraftNorth,
            errorEastM = d.fixEastM - aircraftEast,
            fixAgeMs = now - d.fixAtMs,
        )) {
            is TagDescent.Step.HandBack -> {
                endShadowSegmentLocked(shadow, "tag gone ${now - d.fixAtMs}ms", effects)
            }

            is TagDescent.Step.Fly -> {
                step.entered?.let { entered ->
                    if (entered == TagDescentPhase.TERMINAL) d.terminalAtMs = now
                    val age = now - d.fixAtMs
                    val sayComplete = entered == TagDescentPhase.TERMINAL && !shadow.saidComplete
                    if (sayComplete) shadow.saidComplete = true
                    effects += {
                        log("shadow descent phase: $entered (fix ${age}ms old)")
                        record.event(
                            EventCode.TAG_DESCENT_PHASE,
                            "shadow ${entered.name.lowercase()} fixAge=$age",
                        )
                        if (sayComplete) announce(GuidedStatusTexts.DESCENT_SHADOW_COMPLETE)
                    }
                }
                recordShadowCmd(shadow, step.velocities, now, effects)
            }
        }
    }

    /**
     * One shadow segment ending — recorded in the live exit's own words, shadow-marked, with
     * nothing released because nothing was held. The mode survives: [tickShadowLocked] re-arms
     * the next segment the moment the gates hold again. Must hold [lock].
     */
    private fun endShadowSegmentLocked(
        shadow: ShadowRun,
        detail: String,
        effects: MutableList<() -> Unit>,
    ) {
        shadow.run = null
        shadow.lastBlocker = null
        shadow.lastCmd = null
        shadow.lastCmdAtMs = null
        effects += {
            log("shadow descent segment ended: $detail")
            record.event(EventCode.TAG_DESCENT_ENDED, "shadow $detail", warn = true)
        }
    }

    /**
     * One would-be command: to the flight record and the comparison view, **never to the
     * port** — there is deliberately no path from here to [performSend], and the suite pins
     * that a shadow session leaves the port untouched from first tick to last. Must hold
     * [lock]; the record write rides the effect list like every other side effect.
     */
    private fun recordShadowCmd(
        shadow: ShadowRun,
        v: StickVelocities,
        now: Long,
        effects: MutableList<() -> Unit>,
    ) {
        shadow.lastCmd = v
        shadow.lastCmdAtMs = now
        effects += {
            record.stickCmd(
                setpoint = Setpoint(
                    frame = SetpointFrame.NED_VELOCITY,
                    north = v.north,
                    east = v.east,
                    down = v.down,
                    yawRateDegPerS = v.yawRateDegPerS,
                ),
                axes = StickMapping.toDji(v),
                modes = StickModes.UNKNOWN,
                source = CommandSource(TAG_DESCENT_SHADOW_SOURCE, null, null),
                range = null,
                accepted = null,
                error = null,
            )
        }
    }


    /**
     * The region of interest's camera half, once per tick, in **every** phase. Must hold [lock].
     *
     * This absorbed the orbit's own camera path on 2026-07-27 and shares all of its arithmetic — the same
     * [OrbitGuidance.gimbalPitchDeg] solution against a target assumed to sit at the takeoff datum's
     * ground level, the same reachable-angle clamp, the same rate limiter and deadband. What it adds
     * is the two things an ROI has and an orbit does not: a solution that **moves continuously**
     * (the aircraft flies past the target rather than around it, so the limiter earns its keep here
     * in a way it never did there), and a close-in regime where the geometry stops being worth
     * chasing.
     *
     * **Open loop, and there is nothing here that could be otherwise.** No gimbal attitude is read,
     * no age is consulted, and no timeout waits on the camera having arrived. [ManoeuvreGimbal]
     * cannot express an age, which is how the rule is enforced rather than remembered — and an ROI
     * held on a hovering aircraft is exactly the silence `KeyGimbalAttitude` produces when it is
     * healthy.
     *
     * Absence, never zero: a stale fix, an unusable altitude or a target inside
     * [RoiGuidance.MIN_RANGE_M] all mean *no new command*, and the last angle stands. The camera is
     * never swung to a default, because a default is a lie about where the target is.
     */
    private fun updateRoiCameraLocked(now: Long, effects: MutableList<() -> Unit>) {
        val target = roiTrackingLocked() ?: return
        val gimbal = manoeuvreGimbal ?: return
        val state = aircraftState()
        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
        if (fix == null || !state.isFresh(Signal.POSITION)) return
        val altitude = usableAltitude(state) ?: return
        val horizontal = RepositionGuidance.horizontalMetres(
            fix.first, fix.second, target.latDeg, target.lonDeg,
        )

        // The azimuth half's honesty, evaluated whether or not the pitch half commands anything: an
        // operator whose subject is off to one side is owed the reason, and the reason is that this
        // is not our aircraft to turn.
        announceAzimuthLocked(target, fix, state, effects)

        if (RoiGuidance.tooClose(horizontal)) {
            // Ill-conditioned: the solution runs to −90° and then swings with every metre of GPS
            // noise. Hold the last angle and say so once — a camera hunting at the limit is worse
            // than a camera pointing approximately down.
            if (!target.saidTooClose) {
                target.saidTooClose = true
                effects += { announce(GuidedStatusTexts.ROI_TOO_CLOSE) }
            }
            return
        }
        target.saidTooClose = false

        // **The height the camera has to look down through**, which is the aircraft's height above the
        // *target* rather than above the ground: a target with a readable height of its own
        // ([RoiState.relAltM], set only for the frames whose z is our own datum) subtracts, and a
        // target without one keeps the measured ground-level assumption by subtracting nothing. A
        // subject above the aircraft makes this negative, which solves to a positive (upward) pitch and
        // is clamped-and-announced by `reachablePitch` below like any other unreachable angle.
        val heightAboveTarget = altitude - (target.relAltM ?: 0.0)
        val solution = OrbitGuidance.gimbalPitchDeg(heightAboveTarget, horizontal)
        val (commanded, outOfRange) = reachablePitch(solution, gimbal)
        if (outOfRange) {
            if (!target.saidGimbalRange) {
                target.saidGimbalRange = true
                effects += { announce(GuidedStatusTexts.ROI_GIMBAL_RANGE) }
            }
        } else {
            target.saidGimbalRange = false
        }
        if (!gimbalDueLocked(now, commanded)) return
        effects += {
            // Re-read the suspension at run time rather than trusting the decision taken above: an
            // abort queued earlier in *this* tick has already run by now, and §9.5's rule is that no
            // gimbal command goes out once the aircraft has been handed back.
            val stillTracking = synchronized(lock) { roiTrackingLocked() != null }
            if (stillTracking) aimCamera(gimbal, commanded)
        }
    }

    /**
     * Says [GuidedStatusTexts.ROI_PITCH_ONLY] when the nose is more than
     * [RoiGuidance.YAW_DEADBAND_DEG] off the target's bearing **and nothing of ours is flying**.
     * Must hold [lock].
     *
     * The line this draws is the whole of §9.3: *we point the camera freely; we point the aircraft
     * only when we are already the one flying it.* An RC pilot's aircraft is not turned by us, ever,
     * and neither is one being flown by the operator's own sticks through the Stage A passthrough —
     * that hand is relaying a yaw of its own, and ours would be fighting it.
     */
    private fun announceAzimuthLocked(
        target: RoiState,
        fix: Pair<Double, Double>,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ) {
        if (yawAuthorityLocked()) {
            target.saidPitchOnly = false
            return
        }
        val (losNorth, losEast) = RepositionGuidance.nedMetres(
            fix.first, fix.second, target.latDeg, target.lonDeg,
        )
        val error = RoiGuidance.azimuthErrorDeg(losNorth, losEast, usableHeading(state))
        if (error == null || abs(error) <= RoiGuidance.YAW_DEADBAND_DEG) {
            target.saidPitchOnly = false
            return
        }
        if (!target.saidPitchOnly) {
            target.saidPitchOnly = true
            effects += { announce(GuidedStatusTexts.ROI_PITCH_ONLY) }
        }
    }

    /**
     * The ROI's yaw half, applied to a manoeuvre's setpoint. Must hold [lock].
     *
     * Returns [v] **untouched** when there is no ROI — which is what keeps M3's "this bridge never
     * generates yaw" true everywhere it was true before — and otherwise replaces its yaw rate with
     * [RoiGuidance.yawRate]: the same bounded closed loop the orbit's nose-to-centre uses, the same
     * [GuidedEnvelope.YAW_RATE_MAX_DEGS] clamp, the same stale-heading-commands-zero rule, and a
     * feed-forward computed from the setpoint the aircraft is about to be given.
     *
     * **Called only from the two manoeuvre ticks**, which is the enforcement of the scope: this
     * engine can only reach them while DJI reports it holds virtual-stick authority, which requires
     * the interlock. There is deliberately no second interlock check here — one property, one place.
     */
    private fun roiYawLocked(
        v: StickVelocities,
        fix: Pair<Double, Double>,
        state: AircraftState,
        effects: MutableList<() -> Unit>,
    ): StickVelocities {
        val target = roiTrackingLocked() ?: return v
        val (losNorth, losEast) = RepositionGuidance.nedMetres(
            fix.first, fix.second, target.latDeg, target.lonDeg,
        )
        val heading = usableHeading(state)
        if (heading == null) effects += { announce(GuidedStatusTexts.ROI_NO_HEADING) }
        return v.copy(
            yawRateDegPerS = RoiGuidance.yawRate(
                losNorthM = losNorth,
                losEastM = losEast,
                headingDeg = heading,
                commandedNorthMs = v.north,
                commandedEastMs = v.east,
            )
        )
    }

    /**
     * **The nose, for a leg flown toward a target** — a plain goto or a mission leg. Must hold
     * [lock].
     *
     * The priority order is the decision doc's, in one place so it cannot be honoured differently in
     * two ticks:
     *
     *  1. **An ROI wins.** The operator said "look at that", so the camera is the point of the
     *     flight and the nose serves the camera. Facing forward is the *default*, not a priority.
     *  2. **Otherwise heading follows course**, when [headingFollowsCourse] is on — the bearing to
     *     the target under [HeadingGuidance], which is the orbit's bounded closed loop with the
     *     feed-forward dropped.
     *  3. **Otherwise the setpoint is returned untouched**, yaw included, which is M3's rule intact
     *     and is exactly what the flag turns the aircraft back to.
     *
     * Inside [RepositionGuidance.R_ACCEPT_M] — which includes a vertical-only leg, whose horizontal
     * error is zero — the nose **holds**, because a metre of GPS noise swings the bearing wildly
     * when you are nearly on top of the target. A stale heading commands zero and announces; the
     * translation continues, which is the graduated treatment every other feed gets here.
     *
     * Note the yaw rate is set to exactly zero in the hold and stale cases rather than left alone:
     * the incoming setpoint's yaw is always zero from the guidance law, but writing it makes the
     * property "this branch never *generates* yaw" true by construction rather than by inheritance.
     */
    private fun targetYawLocked(
        v: StickVelocities,
        fix: Pair<Double, Double>,
        state: AircraftState,
        errorNorthM: Double,
        errorEastM: Double,
        effects: MutableList<() -> Unit>,
    ): StickVelocities {
        if (roiTrackingLocked() != null) return roiYawLocked(v, fix, state, effects)
        if (!headingFollowsCourse()) return v
        if (HeadingGuidance.holdingHeading(errorNorthM, errorEastM)) {
            return v.copy(yawRateDegPerS = 0.0)
        }
        val heading = usableHeading(state)
        if (heading == null) {
            effects += { announce(GuidedStatusTexts.HEADING_NO_HEADING) }
            return v.copy(yawRateDegPerS = 0.0)
        }
        return v.copy(yawRateDegPerS = HeadingGuidance.yawRate(errorNorthM, errorEastM, heading))
    }

    /**
     * The ROI being tracked right now, or null — no target, or one suspended by an abort. Must hold
     * [lock]. The single place "is an ROI in force?" is answered, so the abort's suspension cannot
     * be honoured in one branch and forgotten in another.
     */
    /**
     * **The one target this engine is pointing at**, or null when it is pointing at nothing. Must
     * hold [lock].
     *
     * The precedence, which is the whole of M4-6's *"orbit implies ROI"* made concrete:
     *
     *  1. **an explicit ROI wins**, always. The operator's click says what they want in frame, and
     *     pointing at a circle's centre instead would be obeying the shape of the manoeuvre over
     *     its purpose.
     *  2. otherwise **the running orbit's centre**, which is why a join leg turns towards the
     *     circle instead of flying to it sideways.
     *  3. otherwise nothing.
     *
     * The implied one is gated on `orbit != null` rather than being cleared when an orbit ends.
     * There are eleven places an orbit can end — every rung of the abort ladder, three timeouts,
     * a replacement, a completion — and a rule that has to be remembered at eleven sites is a rule
     * that will be forgotten at one of them. Gating it here makes "the implied ROI dies with the
     * orbit" true by construction, and `DO_SET_ROI_NONE` giving the centre back needs no code at
     * all: clearing [roi] simply exposes what was underneath.
     */
    private fun roiTrackingLocked(): RoiState? {
        roi?.let { return it.takeIf { r -> !r.suspended } }
        return orbitRoi?.takeIf { !it.suspended }
    }

    /**
     * Whether this engine may turn the aircraft for an ROI. Must hold [lock].
     *
     * True only while DJI-confirmed authority is held **and one of our own manoeuvres is flying** —
     * a goto, an orbit, or a mission leg. Deliberately false during
     * stick passthrough: the operator's own hand is on the yaw axis there and [StickMapping] is
     * already relaying it, so an ROI yaw would be this bridge overruling the person flying rather
     * than flying on their behalf.
     */
    private fun yawAuthorityLocked(): Boolean =
        phaseLocked == GuidedPhase.ENGAGED && (reposition != null || orbit != null || mission != null)

    /**
     * The reachable angle nearest [solutionDeg], and whether reaching for it meant clamping.
     *
     * With **no reported range nothing is clamped and nothing is invented**: a request the gimbal
     * cannot honour comes back as DJI's own refusal, which says more than our guess at the envelope
     * would. [ManoeuvreGimbal.pitchRangeDeg] is a fact about the airframe, never a measurement of
     * where the camera is pointing — which is why consulting it does not reopen the closed-loop
     * door.
     */
    private fun reachablePitch(solutionDeg: Double, gimbal: ManoeuvreGimbal): Pair<Double, Boolean> {
        val range = gimbal.pitchRangeDeg()
        val commanded = if (range != null && range.start <= range.endInclusive) {
            solutionDeg.coerceIn(range.start, range.endInclusive)
        } else {
            solutionDeg
        }
        return commanded to (commanded != solutionDeg)
    }

    /**
     * The camera's rate limiter and deadband, shared by both things that aim it. Must hold [lock].
     *
     * True — and the limiter's state is advanced — when this angle may go out now: it has moved by
     * at least [OrbitGuidance.GIMBAL_DEADBAND_DEG] since the last one **we commanded**, and at least
     * [OrbitGuidance.GIMBAL_MIN_INTERVAL_MS] have passed. Both numbers are the orbit's, unchanged:
     * the command path is a `performAction` with a measured swallowed-callback risk, QGC's own gimbal
     * queue stalls ~1.2 s behind an unacknowledged command, and 10 Hz absolute angles at DJI is an
     * unmeasured load on the link that also carries our setpoints.
     *
     * Neither remembered value is ever a gimbal *reading*; they are what we asked for and when.
     */
    private fun gimbalDueLocked(now: Long, pitchDeg: Double): Boolean {
        val last = gimbalLastPitchDeg
        if (last != null && abs(pitchDeg - last) < OrbitGuidance.GIMBAL_DEADBAND_DEG) return false
        val lastAt = gimbalLastAtMs
        if (lastAt != null && now - lastAt < OrbitGuidance.GIMBAL_MIN_INTERVAL_MS) return false
        gimbalLastPitchDeg = pitchDeg
        gimbalLastAtMs = now
        return true
    }

    /**
     * One absolute angle to the camera, fire and forget. A camera that will not aim is **not** a
     * reason to stop flying: the aircraft keeps its authority and the operator keeps their aircraft.
     */
    private fun aimCamera(gimbal: ManoeuvreGimbal, pitchDeg: Double) {
        try {
            gimbal.aimPitch(pitchDeg)
        } catch (t: Throwable) {
            log("gimbal aim failed: $t")
        }
    }

    /**
     * The heading the nose-to-centre loop may act on, or null when it cannot be trusted.
     *
     * `yawDeg` arrives on `Signal.ATTITUDE` with its own 2 s limit, and a **stale heading commands
     * zero yaw rate** — never a guess. Deliberately *not* given the relative-altitude treatment
     * (where POSITION's freshness stands in as a liveness proxy for a change-driven key): a
     * circling aircraft is turning continuously, so its attitude feed has no reason to go quiet,
     * and a heading that has stopped arriving during an orbit is a fact worth acting on rather than
     * a quantiser artefact.
     */
    private fun usableHeading(state: AircraftState): Double? {
        val yaw = state.yawDeg ?: return null
        if (!state.isFresh(Signal.ATTITUDE)) return null
        if (!yaw.isFinite()) return null
        return yaw
    }

    /**
     * The aircraft's measured speed, m/s, or null when the velocity feed cannot vouch for one
     * — stale, never delivered, or component-gone. Arrival's second conjunct reads this; a
     * null withholds arrival rather than substituting a cached number, because on this
     * airframe a dead velocity feed and a hover are the same bytes (`SampleAges`).
     */
    private fun measuredSpeedOrNull(state: AircraftState): Double? {
        if (!state.ages.isFresh(Signal.VELOCITY)) return null
        val vn = state.velocityNorth ?: return null
        val ve = state.velocityEast ?: return null
        val vd = state.velocityDown ?: 0.0
        return sqrt(vn * vn + ve * ve + vd * vd)
    }

    /**
     * The lateral ground speed the autoland commit gate may act on, m/s, or null when the
     * velocity feed cannot vouch for one — the machine's `lateralSpeedM` input, null defaulting
     * the commit closed.
     *
     * **Fourth appearance of the change-driven-key lesson** (gimbal attitude, RC keys, relative
     * altitude — now this): `KeyAircraftVelocity` is change-only on this airframe (the
     * `signal_stale` KDoc records it), so an aircraft pinned at the FC floor with a constant
     * 0.0 reading legitimately quiets the feed — exactly at the moment this gate is consulted.
     * Freshness-by-delivery-age would therefore deadlock the commit the way it deadlocked the
     * hover climb: the commit needs "slow", slow keeps the feed silent, silence reads "stale".
     * The rule is [usableAltitude]'s verbatim: **an unchanged value from a live component is
     * current**, with POSITION's continuous GPS jitter as the liveness proxy; any real
     * acceleration changes the value and true delivery freshness returns within one report.
     *
     * Lateral only — no vertical term — because the budget this gate enforces is the
     * measured lateral momentum carry (landing06, `LAND_COMMIT_SPEED_MS`'s KDoc); the vertical
     * axis at a commit is either the FC floor's pinned zero or DJI's own descent, neither ours
     * to gate on.
     */
    private fun lateralSpeedOrNull(state: AircraftState): Double? {
        val vn = state.velocityNorth ?: return null
        val ve = state.velocityEast ?: return null
        val alive = state.ages.isFresh(Signal.VELOCITY) || state.ages.isFresh(Signal.POSITION)
        if (!alive) return null
        return hypot(vn, ve)
    }

    /**
     * The relative altitude the vertical axis may act on, or null when it cannot be trusted.
     *
     * The relative-altitude delivery is **change-driven and 0.1 m-quantised** (measured
     * 2026-07-26 22:28, record `20260726-222813.001`: a hovering aircraft's relalt age reached
     * 9.5 s while position and attitude ages sat near 50 ms). Freshness-by-delivery-age
     * therefore deadlocks a hover: the climb is blocked because altitude reads "stale", and
     * altitude stays "stale" because the aircraft is not climbing — Change Altitude was
     * unusable from a hover until this was diagnosed on the bench. Third appearance of the
     * change-driven-key lesson (gimbal attitude, then the RC keys, now this).
     *
     * The rule that replaces it is the gimbal's lesson applied: **an unchanged value from a
     * live component is current, not stale.** POSITION jitters continuously even in a hover
     * (GPS noise defeats the quantiser — measured ages ~50 ms throughout), so its freshness
     * is the liveness proxy: the altitude is usable when it exists and either its own
     * delivery or POSITION's is fresh. The ceiling stays protected, because any actual climb
     * changes relalt and true delivery freshness returns within one report.
     */
    private fun usableAltitude(state: AircraftState): Double? {
        val altitude = state.relativeAltitude ?: return null
        val alive = state.ages.isFresh(Signal.ALTITUDE) || state.ages.isFresh(Signal.POSITION)
        return if (alive) altitude else null
    }

    /**
     * The Q1 ceiling, gating exactly one thing: the climb component. Lateral motion and
     * descent are untouched in every altitude state, because blocking those would trade a
     * bounded envelope breach for a disengagement at the worst moment. Unknown or stale
     * altitude blocks climb the same way — the ceiling cannot be enforced blind. Shared by
     * both setpoint sources; must hold [lock].
     */
    private fun climbGatedLocked(velocities: StickVelocities, effects: MutableList<() -> Unit>): StickVelocities {
        if (velocities.down >= 0.0) return velocities
        val state = aircraftState()
        val altitude = usableAltitude(state)
        if (altitude == null) {
            effects += { announce(GuidedStatusTexts.NO_ALTITUDE) }
            return velocities.copy(down = 0.0)
        }
        if (altitude >= GuidedEnvelope.CEILING_M) {
            effects += { announce(GuidedStatusTexts.CEILING) }
            return velocities.copy(down = 0.0)
        }
        return velocities
    }

    private fun tickReleasingLocked(now: Long, effects: MutableList<() -> Unit>) {
        if (!interlockEnabled()) {
            effects += { abort(DisengageReason.INTERLOCK) }
            return
        }
        val snapshot = vs
        if (snapshot == null || !authorityOk(snapshot)) {
            effects += { abort(DisengageReason.AUTHORITY, snapshot?.authority ?: "none") }
            return
        }
        val seizedMode = modeSeizedLocked(now)
        if (seizedMode != null) {
            effects += { abort(DisengageReason.AUTHORITY, "MODE_" + seizedMode) }
            return
        }
        if (rcAbortDueLocked(now)) {
            effects += { abort(DisengageReason.RC_STICKS) }
            return
        }
        val reason = releaseReason
        val plan = releasePlan
        val started = releaseStartedAtMs
        if (reason == null || plan == null || started == null) {
            effects += { abort(DisengageReason.AUTHORITY, "RELEASE_STATE_LOST") }
            return
        }

        // The operator's link came back before the release step ran: their hand outranks our
        // wind-down. Idle and timeout wind-downs do not resume — those ended on purpose.
        val frameAt = lastFrameAtMs
        val resumable = reason == DisengageReason.RELEASED || reason == DisengageReason.LINK_LOST
        if (resumable && frameAt != null && now - frameAt <= GuidedEnvelope.INPUT_STALE_MS) {
            phaseLocked = GuidedPhase.ENGAGED
            engagedAtMs = now
            clearReleaseLocked()
            lastNonNeutralAtMs = now
            effects += {
                log("input resumed during $reason wind-down — passthrough restored")
                announce(GuidedStatusTexts.RESUMED)
            }
            return
        }

        val elapsed = now - started
        if (elapsed < plan.rampToZeroMs) {
            val factor = (1.0 - elapsed.toDouble() / plan.rampToZeroMs).coerceIn(0.0, 1.0)
            val v = rampFrom.scaled(factor)
            effects += { performSend(v, null) }
            return
        }
        val hold = plan.holdZeroMs
        if (hold == null || elapsed < plan.rampToZeroMs + hold) {
            // Zero commanded velocity is DJI's own position hold — the benign state, held for
            // as long as the plan says (forever, for FreezeAndHold).
            effects += { performSend(StickVelocities.ZERO, null) }
            return
        }
        if (plan.release) {
            phaseLocked = GuidedPhase.IDLE
            val detail = releaseDetail
            clearReleaseLocked()
            effects += {
                log("release complete — reason=${reason.wire}${detail?.let { " ($it)" } ?: ""}")
                record.event(EventCode.GUIDED_RELEASED, reason.wire + (detail?.let { " $it" } ?: ""))
                record.event(EventCode.VS_DISABLE_REQUEST, reason.wire)
                requestDisable()
                announce(GuidedStatusTexts.off(reason.wire, detail))
            }
        }
    }

    /** Must hold [lock]. Enters [GuidedPhase.RELEASING] and announces the wind-down. */
    private fun beginReleaseLocked(
        now: Long,
        reason: DisengageReason,
        detail: String?,
        plan: LinkLossPlan,
        effects: MutableList<() -> Unit>,
    ) {
        phaseLocked = GuidedPhase.RELEASING
        releaseReason = reason
        releaseDetail = detail
        releasePlan = plan
        releaseStartedAtMs = now
        rampFrom = lastCommanded
        effects += {
            log(
                "wind-down begun: reason=${reason.wire} plan=[ramp=${plan.rampToZeroMs}ms " +
                    "hold=${plan.holdZeroMs?.toString() ?: "forever"} release=${plan.release}]"
            )
            announce(GuidedStatusTexts.stopping(reason.wire))
        }
    }

    /** Must hold [lock]. */
    private fun clearReleaseLocked() {
        releaseReason = null
        releaseDetail = null
        releasePlan = null
        releaseStartedAtMs = null
        rampFrom = StickVelocities.ZERO
    }

    /** Must hold [lock]. Gesture 1's debounced verdict. */
    private fun rcAbortDueLocked(now: Long): Boolean {
        val since = rcDeflectedSinceMs ?: return false
        return now - since >= RC_ABORT_SUSTAIN_MS
    }

    // ---------------------------------------------------------------------- abort

    /**
     * Q3: disengage virtual stick and hand the aircraft back to the RC. **Idempotent and safe
     * from any thread** — the phase flips to IDLE under the lock before any side effect, so a
     * second caller (a DJI callback racing the tick racing the UI) finds nothing to do.
     *
     * The sequence is one zero setpoint (only if we actually held authority — a cancelled
     * engagement has nothing to zero), then `disableVirtualStick`, then the Q5 announcement
     * with the reason. The disable is explicit rather than trusting silence, because DJI
     * documents no timeout and *"stop sending"* has no measured meaning
     * (`docs/m3-guided-control.md` §3.5).
     */
    fun abort(reason: DisengageReason, detail: String? = null) {
        val hadAuthority: Boolean
        val roiStopped: Boolean
        val climbCancelled: Boolean
        val alreadyIdle: Boolean
        var descentEnded = false
        var committedLandingDropped = false
        var confirmedLandingDropped = false
        var shadowStopped = false
        var shadowSegmentEnded = false
        // The mission's half of the same event. Captured under the lock alongside the manoeuvres it
        // sits beside, and delivered after them, so the executor can never be told a run stopped
        // while this engine still holds the route.
        var pausedMission: Triple<MissionRunSink, MissionPauseCause, Int>? = null
        // The plan's ROI ending, captured under the lock and said after it — one owner
        // ([endPlanRoiLocked]) for all five paths that drop a run, because "a plan's ROI dies with
        // the plan" remembered at five sites is a rule forgotten at one of them.
        val roiEffects = mutableListOf<() -> Unit>()
        synchronized(lock) {
            // **Above the IDLE early-return, and that placement is the safety property.** A
            // pending takeoff climb exists precisely while this engine is IDLE — DJI is flying its
            // own takeoff and we hold nothing — so an abort that returned early would leave the
            // intention armed and it would fire seconds later on an aircraft the operator had just
            // taken back. Every rung of the ladder ends here, so putting the clear here means
            // every rung cancels it, rather than each rung having to remember to.
            climbCancelled = takeoffClimb.cancel()
            // **Shadow mode survives every abort except the bridge's own stop** — and the first
            // build of this line got it wrong in exactly the way that flew: on 2026-07-28
            // (flight 20260728-152922, t=61.7) QGC's joystick stream held a live passthrough
            // engagement while Ivan enabled shadow, he pulled a stick, the passthrough's
            // RC-abort rung fired abort(RC_STICKS), and an unconditional clear here killed the
            // *mode* 2.3 s after he enabled it — "shadow mode off (sticks)", the exact spec
            // violation ("stick input never cancels a shadow: the operator is flying, a
            // deflection is the flight"). The suite missed it because the never-cancels test
            // ran with the engine IDLE, where this rung never fires: the property lived in two
            // places and only one was pinned. Now it lives in one — the shadow tick records
            // stick edges, and this line touches shadow only for [DisengageReason.STOPPED],
            // because a stopping bridge takes its recorder with it. STOP-from-the-phone
            // (interlock off) leaves the mode enabled-but-gated, visibly "blocked", since a
            // recorder cannot move the aircraft and STOP's job is the aircraft.
            // Still **above the IDLE early-return**, for the pending climb's reason: shadow
            // exists precisely while this engine may be IDLE.
            shadowStopped = reason == DisengageReason.STOPPED && shadowDescent != null
            shadowSegmentEnded = shadowStopped && shadowDescent?.run != null
            if (shadowStopped) shadowDescent = null
            alreadyIdle = phaseLocked == GuidedPhase.IDLE
            if (alreadyIdle) {
                hadAuthority = false
                roiStopped = false
                return@synchronized
            }
            hadAuthority = phaseLocked == GuidedPhase.ENGAGED || phaseLocked == GuidedPhase.RELEASING
            phaseLocked = GuidedPhase.IDLE
            mission?.let { run ->
                pausedMission = Triple(
                    run.sink, MissionAbortPolicy.causeOf(reason, detail), run.cursorSeq(),
                )
                // **Above the ROI suspension below, and the order is the property.** Suspending a
                // plan's ROI would remember it, and the next confirmed engagement un-suspends
                // everything it remembers — so a dead plan's target would start driving the camera
                // again after a handback. It is cleared instead; a resume re-takes it from the cursor.
                endPlanRoiLocked(run, roiEffects)
            }
            mission = null
            missionSetpointAtMs = null
            // Any abort ends the manoeuvre with the engagement: a target — or a circle, or a
            // descent — that survived into the next engagement would fly the aircraft on a
            // command nobody just gave. Yaw needs no row of its own here: it is part of the
            // setpoint stream this sequence zeroes and then disables, so every existing abort
            // zeroes it for free — the ROI's yaw half included, since it lives in that same
            // stream.
            //
            // **The descent's clear on this line is rule 1's second half.** The RC-stick rung
            // arrives here like every other rung, the state dies under the lock before any side
            // effect, and there is no resume path anywhere: re-engaging the sticks later finds
            // no descent, and only a fresh armTagDescent() through every gate can start one.
            reposition = null
            orbit = null
            descentEnded = tagDescent != null
            // Stage C: any abort of a committed DJI landing stops OUR engagement exactly like
            // every other abort — and what happens to the landing itself depends on the
            // reason, decided below: rule 1 withdraws it (KeyStopAutoLanding), everything else
            // leaves it to DJI and says so, because the landing stands on DJI's side whatever
            // our engagement does. Both captured under the lock, acted on after it.
            committedLandingDropped =
                tagDescent?.let { it.machine.phase == TagDescentPhase.DJI_LANDING } == true
            confirmedLandingDropped =
                tagDescent?.let { it.machine.phase == TagDescentPhase.DJI_LANDING && it.djiConfirmed } == true
            tagDescent = null
            // The ROI's *camera* half stops differently, and deliberately so (§9.5): the gimbal is
            // no longer driven and **stays exactly where it is** — no recentring, no stowing. A
            // camera slewing during an abort is noise at the moment the operator least needs it, and
            // an angle commanded after handback is a command the now-flying RC pilot did not ask
            // for. The target is *remembered*, not dropped: it is still the place the operator asked
            // about, and the next confirmed engagement re-acquires it. `DO_SET_ROI_NONE` clears it
            // in any state, because turning something off is always allowed.
            roiStopped = roiTrackingLocked() != null
            roi?.suspended = true
            orbitRoi?.suspended = true
            clearReleaseLocked()
        }
        // Said even when there was nothing else to abort — an operator who grabbed the sticks
        // between DJI's hop and our climb has ended something, and is owed the sentence.
        if (climbCancelled) announceClimbCancelled(reason.wire)
        if (shadowStopped) {
            log("shadow descent mode ended by abort: ${reason.wire}")
            if (shadowSegmentEnded) {
                record.event(EventCode.TAG_DESCENT_ENDED, "shadow ${reason.wire}", warn = true)
            }
            record.event(EventCode.TAG_DESCENT_PHASE, "shadow mode off (${reason.wire})")
            announce(GuidedStatusTexts.shadowOff(reason.wire))
        }
        if (alreadyIdle) return
        log("ABORT: reason=${reason.wire}${detail?.let { " ($it)" } ?: ""}")
        pausedMission?.let { (sink, cause, seq) ->
            log("mission paused at item $seq: $cause")
            record.event(EventCode.GOTO_ENDED, "mission paused seq=$seq cause=$cause", warn = true)
            sink.onPaused(cause, seq)
        }
        // The plan's ROI, if the plan had one and it was still set — said after the pause it belongs to.
        roiEffects.forEach { it() }
        if (descentEnded) {
            record.event(
                EventCode.TAG_DESCENT_ENDED,
                reason.wire + if (confirmedLandingDropped) " dji-confirmed" else "",
                warn = true,
            )
            if (committedLandingDropped) {
                if (reason == DisengageReason.RC_STICKS) {
                    // Rule 1's one action: the pilot's hand outranks the landing we started.
                    stopDjiLanding(reason.wire)
                } else {
                    log(
                        "engagement ended (${reason.wire}) during DJI's landing — no stop " +
                            "sent; DJI may complete the landing on its own"
                    )
                }
            }
        }
        if (roiStopped) {
            record.event(EventCode.ROI_CLEARED, "tracking stopped: ${reason.wire}")
            announce(GuidedStatusTexts.ROI_TRACKING_STOPPED)
        }
        if (hadAuthority) {
            try {
                performSend(StickVelocities.ZERO, null)
            } catch (t: Throwable) {
                // The zero is best-effort — the disable below is the act that matters, and an
                // abort must never be derailed by the send path being the broken part.
                log("zero setpoint on abort failed: $t")
            }
        }
        record.event(
            EventCode.GUIDED_RELEASED,
            reason.wire + (detail?.let { " $it" } ?: ""),
            warn = reason != DisengageReason.STOPPED,
        )
        record.event(EventCode.VS_DISABLE_REQUEST, reason.wire)
        requestDisable()
        announce(GuidedStatusTexts.off(reason.wire, detail))
    }

    /** `disableVirtualStick`, callbacks logged and recorded but never waited on. */
    private fun requestDisable() {
        port.disable(
            onSuccess = {
                record.event(EventCode.VS_DISABLE_RESULT, "ok")
                log("disableVirtualStick accepted by DJI")
            },
            onFailure = { djiError ->
                record.event(EventCode.VS_DISABLE_RESULT, djiError, warn = true)
                log("disableVirtualStick refused by DJI: $djiError")
            },
        )
    }

    // ---------------------------------------------------------------------- output

    /**
     * One setpoint to DJI, recorded whatever happens. [source] names the inbound message that
     * asked for this send — `MANUAL_CONTROL` with its wire sequence for stick passthrough,
     * [REPOSITION_SOURCE] for guidance-law sends — and is null for ramps, holds and zeros,
     * because no inbound message asked for those: they are this engine's own withdrawal, and
     * the flight log must be able to tell the two apart.
     *
     * A send that throws costs the engagement: if this bridge cannot deliver setpoints it has
     * no business holding stick authority, so the failure aborts (gesture-3 family) rather
     * than being retried into a command loop.
     */
    private fun performSend(v: StickVelocities, source: CommandSource?) {
        val axes = StickMapping.toDji(v)
        val report = try {
            port.sendAdvancedParam(
                pitch = axes.pitch ?: 0.0,
                roll = axes.roll ?: 0.0,
                yaw = axes.yaw ?: 0.0,
                verticalThrottle = axes.verticalThrottle ?: 0.0,
            )
        } catch (t: Throwable) {
            SendReport(StickModes.UNKNOWN, t.toString())
        }
        val range = synchronized(lock) {
            lastCommanded = v
            // Conjunct 3 of the heartbeat claim, stamped **after** the send rather than before it:
            // the mode appears only once the aircraft has genuinely been commanded, which is the
            // ordering that distinguishes a report from the forbidden echo. A send DJI refused does
            // not count — it aborts below, and a claim resting on a failed send would be a claim
            // about a setpoint that never arrived.
            if (mission != null && report.error == null) missionSetpointAtMs = nowMs()
            if (!rangeRecorded && report.error == null) {
                rangeRecorded = true
                ENVELOPE_RANGE
            } else {
                null
            }
        }
        record.stickCmd(
            setpoint = Setpoint(
                frame = SetpointFrame.NED_VELOCITY,
                north = v.north,
                east = v.east,
                down = v.down,
                yawRateDegPerS = v.yawRateDegPerS,
            ),
            axes = axes,
            modes = report.modes,
            source = source,
            range = range,
            accepted = report.error == null,
            error = report.error,
        )
        if (report.error != null) {
            log("sendVirtualStickAdvancedParam failed: ${report.error}")
            val engaged = synchronized(lock) { phaseLocked != GuidedPhase.IDLE }
            if (engaged) abort(DisengageReason.AUTHORITY, "SEND_FAILED")
        }
    }

    /** Conjunct A — DJI's own three facts, all required, none latched. */
    private fun authorityOk(s: VirtualStickSnapshot): Boolean =
        s.enabled && s.advanced && s.authority == "MSDK"

    /**
     * A `STATUSTEXT` at ERROR, identical text suppressed for [ANNOUNCE_REPEAT_MS] — the
     * established idiom (`GimbalManager.announce` has the full argument): QGC surfaces only
     * severities ≥ ERROR to the operator, and a gate consulted at 25 Hz must not be able to
     * print at 25 Hz.
     */
    /**
     * The logcat twin of [announce], for the refusal gates a 25 Hz stream can hit on every
     * frame: identical text is suppressed for [LOG_REPEAT_MS], a *changed* text logs
     * immediately, and the first occurrence goes out verbatim. The flight record and the
     * `STATUSTEXT` side are untouched — this dedups only the developer channel that was
     * measured flooding (first-engagement session, 25 lines/s under a held stick).
     */
    /**
     * When the controller that owns the current engagement was last heard from, or null if it has
     * never been heard from at all. Must hold [lock].
     *
     * Every engagement watchdog — the released/link-lost discrimination on the stick path and
     * the Q4 check in the reposition, mission, orbit and tag-descent ticks — reads through here,
     * so there is a single sentence to disagree with: liveness is evaluated within the
     * commanding origin, and traffic from any other controller is not evidence that this one is
     * alive (`docs/zenoh-dimos-transport.md` §4.3). What "alive" *means* for each origin lives
     * in [controllerSeenAtLocked], the one owner, shared with the descent arm gate's LINK_DOWN
     * rung — the pre-engagement form of the same question.
     *
     * The null-fallback to [ControlOrigin.MAVLINK] is the fail-safe direction: an engagement
     * whose origin was somehow never recorded demands heartbeat evidence rather than being
     * granted [ControlOrigin.PHONE]'s alive-by-identity reading.
     */
    private fun commandingControllerSeenAtLocked(now: Long): Long? =
        controllerSeenAtLocked(engagementOrigin ?: ControlOrigin.MAVLINK, now)

    /**
     * **The one place an origin's liveness semantics live.** When [origin]'s controller was last
     * known alive, or null if it has never been heard from. Must hold [lock].
     *
     *  - [ControlOrigin.MAVLINK]: the last inbound payload's stamp ([gcsSeenAtMs], written by
     *    [onInbound] and by every command handler on arrival). Recency is the evidence; silence
     *    past [GuidedEnvelope.LINK_LOST_MS] is link loss. Byte-for-byte the Q4 watchdog.
     *  - [ControlOrigin.PHONE]: [now] — alive by identity, never from the map. The phone app is
     *    this very process: a dead app cannot execute this line, and its death stops the 10 Hz
     *    stick stream so DJI's own virtual-stick timeout takes the aircraft (the existing
     *    failsafe backstop). A heartbeat would be the process attesting its own liveness to
     *    itself. Measured need: landing08 (`datasets/landing08/20260729-112216.001.jsonl`) —
     *    under the MAVLINK-only spelling the phone takeoff's climb died `link-lost` 1.6 s after
     *    engaging (t=32.33→33.93) and six descent arms were refused `LINK_DOWN`, on a flight
     *    whose operator was holding the commander in their hand.
     *
     * Exhaustive `when`, deliberately: a new origin must state its liveness semantics here or
     * not compile — there is no default to inherit by accident.
     */
    private fun controllerSeenAtLocked(origin: ControlOrigin, now: Long): Long? = when (origin) {
        ControlOrigin.MAVLINK -> gcsSeenAtMs[origin]
        ControlOrigin.PHONE -> now
    }

    private fun logThrottled(message: String) {
        val now = nowMs()
        synchronized(lock) {
            val last = lastThrottledLogAtMs
            if (message == lastThrottledLog && last != null && now - last < LOG_REPEAT_MS) return
            lastThrottledLog = message
            lastThrottledLogAtMs = now
        }
        log(message)
    }

    private fun announce(text: String) {
        val now = nowMs()
        synchronized(lock) {
            val last = lastAnnouncedAtMs
            if (text == lastAnnouncement && last != null && now - last < ANNOUNCE_REPEAT_MS) return
            lastAnnouncement = text
            lastAnnouncedAtMs = now
        }
        announcer.say(Severity.ERROR, text)
    }
}

/**
 * Where Stage A is in its engagement lifecycle. Derived state is deliberately minimal — four
 * phases, no latches: ENGAGED is re-earned every tick from DJI's own reports.
 */
enum class GuidedPhase {
    /** No authority held, none being sought. The only phase that can begin an engagement. */
    IDLE,

    /** We asked DJI for virtual stick; DJI has confirmed nothing. Claims nothing, sends nothing. */
    ENGAGING,

    /** DJI reports enabled + advanced + MSDK authority; setpoints stream at 10 Hz. */
    ENGAGED,

    /** A terminal sequence (ramp / hold / release) is running. Still holding authority. */
    RELEASING,
}
