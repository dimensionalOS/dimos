package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.ActionOutcome
import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.handshake.toMavResult
import io.dronefleet.mavlink.common.GimbalManagerSetAttitude

/**
 * The MAVLink half of the gimbal: QGroundControl's camera controls in, DJI-free aiming intent out,
 * and the three advertisement messages that make those controls exist at all.
 *
 * No DJI code, no Android, no clock it did not receive — so the whole of it, including every
 * refusal path, is exercised by `GimbalManagerTest` without an aircraft. The counterpart of
 * `command/CommandDispatcher`, and the shape is deliberately the same.
 *
 * ## What QGC sends, and how it arrives
 *
 * Derived from QGC `da14fad28` source on 2026-07-26. **Not measured on the wire** — the operator's
 * QGC holds UDP 14550 with a real aircraft and the aircraft is powered down, so no end-to-end run
 * was made. Every claim below is a line reference, and `docs/gimbal.md` lists what a measurement
 * would still settle.
 *
 * | QGC control | on the wire | acknowledged? |
 * |---|---|---|
 * | **Center** button | `COMMAND_LONG` 1000, `param1 = 0` (pitch°), `param2 = 0` (yaw°), `param3/4` NaN, `param5 = 44` (flags), `param7 = 1` (device id) | yes, and `showError = true` |
 * | **Tilt 90** button | the same, `param1 = -90` | yes, `showError = true` |
 * | on-screen drag | the same, at **10 Hz**, pitch/yaw accumulated from the *reported* angle | yes, `showError = false` |
 * | acquire / release control | `COMMAND_LONG` **1001**, `param1/2` = 255/190 or −3/−3 | yes, `showError = true` |
 * | joystick pitch/yaw buttons | raw `GIMBAL_MANAGER_SET_ATTITUDE` (**282**), q all-NaN, rates in rad/s, repeated at 2 Hz | **no** |
 *
 * Three consequences shape this class.
 *
 *  1. **The two controls Ivan asked for are QGC's own buttons.** "Down" is **Tilt 90**
 *     (`param1 = -90`) and "forward" is **Center** (`param1 = 0`); both are unconditionally
 *     visible in the gimbal drawer as soon as a gimbal registers
 *     (`src/Toolbar/GimbalIndicator.qml:175-191`), gated on no capability flag. So the common case
 *     is one press and needs no preset of our own — a preset here would be a second, differently
 *     labelled way to send the same command.
 *  2. **`MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` must be acknowledged, promptly.** It is not in QGC's
 *     retry list, so it is sent exactly once — but `MavCommandQueue::sendWorker` refuses to send a
 *     command that is still pending an ack to the same component
 *     (`MavCommandQueue.cc:274-286`), and entries clear only on ack or after a 1200 ms sweep. An
 *     unacknowledged first command therefore silently swallows every command for the next ~1.2 s,
 *     which at 10 Hz is a drag control that moves once and stops. `HandshakeResponder` sends
 *     exactly one ack per command for us; this class only has to answer quickly and truthfully.
 *  3. **`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` is sent before the operator's first command**, from
 *     `_tryGetGimbalControl` (`GimbalController.cc:346-363`), with `showError = true`. Failing to
 *     answer it `ACCEPTED` pops a dialog at the operator every time they touch the camera.
 *
 * ## The interlock: gimbal aiming is **not** gated, and that is a decision
 *
 * `CommandInterlock` is deliberately not consulted anywhere in this file. The argument, so that it
 * can be disagreed with rather than discovered:
 *
 *  - The interlock's own KDoc defines it as *"the switch between 'this bridge translates telemetry'
 *    and 'this bridge can move an aircraft'"*. A gimbal cannot move an aircraft. It has no
 *    authority over position, altitude, heading or motors, and a 3-axis camera mount on a 249 g
 *    airframe has no meaningful attitude coupling either.
 *  - Ivan chose (M3 Q1) to keep **one** switch precisely so that its meaning stays sharp. Putting
 *    a second, much weaker meaning behind it blunts the first one.
 *  - **The decisive argument is not convenience, it is interlock erosion.** Aiming the camera is
 *    the routine act of a flight; Return and Land are not. Gating the routine act behind the
 *    switch that arms the dangerous ones means the switch is on for most of every flight, and a
 *    safety interlock that is habitually on is a formality. The way to keep "commands armed" a
 *    deliberate act is to keep it rare.
 *  - What remains is real but small: an unexpected gimbal slew in flight is startling, and this
 *    link is unauthenticated, so anyone on the network can aim the camera. Against that: the pilot
 *    holding the RC always retains gimbal authority through the wheel and can override at any
 *    moment, and the same unauthenticated network already carries the video.
 *
 * Two guards remain regardless of the interlock, and they are not nothing: nothing here exists
 * unless a link is running (`Bridge.start` creates it, `Bridge.stop` clears it), and every command
 * is refused with `NO_PRODUCT` if no aircraft is connected, checked per call inside
 * [KeyManagerGimbalPort].
 *
 * **This is reversible in one place.** Gating gimbal aiming means giving this class a
 * `CommandInterlock` and returning `MAV_RESULT_TEMPORARILY_REJECTED` from [onPitchYaw] while it is
 * off, plus a `STATUSTEXT` saying so. `docs/gimbal.md` records it as Ivan's to overturn.
 *
 * ## Rate handling
 *
 * QGC's drag emits 10 Hz of absolute-angle setpoints. Rather than hand DJI ten `performAction`
 * calls a second, [onPitchYaw] sends immediately when [MIN_ROTATE_INTERVAL_MS] has elapsed and
 * otherwise holds the setpoint for [tick] to flush. Coalescing keeps the **latest** setpoint, so
 * the last angle of a drag is always delivered — a plain "drop anything inside the window" filter
 * would leave the camera stopped short of wherever the operator let go, which is this project's
 * characteristic failure shape (a control that stops halfway while the display says otherwise).
 */
class GimbalManager(
    /**
     * Where an operator-facing sentence goes — every attached interface, not one link. Replaced
     * the `send: (Any) -> Unit` that built a `STATUSTEXT` here; [ANNOUNCE_REPEAT_MS] and the
     * suppression it drives are untouched, and the 50-byte clamp moved with the message it
     * belongs to (`mavlink/StatusTextSink`).
     */
    private val announcer: Announcer,
    /** Optional trace hook; keeps `android.util.Log` out of a unit-testable layer. */
    private val log: (String) -> Unit = {},
    /** Wall clock, injected so both windows are testable without sleeping. */
    private val nowMs: () -> Long = { System.currentTimeMillis() },
    /**
     * Milliseconds since boot, for the `time_boot_ms` field of all three advertisement messages.
     * `SystemClock.elapsedRealtime()` in the app, which is what `Bridge` stamps its telemetry
     * with. QGroundControl reads the field in none of the three, so this is correctness for
     * anyone else rather than for our own consumer.
     */
    private val timeBootMs: () -> Long = nowMs,
) {

    companion object {
        /**
         * `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`. Spelled as a number for the same reason
         * `HandshakeResponder` spells its ids as numbers: a dialect that renames or lacks the enum
         * entry still compiles, and still routes the same button.
         */
        const val MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000

        /** `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`. */
        const val MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001

        /**
         * Minimum spacing between `KeyRotateByAngle` calls, milliseconds.
         *
         * QGC's on-screen drag fires every 100 ms (`OnScreenGimbalController.qml:62-73`). 200 ms
         * halves that and matches `Bridge`'s telemetry period exactly, so a coalesced setpoint
         * waits at most one tick and the two cadences cannot beat against each other.
         *
         * **Chosen, not measured.** MSDK 5.18.0 documents no rate limit for `KeyRotateByAngle`
         * (the warning about call frequency is on `KeyRotateBySpeed`), and nothing has been run
         * against hardware. If the gimbal turns out to keep up with 10 Hz, this can go to 100 ms
         * or away entirely; if it turns out to choke, this is the knob.
         */
        const val MIN_ROTATE_INTERVAL_MS = 200L

        /**
         * How close a new setpoint must be to the last one to be treated as the same request.
         *
         * 0.1° is below what the airframe can hold and far below what an operator can see, so
         * dropping a repeat inside it costs nothing and stops a stationary drag from re-commanding
         * the same angle forever.
         */
        const val PITCH_DEADBAND_DEG = 0.1

        /**
         * How long identical `STATUSTEXT` is suppressed. Deliberately the same number as
         * `CommandDispatcher.ACTION_REPEAT_MS` and `HandshakeResponder.MODE_REFUSAL_REPEAT_MS`:
         * three windows over the same class of retry burst that could drift apart would be a bug
         * waiting for a QGC release.
         *
         * It matters more here than it does there. A drag past the gimbal's limit, or any drag at
         * all (every QGC gimbal command carries a yaw), would otherwise produce ten identical red
         * lines a second.
         */
        const val ANNOUNCE_REPEAT_MS = 5_000L

        /** `GIMBAL_MANAGER_STATUS` cadence. 1 Hz is what QGC's own MockLink gimbal uses. */
        const val STATUS_PERIOD_MS = 1_000L

        /**
         * `GIMBAL_DEVICE_ATTITUDE_STATUS` cadence, matching `Bridge`'s fast telemetry set.
         *
         * It must keep coming: QGC's on-screen drag is closed-loop on the reported angle
         * (`GimbalController.cc:449-450`), so a frozen feed makes every drag tick command the same
         * absolute angle and the camera moves one step and stops.
         */
        const val ATTITUDE_PERIOD_MS = 200L

        /** Our own MAVLink identity, mirrored from `MavlinkLink` so this file imports no transport. */
        private const val SYSTEM_ID = 1
        private const val COMPONENT_ID = 1

        /** `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` param convention: leave this control unchanged. */
        private const val CONTROL_UNCHANGED = -1
        /** …set it to the component that sent the command. */
        private const val CONTROL_SENDER = -2
        /** …release it. */
        private const val CONTROL_RELEASE = -3
    }

    /**
     * The DJI layer — [MsdkGimbalAim] in the app, set by `Bridge.start` and cleared by
     * `Bridge.stop`, so it is non-null exactly while a link exists. A null is reported to the
     * operator as a failure rather than absorbed. Read on the `mavlink-rx` and `mavlink-tx`
     * threads.
     */
    @Volatile
    var aim: GimbalAim? = null

    /** Whoever last claimed primary control through `DO_GIMBAL_MANAGER_CONFIGURE`. 0/0 = nobody. */
    @Volatile
    private var primarySysid: Int = 0

    @Volatile
    private var primaryCompid: Int = 0

    /** A setpoint that arrived inside [MIN_ROTATE_INTERVAL_MS], waiting for [tick]. */
    @Volatile
    private var pendingPitchDeg: Double? = null

    /**
     * Every "when did this last happen?" here is a **nullable** Long rather than a `Long.MIN_VALUE`
     * sentinel, and that is a bug fix rather than a style. `now - Long.MIN_VALUE` overflows to a
     * negative number, so a sentinel makes every `elapsed < window` test read *true* on the very
     * first call — which silently deferred the first gimbal command ever sent and suppressed the
     * first advertisement. `null` cannot be arithmetic'd by accident.
     */
    @Volatile
    private var lastRotateAtMs: Long? = null

    @Volatile
    private var lastPitchDeg: Double? = null

    /**
     * **The pitch this bridge believes the camera holds** — [GimbalAim.believedPitch], surfaced
     * where `Bridge`'s consumers already look, and the **only** camera-pointing answer this
     * package gives out.
     *
     * Until 2026-07-28 this class exposed `commandedPitchDeg` instead (a view of [dispatch]'s
     * own ask-stamped record), whose KDoc mandated: *a consumer must fall back to the reported
     * angle and say so, never substitute a zero*. Every consumer then implemented — or forgot —
     * that mandate for itself, and the forgetting was measured: two 2026-07-28 sessions of
     * descent denials against a camera the RC wheel genuinely held at −90°. The mandate is now
     * the implementation ([PitchBelief.of], one place), the commanded half is the
     * success-stamped `CommandedGimbalPort` record rather than the ask (a refused rotate must
     * not be believed), and this property replaced that one so a second resolution cannot grow
     * back beside it.
     *
     * Null when no aim layer is attached or neither source has spoken — and null is never zero.
     */
    fun believedPitch(): PitchBelief? = aim?.believedPitch()

    @Volatile
    private var lastStatusAtMs: Long? = null

    @Volatile
    private var lastAttitudeAtMs: Long? = null

    @Volatile
    private var lastAnnouncement: String? = null

    @Volatile
    private var lastAnnouncedAtMs: Long? = null

    /** True when [since] is set and [now] is still inside [window] of it. */
    private fun within(now: Long, since: Long?, window: Long): Boolean =
        since != null && now - since < window

    /**
     * Registers this manager with the responder. Idempotent; every registration overwrites.
     *
     * Note what this needed from `handshake/`: **nothing new.** `registerMessageProvider` and
     * `registerCommandHandler` already existed for M1 and M2, and the one inbound *message* the
     * gimbal cares about (282) is routed from `Bridge`, which is the class whose documented job is
     * "routes inbound". That is deliberate — `handshake/` is being edited by someone else, and a
     * feature that can be added without touching it should be.
     */
    fun attachTo(responder: HandshakeResponder) {
        responder.registerMessageProvider(GimbalEncoder.MESSAGE_ID_GIMBAL_MANAGER_INFORMATION) {
            // Null when there is no gimbal to describe, which HandshakeResponder turns into
            // MAV_RESULT_UNSUPPORTED. QGC re-asks, so "not yet" is not "never".
            GimbalEncoder.managerInformation(reading(), timeBootMs())
        }
        // The handlers decide in [Verdict]; `.toMavResult()` puts the wire type on here, at the
        // one point in this class that knows there is a wire (`handshake/Verdicts.kt`).
        responder.registerCommandHandler(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) { onPitchYaw(it).toMavResult() }
        responder.registerCommandHandler(MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE) { onConfigure(it).toMavResult() }
    }

    // ------------------------------------------------------- outbound, on the tick

    /**
     * One telemetry tick: flush any coalesced setpoint, then return whatever advertisement
     * messages are due.
     *
     * Owns its own cadence rather than taking `Bridge`'s divisors, for the reason
     * `SimulatorControl.noticeIfDue` does: the cadence is then a property this file's tests can
     * assert, and `Bridge`'s change stays one line.
     *
     * Returns an empty list, and calls nothing, whenever there is no gimbal to advertise —
     * [GimbalReading.isAdvertisable]. That is the whole gate: a bridge with no gimbal attached
     * emits exactly the bytes it emitted before this package existed.
     */
    fun tick(): List<Any> {
        flushPending()
        val reading = reading()
        if (!reading.isAdvertisable()) return emptyList()
        val now = nowMs()
        val boot = timeBootMs()
        val due = mutableListOf<Any>()
        if (!within(now, lastAttitudeAtMs, ATTITUDE_PERIOD_MS)) {
            GimbalEncoder.deviceAttitudeStatus(reading, boot)?.let {
                lastAttitudeAtMs = now
                due.add(it)
            }
        }
        if (!within(now, lastStatusAtMs, STATUS_PERIOD_MS)) {
            GimbalEncoder.managerStatus(reading, boot, primarySysid, primaryCompid)?.let {
                lastStatusAtMs = now
                due.add(it)
            }
        }
        return due
    }

    /**
     * Sends a setpoint that arrived too soon after the last one. Runs on the telemetry thread.
     *
     * A failure discovered here can no longer reach the `COMMAND_ACK` — that was answered when the
     * command arrived — so it reaches the operator the only remaining way, as a `STATUSTEXT`. This
     * is the one place in the package where the ack and the truth can come apart, and it is
     * bounded to [MIN_ROTATE_INTERVAL_MS]; see [onPitchYaw] for why the first command after a
     * quiet period is never deferred.
     */
    private fun flushPending() {
        val pitch = pendingPitchDeg ?: return
        if (within(nowMs(), lastRotateAtMs, MIN_ROTATE_INTERVAL_MS)) return
        pendingPitchDeg = null
        val outcome = dispatch(pitch) ?: return
        if (outcome !is ActionOutcome.Requested) {
            log("deferred gimbal setpoint $pitch was not accepted: $outcome")
        }
    }

    // ------------------------------------------------------ COMMAND_LONG/INT 1000

    /**
     * `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` — every angle control QGroundControl has.
     *
     * `param1` pitch°, `param2` yaw°, `param3` pitch rate °/s, `param4` yaw rate °/s, `param5`
     * `GIMBAL_MANAGER_FLAGS`, `param7` gimbal device id. **Degrees**, not radians: the command's
     * own definition says `units="deg"` (`common.xml:2106-2111`) and QGC fills it straight from
     * its degree-valued facts (`GimbalController.cc:494-504`). The `GIMBAL_MANAGER_SET_PITCHYAW`
     * *message* (287) uses radians for the same quantities — QGC never sends it, and the unit
     * difference between the message and the command is exactly the sort of thing that would
     * otherwise be discovered by pointing a camera at the sky.
     *
     * NaN in an angle field means "leave this axis alone"; a NaN pitch is therefore a valid
     * command that asks for nothing, and is answered `ACCEPTED` because that is what happened.
     */
    private fun onPitchYaw(request: HandshakeResponder.CommandRequest): Verdict {
        if (!addressedToOurGimbal(request.param7)) {
            log("PITCHYAW for gimbal device ${request.param7} — not ours (${GimbalEncoder.GIMBAL_DEVICE_ID})")
            return Verdict.DENIED
        }

        // Said before anything is attempted, so an operator learns the airframe's limit even if
        // the pitch half then fails. Both of QGC's preset buttons hard-code yaw = 0, so this is
        // the common path rather than an edge case.
        if (request.param2.isFinite()) announce(GimbalStatusTexts.YAW_UNAVAILABLE)
        if (isRateRequest(request.param3) || isRateRequest(request.param4)) {
            announce(GimbalStatusTexts.RATE_UNSUPPORTED)
        }

        val pitch = request.param1
        if (!pitch.isFinite()) {
            // MAVLink's "do not change this axis". Nothing to do, and nothing went wrong.
            log("PITCHYAW with no pitch (param1=$pitch) — nothing to command")
            return Verdict.ACCEPTED
        }
        return aimPitch(pitch.toDouble())
    }

    /**
     * The single place a pitch is attempted, and the only place the repeat and rate windows are
     * consulted.
     *
     * The ordering matters. A setpoint inside [MIN_ROTATE_INTERVAL_MS] is **held, not dropped**,
     * and answered `ACCEPTED` — which is a claim we make good on within one tick and which the
     * command's own semantics ("valid and executed") stretch to cover. The alternative, holding
     * the ack until the deferred call returns, is exactly the `MAV_RESULT_IN_PROGRESS` behaviour
     * `HandshakeResponder.registerCommandHandler` forbids: QGC would keep the command pending
     * forever and refuse to send the next one.
     *
     * **The first command after a quiet period is never deferred** — `lastRotateAtMs` starts
     * `null` and any gap longer than the window clears it — so a genuinely unavailable
     * gimbal is discovered synchronously on the first press and refused with a truthful ack. Only
     * the middle of a fast drag can be answered optimistically, and by then the operator has
     * already had the truthful answer to the first one.
     */
    private fun aimPitch(pitchDeg: Double): Verdict {
        val now = nowMs()
        val settled = lastPitchDeg
        if (settled != null &&
            kotlin.math.abs(settled - pitchDeg) < PITCH_DEADBAND_DEG &&
            within(now, lastRotateAtMs, MIN_ROTATE_INTERVAL_MS)
        ) {
            // The same angle we just asked for. Re-commanding it would be a second command for
            // one intent, which is the rule CommandDispatcher.ACTION_REPEAT_MS enforces for
            // Return and Land.
            log("pitch $pitchDeg repeats the last request inside the window — already asked")
            return Verdict.ACCEPTED
        }
        if (within(now, lastRotateAtMs, MIN_ROTATE_INTERVAL_MS)) {
            pendingPitchDeg = pitchDeg
            return Verdict.ACCEPTED
        }
        return resultFor(dispatch(pitchDeg))
    }

    /**
     * Hands one pitch to the DJI layer and turns whatever comes back into an operator-facing
     * sentence. Returns null when there was nothing to hand it to.
     *
     * Stamps [lastRotateAtMs] on **every** attempt, including a refused one, so a gimbal that is
     * refusing cannot be asked ten times a second either.
     */
    private fun dispatch(pitchDeg: Double): ActionOutcome? {
        val sink = aim
        if (sink == null) {
            // Interlock-free, but not aircraft-free: with no link there is no DJI layer, and an
            // operator whose camera does not move is owed the reason.
            announce(GimbalStatusTexts.unavailable("NO_AIRCRAFT_LINK"))
            log("gimbal aim requested with no GimbalAim attached")
            return null
        }
        lastRotateAtMs = nowMs()
        lastPitchDeg = pitchDeg
        val outcome = try {
            sink.aimPitch(pitchDeg)
        } catch (e: Throwable) {
            // An unwritten or half-written DJI layer throws. That is a failure, never a success,
            // and it must not kill the thread that caught it.
            announce(GimbalStatusTexts.threw(e))
            log("gimbal aim threw: $e")
            return ActionOutcome.Unavailable(e::class.java.simpleName)
        }
        when (outcome) {
            is ActionOutcome.Requested ->
                // Deliberately silent. Unlike Return and Land, this bridge has a continuous and
                // honest confirmation channel for the gimbal — the attitude QGC is already
                // drawing, five times a second, straight from KeyGimbalAttitude. See
                // GimbalStatusTexts' KDoc.
                log("pitch $pitchDeg handed to DJI")

            is ActionOutcome.Refused -> {
                announce(GimbalStatusTexts.refusal(outcome.djiError))
                log("gimbal aim refused by DJI: ${outcome.djiError}")
            }

            is ActionOutcome.Unavailable -> {
                announce(GimbalStatusTexts.unavailable(outcome.reason))
                log("gimbal aim unavailable: ${outcome.reason}")
            }
        }
        return outcome
    }

    /**
     * An outcome as a `COMMAND_ACK` result.
     *
     * `UNSUPPORTED` is reserved for the one case that means "this bridge has no such capability at
     * all" — DJI's key declaring itself unperformable — because QGC renders each result as a
     * different sentence to the operator (`MavCommandQueue.cc:466-479`) and "not supported" is a
     * statement about the whole capability rather than about this attempt. Everything else that
     * failed on our side is `TEMPORARILY_REJECTED`: no product connected now may well be a product
     * connected in ten seconds.
     */
    private fun resultFor(outcome: ActionOutcome?): Verdict = when {
        outcome == null -> Verdict.TEMPORARILY_REJECTED
        outcome is ActionOutcome.Requested -> Verdict.ACCEPTED
        outcome is ActionOutcome.Refused -> Verdict.DENIED
        outcome is ActionOutcome.Unavailable &&
            outcome.reason == MsdkGimbalAim.CANNOT_PERFORM_ACTION -> Verdict.UNSUPPORTED
        else -> Verdict.TEMPORARILY_REJECTED
    }

    // ------------------------------------------------------- the bridge's own aiming

    /**
     * DJI's own reported pitch travel, or null when DJI has not said — for a bridge-owned
     * manoeuvre that has to know whether its pointing solution is reachable before it commands it.
     *
     * Never invented; this is `KeyGimbalAttitudeRange` and nothing else. It is a fact about the
     * *airframe*, not a measurement of where the camera is pointing, which is why exposing it does
     * not reopen the closed-loop door: see `guided/ManoeuvreGimbal`.
     */
    fun reportedPitchRange(): ClosedFloatingPointRange<Double>? {
        val limits = reading().limits ?: return null
        val min = limits.pitchMinDeg ?: return null
        val max = limits.pitchMaxDeg ?: return null
        if (min > max) return null
        return min..max
    }

    /**
     * An absolute pitch commanded by **this bridge** rather than by an operator's press — today
     * only the orbit holding the circle's centre in frame.
     *
     * Routed through the same [aimPitch] as QGC's own controls, deliberately: one path to DJI, one
     * rate window, one deadband, one place a refusal is announced. There is no `COMMAND_ACK` to
     * return to, so the verdict is dropped — the operator's channel for a failure here is the
     * `STATUSTEXT` [dispatch] already emits.
     *
     * This does **not** contradict `docs/decisions/2026-07-25-m2-command-safety.md` §Q4's ban on
     * unrequested commands to the aircraft. The operator requested an orbit, and
     * `docs/decisions/2026-07-27-orbit-yaw.md` makes the camera holding the centre part of what an
     * orbit *is* — "yaw alone only solves azimuth". Nothing here fires without a live manoeuvre the
     * operator commanded and this bridge acknowledged.
     */
    fun aimForManoeuvre(pitchDeg: Double) {
        aimPitch(pitchDeg)
    }

    // ------------------------------------------------------ COMMAND_LONG/INT 1001

    /**
     * `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` — who holds the camera.
     *
     * QGC sends this unprompted before the operator's first command
     * (`GimbalController.cc:346-363`, `:673-691`) with its own ids in `param1`/`param2`, and again
     * with `-3`/`-3` to release. `param3`/`param4` do the same for the secondary controller, which
     * this bridge does not model — there is one ground station.
     *
     * Answering `ACCEPTED` is not a courtesy: the command is sent with `showError = true`, so
     * anything else raises a modal dialog at the operator every time they touch the camera. And it
     * is honest — the whole of what is asked is that we record who has control, which we do, and
     * report it back in `GIMBAL_MANAGER_STATUS`. Nothing about the aircraft changes.
     *
     * The record is real rather than cosmetic. QGC blocks its own controls whenever
     * `GIMBAL_MANAGER_STATUS` names a primary controller that is not this QGC (`:179-180`,
     * `:346-352`), so a wrong answer here locks the operator out of their own camera behind a
     * "Request Gimbal Control?" popup.
     */
    private fun onConfigure(request: HandshakeResponder.CommandRequest): Verdict {
        if (!addressedToOurGimbal(request.param7)) {
            log("CONFIGURE for gimbal device ${request.param7} — not ours")
            return Verdict.DENIED
        }
        primarySysid = resolveControl(primarySysid, request.param1, request.senderSystemId)
        primaryCompid = resolveControl(primaryCompid, request.param2, request.senderComponentId)
        log("gimbal control now sysid=$primarySysid compid=$primaryCompid")
        return Verdict.ACCEPTED
    }

    /**
     * One `DO_GIMBAL_MANAGER_CONFIGURE` control field, resolved against MAVLink's sentinel
     * convention: `-1` leave unchanged, `-2` the sender itself, `-3` release, anything ≥ 0 is a
     * literal id. A value that is none of those is treated as "leave unchanged" rather than
     * guessed at.
     */
    private fun resolveControl(current: Int, param: Float, senderId: Int): Int {
        if (!param.isFinite()) return current
        return when (val v = param.toInt()) {
            CONTROL_UNCHANGED -> current
            CONTROL_SENDER -> senderId
            CONTROL_RELEASE -> 0
            else -> if (v >= 0) v else current
        }
    }

    // ------------------------------------------------------------ inbound messages

    /**
     * Messages `HandshakeResponder` does not itself answer, offered by `Bridge` — today only
     * `GIMBAL_MANAGER_SET_ATTITUDE` (282), QGC's joystick rate path.
     *
     * Returns true when the payload was ours, purely so `Bridge` can log it. There is nothing to
     * acknowledge: 282 is a message, not a command, so MAVLink gives us nothing to refuse with and
     * a `COMMAND_ACK` for it would be discarded ("Ack not in list", `MavCommandQueue.cc:489`).
     * `STATUSTEXT` is the only channel, which is the same position `SET_MODE` put
     * `HandshakeResponder` in.
     *
     * Rates are refused rather than translated to `KeyRotateBySpeed`; [GimbalPort]'s KDoc has the
     * argument. An all-NaN or all-zero rate is QGC's *stop*, and stopping a rate we never started
     * is a no-op worth no words.
     */
    fun onInbound(payload: Any?, senderSystemId: Int, senderComponentId: Int): Boolean {
        val set = payload as? GimbalManagerSetAttitude ?: return false
        if (!addressedToUs(set.targetSystem(), set.targetComponent())) return false
        if (isRateRequest(set.angularVelocityY()) || isRateRequest(set.angularVelocityZ())) {
            log(
                "GIMBAL_MANAGER_SET_ATTITUDE rate from $senderSystemId/$senderComponentId " +
                    "(y=${set.angularVelocityY()} z=${set.angularVelocityZ()} rad/s) — refused"
            )
            announce(GimbalStatusTexts.RATE_UNSUPPORTED)
        }
        return true
    }

    // ------------------------------------------------------------------- plumbing

    /**
     * DJI's own word for a rotation that failed asynchronously, straight to the operator.
     *
     * Public because [MsdkGimbalAim] subscribes to nothing that could carry it back down the
     * return path — the failure arrives on DJI's callback thread long after the `COMMAND_ACK` was
     * sent. Blank strings are dropped: an error with no name is not information.
     */
    fun reportAsyncDjiError(djiError: String) {
        if (djiError.isBlank()) return
        announce(GimbalStatusTexts.djiError(djiError))
    }

    /** `Bridge.stop()`. Forgets the coalesced setpoint and who held control. */
    fun reset() {
        pendingPitchDeg = null
        lastPitchDeg = null
        lastRotateAtMs = null
        lastStatusAtMs = null
        lastAttitudeAtMs = null
        primarySysid = 0
        primaryCompid = 0
    }

    private fun reading(): GimbalReading = aim?.reading() ?: GimbalReading()

    /**
     * `param7` of a gimbal manager command. 0 is MAVLink's "all gimbals"; anything else must name
     * ours. A command for a device we do not manage is denied rather than performed on the one we
     * do — the failure that would otherwise be invisible is a multi-gimbal ground station moving
     * the wrong camera.
     */
    private fun addressedToOurGimbal(param7: Float): Boolean {
        if (!param7.isFinite()) return true
        val id = param7.toInt()
        return id == 0 || id == GimbalEncoder.GIMBAL_DEVICE_ID
    }

    /** As `HandshakeResponder.addressedToUs`: our system, and our component or `MAV_COMP_ID_ALL`. */
    private fun addressedToUs(targetSystem: Int, targetComponent: Int): Boolean =
        (targetSystem == 0 || targetSystem == SYSTEM_ID) &&
            (targetComponent == 0 || targetComponent == COMPONENT_ID)

    /**
     * A rate field that is actually asking for a rate. NaN is MAVLink's "not commanding this", and
     * an exact zero is a stop — neither is a request this bridge has to refuse out loud.
     */
    private fun isRateRequest(rate: Float): Boolean = rate.isFinite() && rate != 0f

    /**
     * Says one sentence on every attached interface, at most once per [ANNOUNCE_REPEAT_MS] for
     * identical text.
     *
     * Severity is `ERROR` for the measured reason every other operator-facing sentence in this
     * project is: QGC surfaces only EMERGENCY/ALERT/CRITICAL/ERROR
     * (`StatusTextHandler.cc:18-24`), and anything below that threshold is an announcement that
     * never happened. It does abuse the taxonomy — "this airframe has no gimbal yaw" is not an
     * error — and the trade is taken knowingly, as it was for the mode refusal and the landing
     * announcement: a puzzling colour beats a message the operator never sees.
     */
    private fun announce(text: String, severity: Severity = Severity.ERROR) {
        val now = nowMs()
        if (text == lastAnnouncement && within(now, lastAnnouncedAtMs, ANNOUNCE_REPEAT_MS)) return
        lastAnnouncement = text
        lastAnnouncedAtMs = now
        announcer.say(severity, text)
    }
}
