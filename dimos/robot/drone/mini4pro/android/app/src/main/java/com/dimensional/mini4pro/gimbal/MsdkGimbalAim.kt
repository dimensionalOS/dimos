package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.ActionOutcome

/**
 * The DJI half of the gimbal: [GimbalAim] made real over a [GimbalPort], carrying every decision
 * that layer is allowed to make.
 *
 * No DJI imports, on purpose — the port is the seam and [KeyManagerGimbalPort] is the only class
 * below it. Everything here runs under `MsdkGimbalAimTest` with a fake port, including the two
 * decisions that matter most:
 *
 *  1. **A pitch command commands the pitch axis and nothing else.** Roll and yaw are passed as
 *     `null` — DJI's `rollIgnored`/`yawIgnored` — on every single call, unconditionally, with no
 *     branch that could ever populate them. That is the handling for
 *     [#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527), and it is asserted by
 *     name in the test suite rather than left as a property of the call site, because the failure
 *     mode is not a crash: it is `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` and a camera that does
 *     not move. See [GimbalAim]'s KDoc for why "don't send 0.0" is not the fix.
 *  2. **A requested angle is clamped to the range DJI itself reported**, and to nothing else.
 *     With no reported range nothing is clamped — see [clampedTarget].
 *  3. **A reading belongs to a link, and dies with it.** `KeyGimbalAttitude` only delivers on
 *     change, so a motionless gimbal cannot be told from a dead one by waiting; liveness comes
 *     from DJI's aircraft-link state instead, and the remembered attitude is forgotten the moment
 *     that goes — see [onLinkLost] and `GimbalReading.isAdvertisable`.
 *
 * It is also where the gimbal's *readings* are collected, which is a small departure from M2's
 * shape worth naming. In M2 `StateCache` reads and `MsdkFlightActions` acts, and they are separate
 * because `StateCache` feeds a 24-field telemetry model on a hot path. The gimbal's state is four
 * values consumed by one caller, and it is fed by the same `KeyManager` subscription group that
 * this class already has to own and tear down. Splitting it would buy a second lock and a second
 * lifecycle for no testability — everything here is already reachable through the fake port.
 *
 * **`StateCache` is deliberately not touched.** `docs/architecture.md` calls `StateCache` and
 * `KeyManagerActionPort` "the only two classes that touch `KeyManager`, one reading, one acting";
 * [KeyManagerGimbalPort] is a third, and it is justified the same way `KeyManagerActionPort` was —
 * it is the one DJI class under a testable seam, kept decision-free. The alternative, threading
 * gimbal fields through `AircraftState`, would put camera aiming inside the aircraft-telemetry
 * hot path and inside a file every other agent is also editing.
 *
 * ## Threads
 *
 * [aimPitch] runs on the MAVLink receive thread and on the telemetry thread's tick (whichever
 * flushes a coalesced setpoint first); port callbacks arrive on the MSDK's callback thread;
 * [reading] runs on the telemetry thread at 5 Hz. Everything shared is under [gate], which is
 * cheap at these rates and removes every ordering question the three threads could pose.
 *
 * Subscriptions are made lazily, from whichever of [aimPitch] or [reading] runs first *after*
 * [GimbalPort.unavailableReason] comes back null — i.e. after MSDK registration, because
 * subscribing earlier silently does nothing and reports no error (`docs/architecture.md`). Since
 * [reading] is called every telemetry tick, in practice the subscription lands within 200 ms of
 * registration completing, with no callback plumbing and nothing to go stale.
 */
class MsdkGimbalAim(
    private val port: GimbalPort,
    /** [GimbalManager.reportAsyncDjiError] in the app: DJI's error name, verbatim, to QGC. */
    private val reportAsyncDjiError: (String) -> Unit,
    private val log: (String) -> Unit = {},
    /**
     * Monotonic clock, injected so [GimbalReading.attitudeAgeMs] is testable without sleeping.
     *
     * **Monotonic, not wall clock**, for `StateCache`'s reason: an NTP step must not turn a fresh
     * gimbal reading into an hour-old one, nor the reverse. `System.nanoTime()` rather than
     * `SystemClock.elapsedRealtime()` keeps this file free of Android imports and therefore
     * runnable in a plain JVM test, which is the entire point of the seam.
     */
    private val nowMs: () -> Long = { System.nanoTime() / 1_000_000L },
) : GimbalAim {

    companion object {
        /**
         * What the operator reads when DJI's own key declaration says the rotation is not
         * performable — `Aim failed: CANNOT_PERFORM_ACTION`.
         *
         * Deliberately the same string `command/MsdkFlightActions` uses for the same flag, and
         * deliberately DJI's own spelling of it: `canPerformAction` is the name in DJI's
         * documentation, so an operator searching for it finds something. It is an
         * `ActionOutcome.Unavailable` and not a `Refused` for the reason that type's KDoc gives —
         * a false capability flag is a statement about a key object in our own process, and no
         * gimbal was consulted.
         */
        const val CANNOT_PERFORM_ACTION = "CANNOT_PERFORM_ACTION"

        /** A pitch that is NaN or infinite reached this layer. A caller bug; fail closed. */
        const val PITCH_NOT_FINITE = "PITCH_NOT_FINITE"

        /**
         * `GimbalAngleRotation.duration`, seconds: how long DJI is asked to take over the move.
         *
         * **Unproven.** DJI documents the field as *"Time of gimbal rotation operation. Unit:
         * second"* and says nothing about what 0 means or what the useful range is, and the
         * aircraft is powered down. RosettaDrone's MSDK v4 equivalent used `time(2)` — two whole
         * seconds — which would be visibly laggy against QGroundControl's 10 Hz drag, where each
         * new setpoint supersedes the last. 0.5 s is chosen to be smooth for the two preset
         * buttons (Center, Tilt 90) while still settling inside three of QGC's drag ticks.
         *
         * If the gimbal is jerky or lags on hardware, this is the knob, and the honest fix is a
         * measurement with a stopwatch and a few values, recorded in `docs/measurements/`.
         */
        const val ROTATION_DURATION_S = 0.5
    }

    private val gate = Any()

    private var listening = false
    private var angles: GimbalAngles? = null
    private var attitudeDeliveredAtMs: Long? = null
    private var workMode: String? = null
    private var connected: Boolean? = null
    private var limits: GimbalLimits? = null

    override fun aimPitch(pitchDeg: Double): ActionOutcome {
        if (!pitchDeg.isFinite()) {
            // Not reachable from GimbalManager, which resolves MAVLink's NaN convention before it
            // gets here. Kept because this is a public interface and a NaN reaching DJI would be
            // serialised as a real double and pointed at a camera.
            log("aim refused: pitch $pitchDeg is not a finite number")
            return ActionOutcome.Unavailable(PITCH_NOT_FINITE)
        }
        port.unavailableReason()?.let { return ActionOutcome.Unavailable(it) }
        // Before ensureListening, so an unperformable gimbal leaves no subscription behind.
        if (!port.canRotateByAngle()) {
            log("gimbal rotation not performable: DJI's key declares canPerformAction false")
            return ActionOutcome.Unavailable(CANNOT_PERFORM_ACTION)
        }
        ensureListening()

        val target = clampedTarget(pitchDeg)
        port.rotateByAngle(
            absolute = true,
            pitchDeg = target,
            // The two nulls are the whole of the #527 handling and they are not conditional on
            // anything. There is deliberately no code path in this class that can put a number in
            // either of them: on this airframe a yaw or roll rotation is answered
            // SDK_SERVICE_GIMBAL_ROTATE_{YAW,ROLL}_NOT_ALLOW, and DJI's serialiser turns a null
            // Double into 0.0 on the wire, so only the *Ignored flags the port sets for a null
            // axis can suppress them. `MsdkGimbalAimTest` asserts both stay null even when the
            // inbound MAVLink command carried a yaw.
            rollDeg = null,
            yawDeg = null,
            durationS = ROTATION_DURATION_S,
            onSuccess = { log("gimbal rotation accepted by DJI: pitch $target") },
            onFailure = { djiError ->
                log("gimbal rotation failed: $djiError")
                reportAsyncDjiError(djiError)
            },
        )
        // The truth about the call we made: asked, not refused on the spot, nothing more known.
        // Whether the camera moved is answered only by KeyGimbalAttitude, arriving later.
        return ActionOutcome.Requested
    }

    /**
     * The pitch we will actually ask for: [pitchDeg] brought inside DJI's own reported range, or
     * [pitchDeg] untouched when DJI has not reported one.
     *
     * **Clamping is silent** — logged, never a `STATUSTEXT` — and that is a considered choice
     * rather than an omission. Three reasons:
     *
     *  - The operator already has continuous, honest feedback: `GIMBAL_DEVICE_ATTITUDE_STATUS`
     *    shows where the camera *is*, so a gimbal that stops at its limit is visible as a gimbal
     *    that stopped at its limit. That is the same thing a physical gimbal does.
     *  - QGroundControl's on-screen drag accumulates from the last reported angle at 10 Hz
     *    (`GimbalController.cc:461-467`), so dragging past the limit would otherwise produce a red
     *    line every de-duplication window for as long as the operator holds the mouse down.
     *  - Nothing was refused. The command was performed, at the nearest angle the gimbal has.
     *
     * The limits themselves are never invented. With no `KeyGimbalAttitudeRange` delivered, the
     * request goes to DJI unmodified and a genuine out-of-range answer comes back as DJI's own
     * error name — which is more use to an operator than our guess at the envelope.
     */
    private fun clampedTarget(pitchDeg: Double): Double {
        val clamped = synchronized(gate) { limits }?.clampPitch(pitchDeg) ?: return pitchDeg
        log("pitch $pitchDeg clamped to $clamped by DJI's reported gimbal range")
        return clamped
    }

    override fun reading(): GimbalReading {
        // Read once, used twice: it decides whether we may subscribe, and it *is* the liveness
        // fact the reading carries. Fresh on every call by the port's contract — a product
        // disconnect is visible on the very next tick, with no listener plumbing to go stale.
        val linked = port.unavailableReason() == null
        if (linked) {
            // Cheap enough to attempt on every telemetry tick: after the first success it is one
            // volatile-ish read under a lock we are taking anyway, and doing it here is what lets
            // the subscription land as soon as MSDK registration completes with no callback
            // plumbing.
            ensureListening()
        } else {
            onLinkLost()
        }
        val now = nowMs()
        return synchronized(gate) {
            GimbalReading(
                pitchDeg = angles?.pitchDeg,
                rollDeg = angles?.rollDeg,
                yawDeg = angles?.yawDeg,
                workMode = workMode,
                connected = connected,
                limits = limits,
                attitudeAgeMs = attitudeDeliveredAtMs?.let { now - it },
                aircraftLinked = linked,
            )
        }
    }

    /**
     * [GimbalAim.believedPitch] — implemented here because this class is the one place both
     * sources already live: the commanded record sits in the port stack this class owns
     * ([GimbalPort.commandedPitchDeg]), and the reported attitude is this class's own listener
     * state. The precedence itself is [PitchBelief.of] and only there.
     *
     * Note the two sources age differently across a link loss, honestly: [onLinkLost] forgets
     * [angles] (a measurement belongs to the link it was made on), while the commanded record
     * survives until `Bridge.stop()` — what was asked was asked, and the ask is this bridge's
     * own act rather than a reading from an aircraft that has gone.
     */
    override fun believedPitch(): PitchBelief? =
        PitchBelief.of(
            commandedDeg = port.commandedPitchDeg(),
            reportedDeg = synchronized(gate) { angles?.pitchDeg },
        )

    /** `Bridge.stop()`. Cancels the subscriptions and forgets everything DJI told us. */
    fun stop() = forget()

    /**
     * Drop every subscription and every remembered fact, so that what is reported afterwards is
     * "we do not know" rather than "here is what it was". Used by [stop] and by [onLinkLost]; it
     * is idempotent, and [GimbalPort.cancelListens] is documented safe with nothing subscribed.
     */
    private fun forget() {
        port.cancelListens()
        synchronized(gate) {
            listening = false
            angles = null
            attitudeDeliveredAtMs = null
            workMode = null
            connected = null
            limits = null
        }
    }

    private fun ensureListening() {
        synchronized(gate) {
            if (listening) return
            // Nothing may touch KeyManager before registration completes: subscribing earlier
            // silently does nothing, with no error and no callback, so a subscription made too
            // early is indistinguishable from a gimbal that never reports.
            if (port.unavailableReason() != null) return
            listening = true
        }
        port.listenAttitude(::onAttitude)
        port.listenAttitudeRange(::onAttitudeRange)
        port.listenWorkMode(::onWorkMode)
        port.listenConnection(::onConnection)
    }

    /**
     * DJI says there is no aircraft. Forget the gimbal, and let it be re-learned if one returns.
     *
     * Necessary because [GimbalReading.isAdvertisable] no longer withdraws on age: without this,
     * an angle measured on one aircraft would still be publishable after that aircraft was
     * unplugged, and would come straight back the moment a different one connected. A reading
     * belongs to the link it was measured on.
     *
     * Dropping the subscriptions with it is deliberate rather than tidy: the next [reading] once
     * the link returns re-subscribes with `getOnce = true`, which is the only mechanism that can
     * give us an attitude before the gimbal next moves. On this airframe that priming get is
     * refused (`REQUEST_HANDLER_NOT_FOUND`) and the SDK falls back to its own cache read —
     * whatever DJI hands a fresh subscriber is what we get, which is exactly the deal every other
     * MSDK app is on.
     */
    private fun onLinkLost() {
        synchronized(gate) { if (!listening) return }
        log("aircraft link lost: forgetting the gimbal reading and the subscriptions")
        forget()
    }

    /**
     * Stamped on **every** delivery, including one carrying the same angles as the last and
     * including one carrying `null` — the rule `StateCache` documents.
     *
     * Two things this records, and one it no longer decides. It records **when** DJI last spoke
     * about the gimbal ([GimbalReading.attitudeAgeMs], reported for diagnostics and the log), and
     * it records a `null` as DJI's documented component-gone signal, which does still withdraw the
     * advertisement. It no longer decides *liveness*: on a change-driven key an old stamp means
     * the camera has not moved. See [GimbalReading.isAdvertisable].
     */
    private fun onAttitude(delivered: GimbalAngles?) {
        val at = nowMs()
        synchronized(gate) {
            angles = delivered
            attitudeDeliveredAtMs = at
        }
    }

    private fun onAttitudeRange(delivered: GimbalLimits?) {
        synchronized(gate) { limits = delivered }
    }

    private fun onWorkMode(delivered: String?) {
        synchronized(gate) { workMode = delivered }
    }

    private fun onConnection(delivered: Boolean?) {
        synchronized(gate) { connected = delivered }
    }
}
