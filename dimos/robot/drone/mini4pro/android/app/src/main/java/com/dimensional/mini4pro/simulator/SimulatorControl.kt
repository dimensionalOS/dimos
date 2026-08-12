package com.dimensional.mini4pro.simulator

import com.dimensional.mini4pro.telemetry.Geo

/**
 * What this bridge believes about the aircraft simulator, and every decision it makes about it.
 *
 * `MsdkFlightActions`' counterpart for the simulator: all the state and all the judgements, above
 * a DJI-free [SimulatorPort], so the parts that keep a simulated aircraft distinguishable from a
 * real one are unit-tested rather than verified by flying.
 *
 * ## The failure this class is designed against
 *
 * `CommandInterlock` names the failure it exists to prevent — *"a forgotten enable: a phone that
 * boots into 'commands live' because of a setting from last week"* — and defeats it structurally,
 * by holding one `AtomicBoolean` and owning no storage at all. **That technique does not work
 * here, and assuming it did would be the whole bug.**
 *
 * The interlock's state belongs to this process, so killing the process destroys it. The
 * simulator's state belongs to the *aircraft's flight controller* (see [SimulatorPort]'s class
 * doc: every operation is a `FlightControllerKey`). Killing the app does not stop it. Neither
 * does unplugging the phone, restarting the bridge, or opening DJI Fly. A simulator started by a
 * session yesterday is still running today, and a fresh process that assumed "off at start"
 * would present a fabricated aircraft as a real one — the exact forgotten-enable failure,
 * arriving through the one door `CommandInterlock`'s technique cannot close.
 *
 * So the rule here is **observe, never assume**, in three parts:
 *
 *  1. **Nothing is persisted, and nothing auto-starts.** As the interlock: no `SharedPreferences`,
 *     no `Context`, no constructor parameter that could produce an instance which begins started.
 *     A start is always an operator act during this run.
 *  2. **[phase] is never "off" without DJI having said so.** A fresh instance is
 *     [SimulatorPhase.UNKNOWN], not [SimulatorPhase.OFF], and a `null` delivery — DJI's
 *     component-gone signal — puts it *back* to `UNKNOWN`. The safe default of an unread
 *     simulator flag is "we cannot see", never "there isn't one".
 *  3. **A simulator we did not start is its own state**, [SimulatorPhase.FOREIGN], reported
 *     differently and loudly, because it means the app's record of intent does not explain the
 *     aircraft. Left over from a crashed session, from DJI Assistant, or from another app.
 *
 * ## What is claimed, and when
 *
 * The project's governing rule is that a claim comes from an observation, never from a request
 * (`PLAN.md`, "never echo a requested mode into the heartbeat"). Applied here: `start()`
 * succeeding means **DJI accepted the request**. It does not make [phase] active. Only
 * `KeyIsSimulatorStarted` delivering `true` does. That is why [SimulatorPhase.STARTING] exists as
 * a distinct state and why it emits no warning to the ground station — a warning sent on
 * [SimulatorPhase.STARTING] would be a claim sourced from our own request.
 *
 * The same asymmetry runs the other way: `stop()` succeeding gives [SimulatorPhase.STOPPING], and
 * only a delivered `false` gives [SimulatorPhase.OFF].
 *
 * ## Stopping
 *
 * [stopIfOurs] is called on bridge teardown and stops **only a simulator this process started**.
 * A foreign simulator is reported, never silently ended: stopping something we did not start is
 * an unrequested act on an aircraft, which is the class of thing
 * `docs/decisions/2026-07-25-m2-command-safety.md` §Q4 forbids, and on a bench it could end a
 * simulated flight somebody else is watching. The operator can always stop one deliberately with
 * [stop] — that is a request, not a watchdog.
 *
 * Thread safety: written from DJI callbacks and the Android main thread, read from the
 * `mavlink-tx` thread. One lock over everything, as `StateCache`.
 */
class SimulatorControl(
    private val port: SimulatorPort,
    /** Trace hook, so transitions land in logcat and the flight recorder. */
    private val log: (String) -> Unit = {},
    /**
     * Fired whenever [phase] may have moved, so a UI that is not otherwise ticking repaints.
     *
     * Needed because the simulator is the one piece of state on the status screen that changes
     * with **no** link running: an operator can start one with the bridge stopped, and without
     * this the banner would keep saying "off" over a simulating aircraft until something else
     * happened to redraw. Never called from inside the lock.
     */
    private val onChange: () -> Unit = {},
    /**
     * Monotonic clock, injected so the notice cadence and "active for N s" are testable without
     * sleeping. The production caller passes `SystemClock.elapsedRealtime`.
     */
    private val nowMs: () -> Long = { System.currentTimeMillis() },
) {

    private val lock = Any()

    private var observedStarted: Boolean? = null
    private var startRequested = false
    private var stopRequested = false

    /**
     * Whether DJI has ever confirmed the start [startRequested] refers to — i.e. whether that
     * request is still *in flight* or has been *fulfilled*.
     *
     * Exists because [startRequested] carries two meanings that a `null` delivery pulls apart. It
     * means **"the simulator is ours"**, which must survive a component-gone signal or
     * [stopIfOurs] would abandon a simulator still running on an aircraft that will come back; and
     * it means **"we asked and are waiting"**, which must *not* survive, because a request that
     * has already been answered is not outstanding. One flag cannot say both once the reading
     * disappears.
     *
     * Found on a real teardown, 2026-07-27
     * (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.2): the key stopped
     * delivering 34 ms before `fcConnected` went false, and an aircraft *going away* was announced
     * to the operator as a simulator *starting up* — "⬤ SIMULATOR STARTING — DJI HAS NOT
     * CONFIRMED" — when the honest answer was that we no longer knew anything. The stale request
     * was the only thing still speaking.
     */
    private var startConfirmed = false
    private var activeSinceMs: Long? = null
    private var aircraft: SimulatedAircraft? = null
    private var subscribed = false

    /** When the last wire notice went out, or null if none is outstanding. */
    private var lastNoticeMs: Long? = null
    private var lastNoticeText: String? = null

    /** Everything a caller needs to render, taken under one lock so the parts agree. */
    data class Snapshot(
        val phase: SimulatorPhase,
        val activeForMs: Long?,
        val aircraft: SimulatedAircraft?,
    )

    val phase: SimulatorPhase get() = synchronized(lock) { computePhase() }

    fun snapshot(): Snapshot = synchronized(lock) {
        val p = computePhase()
        Snapshot(
            phase = p,
            activeForMs = activeSinceMs?.let { nowMs() - it },
            aircraft = aircraft,
        )
    }

    /**
     * Phase is derived, never stored, so there is no field anyone could set to a state the
     * observations do not support. Ordering matters and is the safety property:
     *
     *  - a delivered `true` outranks everything, and whether it reads ACTIVE or FOREIGN turns
     *    only on whether *we* asked;
     *  - `null` (never delivered, or component gone) is STARTING only while a start is genuinely
     *    **in flight** — asked for and never yet confirmed, which is a true statement about us and
     *    is banner-worthy; once [startConfirmed] it is UNKNOWN, because a reading that has gone
     *    away is not a reading with a value, and our own stale request must not speak for DJI;
     *  - only a delivered `false` is OFF.
     *
     * The [startConfirmed] conjunct was added 2026-07-27 after a real teardown reported
     * `ACTIVE → STARTING (DJI reports started=null)` while the aircraft was disconnecting. See the
     * field's own KDoc for why one flag could not say both things.
     */
    private fun computePhase(): SimulatorPhase = when {
        observedStarted == true && stopRequested -> SimulatorPhase.STOPPING
        observedStarted == true && startRequested -> SimulatorPhase.ACTIVE
        observedStarted == true -> SimulatorPhase.FOREIGN
        observedStarted == null && startRequested && !startConfirmed -> SimulatorPhase.STARTING
        observedStarted == null -> SimulatorPhase.UNKNOWN
        startRequested -> SimulatorPhase.STARTING
        else -> SimulatorPhase.OFF
    }

    /**
     * Subscribe to DJI's simulator keys. Idempotent, and safe to call on every `Msdk.state`
     * emission.
     *
     * **Called on MSDK registration, not on bridge start**, and never cancelled by
     * `Bridge.stop()`. The subscription's whole job is to notice a simulator this process did not
     * start; tying it to the link would mean the one window in which we are blind is the window
     * before an operator has begun a session, which is exactly when a leftover simulator is
     * discovered.
     */
    fun observe() {
        synchronized(lock) {
            if (subscribed) return
            subscribed = true
        }
        port.listenIsSimulatorStarted(::onStartedDelivery)
        port.listenSimulatorState(::onStateDelivery)
    }

    /**
     * Ask DJI to start the simulator at [request]'s location.
     *
     * Refuses rather than clamping or guessing. The refusal reasons are all facts about us or
     * about the request, never attributed to the aircraft.
     */
    fun start(request: SimulatorRequest): SimulatorOutcome {
        val coordinate = Geo.coordinateOrNull(request.latitude, request.longitude)
            ?: return refuse("BAD_LOCATION")
        if (request.satelliteCount < MIN_SATELLITES || request.satelliteCount > MAX_SATELLITES) {
            return refuse("BAD_SATELLITE_COUNT")
        }
        synchronized(lock) {
            when (computePhase()) {
                SimulatorPhase.ACTIVE, SimulatorPhase.FOREIGN -> return refuse("ALREADY_RUNNING")
                SimulatorPhase.STARTING -> return refuse("START_IN_FLIGHT")
                SimulatorPhase.STOPPING -> return refuse("STOP_IN_FLIGHT")
                SimulatorPhase.UNKNOWN, SimulatorPhase.OFF -> Unit
            }
            port.unavailableReason()?.let { return refuse(it) }
            startRequested = true
            stopRequested = false
            // A new request is outstanding until DJI says otherwise, whatever the last one did.
            startConfirmed = false
        }
        onChange()
        log("simulator START requested at ${coordinate.first},${coordinate.second} " +
            "sats=${request.satelliteCount} — DJI has not confirmed anything yet")
        port.start(
            latitude = coordinate.first,
            longitude = coordinate.second,
            satelliteCount = request.satelliteCount,
            onSuccess = { log("simulator start ACCEPTED by DJI — not yet observed running") },
            onFailure = { error ->
                synchronized(lock) { startRequested = false; startConfirmed = false }
                onChange()
                log("simulator start REFUSED by DJI: $error")
            },
        )
        return SimulatorOutcome.Requested
    }

    /**
     * Ask DJI to stop the simulator, whoever started it. An operator act only — the automatic
     * path is [stopIfOurs].
     */
    fun stop(): SimulatorOutcome {
        synchronized(lock) {
            if (computePhase() == SimulatorPhase.OFF) return refuse("NOT_RUNNING")
            port.unavailableReason()?.let { return refuse(it) }
            stopRequested = true
        }
        return issueStop()
    }

    /**
     * Stop a simulator **this process started**, and only that. Called unconditionally from
     * `Bridge.stop()`, so it must be cheap and safe when there is nothing to do.
     *
     * Returns null when nothing was issued, so the caller can log the difference between "we
     * withdrew our simulator" and "there was nothing of ours to withdraw".
     */
    fun stopIfOurs(): SimulatorOutcome? {
        synchronized(lock) {
            if (!startRequested) return null
            val unavailable = port.unavailableReason()
            if (unavailable != null) {
                // Nothing to be done: the aircraft is not reachable, so the simulator cannot be
                // stopped from here. Saying so is the honest outcome; pretending we stopped it
                // would be the dangerous one — and the simulator is still running on an aircraft
                // that will come back.
                log("simulator still ours but the aircraft is unreachable — NOT stopped")
                return SimulatorOutcome.Refused(unavailable)
            }
            stopRequested = true
        }
        return issueStop()
    }

    private fun issueStop(): SimulatorOutcome {
        onChange()
        log("simulator STOP requested — DJI has not confirmed anything yet")
        port.stop(
            onSuccess = { log("simulator stop ACCEPTED by DJI — not yet observed off") },
            onFailure = { error ->
                synchronized(lock) { stopRequested = false }
                onChange()
                log("simulator stop REFUSED by DJI: $error")
            },
        )
        return SimulatorOutcome.Requested
    }

    /**
     * `KeyIsSimulatorStarted` delivered. The single input the whole safety story rests on.
     *
     * A `null` clears [observedStarted] back to "we cannot see" and clears nothing else: a
     * component-gone signal is not evidence that a simulator stopped, and forgetting that we
     * started one would make [stopIfOurs] give up on it.
     */
    private fun onStartedDelivery(started: Boolean?) {
        val before: SimulatorPhase
        val after: SimulatorPhase
        synchronized(lock) {
            before = computePhase()
            observedStarted = started
            if (started == false) {
                // Observed off is the only thing that retires an outstanding request, in either
                // direction. Both are cleared: a stop we asked for has completed, and a start we
                // asked for has demonstrably not taken.
                startRequested = false
                stopRequested = false
                startConfirmed = false
                activeSinceMs = null
                aircraft = null
            } else if (started == true) {
                // The request, if there was one, is now answered rather than outstanding. Set
                // unconditionally: a `true` arriving with no request of ours is a FOREIGN
                // simulator, and `startConfirmed` is only ever read alongside `startRequested`.
                startConfirmed = true
                if (activeSinceMs == null) activeSinceMs = nowMs()
            }
            after = computePhase()
            if (after != before) {
                // Make the next notice go out immediately rather than up to a period late.
                lastNoticeMs = null
            }
        }
        if (after != before) {
            onChange()
            log("simulator phase $before → $after (DJI reports started=$started)")
        }
    }

    private fun onStateDelivery(state: SimulatedAircraft?) {
        synchronized(lock) { aircraft = state }
    }

    /**
     * The `STATUSTEXT` body to send now, or null.
     *
     * Called from the telemetry thread. Returns a string only when DJI has **observed** the
     * simulator running — [SimulatorPhase.STARTING] deliberately produces nothing, because a
     * warning sent on our own request would be the echo this project forbids everywhere else.
     *
     * Repeats every [NOTICE_PERIOD_MS] rather than firing once on connect, so a ground station
     * that connects after the simulator started still learns. It also fires immediately whenever
     * the text changes, so ACTIVE → FOREIGN is never hidden behind the period.
     *
     * The period is deliberately long — read [NOTICE_PERIOD_MS] before shortening it. The
     * original argument for repeating included "QGC's message panel scrolls, so a line from ten
     * minutes ago is not a warning", which is true and which pointed the wrong way: repeating
     * often enough to stay on screen means *every other message* scrolls away instead. The panel
     * is not where a persistent condition belongs; the app's banner is.
     */
    fun noticeIfDue(now: Long = nowMs()): String? = synchronized(lock) {
        val text = when (computePhase()) {
            SimulatorPhase.ACTIVE, SimulatorPhase.STOPPING -> SimulatorNotice.WIRE_ACTIVE
            SimulatorPhase.FOREIGN -> SimulatorNotice.WIRE_FOREIGN
            else -> null
        }
        if (text == null) {
            lastNoticeMs = null
            lastNoticeText = null
            return null
        }
        val last = lastNoticeMs
        val due = last == null || text != lastNoticeText || now - last >= NOTICE_PERIOD_MS
        if (!due) return null
        lastNoticeMs = now
        lastNoticeText = text
        return text
    }

    private fun refuse(reason: String): SimulatorOutcome {
        log("simulator request refused: $reason")
        return SimulatorOutcome.Refused(reason)
    }

    companion object {
        /**
         * DJI's `InitializationSettings.createInstance` declares
         * `@IntRange(from=0,to=20) int satelliteCount` (`tools/djidoc
         * ISimulatorManager_InitializationSettings`). Out-of-range values are refused above the
         * seam rather than clamped below it: a clamp changes the operator's number without
         * telling them, and the satellite count is the one input that decides whether the
         * simulated aircraft believes it has a fix.
         */
        const val MIN_SATELLITES = 0
        const val MAX_SATELLITES = 20

        /**
         * How often the ground station is re-told the aircraft is simulated.
         *
         * **Was 5 s until an operator ran it: 12 messages a minute buried everything else in
         * QGC's panel.** That is the failure this project has reasoned itself out of twice
         * already (`TelemetryEncoder.prearmHealthy`, `HandshakeResponder`'s mode-refusal
         * de-duplication) — a channel that cries constantly stops being read, and the messages it
         * drowns are the ones that were worth sending.
         *
         * Five minutes keeps the reason the repeat exists at all — a ground station that connects
         * after the simulator started would otherwise never learn — while costing the panel
         * almost nothing. The latency that buys is acceptable precisely because a simulated
         * aircraft is harmless: the cost of learning late is confusion, not an incident, and the
         * loud channel is the app's own banner in front of the operator. A phase change still
         * fires immediately, so ACTIVE → FOREIGN is never held behind this.
         */
        const val NOTICE_PERIOD_MS = 300_000L
    }
}

/**
 * What this bridge believes about the simulator. Derived from observations only — see
 * [SimulatorControl.computePhase].
 */
enum class SimulatorPhase {
    /**
     * DJI has not told us. **Not the same as off**, and rendered differently everywhere: it is
     * the state of a fresh process before the first delivery, and of a link that has lost the
     * flight controller.
     */
    UNKNOWN,

    /** DJI reports the simulator is not started. The only state that licenses "off". */
    OFF,

    /** We asked DJI to start it and DJI has not yet reported it running. Claims nothing. */
    STARTING,

    /** DJI reports it running, and this process asked for it. */
    ACTIVE,

    /**
     * DJI reports it running and **this process did not start it**. A crashed session, DJI
     * Assistant, another app. The loudest state on the screen, because the aircraft is doing
     * something nothing here can explain.
     */
    FOREIGN,

    /** We asked DJI to stop it and DJI still reports it running. Still simulated. */
    STOPPING,
}

/** Where the simulated aircraft is put, and how many satellites it believes it has. */
data class SimulatorRequest(
    val latitude: Double,
    val longitude: Double,
    /** DJI's documented range is `[0, 20]`. */
    val satelliteCount: Int,
) {
    companion object {
        /**
         * Where a simulated aircraft is put when the real one has no position to borrow — which
         * on a bench, indoors, with no fix, is the normal case.
         *
         * The measured site from `docs/measurements/2026-07-26-velocity-units-and-frame.md`, and
         * chosen for a reason rather than for neatness: the seed coordinate is fed to DJI's
         * flysafe database, so a placeholder inside a restricted zone would produce a simulator
         * that refuses to fly and an afternoon spent debugging the wrong thing. A place the
         * aircraft has demonstrably flown from is known not to be one.
         *
         * The operator always sees the coordinate in the confirmation dialog, and
         * `MainActivity` prefers the aircraft's own reported position whenever there is a valid
         * one. Overridable per launch with `--es simLat / --es simLon / --ei simSats`.
         */
        const val DEFAULT_LATITUDE = 37.9938232
        const val DEFAULT_LONGITUDE = 23.7253477

        /** A confident fix, well inside DJI's `[0, 20]`. */
        const val DEFAULT_SATELLITES = 14
    }
}

/**
 * The result of asking. Deliberately smaller than `command/ActionOutcome`: there is no
 * "succeeded" here, because nothing on this side of the seam can observe a simulator start.
 */
sealed class SimulatorOutcome {
    /** Handed to DJI. Says nothing about the aircraft. */
    object Requested : SimulatorOutcome()

    /** We did not ask. [reason] is a fact about us or the request, never about the aircraft. */
    data class Refused(val reason: String) : SimulatorOutcome()
}
