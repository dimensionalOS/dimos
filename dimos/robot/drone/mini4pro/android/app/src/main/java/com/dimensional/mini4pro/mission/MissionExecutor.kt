package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.guided.BlockPersistence
import com.dimensional.mini4pro.guided.GuidedStatusTexts
import com.dimensional.mini4pro.guided.MissionAbortPolicy
import com.dimensional.mini4pro.guided.MissionConsequence
import com.dimensional.mini4pro.guided.MissionEvent
import com.dimensional.mini4pro.guided.MissionLifecycle
import com.dimensional.mini4pro.guided.MissionPauseCause
import com.dimensional.mini4pro.guided.MissionRunSink
import com.dimensional.mini4pro.guided.MissionTransitions
import com.dimensional.mini4pro.guided.ResumeBlock
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Px4Mode
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.command.Verdict
import io.dronefleet.mavlink.common.MavResult

/**
 * **The mission executor**: the thing that takes an uploaded plan and flies it.
 *
 * It owns the lifecycle (§4), the cursor's *meaning*, the launch gate (§7.2) and the wire (§5). It
 * owns **no thread and no setpoint** — the flying is `GuidedStickEngine`'s, inside the one 10 Hz
 * tick, below the one abort ladder, through the one path to `VirtualStickManager`. A mission is a
 * fourth source of targets in that tick, never a second tick.
 *
 * ```
 * MissionStore ────▶ MissionExecutor ────▶ GuidedStickEngine ────▶ VirtualStickManager
 *  the plan          lifecycle, cursor,     one 10 Hz tick          one aircraft
 *                    launch gate, wire      abort ladder
 *                          ▲                     │
 *                          └──── MissionRunSink ─┘
 * ```
 *
 * ## The three properties this class exists to keep
 *
 *  1. **A mission is not a new authority.** It is a sequence of the authority we already have, with
 *     a gate in front of it and a cursor that only ever advances on an observation.
 *  2. **Nothing resumes by itself.** `PAUSED → RUNNING` has exactly one cause: an explicit operator
 *     Start through the full launch check. No timer, no watchdog, no helpful retry. Ivan confirmed
 *     this as binding (M4-11: *"yeah never automatic"*), and it is enforced structurally — the only
 *     caller of [transition] with [MissionEvent.START_ACCEPTED] is [start].
 *  3. **The wire never overstates.** [runState] reports `ACTIVE` only while the aircraft is
 *     genuinely being commanded, and [modeClaim] is non-null only under all three of M4-1's observed
 *     conjuncts.
 *
 * ## Nothing here is persisted
 *
 * The interlock is off at every process start, and a mission cursor that survived a crash would be a
 * plan resuming into a session whose operator never armed it. `Bridge.stop()` drops everything.
 */
class MissionExecutor(
    private val store: MissionStore,
    /**
     * The engine, or null when no link is up. A function rather than a field because `Bridge`
     * creates and destroys the engine per link while this object lives at service scope, exactly as
     * the store does.
     */
    private val engine: () -> MissionEngine?,
    private val aircraftState: () -> AircraftState,
    private val interlockEnabled: () -> Boolean,
    /** Where `MISSION_ITEM_REACHED` goes — `MissionProgress.reachedSink`. */
    private val reached: MissionReachedSink,
    private val announcer: Announcer,
    private val log: (String) -> Unit = {},
) : MissionExecution, MissionRunSink {

    /**
     * The slice of `GuidedStickEngine` this class is allowed to touch.
     *
     * Three methods, and none of them can command anything: start a route, ask whether one is
     * flying, ask where its cursor is. Deliberately **not** the engine itself — an executor that
     * could reach `abort` or `reposition` would be a second place the abort ladder is decided, and
     * an interface this narrow is the enforcement rather than a convention.
     */
    interface MissionEngine {
        fun missionStart(
            route: com.dimensional.mini4pro.guided.MissionRoute,
            startIndex: Int,
            rejoining: Boolean,
            sink: MissionRunSink,
        ): Verdict

        /** M4-1's third conjunct: engaged, DJI-confirmed, and a setpoint has recently gone out. */
        fun missionFlying(): Boolean
    }

    private val lock = Any()

    private var lifecycle: MissionLifecycle = MissionLifecycle.NO_MISSION

    /** The wire `seq` the cursor points at, or null when there is nothing to point at. */
    private var cursorSeq: Int? = null

    /** The plan id the cursor was taken against. A change under a *paused* cursor invalidates it. */
    private var cursorPlanId: Int? = null

    /** The standing block from the row of §6.2 that caused the pause, or null. */
    private var resumeBlock: ResumeBlock? = null

    /**
     * M4-14: any `DO_SET_HOME` this session blocks mission start until the aircraft lands and takes
     * off again. Set by [onHomeMoved], cleared by [onLanded].
     */
    private var homeMovedThisSession = false

    // ------------------------------------------------------------------ MissionExecution

    override fun isRunning(): Boolean = synchronized(lock) { lifecycle == MissionLifecycle.RUNNING }

    override fun currentSeq(): Int? = synchronized(lock) {
        // §7.1's own sentence: a committed plan that has not started reports 0 — *"the next item we
        // will fly is 0"*, which is true. With nothing committed there is nothing to say.
        when (lifecycle) {
            MissionLifecycle.NO_MISSION -> null
            else -> cursorSeq ?: 0
        }
    }

    /**
     * `MISSION_CURRENT.mission_state`, and **the one mapping that may understate but must never
     * overstate** (§5.2).
     *
     * `ACTIVE` is claimed only while the engine reports it is genuinely flying the route. A
     * lifecycle of `RUNNING` with the engine not yet engaged — the second between an accepted Start
     * and DJI confirming authority, or a tick spent holding zero on a stale fix — reports `PAUSED`,
     * which is the honest understatement: *we are not commanding the aircraft right now*. It is
     * briefly wrong in the direction that cannot hurt anyone, and the alternative is a Plan view
     * animating a flight that has not begun.
     */
    override fun runState(): MissionRunState = synchronized(lock) {
        when (lifecycle) {
            MissionLifecycle.NO_MISSION -> MissionRunState.NO_MISSION
            MissionLifecycle.LOADED -> MissionRunState.NOT_STARTED
            MissionLifecycle.PAUSED -> MissionRunState.PAUSED
            MissionLifecycle.FINISHED -> MissionRunState.COMPLETE
            MissionLifecycle.RUNNING ->
                if (engine()?.missionFlying() == true) MissionRunState.ACTIVE
                else MissionRunState.PAUSED
        }
    }

    /**
     * The heartbeat's `custom_mode` override — `AUTO.MISSION` — or null.
     *
     * **All three of M4-1's conjuncts, every read, none latched:**
     *
     *  1. a plan is committed in the store;
     *  2. this executor's lifecycle is `RUNNING`;
     *  3. `GuidedStickEngine` reports it is genuinely flying the route, which by construction means
     *     **DJI itself** reports enabled + advanced + authority `MSDK` *and* a setpoint has gone out
     *     inside the watchdog.
     *
     * Conjunct 3 is what makes this a report rather than an echo. It is set **after a setpoint has
     * gone out**, never because a `SET_MODE` arrived, and the instant any conjunct drops the
     * heartbeat reverts to `Px4Mode.customMode(state.flightMode)` — whatever the aircraft says.
     * `PLAN.md`: *"never echo a requested mode back into the heartbeat — that field is exactly what
     * QGC polls to decide we complied."*
     *
     * Ivan answered M4-1 *"why not?"*, and the answer to "why not" is on the record: under those
     * three conditions `AUTO.MISSION` is a **true description of what the aircraft is doing** in
     * PX4's own vocabulary. Without it QGC's Start and Continue buttons do not work at all —
     * `PX4FirmwarePlugin::startMission` sends an unacknowledged `SET_MODE` and then polls our
     * heartbeat's flight-mode string three times over 1.3 s before giving up with a modal.
     */
    override fun modeClaim(): Long? = synchronized(lock) {
        if (store.plan() == null) return null
        if (lifecycle != MissionLifecycle.RUNNING) return null
        if (engine()?.missionFlying() != true) return null
        Px4Mode.AUTO_MISSION
    }

    /**
     * `MISSION_SET_CURRENT` (#41) and `MAV_CMD_DO_SET_MISSION_CURRENT` (224) — **still refused**
     * (M4-10, left unanswered and therefore unchanged).
     *
     * Setting the cursor from the ground is a genuinely useful feature ("skip the first two legs")
     * and a genuine hazard: a cursor set to an item 200 m away becomes a rejoin leg nobody drew,
     * and the rejoin gate would then be the only thing between the operator and a flight across the
     * whole site. It deserves its own decision rather than arriving as a side effect of this build.
     *
     * `DENIED`, never `UNSUPPORTED`: our first answer to command 224 decides, for the lifetime of
     * QGC's vehicle instance, which protocol it speaks to us (`MavCommandQueue.cc:129-160`), and
     * `UNSUPPORTED` caches a fallback to the deprecated message that has no reply channel at all.
     */
    override fun setCurrent(seq: Int): MavResult = MavResult.MAV_RESULT_DENIED

    /**
     * **Start** — QGC's Start Mission and Continue Mission, both of which arrive as a `SET_MODE` to
     * `AUTO.MISSION` (M4-2: the button lives in QGC).
     *
     * Returns true only when responsibility was taken **and** the operator was told whatever there
     * is to tell, which is `CommandDispatcher.onModeRequest`'s existing contract unchanged: a
     * refusal here is announced, so returning true for one is correct and returning false would let
     * `HandshakeResponder`'s generic "bridge is telemetry-only" warning contradict a sentence that
     * is more specific and more true.
     *
     * **This is the only caller that can produce `START_ACCEPTED`**, which is how M4-11 — *resume is
     * never automatic* — is enforced structurally rather than by discipline.
     */
    override fun start(): Boolean {
        val resuming: Boolean
        val inputs: LaunchInputs
        synchronized(lock) {
            if (lifecycle == MissionLifecycle.RUNNING) {
                // Already flying. QGC's guided buttons send the same SET_MODE three times about
                // 1.34 s apart, so this is the ordinary case rather than an error, and re-running a
                // launch check on a moving aircraft is exactly what must not happen.
                log("mission start while already running — ignored")
                return true
            }
            resuming = lifecycle == MissionLifecycle.PAUSED
            inputs = gatherLocked(resuming)
        }

        val verdict = MissionLaunch.evaluate(inputs)
        if (verdict is LaunchVerdict.Refused) {
            synchronized(lock) {
                // A block is **spent by telling the operator** (§6.1) when its persistence says so:
                // the refusal is the message, and a second deliberate Start may then proceed. A
                // block that stands on an observed condition, or for the session, is not spent.
                if (resumeBlock?.persistence == BlockPersistence.SPENT_ON_REFUSAL) {
                    log("resume block ${resumeBlock?.name} spent by refusing this Start")
                    resumeBlock = null
                }
                transitionLocked(MissionEvent.START_REFUSED)
            }
            log("mission start refused: ${verdict.reason}")
            announce(
                if (resuming) GuidedStatusTexts.resumeRefused(verdict.reason)
                else GuidedStatusTexts.missionRefused(verdict.reason)
            )
            return true
        }

        val cleared = verdict as LaunchVerdict.Cleared
        val runner = engine()
        if (runner == null) {
            synchronized(lock) { transitionLocked(MissionEvent.START_REFUSED) }
            announce(GuidedStatusTexts.missionRefused(MissionLaunch.REASON_NO_PLAN))
            return true
        }
        // The engine's own engagement gates run here — SDK reachable, RC stick feed alive, a fresh
        // fix to fly from. They are not duplicated above: a second copy is a second place for the
        // same property to rot.
        val taken = runner.missionStart(cleared.route, cleared.startIndex, cleared.rejoining, this)
        if (taken != Verdict.ACCEPTED) {
            synchronized(lock) { transitionLocked(MissionEvent.START_REFUSED) }
            log("mission start refused by the engine: $taken")
            announce(GuidedStatusTexts.missionRefused("not ready to fly"))
            return true
        }
        synchronized(lock) {
            cursorPlanId = cleared.route.planId
            resumeBlock = null
            transitionLocked(MissionEvent.START_ACCEPTED)
        }
        return true
    }

    // ------------------------------------------------------------------ MissionRunSink

    override fun onItemReached(seq: Int) {
        log("mission item $seq reached")
        reached.onReached(seq)
    }

    override fun onCursor(seq: Int) {
        synchronized(lock) { cursorSeq = seq }
    }

    override fun onFinished(seq: Int) {
        synchronized(lock) {
            cursorSeq = seq
            transitionLocked(MissionEvent.LAST_ITEM_COMPLETE)
        }
        log("mission finished at item $seq — holding, in the air")
    }

    override fun onPaused(cause: MissionPauseCause, cursorSeq: Int) {
        val consequence = MissionAbortPolicy.consequence(cause)
        synchronized(lock) {
            this.cursorSeq = cursorSeq
            when (consequence) {
                // Rows 10 and 11 never reach here — they degrade an axis rather than ending a run —
                // but the branch exists so that adding a row with no mission consequence cannot
                // silently pause one.
                MissionConsequence.None -> return

                MissionConsequence.PauseResumable -> {
                    resumeBlock = null
                    transitionLocked(MissionEvent.RUN_PAUSED)
                }

                is MissionConsequence.PauseBlocked -> {
                    resumeBlock = consequence.block
                    transitionLocked(MissionEvent.RUN_PAUSED)
                }

                MissionConsequence.Abandon -> {
                    this.cursorSeq = null
                    cursorPlanId = null
                    resumeBlock = null
                    transitionLocked(MissionEvent.BRIDGE_STOPPED)
                }
            }
        }
        log("mission paused: cause=$cause consequence=$consequence cursor=$cursorSeq")
    }

    // ------------------------------------------------------------------ transport events

    /**
     * The store committed a plan. Called by the transport **synchronously on the commit edge**, so a
     * paused cursor is dropped on the same edge rather than discovered later (§2.1 rule 5).
     *
     * M4-12: while `RUNNING` this is a no-op for the run itself — the engine holds an immutable
     * route and carries on with the plan it began with — and the operator is told by the transport
     * that the new plan takes effect at the next Start.
     */
    fun onPlanCommitted() {
        val dropped = synchronized(lock) {
            val wasPaused = lifecycle == MissionLifecycle.PAUSED
            if (lifecycle != MissionLifecycle.RUNNING) {
                cursorSeq = null
                cursorPlanId = null
                resumeBlock = null
            }
            transitionLocked(MissionEvent.PLAN_COMMITTED)
            wasPaused
        }
        if (dropped) log("a paused cursor was dropped: the plan it indexed has been replaced")
    }

    /** The store was cleared. */
    fun onPlanCleared() {
        synchronized(lock) {
            cursorSeq = null
            cursorPlanId = null
            resumeBlock = null
            transitionLocked(MissionEvent.PLAN_CLEARED)
        }
    }

    /** A `DO_SET_HOME` was accepted this session. M4-14: mission start is blocked until a re-takeoff. */
    fun onHomeMoved() {
        synchronized(lock) { homeMovedThisSession = true }
        log("home was moved this session — mission start blocked until the aircraft lands and takes off")
    }

    /** The aircraft landed: the home-moved block clears, because the next takeoff re-bases the datum. */
    fun onLanded() {
        synchronized(lock) { homeMovedThisSession = false }
    }

    /** `Bridge.stop()`. Nothing is persisted; the next process start is `NO_MISSION`. */
    fun onBridgeStopped() {
        synchronized(lock) {
            cursorSeq = null
            cursorPlanId = null
            resumeBlock = null
            transitionLocked(MissionEvent.BRIDGE_STOPPED)
        }
    }

    // ------------------------------------------------------------------ internals

    /** The lifecycle, for tests and for the situation view. */
    val state: MissionLifecycle get() = synchronized(lock) { lifecycle }

    /** The standing resume block, for tests and for the situation view. */
    val block: ResumeBlock? get() = synchronized(lock) { resumeBlock }

    private fun transitionLocked(event: MissionEvent) {
        val next = MissionTransitions.next(lifecycle, event)
        if (next == null) {
            // Illegal is not the same as no-change: it means a caller asked for something that
            // cannot happen from here, and swallowing it silently is how a state machine rots.
            //
            // But it is also not the same as *moot*. A run-report arriving with no run — the
            // operator taking the sticks after a mission has already finished — is the ordinary
            // shape of a flight, and calling it illegal taught the reader to skip the line that
            // is the only warning a real sequencing bug would give.
            if (MissionTransitions.isMoot(lifecycle, event)) {
                log("mission $event ignored in $lifecycle: no run for it to be about")
            } else {
                log("illegal mission transition ignored: $lifecycle + $event")
            }
            return
        }
        if (next != lifecycle) log("mission lifecycle: $lifecycle -> $next ($event)")
        lifecycle = next
    }

    private fun gatherLocked(resuming: Boolean): LaunchInputs {
        val s = aircraftState()
        val fix = Geo.coordinateOrNull(s.latitude, s.longitude)
        return LaunchInputs(
            interlockOn = interlockEnabled(),
            plan = store.plan(),
            resuming = resuming,
            cursorSeq = cursorSeq,
            planIdAtPause = cursorPlanId,
            resumeBlock = standingBlockLocked(s),
            fix = fix,
            positionFresh = s.isFresh(Signal.POSITION),
            linkAlive = s.fcConnected,
            relativeAltitudeM = s.relativeAltitude,
            amslDatumM = s.takeoffAltitudeAmsl,
            homeSet = s.homeLocationSet,
            homeLatDeg = s.homeLatitude,
            homeLonDeg = s.homeLongitude,
            batteryPercent = s.batteryPercent,
            isFlying = s.isFlying,
            homeMovedThisSession = homeMovedThisSession,
            takeoffAvailable = true,
        )
    }

    /**
     * The block that actually stands right now. Must hold [lock].
     *
     * A [BlockPersistence.WHILE_CONDITION] block is re-evaluated against an observation rather than
     * remembered: the interlock coming back on genuinely clears the interlock block, and DJI's
     * flight mode leaving the seized one genuinely clears that. **This is not an automatic resume**
     * — clearing a block does not start anything; it only stops one particular refusal, and the
     * Start that follows is still the operator's own act through the whole launch check.
     */
    private fun standingBlockLocked(s: AircraftState): ResumeBlock? {
        val block = resumeBlock ?: return null
        return when (block.persistence) {
            BlockPersistence.SESSION, BlockPersistence.SPENT_ON_REFUSAL -> block
            BlockPersistence.WHILE_CONDITION -> when (block) {
                ResumeBlock.INTERLOCK_OFF -> block.takeIf { !interlockEnabled() }
                ResumeBlock.MODE_SEIZED -> block.takeIf {
                    // A mode we may fly from is one DJI is not using to fly the aircraft itself.
                    // `APAS` is what a parked Mini 4 Pro reports and `JOYSTICK` is ours.
                    val mode = s.flightMode
                    mode != null && mode !in FLYABLE_MODES
                }

                else -> block
            }
        }
    }

    private fun announce(text: String) = announcer.say(Severity.ERROR, text)

    private companion object {
        /**
         * The flight modes a mission may be started from — i.e. the ones in which DJI is *not*
         * flying the aircraft on its own authority.
         *
         * Deliberately a tiny allow-list rather than a block-list of DJI's ~79 `FCFlightMode`
         * constants: a mode we have never seen is one we cannot vouch for, and the fail-closed
         * answer to "may we take this aircraft" is no.
         */
        val FLYABLE_MODES = setOf(
            // What a parked Mini 4 Pro reports on the ground with motors off (measured).
            "APAS",
            // Ours, while virtual stick has authority (measured, 0.3 s after ENGAGED).
            "JOYSTICK",
            "GPS_ATTI", "ATTI", "GPS_NORMAL", "P_GPS",
        )
    }
}
