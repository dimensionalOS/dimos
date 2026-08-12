package com.dimensional.mini4pro.mission

import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MissionState

/**
 * The execution half's run state, one-to-one with MAVLink's `MISSION_STATE`
 * (`MISSION_CURRENT.mission_state`).
 *
 * Our own enum rather than the library's, so the transport layer stays free of MAVLink types
 * where it can and so a state this bridge does not have cannot be named. [wire] is the one
 * place the mapping lives.
 */
enum class MissionRunState(val wire: MissionState) {
    NO_MISSION(MissionState.MISSION_STATE_NO_MISSION),
    NOT_STARTED(MissionState.MISSION_STATE_NOT_STARTED),
    ACTIVE(MissionState.MISSION_STATE_ACTIVE),
    PAUSED(MissionState.MISSION_STATE_PAUSED),
    COMPLETE(MissionState.MISSION_STATE_COMPLETE),
}

/**
 * **The whole seam between the transport half of M4 and the execution half.** Everything in
 * `mission/` is implementable against these six methods and nothing else.
 *
 * Contract notes, because they matter more than the signatures:
 *
 *  1. [isRunning] is polled **on the receiving thread** and must not block. It gates a protocol
 *     whose retry window is 250 ms.
 *  2. [currentSeq] and [runState] are polled at 1 Hz from `Bridge.tick`, never pushed. The
 *     executor owns its own 10 Hz thread and a push would cross threads for no benefit.
 *  3. Arrival is the other direction and **is** pushed, through [MissionReachedSink], because it
 *     is an event whose timestamp is the entire value of `MISSION_ITEM_REACHED`.
 */
interface MissionExecution {

    /**
     * True while the plan is being flown. Two things in the transport half depend on it, and
     * since M4-12 they are no longer the same thing:
     *
     *  - an **upload** is accepted anyway. It commits to the store, this executor keeps flying
     *    the snapshot it began with, and the operator is told
     *    [MissionStatusTexts.PLAN_STORED_FOR_NEXT_START]. The immutable snapshot is what makes
     *    that safe: nothing the transport does can reach a plan already handed over.
     *  - a **clear** is refused, `MAV_MISSION_DENIED` plus
     *    [MissionStatusTexts.MISSION_RUNNING] — see that constant for why the two differ.
     *
     * A **read** is never blocked in any state; it is the one thing an operator watching an
     * aircraft fly a plan actually needs.
     */
    fun isRunning(): Boolean

    /**
     * `MISSION_CURRENT.seq`: the item that will be flown next, **wire-numbered** (§4.1 — QGC
     * deleted the planned-home marker and renumbered down by one before uploading, and adds the 1
     * back for display). Null means the executor has nothing to say, which for a committed plan
     * that has not started is reported as 0 — *"the next item we will fly is 0"*, which is true.
     */
    fun currentSeq(): Int?

    /** `MISSION_CURRENT.mission_state`. */
    fun runState(): MissionRunState

    /**
     * The heartbeat's `custom_mode` override, or null.
     *
     * **MUST be non-null only while all three of §7.3's conditions hold:**
     *
     *  1. a plan is committed in the store, **and**
     *  2. the execution half's run state is `ACTIVE` (or `PAUSED`), **and**
     *  3. the guided engine is `ENGAGED` — which by Stage A's construction means DJI itself
     *     reports enabled + advanced + authority `MSDK`, and comes from `VirtualStickState`,
     *     **not** from our request.
     *
     * Condition 3 is what makes this a *report* rather than an echo, and it is the same evidence
     * Stage A already latches on. **Returning a non-null value on the strength of a `SET_MODE`
     * having arrived is the failure this whole layer exists to prevent** — `PLAN.md`: *"never
     * echo a requested mode back into the heartbeat — that field is exactly what QGC polls to
     * decide we complied."* The instant any of the three drops, the heartbeat must revert to
     * `Px4Mode.customMode(state.flightMode)`, whatever the aircraft says.
     *
     * **The claim is coming and is no longer in doubt.** M4-1 answered *"why not?"*, and the
     * answer to "why not" is on the record: the rule against echoing exists because that field is
     * what QGC polls to decide we complied, and the escape is that a value gated on the three
     * conjuncts above is not an echo — it is set after a setpoint has gone out, never because a
     * `SET_MODE` arrived, and any conjunct failing reverts it on the same tick. Under those
     * conditions `AUTO.MISSION` is a true description of what the aircraft is doing, in PX4's own
     * vocabulary. Without it QGC's Start and Continue buttons do not work at all.
     *
     * **Nothing consumes it on this branch, and that is deliberate.** Wiring it into
     * `TelemetryEncoder.heartbeat` belongs with the executor that can satisfy conjuncts 2 and 3;
     * transport can satisfy only conjunct 1, and a heartbeat claim backed by one third of its
     * evidence is the failure this whole layer exists to prevent. So the contract is written down
     * here where the implementer will read it, [NoMissionExecution] returns null, and the encoder
     * is untouched.
     */
    fun modeClaim(): Long?

    /**
     * Set-current, from either door: `MISSION_SET_CURRENT` (#41) or
     * `MAV_CMD_DO_SET_MISSION_CURRENT` (224). [seq] is wire-numbered — QGC has already subtracted
     * 1 for PX4 (`Vehicle.cc:2109-2111`).
     *
     * Returns the ack the transport should send. **Our first answer to command 224 decides, for
     * the lifetime of QGC's vehicle instance, which protocol it speaks to us**
     * (`MavCommandQueue.cc:129-160`): `MAV_RESULT_UNSUPPORTED` caches UNSUPPORTED and falls back
     * forever to the deprecated message, which has no reply channel at all. So a refusal here
     * must be `MAV_RESULT_DENIED` or `TEMPORARILY_REJECTED`, never `UNSUPPORTED` — the same
     * argument that makes `DO_REPOSITION` better than `SET_MODE` (`docs/m3-stage-b.md` §2).
     */
    fun setCurrent(seq: Int): MavResult

    /**
     * `SET_MODE` to `AUTO.MISSION`. Returns true only when responsibility was taken **and** the
     * operator was told whatever there is to tell — `CommandDispatcher.onModeRequest`'s existing
     * contract, unchanged.
     */
    fun start(): Boolean
}

/** Transport → execution, one call per item, on the executor's thread. */
fun interface MissionReachedSink {
    fun onReached(seq: Int)
}

/**
 * The execution half, absent.
 *
 * M4's transport layer is built and its executor is not, and this object is what makes that
 * legible rather than a gap. Every question gets the honest answer for a bridge that holds a plan
 * and flies nothing:
 *
 *  - nothing is running, so no transaction is ever refused on those grounds;
 *  - there is no cursor, so `MISSION_CURRENT` reports the plan's start;
 *  - **[modeClaim] is null**, so the heartbeat keeps coming from the aircraft. M4-1 settled that
 *    the claim *will* be made; it is null here because two of its three conjuncts are facts only
 *    an executor can observe, and a claim backed by the one conjunct transport can check would
 *    be the echo the rule forbids;
 *  - [setCurrent] and [start] refuse, because there is nothing to jump within and nothing to
 *    start. [setCurrent] refuses with `DENIED` rather than `UNSUPPORTED` precisely so QGC does
 *    **not** cache the fallback and lose its reply channel for the rest of the session.
 *
 * A test that replays the whole mission surface against this object and asserts nothing flies is
 * the point of it existing as a named type rather than as a nullable field.
 */
object NoMissionExecution : MissionExecution {
    override fun isRunning(): Boolean = false
    override fun currentSeq(): Int? = null
    override fun runState(): MissionRunState = MissionRunState.NO_MISSION
    override fun modeClaim(): Long? = null
    override fun setCurrent(seq: Int): MavResult = MavResult.MAV_RESULT_DENIED
    override fun start(): Boolean = false
}
