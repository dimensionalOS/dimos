package com.dimensional.mini4pro.guided

/**
 * What the engine tells the thing that owns the plan. **The whole seam between the flying half and
 * the lifecycle half**, in four methods.
 *
 * The split it draws: the engine owns the *route* and the 10 Hz tick that flies it; the executor
 * owns the *lifecycle*, the cursor's meaning, the launch checks and the wire. Neither reaches into
 * the other — the engine cannot read a lifecycle state and the executor cannot compute a setpoint —
 * which is what keeps `docs/m4-mission-execution.md` §1's "one setpoint-stream owner" a structural
 * fact rather than a convention.
 *
 * **Every method is called on the engine's own 10 Hz thread, while the engine's lock is not held**
 * (they run from the tick's effect list, exactly as `announce` and `record.event` do). An
 * implementation must therefore be safe to call from that thread and must not block: the tick that
 * calls it is the tick that owes DJI its next setpoint.
 */
interface MissionRunSink {

    /**
     * The item at [seq] completed — the advancing edge, **once per item**, and the moment
     * `MISSION_ITEM_REACHED` goes on the wire. The timestamp is the entire value of that message.
     */
    fun onItemReached(seq: Int)

    /**
     * The cursor now points at [seq]: this is the item that will be flown next, which is exactly
     * what `MISSION_CURRENT.seq` means.
     *
     * Called on every advance and once at the start of a run, so a ground station never has to
     * infer the cursor from an arrival it may not have decoded.
     */
    fun onCursor(seq: Int)

    /**
     * The last item completed. The aircraft is **holding, in the air** — M4-5 reversed the
     * ground-to-ground answer, so this is not a landing and the operator must be told so
     * unmistakably.
     */
    fun onFinished(seq: Int)

    /**
     * The run stopped part-way, for [cause] — one row of §6.2 — with the cursor at [cursorSeq].
     *
     * The engine has already done whatever the abort ladder does to the *engagement*; this is the
     * second half of the same event, and it is the executor's business what it means for the plan.
     * **Never called with the mission still flying**: by the time this arrives the engine has let go
     * of the route.
     */
    fun onPaused(cause: MissionPauseCause, cursorSeq: Int)
}
