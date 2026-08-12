package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.mission.MissionFixtures.takeoffAndHoldPlan
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MissionCurrent
import io.dronefleet.mavlink.common.MissionItemReached
import io.dronefleet.mavlink.common.MissionState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * M4 transport — the progress messages: `docs/m4-mission-transport.md` §7.1 and §7.2.
 *
 * Written to fail loudly for the progress landmines:
 *
 *  - **`MISSION_CURRENT` sent with an empty store** (JC-7). `seq` has no "none" value, so a 0
 *    for a vehicle with no mission makes QGC highlight plan item 1 — a wrong sentinel where the
 *    rule is *sentinels or silence*
 *  - the off-by-one of §4.1 applied here as well as by QGC, so a twelve-item plan highlights the
 *    wrong waypoint. Our sequence space is the **wire's**; QGC adds the 1 back for display
 *  - a heartbeat rate rather than a 1 Hz one, or a change that waits for the next tick
 *  - `mission_state` contradicting `total` in the same message
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted across the four
 * mission suites, code reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | JC-7 removed (`MISSION_CURRENT` sent for an empty store as seq 0) | 2 |
 *  | `seq` published as `cursor + 1` (the off-by-one applied twice) | 3 |
 *  | the executor's cursor ignored (always 0) | 2 |
 *  | emit-on-change removed (1 Hz only) | 5 |
 *  | the 1 Hz cadence removed (emit only on change) | 1 |
 *  | `total` filled from the executor rather than the store | 1 |
 *  | `NO_MISSION` reported verbatim beside a non-zero `total` | 1 |
 *  | `MISSION_ITEM_REACHED` not sent on the arrival edge | 1 |
 *  | a cursor past the end of the stored plan published rather than withheld | 1 |
 */
class MissionProgressTest {

    private val sent = mutableListOf<Any>()
    private val store = MissionStore()
    private var executor: MissionExecution = NoMissionExecution

    private val progress = MissionProgress(
        store = store,
        execution = { executor },
        send = { sent.add(it) },
    )

    /** An executor with a cursor, so the wire numbering can be pinned against something. */
    private class AtItem(private val seq: Int, private val state: MissionRunState) : MissionExecution {
        override fun isRunning() = state == MissionRunState.ACTIVE
        override fun currentSeq(): Int = seq
        override fun runState() = state
        override fun modeClaim(): Long? = null
        override fun setCurrent(seq: Int) = MavResult.MAV_RESULT_DENIED
        override fun start() = false
    }

    private fun current(nowMs: Long): List<MissionCurrent> =
        progress.messages(nowMs).filterIsInstance<MissionCurrent>()

    @Test
    fun `nothing at all is sent while the store is empty`() {
        // JC-7. The same rule as HOME_POSITION, which is simply not sent when home is unknown:
        // PLAN.md's boundary is sentinels or silence, and `seq` has no sentinel.
        assertTrue(progress.messages(0L).isEmpty())
        assertTrue(progress.messages(10_000L).isEmpty())
        assertTrue(progress.messages(60_000L).isEmpty())
    }

    @Test
    fun `a committed plan reports seq 0 before the mission starts`() {
        store.commit(takeoffAndHoldPlan(), null, null, 0L)
        val message = current(0L).single()

        // "The next item we will fly is 0", which is true — and it is wire-numbered, so QGC draws
        // it as plan item 1. The planned-home marker it deleted before uploading is the 1.
        assertEquals(0, message.seq())
        assertEquals(3, message.total())
        // The executor does not exist, so it reports NO_MISSION — which would contradict a total
        // of 3 in the very same message. A plan is committed, so NOT_STARTED is transport's own
        // truth and the message stays internally consistent.
        assertEquals(MissionState.MISSION_STATE_NOT_STARTED, message.missionState().entry())
    }

    @Test
    fun `seq is the wire number, never the plan-view number`() {
        store.commit(takeoffAndHoldPlan(), null, null, 0L)
        executor = AtItem(2, MissionRunState.ACTIVE)

        val message = current(0L).single()
        // Getting this wrong is invisible in a two-item plan and highlights the wrong waypoint in
        // a twelve-item one. QGC adds the 1 back itself (`MissionController::currentMissionIndex`).
        assertEquals(2, message.seq())
        assertEquals(MissionState.MISSION_STATE_ACTIVE, message.missionState().entry())
    }

    @Test
    fun `it streams at 1 Hz and jumps immediately on a change`() {
        store.commit(takeoffAndHoldPlan(), null, null, 0L)

        assertEquals(1, current(0L).size)
        // Nothing has changed and the second has not elapsed.
        assertTrue(progress.messages(500L).isEmpty())
        assertEquals(1, current(1_000L).size)

        // A cursor move is announced on the very next tick rather than up to a second later —
        // QGC's Plan view animates from this message alone.
        executor = AtItem(1, MissionRunState.ACTIVE)
        val onChange = current(1_100L).single()
        assertEquals(1, onChange.seq())
    }

    @Test
    fun `a state change alone is enough to re-send`() {
        store.commit(takeoffAndHoldPlan(), null, null, 0L)
        executor = AtItem(1, MissionRunState.ACTIVE)
        assertEquals(1, current(0L).size)

        executor = AtItem(1, MissionRunState.PAUSED)
        val paused = current(10L).single()
        assertEquals(MissionState.MISSION_STATE_PAUSED, paused.missionState().entry())
    }

    @Test
    fun `a cursor that is not an index into the stored plan is withheld, never published`() {
        // M4-12's cost, and the reviewer measured it: `seq=8 total=12` before an upload,
        // `seq=8 total=3` after — QGC highlights item 9 of a three-item plan and the operator is
        // looking at a plan the aircraft has never flown a metre of, with the wrong item lit.
        //
        // Unlike the NO_MISSION-beside-a-total contradiction there is no true value to substitute:
        // clamping would claim the mission is at the last item of the new plan. So it is silence,
        // on the same rule as JC-7.
        store.commit(List(12) { MissionFixtures.waypoint(it, 10.0 * (it + 1)) }, null, null, 0L)
        executor = AtItem(8, MissionRunState.ACTIVE)
        val consistent = current(0L).single()
        assertEquals(8, consistent.seq())
        assertEquals(12, consistent.total())

        // A fresh, shorter plan lands mid-flight. The executor keeps its own snapshot and its
        // cursor stops being an index into what we hold.
        store.commit(takeoffAndHoldPlan(), null, null, 1L)
        assertTrue(progress.messages(2_000L).isEmpty())
        assertTrue(progress.messages(10_000L).isEmpty())

        // The boundary, both sides of it: `total - 1` is a real index and is published; `total` is
        // one past the end and is not.
        executor = AtItem(2, MissionRunState.ACTIVE)
        assertEquals(2, current(11_000L).single().seq())
        executor = AtItem(3, MissionRunState.ACTIVE)
        assertTrue(progress.messages(12_000L).isEmpty())
    }

    @Test
    fun `clearing the plan silences it again`() {
        store.commit(takeoffAndHoldPlan(), null, null, 0L)
        assertEquals(1, current(0L).size)

        store.clear()
        assertTrue(progress.messages(5_000L).isEmpty())

        // And the next upload announces itself immediately rather than waiting for the change
        // memory to disagree with something stale.
        store.commit(takeoffAndHoldPlan(), null, null, 1L)
        assertEquals(1, current(5_100L).size)
    }

    @Test
    fun `an arrival is announced on the edge, once per item`() {
        // JC-6: nothing in QGC reads MISSION_ITEM_REACHED — not decoded, not displayed, not used
        // to advance anything. It is sent anyway because it costs one message per waypoint and
        // puts an unambiguous arrival timestamp on the wire that a pcap can see. Recorded as
        // knowingly redundant so nobody optimises it away and nobody debugs its absence.
        progress.reachedSink.onReached(0)
        progress.reachedSink.onReached(1)

        val reached = sent.filterIsInstance<MissionItemReached>()
        assertEquals(listOf(0, 1), reached.map { it.seq() })
    }
}
