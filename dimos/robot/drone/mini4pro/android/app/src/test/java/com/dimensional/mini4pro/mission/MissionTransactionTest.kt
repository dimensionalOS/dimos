package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.command.StatusTexts
import com.dimensional.mini4pro.mission.MissionFixtures.LON
import com.dimensional.mini4pro.mission.MissionFixtures.takeoffAndHoldPlan
import com.dimensional.mini4pro.mission.MissionFixtures.item
import com.dimensional.mini4pro.mission.MissionFixtures.latE7
import com.dimensional.mini4pro.mission.MissionFixtures.northOf
import com.dimensional.mini4pro.mission.MissionFixtures.waypoint
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavFrame
import io.dronefleet.mavlink.common.MavMissionResult
import io.dronefleet.mavlink.common.MavMissionType
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.MissionAck
import io.dronefleet.mavlink.common.MissionClearAll
import io.dronefleet.mavlink.common.MissionCount
import io.dronefleet.mavlink.common.MissionItem
import io.dronefleet.mavlink.common.MissionItemInt
import io.dronefleet.mavlink.common.MissionRequest
import io.dronefleet.mavlink.common.MissionRequestInt
import io.dronefleet.mavlink.common.MissionRequestList
import io.dronefleet.mavlink.common.MissionRequestPartialList
import io.dronefleet.mavlink.common.MissionSetCurrent
import io.dronefleet.mavlink.common.MissionWritePartialList
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * M4 transport — the transaction engine: `docs/m4-mission-transport.md` §2, plus the three tests
 * that moved here from `HandshakeResponderTest` when its five mission branches were deleted
 * (JC-10).
 *
 * Same protocol as the guided suites: a fake send, a hand-cranked clock, no threads.
 *
 * Written to fail loudly for the transaction landmines:
 *
 *  - **a partial plan reaching the store** — the property this layer exists to have rather than
 *    to enforce. The execution half reads the store on its own 10 Hz thread, and a plan with a
 *    discontinuity in it is a discontinuity an aircraft flies
 *  - **a download spliced out of two plans**, which is the same defect pointing outwards: a commit
 *    between two instalments of a read hands QGC items 0–2 of one plan and 3–4 of another, raises
 *    nothing, and shows the operator a plan neither end ever held
 *  - a failed upload destroying the previous plan, so after a refusal the two ends disagree about
 *    what the plan is
 *  - `MAV_MISSION_INVALID_SEQUENCE` used as a "please resend" nudge, which under our PX4 identity
 *    kills the transaction outright (`PlanManager.cc:568` tolerates it only for ArduPilot)
 *  - a successful upload that is never acked, which QGC treats as a failure **and throws its own
 *    plan away for** (`:203`, `:843`)
 *  - an abandoned transaction that holds RECEIVING forever, so the next upload collides with a
 *    ghost
 *  - **a ±1 on either set-current door**, which is the one path QGC has already subtracted one on
 *    (`Vehicle.cc:2109-2111`). Both stubs used to take `seq` and discard it, so a decrement on
 *    both doors shipped green through all 1147 tests — measured, not supposed
 *  - a read refused, or answered with a lie, while the interlock is off (JC-1)
 *  - an upload accepted while the executor is flying the plan
 *  - the mission type dropped from a reply, which QGC silently discards (`:323`, `:411`, `:497`)
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted across the four
 * mission suites, code reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | items committed as they arrive instead of into the pending buffer | 6 |
 *  | a refused upload clears the store instead of leaving it alone | 1 |
 *  | the final `MISSION_ACK(ACCEPTED)` not sent | 7 |
 *  | admission not called before the ack (every item accepted) | 1 |
 *  | whole-plan check skipped (only per-item validation runs) | 1 |
 *  | a sequence gap answered `INVALID_SEQUENCE` instead of dropped | 1 |
 *  | a sequence gap accepted into the buffer at the expected index | 1 — was claimed 2, re-measured 2026-07-27 |
 *  | a duplicate item ends the transaction | 1 |
 *  | duplicate `MISSION_COUNT` ignored instead of restarting | 2 |
 *  | `MISSION_COUNT` while reading refused instead of taking over | 1 |
 *  | `MISSION_REQUEST_LIST` mid-upload discards the pending buffer | 1 |
 *  | the 5 s item timeout removed | 1 |
 *  | the 60 s transaction cap removed | 1 |
 *  | the timeout ends in silence instead of an explicit ack | 2 |
 *  | interlock ignored for upload (upload accepted with commands off) | 2 |
 *  | interlock ignored for clear-all | 2 |
 *  | interlock made to gate the *read* as well (JC-1 reversed, count 0 reported) | 2 |
 *  | the mid-flight upload announcement removed (M4-12's deferral goes unsaid) | 1 |
 *  | the deferral asked after the commit instead of before it | 1 |
 *  | executor-running veto removed for clear-all | 2 |
 *  | the running refusal sent as `UNSUPPORTED` rather than `DENIED` | 2 |
 *  | `MISSION_COUNT` 0 bypasses the clear path's guards | 2 |
 *  | mission type not echoed on the ack | 2 |
 *  | mission type not echoed on `MISSION_COUNT` | 1 |
 *  | the read-back normalises the item instead of serving it verbatim | 1 |
 *  | `MISSION_ITEM` (float) silently accepted rather than refused | 1 |
 *  | partial-list messages left to silence | 1 |
 *  | an inbound `MISSION_ACK` treated as a transaction start | 1 |
 *  | an out-of-range read answered with an item instead of `INVALID_SEQUENCE` | 1 |
 *  | each read instalment served from the live store instead of the download's snapshot | 2 |
 *  | the download's snapshot torn down when a `MISSION_COUNT` arrives (the splice, by the back door) | 2 |
 *  | the sender of a `MISSION_ITEM_INT` not checked against whose transaction is open | 1 |
 *  | the first-leg bound removed (the leg from home to item 0, unbounded again) | 1 |
 *  | the first-leg check run against an empty list rather than the plan | 1 |
 *  | the set-current refusal announced as "no mission running" whatever the result code said | 1 |
 *  | the transaction phase released after the commit instead of inside the deciding section | 1 |
 *  | `seq - 1` into the executor on the `MISSION_SET_CURRENT` (#41) door | 1 — **was 0 before this suite grew a recording stub** |
 *  | `seq - 1` into the executor on the `MAV_CMD_DO_SET_MISSION_CURRENT` (224) door | 1 — same |
 *  | `seq - 1` on **both** doors at once | 1 — same |
 *  | the addressing check removed (a message for another system is answered) | 1 |
 *  | `setCurrent` refusal returned as `UNSUPPORTED` (QGC caches the reply-less fallback) | 2 |
 *  | the stub claims `AUTO.MISSION` in the heartbeat | 1 |
 *
 * The 7 and the 6 are the two that matter. **The missing final ack** takes out most of the suite
 * because a successful upload QGC is not told about is an upload that did not happen on either
 * side — it throws its own plan away after 1500 ms. **Committing as items arrive** is the partial
 * plan reaching the store, which is the one state this layer is built to make unrepresentable
 * rather than merely refuse.
 */
class MissionTransactionTest {

    private companion object {
        const val GCS_SYS = 255
        const val GCS_COMP = 190
    }

    private val sent = mutableListOf<Any>()
    private val store = MissionStore()
    private var clock = 1_000L
    private var interlock = true
    private var executor: MissionExecution = NoMissionExecution

    /** DJI's home at the instant of commit. Null is "the aircraft has not told us yet". */
    private var home: GeoPoint? = GeoPoint(MissionFixtures.LAT, LON)

    private val transaction = MissionTransaction(
        store = store,
        send = { sent.add(it) },
        interlockEnabled = { interlock },
        execution = { executor },
        homeAtUpload = { home },
        amslDatumAtUpload = { 103.2 },
        nowMs = { clock },
    )

    /** An executor that reports a mission in flight, and refuses everything else. */
    private class RunningExecution : MissionExecution {
        override fun isRunning() = true
        override fun currentSeq(): Int? = 1
        override fun runState() = MissionRunState.ACTIVE
        override fun modeClaim(): Long? = null
        override fun setCurrent(seq: Int) = MavResult.MAV_RESULT_DENIED
        override fun start() = false
    }

    /**
     * An executor that **records the sequence number it was handed** and answers with whatever the
     * test asked for.
     *
     * The recording is the point. Both doors into set-current take a wire `seq` that QGC has
     * already decremented for us (`Vehicle.cc:2109-2111`), a stub that discards it makes a `± 1`
     * on either path invisible, and that was measured: decrementing the seq on *both* doors left
     * the whole 1147-test suite green.
     */
    private class RecordingSetCurrent(
        private val result: MavResult = MavResult.MAV_RESULT_DENIED,
        private val running: Boolean = false,
    ) : MissionExecution {
        val seqs = mutableListOf<Int>()
        override fun isRunning() = running
        override fun currentSeq(): Int? = null
        override fun runState() =
            if (running) MissionRunState.ACTIVE else MissionRunState.NO_MISSION
        override fun modeClaim(): Long? = null
        override fun setCurrent(seq: Int): MavResult {
            seqs.add(seq)
            return result
        }
        override fun start() = false
    }

    /**
     * An executor that runs [onAsked] when it is asked whether a mission is flying.
     *
     * That question is put on the receive thread **between** the critical section that assembled
     * the finished plan and the commit that follows it — which is precisely the window
     * [MissionTransaction.tick] must not be able to act in. Calling `tick()` from here reproduces
     * the two-thread interleaving on one thread, deterministically, with no sleeping.
     */
    private class TickAtCommit(private val onAsked: () -> Unit) : MissionExecution {
        override fun isRunning(): Boolean {
            onAsked()
            return false
        }
        override fun currentSeq(): Int? = null
        override fun runState() = MissionRunState.NO_MISSION
        override fun modeClaim(): Long? = null
        override fun setCurrent(seq: Int) = MavResult.MAV_RESULT_DENIED
        override fun start() = false
    }

    /** `MISSION_SET_CURRENT` (#41), the reply-less message door. */
    private fun setCurrentMessage(seq: Int) = transaction.onInbound(
        MissionSetCurrent.builder().targetSystem(1).targetComponent(1).seq(seq).build(),
        GCS_SYS, GCS_COMP,
    )

    // ------------------------------------------------------------------------------- helpers

    private fun requestList(type: MavMissionType = MavMissionType.MAV_MISSION_TYPE_MISSION) =
        transaction.onInbound(
            MissionRequestList.builder()
                .targetSystem(1).targetComponent(1).missionType(type).build(),
            GCS_SYS, GCS_COMP,
        )

    private fun count(n: Int, type: MavMissionType = MavMissionType.MAV_MISSION_TYPE_MISSION) =
        transaction.onInbound(
            MissionCount.builder()
                .targetSystem(1).targetComponent(1).count(n).missionType(type).build(),
            GCS_SYS, GCS_COMP,
        )

    private fun deliver(
        stored: StoredItem,
        fromSystem: Int = GCS_SYS,
        fromComponent: Int = GCS_COMP,
    ) = transaction.onInbound(
        MissionItemInt.builder()
            .targetSystem(1).targetComponent(1)
            .seq(stored.seq)
            .frame(EnumValue.create(MavFrame::class.java, stored.frame))
            .command(EnumValue.create(MavCmd::class.java, stored.command))
            .current(stored.current).autocontinue(stored.autocontinue)
            .param1(stored.param1).param2(stored.param2)
            .param3(stored.param3).param4(stored.param4)
            .x(stored.x).y(stored.y).z(stored.z)
            .missionType(EnumValue.create(MavMissionType::class.java, stored.missionType))
            .build(),
        fromSystem, fromComponent,
    )

    /** Runs a whole successful upload of [items] and clears the trace. */
    private fun upload(items: List<StoredItem>) {
        count(items.size)
        items.forEach { deliver(it) }
        sent.clear()
    }

    /** One `MISSION_REQUEST_INT` from the ground station — an instalment of a download. */
    private fun readItem(seq: Int) = transaction.onInbound(
        MissionRequestInt.builder()
            .targetSystem(1).targetComponent(1).seq(seq)
            .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
        GCS_SYS, GCS_COMP,
    )

    private val acks get() = sent.filterIsInstance<MissionAck>()
    private val requests get() = sent.filterIsInstance<MissionRequestInt>()
    private val texts get() = sent.filterIsInstance<Statustext>()

    private fun singleAck(): MissionAck = acks.single()

    // ------------------------------------------------------------------------------ download

    @Test
    fun `an empty store downloads as a count of zero, for every mission type`() {
        // Moved from HandshakeResponderTest. QGC's PlanManager treats MISSION_COUNT 0 as a
        // completed read transaction and stops there (`:338`), which is why this is still the
        // right answer for an empty store rather than a refusal.
        listOf(
            MavMissionType.MAV_MISSION_TYPE_MISSION,
            MavMissionType.MAV_MISSION_TYPE_FENCE,
            MavMissionType.MAV_MISSION_TYPE_RALLY,
        ).forEach { type ->
            sent.clear()
            requestList(type)
            val reply = sent.filterIsInstance<MissionCount>().single()
            assertEquals(0, reply.count())
            // Echoing the type is the difference between a reply and silence: QGC checks it on
            // every inbound message and silently drops a mismatch.
            assertEquals(type, reply.missionType().entry())
            assertEquals(GCS_SYS, reply.targetSystem())
        }
    }

    @Test
    fun `a read serves the stored bytes back verbatim`() {
        val original = item(
            0, MissionCommands.NAV_WAYPOINT,
            param1 = 3.5f, param2 = 4f, param3 = 0f, param4 = Float.NaN,
            x = northOf(12.0), y = latE7(LON), z = 12.345f, current = 1, autocontinue = 0,
        )
        upload(listOf(original))

        requestList()
        assertEquals(1, sent.filterIsInstance<MissionCount>().single().count())

        sent.clear()
        transaction.onInbound(
            MissionRequestInt.builder()
                .targetSystem(1).targetComponent(1).seq(0)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        val served = sent.filterIsInstance<MissionItemInt>().single()
        // The read-back is what an operator sees after a reconnect. A paraphrase of their plan is
        // worse than no plan, because it looks authoritative.
        assertEquals(0, served.seq())
        assertEquals(MissionCommands.NAV_WAYPOINT, served.command().value())
        assertEquals(MissionFrames.GLOBAL_RELATIVE_ALT, served.frame().value())
        assertEquals(3.5f, served.param1(), 0f)
        assertEquals(4f, served.param2(), 0f)
        assertTrue(served.param4().isNaN())
        assertEquals(northOf(12.0), served.x())
        assertEquals(12.345f, served.z(), 0f)
        assertEquals(1, served.current())
        assertEquals(0, served.autocontinue())
    }

    @Test
    fun `the pre-int request form is answered too`() {
        // MISSION_REQUEST (#40) is field-identical to #51 and QGC decodes both as
        // mission_request_int. Other ground stations only know the old one.
        upload(listOf(waypoint(0, 10.0)))
        transaction.onInbound(
            MissionRequest.builder()
                .targetSystem(1).targetComponent(1).seq(0)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(1, sent.filterIsInstance<MissionItemInt>().size)
    }

    @Test
    fun `a read past the end is INVALID_SEQUENCE rather than an invented item`() {
        upload(listOf(waypoint(0, 10.0)))
        transaction.onInbound(
            MissionRequestInt.builder()
                .targetSystem(1).targetComponent(1).seq(7)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, singleAck().type().entry())
        assertTrue(sent.filterIsInstance<MissionItemInt>().isEmpty())
    }

    @Test
    fun `a plan committed mid-read cannot splice itself into the download`() {
        // THE download landmine. MISSION_COUNT and the items that follow it are one answer
        // delivered in instalments; serving each instalment from the store at its own instant
        // hands QGC items 0-2 of the old plan and 3-4 of the new one — a plan neither end ever
        // held, assembled with no error raised anywhere, and shown to the operator as theirs.
        val first = listOf(waypoint(0, 10.0), waypoint(1, 20.0), waypoint(2, 30.0))
        upload(first)

        requestList()
        assertEquals(3, sent.filterIsInstance<MissionCount>().single().count())
        sent.clear()

        readItem(0)
        assertEquals(first[0].x, sent.filterIsInstance<MissionItemInt>().single().x())
        sent.clear()

        // A different, shorter plan commits between instalment 1 and instalment 2.
        upload(listOf(waypoint(0, 40.0), waypoint(1, 50.0)))
        assertEquals(2, store.count)
        sent.clear()

        readItem(1)
        readItem(2)
        val served = sent.filterIsInstance<MissionItemInt>()
        assertEquals(listOf(1, 2), served.map { it.seq() })
        // Both from the plan the read began with — not the store's current one, and item 2 is not
        // an INVALID_SEQUENCE either, which is what a shorter new plan used to produce mid-read.
        assertEquals(first[1].x, served[0].x())
        assertEquals(first[2].x, served[1].x())
        assertTrue("a consistent read raises nothing", acks.isEmpty())
    }

    @Test
    fun `a request past the end of the snapshot the read began with is INVALID_SEQUENCE`() {
        // The other half: a read that outlives its snapshot's range gets the answer that is true
        // of *that read* — the plan it is reading has this many items and no more. Serving item 3
        // of a plan committed after the count went out would be true of nothing.
        upload(listOf(waypoint(0, 10.0), waypoint(1, 20.0)))
        requestList()
        assertEquals(2, sent.filterIsInstance<MissionCount>().single().count())

        upload(listOf(waypoint(0, 10.0), waypoint(1, 20.0), waypoint(2, 30.0), waypoint(3, 40.0)))
        assertEquals(4, store.count)
        sent.clear()

        readItem(3)
        assertEquals(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, singleAck().type().entry())
        assertTrue(sent.filterIsInstance<MissionItemInt>().isEmpty())
    }

    // -------------------------------------------------------------------------------- upload

    @Test
    fun `a good upload is requested item by item and acked once at the end`() {
        val plan = takeoffAndHoldPlan()
        count(plan.size)

        // We drive the pace by index, one request at a time (`_handleMissionRequest`, `:487`).
        assertEquals(listOf(0), requests.map { it.seq() })
        assertTrue("nothing is committed until the last item", store.isEmpty)

        deliver(plan[0])
        assertEquals(listOf(0, 1), requests.map { it.seq() })
        assertTrue(store.isEmpty)

        deliver(plan[1])
        assertEquals(listOf(0, 1, 2), requests.map { it.seq() })
        assertTrue(store.isEmpty)

        deliver(plan[2])
        // The final MISSION_ACK is mandatory and it is ours to send: without it QGC reports
        // "vehicle failed to send final ack" and **throws the plan away**.
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(MavMissionType.MAV_MISSION_TYPE_MISSION, singleAck().missionType().entry())
        assertEquals(3, store.count)
        assertEquals(plan, store.items)
        // Provenance recorded at the commit, never used as a datum.
        assertEquals(GeoPoint(MissionFixtures.LAT, LON), store.plan()!!.homeAtUpload)
        assertEquals(103.2, store.plan()!!.amslDatumAtUpload!!, 0.0)
    }

    @Test
    fun `a refused item ends the transaction and leaves the previous plan alone`() {
        upload(takeoffAndHoldPlan())
        val previous = store.items
        val previousPlanId = store.planId

        count(2)
        deliver(waypoint(0, 10.0))
        sent.clear()
        // An absolute frame: the refusal that will surprise an operator.
        deliver(item(1, MissionCommands.NAV_WAYPOINT, frame = MissionFrames.GLOBAL,
            x = northOf(20.0), y = latE7(LON)))

        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME, singleAck().type().entry())
        assertEquals(MissionStatusTexts.ABSOLUTE_ALTITUDE, texts.single().text())
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, texts.single().severity().entry())
        // The corollary that makes a failed upload safe: after a refusal both ends agree that the
        // old plan is the plan. QGC discards its own half on the same edge.
        assertEquals(previous, store.items)
        assertEquals(previousPlanId, store.planId)
        assertFalse(transaction.isReceiving)
    }

    @Test
    fun `a whole-plan refusal names the reason and not an item`() {
        // Two waypoints 2600 m apart: each item is fine on its own, the plan is not. (900 m until
        // 2026-07-30, when the leg bound became Ivan's 2 km — the scenario moved, the assertion did
        // not.)
        count(2)
        deliver(waypoint(0, 0.0))
        sent.clear()
        deliver(waypoint(1, 2_600.0))

        assertEquals(MavMissionResult.MAV_MISSION_ERROR, singleAck().type().entry())
        assertTrue(store.isEmpty)
        assertTrue(texts.single().text().contains("${MissionAdmission.MAX_LEG_M.toInt()}m max"))
    }

    @Test
    fun `the first leg is checked against home when home is known, and skipped when it is not`() {
        // F10, settled 2026-07-27. Every leg *between two items* was bounded; the first leg — from
        // where the aircraft is standing to the first waypoint — had no predecessor in the list
        // and so was bounded by nothing. A one-waypoint plan 3 km away passed every check.
        //
        // The guarantee is still the execution half's at Start, where home is known because the
        // aircraft is on it. This is the courtesy half: refuse it at the desk, where it costs the
        // operator seconds rather than a walk back to the aircraft with propellers turning.
        count(1)
        deliver(waypoint(0, 2_600.0))
        assertEquals(MavMissionResult.MAV_MISSION_ERROR, singleAck().type().entry())
        // The sentence reports what was *measured*, not what the fixture asked for. The fixture
        // converts 2600 m to degrees, the wire quantises that to 1e-7 degrees, and we measure the
        // quantised value back — so the number lands a few millimetres either side of 2600 and the
        // truncation to whole metres decides which integer you see. (2600 was chosen over a round
        // 2500 for exactly that reason: 2500 round-trips to 2499.9955 and reads as `2499`.)
        //
        // The distance moved on 2026-07-30 because the bound did — Ivan's 2 km — and the fixture
        // moved with it rather than the assertion: **this is still the reported number, not
        // `MAX_LEG_M`**, which is the property the exact string is here to pin.
        assertEquals("Leg 0 is 2600m: 2000m max", texts.single().text())
        assertTrue("and nothing partial reaches the store", store.isEmpty)

        // Inside the bound, the same plan is fine.
        sent.clear()
        count(1)
        deliver(waypoint(0, 90.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())

        // Home unknown: the check does not run. That is expected, not a gap — there is nothing to
        // measure from, and Start will measure it when there is.
        sent.clear()
        home = null
        count(1)
        deliver(waypoint(0, 2_600.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(1, store.count)
        assertNull("and the plan records that home was unknown", store.plan()!!.homeAtUpload)
    }

    @Test
    fun `an over-long plan is refused at the count, before a single round trip`() {
        count(MissionAdmission.MAX_ITEMS + 1)
        assertEquals(MavMissionResult.MAV_MISSION_NO_SPACE, singleAck().type().entry())
        assertTrue("no item may be requested for a plan we have already refused", requests.isEmpty())
        assertFalse(transaction.isReceiving)
    }

    @Test
    fun `an upload of zero items clears the plan`() {
        upload(takeoffAndHoldPlan())
        count(0)
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertTrue(store.isEmpty)
        assertEquals(MissionStatusTexts.PLAN_CLEARED, texts.single().text())
    }

    // ---------------------------------------------------------- the seven ways it goes wrong

    @Test
    fun `a duplicate MISSION_COUNT restarts the transaction from seq 0`() {
        // QGC re-sends MISSION_COUNT when it saw no request from us (`:205-214`), so a duplicate
        // means it believes we heard nothing — and the only state both ends agree on is the
        // start.
        count(3)
        deliver(waypoint(0, 10.0))
        sent.clear()

        count(3)
        assertEquals(listOf(0), requests.map { it.seq() })
        assertEquals(0, transaction.received)
        assertTrue(acks.isEmpty())
    }

    @Test
    fun `a MISSION_COUNT with a different count resizes and restarts`() {
        count(3)
        deliver(waypoint(0, 10.0))
        sent.clear()

        // There is no reading under which the old buffer is still valid.
        count(2)
        assertEquals(listOf(0), requests.map { it.seq() })
        deliver(waypoint(0, 10.0))
        deliver(waypoint(1, 30.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(2, store.count)
    }

    @Test
    fun `a MISSION_COUNT arriving mid-read abandons the read and begins the write`() {
        upload(takeoffAndHoldPlan())
        requestList()
        sent.clear()

        // The writer is the newer intent and the reader has its own timeout. Refusing the
        // operator's upload because a stale download never closed would be the wrong trade.
        count(1)
        assertEquals(listOf(0), requests.map { it.seq() })
        assertTrue(acks.isEmpty())
    }

    @Test
    fun `a MISSION_REQUEST_LIST mid-upload is answered from the committed store`() {
        upload(takeoffAndHoldPlan())
        count(2)
        deliver(waypoint(0, 10.0))
        sent.clear()

        // A read is non-destructive and the committed store is a true answer to it. This can
        // genuinely happen: QGC's Fly and Plan views both hold a PlanMasterController.
        requestList()
        assertEquals(3, sent.filterIsInstance<MissionCount>().single().count())
        assertTrue("the pending buffer must survive a read", transaction.isReceiving)
        assertEquals(1, transaction.received)

        // …and the upload still completes.
        deliver(waypoint(1, 30.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(2, store.count)
    }

    @Test
    fun `an item from a second ground station cannot join the first one's transaction`() {
        // Two ground stations on one link is not exotic — QGC plus a tablet, or a second operator.
        // An item from A folded into B's transaction produces a plan **nobody authored**: every
        // item admissible on its own, the count right, the ack sent, and a list of waypoints in an
        // order no operator ever put them in.
        count(2)
        deliver(waypoint(0, 10.0))
        sent.clear()

        // The other station's idea of item 1, arriving into our transaction with B.
        deliver(waypoint(1, 300.0), fromSystem = 254, fromComponent = 190)
        assertTrue("a message that was not part of this conversation cannot end it", acks.isEmpty())
        assertTrue(requests.isEmpty())
        assertEquals("the intruder's item must not be counted", 1, transaction.received)
        assertTrue(transaction.isReceiving)

        // The owner's own item 1 still completes the transaction, and it is that item that lands.
        deliver(waypoint(1, 30.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(listOf(waypoint(0, 10.0), waypoint(1, 30.0)), store.items)
    }

    @Test
    fun `a sequence gap is dropped, never nacked`() {
        // THE landmine: MAV_MISSION_INVALID_SEQUENCE is the code this case was invented for, and
        // under our PX4 identity sending it *kills the transaction* — QGC tolerates it only for
        // ArduPilot (`:568`). Dropping costs one retry cycle; nacking costs the upload.
        count(3)
        deliver(waypoint(0, 10.0))
        sent.clear()

        deliver(waypoint(2, 30.0))
        assertTrue("a gap must not be acked", acks.isEmpty())
        assertTrue("a gap must not advance the request", requests.isEmpty())
        assertEquals(1, transaction.received)
        assertTrue(transaction.isReceiving)

        // The transaction is still alive and the right item still completes it.
        deliver(waypoint(1, 20.0))
        deliver(waypoint(2, 30.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
    }

    @Test
    fun `a duplicate item is dropped and the transaction continues`() {
        // A duplicate is a retransmission, not a disagreement.
        count(2)
        deliver(waypoint(0, 10.0))
        sent.clear()

        deliver(waypoint(0, 10.0))
        assertTrue(acks.isEmpty())
        assertEquals(1, transaction.received)
        assertTrue(transaction.isReceiving)

        deliver(waypoint(1, 30.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(2, store.count)
    }

    @Test
    fun `QGC vanishing mid-upload times out into an ack, and the old plan survives`() {
        upload(takeoffAndHoldPlan())
        val previous = store.items

        count(12)
        deliver(waypoint(0, 10.0))
        deliver(waypoint(1, 20.0))
        deliver(waypoint(2, 30.0))
        deliver(waypoint(3, 40.0))
        sent.clear()

        clock += MissionTransaction.ITEM_TIMEOUT_MS - 1
        transaction.tick()
        assertTrue("the timer must not fire early", sent.isEmpty())

        clock += 1
        transaction.tick()
        // Never silence: once we have requested item 0, a stall is unrecoverable from QGC's side,
        // so our own timeout has to end in an explicit MISSION_ACK.
        assertEquals(
            MavMissionResult.MAV_MISSION_OPERATION_CANCELLED,
            singleAck().type().entry(),
        )
        assertEquals("Upload stopped: 4 of 12 items, plan unchanged", texts.single().text())
        assertEquals(previous, store.items)
        assertFalse(transaction.isReceiving)

        // And the next upload does not collide with a ghost.
        sent.clear()
        count(1)
        assertEquals(listOf(0), requests.map { it.seq() })
    }

    @Test
    fun `a slow but live upload is still cut off by the whole-transaction cap`() {
        val startedAt = clock
        count(MissionAdmission.MAX_ITEMS)
        // An item every 4 s keeps the 5 s idle timer alive indefinitely — it is refreshed on
        // every arrival and never gets within a second of expiring. The 60 s cap is the only
        // thing that stops an upload from outlasting the operator's attention.
        var delivered = 0
        while (delivered < MissionAdmission.MAX_ITEMS) {
            clock += 4_000L
            transaction.tick()
            if (!transaction.isReceiving) break
            deliver(waypoint(delivered, 10.0 * (delivered + 1)))
            delivered++
        }

        assertFalse("the cap must have fired", transaction.isReceiving)
        assertTrue(
            "it must be the cap and not the idle timer",
            clock - startedAt >= MissionTransaction.TRANSACTION_TIMEOUT_MS,
        )
        assertTrue("items were still arriving when it fired", delivered in 1 until MissionAdmission.MAX_ITEMS)
        assertEquals(
            MavMissionResult.MAV_MISSION_OPERATION_CANCELLED,
            acks.last().type().entry(),
        )
        assertTrue(store.isEmpty)
    }

    @Test
    fun `the transaction cap cannot fire against an upload that has already completed`() {
        // The transaction ends inside the critical section that decided it, not after the commit.
        // With the phase left as RECEIVING across that window, a tick() from Bridge's thread
        // discards the transaction, sends OPERATION_CANCELLED and announces "plan unchanged" — and
        // then this thread commits and sends ACCEPTED. Two acks for one upload, and an operator
        // told "unchanged" about a plan that is sitting in the store.
        //
        // **What this test can and cannot distinguish, stated plainly.** The arrangement it kills
        // is any where the phase outlives the section that decided the plan — measured against
        // "the phase released only after the commit", 1. It does *not* reproduce the pre-fix
        // code's own window, which lay between the lock release and a call to a pure function
        // (`MissionAdmission.planRefusal`) with no injectable seam inside it: from one thread that
        // window is unobservable, and a two-thread version of it would be a race the test would
        // sometimes lose. So this pins the property that makes the whole class unreachable —
        // nothing outside the deciding section ever sees RECEIVING — rather than the one
        // interleaving.
        count(2)
        deliver(waypoint(0, 10.0))
        // The cap is now due: the transaction has been open longer than 60 s. The idle timer is
        // not — every arrival refreshes it — so this is the cap and nothing else.
        clock += MissionTransaction.TRANSACTION_TIMEOUT_MS
        sent.clear()

        var ticks = 0
        executor = TickAtCommit { ticks++; transaction.tick() }
        deliver(waypoint(1, 30.0))

        assertEquals("the window must actually have been entered", 1, ticks)
        assertEquals(2, store.count)
        assertEquals(
            "one upload gets exactly one ack",
            listOf(MavMissionResult.MAV_MISSION_ACCEPTED),
            acks.map { it.type().entry() },
        )
        assertTrue(
            "and nothing may say the plan is unchanged about a plan that is in the store",
            texts.none { it.text().contains("plan unchanged") },
        )
    }

    @Test
    fun `an abandoned read closes silently`() {
        upload(takeoffAndHoldPlan())
        requestList()
        sent.clear()

        clock += MissionTransaction.READ_IDLE_TIMEOUT_MS
        transaction.tick()
        // QGC has its own error path for a stalled read and a second complaint from us is noise.
        assertTrue(sent.isEmpty())
    }

    // ------------------------------------------------------------- when we refuse to talk at all

    @Test
    fun `with the interlock off an upload is refused byte for byte as it was before M4`() {
        interlock = false
        count(3)
        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED, singleAck().type().entry())
        assertEquals(MavMissionType.MAV_MISSION_TYPE_MISSION, singleAck().missionType().entry())
        assertTrue("the pre-M4 refusal carried no sentence", texts.isEmpty())
        assertTrue(requests.isEmpty())
        assertFalse(transaction.isReceiving)
    }

    @Test
    fun `with the interlock off a clear-all is refused byte for byte as it was before M4`() {
        upload(takeoffAndHoldPlan())
        interlock = false
        transaction.onInbound(
            MissionClearAll.builder().targetSystem(1).targetComponent(1)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED, singleAck().type().entry())
        assertEquals(3, store.count)
    }

    @Test
    fun `with the interlock off a read still tells the truth`() {
        // JC-1, and it is a deliberate divergence from M2 §Q2's "the pre-feature answer". §Q2
        // exists so that switching commands off cannot leave a half-armed actuating path, and a
        // read actuates nothing. Reporting zero while we hold a plan would be a lie told to make
        // a rule easier to state — and the operator most likely to have the interlock off is
        // exactly the one checking what the bridge is holding.
        upload(takeoffAndHoldPlan())
        interlock = false
        requestList()
        assertEquals(3, sent.filterIsInstance<MissionCount>().single().count())

        sent.clear()
        transaction.onInbound(
            MissionRequestInt.builder().targetSystem(1).targetComponent(1).seq(1)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(1, sent.filterIsInstance<MissionItemInt>().single().seq())
    }

    @Test
    fun `an upload while the executor is flying is accepted and says it takes effect at next Start`() {
        // M4-12: Ivan overruled the design's MAV_MISSION_DENIED with "YES". Of the two readings,
        // this is the safe one — the plan lands in the store, the running mission carries on with
        // the snapshot it began with, and nothing swaps under a moving aircraft.
        upload(takeoffAndHoldPlan())
        executor = RunningExecution()

        count(2)
        assertEquals("the operator's upload must not be refused mid-flight", listOf(0), requests.map { it.seq() })
        deliver(waypoint(0, 10.0))
        deliver(waypoint(1, 30.0))

        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertEquals(2, store.count)
        // An accepted upload that silently does not take effect is exactly the belief this
        // project's honesty boundaries exist to prevent, so it is said out loud.
        assertEquals(MissionStatusTexts.PLAN_STORED_FOR_NEXT_START, texts.single().text())
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, texts.single().severity().entry())
    }

    @Test
    fun `an upload with nothing flying is accepted silently`() {
        // The sentence is about the *deferral*, not about the upload. With no mission running
        // there is nothing deferred, and announcing anyway would be noise on the routine path.
        count(1)
        deliver(waypoint(0, 10.0))
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertTrue(texts.isEmpty())
    }

    @Test
    fun `a clear-all while the executor is flying is still DENIED`() {
        // M4-12 answered the *upload* question. Its argument carries over — the executor holds
        // its own snapshot — but its consequence does not: an emptied store withholds
        // MISSION_CURRENT entirely (JC-7) and answers a read with count 0, so QGC would show an
        // empty Plan view for an aircraft that is visibly flying a plan. That is the state §3.2
        // exists to prevent. Replacing a plan leaves the operator something to look at; removing
        // one does not.
        upload(takeoffAndHoldPlan())
        executor = RunningExecution()
        transaction.onInbound(
            MissionClearAll.builder().targetSystem(1).targetComponent(1)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        // DENIED renders as "Not accepting any mission commands" (`:788`), which is precisely
        // true while we are busy — as opposed to UNSUPPORTED's "we will never do this" (JC-4).
        assertEquals(MavMissionResult.MAV_MISSION_DENIED, singleAck().type().entry())
        assertEquals(MissionStatusTexts.MISSION_RUNNING, texts.single().text())
        assertEquals(3, store.count)
    }

    @Test
    fun `an upload of zero items gets the same answer as a clear-all, in every state`() {
        // One operator intent must not get two different answers depending on which message QGC
        // happened to choose, so MISSION_COUNT 0 is routed to the clear path verbatim.
        upload(takeoffAndHoldPlan())
        executor = RunningExecution()
        count(0)
        assertEquals(MavMissionResult.MAV_MISSION_DENIED, singleAck().type().entry())
        assertEquals(3, store.count)

        sent.clear()
        executor = NoMissionExecution
        count(0)
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertTrue(store.isEmpty)
    }

    @Test
    fun `a read is never blocked, in any state`() {
        // The one thing an operator watching an aircraft fly a plan actually needs.
        upload(takeoffAndHoldPlan())
        executor = RunningExecution()
        interlock = false
        requestList()
        assertEquals(3, sent.filterIsInstance<MissionCount>().single().count())
    }

    // ---------------------------------------------------------------------- the other messages

    @Test
    fun `clear-all empties the store and says so once per link session`() {
        upload(takeoffAndHoldPlan())
        val clear = MissionClearAll.builder().targetSystem(1).targetComponent(1)
            .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build()

        transaction.onInbound(clear, GCS_SYS, GCS_COMP)
        // Honest now that the plan we clear is ours and only ours: HandshakeResponder's old
        // refusal was right when the aircraft's DJI Fly route was the only mission in play.
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertTrue(store.isEmpty)
        // JC-3: the belief this prevents — "the aircraft now has no mission at all" — is one an
        // operator would be *right* to hold about any other autopilot.
        assertEquals(MissionStatusTexts.PLAN_CLEARED, texts.single().text())

        sent.clear()
        transaction.onInbound(clear, GCS_SYS, GCS_COMP)
        assertEquals(MavMissionResult.MAV_MISSION_ACCEPTED, singleAck().type().entry())
        assertTrue("once per link session, or it is alarm fatigue", texts.isEmpty())

        // A new link is a new session, and a ground station that has been told nothing.
        transaction.reset()
        sent.clear()
        transaction.onInbound(clear, GCS_SYS, GCS_COMP)
        assertEquals(MissionStatusTexts.PLAN_CLEARED, texts.single().text())
    }

    @Test
    fun `the float item form is refused rather than converted`() {
        // A float latitude is ~1 m of precision lost — the same defect DO_SET_HOME has, and it
        // was measured there. Converting would hide it.
        count(1)
        sent.clear()
        transaction.onInbound(
            MissionItem.builder()
                .targetSystem(1).targetComponent(1).seq(0)
                .frame(EnumValue.create(MavFrame::class.java, MissionFrames.GLOBAL_RELATIVE_ALT))
                .command(EnumValue.create(MavCmd::class.java, MissionCommands.NAV_WAYPOINT))
                .current(0).autocontinue(1)
                .param1(0f).param2(0f).param3(0f).param4(Float.NaN)
                .x(38.0f).y(23.7f).z(10f)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION))
                .build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED, singleAck().type().entry())
        assertTrue(store.isEmpty)
        assertFalse(transaction.isReceiving)
    }

    @Test
    fun `partial list messages are answered honestly rather than left to silence`() {
        // §1.5: dead protocol for QGC, live for another GCS. Silence makes a sender retry into a
        // timeout and blame the link; MAV_MISSION_ERROR says less than "we do not do this".
        transaction.onInbound(
            MissionWritePartialList.builder().targetSystem(1).targetComponent(1)
                .startIndex(0).endIndex(2)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED, singleAck().type().entry())

        sent.clear()
        transaction.onInbound(
            MissionRequestPartialList.builder().targetSystem(1).targetComponent(1)
                .startIndex(0).endIndex(2)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(MavMissionResult.MAV_MISSION_UNSUPPORTED, singleAck().type().entry())
    }

    @Test
    fun `an inbound MISSION_ACK closes the read instead of starting anything`() {
        upload(takeoffAndHoldPlan())
        requestList()
        sent.clear()

        transaction.onInbound(
            MissionAck.builder().targetSystem(1).targetComponent(1)
                .type(MavMissionResult.MAV_MISSION_ACCEPTED)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertTrue("an ack is consumed, never answered", sent.isEmpty())

        // The read is closed, so the idle timer has nothing left to fire on.
        clock += MissionTransaction.READ_IDLE_TIMEOUT_MS * 2
        transaction.tick()
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `set-current refuses through a channel that can be heard`() {
        // §1.6: our first ack to command 224 decides, for the lifetime of QGC's vehicle instance,
        // which protocol it speaks to us. An UNSUPPORTED caches the fallback and every later
        // press goes to the reply-less MISSION_SET_CURRENT message with no ack at all.
        val result = transaction.onSetCurrentCommand(2)
        assertEquals(MavResult.MAV_RESULT_DENIED, result)
        assertTrue(
            "UNSUPPORTED would cost us the reply channel for the whole session",
            result != MavResult.MAV_RESULT_UNSUPPORTED,
        )

        // The message form has no reply at all, so the refusal can only be a sentence.
        transaction.onInbound(
            MissionSetCurrent.builder().targetSystem(1).targetComponent(1).seq(2).build(),
            GCS_SYS, GCS_COMP,
        )
        assertEquals(1, texts.size)
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, texts.single().severity().entry())
    }

    @Test
    fun `both set-current doors hand the executor the wire seq, unaltered`() {
        // **The one path where a ±1 is invisible**, and it was measured: decrementing the seq on
        // *both* doors left the whole 1147-test suite green, because both stubs took `seq` and
        // threw it away and only the returned result was ever asserted.
        //
        // This is also the one place QGC has *already* subtracted one — it deletes the planned-home
        // marker and renumbers down by one before uploading, and `Vehicle.cc:2109-2111` sends the
        // decremented value. Our sequence space is the wire's. Subtracting again jumps the aircraft
        // to the item before the one the operator pressed, and at seq 0 it asks for item -1.
        val recorder = RecordingSetCurrent()
        executor = recorder

        // MAV_CMD_DO_SET_MISSION_CURRENT (224), the command door.
        assertEquals(MavResult.MAV_RESULT_DENIED, transaction.onSetCurrentCommand(7))
        // MISSION_SET_CURRENT (#41), the reply-less message door.
        setCurrentMessage(4)
        // And zero, where an off-by-one stops being a wrong item and becomes a negative index.
        transaction.onSetCurrentCommand(0)
        setCurrentMessage(0)

        assertEquals(listOf(7, 4, 0, 0), recorder.seqs)
    }

    @Test
    fun `a set-current refusal says what the result code says and nothing more`() {
        // The sentence used to be "Item N refused: no mission running" for *every* non-ACCEPTED
        // result. That was true of the stub, which is the only executor there has ever been — and
        // it is a lie the day a real one refuses a bad index or an envelope violation, because the
        // operator is then told no mission is running while watching one fly. This project's rule
        // is that a sentence must not name a state we have not observed, and a MAV_RESULT is the
        // entire observation available on this path.
        upload(takeoffAndHoldPlan())

        executor = RecordingSetCurrent(MavResult.MAV_RESULT_TEMPORARILY_REJECTED, running = true)
        setCurrentMessage(3)
        assertEquals("Item 3 set-current: rejected for now", texts.single().text())
        assertFalse(
            "the sentence must not claim a state we did not observe",
            texts.single().text().contains("no mission running"),
        )

        sent.clear()
        executor = RecordingSetCurrent(MavResult.MAV_RESULT_DENIED, running = true)
        setCurrentMessage(3)
        assertEquals("Item 3 set-current: denied", texts.single().text())
        assertEquals(MavSeverity.MAV_SEVERITY_ERROR, texts.single().severity().entry())

        // And an executor that accepted says nothing at all: the aircraft moving is the report.
        sent.clear()
        executor = RecordingSetCurrent(MavResult.MAV_RESULT_ACCEPTED, running = true)
        setCurrentMessage(3)
        assertTrue(texts.isEmpty())
    }

    @Test
    fun `a message addressed to another system is ignored`() {
        transaction.onInbound(
            MissionRequestList.builder()
                .targetSystem(42).targetComponent(1)
                .missionType(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)).build(),
            GCS_SYS, GCS_COMP,
        )
        assertTrue(sent.isEmpty())
    }

    @Test
    fun `every refusal sentence fits the STATUSTEXT field`() {
        // 50 bytes is a fixed-width char[50] with no length prefix: anything longer is silently
        // cut on the wire, and the operator then searches for a string that does not exist.
        val sentences = listOf(
            MissionStatusTexts.ABSOLUTE_ALTITUDE,
            MissionStatusTexts.MISSION_RUNNING,
            MissionStatusTexts.PLAN_STORED_FOR_NEXT_START,
            MissionStatusTexts.PLAN_CLEARED,
            MissionStatusTexts.NO_NAVIGABLE_ITEM,
            MissionStatusTexts.uploadAbandoned(4, 12),
            MissionStatusTexts.planTooLong(137, 20),
            MissionStatusTexts.speedRefused(3, 8.0, 3.0),
            MissionStatusTexts.altitudeRefused(2, 45.0, 30.0),
            // The two longest sentences in this family, measured at the bounds actually in force
            // since 2026-07-30 (2 km per leg, 4 km per path): four digits on both sides of each,
            // which is the worst case a 50-byte STATUSTEXT has to carry.
            MissionStatusTexts.legTooLong(4, 2_400.0, MissionAdmission.MAX_LEG_M),
            MissionStatusTexts.planTooFar(4_210.0, MissionAdmission.MAX_TOTAL_M),
            MissionStatusTexts.itemRefused(5, "no camera control"),
            MissionStatusTexts.frameRefused(4, 10),
            MissionStatusTexts.setCurrentRefused(19, MavResult.MAV_RESULT_TEMPORARILY_REJECTED),
        )
        sentences.forEach { text ->
            assertTrue(
                "'$text' is ${text.toByteArray(Charsets.UTF_8).size} bytes",
                text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
            )
        }
        // The counted ones from the design document, pinned exactly.
        assertEquals(38, MissionStatusTexts.ABSOLUTE_ALTITUDE.toByteArray().size)
        assertEquals(41, MissionStatusTexts.MISSION_RUNNING.toByteArray().size)
        assertEquals(40, MissionStatusTexts.PLAN_STORED_FOR_NEXT_START.toByteArray().size)
        assertEquals(38, MissionStatusTexts.PLAN_CLEARED.toByteArray().size)
        assertEquals(45, MissionStatusTexts.uploadAbandoned(4, 12).toByteArray().size)
        assertEquals(33, MissionStatusTexts.itemRefused(5, "no camera control").toByteArray().size)
        assertEquals(33, MissionStatusTexts.speedRefused(3, 8.0, 3.0).toByteArray().size)
    }

    @Test
    fun `the stub executor answers everything without flying anything`() {
        assertFalse(NoMissionExecution.isRunning())
        assertNull(NoMissionExecution.currentSeq())
        assertEquals(MissionRunState.NO_MISSION, NoMissionExecution.runState())
        // The one that matters. Claiming AUTO.MISSION is an unanswered decision belonging to the
        // operator (§7.3), and the safe answer to an unanswered decision is the pre-feature
        // behaviour — the heartbeat keeps coming from the aircraft.
        assertNull(
            "a mode claim from a stub would be exactly the echo this layer exists to prevent",
            NoMissionExecution.modeClaim(),
        )
        assertEquals(MavResult.MAV_RESULT_DENIED, NoMissionExecution.setCurrent(0))
        assertFalse(NoMissionExecution.start())
    }

    @Test
    fun `nothing in the inbound mission surface can commit a plan by itself`() {
        // The whole surface, replayed against an empty store with the interlock off — the state a
        // phone boots into. Nothing may reach the store.
        interlock = false
        val type = EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION)
        val surface = listOf(
            MissionRequestList.builder().targetSystem(1).targetComponent(1).missionType(type).build(),
            MissionCount.builder().targetSystem(1).targetComponent(1).count(3).missionType(type).build(),
            MissionRequestInt.builder().targetSystem(1).targetComponent(1).seq(0).missionType(type).build(),
            MissionRequest.builder().targetSystem(1).targetComponent(1).seq(0).missionType(type).build(),
            MissionClearAll.builder().targetSystem(1).targetComponent(1).missionType(type).build(),
            MissionSetCurrent.builder().targetSystem(1).targetComponent(1).seq(0).build(),
            MissionAck.builder().targetSystem(1).targetComponent(1)
                .type(MavMissionResult.MAV_MISSION_ACCEPTED).missionType(type).build(),
        )
        surface.forEach { transaction.onInbound(it, GCS_SYS, GCS_COMP) }
        // And a full item delivery on top, which cannot land because no transaction was opened.
        takeoffAndHoldPlan().forEach { deliver(it) }

        assertTrue(store.isEmpty)
        assertEquals(0, store.planId)
        assertNotNull("the surface must still have answered", acks.firstOrNull())
    }
}
