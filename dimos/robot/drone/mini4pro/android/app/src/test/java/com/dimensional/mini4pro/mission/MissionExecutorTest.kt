package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.guided.MissionLifecycle
import com.dimensional.mini4pro.guided.MissionPauseCause
import com.dimensional.mini4pro.guided.MissionRoute
import com.dimensional.mini4pro.guided.MissionRunSink
import com.dimensional.mini4pro.guided.ResumeBlock
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Px4Mode
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The lifecycle half: what a Start does, what a pause does, what the wire says, and the one
 * property everything else hangs off — **`PAUSED → RUNNING` has exactly one cause.**
 *
 * The engine is faked here, because this file is about the *decisions* rather than the flying;
 * `GuidedMissionTest` is the other way round. The launch gate is pinned next door in
 * `MissionLaunchTest`, so what is asserted here is that the executor **consults** it, spends and
 * clears blocks correctly, and never claims on the wire what it has not observed.
 *
 * Written to fail loudly for:
 *
 *  - **an automatic resume**. Any timer, watchdog or helpful retry that moves `PAUSED` to `RUNNING`
 *    without a Start is the failure M4-11 was answered to prevent, and the enumeration below is what
 *    catches one added later.
 *  - **`modeClaim` non-null with any conjunct missing**, including the two this class owns.
 *  - **`runState` overstating** — `ACTIVE` while the aircraft is not being commanded.
 *  - a `MISSION_SET_CURRENT` answered `UNSUPPORTED`, which permanently costs QGC its reply channel.
 *  - a resume block that never clears, or one that clears without the operator being told.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one at a time, applied to the shipped source, the **whole** suite
 * run, confirmed red, reverted. Counts are failing tests across all 1802.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a resume block never spent by its refusal (a leg timeout locks the session out) | 1 |
 *  | a `WHILE_CONDITION` block never re-evaluated | 1 |
 *  | the claim drops its `RUNNING` conjunct | 1 |
 *  | the claim drops its committed-plan conjunct (alone) | **0 — alive on purpose** |
 *  | **both** claim conjuncts dropped | 1 |
 *  | `runState` reports `ACTIVE` without the engine flying | 1 |
 *  | `setCurrent` answers `UNSUPPORTED` | 1 |
 *  | the store's change notification removed | 1 |
 *
 * **The committed-plan conjunct is structurally implied and kept anyway.** `RUNNING` is reachable
 * only from `LOADED`, `PAUSED` or `FINISHED`, all of which require a plan, and clearing the store
 * drops straight to `NO_MISSION` — so `store.plan() != null` is already true whenever the second
 * conjunct is. It stays because M4-1's argument is *three observed facts*, and a reader of
 * [MissionExecutor.modeClaim] must be able to see all three rather than be told that one of them
 * follows from another. Dropping both is caught.
 */
class MissionExecutorTest {

    private companion object {
        const val LAT = MissionFixtures.LAT
        const val LON = MissionFixtures.LON

        fun plan(): List<StoredItem> = listOf(
            MissionFixtures.waypoint(0, 20.0),
            MissionFixtures.waypoint(1, 40.0),
        )
    }

    /** Records what was asked of the engine, and answers however the test needs. */
    private class FakeEngine : MissionExecutor.MissionEngine {
        var verdict = Verdict.ACCEPTED
        var flying = false
        var starts = 0
        var lastIndex: Int? = null
        var lastRejoining: Boolean? = null
        var sink: MissionRunSink? = null

        override fun missionStart(
            route: MissionRoute,
            startIndex: Int,
            rejoining: Boolean,
            sink: MissionRunSink,
        ): Verdict {
            starts++
            lastIndex = startIndex
            lastRejoining = rejoining
            this.sink = sink
            if (verdict == Verdict.ACCEPTED) sink.onCursor(route[startIndex].seq)
            return verdict
        }

        override fun missionFlying(): Boolean = flying
    }

    private class Harness(engineAttached: Boolean = true) {
        var interlock = true
        var state = stateAt()
        val engine = FakeEngine()
        val wire = mutableListOf<Any>()
        val reached = mutableListOf<Int>()
        val store = MissionStore()

        val executor = MissionExecutor(
            store = store,
            engine = { if (engineAttached) engine else null },
            aircraftState = { state },
            interlockEnabled = { interlock },
            reached = { seq -> reached += seq },
            announcer = Announcer(StatusTextSink { wire += it }),
        )

        init {
            // The store's own change notification is what `Bridge` wires; here it is called by hand
            // so a test can commit without a transaction.
            }

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun commit(items: List<StoredItem> = plan()) {
            store.commit(items, GeoPoint(LAT, LON), 100.0, 0L)
            executor.onPlanCommitted()
        }

        fun clear() {
            store.clear()
            executor.onPlanCleared()
        }

        /** Start, and have the engine confirm it is flying — the ordinary running state. */
        fun running(): Harness {
            commit()
            assertTrue(executor.start())
            engine.flying = true
            return this
        }

        fun pause(cause: MissionPauseCause, seq: Int = 0) {
            engine.flying = false
            executor.onPaused(cause, seq)
        }

        companion object {
            fun stateAt(
                isFlying: Boolean? = true,
                battery: Int? = 80,
                homeSet: Boolean? = true,
                flightMode: String? = "JOYSTICK",
                positionAge: Long = 0L,
            ) = AircraftState(
                latitude = LAT, longitude = LON,
                relativeAltitude = 20.0, takeoffAltitudeAmsl = 100.0,
                homeLatitude = LAT, homeLongitude = LON, homeLocationSet = homeSet,
                batteryPercent = battery, isFlying = isFlying, flightMode = flightMode,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to 0L,
                    Signal.VELOCITY to 0L,
                ),
            )
        }
    }

    // ------------------------------------------------------------------ the happy path

    @Test
    fun `a committed plan is LOADED, and a Start makes it RUNNING`() {
        val h = Harness()
        assertEquals(MissionLifecycle.NO_MISSION, h.executor.state)
        h.commit()
        assertEquals(MissionLifecycle.LOADED, h.executor.state)
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
        assertEquals(1, h.engine.starts)
        assertEquals(0, h.engine.lastIndex)
        assertFalse(h.engine.lastRejoining!!)
    }

    @Test
    fun `the cursor is reported from the engine's own advances, and reaching is passed straight on`() {
        val h = Harness().running()
        assertEquals(0, h.executor.currentSeq())
        h.engine.sink!!.onItemReached(0)
        h.engine.sink!!.onCursor(1)
        assertEquals(listOf(0), h.reached)
        assertEquals(1, h.executor.currentSeq())
    }

    @Test
    fun `the last item completing makes the mission FINISHED, which means holding in the air`() {
        val h = Harness().running()
        h.engine.sink!!.onFinished(1)
        assertEquals(MissionLifecycle.FINISHED, h.executor.state)
        assertEquals(MissionRunState.COMPLETE, h.executor.runState())
        // Not landed. `FINISHED` is "arrived at the last item and holding" (M4-5).
    }

    // ------------------------------------------------------------------ resume discipline

    @Test
    fun `THE BINDING ANSWER - nothing but a Start moves PAUSED to RUNNING`() {
        val h = Harness().running()
        h.pause(MissionPauseCause.RC_STICK_GRAB, seq = 1)
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)

        // Every event the outside world can deliver, none of which is a Start.
        h.executor.onLanded()
        h.executor.onHomeMoved()
        h.executor.onItemReached(1)
        h.executor.onCursor(1)
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        // ...and the engine reporting a pause again cannot resume it either.
        h.executor.onPaused(MissionPauseCause.QGC_LINK_LOSS, 1)
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
    }

    @Test
    fun `a resume goes through the engine as a rejoin, at the paused cursor`() {
        val h = Harness().running()
        h.engine.sink!!.onCursor(1)
        h.pause(MissionPauseCause.RC_STICK_GRAB, seq = 1)
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
        assertEquals(2, h.engine.starts)
        assertEquals(1, h.engine.lastIndex)
        assertTrue("a resume must fly a resting rejoin leg", h.engine.lastRejoining!!)
    }

    @Test
    fun `a refused Start leaves the state exactly where it was, and says why`() {
        val h = Harness().running()
        h.pause(MissionPauseCause.RC_STICK_GRAB, seq = 1)
        h.interlock = false
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        assertEquals(1, h.engine.starts) // nothing reached the engine
        assertTrue(h.texts().any { it.contains(MissionLaunch.REASON_INTERLOCK) })
    }

    @Test
    fun `a WHILE_CONDITION block clears when its condition does, and not before`() {
        val h = Harness().running()
        h.interlock = false
        h.pause(MissionPauseCause.INTERLOCK_OFF, seq = 1)
        assertEquals(ResumeBlock.INTERLOCK_OFF, h.executor.block)
        // With the interlock still off, the refusal names the block.
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        // Turning it back on clears the condition — and does **not** resume anything by itself.
        h.interlock = true
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
    }

    @Test
    fun `a SPENT_ON_REFUSAL block refuses once, tells the operator, and clears`() {
        // §6.1: "spent as soon as its condition clears **or the operator is told about it**". A leg
        // timeout usually means wind or an obstacle brake, and the useful safety behaviour is one
        // refusal that makes the operator stop and look — not a lockout they work around by
        // restarting the app.
        val h = Harness().running()
        h.pause(MissionPauseCause.LEG_TIMEOUT, seq = 1)
        assertEquals(ResumeBlock.LEG_TIMEOUT, h.executor.block)
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        assertTrue(h.texts().any { it.contains(ResumeBlock.LEG_TIMEOUT.reason) })
        assertNull("the block was not spent by the refusal", h.executor.block)
        // The second, deliberate press proceeds.
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
    }

    @Test
    fun `a SESSION block stands, however many times Start is pressed`() {
        val h = Harness().running()
        h.pause(MissionPauseCause.SERIOUS_LOW_BATTERY, seq = 1)
        repeat(3) {
            assertTrue(h.executor.start())
            assertEquals(MissionLifecycle.PAUSED, h.executor.state)
        }
        assertEquals(ResumeBlock.BATTERY_DEPLETED, h.executor.block)
    }

    @Test
    fun `a resumable pause carries no block at all`() {
        val h = Harness().running()
        h.pause(MissionPauseCause.QGC_LINK_LOSS, seq = 1)
        assertNull(h.executor.block)
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
    }

    @Test
    fun `an abandoning cause drops the cursor entirely`() {
        val h = Harness().running()
        h.pause(MissionPauseCause.BRIDGE_STOPPED, seq = 1)
        assertEquals(MissionLifecycle.NO_MISSION, h.executor.state)
        assertNull(h.executor.currentSeq())
    }

    // ------------------------------------------------------------------ the store's edges

    @Test
    fun `a plan committed under a paused cursor drops it, on the same edge`() {
        val h = Harness().running()
        h.engine.sink!!.onCursor(1)
        h.pause(MissionPauseCause.RC_STICK_GRAB, seq = 1)
        h.commit(listOf(MissionFixtures.waypoint(0, 15.0), MissionFixtures.waypoint(1, 30.0)))
        assertEquals(MissionLifecycle.LOADED, h.executor.state)
        assertEquals(0, h.executor.currentSeq())
    }

    @Test
    fun `M4-12 - a plan committed under a RUNNING mission does not disturb the run`() {
        val h = Harness().running()
        h.engine.sink!!.onCursor(1)
        h.commit(listOf(MissionFixtures.waypoint(0, 15.0)))
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
        assertEquals(1, h.executor.currentSeq())
        assertEquals(1, h.engine.starts) // nothing was restarted
    }

    @Test
    fun `clearing the plan leaves nothing behind`() {
        val h = Harness().running()
        h.clear()
        assertEquals(MissionLifecycle.NO_MISSION, h.executor.state)
        assertNull(h.executor.currentSeq())
        assertEquals(MissionRunState.NO_MISSION, h.executor.runState())
    }

    @Test
    fun `a bridge stop leaves nothing to resume into the next session`() {
        val h = Harness().running()
        h.executor.onBridgeStopped()
        assertEquals(MissionLifecycle.NO_MISSION, h.executor.state)
        assertNull(h.executor.currentSeq())
    }

    @Test
    fun `the store's own change hook fires on commit and on clear`() {
        // §2.1 rule 5, wired where every write must pass rather than on one caller.
        val seen = mutableListOf<Boolean>()
        val store = MissionStore(onChanged = { seen += (it != null) })
        store.commit(plan(), GeoPoint(LAT, LON), 100.0, 0L)
        store.clear()
        assertEquals(listOf(true, false), seen)
    }

    // ------------------------------------------------------------------ the wire

    @Test
    fun `THE CLAIM NEEDS ALL THREE CONJUNCTS`() {
        val h = Harness()
        // 1. No plan.
        assertNull(h.executor.modeClaim())
        h.commit()
        assertNull(h.executor.modeClaim())
        // 2. Not RUNNING.
        h.engine.flying = true
        assertNull("claimed while merely LOADED", h.executor.modeClaim())
        assertTrue(h.executor.start())
        assertEquals(Px4Mode.AUTO_MISSION, h.executor.modeClaim())
        // 3. The engine stops flying: the claim goes with it.
        h.engine.flying = false
        assertNull(h.executor.modeClaim())
        // ...and a paused mission claims nothing, whatever the engine says.
        h.engine.flying = true
        h.executor.onPaused(MissionPauseCause.RC_STICK_GRAB, 0)
        assertNull("claimed while PAUSED", h.executor.modeClaim())
    }

    @Test
    fun `with no engine attached nothing is ever claimed`() {
        val h = Harness(engineAttached = false)
        h.commit()
        assertTrue(h.executor.start())
        assertNull(h.executor.modeClaim())
        assertEquals(MissionLifecycle.LOADED, h.executor.state)
    }

    @Test
    fun `runState may understate but never overstates`() {
        val h = Harness()
        assertEquals(MissionRunState.NO_MISSION, h.executor.runState())
        h.commit()
        assertEquals(MissionRunState.NOT_STARTED, h.executor.runState())
        assertTrue(h.executor.start())
        // RUNNING but the engine is not yet flying: PAUSED is the honest understatement — we are not
        // commanding the aircraft right now — and it is briefly wrong in the direction that cannot
        // hurt anyone. A Plan view animating a flight that has not begun is the alternative.
        assertEquals(MissionRunState.PAUSED, h.executor.runState())
        h.engine.flying = true
        assertEquals(MissionRunState.ACTIVE, h.executor.runState())
        h.pause(MissionPauseCause.RC_STICK_GRAB, 0)
        assertEquals(MissionRunState.PAUSED, h.executor.runState())
    }

    @Test
    fun `no state ever reports ACTIVE without the engine flying`() {
        val h = Harness()
        h.commit()
        for (state in MissionLifecycle.values()) {
            h.engine.flying = false
            assertFalse(
                "state $state reported ACTIVE with nothing flying",
                h.executor.runState() == MissionRunState.ACTIVE
            )
        }
    }

    @Test
    fun `isRunning is true only while RUNNING, which is what the transport's veto reads`() {
        val h = Harness()
        assertFalse(h.executor.isRunning())
        h.commit()
        assertFalse(h.executor.isRunning())
        assertTrue(h.executor.start())
        assertTrue(h.executor.isRunning())
        h.pause(MissionPauseCause.RC_STICK_GRAB, 0)
        assertFalse(h.executor.isRunning())
    }

    @Test
    fun `set-current is refused with DENIED, never UNSUPPORTED`() {
        // Our first answer to command 224 decides which protocol QGC speaks to us for the lifetime
        // of its vehicle instance: UNSUPPORTED caches a fallback to the deprecated message, which
        // has no reply channel at all.
        val h = Harness()
        assertEquals(MavResult.MAV_RESULT_DENIED, h.executor.setCurrent(0))
        assertEquals(MavResult.MAV_RESULT_DENIED, h.executor.setCurrent(3))
    }

    @Test
    fun `currentSeq is null with nothing committed and 0 with a plan that has not started`() {
        val h = Harness()
        assertNull(h.executor.currentSeq())
        h.commit()
        // §7.1's own sentence: "the next item we will fly is 0", which is true.
        assertEquals(0, h.executor.currentSeq())
    }

    // ------------------------------------------------------------------ M4-14

    @Test
    fun `a DO_SET_HOME blocks Start until the aircraft lands and takes off again`() {
        val h = Harness()
        h.commit()
        h.executor.onHomeMoved()
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.LOADED, h.executor.state)
        assertTrue(h.texts().any { it.contains(MissionLaunch.REASON_HOME_MOVED) })
        h.executor.onLanded()
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
    }

    @Test
    fun `a Start while already running is a no-op rather than a second launch check`() {
        // QGC's guided buttons send the same SET_MODE three times about 1.34 s apart, so this is the
        // ordinary case. Re-running a launch check against a moving aircraft is what must not happen.
        val h = Harness().running()
        assertTrue(h.executor.start())
        assertTrue(h.executor.start())
        assertEquals(1, h.engine.starts)
        assertEquals(MissionLifecycle.RUNNING, h.executor.state)
    }

    @Test
    fun `an engine that refuses the route leaves the state where it was, and says so`() {
        val h = Harness()
        h.commit()
        h.engine.verdict = Verdict.DENIED
        assertTrue(h.executor.start())
        assertEquals(MissionLifecycle.LOADED, h.executor.state)
        assertTrue(h.texts().any { it.contains("Mission refused") })
    }
}
