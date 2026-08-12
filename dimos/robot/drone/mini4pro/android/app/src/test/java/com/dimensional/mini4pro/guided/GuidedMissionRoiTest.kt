package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.StatusTexts
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot

/**
 * **A plan's `DO_SET_ROI_LOCATION` / `DO_SET_ROI_NONE` items, inside the engine** — the camera pointed
 * by the plan rather than by a hand on QGC's map: when each item acts, which state it acts on, what the
 * nose does about it, and what a mission's ending does to a camera the plan aimed.
 *
 * Design authority: Ivan, 2026-07-30 — *"fix mission execution to respect plan ROI"* — against
 * `/home/lesh/Documents/QGroundControl Daily/Missions/big1.plan` (transcribed in
 * `MissionBig1PlanTest`), whose **item 6** is a `DO_SET_ROI_LOCATION` at 37.99387681/23.7257871 in
 * `MAV_FRAME_GLOBAL_RELATIVE_ALT` with `z = 0` and whose **item 8** is a `DO_SET_ROI_NONE`. His
 * sequencing expectation is the specification, verbatim: *"after wp6 it looks at roi, at wp8 it should
 * stop"* — QGC's display numbering, which is the wire `seq` plus one for the deleted planned-home
 * marker, so on the wire it is: the ROI is in force for the leg to item 7 and off again once item 7 is
 * reached. Every fixture here is that shape (an ROI item between two waypoints, a clear after the
 * second) unless it is deliberately broken.
 *
 * Same protocol as `GuidedMissionTest` and `GuidedPrecisionLandTest`: fake port, fake gimbal,
 * hand-cranked clock, no aircraft. The pointing arithmetic is pinned in `RoiGuidanceTest`, the live
 * `DO_SET_ROI` door in `GuidedRoiTest`, the store's sticky walk in `MissionStoreTest`, and the route
 * translation in `MissionLaunchTest`. **This file is about what the engine does with a route that
 * carries an ROI.**
 *
 * ## The sequencing precedent this feature follows, rather than inventing
 *
 * A plan's `DO_` items are sticky state and are resolved **once, at commit**, by the same walk that
 * resolves `DO_CHANGE_SPEED` (`MissionStore.resolve` → `ResolvedLeg.roi`). `MissionLaunch.routeOf` now
 * carries that answer onto every step as `MissionStep.roi` instead of dropping it, and the engine
 * applies the **difference** on each cursor move. So "when does an ROI item act?" is answered in exactly
 * one place and it is the place that already answered it for speed. Nothing here re-scans a plan, and
 * nothing here can apply an ROI a leg early or a leg late, because the only thing that can be applied is
 * the ROI the route says belongs to the step the cursor is on.
 *
 * ## The yaw verdict, argued rather than assumed
 *
 * **The mission leg already routes its setpoint through the one yaw owner, and this suite proves it.**
 * `tickMissionLocked` ends every ordinary leg with `targetYawLocked`, whose first rule is *an ROI wins*
 * → `roiYawLocked` → `RoiGuidance.yawRate`; `yawAuthorityLocked` counts `mission != null` as one of our
 * own manoeuvres, so the permission is the same one an orbit and a goto have. The climb leg
 * (`tickMissionClimbLocked`) and the completion hold do the same. That matters on **this** airframe more
 * than on any other: DJI #527 means the gimbal cannot yaw at all, so if the mission leg did not steer
 * the nose, a plan's ROI would leave the camera pointing at nothing while the aircraft flew along the
 * route — the operator believing their subject was in frame, which is the failure mode this project
 * calls dishonest rather than merely wrong. `THE YAW` asserts the leg's commanded yaw *equals*
 * `RoiGuidance.yawRate` evaluated independently, so a future edit that gives the mission its own yaw
 * law fails here rather than in the air.
 *
 * Written to fail loudly for:
 *
 *  - **the route dropping the ROI again** — the 2026-07-30 gap, which is what this feature closed.
 *  - **the mission growing its own ROI state**: a second target, a second aiming path, or a second
 *    clearing path. `THE DOORS` pins that the plan's item and QGC's live click are the *same* state, by
 *    clearing one through the other.
 *  - **a `DO_SET_ROI_NONE` item ignored**, so the camera stays on a subject the plan finished with.
 *  - **the ROI applied one leg early or one leg late.**
 *  - **the relative-frame height lost on the plan path**, so a target 10 m up is aimed at as if it were
 *    on the ground (11° of pitch error at 50 m of range in this fixture).
 *  - **the nose not steered on a mission leg**, or steered by a second law.
 *  - **a mission ending with the camera left on a plan's target silently** — or, just as bad, the camera
 *    *swung to a default* nobody asked for.
 *  - **an operator's own ROI taken away** by a mission ending, which is theirs and outlives manoeuvres.
 *  - **a plan's ROI remembered across an abort**, which would re-acquire a dead plan's target the next
 *    time an engagement is confirmed.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-30, one breakage at a time applied to the shipped `src/main`, the **whole**
 * suite run per mutant with `test-results` deleted first, confirmed red, reverted. Counts are failing
 * tests across all 2644 — **measured, not estimated**.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `routeOf` drops the ROI again (`roi = null` on every step) | 2 |
 *  | the mission grows its own ROI state instead of the shared owner | 5 |
 *  | a `DO_SET_ROI_NONE` item ignored (the clear never applied) | 1 |
 *  | the ROI applied one leg **early** (at the step before the plan's) | 11 |
 *  | the ROI applied one leg **late** (on the step after) | 14 |
 *  | the relative-altitude honouring lost on the mission path | 4 |
 *  | the mission leg's yaw no longer routed through the ROI owner | 2 |
 *  | a mission ending leaves the plan's ROI driving the camera | 3 |
 *  | a mission ending clears the **operator's** ROI too | 1 |
 *  | the ROI re-taken at every corner (the change test removed) | 5 |
 *
 * ### The three results worth reading rather than counting
 *
 * **The two off-by-one rows are the largest, at 11 and 14, and that is the shape a sequencing feature
 * should have.** Almost every test in this file names a leg, so moving *which* leg the ROI belongs to
 * breaks the camera test, the yaw test, the height test, the record test and both ending tests at once.
 * The late mutant scores three higher than the early one because it also drags the ROI past the plan's
 * `DO_SET_ROI_NONE` — the clear arrives a leg after the plan asked for it, so the endings fail too.
 * A sequencing edit that comes back green on this file has not been tested; it has been deleted.
 *
 * **The `DO_SET_ROI_NONE` row is 1, and the low number is the point rather than a gap.** Exactly one
 * test can distinguish "the clear was applied" from "the clear was never needed", because every other
 * assertion about the camera is satisfied by an ROI that is still correctly *set*. That single test is
 * Ivan's sentence — *"at wp8 it should stop"* — and it is the reason it is written as an assertion that
 * the camera stops being *driven* over twenty further ticks, not merely that a record line appeared.
 *
 * **`the ROI re-taken at every corner` reaches outside this file (5), and the outside one is the
 * interesting one.** `GuidedMissionTest::no STATUSTEXT is emitted per leg` fails, because re-taking an
 * unchanged ROI announces `ROI accepted` at every corner — which is the M4 rule that mission *progress*
 * never touches the 50-byte announcement channel, broken from an unexpected direction. The re-take also
 * resets the camera's rate limiter each leg, which is why the two ending tests fail with it: the
 * provenance of the ROI in force gets rewritten by the last corner rather than naming the item that
 * actually set it.
 *
 * ### One caveat about re-measuring
 *
 * The `second ROI state` row measured **6** on its first run and **5** on a clean re-run: the extra was
 * `TagRecogniserTest::anArmingRuleThatThrowsDisarms`, which drives a real background thread and can fail
 * on timing alone under a mutant run's load. `GuidedPrecisionLandTest`'s table carries the same warning
 * about the same suite. A re-measurement that comes back one higher than a row above should check
 * **which** test failed before believing the number. Pre-existing, not this feature's, and left alone.
 */
class GuidedMissionRoiTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        /** The published pressure-altitude datum this session. */
        const val DATUM = 100.0

        /** Cruise: the height every leg here is flown at. */
        const val ALT = 20.0

        /** The wire `seq` of each step, spaced the way a plan with `DO_` items between waypoints is. */
        const val SEQ_FIRST = 1
        const val SEQ_UNDER_ROI = 3
        const val SEQ_AFTER_NONE = 5

        /** The ROI's place: 60 m north and 40 m east of the start, so no leg points at it by accident. */
        const val ROI_NORTH = 60.0
        const val ROI_EAST = 40.0

        fun latNorthOf(metres: Double): Double = LAT + metres / Geo.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        /** The plan's ROI item as `MissionLaunch.routeOf` spells it — frame 3, the item's own z. */
        fun roiAt(northM: Double = ROI_NORTH, eastM: Double = ROI_EAST, relAltM: Double? = 0.0) =
            RoiCommand.pointingAt(latNorthOf(northM), lonEastOf(eastM), relAltM)

        fun waypoint(
            seq: Int,
            northM: Double,
            eastM: Double = 0.0,
            rest: Boolean = false,
            roi: RoiCommand? = null,
        ) = MissionStep(
            seq = seq,
            kind = MissionStepKind.WAYPOINT,
            latDeg = latNorthOf(northM),
            lonDeg = lonEastOf(eastM),
            relAltM = ALT,
            switchRadiusM = MissionGuidance.R_SWITCH_M,
            rest = rest,
            roi = roi,
        )

        /**
         * **big1's shape**: three waypoints due north at 30 / 60 / 90 m with a `DO_SET_ROI_LOCATION`
         * between the first and the second and a `DO_SET_ROI_NONE` between the second and the third.
         * The sticky resolution therefore puts the ROI on the **middle** step only.
         */
        fun roiRoute(relAltM: Double? = 0.0): MissionRoute = MissionRoute.of(
            planId = 1,
            steps = listOf(
                waypoint(SEQ_FIRST, 30.0),
                waypoint(SEQ_UNDER_ROI, 60.0, roi = roiAt(relAltM = relAltM)),
                waypoint(SEQ_AFTER_NONE, 90.0, rest = true),
            ),
        )

        /** The same route with the ROI never cleared — a plan that ends with the camera still on target. */
        fun uncleared(): MissionRoute = MissionRoute.of(
            planId = 1,
            steps = listOf(
                waypoint(SEQ_FIRST, 30.0),
                waypoint(SEQ_UNDER_ROI, 60.0, roi = roiAt()),
                waypoint(SEQ_AFTER_NONE, 90.0, rest = true, roi = roiAt()),
            ),
        )

        /** No ROI anywhere — the control case, and every plan authored before 2026-07-30. */
        fun plainRoute(): MissionRoute = MissionRoute.of(
            planId = 1,
            steps = listOf(
                waypoint(SEQ_FIRST, 30.0),
                waypoint(SEQ_UNDER_ROI, 60.0),
                waypoint(SEQ_AFTER_NONE, 90.0, rest = true),
            ),
        )

        /** The depression angle the camera should be commanded, from the aircraft to the ROI. */
        fun expectedPitch(northM: Double, eastM: Double, altM: Double, targetRelAltM: Double): Double {
            val horizontal = hypot(ROI_NORTH - northM, ROI_EAST - eastM)
            return OrbitGuidance.gimbalPitchDeg(altM - targetRelAltM, horizontal)
        }
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var enableOnSuccess: (() -> Unit)? = null

        data class Sent(val pitch: Double, val roll: Double, val yaw: Double, val verticalThrottle: Double)

        val sent = mutableListOf<Sent>()
        var onState: ((VirtualStickSnapshot) -> Unit)? = null
        var onRc: ((RcSticks) -> Unit)? = null

        val modes = StickModes(
            rollPitch = "VELOCITY", yaw = "ANGULAR_VELOCITY", vertical = "VELOCITY",
            coordinateSystem = "GROUND", advanced = true,
        )

        override fun unavailableReason(): String? = null

        override fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            enableOnSuccess = onSuccess
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) = Unit

        override fun setAdvancedMode(enabled: Boolean) = Unit

        override fun sendAdvancedParam(
            pitch: Double, roll: Double, yaw: Double, verticalThrottle: Double,
        ): SendReport {
            sent += Sent(pitch, roll, yaw, verticalThrottle)
            return SendReport(modes, null)
        }

        override fun listenState(
            onState: (VirtualStickSnapshot) -> Unit,
            onAuthorityReason: (String) -> Unit,
        ) {
            this.onState = onState
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() = Unit
    }

    private class RecordingSink : MissionRunSink {
        val reached = mutableListOf<Int>()
        val cursors = mutableListOf<Int>()
        val finished = mutableListOf<Int>()
        val paused = mutableListOf<Pair<MissionPauseCause, Int>>()

        override fun onItemReached(seq: Int) { reached += seq }
        override fun onCursor(seq: Int) { cursors += seq }
        override fun onFinished(seq: Int) { finished += seq }
        override fun onPaused(cause: MissionPauseCause, cursorSeq: Int) { paused += cause to cursorSeq }
    }

    private class RecordedCmd(val setpoint: Setpoint?, val source: CommandSource?)

    private class Harness {
        var now = 1_000L
        var interlock = true
        var headingFollows = true

        /** Where the aircraft is: metres north/east of the route's origin, and metres above the datum. */
        var northM = 0.0
        var eastM = 0.0
        var relAlt: Double? = ALT
        var vn = 0.0
        var ve = 0.0
        var yawDeg: Double? = 0.0

        val port = FakeVirtualStickPort()
        val sink = RecordingSink()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        /** Every absolute pitch this bridge asked the camera for, in order. */
        val aimed = mutableListOf<Double>()

        val gimbal = object : ManoeuvreGimbal {
            override fun pitchRangeDeg(): ClosedFloatingPointRange<Double> = -90.0..30.0
            override fun aimPitch(pitchDeg: Double) { aimed += pitchDeg }
        }

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state() },
            announcer = Announcer(StatusTextSink { wire += it }),
            headingFollowsCourse = { headingFollows },
            manoeuvreGimbal = gimbal,
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) {
                    cmds += RecordedCmd(setpoint, source)
                }

                override fun event(code: String, message: String?, warn: Boolean) {
                    events += code to message
                }
            },
            nowMs = { now },
        )

        init {
            engine.attach()
            port.onRc!!(RcSticks(0, 0, 0, 0))
        }

        fun state(): AircraftState = AircraftState(
            fcConnected = true,
            latitude = latNorthOf(northM), longitude = lonEastOf(eastM),
            relativeAltitude = relAlt, takeoffAltitudeAmsl = DATUM,
            velocityNorth = vn, velocityEast = ve, velocityDown = 0.0,
            yawDeg = yawDeg, isFlying = true,
            ages = SampleAges.of(
                Signal.POSITION to 0L, Signal.ALTITUDE to 0L,
                Signal.VELOCITY to 0L, Signal.ATTITUDE to 0L,
            ),
        )

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun codes(): List<String> = events.map { it.first }

        fun messagesFor(code: String): List<String> =
            events.filter { it.first == code }.mapNotNull { it.second }

        fun place(north: Double = northM, east: Double = eastM, alt: Double? = relAlt) {
            northM = north
            eastM = east
            relAlt = alt
        }

        fun tickAlive(advanceMs: Long = 100) {
            now += advanceMs
            engine.onInbound("heartbeat")
            engine.tick(now)
        }

        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tickAlive(40)
        }

        fun flying(route: MissionRoute = roiRoute(), startIndex: Int = 0): Harness {
            assertEquals(Verdict.ACCEPTED, engine.missionStart(route, startIndex, false, sink))
            confirm()
            return this
        }

        /**
         * Sit on a step's coordinate until the cursor has advanced past it. A fly-through step needs
         * only the geometric crossing; the last one needs M3's settled predicate for its ticks.
         */
        fun reach(northM: Double, eastM: Double = 0.0, extra: Int = 2) {
            place(northM, eastM)
            vn = 0.0
            ve = 0.0
            repeat(RepositionGuidance.ARRIVE_TICKS + extra) { tickAlive() }
        }

        /** The last setpoint the recorder saw, in our own NED frame. */
        fun lastSetpoint(): Setpoint? = cmds.lastOrNull { it.setpoint != null }?.setpoint

        fun lastYaw(): Double = lastSetpoint()?.yawRateDegPerS ?: 0.0
    }

    // ------------------------------------------------------------------ the sequencing

    @Test
    fun `THE SEQUENCING - the ROI comes on when the cursor reaches the step the plan attached it to`() {
        val h = Harness().flying()
        h.tickAlive()

        // Leg one: the plan has said nothing about the camera yet. Nothing is aimed, nothing is
        // announced, nothing is on the record — an ROI item applied at Start would be a leg early.
        assertTrue("no ROI may be in force before the plan's item is reached", h.aimed.isEmpty())
        assertFalse(h.codes().contains(EventCode.ROI_ACCEPTED))
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_STARTED))

        // Reaching item 1 moves the cursor to item 3, and item 3 is the step the ROI belongs to.
        h.reach(30.0)
        assertEquals(listOf(SEQ_FIRST), h.sink.reached)
        assertEquals(listOf(SEQ_FIRST, SEQ_UNDER_ROI), h.sink.cursors)
        val accepted = h.messagesFor(EventCode.ROI_ACCEPTED).single()
        assertTrue("the record must name the plan item: $accepted", accepted.contains("seq=$SEQ_UNDER_ROI"))
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_STARTED))

        // And the camera is on it from the next tick, at the angle the geometry says.
        h.tickAlive()
        assertEquals(
            expectedPitch(30.0, 0.0, ALT, 0.0), h.aimed.first(), 0.05,
        )
    }

    @Test
    fun `and it comes off at the DO_SET_ROI_NONE - Ivan's -at wp8 it should stop-`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()
        val aimedUnderRoi = h.aimed.size
        assertTrue("the ROI leg must aim the camera", aimedUnderRoi > 0)

        // Item 3 reached: the cursor moves to item 5, whose sticky state is "no ROI" because the plan's
        // `DO_SET_ROI_NONE` sits between them. That is Ivan's "at wp8 it should stop".
        h.reach(60.0)
        assertEquals(listOf(SEQ_FIRST, SEQ_UNDER_ROI), h.sink.reached)
        val cleared = h.messagesFor(EventCode.ROI_CLEARED).single()
        assertTrue("the clear must name the item that asked for it: $cleared", cleared.contains("seq=$SEQ_AFTER_NONE"))
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_CLEARED))

        // The camera stops being driven — and is left exactly where it was, never swung to a default.
        val after = h.aimed.size
        repeat(20) { h.tickAlive() }
        assertEquals("nothing may aim the camera once the plan's ROI is cleared", after, h.aimed.size)
        assertEquals(0.0, h.lastYaw(), 1e-9)
    }

    @Test
    fun `an unchanged ROI is not re-taken at every corner`() {
        // Two consecutive steps under the same ROI. The plan said one thing once; re-taking it would
        // reset the camera's rate limiter and re-announce the target on every leg.
        val route = MissionRoute.of(
            planId = 1,
            steps = listOf(
                waypoint(SEQ_FIRST, 30.0, roi = roiAt()),
                waypoint(SEQ_UNDER_ROI, 60.0, roi = roiAt()),
                waypoint(SEQ_AFTER_NONE, 90.0, rest = true, roi = roiAt()),
            ),
        )
        val h = Harness().flying(route)
        h.tickAlive()
        h.reach(30.0)
        h.reach(60.0)
        assertEquals(
            "one ROI item, one acceptance",
            1, h.messagesFor(EventCode.ROI_ACCEPTED).size,
        )
        assertTrue("and no clear in the middle of it", h.messagesFor(EventCode.ROI_CLEARED).isEmpty())
    }

    @Test
    fun `a route with no ROI items points nothing and yaws for the course alone`() {
        // The control case, and the forward-compatibility property: every plan authored before
        // 2026-07-30 has no ROI items, and this feature must change none of them.
        val h = Harness().flying(plainRoute())
        h.tickAlive()
        h.reach(30.0)
        h.reach(60.0)
        assertTrue(h.aimed.isEmpty())
        assertFalse(h.codes().contains(EventCode.ROI_ACCEPTED))
        assertFalse(h.codes().contains(EventCode.ROI_CLEARED))
        assertFalse(h.texts().contains(GuidedStatusTexts.MISSION_ROI_ENDED))
    }

    // ------------------------------------------------------------------ one state, two doors

    @Test
    fun `THE DOORS - the plan's item and QGC's live click are ONE state, not two`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()
        assertTrue("the plan's ROI must be driving the camera", h.aimed.isNotEmpty())

        // A live `DO_SET_ROI_NONE` — QGC's ROI-off button — must be able to switch off an ROI the
        // *plan* set. If the mission had grown its own target this clear would peel off the wrong
        // layer and the camera would keep tracking.
        assertEquals(Verdict.ACCEPTED, h.engine.roi(RoiCommand.clearing()))
        val before = h.aimed.size
        repeat(20) { h.tickAlive() }
        assertEquals("one state: the operator's clear must stop the plan's ROI", before, h.aimed.size)
        assertEquals(0.0, h.lastYaw(), 1e-9)

        // ...and the live line carries no `seq`, which is how a reader tells the two doors apart.
        val cleared = h.messagesFor(EventCode.ROI_CLEARED)
        assertTrue("a live clear names no plan item: $cleared", cleared.none { it.contains("seq=") })
    }

    @Test
    fun `and the operator's own click replaces the plan's target in that same one state`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()
        val planAngle = h.aimed.last()

        // The live hand outranks the plan (`docs/m4-mission-transport.md` §6.3): a click 5 m from the
        // aircraft is a much steeper depression, so the camera visibly follows the operator.
        val live = RoiCommand.pointingAt(latNorthOf(35.0), lonEastOf(0.0), relativeAltM = 0.0)
        assertEquals(Verdict.ACCEPTED, h.engine.roi(live))
        h.tickAlive()
        val liveAngle = h.aimed.last()
        assertTrue(
            "the operator's target must be the one aimed at ($planAngle then $liveAngle)",
            liveAngle < planAngle - 10.0,
        )
        assertEquals(expectedPitch(30.0, 0.0, ALT, 0.0), planAngle, 0.05)
    }

    // ------------------------------------------------------------------ the height

    @Test
    fun `THE HEIGHT - a plan ROI's relative-frame z reaches the pointing solution unchanged`() {
        // big1's own ROI z is 0, so this is the case the fix exists for: a target 10 m above the takeoff
        // datum at 50 m of range is 11.3° of depression, not 21.8°. The store keeps frame 3's z (it is
        // *our* datum), `RoiCommand.pointingAt` carries it, and nothing on the plan path re-decides it.
        val h = Harness().flying(roiRoute(relAltM = 10.0))
        h.reach(30.0)
        h.tickAlive()
        val aimed = h.aimed.first()
        assertEquals(expectedPitch(30.0, 0.0, ALT, 10.0), aimed, 0.05)
        assertEquals(-11.3, aimed, 0.1)
        // And the operator is told the number that is in the solution, rather than the ground-level
        // assumption they would be owed if the height had been discarded.
        assertTrue(h.texts().contains(GuidedStatusTexts.roiTargetHeight(10.0)))
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_GROUND_LEVEL))
        // The record carries it too, so a post-flight reader can check the geometry.
        assertTrue(h.messagesFor(EventCode.ROI_ACCEPTED).single().contains("relAlt=10.0"))
    }

    @Test
    fun `and the camera keeps tracking as the leg carries the aircraft past the target`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()
        val atStart = h.aimed.last()

        // Fly the leg. The rate limiter is 500 ms and the deadband 1.5°, so this walks in 600 ms steps
        // and asserts the *solution* is being recomputed rather than commanded once and forgotten.
        for (north in listOf(40.0, 50.0)) {
            h.place(north)
            h.tickAlive(600)
        }
        val atEnd = h.aimed.last()
        assertTrue(
            "the depression must steepen as the aircraft closes on the target ($atStart then $atEnd)",
            atEnd < atStart - 3.0,
        )
        assertEquals(expectedPitch(50.0, 0.0, ALT, 0.0), atEnd, 0.05)
    }

    // ------------------------------------------------------------------ the nose

    @Test
    fun `THE YAW - a mission leg's nose is steered onto the ROI by the one yaw owner`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()

        // The aircraft is at 30 N / 0 E with the nose due north; the ROI bears 53° east of north, so
        // the nose must be swinging clockwise. On this airframe that is the *only* way the camera can
        // be brought onto the target at all — the gimbal cannot yaw (DJI #527).
        val yaw = h.lastYaw()
        assertTrue("the nose must turn toward the ROI, not sit at zero: $yaw", yaw > 1.0)

        // ...and it must be the shared law, evaluated on this leg's own numbers. A mission-local yaw
        // law would pass an "is it positive" test and fail this one.
        val (losNorth, losEast) = RepositionGuidance.nedMetres(
            latNorthOf(30.0), lonEastOf(0.0), latNorthOf(ROI_NORTH), lonEastOf(ROI_EAST),
        )
        val setpoint = h.lastSetpoint()!!
        assertEquals(
            RoiGuidance.yawRate(
                losNorthM = losNorth, losEastM = losEast, headingDeg = 0.0,
                commandedNorthMs = setpoint.north ?: 0.0, commandedEastMs = setpoint.east ?: 0.0,
            ),
            yaw, 0.01,
        )
        // The tolerance is the *record's*, not the law's: `Setpoint` quantises the velocities this
        // recomputation feeds back into the feed-forward term, so an exact comparison would be
        // asserting `Setpoint.DIGITS` rather than the yaw law. 0.01 °/s is four orders below the
        // envelope and far below any second law that could be written.
        // The flight path is untouched: an ROI is a modifier, so the leg is still flown due north.
        assertTrue("the leg must still fly its own course", (setpoint.north ?: 0.0) > 0.5)
        assertTrue(abs(setpoint.east ?: 0.0) < 0.05)
    }

    @Test
    fun `and the mission's own ROI never yaws an aircraft nobody of ours is flying`() {
        // The line §9.3 draws, on the mission path: with the plan's ROI in force but the run gone, the
        // camera is not driven and no yaw is generated — the aircraft is not ours to turn.
        val h = Harness().flying()
        h.reach(30.0)
        h.tickAlive()
        assertTrue(h.aimed.isNotEmpty())
        h.engine.abort(GuidedStickEngine.DisengageReason.STOPPED)
        val before = h.aimed.size
        repeat(10) { h.tickAlive() }
        assertEquals(before, h.aimed.size)
    }

    @Test
    fun `a stale heading on an ROI leg says so and commands no yaw, and the leg carries on`() {
        val h = Harness().flying()
        h.reach(30.0)
        h.yawDeg = null
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_NO_HEADING))
        assertEquals(0.0, h.lastYaw(), 1e-9)
        assertTrue("the flight path continues", (h.lastSetpoint()!!.north ?: 0.0) > 0.5)
    }

    // ------------------------------------------------------------------ the endings

    @Test
    fun `THE ENDING - a plan that never clears its ROI says so, and the camera is not swung anywhere`() {
        val h = Harness().flying(uncleared())
        h.reach(30.0)
        h.tickAlive()
        h.reach(60.0)
        val aimedInFlight = h.aimed.size
        assertTrue(aimedInFlight > 0)

        // The last item, reached and resting: the mission is complete and holding, in the air.
        h.reach(90.0, extra = 4)
        assertEquals(listOf(SEQ_AFTER_NONE), h.sink.finished)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_DONE_HOLDING))

        // The plan's ROI ends with the plan, by name, at warn severity — and the camera is left where
        // the plan left it. No recentring, no stowing, no nadir: a camera swung to a default is a lie
        // about where the target is.
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_ROI_ENDED))
        val cleared = h.messagesFor(EventCode.ROI_CLEARED).single()
        assertTrue("the record must say the plan ended with it set: $cleared", cleared.contains("plan ended"))
        // Named by the item that **set** it, which is what a reader needs to find the aiming's origin:
        // the ROI was unchanged across the last corner, so nothing re-took it there.
        assertTrue(cleared, cleared.contains("seq=$SEQ_UNDER_ROI"))
        val atEnd = h.aimed.size
        repeat(20) { h.tickAlive() }
        assertEquals("the camera must not be driven after the plan ends", atEnd, h.aimed.size)
        // The hold is a hold: zero setpoint, and no leftover ROI yaw in it.
        assertEquals(0.0, h.lastYaw(), 1e-9)
    }

    @Test
    fun `and a mission ending does NOT take away an ROI the operator set by hand`() {
        // Theirs outlives every manoeuvre — the rule `abort` has honoured since 2026-07-27, and the one
        // a mission ending must not quietly reverse.
        val h = Harness().flying(plainRoute())
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.roi(RoiCommand.pointingAt(latNorthOf(ROI_NORTH), lonEastOf(ROI_EAST), 0.0)),
        )
        h.tickAlive()
        h.reach(30.0)
        h.reach(60.0)
        h.reach(90.0, extra = 4)
        assertEquals(listOf(SEQ_AFTER_NONE), h.sink.finished)
        assertFalse(
            "the plan never set this ROI, so the plan's ending must not end it",
            h.texts().contains(GuidedStatusTexts.MISSION_ROI_ENDED),
        )
        val before = h.aimed.size
        // Far enough to clear the camera's 1.5° deadband — this is asserting that the ROI is still
        // *driving*, not that a limiter happens to be due.
        h.place(60.0)
        h.tickAlive(600)
        assertTrue("the operator's ROI keeps the camera", h.aimed.size > before)
    }

    @Test
    fun `a pause ends the plan's ROI, and the resume takes it again from the cursor's own step`() {
        val h = Harness().flying(uncleared())
        h.reach(30.0)
        h.tickAlive()
        assertTrue(h.aimed.isNotEmpty())

        // §6.2's Pause: the plan pauses resumably at the item it was on. Its ROI is dropped rather than
        // remembered — a paused plan's target driving the camera is a camera nobody is commanding.
        assertEquals(Verdict.ACCEPTED, h.engine.pause())
        assertEquals(MissionPauseCause.GCS_PAUSE, h.sink.paused.single().first)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_ROI_ENDED))
        val paused = h.aimed.size
        repeat(10) { h.tickAlive() }
        assertEquals(paused, h.aimed.size)

        // The resume needs no machinery of its own: the cursor's step still knows which ROI the plan
        // had in force there, so Start re-takes it.
        val route = uncleared()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.missionStart(route, startIndex = 1, rejoining = true, sink = h.sink),
        )
        h.tickAlive()
        assertEquals(2, h.messagesFor(EventCode.ROI_ACCEPTED).size)
        assertTrue(h.aimed.size > paused)
    }

    @Test
    fun `an abort DROPS the plan's ROI rather than remembering it for the next engagement`() {
        // An operator's ROI is *suspended* by an abort and re-acquired when an engagement is next
        // confirmed (§9.5). A plan's must not be: the plan is gone, and re-acquiring its target after a
        // handback would be a dead plan's camera command arriving on somebody else's aircraft.
        val h = Harness().flying(uncleared())
        h.reach(30.0)
        h.tickAlive()
        assertTrue(h.aimed.isNotEmpty())

        h.engine.abort(GuidedStickEngine.DisengageReason.RC_STICKS)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_ROI_ENDED))
        assertNull(h.engine.missionCursorSeq())

        // A fresh engagement, with nothing that mentions an ROI: the camera must stay put.
        val before = h.aimed.size
        assertEquals(Verdict.ACCEPTED, h.engine.missionStart(plainRoute(), 0, false, h.sink))
        h.confirm()
        repeat(20) { h.tickAlive() }
        assertEquals("no dead plan's ROI may come back", before, h.aimed.size)
    }

    // ------------------------------------------------------------------ the record

    @Test
    fun `THE RECORD - a plan's aiming is tied to the plan item, a hand's is not`() {
        val h = Harness().flying()
        h.reach(30.0)
        // The plan's set names its item; the geometry is on the line too, so a reader can check the
        // pointing solution against `mission_current` without the video.
        val accepted = h.messagesFor(EventCode.ROI_ACCEPTED).single()
        assertTrue(accepted, accepted.contains("lat=${"%.7f".format(latNorthOf(ROI_NORTH))}"))
        assertTrue(accepted, accepted.contains("relAlt=0.0"))
        assertTrue(accepted, accepted.endsWith("seq=$SEQ_UNDER_ROI"))

        // A live command on the same flight is distinguishable by the absence of that tail.
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.roi(RoiCommand.pointingAt(latNorthOf(10.0), lonEastOf(10.0), 0.0)),
        )
        assertFalse(h.messagesFor(EventCode.ROI_ACCEPTED).last().contains("seq="))
    }

    @Test
    fun `a plan ROI this bridge cannot point at is refused BY NAME and the route flies on`() {
        // No camera attached: the live door refuses an ROI it cannot honour rather than accepting
        // politely, and the plan's door inherits that refusal — with the item named on the record. The
        // mission is *not* stopped: an ROI is a modifier, so failing to take one is not a reason to stop
        // flying a route the operator authored.
        val h = Harness()
        val engine = GuidedStickEngine(
            port = h.port,
            interlockEnabled = { h.interlock },
            aircraftState = { h.state() },
            announcer = Announcer(StatusTextSink { h.wire += it }),
            manoeuvreGimbal = null,
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) = Unit

                override fun event(code: String, message: String?, warn: Boolean) {
                    h.events += code to message
                }
            },
            nowMs = { h.now },
        )
        engine.attach()
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        assertEquals(Verdict.ACCEPTED, engine.missionStart(roiRoute(), 0, false, h.sink))
        h.port.enableOnSuccess?.invoke()
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
        h.now += 40
        engine.onInbound("heartbeat")
        engine.tick(h.now)
        h.place(30.0)
        repeat(RepositionGuidance.ARRIVE_TICKS + 2) {
            h.now += 100
            engine.onInbound("heartbeat")
            engine.tick(h.now)
        }
        val denied = h.messagesFor(EventCode.ROI_DENIED).single()
        assertEquals(GuidedStatusTexts.REASON_ROI_NO_GIMBAL + " seq=$SEQ_UNDER_ROI", denied)
        assertTrue(
            h.texts().contains(
                StatusTexts.clamp(GuidedStatusTexts.roiRefused(GuidedStatusTexts.REASON_ROI_NO_GIMBAL)),
            )
        )
        assertNotNull("the mission must keep flying", engine.missionCursorSeq())
        assertEquals(SEQ_UNDER_ROI, engine.missionCursorSeq())
    }
}
