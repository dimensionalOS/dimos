package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
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
import io.dronefleet.mavlink.common.ManualControl
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt

/**
 * M4 inside the engine — a route flown as a **fourth setpoint source** in the one 10 Hz tick: the
 * two completion tests, the fly-through envelope, the cursor's discipline, the takeoff seam, the
 * heartbeat claim's third conjunct, and every abort gesture applied mid-mission.
 *
 * Same protocol as `GuidedOrbitTest` and `GuidedRepositionTest`: fake port, hand-cranked clock, no
 * aircraft. The pure arithmetic is pinned next door in `MissionGuidanceTest`; the lifecycle and the
 * launch gate in `MissionLifecycleTest` and `MissionLaunchTest`. **This file is about what the
 * engine does with them.**
 *
 * Written to fail loudly for the M4 landmines:
 *
 *  - **the cursor advancing on a stale fix**, which is a dead-reckoned mission and the one thing
 *    §3.4 forbids outright.
 *  - **the final waypoint's arrival test replaced** by the geometric one, so a fly-through at cruise
 *    would declare the mission complete two metres before it stopped.
 *  - **`e_stop` reading the current leg** (fly-through broken) or **infinity** (the envelope made
 *    dead code) at the engine level, not only in the arithmetic.
 *  - **a mission surviving an abort** — any abort — or, worse, surviving one and lying in ambush
 *    for the next engagement.
 *  - **`modeClaim`'s third conjunct latching**: a claim that stays true after the engine is no
 *    longer engaged is the echo `PLAN.md` forbids, and it is the mutation this design most needs to
 *    survive.
 *  - the rejoin leg flown as a **pass-through** rather than as a resting one.
 *  - **`MISSION_ITEM_REACHED` emitted twice**, or not at all, for one item.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, the **whole**
 * suite run, confirmed red and reverted. Counts are failing tests across all 1802 — **measured, not
 * estimated**.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a stale fix flown on (the cursor advances blind) | 2 |
 *  | the resting waypoint uses the geometric test instead of `RepositionGuidance.settled` | 3 |
 *  | `restAhead` forced to 0 inside the tick (`e_stop` = the current leg) | 1 |
 *  | `restAhead` forced to `1e9` inside the tick (`e_stop` = infinity) | 1 |
 *  | `missionFlying` drops its setpoint-recency conjunct | 1 |
 *  | the mission survives an abort | 2 |
 *  | `missionFlying` drops its `ENGAGED` conjunct (alone) | **0 — alive on purpose** |
 *  | the heartbeat claim stamped even when the send failed | **0 — alive on purpose** |
 *  | **both** the `ENGAGED` conjunct and the abort's clear removed | 3 |
 *  | `rejoining` ignored (a rejoin flown as a pass-through) | 1 |
 *  | the cursor advances on every tick | 4 |
 *  | the per-leg timeout removed | 1 |
 *  | the whole-mission cap removed | 1 |
 *  | the takeoff asked for on every tick rather than once | 1 |
 *  | the takeoff's cursor advance no longer gated on `isFlying` | 2 |
 *  | an ROI no longer outranks heading-follows-course | 4 |
 *  | heading-follows-course ignores its flag | 4 |
 *
 * ### The takeoff's two phases, measured 2026-07-27 after the merge with M2.5
 *
 * Same protocol, whole suite (1899) against each. These cover the seam the executor left open and
 * the two-phase takeoff closed.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the handback is `isFlying` alone again (the executor's own placeholder) | 7 |
 *  | the mode-seize suppression removed (DJI's takeoff read as a seizure) | 7 |
 *  | the watch re-armed on every tick (the conjuncts never accumulate) | 7 |
 *  | the climb's arrival test dropped — the cursor moves as phase two begins | 6 |
 *  | phase two keeps the old seize grace instead of restamping it | 2 |
 *  | the climb flies to the item's coordinate too (a takeoff that translates) | 1 |
 *  | the climb's ceiling gate removed | 1 |
 *  | the climb inherits phase one's flat 30 s bound | 1 |
 *  | the seize suppression covers phase two as well | 1 |
 *  | the climb's ROI/heading pass dropped | 1 |
 *  | a mission carries no yaw authority (the pitch-only sentence returns) | 1 |
 *
 * **Two of these started at zero and were findings, not formalities.** *The climb flies to the
 * item's coordinate* scored 0 because a takeoff item's coordinate **is** where the aircraft is —
 * `MissionLaunch` resolves it to home — so in still air the horizontal term is zero either way and
 * every test passed. It is visible only after drift, which is what wind does during a 5 s hop, and
 * `A TAKEOFF NEVER TRANSLATES` now places the aircraft 12 m off before the climb. *The climb's
 * ceiling gate* scored 0 for the neighbouring reason: the launch check refuses an item above the
 * ceiling, so no route reaching the tick could exercise the gate — the test therefore hands the
 * engine a route the launch check would have refused, because this layer must not be the one that
 * trusts a check upstream of it.
 *
 * **And the harness's own timing was a finding.** *Phase two keeps the old seize grace* scored 0
 * until `handedBack` used the **measured** gaps between DJI's mode changes (5.161 s in total)
 * rather than one tick each: with the whole takeoff compressed into 500 ms the engagement's grace
 * had not expired, so a test that should have caught a real-air failure passed. The spacing is now
 * the recorded one, and the mutation dies twice over.
 *
 * ### The three results worth reading rather than counting
 *
 * **`restAhead` forced to 0 scored zero on the first sweep**, and that was a finding rather than a
 * formality. Neither existing test could see it: the envelope property cannot, because braking
 * *early* is inside the bound it checks, and the corner test cannot, because at 3 m out the P-term
 * binds either way. It is a **behaviour** mutant — the aircraft crawls at every waypoint, which is
 * exactly what Ivan ruled out — so it needed a behaviour test, which is
 * `THE BRAKE LOOKS PAST THE CURRENT WAYPOINT`. It now dies. §10.1 predicted this split ("should fail
 * a *behaviour* test, not the safety test") and the first sweep proved the prediction.
 *
 * **`missionFlying` dropping its `ENGAGED` conjunct is masked exactly, by the abort that clears the
 * route.** Every path that leaves `ENGAGED` while a mission is running goes through `abort`, which
 * nulls `mission` on the same call — so `mission != null` already implies engaged, and the explicit
 * conjunct is defence in depth. It is kept, because it is the layer that will still be right the day
 * a wind-down keeps a route alive, and because it is the sentence a reader of `modeClaim` needs to
 * see. Removing **both** layers is caught by 3 tests, which is what shows the property is tested.
 *
 * **The claim's `report.error == null` conjunct is masked the same way**: a send that throws aborts
 * inside the same `performSend` call, so the mission is gone before anything could read a stamp that
 * should not have been written. Alive, kept, and recorded.
 *
 * ## The camera at the mission's takeoff — measured 2026-07-30, the landing16 campaign
 *
 * Whole suite per mutant, **2644 tests**, `test-results` deleted first, confirmed red, reverted.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the mission takeoff's nadir aim dropped (`armWatch(aimCameraNadir = false)`) | 1 |
 *
 * **A 1 that cost a whole autonomous flight, and the number is right rather than thin.** Until
 * 2026-07-30 the phone and QGC takeoff doors aimed the camera down and the mission's `NAV_TAKEOFF`
 * did not — nothing *disagreed*, one door was simply silent, which is the two-places-for-one-
 * property failure with one place missing and the hardest version to see. So there was exactly one
 * line to add and there is exactly one test that can see it; the count is not a measure of how much
 * rests on it. `datasets/landing16/20260730-161329.001.jsonl` is what rests on it: the gimbal held
 * pitch 0° from takeoff at t=54.2, the detector's acquisition pass saw the horizon, nothing latched,
 * the climb closed the one-way band at t=62.0, and the flight's own precision landing was refused
 * `NO_TAG_LATCHED` at t=188.8 while hovering over the pad. **Zero `tag` lines in 241 seconds.**
 *
 * The mutant's shape matters as much as its count: it flips the flag on the *seam*
 * ([TakeoffClimb.armWatch]'s pending intention) rather than deleting the call, because that is the
 * regression a future edit would actually make — the aim itself is now shared with the command
 * doors ([GuidedStickEngine] `aimTakeoffCameraNadir`, one implementation for all three), so what can
 * still be lost is the *flag*, not the code.
 */
class GuidedMissionTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7
        const val DATUM = 100.0
        const val ALT = 20.0

        fun latNorthOf(metres: Double): Double = LAT + metres / Geo.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        fun waypoint(seq: Int, northM: Double, eastM: Double = 0.0, rest: Boolean = false) =
            MissionStep(
                seq = seq,
                kind = MissionStepKind.WAYPOINT,
                latDeg = latNorthOf(northM),
                lonDeg = lonEastOf(eastM),
                relAltM = ALT,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
                rest = rest,
            )

        /** Three waypoints due north at 30 / 60 / 90 m; the last one rests. */
        fun threeLegs(): MissionRoute = MissionRoute.of(
            planId = 1,
            steps = listOf(
                waypoint(0, 30.0),
                waypoint(1, 60.0),
                waypoint(2, 90.0, rest = true),
            ),
        )
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
        var enableCalls = 0
        var disableCalls = 0
        var enableOnSuccess: (() -> Unit)? = null
        var throwOnSend: Throwable? = null

        data class Sent(val pitch: Double, val roll: Double, val yaw: Double, val verticalThrottle: Double)

        val sent = mutableListOf<Sent>()
        var onState: ((VirtualStickSnapshot) -> Unit)? = null
        var onReason: ((String) -> Unit)? = null
        var onRc: ((RcSticks) -> Unit)? = null

        val modes = StickModes(
            rollPitch = "VELOCITY", yaw = "ANGULAR_VELOCITY", vertical = "VELOCITY",
            coordinateSystem = "GROUND", advanced = true,
        )

        override fun unavailableReason(): String? = unavailable

        override fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            enableCalls++
            enableOnSuccess = onSuccess
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            disableCalls++
        }

        override fun setAdvancedMode(enabled: Boolean) = Unit

        override fun sendAdvancedParam(
            pitch: Double, roll: Double, yaw: Double, verticalThrottle: Double,
        ): SendReport {
            throwOnSend?.let { throw it }
            sent += Sent(pitch, roll, yaw, verticalThrottle)
            return SendReport(modes, null)
        }

        override fun listenState(
            onState: (VirtualStickSnapshot) -> Unit,
            onAuthorityReason: (String) -> Unit,
        ) {
            this.onState = onState
            this.onReason = onAuthorityReason
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() = Unit
    }

    /** Everything the engine told the lifecycle half, in order. */
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

    private class FakeTakeoff : MissionTakeoff {
        var calls = 0
        var failWith: String? = null
        override fun startTakeoff(onFailure: (String) -> Unit) {
            calls++
            failWith?.let(onFailure)
        }
    }

    private class RecordedCmd(val setpoint: Setpoint?, val source: CommandSource?)

    /** Enough of a camera for an ROI to be accepted; the aim itself is `GuidedRoiTest`'s business. */
    private class FakeGimbal : ManoeuvreGimbal {
        val aimed = mutableListOf<Double>()
        override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? = null
        override fun aimPitch(pitchDeg: Double) { aimed += pitchDeg }
    }

    private class Harness(withTakeoff: Boolean = false) {
        var now = 1_000L
        var interlock = true
        var headingFollows = true
        var state = stateAt()
        val port = FakeVirtualStickPort()
        val takeoff = FakeTakeoff()
        val gimbal = FakeGimbal()
        val sink = RecordingSink()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            headingFollowsCourse = { headingFollows },
            missionTakeoff = if (withTakeoff) takeoff else null,
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

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        fun place(
            northM: Double = 0.0, eastM: Double = 0.0, relAlt: Double? = ALT,
            vn: Double = 0.0, ve: Double = 0.0,
            positionAge: Long = 0L, velocityAge: Long = 0L, attitudeAge: Long = 0L,
            yawDeg: Double? = 0.0, isFlying: Boolean? = true, flightMode: String? = null,
            fcConnected: Boolean = true,
        ) {
            state = stateAt(
                latNorthOf(northM), lonEastOf(eastM), relAlt, vn, ve,
                positionAge, velocityAge, attitudeAge, yawDeg, isFlying, flightMode, fcConnected,
            )
        }

        fun start(
            route: MissionRoute = threeLegs(),
            startIndex: Int = 0,
            rejoining: Boolean = false,
        ): Verdict = engine.missionStart(route, startIndex, rejoining, sink)

        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tick(40)
        }

        fun tickAlive(advanceMs: Long = 100) {
            now += advanceMs
            engine.onInbound("heartbeat")
            engine.tick(now)
        }

        fun tick(advanceMs: Long = 0) {
            now += advanceMs
            engine.tick(now)
        }

        fun frame(x: Int = 0, y: Int = 0, z: Int = 500, r: Int = 0) {
            engine.onInbound(ManualControl.builder().target(1).x(x).y(y).z(z).r(r).buttons(0).build(), null)
        }

        /**
         * Start a takeoff plan and walk DJI's **measured** takeoff trail all the way to the
         * handback, leaving the run in phase two at the height DJI actually stops at (1.0 m —
         * `docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md`, §2).
         *
         * The trail is the one both 2026-07-27 records show, in order:
         * `GPS_ATTI → MOTOR_START → AUTO_TAKE_OFF`, `isFlying` true *inside* `AUTO_TAKE_OFF`, and
         * only then the mode returning to `GPS_ATTI`.
         */
        fun handedBack(route: MissionRoute): Harness {
            place(isFlying = false, flightMode = "GPS_ATTI", relAlt = 0.0)
            assertEquals(Verdict.ACCEPTED, start(route))
            confirm()
            tickAlive()
            // The gaps are the measured ones, not one tick each: 5.161 s from arm to handback, of
            // which `MOTOR_START → AUTO_TAKE_OFF` is 1.6 s and `isFlying` arrives 1.4 s after that
            // with 1.9 s still to run (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md`
            // §1). The spacing matters: a handback that arrives inside the *engagement's* own grace
            // windows would let this harness pass tests a real takeoff would fail.
            place(isFlying = false, flightMode = "MOTOR_START", relAlt = 0.0)
            tickAlive(300)
            place(isFlying = false, flightMode = "AUTO_TAKE_OFF", relAlt = 0.3)
            tickAlive(1_600)
            place(isFlying = true, flightMode = "AUTO_TAKE_OFF", relAlt = 0.8)
            tickAlive(1_400)
            place(isFlying = true, flightMode = "GPS_ATTI", relAlt = 1.0)
            tickAlive(1_900)
            return this
        }

        /** Accept the default route and get DJI's confirmation, aircraft at the origin. */
        fun flying(route: MissionRoute = threeLegs()): Harness {
            assertEquals(Verdict.ACCEPTED, start(route))
            confirm()
            return this
        }

        companion object {
            fun stateAt(
                latDeg: Double = LAT, lonDeg: Double = LON, relAlt: Double? = ALT,
                vn: Double = 0.0, ve: Double = 0.0,
                positionAge: Long = 0L, velocityAge: Long = 0L, attitudeAge: Long = 0L,
                yawDeg: Double? = 0.0, isFlying: Boolean? = true, flightMode: String? = null,
                fcConnected: Boolean = true,
            ) = AircraftState(
                fcConnected = fcConnected,
                latitude = latDeg, longitude = lonDeg,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = DATUM,
                velocityNorth = vn, velocityEast = ve, velocityDown = 0.0,
                yawDeg = yawDeg, isFlying = isFlying, flightMode = flightMode,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to 0L,
                    Signal.VELOCITY to velocityAge,
                    Signal.ATTITUDE to attitudeAge,
                ),
            )
        }
    }

    // ------------------------------------------------------------------ accept and engage

    @Test
    fun `a route while idle is ACCEPTED, engages, and flies only after DJI confirms`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.start())
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_STARTED))
        // The cursor is published immediately: a ground station must not have to infer it.
        assertEquals(listOf(0), h.sink.cursors)
        // Nothing may flow before DJI's own state confirms authority.
        h.tickAlive()
        assertEquals(0, h.port.sent.size)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `a route reuses an existing engagement - one enable, one owner`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        assertEquals(1, h.port.enableCalls)
        assertEquals(Verdict.ACCEPTED, h.start())
        assertEquals(1, h.port.enableCalls)
    }

    @Test
    fun `a route with the interlock off is refused and nothing engages`() {
        val h = Harness()
        h.interlock = false
        assertEquals(Verdict.DENIED, h.start())
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
    }

    @Test
    fun `a route is refused with no RC stick feed, because abort gesture 1 would be blind`() {
        val h = Harness()
        h.port.onRc!!(RcSticks(null, null, null, null))
        assertEquals(Verdict.DENIED, h.start())
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    @Test
    fun `a route is refused on a stale fix - nothing is flown from a cached position`() {
        val h = Harness()
        h.place(positionAge = 5_000L)
        assertEquals(Verdict.DENIED, h.start())
    }

    // ------------------------------------------------------------------ the two completion tests

    @Test
    fun `THE FLY-THROUGH - an intermediate waypoint completes on the geometric test, at speed`() {
        val h = Harness().flying()
        // 2 m short of waypoint 0, doing 3 m/s: inside the switch radius, nowhere near settled.
        h.place(northM = 28.0, vn = 3.0)
        h.tickAlive()
        assertEquals(listOf(0), h.sink.reached)
        assertEquals(listOf(0, 1), h.sink.cursors)
        // ...and it did **not** need five consecutive ticks or a settled speed to do it.
        assertTrue(h.sink.finished.isEmpty())
    }

    @Test
    fun `THE RESTING WAYPOINT - the last one uses M3's arrival test, so a fly-through cannot finish it`() {
        val h = Harness().flying()
        // Teleport past the first two, then arrive at the last one at cruise speed.
        h.place(northM = 30.0); h.tickAlive()
        h.place(northM = 60.0); h.tickAlive()
        assertEquals(listOf(0, 1), h.sink.reached)
        h.place(northM = 89.0, vn = GuidedEnvelope.HORIZONTAL_MAX_MS)
        repeat(RepositionGuidance.ARRIVE_TICKS * 3) { h.tickAlive() }
        assertTrue("a fly-through completed the resting waypoint", h.sink.finished.isEmpty())
        // Come to rest, and it completes — after the required consecutive ticks and not before.
        h.place(northM = 89.5, vn = 0.1)
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        assertTrue(h.sink.finished.isEmpty())
        h.tickAlive()
        assertEquals(listOf(2), h.sink.finished)
        assertEquals(listOf(0, 1, 2), h.sink.reached)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_DONE_HOLDING))
    }

    @Test
    fun `a dead velocity feed withholds arrival at the last waypoint rather than reading zero`() {
        val h = Harness().flying()
        h.place(northM = 30.0); h.tickAlive()
        h.place(northM = 60.0); h.tickAlive()
        // Sitting on the last waypoint, but the velocity feed cannot vouch for a speed. On this
        // airframe a hover and a dead feed are the same bytes, so arrival is withheld.
        h.place(northM = 90.0, velocityAge = 5_000L)
        repeat(RepositionGuidance.ARRIVE_TICKS * 3) { h.tickAlive() }
        assertTrue(h.sink.finished.isEmpty())
    }

    @Test
    fun `THE HALF-PLANE - a waypoint the aircraft cannot quite reach is passed once it is behind`() {
        val h = Harness().flying()
        // 4 m past waypoint 0 and 4 m to the east: outside the 3 m radius, inside the corridor, and
        // behind the plane. Without the half-plane term this leg hangs until its timeout.
        h.place(northM = 34.0, eastM = 4.0)
        h.tickAlive()
        assertEquals(listOf(0), h.sink.reached)
    }

    // ------------------------------------------------------------------ cursor discipline

    @Test
    fun `THE CURSOR NEVER ADVANCES ON A STALE FIX, and holds zero on that tick`() {
        val h = Harness().flying()
        // Sitting exactly on waypoint 0 — the geometric test would fire instantly — but the fix is
        // stale. Absence is not zero, and a dead-reckoned cursor is the thing §3.4 forbids.
        h.place(northM = 30.0, positionAge = 5_000L)
        repeat(20) { h.tickAlive() }
        assertTrue("the cursor advanced on a stale fix", h.sink.reached.isEmpty())
        assertEquals(0.0, h.port.sent.last().pitch, 1e-12)
        assertEquals(0.0, h.port.sent.last().roll, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_POSITION_HOLD))
        // A fresh fix on the very next tick advances it, so the withholding is the *fix's* doing.
        h.place(northM = 30.0)
        h.tickAlive()
        assertEquals(listOf(0), h.sink.reached)
    }

    @Test
    fun `MISSION_ITEM_REACHED is emitted exactly once per item, on the advancing edge`() {
        val h = Harness().flying()
        h.place(northM = 30.0)
        repeat(10) { h.tickAlive() }
        // Ten ticks sitting on the *old* waypoint's position: one reached, and the cursor has moved
        // on, so the following nine ticks are flying leg 1 rather than re-reaching leg 0.
        assertEquals(listOf(0), h.sink.reached)
        assertEquals(listOf(0, 1), h.sink.cursors)
    }

    @Test
    fun `no STATUSTEXT is emitted per leg - progress lives in MISSION_CURRENT`() {
        // A five-waypoint mission would otherwise produce five announcements on a 50-byte channel
        // at severity ERROR that exists for things the operator must act on.
        val h = Harness().flying()
        val before = h.texts().size
        h.place(northM = 30.0); h.tickAlive()
        h.place(northM = 60.0); h.tickAlive()
        assertEquals(before, h.texts().size)
    }

    // ------------------------------------------------------------------ the envelope

    @Test
    fun `THE ENVELOPE HOLDS AT THE ENGINE - commanded speed never exceeds the bound to the next rest`() {
        // The arithmetic's property test proves the formula; this proves the *engine* feeds it the
        // right numbers — the mutation "restAheadM forced to 1e9 in the tick" dies here and not
        // there.
        val h = Harness().flying()
        var north = 0.0
        repeat(1_500) {
            h.place(northM = north)
            h.tickAlive()
            val sent = h.port.sent.lastOrNull() ?: return@repeat
            // The recorder carries the setpoint in our own NED frame, which is what the bound is
            // stated in; the port's axes are DJI's.
            val setpoint = h.cmds.last().setpoint ?: return@repeat
            val speed = hypot(setpoint.north ?: 0.0, setpoint.east ?: 0.0)
            // The true distance to the next resting point: the last waypoint is at 90 m.
            val stop = (90.0 - north).coerceAtLeast(0.0)
            val bound = minOf(
                GuidedEnvelope.HORIZONTAL_MAX_MS,
                sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * stop),
            )
            assertTrue(
                "at $north m: |v| $speed exceeded $bound",
                speed <= bound + 1e-6,
            )
            north += (setpoint.north ?: 0.0) * 0.1
            assertTrue(sent.pitch.isFinite())
        }
        assertEquals(listOf(2), h.sink.finished)
    }

    @Test
    fun `THE BRAKE LOOKS PAST THE CURRENT WAYPOINT - full speed 8 m out with a long plan left`() {
        // The mutation this exists for: `restAhead` forced to 0 inside the tick, i.e. `e_stop`
        // reading the current leg. Added after that mutant scored **zero** on the first sweep — the
        // envelope property could not see it (braking early is inside the bound) and the corner test
        // could not either (1.5 m/s either way at 3 m out). It is a *behaviour* mutant, so it needs
        // a behaviour test, exactly as §10.1 predicted.
        //
        // 8 m from waypoint 0 with 60 m of plan beyond it: the true stop is 68 m away, so the brake
        // is nowhere near binding and the aircraft flies at the full envelope cap. With `e_stop`
        // reading the current leg alone the brake gives sqrt(2·0.5·8) = 2.83 m/s and the aircraft
        // crawls at every waypoint — which is precisely the behaviour Ivan ruled out.
        val h = Harness().flying()
        h.place(northM = 22.0)
        h.tickAlive()
        val setpoint = h.cmds.last().setpoint!!
        assertEquals(
            GuidedEnvelope.HORIZONTAL_MAX_MS,
            hypot(setpoint.north ?: 0.0, setpoint.east ?: 0.0),
            1e-9,
        )
    }

    @Test
    fun `THE CORNER IS TAKEN AT SPEED - no dip in the last six metres of a fly-through leg`() {
        // **The regression this exists for was flown, not imagined.** Until 2026-07-27 the
        // proportional term read the current leg, so the commanded speed decayed from the envelope
        // to 0.5 × R_SWITCH_M = 1.45 m/s over the last six metres of *every* intermediate leg and
        // snapped back the instant the cursor advanced. The first mission ever flown measured
        // exactly that at three consecutive waypoints — 1.46, 1.47, 1.44 m/s — an aircraft braking
        // into every corner for a stop the mission never asked for
        // (`docs/measurements/2026-07-27-first-mission-flown.md` §7.1).
        //
        // Walked in rather than sampled at one point, because the old law's signature is a *decay*:
        // a single assertion at 3 m would pass against a law that had already started slowing at 6.
        val h = Harness().flying()
        for (remaining in listOf(8.0, 6.0, 5.0, 4.0, 3.2, 3.05)) {
            h.place(northM = 30.0 - remaining)
            h.tickAlive()
            val setpoint = h.cmds.last().setpoint!!
            assertEquals(
                "the corner dipped $remaining m out",
                GuidedEnvelope.HORIZONTAL_MAX_MS,
                hypot(setpoint.north ?: 0.0, setpoint.east ?: 0.0),
                1e-9,
            )
        }
        // And the last leg still slows, because that one really does end in a stop — started at the
        // resting waypoint directly, since reaching it the long way is a different test's job.
        val last = Harness()
        assertEquals(Verdict.ACCEPTED, last.start(threeLegs(), startIndex = 2))
        last.confirm()
        last.place(northM = 86.0) // 4 m short of the last waypoint, which rests
        last.tickAlive()
        val approach = last.cmds.last().setpoint!!
        assertTrue(
            "the final approach did not slow",
            hypot(approach.north ?: 0.0, approach.east ?: 0.0) < GuidedEnvelope.HORIZONTAL_MAX_MS,
        )
    }

    @Test
    fun `flying through, the aircraft does not brake to zero at an intermediate waypoint`() {
        // The behaviour the fly-through exists for, as distinct from the safety property above: at
        // the switch point the aircraft is still moving at the P-term's speed rather than stopped.
        val h = Harness().flying()
        h.place(northM = 27.0)
        h.tickAlive()
        val setpoint = h.cmds.last().setpoint!!
        assertTrue(
            "braked to a crawl 3 m out",
            hypot(setpoint.north ?: 0.0, setpoint.east ?: 0.0) > 1.0,
        )
    }

    // ------------------------------------------------------------------ the abort table

    @Test
    fun `the interlock going off mid-mission pauses the plan, blocked, at its cursor`() {
        val h = Harness().flying()
        h.place(northM = 30.0); h.tickAlive()
        h.interlock = false
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(listOf(MissionPauseCause.INTERLOCK_OFF to 1), h.sink.paused)
    }

    @Test
    fun `an RC stick grab pauses the plan, resumably, at its cursor`() {
        val h = Harness().flying()
        h.port.onRc!!(RcSticks(400, 0, 0, 0))
        h.now += GuidedStickEngine.RC_ABORT_SUSTAIN_MS
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(listOf(MissionPauseCause.RC_STICK_GRAB to 0), h.sink.paused)
    }

    @Test
    fun `DJI's own authority change pauses the plan with DJI's reason`() {
        val h = Harness().flying()
        h.port.onReason!!(MissionAbortPolicy.REASON_RC_GO_HOME)
        assertEquals(listOf(MissionPauseCause.RC_RETURN_HOME to 0), h.sink.paused)
    }

    @Test
    fun `QGC link loss pauses the plan, resumably`() {
        val h = Harness().flying()
        h.tick(GuidedEnvelope.LINK_LOST_MS + 100)
        assertEquals(listOf(MissionPauseCause.QGC_LINK_LOSS to 0), h.sink.paused)
    }

    @Test
    fun `a position feed that stays dead pauses the plan, resumably`() {
        val h = Harness().flying()
        h.place(positionAge = 5_000L)
        h.tickAlive()
        assertTrue(h.sink.paused.isEmpty()) // held at zero first, for the full window
        h.now += RepositionGuidance.POSITION_LOST_MS
        h.tickAlive()
        assertEquals(listOf(MissionPauseCause.POSITION_LOST to 0), h.sink.paused)
    }

    @Test
    fun `a leg that does not finish in its own time pauses the plan, blocked`() {
        val h = Harness().flying()
        // The first leg is 30 m, so its timeout is 30 s + 30 s. Sit still past it.
        val timeout = MissionGuidance.legTimeoutMs(30.0)
        var elapsed = 0L
        while (elapsed <= timeout + 200) {
            h.tickAlive()
            elapsed += 100
        }
        assertEquals(listOf(MissionPauseCause.LEG_TIMEOUT to 0), h.sink.paused)
    }

    @Test
    fun `a mission that outruns its whole-flight cap pauses, blocked`() {
        val h = Harness().flying()
        // Keep completing legs so no *leg* times out, then let the whole-mission clock run out.
        h.place(northM = 30.0); h.tickAlive()
        h.now += MissionGuidance.MISSION_MAX_S * 1_000L
        h.place(northM = 30.0)
        h.tickAlive()
        assertEquals(listOf(MissionPauseCause.MISSION_TIMEOUT to 1), h.sink.paused)
    }

    @Test
    fun `a deliberate GCS stick deflection pauses the plan and passthrough takes the aircraft`() {
        val h = Harness().flying()
        h.frame() // the stream at rest, so a deflection can be read as intent
        h.now += 40
        h.frame(x = 500)
        assertEquals(listOf(MissionPauseCause.GCS_STICK_DEFLECTION to 0), h.sink.paused)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_STICKS))
        // Still engaged: the operator chose the GCS-stick channel and gets GCS-stick control.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `QGC's Pause pauses the plan and the aircraft keeps station`() {
        val h = Harness().flying()
        h.place(northM = 10.0)
        h.tickAlive()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.reposition(
                RepositionCommand(
                    isCommandInt = false, frame = 0, latE7 = 0, lonE7 = 0,
                    zAmslM = Float.NaN, groundSpeedMs = -1f, yawRad = Float.NaN,
                )
            ),
        )
        assertEquals(listOf(MissionPauseCause.GCS_PAUSE to 0), h.sink.paused)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_PAUSED))
        // Keeping station: still engaged, zero commanded velocity.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().pitch, 1e-12)
    }

    @Test
    fun `a goto commanded mid-mission pauses the plan rather than deleting it`() {
        val h = Harness().flying()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.reposition(
                RepositionCommand(
                    isCommandInt = true, frame = 0,
                    latE7 = (latNorthOf(10.0) * 1e7).toInt(), lonE7 = (LON * 1e7).toInt(),
                    zAmslM = (DATUM + ALT).toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
                )
            ),
        )
        // **The cause has to name what actually happened.** Until 2026-07-27 this said
        // GCS_STICK_DEFLECTION, so an operator reading their own record was told a stick had been
        // deflected when none had been touched. The consequence was right and the account was
        // false, which is the worse of the two failures because it will be believed.
        assertEquals(listOf(MissionPauseCause.GCS_NEW_DESTINATION to 0), h.sink.paused)
    }

    @Test
    fun `an orbit commanded mid-mission pauses the plan under the same honest cause`() {
        val h = Harness().flying()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.orbit(
                OrbitCommand(
                    isCommandInt = true, frame = 0,
                    latE7 = (latNorthOf(10.0) * 1e7).toInt(), lonE7 = (LON * 1e7).toInt(),
                    radiusM = 20f, velocityMs = Float.NaN,
                    yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat(),
                    turns = Float.NaN,
                    zAmslM = (DATUM + ALT).toFloat(),
                )
            ),
        )
        assertEquals(listOf(MissionPauseCause.GCS_NEW_DESTINATION to 0), h.sink.paused)
    }

    @Test
    fun `a send that throws pauses the plan, resumably`() {
        val h = Harness().flying()
        h.port.throwOnSend = IllegalStateException("boom")
        h.tickAlive()
        assertEquals(listOf(MissionPauseCause.SEND_FAILED to 0), h.sink.paused)
    }

    @Test
    fun `stop abandons the plan, because nothing about a mission is persisted`() {
        val h = Harness().flying()
        h.engine.stop()
        assertEquals(listOf(MissionPauseCause.BRIDGE_STOPPED to 0), h.sink.paused)
    }

    @Test
    fun `AN ABORTED MISSION IS GONE, not lying in ambush for the next engagement`() {
        val h = Harness().flying()
        h.engine.abort(GuidedStickEngine.DisengageReason.RC_STICKS)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertNull(h.engine.missionCursorSeq())
        assertFalse(h.engine.missionFlying(h.now))
        // Re-engage through a fresh path and confirm nothing flies: no route means the passthrough
        // branch, and no route means no second `paused` report either.
        h.frame()
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS + 100
        h.frame(x = 500)
        h.confirm()
        repeat(5) { h.tickAlive() }
        assertEquals(1, h.sink.paused.size)
    }

    // ------------------------------------------------------------------ the rejoin

    @Test
    fun `THE REJOIN IS A RESTING LEG - a fly-through cannot complete it`() {
        val h = Harness()
        // Resume at waypoint 1, which is an *intermediate* step and would normally be passed
        // through. §6.3 says the rejoin leg is one nobody drew, so it gets the conservative test.
        assertEquals(Verdict.ACCEPTED, h.start(startIndex = 1, rejoining = true))
        h.confirm()
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_REJOINING))
        h.place(northM = 60.0, vn = GuidedEnvelope.HORIZONTAL_MAX_MS)
        repeat(RepositionGuidance.ARRIVE_TICKS * 3) { h.tickAlive() }
        assertTrue("a rejoin completed on a fly-through", h.sink.reached.isEmpty())
        // At rest, it completes.
        h.place(northM = 60.0, vn = 0.1)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(listOf(1), h.sink.reached)
        // And the *next* leg is an ordinary one again: the rejoin flag is spent, not sticky.
        h.place(northM = 88.0, vn = GuidedEnvelope.HORIZONTAL_MAX_MS)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertTrue(h.sink.finished.isEmpty()) // the last one still rests, whatever the rejoin did
    }

    @Test
    fun `a start index outside the route is refused rather than clamped`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.start(startIndex = 9, rejoining = true))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    // ------------------------------------------------------------------ the takeoff, both phases

    /** A takeoff item at [ALT], then one waypoint — the shortest plan that has both phases in it. */
    private fun takeoffRoute() = MissionRoute.of(
        planId = 1,
        steps = listOf(
            MissionStep(0, MissionStepKind.TAKEOFF, LAT, LON, ALT, MissionGuidance.R_SWITCH_M, rest = true),
            waypoint(1, 30.0, rest = true),
        ),
    )

    private fun takeoffHarness() = Harness(withTakeoff = true)


    @Test
    fun `THE TAKEOFF SEAM - isFlying alone does not hand back, and does not move the cursor`() {
        // The landmine, at the engine level. `isFlying` goes true 1.4 s and 0.6 s *into*
        // `AUTO_TAKE_OFF` in the measured records, with seconds of DJI's own climb still to run.
        // A cursor that moved there would start flying the next leg horizontally, a metre up,
        // against DJI. Nothing may leave phase one until the mode leaves the takeoff family.
        val h = takeoffHarness()
        h.place(isFlying = false, flightMode = "GPS_ATTI", relAlt = 0.0)
        assertEquals(Verdict.ACCEPTED, h.start(takeoffRoute()))
        h.confirm()
        repeat(5) { h.tickAlive() }
        // Asked exactly once, however many ticks pass.
        assertEquals(1, h.takeoff.calls)
        assertTrue(h.texts().contains(GuidedStatusTexts.MISSION_TAKEOFF))

        // DJI's own takeoff, exactly as recorded: the mode trail, then `isFlying`.
        h.place(isFlying = false, flightMode = "MOTOR_START", relAlt = 0.0)
        h.tickAlive()
        h.place(isFlying = false, flightMode = "AUTO_TAKE_OFF", relAlt = 0.3)
        h.tickAlive()
        h.place(isFlying = true, flightMode = "AUTO_TAKE_OFF", relAlt = 0.8)
        repeat(10) { h.tickAlive() }
        assertTrue("the cursor moved while DJI was still climbing", h.sink.reached.isEmpty())
        // Zero commanded throughout phase one: DJI is flying it and ours would be fighting it.
        assertEquals(0.0, h.port.sent.last().pitch, 1e-12)
        assertEquals(0.0, h.port.sent.last().roll, 1e-12)
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-12)

        // The mode leaves the takeoff family: DJI has let go, and phase two begins — which is
        // still not the cursor moving.
        h.place(isFlying = true, flightMode = "GPS_ATTI", relAlt = 1.0)
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.TAKEOFF_CLIMB_ENGAGING))
        assertTrue("the cursor moved at the handback, not at the top of the climb", h.sink.reached.isEmpty())
    }

    /**
     * **The mission's `NAV_TAKEOFF` points the camera down, like every other takeoff door — the
     * landing16 latch.**
     *
     * The failure this pins, in full, because it cost a whole autonomous flight
     * (`datasets/landing16/20260730-161329.001.jsonl`, 2026-07-30): the phone and QGC takeoff doors
     * had passed `aimCameraNadir = true` since 2026-07-29, and the mission's takeoff item did not.
     * So the aircraft climbed with the gimbal wherever the RC wheel had left it — pitch 0° for the
     * whole record — through the 1–2 m pass that `vision/TagArming` opens the detector for and that
     * detects at 100 %. Nothing was seen, nothing latched, the climb crossed the detector's ceiling
     * at t=62.0 and closed the one-way acquisition band for the rest of the flight, and the
     * flight's own precision landing was refused `NO_TAG_LATCHED` at t=188.8 while hovering over
     * the pad with the camera pointing straight at it. **Zero `tag` lines in 241 seconds**, against
     * 617 on the manual flight of the same morning (`20260730-101110`).
     *
     * Written to fail on the shape as well as the fact: the aim must be **once**, at the handback
     * and not before (the camera must not be commanded while DJI is flying its own takeoff — the
     * `flyTakeoffClimb` argument), at nadir, and on the record.
     */
    @Test
    fun `THE MISSION TAKEOFF AIMS THE CAMERA AT NADIR - the landing16 latch`() {
        val h = takeoffHarness()
        h.place(isFlying = false, flightMode = "GPS_ATTI", relAlt = 0.0)
        assertEquals(Verdict.ACCEPTED, h.start(takeoffRoute()))
        h.confirm()
        h.tickAlive()

        // Nothing while DJI flies its own takeoff: an aim into that window would be a commanded
        // angle DJI might silently move, which is what makes `PitchBelief`'s commanded half a lie.
        h.place(isFlying = false, flightMode = "MOTOR_START", relAlt = 0.0)
        h.tickAlive(300)
        h.place(isFlying = false, flightMode = "AUTO_TAKE_OFF", relAlt = 0.3)
        h.tickAlive(1_600)
        h.place(isFlying = true, flightMode = "AUTO_TAKE_OFF", relAlt = 0.8)
        h.tickAlive(1_400)
        assertTrue("the camera was aimed while DJI still had the aircraft", h.gimbal.aimed.isEmpty())

        // DJI lets go: the camera goes to nadir, exactly once, and the record says which door.
        h.place(isFlying = true, flightMode = "GPS_ATTI", relAlt = 1.0)
        h.tickAlive(1_900)
        assertEquals(
            "the mission takeoff must aim the camera at nadir at the handback",
            listOf(TagDescentGuidance.NADIR_PITCH_DEG), h.gimbal.aimed,
        )
        val ended = h.events.filter { it.first == EventCode.TAKEOFF_CLIMB_ENDED }.map { it.second }
        assertEquals(listOf("mission seq=0 nadir"), ended)

        // And exactly once: the whole climb that follows must not re-aim it.
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = ALT / 2)
        repeat(10) { h.tickAlive() }
        assertEquals(1, h.gimbal.aimed.size)
    }

    /**
     * **A takeoff that never happens never aims a camera.** The flag rides
     * [TakeoffClimb.armWatch]'s pending intention, so every rung that kills the watch kills the
     * camera move with it — here the leg timeout, which is the mission's own ending for a DJI that
     * never lets go. The alternative shape (aim at dispatch, or on `isFlying`) would leave a
     * grounded aircraft with its camera pointed at the ground for the rest of the session.
     */
    @Test
    fun `a mission takeoff that times out never aims the camera`() {
        val h = takeoffHarness()
        h.place(isFlying = false, flightMode = "GPS_ATTI", relAlt = 0.0)
        assertEquals(Verdict.ACCEPTED, h.start(takeoffRoute()))
        h.confirm()
        h.tickAlive()
        // DJI never gets there: motors turn, the aircraft never flies, the leg bound expires.
        h.place(isFlying = false, flightMode = "MOTOR_START", relAlt = 0.0)
        repeat(4) { h.tickAlive(MissionGuidance.legTimeoutMs(0.0) / 3) }
        assertTrue("a takeoff that never happened aimed a camera", h.gimbal.aimed.isEmpty())
    }

    @Test
    fun `the takeoff climbs to the item's height, vertically only, and then advances`() {
        val h = takeoffHarness()
        h.handedBack(takeoffRoute())
        // Phase two: full climb rate, and *nothing* horizontal — the takeoff item's coordinate is
        // where the aircraft already is, so a horizontal term here could only ever be error.
        h.tickAlive()
        val climbing = h.port.sent.last()
        assertEquals(0.0, climbing.pitch, 1e-12)
        assertEquals(0.0, climbing.roll, 1e-12)
        assertEquals(GuidedEnvelope.VERTICAL_MAX_MS, climbing.verticalThrottle, 1e-9)
        assertTrue(h.sink.reached.isEmpty())

        // Still climbing at half height, still no cursor.
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = ALT / 2)
        repeat(RepositionGuidance.ARRIVE_TICKS * 2) { h.tickAlive() }
        assertTrue(h.sink.reached.isEmpty())
        assertTrue(h.port.sent.last().verticalThrottle > 0.0)

        // Arrived and at rest: M3's own predicate, and only then the cursor.
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = ALT)
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        assertTrue("advanced before the arrival test was satisfied", h.sink.reached.isEmpty())
        h.tickAlive()
        assertEquals(listOf(0), h.sink.reached)
        assertEquals(listOf(0, 1), h.sink.cursors)
    }

    @Test
    fun `THE CLIMB IS THE SAME LAW THE OPERATOR'S TAKEOFF FLIES`() {
        // The climb is not a second vertical law. With no horizontal error and nothing ahead to
        // rest for, the mission's law reduces to `RepositionGuidance.clampedSpeed` on the vertical
        // axis — the arithmetic M2.5's own two-phase takeoff flies — and this asserts the identity
        // rather than a number, so moving either law moves both or fails here.
        for (remaining in listOf(0.4, 1.0, 2.0, 5.0, 12.0)) {
            val h = takeoffHarness()
            h.handedBack(takeoffRoute())
            h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = ALT - remaining)
            h.tickAlive()
            val expected = RepositionGuidance.clampedSpeed(remaining, GuidedEnvelope.VERTICAL_MAX_MS)
            assertEquals(
                "the climb diverged from M3's vertical law $remaining m out",
                expected, h.port.sent.last().verticalThrottle, 1e-9,
            )
        }
    }

    @Test
    fun `A TAKEOFF NEVER TRANSLATES - the climb is vertical even when the aircraft has drifted`() {
        // The mutation this exists for: the climb reusing the ordinary leg law, which would fly to
        // the takeoff item's *coordinate*. That is invisible in a test where the aircraft sits on
        // the item — which is where DJI's hop leaves it in still air — and very visible in wind. A
        // takeoff item's coordinate is `home` by construction, so any horizontal command here is a
        // metre-high translation nobody asked for, back towards a point the plan never meant as a
        // destination.
        val h = takeoffHarness()
        h.handedBack(takeoffRoute())
        h.state = Harness.stateAt(
            latDeg = latNorthOf(12.0), lonDeg = lonEastOf(-7.0), relAlt = 1.0,
            isFlying = true, flightMode = "JOYSTICK",
        )
        repeat(3) { h.tickAlive() }
        val sent = h.port.sent.last()
        assertEquals("the climb flew north towards the takeoff item", 0.0, sent.roll, 1e-12)
        assertEquals("the climb flew east towards the takeoff item", 0.0, sent.pitch, 1e-12)
        assertEquals(GuidedEnvelope.VERTICAL_MAX_MS, sent.verticalThrottle, 1e-9)
        // And the drift does not hold the climb open either: the arrival test is the vertical one.
        h.state = Harness.stateAt(
            latDeg = latNorthOf(12.0), lonDeg = lonEastOf(-7.0), relAlt = ALT,
            isFlying = true, flightMode = "JOYSTICK",
        )
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(listOf(0), h.sink.reached)
    }

    @Test
    fun `the climb stops at the ceiling, whatever the plan said`() {
        // The engine's own gate, tested against a route the launch check would have refused —
        // because `tickMissionClimbLocked` must not be the layer that trusts a check upstream of
        // it. This is the same `climbGatedLocked` every other climbing setpoint passes through.
        val overTheTop = MissionRoute.of(
            planId = 1,
            steps = listOf(
                MissionStep(
                    0, MissionStepKind.TAKEOFF, LAT, LON,
                    GuidedEnvelope.CEILING_M + 40.0, MissionGuidance.R_SWITCH_M, rest = true,
                ),
                waypoint(1, 30.0, rest = true),
            ),
        )
        val h = takeoffHarness()
        h.handedBack(overTheTop)
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = GuidedEnvelope.CEILING_M - 20.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.VERTICAL_MAX_MS, h.port.sent.last().verticalThrottle, 1e-9)
        // At the ceiling the climb is cut, and the operator is told which limit stopped it.
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = GuidedEnvelope.CEILING_M)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.CEILING))
    }

    @Test
    fun `the climb is bounded by a timeout scaled to its own height`() {
        val h = takeoffHarness()
        h.handedBack(takeoffRoute())
        // A climb that never arrives — the aircraft sits where DJI left it, with the mode DJI
        // reports once we are the ones commanding. The bound is the leg timeout for the climb's
        // own height, not the flat one phase one used.
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = 1.0)
        val bound = MissionGuidance.legTimeoutMs(ALT - 1.0)
        assertTrue("the climb inherited phase one's flat bound", bound > MissionGuidance.legTimeoutMs(0.0))
        h.now += bound - 1_000
        h.tickAlive()
        assertTrue("ended early", h.sink.paused.isEmpty())
        h.now += 2_000
        h.tickAlive()
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertEquals(MissionPauseCause.LEG_TIMEOUT, h.sink.paused.last().first)
    }

    @Test
    fun `the climb gets the ordinary grace for DJI's mode to become JOYSTICK`() {
        // DJI hands back in `GPS_ATTI` — that *is* the handback signal — and the mode does not
        // become `JOYSTICK` until our setpoints have been flowing for a moment (measured at ~0.3 s
        // on the first engagement). The engagement itself is minutes old by then, so without the
        // grace being restamped at the handback the very first climbing tick reads its own
        // handback signal as DJI seizing the aircraft, and the mission dies at the top of the hop.
        val h = takeoffHarness()
        h.handedBack(takeoffRoute())
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals("the handback mode read as a seizure", GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.sink.paused.isEmpty())
        assertTrue(h.port.sent.last().verticalThrottle > 0.0)

        // The grace is a window, not a removal: a mode that never becomes JOYSTICK is still a
        // seizure, on the same clock every other engagement is held to.
        h.now += GuidedStickEngine.MODE_SEIZE_GRACE_MS
        h.tickAlive()
        assertNotEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.sink.paused.isNotEmpty())
    }

    @Test
    fun `PHASE ONE RUNS ON A PARKED AIRCRAFT'S SILENT POSITION FEED`() {
        // The other half of the bench failure of 2026-07-27. A parked aircraft publishes no
        // position, because `KeyAircraftLocation` is change-driven and nothing is changing — so on
        // the ground `POSITION` is stale by construction, permanently, with a perfect fix on the
        // screen. Below the tick's position block that stale reading returns before DJI is ever
        // asked to take off: the mission would command zero at a motionless aircraft and then
        // release itself after `POSITION_LOST_MS` having done nothing whatsoever.
        val h = takeoffHarness()
        h.place(isFlying = false, flightMode = "GPS_ATTI", relAlt = 0.0, positionAge = 60_000)
        assertEquals(Verdict.ACCEPTED, h.start(takeoffRoute()))
        h.confirm()
        h.tickAlive()
        assertEquals("DJI was never asked to take off", 1, h.takeoff.calls)

        // And it keeps waiting for DJI across the whole of `POSITION_LOST_MS`, rather than
        // releasing itself for want of a position it does not need and cannot have.
        repeat(((RepositionGuidance.POSITION_LOST_MS / 100) + 20).toInt()) { h.tickAlive() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue("the mission released itself while DJI was taking off", h.sink.paused.isEmpty())
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-12)

        // Phase one is still bounded — by its own leg timeout, not by the position feed.
        h.now += MissionGuidance.legTimeoutMs(0.0)
        h.tickAlive()
        assertEquals(MissionPauseCause.LEG_TIMEOUT, h.sink.paused.last().first)
    }

    @Test
    fun `A MISSION RELEASES A CIRCLE'S SUBJECT - a plan is a sequence of destinations`() {
        // The third release point, and the one with no test until the mutation sweep asked for it.
        // An orbit implies a subject at its centre and that subject outlives the circle (Ivan,
        // 2026-07-27) — but a *plan* is a sequence of destinations, so it lets go for the same
        // reason a single goto does. Without this the first leg of every mission flown after an
        // orbit would fly with the nose locked on a point the operator left behind.
        val h = takeoffHarness()
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.orbit(
                OrbitCommand(
                    isCommandInt = true, frame = 0,
                    latE7 = (LAT * 1e7).toInt(), lonE7 = (lonEastOf(40.0) * 1e7).toInt(),
                    radiusM = 20f, velocityMs = Float.NaN,
                    yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat(),
                    turns = Float.NaN,
                    zAmslM = (DATUM + ALT).toFloat(),
                )
            )
        )
        h.confirm()
        h.tickAlive()

        // Now fly a plan. The nose belongs to the leg, not to the circle's centre off to the east.
        assertEquals(Verdict.ACCEPTED, h.start(threeLegs()))
        h.confirm()
        h.place(northM = 0.0, yawDeg = 0.0) // pointing north, straight up the first leg
        repeat(3) { h.tickAlive() }
        assertEquals("the mission flew with the old circle's subject still held", 0.0, h.port.sent.last().yaw, 1e-6)
    }

    @Test
    fun `an ROI still points the aircraft during the takeoff climb`() {
        // The climb is a leg like any other as far as the camera is concerned. It goes through the
        // same [targetYawLocked] every other setpoint source does, so an ROI set before the mission
        // is still tracked while the aircraft is going straight up — and the yaw is the *only*
        // horizontal thing a climb may command.
        val h = takeoffHarness()
        assertEquals(Verdict.ACCEPTED, h.engine.roi(
            RoiCommand(
                command = RoiCommand.MAV_CMD_DO_SET_ROI_LOCATION, isCommandInt = true, frame = 0,
                latE7 = (LAT * 1e7).toInt(), lonE7 = (lonEastOf(80.0) * 1e7).toInt(),
                param1 = Float.NaN, param5 = Float.NaN, param6 = Float.NaN,
            )
        ))
        h.handedBack(takeoffRoute())
        h.place(isFlying = true, flightMode = "JOYSTICK", relAlt = 5.0)
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals("an ROI made the climb translate", 0.0, sent.pitch, 1e-12)
        assertEquals(0.0, sent.roll, 1e-12)
        assertTrue("the climb ignored the ROI", sent.yaw > 0.0) // the target is due east of us
        assertTrue(sent.verticalThrottle > 0.0)
        // And the operator is not told the opposite of what is happening. §9.3's sentence is for an
        // aircraft we are *not* flying; a mission is one we are, so it must not appear — the nose is
        // 90° off the target here, which is exactly when it would.
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_PITCH_ONLY))
    }

    @Test
    fun `an abort during the climb pauses the mission at the takeoff item`() {
        val h = takeoffHarness()
        h.handedBack(takeoffRoute())
        h.tickAlive()
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.sink.paused.last().second)
        assertTrue(h.sink.reached.isEmpty())
    }

    @Test
    fun `a takeoff DJI refuses ends the mission with DJI's own error`() {
        val route = MissionRoute.of(
            planId = 1,
            steps = listOf(
                MissionStep(0, MissionStepKind.TAKEOFF, LAT, LON, ALT, MissionGuidance.R_SWITCH_M, rest = true),
                waypoint(1, 30.0, rest = true),
            ),
        )
        val h = Harness(withTakeoff = true)
        h.takeoff.failWith = "SYSTEM_ERROR"
        h.place(isFlying = false)
        assertEquals(Verdict.ACCEPTED, h.start(route))
        h.confirm()
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.sink.paused.isNotEmpty())
        assertTrue(h.texts().any { it.contains("SYSTEM_ERROR") })
    }

    @Test
    fun `a plan beginning with a takeoff is refused when nothing can start one`() {
        val route = MissionRoute.of(
            planId = 1,
            steps = listOf(
                MissionStep(0, MissionStepKind.TAKEOFF, LAT, LON, ALT, MissionGuidance.R_SWITCH_M, rest = true),
            ),
        )
        val h = Harness(withTakeoff = false)
        h.place(isFlying = false)
        assertEquals(Verdict.DENIED, h.start(route))
    }

    // ------------------------------------------------- the heartbeat claim's third conjunct

    @Test
    fun `THE CLAIM CANNOT BE TRUE WHILE THE ENGINE IS NOT ENGAGED`() {
        // The mutation this design most needs to survive. `missionFlying` is conjunct 3 of M4-1's
        // three, and it is re-earned from DJI's own report on the tick it is read.
        val h = Harness()
        assertFalse(h.engine.missionFlying(h.now))
        assertEquals(Verdict.ACCEPTED, h.start())
        // ENGAGING: we asked; DJI has confirmed nothing.
        assertFalse("claimed while merely ENGAGING", h.engine.missionFlying(h.now))
        h.confirm()
        // ENGAGED, but no setpoint has gone out for the mission yet.
        assertFalse("claimed before a setpoint went out", h.engine.missionFlying(h.now))
        h.tickAlive()
        assertTrue(h.engine.missionFlying(h.now))
        // Authority drops: the claim goes with it, on the same tick.
        h.port.onState!!(VirtualStickSnapshot(enabled = false, advanced = false, authority = "RC"))
        assertFalse(h.engine.missionFlying(h.now))
    }

    @Test
    fun `the claim is not latched - it lapses when the setpoint stream stops`() {
        val h = Harness().flying()
        h.tickAlive()
        assertTrue(h.engine.missionFlying(h.now))
        // Time passes with no tick. The claim is a *report* on the setpoint stream, so it lapses.
        assertFalse(h.engine.missionFlying(h.now + GuidedStickEngine.MISSION_CLAIM_STALE_MS + 1))
    }

    @Test
    fun `the claim is stamped after the send, never before it`() {
        // The ordering that distinguishes a report from the echo: a send DJI refused does not count.
        val h = Harness().flying()
        h.port.throwOnSend = IllegalStateException("boom")
        h.tickAlive()
        assertFalse(h.engine.missionFlying(h.now))
    }

    // ------------------------------------------------------------------ the nose

    @Test
    fun `a mission leg turns the nose toward its waypoint`() {
        val h = Harness().flying()
        // Target due north, nose due east: turn left, at the cap.
        h.place(northM = 0.0, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(-GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
    }

    @Test
    fun `with heading-follows-course off, a mission leg commands exactly zero yaw`() {
        val h = Harness()
        h.headingFollows = false
        h.flying()
        h.place(northM = 0.0, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
    }

    @Test
    fun `a stale heading on a mission leg commands zero yaw and says so`() {
        val h = Harness().flying()
        h.place(northM = 0.0, yawDeg = 90.0, attitudeAge = 10_000L)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.HEADING_NO_HEADING))
    }

    @Test
    fun `the substitution is announced once, at the start of the mission`() {
        val h = Harness().flying()
        assertEquals(1, h.texts().count { it == GuidedStatusTexts.HEADING_FOLLOWS })
    }

    // ------------------------------------------------------------------ the ending

    @Test
    fun `a finished mission holds in the air and does not descend`() {
        val h = Harness().flying()
        h.place(northM = 30.0); h.tickAlive()
        h.place(northM = 60.0); h.tickAlive()
        h.place(northM = 90.0, vn = 0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS + 5) { h.tickAlive() }
        assertEquals(listOf(2), h.sink.finished)
        // Holding: still engaged, zero on every axis — including the vertical one, which is the
        // whole of M4-5.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        val last = h.port.sent.last()
        assertEquals(0.0, last.pitch, 1e-12)
        assertEquals(0.0, last.roll, 1e-12)
        assertEquals(0.0, last.verticalThrottle, 1e-12)
    }

    @Test
    fun `the hold is bounded - Q1's idle window ends it rather than holding authority forever`() {
        val h = Harness().flying()
        h.place(northM = 30.0); h.tickAlive()
        h.place(northM = 60.0); h.tickAlive()
        h.place(northM = 90.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(listOf(2), h.sink.finished)
        h.now += GuidedEnvelope.IDLE_DISENGAGE_MS
        h.tickAlive()
        assertEquals(listOf(MissionPauseCause.IDLE_HOLD to 2), h.sink.paused)
    }

    @Test
    fun `an aborted mission never reports a cursor the engine no longer has`() {
        val h = Harness().flying()
        assertEquals(0, h.engine.missionCursorSeq())
        h.place(northM = 30.0); h.tickAlive()
        assertEquals(1, h.engine.missionCursorSeq())
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        assertNull(h.engine.missionCursorSeq())
        assertNotEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }
}
