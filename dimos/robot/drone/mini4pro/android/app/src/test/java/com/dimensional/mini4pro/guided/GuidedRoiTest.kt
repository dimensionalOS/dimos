package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.StatusTexts
import com.dimensional.mini4pro.command.Verdict
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import io.dronefleet.mavlink.common.ManualControl
import io.dronefleet.mavlink.common.Statustext
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.roundToInt

/**
 * M4 — the region of interest through [GuidedStickEngine.roi]: the truthful ack, every refusal and
 * its sentence, the discarded altitude, the open-loop camera and its limiter, and — the part that
 * is flight control rather than photography — **exactly when the aircraft may be turned for it**.
 *
 * The arithmetic is pinned next door in `RoiGuidanceTest`. This file is about what the *engine*
 * does with it: same protocol as `GuidedOrbitTest` and `GuidedRepositionTest`, with a fake port, a
 * fake gimbal, a hand-cranked clock and no aircraft.
 *
 * **This is the wire door only.** Since 2026-07-30 a plan's own `DO_SET_ROI_LOCATION` /
 * `DO_SET_ROI_NONE` items are a second door onto the same state, entering below the liveness stamp at
 * `GuidedStickEngine.applyRoi`; every gate asserted here binds them too, and *when* each plan item acts
 * is `GuidedMissionRoiTest`'s subject with its own mutation table. Nothing in this file changed for it,
 * which is the property that made it worth doing that way.
 *
 * Written to fail loudly for the things that would make this feature dangerous or dishonest:
 *
 *  - **yaw on an aircraft that is not ours to turn** — during stick passthrough, while the RC pilot
 *    is flying, or with nothing of ours in progress. That is the line
 *    `docs/m4-mission-execution.md` §9.3 draws and the one this suite guards hardest
 *  - the discarded terrain altitude being **quietly used** instead of announced
 *  - the ROI path **consulting gimbal attitude or its age** — the change-driven-key trap, for the
 *    fourth time; an ROI held on a hovering aircraft is exactly when that key goes silent
 *  - a camera **hunting** at close range instead of holding its last angle
 *  - an unreachable angle commanded silently, so the operator believes the target is in frame
 *  - a refusal with no sentence, which is the `DO_ORBIT` gap of 2026-07-27
 *  - an ROI **surviving an abort as a live tracker**, slewing the camera after handback
 *  - an explicit ROI failing to outrank an orbit's own centre, or failing to give it back
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, run,
 * confirmed red and reverted. Counts are failing tests across the five suites this feature touches
 * (`GuidedRoiTest`, `RoiGuidanceTest`, `GuidedOrbitTest`, `OrbitGuidanceTest`,
 * `GuidedRepositionTest`) — **measured, not estimated**.
 *
 * The arithmetic ([RoiGuidance], and the pitch solution it shares with [OrbitGuidance]):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | gimbal pitch sign flipped (the camera aimed at the sky) | 15 |
 *  | the 30 °/s yaw clamp removed from the ROI law | 3 |
 *  | a stale heading guessed at zero instead of commanding no yaw | 1 |
 *  | the ±180° wrap dropped from the azimuth error | 1 |
 *  | the bearing-rate feed-forward's sign flipped | 3 |
 *  | the close-in floor removed (both loops live at 1 m) | 3 |
 *
 * The engine ([GuidedStickEngine.roi], the camera pass and the yaw scope):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `ACCEPTED` returned before any validation | 26 |
 *  | refusals answer `DENIED` with **no sentence** | 6 |
 *  | the ground-level announcement dropped (the discarded height hidden) | 1 |
 *  | the yaw-authority gate always true (the pitch-only sentence never fires) | 2 |
 *  | **an ROI yaw added to the stick passthrough** (the line, crossed) | 1 |
 *  | the gimbal range clamp removed (an unreachable angle commanded silently) | 2 |
 *  | the gimbal rate limit removed (commanded every tick) | 2 |
 *  | the gimbal deadband removed | 3 |
 *  | the close-in hold removed (the camera hunts at 1 m) | 1 |
 *  | the position-freshness gate removed (a cached fix aimed on) | 1 |
 *  | an abort no longer stops the tracking | 1 |
 *  | an explicit ROI no longer outranks the orbit's centre (camera) | 1 |
 *  | an explicit ROI no longer outranks the orbit's nose-to-centre (yaw) | 1 |
 *  | `DO_SET_ROI_NONE` gated behind the shape checks and the interlock | 3 |
 *  | the seam grows an attitude-age accessor (the closed-loop door) | 2 |
 *
 * ### The two that had to be sharpened, and why they are worth reading
 *
 * Both scored **0 on the first pass** and both were masked by defence in depth rather than by being
 * untested, which is exactly the case a count is for.
 *
 * **The ±180° wrap** survived because every azimuth case written by hand happened to be one the raw
 * subtraction gets right. It only matters across the ±180° seam — a target due south with the nose
 * 10° west of north is 10° away, not 350° — so the test now walks all 360 headings and asserts the
 * fold, and the mutant dies.
 *
 * **The ROI outranking the orbit's camera** survived because the *last* angle commanded was still
 * the ROI's: with both aimers live they take turns through the shared rate limiter, so an assertion
 * on the final value passes while the real behaviour is a camera whipping between the click and the
 * circle's centre every 500 ms. The test now asserts that the centre angle is **never** commanded
 * again after the click, which is the property that was meant all along.
 *
 * ## The relative-frame altitude (2026-07-30) — measured kill counts
 *
 * Ivan, 2026-07-30: *"ROI in the plan does have a height setting, it should be relative height to
 * takeoff height"*. A `MAV_FRAME_GLOBAL_RELATIVE_ALT` z is **our own datum** and is now used; QGC's
 * Fly-view AMSL (frame 0) is still discarded, which is the measured decision and the thing this
 * campaign's second row exists to protect. Same protocol as above: one breakage at a time against the
 * 2625-test tree, whole suite per mutant, `test-results` deleted first, confirmed red, reverted. **No
 * survivors.** The rest of that day's campaign — the precision `NAV_LAND` sequence — is tabulated in
 * `GuidedPrecisionLandTest`.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a relative-frame z ignored (ground level assumed anyway) | 3 |
 *  | an AMSL/terrain-frame z trusted (the measured discard reverted) | 2 |
 *  | the height difference's sign flipped (`altitude + target` instead of `−`) | 2 |
 *
 * The middle row is the one to keep an eye on: it is the *original* decision, and the two tests that
 * kill it are the frame walk and the once-per-ROI announcement. A future edit that "simplifies" the
 * frame check into trusting every z would put a 41.5 m barometric disagreement into a pointing solution
 * and tell the operator their height had been ignored while it was being used.
 */
class GuidedRoiTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        /** The published pressure-altitude datum this session. Another session saw it 41.5 m away. */
        const val DATUM = 100.0

        /** Default aircraft height above the datum. */
        const val ALT = 10.0

        /** Where the aircraft starts: this far north of the ROI, so the target bears due south. */
        const val START_NORTH = 10.0

        /** Nose due south — pointed straight at the ROI from the starting position. */
        const val NOSE_AT_ROI = 180.0

        fun latNorthOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (RepositionGuidance.METRES_PER_DEG * cos(Math.toRadians(LAT)))
    }

    private class FakeVirtualStickPort : VirtualStickPort {
        var unavailable: String? = null
        var enableCalls = 0
        var disableCalls = 0
        var enableOnSuccess: (() -> Unit)? = null
        var enableOnFailure: ((String) -> Unit)? = null

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
            enableOnFailure = onFailure
        }

        override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
            disableCalls++
        }

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
            this.onReason = onAuthorityReason
        }

        override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
            this.onRc = onDelivery
        }

        override fun cancelListens() = Unit
    }

    /**
     * The camera, faked. **It has no attitude and no age to report, because [ManoeuvreGimbal] has no
     * way to ask for one** — which is the point, and what the structural test below asserts.
     */
    private class FakeGimbal : ManoeuvreGimbal {
        var range: ClosedFloatingPointRange<Double>? = null
        var throwOnAim: Throwable? = null
        val aimed = mutableListOf<Double>()

        override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? = range

        override fun aimPitch(pitchDeg: Double) {
            throwOnAim?.let { throw it }
            aimed += pitchDeg
        }
    }

    private class RecordedCmd(
        val setpoint: Setpoint?, val axes: StickAxes, val source: CommandSource?,
        val accepted: Boolean?, val error: String?,
    )

    private class Harness(withGimbal: Boolean = true) {
        var now = 1_000L
        var interlock = true

        /**
         * Heading-follows-course, defaulting on as the shipped flag does. A few tests below turn it
         * off so that an assertion about **the ROI's** contribution to the yaw axis is not reading
         * the leg's own nose-toward-the-target law instead.
         */
        var headingFollows = true
        var state = stateAt()
        val port = FakeVirtualStickPort()
        val gimbal = FakeGimbal()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            headingFollowsCourse = { headingFollows },
            manoeuvreGimbal = if (withGimbal) gimbal else null,
            record = object : GuidedRecord {
                override fun stickCmd(
                    setpoint: Setpoint?, axes: StickAxes, modes: StickModes,
                    source: CommandSource?, range: StickRange?, accepted: Boolean?, error: String?,
                ) {
                    cmds += RecordedCmd(setpoint, axes, source, accepted, error)
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

        fun said(text: String): Int = texts().count { it == text }

        /** The aircraft as the encoder would snapshot it — everything fresh unless said otherwise. */
        fun place(
            latDeg: Double = latNorthOf(START_NORTH), lonDeg: Double = LON, relAlt: Double? = ALT,
            positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
            attitudeAge: Long = 0L, yawDeg: Double? = NOSE_AT_ROI,
            datum: Double? = DATUM, flightMode: String? = null,
        ) {
            state = stateAt(
                latDeg, lonDeg, relAlt,
                positionAge, altitudeAge, velocityAge, attitudeAge, yawDeg, datum, flightMode,
            )
        }

        /** `DO_SET_ROI_LOCATION` as QGC's map click sends it: `COMMAND_INT`, frame 0, 1e7 x/y. */
        fun roi(
            latDeg: Double = LAT, lonDeg: Double = LON,
            frame: Int = 0, isInt: Boolean = true,
            command: Int = RoiCommand.MAV_CMD_DO_SET_ROI_LOCATION,
            param1: Float = Float.NaN,
            z: Float = 0f,
        ): Verdict = engine.roi(
            RoiCommand(
                command = command, isCommandInt = isInt, frame = frame,
                latE7 = (latDeg * 1e7).roundToInt(), lonE7 = (lonDeg * 1e7).roundToInt(),
                param1 = param1,
                param5 = if (isInt) Float.NaN else latDeg.toFloat(),
                param6 = if (isInt) Float.NaN else lonDeg.toFloat(),
                z = z,
            )
        )

        fun roiNone(isInt: Boolean = true): Verdict = engine.roi(
            RoiCommand(command = RoiCommand.MAV_CMD_DO_SET_ROI_NONE, isCommandInt = isInt)
        )

        fun goto(latDeg: Double = latNorthOf(60.0), lonDeg: Double = LON): Verdict = engine.reposition(
            RepositionCommand(
                isCommandInt = true, frame = 0,
                latE7 = (latDeg * 1e7).roundToInt(), lonE7 = (lonDeg * 1e7).roundToInt(),
                zAmslM = (DATUM + ALT).toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
            )
        )

        fun orbit(radius: Float = 20f): Verdict = engine.orbit(
            OrbitCommand(
                isCommandInt = true, frame = 0,
                radiusM = radius, velocityMs = Float.NaN,
                yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat(), turns = Float.NaN,
                latE7 = (LAT * 1e7).roundToInt(), lonE7 = (LON * 1e7).roundToInt(),
                zAmslM = (DATUM + ALT).toFloat(),
            )
        )

        /** DJI confirms the engagement the way the state listener would. */
        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tick(40)
        }

        /** One engine tick with the GCS link demonstrably alive (any inbound traffic). */
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

        /** Put the aircraft on the default 20 m orbit at [bearingDeg] from the centre, at rest. */
        fun placeOnCircle(bearingDeg: Double, radius: Double = 20.0) {
            val rad = Math.toRadians(bearingDeg)
            val (lat, lon) = RepositionGuidance.offsetCoordinate(
                LAT, LON, radius * kotlin.math.cos(rad), radius * kotlin.math.sin(rad),
            )
            place(latDeg = lat, lonDeg = lon, yawDeg = bearingDeg + 180.0)
        }

        companion object {
            fun stateAt(
                latDeg: Double = latNorthOf(START_NORTH), lonDeg: Double = LON, relAlt: Double? = ALT,
                positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
                attitudeAge: Long = 0L, yawDeg: Double? = NOSE_AT_ROI,
                datum: Double? = DATUM, flightMode: String? = null,
            ) = AircraftState(
                latitude = latDeg, longitude = lonDeg,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = datum,
                velocityNorth = 0.0, velocityEast = 0.0, velocityDown = 0.0,
                yawDeg = yawDeg,
                flightMode = flightMode,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to altitudeAge,
                    Signal.VELOCITY to velocityAge,
                    Signal.ATTITUDE to attitudeAge,
                ),
            )
        }
    }

    // ------------------------------------------------------- accept, and the ack

    @Test
    fun `a map click is ACCEPTED, aims the camera, and flies nothing`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.roi())
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_STARTED))
        assertTrue(h.events.any { it.first == "roi_accepted" })
        // An ROI is a modifier, not a manoeuvre: it does not engage, does not ask DJI for
        // anything, and cannot start the aircraft moving.
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        h.tickAlive()
        assertTrue(h.port.sent.isEmpty())
        // …but the camera is pointed, because pointing a camera is not flying an aircraft.
        assertEquals(1, h.gimbal.aimed.size)
        assertEquals(-45.0, h.gimbal.aimed.last(), 1e-6)
    }

    @Test
    fun `THE DISCARDED DATUM - the height is announced once per ROI, in the operator's own words`() {
        val h = Harness()
        h.roi()
        assertEquals(1, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
        // Not repeated by the tracking, however long it runs: it is a fact about the *command*.
        repeat(200) { h.tickAlive() }
        assertEquals(1, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
        // A second click is a second ROI, and is owed the sentence again.
        h.now += GuidedStickEngine.ANNOUNCE_REPEAT_MS + 1
        h.roi(latDeg = latNorthOf(-20.0))
        assertEquals(2, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
    }

    @Test
    fun `THE DISCARDED DATUM - an AMSL frame's z changes nothing at all, whatever it says`() {
        // **The measured discard, and it must survive** the 2026-07-30 change that started honouring a
        // *relative*-frame z. QGC's Fly-view ROI is a terrain-database AMSL in `frame = 0`; our datum
        // is a barometer that moved 41.5 m between sessions, so their difference is not a round trip,
        // it is two unrelated numbers. This test was structural until the relative frame arrived (the
        // type had no field a height could come in through); it is now behavioural, which is the
        // strongest form still available — and it walks the frames rather than one of them, because the
        // wrong fix for the relative case is "trust z everywhere".
        //
        // A 500 m AMSL would tip the camera to −0.2° instead of −45° if it were believed.
        for (frame in listOf(0, 5, 10, 11)) {
            val h = Harness()
            h.roi(frame = frame, z = 500f)
            h.tickAlive()
            assertEquals("frame $frame", -45.0, h.gimbal.aimed.last(), 1e-6)
            assertEquals("frame $frame", 1, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
        }
    }

    @Test
    fun `a COMMAND_LONG's param7 is never trusted - there is no frame to interpret it in`() {
        // The legacy 201 carries no frame at all, so its `param7` could be anything. Unknown is never
        // zero: the ground-level assumption stands and is announced.
        val h = Harness()
        h.roi(isInt = false, command = RoiCommand.MAV_CMD_DO_SET_ROI, param1 = 3f, z = 500f)
        h.tickAlive()
        // 0.01° of tolerance, not 1e-6: 201 carries its coordinates in **float32**, whose spacing at
        // this latitude is ~0.5 m on the ground (the class KDoc's own argument for why 201 is accepted
        // for aiming and by no manoeuvre). A believed 500 m z would be −0.2°, nowhere near this.
        assertEquals(-45.0, h.gimbal.aimed.last(), 0.01)
        assertEquals(1, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
    }

    @Test
    fun `THE HONOURED DATUM - a relative-frame z is our own datum, so the solve reads the height above the TARGET`() {
        // Ivan, 2026-07-30: "ROI in the plan does have a height setting, it should be relative height
        // to takeoff height". A `MAV_FRAME_GLOBAL_RELATIVE_ALT` z is metres above the takeoff point —
        // the same datum as our own altitude, the ceiling, the waypoints and the tag fixes — so it is
        // directly usable and the depression solves on the *difference*.
        //
        // The aircraft is 10 m up and 10 m away. Ground level → −45°; a 5 m-high target → −atan2(5, 10)
        // = −26.57°, which is the number a rooftop or a hillside subject actually needs.
        val h = Harness()
        h.roi(frame = 3, z = 5f)
        h.tickAlive()
        assertEquals(-Math.toDegrees(Math.atan2(5.0, 10.0)), h.gimbal.aimed.last(), 1e-6)
        // And the operator is told the number that is now in the solution, instead of being told the
        // height was ignored — the two facts are different and only one of them is true here.
        assertEquals(1, h.said(GuidedStatusTexts.roiTargetHeight(5.0)))
        assertEquals(0, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
    }

    @Test
    fun `a relative-frame z of zero is honoured and announced, not silently the same as no height`() {
        // big1.plan's own ROI item is exactly this (`frame = 3`, z = 0), so the arithmetic is
        // identical to the ground-level assumption — and the *sentence* still differs, deliberately:
        // "your target is at takeoff level because you said so" and "assumed, because nobody could
        // tell" are different facts, and the second one is a warning.
        val h = Harness()
        h.roi(frame = 3, z = 0f)
        h.tickAlive()
        assertEquals(-45.0, h.gimbal.aimed.last(), 1e-6)
        assertEquals(1, h.said(GuidedStatusTexts.roiTargetHeight(0.0)))
        assertEquals(0, h.said(GuidedStatusTexts.ROI_GROUND_LEVEL))
    }

    @Test
    fun `a target ABOVE the aircraft aims UP, and says so when the gimbal cannot reach that far`() {
        // A tower. The sign must not be flattened to zero and must not be flipped: 30 m of target
        // above a 10 m aircraft is 20 m of *negative* height difference, which solves to a positive
        // (upward) pitch of +63.4°.
        val free = Harness()
        free.roi(frame = 3, z = 30f)
        free.tickAlive()
        assertEquals(Math.toDegrees(Math.atan2(20.0, 10.0)), free.gimbal.aimed.last(), 1e-6)

        // With a reported travel that stops at +30°, the existing clamp bites and the operator is told
        // the subject is not in frame — the same path any unreachable angle takes, and no new one.
        val bounded = Harness()
        bounded.gimbal.range = -90.0..30.0
        bounded.roi(frame = 3, z = 30f)
        bounded.tickAlive()
        assertEquals(30.0, bounded.gimbal.aimed.last(), 1e-6)
        assertEquals(1, bounded.said(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `the interlock being off does not stop the camera - aiming is not flying`() {
        // The one gate that looks as if it ought to be here and deliberately is not. Manual gimbal
        // aiming has never been interlock-gated in this project; the ROI's *yaw* half is gated by
        // construction, because it can only run inside a manoeuvre tick and a manoeuvre needs
        // DJI-confirmed authority, which needs the interlock.
        val h = Harness()
        h.interlock = false
        assertEquals(Verdict.ACCEPTED, h.roi())
        h.tickAlive()
        assertEquals(1, h.gimbal.aimed.size)
        assertEquals(-45.0, h.gimbal.aimed.last(), 1e-6)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    // ---------------------------------------------- refusals, and every one has a sentence

    @Test
    fun `a COMMAND_LONG 195 is refused as an unmeasured shape`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.roi(isInt = false))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_ROI_LONG_FORM) })
        assertTrue(h.gimbal.aimed.isEmpty())
    }

    @Test
    fun `the global frame family is accepted and a local frame is refused`() {
        // Wider than `DO_REPOSITION`'s single measured frame, and the argument is that this command
        // *discards* z: the frame decides only the encoding of the horizontal pair, which is
        // identical across the global family. A LOCAL_NED frame's x/y would be centimetres, and
        // there the difference is real.
        for (frame in RoiCommand.GLOBAL_FRAMES) {
            assertEquals("frame $frame", Verdict.ACCEPTED, Harness().roi(frame = frame))
        }
        for (frame in listOf(1, 2, 4, 8, 9, 20)) {
            val h = Harness()
            assertEquals("frame $frame", Verdict.DENIED, h.roi(frame = frame))
            assertTrue("frame $frame", h.texts().any { it.contains("FRAME_$frame") })
        }
    }

    @Test
    fun `a target that is not a coordinate is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.roi(latDeg = 0.0, lonDeg = 0.0))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_BAD_TARGET) })
    }

    @Test
    fun `with no camera at all the ROI is refused, not accepted politely`() {
        // `Vehicle::_handleCommandAck` flips QGC's own `isROIEnabled` on our ACCEPTED and draws the
        // marker on the map. Accepting here would put a marker on the operator's screen for a
        // camera that does not exist.
        val h = Harness(withGimbal = false)
        assertEquals(Verdict.DENIED, h.roi())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_ROI_NO_GIMBAL) })
        // …and the aircraft is unaffected: a bridge with no camera still flies.
        assertEquals(Verdict.ACCEPTED, h.goto())
    }

    @Test
    fun `the legacy DO_SET_ROI carries the same intent in param5-6, and its NONE clears`() {
        // QGC does not send 201; this door is for imported plans and older ground stations. Its
        // coordinates are float32 degrees — about 0.5 m of resolution at our latitudes, which is
        // fine for aiming a camera and is why no *manoeuvre* in this project accepts float
        // coordinates.
        val h = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            h.roi(
                command = RoiCommand.MAV_CMD_DO_SET_ROI, isInt = false,
                param1 = RoiCommand.MAV_ROI_LOCATION.toFloat(),
            ),
        )
        h.tickAlive()
        assertEquals(1, h.gimbal.aimed.size)
        // …and the same command id, with MAV_ROI_NONE, means stop.
        assertEquals(
            Verdict.ACCEPTED,
            h.engine.roi(
                RoiCommand(
                    command = RoiCommand.MAV_CMD_DO_SET_ROI, isCommandInt = false,
                    param1 = RoiCommand.MAV_ROI_NONE.toFloat(),
                )
            ),
        )
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_CLEARED))
        val before = h.gimbal.aimed.size
        h.place(latDeg = latNorthOf(40.0))
        repeat(20) { h.tickAlive() }
        assertEquals("a cleared ROI kept driving the camera", before, h.gimbal.aimed.size)
    }

    @Test
    fun `a legacy ROI mode that is not a fixed location is refused, not approximated`() {
        // MAV_ROI_WPNEXT, _WPINDEX and _TARGET are different intents — a moving target, or one named
        // by a mission index. Pointing approximately at whatever the coordinates happen to hold
        // would be interpreting a command we have not measured.
        for (mode in listOf(1, 2, 4)) {
            val h = Harness()
            assertEquals(
                "mode $mode", Verdict.DENIED,
                h.roi(command = RoiCommand.MAV_CMD_DO_SET_ROI, isInt = false, param1 = mode.toFloat()),
            )
            assertTrue("mode $mode", h.texts().any { it.contains(GuidedStatusTexts.REASON_ROI_MODE) })
        }
    }

    @Test
    fun `EVERY ROI refusal carries a sentence, and every sentence fits the field`() {
        val refusals: List<Pair<String, (Harness) -> Verdict>> = listOf(
            "long form" to { h -> h.roi(isInt = false) },
            "frame" to { h -> h.roi(frame = 1) },
            "bad target" to { h -> h.roi(latDeg = 0.0, lonDeg = 0.0) },
            "roi mode" to { h ->
                h.roi(command = RoiCommand.MAV_CMD_DO_SET_ROI, isInt = false, param1 = 2f)
            },
        )
        for ((name, refusal) in refusals) {
            val h = Harness()
            assertEquals(name, Verdict.DENIED, refusal(h))
            assertTrue("$name produced no STATUSTEXT", h.texts().isNotEmpty())
            assertTrue("$name produced no roi_denied event", h.events.any { it.first == "roi_denied" })
            for (text in h.texts()) {
                assertTrue(
                    "$name said ${text.toByteArray(Charsets.UTF_8).size} bytes: $text",
                    text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
                )
            }
        }
        // The no-camera refusal needs its own harness, and is owed the same sentence.
        val noCam = Harness(withGimbal = false)
        assertEquals(Verdict.DENIED, noCam.roi())
        assertTrue(noCam.texts().isNotEmpty())
    }

    @Test
    fun `every ROI sentence fits the 50-byte STATUSTEXT field`() {
        for (text in listOf(
            GuidedStatusTexts.ROI_STARTED,
            GuidedStatusTexts.ROI_GROUND_LEVEL,
            GuidedStatusTexts.ROI_CLEARED,
            GuidedStatusTexts.ROI_PITCH_ONLY,
            GuidedStatusTexts.ROI_TOO_CLOSE,
            GuidedStatusTexts.ROI_GIMBAL_RANGE,
            GuidedStatusTexts.ROI_NO_HEADING,
            GuidedStatusTexts.ROI_TRACKING_STOPPED,
            GuidedStatusTexts.roiRefused(GuidedStatusTexts.REASON_ROI_NO_GIMBAL),
            GuidedStatusTexts.roiRefused(GuidedStatusTexts.REASON_ROI_LONG_FORM),
            GuidedStatusTexts.roiRefused(GuidedStatusTexts.REASON_ROI_MODE),
            GuidedStatusTexts.roiRefused(GuidedStatusTexts.REASON_BAD_TARGET),
        )) {
            assertTrue(
                "${text.toByteArray(Charsets.UTF_8).size} bytes: $text",
                text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
            )
        }
    }

    // ------------------------------------------------------ clearing is always allowed

    @Test
    fun `DO_SET_ROI_NONE is accepted in every state, in both wire shapes`() {
        // Turning something off is always allowed — including with nothing set, with the interlock
        // off, mid-orbit, and as a COMMAND_LONG, whose shape is refused for *setting*.
        assertEquals(Verdict.ACCEPTED, Harness().roiNone())
        assertEquals(Verdict.ACCEPTED, Harness().roiNone(isInt = false))
        val off = Harness()
        off.interlock = false
        assertEquals(Verdict.ACCEPTED, off.roiNone())

        val flying = Harness()
        flying.roi()
        flying.orbit()
        flying.confirm()
        assertEquals(Verdict.ACCEPTED, flying.roiNone())
        assertTrue(flying.texts().contains(GuidedStatusTexts.ROI_CLEARED))
        assertTrue(flying.events.any { it.first == "roi_cleared" })
        // The orbit is untouched: clearing an ROI is not cancelling a manoeuvre.
        flying.tickAlive()
        assertEquals(GuidedPhase.ENGAGED, flying.engine.phase)
        assertEquals(GuidedStickEngine.ORBIT_SOURCE, flying.cmds.last().source?.messageName)
    }

    // ---------------------------------------------------------------- the camera

    @Test
    fun `the camera is aimed at the hand-computed angle, at several ranges and heights`() {
        // −atan2(height above our datum, horizontal distance) — both quantities ours, the target
        // assumed to be on the ground at the takeoff datum.
        val cases = listOf(
            Triple(10.0, 10.0, -45.0),
            Triple(20.0, 10.0, -26.56505117707799),
            Triple(10.0, 20.0, -63.43494882292201),
            Triple(50.0, 5.0, -5.710593137499643),
        )
        for ((north, alt, expected) in cases) {
            val h = Harness()
            h.roi()
            h.place(latDeg = latNorthOf(north), relAlt = alt)
            h.tickAlive()
            assertEquals("north=$north alt=$alt", expected, h.gimbal.aimed.last(), 1e-6)
        }
    }

    @Test
    fun `the solution is clamped to DJI's own reported range, and the operator is told once`() {
        val h = Harness()
        // The measured travel on this airframe is −90..+60; a shallower gimbal cannot look 45° down.
        h.gimbal.range = -20.0..60.0
        h.roi()
        h.tickAlive()
        assertEquals(-20.0, h.gimbal.aimed.last(), 1e-12)
        assertEquals(1, h.said(GuidedStatusTexts.ROI_GIMBAL_RANGE))
        // Held, not repeated, however long the condition lasts.
        repeat(200) { h.tickAlive() }
        assertEquals(1, h.said(GuidedStatusTexts.ROI_GIMBAL_RANGE))
        // …and re-armed when the target comes back inside the travel, because that is news too.
        h.place(latDeg = latNorthOf(60.0))
        repeat(10) { h.tickAlive() }
        assertNotEquals(-20.0, h.gimbal.aimed.last())
        h.place(latDeg = latNorthOf(START_NORTH))
        repeat(10) { h.tickAlive() }
        assertEquals(2, h.said(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `with no reported range nothing is clamped and nothing is invented`() {
        val h = Harness()
        h.gimbal.range = null
        h.roi()
        h.tickAlive()
        assertEquals(-45.0, h.gimbal.aimed.last(), 1e-6)
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `inside the minimum range the camera holds its last angle and says so once`() {
        val h = Harness()
        h.roi()
        h.place(latDeg = latNorthOf(20.0))
        h.tickAlive()
        val held = h.gimbal.aimed.last()
        // Now inside the floor: the solution runs to −90° and swings with every metre of GPS noise.
        h.place(latDeg = latNorthOf(1.0))
        repeat(200) { h.tickAlive() }
        assertEquals("the camera hunted at close range", held, h.gimbal.aimed.last(), 1e-12)
        assertEquals(1, h.said(GuidedStatusTexts.ROI_TOO_CLOSE))
        // Backing off resumes tracking, and re-arms the sentence.
        h.place(latDeg = latNorthOf(20.0))
        repeat(10) { h.tickAlive() }
        h.place(latDeg = latNorthOf(1.0))
        repeat(10) { h.tickAlive() }
        assertEquals(2, h.said(GuidedStatusTexts.ROI_TOO_CLOSE))
    }

    @Test
    fun `the rate limiter holds the camera to one command per interval, however fast the solution moves`() {
        val h = Harness()
        h.roi()
        h.tickAlive()
        val first = h.gimbal.aimed.size
        // Walk the aircraft away from the target, changing the solution by far more than the
        // deadband on every 100 ms tick. An ROI's solution moves continuously — unlike an orbit's,
        // which is constant on a steady circle — so this limiter is what stops ten `performAction`s
        // a second going at DJI over the link that also carries our setpoints.
        for (step in 1..20) {
            h.place(latDeg = latNorthOf(START_NORTH + step * 3.0))
            h.tickAlive()
        }
        // 2 s of ticking at 500 ms minimum spacing: four more commands, not twenty.
        assertEquals(4, h.gimbal.aimed.size - first)
    }

    @Test
    fun `the deadband holds the camera still for a solution that has barely moved`() {
        val h = Harness()
        h.roi()
        h.tickAlive()
        val first = h.gimbal.aimed.size
        // 0.2 mm of drift per tick at 10 m of range — a *total* of 0.12 m over a minute, which
        // moves the solution by a third of a degree. The deadband compares against the last angle we
        // commanded, not against the last tick, so this is the honest form of "barely moved": the
        // camera is already pointing there and must not be told again.
        for (step in 1..600) {
            h.place(latDeg = latNorthOf(START_NORTH + step * 0.0002))
            h.tickAlive()
        }
        assertEquals(first, h.gimbal.aimed.size)
    }

    @Test
    fun `a stale position fix stops the camera - absence, never a default angle`() {
        val h = Harness()
        h.roi()
        h.tickAlive()
        val aimed = h.gimbal.aimed.size
        // The fix ages out while the aircraft is somewhere else entirely. A cached fix is never
        // aimed on, and the camera is never swung to a default — a default is a lie about where
        // the target is.
        h.place(latDeg = latNorthOf(80.0), positionAge = 5_000L)
        repeat(100) { h.tickAlive() }
        assertEquals(aimed, h.gimbal.aimed.size)
        // …and it resumes the moment the feed does.
        h.place(latDeg = latNorthOf(80.0))
        h.tickAlive()
        assertTrue(h.gimbal.aimed.size > aimed)
    }

    @Test
    fun `no usable altitude stops the camera - there is no honest pointing solution without one`() {
        val h = Harness()
        h.roi()
        h.tickAlive()
        val aimed = h.gimbal.aimed.size
        h.place(latDeg = latNorthOf(40.0), relAlt = null)
        repeat(100) { h.tickAlive() }
        assertEquals(aimed, h.gimbal.aimed.size)
    }

    @Test
    fun `a camera that will not aim costs neither the aircraft nor the manoeuvre`() {
        val h = Harness()
        h.gimbal.throwOnAim = IllegalStateException("gimbal gone")
        h.roi()
        h.goto()
        h.confirm()
        repeat(20) { h.tickAlive() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.port.sent.isNotEmpty())
    }

    // ------------------------------------------------- the yaw, and who is flying

    @Test
    fun `while a goto of ours is flying, the nose is slaved to the ROI bearing`() {
        val h = Harness()
        h.roi()
        // 40 m north of the ROI (so it bears due south) with the nose 10° past it, flying further
        // north — a path straight along the line of sight, so the feed-forward is exactly zero and
        // what is left is the proportional term alone.
        h.place(latDeg = latNorthOf(40.0), yawDeg = 190.0)
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(90.0)))
        h.confirm()
        h.tickAlive()
        assertEquals(-10.0, h.port.sent.last().yaw, 1e-6)
        // The translation is the goto's own, untouched: an ROI changes where the camera and the
        // nose point and **nothing about the flight path**.
        assertTrue(h.port.sent.last().roll > 0.0) // roll drives NORTH (measured 2026-07-26)
        assertEquals(GuidedStickEngine.REPOSITION_SOURCE, h.cmds.last().source?.messageName)
    }

    @Test
    fun `THE LINE - nothing of ours flying means pitch only, and we say so`() {
        // §9.3, and the rule this suite guards hardest: we do not yaw an aircraft somebody else is
        // flying. Nothing of ours is in progress here, so the camera tilts and the airframe is left
        // alone — with the reason on the wire, because the operator's subject will be off to one
        // side and they are owed an explanation.
        val h = Harness()
        h.roi()
        h.place(latDeg = latNorthOf(40.0), yawDeg = 90.0) // nose east, target due south: 90° off
        repeat(50) { h.tickAlive() }
        assertTrue(h.port.sent.isEmpty()) // nothing is being commanded at all
        assertTrue(h.gimbal.aimed.isNotEmpty()) // …except the camera
        assertEquals(1, h.said(GuidedStatusTexts.ROI_PITCH_ONLY))
    }

    @Test
    fun `THE LINE - stick passthrough never gets an ROI yaw`() {
        // The operator's own hand is on the yaw axis during passthrough — Stage A relays their `r`
        // to a yaw rate — so an ROI yaw here would be this bridge overruling the person flying
        // rather than flying on their behalf.
        val h = Harness()
        h.roi()
        h.place(latDeg = latNorthOf(40.0), yawDeg = 90.0)
        h.frame() // neutral, so the stream proves its convention
        h.now += 40
        h.frame(x = 500) // a deliberate deflection engages
        h.confirm()
        repeat(20) { h.tickAlive() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.port.sent.isNotEmpty())
        for (sent in h.port.sent) {
            assertEquals("an ROI yawed an aircraft the operator was flying", 0.0, sent.yaw, 1e-12)
        }
        assertTrue(h.said(GuidedStatusTexts.ROI_PITCH_ONLY) >= 1)
    }

    @Test
    fun `a stale heading commands zero yaw and says so, and the goto carries on`() {
        val h = Harness()
        h.roi()
        h.place(latDeg = latNorthOf(40.0), yawDeg = 190.0, attitudeAge = 5_000L)
        h.goto(latDeg = latNorthOf(90.0))
        h.confirm()
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_NO_HEADING))
        assertTrue(h.port.sent.last().roll > 0.0) // still flying the leg
    }

    @Test
    fun `the ROI yaw never leaves the 30 deg per second envelope`() {
        val h = Harness()
        h.roi()
        h.goto(latDeg = latNorthOf(90.0))
        h.confirm()
        // Every heading, including the 180° singularity where the loop must pick a side.
        for (step in 0 until 72) {
            h.place(latDeg = latNorthOf(40.0), yawDeg = step * 5.0)
            h.tickAlive()
        }
        for (sent in h.port.sent) {
            assertTrue(
                "yaw ${sent.yaw} left the envelope",
                abs(sent.yaw) <= GuidedEnvelope.YAW_RATE_MAX_DEGS + 1e-9,
            )
        }
    }

    @Test
    fun `inside the minimum range the aircraft is not turned either`() {
        val h = Harness()
        h.roi()
        // 1 m from the target with the nose 90° off: the loop would ask for the yaw cap, and the
        // aircraft would pirouette under the operator's subject.
        h.place(latDeg = latNorthOf(1.0), yawDeg = 90.0)
        h.goto(latDeg = latNorthOf(60.0))
        h.confirm()
        repeat(10) { h.tickAlive() }
        for (sent in h.port.sent) {
            assertEquals(0.0, sent.yaw, 1e-12)
        }
    }

    @Test
    fun `a goto holding station after arrival still points the nose at the ROI`() {
        // "Fly there and look at the thing" is the case this exists for: the translation is zero,
        // the aircraft is still ours, and the nose swings onto the target.
        val h = Harness()
        h.roi()
        h.place(latDeg = latNorthOf(40.0), yawDeg = 190.0)
        h.goto(latDeg = latNorthOf(40.0)) // already there
        h.confirm()
        repeat(RepositionGuidance.ARRIVE_TICKS + 3) { h.tickAlive() }
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_ARRIVED))
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.roll, 1e-12)
        assertEquals(0.0, sent.pitch, 1e-12)
        assertEquals(-10.0, sent.yaw, 1e-6)
    }

    // -------------------------------------------- the ROI outranks an orbit's centre

    @Test
    fun `an explicit ROI replaces the orbit's own centre, and clearing it stops the tracking`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        // Circling, camera on the centre: −atan2(10, 20).
        assertEquals(-26.56505117707799, h.gimbal.aimed.last(), 1e-6)
        val nose = h.port.sent.last().yaw

        // Now the operator clicks somewhere else — 60 m east of the centre. The circle is unchanged
        // and the camera is not.
        val beforeRoi = h.gimbal.aimed.size
        h.roi(lonDeg = lonEastOf(60.0))
        h.placeOnCircle(0.0)
        repeat(10) { h.tickAlive() }
        val toRoi = h.gimbal.aimed.last()
        // **Not once** back to the centre after the click. Two things aiming one camera is not a
        // draw the ROI wins on the last tick — it is a camera whipping between the two every rate
        // window, and the operator watching the subject leave frame twice a second.
        for (aim in h.gimbal.aimed.drop(beforeRoi)) {
            assertTrue(
                "the camera went back to the orbit's centre at $aim",
                abs(aim - -26.56505117707799) > 1e-3,
            )
        }
        assertNotEquals(-26.56505117707799, toRoi, 1e-6)
        // sqrt(20² + 60²) = 63.2456 m out, 10 m up.
        assertEquals(-Math.toDegrees(Math.atan2(10.0, 63.245553203367585)), toRoi, 1e-3)
        // …and the nose is on the ROI's bearing rather than the centre's, which is a different
        // direction entirely (the loop saturates swinging to it).
        assertNotEquals(nose, h.port.sent.last().yaw, 1e-6)
        // A hard swing the other way. Asserted as a fraction of the envelope rather than as
        // saturation: at the old 30 °/s clamp this geometry pinned the loop, and at 90 it no
        // longer does — which is the point of raising it, and not something for a test to hide.
        assertTrue(h.port.sent.last().yaw < -0.5 * GuidedEnvelope.YAW_RATE_MAX_DEGS)

        // **Cancel means cancel** (Ivan, 2026-07-27, superseding the fall-back-to-the-centre rule
        // this test used to assert). Clearing stops the camera tracking altogether rather than
        // peeling off the operator's layer and leaving the circle's underneath: two layers you have
        // to press through twice is the kind of cleverness that surprises a pilot mid-flight.
        val aimedBefore = h.gimbal.aimed.size
        h.roiNone()
        h.placeOnCircle(0.0)
        repeat(10) { h.tickAlive() }
        assertEquals("the camera kept being driven after a cancel", aimedBefore, h.gimbal.aimed.size)
        assertEquals("the nose kept being turned after a cancel", 0.0, h.port.sent.last().yaw, 1e-9)
    }

    // ------------------------------------------------------------- the abort ladder

    @Test
    fun `an abort stops the camera being driven and says so - and the target is remembered`() {
        val h = Harness()
        h.roi()
        h.goto(latDeg = latNorthOf(60.0))
        h.confirm()
        h.place(latDeg = latNorthOf(40.0))
        repeat(5) { h.tickAlive() }
        val aimed = h.gimbal.aimed.size
        assertTrue(aimed > 0)

        h.interlock = false
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_TRACKING_STOPPED))
        assertTrue(h.events.any { it.first == "roi_cleared" && it.second!!.contains("interlock") })

        // No further gimbal commands, however far the aircraft moves. A camera slewing during an
        // abort is noise at the moment the operator least needs it, and an angle commanded after
        // handback is a command the now-flying RC pilot did not ask for.
        for (step in 1..40) {
            h.place(latDeg = latNorthOf(40.0 + step * 2.0))
            h.tickAlive()
        }
        assertEquals("the camera kept slewing after an abort", aimed, h.gimbal.aimed.size)
    }

    @Test
    fun `the remembered ROI is re-acquired when authority is confirmed again`() {
        val h = Harness()
        h.roi()
        h.goto(latDeg = latNorthOf(60.0))
        h.confirm()
        h.tickAlive()
        h.interlock = false
        h.tickAlive()
        val aimed = h.gimbal.aimed.size

        h.interlock = true
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS + 100
        h.place(latDeg = latNorthOf(40.0), yawDeg = 190.0)
        h.goto(latDeg = latNorthOf(80.0))
        h.confirm()
        repeat(5) { h.tickAlive() }
        assertTrue("the remembered ROI was not re-acquired", h.gimbal.aimed.size > aimed)
        // …including its yaw half, because the manoeuvre is ours again.
        assertEquals(-10.0, h.port.sent.last().yaw, 1e-6)
    }

    @Test
    fun `DO_SET_ROI_NONE clears a suspended ROI too, and nothing re-acquires it`() {
        val h = Harness()
        // Heading-follows-course off, so the yaw assertion at the end reads the *ROI's* contribution
        // and nothing else: with it on the leg turns its own nose toward the target, which is
        // correct and is a different property, pinned in `GuidedOrbitTest`.
        h.headingFollows = false
        h.roi()
        h.goto(latDeg = latNorthOf(60.0))
        h.confirm()
        h.tickAlive()
        h.interlock = false
        h.tickAlive()
        assertEquals(Verdict.ACCEPTED, h.roiNone())
        val aimed = h.gimbal.aimed.size

        h.interlock = true
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS + 100
        h.place(latDeg = latNorthOf(40.0))
        h.goto(latDeg = latNorthOf(80.0))
        h.confirm()
        repeat(10) { h.tickAlive() }
        assertEquals("a cleared ROI came back", aimed, h.gimbal.aimed.size)
        for (sent in h.port.sent.takeLast(10)) {
            assertEquals(0.0, sent.yaw, 1e-12)
        }
    }

    // ------------------------------------------------------------- the open-loop rule

    @Test
    fun `THE OPEN-LOOP RULE - the seam the ROI aims through cannot express an attitude or an age`() {
        // The fourth appearance of the change-driven-key trap, and the reason it is structural
        // rather than remembered: `KeyGimbalAttitude` goes silent when the gimbal is motionless and
        // DJI refuses a direct `get` on it, so "the attitude is old" and "the camera has not moved"
        // are the same bytes — and an ROI held on a hovering aircraft is exactly that case.
        val declared = ManoeuvreGimbal::class.java.declaredMethods.map { it.name }.toSet()
        assertEquals(setOf("pitchRangeDeg", "aimPitch"), declared)
        for (name in declared) {
            assertFalse(
                "the seam grew a method that could carry feedback: $name",
                name.contains("age", ignoreCase = true) ||
                    name.contains("attitude", ignoreCase = true) ||
                    name.contains("reading", ignoreCase = true) ||
                    name.contains("current", ignoreCase = true),
            )
        }
    }

    @Test
    fun `THE OPEN-LOOP RULE - a camera that reports nothing at all is still commanded, forever`() {
        // The behavioural half. The fake reports no attitude, ever — as a healthy stabilised gimbal
        // does — and the ROI must go on issuing absolute angles rather than deciding the camera is
        // dead and giving up on it.
        val h = Harness()
        h.roi()
        for (step in 1..1_200) {
            h.place(latDeg = latNorthOf(START_NORTH + step * 0.05))
            h.tickAlive()
        }
        assertTrue("the camera stopped being commanded", h.gimbal.aimed.size > 20)
    }
}
