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
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * M3 Stage C — `DO_ORBIT` through [GuidedStickEngine.orbit]: the truthful ack, every refusal and
 * its sentence, the two phases, the one scoped yaw exception, the open-loop gimbal, and every
 * Stage A abort applied mid-circle. Same protocol as `GuidedRepositionTest`: fake port, fake
 * gimbal, hand-cranked clock, no aircraft.
 *
 * The pure arithmetic — the curvature cap as a property, the `cos(latitude)` join point, the
 * ±180° wrap, the signs, the yaw law, the gimbal solution — is pinned next door in
 * `OrbitGuidanceTest`. This file is about what the *engine* does with it.
 *
 * Written to fail loudly for the Stage C landmines:
 *
 *  - a radius outside the band **clamped instead of refused** — a clamped circle is a different
 *    circle, drawn somewhere the operator did not click
 *  - a refusal with no sentence, which is the exact gap measured on 2026-07-27: `DO_ORBIT` was
 *    answered with a bare `MAV_RESULT_UNSUPPORTED` and no `STATUSTEXT` at all
 *  - **yaw leaking out of the orbit's circling branch** into passthrough, into a plain
 *    reposition, or into the orbit's own resting legs
 *  - a **stale heading guessed at** instead of commanding zero yaw
 *  - the orbit path **consulting gimbal attitude or its age** — the change-driven-key trap, which
 *    on a steady orbit is exactly when that key goes silent
 *  - the swept angle completing early because the ±180° wrap was dropped
 *  - a circle taken whose far side leaves the Q1 leg bound
 *  - an orbit surviving an abort, an engage refusal, or a pause
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, run,
 * confirmed red and reverted. Counts are failing tests across the five guided suites
 * (`GuidedOrbitTest`, `OrbitGuidanceTest`, `GuidedRepositionTest`, `RepositionGuidanceTest`,
 * `GuidedStickEngineTest`) — **measured, not estimated**.
 *
 * The arithmetic ([OrbitGuidance], and the two functions [RepositionGuidance] grew for it):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | curvature cap dropped (tangential bounded only by the Q1 envelope) | 6 |
 *  | curvature cap relaxed to `sqrt(2·a_max·R)` | 6 |
 *  | tangential speed stepped to the cap instead of ramped | 4 |
 *  | `cos(latitude)` dropped from the join point's east offset | 6 |
 *  | the ±180° wrap dropped from the swept-angle accumulator | 8 |
 *  | tangential rotation flipped (a clockwise circle flown anticlockwise) | 7 |
 *  | radial correction sign flipped (pushes away from the circle) | 4 |
 *  | radial cap raised to the full horizontal envelope | 5 |
 *  | join point taken on the far side of the circle | 24 |
 *  | a stale heading guessed at zero instead of commanding no yaw | 4 |
 *  | the 30 °/s yaw clamp removed | 3 |
 *  | yaw feed-forward direction sign dropped | 2 |
 *  | gimbal pitch sign flipped (the camera aimed at the sky) | 7 |
 *  | the M3 arrival test reduced to its distance conjunct alone | 5 |
 *
 * The engine ([GuidedStickEngine.orbit] and the orbit tick):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `ACCEPTED` returned before any validation | 61 |
 *  | datum subtraction dropped (the absolute AMSL trusted) | 23 |
 *  | refusals answer `DENIED` with **no sentence** | 12 |
 *  | radius clamped into the band instead of refused | 2 |
 *  | the leg bound checks only the centre, not the whole circle | 1 |
 *  | the ceiling cap removed at accept | 1 |
 *  | yaw leaks into the orbit's resting legs | 2 |
 *  | the heading's freshness gate removed (a stale `yawDeg` flown on) | 2 |
 *  | the gimbal seam grows an attitude-age accessor (the closed-loop door) | 1 |
 *  | the gimbal rate limit removed (commanded every tick) | 1 |
 *  | the gimbal range clamp removed (an unreachable angle commanded silently) | 1 |
 *  | the `ORBIT_MAX_S` time cap removed | 1 |
 *  | a stale position fix circled on (the cached fix) | 1 |
 *  | a deliberate stick deflection no longer cancels the orbit | 2 |
 *  | Pause no longer ends the orbit | 2 |
 *  | the orbit survives an abort | 1 — see below |
 *  | swept counter not zeroed at `CIRCLE` entry (one layer alone) | **0 — alive on purpose** |
 *  | swept counter's field initialiser made stale (the other layer alone) | **0 — alive on purpose** |
 *  | **both** swept-counter layers removed at once | 5 |
 *
 * ### The two results worth reading rather than counting
 *
 * **The survives-abort mutant scored 0 on the first pass**, exactly as its Stage B twin did, and
 * for the same reason: a lingering circle cannot actually fly, because the deflection that
 * re-engages is itself deliberate and fires the stick-interrupt, which clears it — defence in
 * depth masking the primary clear. The observable difference is this bridge *believing* it still
 * had a circle, which shows up as a spurious `orbit_ended replaced` when the next one is
 * commanded. `an aborted circle is gone, not lying in ambush for the next engagement` asserts its
 * absence, and the mutant dies. The layering is kept; the test pins the primary layer alone.
 *
 * **The swept counter is genuinely two layers, and each masks the other exactly.** A circle is
 * always a fresh `OrbitState` whose counter starts at zero, *and* entering `CIRCLE` zeroes it
 * again. Break either alone and the other still gives the right answer, so both mutants live —
 * on purpose, and recorded rather than removed, because the reset at `CIRCLE` entry is what will
 * still be right the day a resume path exists. Removing **both** is caught by 5 tests, which is
 * the measurement that shows the property is tested at all.
 */
class GuidedOrbitTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        /** The published pressure-altitude datum this session. Another session saw it 41.5 m away. */
        const val DATUM = 100.0

        /** Default aircraft height above the datum. */
        const val ALT = 10.0

        /** The default circle: centred on (LAT, LON), this many metres of radius. */
        const val RADIUS = 20.0

        /** Where the aircraft starts: due north of the centre, outside the circle. */
        const val START_NORTH = 40.0

        /** Nose due south — pointed straight at the centre from the starting position. */
        const val NOSE_AT_CENTRE = 180.0

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
    private class FakeManoeuvreGimbal : ManoeuvreGimbal {
        var range: ClosedFloatingPointRange<Double>? = null
        var rangeCalls = 0
        var throwOnAim: Throwable? = null
        val aimed = mutableListOf<Double>()

        override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? {
            rangeCalls++
            return range
        }

        override fun aimPitch(pitchDeg: Double) {
            throwOnAim?.let { throw it }
            aimed += pitchDeg
        }
    }

    private class RecordedCmd(
        val setpoint: Setpoint?, val axes: StickAxes, val source: CommandSource?,
        val accepted: Boolean?, val error: String?,
    )

    private class Harness(policy: LinkLossPolicy = LinkLossPolicy.SHIPPED) {
        var now = 1_000L
        var interlock = true

        /**
         * Heading-follows-course, defaulting on as the shipped flag does. Turning it off is the way
         * back to the flight-verified "keep heading" behaviour, and the tests below use it to pin
         * that the way back still exists.
         */
        var headingFollows = true
        var state = stateAt()
        val port = FakeVirtualStickPort()
        val gimbal = FakeManoeuvreGimbal()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            headingFollowsCourse = { headingFollows },
            manoeuvreGimbal = gimbal,
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
            policy = policy,
            nowMs = { now },
        )

        init {
            engine.attach()
            port.onRc!!(RcSticks(0, 0, 0, 0))
        }

        fun texts(): List<String> = wire.filterIsInstance<Statustext>().map { it.text() }

        /** The aircraft as the encoder would snapshot it — everything fresh unless said otherwise. */
        fun place(
            latDeg: Double = latNorthOf(START_NORTH), lonDeg: Double = LON, relAlt: Double? = ALT,
            vn: Double = 0.0, ve: Double = 0.0, vd: Double = 0.0,
            positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
            attitudeAge: Long = 0L, yawDeg: Double? = NOSE_AT_CENTRE,
            datum: Double? = DATUM, flightMode: String? = null,
        ) {
            state = stateAt(
                latDeg, lonDeg, relAlt, vn, ve, vd,
                positionAge, altitudeAge, velocityAge, attitudeAge, yawDeg, datum, flightMode,
            )
        }

        /** Put the aircraft on the circle at [bearingDeg] from the centre, at rest. */
        fun placeOnCircle(bearingDeg: Double, radius: Double = RADIUS, relAlt: Double? = ALT) {
            val rad = Math.toRadians(bearingDeg)
            val (lat, lon) = RepositionGuidance.offsetCoordinate(
                LAT, LON, radius * kotlin.math.cos(rad), radius * kotlin.math.sin(rad),
            )
            // The nose-at-centre heading from a point bearing β from the centre is β + 180.
            place(latDeg = lat, lonDeg = lon, relAlt = relAlt, yawDeg = bearingDeg + 180.0)
        }

        fun orbit(
            centreLatDeg: Double = LAT, centreLonDeg: Double = LON,
            radius: Float = RADIUS.toFloat(), zAmsl: Double = DATUM + ALT,
            frame: Int = 0, velocity: Float = Float.NaN,
            yawBehaviour: Float = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat(),
            turns: Float = Float.NaN, isInt: Boolean = true,
            origin: ControlOrigin = ControlOrigin.MAVLINK,
        ): Verdict = engine.orbit(
            OrbitCommand(
                isCommandInt = isInt, frame = frame,
                radiusM = radius, velocityMs = velocity,
                yawBehaviour = yawBehaviour, turns = turns,
                latE7 = (centreLatDeg * 1e7).roundToInt(), lonE7 = (centreLonDeg * 1e7).roundToInt(),
                zAmslM = zAmsl.toFloat(),
            ),
            origin,
        )

        fun goto(
            latDeg: Double = latNorthOf(50.0), lonDeg: Double = LON, zAmsl: Double = DATUM + ALT,
        ): Verdict = engine.reposition(
            RepositionCommand(
                isCommandInt = true, frame = 0,
                latE7 = (latDeg * 1e7).roundToInt(), lonE7 = (lonDeg * 1e7).roundToInt(),
                zAmslM = zAmsl.toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
            )
        )

        fun pause(): Verdict = engine.reposition(
            RepositionCommand(
                isCommandInt = false, frame = 0, latE7 = 0, lonE7 = 0,
                zAmslM = Float.NaN, groundSpeedMs = -1f, yawRad = Float.NaN,
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

        /** One engine tick with the link silent. */
        fun tick(advanceMs: Long = 0) {
            now += advanceMs
            engine.tick(now)
        }

        fun frame(x: Int = 0, y: Int = 0, z: Int = 500, r: Int = 0) {
            engine.onInbound(ManualControl.builder().target(1).x(x).y(y).z(z).r(r).buttons(0).build(), null)
        }

        /**
         * Accept the default circle, get DJI's confirmation, then teleport onto the join point and
         * hold still until the M3 arrival test fires — i.e. the whole join leg, without simulating
         * the aircraft's dynamics, which is not what this suite is about.
         */
        fun reachCircle(): Harness {
            orbit()
            confirm()
            placeOnCircle(0.0)
            repeat(RepositionGuidance.ARRIVE_TICKS) { tickAlive() }
            return this
        }

        companion object {
            fun stateAt(
                latDeg: Double = latNorthOf(START_NORTH), lonDeg: Double = LON, relAlt: Double? = ALT,
                vn: Double = 0.0, ve: Double = 0.0, vd: Double = 0.0,
                positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
                attitudeAge: Long = 0L, yawDeg: Double? = NOSE_AT_CENTRE,
                datum: Double? = DATUM, flightMode: String? = null,
            ) = AircraftState(
                latitude = latDeg, longitude = lonDeg,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = datum,
                velocityNorth = vn, velocityEast = ve, velocityDown = vd,
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
    fun `an orbit while idle is ACCEPTED, engages, and flies only after DJI confirms`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.orbit())
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_STARTED))
        assertTrue(h.events.any { it.first == "orbit_accepted" })
        // Nothing may flow before DJI's own state confirms authority.
        h.tickAlive()
        assertEquals(0, h.port.sent.size)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ENGAGED_ORBIT))
    }

    @Test
    fun `an orbit reuses an existing stick engagement - one enable, one owner`() {
        val h = Harness()
        h.frame() // neutral, at rest
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertEquals(1, h.port.enableCalls)
        assertEquals(Verdict.ACCEPTED, h.orbit())
        assertEquals(1, h.port.enableCalls)
    }

    // ------------------------------------------- refusals, and every one has a sentence

    @Test
    fun `the interlock off answers UNSUPPORTED with no sentence - the pre-feature reply, byte for byte`() {
        val h = Harness()
        h.interlock = false
        assertEquals(Verdict.UNSUPPORTED, h.orbit())
        // Deliberately silent: QGC raises its own modal for an unsupported command, and the
        // difference between "commands are off" and "this circle was refused" is the whole point.
        assertTrue(h.texts().isEmpty())
        assertFalse(h.events.any { it.first == "orbit_accepted" })
    }

    @Test
    fun `a radius outside the band is REFUSED, not clamped`() {
        for (radius in listOf(4.9f, 50.1f, 0f, Float.NaN, 200f)) {
            val h = Harness()
            assertEquals("radius $radius", Verdict.DENIED, h.orbit(radius = radius))
            // Refused means *nothing was taken*: no engagement, no circle, no event.
            assertEquals("radius $radius", GuidedPhase.IDLE, h.engine.phase)
            assertEquals("radius $radius", 0, h.port.enableCalls)
            assertFalse("radius $radius", h.events.any { it.first == "orbit_accepted" })
            // …and the operator is told which limit, by number.
            assertTrue(
                "radius $radius said: ${h.texts()}",
                h.texts().any { it.contains("5-50m") },
            )
        }
    }

    @Test
    fun `the radius band is inclusive at both ends`() {
        assertEquals(Verdict.ACCEPTED, Harness().orbit(radius = OrbitGuidance.R_MIN_M.toFloat()))
        // A 50 m circle needs the aircraft near its centre to fit the Q1 leg bound, so this one
        // starts on the circle rather than 40 m north of a small one.
        val h = Harness()
        h.place(latDeg = LAT, lonDeg = LON)
        assertEquals(Verdict.ACCEPTED, h.orbit(radius = OrbitGuidance.R_MAX_M.toFloat()))
    }

    @Test
    fun `a negative radius is the counter-clockwise direction, not an invalid radius`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.orbit(radius = -RADIUS.toFloat()))
        assertTrue(h.events.any { it.first == "orbit_accepted" && it.second!!.contains("dir=-1") })
    }

    @Test
    fun `a finite tangential velocity is refused exactly as a ground-speed request is`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(velocity = 5f))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_SPEED) })
        // QGC's own "vehicle default" is NaN, and −1 is the other default spelling: both fly.
        assertEquals(Verdict.ACCEPTED, Harness().orbit(velocity = Float.NaN))
        assertEquals(Verdict.ACCEPTED, Harness().orbit(velocity = -1f))
    }

    @Test
    fun `a COMMAND_LONG orbit is refused as an unmeasured shape`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(isInt = false))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_ORBIT_LONG_FORM) })
    }

    @Test
    fun `a frame other than the measured MAV_FRAME_GLOBAL is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(frame = 6))
        assertTrue(h.texts().any { it.contains("FRAME_6") })
    }

    @Test
    fun `no published datum refuses the orbit rather than guessing at the altitude`() {
        val h = Harness()
        h.place(datum = null)
        assertEquals(Verdict.DENIED, h.orbit())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_NO_DATUM) })
    }

    @Test
    fun `a stale or absent position fix refuses the orbit - there is nothing to join from`() {
        val h = Harness()
        h.place(positionAge = 5_000L)
        assertEquals(Verdict.DENIED, h.orbit())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_NO_FIX) })
    }

    @Test
    fun `a centre below the takeoff datum is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(zAmsl = DATUM - 5.0))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_BELOW_DATUM) })
    }

    @Test
    fun `an altitude above the ceiling is capped and announced, not refused`() {
        // The JC-5 reading Stage B already applies to a goto: the lateral intent is honoured and
        // the operator is told exactly what was not. The circle is still the circle they drew.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.orbit(zAmsl = DATUM + 150.0))
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_CAPPED))
        assertTrue(
            h.events.any {
                it.first == "orbit_accepted" && it.second!!.contains("relAlt=%.1f".format(GuidedEnvelope.CEILING_M))
            }
        )
    }

    @Test
    fun `a circle whose far side leaves the Q1 leg bound is refused, centre alone is not enough`() {
        // A centre 15 m inside the bound with a 20 m radius reaches 5 m past it — bounding only the
        // centre would let this fly. Derived from the bound (2 km since 2026-07-30) rather than
        // written out, so the *property* is what is tested and not the number that was in force.
        val centre = GuidedEnvelope.MAX_REPOSITION_DISTANCE_M - 15.0
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(centreLatDeg = latNorthOf(START_NORTH + centre)))
        assertTrue(h.texts().any { it.contains("beyond") || it.contains("circle") })
        // …and the same centre with a radius that fits is taken.
        val ok = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            ok.orbit(centreLatDeg = latNorthOf(START_NORTH + centre), radius = 10f),
        )
    }

    @Test
    fun `a centre that is not a coordinate is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.orbit(centreLatDeg = 0.0, centreLonDeg = 0.0))
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_BAD_TARGET) })
    }

    @Test
    fun `no RC stick feed refuses the orbit - abort gesture 1 would be blind`() {
        val h = Harness()
        h.port.onRc!!(RcSticks(null, null, null, null))
        assertEquals(Verdict.DENIED, h.orbit())
        assertTrue(h.texts().any { it.contains("NO_RC_FEED") })
    }

    @Test
    fun `an SDK that cannot be asked refuses the orbit, with DJI's own word`() {
        val h = Harness()
        h.port.unavailable = "NO_PRODUCT"
        assertEquals(Verdict.DENIED, h.orbit())
        assertTrue(h.texts().any { it.contains("NO_PRODUCT") })
    }

    @Test
    fun `EVERY orbit refusal carries a sentence - the 2026-07-27 gap, closed`() {
        // The measured gap: `DO_ORBIT` answered `MAV_RESULT_UNSUPPORTED` with no `STATUSTEXT`, the
        // only refusal in this project that never said why. Enumerated here so a new refusal path
        // added without a sentence fails rather than being noticed on a bench log.
        val refusals: List<Pair<String, (Harness) -> Verdict>> = listOf(
            "long form" to { h -> h.orbit(isInt = false) },
            "frame" to { h -> h.orbit(frame = 3) },
            "velocity" to { h -> h.orbit(velocity = 4f) },
            "radius low" to { h -> h.orbit(radius = 1f) },
            "radius high" to { h -> h.orbit(radius = 90f) },
            "z NaN" to { h -> h.orbit(zAmsl = Double.NaN) },
            "bad centre" to { h -> h.orbit(centreLatDeg = 0.0, centreLonDeg = 0.0) },
            "too far" to { h ->
                h.orbit(centreLatDeg = latNorthOf(GuidedEnvelope.MAX_REPOSITION_DISTANCE_M + 100.0))
            },
            "below datum" to { h -> h.orbit(zAmsl = DATUM - 1.0) },
        )
        for ((name, refusal) in refusals) {
            val h = Harness()
            assertEquals(name, Verdict.DENIED, refusal(h))
            assertTrue("$name produced no STATUSTEXT", h.texts().isNotEmpty())
            assertTrue("$name produced no orbit_denied event", h.events.any { it.first == "orbit_denied" })
            for (text in h.texts()) {
                assertTrue(
                    "$name said ${text.length} bytes: $text",
                    text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
                )
            }
        }
    }

    // ------------------------------------------------- param3 and param4, announced

    @Test
    fun `param3 UNCHANGED is flown as nose-to-centre, and the substitution is announced`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.orbit(yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat()))
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_NOSE_TO_CENTRE))
    }

    @Test
    fun `param3 asking for nose-to-centre needs no substitution announcement`() {
        val h = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            h.orbit(yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER.toFloat()),
        )
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_NOSE_TO_CENTRE))
    }

    @Test
    fun `param4 NaN is one turn, zero is unbounded and said so, and too many turns are truncated`() {
        val nan = Harness()
        nan.orbit(turns = Float.NaN)
        assertTrue(nan.events.any { it.first == "orbit_accepted" && it.second!!.contains("sweep=360") })
        assertFalse(nan.texts().contains(GuidedStatusTexts.ORBIT_UNBOUNDED))

        val forever = Harness()
        forever.orbit(turns = 0f)
        assertTrue(forever.texts().contains(GuidedStatusTexts.ORBIT_UNBOUNDED))

        // At R = 20 the cap allows 1547° — a bit over four turns — so five is the first request
        // that has to be truncated. Written from the constant rather than from the number, so a
        // changed ORBIT_MAX_S moves the test with the code.
        val many = Harness()
        assertTrue(5.0 * OrbitGuidance.DEGREES_PER_TURN > OrbitGuidance.maxSweepDeg(RADIUS))
        many.orbit(turns = 5f)
        assertTrue(many.texts().contains(GuidedStatusTexts.ORBIT_TURNS_CAPPED))
        // Truncated to what the 180 s cap actually allows at this radius, not to the request.
        assertTrue(
            many.events.any {
                it.first == "orbit_accepted" &&
                    it.second!!.contains("sweep=%.0f".format(OrbitGuidance.maxSweepDeg(RADIUS)))
            }
        )
    }

    @Test
    fun `a modest turn count inside the time cap is taken as asked`() {
        val h = Harness()
        h.orbit(turns = 2f)
        assertTrue(h.events.any { it.first == "orbit_accepted" && it.second!!.contains("sweep=720") })
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_TURNS_CAPPED))
    }

    // -------------------------------------------------------- the join, then the circle

    /** Teleport the aircraft [steps] × [degreesPerStep] around the circle, ticking at each stop. */
    private fun walk(
        h: Harness,
        steps: Int,
        degreesPerStep: Double,
        radius: Double = RADIUS,
        startBearing: Double = 0.0,
        radialOffset: (Int) -> Double = { 0.0 },
    ) {
        var bearing = startBearing
        for (step in 1..steps) {
            bearing += degreesPerStep
            h.placeOnCircle(bearing, radius + radialOffset(step))
            h.tickAlive()
        }
    }

    /** The tangential and outward-radial components of the last commanded setpoint, m/s. */
    private fun components(h: Harness, bearingDeg: Double, direction: Int): Pair<Double, Double> {
        val setpoint = h.cmds.last().setpoint!!
        val rad = Math.toRadians(bearingDeg)
        val unitRadialNorth = kotlin.math.cos(rad)
        val unitRadialEast = kotlin.math.sin(rad)
        val tangential = direction * (-unitRadialEast * setpoint.north!! + unitRadialNorth * setpoint.east!!)
        val radial = unitRadialNorth * setpoint.north!! + unitRadialEast * setpoint.east!!
        return tangential to radial
    }

    @Test
    fun `the join leg is an ordinary resting leg under the M3 law - and commands no yaw`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.tickAlive()
        // 40 m north of the centre with a 20 m circle: the join point is 20 m north of the centre,
        // so the leg is 20 m due south and the M3 law's envelope cap decides — 3 m/s.
        val sent = h.port.sent.last()
        assertEquals(-GuidedEnvelope.HORIZONTAL_MAX_MS, sent.roll, 1e-9) // roll drives north
        assertEquals(0.0, sent.pitch, 1e-9)
        // **The yaw exception is scoped to circling.** A resting leg is not circling.
        assertEquals(0.0, sent.yaw, 1e-9)
        assertEquals(GuidedStickEngine.ORBIT_SOURCE, h.cmds.last().source?.messageName)
    }

    @Test
    fun `the join ends on the M3 arrival test, and the circle begins from rest`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        // Four settled ticks are not enough: the M3 arrival test wants ARRIVE_TICKS consecutive.
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_CIRCLING))
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_CIRCLING))
        assertTrue(h.events.any { it.first == "orbit_circling" })
        // The transition itself commands zero: the circle starts from rest, by construction.
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.pitch, 1e-12)
        assertEquals(0.0, sent.roll, 1e-12)
        assertEquals(0.0, sent.yaw, 1e-12)
    }

    @Test
    fun `an unsettled tick resets the arrival count, so a fly-through never starts the circle`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        // One tick at speed — through the join point rather than resting on it.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, vn = 3.0)
        h.tickAlive()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_CIRCLING))
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_CIRCLING))
    }

    @Test
    fun `the first circling tick ramps from rest instead of stepping to the cap`() {
        val h = Harness().reachCircle()
        h.tickAlive()
        // One 100 ms tick of a_max = 0.5 m/s² is 0.05 m/s, and no more. Due north of the centre,
        // clockwise, that is 0.05 m/s east — which rides DJI's `pitch` (measured 2026-07-26).
        val sent = h.port.sent.last()
        assertEquals(0.05, sent.pitch, 1e-9)
        assertEquals(0.0, sent.roll, 1e-9)
        assertNotEquals(OrbitGuidance.tangentialCap(RADIUS), sent.pitch, 1e-6)
    }

    @Test
    fun `the ramp climbs one a_max step per tick and settles at the curvature cap`() {
        val h = Harness().reachCircle()
        val cap = OrbitGuidance.tangentialCap(RADIUS)
        var previous = 0.0
        repeat(200) {
            h.tickAlive()
            val speed = kotlin.math.hypot(h.port.sent.last().pitch, h.port.sent.last().roll)
            assertTrue("stepped from $previous to $speed", speed - previous <= 0.05 + 1e-9)
            assertTrue("exceeded the curvature cap: $speed > $cap", speed <= cap + 1e-9)
            previous = speed
        }
        assertEquals(cap, previous, 1e-9)
    }

    @Test
    fun `PROPERTY - no tick of a flown orbit ever exceeds the curvature cap or the radial cap`() {
        // The engine-level twin of `OrbitGuidanceTest`'s property: not the formula in isolation but
        // the setpoints that actually leave this class, over a circle walked with the aircraft
        // pushed on and off the ring so the radial term is exercised too.
        val random = kotlin.random.Random(20260727)
        for (radius in listOf(5.0, 12.0, 20.0, 35.0, 50.0)) {
            val h = Harness()
            h.place(latDeg = LAT, lonDeg = LON)
            assertEquals(Verdict.ACCEPTED, h.orbit(radius = radius.toFloat(), turns = 0f))
            h.confirm()
            h.placeOnCircle(0.0, radius)
            repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }

            val cap = OrbitGuidance.tangentialCap(radius)
            var bearing = 0.0
            repeat(300) {
                bearing += 3.0
                val offset = (random.nextDouble() - 0.5) * 8.0
                h.placeOnCircle(bearing, radius + offset)
                h.tickAlive()
                val (tangential, radial) = components(h, bearing, direction = 1)
                assertTrue("R=$radius tangential $tangential > $cap", tangential <= cap + 1e-9)
                assertTrue(
                    "R=$radius radial |$radial| > ${OrbitGuidance.V_RADIAL_MAX_MS}",
                    abs(radial) <= OrbitGuidance.V_RADIAL_MAX_MS + 1e-9,
                )
                // And the same claim restated as the acceleration it is about.
                assertTrue(
                    "R=$radius asked for ${tangential * tangential / radius} m/s²",
                    tangential * tangential / radius <= RepositionGuidance.A_MAX_MS2 + 1e-9,
                )
            }
        }
    }

    /**
     * **The numbers here moved on 2026-07-27 and the property did not**, which is the distinction
     * worth preserving: this test is named for the *sign* of the correction, and the sign is what
     * it asserts.
     *
     * It used to assert the radial component was exactly `V_RADIAL_MAX_MS`. That was true only
     * because nothing bounded the composed vector: radial and tangential are orthogonal, so at
     * their caps the ground speed reached `sqrt(3.0² + 1.0²) = 3.162 m/s`, 5.4 % over the envelope
     * — found in flight (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.1) and
     * settled by Ivan as S-3: the envelope bounds **ground speed**. `OrbitGuidance.circleVelocity`
     * now scales the composed vector back, uniformly, so **both** components shrink by the same
     * factor during a correction at full tangential speed.
     *
     * The expectation is therefore derived from the constants rather than written as a literal. A
     * hard-coded `0.9487` would look like a test of the correction and be a test of the ratio
     * between three envelope numbers.
     */
    @Test
    fun `outside the ring the radial correction points inward, inside it points outward`() {
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() } // up to the cap, so the tangential term is not the story
        // Both axes at their caps → the composed vector is over the envelope → uniform scale-back.
        val tangential = OrbitGuidance.tangentialCap(RADIUS)
        val composed = hypot(OrbitGuidance.V_RADIAL_MAX_MS, tangential)
        val scale = min(1.0, GuidedEnvelope.HORIZONTAL_MAX_MS / composed)
        val expected = OrbitGuidance.V_RADIAL_MAX_MS * scale

        // 4 m outside, due north: the correction points south, on top of the eastward tangent.
        h.placeOnCircle(0.0, RADIUS + 4.0)
        h.tickAlive()
        val outside = h.cmds.last().setpoint!!.north!!
        assertTrue("outside the ring the correction must point inward, got $outside", outside < 0.0)
        assertEquals(-expected, outside, 1e-9)

        // 4 m inside: it points north.
        h.placeOnCircle(0.0, RADIUS - 4.0)
        h.tickAlive()
        val inside = h.cmds.last().setpoint!!.north!!
        assertTrue("inside the ring the correction must point outward, got $inside", inside > 0.0)
        assertEquals(expected, inside, 1e-9)

        // And the reason the numbers moved: the ground speed now sits on the envelope, not over it.
        val v = h.cmds.last().setpoint!!
        assertTrue(
            "ground speed ${hypot(v.north!!, v.east!!)} exceeds the envelope",
            hypot(v.north!!, v.east!!) <= GuidedEnvelope.HORIZONTAL_MAX_MS + 1e-9,
        )
    }

    // ------------------------------------------------ the swept angle, and completion

    /**
     * The swept angle the `orbit_ended` event recorded, or null if the orbit has not ended.
     *
     * Read as a number rather than compared as a string because the teleported walk below
     * accumulates ~1e-13° of floating-point error over a turn — real enough that
     * `swept >= 360.0` misses by a whisker on the 36th synthetic step, and meaningless enough
     * that tightening the *engine's* comparison to accommodate a test would be the wrong repair.
     * The assertions are therefore "one turn, to within a step", which is what the property
     * actually claims.
     */
    private fun endedSweep(h: Harness): Double? = h.events
        .firstOrNull { it.first == "orbit_ended" }
        ?.second
        ?.let { Regex("swept=(-?[0-9]+)").find(it)?.groupValues?.get(1)?.toDouble() }

    @Test
    fun `a full clockwise turn completes - and crossing the atan2 discontinuity does not end it early`() {
        val h = Harness().reachCircle()
        // Steps of 10° starting due north, so the walk crosses due south — where atan2 jumps from
        // +180 to −180 — at step 18. Without the ±180° wrap that one tick charges ~−350°, and the
        // orbit either "completes" on the spot or never completes at all; both are caught here.
        walk(h, steps = 35, degreesPerStep = 10.0)
        assertFalse("completed before a full turn: ${h.texts()}", h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        walk(h, steps = 2, degreesPerStep = 10.0, startBearing = 350.0)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second!!.startsWith("complete") })
        assertEquals(360.0, endedSweep(h)!!, 15.0)
    }

    @Test
    fun `a full anticlockwise turn completes too, on the same counter`() {
        val h = Harness()
        h.orbit(radius = -RADIUS.toFloat())
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        walk(h, steps = 35, degreesPerStep = -10.0)
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        walk(h, steps = 2, degreesPerStep = -10.0, startBearing = -350.0)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        // The counter runs *up* for an anticlockwise circle too — it is progress, not bearing.
        assertEquals(360.0, endedSweep(h)!!, 15.0)
    }

    @Test
    fun `two turns take two turns`() {
        val h = Harness()
        h.orbit(turns = 2f)
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        walk(h, steps = 71, degreesPerStep = 10.0)
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        walk(h, steps = 2, degreesPerStep = 10.0, startBearing = 710.0)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        assertEquals(720.0, endedSweep(h)!!, 15.0)
    }

    @Test
    fun `an unbounded orbit never completes on the swept angle`() {
        val h = Harness()
        h.orbit(turns = 0f)
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        walk(h, steps = 108, degreesPerStep = 10.0) // three full turns
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `on completion the aircraft comes to rest under the M3 arrival test, then holds`() {
        val h = Harness().reachCircle()
        walk(h, steps = 37, degreesPerStep = 10.0)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        // The completion point is where it stopped; sitting still there settles it in ARRIVE_TICKS.
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.pitch, 1e-12)
        assertEquals(0.0, sent.roll, 1e-12)
        assertEquals(0.0, sent.yaw, 1e-12)
        // Still engaged: disengaging stays an explicit act, exactly as after a goto arrives.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    // ---------------------------------------------- yaw: the one exception, and its edges

    /** Reach the circle and ramp the tangential speed to the curvature cap. */
    private fun atFullSpeed(): Harness {
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() }
        return h
    }

    @Test
    fun `while circling, the nose is turned to the centre`() {
        val h = atFullSpeed()
        // Due north of the centre with the nose pointed east: the centre bears 180, so the error
        // is +90° and the correction alone saturates the existing 30 °/s envelope.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)

        // Already nose-on, all that is left is the orbital rate as feed-forward: v/R = 3/20.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 180.0)
        h.tickAlive()
        assertEquals(
            Math.toDegrees(OrbitGuidance.tangentialCap(RADIUS) / RADIUS),
            h.port.sent.last().yaw,
            1e-9,
        )
    }

    // ## The orbit's centre became an ROI — measured mutation counts, 2026-07-27
    //
    // One breakage at a time, applied to the shipped source, the **whole** suite (1907) run against
    // each, confirmed red, reverted.
    //
    //  | mutation | tests that failed |
    //  |---|---|
    //  | the orbit implies no ROI at all (the old two-system world) | 16 |
    //  | a new destination does not release the circle's subject | 4 |
    //  | the implied ROI starts only once circling (the join flies sideways again) | 4 |
    //  | the implied ROI outranks an explicit one | 3 |
    //  | an explicit cancel leaves the circle's subject pointing | 1 |
    //  | a mission does not release it | 1 |
    //  | the subject dies with the circle again | **0 — equivalent, see below** |
    //  | an explicit ROI sits on top rather than replacing | **0 — masked, see below** |
    //  | **both** the accept-replace and the cancel-clears-both removed | 2 |
    //
    // ## The lifetime, after Ivan changed it on 2026-07-27
    //
    // An orbit **implies a subject at its centre**, and that subject **outlives the circle**. It is
    // released by a new destination (a goto or a mission — the operator is going somewhere else, so
    // the nose goes back to following its course), by an explicit `DO_SET_ROI_NONE`, or by being
    // replaced with an explicit ROI or a fresh circle. It is *not* released by the manoeuvre that
    // implied it finishing, which is what shipped an hour earlier and was wrong: the camera swung
    // off the subject exactly as the aircraft settled to watch it.
    //
    // **"The subject dies with the circle" scores zero because it is unreachable, not untested.**
    // Every normal ending — the sweep completing, the time cap, a Pause — leaves the `OrbitState`
    // alive in its FINISH or HOLD phase, and the only things that null it outright are a
    // replacement (which releases or replaces the subject anyway) and an abort (which suspends it).
    // So there is no state in which the orbit is gone and the subject is still live, and a gate on
    // `orbit == null` can never answer differently. Recorded rather than papered over.
    //
    // **"An explicit ROI sits on top" is masked by cancel clearing both.** With the layers stacked,
    // the only way to observe the difference is a cancel — and a cancel clears both either way.
    // Removing both layers is caught twice, which is what shows the property is tested.
    @Test
    fun `THE JOIN TURNS TOWARDS THE CENTRE, not sideways to the circle`() {
        // **Ivan's observation from the bench, 2026-07-27.** The join leg used to command exactly
        // zero yaw: the aircraft flew to the circle facing wherever it happened to be facing, and
        // the nose-to-centre law only began on arrival — saturating the yaw clamp catching up an
        // angle it could have been closing the whole way in. The measured flight has three seconds
        // of `yawrate=0` on the join and then the clamp for a second and a half.
        //
        // The fix was not a new law: the orbit now *implies an ROI at its own centre* (M4-6), and
        // the join leg was already asking the ROI system for its yaw.
        val h = Harness()
        h.orbit()
        h.confirm()
        // 40 m north of the centre, nose east. The centre bears 180, so the error is +90°.
        h.place(latDeg = latNorthOf(START_NORTH), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
        // And it is still *joining*: the translation is the ordinary resting leg toward the circle,
        // not a circling command.
        assertTrue("the join stopped translating", h.port.sent.last().roll < 0.0)

        // Nose already on the centre: nothing to correct, and no orbital rate yet to feed forward,
        // so the join commands no yaw at all — which is the old behaviour, in the one case where
        // the old behaviour was right.
        h.place(latDeg = latNorthOf(START_NORTH), lonDeg = LON, yawDeg = 180.0)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-6)
    }

    @Test
    fun `THE IMPLIED ROI DIES WITH THE ORBIT, without eleven places having to remember`() {
        // The centre is an ROI for exactly as long as the circle is being flown or flown to. A
        // target that outlived its manoeuvre would turn the nose during someone else's goto.
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() }
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)

        // A goto replaces the orbit. The old centre is not a target any more, and with the nose
        // already on the new course there is nothing for heading-follows-course to correct either.
        h.goto(latDeg = latNorthOf(RADIUS + 40.0))
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 0.0)
        repeat(3) { h.tickAlive() }
        assertEquals(0.0, h.port.sent.last().yaw, 1e-6)
    }

    @Test
    fun `an anticlockwise circle turns the nose the other way`() {
        val h = Harness()
        h.orbit(radius = -RADIUS.toFloat())
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        repeat(200) { h.tickAlive() }
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 180.0)
        h.tickAlive()
        assertEquals(
            -Math.toDegrees(OrbitGuidance.tangentialCap(RADIUS) / RADIUS),
            h.port.sent.last().yaw,
            1e-9,
        )
    }

    @Test
    fun `a stale heading commands exactly zero yaw, announced - never a guess`() {
        val h = atFullSpeed()
        // `yawDeg` rides Signal.ATTITUDE with a 2 s limit; past it the yaw axis goes inert while
        // the lateral half of the orbit carries on. The graduated treatment the vertical axis gets.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0, attitudeAge = 5_000L)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_NO_HEADING))
        // The circle keeps flying: the tangential command is untouched.
        assertEquals(OrbitGuidance.tangentialCap(RADIUS), h.port.sent.last().pitch, 1e-9)
    }

    @Test
    fun `an absent heading commands exactly zero yaw too`() {
        val h = atFullSpeed()
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = null)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_NO_HEADING))
    }

    @Test
    fun `the heading recovering restores the yaw loop on the very next tick`() {
        val h = atFullSpeed()
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0, attitudeAge = 5_000L)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0, attitudeAge = 0L)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
    }

    @Test
    fun `the PASSTHROUGH branch never adds yaw of its own`() {
        // Stage A passes the operator's *own* yaw stick through — that is the `r` axis and it is
        // the operator's hand, not this bridge's opinion. What must never happen is the engine
        // *generating* yaw outside an orbit. With `r` centred, every passthrough tick is zero, and
        // it stays zero however the aircraft happens to be pointed.
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500, r = 0)
        h.confirm()
        for (heading in listOf(0.0, 90.0, 180.0, 271.0)) {
            h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = heading)
            h.tickAlive()
            h.frame(x = 500, r = 0)
            assertEquals("heading $heading", 0.0, h.port.sent.last().yaw, 1e-12)
        }
    }

    @Test
    fun `the passthrough yaw axis is still the operator's to command`() {
        // The counterpart of the test above, so the distinction is pinned rather than implied:
        // removing the `r` mapping would be a regression, not a safety improvement.
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(r = 1000)
        h.confirm()
        h.tickAlive()
        h.frame(r = 1000)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
    }

    @Test
    fun `THE WAY BACK - with heading-follows-course off, a plain reposition commands exactly zero yaw`() {
        // The behaviour this test pinned unconditionally until 2026-07-27, now reachable through
        // the one flag. It is the flight-verified one, which is the whole reason the flag exists:
        // a frame surprise or a yaw-mixing surprise in the air is a tap rather than a new build.
        val h = Harness()
        h.headingFollows = false
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        for (heading in listOf(0.0, 90.0, 180.0, 271.0)) {
            h.place(latDeg = LAT, lonDeg = LON, yawDeg = heading)
            h.tickAlive()
            assertEquals("heading $heading", 0.0, h.port.sent.last().yaw, 1e-12)
        }
    }

    @Test
    fun `with heading-follows-course on, a plain reposition turns the nose toward its target`() {
        // Ivan, watching a leg flown with the nose fixed: "the drone is still not turning towards
        // the destination when it's flying a waypoint. No yaw change."
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(50.0), lonDeg = LON))
        h.confirm()
        // Aircraft at the centre, target 50 m north, nose due east: turn left, at the cap.
        h.place(latDeg = LAT, lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(-GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
        // Nose already on the target: nothing to turn.
        h.place(latDeg = LAT, lonDeg = LON, yawDeg = 0.0)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-9)
    }

    @Test
    fun `a goto taken over from an orbit stops yawing for the circle and yaws for its own target`() {
        val h = atFullSpeed()
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertNotEquals(0.0, h.port.sent.last().yaw, 1e-9)
        // The goto's default target is 50 m north of the centre, i.e. 30 m further north than the
        // aircraft. With the nose at 090 the leg's own law turns it left; the *circle's* law is
        // gone, which is what this asserts by taking the flag away and seeing exactly zero.
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.headingFollows = false
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().yaw, 1e-12)
    }

    // ------------------------------------------- the gimbal, open loop and rate limited

    @Test
    fun `the gimbal is pointed at the centre, at the takeoff datum's ground level`() {
        val h = Harness().reachCircle()
        // 10 m above the datum, 20 m from the centre: −atan2(10, 20) = −26.565°, negative down.
        // The target is assumed to be at ground level in *our* frame — not at the AMSL the
        // command carried, which is composed against a datum with no relationship to ours.
        assertEquals(-26.56505117707799, h.gimbal.aimed.last(), 1e-9)
        // **Two commands, not one, since the orbit's centre became an ROI (2026-07-27):** the first
        // goes out while the engagement is still being confirmed, from the start position 40 m out,
        // because the ROI camera pass runs in every phase — pointing a camera is not flying an
        // aircraft, and the operator's subject starts coming into frame while DJI is still
        // answering. The second is the circle's own solution once the aircraft is on the ring.
        assertEquals(2, h.gimbal.aimed.size)
        assertEquals(-14.036243467926479, h.gimbal.aimed.first(), 1e-9)
    }

    @Test
    fun `the gimbal already holds the centre during the join, not only while circling`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.tickAlive()
        // Still 40 m out: −atan2(10, 40) = −14.036°.
        assertEquals(-14.036243467926479, h.gimbal.aimed.single(), 1e-9)
    }

    @Test
    fun `on a steady circle the camera is commanded once and then left alone`() {
        // The pleasant consequence the decision doc designs around: radius and altitude are both
        // held, so atan2 has two fixed arguments and the camera simply sits still. This is what
        // makes the orbit the least demanding possible use of a command channel with a measured
        // swallowed-callback risk.
        val h = Harness().reachCircle()
        val onArrival = h.gimbal.aimed.size
        walk(h, steps = 72, degreesPerStep = 5.0)
        // A whole revolution, and the camera is not commanded once more.
        assertEquals(onArrival, h.gimbal.aimed.size)
    }

    @Test
    fun `a moved solution waits for the rate limit before it is re-commanded`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        h.tickAlive()

        // Climb to 30 m: −atan2(30, 20) = −56.31°, far outside the 1.5° deadband. Driven well past
        // the window first, so that what follows starts from a command whose time we know — the
        // camera's first aim goes out during the engagement now, not on the first orbit tick, and a
        // window measured from an unknown start is not a measurement.
        h.placeOnCircle(0.0, RADIUS, relAlt = 30.0)
        // Ticked until the command actually goes out, rather than for a fixed number of ticks: that
        // pins the window's start to the tick it really started on. The camera's first aim now
        // happens during the engagement, so counting from the first orbit tick would measure a
        // window that began somewhere else.
        val n0 = h.gimbal.aimed.size
        var guard = 0
        while (h.gimbal.aimed.size == n0 && guard++ < 30) h.tickAlive()
        val first = h.gimbal.aimed.last()
        assertEquals(-56.309932474020215, first, 1e-9)
        val before = h.gimbal.aimed.size

        // Back to 10 m — the same size of move, the other way — and now the window is the only
        // thing that can be holding it back.
        h.placeOnCircle(0.0, RADIUS, relAlt = 10.0)
        repeat(4) { h.tickAlive() } // 100, 200, 300, 400 ms since the last command
        assertEquals("commanded inside the rate window", before, h.gimbal.aimed.size)
        h.tickAlive() // 500 ms
        assertEquals(before + 1, h.gimbal.aimed.size)
        assertEquals(-26.56505117707799, h.gimbal.aimed.last(), 1e-9)
        assertNotEquals(first, h.gimbal.aimed.last(), 1e-6)
    }

    @Test
    fun `a solution that moved less than the deadband is never re-commanded, however long we wait`() {
        val h = Harness().reachCircle()
        // 10 m → 10.5 m at 20 m out moves the solution by 1.14°, under the 1.5° deadband.
        val before = h.gimbal.aimed.size
        h.placeOnCircle(0.0, RADIUS, relAlt = 10.5)
        repeat(100) { h.tickAlive() }
        assertEquals(before, h.gimbal.aimed.size)
    }

    @Test
    fun `the solution is clamped to DJI's own reported range, and the operator is told`() {
        val h = Harness()
        // The measured travel on this airframe is −90..+60; a shallower gimbal cannot look 26° down.
        h.gimbal.range = -20.0..60.0
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(-20.0, h.gimbal.aimed.last(), 1e-12)
        assertTrue(h.texts().contains(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `with no reported range nothing is clamped and nothing is invented`() {
        val h = Harness()
        h.gimbal.range = null
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(-26.56505117707799, h.gimbal.aimed.last(), 1e-9)
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `a solution inside the reported range says nothing about limits`() {
        val h = Harness()
        h.gimbal.range = -90.0..60.0
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        assertEquals(-26.56505117707799, h.gimbal.aimed.last(), 1e-9)
        assertFalse(h.texts().contains(GuidedStatusTexts.ROI_GIMBAL_RANGE))
    }

    @Test
    fun `a camera that will not aim does not cost the aircraft its circle`() {
        val h = Harness()
        h.gimbal.throwOnAim = IllegalStateException("gimbal gone")
        h.orbit()
        h.confirm()
        h.placeOnCircle(0.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        repeat(50) { h.tickAlive() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_CIRCLING))
        assertTrue(h.port.sent.isNotEmpty())
    }

    @Test
    fun `an orbit with no gimbal at all flies the circle perfectly well`() {
        // Null is every Stage A/B configuration, and a bridge with no camera attached must still
        // be able to circle. The absence is a missing capability, never a gate on flying.
        val h = Harness()
        val engine = GuidedStickEngine(
            port = h.port,
            interlockEnabled = { true },
            aircraftState = { h.state },
            announcer = Announcer(StatusTextSink { h.wire += it }),
            manoeuvreGimbal = null,
            nowMs = { h.now },
        )
        engine.attach()
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        val accepted = engine.orbit(
            OrbitCommand(
                isCommandInt = true, frame = 0,
                radiusM = RADIUS.toFloat(), velocityMs = Float.NaN,
                yawBehaviour = OrbitCommand.YAW_BEHAVIOUR_UNCHANGED.toFloat(), turns = Float.NaN,
                latE7 = (LAT * 1e7).roundToInt(), lonE7 = (LON * 1e7).roundToInt(),
                zAmslM = (DATUM + ALT).toFloat(),
            )
        )
        assertEquals(Verdict.ACCEPTED, accepted)
        engine.tick(h.now)
    }

    @Test
    fun `THE OPEN-LOOP RULE - the gimbal seam cannot express an attitude or an age`() {
        // Structural, on purpose. `KeyGimbalAttitude` is change-driven: a motionless gimbal stops
        // delivering, and DJI refuses a direct `get` on it — so "the attitude is old" and "the
        // camera has not moved" are the same bytes, and a steady orbit is exactly when that
        // happens. The rule against gating on it has now bitten this project three times, so it is
        // enforced by making it unsayable rather than by remembering it.
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
        // The behavioural half. The fake reports no attitude, ever — as a healthy stabilised
        // gimbal does — and the orbit must go on issuing absolute angles rather than deciding the
        // camera is dead and giving up on it.
        val h = Harness().reachCircle()
        var commands = h.gimbal.aimed.size
        // Climb steadily so the solution keeps moving past the deadband, for two minutes.
        for (step in 1..1_200) {
            h.placeOnCircle(0.0, RADIUS, relAlt = 5.0 + step * 0.02)
            h.tickAlive()
        }
        assertTrue("the camera stopped being commanded", h.gimbal.aimed.size > commands + 20)
        commands = h.gimbal.aimed.size
        // …and the aircraft is still flying its circle while that happens.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(commands > 0)
    }

    @Test
    fun `THE SUBJECT IS KEPT WHEN THE CIRCLE ENDS - the nose stays on the centre`() {
        // **Ivan, 2026-07-27, and it reversed what shipped an hour earlier.** The implied ROI used
        // to die with the circle, so the nose was released the moment the sweep completed and the
        // camera swung off the subject exactly as the aircraft settled to watch it. Keeping it is
        // what an operator means by orbiting something: the circle was the manoeuvre, the subject
        // is the point.
        //
        // It is released by a *new destination* or an explicit cancel — not by the manoeuvre that
        // implied it finishing. Both of those have their own tests.
        val h = Harness().reachCircle()
        walk(h, steps = 37, degreesPerStep = 10.0)
        // Nose 90° off the centre while coming to rest: a loop that had been released would command
        // nothing here, and the one that is still running commands a correction.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
        // The translation is still the wind-down's — the subject is kept, the manoeuvre is not.
        assertTrue(h.port.sent.last().roll < GuidedEnvelope.HORIZONTAL_MAX_MS)
    }

    @Test
    fun `A NEW DESTINATION RELEASES THE CIRCLE'S SUBJECT - the nose goes back to its course`() {
        val h = Harness().reachCircle()
        walk(h, steps = 5, degreesPerStep = 10.0)
        h.goto(latDeg = latNorthOf(RADIUS + 60.0))
        // Nose already on the new course, and nothing left pointing at the old centre, so the
        // commanded yaw is zero rather than a correction towards a subject the operator has left.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 0.0)
        repeat(3) { h.tickAlive() }
        assertEquals(0.0, h.port.sent.last().yaw, 1e-6)
    }

    @Test
    fun `AN EXPLICIT CANCEL RELEASES THE CIRCLE'S SUBJECT TOO, mid-orbit`() {
        // The third and last way to let go, and the one that must work in every state: turning
        // something off is always allowed. Fired *during* the circle, where the old rule would have
        // fallen back to the centre and kept pointing.
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() }
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)

        assertEquals(Verdict.ACCEPTED, h.engine.roi(RoiCommand(command = RoiCommand.MAV_CMD_DO_SET_ROI_NONE, isCommandInt = true)))
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals("the circle kept turning the nose after a cancel", 0.0, h.port.sent.last().yaw, 1e-9)
        // The circle itself is untouched — cancelling what the camera looks at is not cancelling
        // the manoeuvre, and the aircraft keeps flying the ring.
        assertTrue(h.port.sent.last().pitch != 0.0 || h.port.sent.last().roll != 0.0)
    }

    @Test
    fun `a fresh circle brings its own subject, replacing the last one`() {
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() }
        h.goto(latDeg = latNorthOf(RADIUS + 60.0)) // releases it
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        val released = h.port.sent.last().yaw

        h.orbit()
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertNotEquals("a new circle did not bring a subject with it", released, h.port.sent.last().yaw, 1e-9)
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
    }

    // ------------------------------------------------ termination, and the abort ladder

    @Test
    fun `the time cap ends a circle that would otherwise have none`() {
        // An orbit is the first thing this project flies with no natural completion. The aircraft
        // is parked on the circle so the swept angle never advances — only the clock can end this.
        val h = Harness().reachCircle()
        repeat(91) { h.tickAlive(2_000) } // 182 s past the accept
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_TIME_LIMIT))
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second!!.startsWith("time limit") })
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
    }

    @Test
    fun `the time cap does not fire one tick early`() {
        val h = Harness().reachCircle()
        repeat(89) { h.tickAlive(2_000) } // 178 s
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_TIME_LIMIT))
    }

    @Test
    fun `the join leg keeps the ordinary manoeuvre timeout - a leg that has not finished is not going to`() {
        val h = Harness()
        h.orbit()
        h.confirm()
        // Never gets there: the position feed keeps saying it is still 40 m out.
        // 152 s of joining, past the 150 s bound. The bound moved with the ceiling on
        // 2026-07-27 — see GuidedEnvelope.MANOEUVRE_TIMEOUT_MS — so this count is derived from
        // the constant rather than written out, and a future move will not silently pass.
        //
        // Since 2026-07-30 the join carries the ordinary **derived** deadline
        // (`OrbitState.joinDeadlineMs`), read at the join leg's own length. This fixture's join is
        // tens of metres, which derives to well under the floor, so the bound here is the flat
        // constant and this test is unchanged by that work — asserted rather than assumed:
        assertEquals(
            GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
            GuidedEnvelope.manoeuvreDeadlineMs(40.0, 0.0),
        )
        val ticks = (GuidedEnvelope.MANOEUVRE_TIMEOUT_MS / 2_000L).toInt() + 2
        repeat(ticks) { h.tickAlive(2_000) }
        assertTrue(h.texts().any { it.contains("timeout") })
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
    }

    @Test
    fun `a Pause ends the orbit and keeps station`() {
        val h = Harness().reachCircle()
        walk(h, steps = 5, degreesPerStep = 10.0)
        assertEquals(Verdict.ACCEPTED, h.pause())
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_PAUSED))
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second == "paused" })
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.pitch, 1e-12)
        assertEquals(0.0, sent.roll, 1e-12)
        // Still engaged — a pause is a withdrawal of the manoeuvre, not of the authority.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        // **And the subject survives the pause even though the manoeuvre does not**: a Pause
        // withdraws the circle, and the operator is still looking at the thing they were circling.
        // Asserted by putting the nose 90° off the centre, where a released loop commands nothing
        // and a live one saturates.
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, yawDeg = 90.0)
        h.tickAlive()
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, h.port.sent.last().yaw, 1e-9)
        assertEquals("the pause stopped keeping station", 0.0, h.port.sent.last().pitch, 1e-9)
    }

    @Test
    fun `M4-8 - a paused orbit does not resume, and a fresh circle starts with the counter at zero`() {
        val h = Harness().reachCircle()
        walk(h, steps = 30, degreesPerStep = 10.0) // 300° of progress, nearly a turn
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
        assertEquals(Verdict.ACCEPTED, h.pause())

        // The same circle, commanded again. Swept progress is a claim about where the aircraft has
        // been, and after a pause of unknown length with unknown drift it cannot be re-verified —
        // so it goes to zero and the manoeuvre re-enters at the join.
        assertEquals(Verdict.ACCEPTED, h.orbit())
        h.placeOnCircle(300.0)
        repeat(RepositionGuidance.ARRIVE_TICKS) { h.tickAlive() }
        walk(h, steps = 10, degreesPerStep = 10.0, startBearing = 300.0) // only 100° more
        // With the old 300° surviving this would be 400° and the orbit would have "completed"
        // three-quarters of the way round, pointing the camera at nothing.
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_COMPLETE))
    }

    @Test
    fun `a new DO_ORBIT replaces the live one rather than stacking on it`() {
        val h = Harness().reachCircle()
        walk(h, steps = 10, degreesPerStep = 10.0)
        assertEquals(Verdict.ACCEPTED, h.orbit(radius = 30f))
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second == "replaced" })
        assertEquals(1, h.port.enableCalls) // no re-engagement: the authority is reused
    }

    @Test
    fun `a DO_REPOSITION ends the orbit`() {
        val h = Harness().reachCircle()
        assertEquals(Verdict.ACCEPTED, h.goto())
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second == "replaced by goto" })
        // …and what flies from here is the goto, not the circle.
        h.tickAlive()
        assertEquals(GuidedStickEngine.REPOSITION_SOURCE, h.cmds.last().source?.messageName)
    }

    @Test
    fun `a deliberate GCS deflection cancels the orbit and passthrough takes over`() {
        val h = Harness().reachCircle()
        h.frame() // neutral, so the stream proves its convention
        h.now += 40
        h.frame(x = 500)
        assertTrue(h.texts().contains(GuidedStatusTexts.ORBIT_STICKS))
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second == "sticks" })
        h.tickAlive()
        assertEquals("MANUAL_CONTROL", h.cmds.last().source?.messageName)
    }

    @Test
    fun `a stream never seen at rest cannot cancel an orbit - the centre-zero landmine`() {
        // A centre-zero stream's *idle* frame reads deliberate (z = 0 is 500 off centre). Without
        // the at-rest gate, merely enabling that regime would cancel every orbit and command a
        // full-scale descent.
        val h = Harness().reachCircle()
        h.frame(z = 0)
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_STICKS))
        assertTrue(h.texts().contains(GuidedStatusTexts.CENTER_FIRST))
        h.tickAlive()
        assertEquals(GuidedStickEngine.ORBIT_SOURCE, h.cmds.last().source?.messageName)
    }

    @Test
    fun `the interlock going off mid-circle aborts, and the circle does not survive it`() {
        val h = Harness().reachCircle()
        h.interlock = false
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().any { it.contains("interlock") })
        // A circle that survived into the next engagement would fly the aircraft on a command
        // nobody just gave.
        h.interlock = true
        val before = h.port.sent.size
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS + 100 // past the anti-hammering window
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        h.tickAlive()
        // What flies is the operator's stick, from scratch — not a resurrected circle.
        assertEquals("MANUAL_CONTROL", h.cmds.last().source?.messageName)
        assertTrue(h.port.sent.size > before)
        // …and no `orbit_circling` a second time: there is no circle left to re-enter.
        assertEquals(1, h.events.count { it.first == "orbit_circling" })
    }

    @Test
    fun `RC sticks abort a circle, and DJI's own authority word does too`() {
        val sticks = Harness().reachCircle()
        sticks.port.onRc!!(RcSticks(400, 0, 0, 0))
        sticks.tickAlive()
        sticks.tickAlive()
        assertEquals(GuidedPhase.IDLE, sticks.engine.phase)
        assertTrue(sticks.texts().any { it.contains("sticks") })

        val authority = Harness().reachCircle()
        authority.port.onReason!!("RC_ONE_KEY_GO_HOME")
        assertEquals(GuidedPhase.IDLE, authority.engine.phase)
        assertTrue(authority.texts().any { it.contains("RC_ONE_KEY_GO_HOME") })
    }

    @Test
    fun `an aborted circle is gone, not lying in ambush for the next engagement`() {
        // The primary clear, pinned on its own. A stale circle cannot actually *fly* — the
        // deflection that re-engages is itself deliberate and clears it on the way past — so the
        // observable difference is a spurious `orbit_ended replaced` when the next circle is
        // commanded, i.e. this bridge believing it still had one. Same shape as the
        // target-survives-abort mutant in `GuidedRepositionTest`, and pinned the same way.
        val h = Harness().reachCircle()
        h.port.onReason!!("RC_ONE_KEY_GO_HOME")
        assertEquals(GuidedPhase.IDLE, h.engine.phase)

        assertEquals(Verdict.ACCEPTED, h.orbit())
        assertFalse(
            "the aborted circle was still there: ${h.events.filter { it.first == "orbit_ended" }}",
            h.events.any { it.first == "orbit_ended" && it.second == "replaced" },
        )
        // …and nothing spurious was said to the operator about sticks cancelling anything.
        assertFalse(h.texts().contains(GuidedStatusTexts.ORBIT_STICKS))
    }

    @Test
    fun `an engage DJI refuses drops the circle waiting on it`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.orbit())
        h.port.enableOnFailure!!("VIRTUAL_STICK_NOT_ALLOWED")
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.events.any { it.first == "orbit_ended" && it.second == "engage refused" })
        assertTrue(h.texts().any { it.contains("VIRTUAL_STICK_NOT_ALLOWED") })
    }

    @Test
    fun `a stale position holds zero rather than circling on a cached fix, then releases`() {
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() } // up to speed, so a cached fix would still be moving
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, positionAge = 5_000L)
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_POSITION_HOLD))
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.pitch, 1e-12)
        assertEquals(0.0, sent.roll, 1e-12)
        assertEquals(0.0, sent.yaw, 1e-12)
        // Bounded: our feed being dead says nothing about DJI's own GPS, and DJI's failsafes are
        // the better pilot of a blind bridge.
        repeat(110) { h.tickAlive() } // past POSITION_LOST_MS
        assertTrue(h.texts().any { it.contains("no-fix") })
    }

    @Test
    fun `total link silence mid-circle runs the armed link-loss policy`() {
        val h = Harness().reachCircle()
        h.tick(GuidedEnvelope.LINK_LOST_MS + 100)
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().any { it.contains("link-lost") })
    }

    @Test
    fun `an unknowable altitude leaves the circle flying and the vertical axis inert`() {
        val h = Harness().reachCircle()
        repeat(200) { h.tickAlive() }
        h.place(latDeg = latNorthOf(RADIUS), lonDeg = LON, relAlt = null)
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_ALTITUDE))
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-12)
        // The lateral half carries on — the graduated treatment, not a stop.
        assertEquals(OrbitGuidance.tangentialCap(RADIUS), h.port.sent.last().pitch, 1e-9)
    }

    // ------------------------------------------------------------- the sentences

    @Test
    fun `every orbit sentence fits the 50-byte STATUSTEXT field`() {
        val sentences = listOf(
            GuidedStatusTexts.ORBIT_STARTED,
            GuidedStatusTexts.ENGAGED_ORBIT,
            GuidedStatusTexts.ORBIT_CIRCLING,
            GuidedStatusTexts.ORBIT_NOSE_TO_CENTRE,
            GuidedStatusTexts.ORBIT_COMPLETE,
            GuidedStatusTexts.ORBIT_TIME_LIMIT,
            GuidedStatusTexts.ORBIT_PAUSED,
            GuidedStatusTexts.ORBIT_STICKS,
            GuidedStatusTexts.ORBIT_UNBOUNDED,
            GuidedStatusTexts.ORBIT_TURNS_CAPPED,
            GuidedStatusTexts.ORBIT_CAPPED,
            GuidedStatusTexts.ROI_NO_HEADING,
            GuidedStatusTexts.ROI_GIMBAL_RANGE,
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_ORBIT_RADIUS),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_ORBIT_TOO_FAR),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_ORBIT_LONG_FORM),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_SPEED),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_NO_DATUM),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_NO_FIX),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_BELOW_DATUM),
            GuidedStatusTexts.orbitRefused(GuidedStatusTexts.REASON_BAD_TARGET),
            GuidedStatusTexts.orbitRefused("SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW"),
        )
        for (sentence in sentences) {
            val bytes = sentence.toByteArray(Charsets.UTF_8).size
            assertTrue("$bytes bytes: $sentence", bytes <= StatusTexts.MAX_BYTES)
            assertTrue("empty sentence", sentence.isNotEmpty())
        }
    }

    @Test
    fun `the radius sentence names the limits by number, from the constants themselves`() {
        assertEquals("radius 5-50m only", GuidedStatusTexts.REASON_ORBIT_RADIUS)
        assertTrue(GuidedStatusTexts.ORBIT_TIME_LIMIT.contains(OrbitGuidance.ORBIT_MAX_S.toString()))
        assertTrue(GuidedStatusTexts.ORBIT_CAPPED.contains(GuidedEnvelope.CEILING_M.toInt().toString()))
    }

    @Test
    fun `param3 = 5 is a named integer, because our dialect's enum stops at 4`() {
        // `ORBIT_YAW_BEHAVIOUR_UNCHANGED` exists only in QGC's newer definitions; ours ends at
        // `4 = RC_CONTROLLED`. A lookup would fail to compile or, worse, resolve to whatever the
        // number comes to mean later.
        assertEquals(5, OrbitCommand.YAW_BEHAVIOUR_UNCHANGED)
        assertEquals(0, OrbitCommand.YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER)
        assertEquals(34, OrbitCommand.MAV_CMD_DO_ORBIT)
    }
}
