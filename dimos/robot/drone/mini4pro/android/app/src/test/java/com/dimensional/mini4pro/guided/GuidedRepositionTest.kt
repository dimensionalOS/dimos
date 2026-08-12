package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.Announcer
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
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * M3 Stage B — `DO_REPOSITION` through [GuidedStickEngine.reposition]: the truthful ack, the
 * altitude datum, the guidance path, arrival, keep-station, and every Stage A abort applied
 * mid-manoeuvre. Same protocol as `GuidedStickEngineTest`: fake port, hand-cranked clock.
 *
 * Written to fail loudly for the Stage B landmines:
 *
 *  - an inbound AMSL trusted absolutely instead of subtracted against our own published datum
 *    (it moved 41.5 m between sessions)
 *  - a missing cos(latitude) — invisible at the equator, 21% at 38°N (pinned here end to end
 *    and in `RepositionGuidanceTest` in isolation)
 *  - an `ACCEPTED` for a target that was not actually taken, or a refusal without a sentence
 *  - arrival declared on a fly-through (either conjunct alone, or non-consecutive ticks)
 *  - a reposition flown on a cached position fix
 *  - a GCS stick deflection that cannot interrupt — or a centre-zero stream that can
 *
 * Mutation-checked 2026-07-26, one breakage at a time, failing tests counted across the three
 * guided suites (this one, `RepositionGuidanceTest`, `GuidedStickEngineTest`), code reverted
 * after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | ACCEPTED before validation (every refusal acked ACCEPTED) | 14 |
 *  | datum subtraction dropped (`targetRel = z`, the absolute number trusted) | 8 |
 *  | datum gate removed (null datum accepted, 0 substituted) | 1 |
 *  | position freshness dropped at accept (stale fix accepted) | 1 |
 *  | 100 m distance gate removed | 1 |
 *  | ceiling cap at accept removed (target taken at 80 m) | 1 |
 *  | below-datum refusal removed | 1 |
 *  | interlock-off answers ACCEPTED | 1 |
 *  | arrival: distance conjunct dropped | 4 |
 *  | arrival: settle conjunct dropped | 2 |
 *  | arrival: k-consecutive dropped (single settled tick arrives) | 2 |
 *  | arrival: reset-on-unsettled dropped (ticks accumulate across breaks) | 1 |
 *  | stale velocity read as settled (arrival's null speed treated as 0) | 1 |
 *  | stale position flown on (cached fix fed to the law) | 2 |
 *  | position-lost release removed (holds zero forever on a dead feed) | 1 |
 *  | link watchdog removed from the reposition tick | 2 |
 *  | manoeuvre timeout removed from the reposition tick | 1 |
 *  | idle-after-arrival disengage removed | 1 |
 *  | vertical error sign flipped at the engine seam (`repo.relAltM − alt`) | 3 |
 *  | stick interrupt removed (deliberate deflection ignored mid-goto) | 2 |
 *  | stick interrupt ungated (centre-zero stream cancels into a descent) | 1 |
 *  | target survives abort (`reposition` not cleared) | 1 — see below |
 *  | target survives enable refusal | 1 |
 *  | guidance sends recorded without their DO_REPOSITION source | 2 |
 *
 * Added with Change Altitude (2026-07-26 night, measured shape — record
 * `20260726-221915.001`), same protocol:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | Change Altitude branch removed (shape falls to the unmeasured-shape refusal) | 5 |
 *  | "current position" resolves to (0, 0) instead of the fix | 4 |
 *  | datum subtraction dropped from the shared accept path | 12 |
 *  | usableAltitude liveness proxy dropped (delivery-age only — the hover deadlock) | 2 |
 *  | usableAltitude aliveness check removed entirely (cached altitude always trusted) | 2 |
 *
 * Added with `ControlOrigin` (2026-07-27), same protocol — the full table, including the three
 * mutants that are alive on purpose because one origin cannot distinguish them, is in
 * `GuidedStickEngineTest`'s header:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `reposition` stops stamping its own controller's liveness | 2 |
 *  | the reposition tick's link watchdog removed entirely | 2 |
 *
 * The 14 and the 8 are the ack and the datum — the honest channel and landmine 1; much of the
 * suite refuses to run past either lie.
 *
 * **The target-survives-abort mutant scored 0 on the first pass**, and the reason is worth
 * keeping: a lingering target cannot actually fly, because the deflection that re-engages is
 * itself deliberate and fires the goto-interrupt, which clears it — defence in depth masking
 * the primary clear. The observable difference is the *spurious* `Goto cancelled - GCS sticks`
 * announcement and `goto_ended` event during the re-engagement; the abort test now asserts
 * their absence, which is what makes both survives-mutants killable. The layering is kept —
 * two independent reasons the stale target cannot fly — but the tests pin each layer alone.
 */
class GuidedRepositionTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        /** The published pressure-altitude datum this session. Another session saw it 41.5 m away. */
        const val DATUM = 100.0

        /** Default aircraft height above the datum. */
        const val ALT = 10.0

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

    private class RecordedCmd(
        val setpoint: Setpoint?, val axes: StickAxes, val source: CommandSource?,
        val accepted: Boolean?, val error: String?,
    )

    private class Harness(policy: LinkLossPolicy = LinkLossPolicy.SHIPPED) {
        var now = 1_000L
        var interlock = true
        var state = stateAt()
        val port = FakeVirtualStickPort()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
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
            latDeg: Double = LAT, lonDeg: Double = LON, relAlt: Double? = ALT,
            vn: Double = 0.0, ve: Double = 0.0, vd: Double = 0.0,
            positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
            datum: Double? = DATUM, flightMode: String? = null,
        ) {
            state = stateAt(latDeg, lonDeg, relAlt, vn, ve, vd, positionAge, altitudeAge, velocityAge, datum, flightMode)
        }

        fun goto(
            latDeg: Double = latNorthOf(50.0), lonDeg: Double = LON, zAmsl: Double = DATUM + ALT,
            frame: Int = 0, speed: Float = -1f, yaw: Float = Float.NaN, isInt: Boolean = true,
            p5: Float = Float.NaN, p6: Float = Float.NaN,
            origin: ControlOrigin = ControlOrigin.MAVLINK,
        ): Verdict = engine.reposition(
            RepositionCommand(
                isCommandInt = isInt, frame = frame,
                latE7 = (latDeg * 1e7).roundToInt(), lonE7 = (lonDeg * 1e7).roundToInt(),
                zAmslM = zAmsl.toFloat(), groundSpeedMs = speed, yawRad = yaw,
                param5 = p5, param6 = p6,
            ),
            origin,
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

        companion object {
            fun stateAt(
                latDeg: Double = LAT, lonDeg: Double = LON, relAlt: Double? = ALT,
                vn: Double = 0.0, ve: Double = 0.0, vd: Double = 0.0,
                positionAge: Long = 0L, altitudeAge: Long = 0L, velocityAge: Long = 0L,
                datum: Double? = DATUM, flightMode: String? = null,
            ) = AircraftState(
                latitude = latDeg, longitude = lonDeg,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = datum,
                velocityNorth = vn, velocityEast = ve, velocityDown = vd,
                flightMode = flightMode,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to altitudeAge,
                    Signal.VELOCITY to velocityAge,
                ),
            )
        }
    }

    // ------------------------------------------------------- accept and engage

    @Test
    fun `a goto while idle is ACCEPTED, engages, and flies only after DJI confirms`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_STARTED))
        assertTrue(h.events.any { it.first == "goto_accepted" })
        // Nothing may flow before DJI's own state confirms authority.
        h.tickAlive()
        assertEquals(0, h.port.sent.size)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ENGAGED_GOTO))
        h.tickAlive()
        // 50 m north: envelope-capped 3 m/s north, which rides DJI `roll` (measured 2026-07-26).
        val sent = h.port.sent.last()
        assertEquals(3.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
        assertEquals(0.0, sent.yaw, 1e-9)
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
    }

    @Test
    fun `a goto reuses an existing stick engagement - one enable, one owner`() {
        val h = Harness()
        h.frame() // neutral, at rest
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertEquals(1, h.port.enableCalls)
        assertEquals(Verdict.ACCEPTED, h.goto())
        assertEquals(1, h.port.enableCalls)
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
    }

    @Test
    fun `interlock off answers UNSUPPORTED - the pre-Stage-B reply, no engagement, no sentence`() {
        val h = Harness()
        h.interlock = false
        assertEquals(Verdict.UNSUPPORTED, h.goto())
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().isEmpty())
    }

    // -------------------------------------------------------------- refusals

    @Test
    fun `no published datum - DENIED with the sentence, nothing taken`() {
        // Landmine 1: with no AMSL of our own published this link, an inbound AMSL is
        // uninterpretable and must be refused, never guessed at.
        val h = Harness()
        h.place(datum = null)
        assertEquals(Verdict.DENIED, h.goto())
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertTrue(h.texts().contains("Goto refused: NO_ALT_DATUM"))
        assertTrue(h.events.any { it.first == "goto_denied" })
        assertTrue(h.events.none { it.first == "goto_accepted" })
    }

    @Test
    fun `no position fix - DENIED`() {
        val h = Harness()
        h.state = AircraftState(relativeAltitude = ALT, takeoffAltitudeAmsl = DATUM)
        assertEquals(Verdict.DENIED, h.goto())
        assertTrue(h.texts().contains("Goto refused: NO_POSITION_FIX"))
    }

    @Test
    fun `a stale position fix at accept is no fix - DENIED`() {
        val h = Harness()
        h.place(positionAge = 5_000)
        assertEquals(Verdict.DENIED, h.goto())
        assertTrue(h.texts().contains("Goto refused: NO_POSITION_FIX"))
    }

    @Test
    fun `port unavailable - DENIED with the port's reason verbatim`() {
        val h = Harness()
        h.port.unavailable = "NO_PRODUCT"
        assertEquals(Verdict.DENIED, h.goto())
        assertTrue(h.texts().contains("Goto refused: NO_PRODUCT"))
    }

    @Test
    fun `dead RC stick feed - DENIED, the abort gesture would be blind`() {
        val h = Harness()
        h.port.onRc!!(RcSticks(0, null, 0, 0))
        assertEquals(Verdict.DENIED, h.goto())
        assertTrue(h.texts().contains("Goto refused: NO_RC_FEED"))
    }

    @Test
    fun `a target beyond the reach bound is refused naming the limit - never silently clamped`() {
        // Ivan's 2 km since 2026-07-30; derived from the constant rather than written out, because
        // a hard-coded 100 in an assertion looks like a test of the bound and is a test of the
        // number 100 (the lesson `docs/decisions/2026-07-27-ceiling-100m.md` recorded).
        val h = Harness()
        assertEquals(
            Verdict.DENIED,
            h.goto(latDeg = latNorthOf(GuidedEnvelope.MAX_REPOSITION_DISTANCE_M + 50.0)),
        )
        assertTrue(h.texts().contains("Goto refused: ${GuidedStatusTexts.REASON_TOO_FAR}"))
        // **And the sentence names the actual limit.** Asserted against the rendered text and not
        // against the constant that produced it, so a refusal string that stops naming its number
        // fails here rather than passing by comparing itself with itself.
        assertTrue(
            "the refusal must name the limit, got ${h.texts()}",
            h.texts().any { it.contains("${GuidedEnvelope.MAX_REPOSITION_DISTANCE_M.toInt()}m") },
        )
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        // Just inside it is taken — the bound is where it says it is.
        val ok = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            ok.goto(latDeg = latNorthOf(GuidedEnvelope.MAX_REPOSITION_DISTANCE_M - 50.0)),
        )
    }

    @Test
    fun `an unmeasured frame is refused, not interpreted`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(frame = 3))
        assertTrue(h.texts().contains("Goto refused: FRAME_3"))
    }

    @Test
    fun `a finite yaw is Change Heading's shape and is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(yaw = 1.57f))
        assertTrue(h.texts().contains("Goto refused: YAW_UNSUPPORTED"))
    }

    @Test
    fun `a positive speed request is refused - the envelope is not negotiable from MAVLink`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(speed = 8f))
        assertTrue(h.texts().contains("Goto refused: SPEED_UNSUPPORTED"))
    }

    @Test
    fun `a non-finite altitude is refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(zAmsl = Double.NaN))
        assertTrue(h.texts().contains("Goto refused: ALT_NOT_A_NUMBER"))
    }

    @Test
    fun `coordinates that are not a coordinate are refused`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(latDeg = 95.0))
        assertTrue(h.texts().contains("Goto refused: BAD_TARGET"))
    }

    @Test
    fun `a target below the takeoff datum is refused - the foreign-datum signature descends`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.goto(zAmsl = DATUM - 5.0))
        assertTrue(h.texts().contains("Goto refused: target below takeoff"))
    }

    @Test
    fun `a COMMAND_LONG goto with coordinates is an unmeasured shape and is refused`() {
        val h = Harness()
        assertEquals(
            Verdict.DENIED,
            h.goto(isInt = false, p5 = 38.0004f, p6 = 23.7f, zAmsl = DATUM + ALT),
        )
        assertTrue(h.texts().contains("Goto refused: COMMAND_LONG goto"))
    }

    // ---------------------------------------------- Change Altitude (measured 2026-07-26)
    //
    // QGC's Change Altitude arrived as COMMAND_LONG 192 with param1=-1, param2=1,
    // param4/5/6=NaN and param7 = target AMSL (record 20260726-221915.001, t=29.4). The
    // target is the current fix; every Go-to gate applies unchanged.

    @Test
    fun `the measured Change Altitude shape is accepted and targets the current position`() {
        val h = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            h.goto(isInt = false, p5 = Float.NaN, p6 = Float.NaN, zAmsl = DATUM + 10.0),
        )
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        val accepted = h.events.last { it.first == "goto_accepted" }.second!!
        // dist=0.0 is the "current position" claim: any other target would carry distance.
        assertTrue("expected zero distance, got: $accepted", accepted.contains("dist=0.0"))
        assertTrue("expected relAlt=10.0, got: $accepted", accepted.contains("relAlt=10.0"))
    }

    @Test
    fun `Change Altitude reads param7 against our own datum, never absolutely`() {
        // Same request decoded under two datums 41.5 m apart (the measured swing): the
        // relative target must follow the datum, not the absolute number.
        for (datum in listOf(100.0, 58.5)) {
            val h = Harness()
            h.place(datum = datum)
            assertEquals(
                Verdict.ACCEPTED,
                h.goto(isInt = false, p5 = Float.NaN, p6 = Float.NaN, zAmsl = datum + 10.0),
            )
            val accepted = h.events.last { it.first == "goto_accepted" }.second!!
            assertTrue("datum $datum: $accepted", accepted.contains("relAlt=10.0"))
        }
    }

    @Test
    fun `Change Altitude above the ceiling is capped and announced`() {
        val h = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            h.goto(isInt = false, p5 = Float.NaN, p6 = Float.NaN, zAmsl = DATUM + 150.0),
        )
        val accepted = h.events.last { it.first == "goto_accepted" }.second!!
        // Against the constant, not a literal: the ceiling moved once already (30 → 100 on
        // 2026-07-27) and a hard-coded number here would have passed for the wrong reason.
        assertTrue(accepted.contains("relAlt=%.1f".format(GuidedEnvelope.CEILING_M)))
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_CAPPED))
    }

    @Test
    fun `Change Altitude below the datum is refused - same foreign-datum guard as Go-to`() {
        val h = Harness()
        assertEquals(
            Verdict.DENIED,
            h.goto(isInt = false, p5 = Float.NaN, p6 = Float.NaN, zAmsl = DATUM - 5.0),
        )
        assertTrue(h.texts().contains("Goto refused: target below takeoff"))
    }

    @Test
    fun `a finite param4 in the Change Altitude shape is refused - Stage B does not yaw`() {
        val h = Harness()
        assertEquals(
            Verdict.DENIED,
            h.goto(isInt = false, p5 = Float.NaN, p6 = Float.NaN, zAmsl = DATUM + 10.0, yaw = 0.5f),
        )
    }

    // ------------------------------------------------------- the altitude datum

    @Test
    fun `the inbound AMSL is read against our own datum, never absolutely`() {
        // QGC composes z from the AMSL we last published; the same request in two sessions
        // whose datum moved 41.5 m must decode to the same relative target. z − datum is the
        // whole rule; trusting the absolute z in either session flies a huge vertical error.
        val h1 = Harness()
        h1.place(datum = 100.0, relAlt = 10.0)
        assertEquals(Verdict.ACCEPTED, h1.goto(zAmsl = 110.0)) // stay at 10 m rel
        h1.confirm()
        h1.tickAlive()
        assertEquals(0.0, h1.port.sent.last().verticalThrottle, 1e-9)

        val h2 = Harness()
        h2.place(datum = 58.5, relAlt = 10.0) // the measured between-sessions swing
        assertEquals(Verdict.ACCEPTED, h2.goto(zAmsl = 68.5)) // same request, new datum
        h2.confirm()
        h2.tickAlive()
        assertEquals(0.0, h2.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `a target above the aircraft climbs and below descends - through the datum round trip`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 20.0)) // 10 m above
        h.confirm()
        h.tickAlive()
        assertEquals(+1.5, h.port.sent.last().verticalThrottle, 1e-9) // DJI up-positive
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 5.0)) // 5 m below
        h.tickAlive()
        assertTrue(h.port.sent.last().verticalThrottle < 0.0)
    }

    // ---------------------------------------------------------------- ceiling

    @Test
    fun `a target above the ceiling is capped to it and announced, not refused`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 150.0))
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_CAPPED))
        h.confirm()
        // Just under the ceiling, the remaining commanded climb is against the cap, not against
        // what was asked: error 0.1 m → 0.05 m/s, where the uncapped target would still demand
        // the full 1.5. Written against the constant so it survives the ceiling moving again.
        h.place(relAlt = GuidedEnvelope.CEILING_M - 0.1)
        h.tickAlive()
        assertEquals(+0.05, h.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `at or above the ceiling the climb axis is gated during repositions too`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 30.0))
        h.confirm()
        h.place(relAlt = 101.0) // above the ceiling — the climb must be refused, not merely capped
        // Target rel 30, aircraft 31: guidance wants a descent — allowed. Make it want a climb:
        h.place(relAlt = 25.0)
        h.tickAlive()
        assertTrue(h.port.sent.last().verticalThrottle > 0.0) // below ceiling: climb passes
        h.place(relAlt = 101.0)
        h.tickAlive()
        // Above the ceiling nothing may climb, whatever the target says.
        assertTrue(h.port.sent.last().verticalThrottle <= 0.0)
    }

    @Test
    fun `altitude unusable - stale with a dead position feed - nothing is commanded at all`() {
        // Both the altitude delivery AND the liveness proxy (POSITION) are stale. The stale-fix
        // rule (landmine 7) outranks the altitude gate: the whole setpoint is zeroed the same
        // tick, so no axis — vertical included — flies on a cached fix.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 20.0))
        h.confirm()
        h.place(altitudeAge = 10_000, positionAge = 10_000)
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
        assertEquals(0.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
    }

    @Test
    fun `a hovering aircraft's silent altitude with a live position feed may climb - the change-driven deadlock`() {
        // Measured 2026-07-26 22:28 (record 20260726-222813.001): relalt is change-driven and
        // 0.1 m-quantised, so a hover goes silent (age 9.5 s) while POSITION jitters at ~50 ms.
        // Freshness-by-delivery-age deadlocks Change Altitude from a hover: blocked because
        // "stale", stale because not climbing. An unchanged value from a live component is
        // current — POSITION freshness vouches for the cached altitude.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(zAmsl = DATUM + 20.0))
        h.confirm()
        h.place(altitudeAge = 10_000, positionAge = 0)
        h.tickAlive()
        val sent = h.port.sent.last()
        assertTrue("expected a climb, got verticalThrottle=${sent.verticalThrottle}", sent.verticalThrottle > 0.5)
        assertTrue(h.texts().none { it == GuidedStatusTexts.NO_ALTITUDE })
    }

    // ------------------------------------------------------------ the guidance

    @Test
    fun `a diagonal target is flown in a straight line at the envelope`() {
        val h = Harness()
        assertEquals(
            Verdict.ACCEPTED,
            h.goto(latDeg = latNorthOf(30.0), lonDeg = lonEastOf(30.0)),
        )
        h.confirm()
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals(sent.roll, sent.pitch, 1e-3) // north == east component
        assertEquals(3.0, Math.hypot(sent.roll, sent.pitch), 1e-3)
    }

    @Test
    fun `the approach rides the braking envelope - 6 m out commands sqrt(2 a e), not the gain`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(6.0)))
        h.confirm()
        h.tickAlive()
        assertEquals(sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * 6.0), h.port.sent.last().roll, 1e-3)
    }

    @Test
    fun `guidance sends are recorded with their DO_REPOSITION source - ramps and holds with null`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        val flying = h.cmds.last()
        assertEquals("DO_REPOSITION", flying.source!!.messageName)
        assertEquals("NED_VELOCITY", flying.setpoint!!.frame)
        // Arrive, then the keep-station zeros are ours, not the command's.
        h.place(latDeg = latNorthOf(50.0))
        repeat(RepositionGuidance.ARRIVE_TICKS + 1) { h.tickAlive() }
        assertTrue(h.cmds.last().source == null)
    }

    // ------------------------------------------------------------------ arrival

    @Test
    fun `arrival needs both conjuncts held five consecutive ticks - then station is kept, engaged`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        // Teleport to 0.5 m short of the target, hovering: both conjuncts true.
        h.place(latDeg = latNorthOf(49.5))
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() }
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_ARRIVED })
        h.tickAlive() // the k-th consecutive tick
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_ARRIVED))
        assertTrue(h.events.any { it.first == "goto_arrived" })
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        // Keep-station: zeros keep flowing, engagement holds.
        repeat(5) { h.tickAlive() }
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertEquals(0, h.port.disableCalls)
    }

    @Test
    fun `a fly-through never declares arrival - inside the circle at speed is not arrived`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(latDeg = latNorthOf(49.5), vn = 2.5) // 0.5 m out, 2.5 m/s
        repeat(20) { h.tickAlive() }
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_ARRIVED })
    }

    @Test
    fun `hovering outside the acceptance radius is not arrival`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(latDeg = latNorthOf(46.0)) // 4 m out, settled
        repeat(20) { h.tickAlive() }
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_ARRIVED })
    }

    @Test
    fun `the arrival ticks must be consecutive - a break resets the count`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(latDeg = latNorthOf(49.5))
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() } // 4 settled ticks
        h.place(latDeg = latNorthOf(49.5), vn = 2.0) // one unsettled tick
        h.tickAlive()
        h.place(latDeg = latNorthOf(49.5))
        repeat(RepositionGuidance.ARRIVE_TICKS - 1) { h.tickAlive() } // 4 more, not 5
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_ARRIVED })
        h.tickAlive()
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_ARRIVED))
    }

    @Test
    fun `a stale velocity feed withholds arrival - a dead feed and a hover are the same bytes`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(latDeg = latNorthOf(49.5), velocityAge = 10_000)
        repeat(20) { h.tickAlive() }
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_ARRIVED })
    }

    @Test
    fun `an arrived hold idle-disengages after five minutes without a command`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(latDeg = latNorthOf(50.0))
        repeat(RepositionGuidance.ARRIVE_TICKS + 1) { h.tickAlive() }
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_ARRIVED))
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.IDLE_DISENGAGE_MS + 2_000 && h.engine.phase != GuidedPhase.IDLE) {
            h.tickAlive(1_000)
            elapsed += 1_000
        }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: idle"))
    }

    // ------------------------------------------------- the position feed, stale

    @Test
    fun `a stale position mid-flight commands zero immediately - never a cached fix`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        h.place(positionAge = 5_000) // same coordinates, dead feed
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.roll, 1e-9)
        assertEquals(0.0, sent.pitch, 1e-9)
        assertEquals(0.0, sent.verticalThrottle, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_POSITION_HOLD))
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        // The feed returning resumes the manoeuvre.
        h.place()
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
    }

    @Test
    fun `a position feed dead past ten seconds releases - DJI's failsafes have their own GPS`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(positionAge = 5_000)
        var elapsed = 0L
        while (elapsed <= RepositionGuidance.POSITION_LOST_MS + 1_000) {
            h.tickAlive(500)
            elapsed += 500
        }
        assertTrue(h.texts().contains("Virtual stick stopping: no-fix"))
        repeat(20) { h.tickAlive() }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: no-fix"))
    }

    // ------------------------------------------------ sticks outrank the controller

    @Test
    fun `a deliberate GCS deflection cancels the goto into passthrough - the hand wins`() {
        val h = Harness()
        h.frame() // the stream proves its convention at rest first
        h.now += 40
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        assertEquals("DO_REPOSITION", h.cmds.last().source!!.messageName)
        h.frame(x = 800)
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_STICKS))
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "sticks" })
        h.tickAlive(40)
        val sent = h.port.sent.last()
        assertEquals(2.4, sent.roll, 1e-9) // 0.8 deflection under the 3 m/s envelope
        assertEquals("MANUAL_CONTROL", h.cmds.last().source!!.messageName)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `a centre-zero stream cannot cancel a reposition into a descent`() {
        // Landmine 2 crossed with landmine 4: a centre-zero stream's *idle* frame reads
        // deliberate (z=0). If it could interrupt, enabling that regime mid-goto would cancel
        // the manoeuvre and hand the setpoint to a stream whose rest is a full-scale descent.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto()) // no neutral frame ever seen
        h.confirm()
        h.tickAlive()
        repeat(10) {
            h.frame(z = 0) // the centre-zero regime idling
            h.tickAlive(40)
        }
        // Still repositioning, still no descent, and the operator was told why.
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        assertTrue(h.port.sent.none { it.verticalThrottle < -1.0 })
        assertTrue(h.texts().contains(GuidedStatusTexts.CENTER_FIRST))
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_STICKS })
    }

    // --------------------------------------------- the Stage A aborts, mid-goto

    @Test
    fun `the RC pilot's hands abort a reposition entirely`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.port.onRc!!(RcSticks(0, 0, 0, 200))
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(1, h.port.disableCalls)
        assertTrue(h.texts().contains("Virtual stick off: sticks"))
    }

    @Test
    fun `the interlock switch aborts a reposition on the next tick`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.interlock = false
        h.tickAlive()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: interlock"))
    }

    @Test
    fun `DJI reporting authority elsewhere aborts a reposition immediately`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "RC"))
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
    }

    @Test
    fun `a mode seizure aborts a reposition - the overheat GO_HOME wire`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.place(flightMode = "GO_HOME")
        h.tickAlive(GuidedStickEngine.MODE_SEIZE_GRACE_MS + 100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: authority MODE_GO_HOME"))
    }

    @Test
    fun `link loss mid-goto runs the armed policy and the target does not survive it`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        repeat(35) { h.tick(100) } // total silence
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick stopping: link-lost"))
        // The operator's sticks coming back resume *passthrough*, never the dead manoeuvre.
        h.frame()
        h.tick(10)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.frame()
        h.tick(100)
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.roll, 1e-9) // neutral passthrough, not 3 m/s toward the target
        assertTrue(h.cmds.last().source?.messageName != "DO_REPOSITION")
    }

    @Test
    fun `a short goto that never arrives times out at the flat floor - the deadline's floor`() {
        // 90 m: under `GuidedEnvelope.manoeuvreDeadlineMs` this derives to ~63 s of travel, so the
        // **floor** is what bounds it and the manoeuvre keeps exactly the 150 s it had before
        // 2026-07-30. Drop the floor from the derivation and this fires at about 63 s instead —
        // which is the regression the floor exists to prevent, and this test is what measures it.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(90.0)))
        h.confirm()
        assertEquals(
            GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
            GuidedEnvelope.manoeuvreDeadlineMs(90.0, 0.0),
        )
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS - 20_000) {
            h.tickAlive(500) // the position never changes: the aircraft is not getting there
            elapsed += 500
        }
        assertTrue("nothing may time out inside the floor", h.texts().none { it.contains("timeout") })
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        while (elapsed <= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS + 1_000) {
            h.tickAlive(500)
            elapsed += 500
        }
        assertTrue(h.texts().contains("Virtual stick stopping: timeout"))
        repeat(20) { h.tickAlive() }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick off: timeout"))
    }

    @Test
    fun `THE LONG LEG - a 1500 m goto is not aborted at 150 s, and is still bounded`() {
        // The acceptance test for the distance-derived deadline (2026-07-30). Under the flat Q1
        // timeout this manoeuvre died at 150 s **every time**, with the aircraft ~450 m along a
        // 1500 m leg and no reason an operator could act on — strictly worse than the desk refusal
        // that used to stop the command being accepted at all. Revert
        // `GuidedEnvelope.manoeuvreDeadlineMs` to the flat constant and the first half of this test
        // goes red.
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(1_500.0)))
        h.confirm()
        val deadline = GuidedEnvelope.manoeuvreDeadlineMs(1_500.0, 0.0)
        assertTrue("a 1500 m leg must outlast the flat timeout", deadline > 900_000L)

        // Well past the old flat bound, and the manoeuvre is still flying at the envelope cap.
        var elapsed = 0L
        while (elapsed <= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS + 60_000) {
            h.tickAlive(1_000)
            elapsed += 1_000
        }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().none { it.contains("timeout") })
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)

        // And it is a deadline, not an absence of one: past its own derived bound the manoeuvre
        // ends exactly as a short one does. (The position never changes here, so this aircraft
        // genuinely is not getting there.)
        while (elapsed <= deadline + 2_000) {
            h.tickAlive(1_000)
            elapsed += 1_000
        }
        assertTrue(h.texts().contains("Virtual stick stopping: timeout"))
    }

    // ------------------------------------------------------ replace, resume, pause

    @Test
    fun `a new goto mid-flight replaces the target and is flown from the same engagement`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto()) // 50 m north
        h.confirm()
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = LAT, lonDeg = lonEastOf(40.0)))
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "replaced" })
        h.tickAlive()
        val sent = h.port.sent.last()
        assertEquals(3.0, sent.pitch, 1e-3) // east rides DJI pitch
        assertEquals(0.0, sent.roll, 1e-3)
        assertEquals(1, h.port.enableCalls)
    }

    @Test
    fun `a goto during a wind-down re-engages - an acknowledged command outranks the wind-down`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        repeat(35) { h.tick(100) }
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        assertEquals(Verdict.ACCEPTED, h.goto(latDeg = latNorthOf(20.0)))
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-3)
        assertEquals(0, h.port.disableCalls)
    }

    @Test
    fun `pause drops the target and keeps station - accepted, announced, idle clock running`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.tickAlive()
        assertEquals(3.0, h.port.sent.last().roll, 1e-9)
        assertEquals(Verdict.ACCEPTED, h.pause())
        assertTrue(h.texts().contains(GuidedStatusTexts.GOTO_PAUSED))
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "paused" })
        repeat(5) { h.tickAlive() }
        val sent = h.port.sent.last()
        assertEquals(0.0, sent.roll, 1e-9)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `pause with nothing to pause is DENIED`() {
        val h = Harness()
        assertEquals(Verdict.DENIED, h.pause())
        assertTrue(h.texts().contains("Goto refused: nothing to pause"))
    }

    @Test
    fun `DJI refusing the engage drops the accepted target - it never lies in ambush`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        h.port.enableOnFailure!!.invoke("CONTROL_AUTH_TAKING_OFF")
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick refused: CONTROL_AUTH_TAKING_OFF"))
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "engage refused" })
        // A later stick engagement must not fly toward the dead target — and must not have to
        // *cancel* it either: a target still lingering would make the engaging deflection fire
        // the goto-interrupt, announcing a cancellation of a manoeuvre that already ended.
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        h.frame(x = 0)
        h.tickAlive(40)
        assertEquals(0.0, h.port.sent.last().roll, 1e-9)
        assertTrue(h.cmds.none { it.source?.messageName == "DO_REPOSITION" })
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_STICKS })
    }

    @Test
    fun `an abort clears the target - a re-engagement flies nothing and cancels nothing`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        h.engine.abort(GuidedStickEngine.DisengageReason.INTERLOCK)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        h.frame()
        h.tickAlive(40)
        assertEquals(0.0, h.port.sent.last().roll, 1e-9)
        // Belt and braces exists (the engaging deflection would interrupt a lingering target),
        // but the abort's own clear is the property: no spurious cancellation is ever announced.
        assertTrue(h.texts().none { it == GuidedStatusTexts.GOTO_STICKS })
        assertTrue(h.events.none { it.first == "goto_ended" })
    }

    @Test
    fun `an unconfirmed engage withdraws the target with the engagement`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.tickAlive(GuidedStickEngine.ENGAGE_CONFIRM_MS + 100)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.texts().contains("Virtual stick refused: NO_CONFIRM"))
        assertTrue(h.events.any { it.first == "goto_ended" && it.second == "engage unconfirmed" })
    }

    // ------------------------------------------------- the commanding controller (ControlOrigin)

    /**
     * The command's own stamp is what keeps the first seconds of a manoeuvre alive, and it is
     * keyed to the controller the command came in on.
     *
     * This harness never calls `onInbound`, so the *only* liveness evidence the engine has is the
     * one `reposition` wrote for its own origin. Without it the Q4 watchdog would fire on the
     * first tick and run the armed policy against a manoeuvre that was acknowledged milliseconds
     * earlier — `reposition`'s own comment says why the stamp is taken before the gates rather
     * than left to `Bridge`'s later `onInbound` call.
     *
     * 2 s of silence, against `LINK_LOST_MS` of 3 s.
     */
    @Test
    fun `a goto stamps its own controller, so the manoeuvre is not born already expired`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto())
        h.confirm()
        repeat(20) { h.tick(100) }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().none { it.contains("link-lost") })
    }

    /**
     * Naming the origin explicitly is the same command as letting it default.
     *
     * The whole of `ControlOrigin`'s value today is that it changes nothing: one origin means the
     * per-origin map has one entry, and the watchdogs read the same number they read before it
     * was a map. This is that claim, made where it can fail.
     */
    @Test
    fun `an explicitly-named MAVLINK origin is the command that already existed`() {
        val h = Harness()
        assertEquals(Verdict.ACCEPTED, h.goto(origin = ControlOrigin.MAVLINK))
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        repeat(20) { h.tick(100) }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().none { it.contains("link-lost") })
    }
}
