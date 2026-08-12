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
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.roundToInt

/**
 * M3 Stage D — the tag-tracked descent through [GuidedStickEngine.armTagDescent] and its tick:
 * the arm gates and their named refusals, the centring and the cone as the aircraft actually
 * flies them, the staleness ladder, the terminal hold, and **every cancel path** — above all
 * rule 1, verbatim from Ivan: *"manual stick takeover cancels the landing completely."* Same
 * protocol as `GuidedOrbitTest`: fake port, hand-cranked clock, no aircraft, no detector — the
 * sensor is a supplier of plain values.
 *
 * The pure arithmetic — the cone, the ladder, the laws, the terminal latch — is pinned next
 * door in `TagDescentGuidanceTest`. This file is about what the *engine* does with it.
 *
 * Written to fail loudly for the Stage D landmines:
 *
 *  - an RC stick grab that does **not** kill the descent, or a killed descent that **resumes**
 *    on re-engagement — rule 1, both halves
 *  - an arm above the measured detection band, or without a latch, or with the camera off
 *    nadir — a descent armed on a sensor that is not there
 *  - descent continuing on a stale fix, or outside the alignment cone
 *  - the terminal hold descending below the target — Stage C flown by accident
 *  - a cancel path with no sentence and no flight-record line
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-28, one breakage at a time, applied to the shipped source, the whole
 * suite run (`:app:testDebugUnitTest`, 2296 tests, test-results deleted first), confirmed red,
 * reverted — and the whole campaign **re-measured against the final tree after shadow mode
 * landed**, since the shared arm gate and the shadow suite change what several mutants kill.
 * Counts are failing tests across the whole suite — **measured, not estimated**. The
 * arithmetic-side mutations are in `TagDescentGuidanceTest`'s table.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `abort()` no longer clears the descent (rule 1's clearing line removed) | 2 |
 *  | the RC-stick rung never fires while a descent is armed | 2 |
 *  | a deliberate GCS deflection no longer cancels the descent | 1 |
 *  | `ACCEPTED` returned before any arm gate | 39 |
 *  | the ceiling gate removed (arm at any height) | 2 |
 *  | the latch gate removed (arm with no tag ever seen) | 3 |
 *  | the freshness gate removed (arm on a fix from the climb-out) | 2 |
 *  | the nadir gate removed at arm | 1 |
 *  | the mid-descent nadir check removed | 1 |
 *  | the id-match dropped at fix ingest (a false id steers the descent) | 1 |
 *  | the live tick skips the staleness ladder (flies every fix as fresh) | 5 |
 *  | T_ABORT's handback no longer releases (holds blind forever) | 1 |
 *  | the terminal hold's idle window removed (authority held forever) | 1 |
 *  | refusals answer `DENIED` with no sentence | 10 |
 *  | **shadow actuates** (the would-be command also reaches the port) | 3 |
 *  | **shadow cancels on sticks** (instead of recording the edge) | 1 |
 *  | **shadow skips the ladder** (its step fed age 0 — mode divergence) | 2 |
 *
 * Round 3, 2026-07-28 evening: the arm gate's camera check moved **above** the fix-freshness
 * check (cause before symptom — the two denial-session records in this file's new tests), and
 * the pitch it judges became the *believed* pitch (`gimbal/PitchBelief`: commanded first,
 * reported fallback). The reorder and the belief have their own measured table in
 * `GuidedTakeoffClimbTest`'s 2026-07-28 addendum; the rows above were not re-measured — the
 * gates' contents are unchanged, only the order of two of them moved, and the reorder itself is
 * pinned red here (2 tests).
 *
 * Round 2, measured 2026-07-28 after the first shadow flights (suite 2307) — each of these is
 * a failure class the air, not the keyboard, surfaced:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | **the flown bug**: `abort()` kills shadow mode on any reason, not only STOPPED | 1 |
 *  | `abort()` never ends shadow mode, STOPPED included | 1 |
 *  | east error sign flipped at the live tick's step call | 2 |
 *  | north/east errors swapped at the same site | 4 |
 *  | the blocker never reaches the comparison feed | 1 |
 *
 * ### The masking layer, observed again and recorded rather than smoothed over
 *
 * With the abort-keeps-descent mutant applied, `after an RC grab the descent is gone -
 * re-engaging does not resume it` **passed** — the same defence-in-depth masking its Stage B
 * and Stage C twins showed: the deflection that re-engages is itself deliberate, so the
 * GCS-deflection interrupt (the *other* clearing layer) swept the surviving state away during
 * re-engagement, one line before the assert would have seen it. The mutant still scored 2,
 * because `a deliberate nudge sustained cancels completely` asserts the descent is gone on the
 * abort itself, with no re-engagement in between to mask it — which is why that test exists in
 * that exact shape. The layering is kept: either layer alone still leaves no path on which a
 * dead descent flies, and the primary layer is pinned on its own.
 */
class GuidedTagDescentTest {

    private companion object {
        /** The project's home latitude family — non-equatorial, cos 38 = 0.788. */
        const val LAT = 38.0
        const val LON = 23.7
        const val DATUM = 100.0

        /** Default aircraft height above the datum: inside the 7 m arm band. */
        const val ALT = 5.0

        fun latNorthOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (RepositionGuidance.METRES_PER_DEG * kotlin.math.cos(Math.toRadians(LAT)))
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

        // The sensor, as the Bridge seam would report it. `fixAtMs` is on this harness's own
        // clock, and the supplier computes the age at read time — exactly as `Bridge` computes
        // it from the fix's monotonic stamp.
        var senseAbsent = false
        var latchedId: Int? = 0
        var fixId: Int? = 0
        var fixNorth: Double? = 0.0
        var fixEast: Double? = 0.0
        var fixAtMs: Long? = 900L
        var cameraPitch: Double? = -90.0

        val port = FakeVirtualStickPort()
        val wire = mutableListOf<Any>()
        val cmds = mutableListOf<RecordedCmd>()
        val events = mutableListOf<Pair<String, String?>>()

        val engine = GuidedStickEngine(
            port = port,
            interlockEnabled = { interlock },
            aircraftState = { state },
            announcer = Announcer(StatusTextSink { wire += it }),
            tagSense = {
                if (senseAbsent) null else TagDescentSense(
                    latched = latchedId != null,
                    latchedTagId = latchedId,
                    fixTagId = fixId,
                    fixNorthM = fixNorth,
                    fixEastM = fixEast,
                    fixAgeMs = fixAtMs?.let { now - it },
                )
            },
            cameraPitchDeg = { cameraPitch },
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

        /** Place the aircraft [northM]/[eastM] metres from home, [relAlt] above the datum. */
        fun place(
            northM: Double = 0.0, eastM: Double = 0.0, relAlt: Double? = ALT,
            positionAge: Long = 0L, datum: Double? = DATUM,
        ) {
            state = stateAt(
                latDeg = latNorthOf(northM), lonDeg = lonEastOf(eastM), relAlt = relAlt,
                positionAge = positionAge, datum = datum,
            )
        }

        /** The camera sees the tag *now*, [northM]/[eastM] metres from home. */
        fun seeTag(northM: Double = 0.0, eastM: Double = 0.0) {
            fixNorth = northM
            fixEast = eastM
            fixAtMs = now
        }

        fun arm(): Verdict {
            // The link watchdog needs the commanding controller to have been heard from — in
            // the field QGC's traffic is continuous; here one payload is the evidence.
            engine.onInbound("heartbeat")
            return engine.armTagDescent()
        }

        /** DJI confirms the engagement the way the state listener would. */
        fun confirm() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tickAlive(40)
        }

        /** One engine tick with the GCS link demonstrably alive. */
        fun tickAlive(advanceMs: Long = 100) {
            now += advanceMs
            engine.onInbound("heartbeat")
            engine.tick(now)
        }

        /** One tick during which the camera also sees the tag again — the healthy cadence. */
        fun tickSeeing(advanceMs: Long = 100) {
            now += advanceMs
            seeTag(fixNorth ?: 0.0, fixEast ?: 0.0)
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

        /** Arm, confirm, and take one seeing tick: a descent in steady TRACKING. */
        fun descending(): Harness {
            seeTag(0.0, 0.0)
            place(northM = 0.0, eastM = 0.0, relAlt = ALT)
            check(arm() == Verdict.ACCEPTED)
            confirm()
            tickSeeing()
            return this
        }

        /** Arm above the band, confirm, one seeing tick: an APPROACH descending toward it. */
        fun approaching(relAlt: Double = 10.0): Harness {
            seeTag(0.0, 0.0)
            place(northM = 0.0, eastM = 0.0, relAlt = relAlt)
            check(arm() == Verdict.ACCEPTED)
            confirm()
            tickSeeing()
            return this
        }

        // ------- the landing08 shapes: a flight on which nothing ever arrives on the wire -----

        /** One tick during which the camera sees the tag again but no wire carries anything. */
        fun tickSeeingSilent(advanceMs: Long = 100) {
            now += advanceMs
            seeTag(fixNorth ?: 0.0, fixEast ?: 0.0)
            engine.tick(now)
        }

        /** DJI confirms the engagement with the link as silent as landing08's was. */
        fun confirmSilent() {
            port.enableOnSuccess?.invoke()
            port.onState!!(VirtualStickSnapshot(enabled = true, advanced = true, authority = "MSDK"))
            tickSeeingSilent(40)
        }

        /**
         * The phone-armed descent in steady TRACKING with **zero MAVLink traffic ever** —
         * landing08's whole flight shape, past the arm it was denied at.
         */
        fun descendingFromPhone(): Harness {
            seeTag(0.0, 0.0)
            place(northM = 0.0, eastM = 0.0, relAlt = ALT)
            check(engine.armTagDescentFromPhone() == Verdict.ACCEPTED)
            confirmSilent()
            tickSeeingSilent()
            return this
        }

        companion object {
            fun stateAt(
                latDeg: Double = LAT, lonDeg: Double = LON, relAlt: Double? = ALT,
                positionAge: Long = 0L, datum: Double? = DATUM,
                homeLat: Double = LAT, homeLon: Double = LON,
            ) = AircraftState(
                latitude = latDeg, longitude = lonDeg,
                homeLatitude = homeLat, homeLongitude = homeLon,
                relativeAltitude = relAlt, takeoffAltitudeAmsl = datum,
                velocityNorth = 0.0, velocityEast = 0.0, velocityDown = 0.0,
                yawDeg = 0.0,
                ages = SampleAges.of(
                    Signal.POSITION to positionAge,
                    Signal.ALTITUDE to 0L,
                    Signal.VELOCITY to 0L,
                    Signal.ATTITUDE to 0L,
                ),
            )
        }
    }

    private fun lastDescentCmd(h: Harness): RecordedCmd {
        val cmd = h.cmds.lastOrNull { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE }
        assertNotNull("no TAG_DESCENT setpoint was recorded", cmd)
        return cmd!!
    }

    // ---------------------------------------------------------- arm, and the take

    @Test
    fun `an arm while idle is ACCEPTED, engages, and flies only after DJI confirms`() {
        val h = Harness()
        h.seeTag(0.0, 0.0)
        assertEquals(Verdict.ACCEPTED, h.arm())
        assertEquals(GuidedPhase.ENGAGING, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_ARMED))
        assertTrue(h.events.any { it.first == "tag_descent_armed" })
        h.tickAlive()
        assertEquals("nothing may flow before DJI confirms", 0, h.port.sent.size)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertTrue(h.texts().contains(GuidedStatusTexts.ENGAGED_DESCENT))
        assertNotNull(h.engine.situation().descent)
    }

    @Test
    fun `an arm reuses an existing stick engagement - one enable, one owner`() {
        val h = Harness()
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertEquals(1, h.port.enableCalls)
        h.seeTag(0.0, 0.0)
        assertEquals(Verdict.ACCEPTED, h.arm())
        assertEquals(1, h.port.enableCalls)
    }

    // ------------------------------------------- refusals, every one with a sentence

    private fun assertRefused(h: Harness, verdict: Verdict, saidContaining: String) {
        assertEquals(Verdict.DENIED, verdict)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_denied" })
        assertFalse(h.events.any { it.first == "tag_descent_armed" })
        assertTrue(
            "expected a sentence containing '$saidContaining', said: ${h.texts()}",
            h.texts().any { it.contains(saidContaining) },
        )
    }

    @Test
    fun `the interlock off answers UNSUPPORTED with no sentence - the pre-feature reply`() {
        val h = Harness()
        h.interlock = false
        h.seeTag()
        assertEquals(Verdict.UNSUPPORTED, h.arm())
        assertTrue(h.texts().isEmpty())
        assertTrue(h.events.none { it.first.startsWith("tag_descent") })
    }

    @Test
    fun `no detector wired - refused by name`() {
        val h = Harness()
        h.senseAbsent = true
        assertRefused(h, h.arm(), GuidedStatusTexts.REASON_NO_DETECTOR)
    }

    @Test
    fun `no latch - refused by name, however fresh the fix claims to be`() {
        val h = Harness()
        h.latchedId = null
        h.seeTag()
        assertRefused(h, h.arm(), GuidedStatusTexts.REASON_NO_TAG)
    }

    @Test
    fun `a fix older than the arm bound - refused - and one at the bound arms`() {
        val stale = Harness()
        stale.seeTag()
        stale.fixAtMs = stale.now - (TagDescentGuidance.ARM_FRESH_MS + 1)
        assertRefused(stale, stale.arm(), GuidedStatusTexts.REASON_TAG_STALE)

        val edge = Harness()
        edge.seeTag()
        edge.fixAtMs = edge.now - TagDescentGuidance.ARM_FRESH_MS
        assertEquals(Verdict.ACCEPTED, edge.arm())
    }

    @Test
    fun `a fix of a different id than the latch is not evidence - refused`() {
        val h = Harness()
        h.seeTag()
        h.fixId = 46 // the measured false-id shape: a wrong id alongside the true tag
        assertRefused(h, h.arm(), GuidedStatusTexts.REASON_TAG_STALE)
    }

    // The named successor of `above the measured detection band - refused with the number`:
    // since the approach (Ivan's brief, 2026-07-29, after landing13 t=41.8), above the band
    // is ACCEPTED into the approach segment, and the refusal moved up to the decode-reach
    // ceiling. Below-band arms are pinned unchanged.
    @Test
    fun `above the 7 m band the arm approaches instead of refusing - below it, unchanged`() {
        val above = Harness()
        above.place(relAlt = TagDescentGuidance.ARM_CEILING_M + 0.5)
        above.seeTag()
        assertEquals(Verdict.ACCEPTED, above.arm())
        assertEquals(true, above.engine.situation().descent?.approach)

        val atBand = Harness()
        atBand.place(relAlt = TagDescentGuidance.ARM_CEILING_M)
        atBand.seeTag()
        assertEquals(Verdict.ACCEPTED, atBand.arm())
        assertEquals(
            "an arm at or below the ceiling must never enter APPROACH",
            false, atBand.engine.situation().descent?.approach,
        )
    }

    @Test
    fun `above the decode reach - refused with the number - at the approach ceiling arms`() {
        val high = Harness()
        high.place(relAlt = TagDescentGuidance.APPROACH_CEILING_M + 0.5)
        high.seeTag()
        assertRefused(high, high.arm(), "12m")

        val edge = Harness()
        edge.place(relAlt = TagDescentGuidance.APPROACH_CEILING_M)
        edge.seeTag()
        assertEquals(Verdict.ACCEPTED, edge.arm())
        assertEquals(true, edge.engine.situation().descent?.approach)
    }

    @Test
    fun `the approach arm is recorded with its entry height and marker - and announced`() {
        val h = Harness()
        h.place(relAlt = 10.2)
        h.seeTag()
        assertEquals(Verdict.ACCEPTED, h.arm())
        val armed = h.events.last { it.first == "tag_descent_armed" }
        assertTrue(
            "the record must carry the entry height and the approach marker: ${armed.second}",
            armed.second!!.contains("height=10.2") && armed.second!!.endsWith(" approach"),
        )
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_APPROACH))
    }

    @Test
    fun `an arm above the band skips no other gate - nadir, staleness and RC bind at 10 m`() {
        // The ceiling is the ONLY conjunct the approach relaxes. One harness per gate,
        // all placed at the 10 m takeoff hover the feature exists for.
        val nadir = Harness()
        nadir.place(relAlt = 10.0)
        nadir.seeTag()
        nadir.cameraPitch = 0.0
        assertRefused(nadir, nadir.arm(), GuidedStatusTexts.REASON_NOT_NADIR)

        val stale = Harness()
        stale.place(relAlt = 10.0)
        stale.seeTag()
        stale.fixAtMs = stale.now - (TagDescentGuidance.ARM_FRESH_MS + 1)
        assertRefused(stale, stale.arm(), GuidedStatusTexts.REASON_TAG_STALE)

        val noRc = Harness()
        noRc.place(relAlt = 10.0)
        noRc.port.onRc!!(RcSticks(null, 0, 0, 0))
        noRc.seeTag()
        assertRefused(noRc, noRc.arm(), "NO_RC_FEED")
    }

    @Test
    fun `camera not commanded to nadir - refused - the tolerance edge arms`() {
        for (pitch in listOf(null, 0.0, -60.0, -90.0 - TagDescentGuidance.NADIR_TOLERANCE_DEG - 0.1)) {
            val h = Harness()
            h.cameraPitch = pitch
            h.seeTag()
            assertRefused(h, h.arm(), GuidedStatusTexts.REASON_NOT_NADIR)
        }
        val edge = Harness()
        edge.cameraPitch = -90.0 + TagDescentGuidance.NADIR_TOLERANCE_DEG
        edge.seeTag()
        assertEquals(Verdict.ACCEPTED, edge.arm())
    }

    /**
     * **The flown bug, pinned in its exact shape** (records 20260728-213858 and -214210): the
     * camera was never aimed by the app, so `TagWorld.fix` refused every sighting and the fix
     * was absent — and the old gate order blamed the tag (`TAG_NOT_IN_VIEW`, 20 ms after a
     * sighting) instead of the camera, which cost Ivan the session. A camera problem must name
     * the camera even when its downstream symptom — no fix — is also present.
     */
    @Test
    fun `camera never commanded nor reported and no fix at all - refused by naming the camera, not the tag`() {
        val h = Harness()
        h.cameraPitch = null
        h.fixNorth = null
        h.fixEast = null
        h.fixAtMs = null
        assertRefused(h, h.arm(), GuidedStatusTexts.REASON_NOT_NADIR)
        assertFalse(
            "the refusal must not blame the tag for a camera fault",
            h.texts().any { it.contains(GuidedStatusTexts.REASON_TAG_STALE) },
        )
    }

    /** Cause before symptom holds for an off-nadir camera too, not only for a missing belief. */
    @Test
    fun `camera off nadir with a stale fix - the camera is named, not the tag`() {
        val h = Harness()
        h.cameraPitch = -60.0
        h.seeTag()
        h.fixAtMs = h.now - (TagDescentGuidance.ARM_FRESH_MS + 500)
        assertRefused(h, h.arm(), GuidedStatusTexts.REASON_NOT_NADIR)
        assertFalse(h.texts().any { it.contains(GuidedStatusTexts.REASON_TAG_STALE) })
    }

    @Test
    fun `another manoeuvre flying - refused, never silently replaced`() {
        val h = Harness()
        h.engine.reposition(
            RepositionCommand(
                isCommandInt = true, frame = 0,
                latE7 = (latNorthOf(50.0) * 1e7).roundToInt(), lonE7 = (LON * 1e7).roundToInt(),
                zAmslM = (DATUM + ALT).toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
            )
        ).also { assertEquals(Verdict.ACCEPTED, it) }
        h.seeTag()
        assertEquals(Verdict.DENIED, h.engine.armTagDescent())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_DESCENT_BUSY) })
        // The goto is untouched: refusing the descent must not have cost the manoeuvre.
        assertNotNull(h.engine.situation().goto)
    }

    @Test
    fun `the commanding controller never heard from - refused - the descent inherits the link watchdog`() {
        val h = Harness()
        h.seeTag()
        // Deliberately not h.arm(): no inbound traffic ever.
        assertEquals(Verdict.DENIED, h.engine.armTagDescent())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_LINK_DOWN) })
    }

    @Test
    fun `no RC feed - refused - abort gesture 1 must not be blind`() {
        val h = Harness()
        h.port.onRc!!(RcSticks(null, 0, 0, 0))
        h.seeTag()
        assertRefused(h, h.arm(), "NO_RC_FEED")
    }

    @Test
    fun `no fresh position, no home, no usable altitude - each refused by name`() {
        val noFix = Harness()
        noFix.place(positionAge = 10_000L)
        noFix.seeTag()
        assertRefused(noFix, noFix.arm(), GuidedStatusTexts.REASON_NO_FIX)

        val noHome = Harness()
        noHome.state = noHome.state.copy(homeLatitude = null, homeLongitude = null)
        noHome.seeTag()
        assertRefused(noHome, noHome.arm(), GuidedStatusTexts.REASON_NO_HOME)

        val noAlt = Harness()
        noAlt.place(relAlt = null)
        noAlt.seeTag()
        assertRefused(noAlt, noAlt.arm(), GuidedStatusTexts.REASON_NO_DATUM)
    }

    // ----------------------------------------------------- tracking, cone, ladder

    @Test
    fun `tracking flies toward the tag and descends only inside the cone`() {
        val h = Harness().descending()
        // Aircraft 2 m north of the tag at 5 m: outside the 0.75 m cone — centring only.
        h.place(northM = 2.0)
        h.tickSeeing()
        var sent = h.port.sent.last()
        assertTrue("must fly south toward the tag (roll = north)", sent.roll < 0.0)
        assertEquals("no descent outside the cone", 0.0, sent.verticalThrottle, 1e-9)

        // Inside the cone: descending, at the cap this far above the target.
        h.place(northM = 0.1)
        h.tickSeeing()
        sent = h.port.sent.last()
        assertEquals(-TagDescentGuidance.V_DESCENT_MAX_MS, sent.verticalThrottle, 1e-9)
        assertEquals("the descent generates no yaw", 0.0, sent.yaw, 1e-9)
        val cmd = lastDescentCmd(h)
        assertEquals(GuidedStickEngine.TAG_DESCENT_SOURCE, cmd.source?.messageName)
    }

    @Test
    fun `the error is the fix minus the aircraft, in the home frame - east too`() {
        val h = Harness().descending()
        h.seeTag(northM = 0.0, eastM = 3.0)
        h.place(northM = 0.0, eastM = 0.0)
        h.tickSeeing()
        val sent = h.port.sent.last()
        assertTrue("tag east of the aircraft must command east (pitch = east)", sent.pitch > 0.0)
        assertEquals(0.0, sent.roll, 1e-6)
    }

    @Test
    fun `a stale fix halts the descent - announced, recorded - and a returning tag resumes tracking`() {
        val h = Harness().descending()
        h.place(northM = 0.0)
        // Let the fix age past T_HOLD without a new sighting: ticks advance, no seeTag.
        repeat(5) { h.tickAlive() }
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_HOLDING))
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second!!.startsWith("holding") })
        assertEquals("descent must halt on a stale fix", 0.0, h.port.sent.last().verticalThrottle, 1e-9)

        // The tag comes back: tracking again, announced.
        h.tickSeeing()
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_TRACKING))
        assertTrue(h.port.sent.last().verticalThrottle < 0.0)
    }

    @Test
    fun `past T_CLIMB the aircraft climbs to reacquire - and the climb stops at the arm ceiling`() {
        val h = Harness().descending()
        repeat(21) { h.tickAlive() } // ~2.1 s without a sighting
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_CLIMBING))
        assertEquals(
            "a slow climb, up-positive at DJI",
            TagDescentGuidance.V_REACQUIRE_MS, h.port.sent.last().verticalThrottle, 1e-9,
        )
        // At the ceiling the climb stops — bounded, not a fly-away.
        h.place(relAlt = TagDescentGuidance.ARM_CEILING_M)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `past T_ABORT the descent hands back - hover wind-down, announced, recorded`() {
        val h = Harness().descending()
        repeat(101) { h.tickAlive() } // ~10.1 s without a sighting
        assertNull("the descent is dead", h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second!!.startsWith("tag gone") })
        assertTrue(h.texts().any { it.contains("tag-lost") })
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
        // The wind-down completes (500 ms ramp + 1000 ms hold) and DJI is asked to let go.
        repeat(20) { h.tickAlive() }
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertTrue(h.port.disableCalls > 0)
    }

    // ----------------------------------------------------- the approach segment

    @Test
    fun `the approach descends toward the band on baro height - inside its wide cone only`() {
        val h = Harness().approaching()
        val sent = h.port.sent.last()
        assertEquals(-TagDescentGuidance.V_DESCENT_MAX_MS, sent.verticalThrottle, 1e-9)
        assertEquals(0.0, sent.yaw, 1e-9)
        // Off-centre by more than the 1.5 m cone at 10 m: centring only, no descent.
        h.place(northM = 2.5, relAlt = 10.0)
        h.tickSeeing()
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertTrue("must fly south toward the tag", h.port.sent.last().roll < 0.0)
    }

    @Test
    fun `the band entry hands off seamlessly - recorded by name, said in its own words`() {
        val h = Harness().approaching()
        h.place(relAlt = TagDescentGuidance.APPROACH_BAND_ENTRY_M - 0.1)
        h.tickSeeing()
        assertTrue(
            "the handoff must be a named record line",
            h.events.any {
                it.first == "tag_descent_phase" &&
                    it.second!!.startsWith("tracking") && it.second!!.contains("band_entry")
            },
        )
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_BAND_ENTRY))
        assertEquals(false, h.engine.situation().descent?.approach)
        // From here it is today's machine, mid-descent inside the band.
        assertTrue(h.port.sent.last().verticalThrottle < 0.0)
    }

    @Test
    fun `sparse fixes are the approach's weather - descend through short gaps, hold long ones`() {
        val h = Harness().approaching()
        // 1.2 s without a decode (the HOLD rung — the measured gap shape at 8-10 m): the
        // approach keeps descending, blind-bounded by the climb rung.
        repeat(12) { h.tickAlive() }
        assertTrue(h.port.sent.last().verticalThrottle < 0.0)
        // Past T_CLIMB: hold — and never a climb up here, nor a CLIMBING announcement; the
        // approach has no phase edges of its own between arm and band entry.
        repeat(10) { h.tickAlive() }
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertFalse(h.texts().contains(GuidedStatusTexts.DESCENT_CLIMBING))
        assertEquals(true, h.engine.situation().descent?.approach)
        // A decode burst resumes the descent.
        h.tickSeeing()
        assertTrue(h.port.sent.last().verticalThrottle < 0.0)
    }

    @Test
    fun `the approach past T_ABORT hands back - the tag cannot be seen from this height`() {
        val h = Harness().approaching()
        repeat(101) { h.tickAlive() } // ~10.1 s without a decode
        assertNull("the approach is dead", h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second!!.startsWith("tag gone") })
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
    }

    @Test
    fun `shadow arms above the band into the approach - same gate, same label, no actuation`() {
        val h = Harness()
        h.seeTag()
        h.place(relAlt = 10.0)
        assertTrue(h.engine.setShadowDescent(true))
        h.tickAlive()
        val armed = h.events.last { it.first == "tag_descent_armed" }
        assertTrue(
            "the shadow's armed line must carry the approach marker: ${armed.second}",
            armed.second!!.startsWith("shadow") && armed.second!!.endsWith(" approach"),
        )
        repeat(5) { h.tickSeeing() }
        assertEquals("shadow still actuates nothing", 0, h.port.sent.size)
        assertEquals(true, h.engine.situation().descent?.approach)
    }

    // ------------------------------------------------------------------ terminal

    private fun reachTerminal(h: Harness): Harness {
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        repeat(TagDescentGuidance.TERMINAL_TICKS) { h.tickSeeing() }
        assertTrue(h.engine.situation().descent!!.terminal)
        return h
    }

    @Test
    fun `at the target, centred, held - stage B complete, holding, announced and recorded`() {
        val h = reachTerminal(Harness().descending())
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_COMPLETE))
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second!!.startsWith("terminal") })
        // Holding: at the target exactly, nothing vertical is commanded. (Slightly above it
        // the hold legitimately trims back down toward the target — that is the hold working,
        // and `the terminal hold never descends below the target` pins the direction bound.)
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M)
        h.tickSeeing()
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
    }

    @Test
    fun `the terminal hold never descends below the target`() {
        val h = reachTerminal(Harness().descending())
        // The aircraft sinks below the target (gust, baro step): the hold climbs back, gently.
        h.place(northM = 0.05, relAlt = 0.4)
        h.tickSeeing()
        assertTrue(
            "below the target the hold must climb or hold, never descend",
            h.port.sent.last().verticalThrottle >= 0.0,
        )
        assertTrue(h.port.sent.last().verticalThrottle > 0.0)
    }

    @Test
    fun `a fly-through past the target height cannot declare the stage complete`() {
        val h = Harness().descending()
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        repeat(TagDescentGuidance.TERMINAL_TICKS - 1) { h.tickSeeing() }
        // One tick blown off-centre resets the count.
        h.place(northM = 1.0, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        h.tickSeeing()
        h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
        repeat(TagDescentGuidance.TERMINAL_TICKS - 1) { h.tickSeeing() }
        assertFalse(h.engine.situation().descent!!.terminal)
    }

    @Test
    fun `the terminal hold is bounded by the idle window - authority is not held forever`() {
        val h = reachTerminal(Harness().descending())
        var remaining = GuidedEnvelope.IDLE_DISENGAGE_MS / 100 + 2
        while (remaining-- > 0 && h.engine.situation().descent != null) h.tickSeeing()
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "idle after terminal" })
    }

    // ------------------------------------------------- rule 1: the sticks, verbatim

    @Test
    fun `a breathed-on stick does not cancel - below the dead-band, however long it lasts`() {
        val h = Harness().descending()
        // 5 % of travel: transmitter slop territory, half the dead-band.
        h.port.onRc!!(RcSticks(33, 0, 0, 0))
        repeat(10) { h.tickSeeing() }
        assertNotNull("a breathed-on stick must not cancel", h.engine.situation().descent)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `a deliberate nudge sustained cancels completely - aircraft handed back, descent dead`() {
        val h = Harness().descending()
        // 15 % of travel — a real hand — sustained past the debounce.
        h.port.onRc!!(RcSticks(100, 0, 0, 0))
        h.tickSeeing(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals("total disengage, not a downgrade", GuidedPhase.IDLE, h.engine.phase)
        assertNull("the manoeuvre is dead", h.engine.situation().descent)
        assertTrue(h.texts().any { it.contains("Virtual stick off: sticks") })
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "sticks" })
        assertTrue("the aircraft is handed back", h.port.disableCalls > 0)
    }

    @Test
    fun `a nudge shorter than the debounce does not cancel - a knocked stick is not a hand`() {
        val h = Harness().descending()
        h.port.onRc!!(RcSticks(100, 0, 0, 0))
        h.tickSeeing(GuidedStickEngine.RC_ABORT_SUSTAIN_MS - 150)
        assertNotNull(h.engine.situation().descent)
        // The stick returns to centre before the debounce elapses: the clock resets.
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        repeat(5) { h.tickSeeing() }
        assertNotNull(h.engine.situation().descent)
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `after an RC grab the descent is gone - re-engaging does not resume it, arming again does`() {
        val h = Harness().descending()
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        h.tickSeeing(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        val sendsAfterAbort = h.cmds.size

        // The operator re-engages the GCS sticks: passthrough, and only passthrough. (Past the
        // engage retry window, which exists so a held stick cannot hammer DJI.)
        h.now += GuidedStickEngine.ENGAGE_RETRY_MS
        h.frame()
        h.now += 40
        h.frame(x = 500)
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertNull("no descent resumed itself", h.engine.situation().descent)
        assertTrue(
            "no TAG_DESCENT setpoint after the grab without a fresh arm",
            h.cmds.drop(sendsAfterAbort).none { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SOURCE },
        )

        // A fresh deliberate arm — through every gate — is the only way back down.
        h.seeTag(0.0, 0.0)
        assertEquals(Verdict.ACCEPTED, h.arm())
        assertNotNull(h.engine.situation().descent)
    }

    @Test
    fun `a deliberate GCS stick deflection cancels the descent - the operator chose that channel`() {
        val h = Harness().descending()
        h.frame() // neutral, recently at rest
        h.now += 40
        h.frame(x = 500)
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_STICKS))
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "sticks" })
        assertEquals("passthrough has the aircraft", GuidedPhase.ENGAGED, h.engine.phase)
    }

    // ------------------------------- the commanding origin: landing08, the phone-only flight
    //
    // datasets/landing08/20260729-112216.001.jsonl, 2026-07-29: the first flight with no QGC —
    // zero mav_in lines — and every arm press refused `tag_descent_denied LINK_DOWN` (t=51.4,
    // 60.2, 63.4, 65.2, 71.5, 76.9), because the arm was judged as a MAVLINK-origin engagement
    // and demanded a heartbeat from a ground station that was never there. The phone door
    // (armTagDescentFromPhone → ControlOrigin.PHONE) is the fix; these tests pin it, and pin
    // what deliberately did NOT weaken with it: the RC-feed gate, rule 1 on both channels, and
    // the MAVLINK arm's own heartbeat demand.

    @Test
    fun `landing08 fixed - a phone arm with zero MAVLink traffic ever is accepted, not LINK_DOWN`() {
        val h = Harness()
        h.seeTag(0.0, 0.0)
        h.place()
        // No h.arm() heartbeat, no onInbound of any kind — the record's exact link state.
        assertEquals(Verdict.ACCEPTED, h.engine.armTagDescentFromPhone())
        assertTrue(h.texts().none { it.contains(GuidedStatusTexts.REASON_LINK_DOWN) })
        // The label is on the record, so the next landing08 is diagnosable from the JSONL alone.
        assertTrue(h.events.any { it.first == "tag_descent_armed" && it.second!!.contains("origin=phone") })
    }

    @Test
    fun `a phone descent flies through total MAVLink silence - the operator is holding the watchdog`() {
        val h = Harness().descendingFromPhone()
        // Far past LINK_LOST_MS with not one inbound byte: the engagement must hold and the
        // descent must still be flying on its sensor, not dying on a link it never had.
        repeat(((GuidedEnvelope.LINK_LOST_MS * 4) / 100).toInt()) { h.tickSeeingSilent() }
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        assertNotNull("the descent is alive", h.engine.situation().descent)
        assertTrue(h.events.none { it.first == "tag_descent_ended" && it.second == "link-lost" })
        assertTrue(h.texts().none { it.contains("link-lost") })
    }

    @Test
    fun `a MAVLINK arm still demands the heartbeat - landing08's fix does not leak onto the seam`() {
        // The transport seam (armTagDescent's MAVLINK default — the door a future QGC handler
        // enters by) keeps the LINK_DOWN rung byte-for-byte: never heard from means refused.
        val h = Harness()
        h.seeTag(0.0, 0.0)
        h.place()
        assertEquals(Verdict.DENIED, h.engine.armTagDescent())
        assertTrue(h.texts().any { it.contains(GuidedStatusTexts.REASON_LINK_DOWN) })
    }

    @Test
    fun `NO_RC_FEED refuses a phone arm exactly as before - gesture 1 must not be blind`() {
        // The one link a phone flight genuinely depends on is the physical RC: it is abort
        // gesture 1, and PHONE's alive-by-identity liveness must not soften this gate by a byte.
        val h = Harness()
        h.port.onRc!!(RcSticks(0, null, 0, 0))
        h.seeTag(0.0, 0.0)
        h.place()
        assertEquals(Verdict.DENIED, h.engine.armTagDescentFromPhone())
        assertTrue(h.texts().any { it.contains("NO_RC_FEED") })
    }

    @Test
    fun `an RC grab cancels a phone descent completely - rule 1 gesture 1 unweakened`() {
        val h = Harness().descendingFromPhone()
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        h.tickSeeingSilent(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals("total disengage, not a downgrade", GuidedPhase.IDLE, h.engine.phase)
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "sticks" })
    }

    @Test
    fun `a QGC that connects mid-flight and deflects cancels a phone descent - rule 1 channel 2`() {
        // Landing08's inverse: the phone flight that stops being phone-only. A ground station
        // appearing mid-descent and deflecting its sticks must win over a PHONE-origin
        // engagement exactly as over a MAVLINK one — rule 1 is absolute, and the sticks' sender
        // owns the engagement from that moment.
        val h = Harness().descendingFromPhone()
        h.frame() // QGC arrives: a frame at rest…
        h.now += 40
        h.frame(x = 500) // …then a deliberate deflection
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_STICKS))
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "sticks" })
        assertEquals("passthrough has the aircraft", GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `shadow mode runs on a phone-only flight - segments arm and hold through total silence`() {
        // The shadow gate judges arms at the origin a live arm would carry (PHONE), and the
        // shadow's mirrored link rung is gone with the live one it mirrored: a hand-flown tag
        // landing with no QGC — the exact flight shadow mode exists for — must produce a
        // continuous timeline, not `shadow blocked LINK_DOWN` from birth.
        val h = Harness()
        h.seeTag(0.0, 0.0)
        h.place(northM = 0.0, relAlt = ALT)
        assertTrue(h.engine.setShadowDescent(true))
        h.tickSeeingSilent()
        assertTrue(h.events.any { it.first == "tag_descent_armed" && it.second!!.startsWith("shadow") })
        assertTrue(h.events.none { it.second?.contains("shadow blocked ${GuidedStatusTexts.REASON_LINK_DOWN}") == true })
        repeat(((GuidedEnvelope.LINK_LOST_MS * 2) / 100).toInt()) { h.tickSeeingSilent() }
        assertNotNull("the segment held through the silence", h.engine.situation().descent)
        assertTrue(h.events.none { it.first == "tag_descent_ended" && it.second == "shadow link-lost" })
    }

    // --------------------------------------------------------- the other cancels

    @Test
    fun `disarm from the phone - descent dead, aircraft holding, announced - and idempotent`() {
        val h = Harness().descending()
        assertEquals(Verdict.ACCEPTED, h.engine.disarmTagDescent())
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_DISARMED))
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "disarmed" })
        // Still engaged, holding station at the synthesised arrival hold.
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.tickAlive()
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertEquals(0.0, h.port.sent.last().roll, 1e-6)
        // A second disarm finds nothing, which is the truth.
        assertEquals(Verdict.DENIED, h.engine.disarmTagDescent())
    }

    @Test
    fun `pause ends the descent the way disarm does - one withdrawal grammar`() {
        val h = Harness().descending()
        assertEquals(Verdict.ACCEPTED, h.engine.pause())
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "paused" })
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_DISARMED))
    }

    @Test
    fun `the latch dying mid-descent cancels to a hold, announced`() {
        val h = Harness().descending()
        h.latchedId = null // a new flight began, or the detector was stopped
        h.tickSeeing()
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_LATCH_LOST))
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "latch lost" })
        assertEquals("ours and holding", GuidedPhase.ENGAGED, h.engine.phase)
    }

    @Test
    fun `the camera leaving nadir mid-descent cancels to a hold - refusal-grade inconsistency`() {
        val h = Harness().descending()
        h.cameraPitch = -60.0
        h.tickSeeing()
        assertNull(h.engine.situation().descent)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_GIMBAL))
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second!!.startsWith("camera left nadir") })
    }

    @Test
    fun `the interlock going off mid-descent aborts completely`() {
        val h = Harness().descending()
        h.interlock = false
        h.tickSeeing()
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "interlock" })
    }

    @Test
    fun `total link silence mid-descent runs the armed policy - the inherited watchdog, no second one`() {
        val h = Harness().descending()
        // Ticks without any inbound traffic: the fix also goes stale, but the link watchdog
        // sits above the ladder and fires first at LINK_LOST_MS.
        repeat(31) { h.tick(100) }
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "link-lost" })
        assertEquals(GuidedPhase.RELEASING, h.engine.phase)
    }

    @Test
    fun `a goto replaces the descent, recorded - an acknowledged command outranks it`() {
        val h = Harness().descending()
        val verdict = h.engine.reposition(
            RepositionCommand(
                isCommandInt = true, frame = 0,
                latE7 = (latNorthOf(50.0) * 1e7).roundToInt(), lonE7 = (LON * 1e7).roundToInt(),
                zAmslM = (DATUM + ALT).toFloat(), groundSpeedMs = -1f, yawRad = Float.NaN,
            )
        )
        assertEquals(Verdict.ACCEPTED, verdict)
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "replaced by goto" })
    }

    @Test
    fun `a stale position fix is never centred on - zero, held, then released past the bound`() {
        val h = Harness().descending()
        h.place(northM = 2.0, positionAge = 5_000L)
        h.tickSeeing()
        assertEquals(0.0, h.port.sent.last().roll, 1e-9)
        assertEquals(0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_POSITION_HOLD))
        // Past POSITION_LOST_MS the engine releases entirely.
        repeat(101) { h.tickSeeing() }
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "no position fix" })
    }

    @Test
    fun `unknown altitude mid-descent - centring continues, nothing vertical, announced`() {
        val h = Harness().descending()
        h.place(northM = 2.0, relAlt = null)
        h.tickSeeing()
        val sent = h.port.sent.last()
        assertTrue("centring continues", sent.roll < 0.0)
        assertEquals("nothing vertical without a height", 0.0, sent.verticalThrottle, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.NO_ALTITUDE))
    }

    @Test
    fun `a fix of the wrong id mid-descent is ignored - the believed fix ages instead`() {
        val h = Harness().descending()
        h.fixId = 102 // the measured false-id shape
        // The wrong-id "fresh" fix must not rejuvenate the ladder: with only it arriving, the
        // believed fix ages and the descent holds.
        repeat(5) {
            h.now += 100
            h.fixAtMs = h.now // the impostor is always fresh
            h.engine.onInbound("heartbeat")
            h.engine.tick(h.now)
        }
        assertEquals("a false id must not keep the descent alive", 0.0, h.port.sent.last().verticalThrottle, 1e-9)
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_HOLDING))
    }

    @Test
    fun `every descent setpoint is recorded with its own source name`() {
        val h = Harness().descending()
        h.tickSeeing()
        val cmd = lastDescentCmd(h)
        assertEquals(GuidedStickEngine.TAG_DESCENT_SOURCE, cmd.source?.messageName)
        assertNotNull(cmd.setpoint)
        assertEquals(true, cmd.accepted)
    }

    @Test
    fun `the situation exposes the descent for the screen, and a withdrawal has something to act on`() {
        val h = Harness().descending()
        val situation = h.engine.situation()
        assertNotNull(situation.descent)
        assertEquals(0, situation.descent!!.tagId)
        assertFalse(situation.descent!!.terminal)
        assertTrue(situation.hasManoeuvre)
    }

    // ------------------------------------------------- flight replay parity

    /**
     * One instant of the first live engagement, exactly as flight `20260728-151517` recorded
     * it: the `dji_state` at-or-before the command, the newest `tag` fix at-or-before it with
     * its true age, and the `stick_cmd` the aircraft was actually given.
     */
    private data class FlightRow(
        val latDeg: Double, val lonDeg: Double, val relAlt: Double,
        val fixNorthM: Double, val fixEastM: Double, val fixAgeMs: Long,
        val recordedVn: Double, val recordedVe: Double, val recordedVd: Double,
    )

    /**
     * **The flight that flew, replayed** — engagement 1 of `20260728-151517` (t = 83.6–86.9 s,
     * ended by Ivan's thumb, rule 1's first firing in the air). Six recorded instants driven
     * through the engine; the engine must reproduce the recorded commands.
     *
     * This is the test that settles the frame question with ground truth, and the one that
     * would catch any future frame regression between the fix's home-anchored north/east and
     * the engine's error arithmetic: the recorded commands converged the aircraft from 0.52 m
     * of lateral error to 0.19 m in 3.3 s (and engagement 2 reached **0.03 m**), so an engine
     * that reproduces them byte-for-near-byte is computing the same frame the aircraft flew.
     * It also pins the property directly: every command's dot product with (fix − aircraft)
     * is positive — the command points at the tag, whatever the fix's coordinates from home
     * happen to be (comparing a command against the fix's *absolute* direction is the analysis
     * error that briefly read this flight as a frame bug: at t=83.9 the fix sat at
     * (+0.21, +0.26) from home but the aircraft at (+0.02, +0.75), so the honest command
     * points north-WEST while the fix-from-home vector points north-east — 85° apart, both
     * right).
     */
    @Test
    fun `flight 20260728-151517 engagement 1 replays - recorded states reproduce recorded commands`() {
        val homeLat = 37.9938362
        val homeLon = 23.7253199
        val rows = listOf(
            FlightRow(37.9938364, 23.7253284, 4.6, 0.209, 0.260, 68, 0.099, -0.244, 0.4), // t=83.928
            FlightRow(37.9938361, 23.7253276, 4.5, 0.200, 0.274, 25, 0.107, -0.203, 0.4), // t=84.527
            FlightRow(37.9938366, 23.7253258, 4.2, 0.188, 0.297, 95, 0.077, -0.110, 0.4), // t=85.127
            FlightRow(37.9938366, 23.7253248, 4.0, 0.183, 0.309, 52, 0.072, -0.059, 0.4), // t=85.727
            FlightRow(37.9938364, 23.7253247, 3.7, 0.176, 0.321, 12, 0.078, -0.052, 0.4), // t=86.327
            FlightRow(37.9938362, 23.7253247, 3.5, 0.171, 0.327, 81, 0.085, -0.046, 0.4), // t=86.827
        )

        val h = Harness()
        fun apply(row: FlightRow, atTickMs: Long) {
            h.state = Harness.stateAt(
                latDeg = row.latDeg, lonDeg = row.lonDeg, relAlt = row.relAlt,
                homeLat = homeLat, homeLon = homeLon,
            )
            h.fixNorth = row.fixNorthM
            h.fixEast = row.fixEastM
            h.fixAtMs = atTickMs - row.fixAgeMs
        }
        apply(rows[0], h.now)
        assertEquals(Verdict.ACCEPTED, h.arm())
        h.confirm()

        for (row in rows) {
            apply(row, atTickMs = h.now + 100)
            h.tickAlive()
            val sp = lastDescentCmd(h).setpoint!!
            // The recorded command, reproduced. 0.02 m/s of tolerance covers the recorded
            // fix/state being quantised (1e-7 deg of latitude is 1.1 cm) and the sub-tick
            // timing between the record's state line and the engine's snapshot.
            assertEquals("vn at fix(${row.fixNorthM},${row.fixEastM})", row.recordedVn, sp.north!!, 0.02)
            assertEquals("ve at fix(${row.fixNorthM},${row.fixEastM})", row.recordedVe, sp.east!!, 0.02)
            assertEquals(row.recordedVd, sp.down!!, 0.02)
            // And the frame property itself, independent of the recording: the command points
            // from the aircraft toward the fix.
            val (acN, acE) = RepositionGuidance.nedMetres(homeLat, homeLon, row.latDeg, row.lonDeg)
            val errN = row.fixNorthM - acN
            val errE = row.fixEastM - acE
            assertTrue(
                "command (${sp.north},${sp.east}) must point at the error ($errN,$errE)",
                sp.north!! * errN + sp.east!! * errE > 0.0,
            )
        }
    }

    // ------------------------------------------------------------- shadow mode

    /** Shadow mode on, gates holding, one tick taken: a shadow segment in steady TRACKING. */
    private fun shadowing(): Harness {
        val h = Harness()
        h.seeTag(0.0, 0.0)
        h.place(northM = 0.0, relAlt = ALT)
        assertTrue(h.engine.setShadowDescent(true))
        h.tickAlive()
        return h
    }

    @Test
    fun `shadow mode arms itself when the gates hold - and actuates nothing, ever`() {
        val h = shadowing()
        assertTrue(h.texts().contains(GuidedStatusTexts.DESCENT_SHADOW_ON))
        assertTrue(h.events.any { it.first == "tag_descent_armed" && it.second!!.startsWith("shadow") })
        assertEquals("no engagement is ever sought", GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.enableCalls)
        repeat(10) { h.tickSeeing() }
        assertEquals("nothing reaches the port, first tick to last", 0, h.port.sent.size)
        // The would-be commands are in the record, doubly marked: their own source name, and
        // accepted == null — the value that means no SDK call was made at all.
        val cmd = h.cmds.last { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SHADOW_SOURCE }
        assertNull(cmd.accepted)
        assertNotNull(cmd.setpoint)
        // Visible as a shadow, and not a manoeuvre a withdrawal could act on.
        val situation = h.engine.situation()
        assertTrue(situation.descent!!.shadow)
        assertFalse(situation.hasManoeuvre)
    }

    @Test
    fun `stick input never cancels a shadow - the would-cancel edge is recorded instead`() {
        val h = shadowing()
        // A real grab, sustained far past the live dead-band and debounce.
        h.port.onRc!!(RcSticks(200, 0, 0, 0))
        repeat(10) { h.tickSeeing() }
        assertNotNull("the shadow keeps running under full deflection", h.engine.situation().descent)
        assertEquals(0, h.port.sent.size)
        val edges = h.events.filter {
            it.first == "tag_descent_phase" && it.second!!.startsWith("shadow would-cancel sticks")
        }
        assertEquals("one edge per crossing, not one per tick", 1, edges.size)
        assertTrue("the magnitude that crossed the dead-band is recorded", edges[0].second!!.contains("0.30"))
        // The sticks centre and deflect again: that is a second crossing, and a second edge.
        h.port.onRc!!(RcSticks(0, 0, 0, 0))
        h.tickSeeing()
        h.port.onRc!!(RcSticks(150, 0, 0, 0))
        repeat(4) { h.tickSeeing() }
        assertEquals(
            2,
            h.events.count {
                it.first == "tag_descent_phase" && it.second!!.startsWith("shadow would-cancel")
            },
        )
    }

    @Test
    fun `a shadow segment ends the way a live run does, releases nothing, and re-arms on its own`() {
        val h = shadowing()
        // The tag goes away past the abort bound: the segment ends in the live exit's words…
        repeat(101) { h.tickAlive() }
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second!!.startsWith("shadow tag gone") })
        // …with nothing released, because nothing was held.
        assertEquals(GuidedPhase.IDLE, h.engine.phase)
        assertEquals(0, h.port.disableCalls)
        assertEquals(0, h.port.sent.size)
        // Blocked edges are recorded on change while the gates refuse…
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second!!.startsWith("shadow blocked") })
        // …and the tag coming back re-arms a fresh segment, automatically: one manual landing
        // is a continuous timeline, not one truncated run.
        h.tickSeeing()
        assertEquals(
            2,
            h.events.count { it.first == "tag_descent_armed" && it.second!!.startsWith("shadow") },
        )
    }

    @Test
    fun `blocked edges record once per blocker, not at tick rate`() {
        val h = Harness()
        h.seeTag(0.0, 0.0)
        // Above the APPROACH ceiling — since the approach, 7-12 m arms (into the approach
        // segment) rather than blocking, so the height blocker this test pins moved up to
        // the decode-reach ceiling.
        h.place(relAlt = TagDescentGuidance.APPROACH_CEILING_M + 1.0)
        h.engine.setShadowDescent(true)
        // Three ticks: enough to prove once-per-blocker, short enough that the fix stays
        // fresh — at the fifth tick the fix would age past the arm bound and the blocker
        // would legitimately *change*, which is a new edge, not a repeat.
        repeat(3) { h.tickAlive() }
        assertEquals(
            1,
            h.events.count { it.first == "tag_descent_phase" && it.second!!.startsWith("shadow blocked") },
        )
        // And the *current* blocker rides the comparison feed, named — the screen's answer to
        // shadow-broken vs shadow-blocked, which is the ambiguity that flew on 152922.
        val cmp = h.engine.shadowComparison(h.now)!!
        assertFalse(cmp.segmentArmed)
        assertTrue("blocker '${cmp.blocker}'", cmp.blocker!!.contains("12m"))
        h.place(relAlt = ALT)
        h.tickSeeing()
        assertTrue(h.events.any { it.first == "tag_descent_armed" && it.second!!.startsWith("shadow") })
    }

    @Test
    fun `live and shadow compute identical transition sequences for identical inputs`() {
        fun script(h: Harness) {
            repeat(5) { h.tickAlive() } // the fix ages past T_HOLD → holding
            h.tickSeeing() // the tag returns → tracking
            h.place(northM = 0.05, relAlt = TagDescentGuidance.TARGET_HEIGHT_M + 0.1)
            repeat(TagDescentGuidance.TERMINAL_TICKS) { h.tickSeeing() } // → terminal
        }

        fun phases(h: Harness): List<String> = h.events
            .filter { it.first == "tag_descent_phase" }
            .map { it.second!!.removePrefix("shadow ").substringBefore(" fixAge") }
            .filter { it in setOf("tracking", "holding", "climbing", "terminal") }

        val live = Harness().descending()
        script(live)

        val shadow = shadowing()
        shadow.tickSeeing() // mirror descending()'s trailing seeing tick
        script(shadow)

        assertEquals(listOf("holding", "tracking", "terminal"), phases(live))
        assertEquals(
            "the law must compute the same transitions in both modes — that identity is what " +
                "makes shadow evidence transferable to live",
            phases(live),
            phases(shadow),
        )
    }

    @Test
    fun `the comparison feed pairs the sticks with the would-be command, and blanks when stale`() {
        val h = shadowing()
        // Half-forward on the right stick, aircraft heading north: the operator's arrow is
        // half the horizontal envelope, due north — StickMapping's scale, no second mapping.
        h.port.onRc!!(RcSticks(0, 0, 0, 330))
        h.tickSeeing()
        val cmp = h.engine.shadowComparison(h.now)
        assertNotNull(cmp)
        assertTrue(cmp!!.segmentArmed)
        assertEquals(GuidedEnvelope.HORIZONTAL_MAX_MS / 2, cmp.operator!!.north, 0.01)
        assertEquals(0.0, cmp.operator!!.east, 0.01)
        // The shadow's would-be command: descending at the cap, centred over the tag.
        assertEquals(TagDescentGuidance.V_DESCENT_MAX_MS, cmp.shadow!!.down, 1e-9)
        // A stale command blanks rather than freezes — a frozen arrow reads as a live opinion.
        val later = h.now + GuidedStickEngine.SHADOW_CMD_STALE_MS + 1
        assertNull(h.engine.shadowComparison(later)!!.shadow)
        // And the whole feed disappears with the mode.
        h.engine.setShadowDescent(false)
        assertNull(h.engine.shadowComparison(h.now))
    }

    @Test
    fun `an RC grab that aborts a live passthrough leaves shadow mode running - flight 152922's shape`() {
        // The exact sequence that flew on 2026-07-28 (20260728-152922, t=54.9–61.7): QGC's
        // joystick stream holds a live passthrough engagement, the operator enables shadow,
        // then pulls a stick. The passthrough must die (rule 1); the shadow must not — the
        // operator is flying, and a deflection is the flight.
        val h = Harness()
        h.frame() // neutral, at rest
        h.now += 40
        h.frame(x = 500) // deliberate: passthrough engages
        h.confirm()
        assertEquals(GuidedPhase.ENGAGED, h.engine.phase)
        h.seeTag(0.0, 0.0)
        assertTrue(h.engine.setShadowDescent(true))
        h.tickAlive() // a shadow segment arms beside the live passthrough
        assertTrue(h.engine.shadowComparison(h.now)!!.segmentArmed)

        // The stick pull: lv reached 59 % of travel on the flight.
        h.port.onRc!!(RcSticks(0, -390, 0, 0))
        h.tickAlive(GuidedStickEngine.RC_ABORT_SUSTAIN_MS)
        assertEquals("the passthrough dies - rule 1", GuidedPhase.IDLE, h.engine.phase)
        assertNotNull("the shadow does not", h.engine.shadowComparison(h.now))
        assertTrue(h.events.none { it.first == "tag_descent_phase" && it.second!!.startsWith("shadow mode off") })
        // …and it is still doing its job: the would-cancel edge is on the record, and the
        // would-be commands keep flowing while the operator flies.
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second!!.startsWith("shadow would-cancel") })
        val before = h.cmds.count { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SHADOW_SOURCE }
        repeat(3) { h.tickSeeing() }
        assertTrue(
            h.cmds.count { it.source?.messageName == GuidedStickEngine.TAG_DESCENT_SHADOW_SOURCE } > before,
        )
    }

    @Test
    fun `an abort ends shadow mode - STOP means stop recording too`() {
        val h = shadowing()
        h.engine.abort(GuidedStickEngine.DisengageReason.STOPPED, "stopped from the phone")
        assertNull(h.engine.shadowComparison(h.now))
        assertNull(h.engine.situation().descent)
        assertTrue(h.events.any { it.first == "tag_descent_ended" && it.second == "shadow stopped" })
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second == "shadow mode off (stopped)" })
    }

    @Test
    fun `a live arm supersedes shadow mode, recorded`() {
        val h = shadowing()
        assertEquals(Verdict.ACCEPTED, h.arm())
        assertNull("one recorder of this tag at a time", h.engine.shadowComparison(h.now))
        assertTrue(h.events.any { it.first == "tag_descent_phase" && it.second == "shadow mode off (live arm)" })
        assertFalse(h.engine.situation().descent!!.shadow)
    }
}
