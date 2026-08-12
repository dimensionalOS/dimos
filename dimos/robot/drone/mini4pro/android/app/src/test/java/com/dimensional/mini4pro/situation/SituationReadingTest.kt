package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.guided.GuidedGoto
import com.dimensional.mini4pro.guided.GuidedOrbit
import com.dimensional.mini4pro.guided.GuidedPhase
import com.dimensional.mini4pro.guided.GuidedRoi
import com.dimensional.mini4pro.guided.GuidedSituation
import com.dimensional.mini4pro.mission.GeoPoint
import com.dimensional.mini4pro.mission.LegKind
import com.dimensional.mini4pro.mission.MissionPlan
import com.dimensional.mini4pro.mission.ResolvedLeg
import com.dimensional.mini4pro.mission.StoredItem
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **The honesty rules of the situation view** — what may be drawn, given what is known.
 *
 * This is the file that stands between "the aircraft is there" and a triangle sitting in the
 * middle of a map. A picture makes its claim instantly and carries no qualifier, so this
 * project's display discipline is applied here by *removal*: a symbol whose input is stale is
 * not greyed and not frozen, it is gone, and a note says why.
 *
 * Written to fail loudly for:
 *
 *  - **an aircraft drawn from a stale fix** — the single worst failure available to this
 *    feature, because a frozen symbol in a plausible place is indistinguishable from a live one
 *  - **a heading drawn from a stale attitude**, which on this airframe is the *normal* state:
 *    `Signal.ATTITUDE` is change-driven and was measured 15.3 s between deliveries on a
 *    motionless aircraft
 *  - **DJI's no-home filler plotted as a home** — the 4.583662361046586E7-in-both-fields value
 *    that once produced 220 fabricated `HOME_POSITION` messages
 *  - **a manoeuvre outliving the engine that was flying it**, i.e. any cache of "last commanded"
 *    geometry in this layer
 *  - **a suspended ROI drawn as a tracked one**, claiming a camera is on a target it drifted off
 *  - **a loaded plan drawn as a flying one**, inventing an executor that does not exist
 *  - a missing symbol leaving an unexplained blank frame rather than a note
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time against the shipped source, run,
 * confirmed red, reverted. Counts are failing tests across `SituationReadingTest`,
 * `SituationSceneTest` and `SituationHonestyTest` — measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | position freshness gate dropped (a stale fix is drawn) | 4 |
 *  | attitude freshness gate dropped (a stale heading is drawn) | 1 |
 *  | `Geo.coordinateOrNull` on the fix replaced by a plain null check | 1 |
 *  | home taken from the raw fields instead of `TelemetryEncoder.homeCoordinate` | 2 |
 *  | ROI `tracking` forced true | 1 |
 *  | `PlanMark.flying` forced true in `planMarkOf` | 2 |
 *  | legs with a null target given the previous waypoint's position | 1 |
 *  | notes suppressed when a symbol is dropped | 6 |
 *  | orbit accepted with a zero or non-finite radius | 1 |
 *
 * And the scene's share of the same discipline, measured the same way:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the goto leg drawn from home when the aircraft symbol was withheld | 1 |
 *  | the ROI line drawn for a suspended ROI | 1 |
 *  | the aircraft painted first instead of last | 1 |
 *  | the orbit chevron mirrored (a clockwise circle drawn anticlockwise) | 2 |
 *  | the aircraft dart drawn with `sin` and `cos` swapped | 3 |
 *  | a loaded plan drawn solid rather than broken | 1 |
 *
 * Several of these are single-test kills, and that is recorded rather than padded: each names a
 * distinct lie the picture could tell, and one test that fails is a mutation that does not ship.
 */
class SituationReadingTest {

    private val lat = 37.9938232
    private val lon = 23.7253477

    /** Everything fresh, at the site. The baseline every test below removes one thing from. */
    private fun live(
        latitude: Double? = lat,
        longitude: Double? = lon,
        yaw: Double? = 90.0,
        positionAgeMs: Long? = 100L,
        attitudeAgeMs: Long? = 200L,
        homeLat: Double? = lat,
        homeLon: Double? = lon,
        homeSet: Boolean? = true,
    ) = AircraftState(
        fcConnected = true,
        latitude = latitude,
        longitude = longitude,
        yawDeg = yaw,
        homeLatitude = homeLat,
        homeLongitude = homeLon,
        homeLocationSet = homeSet,
        ages = SampleAges.of(
            buildMap {
                positionAgeMs?.let { put(Signal.POSITION, it) }
                attitudeAgeMs?.let { put(Signal.ATTITUDE, it) }
            },
        ),
    )

    // ── the aircraft symbol ──────────────────────────────────────────────────

    @Test
    fun `a fresh fix and a fresh attitude draw an oriented aircraft`() {
        val s = SituationReading.read(SituationSource.LIVE, live())
        assertNotNull("aircraft", s.aircraft)
        val a = s.aircraft!!
        assertEquals(lat, a.fix.latDeg, 1e-9)
        assertEquals(lon, a.fix.lonDeg, 1e-9)
        assertEquals(90.0, a.headingDeg!!, 1e-9)
        assertTrue("nothing was withheld, so there is nothing to explain", s.notes.isEmpty())
    }

    @Test
    fun `a stale position drops the aircraft entirely and says so`() {
        // POSITION's limit is 1000 ms against a measured ~12 Hz feed: past it the feed is dead,
        // not quiet.
        val s = SituationReading.read(SituationSource.LIVE, live(positionAgeMs = 4_000L))
        assertNull("a stale fix must not be drawn anywhere", s.aircraft)
        assertTrue(SituationReading.NOTE_NO_FIX in s.notes)
    }

    @Test
    fun `a position never delivered is not a position`() {
        val s = SituationReading.read(SituationSource.LIVE, live(positionAgeMs = null))
        assertNull(s.aircraft)
        assertTrue(SituationReading.NOTE_NO_FIX in s.notes)
    }

    @Test
    fun `a stale attitude degrades to a place with no heading, it does not freeze one`() {
        val s = SituationReading.read(SituationSource.LIVE, live(attitudeAgeMs = 9_000L))
        val a = s.aircraft!!
        assertEquals("we still know where it is", lat, a.fix.latDeg, 1e-9)
        assertNull("we do not know which way it faces", a.headingDeg)
        assertTrue(SituationReading.NOTE_NO_HEADING in s.notes)
    }

    @Test
    fun `with no fix at all the heading note is not piled on top`() {
        val s = SituationReading.read(
            SituationSource.LIVE,
            live(positionAgeMs = 5_000L, attitudeAgeMs = 9_000L),
        )
        assertEquals(listOf(SituationReading.NOTE_NO_FIX), s.notes)
    }

    @Test
    fun `a coordinate Geo refuses is not drawn`() {
        // DJI's filler: one number written into both fields.
        val filler = 4.583662361046586E7
        val s = SituationReading.read(
            SituationSource.LIVE,
            live(latitude = filler, longitude = filler),
        )
        assertNull(s.aircraft)
        assertTrue(SituationReading.NOTE_NO_FIX in s.notes)
    }

    @Test
    fun `a non-finite yaw is not a heading`() {
        val s = SituationReading.read(SituationSource.LIVE, live(yaw = Double.NaN))
        assertNotNull(s.aircraft)
        assertNull(s.aircraft!!.headingDeg)
        assertTrue(SituationReading.NOTE_NO_HEADING in s.notes)
    }

    // ── home ─────────────────────────────────────────────────────────────────

    @Test
    fun `home is drawn when DJI says there is one`() {
        val s = SituationReading.read(SituationSource.LIVE, live())
        assertEquals(lat, s.home!!.latDeg, 1e-9)
    }

    @Test
    fun `DJI saying there is no home outranks a populated home coordinate`() {
        // The measured sentinel case: a plausible-looking pair with isHomeLocationSet=false.
        val s = SituationReading.read(
            SituationSource.LIVE,
            live(homeLat = lat, homeLon = lon, homeSet = false),
        )
        assertNull("the picture and the MAVLink wire must agree there is no home", s.home)
    }

    @Test
    fun `the no-home filler is not plotted as a home`() {
        val filler = 4.583662361046586E7
        val s = SituationReading.read(
            SituationSource.LIVE,
            live(homeLat = filler, homeLon = filler, homeSet = null),
        )
        assertNull(s.home)
    }

    // ── manoeuvres ───────────────────────────────────────────────────────────

    @Test
    fun `with no engine there is no manoeuvre geometry at all`() {
        val s = SituationReading.read(SituationSource.LIVE, live(), guided = null)
        assertNull(s.orbit)
        assertNull(s.goto)
        assertNull(s.roi)
    }

    @Test
    fun `an idle engine draws no manoeuvre, so nothing can outlive one`() {
        val s = SituationReading.read(SituationSource.LIVE, live(), guided = GuidedSituation.IDLE)
        assertNull(s.orbit)
        assertNull(s.goto)
        assertNull(s.roi)
    }

    @Test
    fun `an accepted orbit is drawn with its centre, radius and direction`() {
        val guided = GuidedSituation(
            phase = GuidedPhase.ENGAGED,
            orbit = GuidedOrbit(lat, lon, radiusM = 45.0, direction = -1, relAltM = 30.0, circling = true),
        )
        val s = SituationReading.read(SituationSource.LIVE, live(), guided)
        assertEquals(45.0, s.orbit!!.radiusM, 1e-9)
        assertEquals(-1, s.orbit!!.direction)
    }

    @Test
    fun `an orbit with no radius is not a circle`() {
        for (bad in listOf(0.0, -5.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            val guided = GuidedSituation(
                phase = GuidedPhase.ENGAGED,
                orbit = GuidedOrbit(lat, lon, bad, 1, 30.0, circling = true),
            )
            assertNull("radius $bad must not be drawn", SituationReading.read(SituationSource.LIVE, live(), guided).orbit)
        }
    }

    @Test
    fun `a goto carries whether it is still going somewhere`() {
        val guided = GuidedSituation(
            phase = GuidedPhase.ENGAGED,
            goto = GuidedGoto(lat + 0.001, lon, relAltM = 20.0, arrived = true),
        )
        val s = SituationReading.read(SituationSource.LIVE, live(), guided)
        assertTrue(s.goto!!.arrived)
    }

    @Test
    fun `a tracked ROI is drawn as tracked and says nothing`() {
        val guided = GuidedSituation(
            phase = GuidedPhase.ENGAGED,
            roi = GuidedRoi(lat + 0.002, lon, tracking = true),
        )
        val s = SituationReading.read(SituationSource.LIVE, live(), guided)
        assertTrue(s.roi!!.tracking)
        assertFalse(SituationReading.NOTE_ROI_SUSPENDED in s.notes)
    }

    @Test
    fun `a remembered but suspended ROI is not a camera on a target`() {
        val guided = GuidedSituation(
            phase = GuidedPhase.IDLE,
            roi = GuidedRoi(lat + 0.002, lon, tracking = false),
        )
        val s = SituationReading.read(SituationSource.LIVE, live(), guided)
        assertNotNull("the place the operator asked about is still known", s.roi)
        assertFalse(s.roi!!.tracking)
        assertTrue(SituationReading.NOTE_ROI_SUSPENDED in s.notes)
    }

    // ── the plan ─────────────────────────────────────────────────────────────

    private fun plan(vararg legs: ResolvedLeg): MissionPlan = object : MissionPlan {
        override val planId = 1
        override val items: List<StoredItem> = emptyList()
        override val legs: List<ResolvedLeg> = legs.toList()
        override val homeAtUpload: GeoPoint? = null
        override val amslDatumAtUpload: Double? = null
        override val uploadedAtMs = 0L
    }

    private fun leg(seq: Int, target: GeoPoint?, kind: LegKind = LegKind.WAYPOINT) = ResolvedLeg(
        seq = seq, kind = kind, target = target, relativeAltM = 20.0, holdSeconds = 0.0,
        acceptRadiusM = null, speedLimitMps = null, roi = null, orbit = null,
    )

    @Test
    fun `a committed plan becomes drawable points in sequence order`() {
        val mark = SituationReading.planMarkOf(
            plan(
                leg(1, GeoPoint(lat, lon)),
                leg(2, GeoPoint(lat + 0.001, lon + 0.001)),
            ),
        )!!
        assertEquals(listOf(1, 2), mark.points.map { it.seq })
        assertEquals("WAYPOINT", mark.points.first().kind)
    }

    @Test
    fun `a plan in the store is never a plan being flown`() {
        val mark = SituationReading.planMarkOf(plan(leg(1, GeoPoint(lat, lon))))!!
        assertFalse("no executor exists; M4-2 puts Start in QGC", mark.flying)
        assertNull("nothing knows which leg is current, because nothing is on one", mark.currentSeq)
    }

    @Test
    fun `a loaded plan says out loud that nothing is flying it`() {
        val s = SituationReading.read(
            SituationSource.LIVE,
            live(),
            plan = SituationReading.planMarkOf(
                plan(leg(1, GeoPoint(lat, lon)), leg(2, GeoPoint(lat + 0.001, lon))),
            ),
        )
        assertTrue(SituationReading.NOTE_PLAN_NOT_FLYING in s.notes)
    }

    @Test
    fun `a leg with no place of its own contributes no point`() {
        val mark = SituationReading.planMarkOf(
            plan(
                leg(1, GeoPoint(lat, lon)),
                leg(2, null, LegKind.RTL),
                leg(3, GeoPoint(lat + 0.002, lon)),
            ),
        )!!
        assertEquals("RTL must not inherit the previous waypoint", listOf(1, 3), mark.points.map { it.seq })
    }

    @Test
    fun `a plan whose every leg is unplottable is no plan at all`() {
        assertNull(SituationReading.planMarkOf(plan(leg(1, null, LegKind.RTL))))
        assertNull(SituationReading.planMarkOf(plan(leg(1, GeoPoint(500.0, 900.0)))))
        assertNull(SituationReading.planMarkOf(null))
    }

    @Test
    fun `an empty plan is dropped from the situation rather than drawn as a blank`() {
        val s = SituationReading.read(SituationSource.LIVE, live(), plan = PlanMark(emptyList()))
        assertNull(s.plan)
    }

    // ── the whole picture ────────────────────────────────────────────────────

    @Test
    fun `a bare state knows nothing and admits it`() {
        val s = SituationReading.read(SituationSource.LIVE, AircraftState())
        assertFalse(s.hasAnything)
        assertTrue(SituationReading.NOTE_NO_FIX in s.notes)
    }

    @Test
    fun `the source is carried through untouched`() {
        assertEquals(
            SituationSource.REPLAY,
            SituationReading.read(SituationSource.REPLAY, live()).source,
        )
    }
}
