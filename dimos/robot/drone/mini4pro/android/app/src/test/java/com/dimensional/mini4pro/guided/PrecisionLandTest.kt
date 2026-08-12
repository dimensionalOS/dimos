package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.StatusTexts
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos

/**
 * The tag-landing sequence's **arithmetic and its two gates** — [PrecisionLand], pure, with no engine,
 * no clock and no fakes at all, exactly as `MissionLaunchTest` pins §7.2's launch admission.
 *
 * What the engine does with these answers is `GuidedPrecisionLandTest`'s business, and the mutation
 * table for the whole feature lives there.
 *
 * Written to fail loudly for:
 *
 *  - **either gate dropped** — the 20 m radius from the recorded takeoff point, or the
 *    below-plan-takeoff-height floor. Both are Ivan's *"just error out"* conditions and both are the
 *    difference between a landing over the pad and a landing somewhere nobody planned.
 *  - **the site cross-check dropped**, which is the "plan drawn at another site" hazard: a plan whose
 *    Land item sits 200 m from where the aircraft actually took off must not fly its sequence.
 *  - **the reference point becoming the drawn coordinate** rather than the recorded takeoff position,
 *    which is the correction Ivan made to this feature's first design and the thing every test here
 *    is arranged around.
 *  - **an unknown treated as zero** — no recorded takeoff point, no plan takeoff height, no usable
 *    altitude — each of which would make a gate vacuous exactly when nobody has measured anything.
 *  - **the never-climb clamp dropped**, which would fly a low Land item's sequence *up* to 8 m.
 *
 * ## Measured mutation kill counts
 *
 * The campaign's table lives in `GuidedPrecisionLandTest` (2026-07-30, whole suite per mutant, 2625
 * tests, no survivors). The rows this file's own tests kill, from that measurement: the 20 m radius
 * gate (2), the plan-height gate (2), the site cross-check (3), the reference point becoming the drawn
 * coordinate (4), the never-climb clamp (2). Each of those counts is split between this file and the
 * engine's, deliberately: the law is pinned here where it can be read without a fake, and the engine's
 * *use* of it is pinned there where the aircraft is.
 */
class PrecisionLandTest {

    private companion object {
        /** The project's home latitude family — cos 38° = 0.788, so a missing term is 21 %, not nothing. */
        const val LAT = 38.0
        const val LON = 23.7

        /** big1.plan's own numbers: takeoff 10 m, land item 15 m. */
        const val PLAN_TAKEOFF_M = 10.0
        const val ITEM_ALT_M = 15.0

        fun latNorthOf(metres: Double): Double = LAT + metres / RepositionGuidance.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (RepositionGuidance.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        /**
         * big1.plan's shape: the aircraft ~8 m from the pad at 60 m, the Land item drawn 0.3 m from
         * where we took off, the plan's takeoff item at 10 m.
         */
        fun gate(
            aircraft: Pair<Double, Double>? = latNorthOf(8.0) to LON,
            takeoff: Pair<Double, Double>? = LAT to LON,
            item: Pair<Double, Double> = latNorthOf(0.3) to LON,
            itemRelAltM: Double? = ITEM_ALT_M,
            currentRelAltM: Double? = 60.0,
            planTakeoffRelAltM: Double? = PLAN_TAKEOFF_M,
        ): PrecisionLand.Decision = PrecisionLand.gate(
            aircraft = aircraft,
            takeoff = takeoff,
            itemLatDeg = item.first,
            itemLonDeg = item.second,
            itemRelAltM = itemRelAltM,
            currentRelAltM = currentRelAltM,
            planTakeoffRelAltM = planTakeoffRelAltM,
        )

        fun refusal(decision: PrecisionLand.Decision): String =
            (decision as? PrecisionLand.Decision.Refused)?.reason ?: "CLEAR"
    }

    // ------------------------------------------------------------------ the happy path

    @Test
    fun `big1's shape clears, and the target is the RECORDED takeoff point at the ITEM's altitude`() {
        val clear = gate() as PrecisionLand.Decision.Clear
        // Not the drawn coordinate — 0.3 m away in this plan and up to 20 m away in general. Ivan:
        // "plan's lat/lng will slightly differ from executed one, can be 5 m away for example".
        assertEquals(LAT, clear.takeoffLatDeg, 1e-12)
        assertEquals(LON, clear.takeoffLonDeg, 1e-12)
        // The item's own altitude is flown to as an ordinary waypoint altitude, which in big1.plan
        // makes the transit a 60 m -> 15 m descent while translating ~8 m.
        assertEquals(ITEM_ALT_M, clear.transitRelAltM, 1e-12)
    }

    @Test
    fun `a five-metre disagreement between the plan and the executed takeoff passes harmlessly`() {
        // Ivan's own example. The drawn item is 5 m from the pad; the sequence still flies to the pad.
        val clear = gate(item = latNorthOf(5.0) to LON) as PrecisionLand.Decision.Clear
        assertEquals(LAT, clear.takeoffLatDeg, 1e-12)
    }

    // ------------------------------------------------------------------ gate 1: the radius

    @Test
    fun `THE 20 M GATE - an aircraft further than LAND_TAG_RADIUS_M from the pad is refused`() {
        // Just inside, and it clears: the bound is on the aircraft's own distance from where it took
        // off, measured at the moment the item begins.
        assertTrue(
            gate(aircraft = latNorthOf(PrecisionLand.LAND_TAG_RADIUS_M - 0.5) to LON)
                is PrecisionLand.Decision.Clear
        )
        assertEquals(
            GuidedStatusTexts.REASON_LAND_TOO_FAR,
            refusal(gate(aircraft = latNorthOf(PrecisionLand.LAND_TAG_RADIUS_M + 0.5) to LON)),
        )
        // And the east axis too, which is where a missing cos(latitude) term would hide: 25 m east is
        // 25 m, not 19.7 m.
        assertEquals(
            GuidedStatusTexts.REASON_LAND_TOO_FAR,
            refusal(gate(aircraft = LAT to lonEastOf(25.0))),
        )
    }

    @Test
    fun `the radius is measured from the RECORDED takeoff point, not from the drawn item`() {
        // The aircraft sits exactly on the drawn Land item, which is 40 m from where it took off.
        // Reading the drawn coordinate as the reference would clear this; the pad is what counts.
        val drawn = latNorthOf(40.0) to LON
        assertEquals(
            GuidedStatusTexts.REASON_LAND_SITE,
            refusal(gate(aircraft = drawn, item = drawn)),
        )
    }

    // ------------------------------------------------------------------ gate 2: the plan's height

    @Test
    fun `THE PLAN-HEIGHT GATE - below the plan's own NAV_TAKEOFF altitude is refused`() {
        // big1.plan cleared 10 m. At 9.9 m the sequence must not start; at exactly 10 m it may.
        assertEquals(
            GuidedStatusTexts.REASON_LAND_TOO_LOW,
            refusal(gate(currentRelAltM = PLAN_TAKEOFF_M - 0.1)),
        )
        assertTrue(gate(currentRelAltM = PLAN_TAKEOFF_M) is PrecisionLand.Decision.Clear)
    }

    @Test
    fun `a plan with no takeoff item is refused BY NAME rather than compared against zero`() {
        // Unknown is never zero (CLAUDE.md): defaulting the cleared height to 0 m would make the gate
        // vacuous exactly when nothing measured it.
        assertEquals(
            GuidedStatusTexts.REASON_LAND_NO_PLAN_TAKEOFF,
            refusal(gate(planTakeoffRelAltM = null)),
        )
        assertEquals(
            GuidedStatusTexts.REASON_LAND_NO_PLAN_TAKEOFF,
            refusal(gate(planTakeoffRelAltM = Double.NaN)),
        )
    }

    // ------------------------------------------------------------------ the cross-check

    @Test
    fun `THE SITE CROSS-CHECK - a plan drawn at another site is refused, naming itself`() {
        // The aircraft is over its own pad and everything else is fine; only the *plan* is wrong.
        assertEquals(
            GuidedStatusTexts.REASON_LAND_SITE,
            refusal(gate(aircraft = LAT to LON, item = latNorthOf(200.0) to LON)),
        )
        // Just inside the bound and it clears — a plan drawn at the same site, badly.
        assertTrue(
            gate(item = latNorthOf(PrecisionLand.LAND_TAG_RADIUS_M - 0.5) to LON)
                is PrecisionLand.Decision.Clear
        )
    }

    // ------------------------------------------------------------------ the unknowns

    @Test
    fun `no recorded takeoff point is refused - there is no pad to fly to`() {
        assertEquals(GuidedStatusTexts.REASON_LAND_NO_TAKEOFF_POINT, refusal(gate(takeoff = null)))
    }

    @Test
    fun `no usable altitude is refused - the plan-height gate cannot be evaluated blind`() {
        assertEquals(GuidedStatusTexts.REASON_NO_DATUM, refusal(gate(currentRelAltM = null)))
    }

    @Test
    fun `an item with no height of its own is refused rather than flown at the current one`() {
        assertEquals(GuidedStatusTexts.REASON_LAND_NO_HEIGHT, refusal(gate(itemRelAltM = null)))
    }

    @Test
    fun `the unknowns are judged before the bounds, so a refusal names the cause`() {
        // Everything wrong at once: the first sentence is the one about the missing reference point,
        // because a radius measured from nothing is not a measurement.
        assertEquals(
            GuidedStatusTexts.REASON_LAND_NO_TAKEOFF_POINT,
            refusal(
                gate(
                    aircraft = latNorthOf(500.0) to LON, takeoff = null,
                    currentRelAltM = 1.0, planTakeoffRelAltM = null,
                )
            ),
        )
    }

    // ------------------------------------------------------------------ the never-climb clamp

    @Test
    fun `THE NEVER-CLIMB CLAMP - the arm height is min(8, current), never a climb`() {
        // Above it: the sequence comes down to 8 m.
        assertEquals(
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M,
            PrecisionLand.armHeightTargetM(15.0),
            1e-12,
        )
        // Below it: the sequence arms where it is. A Land item authored at 4 m must not fly the
        // aircraft back *up* to 8 m for no reason anybody asked for.
        assertEquals(4.0, PrecisionLand.armHeightTargetM(4.0), 1e-12)
        assertEquals(
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M,
            PrecisionLand.armHeightTargetM(PrecisionLand.LAND_TAG_ARM_HEIGHT_M),
            1e-12,
        )
        // Fail-closed on an impossible argument; the caller has already refused a null altitude.
        assertEquals(
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M,
            PrecisionLand.armHeightTargetM(Double.NaN),
            1e-12,
        )
    }

    /**
     * The **second** never-climb clamp, at the bottom of the acquisition band. A Land item authored
     * at 4 m starts looking at 4 m (the first clamp), and a fixed 5 m floor would then fly the
     * aircraft a metre back **up** to go hunting — a climb, in a sequence whose remaining job is
     * entirely downward.
     */
    @Test
    fun `THE ACQUISITION FLOOR IS ALSO CLAMPED - min(5, the arm height), never a climb`() {
        // The ordinary case: start looking at 8 m, keep looking down to 5 m.
        assertEquals(
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M,
            PrecisionLand.acquireFloorTargetM(PrecisionLand.LAND_TAG_ARM_HEIGHT_M),
            1e-12,
        )
        // Already at or below the floor: the phase has nowhere to descend to and holds where it is,
        // trying, rather than climbing to look.
        assertEquals(4.0, PrecisionLand.acquireFloorTargetM(4.0), 1e-12)
        assertEquals(
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M,
            PrecisionLand.acquireFloorTargetM(PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M),
            1e-12,
        )
        assertEquals(
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M,
            PrecisionLand.acquireFloorTargetM(Double.NaN),
            1e-12,
        )
    }

    /**
     * **The acquisition band is a band**: it starts inside the approach segment and ends inside the
     * reliable-decode region, and it never reaches anything the descent or the firmware owns.
     *
     * The numbers it is measured against: the 100 % per-frame band tops out at 7.0 m and the rate
     * collapses above it (92.6 % at 7–8 m, 2.7 % at 8–9 m —
     * `docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1), which is why one attempt at the
     * top of the band is a coin flip and why the floor is two whole metres under the cliff. Below,
     * the descent's own commit ceiling is 2.5 m and the FC's measured virtual-stick floor is
     * ~1.0–1.4 m; the acquisition floor must not go near either.
     */
    @Test
    fun `the acquisition band spans the arm height down into the reliable decode region`() {
        assertTrue(
            "the band must descend, not climb",
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M < PrecisionLand.LAND_TAG_ARM_HEIGHT_M,
        )
        assertTrue(
            "the floor must be inside the reliable-decode region, under the 7 m cliff",
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M < TagDescentGuidance.ARM_CEILING_M,
        )
        assertTrue(
            "the floor must stay well above the descent's own commit ceiling",
            PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M > TagDescentGuidance.COMMIT_CEILING_M,
        )
        // And the whole band is inside the detector's ceiling, so the camera is looking throughout.
        assertTrue(
            "the band must sit under the detector's own ceiling",
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M <
                com.dimensional.mini4pro.vision.TagArming.ACQUIRE_CEILING_M,
        )
    }

    /**
     * **The wait at the floor, sized against the flight that measured it.** landing17
     * (`datasets/landing17/20260730-172355.001.jsonl`) refused at t=262.192 and the tag first
     * decoded at t=262.459 — 267 ms later — after which it stayed in view for 26 s. Three
     * quantities the window has to cover, all measured on that flight:
     *
     *  - the 267 ms it took the marker to become decodable at all;
     *  - the **2.20 s** largest gap between sightings during the hover that followed (13 gaps
     *    exceeded [TagDescentGuidance.ARM_FRESH_MS] across those 26 s, median gap 116 ms), so a
     *    window sized on the median would refuse in the middle of a working acquisition;
     *  - the aircraft's own settle: the arrival test fires up to
     *    [RepositionGuidance.VERTICAL_ACCEPT_M] high and, measured, with 0.4 m/s still on it.
     *
     * Pinned as inequalities against those numbers rather than as `assertEquals(10_000)`, because
     * what matters is the margin, not the round number.
     */
    @Test
    fun `THE HOLD WINDOW - long enough for landing17's worst dry spell, short enough to end`() {
        // The measured acquisition delay, and the measured worst gap between fixes at that height.
        val landing17FirstDecodeMs = 267L
        val landing17WorstFixGapMs = 2_197L
        assertTrue(
            "the window must cover the delay that refuted the instant refusal",
            PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS > landing17FirstDecodeMs * 4,
        )
        assertTrue(
            "the window must survive a dry spell in the middle of a good acquisition",
            PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS > landing17WorstFixGapMs * 2,
        )
        assertTrue(
            "the window must be many freshness bounds wide, or it is one coin flip again",
            PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS > TagDescentGuidance.ARM_FRESH_MS * 10,
        )
        // And bounded: a wait that outlasts an operator's patience is a hang, not a wait. The
        // aim bound is this file's other clock and the hold is deliberately the longer of the two.
        assertTrue(
            "the wait must end while the operator still has a decision to make",
            PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS <= 15_000L,
        )
        assertTrue(PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS > PrecisionLand.NADIR_AIM_LIMIT_MS)
    }

    // ------------------------------------------------------------------ the numbers themselves

    @Test
    fun `the arm height sits above the tag band and well under the approach ceiling`() {
        // The whole reason 8 m is the number: an arm there enters APPROACH (Ivan's spec), so the
        // approach segment does the centring, and it is far enough under the decode-reach ceiling to
        // survive the ~1.2 m of within-session barometric wander that decides what 8 m *reads* as.
        assertTrue(
            "the arm must be above the band or the approach segment is skipped",
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M > TagDescentGuidance.ARM_CEILING_M,
        )
        assertTrue(
            "the arm must be under the decode-reach ceiling or it refuses itself",
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M < TagDescentGuidance.APPROACH_CEILING_M,
        )
        // Even at the arrival test's full vertical tolerance the arm is still inside that window.
        assertTrue(
            PrecisionLand.LAND_TAG_ARM_HEIGHT_M + RepositionGuidance.VERTICAL_ACCEPT_M <
                TagDescentGuidance.APPROACH_CEILING_M
        )
    }

    @Test
    fun `the takeoff radius is the tightest of the three bounds in this layer`() {
        // A bound that gates a leg ending on the ground should not be looser than the one that gates
        // a leg nobody drew (50 m) or line of sight (150 m).
        assertTrue(PrecisionLand.LAND_TAG_RADIUS_M < MissionGuidance.REJOIN_MAX_M)
        assertTrue(PrecisionLand.LAND_TAG_RADIUS_M < MissionGuidance.MAX_HOME_DIST_M)
    }

    @Test
    fun `every refusal reason fits the sentence it has to ride in`() {
        val reasons = listOf(
            GuidedStatusTexts.REASON_LAND_NO_TAKEOFF_POINT,
            GuidedStatusTexts.REASON_LAND_NO_HEIGHT,
            GuidedStatusTexts.REASON_LAND_NO_PLAN_TAKEOFF,
            GuidedStatusTexts.REASON_LAND_SITE,
            GuidedStatusTexts.REASON_LAND_TOO_FAR,
            GuidedStatusTexts.REASON_LAND_TOO_LOW,
            GuidedStatusTexts.REASON_LAND_PHASE_TIMEOUT,
            GuidedStatusTexts.REASON_LAND_ARM,
        )
        for (reason in reasons) {
            val sentence = GuidedStatusTexts.tagLandRefused(reason)
            assertTrue(
                "'$sentence' is ${sentence.toByteArray(Charsets.UTF_8).size} bytes",
                sentence.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
            )
            // The frame must survive too: a reason so long that `preferring` drops "Tag land refused"
            // leaves the operator a bare word with no idea which subsystem said it.
            assertTrue("the frame was dropped for '$reason'", sentence.startsWith("Tag land refused: "))
        }
    }

    @Test
    fun `the three phase sentences fit the field`() {
        for (text in listOf(
            GuidedStatusTexts.LAND_TAG_TRANSIT,
            GuidedStatusTexts.LAND_TAG_AIMING,
            GuidedStatusTexts.LAND_TAG_LOWERING,
            GuidedStatusTexts.LAND_TAG_ARMED,
        )) {
            assertTrue(
                "'$text' is ${text.toByteArray(Charsets.UTF_8).size} bytes",
                text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
            )
        }
    }
}
