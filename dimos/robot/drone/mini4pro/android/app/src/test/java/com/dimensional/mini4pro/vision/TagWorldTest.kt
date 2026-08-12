package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Pixels to a place, and every refusal along the way.**
 *
 * The only part of this package whose output is a number a controller might fly on, and the part an
 * aircraft cannot check — an aircraft can tell you the detector saw a tag; it cannot tell you the
 * tag was really 1.4 m north. So it is pure, and it is tested here.
 *
 * Half of these tests assert that a fix is **not** produced. That is deliberate and it is the more
 * important half: this project's standing rule is that unknown is never zero, and a fix computed
 * with a heading of 0 because none had arrived is a confidently wrong place a landing controller
 * cannot tell from a right one.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | the nadir Y offset reverted to 0 — the image-centre bug this replaced | 8 |
 * | the nadir X offset reverted to 0 | 8 |
 * | the nadir offset's sign flipped | 8 |
 * | `fix` reverted to `cameraFrame`, measuring from the image centre | 5 |
 * | the nadir Y offset scaled by height instead of width | **1, and it survived at first** |
 * | `NADIR_IMAGE_UP_IS_NOSE` inverted | 3 |
 * | the focal length does not scale with frame width | 2 |
 * | `metric` claimed true | 1 |
 * | `bearingAssumed` claimed false | 1 |
 * | a missing heading defaults to north instead of refusing | 1 |
 * | a missing camera pitch defaults to nadir instead of refusing | 1 |
 * | the nadir tolerance check is dropped | 1 |
 * | a non-positive altitude is accepted | 1 |
 * | the yaw rotation is dropped — body offsets used as world | 1 |
 *
 * Most rows are 1 because each refusal is pinned by one test that asserts several of them in a row,
 * and the first failing assertion ends that test. That is the honest count, not a weaker one: a
 * mutation that survives is what matters, and none here does.
 *
 * **One did, and it is the reason this table is worth keeping.** "The nadir Y offset scaled by
 * height instead of width" scored **0** on its first run. `theNadirOffsetScalesWithWidthOnBothAxes`
 * checked only 960×540, where the width ratio and the height ratio are both exactly 0.5 and the two
 * scalings give the identical number — a test that could not fail. It now also checks 1280×960,
 * where they are 0.667 and 0.889, and the mutation dies. The bug it would have shipped is one that
 * appears on no 16:9 stream and every other one.
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 *
 * ## Mutations killed, measured 2026-07-29 — the metric path from the solved pose
 *
 * | mutation | failures |
 * |---|---|
 * | trusted gate bypassed — `fix` believes `detection.solve` raw, `TagPose.trusted` uncalled | 1 |
 * | fallback dropped — an untrusted or missing solve produces **no fix at all** | 20 |
 * | metric nadir-ray correction dropped — solved translation used as a plumb ray | 7 |
 * | metric camera→body axis sign flipped (`camY` negated on the solved path) | 6 |
 * | heading rotation dropped — body offsets flown as world, both paths | 3 |
 * | `metric` stuck false on the trusted path | 5 |
 * | `metric` stuck true on the bearing path | 3 |
 * | `rangeM` fed the baro altitude instead of the solved `tz` | 3 |
 * | `fromHeightM` fed the solved `tz` instead of the baro | 3 |
 * | calibration single-owner: `fix` reads `CameraCalibration.ASSUMED`, ignores its argument | 1 |
 *
 * Same protocol as above — one mutation alone, whole suite (2 488 tests), fresh
 * `test-results` per run, counts from the fresh XML, reverted after. Nothing survived and
 * nothing was NO-RUN. The intake's rows are `CameraCalibrationTest`'s table and the record
 * line's are `RecordedTagSinkTest`'s.
 *
 * ## Mutations killed, measured 2026-07-29 — the range ladder (landing07)
 *
 * | mutation | failures |
 * |---|---|
 * | SIZE rung ignores [TagWorld.SIZE_RANGE_MIN_PIXELS] — a 10 px smudge's "range" believed | 1 |
 * | fix lateral scaled by the altitude while SIZE provenance is claimed — the two-places split | 1 |
 *
 * Same protocol, whole 2511-test suite per mutant, fresh `test-results`, reverted after. The
 * ladder's law-side and engine-side rows (precedence, freshness, the frozen commit window, the
 * floor seam, the record lines) are `GuidedAutolandTest`'s and `TagDescentGuidanceTest`'s
 * tables — one campaign, three owning files, each row recorded where its property lives.
 */
class TagWorldTest {

    private fun det(cx: Double, cy: Double, id: Int = 0, px: Double = 100.0) =
        TagDetection(id, hamming = 0, centreX = cx, centreY = cy, longestEdgePixels = px, decisionMargin = 40.0)

    private fun pose(
        n: Double? = 0.0,
        e: Double? = 0.0,
        alt: Double? = 10.0,
        heading: Double? = 0.0,
        pitch: Double? = -90.0,
        reported: Boolean = false,
    ) = CameraPose(n, e, alt, heading, pitch, reported)

    /**
     * The measured nadir pixel at 1920×1080, **written out rather than read from [TagWorld]**.
     *
     * Reading `TagWorld.nadirPointX(1920)` here would make every test below agree with whatever
     * the constant happens to say, including a constant somebody has broken. Duplicating the
     * number is the point: `docs/measurements/2026-07-28-nadir-image-point.md` says 970.7 and
     * 615.2, and if `src/main` stops saying that, these fail.
     */
    private val nadirX = 970.7
    private val nadirY = 615.2

    // ─────────────────────────────────────────────────────── the intrinsics

    /**
     * The focal length is the fitted one, not DJI's published figure.
     *
     * 1465.9 px and 1448.3 px from two independent flights, agreeing to 1.2 %. The 82.1° diagonal
     * field of view DJI publishes implies 1266 px and is 13 % low — the spec is presumably the
     * sensor's full frame and the video stream is cropped.
     */
    @Test
    fun theFocalLengthIsTheFittedOneAndScalesWithWidth() {
        assertEquals(1457.0, CameraCalibration.ASSUMED.fxAt1920, 0.0)
        assertEquals(1457.0, TagWorld.focalPx(1920), 1e-9)
        assertEquals(728.5, TagWorld.focalPx(960), 1e-9)
        assertEquals(364.25, TagWorld.focalPx(480), 1e-9)
        // The published-spec value must not creep back in.
        assertTrue("1266 px is DJI's number and it is 13 % low", CameraCalibration.ASSUMED.fxAt1920 > 1400)
    }

    /**
     * **The bearing is resolution-independent**, which is the property the scaling exists for: the
     * same tag seen at half resolution is at the same angle, so it must produce the same metres.
     */
    @Test
    fun theSameBearingAtHalfResolutionGivesTheSamePlace() {
        val full = TagWorld.cameraFrame(960.0 + 200.0, 540.0, 1920, 1080, 10.0)
        val half = TagWorld.cameraFrame(480.0 + 100.0, 270.0, 960, 540, 10.0)
        assertEquals(full.first, half.first, 1e-9)
        assertEquals(full.second, half.second, 1e-9)
    }

    /**
     * [TagWorld.cameraFrame] still measures from the **image centre**, and that is not an oversight.
     *
     * It owes its caller a direction in the *optical* frame, which is referred to the principal
     * point — and the principal point is still unmeasured, still assumed to be the centre. The
     * nadir pixel is a different quantity ([theNadirPixelIsTheMeasuredOne]) and putting it here
     * would mislabel a nadir-referred direction as an optical-frame one.
     */
    @Test
    fun theOpticalFrameStillMeasuresFromTheImageCentre() {
        val (x, y, z) = TagWorld.cameraFrame(960.0, 540.0, 1920, 1080, 7.0)
        assertEquals(0.0, x, 1e-12)
        assertEquals(0.0, y, 1e-12)
        assertEquals(7.0, z, 1e-12)
    }

    /**
     * **The nadir pixel is the measured one**, not the image centre.
     *
     * Three yaw turns over a tag on 2026-07-28 put "straight down" at (970.7, 615.2) — 2.99° from
     * the centre this code assumed for its whole life before that.
     */
    @Test
    fun theNadirPixelIsTheMeasuredOne() {
        assertEquals(nadirX, TagWorld.nadirPointX(1920), 1e-9)
        assertEquals(nadirY, TagWorld.nadirPointY(1920, 1080), 1e-9)
        assertNotEquals(960.0, TagWorld.nadirPointX(1920), 1.0)
        assertNotEquals(540.0, TagWorld.nadirPointY(1920, 1080), 1.0)
    }

    /**
     * The nadir offset scales with **width on both axes** — including the vertical one.
     *
     * What is fixed is the *angle*; the pixels per radian is the focal length, and that scales with
     * width. Scaling the y offset by height instead would give the same answer at 16:9 and a wrong
     * one the moment the stream's aspect ratio changed, which is the kind of bug that only appears
     * on the day somebody reconfigures the encoder.
     */
    @Test
    fun theNadirOffsetScalesWithWidthOnBothAxes() {
        assertEquals(960 / 2.0 + (nadirX - 960.0) / 2.0, TagWorld.nadirPointX(960), 1e-9)
        assertEquals(540 / 2.0 + (nadirY - 540.0) / 2.0, TagWorld.nadirPointY(960, 540), 1e-9)

        // **A non-16:9 frame, and it is the only assertion here that can fail.** At 960×540 the
        // width ratio and the height ratio are both exactly 0.5, so scaling the y offset by height
        // gives the identical answer and the mutation is invisible. It was invisible: this test
        // passed with the wrong scaling until a mutation run found it surviving. 1280×960 is 4:3,
        // where the two ratios are 0.667 and 0.889 and the answers differ by 16.7 px.
        assertEquals(960 / 2.0 + (nadirY - 540.0) * (1280.0 / 1920.0),
            TagWorld.nadirPointY(1280, 960), 1e-9)
        assertNotEquals("scaling y by height must not pass",
            960 / 2.0 + (nadirY - 540.0) * (960.0 / 1080.0), TagWorld.nadirPointY(1280, 960), 1.0)
        // The same real direction, at two resolutions, is the same place.
        val full = TagWorld.nadirFrame(nadirX + 200.0, nadirY, 1920, 1080, 10.0)
        val half = TagWorld.nadirFrame(TagWorld.nadirPointX(960) + 100.0, TagWorld.nadirPointY(960, 540), 960, 540, 10.0)
        assertEquals(full.first, half.first, 1e-9)
        assertEquals(full.second, half.second, 1e-9)
    }

    /** A tag on the nadir pixel is directly beneath the aircraft: no lateral offset at all. */
    @Test
    fun aTagAtTheNadirPixelIsDirectlyBeneath() {
        val (x, y) = TagWorld.nadirFrame(nadirX, nadirY, 1920, 1080, 7.0)
        assertEquals(0.0, x, 1e-12)
        assertEquals(0.0, y, 1e-12)
    }

    /**
     * **The regression this correction exists for.** A tag at the image centre is *not* beneath the
     * aircraft, and before 2026-07-28 this code said it was.
     *
     * 75.2 px above the nadir pixel at 1457 px focal length and 10 m is 0.516 m — and "above" is
     * ahead, so the aircraft that flew to it stopped half a metre short.
     */
    @Test
    fun aTagAtTheImageCentreIsNotBeneathTheAircraft() {
        val f = TagWorld.fix(det(960.0, 540.0), 1920, 1080, pose(heading = 0.0), 0)!!
        assertEquals((nadirY - 540.0) / 1457.0 * 10.0, f.northM, 1e-6)
        assertEquals(-(nadirX - 960.0) / 1457.0 * 10.0, f.eastM, 1e-6)
        assertTrue("the whole point is that this is not zero", f.northM > 0.5)
    }

    /** Right in the image is +x, down in the image is +y — the frame the seam documents. */
    @Test
    fun theCameraFrameIsRightAndDown() {
        val right = TagWorld.cameraFrame(1160.0, 540.0, 1920, 1080, 10.0)
        assertTrue(right.first > 0)
        assertEquals(0.0, right.second, 1e-12)
        val down = TagWorld.cameraFrame(960.0, 740.0, 1920, 1080, 10.0)
        assertEquals(0.0, down.first, 1e-12)
        assertTrue(down.second > 0)
        // 200 px at 1457 px focal length and 10 m range is 1.373 m.
        assertEquals(200.0 / 1457.0 * 10.0, right.first, 1e-9)
    }

    /** The size-based range is carried for cross-checking, and it is the `f·S/px` the fit came from. */
    @Test
    fun theSizeBasedRangeIsTheFitsOwnRelation() {
        // A 75 mm tag 100 px across at 1920: 1457 × 0.075 / 100 = 1.093 m.
        assertEquals(1.09275, TagWorld.rangeFromSize(100.0, 1920, 0.075)!!, 1e-5)
        assertNull(TagWorld.rangeFromSize(0.0, 1920, 0.075))
        assertNull(TagWorld.rangeFromSize(100.0, 1920, 0.0))
    }

    // ─────────────────────────────────────────── the range ladder (landing07)

    /**
     * **The SIZE rung: a measurable tag's own size scales the bearing fix, not the baro.**
     *
     * Landing07's landing B (`datasets/landing07/20260729-095413.001.jsonl`, 2026-07-29): the
     * baro read 0.7 m while the tag's size said 1.93 m — and the size was right. A bearing
     * fix's lateral offset scaled by the lying baro under-reported the error 2.76×, which is
     * the whole cascade. Here: identical detection, baro 0.7 vs size range 1.93 — the lateral
     * must be the size range's, 1.93/0.7 = 2.76× the baro-scaled one, exactly.
     */
    @Test
    fun aMeasurableTagsSizeOutranksTheBarometer() {
        val d = det(nadirX, nadirY - 200.0, px = 55.0) // landing B's tag: ~55 px, well above 20
        val sized = TagWorld.fix(d, 1920, 1080, pose(alt = 0.7, heading = 0.0), 0, sizeRangeM = 1.93)!!
        val baroScaled = TagWorld.fix(d, 1920, 1080, pose(alt = 0.7, heading = 0.0), 0)!!
        assertEquals(RangeSource.SIZE, sized.rangeSource)
        assertEquals(RangeSource.BARO, baroScaled.rangeSource)
        // Same pixels, same direction — the scale is the instrument, and only the instrument.
        assertEquals(200.0 / 1457.0 * 1.93, sized.northM, 1e-9)
        assertEquals(200.0 / 1457.0 * 0.7, baroScaled.northM, 1e-9)
        assertEquals(1.93 / 0.7, sized.northM / baroScaled.northM, 1e-9)
        // The provenance and the number travel together: tagRangeM() is the scale that flew.
        assertEquals(1.93, sized.tagRangeM()!!, 0.0)
        assertNull("a baro-scaled fix has no tag-vouched range", baroScaled.tagRangeM())
        // And the frame's baro reading is still on both, for the record's cross-reading.
        assertEquals(0.7, sized.fromHeightM, 0.0)
    }

    /**
     * **The SIZE rung's floor: below [TagWorld.SIZE_RANGE_MIN_PIXELS] the size is not a
     * range** — the fit's residual triples below ~20 px (`2026-07-27-tag-detection-rate.md`
     * §3), so a 10 px smudge's "range" is never believed over the baro, and the boundary is
     * inclusive on the usable side like every gate in this project.
     */
    @Test
    fun aTinyTagsSizeIsNotARangeAndTheLadderFallsToBaro() {
        assertEquals(20.0, TagWorld.SIZE_RANGE_MIN_PIXELS, 0.0)
        val tiny = det(nadirX, nadirY - 200.0, px = 10.0)
        val f = TagWorld.fix(tiny, 1920, 1080, pose(alt = 6.0, heading = 0.0), 0, sizeRangeM = 10.93)!!
        assertEquals(RangeSource.BARO, f.rangeSource)
        assertEquals("the baro scaled it", 200.0 / 1457.0 * 6.0, f.northM, 1e-9)
        assertNull(f.tagRangeM())

        val atFloor = det(nadirX, nadirY - 200.0, px = TagWorld.SIZE_RANGE_MIN_PIXELS)
        val g = TagWorld.fix(atFloor, 1920, 1080, pose(alt = 6.0, heading = 0.0), 0, sizeRangeM = 5.46)!!
        assertEquals("exactly at the floor is usable", RangeSource.SIZE, g.rangeSource)

        // No size range supplied at all (degenerate pixels, legacy caller): baro, not a guess.
        val h = TagWorld.fix(det(nadirX, nadirY - 200.0), 1920, 1080, pose(alt = 6.0, heading = 0.0), 0)!!
        assertEquals(RangeSource.BARO, h.rangeSource)
    }

    /** A trusted solve outranks everything: SOLVE provenance, and tagRangeM() is the solved tz. */
    @Test
    fun aTrustedSolveIsTheLaddersTopRung() {
        val solve = TagPoseSolve(
            qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0,
            tx = 0.0, ty = 0.0, tz = 2.0,
            err1 = 1.5e-7, err2 = 6.5e-7, tagSizeM = 0.075,
        )
        val d = TagDetection(
            0, hamming = 0, centreX = nadirX, centreY = nadirY,
            longestEdgePixels = 100.0, decisionMargin = 40.0, solve = solve,
        )
        val f = TagWorld.fix(d, 1920, 1080, pose(alt = 0.7), 0, sizeRangeM = 1.95)!!
        assertTrue(f.metric)
        assertEquals(RangeSource.SOLVE, f.rangeSource)
        assertEquals(2.0, f.tagRangeM()!!, 0.0)
        assertEquals("the size range still rides along for the record", 1.95, f.sizeRangeM!!, 0.0)
    }

    // ─────────────────────────────────────────────────────────── the refusals

    @Test
    fun everyMissingInputRefusesRatherThanDefaulting() {
        val d = det(960.0, 540.0)
        val cases = mapOf(
            "north" to pose(n = null),
            "east" to pose(e = null),
            "altitude" to pose(alt = null),
            "heading" to pose(heading = null),
            "camera pitch" to pose(pitch = null),
        )
        for ((what, p) in cases) {
            assertNull("a missing $what must refuse, not default", TagWorld.fix(d, 1920, 1080, p, 0))
        }
    }

    /**
     * **A camera that is not near nadir refuses.** Not because the geometry is impossible, but
     * because it becomes dependent on the gimbal's lever arm and on the ground being flat under a
     * slanted ray — two further unmeasured terms stacked on the ones already there.
     */
    @Test
    fun aCameraAwayFromNadirRefuses() {
        val d = det(960.0, 540.0)
        assertNotNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -90.0), 0))
        assertNotNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -80.0), 0))
        assertNotNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -100.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -70.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = 0.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -30.0), 0))
        assertEquals(12.0, TagWorld.NADIR_TOLERANCE_DEG, 0.0)
    }

    /**
     * **The nadir tolerance judges the believed pitch whatever its source.** The reported
     * fallback (2026-07-28) must not arrive with a looser gate than the commanded angle had: a
     * reported −60° stacks exactly the same unmeasured terms a commanded −60° does.
     */
    @Test
    fun aReportedPitchIsJudgedByTheSameToleranceAsACommandedOne() {
        val d = det(960.0, 540.0)
        assertNotNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -80.0, reported = true), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -60.0, reported = true), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = null, reported = true), 0))
    }

    /**
     * **The provenance flag on the fix is the pose's, both ways.** A reader joining fixes
     * against the gimbal record must be able to tell a commanded-pitch fix from a
     * reported-pitch one — the `bearingAssumed` discipline applied to the 2026-07-28 fallback.
     */
    @Test
    fun theFixCarriesWhichPitchBeliefItRestedOn() {
        val d = det(960.0, 540.0)
        assertTrue(TagWorld.fix(d, 1920, 1080, pose(reported = true), 0)!!.pitchReported)
        assertFalse(TagWorld.fix(d, 1920, 1080, pose(reported = false), 0)!!.pitchReported)
    }

    /** At or below the tag plane there is no downward ray, and a negative range would mirror it. */
    @Test
    fun aNonPositiveAltitudeRefuses() {
        val d = det(1160.0, 540.0)
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = 0.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = -1.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = Double.NaN), 0))
        assertNotNull(TagWorld.fix(d, 1920, 1080, pose(alt = 0.5), 0))
    }

    // ────────────────────────────────────────────────────────── the geometry

    /** Nose north, tag above centre in the image: the tag is ahead, so north of the aircraft. */
    @Test
    fun aTagAboveCentreIsAheadOfTheAircraft() {
        // 200 px above the NADIR pixel at 1457 px focal and 10 m: 1.373 m ahead.
        val f = TagWorld.fix(det(nadirX, nadirY - 200.0), 1920, 1080, pose(heading = 0.0), 0)!!
        assertEquals(200.0 / 1457.0 * 10.0, f.northM, 1e-9)
        assertEquals(0.0, f.eastM, 1e-9)
    }

    /** Nose north, tag right of centre: the tag is to starboard, so east. */
    @Test
    fun aTagRightOfCentreIsEastWhenTheNoseIsNorth() {
        val f = TagWorld.fix(det(nadirX + 200.0, nadirY), 1920, 1080, pose(heading = 0.0), 0)!!
        assertEquals(0.0, f.northM, 1e-9)
        assertEquals(200.0 / 1457.0 * 10.0, f.eastM, 1e-9)
    }

    /** Turn the aircraft 90° and the same pixel offset comes out in a different world direction. */
    @Test
    fun theHeadingRotatesTheOffsetIntoTheWorld() {
        val d = det(nadirX, nadirY - 200.0)
        val metres = 200.0 / 1457.0 * 10.0
        // Nose east: "ahead" is east.
        val east = TagWorld.fix(d, 1920, 1080, pose(heading = 90.0), 0)!!
        assertEquals(0.0, east.northM, 1e-9)
        assertEquals(metres, east.eastM, 1e-9)
        // Nose south: "ahead" is south.
        val south = TagWorld.fix(d, 1920, 1080, pose(heading = 180.0), 0)!!
        assertEquals(-metres, south.northM, 1e-9)
        assertEquals(0.0, south.eastM, 1e-6)
        // Nose west, negative form, which is what `yawDeg`'s [-180, 180] range actually delivers.
        val west = TagWorld.fix(d, 1920, 1080, pose(heading = -90.0), 0)!!
        assertEquals(0.0, west.northM, 1e-9)
        assertEquals(-metres, west.eastM, 1e-9)
    }

    /** The fix is relative to where the aircraft is, not to the datum. */
    @Test
    fun theFixIsAddedToTheAircraftsOwnPosition() {
        val f = TagWorld.fix(det(nadirX, nadirY), 1920, 1080, pose(n = 12.0, e = -5.0), 0)!!
        assertEquals(12.0, f.northM, 1e-9)
        assertEquals(-5.0, f.eastM, 1e-9)
    }

    /** The range used is the **altitude**, not the tag's apparent size. See `TagWorld`'s KDoc. */
    @Test
    fun theRangeComesFromAltitudeNotFromTheTagsSize() {
        val small = det(nadirX + 200.0, nadirY, px = 15.0)
        val large = det(nadirX + 200.0, nadirY, px = 400.0)
        val a = TagWorld.fix(small, 1920, 1080, pose(alt = 6.0), 0)!!
        val b = TagWorld.fix(large, 1920, 1080, pose(alt = 6.0), 0)!!
        assertEquals("apparent size must not move the fix", a.eastM, b.eastM, 1e-12)
        assertEquals(6.0, a.fromHeightM, 1e-9)
    }

    // ─────────────────────────────────────────────── the honesty labels

    /**
     * **`metric` stays false.** Not a placeholder: the focal length is a fit, the principal point is
     * assumed to be the image centre, and distortion is entirely unmeasured.
     */
    @Test
    fun everyFixIsLabelledNonMetric() {
        val f = TagWorld.fix(det(1000.0, 500.0), 1920, 1080, pose(), 0)!!
        assertFalse("metric must stay false until a real calibration exists", f.metric)
    }

    /**
     * **`bearingAssumed` stays true.** Nothing in this project has flown the one manoeuvre that
     * settles which way the image sits on the airframe at nadir.
     */
    @Test
    fun everyFixIsLabelledBearingAssumed() {
        val f = TagWorld.fix(det(1000.0, 500.0), 1920, 1080, pose(), 0)!!
        assertTrue("the camera-to-body rotation has not been measured", f.bearingAssumed)
        assertTrue(TagWorld.NADIR_IMAGE_UP_IS_NOSE)
    }

    /** The frame's own arrival time travels onto the fix, not the time it was computed. */
    @Test
    fun theFixCarriesTheFramesTimestamp() {
        val f = TagWorld.fix(det(1000.0, 500.0), 1920, 1080, pose(), 987_654_321L)!!
        assertEquals(987_654_321L, f.atNanos)
    }

    /** The tag's own id and size ride along, because a fix without a trust proxy is a bare number. */
    @Test
    fun theFixCarriesTheIdAndTheSizeProxy() {
        val f = TagWorld.fix(det(1000.0, 500.0, id = 7, px = 63.5), 1920, 1080, pose(), 0)!!
        assertEquals(7, f.tagId)
        assertEquals(63.5, f.pixelSize, 1e-9)
    }

    // ──────────────────────────────────────── the metric path (2026-07-28)

    private fun trustedSolve(
        tx: Double,
        ty: Double,
        tz: Double,
        err1: Double = 5.0e-8,
        err2: Double = Double.POSITIVE_INFINITY,
    ) = TagPoseSolve(
        qx = 0.005, qy = 0.017, qz = -0.9998, qw = 0.011,
        tx = tx, ty = ty, tz = tz, err1 = err1, err2 = err2, tagSizeM = 0.075,
    )

    private fun solvedDet(
        cx: Double,
        cy: Double,
        px: Double = 100.0,
        solve: TagPoseSolve?,
    ) = TagDetection(0, hamming = 0, centreX = cx, centreY = cy, longestEdgePixels = px,
        decisionMargin = 170.0, solve = solve)

    /**
     * **A trusted solve makes the fix metric, and says so** — and says no more than that.
     * `rangeM` is the solve's own `tz`; `fromHeightM` stays the barometer, because the descent
     * laws fly height off that source and moving them onto the solved range is a flight-gated
     * decision this pass refuses to smuggle. `bearingAssumed` stays true: the solved translation
     * crosses camera→body through the same unflown NADIR_IMAGE_UP_IS_NOSE convention the pixel
     * ray does — a solve is a better *scale*, not a measured *rotation*.
     */
    @Test
    fun aTrustedSolveMakesTheFixMetricAndCarriesTheSolvedRange() {
        val d = solvedDet(nadirX, nadirY, px = 110.0, solve = trustedSolve(0.0, 0.0, 0.988))
        val f = TagWorld.fix(d, 1920, 1080, pose(alt = 0.9), 0)!!
        assertTrue("a trusted solve is a metric fix", f.metric)
        assertEquals(0.988, f.rangeM!!, 1e-12)
        assertEquals("height stays the baro — the solved range must not leak into it",
            0.9, f.fromHeightM, 1e-12)
        assertTrue("the camera-to-body rotation is still the same assumption",
            f.bearingAssumed)
    }

    /** The pitch-belief provenance rides the metric path exactly as it rides the bearing one. */
    @Test
    fun theMetricFixCarriesWhichPitchBeliefItRestedOn() {
        val d = solvedDet(nadirX, nadirY, px = 110.0, solve = trustedSolve(0.0, 0.0, 1.0))
        assertTrue(TagWorld.fix(d, 1920, 1080, pose(reported = true), 0)!!.pitchReported)
        assertFalse(TagWorld.fix(d, 1920, 1080, pose(reported = false), 0)!!.pitchReported)
    }

    /**
     * **An untrusted solve is no solve: the fix is the bearing fix, field for field.** The gate
     * is [TagPose.trusted]'s, *called* — its thresholds live in one object with their
     * measurements, and a flight with zero believable solves must produce fixes
     * indistinguishable from every flight before the metric path existed.
     */
    @Test
    fun anUntrustedSolveFallsBackToTheBearingFixFieldForField() {
        val cases = mapOf(
            // Below the 60 px gate: the solver's tilt error doubles bin to bin down here.
            "pixel gate" to
                solvedDet(1100.0, 700.0, px = 59.9, solve = trustedSolve(0.1, 0.2, 3.0)),
            // Ambiguity ratio 0.8 > 0.5: the two PnP minima are a coin flip.
            "ambiguity gate" to
                solvedDet(1100.0, 700.0, solve = trustedSolve(0.1, 0.2, 3.0, err1 = 8.0e-8, err2 = 1.0e-7)),
            // A partial solve (non-finite translation) is refused entire by the gate.
            "partial solve" to
                solvedDet(1100.0, 700.0, solve = trustedSolve(Double.NaN, 0.2, 3.0)),
        )
        for ((what, d) in cases) {
            // The bearing fix from the *identical* detection with the solve stripped.
            val bare = TagWorld.fix(d.copy(solve = null), 1920, 1080, pose(heading = 37.0), 5L)!!
            val f = TagWorld.fix(d, 1920, 1080, pose(heading = 37.0), 5L)
            assertEquals("the $what must fall back to the bearing fix exactly", bare, f!!)
            assertFalse(f.metric)
            assertNull("no believed solve, no claimed range", f.rangeM)
        }
        // The px=100 default of det() is above the gate: same detection with px=60.0 believes.
        assertTrue(TagWorld.fix(
            solvedDet(1100.0, 700.0, px = 60.0, solve = trustedSolve(0.1, 0.2, 3.0)),
            1920, 1080, pose(heading = 37.0), 5L,
        )!!.metric)
    }

    /**
     * **The metric path refuses exactly where the bearing path does** — a trusted solve buys a
     * better scale, never a bypass. Position, heading and believed pitch are still needed to
     * cross into the world; the nadir tolerance still stacks the same unmeasured terms; and the
     * altitude gate is deliberately kept even though the metric lateral does not need the baro
     * (see [TagWorld.fix]'s KDoc: when-fixes-exist must not move in the pass that changes the
     * arithmetic).
     */
    @Test
    fun theMetricPathRefusesExactlyWhereTheBearingPathDoes() {
        val d = solvedDet(nadirX, nadirY, px = 110.0, solve = trustedSolve(0.0, 0.0, 1.0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(n = null), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(heading = null), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = null), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(pitch = -60.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = null), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = 0.0), 0))
        assertNull(TagWorld.fix(d, 1920, 1080, pose(alt = -0.1), 0))
    }

    /**
     * **The metric frame chain, pinned step by step**: nadir-ray correction, then the same
     * camera→body→world lines as the bearing path. A solve dead on the optical axis
     * (tx = ty = 0) is *not* beneath the aircraft — straight down is the measured nadir pixel,
     * 2.99° away — so the fix lands `tz·(75.2, −10.7)/1457` (ahead, west at heading 0), the
     * image-centre regression's twin on the solved path.
     */
    @Test
    fun aSolveOnTheOpticalAxisIsNotBeneathTheAircraft() {
        val d = solvedDet(960.0, 540.0, px = 110.0, solve = trustedSolve(0.0, 0.0, 3.0))
        val f = TagWorld.fix(d, 1920, 1080, pose(heading = 0.0, alt = 3.0), 0)!!
        assertEquals(3.0 * (nadirY - 540.0) / 1457.0, f.northM, 1e-9)   // 0.15484 m ahead
        assertEquals(-3.0 * (nadirX - 960.0) / 1457.0, f.eastM, 1e-9)   // 0.02203 m west
        assertTrue("the whole point is that this is not zero", f.northM > 0.15)
    }

    /** Turn the aircraft and the same solved translation comes out in a different world direction. */
    @Test
    fun theHeadingRotatesTheSolvedTranslationIntoTheWorld() {
        // ty = −0.5 (tag above image centre = ahead), tz = 2.0. After the nadir-ray correction:
        // camX = −2·10.7/1457 = −0.0146877, camY = −0.5 − 2·75.2/1457 = −0.6032258.
        val d = solvedDet(960.0, 175.0, px = 110.0, solve = trustedSolve(0.0, -0.5, 2.0))
        val north = TagWorld.fix(d, 1920, 1080, pose(heading = 0.0, alt = 2.0), 0)!!
        assertEquals(0.603225806451613, north.northM, 1e-9)
        assertEquals(-0.014687714481812, north.eastM, 1e-9)
        val east = TagWorld.fix(d, 1920, 1080, pose(heading = 90.0, alt = 2.0), 0)!!
        assertEquals(0.014687714481812, east.northM, 1e-9)
        assertEquals(0.603225806451613, east.eastM, 1e-9)
        val south = TagWorld.fix(d, 1920, 1080, pose(heading = 180.0, alt = 2.0), 0)!!
        assertEquals(-0.603225806451613, south.northM, 1e-9)
        assertEquals(0.014687714481812, south.eastM, 1e-9)
    }

    /** The metric fix is relative to where the aircraft is, exactly as the bearing fix is. */
    @Test
    fun theMetricFixIsAddedToTheAircraftsOwnPosition() {
        val d = solvedDet(nadirX, nadirY, px = 110.0, solve = trustedSolve(0.0, 0.0, 1.0))
        val f = TagWorld.fix(d, 1920, 1080, pose(n = 12.0, e = -5.0), 0)!!
        // tx = ty = 0 → after correction, small negative offsets — the nadir-ray terms only.
        assertEquals(12.0 + 1.0 * (nadirY - 540.0) / 1457.0, f.northM, 1e-9)
        assertEquals(-5.0 - 1.0 * (nadirX - 960.0) / 1457.0, f.eastM, 1e-9)
    }

    /**
     * **When the solved range equals the altitude, the metric lateral equals the bearing lateral
     * algebraically.** The solve's projection identity (`tx/tz = (cx − 960)/f`) plus the shared
     * nadir-ray correction make the two paths the same arithmetic with a different range source —
     * which is the design: the switch changes *scale honesty*, never direction. A step between
     * the paths at the 60 px boundary would be a bug this test makes visible.
     */
    @Test
    fun theMetricLateralEqualsTheBearingLateralWhenRangesAgree() {
        val cx = 1100.0
        val cy = 700.0
        val alt = 2.0
        // The solve that projects exactly to (1100, 700) at range 2.0 under the assumed camera.
        val solved = trustedSolve(
            tx = (cx - 960.0) / 1457.0 * alt,
            ty = (cy - 540.0) / 1457.0 * alt,
            tz = alt,
        )
        val metric = TagWorld.fix(solvedDet(cx, cy, px = 110.0, solve = solved), 1920, 1080,
            pose(heading = 63.0, alt = alt), 0)!!
        val bearing = TagWorld.fix(det(cx, cy), 1920, 1080, pose(heading = 63.0, alt = alt), 0)!!
        assertTrue(metric.metric)
        assertFalse(bearing.metric)
        assertEquals(bearing.northM, metric.northM, 1e-12)
        assertEquals(bearing.eastM, metric.eastM, 1e-12)
    }

    /**
     * **The calibration the caller hands in is the calibration both paths compute with.** One
     * `CameraCalibration` per session feeds the JNI solve and this projection; a path that
     * quietly read [CameraCalibration.ASSUMED] instead of its argument would let the solve and
     * the world math disagree about the camera — the exact drift the single owner exists to
     * prevent.
     */
    @Test
    fun theCalibrationHandedInMovesBothPaths() {
        // A doubled focal length halves every bearing offset.
        val fx2 = CameraCalibration.ASSUMED.copy(fxAt1920 = 2914.0, fyAt1920 = 2914.0)
        val b1 = TagWorld.fix(det(1100.0, 700.0), 1920, 1080, pose(), 0)!!
        val b2 = TagWorld.fix(det(1100.0, 700.0), 1920, 1080, pose(), 0, fx2)!!
        // Same reference pixel (nadir offsets scale by fx too — the *angle* would change, but
        // the pixel is stored absolute), so the ray simply shortens with the doubled focal.
        assertEquals(b1.northM / 2.0, b2.northM, 1e-9)
        assertEquals(b1.eastM / 2.0, b2.eastM, 1e-9)
        // A calibrated principal point moves the metric path's nadir-ray correction: with cx at
        // the measured nadir pixel the x-correction vanishes.
        val cxAtNadir = CameraCalibration.ASSUMED.copy(cxAt1920 = 970.7)
        val d = solvedDet(970.7, 615.2, px = 110.0, solve = trustedSolve(0.0, 0.0, 2.0))
        val m = TagWorld.fix(d, 1920, 1080, pose(heading = 0.0, alt = 2.0), 0, cxAtNadir)!!
        assertEquals("cx at the nadir pixel leaves no x-correction", 0.0, m.eastM, 1e-12)
        val mDefault = TagWorld.fix(d, 1920, 1080, pose(heading = 0.0, alt = 2.0), 0)!!
        assertEquals(-2.0 * 10.7 / 1457.0, mDefault.eastM, 1e-9)
    }

    // ── landing06 fixture vectors: real recorded bytes, session 20260728-205913 ──
    //
    // Three trusted solves out of that flight's 59 (of 321 detections), pulled verbatim from
    // `datasets/landing06/20260728-205913.001.jsonl` — the same bytes `landing06.db`'s
    // `detections_blob` carries. Chosen to span the flight: the first believable solve of the
    // first approach, and two from the final descent where the baro is at its measured worst.
    // Heading and altitude are the record's own (`dji_state` at ≤18 ms, the line's `from_h`).
    // Expected values are this file's frame chain evaluated by hand — if `src/main` stops
    // agreeing with them, the geometry moved, not the data.

    private fun landing06(
        cx: Double, cy: Double, px: Double,
        c: DoubleArray, q: DoubleArray, tx: Double, ty: Double, tz: Double, err1: Double,
    ) = TagDetection(
        id = 0, hamming = 0, centreX = cx, centreY = cy, longestEdgePixels = px,
        decisionMargin = 170.0,
        corners = TagCorners(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]),
        solve = TagPoseSolve(
            qx = q[0], qy = q[1], qz = q[2], qw = q[3],
            tx = tx, ty = ty, tz = tz,
            err1 = err1, err2 = Double.POSITIVE_INFINITY, tagSizeM = 0.075,
        ),
    )

    /** t=20.114541 s: px 110.8, solve t=(−0.065, 0.039, 0.988), yaw −90.9°, baro 0.9 m. */
    private val landing06First = landing06(
        864.2, 597.1, 110.8,
        doubleArrayOf(920.55, 543.21, 809.78, 540.48, 807.87, 650.86, 918.42, 653.43),
        doubleArrayOf(0.005368, 0.017026, -0.999778, 0.011228),
        -0.065, 0.039, 0.988, 5.2024e-8,
    )

    /** t=144.554696 s: px 146.8, solve t=(0.009, 0.032, 0.746), yaw −93.9°, baro 1.1 m. */
    private val landing06Mid = landing06(
        978.3, 601.8, 146.8,
        doubleArrayOf(1062.84, 541.94, 918.35, 517.17, 894.11, 661.32, 1038.53, 686.75),
        doubleArrayOf(-0.023904, -0.001771, -0.996175, 0.084027),
        0.009, 0.032, 0.746, 4.0302e-8,
    )

    /** t=147.656306 s: px 203.8, solve t=(0.059, 0.079, 0.540), yaw −94.3°, baro 1.0 m. */
    private val landing06Last = landing06(
        1118.9, 753.2, 203.8,
        doubleArrayOf(1236.38, 671.19, 1035.77, 635.4, 1002.42, 834.42, 1202.58, 871.85),
        doubleArrayOf(-0.036608, -0.005795, -0.995734, 0.084503),
        0.059, 0.079, 0.540, 7.48e-8,
    )

    /** The real solves pass the real gate — the fixtures are inside the trusted population. */
    @Test
    fun theLandingSixSolvesAreTrusted() {
        for (d in listOf(landing06First, landing06Mid, landing06Last)) {
            assertNotNull(TagPose.trusted(d.longestEdgePixels, d.solve))
        }
    }

    /**
     * **The metric fixes from landing06's own bytes**, at the pose the record says the aircraft
     * held (aircraft at the origin so the fix reads as the offset). The bearing fix from the
     * identical detection is asserted beside each, because the *disagreement* is the feature:
     * the directions match and the scales differ by exactly the baro-versus-tag range gap —
     * +0.088 m at first sight, **−0.354 m and −0.460 m in the final descent**, which is the
     * measured baro dishonesty the metric path exists to not fly.
     */
    @Test
    fun theLandingSixMetricFixesComeOutWhereTheFrameChainSays() {
        // t=20.114541: metric (−0.0724, −0.0109) from range 0.988; bearing (−0.0660, −0.0101)
        // from baro 0.9 — 6.5 mm apart, ranges 88 mm apart.
        val f1 = TagWorld.fix(landing06First, 1920, 1080, pose(heading = -90.9, alt = 0.9), 0)!!
        assertTrue(f1.metric)
        assertEquals(-0.072435203, f1.northM, 1e-6)
        assertEquals(-0.010857125, f1.eastM, 1e-6)
        assertEquals(0.988, f1.rangeM!!, 1e-12)
        assertEquals(0.9, f1.fromHeightM, 1e-12)
        val b1 = TagWorld.fix(landing06First.copy(solve = null), 1920, 1080,
            pose(heading = -90.9, alt = 0.9), 0)!!
        assertEquals(-0.065953361, b1.northM, 1e-6)
        assertEquals(-0.010145809, b1.eastM, 1e-6)

        // t=144.554696: the baro says 1.1 m, the tag says 0.746 m.
        val f2 = TagWorld.fix(landing06Mid, 1920, 1080, pose(heading = -93.9, alt = 1.1), 0)!!
        assertEquals(0.003071009, f2.northM, 1e-6)
        assertEquals(-0.006727681, f2.eastM, 1e-6)
        assertEquals(0.746, f2.rangeM!!, 1e-12)

        // t=147.656306: the baro says 1.0 m, the tag says 0.540 m — and the bearing fix is
        // 5 cm further out than the metric one, scaled up by exactly the baro's excess.
        val f3 = TagWorld.fix(landing06Last, 1920, 1080, pose(heading = -94.3, alt = 1.0), 0)!!
        assertEquals(0.058712993, f3.northM, 1e-6)
        assertEquals(0.046858708, f3.eastM, 1e-6)
        assertEquals(0.540, f3.rangeM!!, 1e-12)
        val b3 = TagWorld.fix(landing06Last.copy(solve = null), 1920, 1080,
            pose(heading = -94.3, alt = 1.0), 0)!!
        assertEquals(0.108531161, b3.northM, 1e-6)
        assertEquals(0.086822032, b3.eastM, 1e-6)
    }

    /**
     * **Same direction, honest scale** — on real bytes. The metric lateral is the bearing
     * lateral times `tz/alt` to under half a millimetre on all three vectors (the residual is
     * the solve's projection noise plus the record's 3 dp rounding), which pins that the two
     * paths share one frame chain end to end: any axis flip, dropped rotation or divergent
     * nadir correction would blow this bound by orders of magnitude.
     */
    @Test
    fun theMetricAndBearingPathsAgreeOnDirectionOnRealBytes() {
        val cases = listOf(
            Triple(landing06First, -90.9 to 0.9, 0.988),
            Triple(landing06Mid, -93.9 to 1.1, 0.746),
            Triple(landing06Last, -94.3 to 1.0, 0.540),
        )
        for ((d, hAlt, tz) in cases) {
            val (heading, alt) = hAlt
            val m = TagWorld.fix(d, 1920, 1080, pose(heading = heading, alt = alt), 0)!!
            val b = TagWorld.fix(d.copy(solve = null), 1920, 1080,
                pose(heading = heading, alt = alt), 0)!!
            assertEquals(b.northM * (tz / alt), m.northM, 5e-4)
            assertEquals(b.eastM * (tz / alt), m.eastM, 5e-4)
        }
    }
}
