package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **One camera, one owner, and a drop-in that is applied whole or not at all.**
 *
 * Half of these tests are refusals, and that is the point: the intake exists so Ivan's chessboard
 * session becomes an `adb push`, and the cost of an intake is every malformed, misplaced or
 * contradictory file that could ride in with it. Each gate refuses **by name** and refuses the
 * **whole** file — [CameraCalibration.Loaded.Refused] carries no calibration at all, so a
 * half-applied file is unrepresentable, not merely untested.
 *
 * ## Mutations killed, measured 2026-07-29 — the intake's rows of the metric-fix campaign
 *
 * | mutation | failures |
 * |---|---|
 * | calibration single-owner: [CameraCalibration.fxPx] ignores `fxAt1920`, hardcodes 1457 | 1 |
 * | malformed file half-applied: the distortion refusal deleted, matrix applied anyway | 2 |
 * | malformed file half-applied: the geometry refusal deleted, foreign file applied unscaled | 1 |
 * | malformed file half-applied: the file's `cy` silently replaced by the 540 default | 2 |
 * | a `calibrated: false` file laundered into `measured = true` | 1 |
 *
 * The metric-path and record rows of the same campaign are in `TagWorldTest`'s and
 * `RecordedTagSinkTest`'s tables. Each row is one mutation applied alone to `src/main`, the
 * **whole suite** (2 488 tests) run against it with
 * `app/build/test-results/testDebugUnitTest` deleted first, the failure count read from the
 * fresh XML, and the mutation reverted. Nothing survived and nothing was NO-RUN.
 */
class CameraCalibrationTest {

    // ───────────────────────────────────────────────── the assumed prior

    /**
     * The default is the shipped prior, byte for byte: the two-flight focal fit, the assumed
     * image-centre principal point, the measured nadir pixel — and it says it is assumed.
     * Written as literals, not read back from the object: `calib/mini4pro_gimbal_1080.yaml`
     * and `2026-07-28-nadir-image-point.md` say these numbers, and if `src/main` stops saying
     * them, this fails.
     */
    @Test
    fun theAssumedPriorIsTheShippedNumbersAndSaysSo() {
        val c = CameraCalibration.ASSUMED
        assertEquals(1457.0, c.fxAt1920, 0.0)
        assertEquals(1457.0, c.fyAt1920, 0.0)
        assertEquals(960.0, c.cxAt1920, 0.0)
        assertEquals(540.0, c.cyAt1920, 0.0)
        assertEquals(970.7, c.nadirXAt1920, 0.0)
        assertEquals(615.2, c.nadirYAt1920, 0.0)
        assertFalse("the prior must never claim to be a measurement", c.measured)
    }

    /**
     * Every per-width number scales with **width on both axes** — the angle is what is fixed,
     * pixels-per-radian is the focal length, and the focal length scales with width. The 4:3
     * case is the assertion that can actually fail: at 16:9 the width and height ratios agree
     * and a height-scaled y would pass by coincidence (the mutation `TagWorldTest` caught
     * surviving on exactly this).
     */
    @Test
    fun everyPerWidthNumberScalesWithWidthOnBothAxes() {
        val c = CameraCalibration.ASSUMED
        assertEquals(1457.0, c.fxPx(1920), 1e-9)
        assertEquals(728.5, c.fxPx(960), 1e-9)
        assertEquals(728.5, c.fyPx(960), 1e-9)
        assertEquals(960.0, c.cxPx(1920), 1e-9)
        assertEquals(540.0, c.cyPx(1920, 1080), 1e-9)
        assertEquals(970.7, c.nadirPointX(1920), 1e-9)
        assertEquals(615.2, c.nadirPointY(1920, 1080), 1e-9)
        // 4:3 frame: the y offset must ride width/1920, not height/1080.
        assertEquals(480.0 + (615.2 - 540.0) * (1280.0 / 1920.0), c.nadirPointY(1280, 960), 1e-9)
        assertNotEquals("scaling y by height must not pass",
            480.0 + (615.2 - 540.0) * (960.0 / 1080.0), c.nadirPointY(1280, 960), 1.0)
        // And a calibrated cy follows the same rule.
        val moved = c.copy(cyAt1920 = 560.0)
        assertEquals(480.0 + 20.0 * (1280.0 / 1920.0), moved.cyPx(1280, 960), 1e-9)
    }

    // ───────────────────────────────────────────────── the intake, applied

    /** A plausible chessboard result, in exactly the schema `dimos cameracalibrate` writes. */
    private fun chessboardYaml(
        fx: Double = 1450.2,
        fy: Double = 1452.8,
        cx: Double = 1004.3,
        cy: Double = 528.9,
        width: Int = 1920,
        height: Int = 1080,
        k1: Double = 0.012,
        skew: Double = 0.0,
    ) = """
        image_width: $width
        image_height: $height
        camera_name: mini4pro_gimbal_1080p
        distortion_model: plumb_bob
        camera_matrix:
          rows: 3
          cols: 3
          data:
          - $fx
          - $skew
          - $cx
          - 0.0
          - $fy
          - $cy
          - 0.0
          - 0.0
          - 1.0
        distortion_coefficients:
          rows: 1
          cols: 5
          data:
          - $k1
          - -0.003
          - 0.0001
          - -0.0002
          - 0.0
    """.trimIndent()

    private fun applied(text: String): CameraCalibration {
        val loaded = CameraCalibration.parse(text, "camera_calibration.yaml")
        assertTrue("expected Applied, got $loaded", loaded is CameraCalibration.Loaded.Applied)
        return (loaded as CameraCalibration.Loaded.Applied).calibration
    }

    private fun refused(text: String): String {
        val loaded = CameraCalibration.parse(text, "camera_calibration.yaml")
        assertTrue("expected Refused, got $loaded", loaded is CameraCalibration.Loaded.Refused)
        return (loaded as CameraCalibration.Loaded.Refused).why
    }

    /**
     * The whole point of the seam: the chessboard file applies — every intrinsic taken from the
     * file, marked measured, named — while the nadir pixel stays the yaw-turn measurement,
     * which a chessboard does not re-measure (it is pointing + optics summed, a different
     * quantity — `CameraCalibration.nadirPointX`'s KDoc).
     */
    @Test
    fun aChessboardFileAppliesWholeAndIsMeasured() {
        val c = applied(chessboardYaml())
        assertEquals(1450.2, c.fxAt1920, 0.0)
        assertEquals(1452.8, c.fyAt1920, 0.0)
        assertEquals(1004.3, c.cxAt1920, 0.0)
        assertEquals(528.9, c.cyAt1920, 0.0)
        assertTrue(c.measured)
        assertEquals("camera_calibration.yaml", c.source)
        assertEquals("the nadir pixel is not the chessboard's to change",
            970.7, c.nadirXAt1920, 0.0)
        assertEquals(615.2, c.nadirYAt1920, 0.0)
    }

    /** Comments — which the repo prior is mostly made of — are stripped, not stumbled over. */
    @Test
    fun commentsAndBlankLinesDoNotConfuseTheParser() {
        val c = applied("# a prior, not a calibration\n\n" + chessboardYaml() + "\n# trailing\n")
        assertEquals(1450.2, c.fxAt1920, 0.0)
    }

    /**
     * **A file that disclaims itself stays an assumption.** The repo prior carries
     * `provenance: calibrated: false`; pushing it to the phone must not launder the fit into a
     * measurement — the flag rides the record's `camera_calibration` event and a reader
     * believes it.
     */
    @Test
    fun aFileMarkedUncalibratedAppliesButIsNotMeasured() {
        val text = chessboardYaml() + "\nprovenance:\n  calibrated: false\n"
        val c = applied(text)
        assertFalse("calibrated: false must survive the parse", c.measured)
        assertEquals(1450.2, c.fxAt1920, 0.0)
        assertTrue("the source must say the file disclaimed itself",
            c.source.contains("calibrated: false"))
    }

    // ───────────────────────────────────────────────── the intake, refused

    @Test
    fun aMissingCameraMatrixRefusesByName() {
        assertTrue(refused("image_width: 1920\nimage_height: 1080\n")
            .contains("camera_matrix"))
    }

    @Test
    fun aMissingImageSizeRefusesByName() {
        assertTrue(refused(chessboardYaml().replace("image_width: 1920\n", ""))
            .contains("image_width"))
    }

    /**
     * The fraction-of-width focal scaling is only known valid if DJI *scales* between stream
     * sizes, and nobody has checked whether it scales or crops — the prior yaml's own closing
     * argument for why there is no 720p variant. So another geometry refuses rather than
     * rescales.
     */
    @Test
    fun aForeignGeometryRefusesRatherThanRescales() {
        val why = refused(chessboardYaml(width = 1280, height = 720))
        assertTrue(why, why.contains("1280x720"))
        assertTrue(why, why.contains("1920x1080"))
    }

    /** Eight values is not a 3×3 — and fx from a broken matrix must not apply with default cx. */
    @Test
    fun aShortMatrixRefusesWhole() {
        val text = chessboardYaml().lines().filterNot { it.trim() == "- 1.0" }
            .joinToString("\n")
        assertTrue(refused(text).contains("8 values"))
    }

    @Test
    fun aNonFiniteMatrixValueRefuses() {
        assertTrue(refused(chessboardYaml().replace("- 1450.2", "- banana"))
            .contains("non-finite"))
    }

    @Test
    fun aSkewedMatrixRefuses() {
        assertTrue(refused(chessboardYaml(skew = 0.5)).contains("pinhole"))
    }

    /**
     * **The known-bad focal lengths refuse**: the 1000 px placeholder that is 31 % low
     * (`drone_basic.py`'s, the number this file exists to displace) and DJI's spec-derived
     * 1266 px that is 13 % low. The band is ±10 % around a fit that reproduced to 1.2 % — an
     * honest result outside it contradicts the fit and deserves a human.
     */
    @Test
    fun anImplausibleFocalLengthRefuses() {
        assertTrue(refused(chessboardYaml(fx = 1000.0, fy = 1000.0)).contains("1310"))
        assertTrue(refused(chessboardYaml(fx = 1266.0, fy = 1266.0)).contains("focal"))
        assertTrue(refused(chessboardYaml(fx = 1650.0, fy = 1650.0)).contains("focal"))
    }

    /** fx/fy is measured equal to 0.85 %; a 10 % anisotropy is not this camera. */
    @Test
    fun aWildAspectRatioRefuses() {
        assertTrue(refused(chessboardYaml(fx = 1457.0, fy = 1320.0)).contains("aspect"))
    }

    /** The worst structural excursion ever fitted is 128 px; 200+ px off centre is not believable. */
    @Test
    fun aFarOffPrincipalPointRefuses() {
        assertTrue(refused(chessboardYaml(cx = 1200.0)).contains("principal point"))
        assertTrue(refused(chessboardYaml(cy = 320.0)).contains("principal point"))
    }

    /**
     * **Distortion this pipeline would have to ignore refuses the file** — applying the matrix
     * while dropping a model big enough to matter would be applying half the file, the exact
     * thing the intake promises never to do. The bound is the plumb-line measurement's own CI
     * on this stream (k1 ∈ [−0.031, +0.054]: DJI already rectifies).
     */
    @Test
    fun aRealDistortionModelRefusesRatherThanBeingSilentlyDropped() {
        val why = refused(chessboardYaml(k1 = 0.2))
        assertTrue(why, why.contains("distortion"))
        assertTrue(why, why.contains("no undistortion"))
        // At the measured-zero scale it applies — a chessboard's small residual is a zero here.
        assertTrue(CameraCalibration.parse(chessboardYaml(k1 = 0.012), "f.yaml")
            is CameraCalibration.Loaded.Applied)
    }

    /** Every refusal names the file, so the operator knows which drop-in was disbelieved. */
    @Test
    fun everyRefusalNamesTheFile() {
        for (bad in listOf(
            "",
            "image_width: 1920\nimage_height: 1080\n",
            chessboardYaml(fx = 1000.0, fy = 1000.0),
            chessboardYaml(k1 = 0.9),
        )) {
            assertTrue(refused(bad).contains("camera_calibration.yaml"))
        }
    }
}
