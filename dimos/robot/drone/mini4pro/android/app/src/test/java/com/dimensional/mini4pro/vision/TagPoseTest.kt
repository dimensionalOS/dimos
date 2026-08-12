package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * **The gates that decide whether a solved orientation deserves belief, and the quaternion
 * arithmetic under them.** `TagPose.trusted` is the only door a solve can pass on its way to
 * the bus, so this file is where "the gates cannot be bypassed" is enforced test by test:
 * every way a solve can be unworthy — absent, partial, too small, ambiguous, behind the camera
 * — must come back null, because null is what degrades the message to the pre-solve contract.
 *
 * The thresholds themselves are **measured**, not designed
 * (`docs/measurements/2026-07-28-pose-solve-stability.md`), and the boundary tests below pin
 * the exact comparison direction: a gate that silently became exclusive where it was inclusive
 * would move the working envelope by one detection nobody could point to.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * Each row applied alone to `src/main`, `app/build/test-results/testDebugUnitTest` deleted
 * first, the **whole suite** run (2 260 tests), reverted; the count is failing tests.
 * Harness: session scratchpad `mutate_pose.py`, the `tmp/mutate.py` protocol.
 *
 * | mutation | failures |
 * |---|---|
 * | `MIN_SOLVE_PIXELS` lowered to 0 (gate open) | 4 |
 * | pixel-gate check dropped from `trusted` | 3 |
 * | pixel gate boundary flipped (`<=` for `<` at 60 px) | 1 |
 * | ambiguity gate dropped from `trusted` | 3 |
 * | ambiguity gate inverted (`<` for `>`) | 9 |
 * | `MAX_AMBIGUITY_RATIO` raised to 1e9 (gate open) | 4 |
 * | partial-solve finiteness check dropped | 3 |
 * | `err1` validity check dropped | 2 |
 * | `err2` NaN check dropped | 2 |
 * | `tz <= 0` (tag behind the camera) check dropped | 2 |
 * | infinite `err2` treated as fully ambiguous | 2 |
 * | `quaternionOf` stops normalising the sign (`w < 0` kept) | **0 → 1** |
 *
 * **The last row survived the first pass and that is why its test looks the way it does**:
 * below 180° every Shepperd branch yields a non-negative `w` by construction, so the original
 * angles (0–180°) could not see the normalisation at all. The 181° and 250° cases were added,
 * the mutation re-run alone, and it now kills — the same shape of hole `TagWorldTest`'s table
 * records, closed the same way. Nothing else survived and nothing was NO-RUN.
 *
 * The gates' *encoder-side* rows (bypass, partial bypass, the solved box and the id marker)
 * are `ZenohDetectionTest`'s table; the record-side rows are `RecordedTagSinkTest`'s.
 */
class TagPoseTest {

    private fun solve(
        qx: Double = 0.0, qy: Double = 0.0, qz: Double = 0.0, qw: Double = 1.0,
        tx: Double = 0.05, ty: Double = -0.02, tz: Double = 1.5,
        err1: Double = 1.5e-7, err2: Double = 6.5e-7,
        tagSizeM: Double = 0.075,
    ) = TagPoseSolve(qx, qy, qz, qw, tx, ty, tz, err1, err2, tagSizeM)

    // ─────────────────────────────────────────────────────────── the gates

    @Test
    fun aWholeSolveAboveBothGatesPassesUnchanged() {
        val s = solve()
        // The same instance, not a copy: the gate is a filter, never an editor.
        assertSame(s, TagPose.trusted(80.0, s))
    }

    @Test
    fun noSolveIsNoSolve() {
        assertNull(TagPose.trusted(500.0, null))
    }

    /**
     * **The pixel gate, at its measured value and boundary.** 60 px is where the offline tilt
     * error last sits near the ~3° systematic floor; the gate is inclusive at exactly 60
     * because the measurement binned `[60, 70)` as passing.
     */
    @Test
    fun thePixelGateRefusesBelowSixtyAndAdmitsAtIt() {
        assertEquals(60.0, TagPose.MIN_SOLVE_PIXELS, 0.0)
        assertNull(TagPose.trusted(59.999, solve()))
        assertNotNull(TagPose.trusted(60.0, solve()))
        // Deep below the gate — the descent's 8 m arming altitude puts the 75 mm tag at ~14 px,
        // and that solve is measured garbage (82 % ambiguity flips below 20 px).
        assertNull(TagPose.trusted(14.0, solve()))
        // A pixel size that is not a number is not a big tag.
        assertNull(TagPose.trusted(Double.NaN, solve()))
    }

    /** **The ambiguity gate, at its measured value and boundary.** */
    @Test
    fun theAmbiguityGateRefusesNearOneAndAdmitsAtTheBound() {
        assertEquals(0.5, TagPose.MAX_AMBIGUITY_RATIO, 0.0)
        // err1/err2 = 0.94 — the measured median among the >10° outliers. Flip country.
        assertNull(TagPose.trusted(80.0, solve(err1 = 9.4e-7, err2 = 1.0e-6)))
        // Exactly at the bound passes; just past it is refused.
        assertNotNull(TagPose.trusted(80.0, solve(err1 = 5.0e-7, err2 = 1.0e-6)))
        assertNull(TagPose.trusted(80.0, solve(err1 = 5.01e-7, err2 = 1.0e-6)))
    }

    /**
     * **An infinite `err2` is the unambiguous case and passes** — true of 59 % of the measured
     * solves. Treating it as missing data would refuse the *best* solves the estimator makes.
     */
    @Test
    fun noSecondMinimumIsUnambiguousNotMissing() {
        assertNotNull(TagPose.trusted(80.0, solve(err2 = Double.POSITIVE_INFINITY)))
        assertEquals(0.0, TagPose.ambiguityRatio(1.5e-7, Double.POSITIVE_INFINITY), 0.0)
    }

    /**
     * **A partial solve is refused entire, field by field.** A quaternion with a NaN component
     * is not three-quarters of a rotation, and a gate that let one through would put a partial
     * claim on a bus where nothing downstream can see which quarter is missing.
     */
    @Test
    fun aPartialSolveIsRefusedWhicheverFieldIsBad() {
        val bad = listOf(
            solve(qx = Double.NaN),
            solve(qy = Double.NaN),
            solve(qz = Double.POSITIVE_INFINITY),
            solve(qw = Double.NaN),
            solve(tx = Double.NaN),
            solve(ty = Double.NEGATIVE_INFINITY),
            solve(tz = Double.NaN),
            solve(err1 = Double.NaN),
            solve(err1 = Double.POSITIVE_INFINITY),
            solve(err1 = -1e-9),
            solve(err2 = Double.NaN),
            solve(tagSizeM = 0.0),
            solve(tagSizeM = -0.075),
            solve(tagSizeM = Double.NaN),
        )
        for (s in bad) assertNull("$s must be refused", TagPose.trusted(500.0, s))
    }

    /** A tag at or behind the camera is geometry no real detection produces. */
    @Test
    fun aTagBehindTheCameraIsRefused() {
        assertNull(TagPose.trusted(500.0, solve(tz = 0.0)))
        assertNull(TagPose.trusted(500.0, solve(tz = -1.5)))
    }

    @Test
    fun theRatioArithmeticIsPinnedAtItsEdges() {
        assertEquals(0.25, TagPose.ambiguityRatio(1.0, 4.0), 1e-12)
        // Both zero: a degenerate output no evidence can distinguish — fully ambiguous.
        assertEquals(1.0, TagPose.ambiguityRatio(0.0, 0.0), 0.0)
        assertTrue(TagPose.ambiguityRatio(Double.NaN, 4.0).isNaN())
        assertTrue(TagPose.ambiguityRatio(1.0, Double.NaN).isNaN())
    }

    // ─────────────────────────────────────────── the rotation-matrix arithmetic

    private fun rotZ(deg: Double): DoubleArray {
        val r = Math.toRadians(deg)
        return doubleArrayOf(cos(r), -sin(r), 0.0, sin(r), cos(r), 0.0, 0.0, 0.0, 1.0)
    }

    /** Applies quaternion (x,y,z,w) to vector v — the check that a quaternion means its matrix. */
    private fun rotate(q: DoubleArray, v: DoubleArray): DoubleArray {
        val (x, y, z, w) = q
        val t = doubleArrayOf(
            2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]),
        )
        return doubleArrayOf(
            v[0] + w * t[0] + y * t[2] - z * t[1],
            v[1] + w * t[1] + z * t[0] - x * t[2],
            v[2] + w * t[2] + x * t[1] - y * t[0],
        )
    }

    @Test
    fun theIdentityAndAPlainYawConvert() {
        val i = TagPose.quaternionOf(doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))!!
        assertEquals(0.0, i[0], 1e-15)
        assertEquals(0.0, i[1], 1e-15)
        assertEquals(0.0, i[2], 1e-15)
        assertEquals(1.0, i[3], 1e-15)

        val q = TagPose.quaternionOf(rotZ(90.0))!!
        assertEquals(0.0, q[0], 1e-12)
        assertEquals(0.0, q[1], 1e-12)
        assertEquals(sqrt(0.5), q[2], 1e-12)
        assertEquals(sqrt(0.5), q[3], 1e-12)
    }

    /**
     * Every branch of Shepperd's method, exercised by the rotation that forces it: 180° about
     * each axis zeroes the trace-dominant path and makes one diagonal element the pivot.
     */
    @Test
    fun theDegenerateRotationsTakeEveryBranchAndStayUnit() {
        val cases = mapOf(
            "x180" to doubleArrayOf(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0),
            "y180" to doubleArrayOf(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
            "z180" to doubleArrayOf(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
        )
        for ((name, r) in cases) {
            val q = TagPose.quaternionOf(r)!!
            val n = sqrt(q.sumOf { it * it })
            assertEquals("$name must be unit", 1.0, n, 1e-12)
            // The matrix and the quaternion must rotate a basis vector identically.
            for (axis in 0..2) {
                val v = DoubleArray(3) { if (it == axis) 1.0 else 0.0 }
                val byMatrix = DoubleArray(3) { row -> r[row * 3 + axis] }
                val byQuat = rotate(q, v)
                for (k in 0..2) assertEquals("$name axis $axis", byMatrix[k], byQuat[k], 1e-12)
            }
        }
    }

    /**
     * A rotation like the ones the aircraft actually produces — a large yaw with a small tilt,
     * the yaw-over-tag flight's shape — survives the round trip through the matrix.
     */
    @Test
    fun aRealisticTagRotationRoundTrips() {
        val yaw = Math.toRadians(117.0)
        val tilt = Math.toRadians(3.0)
        // Rz(yaw) * Rx(tilt), composed by hand.
        val cz = cos(yaw); val sz = sin(yaw); val cx = cos(tilt); val sx = sin(tilt)
        val r = doubleArrayOf(
            cz, -sz * cx, sz * sx,
            sz, cz * cx, -cz * sx,
            0.0, sx, cx,
        )
        val q = TagPose.quaternionOf(r)!!
        for (axis in 0..2) {
            val v = DoubleArray(3) { if (it == axis) 1.0 else 0.0 }
            val byQuat = rotate(q, v)
            for (k in 0..2) assertEquals(r[k * 3 + axis], byQuat[k], 1e-12)
        }
    }

    /**
     * **`w >= 0`, always.** The two antipodal quaternions are one rotation; the record and the
     * bus must not carry two spellings of it, or byte-level parity would depend on which side
     * of the sphere the arithmetic happened to land on.
     *
     * The angles past 180° are the load-bearing ones, and their absence was caught by a
     * mutation: below 180° every Shepperd branch already yields a non-negative `w` by
     * construction, so a test that stopped at 180° scored **zero kills** against dropping the
     * normalisation. At 181° and 250° the x-branch's natural `w` is negative and only the
     * normalisation flips it.
     */
    @Test
    fun theSignConventionIsWNonNegative() {
        // 180° about x lands exactly on w = 0; past it the un-normalised w goes negative.
        for (deg in listOf(0.0, 90.0, 179.0, 180.0, 181.0, 250.0, 359.0)) {
            val rad = Math.toRadians(deg)
            val r = doubleArrayOf(
                1.0, 0.0, 0.0,
                0.0, cos(rad), -sin(rad),
                0.0, sin(rad), cos(rad),
            )
            val q = TagPose.quaternionOf(r)!!
            assertTrue("w must be non-negative at $deg deg, got ${q[3]}", q[3] >= 0.0)
            // And the flipped quaternion still means the same rotation.
            for (axis in 0..2) {
                val v = DoubleArray(3) { if (it == axis) 1.0 else 0.0 }
                val byQuat = rotate(q, v)
                for (k in 0..2) assertEquals("$deg deg axis $axis", r[k * 3 + axis], byQuat[k], 1e-12)
            }
        }
    }

    /** The JNI boundary's NaN-when-unsolved arrives here; it must convert to "no rotation". */
    @Test
    fun aNonFiniteOrMisshapenMatrixIsNull() {
        val nan = DoubleArray(9) { Double.NaN }
        assertNull(TagPose.quaternionOf(nan))
        val oneBad = rotZ(45.0).also { it[4] = Double.NaN }
        assertNull(TagPose.quaternionOf(oneBad))
        assertNull(TagPose.quaternionOf(DoubleArray(4)))
        assertNull(TagPose.quaternionOf(DoubleArray(0)))
    }

    /** |Δ| between the two gate constants and the doc's numbers is zero — the doc IS the value. */
    @Test
    fun theThresholdsAreTheMeasuredOnes() {
        assertEquals(60.0, TagPose.MIN_SOLVE_PIXELS, 0.0)
        assertEquals(0.5, TagPose.MAX_AMBIGUITY_RATIO, 0.0)
        // And the working geometry they imply for the shipped 75 mm marker: ~1.8 m of range at
        // the gate, from the same focal constant the solve uses. If someone changes the marker
        // or the focal fit, this states what the envelope became.
        val rangeAtGate = TagWorld.focalPx(1920) * 0.075 / TagPose.MIN_SOLVE_PIXELS
        assertEquals(1.82, rangeAtGate, 0.01)
    }
}
