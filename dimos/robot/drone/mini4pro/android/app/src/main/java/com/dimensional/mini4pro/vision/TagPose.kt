package com.dimensional.mini4pro.vision

import kotlin.math.sqrt

/**
 * **The four corners of one detected tag, in pixels** — image x right, y **down**, the same
 * convention every other pixel in this package uses.
 *
 * The order is apriltag's own (`apriltag_detection_t.p`): counter-clockwise around the quad,
 * starting at the corner the library maps to the tag frame's (−1, +1). Preserved rather than
 * normalised, because the order *is* information — it is what makes the pose solve's sign
 * conventions reproducible offline from the flight record, which `tools/tagcorners` today can
 * only approximate by re-detecting.
 */
data class TagCorners(
    val x0: Double, val y0: Double,
    val x1: Double, val y1: Double,
    val x2: Double, val y2: Double,
    val x3: Double, val y3: Double,
)

/**
 * **One tag's pose as apriltag's own estimator produced it, raw and ungated.**
 *
 * This is evidence, not belief: it is carried on every sighting and every flight-record line
 * *whether or not it deserves publishing*, and the decision to believe it is [TagPose.trusted]'s,
 * taken at the one place a solved orientation can reach a consumer. Recording only the solves
 * that passed would make the gates unmeasurable from flight data — the same argument that keeps
 * `decision_margin` recorded and unthresholded.
 *
 * ## The frames, exactly
 *
 * [qx]..[qw] rotate the **tag frame** into `drone/camera_optical`. Both conventions are
 * apriltag's (`estimate_tag_pose_orthogonal_iteration`): camera x right, y down, z out the lens
 * — which *is* the optical frame — and tag x right, y down **on the printed face**, z into the
 * tag. A tag lying flat under a nadir camera therefore solves to a rotation about z only (its
 * z points away from the camera, along +z optical), which is what the yaw-over-tag dataset shows.
 *
 * [tx],[ty],[tz] are the tag centre in metres in the same camera frame. Note this is the
 * estimator's own translation — a *second* opinion about where the tag is, beside the
 * apparent-size range the sighting's `x, y, z` already carry. The published
 * `results[0].pose.position` deliberately stays the latter (`docs/zenoh-topics.md`); this one
 * is published only inside the solved `bbox`.
 *
 * ## The two errors
 *
 * [err1] is the best solution's object-space error, [err2] the second local minimum's —
 * planar PnP is two-fold ambiguous and the loser's error is the only evidence of how close the
 * flip was. `err1 <= err2` always (the JNI shim sorts), and **[err2] is `+Infinity` when
 * Schweighofer–Pinz found no second minimum**, i.e. the solve was unambiguous. That infinity is
 * a real value with a real meaning and it survives the flight record (`"Infinity"`, the format's
 * own non-finite encoding).
 *
 * ## The caveat that never leaves this type
 *
 * The solve rests on the fitted focal length (±1.2 %) and a principal point **assumed** to be
 * the image centre — unmeasured, 2.99° from the measured nadir pixel, and that ambiguity bleeds
 * into this orientation two measured ways
 * (`docs/measurements/2026-07-28-pose-solve-stability.md` §5): a level nadir hover solves to a
 * **median 3.0° of tilt** that cannot be attributed between optics and gimbal pointing — a
 * systematic floor no gate removes — and re-solving identical detections with the principal
 * point moved to the measured nadir pixel shifts the orientation by 0.6° median, 1.1° p90 at
 * the publishing threshold. `metric` stays false on every sighting that carries one of these.
 */
data class TagPoseSolve(
    val qx: Double,
    val qy: Double,
    val qz: Double,
    val qw: Double,
    val tx: Double,
    val ty: Double,
    val tz: Double,
    val err1: Double,
    val err2: Double,
    /** The printed marker's black square, side to side, metres — the solve's scale input. */
    val tagSizeM: Double,
)

/**
 * **Whether a pose solve deserves belief, and the arithmetic that makes one.** Pure, and every
 * threshold in it measured — see `docs/measurements/2026-07-28-pose-solve-stability.md`, which
 * ran the *same vendored estimator* over the yaw-over-tag flight's 4 090 offline detections.
 *
 * The gates exist because the estimator is confidently wrong in two specific, measured ways:
 * below a pixel size its answer is scatter, and near ambiguity-ratio 1 its answer flips frame to
 * frame between the two planar-PnP minima. Neither failure announces itself in the solve's own
 * numbers, which is why the gate is a separate, tested object rather than a comment.
 */
object TagPose {

    /**
     * **The pixel-size gate: no orientation is published from a tag smaller than this.**
     *
     * Measured, not chosen — the same vendored estimator run over the yaw-over-tag flight's
     * 3 800 offline detections, `docs/measurements/2026-07-28-pose-solve-stability.md` §3.
     * **Tilt is the axis that fails**; yaw about the optical axis stays inside 1.3° at every
     * size measured, down to 20 px. Against the climb segment's own ≥100 px baseline, the tilt
     * error's p90 by pixel bin:
     *
     * | px | 85–120 | 70–85 | 60–70 | 55–60 | 20–50 | <20 |
     * |---|---|---|---|---|---|---|
     * | tilt p90 | 2.2° | 3.6° | 5.5° | 7.9° | 5.7–7.7°, max >10° | flips: 82 % ratio>0.5 |
     *
     * 60 px is where the p90 last sits near the ~3° systematic tilt floor the solve carries
     * anyway (the unmeasured principal point / gimbal-pointing ambiguity, §5): below it the
     * *solver* becomes the dominant error and doubles within one bin, and under 50 px the
     * ambiguity flip fraction reaches 13–31 %. Below the gate the angular error is garbage
     * however confident the solver's own residual sounds — err1 is 1e-7-ish throughout.
     *
     * At 60 px this 75 mm tag is ~1.8 m away, so with the shipped marker the orientation
     * publishes only through the last two metres of a descent — "appropriate height" is a pixel
     * count, and a larger landing board raises the height without touching this constant.
     */
    const val MIN_SOLVE_PIXELS = 60.0

    /**
     * **The ambiguity gate: the two minima must disagree by at least a factor of two.**
     *
     * Planar PnP has two solutions and [TagPoseSolve.err1]/[TagPoseSolve.err2] near 1 means the
     * solver picked between them on noise — the orientation then flips frame to frame while
     * every other number stays plausible. Measured on the same dataset (§4): **every** tilt
     * outlier worse than 10° above the pixel gate carried a ratio > 0.5 (10 of 10), while only
     * 3.1 % of well-behaved solves did; tightening to 0.3 refuses 11 % of good solves and buys
     * 0.1° of p90. So 0.5: it removes the flip population and costs 4.3 % of the hover
     * window's publishable solves.
     *
     * `err2 = +Infinity` — no second minimum found, true of 59 % of the measured solves — is a
     * ratio of 0 and passes: that is the *unambiguous* case, not a missing value.
     */
    const val MAX_AMBIGUITY_RATIO = 0.5

    /**
     * `err1/err2` as the gate consumes it: 0 when unambiguous (`err2` infinite), 1 when
     * degenerate (both zero — a solver output no evidence can distinguish), NaN only when a
     * field is NaN, which [trusted] refuses separately as a partial solve.
     */
    fun ambiguityRatio(err1: Double, err2: Double): Double = when {
        err1.isNaN() || err2.isNaN() -> Double.NaN
        err2 == Double.POSITIVE_INFINITY -> 0.0
        err2 <= 0.0 -> 1.0
        else -> err1 / err2
    }

    /**
     * **The one place a solve becomes believable: returns the solve when every gate passes,
     * null otherwise.** Null means the message stays exactly what it was before pose solving
     * existed — NaN orientation, refused bbox — never a partial claim.
     *
     * The checks, in order, and why each refuses rather than repairs:
     *
     *  1. **No solve is no solve.** Legacy sightings and disabled solving both land here.
     *  2. **A partial solve is refused entire.** Any non-finite quaternion or translation
     *     component, a NaN error, a non-positive tag size — one bad field poisons the claim,
     *     because a quaternion with three good components is not three-quarters of a rotation.
     *     `err2` alone may be infinite (that is a value, see [MAX_AMBIGUITY_RATIO]).
     *  3. **A tag behind or at the camera (`tz <= 0`) is geometry no real detection produces**,
     *     and publishing it would put the box on the wrong side of the lens.
     *  4. **The pixel-size gate** ([MIN_SOLVE_PIXELS]).
     *  5. **The ambiguity gate** ([MAX_AMBIGUITY_RATIO]).
     */
    fun trusted(pixelSize: Double, solve: TagPoseSolve?): TagPoseSolve? {
        if (solve == null) return null
        val finiteFields = listOf(
            solve.qx, solve.qy, solve.qz, solve.qw,
            solve.tx, solve.ty, solve.tz,
        )
        if (finiteFields.any { !it.isFinite() }) return null
        if (!solve.err1.isFinite() || solve.err1 < 0.0) return null
        if (solve.err2.isNaN()) return null
        if (!solve.tagSizeM.isFinite() || solve.tagSizeM <= 0.0) return null
        if (solve.tz <= 0.0) return null
        if (!pixelSize.isFinite() || pixelSize < MIN_SOLVE_PIXELS) return null
        if (ambiguityRatio(solve.err1, solve.err2) > MAX_AMBIGUITY_RATIO) return null
        return solve
    }

    /**
     * A row-major 3×3 rotation matrix as an `(x, y, z, w)` quaternion, or null when any element
     * is non-finite — the JNI boundary's NaN-when-unsolved convention arrives here.
     *
     * Shepperd's method: pick the largest of the four squared components before any square
     * root, so the division is always by the best-conditioned term and no near-zero trace can
     * blow up the other three. The result is normalised (the input is an SVD-orthogonalised
     * rotation, but "should be unit" is not "is unit").
     *
     * Convention pinned by `TagPoseTest`: `w` is non-negative — the two antipodal quaternions
     * are one rotation, and the record and the bus must not carry two spellings of it.
     */
    fun quaternionOf(r: DoubleArray): DoubleArray? {
        if (r.size != 9 || r.any { !it.isFinite() }) return null
        val t = r[0] + r[4] + r[8]
        var x: Double
        var y: Double
        var z: Double
        var w: Double
        if (t >= r[0] && t >= r[4] && t >= r[8]) {
            w = sqrt(1.0 + t) / 2.0
            val s = 4.0 * w
            x = (r[7] - r[5]) / s
            y = (r[2] - r[6]) / s
            z = (r[3] - r[1]) / s
        } else if (r[0] >= r[4] && r[0] >= r[8]) {
            x = sqrt(1.0 + r[0] - r[4] - r[8]) / 2.0
            val s = 4.0 * x
            w = (r[7] - r[5]) / s
            y = (r[1] + r[3]) / s
            z = (r[2] + r[6]) / s
        } else if (r[4] >= r[8]) {
            y = sqrt(1.0 - r[0] + r[4] - r[8]) / 2.0
            val s = 4.0 * y
            w = (r[2] - r[6]) / s
            x = (r[1] + r[3]) / s
            z = (r[5] + r[7]) / s
        } else {
            z = sqrt(1.0 - r[0] - r[4] + r[8]) / 2.0
            val s = 4.0 * z
            w = (r[3] - r[1]) / s
            x = (r[2] + r[6]) / s
            y = (r[5] + r[7]) / s
        }
        val norm = sqrt(x * x + y * y + z * z + w * w)
        if (!norm.isFinite() || norm == 0.0) return null
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        if (w < 0.0) {
            x = -x
            y = -y
            z = -z
            w = -w
        }
        return doubleArrayOf(x, y, z, w)
    }
}
