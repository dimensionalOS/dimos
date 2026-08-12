package com.dimensional.mini4pro.vision

/**
 * **The one owner of what this project believes about the camera** — focal length, principal
 * point, and the measured nadir pixel — with its provenance on the same object.
 *
 * It exists because the camera's numbers used to live in two places that could drift: the JNI
 * pose solve was handed `TagWorld.focalPx` and a principal point spelled `width / 2.0` inline in
 * `AprilTagDetector.detect`, while `TagWorld`'s projection read its own constants. Two spellings
 * of one camera is the two-places-for-one-property failure this project keeps deleting
 * (`gimbal/PitchBelief` is the same move for camera *pointing*): the solve and the bearing math
 * must not be able to disagree about the instrument that produced both. Now `Bridge` resolves
 * one instance per session, `AprilTagDetector` feeds it to the solver and `TagWorld` projects
 * with it, and the session's flight record says which one it was (`camera_calibration` event).
 *
 * ## [ASSUMED] — the prior this flies on until Ivan's chessboard session
 *
 * Every number in the default is inherited from `calib/mini4pro_gimbal_1080.yaml` and the
 * measurement documents it cites, and the file's own headline holds: **this is a prior, not a
 * calibration.**
 *
 *  - **fx = fy = 1457.0 px at 1920 wide — FITTED, ±1.2 %.** Two independent flights fitted the
 *    focal length against recorded altitude (1465.9 and 1448.3 px); the mean is used. DJI's
 *    published 82.1° diagonal implies 1266 px and is 13 % low for this stream — the spec is
 *    presumably the full sensor and the video is cropped. fx = fy is *bounded*, not assumed:
 *    1203 detections put fy within 0.85 % of fx.
 *    `docs/measurements/2026-07-27-tag-detection-rate.md` §3.
 *  - **cx, cy = the image centre — ASSUMED, the weakest number here.** The two flights' own
 *    fits put the optical axis 46 and 128 px off centre and disagree with each other by 82 px,
 *    twenty times their formal errors — structurally inseparable from gimbal pointing bias in
 *    a nadir descent. This is the term behind the solve's measured ~3° systematic tilt floor
 *    (`2026-07-28-pose-solve-stability.md` §5), ≈5 cm of lateral error at 1 m. It is the one
 *    number the chessboard exists to produce.
 *  - **nadir pixel = (970.7, 615.2) at 1920×1080 — MEASURED**, ±2.3/±4.0 px, from three yaw
 *    turns over a fixed ground point (`2026-07-28-nadir-image-point.md`). It is the *sum* of
 *    principal point and gimbal nadir error and is exactly the right reference for plumb-line
 *    arithmetic, and exactly the wrong one for an optical-frame projection — both consumers
 *    read it from here so the distinction is made once ([nadirPointX] vs [cxPx]).
 *
 * ## The intake: a chessboard result becomes a data drop-in
 *
 * [load] reads **[FILE_NAME] in the app's files directory** (the flight records' own home,
 * `/storage/emulated/0/Android/data/com.dimensional.mini4pro/files/` — one `adb push` away),
 * in the ROS `camera_info` YAML schema that `dimos cameracalibrate` writes and that
 * `calib/mini4pro_gimbal_1080.yaml` already carries. No file is the normal case and means
 * [ASSUMED]. A file that parses and passes every sanity gate replaces the intrinsics **whole**;
 * a file that fails any gate is **refused by name and applied not at all** — [Loaded.Refused]
 * carries the sentence, the session flies on [ASSUMED], and the record says so. There is no
 * partial application on purpose: fx from a file and cy from the prior is a camera nobody has
 * ever measured.
 *
 * The gates are sanity bounds from this project's own measurements, not guesses — each one
 * refuses a file that contradicts something already measured, which is a finding a human should
 * see rather than a value software should silently fly:
 *
 *  - **1920×1080 only** ([EXPECTED_WIDTH]): fx is carried as a fraction of width, valid only if
 *    the pipeline *scales* between resolutions. Nobody has checked whether DJI scales or crops
 *    (the prior yaml's own closing argument), so a calibration at another geometry is refused
 *    rather than rescaled.
 *  - **fx and fy within [[FOCAL_MIN_PX], [FOCAL_MAX_PX]]**: ±10 % around the two-flight fit
 *    that reproduced to 1.2 %. Refuses the known 1000 px placeholder (31 % low) and DJI's
 *    spec-derived 1266 px (13 % low); an honest chessboard landing outside this band would
 *    mean the fit was badly wrong, which deserves a human, not an auto-apply.
 *  - **|fx/fy − 1| ≤ [MAX_ASPECT_DEVIATION]**: the aspect is *measured* equal to 0.85 %; 5 % is
 *    six times outside anything this stream has shown.
 *  - **principal point within [MAX_PRINCIPAL_OFFSET_PX] of the centre**: the worst structural
 *    disagreement ever fitted is 128 px; 200 px (≈7.8°) is past anything the evidence allows.
 *  - **every distortion coefficient within ±[MAX_IGNORED_DISTORTION]**: this pipeline applies
 *    **no undistortion**, so a model large enough to matter must not be silently dropped —
 *    that would be half-applying the file. The bound is the plumb-line measurement's own 95 %
 *    CI on k1 ([−0.031, +0.054], corner shift +16 px CI [−19, +34] on a 1101 px radius —
 *    DJI evidently rectifies the stream); a chessboard result inside it is a measured zero,
 *    outside it a contradiction to look at.
 *
 * A parsed file is [measured] `= true` unless it carries the prior's own
 * `provenance: calibrated: false` marker — so pushing the repo's prior yaml to the phone does
 * not launder an assumption into a measurement.
 *
 * ## What a real calibration does and does not change
 *
 * It moves the solve's and the optical projection's intrinsics, which removes the assumed-
 * principal-point half of the ~3° tilt floor. It does **not** move the nadir pixel — that is a
 * separate measurement of *pointing + optics summed*, still the right reference for
 * [TagWorld.fix]'s plumb-line arithmetic — and it does not, by itself, flip any `metric` flag
 * on sightings: `TagSighting.Sighting.metric` describes the camera-frame pose those sightings
 * carry, and re-labelling history's convention is not this class's call.
 */
data class CameraCalibration(
    /** Focal length, pixels, for a 1920-wide frame. See the class KDoc for the fit. */
    val fxAt1920: Double,
    /** Focal length, y, same scale. Equal to [fxAt1920] in [ASSUMED]; bounded within 0.85 %. */
    val fyAt1920: Double,
    /** Principal point x at 1920 wide. **Assumed 960.0** until the chessboard says otherwise. */
    val cxAt1920: Double,
    /** Principal point y at 1920×1080. **Assumed 540.0.** */
    val cyAt1920: Double,
    /** The pixel straight-down appears at, x, at 1920 wide. **Measured**, ±2.3 px. */
    val nadirXAt1920: Double,
    /** The pixel straight-down appears at, y, at 1920×1080. **Measured**, ±4.0 px. */
    val nadirYAt1920: Double,
    /**
     * **False while fx/cx/cy are the fitted-and-assumed prior; true only for a parsed
     * calibration file that did not disclaim itself.** Recorded with every session so a reader
     * of any flight record knows which camera model its numbers rest on.
     */
    val measured: Boolean,
    /** Where the numbers came from, as a sentence for the record's `camera_calibration` event. */
    val source: String,
) {

    /**
     * The focal length for a frame [width] px across. Scales linearly with width because the
     * stream is the same crop of the same sensor at every encoder resolution — a detection at
     * 960 has half the pixels per radian, and the *bearing* it implies is identical, which is
     * the property the scaling preserves.
     */
    fun fxPx(width: Int): Double = fxAt1920 * (width / 1920.0)

    /** See [fxPx]. */
    fun fyPx(width: Int): Double = fyAt1920 * (width / 1920.0)

    /**
     * The principal point, x, for a frame [width] across — the image centre plus the calibrated
     * offset scaled **with width**, the angular-offset convention every per-width number here
     * follows (see [nadirPointY] for why the y offset must not scale with height).
     */
    fun cxPx(width: Int): Double = width / 2.0 + (cxAt1920 - 960.0) * (width / 1920.0)

    /** The principal point, y. Offset scales with **width** — see [nadirPointY]. */
    fun cyPx(width: Int, height: Int): Double =
        height / 2.0 + (cyAt1920 - 540.0) * (width / 1920.0)

    /**
     * The pixel a point directly beneath the aircraft appears at, x, for a frame [width] across.
     *
     * **This is not the principal point and must not be used as one.** Aircraft yaw rotates the
     * camera about gravity's vertical rather than about the optical axis, so the yaw-circle
     * measurement gives `principal + f·tan(gimbal nadir error)` — summed, inseparable without a
     * chessboard, and 2.99° from the assumed centre. It is exactly right for [TagWorld.fix],
     * which scales rays it needs pointing *straight down*, and exactly wrong for
     * [TagWorld.cameraFrame], which owes its caller an optical-frame direction. Both read this
     * object, each the field it means.
     */
    fun nadirPointX(width: Int): Double =
        width / 2.0 + (nadirXAt1920 - 960.0) * (width / 1920.0)

    /**
     * The nadir pixel, y. The offset scales with **width on both axes**, not with each axis's
     * own size, because the fixed thing is the *angle* and pixels-per-radian is [fxPx], which
     * scales with width. Scaled by height it would drift the moment the stream left 16:9 —
     * the mutation `TagWorldTest` caught surviving in 2026-07-28's campaign.
     */
    fun nadirPointY(width: Int, height: Int): Double =
        height / 2.0 + (nadirYAt1920 - 540.0) * (width / 1920.0)

    /** The result of asking for a calibration: applied whole, or refused by name. Never both. */
    sealed class Loaded {
        /** [calibration] is in force — the file's, or [ASSUMED] when there was no file. */
        data class Applied(val calibration: CameraCalibration) : Loaded()

        /**
         * The file exists and is not believed: [why] names the file and the first gate it
         * failed. The caller flies on [ASSUMED] and records this sentence — a malformed
         * calibration must be *loud*, because the operator who pushed it believes it applied.
         */
        data class Refused(val why: String) : Loaded()
    }

    companion object {

        /**
         * The prior, exactly `calib/mini4pro_gimbal_1080.yaml`'s numbers plus the measured
         * nadir pixel. Every consumer's default, so a session with no calibration file behaves
         * byte-identically to every flight before this seam existed.
         */
        val ASSUMED = CameraCalibration(
            fxAt1920 = 1457.0,
            fyAt1920 = 1457.0,
            cxAt1920 = 960.0,
            cyAt1920 = 540.0,
            nadirXAt1920 = 970.7,
            nadirYAt1920 = 615.2,
            measured = false,
            source = "assumed prior: two-flight focal fit, image-centre principal point " +
                "(calib/mini4pro_gimbal_1080.yaml), measured nadir pixel",
        )

        /** The drop-in's name in the app's files directory. One `adb push` away, like the logs. */
        const val FILE_NAME = "camera_calibration.yaml"

        /** The only stream geometry the width-fraction scaling is known valid for. */
        const val EXPECTED_WIDTH = 1920
        const val EXPECTED_HEIGHT = 1080

        /** ±10 % around the 1457 px fit that reproduced to 1.2 %. Catches the 1000 px placeholder. */
        const val FOCAL_MIN_PX = 1310.0
        const val FOCAL_MAX_PX = 1600.0

        /** fx/fy measured equal within 0.85 %; 5 % is six times outside the evidence. */
        const val MAX_ASPECT_DEVIATION = 0.05

        /** Worst fitted principal-point excursion is 128 px; 200 px (≈7.8°) is past belief. */
        const val MAX_PRINCIPAL_OFFSET_PX = 200.0

        /**
         * The largest distortion coefficient this undistortion-free pipeline may shrug at —
         * the plumb-line k1 CI's own magnitude (`calib/mini4pro_gimbal_1080.yaml`: k1 95 % CI
         * [−0.031, +0.054], corner shift CI [−19, +34] px). Bigger means the model matters,
         * and ignoring a model that matters would be applying half the file.
         */
        const val MAX_IGNORED_DISTORTION = 0.06

        /**
         * Read [FILE_NAME] under [dir], or [ASSUMED] when it is not there. Absence is the
         * normal, quiet case; an unreadable or refused file is loud — see [Loaded.Refused].
         */
        fun load(dir: java.io.File): Loaded {
            val file = java.io.File(dir, FILE_NAME)
            if (!file.exists()) return Loaded.Applied(ASSUMED)
            val text = try {
                file.readText()
            } catch (e: Exception) {
                return Loaded.Refused("calibration refused: $FILE_NAME unreadable (${e.message})")
            }
            return parse(text, file.name)
        }

        /**
         * One calibration from one `camera_info` YAML, or a refusal naming the first fault.
         *
         * The parser reads exactly the subset `dimos cameracalibrate` writes — scalar
         * `image_width`/`image_height`, a `camera_matrix` block with nine `- value` items,
         * optional `distortion_coefficients`, optional `provenance: calibrated:` — and is
         * strict about it. Every fault refuses the **whole** file: parsing continues past
         * nothing, repairs nothing, defaults nothing that the gates below check.
         */
        fun parse(text: String, sourceName: String): Loaded {
            fun refuse(why: String) = Loaded.Refused("calibration refused: $sourceName — $why")

            val lines = text.replace("\r", "").lines().map { line ->
                val hash = line.indexOf('#')
                if (hash >= 0) line.substring(0, hash) else line
            }

            val width = scalarLong(lines, "image_width")
                ?: return refuse("no image_width")
            val height = scalarLong(lines, "image_height")
                ?: return refuse("no image_height")
            if (width != EXPECTED_WIDTH.toLong() || height != EXPECTED_HEIGHT.toLong()) {
                return refuse(
                    "geometry ${width}x$height — this pipeline detects at " +
                        "${EXPECTED_WIDTH}x$EXPECTED_HEIGHT and whether DJI scales or crops " +
                        "other streams is unverified, so no rescaling is attempted"
                )
            }

            val m = blockData(lines, "camera_matrix")
                ?: return refuse("no camera_matrix data block")
            if (m.size != 9) return refuse("camera_matrix has ${m.size} values, not 9")
            if (m.any { !it.isFinite() }) return refuse("camera_matrix carries a non-finite value")
            val fx = m[0]
            val fy = m[4]
            val cx = m[2]
            val cy = m[5]
            // The zero pattern of a pinhole K. A matrix that is not one is not a camera_info.
            if (m[1] != 0.0 || m[3] != 0.0 || m[6] != 0.0 || m[7] != 0.0 || m[8] != 1.0) {
                return refuse("camera_matrix is not an unskewed pinhole K (off-slots not 0/1)")
            }
            if (fx < FOCAL_MIN_PX || fx > FOCAL_MAX_PX || fy < FOCAL_MIN_PX || fy > FOCAL_MAX_PX) {
                return refuse(
                    "focal $fx/$fy px outside [$FOCAL_MIN_PX, $FOCAL_MAX_PX] — ±10 % around " +
                        "the two-flight fit; a genuine result out here contradicts the fit " +
                        "and deserves a human before it flies"
                )
            }
            if (kotlin.math.abs(fx / fy - 1.0) > MAX_ASPECT_DEVIATION) {
                return refuse("fx/fy aspect ${fx / fy} — measured equal within 0.85 %")
            }
            if (kotlin.math.abs(cx - 960.0) > MAX_PRINCIPAL_OFFSET_PX ||
                kotlin.math.abs(cy - 540.0) > MAX_PRINCIPAL_OFFSET_PX
            ) {
                return refuse(
                    "principal point ($cx, $cy) over $MAX_PRINCIPAL_OFFSET_PX px off centre " +
                        "— the worst structural excursion ever fitted is 128 px"
                )
            }

            val d = blockData(lines, "distortion_coefficients")
            if (d != null && d.any { !it.isFinite() || kotlin.math.abs(it) > MAX_IGNORED_DISTORTION }) {
                return refuse(
                    "distortion ${d.joinToString()} exceeds ±$MAX_IGNORED_DISTORTION — this " +
                        "pipeline applies no undistortion, and a model big enough to matter " +
                        "must not be silently dropped"
                )
            }

            // The prior's own escape hatch: a file that says `calibrated: false` about itself
            // is applied but never promoted to a measurement.
            val disclaimed = scalarString(lines, "calibrated") == "false"

            return Loaded.Applied(
                CameraCalibration(
                    fxAt1920 = fx,
                    fyAt1920 = fy,
                    cxAt1920 = cx,
                    cyAt1920 = cy,
                    // The nadir pixel is a separate measurement (pointing + optics summed) and
                    // a chessboard does not re-measure it; the prior's stands.
                    nadirXAt1920 = ASSUMED.nadirXAt1920,
                    nadirYAt1920 = ASSUMED.nadirYAt1920,
                    measured = !disclaimed,
                    source = if (disclaimed) "$sourceName (file marked calibrated: false)"
                    else sourceName,
                )
            )
        }

        /** The first `key: <integer>` scalar anywhere, or null. */
        private fun scalarLong(lines: List<String>, key: String): Long? =
            scalarString(lines, key)?.toLongOrNull()

        /** The first `key: <token>` scalar anywhere, or null. */
        private fun scalarString(lines: List<String>, key: String): String? {
            for (line in lines) {
                val t = line.trim()
                if (t.startsWith("$key:")) {
                    val v = t.removePrefix("$key:").trim()
                    if (v.isNotEmpty()) return v
                }
            }
            return null
        }

        /**
         * The `- value` items of the `data:` block inside the section named [section], or null
         * when the section or its data block is absent. Values that do not parse as numbers
         * come back NaN — a *present, refusable* fault, deliberately not an absence.
         */
        private fun blockData(lines: List<String>, section: String): List<Double>? {
            var inSection = false
            var inData = false
            val out = ArrayList<Double>()
            var found = false
            for (line in lines) {
                val t = line.trim()
                if (t.isEmpty()) continue
                val topLevel = line.isNotEmpty() && !line[0].isWhitespace()
                if (topLevel) {
                    if (inSection && found) break
                    inSection = t.startsWith("$section:")
                    inData = false
                    continue
                }
                if (!inSection) continue
                if (t.startsWith("data:")) {
                    inData = true
                    found = true
                    continue
                }
                if (inData) {
                    if (t.startsWith("- ")) {
                        out.add(t.removePrefix("- ").trim().toDoubleOrNull() ?: Double.NaN)
                    } else {
                        inData = false
                    }
                }
            }
            return if (found) out else null
        }
    }
}
