package com.dimensional.mini4pro.vision

/**
 * **The AprilTag reference C library**, vendored under `android/app/src/apriltag` and built by
 * CMake into `libapriltagjni.so` — 47 KB of arm64 in an APK that is otherwise 178 MB.
 *
 * It was chosen by measurement rather than by preference, against OpenCV's `ArucoDetector` on
 * byte-identical frames from two real flights
 * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md`): **1.67× cheaper per frame and a strict
 * superset of ArUco's detections**, holding 100 % per-frame detection to 7.0 m where ArUco reached
 * 3.0 m. There is no frame in 1978 that ArUco found and this did not.
 *
 * This class was `AprilTagProbe` in `src/androidTest` until 2026-07-27. It moved because a
 * measurement became a sensor; [detect] then grew each tag's centre, hamming distance and
 * decision margin, which a comparison did not need and a sighting cannot do without. Since
 * 2026-07-28 it also returns the four corners (always) and, when [tagSizeM] is set, the
 * library's own pose estimate with both ambiguity errors — raw, gated nowhere in this class;
 * see [TagPose.trusted] for the gates and their measurements.
 *
 * ## The parameters, and which of them are a choice
 *
 * [quadDecimate] is the one that decides everything. Upstream's default is **2.0** — the quad
 * search runs on a half-resolution image and the decode on the full one — and it is where nearly
 * all of the library's speed reputation comes from. **This project's default is 1.0**, deliberately
 * not upstream's, because 2.0 is 3.4× cheaper and takes the 100 % detection band from 7.0 m back
 * down to 4.5 m. Adopting the library at its defaults would have bought speed and quietly returned
 * the working height to roughly where OpenCV had it.
 *
 * [nthreads] is the knob that makes 10 Hz arguable at all. Measured, 1080p, `quad_decimate = 1`:
 *
 * | nthreads | ms/frame | cores |
 * |---|---|---|
 * | 1 | 108.8 | 0.73 |
 * | 2 | 86.4 | 1.68 |
 * | 4 | 55.9 | 2.43 |
 * | 8 | 38.3 | 2.88 |
 *
 * **Every one of those was measured on an idle phone replaying a file.** In flight the same cores
 * carry MSDK's decode, the RTP forward, the flight record and the abort ladder, so the thread count
 * is the number in this file most likely to mislead, and the reason [DEFAULT_THREADS] is 2 rather
 * than 8: a detector that takes 2.9 of eight cores on a device clamped to a controller in the sun
 * is a different proposition from one that takes 0.73.
 *
 * [maxHamming] is upstream's error-correction budget: how many corrected bit errors still count as
 * a decode. **This project's default is 1, not upstream's 2**, and that is measured rather than
 * cautious — see [DEFAULT_MAX_HAMMING] and
 * `docs/measurements/2026-07-28-maxhamming.md`. Both of the false ids that lost this detector the
 * one axis it lost on were decoded at hamming **2**, the maximum budget; a budget of 1 removes them
 * both and costs 0.1–0.4 percentage points of detection rate.
 *
 * ## Lifetime
 *
 * [close] must be called. The native side holds a detector, a family and a reused image, none of
 * which the JVM knows about. There is no finalizer on purpose: a finalizer that runs on a random
 * thread while a detect is in flight is a crash. [TagRecogniser] owns the one instance and closes
 * it on its own worker thread, which is also the only thread that ever calls [detect].
 *
 * **Not thread-safe, and it cannot be made so cheaply**: the native side reuses one `image_u8_t`
 * across frames precisely so that a detect allocates nothing. [detect] from two threads at once
 * would race on that buffer. Nothing in this project does; [TagRecogniser] is the only caller.
 */
class AprilTagDetector(
    nthreads: Int = DEFAULT_THREADS,
    quadDecimate: Float = DEFAULT_QUAD_DECIMATE,
    quadSigma: Float = 0.0f,
    refineEdges: Boolean = true,
    maxHamming: Int = DEFAULT_MAX_HAMMING,
    /**
     * The printed marker's black square, side to side, metres — the pose solve's scale input.
     * **`<= 0` disables the solve** (corners still cross the boundary), which is both the legacy
     * behaviour and the off switch. When enabled, the intrinsics handed to the solver come from
     * [calibration] — the assumptions and their measured costs are that class's KDoc, restated
     * at [TagPoseSolve] and gated at [TagPose.trusted].
     */
    private val tagSizeM: Double = 0.0,
    /**
     * **The session's camera model — the same instance `TagWorld`'s projection reads**, resolved
     * once in `Bridge.startTagRecogniser` from `TagRecogniser.Config.calibration`. One object on
     * both sides is what makes it structurally impossible for the solve and the bearing math to
     * disagree about the camera; a second focal length appearing in this file is the
     * two-places-for-one-property failure and must be deleted, not tested around.
     */
    private val calibration: CameraCalibration = CameraCalibration.ASSUMED,
) : TagDetector {

    private var handle: Long = nativeCreate(nthreads, quadDecimate, quadSigma, refineEdges, maxHamming)

    /** Reused across frames for the same reason the native image is: no per-frame allocation. */
    private val ids = IntArray(MAX_TAGS * 2)
    private val geom = DoubleArray(MAX_TAGS * GEOM_STRIDE)

    init {
        check(handle != 0L) { "apriltag_detector_create failed" }
    }

    override fun detect(luma: ByteArray, width: Int, height: Int): Found {
        val h = handle
        check(h != 0L) { "detect after close" }
        // The session's one camera model, per-width. The principal point is the calibration's —
        // the ASSUMED image centre until a chessboard file lands. Never the measured nadir
        // pixel: that is 2.99° away and belongs to `TagWorld.fix`'s plumb-line arithmetic, not
        // to an optical-frame projection — `CameraCalibration.nadirPointX` documents the trap.
        val n = nativeDetect(
            h, luma, width, height,
            calibration.fxPx(width), calibration.fyPx(width),
            calibration.cxPx(width), calibration.cyPx(width, height), tagSizeM,
            ids, geom,
        )
        check(n >= 0) { "nativeDetect failed" }
        if (n == 0) return Found.NOTHING
        return Found(
            List(n) { i ->
                val g = i * GEOM_STRIDE
                TagDetection(
                    id = ids[i * 2],
                    hamming = ids[i * 2 + 1],
                    centreX = geom[g],
                    centreY = geom[g + 1],
                    longestEdgePixels = geom[g + 2],
                    decisionMargin = geom[g + 3],
                    corners = TagCorners(
                        geom[g + 4], geom[g + 5], geom[g + 6], geom[g + 7],
                        geom[g + 8], geom[g + 9], geom[g + 10], geom[g + 11],
                    ),
                    solve = solveOrNull(g),
                )
            }
        )
    }

    /**
     * The slots `tag_jni.c` filled for this detection as a [TagPoseSolve], or null.
     *
     * NaN throughout means no solve ran (the boundary's convention); a rotation that does not
     * convert ([TagPose.quaternionOf] returns null on any non-finite element) refuses the whole
     * solve rather than shipping a translation with no orientation — a partial solve is not a
     * smaller claim, it is a different one.
     */
    private fun solveOrNull(g: Int): TagPoseSolve? {
        val q = TagPose.quaternionOf(geom.copyOfRange(g + 12, g + 21)) ?: return null
        val err1 = geom[g + 24]
        if (!err1.isFinite()) return null
        return TagPoseSolve(
            qx = q[0], qy = q[1], qz = q[2], qw = q[3],
            tx = geom[g + 21], ty = geom[g + 22], tz = geom[g + 23],
            err1 = err1,
            // May be +Infinity: no second minimum found, i.e. unambiguous. A value, not a gap.
            err2 = geom[g + 25],
            tagSizeM = tagSizeM,
        )
    }

    override fun close() {
        if (handle != 0L) {
            nativeDestroy(handle)
            handle = 0L
        }
    }

    companion object {
        /**
         * How many tags one frame may report. Sixteen against a board that will carry two, because
         * the cost is 96 bytes of reused array and the alternative — silently truncating in a scene
         * that turned out to be full of markers — is the kind of cap nobody notices.
         */
        private const val MAX_TAGS = 16

        /**
         * Doubles per detection in `geomOut` — must match `GEOM_STRIDE` in `tag_jni.c`, whose
         * comment holds the slot-by-slot layout. 4 legacy values, 8 corner coordinates, then the
         * pose solve: R row-major (9), t (3), and the two ambiguity errors.
         */
        private const val GEOM_STRIDE = 26

        /**
         * **2, and measured rather than assumed.** Interpolating the measured table, 2 threads is
         * ~86 ms and 4 is ~56, both inside the 100 ms a 10 Hz cap allows — and 2 costs 1.68 cores
         * against 4's 2.43 on a phone that is also the pilot. Start low and raise it against a
         * number, which is the instruction this default encodes.
         */
        const val DEFAULT_THREADS = 2

        /** **Not upstream's 2.0.** See the class KDoc: 2.0 takes the working height from 7 m to 4.5. */
        const val DEFAULT_QUAD_DECIMATE = 1.0f

        /**
         * **1, not upstream's 2**, and this is the one default here that is a *measurement of a
         * safety trade* rather than of a cost.
         *
         * The comparison that chose this detector found the one thing it was worse at than the
         * OpenCV it replaced: **2 false ids in 1978 frames against zero**. A false id is worse than
         * a miss for a landing controller, because a miss is visible and a false positive is
         * confident. That document named `maxhamming` as the lever and said it was untested.
         *
         * It is now tested, over both flights' full descent windows with the `luma_sum` gate
         * confirming identical input frames (`docs/measurements/2026-07-28-maxhamming.md`):
         *
         * | budget | 25 Hz flight | 5 Hz flight | false ids |
         * |---|---|---|---|
         * | 0 | 500/718 = 69.6 % | 890/1256 = 70.9 % | **none** |
         * | **1** | **503/718 = 70.1 %** | **900/1256 = 71.7 %** | **none** |
         * | 2 (upstream) | 506/718 = 70.5 % | 902/1256 = 71.8 % | id 102 ×1, id 46 ×1 |
         *
         * **Both false ids were decoded at hamming exactly 2** — the top of the budget — so cutting
         * it to 1 removes them and keeps most of what the correction buys: 3 of the 6 extra
         * detections on one flight, 10 of the 12 on the other. 0 removes them too and gives back
         * more, so 1 is the honest middle: it is the largest budget with no measured false positive.
         *
         * **Two flights, one site, one afternoon, one printed tag.** This is a better basis than
         * the guess it replaces, and it is not a guarantee. `TagLatch`'s three-sightings-in-two-
         * seconds filter stays regardless — it is the defence against the false id nobody has seen
         * yet, and this default is the defence against the two that were.
         */
        const val DEFAULT_MAX_HAMMING = 1

        /**
         * True once the native library is present and loadable.
         *
         * A missing `.so` is a build-configuration failure, not a detector result, and it must not
         * be reported as "apriltag found nothing". Callers check this before constructing one; the
         * UI says "no on-board detector" rather than "tag not seen", because an operator deciding
         * whether to trust a descent needs those two apart.
         */
        val available: Boolean by lazy {
            runCatching { System.loadLibrary("apriltagjni") }.isSuccess
        }

        @JvmStatic
        private external fun nativeCreate(
            nthreads: Int,
            quadDecimate: Float,
            quadSigma: Float,
            refineEdges: Boolean,
            maxHamming: Int,
        ): Long

        /**
         * @param idsOut 2 ints per detection: id, hamming.
         * @param geomOut [GEOM_STRIDE] doubles per detection — layout in `tag_jni.c`. The pose
         *   slots are NaN when [tagSizeM] `<= 0` (solve off) and the corner slots are always
         *   real.
         * @return detections written, capped by the smaller of the two arrays; negative on failure.
         */
        @JvmStatic
        private external fun nativeDetect(
            handle: Long,
            luma: ByteArray,
            width: Int,
            height: Int,
            fx: Double,
            fy: Double,
            cx: Double,
            cy: Double,
            tagSizeM: Double,
            idsOut: IntArray,
            geomOut: DoubleArray,
        ): Int

        @JvmStatic
        private external fun nativeDestroy(handle: Long)
    }
}
