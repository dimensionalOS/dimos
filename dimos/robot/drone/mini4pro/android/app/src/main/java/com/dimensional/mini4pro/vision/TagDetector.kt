package com.dimensional.mini4pro.vision

/**
 * **One tag, in one frame, in pixels.** Nothing here is metric and nothing here knows where the
 * aircraft is; turning this into a place in the world is [TagWorld]'s job, with the aircraft's own
 * state and a stated set of assumptions.
 *
 * Pixel coordinates are the image's: [centreX] right from the left edge, [centreY] **down** from
 * the top. That is the convention every frame buffer and every detector library uses, and
 * converting it here would put a sign flip somewhere a reader would have to go and find.
 */
data class TagDetection(
    val id: Int,
    /**
     * Bit errors apriltag corrected to arrive at [id], `0..maxhamming`.
     *
     * Recorded per detection because it is the per-detection half of the one question this
     * detector loses to OpenCV on. Two false ids in 1978 frames
     * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §4) is a rate a landing controller is
     * least tolerant of, `maxhamming` is the lever, and a lever cannot be pulled against a budget
     * nobody can see. A `hamming` of 0 is a decode that needed no correction at all.
     */
    val hamming: Int,
    val centreX: Double,
    val centreY: Double,
    /**
     * The longest of the four edges of *this* tag, in pixels. The honest proxy for "how far away
     * and how trustworthy" — the x-axis of every detection-rate curve this project has measured.
     */
    val longestEdgePixels: Double,
    /**
     * apriltag's own confidence in the decode, from `apriltag_detection_t.decision_margin`.
     *
     * Carried and **not thresholded**. What a useful cut-off is on this camera is unmeasured, and
     * a threshold nobody has measured is worse than no threshold, because it looks like one.
     */
    val decisionMargin: Double,
    /**
     * The four corners, since 2026-07-28, or null from a detector that does not report them.
     *
     * Still pixels, so still inside this type's "nothing here is metric" rule. Null is the
     * legacy shape — every consumer treats it as "corners were not measured", never as (0,0)×4.
     */
    val corners: TagCorners? = null,
    /**
     * apriltag's own pose estimate, since 2026-07-28, or null when nothing solved one.
     *
     * The one *metric* field on this type, and deliberately so — the solve is the detector's
     * output, computed in the same native call from the same corners, and splitting it off would
     * manufacture a second detection type for one field. It is **raw**: carried whether or not it
     * deserves belief, which is [TagPose.trusted]'s question and is asked at publication, not
     * here. Null means no solve ran (no intrinsics/tag size, or a legacy detector), never that a
     * solve failed a gate.
     */
    val solve: TagPoseSolve? = null,
)

/**
 * One frame's verdict from a tag detector.
 *
 * Lifted out of `ArucoProbe` on 2026-07-27 when OpenCV left the build. It was declared there
 * because OpenCV was the first detector wired up; it was never an OpenCV concept. A shared verdict
 * type is what made the two detectors comparable frame for frame, which is the whole basis of
 * `docs/measurements/2026-07-27-apriltag-c-vs-opencv.md`.
 *
 * Moved from `src/androidTest` into `src/main` on 2026-07-27 when the detector stopped being a
 * measurement and started being a sensor.
 *
 * [ids] and [longestEdgePixels] are kept as derived properties because the measurement harness and
 * `tools/tagcompare`'s CSV are written against them and the published numbers must stay
 * reproducible from an unchanged column.
 */
data class Found(val tags: List<TagDetection>) {

    val any: Boolean get() = tags.isNotEmpty()

    /** Every id in this frame, in the order the detector reported them. */
    val ids: IntArray get() = IntArray(tags.size) { tags[it].id }

    /** The longest edge of the **largest** tag, or 0.0 when nothing was found. */
    val longestEdgePixels: Double get() = tags.maxOfOrNull { it.longestEdgePixels } ?: 0.0

    /** The largest tag in the frame, or null. Size is the only ranking available without a pose. */
    val largest: TagDetection? get() = tags.maxByOrNull { it.longestEdgePixels }

    companion object {
        val NOTHING = Found(emptyList())
    }
}

/**
 * **What turns a luminance plane into tags.** The seam that keeps everything above it on the JVM.
 *
 * `record/VideoSidecar` puts a file behind `VideoSink` for exactly this reason, and states it: the
 * parts most likely to be wrong and least likely to be exercised on a phone before a flight are the
 * policy parts, and they should be drivable by an in-memory implementation in a plain unit test.
 * Here that is the whole of [TagRecogniser] — the queue, the drop accounting, the arming rule, the
 * latch and the geometry — none of which needs a real detector to be wrong in an interesting way.
 *
 * ## The contract an implementation inherits
 *
 * **[detect] may take a hundred milliseconds and that is expected.** Measured on this phone:
 * 108.8 ms/frame single-threaded at 1080p, 38.3 ms at eight threads. It must therefore never be
 * called from a thread that a video path, a setpoint loop or a UI is waiting on. [TagRecogniser]
 * is what guarantees that, and it is the only thing that calls this.
 *
 * **[luma] belongs to the caller for the duration of the call and no longer.** An implementation
 * that keeps the reference will be handed the same array back, overwritten, on the next frame.
 */
interface TagDetector : java.io.Closeable {

    /**
     * One frame's tags. [luma] is a tightly packed 8-bit luminance plane, `width * height` bytes.
     *
     * Implementations must not throw on an ordinary bad frame — a frame that decodes to nothing is
     * [Found.NOTHING], not an exception. [TagRecogniser] contains throws anyway, because this runs
     * beside a flight.
     */
    fun detect(luma: ByteArray, width: Int, height: Int): Found
}
