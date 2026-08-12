package com.dimensional.mini4pro.vision

/**
 * **What the camera can see, for whoever asks.** One tag, most recently detected, or nothing.
 *
 * This is a *seam*, not an implementation. It exists so the display, and later the landing law,
 * can be written and wired against something that does not exist yet — the same move
 * `command/PendingClimb` and `guided/MissionTakeoff` make, and for the same reason: the thing that
 * consumes a detection and the thing that produces one are on opposite sides of a problem
 * (Android, a decoder, a vision library) that should not leak into either.
 *
 * The implementation is [TagRecogniser], since 2026-07-28.
 *
 * ## Why detection has to end up here, whatever develops it
 *
 * The **phone is the pilot**: it holds the abort ladder, it owns the setpoint stream, and it is the
 * only thing that can close a loop fast enough to land on something. A detection computed anywhere
 * else arrives too late to fly on, and across a link this project has already measured blackholing
 * traffic in one direction. So however the recogniser is developed, this is where it lands, and
 * an on-board one is required rather than optional (Ivan, 2026-07-27).
 *
 * Development happens elsewhere on purpose: flights are converted into a DiMOS **mem2** store and
 * its own tag recognisers run over them there, rather than this repository growing a second
 * detector to maintain. What that leaves here is the on-board implementation and the obligation to
 * check it against the one that ran offline — the split `tools/zenohreplay` already uses, where an
 * offline re-derivation is cross-checked byte-for-byte against the Kotlin encoder that ships.
 *
 * ## What an implementation will have to answer, and what it must not do
 *
 * **It must not consult gimbal attitude to decide whether a detection is usable.** That key is
 * change-driven and goes silent precisely when the gimbal is holding still, which is the whole of a
 * nadir approach — the trap this project has now hit seven times. The camera's pointing is
 * *commanded* and known from the command, not read back. [TagArming] and [CameraPose] are the two
 * places it would have been tempting to break this, and both say so at the field.
 *
 * **It must not block the video path.** Frames arrive on MSDK's decode thread, which is also the
 * passthrough to the ground station. Detection is work; it belongs on its own thread with a
 * bounded queue that drops rather than waits, exactly as `record/Recorder` does. That is
 * [LatestFrame], and on 2026-07-28 the aircraft settled what was until then an inference: holding
 * the frame callback for 100 ms a frame took the *decoded* delivery from 24.0 fps to 9.7 and left
 * the encoded passthrough at 24.0 fps, three runs in a row. They are different threads.
 */
interface TagSighting {

    /** The most recent sighting, or null if nothing has been seen or the last one is stale. */
    fun latest(): Sighting?

    /**
     * **The newest sighting's place in the world**, or null when the newest sighting could not
     * be placed (no position, no heading, no commanded camera angle, camera off nadir — every
     * refusal is [TagWorld.fix]'s) or nothing has been seen this flight.
     *
     * Separate from [latest] on the same argument that separates [latest] from [latched]: this
     * is the *control* question — "where does the camera most recently say the tag is, in the
     * frame a velocity can be computed in" — and unlike [latest] it is **not** blanked by the
     * display staleness bound. A controller must read [TagFix.atNanos] and apply its own
     * staleness ladder, because 400 ms and 2 s and 10 s mean three different things to a
     * descent and this interface cannot know which one its caller is on. It is also not
     * [latched]'s fix: the latch keeps the fix from the **largest** tag ever seen (best for
     * "where is the pad" across a whole flight), while a descent centring over the tag needs
     * the **newest** (best for "where is it relative to me right now").
     *
     * Cleared with the flight, exactly as the latch is.
     */
    fun latestFix(): TagFix?

    /**
     * **Where this flight's tag is**, once one has been seen enough times to believe — or null.
     *
     * Separate from [latest] because they answer different questions and go stale at wildly
     * different rates. [latest] is "can the camera see it right now", measured in tens of
     * milliseconds. This is "does this site have a tag, and where", and it survives the whole flight
     * including the minutes at cruise altitude when nothing is detectable at all.
     */
    fun latched(): Latched?

    /**
     * One detection.
     *
     * The frame is the **camera's**: [x] right, [y] down, [z] forward along the optical axis, in
     * metres. Deliberately not converted to north/east here — that conversion needs the aircraft's
     * yaw and the camera's pointing, and doing it in the detector would bury two assumptions inside
     * a number that looks like a measurement. [TagWorld.fix] does it, refuses when any input is
     * missing, and labels what it assumed.
     */
    data class Sighting(
        val tagId: Int,
        val x: Double,
        val y: Double,
        val z: Double,
        /** Monotonic nanoseconds, from the same clock the flight record and every setpoint use. */
        val atNanos: Long,
        /**
         * How much of the frame the tag occupies, as the longest edge in pixels. The honest proxy
         * for "how far away and how trustworthy": a tag twenty pixels across has a pose solution
         * whose angular error is large however confident the library sounds — **measured now**,
         * not merely asserted: below 20 px the ambiguity flip rate is 82 % and the solver's own
         * residual says nothing about it (`docs/measurements/2026-07-28-pose-solve-stability.md`),
         * which is why [TagPose.MIN_SOLVE_PIXELS] gates every published orientation on this
         * field.
         */
        val pixelSize: Double,
        /**
         * **False when the pose rests on assumed camera intrinsics rather than measured ones.**
         *
         * The focal length is a two-flight *fit* — 1465.9 px and 1448.3 px, reproducing to 1.2 % —
         * the principal point is assumed to be the image centre, and distortion is entirely
         * unmeasured. A pose derived from it is coarse, and anything that flies on it must decide
         * deliberately to. `docs/apriltag-landing-recording.md` §6.2.
         */
        val metric: Boolean = false,
        /**
         * Bit errors the detector corrected to arrive at [tagId], `0..maxhamming`. Zero means the
         * decode needed no correction at all. Carried because a false id is worse than a miss for a
         * landing controller, and this is the per-detection evidence about that.
         */
        val hamming: Int = 0,
        /** The detector's own confidence in the decode. Carried, never thresholded here. */
        val decisionMargin: Double = 0.0,
        /** The tag's centre in the frame, pixels, x right and y **down**. */
        val centreX: Double = 0.0,
        val centreY: Double = 0.0,
        /** The frame's geometry, so a bearing can be re-derived under a real calibration later. */
        val imageWidth: Int = 0,
        val imageHeight: Int = 0,
        /**
         * The tag's four corners in pixels, since 2026-07-28, or null on a sighting rebuilt
         * from a record older than the field. Null is "not measured", never four zeros.
         */
        val corners: TagCorners? = null,
        /**
         * apriltag's own pose estimate, **raw and ungated**, or null when nothing solved one —
         * legacy records, solve disabled, or a solve the estimator itself could not produce.
         *
         * Raw is the contract: whether this deserves belief is [TagPose.trusted]'s question,
         * asked where a consumer could act on the answer (the detections encoder, a landing
         * law), not here. A consumer reading this field directly without that gate is reading
         * scatter below [TagPose.MIN_SOLVE_PIXELS] and coin-flips near ambiguity ratio 1 —
         * both measured, `docs/measurements/2026-07-28-pose-solve-stability.md`.
         */
        val solve: TagPoseSolve? = null,
    ) {
        /**
         * **How old this sighting is**, in milliseconds, at [nowNanos].
         *
         * Present, and named this plainly, because the number is *not small* and a consumer that
         * assumed currency would be wrong in the direction that hurts. Measured on the aircraft on
         * 2026-07-28: frames arrive every 41.5 ms (p50; 47 ms at p90), the detector takes 35–50 ms
         * at two threads, and the 10 Hz cap adds up to another 125 ms of waiting for the next
         * admitted frame. **A sighting handed to a 25 Hz loop is typically 60–160 ms old**, and at
         * 3 m/s of descent that is half a metre.
         *
         * A controller reading this must age-compensate. There is nothing this class can do to make
         * the sighting fresher; what it can do is make being stale impossible to overlook, which is
         * what a method on the value itself does and a bare timestamp does not.
         */
        fun ageMillisAt(nowNanos: Long): Long = (nowNanos - atNanos) / 1_000_000
    }
}
