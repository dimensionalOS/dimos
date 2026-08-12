package com.dimensional.mini4pro.vision

/**
 * **Where decoded luminance comes from.** The seam that keeps everything above it on the JVM, in
 * exactly the shape `record/VideoSidecar` uses with `VideoSink` and for the same stated reason: the
 * parts most likely to be wrong — the queue, the drop rule, the arming policy, the latch — are the
 * parts least likely to be exercised on a phone before a flight, so they must be drivable by an
 * in-memory implementation in a plain unit test.
 *
 * The one production implementation is `MsdkFrameSource`, and it is the only file in this package
 * that imports DJI. What it is covered by, and what it is not, is stated there.
 */
interface FrameSource {

    /**
     * Begin delivering frames to [listener]. Returns null on success, or a sentence saying why not.
     *
     * A sentence rather than a boolean because every reason this can fail — MSDK not registered, no
     * aircraft, no camera — is one an operator can act on, and "video frames: false" is not.
     */
    fun start(listener: FrameListener): String?

    /** Stop delivering. Safe to call twice, and safe to call when [start] failed. */
    fun stop()
}

/**
 * One decoded frame's **luminance plane**, and nothing else.
 *
 * ## The contract, because it is the whole safety argument
 *
 * **This is called on the frame source's own thread, and that thread matters.** For the MSDK source
 * it is MSDK's decode dispatch, which is also — and this is the part that was inference until it was
 * measured — adjacent to the passthrough that feeds QGroundControl. So:
 *
 * **Do no work here.** Copy what you need out of [data] and return. [data] is the source's buffer
 * and is valid only for the duration of the call; the next frame will overwrite it.
 *
 * **Never block.** [TagRecogniser] is the only implementation in the project and it offers into a
 * single-slot mailbox that drops rather than waits. Anything that could stall here stalls the video
 * a pilot is flying on.
 *
 * **Never throw.** The source contains it, but a listener that relies on that is one refactor away
 * from killing a decode thread.
 */
fun interface FrameListener {

    /**
     * @param data the frame buffer. Luminance is `width * height` bytes from [offset], one byte per
     *   pixel, tightly packed — plane 0 of NV21, whose chroma follows and is not wanted.
     * @param offset where the luminance plane starts in [data].
     * @param width pixels across. **1920 on this aircraft**: the source passes −1,−1 for size, so
     *   MSDK delivers the stream's own geometry and any reduction is ours to make deliberately.
     * @param height pixels down. 1080 on this aircraft.
     * @param atNanos `SystemClock.elapsedRealtimeNanos()`, **stamped first thing in the callback**,
     *   on the same clock every flight-record entry, gimbal reading and setpoint uses. That single
     *   fact is what lets a controller age-compensate rather than assume currency, and it is why the
     *   stamp is taken by the source rather than by whatever eventually looks at the frame.
     */
    fun onLuma(data: ByteArray, offset: Int, width: Int, height: Int, atNanos: Long)
}
