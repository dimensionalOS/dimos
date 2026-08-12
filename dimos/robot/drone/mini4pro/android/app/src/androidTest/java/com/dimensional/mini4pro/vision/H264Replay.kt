package com.dimensional.mini4pro.vision

import android.media.MediaCodec
import android.media.MediaFormat
import android.os.SystemClock
import android.util.Log
import java.io.RandomAccessFile
import java.nio.ByteBuffer

/**
 * **Decodes a recorded flight's video on the phone, one indexed frame at a time.**
 *
 * This is the harness the whole tag-detection question gets answered against, and it exists so
 * that no measurement needs an aircraft. Its input is the 2374 frames of
 * `datasets/2026-07-27-apriltag-nadir-land/` — real H.264 off a real Mini 4 Pro, at the resolution
 * and bitrate the aircraft actually sends — so the numbers it produces are about the device and
 * the codec rather than about a synthetic clip.
 *
 * ## Why it decodes to luminance and stops there
 *
 * Every AprilTag detector in existence works on a single-channel grayscale image; colour is thrown
 * away in the first line of every one of them. `MediaCodec` in ByteBuffer mode hands back a
 * `YUV_420_888` [android.media.Image] whose **plane 0 is exactly that grayscale image**, already
 * separated, with no conversion to pay for. So the cheapest honest pipeline is decode → copy plane
 * 0 → detect, and that is what is measured here. Anything that converts to RGB first is paying
 * three times the memory traffic for information the detector discards.
 *
 * The decimation in [LumaSink] is a *stride copy*, not a resampling filter — it takes every
 * [Decimation.step]-th pixel of every step-th row. That is the wrong thing for a picture and the
 * right thing here: a box or bilinear filter costs several times as much and, for a black-and-white
 * fiducial whose edges are what the detector fits, blurs precisely the feature being measured. It
 * does alias, and the honest consequence is that the smallest detectable tag gets *worse* faster
 * than the decimation factor — which is a detection-rate question, measured, not argued.
 *
 * ## What the timings mean, and what they cannot mean
 *
 * A hardware decoder is a pipeline. "Milliseconds to decode one frame" is therefore **not
 * measurable** by wrapping a stopwatch around one frame's `dequeueOutputBuffer` — that number is
 * mostly queueing, and it changes when the consumer downstream gets slower. So:
 *
 *  - **[Result.fps] is the honest decode number.** Frames in, wall clock, no pacing: the ceiling
 *    the pipeline sustains with this consumer attached. Comparing it across consumers is what
 *    isolates the consumer's cost.
 *  - **[Result.sinkMeanMillis] and [Result.consumerMeanMillis] are direct measurements**, because
 *    both run synchronously on the calling thread and nothing pipelines them.
 *
 * That distinction is the reason this class reports two kinds of number rather than pretending
 * every stage is separable.
 *
 * ## Threading
 *
 * Everything here runs on the caller's thread — deliberately. This is a bench harness with no
 * MSDK callback to protect, and a single thread makes the timings mean what they say. **The
 * shipping detector does the opposite**: it hands frames to its own thread through a bounded queue
 * that drops rather than waits, because there the producer is MSDK's decode thread and blocking it
 * stalls QGC's video and the flight record together (`video/RawFrameSink`, `video/RtpVideoSink`).
 */
internal class H264Replay(
    private val streamPath: String,
    private val frames: List<FrameIndex.Frame>,
    private val width: Int = 1920,
    private val height: Int = 1080,
) {

    /**
     * How much of the frame to keep. The detector sees a [LumaFrame] of
     * `width/step × height/step`.
     *
     * `FULL` is what MSDK's own `addFrameListener` would deliver — it offers no scale choice
     * (`ICameraStreamManager.addFrameListener` calls `GLFrameDispatcher.addOnFrameListener(format,
     * -1, -1, listener)`, and −1 means "input size"), so any reduction is ours to do.
     */
    enum class Decimation(val step: Int) { FULL(1), HALF(2), QUARTER(4) }

    /** One decoded frame's luminance, plus where it sits in the flight. */
    class LumaFrame(
        val luma: ByteArray,
        val width: Int,
        val height: Int,
        /** The frame's `n` in the recorder's index — the join back to telemetry. */
        val n: Long,
        /** The flight record's own clock, seconds. */
        val tSeconds: Double,
    )

    /** What a run measured. */
    data class Result(
        val label: String,
        val framesSubmitted: Int,
        val framesDecoded: Int,
        /** Frames that reached the consumer — the window, not the whole stream. */
        val framesConsumed: Int,
        val wallMillis: Long,
        /** Nanoseconds spent copying luminance out of the decoder's image, summed. */
        val sinkNanos: Long,
        /** Nanoseconds spent in the caller's consumer, summed. */
        val consumerNanos: Long,
        val outputWidth: Int,
        val outputHeight: Int,
    ) {
        val fps: Double get() = if (wallMillis <= 0) 0.0 else framesDecoded * 1000.0 / wallMillis
        // Averaged over the frames that actually did the work, not over the whole stream —
        // dividing by frames that were dropped unmapped would understate both by the ratio of the
        // window to the recording, which is the sort of error that looks like good news.
        val sinkMeanMillis: Double
            get() = if (framesConsumed == 0) 0.0 else sinkNanos / 1e6 / framesConsumed
        val consumerMeanMillis: Double
            get() = if (framesConsumed == 0) 0.0 else consumerNanos / 1e6 / framesConsumed
    }

    /**
     * The decoder's output colour format, and why this is a parameter at all.
     *
     * Three ways of getting pixels out of `MediaCodec` were measured on this device on
     * 2026-07-27, and **the two obvious ones both fail**:
     *
     *  - `COLOR_FormatYUV420Flexible` (`0x7F420888`) in ByteBuffer mode works and runs at
     *    **10.6 fps** against 576 fps for decode alone — a 54× collapse, at 0.02 cores of *our*
     *    CPU, because the time is a detile-and-convert pass the codec HAL does in its own process
     *    to turn a vendor-tiled surface into something linear. "Flexible" means "the codec picks",
     *    and on Qualcomm it picks compressed.
     *  - An `ImageReader` fed straight from the decoder's surface — the shape MSDK's own frame
     *    listener uses — **aborts the process** in `Image.getPlanes`
     *    (`JNI DETECTED ERROR IN APPLICATION: non-zero capacity for nullptr pointer`), with or
     *    without `HardwareBuffer.USAGE_CPU_READ_OFTEN`. A SIGABRT, so nothing catches it. MSDK
     *    gets away with the same shape because it puts a **GL shader** between the two, which is
     *    what converts the tiled surface into something an `ImageReader` can hand back.
     *
     * What works is asking for a linear format **by name**. `c2.qti.avc.decoder` advertises
     * `COLOR_FormatYUV420SemiPlanar` (21, NV12) and `COLOR_FormatYUV420Planar` (19, I420)
     * outright, and either gives a plane 0 that is already the grayscale image with nothing to
     * convert. That capability list is printed by `TagProfileTest.codecs`, which exists because
     * two guesses in a row were wrong.
     */
    enum class Colour(val format: Int) {
        /** NV12: plane 0 linear luminance, chroma interleaved after it. */
        SEMI_PLANAR(21),

        /** I420: plane 0 linear luminance, then two chroma planes. */
        PLANAR(19),

        /** "Let the codec choose", which on this device chooses a tiled format. Kept to measure. */
        FLEXIBLE(0x7F420888),
    }

    /**
     * Feed every frame through the decoder, hand each result to [consumer], and time it.
     *
     * [consumer] is null for a decode-only run: the output buffer is released without ever being
     * mapped, which measures the decoder alone. That baseline is the thing every other number is
     * read against.
     *
     * [pacedFps] > 0 sleeps to hold the real arrival rate, for a run whose question is heat and
     * power at the working rate rather than throughput. 0 runs flat out.
     */
    fun run(
        label: String,
        decimation: Decimation = Decimation.FULL,
        colour: Colour = Colour.SEMI_PLANAR,
        codecName: String? = null,
        windowFrom: Double = Double.NEGATIVE_INFINITY,
        windowTo: Double = Double.POSITIVE_INFINITY,
        pacedFps: Double = 0.0,
        loops: Int = 1,
        consumer: ((LumaFrame) -> Unit)? = null,
    ): Result {
        val codec =
            if (codecName != null) MediaCodec.createByCodecName(codecName)
            else MediaCodec.createDecoderByType(MIME)
        val format = MediaFormat.createVideoFormat(MIME, width, height).apply {
            // The stream is Annex-B with in-band SPS/PPS on every keyframe, so no csd-0 is
            // needed: the decoder picks the parameter sets out of the first access unit. That is
            // also exactly how MSDK feeds it, which is the point of using these bytes.
            setInteger(MediaFormat.KEY_COLOR_FORMAT, colour.format)
        }
        codec.configure(format, null, null, 0)
        codec.start()

        val file = RandomAccessFile(streamPath, "r")
        val readBuffer = ByteArray(frames.maxOf { it.length })

        var submitted = 0
        var decoded = 0
        var consumed = 0
        var sinkNanos = 0L
        var consumerNanos = 0L
        var outW = 0
        var outH = 0
        var scratch: ByteArray? = null
        // The padded row pitch, learned from INFO_OUTPUT_FORMAT_CHANGED. The unpadded picture
        // width is the honest default until the decoder says otherwise. `slice-height` — the
        // padded plane height — is deliberately not tracked: only plane 0 is read, and plane 0
        // starts at offset zero in every layout the codec advertises.
        var stride = width

        val info = MediaCodec.BufferInfo()
        val startedAt = SystemClock.elapsedRealtime()
        val paceNanosPerFrame = if (pacedFps > 0.0) (1e9 / pacedFps).toLong() else 0L
        val paceOrigin = System.nanoTime()

        // The whole run — every loop — is fed as one continuous input stream, terminated by an
        // explicit end-of-stream. That is not tidiness: a decoder holds several frames in its
        // reorder buffer and **will not emit them without an EOS**, so a loop that waits for
        // `decoded == submitted` without sending one hangs forever a few frames from the end. It
        // did, on the first run, for ten minutes. Looping inside the input stream rather than
        // restarting the codec also keeps the soak measuring what a flight measures: one long
        // decode, with no per-loop flush the aircraft would never cause.
        val total = frames.size.toLong() * loops
        try {
            run {
                var next = 0L
                var sawInputEnd = false
                var sawOutputEnd = false
                while (!sawOutputEnd) {
                    if (!sawInputEnd) {
                        val inIndex = codec.dequeueInputBuffer(TIMEOUT_US)
                        if (inIndex >= 0) {
                            if (next >= total) {
                                codec.queueInputBuffer(
                                    inIndex, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM,
                                )
                                sawInputEnd = true
                            } else {
                                val frame = frames[(next % frames.size).toInt()]
                                file.seek(frame.offset)
                                file.readFully(readBuffer, 0, frame.length)
                                val input: ByteBuffer = codec.getInputBuffer(inIndex)!!
                                input.clear()
                                input.put(readBuffer, 0, frame.length)
                                // Presentation time carries the frame's index, so a decoded frame
                                // can be joined back to the flight after the decoder reorders.
                                codec.queueInputBuffer(inIndex, 0, frame.length, next, 0)
                                submitted++
                                next++
                            }
                        }
                    }

                    val outIndex = codec.dequeueOutputBuffer(info, TIMEOUT_US)
                    if (info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) sawOutputEnd = true
                    if (outIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                        // The one moment the real layout is knowable. `stride` and `slice-height`
                        // are the padded row pitch and the padded plane height the decoder chose,
                        // and they are routinely larger than the picture — reading plane 0 as if
                        // it were `width * height` tightly packed gives a sheared image that still
                        // looks plausible enough to waste an afternoon.
                        val f = codec.outputFormat
                        stride = if (f.containsKey("stride")) f.getInteger("stride") else width
                    }
                    if (outIndex >= 0) {
                        val idx = (info.presentationTimeUs % frames.size).toInt()
                        // Outside the window a frame is decoded and dropped unmapped. That is not
                        // an optimisation, it is what makes the run finish: mapping is 94 ms a
                        // frame on this device and detection another 81, so paying both for the
                        // 22 s this aircraft spent on the ground before takeoff would quadruple a
                        // run that answers nothing extra. Decode still happens for every frame —
                        // it has to, the stream is inter-coded — and it is nearly free.
                        val inWindow = frames.getOrNull(idx)
                            ?.tSeconds?.let { it in windowFrom..windowTo } ?: true
                        if (consumer != null && inWindow) {
                            // Two ways of reaching plane 0, and the format decides which is
                            // correct. A linear format (NV12/I420) is a plain buffer with a known
                            // row pitch, so `getOutputBuffer` is right and cheap. FLEXIBLE is a
                            // vendor-tiled surface whose bytes mean nothing without the layout —
                            // reading it as if it were linear yields a sheared image that still
                            // looks plausible enough to waste an afternoon — so it has to go
                            // through `getOutputImage`, which is what pays the 94 ms detile.
                            val image =
                                if (colour == Colour.FLEXIBLE) codec.getOutputImage(outIndex) else null
                            val buffer = if (image == null) codec.getOutputBuffer(outIndex) else null
                            if (image != null || buffer != null) {
                                val t0 = System.nanoTime()
                                val w = width / decimation.step
                                val h = height / decimation.step
                                var dst = scratch
                                if (dst == null || dst.size != w * h) {
                                    dst = ByteArray(w * h)
                                    scratch = dst
                                }
                                if (image != null) {
                                    val plane = image.planes[0]
                                    copyLuma(
                                        plane.buffer, 0, plane.rowStride, plane.pixelStride,
                                        image.width, image.height, decimation.step, dst,
                                    )
                                } else if (buffer != null) {
                                    copyLuma(
                                        buffer, info.offset, stride, 1,
                                        width, height, decimation.step, dst,
                                    )
                                }
                                outW = w
                                outH = h
                                sinkNanos += System.nanoTime() - t0

                                val frame = frames.getOrNull(idx)
                                val t1 = System.nanoTime()
                                consumer.invoke(
                                    LumaFrame(
                                        luma = dst,
                                        width = w,
                                        height = h,
                                        n = frame?.n ?: idx.toLong(),
                                        tSeconds = frame?.tSeconds ?: 0.0,
                                    ),
                                )
                                consumerNanos += System.nanoTime() - t1
                                image?.close()
                            }
                        }
                        codec.releaseOutputBuffer(outIndex, false)
                        decoded++
                        if (inWindow) consumed++
                        // Progress, because a run that stops telling you anything is
                        // indistinguishable from one that is merely slow, and both happened.
                        if (decoded % PROGRESS_EVERY == 0) {
                            Log.i(
                                TAG,
                                "$label: $decoded/$total frames, " +
                                    "${SystemClock.elapsedRealtime() - startedAt} ms",
                            )
                        }

                        if (paceNanosPerFrame > 0L) {
                            val due = paceOrigin + paceNanosPerFrame * decoded
                            val sleep = due - System.nanoTime()
                            if (sleep > 0) Thread.sleep(sleep / 1_000_000, (sleep % 1_000_000).toInt())
                        }
                    }
                }
            }
        } finally {
            file.close()
            runCatching { codec.stop() }
            runCatching { codec.release() }
        }

        return Result(
            label = label,
            framesSubmitted = submitted,
            framesDecoded = decoded,
            framesConsumed = consumed,
            wallMillis = SystemClock.elapsedRealtime() - startedAt,
            sinkNanos = sinkNanos,
            consumerNanos = consumerNanos,
            outputWidth = if (outW > 0) outW else width / decimation.step,
            outputHeight = if (outH > 0) outH else height / decimation.step,
        )
    }

    private companion object {
        const val TAG = "TagProfile"
        const val PROGRESS_EVERY = 250
        const val MIME = "video/avc"

        /**
         * 10 ms. Long enough that the loop is not a spin, short enough that input and output are
         * both serviced while the pipeline fills.
         */
        const val TIMEOUT_US = 10_000L

        /**
         * Copy plane 0 into a tightly-packed grayscale buffer, taking every [step]-th pixel.
         *
         * The fast path matters: at `step == 1` and `pixelStride == 1` a whole row is one bulk
         * `get`, which is a `memcpy` rather than 1920 bounds-checked reads. Vendors do use
         * `pixelStride == 2` on the luma plane of some semi-planar layouts, so the slow path is not
         * dead code — and getting it wrong would silently halve the image.
         */
        fun copyLuma(
            src: ByteBuffer,
            base: Int,
            rowStride: Int,
            pixelStride: Int,
            width: Int,
            height: Int,
            step: Int,
            dst: ByteArray,
        ) {
            val outWidth = width / step
            val outHeight = height / step
            var d = 0
            if (step == 1 && pixelStride == 1) {
                for (row in 0 until outHeight) {
                    src.position(base + row * rowStride)
                    src.get(dst, d, outWidth)
                    d += outWidth
                }
                return
            }
            for (row in 0 until outHeight) {
                var s = base + row * step * rowStride
                for (col in 0 until outWidth) {
                    dst[d++] = src.get(s)
                    s += pixelStride * step
                }
            }
        }
    }
}
