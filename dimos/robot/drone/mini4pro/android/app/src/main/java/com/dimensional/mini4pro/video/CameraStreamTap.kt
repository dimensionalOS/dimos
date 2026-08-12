package com.dimensional.mini4pro.video

import android.os.SystemClock
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.datacenter.camera.StreamInfo
import dji.v5.manager.interfaces.ICameraStreamManager
import java.util.concurrent.atomic.AtomicLong

/**
 * Counts the encoded frames MSDK receives from the aircraft.
 *
 * This exists because every interesting failure in the video path is silent, and
 * *this* counter is the one that separates them. "RTSP server up but zero
 * frames" and "frames flowing but no client connected" look identical from the
 * live-stream status alone; the tap tells them apart.
 *
 * It subscribes with `ICameraStreamManager.addReceiveStreamListener`
 * (`tools/djidoc ICameraStreamManager`), which delivers the raw encoded stream —
 * the same feed the future RTP path needs, hence [RawFrameSink].
 *
 * Two notes from the docs that make this cheap rather than intrusive:
 *  - the listener is a *tap* on data MSDK is already receiving; it adds no link
 *    traffic and no decode;
 *  - `setKeepAliveDecoding`'s doc says the decoder pauses when nothing references
 *    a Surface, a `ReceiveStreamListener` or a `CameraFrameListener` — so holding
 *    this listener keeps the pipeline warm rather than fighting it.
 *
 * The callback body does no work: increment, store primitives, return
 * (docs/architecture.md).
 */
internal class CameraStreamTap(
    private val cameraIndex: ComponentIndexType,
    private val sink: RawFrameSink? = null,
) {

    private val frames = AtomicLong()
    private val bytes = AtomicLong()
    private val keyFrames = AtomicLong()

    @Volatile private var width: Int? = null
    @Volatile private var height: Int? = null
    @Volatile private var mime: String? = null
    @Volatile private var frameRate: Int? = null
    @Volatile private var lastFrameAtMs: Long? = null

    @Volatile
    var registered: Boolean = false
        private set

    private val listener = ICameraStreamManager.ReceiveStreamListener { data, offset, length, info ->
        onStream(data, offset, length, info)
    }

    private fun onStream(data: ByteArray?, offset: Int, length: Int, info: StreamInfo?) {
        val n = frames.incrementAndGet()
        bytes.addAndGet(length.toLong().coerceAtLeast(0))
        lastFrameAtMs = SystemClock.elapsedRealtime()

        if (info != null) {
            width = info.width
            height = info.height
            frameRate = info.frameRate
            mime = runCatching { info.mimeType?.name }.getOrNull()
            if (info.isKeyFrame) keyFrames.incrementAndGet()
        }

        // First frame is the single most useful log line in this milestone: it
        // proves the lens/source choice was right.
        if (n == 1L) {
            Log.i(
                TAG,
                "first frame from $cameraIndex: ${info?.width}x${info?.height} " +
                    "@${info?.frameRate} ${runCatching { info?.mimeType?.name }.getOrNull()} len=$length",
            )
        }

        val s = sink
        if (s != null && data != null) {
            // Path 2's entry point. Nothing is attached today.
            runCatching {
                s.onEncodedFrame(
                    data, offset, length,
                    RawFrameInfo(
                        mime = runCatching { info?.mimeType?.name }.getOrNull(),
                        width = info?.width ?: 0,
                        height = info?.height ?: 0,
                        frameRate = info?.frameRate ?: 0,
                        keyFrame = info?.isKeyFrame == true,
                        presentationTimeMs = info?.presentationTimeMs ?: 0L,
                    ),
                )
            }.onFailure { Log.w(TAG, "raw frame sink threw", it) }
        }
    }

    /**
     * Must only be called after MSDK registration completes.
     *
     * ## `setKeepAliveDecoding(true)` is not optional, and the docs are wrong
     *
     * **Measured on hardware, 2026-07-26**: without it the aircraft delivers
     * frames for about four seconds and then stops dead. `tapFrames` froze at 98
     * with the listener still registered, no error anywhere, and `phase=SERVING`
     * on the screen — the single most misleading state this package can produce.
     *
     * `djidoc ICameraStreamManager` (and, until now, `docs/video.md`) says the
     * decoder pauses when nothing references "a Surface, a `ReceiveStreamListener`
     * or a `CameraFrameListener`", from which it follows that holding this
     * listener keeps the pipeline warm. **The bytecode says otherwise.**
     * `CameraStreamManager.lambda$updateAllCameraStream$6`, offsets 210–234:
     *
     * ```
     * 210: StreamDecoder.getFrameDispatcher().hasOutputTarget()
     * 218: ifne 237                 -> keep the stream
     * 221: getfield mKeepAliveDecoding
     * 225: ifne 237                 -> keep the stream
     * 228: StreamObserver.updateStreamSource(null)   <- otherwise, tear it down
     * ```
     *
     * The stream survives only if `hasOutputTarget()` — which iterates
     * `GLFrameDispatcher.mGLImageReaderList`, i.e. Surfaces handed to
     * `putCameraStreamSurface` — or if `mKeepAliveDecoding` is set. Listeners are
     * not consulted at all. `updateAllCameraStream` runs on every airlink event,
     * so any app that renders no preview loses the stream at the first one.
     *
     * A GCS bridge has no Surface by construction, so this flag *is* the
     * passthrough's licence to exist. It is set before the listener is added
     * because both calls end in `updateAllCameraStream()`, and this way the first
     * of those already sees the flag.
     */
    fun start() {
        if (registered) return
        val manager = MediaDataCenter.getInstance().cameraStreamManager
        // Failing to set this is not fatal *yet* — frames flow for a few seconds —
        // so it must not abort the subscription, but it must be shouted about,
        // because the eventual silence looks like a hardware fault.
        runCatching { manager.setKeepAliveDecoding(true) }
            .onSuccess { Log.i(TAG, "keep-alive decoding enabled (no Surface; see start())") }
            .onFailure { Log.e(TAG, "setKeepAliveDecoding(true) failed — expect frames to stop after a few seconds", it) }
        manager.addReceiveStreamListener(cameraIndex, listener)
        registered = true
        Log.i(TAG, "receive-stream listener registered for $cameraIndex")
    }

    fun stop() {
        if (!registered) return
        val manager = MediaDataCenter.getInstance().cameraStreamManager
        runCatching { manager.removeReceiveStreamListener(listener) }
            .onFailure { Log.w(TAG, "removeReceiveStreamListener threw", it) }
        // Hand the decoder back. The flag is process-global (a plain field on
        // MSDK's singleton), and leaving it set would keep the aircraft's video
        // pipeline warm for the rest of the session with nothing consuming it.
        runCatching { manager.setKeepAliveDecoding(false) }
            .onFailure { Log.w(TAG, "setKeepAliveDecoding(false) threw", it) }
        registered = false
        Log.i(TAG, "receive-stream listener removed; ${frames.get()} frames / ${bytes.get()} bytes seen")
    }

    fun counters(): TapCounters = TapCounters(
        frames = frames.get(),
        bytes = bytes.get(),
        width = width,
        height = height,
        mime = mime,
        frameRate = frameRate,
        keyFrames = keyFrames.get(),
        lastFrameAtMs = lastFrameAtMs,
    )

    private companion object {
        const val TAG = "VideoTap"
    }
}
