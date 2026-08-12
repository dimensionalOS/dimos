package com.dimensional.mini4pro.vision

import android.os.SystemClock
import android.util.Log
import com.dimensional.mini4pro.Msdk
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import java.util.concurrent.atomic.AtomicLong

/**
 * **The only file in `vision/` that imports DJI.** `ICameraStreamManager.addFrameListener` →
 * [FrameListener], and nothing else.
 *
 * ## Why this rather than the path the recorder already uses
 *
 * `video/CameraStreamTap` subscribes with `addReceiveStreamListener`, which hands over the
 * aircraft's **encoded** H.264 untouched. That is exactly right for a passthrough that must not
 * decode, and useless to a detector, which needs pixels. `addFrameListener` is MSDK's decoded tap:
 * it takes a [ICameraStreamManager.FrameFormat] and delivers `onFrame(byte[], offset, length,
 * width, height, format)`.
 *
 * **`NV21` is the format asked for, and plane 0 is the whole reason.** NV21 is Y plane first,
 * `width * height` tightly packed bytes, then interleaved VU. The detector wants exactly that first
 * plane and never looks at chroma, so the luminance handed to [FrameListener] is a subrange of
 * MSDK's own buffer with no conversion and no copy on this side of the seam. Asking for `RGBA_8888`
 * would have cost a colour conversion in MSDK and a greyscale conversion here, for the same pixels.
 *
 * **−1, −1 for the size.** MSDK scales when given a size, and scaling is the one thing the
 * measurements say not to do: halving each dimension halves the working height, exactly as the
 * pixel argument predicts (`docs/measurements/2026-07-27-tag-detection-rate.md` §2). So the stream's
 * own 1920×1080 arrives and any reduction is a deliberate decision made above this class, where it
 * can be measured. Note MSDK's own signature has no size arguments at all — the geometry comes back
 * *in* the callback — so there is nothing to pass; this paragraph records that the absence is
 * checked rather than overlooked.
 *
 * ## `setKeepAliveDecoding` is not optional, and a frame listener is not a Surface
 *
 * `video/CameraStreamTap.start` documents this from bytecode and from hardware: MSDK tears the
 * stream down on any airlink event unless `hasOutputTarget()` — Surfaces handed to
 * `putCameraStreamSurface` — or `mKeepAliveDecoding`. **Listeners are not consulted at all.** A GCS
 * bridge has no Surface by construction, so the flag is the stream's licence to exist and it is set
 * here too, before the listener is added, for the same reason: both calls end in
 * `updateAllCameraStream()` and the first should already see the flag.
 *
 * The flag is process-global — a plain field on MSDK's singleton — and `CameraStreamTap` sets and
 * clears it as well. [stop] therefore **does not clear it**: clearing a flag another subscriber is
 * relying on would kill the video passthrough to save a detector's decode, which is precisely
 * backwards. The tap owns the flag's lifetime because the tap outlives this.
 *
 * ## What is covered by tests, and what is not
 *
 * **Nothing in this file is unit-tested and nothing in it can be.** It is a DJI adapter, and it is
 * deliberately shaped so there is nothing in it to test: it holds no policy, no buffer, no clock
 * arithmetic and no branch beyond "is MSDK ready". Everything that could be wrong in an interesting
 * way is on the other side of [FrameSource]. What only the aircraft can settle — the delivery rate,
 * the dispatch thread, and whether occupying that thread starves the passthrough — is measured by
 * `vision/FrameListenerProbeTest` in `src/androidTest`, which is the instrumented run this class
 * exists to make possible.
 */
class MsdkFrameSource(
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val log: (String) -> Unit = { Log.i(TAG, it) },
) : FrameSource {

    private val frames = AtomicLong()

    @Volatile private var listener: ICameraStreamManager.CameraFrameListener? = null

    override fun start(listener: FrameListener): String? {
        if (this.listener != null) return null
        val state = Msdk.state.value
        // Ordered from the outermost cause inwards, so the sentence names the thing to fix rather
        // than the thing that noticed. Touching MediaDataCenter before registration silently does
        // nothing, which is the failure this project's architecture note warns about by name.
        if (!state.registered) return "MSDK is not registered yet"
        if (!state.productConnected) return "no aircraft connected"

        val dji = ICameraStreamManager.CameraFrameListener { data, offset, length, width, height, _ ->
            // **First, before anything can fail**, and on the same clock as every flight-record
            // entry. A sighting is 60–160 ms old by the time a 25 Hz loop sees it, and the only
            // thing that makes that age recoverable rather than a guess is that this number was
            // taken here.
            val at = SystemClock.elapsedRealtimeNanos()
            val n = frames.incrementAndGet()
            // **The single most useful line about this path in a session**, and the same move
            // `video/CameraStreamTap` makes for the encoded stream: it separates "the decoded tap
            // never delivered" from "it delivered and the detector was disarmed", which are the two
            // explanations for a session with no sightings and want opposite responses. It also
            // prints the one thing that would silently break the detector if it ever changed —
            // `length` against `w·h·3/2`, which is what says the luminance plane is tightly packed.
            if (n == 1L) {
                log("first decoded frame: ${width}x$height len=$length offset=$offset " +
                    "expect_nv21=${width * height * 3 / 2} on ${Thread.currentThread().name}")
            }
            // The luminance plane is width*height of the length MSDK reports; a frame that does not
            // carry at least that much is not one this can read, and reading it anyway would hand
            // the detector whatever followed in the buffer.
            val need = width * height
            if (data != null && width > 0 && height > 0 && length >= need) {
                listener.onLuma(data, offset, width, height, at)
            }
        }
        val manager = try {
            MediaDataCenter.getInstance().cameraStreamManager
        } catch (e: Throwable) {
            return "camera stream manager unavailable: ${e.message}"
        }
        // See the KDoc: without this the aircraft stops delivering at the first airlink event, and
        // the failure looks exactly like a hardware fault. Not fatal here — frames do flow for a
        // few seconds — so it is shouted about rather than fatal, as `CameraStreamTap` does.
        runCatching { manager.setKeepAliveDecoding(true) }
            .onFailure { log("setKeepAliveDecoding(true) failed — expect frames to stop: $it") }
        return try {
            manager.addFrameListener(cameraIndex, ICameraStreamManager.FrameFormat.NV21, dji)
            this.listener = dji
            log("frame listener registered for $cameraIndex as NV21")
            null
        } catch (e: Throwable) {
            "addFrameListener refused: ${e.message}"
        }
    }

    override fun stop() {
        val dji = listener ?: return
        listener = null
        runCatching { MediaDataCenter.getInstance().cameraStreamManager.removeFrameListener(dji) }
            .onFailure { log("removeFrameListener threw: $it") }
        // Deliberately **not** clearing setKeepAliveDecoding — see the KDoc. `CameraStreamTap` owns
        // that flag's lifetime and the video passthrough depends on it.
        log("frame listener removed after ${frames.get()} frame(s)")
    }

    /** Frames MSDK delivered, whether or not the listener could read them. For the status screen. */
    fun framesDelivered(): Long = frames.get()

    private companion object {
        const val TAG = "TagFrames"
    }
}
