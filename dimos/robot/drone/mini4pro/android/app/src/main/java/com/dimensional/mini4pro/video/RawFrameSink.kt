package com.dimensional.mini4pro.video

/**
 * The seam for the *second* video path (raw H.264 → RTP), kept DJI-free so that
 * path can be written and tested without the SDK.
 *
 * Path 1 (shipping now) is MSDK's built-in RTSP server: `ILiveStreamManager`
 * with `LiveStreamType.RTSP`, and QGC just takes a URL. Path 2, if latency
 * demands it, is `ICameraStreamManager.addReceiveStreamListener` → an RTP
 * packetizer → UDP straight to the GCS, which skips MSDK's RTSP server, its
 * re-encode and TCP-ish framing entirely.
 *
 * [CameraStreamTap] already subscribes to exactly the stream path 2 needs. Today
 * it only counts frames (that counter is the honest answer to "is the aircraft
 * delivering video at all?"), but anything implementing [RawFrameSink] can be
 * hung off it without touching DJI code. See docs/video.md, and
 * `ref/rosettadrone/app/src/main/java/sq/rogue/rosettadrone/video/` for a
 * ready-made H.264 RTP packetizer (libstreaming, Apache-2.0 — *not* RosettaDrone's
 * own BSD-3, check the headers before reusing).
 */
fun interface RawFrameSink {
    /**
     * Called on an MSDK callback thread with an Annex-B H.264/H.265 slice.
     *
     * Contract, because it is easy to get wrong: **do no work here.** [data] is
     * MSDK's buffer and must be treated as valid only for the duration of the
     * call — copy the `[offset, offset+length)` slice, hand it to your own queue
     * and return. Blocking here stalls MSDK's decode path.
     */
    fun onEncodedFrame(data: ByteArray, offset: Int, length: Int, info: RawFrameInfo)
}

/** Per-frame metadata, projected off DJI's `StreamInfo` into plain types. */
data class RawFrameInfo(
    /** `H264` or `H265`. The Mini 4 Pro measured `Camera.KeyVideoMimeType = H264`. */
    val mime: String?,
    val width: Int,
    val height: Int,
    val frameRate: Int,
    val keyFrame: Boolean,
    val presentationTimeMs: Long,
)
