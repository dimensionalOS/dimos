package com.dimensional.mini4pro.video

/**
 * Where the video path is, and why.
 *
 * Deliberately more granular than "running": every failure mode in this path is
 * silent, so the phase names the precondition we are stuck behind rather than
 * leaving the operator to guess.
 */
enum class VideoPhase {
    /** [VideoStreamer.start] has not been called, or [VideoStreamer.stop] was. */
    STOPPED,

    /** Waiting for `Msdk.state.registered`. Nothing has touched the SDK yet. */
    WAITING_REGISTRATION,

    /** Registered; waiting for the aircraft on the RC-N2 cable. */
    WAITING_AIRCRAFT,

    /** Aircraft up; waiting for MSDK to report the camera. */
    WAITING_CAMERA,

    /** Configuring the live-stream manager / `startStream()` in flight. */
    STARTING,

    /**
     * MSDK's RTSP server is up and `isStreaming()` is true. This does **not**
     * mean a client is pulling frames — check [VideoStatus.fps] and
     * [VideoStatus.zeroBitrateSamples] for that.
     */
    SERVING,

    /** `startStream()` failed, or the configuration is unusable. Retrying. */
    FAILED,
}

/** Our own quality enum so DJI types stay inside this package. */
enum class VideoQuality {
    /** 960x540, 30 fps, ~74 KB/s (per `djidoc ILiveStreamManager`). */
    SD,

    /** 1280x720, 30 fps, ~168 KB/s. The default: enough for a GCS FPV view. */
    HD,

    /** 1920x1080, 30 fps, ~380 KB/s. */
    FULL_HD,

    /**
     * Present in `StreamQuality` but undocumented on the doc page — the doc
     * lists only SD/HD/FULL_HD. Unverified on hardware.
     */
    ORIGINAL,
}

/**
 * Counters from the raw-stream tap. This is the honest answer to "are frames
 * actually arriving from the aircraft?", independent of whether the RTSP server
 * has a client. All monotonic — a positive number is proof, a boolean is not.
 */
data class TapCounters(
    val frames: Long = 0,
    val bytes: Long = 0,
    val width: Int? = null,
    val height: Int? = null,
    /** `H264` or `H265`, from `StreamInfo.mimeType`. */
    val mime: String? = null,
    val frameRate: Int? = null,
    val keyFrames: Long = 0,
    /** Elapsed-realtime ms of the last frame, for staleness. */
    val lastFrameAtMs: Long? = null,
)

/**
 * The whole observable state of the video path. Plain Kotlin — no DJI, no
 * Android — so it can be rendered, logged and tested anywhere.
 */
data class VideoStatus(
    val phase: VideoPhase = VideoPhase.STOPPED,

    /** `RTP`, `RTSP` or `BOTH` — see `VideoStreamer.VideoMode`. */
    val mode: String? = null,

    /** `host:port` the RTP passthrough is pushing to. Null unless RTP is on. */
    val rtpTarget: String? = null,

    /** Counters from [RtpVideoSink]. Null unless the passthrough path is running. */
    val rtp: RtpVideoSink.Counters? = null,

    /** URL to hand a GCS, credentials included. Null until the server is configured. */
    val rtspUrl: String? = null,

    /** Same URL with `***:***` for the userinfo, safe for logcat and the UI. */
    val rtspUrlRedacted: String? = null,

    /** Local IPv4 the URL was built from, and the interface it came from. */
    val localAddress: String? = null,
    val localInterface: String? = null,

    // ---- what we configured, so a wrong lens is visible rather than inferred ----
    /** `ComponentIndexType` name, e.g. `LEFT_OR_MAIN`. */
    val cameraIndex: String? = null,
    /** `CameraVideoStreamSourceType` name as last read back from the airframe. */
    val streamSource: String? = null,
    /** `KeyCameraVideoStreamSourceRange`, as read. Empty means unreadable. */
    val streamSourceRange: List<String> = emptyList(),
    /** Non-null when [StreamSource.plan] had something to say. */
    val sourceWarning: String? = null,
    val quality: VideoQuality? = null,
    /** Null means MSDK AUTO bitrate mode. */
    val requestedBitrateBps: Int? = null,

    // ---- listener bookkeeping: "registered" is a precondition people forget ----
    val liveStreamListenerRegistered: Boolean = false,
    val availableCameraListenerRegistered: Boolean = false,
    val frameTapRegistered: Boolean = false,
    /** Camera indices MSDK reported as available. */
    val availableCameras: List<String> = emptyList(),
    /** `CameraKey.KeyConnection` for our camera index; null = unread. */
    val cameraKeyConnected: Boolean? = null,

    // ---- positive counters, never a bare boolean ----
    /** `LiveStreamStatus` callbacks seen. Zero here means MSDK is saying nothing. */
    val statusUpdates: Long = 0,
    /** `ILiveStreamManager.isStreaming()` as last polled. */
    val streamingReported: Boolean = false,
    val fps: Int? = null,
    val bitrateBps: Int? = null,
    val width: Int? = null,
    val height: Int? = null,
    val packetLossPercent: Int? = null,
    val rttMs: Int? = null,
    val packetCacheLen: Int? = null,
    /**
     * Consecutive 1 Hz samples where the server was up but the bitrate was zero.
     * This is what "RTSP server running, nobody connected" looks like.
     */
    val zeroBitrateSamples: Long = 0,

    val tap: TapCounters = TapCounters(),

    // ---- failure surface ----
    val startAttempts: Long = 0,
    val lastError: String? = null,
    /** Seconds the phase has held, so a stall is visible without a stopwatch. */
    val phaseHeldSeconds: Long = 0,
) {

    /** True only when frames have demonstrably arrived from the aircraft. */
    val framesArriving: Boolean get() = tap.frames > 0

    /**
     * The two counters docs/video.md's failure table is built on, always both
     * present and always both named, even when one of them has no source.
     *
     * They answer different questions and fail independently: `tapFrames` is *the
     * aircraft is delivering to us* and `rtpPkts` is *we are transmitting*. Every
     * row of that table is a different pair of these two, so a reader who has one
     * number has nothing. The RTP line prints `—` rather than `0` when there is no
     * sink at all, because "no socket was ever opened" and "the socket has sent
     * nothing" are two of the rows.
     *
     * On the status screen this sits above the detail, so the diagnosis is
     * available at arm's length on a phone strapped to a controller — the whole
     * point being that neither number should ever require adb to read.
     */
    fun counterLines(): String = buildString {
        appendLine("tapFrames:  ${tap.frames}  ← the aircraft delivering to us")
        appendLine(
            "rtpPkts:    ${rtp?.packetsSent?.toString() ?: "—"}  → us transmitting to " +
                (rtpTarget ?: "no target"),
        )
    }

    /** True when the server is up and something is actually pulling from it. */
    val clientPulling: Boolean get() = phase == VideoPhase.SERVING && (bitrateBps ?: 0) > 0

    /** True when the passthrough path is the one carrying video. */
    val rtpActive: Boolean get() = rtp != null

    /**
     * Whether MSDK's RTSP server is part of this session at all.
     *
     * Errs towards showing the RTSP rows: an unknown mode (nothing started yet)
     * counts as in play, because hiding a row is only safe when we know it is
     * irrelevant. `BOTH` obviously counts.
     */
    private val rtspInPlay: Boolean get() = mode != "RTP" || rtspUrl != null

    /**
     * What to configure in the GCS, in the GCS's own words. Null when there is
     * nothing to configure yet.
     *
     * Both strings name settings verified against QGC's source: `videoSourceRTSP
     * = "RTSP Video Stream"` and `videoSourceUDPH264 = "UDP h.264 Video Stream"`
     * (`src/Settings/VideoSettings.h`), with `udpUrl` defaulting to `0.0.0.0:5600`.
     */
    val gcsHint: String?
        get() = when {
            rtpTarget != null -> {
                val port = rtpTarget.substringAfterLast(':', "5600")
                "QGC: Source = \"UDP h.264 Video Stream\", UDP Port = 0.0.0.0:$port"
            }
            rtspUrlRedacted != null ->
                "QGC: Source = \"RTSP Video Stream\", RTSP URL = ${rtspUrlRedacted}"
            else -> null
        }

    /**
     * One-line summary for logcat. Terse on purpose: this line is printed once a
     * second and has to be greppable.
     */
    fun oneLine(): String = buildString {
        append(phase.name)
        mode?.let { append(" mode=").append(it) }
        append(" src=").append(streamSource ?: "?")
        append(" cam=").append(cameraIndex ?: "?")
        append(" tapFrames=").append(tap.frames)
        append(" tapBytes=").append(tap.bytes)
        append(" statusUpdates=").append(statusUpdates)
        append(" streaming=").append(streamingReported)
        append(" fps=").append(fps ?: -1)
        append(" vbps=").append(bitrateBps ?: -1)
        append(" res=").append(width ?: -1).append('x').append(height ?: -1)
        rtp?.let {
            append(" rtpPkts=").append(it.packetsSent)
            append(" rtpBytes=").append(it.bytesSent)
            if (it.framesDropped > 0) append(" rtpDropped=").append(it.framesDropped)
            if (it.sendErrors > 0) append(" rtpErrors=").append(it.sendErrors)
        }
        if (zeroBitrateSamples > 0) append(" zeroBitrate=").append(zeroBitrateSamples)
        if (packetLossPercent != null) append(" loss=").append(packetLossPercent)
        if (rttMs != null) append(" rtt=").append(rttMs)
        append(" url=").append(rtspUrlRedacted ?: "-")
        lastError?.let { append(" err=").append(it) }
    }

    /**
     * Multi-line block for the app's status screen, in the same shape as
     * `MainActivity`'s other sections. Includes the verdict line, because
     * "it isn't working" is the question this text exists to answer.
     */
    fun describe(): String = buildString {
        appendLine("phase:      ${phase.name}${if (phaseHeldSeconds > 0) " (${phaseHeldSeconds}s)" else ""}")
        appendLine("mode:       ${mode ?: "—"}")
        // Before the detail, because these two are the diagnosis and the rest is
        // supporting evidence.
        append(counterLines())
        // RTSP-only rows, hidden on the passthrough path. Not tidiness: on hardware
        // this block reads `url: —`, `local ip: —` and `live: updates=0 streaming=no
        // fps=— vbps=—` while RTP is streaming perfectly, which is four lines of
        // apparent failure above a working stream. `live` comes from
        // LiveStreamStatus, which never fires when MSDK's server is not running,
        // so in RTP mode it is not a zero — it is a question nobody asked.
        if (rtspInPlay) appendLine("url:        ${rtspUrlRedacted ?: "—"}")
        rtp?.let {
            appendLine("rtp →       ${rtpTarget ?: "—"}")
            appendLine(
                "rtp:        ${it.packetsSent} pkts / ${it.bytesSent} B  " +
                    "frames ${it.framesSent}/${it.framesIn}  dropped=${it.framesDropped} " +
                    "errors=${it.sendErrors} spsInjected=${it.parameterSetsInjected}"
            )
            it.lastError?.let { e -> appendLine("rtp error:  $e") }
        }
        if (rtspInPlay) {
            appendLine("local ip:   ${localAddress ?: "—"}${localInterface?.let { " ($it)" } ?: ""}")
        }
        appendLine("camera:     ${cameraIndex ?: "—"}  source=${streamSource ?: "—"}")
        if (streamSourceRange.isNotEmpty()) appendLine("src range:  $streamSourceRange")
        sourceWarning?.let { appendLine("src warn:   $it") }
        appendLine(
            "listeners:  status=${yn(liveStreamListenerRegistered)} " +
                "cameras=${yn(availableCameraListenerRegistered)} tap=${yn(frameTapRegistered)}"
        )
        appendLine("cameras:    ${availableCameras.ifEmpty { listOf("—") }}  key=${cameraKeyConnected ?: "?"}")
        appendLine("tap:        ${tap.frames} frames / ${tap.bytes} B  ${tap.mime ?: "—"} ${dim()}")
        if (rtspInPlay) {
            appendLine("live:       updates=$statusUpdates streaming=${yn(streamingReported)} fps=${fps ?: "—"} vbps=${bitrateBps ?: "—"}")
        }
        if (zeroBitrateSamples > 0) appendLine("no client:  ${zeroBitrateSamples}s at zero bitrate")
        if (packetLossPercent != null || rttMs != null) {
            appendLine("net:        loss=${packetLossPercent ?: "—"}% rtt=${rttMs ?: "—"}ms cache=${packetCacheLen ?: "—"}")
        }
        if (startAttempts > 0) appendLine("attempts:   $startAttempts")
        lastError?.let { appendLine("error:      $it") }
        gcsHint?.let { appendLine("configure:  $it") }
        appendLine("verdict:    ${verdict()}")
    }

    /**
     * The diagnosis, spelled out. Each branch corresponds to one of the silent
     * failure modes this milestone is prone to.
     */
    fun verdict(): String = when {
        phase == VideoPhase.STOPPED -> "not started"
        phase == VideoPhase.WAITING_REGISTRATION ->
            "waiting for MSDK registration — nothing may touch the SDK yet"
        phase == VideoPhase.WAITING_AIRCRAFT ->
            "registered, no aircraft — check the RC-N2 cable and that DJI Fly is force-stopped"
        phase == VideoPhase.WAITING_CAMERA ->
            "aircraft up, MSDK has not reported the camera yet"
        phase == VideoPhase.FAILED -> "failed: ${lastError ?: "unknown"}"
        phase == VideoPhase.STARTING -> "configuring / startStream in flight"
        // SERVING from here down.
        //
        // The grace matters. On the passthrough path SERVING is reached the
        // instant `addReceiveStreamListener` returns, so for the first second or
        // two zero frames is simply "no frame has arrived yet" — and a verdict
        // that immediately blamed the lens would be crying wolf at exactly the
        // moment an operator is watching it hardest. After the grace the same
        // zero is a diagnosis, and it says how long it has been true so nobody has
        // to reach for a stopwatch to tell the two apart.
        tap.frames == 0L && frameTapRegistered && phaseHeldSeconds < FIRST_FRAME_GRACE_S ->
            "subscribed to the camera stream; waiting for the first frame"
        tap.frames == 0L && frameTapRegistered ->
            "SERVING but zero frames from the aircraft for ${phaseHeldSeconds}s — suspect the lens/source " +
                "(source=${streamSource ?: "?"}; must be ${StreamSource.SINGLE_LENS}) or a camera in playback mode"
        // The passthrough path has its own failure surface: the aircraft can be
        // delivering frames perfectly while every datagram to the GCS is refused.
        rtp != null && rtp.framesSent == 0L && rtp.sendErrors > 0 ->
            "frames arriving but every RTP send failed (${rtp.lastError ?: "no detail"}) — " +
                "check the target $rtpTarget is routable from the phone"
        rtp != null && rtp.packetsSent == 0L ->
            "frames arriving but nothing packetised yet — if this persists the frames " +
                "carry no NAL start codes, which is not H.264 Annex-B"
        rtp != null ->
            "passing through ${rtp.packetsSent} packets / ${rtp.bytesSent / 1024} KiB to " +
                "$rtpTarget, no re-encode" +
                (if (rtp.framesDropped > 0) " (${rtp.framesDropped} frames dropped)" else "") +
                (if (rtp.sendErrors > 0) " (${rtp.sendErrors} send errors)" else "")
        statusUpdates == 0L ->
            "SERVING but no LiveStreamStatus callbacks — the status listener never fired"
        (bitrateBps ?: 0) == 0 ->
            "server up, no client pulling — point the GCS at the URL (QGC: video source " +
                "\"RTSP Video Stream\", RTSP URL = the url above)"
        else -> "streaming ${width ?: "?"}x${height ?: "?"} @ ${fps ?: "?"} fps, ${(bitrateBps ?: 0) / 1000} kbps"
    }

    private fun dim(): String =
        if (tap.width != null && tap.height != null) "${tap.width}x${tap.height}" else ""

    private fun yn(b: Boolean) = if (b) "yes" else "no"

    companion object {
        /**
         * How long SERVING with no frames is merely "not yet" rather than a fault.
         *
         * At 30 fps a frame is due every 33 ms, so this is generous by two orders
         * of magnitude — it is sized for the aircraft's first keyframe after a
         * source switch, not for the frame interval.
         */
        const val FIRST_FRAME_GRACE_S = 3L
    }
}

/**
 * Codes for the flight recorder, so a session that produced no video says why in
 * the file rather than only in a logcat buffer that is gone by the time anyone
 * asks.
 *
 * Named here, in the DJI-free half, for the same reason `SimulatorNotice` names
 * its own: the emitting code should not have to import `record/` to describe
 * itself, and the reader of a log should find the vocabulary next to the thing
 * that speaks it.
 */
object VideoEvents {
    /** Every [VideoPhase] transition. `info`. */
    const val PHASE = "video_phase"

    /** A failure the operator has to act on. `error`. */
    const val FAILED = "video_failed"

    /** The RTP socket was rebuilt onto a new network. `warn`. */
    const val REBIND = "video_rebind"
}
