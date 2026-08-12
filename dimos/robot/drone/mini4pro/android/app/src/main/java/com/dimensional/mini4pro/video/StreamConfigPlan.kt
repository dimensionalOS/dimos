package com.dimensional.mini4pro.video

/**
 * The order in which MSDK's live-stream setters must be called, as data.
 *
 * ## The hazard
 *
 * `ILiveStreamManager.setLiveStreamQuality` is not the innocent setter it looks
 * like. From the bytecode (`javap -p -c
 * dji.v5.manager.datacenter.livestream.LiveStreamManager`):
 *
 * ```
 * public void setLiveStreamQuality(StreamQuality);
 *   VideoBitRateManager.getInstance().stopAutoVideoBitRate()
 *   this.mLiveVideoBitRateMode = LiveVideoBitrateMode.MANUAL      <-- !
 *   MRTCManager.getInstance().updateLiveStreamQuality(quality)
 * ```
 *
 * It forces the bitrate mode to `MANUAL` and stops the automatic bitrate
 * controller, whatever you set before it. So `setLiveVideoBitrateMode(AUTO)`
 * followed by `setLiveStreamQuality(HD)` leaves the stream on MANUAL at
 * `LiveStreamManager`'s constructor default — and reports nothing. The only
 * signal is a bitrate that never adapts, which on a marginal link looks like the
 * aircraft's fault.
 *
 * Nothing in the API says this, and nothing in the docs does either. Keeping the
 * order here, with a test that asserts it, is what stops a later tidy-up from
 * reintroducing it.
 *
 * (For the same reason `setLiveVideoBitrate(int)` also forces `MANUAL` — so the
 * manual branch sets the mode and then the value, which agree.)
 *
 * This plan is only used in [VideoStreamer.VideoMode.RTSP]. The passthrough path
 * has no encoder to configure, which is the point of it.
 */
object StreamConfigPlan {

    enum class Step {
        /** `setCameraIndex(LEFT_OR_MAIN)`. */
        CAMERA_INDEX,

        /** `setLiveStreamSettings(RTSP + RtspSettings)`. */
        SETTINGS,

        /** `setLiveStreamQuality(...)`. **Must precede any bitrate step.** */
        QUALITY,

        /** `setLiveVideoBitrateMode(AUTO)`. */
        BITRATE_MODE_AUTO,

        /** `setLiveVideoBitrateMode(MANUAL)`. */
        BITRATE_MODE_MANUAL,

        /** `setLiveVideoBitrate(bps)`. */
        BITRATE_VALUE,
    }

    data class Plan(val steps: List<Step>) {
        /** Human-readable, for the log line that precedes `startStream`. */
        override fun toString(): String = steps.joinToString(" → ") { it.name }
    }

    /**
     * @param bitrateBps null selects MSDK's automatic bitrate; a value selects
     *   manual at that rate.
     */
    fun of(quality: VideoQuality, bitrateBps: Int?): Plan {
        // `quality` is not read here — it selects the value passed to the QUALITY
        // step, not whether the step happens — but it stays in the signature so
        // the plan reads as "the configuration", not "the bitrate decision".
        val steps = mutableListOf(Step.CAMERA_INDEX, Step.SETTINGS, Step.QUALITY)
        if (bitrateBps == null) {
            steps += Step.BITRATE_MODE_AUTO
        } else {
            require(bitrateBps > 0) { "bitrate ${bitrateBps}bps is not a rate" }
            steps += Step.BITRATE_MODE_MANUAL
            steps += Step.BITRATE_VALUE
        }
        return Plan(steps)
    }
}
