package com.dimensional.mini4pro.video

/**
 * Whether video is wanted this session, and where it is pushed — as a pure
 * function of the launch intent, the saved settings and the MAVLink target.
 *
 * This exists because the answer has four inputs that disagree in interesting
 * ways, and because the last time it was left implicit the result was a flag
 * (`--ez video true`) that `tools/session video` had been passing for weeks into
 * an app that read no such extra. Nothing noticed, because "video did not appear"
 * and "video was never asked for" look identical from the outside. Putting the
 * resolution here means every rule below is a test rather than a habit.
 *
 * ## The rules, and why
 *
 * **Off unless asked.** A camera stream is bandwidth and battery an operator did
 * not request. `enabled` is persisted, so it is asked for once per phone rather
 * than once per launch, and the launch intent can override it in either direction
 * — `--ez video false` genuinely turns it off, which is what makes the flag
 * trustworthy.
 *
 * **The destination follows the MAVLink target by default**, and that is the
 * important one. The relay on hyper1 carries `:14550` and `:5600` in the *same
 * process* (docs/video.md, "The video relay"), so the two streams have exactly
 * one correct address between them. Giving video its own independent default
 * would create a second address to get wrong — and a wrong video address is
 * silent for a whole session, whereas a wrong telemetry address announces itself
 * within seconds because QGC never connects. One address, one mistake, and the
 * mistake is visible.
 *
 * **The fallback is the relay, never the laptop.** With nothing else known,
 * [RELAY_HOST]. `wifi-fix.md`: phone→laptop unicast is blackholed by the AP, in
 * every radio combination tried. That is why MAVLink relays, and it applies
 * identically to RTP. Any host that is not the relay earns a [Plan.warning] —
 * not a refusal, because a different network is a legitimate thing to be on, but
 * never silence.
 */
object VideoRequest {

    /** hyper1, the wired relay. The one address that is known to work from the phone. */
    const val RELAY_HOST = "10.55.1.50"

    /** QGroundControl's default `udpUrl` is `0.0.0.0:5600`; the relay forwards to it. */
    const val DEFAULT_PORT = RtpVideoSink.DEFAULT_PORT

    // Persisted in the app's SharedPreferences, alongside `gcs_host` / `gcs_port`.
    const val PREF_ENABLED = "video_enabled"
    const val PREF_HOST = "video_host"
    const val PREF_PORT = "video_port"

    // Launch extras. `tools/session video` has been sending EXTRA_ENABLED since
    // long before anything read it.
    const val EXTRA_ENABLED = "video"
    const val EXTRA_HOST = "videoHost"
    const val EXTRA_PORT = "videoPort"

    /** Where [Plan.host] came from. Displayed, so a surprising target is explicable. */
    enum class Source {
        /** `--es videoHost` on this launch. */
        INTENT,

        /** The saved video-host override. */
        SAVED,

        /** No override: the same address telemetry is going to. The normal case. */
        FOLLOWS_GCS,

        /** Nothing known at all, so [RELAY_HOST]. */
        DEFAULT_RELAY,
    }

    /**
     * @param warning non-null when the target is worth a second look before a
     *   session starts. Never fatal — being on a different network is legitimate.
     */
    data class Plan(
        val enabled: Boolean,
        val host: String,
        val port: Int,
        val source: Source,
        val warning: String? = null,
    ) {
        val target: String get() = "$host:$port"
    }

    /**
     * @param intentEnabled `--ez video` on this launch, or null when the extra is
     *   absent. Null and `false` are deliberately different: absent means "leave
     *   the saved setting alone", `false` means "turn it off".
     * @param gcsHost where the MAVLink bridge is pointed, or null/blank if unset.
     */
    fun resolve(
        intentEnabled: Boolean? = null,
        savedEnabled: Boolean = false,
        intentHost: String? = null,
        savedHost: String? = null,
        gcsHost: String? = null,
        intentPort: Int? = null,
        savedPort: Int? = null,
    ): Plan {
        val enabled = intentEnabled ?: savedEnabled
        val (host, source) = when {
            !intentHost.isNullOrBlank() -> intentHost.trim() to Source.INTENT
            !savedHost.isNullOrBlank() -> savedHost.trim() to Source.SAVED
            !gcsHost.isNullOrBlank() -> gcsHost.trim() to Source.FOLLOWS_GCS
            else -> RELAY_HOST to Source.DEFAULT_RELAY
        }
        val port = (intentPort ?: savedPort ?: DEFAULT_PORT).let {
            if (it in 1..65535) it else DEFAULT_PORT
        }
        return Plan(enabled = enabled, host = host, port = port, source = source, warning = warn(host))
    }

    /**
     * The one sentence that stands between an operator and a silent session.
     *
     * Deliberately phrased as an observation rather than an error: on a different
     * network a laptop address is exactly right. On *this* one it is the single
     * most likely reason nothing appears, and it costs nothing to say so.
     */
    private fun warn(host: String): String? =
        if (host == RELAY_HOST) null
        else "video is aimed at $host, not the relay $RELAY_HOST — phone→laptop unicast is " +
            "blackholed by this AP (wifi-fix.md), so a direct laptop address usually produces silence"

    /**
     * The status-screen line for the setting itself, as distinct from what the
     * stream is doing.
     *
     * The two have to be separable. "Video is on" and "video is running" differ
     * for a whole class of reasons the video path itself cannot report — the
     * bridge was never started, or it refused to start because there was no WiFi
     * to bind to — and in every one of them [VideoStreamer] is correctly sitting
     * in [VideoPhase.STOPPED] with nothing to say. A screen that showed only the
     * phase would read "not started" and leave the operator to guess whether that
     * meant "you did not ask for it" or "you asked and it did not happen".
     */
    fun statusLine(plan: Plan, phase: VideoPhase): String = when {
        !plan.enabled -> "off — turn it on in Settings to send the camera to the GCS"
        phase == VideoPhase.STOPPED ->
            "ON → ${plan.target}, NOT RUNNING — video starts and stops with the bridge"
        else -> "ON → ${plan.target} (${sourceNote(plan.source)})"
    }

    private fun sourceNote(source: Source): String = when (source) {
        Source.INTENT -> "from --es $EXTRA_HOST"
        Source.SAVED -> "saved video host"
        Source.FOLLOWS_GCS -> "same address as telemetry"
        Source.DEFAULT_RELAY -> "default relay"
    }
}
