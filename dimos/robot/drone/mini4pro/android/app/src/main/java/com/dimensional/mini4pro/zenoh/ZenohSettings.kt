package com.dimensional.mini4pro.zenoh

/**
 * Whether the Zenoh transport is wanted this session and which router it dials — as a pure
 * function of the launch intent and the saved settings.
 *
 * Written to the shape of `video/VideoRequest`, and for the reason that file gives: the last time
 * a resolution like this was left implicit, `tools/session video` passed `--ez video true` for
 * weeks into an app that read no such extra, and nothing noticed, because *"it did not appear"*
 * and *"it was never asked for"* look identical from the outside. Putting the rules here makes
 * each one a test instead of a habit.
 *
 * ## The rules
 *
 * **Off unless asked.** Zenoh is a second network stack and a JNI runtime in a process that flies
 * an aircraft, and `docs/zenoh-dimos-transport.md` §6.4 lists its thermal cost as unmeasured. It
 * is persisted, so it is asked for once per phone rather than once per launch, and the launch
 * intent overrides it in **either** direction — `--ez zenoh false` genuinely turns it off, which
 * is what makes the flag usable for ruling Zenoh out as the cause of something.
 *
 * **The router is hyper1, and it does not follow the GCS address.** This is the one place the
 * resolution deliberately differs from video's, which defaults to wherever telemetry is pointed.
 * Video is a *stream to the ground station*, so following it is right. Zenoh is a *bus*, and the
 * bus is a `zenohd` on hyper1 that DiMOS, a spy and a `dtop` all attach to independently — the
 * ground station's address has nothing to do with it. Defaulting to the GCS would produce a
 * confident connection attempt against QGroundControl's laptop, on a port nothing is listening on.
 *
 * **The fallback is the relay, never the laptop.** [ROUTER_HOST] with nothing else known.
 * `wifi-fix.md`: phone→laptop unicast is blackholed by this AP in every radio combination tried,
 * which is why MAVLink relays through hyper1 and why the Zenoh router lives there too — beside
 * `mavlink-relay.service`, under the same `mini4pro-relay.target`. Any host that is not the relay
 * earns a [Plan.warning] rather than a refusal, because a different network is a legitimate thing
 * to be on and silence is not.
 */
object ZenohSettings {

    /** hyper1, the wired relay, and where `zenoh-router.service` runs. */
    const val ROUTER_HOST = "10.55.1.50"

    /** Zenoh's own default, and what `eclipse/zenoh:1.9.0` listens on. */
    const val ROUTER_PORT = 7447

    // Persisted in the app's SharedPreferences, alongside `gcs_host` / `gcs_port`.
    const val PREF_ENABLED = "zenoh_enabled"
    const val PREF_HOST = "zenoh_host"
    const val PREF_PORT = "zenoh_port"
    const val PREF_PREFIX = "zenoh_prefix"

    /**
     * The video channel, **separately off by default**, and it is a second switch rather than part
     * of the first on purpose.
     *
     * Telemetry on this bus is about 0.12 Mbit/s. Video measured **5.85 Mbit/s at 43 fps**, and
     * those same bytes already go to QGroundControl as RTP on :5600 — so publishing them here
     * **doubles the uplink**, on a network this project has measured blackholing traffic in one
     * direction (`wifi-fix.md`). An operator turning the bus on to see telemetry has not thereby
     * agreed to spend that, which is exactly the shape of `KEY_FAST_RECORD`: the cost is real, it
     * is bounded, and it is the operator's to choose.
     */
    const val PREF_VIDEO = "zenoh_video"

    /**
     * The `detections` channel, **also separately off by default**, and for a cost that is not
     * bandwidth.
     *
     * 10 Hz of 552-byte messages is 5.5 kB/s — 0.05 Mbit/s, against video's 5.85. What makes
     * this a switch is `docs/tag-detector.md` §7: the pose rests on a *fitted* focal length, an
     * *assumed* principal point and no distortion model, and *"publishing a coarse pose onto a bus
     * where a consumer cannot see the flag is the one way this could mislead somebody who never
     * read this document"*. Every message carries the qualification in its `id`, so the flag
     * cannot be separated from the number — and putting a number that coarse onto a shared bus is
     * still the operator's to decide rather than ours to default.
     */
    const val PREF_DETECTIONS = "zenoh_detections"

    // Launch extras, so `tools/session` can drive a measurement session without the dialog.
    const val EXTRA_ENABLED = "zenoh"
    const val EXTRA_HOST = "zenohHost"
    const val EXTRA_PORT = "zenohPort"
    const val EXTRA_PREFIX = "zenohPrefix"

    /** `--ez zenohVideo`. Overrides [PREF_VIDEO] in either direction, like [EXTRA_ENABLED]. */
    const val EXTRA_VIDEO = "zenohVideo"

    /** `--ez zenohDetections`. Overrides [PREF_DETECTIONS] in either direction. */
    const val EXTRA_DETECTIONS = "zenohDetections"

    /** Where [Plan.host] came from. Displayed, so a surprising target is explicable. */
    enum class Source {
        /** `--es zenohHost` on this launch. */
        INTENT,

        /** The saved router override. */
        SAVED,

        /** Nothing was set, so [ROUTER_HOST]. The normal case. */
        DEFAULT_RELAY,
    }

    /**
     * @param warning non-null when the target is worth a second look before a session starts.
     *   Never fatal.
     */
    data class Plan(
        val enabled: Boolean,
        val host: String,
        val port: Int,
        val prefix: String,
        val source: Source,
        val warning: String? = null,
        /**
         * Whether the `video` channel is wanted this session.
         *
         * **Meaningless unless [enabled]**, and [resolve] enforces that rather than leaving it to
         * every caller: a video switch that stayed on while the bus was off would put a second
         * "why is nothing publishing" state on the screen with no way to tell it from the first.
         */
        val video: Boolean = false,
        /**
         * Whether the `detections` channel is wanted this session.
         *
         * **Meaningless unless [enabled]**, for [video]'s reason and enforced the same way.
         */
        val detections: Boolean = false,
    ) {
        val target: String get() = "$host:$port"

        /** The zenoh endpoint. TCP because that is what the router listens on and what works here. */
        val endpoint: String get() = "tcp/$host:$port"

        fun config(bindAddress: String? = null): ZenohConfig =
            ZenohConfig(endpoint = endpoint, prefix = prefix, bindAddress = bindAddress)
    }

    /**
     * @param intentEnabled `--ez zenoh` on this launch, or null when the extra is absent. Null and
     *   `false` are deliberately different: absent leaves the saved setting alone, `false` is a
     *   considered "not this session".
     */
    fun resolve(
        intentEnabled: Boolean? = null,
        savedEnabled: Boolean = false,
        intentHost: String? = null,
        savedHost: String? = null,
        intentPort: Int? = null,
        savedPort: Int? = null,
        intentPrefix: String? = null,
        savedPrefix: String? = null,
        intentVideo: Boolean? = null,
        savedVideo: Boolean = false,
        intentDetections: Boolean? = null,
        savedDetections: Boolean = false,
    ): Plan {
        val enabled = intentEnabled ?: savedEnabled
        val (host, source) = when {
            !intentHost.isNullOrBlank() -> intentHost.trim() to Source.INTENT
            !savedHost.isNullOrBlank() -> savedHost.trim() to Source.SAVED
            else -> ROUTER_HOST to Source.DEFAULT_RELAY
        }
        val port = (intentPort ?: savedPort ?: ROUTER_PORT).let {
            if (it in 1..65535) it else ROUTER_PORT
        }
        // A blank or whitespace prefix would produce `/odom/nav_msgs.Odometry`, a key expression
        // zenoh accepts and no DiMOS module subscribes to. The default is what DiMOS generates.
        val prefix = (intentPrefix ?: savedPrefix)?.trim()?.trim('/')?.takeIf { it.isNotEmpty() }
            ?: ZenohChannel.DEFAULT_PREFIX
        return Plan(
            enabled = enabled,
            host = host,
            port = port,
            prefix = prefix,
            source = source,
            warning = warn(host),
            // `&& enabled`: there is no such thing as video on a bus that is off, and expressing
            // that here means no caller has to remember it.
            video = (intentVideo ?: savedVideo) && enabled,
            detections = (intentDetections ?: savedDetections) && enabled,
        )
    }

    /**
     * The one sentence that stands between an operator and a session that publishes into nothing.
     *
     * Phrased as an observation rather than an error, exactly as `VideoRequest.warn` is: on a
     * different network another address is right. On *this* one it is the single most likely
     * reason a subscriber sees nothing.
     */
    fun warn(host: String): String? = when {
        host == ROUTER_HOST -> null
        host == "127.0.0.1" || host == "localhost" ->
            "Zenoh router is loopback — nothing off this phone can see the bus."
        else ->
            "Zenoh router is $host, not the relay at $ROUTER_HOST. " +
                "Phone→laptop traffic is blackholed on this network (wifi-fix.md); " +
                "if nothing appears on the bus, this is the first thing to check."
    }

    /**
     * The status line. Separate from the session's phase, always, because the two fail for
     * different reasons and one of them is invisible to the publisher — "ON, NOT RUNNING" is the
     * honest reading when the bridge was never started or refused for want of WiFi, states in
     * which the publisher is correctly sitting in [ZenohPublisher.Phase.STOPPED] with nothing to
     * say.
     */
    fun statusLine(plan: Plan, phase: ZenohPublisher.Phase): String = when {
        !plan.enabled && phase == ZenohPublisher.Phase.STOPPED -> "off"
        !plan.enabled -> "OFF, but still running — stop the bridge to end it"
        phase == ZenohPublisher.Phase.STOPPED -> "ON, NOT RUNNING → ${plan.target}"
        phase == ZenohPublisher.Phase.CONNECTING -> "ON, connecting → ${plan.target}"
        else -> "ON, publishing → ${plan.target}  [${plan.prefix}]" +
            (if (plan.video) " +video" else "") +
            (if (plan.detections) " +detections" else "")
    }
}
