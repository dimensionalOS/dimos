package com.dimensional.mini4pro.video

/**
 * RTSP URL construction for MSDK's built-in RTSP server.
 *
 * The MSDK RTSP server does not tell you its URL. You configure a username,
 * password and port through [dji.v5.manager.datacenter.livestream.settings.RtspSettings]
 * and are expected to assemble the address yourself from the *phone's* IP.
 *
 * **The `/streaming/live/1` path suffix is the trap.** It appears nowhere in the
 * API surface — only in one prose example on the `RtspSettings` doc page
 * (`tools/djidoc ILiveStreamManager_LiveStreamSettings_RtspSettings`) and in
 * MSDK issue #624. Without it a player connects to the server and gets 404 /
 * "no such stream", which reads exactly like "video is broken".
 *
 * This object is deliberately DJI-free and Android-free so it is unit-testable:
 * a wrong URL is a silent failure that costs a whole hardware session, and the
 * MSDK is not on the unit-test runtime classpath.
 */
object RtspUrl {

    /** Undocumented, load-bearing. See the class doc and MSDK issue #624. */
    const val PATH = "/streaming/live/1"

    /**
     * MSDK does not document a default RTSP port. 8554 is the port in DJI's own
     * example and the conventional non-privileged RTSP port; Android cannot bind
     * 554 without root anyway.
     */
    const val DEFAULT_PORT = 8554

    /**
     * The credentials are only a token gate on a LAN stream, but they cannot be
     * empty: they go into the URL's userinfo and MSDK rejects blank settings.
     * Keep them ASCII-simple — see [problems].
     */
    const val DEFAULT_USER = "mini4pro"
    const val DEFAULT_PASSWORD = "mini4pro"

    /** Characters safe in a URL userinfo field without percent-encoding. */
    private val SAFE_USERINFO = Regex("[A-Za-z0-9._~-]+")

    /**
     * `rtsp://user:pass@host:port/streaming/live/1`.
     *
     * @throws IllegalArgumentException if [problems] is non-empty — callers that
     *   must not throw should check [problems] first and report the strings.
     */
    fun build(
        host: String,
        port: Int = DEFAULT_PORT,
        user: String = DEFAULT_USER,
        password: String = DEFAULT_PASSWORD,
    ): String {
        val issues = problems(host, port, user, password)
        require(issues.isEmpty()) { "cannot build RTSP URL: ${issues.joinToString("; ")}" }
        return "rtsp://$user:$password@${hostPart(host)}:$port$PATH"
    }

    /**
     * The same URL without credentials, for players that take user/password in
     * separate fields (and for logging, so credentials do not land in logcat).
     */
    fun buildWithoutCredentials(host: String, port: Int = DEFAULT_PORT): String {
        val issues = problems(host, port, DEFAULT_USER, DEFAULT_PASSWORD)
        require(issues.isEmpty()) { "cannot build RTSP URL: ${issues.joinToString("; ")}" }
        return "rtsp://${hostPart(host)}:$port$PATH"
    }

    /** Replaces the userinfo with `***:***`, for logs and the status screen. */
    fun redact(url: String): String =
        Regex("^(rtsp://)[^/@]*@").replace(url) { "${it.groupValues[1]}***:***@" }

    /**
     * Everything wrong with these parameters, in human-readable form. Empty list
     * means [build] will succeed.
     */
    fun problems(host: String?, port: Int, user: String, password: String): List<String> {
        val out = mutableListOf<String>()

        if (host.isNullOrBlank()) {
            out += "no local IP address: the phone has no usable network interface, " +
                "so there is no address a GCS could connect to"
        } else if (host.any { it.isWhitespace() } || host.contains('/') || host.contains('@')) {
            out += "host '$host' is not a bare address"
        }

        if (port !in 1..65535) {
            out += "port $port is out of range 1..65535"
        } else if (port < 1024) {
            out += "port $port is privileged; Android cannot bind below 1024 without root"
        }

        if (user.isEmpty()) out += "username is empty"
        else if (!SAFE_USERINFO.matches(user)) {
            out += "username '$user' has characters that need URL-escaping in the userinfo field"
        }

        if (password.isEmpty()) out += "password is empty"
        else if (!SAFE_USERINFO.matches(password)) {
            out += "password has characters that need URL-escaping in the userinfo field"
        }

        return out
    }

    /** IPv6 literals need brackets in a URL authority. */
    private fun hostPart(host: String): String =
        if (host.contains(':') && !host.startsWith("[")) "[$host]" else host
}
