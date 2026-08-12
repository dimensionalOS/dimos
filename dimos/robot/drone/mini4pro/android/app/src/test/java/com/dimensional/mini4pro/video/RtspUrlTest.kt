package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The RTSP URL is the one artefact a GCS consumes, and every mistake in it looks
 * like "video is broken". These tests exist so the shape is nailed down without
 * hardware — the MSDK is not on the unit-test runtime classpath.
 */
class RtspUrlTest {

    @Test
    fun `matches DJIs documented example`() {
        // From `tools/djidoc ILiveStreamManager_LiveStreamSettings_RtspSettings`:
        //   Username: 123456  Password: 123  Port: 8554  Device IP: 192.168.2.13
        //   -> rtsp://123456:123@192.168.2.13:8554/streaming/live/1
        assertEquals(
            "rtsp://123456:123@192.168.2.13:8554/streaming/live/1",
            RtspUrl.build(host = "192.168.2.13", port = 8554, user = "123456", password = "123"),
        )
    }

    @Test
    fun `path suffix is the undocumented one from MSDK issue 624`() {
        assertEquals("/streaming/live/1", RtspUrl.PATH)
        assertTrue(RtspUrl.build("10.55.1.15").endsWith("/streaming/live/1"))
    }

    @Test
    fun `defaults produce a usable URL for the test rig`() {
        assertEquals(
            "rtsp://mini4pro:mini4pro@10.55.1.15:8554/streaming/live/1",
            RtspUrl.build("10.55.1.15"),
        )
    }

    @Test
    fun `credential-free form is offered for players with separate auth fields`() {
        assertEquals(
            "rtsp://10.55.1.15:8554/streaming/live/1",
            RtspUrl.buildWithoutCredentials("10.55.1.15"),
        )
    }

    @Test
    fun `redact hides the userinfo but keeps the address`() {
        assertEquals(
            "rtsp://***:***@10.55.1.15:8554/streaming/live/1",
            RtspUrl.redact(RtspUrl.build("10.55.1.15")),
        )
    }

    @Test
    fun `redact leaves a credential-free URL alone`() {
        val url = RtspUrl.buildWithoutCredentials("10.55.1.15")
        assertEquals(url, RtspUrl.redact(url))
    }

    @Test
    fun `ipv6 host is bracketed`() {
        assertTrue(RtspUrl.buildWithoutCredentials("fe80::1").contains("[fe80::1]:8554"))
    }

    @Test
    fun `no local address is reported as a problem rather than an empty host`() {
        val problems = RtspUrl.problems(null, 8554, "u", "p")
        assertEquals(1, problems.size)
        assertTrue(problems.first().contains("no local IP"))
    }

    @Test
    fun `privileged ports are rejected because Android cannot bind them`() {
        assertTrue(RtspUrl.problems("10.0.0.1", 554, "u", "p").any { it.contains("privileged") })
        assertTrue(RtspUrl.problems("10.0.0.1", 0, "u", "p").any { it.contains("out of range") })
        assertTrue(RtspUrl.problems("10.0.0.1", 70000, "u", "p").any { it.contains("out of range") })
    }

    @Test
    fun `empty credentials are rejected`() {
        assertTrue(RtspUrl.problems("10.0.0.1", 8554, "", "p").any { it.contains("username is empty") })
        assertTrue(RtspUrl.problems("10.0.0.1", 8554, "u", "").any { it.contains("password is empty") })
    }

    @Test
    fun `credentials needing percent-encoding are rejected, not silently mangled`() {
        // 'a:b' in the userinfo would split the URL at the wrong colon; '@' would
        // break the authority. Both parse as *something*, which is the danger.
        assertTrue(RtspUrl.problems("10.0.0.1", 8554, "a:b", "p").any { it.contains("URL-escaping") })
        assertTrue(RtspUrl.problems("10.0.0.1", 8554, "u", "p@ss").any { it.contains("URL-escaping") })
        assertTrue(RtspUrl.problems("10.0.0.1", 8554, "u", "p/w").any { it.contains("URL-escaping") })
    }

    @Test
    fun `defaults are valid`() {
        assertTrue(
            RtspUrl.problems(
                "10.55.1.15", RtspUrl.DEFAULT_PORT, RtspUrl.DEFAULT_USER, RtspUrl.DEFAULT_PASSWORD,
            ).isEmpty()
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun `build refuses an invalid configuration instead of emitting a broken URL`() {
        RtspUrl.build(host = "", port = 8554)
    }

    @Test
    fun `host with a path is rejected`() {
        assertNotNull(
            RtspUrl.problems("10.0.0.1/live", 8554, "u", "p").firstOrNull { it.contains("bare address") }
        )
    }
}
