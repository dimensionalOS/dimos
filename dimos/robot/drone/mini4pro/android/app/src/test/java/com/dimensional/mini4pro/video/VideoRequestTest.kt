package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Whether video is on, and where it points.
 *
 * These rules were implicit for the whole of M4, and the cost of that is the
 * reason this file exists: `tools/session video` passed `--ez video true` for
 * weeks into an app that read no such extra, and nobody noticed, because a flag
 * that is silently ignored and a feature that silently fails produce the same
 * observation — no picture. Every rule below is therefore a test rather than a
 * convention, including the boring ones.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M1 | `intentEnabled ?: savedEnabled` → `savedEnabled` (the extra ignored again) | 2 |
 *  | M2 | `intentEnabled == true \|\| savedEnabled` (`--ez video false` cannot turn it off) | 1 |
 *  | M3 | default `enabled = true` when nothing is saved | 2 |
 *  | M4 | host precedence: saved checked before intent | 1 |
 *  | M5 | host precedence: `gcsHost` checked before the saved override | 1 |
 *  | M6 | `isNullOrBlank` → `isNullOrEmpty` (a whitespace host counts as set) | 1 |
 *  | M7 | final fallback changed from the relay to `10.55.1.12` (the laptop) | 2 |
 *  | M8 | `warn` returns null for every host | 1 |
 *  | M9 | `warn` returns non-null for the relay too (cries wolf every session) | 1 |
 *  | M10 | out-of-range port passed through instead of falling back to 5600 | 1 |
 *  | M11 | `statusLine` reports STOPPED-while-enabled as if it were running | 1 |
 *  | M12 | `statusLine` drops the target from the enabled line | 1 |
 *
 * No survivors. Several mutants are killed by exactly one test, which is the
 * honest count rather than a comfortable one: the rules here are independent of
 * each other, so a single rule breaking *should* implicate a single test. M6 is
 * the one worth keeping an eye on — `""` and `"   "` reach different branches of
 * `isNullOrEmpty`, and only the second distinguishes it from `isNullOrBlank`.
 */
class VideoRequestTest {

    // ---- the switch -----------------------------------------------------------

    @Test
    fun `video is off when nothing has ever asked for it`() {
        assertFalse(VideoRequest.resolve().enabled)
    }

    @Test
    fun `the saved setting is what carries between launches`() {
        assertTrue(VideoRequest.resolve(savedEnabled = true).enabled)
    }

    @Test
    fun `the intent extra turns video on over a saved off`() {
        assertTrue(VideoRequest.resolve(intentEnabled = true, savedEnabled = false).enabled)
    }

    /**
     * The half that makes the flag trustworthy. A launch flag that can only ever
     * turn something on cannot be used to rule video out as the cause of a
     * problem, which is exactly what a diagnostic flag is for.
     */
    @Test
    fun `the intent extra turns video off over a saved on`() {
        assertFalse(VideoRequest.resolve(intentEnabled = false, savedEnabled = true).enabled)
    }

    @Test
    fun `an absent extra leaves the saved setting alone`() {
        // null is not false: absent means "do not touch", which is why the
        // parameter is nullable rather than defaulting to false.
        assertTrue(VideoRequest.resolve(intentEnabled = null, savedEnabled = true).enabled)
    }

    // ---- where it points ------------------------------------------------------

    @Test
    fun `with nothing configured the target is the relay, not the laptop`() {
        val plan = VideoRequest.resolve()
        assertEquals(VideoRequest.RELAY_HOST, plan.host)
        assertEquals(VideoRequest.Source.DEFAULT_RELAY, plan.source)
    }

    /**
     * The rule the whole design rests on: the relay carries `:14550` and `:5600`
     * in one process, so telemetry and video share an address and there is one
     * mistake to make instead of two — and that one announces itself, because a
     * wrong telemetry address means QGC never connects at all.
     */
    @Test
    fun `video follows the MAVLink target by default`() {
        val plan = VideoRequest.resolve(gcsHost = "10.55.1.50")
        assertEquals("10.55.1.50", plan.host)
        assertEquals(VideoRequest.Source.FOLLOWS_GCS, plan.source)
    }

    @Test
    fun `a saved video host overrides the GCS address`() {
        val plan = VideoRequest.resolve(savedHost = "10.55.1.99", gcsHost = "10.55.1.50")
        assertEquals("10.55.1.99", plan.host)
        assertEquals(VideoRequest.Source.SAVED, plan.source)
    }

    @Test
    fun `the intent host outranks everything`() {
        val plan = VideoRequest.resolve(
            intentHost = "10.55.1.77", savedHost = "10.55.1.99", gcsHost = "10.55.1.50",
        )
        assertEquals("10.55.1.77", plan.host)
        assertEquals(VideoRequest.Source.INTENT, plan.source)
    }

    @Test
    fun `a cleared video host falls through to the GCS address`() {
        // The settings dialog writes "" when the field is emptied, and "" must
        // mean "unset" rather than "send video to nowhere".
        val plan = VideoRequest.resolve(savedHost = "   ", gcsHost = "10.55.1.50")
        assertEquals("10.55.1.50", plan.host)
        assertEquals(VideoRequest.Source.FOLLOWS_GCS, plan.source)
    }

    @Test
    fun `a blank GCS address still lands on the relay`() {
        assertEquals(VideoRequest.RELAY_HOST, VideoRequest.resolve(gcsHost = "").host)
    }

    @Test
    fun `surrounding whitespace does not become part of the host`() {
        assertEquals("10.55.1.50", VideoRequest.resolve(intentHost = " 10.55.1.50 ").host)
    }

    // ---- the port -------------------------------------------------------------

    @Test
    fun `the default port is QGCs own`() {
        assertEquals(5600, VideoRequest.DEFAULT_PORT)
        assertEquals(5600, VideoRequest.resolve().port)
    }

    @Test
    fun `an explicit port is honoured from either source`() {
        assertEquals(5601, VideoRequest.resolve(intentPort = 5601).port)
        assertEquals(5602, VideoRequest.resolve(savedPort = 5602).port)
        assertEquals(5601, VideoRequest.resolve(intentPort = 5601, savedPort = 5602).port)
    }

    @Test
    fun `an impossible port falls back rather than opening a broken socket`() {
        // `--ei videoPort 0` and a negative typo both reach here. Falling back is
        // better than a DatagramSocket exception the operator reads as "video is
        // broken" rather than "that port is not a port".
        assertEquals(VideoRequest.DEFAULT_PORT, VideoRequest.resolve(intentPort = 0).port)
        assertEquals(VideoRequest.DEFAULT_PORT, VideoRequest.resolve(intentPort = 70_000).port)
        assertEquals(VideoRequest.DEFAULT_PORT, VideoRequest.resolve(savedPort = -1).port)
    }

    @Test
    fun `target is host and port together, as the status screen shows it`() {
        assertEquals("10.55.1.50:5600", VideoRequest.resolve().target)
    }

    // ---- the warning ----------------------------------------------------------

    @Test
    fun `the relay is the one target that draws no warning`() {
        assertNull(VideoRequest.resolve().warning)
        assertNull(VideoRequest.resolve(gcsHost = VideoRequest.RELAY_HOST).warning)
    }

    /**
     * `wifi-fix.md`: phone→laptop unicast is blackholed by this AP in every radio
     * combination tried. Aiming video at the laptop is therefore the single most
     * likely reason for a black picture, and it is invisible — the phone's
     * counters all look perfect while the packets go nowhere.
     */
    @Test
    fun `a non-relay target is named and explained, not refused`() {
        val plan = VideoRequest.resolve(gcsHost = "10.55.1.12")
        assertNotNull(plan.warning)
        assertTrue(plan.warning!!, plan.warning!!.contains("10.55.1.12"))
        assertTrue(plan.warning!!, plan.warning!!.contains("wifi-fix.md"))
        // Named, but still usable: a different network is a legitimate place to be.
        assertEquals("10.55.1.12", plan.host)
    }

    // ---- the status line ------------------------------------------------------

    @Test
    fun `an off setting says where to turn it on`() {
        val line = VideoRequest.statusLine(VideoRequest.resolve(), VideoPhase.STOPPED)
        assertTrue(line, line.contains("off"))
        assertTrue(line, line.contains("Settings"))
    }

    /**
     * The state the whole `statusLine`/`phase` split exists for. The bridge was
     * never started, or it refused for want of WiFi — in both cases
     * `VideoStreamer` is correctly in STOPPED with nothing to report, and a screen
     * that showed only the phase would say "not started" and leave the operator
     * unable to tell "you did not ask" from "you asked and it did not happen".
     */
    @Test
    fun `enabled but stopped is called out as not running, with the reason`() {
        val plan = VideoRequest.resolve(savedEnabled = true)
        val line = VideoRequest.statusLine(plan, VideoPhase.STOPPED)
        assertTrue(line, line.contains("NOT RUNNING"))
        assertTrue(line, line.contains("bridge"))
        assertTrue(line, line.contains(plan.target))
    }

    @Test
    fun `a running stream names its target and where the address came from`() {
        val plan = VideoRequest.resolve(savedEnabled = true, gcsHost = "10.55.1.50")
        val line = VideoRequest.statusLine(plan, VideoPhase.SERVING)
        assertTrue(line, line.contains("10.55.1.50:5600"))
        assertFalse(line, line.contains("NOT RUNNING"))
        assertTrue(line, line.contains("telemetry"))
    }

    @Test
    fun `every waiting phase reads as running rather than as not started`() {
        // WAITING_CAMERA is not "off". Conflating the two would send an operator
        // to the Settings dialog to fix a camera problem.
        val plan = VideoRequest.resolve(savedEnabled = true)
        for (phase in VideoPhase.values().filter { it != VideoPhase.STOPPED }) {
            val line = VideoRequest.statusLine(plan, phase)
            assertFalse("$phase: $line", line.contains("NOT RUNNING"))
        }
    }

    @Test
    fun `each host source produces a distinguishable note`() {
        val plan = VideoRequest.resolve(savedEnabled = true)
        val notes = VideoRequest.Source.values().map {
            VideoRequest.statusLine(plan.copy(source = it), VideoPhase.SERVING)
        }
        assertEquals(VideoRequest.Source.values().size, notes.toSet().size)
    }
}
