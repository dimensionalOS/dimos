package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The control loop's ordering rules.
 *
 * These used to be inline in [VideoStreamer], which meant the only way to check
 * them was with an aircraft on the bench — and they are precisely the rules a
 * later edit breaks: gate order, tear down before waiting, back off before
 * retrying, and give up on a `startStream` that never calls back.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | aircraft gate checked before registration | 0 → **2** after adding tests |
 *  | camera gate checked before aircraft | 1 |
 *  | teardown reason dropped when a gate is lost while serving | 3 |
 *  | teardown emitted on the registration gate | 1 |
 *  | teardown emitted when nothing was running | 1 |
 *  | `START_TIMEOUT_MS` check removed (STARTING wedges forever) | 3 |
 *  | start timeout compared with `>` on an exact boundary | 2 |
 *  | streaming grace compared with `>=` instead of `>` | 1 |
 *  | grace ignored (fails immediately after a successful start) | 1 |
 *  | `nextAttemptAtMs` ignored (retries every tick) | 2 |
 *
 * Swapping the registration and aircraft gates originally survived: the only
 * test for gate order set `registered = false` while leaving the aircraft
 * *connected*, so either order gave the same answer. `registration outranks the
 * aircraft gate when both are missing` is the case that actually pins it.
 *
 * ## The stall rule and the progressive retry, measured 2026-07-30
 *
 * Whole suite per mutant, **2711 tests**, `test-results` deleted first, confirmed red, reverted.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the stall rule deleted entirely (a quiet feed polls forever) | 2 |
 *  | the stall fires for a feed that never delivered a frame | 5 |
 *  | the silence boundary loosened from `>=` to `>` | 1 |
 *  | `STALL_AFTER_MS` cut to 500 ms — inside the measured healthy range | 1 |
 *  | no teardown on a stall (bring-up cannot re-subscribe) | 1 |
 *  | the backoff flattened to a constant 5 s | 2 |
 *  | the retry cap removed (the delay grows without bound) | 1 |
 *
 * No survivors. The 5 is the one worth reading: dropping the `framesSeen > 0` guard breaks five
 * tests, not one, because a rule that fires on a feed which never delivered anything also
 * mislabels every "waiting for the first frame" case as a stall — the fault `VideoStatus.verdict`
 * already names, with a different cure, and the one where restarting loops forever against a
 * camera in playback mode.
 *
 * The threshold row is a 1 by construction and depends entirely on a *measurement* rather than on
 * this suite: 55 133 inter-frame gaps across three recorded flights, worst healthy gap 1.043 s.
 * See `VideoLoop.STALL_AFTER_MS` — if that number is ever re-measured, this test changes with it.
 */
class VideoLoopTest {

    private fun input(
        wanted: Boolean = true,
        registered: Boolean = true,
        aircraftConnected: Boolean = true,
        cameraAvailable: Boolean = true,
        serving: Boolean = false,
        startInFlight: Boolean = false,
        nowMs: Long = 100_000,
        attemptStartedAtMs: Long = 0,
        nextAttemptAtMs: Long = 0,
        servingSinceMs: Long = 0,
        streamingReported: Boolean = false,
        framesSeen: Long = 0,
        lastFrameAtMs: Long = 0,
        consecutiveFailures: Int = 0,
    ) = VideoLoop.Input(
        wanted, registered, aircraftConnected, cameraAvailable, serving, startInFlight,
        nowMs, attemptStartedAtMs, nextAttemptAtMs, servingSinceMs, streamingReported,
        framesSeen, lastFrameAtMs, consecutiveFailures,
    )

    /** Serving, settled, and MSDK agreeing — so only the stall rule can decide anything. */
    private fun servingHealthy(
        nowMs: Long = 100_000,
        framesSeen: Long = 500,
        lastFrameAtMs: Long,
    ) = input(
        serving = true,
        streamingReported = true,
        servingSinceMs = 0,
        nowMs = nowMs,
        framesSeen = framesSeen,
        lastFrameAtMs = lastFrameAtMs,
    )

    // ---- the stall ------------------------------------------------------------

    @Test
    fun `a feed that delivered frames and went quiet is a failure, with a teardown`() {
        // The 2026-07-30 shape exactly: aircraft connected, camera reported, phase SERVING,
        // listener registered, 8 frames delivered and then nothing for two minutes. Every gate
        // above this rule says everything is fine, because none of them looks at frames.
        val d = VideoLoop.decide(
            // 110 s of silence, the interval Ivan actually waited before restarting by hand.
            servingHealthy(nowMs = 200_000, framesSeen = 8, lastFrameAtMs = 90_000),
        )
        assertTrue("a stalled feed must fail so the retry ladder restarts it", d is VideoLoop.Fail)
        assertTrue(
            "the message must name the stall and how long it has been quiet",
            (d as VideoLoop.Fail).message.contains("stalled") && d.message.contains("8"),
        )
        assertNotNull("the stream must be torn down, or the bring-up cannot re-subscribe", d.tearDown)
    }

    @Test
    fun `a gap inside the measured healthy range is not a stall`() {
        // The worst inter-frame gap ever recorded on this airframe is 1.043 s (landing18,
        // 55k gaps across three flights — see VideoLoop.STALL_AFTER_MS). Anything at or under
        // that must poll, or a healthy feed restarts itself mid-flight.
        val d = VideoLoop.decide(servingHealthy(nowMs = 100_000, lastFrameAtMs = 100_000 - 1_043))
        assertEquals(VideoLoop.Poll, d)
    }

    @Test
    fun `the stall threshold is exclusive at the boundary and fires one millisecond later`() {
        val justUnder = VideoLoop.decide(
            servingHealthy(nowMs = 100_000, lastFrameAtMs = 100_000 - VideoLoop.STALL_AFTER_MS + 1),
        )
        assertEquals(VideoLoop.Poll, justUnder)
        val exactly = VideoLoop.decide(
            servingHealthy(nowMs = 100_000, lastFrameAtMs = 100_000 - VideoLoop.STALL_AFTER_MS),
        )
        assertTrue("the threshold is reached, so it must fire", exactly is VideoLoop.Fail)
    }

    @Test
    fun `a feed that has never delivered a frame is never called a stall`() {
        // A different fault with a different cure: VideoStatus.verdict already names it
        // ("SERVING but zero frames — suspect the lens/source"), and restarting on it would loop
        // forever against a camera in playback mode. This rule is only for feeds that proved
        // themselves and then stopped.
        val d = VideoLoop.decide(servingHealthy(nowMs = 100_000, framesSeen = 0, lastFrameAtMs = 0))
        assertEquals(VideoLoop.Poll, d)
    }

    @Test
    fun `frames seen but no timestamp yet does not fire`() {
        // Defensive: a counter and its clock are written separately, and "unknown is not zero" —
        // a 0 timestamp read as "1970" would make every feed look stalled by 100 seconds.
        val d = VideoLoop.decide(servingHealthy(nowMs = 100_000, framesSeen = 42, lastFrameAtMs = 0))
        assertEquals(VideoLoop.Poll, d)
    }

    @Test
    fun `a lost aircraft outranks the stall — the cure is waiting, not restarting`() {
        // Tonight's session ended this way: the feed stalled AND the aircraft went away. Failing
        // would put us on the retry ladder against an airframe that is not there; the gate above
        // parks us in WAIT_AIRCRAFT and costs nothing until it returns.
        val d = VideoLoop.decide(
            input(
                serving = true, streamingReported = true, aircraftConnected = false,
                nowMs = 100_000, framesSeen = 159, lastFrameAtMs = 100_000 - 60_000,
            ),
        )
        assertEquals(VideoGate.WAIT_AIRCRAFT, (d as VideoLoop.Wait).gate)
    }

    // ---- the progressive retry ------------------------------------------------

    @Test
    fun `the first retry is prompt and each repeat doubles it`() {
        assertEquals(VideoLoop.RETRY_AFTER_FAILURE_MS, VideoLoop.retryDelayMs(0))
        assertEquals(10_000L, VideoLoop.retryDelayMs(1))
        assertEquals(20_000L, VideoLoop.retryDelayMs(2))
        assertEquals(40_000L, VideoLoop.retryDelayMs(3))
    }

    @Test
    fun `the ladder is capped, and never gives up`() {
        // Capping the interval bounds the cost of a feed that will not come back; capping the
        // *count* would decide in advance that the flight is over. Every step past the cap is
        // still a real, finite delay — never zero (a hammer) and never infinite (a surrender).
        for (n in 4..100) {
            assertEquals(
                "attempt $n must sit at the cap, still retrying",
                VideoLoop.RETRY_CAP_MS, VideoLoop.retryDelayMs(n),
            )
        }
    }

    @Test
    fun `a nonsensical negative history still yields the prompt first delay`() {
        assertEquals(VideoLoop.RETRY_AFTER_FAILURE_MS, VideoLoop.retryDelayMs(-1))
    }

    // ---- gate order -----------------------------------------------------------

    @Test
    fun `nobody wanting video stops, whatever else is true`() {
        assertEquals(VideoLoop.Stop, VideoLoop.decide(input(wanted = false, serving = true)))
    }

    @Test
    fun `registration is the first gate even when everything else looks ready`() {
        // Touching the MSDK before registration silently does nothing, so this
        // ordering is the one rule that must never be relaxed.
        val d = VideoLoop.decide(input(registered = false))
        assertEquals(VideoGate.WAIT_REGISTRATION, (d as VideoLoop.Wait).gate)
    }

    @Test
    fun `registration outranks the aircraft gate when both are missing`() {
        // The interesting case, and the one a swapped gate order survives: with
        // only `registered = false` set, the aircraft is connected and either
        // order gives the same answer. Both missing is what pins the order.
        val d = VideoLoop.decide(input(registered = false, aircraftConnected = false))
        assertEquals(VideoGate.WAIT_REGISTRATION, (d as VideoLoop.Wait).gate)
    }

    @Test
    fun `registration outranks every other gate at once`() {
        val d = VideoLoop.decide(
            input(registered = false, aircraftConnected = false, cameraAvailable = false),
        )
        assertEquals(VideoGate.WAIT_REGISTRATION, (d as VideoLoop.Wait).gate)
    }

    @Test
    fun `a missing aircraft outranks a missing camera`() {
        val d = VideoLoop.decide(input(aircraftConnected = false, cameraAvailable = false))
        assertEquals(VideoGate.WAIT_AIRCRAFT, (d as VideoLoop.Wait).gate)
    }

    @Test
    fun `the camera is the last gate before bringing the stream up`() {
        val d = VideoLoop.decide(input(cameraAvailable = false))
        assertEquals(VideoGate.WAIT_CAMERA, (d as VideoLoop.Wait).gate)
    }

    @Test
    fun `all gates met and no attempt in flight brings the stream up`() {
        assertEquals(VideoLoop.BringUp, VideoLoop.decide(input()))
    }

    // ---- teardown -------------------------------------------------------------

    @Test
    fun `losing the aircraft while serving tears the stream down and says why`() {
        val d = VideoLoop.decide(input(aircraftConnected = false, serving = true))
        assertEquals("aircraft disconnected", d.tearDown)
    }

    @Test
    fun `losing the camera while serving tears the stream down and says why`() {
        // Without the teardown the stale `serving` flag means the camera coming
        // back never re-triggers a bring-up.
        val d = VideoLoop.decide(input(cameraAvailable = false, serving = true))
        assertEquals("camera no longer reported", d.tearDown)
    }

    @Test
    fun `losing a gate while a start is in flight also tears down`() {
        assertNotNull(VideoLoop.decide(input(aircraftConnected = false, startInFlight = true)).tearDown)
    }

    @Test
    fun `waiting behind a gate with nothing running tears nothing down`() {
        assertNull(VideoLoop.decide(input(aircraftConnected = false)).tearDown)
        assertNull(VideoLoop.decide(input(cameraAvailable = false)).tearDown)
    }

    @Test
    fun `the registration gate never tears anything down`() {
        // Nothing can be running yet, and a teardown would be an MSDK call before
        // registration — the very thing the gate exists to prevent.
        assertNull(VideoLoop.decide(input(registered = false, serving = true)).tearDown)
    }

    // ---- startStream in flight ------------------------------------------------

    @Test
    fun `a start in flight is awaited while inside the timeout`() {
        val d = VideoLoop.decide(
            input(startInFlight = true, attemptStartedAtMs = 100_000, nowMs = 100_000 + 5_000),
        )
        assertEquals(VideoLoop.AwaitStart, d)
    }

    @Test
    fun `a start that never calls back eventually fails instead of wedging`() {
        // Without this the loop sits in STARTING for the rest of the session with
        // no retry and no error — the silence this whole class exists to prevent.
        val d = VideoLoop.decide(
            input(
                startInFlight = true,
                attemptStartedAtMs = 100_000,
                nowMs = 100_000 + VideoLoop.START_TIMEOUT_MS,
            ),
        )
        assertTrue(d.toString(), d is VideoLoop.Fail)
        assertTrue((d as VideoLoop.Fail).message.contains("never called back"))
        assertNotNull("a timed-out attempt must be torn down before retrying", d.tearDown)
    }

    @Test
    fun `the start timeout fires exactly at the boundary, not a tick later`() {
        val at = VideoLoop.decide(
            input(startInFlight = true, attemptStartedAtMs = 0, nowMs = VideoLoop.START_TIMEOUT_MS),
        )
        val justBefore = VideoLoop.decide(
            input(startInFlight = true, attemptStartedAtMs = 0, nowMs = VideoLoop.START_TIMEOUT_MS - 1),
        )
        assertTrue(at is VideoLoop.Fail)
        assertEquals(VideoLoop.AwaitStart, justBefore)
    }

    // ---- serving health -------------------------------------------------------

    @Test
    fun `serving and streaming just polls`() {
        val d = VideoLoop.decide(
            input(serving = true, servingSinceMs = 0, nowMs = 60_000, streamingReported = true),
        )
        assertEquals(VideoLoop.Poll, d)
    }

    @Test
    fun `isStreaming false inside the grace window is not yet a failure`() {
        // MSDK takes a moment to reflect a successful start; failing here would
        // put the loop into a restart cycle it never escapes.
        val d = VideoLoop.decide(
            input(
                serving = true,
                servingSinceMs = 0,
                nowMs = VideoLoop.STREAMING_GRACE_MS,
                streamingReported = false,
            ),
        )
        assertEquals(VideoLoop.Poll, d)
    }

    @Test
    fun `isStreaming false after the grace window is a failure`() {
        val d = VideoLoop.decide(
            input(
                serving = true,
                servingSinceMs = 0,
                nowMs = VideoLoop.STREAMING_GRACE_MS + 1,
                streamingReported = false,
            ),
        )
        assertTrue(d is VideoLoop.Fail)
        assertTrue((d as VideoLoop.Fail).message.contains("isStreaming=false"))
    }

    // ---- retry backoff --------------------------------------------------------

    @Test
    fun `a failure's backoff is honoured before the next attempt`() {
        assertEquals(
            VideoLoop.AwaitRetry,
            VideoLoop.decide(input(nowMs = 100_000, nextAttemptAtMs = 105_000)),
        )
    }

    @Test
    fun `the attempt goes ahead once the backoff has elapsed`() {
        assertEquals(
            VideoLoop.BringUp,
            VideoLoop.decide(input(nowMs = 105_000, nextAttemptAtMs = 105_000)),
        )
    }

    @Test
    fun `every decision is reachable, so none of them is dead code`() {
        val seen = mutableSetOf<String>()
        listOf(
            input(wanted = false),
            input(registered = false),
            input(aircraftConnected = false),
            input(cameraAvailable = false),
            input(startInFlight = true, attemptStartedAtMs = 100_000, nowMs = 100_100),
            input(startInFlight = true, attemptStartedAtMs = 0, nowMs = 999_999),
            input(serving = true, streamingReported = true, nowMs = 50_000),
            input(nowMs = 1, nextAttemptAtMs = 99_999),
            input(),
        ).forEach { seen += VideoLoop.decide(it)::class.simpleName!! }
        assertEquals(
            setOf("Stop", "Wait", "AwaitStart", "Fail", "Poll", "AwaitRetry", "BringUp"),
            seen,
        )
    }
}
