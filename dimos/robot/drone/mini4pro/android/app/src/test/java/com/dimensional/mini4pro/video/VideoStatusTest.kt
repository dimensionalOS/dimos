package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The status object exists so that "it isn't working" can be answered from
 * logcat. These tests pin the four silent failure modes to four distinct
 * verdicts — if two of them ever collapse into the same sentence, the value of
 * the whole thing is gone.
 *
 * Extended 2026-07-26 when the block was finally wired into the status screen.
 * Two additions, both mutation-checked:
 *
 *  | # | mutation | tests that failed |
 *  |---|---|---|
 *  | M13 | `counterLines` prints only `tapFrames` | 6 |
 *  | M14 | `counterLines` prints `0` instead of `—` for an absent RTP sink | 1 |
 *  | M15 | `counterLines` dropped from `describe()` | 1 |
 *  | M16 | `counterLines` reports `rtp.framesSent` where it says `packetsSent` | 2 |
 *  | M17 | the first-frame grace removed (blames the lens at t=0) | 1 |
 *  | M18 | the grace applied for ever (never blames the lens) | 3 |
 *  | M19 | grace compared with `<=` on the exact boundary | 1 |
 *  | M25 | RTSP-only rows always shown (`live:` under a working passthrough) | 1 |
 *  | M26 | RTSP-only rows never shown (path B loses its whole readout) | 2 |
 *  | M27 | `rtspInPlay` narrowed to `mode == "RTSP"` (BOTH and unstarted lose them) | 2 |
 *
 * M14 is the one worth explaining. `0` and `—` are two different rows of
 * docs/video.md's failure table: `rtpPkts=0` means a socket exists and has sent
 * nothing (packetiser or target problem), while no sink at all means the RTP
 * path was never built (configuration problem). Collapsing them sends the reader
 * to the wrong half of the table.
 */
class VideoStatusTest {

    private val serving = VideoStatus(
        phase = VideoPhase.SERVING,
        // Past FIRST_FRAME_GRACE_S, so the zero-frame cases below reach the
        // diagnosis rather than the "not yet" branch.
        phaseHeldSeconds = 12,
        rtspUrl = RtspUrl.build("10.55.1.15"),
        rtspUrlRedacted = RtspUrl.redact(RtspUrl.build("10.55.1.15")),
        localAddress = "10.55.1.15",
        localInterface = "wlan0",
        cameraIndex = "LEFT_OR_MAIN",
        streamSource = "DEFAULT_CAMERA",
        streamSourceRange = listOf("DEFAULT_CAMERA"),
        liveStreamListenerRegistered = true,
        availableCameraListenerRegistered = true,
        frameTapRegistered = true,
        availableCameras = listOf("LEFT_OR_MAIN"),
        cameraKeyConnected = true,
        statusUpdates = 12,
        streamingReported = true,
        fps = 30,
        bitrateBps = 2_000_000,
        width = 1280,
        height = 720,
        tap = TapCounters(frames = 340, bytes = 4_500_000, width = 1920, height = 1080, mime = "H264"),
    )

    @Test
    fun `default status is stopped and claims nothing`() {
        val s = VideoStatus()
        assertEquals(VideoPhase.STOPPED, s.phase)
        assertFalse(s.framesArriving)
        assertFalse(s.clientPulling)
        assertEquals("not started", s.verdict())
    }

    @Test
    fun `waiting on registration says so explicitly`() {
        val v = VideoStatus(phase = VideoPhase.WAITING_REGISTRATION).verdict()
        assertTrue(v.contains("registration"))
    }

    @Test
    fun `waiting on the aircraft points at the cable and DJI Fly`() {
        val v = VideoStatus(phase = VideoPhase.WAITING_AIRCRAFT).verdict()
        assertTrue(v.contains("RC-N2"))
        assertTrue(v.contains("DJI Fly"))
    }

    @Test
    fun `serving with zero frames blames the lens, not the network`() {
        val v = serving.copy(tap = TapCounters(frames = 0)).verdict()
        assertTrue(v.contains("zero frames"))
        assertTrue(v.contains(StreamSource.SINGLE_LENS))
    }

    @Test
    fun `serving with frames but zero bitrate means nobody connected`() {
        val v = serving.copy(bitrateBps = 0, zeroBitrateSamples = 9).verdict()
        assertTrue(v.contains("no client"))
        assertTrue(v.contains("RTSP Video Stream"))
    }

    @Test
    fun `serving with no status callbacks is called out separately`() {
        val v = serving.copy(statusUpdates = 0, bitrateBps = 0).verdict()
        assertTrue(v.contains("LiveStreamStatus"))
    }

    @Test
    fun `a healthy stream reports resolution and rate`() {
        val v = serving.verdict()
        assertTrue(v.contains("1280x720"))
        assertTrue(v.contains("30 fps"))
        assertTrue(v.contains("2000 kbps"))
    }

    @Test
    fun `the four silent failures produce four different verdicts`() {
        val verdicts = listOf(
            serving.copy(tap = TapCounters(frames = 0)),
            serving.copy(statusUpdates = 0, bitrateBps = 0),
            serving.copy(bitrateBps = 0),
            serving,
        ).map { it.verdict() }
        assertEquals(4, verdicts.toSet().size)
    }

    @Test
    fun `failure carries the error through`() {
        val v = VideoStatus(phase = VideoPhase.FAILED, lastError = "startStream: boom (-7)").verdict()
        assertTrue(v.contains("boom"))
    }

    @Test
    fun `frame counters drive the booleans, not the phase`() {
        assertTrue(serving.framesArriving)
        assertTrue(serving.clientPulling)
        assertFalse(serving.copy(bitrateBps = 0).clientPulling)
        assertFalse(serving.copy(tap = TapCounters()).framesArriving)
    }

    @Test
    fun `oneLine is a single greppable line with the counters in it`() {
        val line = serving.oneLine()
        assertFalse(line.contains("\n"))
        assertTrue(line.contains("tapFrames=340"))
        assertTrue(line.contains("statusUpdates=12"))
        assertTrue(line.contains("src=DEFAULT_CAMERA"))
        assertTrue(line.contains("res=1280x720"))
    }

    @Test
    fun `neither renderer leaks the RTSP password`() {
        assertFalse(serving.oneLine().contains(RtspUrl.DEFAULT_PASSWORD))
        assertFalse(serving.describe().contains(RtspUrl.DEFAULT_PASSWORD))
    }

    @Test
    fun `describe includes the verdict and the listener bookkeeping`() {
        val text = serving.describe()
        assertTrue(text.contains("verdict:"))
        assertTrue(text.contains("listeners:"))
        assertTrue(text.contains("tap:"))
    }

    // ---- the passthrough path -------------------------------------------------

    private val passthrough = serving.copy(
        mode = "RTP",
        rtspUrl = null,
        rtspUrlRedacted = null,
        rtpTarget = "10.55.1.50:5600",
        statusUpdates = 0,
        streamingReported = false,
        fps = null,
        bitrateBps = null,
        width = null,
        height = null,
        rtp = RtpVideoSink.Counters(
            framesIn = 300, framesSent = 300, packetsSent = 1200, bytesSent = 1_500_000,
        ),
    )

    @Test
    fun `a healthy passthrough says it is not re-encoding`() {
        val v = passthrough.verdict()
        assertTrue(v, v.contains("no re-encode"))
        assertTrue(v, v.contains("10.55.1.50:5600"))
    }

    @Test
    fun `the passthrough is not judged by the RTSP server's bitrate`() {
        // vbps comes from LiveStreamStatus, which never fires when MSDK's server
        // is not running. Reporting "nobody connected" here would be a lie.
        val v = passthrough.verdict()
        assertFalse(v, v.contains("no client pulling"))
        assertFalse(v, v.contains("status listener"))
    }

    @Test
    fun `zero frames still blames the lens even on the passthrough path`() {
        val v = passthrough.copy(tap = TapCounters(frames = 0)).verdict()
        assertTrue(v, v.contains(StreamSource.SINGLE_LENS))
    }

    @Test
    fun `an unroutable RTP target is named rather than left as silence`() {
        val v = passthrough.copy(
            rtp = RtpVideoSink.Counters(
                framesIn = 90, framesSent = 0, sendErrors = 90,
                lastError = "IOException: network unreachable",
            ),
        ).verdict()
        assertTrue(v, v.contains("network unreachable"))
        assertTrue(v, v.contains("10.55.1.50:5600"))
    }

    @Test
    fun `frames that are not Annex-B are called out specifically`() {
        val v = passthrough.copy(
            rtp = RtpVideoSink.Counters(framesIn = 90, framesSent = 0, packetsSent = 0),
        ).verdict()
        assertTrue(v, v.contains("Annex-B"))
    }

    @Test
    fun `dropped frames and send errors are surfaced, not hidden`() {
        val v = passthrough.copy(
            rtp = passthrough.rtp!!.copy(framesDropped = 7, sendErrors = 3),
        ).verdict()
        assertTrue(v, v.contains("7 frames dropped"))
        assertTrue(v, v.contains("3 send errors"))
    }

    @Test
    fun `the RTP counters reach the greppable line`() {
        val line = passthrough.oneLine()
        assertFalse(line.contains("\n"))
        assertTrue(line, line.contains("mode=RTP"))
        assertTrue(line, line.contains("rtpPkts=1200"))
    }

    @Test
    fun `describe shows where RTP is going and what it has sent`() {
        val text = passthrough.describe()
        assertTrue(text, text.contains("10.55.1.50:5600"))
        assertTrue(text, text.contains("rtp:"))
        assertTrue(text, text.contains("mode:"))
    }

    // ---- the GCS hint ---------------------------------------------------------

    @Test
    fun `the RTP hint names QGCs UDP source and port`() {
        // Verified against QGC: videoSourceUDPH264 = "UDP h.264 Video Stream",
        // udpUrl defaults to 0.0.0.0:5600 (src/Settings/VideoSettings.h).
        val hint = passthrough.gcsHint!!
        assertTrue(hint, hint.contains("UDP h.264 Video Stream"))
        assertTrue(hint, hint.contains("0.0.0.0:5600"))
    }

    @Test
    fun `the RTSP hint names QGCs RTSP source and the redacted url`() {
        val hint = serving.gcsHint!!
        assertTrue(hint, hint.contains("RTSP Video Stream"))
        assertTrue(hint, hint.contains("rtsp://***:***@"))
    }

    @Test
    fun `the hint never leaks the password`() {
        assertFalse(serving.gcsHint!!.contains(RtspUrl.DEFAULT_PASSWORD))
    }

    @Test
    fun `there is no hint before anything is configured`() {
        assertEquals(null, VideoStatus().gcsHint)
    }

    @Test
    fun `rtpActive follows the counters, not the mode string`() {
        assertTrue(passthrough.rtpActive)
        assertFalse(serving.rtpActive)
        assertFalse(passthrough.copy(rtp = null).rtpActive)
    }

    // ---- the two counters the failure table is read through --------------------

    /**
     * docs/video.md: "`tapFrames` rising means the aircraft is delivering video;
     * `rtpPkts` rising means we are putting it on the wire. They fail
     * independently and mean different things." A reader with one of the two
     * numbers can diagnose nothing, so both are always printed.
     */
    @Test
    fun `both counters are named and present`() {
        val text = passthrough.counterLines()
        assertTrue(text, text.contains("tapFrames:"))
        assertTrue(text, text.contains("340"))
        assertTrue(text, text.contains("rtpPkts:"))
        assertTrue(text, text.contains("1200"))
    }

    @Test
    fun `the RTP counter is packets, not frames`() {
        // 1200 packets from 300 frames: reading one as the other turns "the
        // packetiser is producing nothing" into "the packetiser is fine".
        val text = passthrough.counterLines()
        assertTrue(text, text.contains("rtpPkts:    1200"))
    }

    @Test
    fun `no RTP sink at all reads as a dash, never as zero`() {
        // Two different rows of the failure table: zero means a socket exists and
        // has sent nothing; a dash means the RTP path was never built.
        val text = passthrough.copy(rtp = null).counterLines()
        assertTrue(text, text.contains("rtpPkts:    —"))
        assertFalse(text, text.contains("rtpPkts:    0"))
    }

    @Test
    fun `a sink that has sent nothing reads as zero, not as a dash`() {
        val text = passthrough.copy(rtp = RtpVideoSink.Counters()).counterLines()
        assertTrue(text, text.contains("rtpPkts:    0"))
    }

    @Test
    fun `the counters name the destination so a wrong target is visible`() {
        assertTrue(passthrough.counterLines().contains("10.55.1.50:5600"))
    }

    @Test
    fun `describe carries both counters, because the screen is the only readout`() {
        // Without adb. That is the requirement — the phone is strapped to the RC.
        val text = passthrough.describe()
        assertTrue(text, text.contains("tapFrames:"))
        assertTrue(text, text.contains("rtpPkts:"))
    }

    // ---- the first-frame grace -------------------------------------------------

    /**
     * On the passthrough path SERVING is reached the instant
     * `addReceiveStreamListener` returns, so zero frames one tick later is "not
     * yet", not "wrong lens". Blaming the lens then is crying wolf at exactly the
     * moment an operator is watching the screen hardest.
     */
    @Test
    fun `zero frames immediately after SERVING is not yet a diagnosis`() {
        val v = passthrough.copy(tap = TapCounters(frames = 0), phaseHeldSeconds = 1).verdict()
        assertTrue(v, v.contains("waiting for the first frame"))
        assertFalse(v, v.contains(StreamSource.SINGLE_LENS))
    }

    @Test
    fun `the grace expires, and then it is a diagnosis with a duration`() {
        val v = passthrough.copy(
            tap = TapCounters(frames = 0),
            phaseHeldSeconds = VideoStatus.FIRST_FRAME_GRACE_S,
        ).verdict()
        assertTrue(v, v.contains(StreamSource.SINGLE_LENS))
        assertTrue(v, v.contains("${VideoStatus.FIRST_FRAME_GRACE_S}s"))
    }

    // ---- what the passthrough does not have to explain --------------------------

    /**
     * `live:` is `LiveStreamStatus`, which never fires when MSDK's RTSP server is
     * not running. Printing `updates=0 streaming=no fps=— vbps=—` above a working
     * passthrough is four lines of apparent failure describing a server nobody
     * asked for — measured on hardware while RTP was streaming at 24 fps.
     */
    @Test
    fun `the passthrough hides the RTSP-only rows`() {
        val text = passthrough.describe()
        assertFalse(text, text.contains("live:"))
        assertFalse(text, text.contains("url:"))
        assertFalse(text, text.contains("local ip:"))
        // …while keeping everything the passthrough is actually judged on.
        assertTrue(text, text.contains("tapFrames:"))
        assertTrue(text, text.contains("rtpPkts:"))
        assertTrue(text, text.contains("verdict:"))
    }

    @Test
    fun `the RTSP path keeps them, and so does an unstarted one`() {
        assertTrue(serving.describe().contains("live:"))
        assertTrue(serving.describe().contains("url:"))
        // Nothing started yet: err towards showing, since hiding a row is only
        // safe once we know it is irrelevant.
        assertTrue(VideoStatus().describe().contains("url:"))
    }

    @Test
    fun `BOTH mode keeps the RTSP rows`() {
        assertTrue(passthrough.copy(mode = "BOTH").describe().contains("live:"))
    }

    @Test
    fun `the grace does not apply before the tap is even registered`() {
        // No subscription means the question "why no frames?" has an earlier
        // answer than the lens, and "waiting for the first frame" would be a lie.
        val v = passthrough.copy(
            tap = TapCounters(frames = 0), frameTapRegistered = false, phaseHeldSeconds = 1,
        ).verdict()
        assertFalse(v, v.contains("waiting for the first frame"))
    }
}
