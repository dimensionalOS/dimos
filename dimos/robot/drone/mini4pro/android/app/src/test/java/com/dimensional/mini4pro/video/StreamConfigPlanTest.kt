package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * One rule, from the bytecode, that nothing else would catch.
 *
 * `LiveStreamManager.setLiveStreamQuality` forces `mLiveVideoBitRateMode =
 * MANUAL` and calls `VideoBitRateManager.stopAutoVideoBitRate()` as a side
 * effect. Set the bitrate mode first and the quality second, and AUTO is silently
 * undone: the stream runs at MANUAL's constructor default and adapts to nothing.
 * There is no error, no callback and no field to read it back from
 * (`getLiveVideoBitrateMode` returns the field the quality setter just
 * overwrote), so on hardware it presents as "the link is bad".
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests
 * counted, then reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | QUALITY appended after the bitrate steps | 4 |
 *  | BITRATE_MODE_AUTO emitted when a bitrate was given | 1 |
 *  | BITRATE_VALUE emitted before BITRATE_MODE_MANUAL | 1 |
 *  | BITRATE_VALUE dropped from the manual branch | 1 |
 *  | SETTINGS emitted after QUALITY | 4 |
 *  | non-positive bitrate accepted | 2 |
 */
class StreamConfigPlanTest {

    private fun steps(bitrate: Int?) = StreamConfigPlan.of(VideoQuality.HD, bitrate).steps

    @Test
    fun `quality is always applied before anything touches the bitrate`() {
        // The whole reason this class exists.
        listOf(null, 2_000_000).forEach { bitrate ->
            val s = steps(bitrate)
            val quality = s.indexOf(StreamConfigPlan.Step.QUALITY)
            val firstBitrate = s.indexOfFirst {
                it == StreamConfigPlan.Step.BITRATE_MODE_AUTO ||
                    it == StreamConfigPlan.Step.BITRATE_MODE_MANUAL ||
                    it == StreamConfigPlan.Step.BITRATE_VALUE
            }
            assertTrue("bitrate=$bitrate gave $s", quality in 0 until firstBitrate)
        }
    }

    @Test
    fun `settings are applied before quality`() {
        val s = steps(null)
        assertTrue(
            s.toString(),
            s.indexOf(StreamConfigPlan.Step.SETTINGS) < s.indexOf(StreamConfigPlan.Step.QUALITY),
        )
    }

    @Test
    fun `no bitrate selects the automatic mode and sets no value`() {
        assertEquals(
            listOf(
                StreamConfigPlan.Step.CAMERA_INDEX,
                StreamConfigPlan.Step.SETTINGS,
                StreamConfigPlan.Step.QUALITY,
                StreamConfigPlan.Step.BITRATE_MODE_AUTO,
            ),
            steps(null),
        )
    }

    @Test
    fun `a bitrate selects manual mode and then the value, in that order`() {
        assertEquals(
            listOf(
                StreamConfigPlan.Step.CAMERA_INDEX,
                StreamConfigPlan.Step.SETTINGS,
                StreamConfigPlan.Step.QUALITY,
                StreamConfigPlan.Step.BITRATE_MODE_MANUAL,
                StreamConfigPlan.Step.BITRATE_VALUE,
            ),
            steps(4_000_000),
        )
    }

    @Test
    fun `the two bitrate modes are never both planned`() {
        listOf(null, 1_000_000).forEach { bitrate ->
            val s = steps(bitrate)
            assertTrue(
                s.toString(),
                !(s.contains(StreamConfigPlan.Step.BITRATE_MODE_AUTO) &&
                    s.contains(StreamConfigPlan.Step.BITRATE_MODE_MANUAL)),
            )
        }
    }

    @Test
    fun `the camera index is pinned before the stream is described`() {
        assertEquals(StreamConfigPlan.Step.CAMERA_INDEX, steps(null).first())
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a zero bitrate is refused rather than sent to the aircraft`() {
        StreamConfigPlan.of(VideoQuality.HD, 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a negative bitrate is refused`() {
        StreamConfigPlan.of(VideoQuality.HD, -1)
    }

    @Test
    fun `the plan prints as an ordered, readable sequence for the log`() {
        val text = StreamConfigPlan.of(VideoQuality.FULL_HD, null).toString()
        assertTrue(text, text.startsWith("CAMERA_INDEX → SETTINGS → QUALITY"))
    }
}
