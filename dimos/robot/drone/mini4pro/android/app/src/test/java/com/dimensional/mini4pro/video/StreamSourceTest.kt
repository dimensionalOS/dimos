package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The single-lens trap (MSDK issue #641). `WIDE_CAMERA` is accepted by the SDK
 * and yields zero frames with no error, so the lens decision is worth testing.
 */
class StreamSourceTest {

    /** What the real aircraft reported in the 2026-07-25 sweep. */
    private val measuredRange = listOf("DEFAULT_CAMERA")

    @Test
    fun `healthy Mini 4 Pro writes nothing`() {
        val plan = StreamSource.plan(measuredRange, "DEFAULT_CAMERA")
        assertEquals(StreamSource.Action.KEEP, plan.action)
        assertNull(plan.desired)
        assertNull(plan.warning)
    }

    @Test
    fun `a stale WIDE_CAMERA selection is switched back and warned about`() {
        val plan = StreamSource.plan(measuredRange, "WIDE_CAMERA")
        assertEquals(StreamSource.Action.SWITCH, plan.action)
        assertEquals("DEFAULT_CAMERA", plan.desired)
        assertNotNull(plan.warning)
        assertTrue(plan.warning!!.contains("641"))
    }

    @Test
    fun `unset source is switched to the single lens`() {
        val plan = StreamSource.plan(measuredRange, null)
        assertEquals(StreamSource.Action.SWITCH, plan.action)
        assertEquals("DEFAULT_CAMERA", plan.desired)
    }

    @Test
    fun `unreadable range with the right source already selected leaves it alone`() {
        val plan = StreamSource.plan(emptyList(), "DEFAULT_CAMERA")
        assertEquals(StreamSource.Action.KEEP, plan.action)
        assertTrue(plan.warning!!.contains("unreadable"))
    }

    @Test
    fun `unreadable range with a wrong source still rescues DEFAULT_CAMERA`() {
        // Never leave the airframe on a lens that produces no frames just because
        // the range key was not cached yet.
        val plan = StreamSource.plan(emptyList(), "WIDE_CAMERA")
        assertEquals(StreamSource.Action.SWITCH, plan.action)
        assertEquals("DEFAULT_CAMERA", plan.desired)
    }

    @Test
    fun `UNKNOWN entries do not count as a readable range`() {
        val plan = StreamSource.plan(listOf("UNKNOWN", ""), "DEFAULT_CAMERA")
        assertEquals(StreamSource.Action.KEEP, plan.action)
        assertTrue(plan.warning!!.contains("unreadable"))
    }

    @Test
    fun `a multi-lens airframe is flagged as not the one we measured`() {
        val plan = StreamSource.plan(listOf("DEFAULT_CAMERA", "WIDE_CAMERA"), "DEFAULT_CAMERA")
        assertEquals(StreamSource.Action.KEEP, plan.action)
        assertTrue(plan.warning!!.contains("2 stream sources"))
    }

    @Test
    fun `we never pick WIDE_CAMERA even when it is offered`() {
        val plan = StreamSource.plan(listOf("WIDE_CAMERA", "DEFAULT_CAMERA"), "WIDE_CAMERA")
        assertEquals("DEFAULT_CAMERA", plan.desired)
    }

    @Test
    fun `an airframe without DEFAULT_CAMERA falls back to its first supported source`() {
        val plan = StreamSource.plan(listOf("INFRARED_CAMERA"), null)
        assertEquals(StreamSource.Action.SWITCH, plan.action)
        assertEquals("INFRARED_CAMERA", plan.desired)
        assertTrue(plan.warning!!.contains("not in the supported range"))
    }

    @Test
    fun `an airframe without DEFAULT_CAMERA already on a supported source is left alone`() {
        val plan = StreamSource.plan(listOf("INFRARED_CAMERA"), "INFRARED_CAMERA")
        assertEquals(StreamSource.Action.KEEP, plan.action)
        assertTrue(plan.warning!!.contains("not in the supported range"))
    }
}
