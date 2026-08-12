package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The Q4 policy shapes. Small on purpose — each policy is a pure value, and the engine's
 * execution of a plan is covered in `GuidedStickEngineTest` — but the *shipped choice* and the
 * distinguishing property of each alternative are pinned here, because swapping the policy is
 * designed to be a one-line change and a one-line change deserves a failing test when it is
 * accidental.
 */
class LinkLossPolicyTest {

    @Test
    fun `the shipped policy is decelerate-then-handback - Ivan's Q4 answer`() {
        assertTrue(LinkLossPolicy.SHIPPED is DecelerateThenHandback)
        assertEquals("DecelerateThenHandback", LinkLossPolicy.SHIPPED.name)
    }

    @Test
    fun `decelerate-then-handback ramps for half a second, holds briefly, releases`() {
        val plan = DecelerateThenHandback().plan()
        assertEquals(500L, plan.rampToZeroMs)
        assertEquals(1_000L, plan.holdZeroMs)
        assertTrue(plan.release)
    }

    @Test
    fun `freeze-and-hold never releases - a returning link resumes control`() {
        val plan = FreezeAndHold().plan()
        assertEquals(500L, plan.rampToZeroMs)
        assertNull("holdZeroMs null means hold forever", plan.holdZeroMs)
        assertFalse(plan.release)
    }

    @Test
    fun `instant handback releases with no deceleration at all`() {
        val plan = InstantHandback().plan()
        assertEquals(0L, plan.rampToZeroMs)
        assertEquals(0L, plan.holdZeroMs)
        assertTrue(plan.release)
    }

    @Test
    fun `policy names are stable - they are logged at engage time and read from flight logs`() {
        assertEquals("DecelerateThenHandback", DecelerateThenHandback().name)
        assertEquals("FreezeAndHold", FreezeAndHold().name)
        assertEquals("InstantHandback", InstantHandback().name)
    }
}
