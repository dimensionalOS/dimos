package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.StatusTexts
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The sentences the gimbal puts in front of an operator, and the 50 bytes they have to fit in.
 *
 * The failure this suite guards against is not a crash. `STATUSTEXT.text` is a fixed-width
 * `char[50]` with no length prefix, so anything longer is **silently cut on the wire** and the
 * operator reads a truncated DJI error name and searches for the wrong string. The gimbal makes
 * that likelier than M2 did: `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` is 39 bytes before any
 * framing at all.
 *
 * Mutation-checked 2026-07-26 across the whole gimbal suite. **Every one was caught.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `preferring` returns the framed form regardless of length | 4 |
 *  | `preferring` always drops the framing | 7 |
 *  | the clamp is dropped for an over-long DJI name | 2 |
 *  | `YAW_UNAVAILABLE` lengthened past 50 bytes | 4 |
 */
class GimbalStatusTextsTest {

    private fun bytes(s: String) = s.toByteArray(Charsets.UTF_8).size

    /** The name #527's reporter actually saw on a Mini 4 Pro. 39 bytes. */
    private val realGimbalError = "SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW"

    @Test
    fun `every fixed sentence fits the field`() {
        listOf(
            GimbalStatusTexts.YAW_UNAVAILABLE,
            GimbalStatusTexts.RATE_UNSUPPORTED,
        ).forEach {
            assertTrue("$it is ${bytes(it)} bytes", bytes(it) <= StatusTexts.MAX_BYTES)
        }
    }

    @Test
    fun `the yaw sentence says what the airframe cannot do, not what the operator did wrong`() {
        // It fires on QGC's Center and Tilt-90 buttons, which hard-code yaw = 0 whether or not
        // the operator meant to turn the camera. Blaming their input would be wrong twice out of
        // three times.
        assertEquals("Gimbal: pitch only, no yaw on this airframe", GimbalStatusTexts.YAW_UNAVAILABLE)
    }

    @Test
    fun `DJI's word survives when ours cannot`() {
        val text = GimbalStatusTexts.refusal(realGimbalError)
        // "Gimbal refused by DJI: " + 39 bytes is 62, so the framing goes and the name stays whole.
        assertEquals(realGimbalError, text)
        assertTrue(bytes(text) <= StatusTexts.MAX_BYTES)
    }

    @Test
    fun `a short DJI name keeps the framing that says who refused`() {
        // The operator's next question is always *who* refused — our interlock and the aircraft
        // lead to different actions.
        assertEquals("Gimbal refused by DJI: GIMBAL_BUSY", GimbalStatusTexts.refusal("GIMBAL_BUSY"))
    }

    @Test
    fun `an unreachable aircraft is a failure, not a refusal`() {
        // "Refused" attributes a decision to the gimbal. If nothing was reachable there was no
        // decision, and saying so would invent one.
        assertEquals("Gimbal failed: NO_PRODUCT", GimbalStatusTexts.unavailable("NO_PRODUCT"))
    }

    @Test
    fun `a DJI name too long even on its own is cut, never sent over the field width`() {
        val monstrous = "SDK_SERVICE_GIMBAL_" + "X".repeat(80)
        val text = GimbalStatusTexts.refusal(monstrous)
        assertEquals(StatusTexts.MAX_BYTES, bytes(text))
        assertTrue(monstrous.startsWith(text))
    }

    @Test
    fun `a thrown implementation is reported by its message, or its class when it has none`() {
        assertEquals(
            "Gimbal failed: port not wired",
            GimbalStatusTexts.threw(IllegalStateException("port not wired")),
        )
        assertEquals(
            "Gimbal failed: IllegalStateException",
            GimbalStatusTexts.threw(IllegalStateException()),
        )
    }

    @Test
    fun `an asynchronous error is marked as DJI's own`() {
        assertEquals("DJI: GIMBAL_BUSY", GimbalStatusTexts.djiError("GIMBAL_BUSY"))
        // "DJI: " + 39 bytes is 44, so even the longest name we have seen keeps its prefix here.
        assertEquals("DJI: $realGimbalError", GimbalStatusTexts.djiError(realGimbalError))
    }

    @Test
    fun `the byte clamp is the one in command StatusTexts, not a second copy`() {
        // There must be exactly one UTF-8-safe cut in this project. A second implementation is
        // how one of them ends up splitting a character in half on a ground station.
        val overlong = "é".repeat(40)
        assertEquals(StatusTexts.clamp(overlong), GimbalStatusTexts.refusal(overlong))
    }
}
