package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [GimbalRecenter] — the decision-free detector that turns DJI's measured landing gimbal slew
 * into one `gimbal_recentered_by_dji` event. The fixtures are the measured shapes from
 * `landingdata.md` §2: the −90 → 0 slew at ~250–300 °/s sampled at ~25 Hz, the 1 Hz held
 * stream around it, and the mode key arriving up to 120 ms *after* the slew begins.
 *
 * Mutation-checked 2026-07-28 by the house protocol (whole 2412-test suite per mutant,
 * test-results deleted first, measured, reverted):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the event never emitted (`return null` on fire) | 3 |
 *  | threshold broken (`MOVE_DEG` 30° → 3°) | 1 |
 */
class GimbalRecenterTest {

    /** The verbatim measured slew curve (`landingdata.md` §2, flight 151517), 40 ms apart. */
    private val slew = listOf(-81.6, -68.2, -52.9, -36.7, -19.7, -6.3, -0.7, -0.1)

    @Test
    fun `the measured landing slew fires exactly one event, with the movement in the message`() {
        val d = GimbalRecenter()
        // A minute of held nadir at 1 Hz first: never an event.
        var t = 0L
        repeat(60) {
            assertNull(d.sample(t, -90.0, "GPS_ATTI"))
            t += 1_000
        }
        // The slew, in CONFIRM_LANDING.
        var fired = 0
        var message: String? = null
        for (p in slew) {
            d.sample(t, p, "CONFIRM_LANDING")?.let { fired++; message = it }
            t += 40
        }
        assertEquals(1, fired)
        assertNotNull(message)
        assertTrue(message!!.contains("CONFIRM_LANDING"))
        // Held level through touchdown: still one event.
        repeat(10) {
            assertNull(d.sample(t, 0.0, "CONFIRM_LANDING"))
            t += 1_000
        }
    }

    @Test
    fun `the mode arriving after the slew begins still fires - the measured delivery skew`() {
        val d = GimbalRecenter()
        var t = 0L
        d.sample(t, -90.0, "GPS_ATTI")
        // First slew samples arrive up to 120 ms before the mode key (measured): the early
        // samples carry the old mode, the later ones the landing mode.
        val modes = listOf("GPS_ATTI", "GPS_ATTI", "GPS_ATTI", "CONFIRM_LANDING",
            "CONFIRM_LANDING", "CONFIRM_LANDING", "CONFIRM_LANDING", "CONFIRM_LANDING")
        var fired = 0
        for ((p, mode) in slew.zip(modes)) {
            t += 40
            if (d.sample(t, p, mode) != null) fired++
        }
        assertEquals(1, fired)
    }

    @Test
    fun `a slew outside a landing mode is not a DJI recenter - operator aiming stays unnamed`() {
        val d = GimbalRecenter()
        var t = 0L
        d.sample(t, -90.0, "GPS_ATTI")
        for (p in slew) {
            t += 40
            assertNull("mode GPS_ATTI must never fire", d.sample(t, p, "GPS_ATTI"))
        }
        // JOYSTICK — the mode every tag descent flies in — must not fire either: during our own
        // virtual-stick descents the gimbal held nadir on every measured flight (§2.3), and a
        // commanded aim in JOYSTICK is ours, not DJI's.
        val d2 = GimbalRecenter()
        d2.sample(0, -90.0, "JOYSTICK")
        assertNull(d2.sample(200, -30.0, "JOYSTICK"))
    }

    @Test
    fun `slow movement never fires - the threshold is a slew, not a drag`() {
        val d = GimbalRecenter()
        // QGC's closed-loop drag: a few degrees per 100 ms tick, even in a landing mode.
        var t = 0L
        var p = -90.0
        while (p < 0.0) {
            assertNull(d.sample(t, p, "CONFIRM_LANDING"))
            t += 500
            p += 10.0 // 20 deg/s: fast for a drag, far under the 250-300 deg/s slew
        }
    }

    @Test
    fun `a reading gap resets the reference - a gap is not a movement`() {
        val d = GimbalRecenter()
        d.sample(0, -90.0, "CONFIRM_LANDING")
        d.sample(100, null, "CONFIRM_LANDING")
        // The next reading is far from the pre-gap one, inside what would have been the
        // window: no event, because nothing measured the path between them.
        assertNull(d.sample(200, 0.0, "CONFIRM_LANDING"))
    }

    @Test
    fun `after a quiet window the detector re-arms - a second landing gets its own event`() {
        val d = GimbalRecenter()
        var t = 0L
        d.sample(t, -90.0, "CONFIRM_LANDING")
        var fired = 0
        for (p in slew) {
            t += 40
            if (d.sample(t, p, "CONFIRM_LANDING") != null) fired++
        }
        assertEquals(1, fired)
        // Held, then a whole new flight's slew minutes later.
        repeat(120) {
            t += 1_000
            d.sample(t, -90.0, "GPS_ATTI")
        }
        for (p in slew) {
            t += 40
            if (d.sample(t, p, "AUTO_LANDING") != null) fired++
        }
        assertEquals(2, fired)
    }
}
