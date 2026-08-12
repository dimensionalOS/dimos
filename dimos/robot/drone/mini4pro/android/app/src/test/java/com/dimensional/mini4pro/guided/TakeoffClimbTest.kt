package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.ClimbArm
import com.dimensional.mini4pro.command.CommandDispatcher
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [TakeoffClimb] alone — the two-phase takeoff's phase machine as pure logic, with no engine, no
 * port, no clock it did not receive and no aircraft.
 *
 * The one question it answers is *when DJI's own takeoff is finished*, and the whole reason it is
 * its own class is that the obvious answer is wrong. Two real 2026-07-27 records
 * (`tmp/session-logs/orbit-first.jsonl`, `tmp/session-logs/orbit-real-air.jsonl`) show `isFlying`
 * going true **1.4 s and 0.6 s into `AUTO_TAKE_OFF`**, with 2.0 s and 2.4 s of DJI's own climb
 * still to run. Both sequences are replayed here tick by tick, in
 * `the measured orbit-first sequence` / `the measured orbit-real-air sequence`, and both suites
 * fail if the machine ever fires while DJI is still flying.
 *
 * Written to fail loudly for:
 *
 *  - firing on `isFlying` alone — the two-controllers-on-one-aircraft bug
 *  - firing on a *stale* `GPS_ATTI` left over from before the takeoff (`flightMode` is
 *    change-driven and carries no staleness limit), which looks identical to DJI having let go
 *  - firing without ever having seen the aircraft on the ground (a takeoff commanded mid-flight,
 *    whose climb target is *below* the aircraft)
 *  - an armed climb with no expiry
 *  - a climb armed for a height DJI's own hop already delivers
 *  - the ceiling capping silently, or not at all
 *  - a fired climb that can fire twice
 *
 * Mutation-checked 2026-07-27, one breakage at a time, `./gradlew testDebugUnitTest` after each,
 * code reverted after each. Counts are across the whole suite (this file plus
 * `GuidedTakeoffClimbTest` plus `CommandDispatcherTest`) and are **measured, not estimated**:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | flight-mode conjunct dropped (fires on `isFlying` alone, mid-`AUTO_TAKE_OFF`) | 6 |
 *  | `TAKEOFF_MODES` emptied (every mode reads as "DJI has let go") | 15 |
 *  | `sawNotFlying` requirement dropped (arming while airborne fires at once) | 2 |
 *  | `sawTakeoffMode` requirement dropped (a stale `GPS_ATTI` ends the takeoff) | 1 |
 *  | `sawTakeoffMode` recorded only once already flying (the measured order lost) | 4 |
 *  | null flight mode treated as "not a takeoff mode" | 1 |
 *  | null `isFlying` treated as flying | 1 |
 *  | `WAIT_LIMIT_MS` expiry removed (an armed climb waits forever) | 5 |
 *  | expiry checked *after* the climb conjuncts instead of before | 5 |
 *  | `Climb` no longer consumes the pending state (it can fire twice) | 3 |
 *  | `NO_CLIMB_BELOW_M` gate removed (a 1.5 m request arms a climb) | 2 |
 *  | ceiling cap removed at arm (121.92 m arms 121.92 m) | 2 |
 *  | ceiling cap applied without reporting `capped` | 2 |
 *  | non-finite request armed instead of refused | 1 |
 *  | `cancel` stops reporting whether anything was armed | 4 |
 *
 * **No mutant in either table survived**, but three of them had to be *made* killable and that is
 * the part worth keeping.
 *
 * Adding the [TakeoffClimb.Pending.sawTakeoffMode] conjunct took `sawNotFlying`, the null-mode
 * branch and the null-`isFlying` branch from killed to **alive**: each test had been proving
 * "the climb does not fire", and once a second conjunct could hold the climb back for its own
 * reason, none of them could say *which* one did. Same shape as the target-survives-abort mutant
 * in `GuidedRepositionTest`'s header, and the same fix — every one of those three tests now
 * satisfies every *other* conjunct explicitly, and two of them go on to assert that the very next
 * `observe` **does** fire, which is what proves nothing else was holding it. Defence in depth is
 * kept; each layer is pinned alone.
 */
class TakeoffClimbTest {

    private companion object {
        const val T0 = 10_000L

        /** QGC's own floor: `Vehicle::minimumTakeoffAltitudeMeters()`, `FirmwarePlugin.h:204`. */
        const val QGC_MIN_TAKEOFF_M = 3.048
    }

    // ------------------------------------------------------------------- arming

    @Test
    fun `a QGC-shaped request arms a climb to exactly that height`() {
        val c = TakeoffClimb()
        val armed = c.arm(QGC_MIN_TAKEOFF_M, T0)
        assertEquals(ClimbArm.Armed(QGC_MIN_TAKEOFF_M, capped = false), armed)
        assertTrue(c.armed)
        assertEquals(QGC_MIN_TAKEOFF_M, c.armedAltitudeM!!, 1e-9)
    }

    @Test
    fun `a request at or below DJI's own hop arms nothing at all`() {
        // The threshold is DJI's 1.2 m plus the reposition law's own vertical acceptance: a target
        // inside that is one the arrival test would satisfy on the first tick, so engaging virtual
        // stick for it would be a whole engagement to fly nowhere.
        for (request in listOf(0.5, CommandDispatcher.DJI_TAKEOFF_HEIGHT_M, 2.0, TakeoffClimb.NO_CLIMB_BELOW_M)) {
            val c = TakeoffClimb()
            assertEquals("$request m", ClimbArm.NothingToDo, c.arm(request, T0))
            assertFalse("$request m", c.armed)
            assertNull(c.armedAltitudeM)
        }
    }

    @Test
    fun `just above the threshold does arm - the gate is a boundary, not a band`() {
        val c = TakeoffClimb()
        val request = TakeoffClimb.NO_CLIMB_BELOW_M + 0.01
        assertEquals(ClimbArm.Armed(request, capped = false), c.arm(request, T0))
    }

    @Test
    fun `above the ceiling is capped to it and says so`() {
        val c = TakeoffClimb()
        val armed = c.arm(GuidedEnvelope.CEILING_M + 21.92, T0)
        // 121.92 m is the top of QGC's own takeoff slider, so this is reachable from a stock
        // ground station: MAX_TAKEOFF_HEIGHT_M lets it through and the M3 ceiling binds it.
        assertEquals(ClimbArm.Armed(GuidedEnvelope.CEILING_M, capped = true), armed)
        assertEquals(GuidedEnvelope.CEILING_M, c.armedAltitudeM!!, 1e-9)
    }

    @Test
    fun `exactly at the ceiling is not capped`() {
        val c = TakeoffClimb()
        assertEquals(
            ClimbArm.Armed(GuidedEnvelope.CEILING_M, capped = false),
            c.arm(GuidedEnvelope.CEILING_M, T0),
        )
    }

    @Test
    fun `a non-finite request arms nothing - NaN is false in every comparison`() {
        for (request in listOf(Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY)) {
            val c = TakeoffClimb()
            assertEquals("$request", ClimbArm.NothingToDo, c.arm(request, T0))
            assertFalse(c.armed)
        }
    }

    @Test
    fun `arming again replaces the previous climb`() {
        val c = TakeoffClimb()
        c.arm(10.0, T0)
        assertEquals(ClimbArm.Armed(25.0, capped = false), c.arm(25.0, T0 + 1_000))
        assertEquals(25.0, c.armedAltitudeM!!, 1e-9)
    }

    @Test
    fun `the camera flag rides the handback it was armed with, and is consumed with it`() {
        // The phone sequence's nadir move fires at the handback and nowhere else; the machine
        // carries the flag and never acts on it. Consumed with the pending state, so a caller
        // that missed the decision cannot aim a camera off a machine that has forgotten why.
        val c = TakeoffClimb()
        c.arm(10.0, T0, aimCameraNadir = true)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(
            TakeoffClimb.Decision.HandedBack(10.0, aimCameraNadir = true, origin = ControlOrigin.MAVLINK),
            c.observe(true, "GPS_ATTI", T0 + 5_000),
        )
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(true, "GPS_ATTI", T0 + 5_100))
    }

    @Test
    fun `a QGC-shaped arm hands back without the camera flag`() {
        val c = TakeoffClimb()
        c.arm(10.0, T0)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(
            TakeoffClimb.Decision.HandedBack(10.0, aimCameraNadir = false, origin = ControlOrigin.MAVLINK),
            c.observe(true, "GPS_ATTI", T0 + 5_000),
        )
    }

    @Test
    fun `re-arming without the flag drops a previous arm's flag - the replacement is total`() {
        val c = TakeoffClimb()
        c.arm(10.0, T0, aimCameraNadir = true)
        c.arm(25.0, T0 + 500)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(
            TakeoffClimb.Decision.HandedBack(25.0, aimCameraNadir = false, origin = ControlOrigin.MAVLINK),
            c.observe(true, "GPS_ATTI", T0 + 5_000),
        )
    }

    @Test
    fun `the origin rides the handback it was armed with - landing08's missing label`() {
        // The door's identity travels exactly as the camera flag does: carried, never acted on
        // by this machine, consumed with the climb. A PHONE arm that handed back MAVLINK would
        // re-create landing08 — the climb engages and the Q4 watchdog kills it 1.6 s later on a
        // flight with no QGC to be heard from.
        val c = TakeoffClimb()
        c.arm(10.0, T0, aimCameraNadir = true, origin = ControlOrigin.PHONE)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(
            TakeoffClimb.Decision.HandedBack(10.0, aimCameraNadir = true, origin = ControlOrigin.PHONE),
            c.observe(true, "GPS_ATTI", T0 + 5_000),
        )
    }

    @Test
    fun `re-arming replaces the origin too - a stale phone label cannot ride a QGC climb`() {
        // The replacement-is-total property, applied to the origin: the second arm's default
        // (MAVLINK, the fail-safe label) wins, so a phone arm abandoned before DJI flew cannot
        // donate its alive-by-identity liveness to a takeoff the ground station commanded.
        val c = TakeoffClimb()
        c.arm(10.0, T0, aimCameraNadir = true, origin = ControlOrigin.PHONE)
        c.arm(25.0, T0 + 500)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(
            TakeoffClimb.Decision.HandedBack(25.0, aimCameraNadir = false, origin = ControlOrigin.MAVLINK),
            c.observe(true, "GPS_ATTI", T0 + 5_000),
        )
    }

    // ------------------------------------------- when DJI's takeoff is finished

    @Test
    fun `nothing armed is Idle whatever the aircraft reports`() {
        val c = TakeoffClimb()
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(true, "GPS_ATTI", T0))
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(false, "MOTOR_START", T0 + 60_000))
    }

    @Test
    fun `the measured orbit-first sequence fires once, and only when DJI lets go`() {
        // tmp/session-logs/orbit-first.jsonl, t in seconds from the recorder's start:
        //   30.802 motorsOn true, flightMode GPS_ATTI -> MOTOR_START
        //   32.403 flightMode -> AUTO_TAKE_OFF
        //   33.802 isFlying false -> true      <-- 2.0 s of DJI's climb still to run
        //   35.803 flightMode -> GPS_ATTI      <-- this is the moment
        val c = TakeoffClimb()
        c.arm(QGC_MIN_TAKEOFF_M, 30_000)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "GPS_ATTI", 30_500))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "MOTOR_START", 30_802))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "AUTO_TAKE_OFF", 32_403))
        // The trap: flying, but DJI is still flying the takeoff.
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "AUTO_TAKE_OFF", 33_802))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "AUTO_TAKE_OFF", 35_000))
        assertEquals(TakeoffClimb.Decision.HandedBack(QGC_MIN_TAKEOFF_M, origin = ControlOrigin.MAVLINK), c.observe(true, "GPS_ATTI", 35_803))
    }

    @Test
    fun `the measured orbit-real-air sequence fires once, and only when DJI lets go`() {
        // tmp/session-logs/orbit-real-air.jsonl: MOTOR_START 26.611, AUTO_TAKE_OFF 28.011,
        // isFlying true 28.611 (0.6 s in, 2.4 s to run), GPS_ATTI 31.011.
        val c = TakeoffClimb()
        c.arm(5.0, 26_000)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "GPS_ATTI", 26_200))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "MOTOR_START", 26_611))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "AUTO_TAKE_OFF", 28_011))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "AUTO_TAKE_OFF", 28_611))
        assertEquals(TakeoffClimb.Decision.HandedBack(5.0, origin = ControlOrigin.MAVLINK), c.observe(true, "GPS_ATTI", 31_011))
    }

    @Test
    fun `the climb fires exactly once - the decision consumes the pending state`() {
        val c = TakeoffClimb()
        c.arm(5.0, T0)
        c.observe(false, "GPS_ATTI", T0)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(TakeoffClimb.Decision.HandedBack(5.0, origin = ControlOrigin.MAVLINK), c.observe(true, "GPS_ATTI", T0 + 5_000))
        assertFalse(c.armed)
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(true, "GPS_ATTI", T0 + 5_100))
    }

    @Test
    fun `a takeoff mode must have been seen - a stale GPS_ATTI is not a finished takeoff`() {
        // The race this closes: `flightMode` is a change-driven key with no staleness limit, so a
        // GPS_ATTI left over from *before* the takeoff, read on the tick isFlying flips, would
        // otherwise look exactly like DJI having let go — and the climb would engage in the
        // middle of AUTO_TAKE_OFF.
        val c = TakeoffClimb()
        c.arm(5.0, T0)
        c.observe(false, "GPS_ATTI", T0)
        for (t in 1_000L until TakeoffClimb.WAIT_LIMIT_MS step 1_000) {
            assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "GPS_ATTI", T0 + t))
        }
        assertTrue("it waits rather than firing", c.armed)
    }

    @Test
    fun `seeing the takeoff mode before the aircraft reports flying is the measured order`() {
        // Both 2026-07-27 records reach MOTOR_START 4.2 s and 2.0 s *before* isFlying goes true,
        // so the flag is always set by the time it is needed. This pins that a takeoff mode seen
        // while still on the ground counts.
        val c = TakeoffClimb()
        c.arm(5.0, T0)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "MOTOR_START", T0 + 1_000))
        assertEquals(TakeoffClimb.Decision.HandedBack(5.0, origin = ControlOrigin.MAVLINK), c.observe(true, "GPS_ATTI", T0 + 5_000))
    }

    @Test
    fun `flying without ever having been seen on the ground never fires`() {
        // A takeoff commanded while the aircraft is already in the air. QGC does not offer the
        // button then and DJI refuses one with the motors running, so this should be unobservable
        // — and if it ever is, waiting is the failure worth having, because the climb's target
        // would be *below* an aircraft that is already up.
        //
        // **Every other conjunct is deliberately satisfied here**: a full DJI mode trail goes by,
        // it ends in a normal mode, and `isFlying` is true throughout. Only the ground was never
        // seen. Without that, this test would pass for the wrong reason — `sawTakeoffMode` would
        // hold the climb back and the `sawNotFlying` mutant would survive behind it.
        val c = TakeoffClimb()
        c.arm(3.0, T0)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "MOTOR_START", T0 + 500))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "AUTO_TAKE_OFF", T0 + 1_500))
        for (t in 2_000L until TakeoffClimb.WAIT_LIMIT_MS step 1_000) {
            assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "GPS_ATTI", T0 + t))
        }
        assertEquals(TakeoffClimb.Decision.Expired, c.observe(true, "GPS_ATTI", T0 + TakeoffClimb.WAIT_LIMIT_MS + 1))
    }

    @Test
    fun `a null flight mode waits - DJI not saying is not DJI having let go`() {
        // Again with every other conjunct satisfied, so this pins the null-mode branch alone.
        val c = TakeoffClimb()
        c.arm(3.0, T0)
        c.observe(false, "GPS_ATTI", T0)
        c.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, null, T0 + 5_000))
        assertTrue(c.armed)
        // …and it is only the null that is holding it: the same tick with a real mode fires.
        assertEquals(TakeoffClimb.Decision.HandedBack(3.0, origin = ControlOrigin.MAVLINK), c.observe(true, "GPS_ATTI", T0 + 5_100))
    }

    @Test
    fun `a null isFlying is neither flying nor on the ground`() {
        // A null cannot supply the ground observation…
        val c = TakeoffClimb()
        c.arm(3.0, T0)
        c.observe(null, "AUTO_TAKE_OFF", T0 + 500)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(null, "GPS_ATTI", T0 + 1_000))
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(true, "GPS_ATTI", T0 + 2_000))

        // …and it cannot supply the flying observation either. Every other conjunct is satisfied
        // here — ground seen, takeoff mode seen, mode back to normal — so the only thing standing
        // between this and a climb is DJI declining to say whether the aircraft is airborne.
        val d = TakeoffClimb()
        d.arm(3.0, T0)
        d.observe(false, "GPS_ATTI", T0)
        d.observe(false, "AUTO_TAKE_OFF", T0 + 1_000)
        assertEquals(TakeoffClimb.Decision.Waiting, d.observe(null, "GPS_ATTI", T0 + 2_000))
        assertTrue(d.armed)
        assertEquals(TakeoffClimb.Decision.HandedBack(3.0, origin = ControlOrigin.MAVLINK), d.observe(true, "GPS_ATTI", T0 + 2_100))
    }

    @Test
    fun `every takeoff mode holds the climb back`() {
        for (mode in TakeoffClimb.TAKEOFF_MODES) {
            val c = TakeoffClimb()
            c.arm(3.0, T0)
            c.observe(false, "GPS_ATTI", T0)
            assertEquals(mode, TakeoffClimb.Decision.Waiting, c.observe(true, mode, T0 + 5_000))
            assertTrue(mode, c.armed)
        }
    }

    @Test
    fun `the measured takeoff modes are in the set`() {
        // Read from two real records; the other two come from the FCFlightMode surface
        // `telemetry/Px4Mode` enumerates and can only ever make the machine wait longer.
        assertTrue(TakeoffClimb.TAKEOFF_MODES.contains("MOTOR_START"))
        assertTrue(TakeoffClimb.TAKEOFF_MODES.contains("AUTO_TAKE_OFF"))
        assertTrue(TakeoffClimb.TAKEOFF_MODES.contains("ASSISTED_TAKE_OFF"))
        // And the mode both records return to must NOT be, or nothing would ever fire.
        assertFalse(TakeoffClimb.TAKEOFF_MODES.contains("GPS_ATTI"))
    }

    // ------------------------------------------------------------ the two exits

    @Test
    fun `an armed climb expires rather than waiting forever`() {
        val c = TakeoffClimb()
        c.arm(3.0, T0)
        assertEquals(TakeoffClimb.Decision.Waiting, c.observe(false, "GPS_ATTI", T0 + TakeoffClimb.WAIT_LIMIT_MS))
        assertEquals(TakeoffClimb.Decision.Expired, c.observe(false, "GPS_ATTI", T0 + TakeoffClimb.WAIT_LIMIT_MS + 1))
        assertFalse(c.armed)
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(false, "GPS_ATTI", T0 + TakeoffClimb.WAIT_LIMIT_MS + 2))
    }

    @Test
    fun `expiry outranks a late arrival - the window is checked first`() {
        // A takeoff that finally completes 31 s after the command is not the takeoff we armed for;
        // by then an operator has had half a minute to do something else with the aircraft.
        val c = TakeoffClimb()
        c.arm(3.0, T0)
        c.observe(false, "GPS_ATTI", T0)
        assertEquals(
            TakeoffClimb.Decision.Expired,
            c.observe(true, "GPS_ATTI", T0 + TakeoffClimb.WAIT_LIMIT_MS + 1),
        )
    }

    @Test
    fun `the wait limit is bounded and comfortably longer than DJI's measured takeoff`() {
        // 4.4 s from MOTOR_START to the mode trail ending, in both 2026-07-27 records. The margin
        // is for the unmeasured gap between our KeyStartTakeoff and the motors spinning.
        assertTrue(TakeoffClimb.WAIT_LIMIT_MS >= 5 * 4_400L)
        assertTrue(TakeoffClimb.WAIT_LIMIT_MS <= 60_000L)
    }

    @Test
    fun `cancel drops the climb and reports whether there was one`() {
        val c = TakeoffClimb()
        assertFalse("nothing armed", c.cancel())
        c.arm(3.0, T0)
        assertTrue(c.cancel())
        assertFalse(c.armed)
        assertFalse("already cancelled", c.cancel())
        c.observe(false, "GPS_ATTI", T0)
        assertEquals(TakeoffClimb.Decision.Idle, c.observe(true, "GPS_ATTI", T0 + 5_000))
    }
}
