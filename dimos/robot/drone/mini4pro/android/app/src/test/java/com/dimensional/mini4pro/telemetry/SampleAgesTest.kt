package com.dimensional.mini4pro.telemetry

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Per-signal sample ages.
 *
 * The bug this exists to catch is a field that **silently freezes**: DJI stops
 * delivering a key, `StateCache` keeps handing out the last value it saw, and a
 * 5 Hz emitter re-sends it forever. Nothing about the value itself changes when
 * that happens, which is precisely why the tests below are all written against
 * *delivery*, never against value equality.
 *
 * Ages are plain numbers, so nothing here sleeps or reads a clock — the clock is
 * `StateCache.nowMs`, injected, and the arithmetic it feeds is
 * [SampleAges.since].
 */
class SampleAgesTest {

    // ── the arithmetic ────────────────────────────────────────────────────────

    @Test
    fun `age is now minus the delivery instant, per signal`() {
        val ages = SampleAges.since(
            nowMs = 10_000,
            deliveredAtMs = mapOf(
                Signal.POSITION to 9_980,      // 20 ms ago
                Signal.ATTITUDE to 9_500,      // 500 ms ago
                Signal.VELOCITY to 1_000,      // 9 s ago
            ),
        )
        assertEquals(20L, ages[Signal.POSITION])
        assertEquals(500L, ages[Signal.ATTITUDE])
        assertEquals(9_000L, ages[Signal.VELOCITY])
    }

    @Test
    fun `a signal that never arrived has no age at all, which is not an age of zero`() {
        val ages = SampleAges.since(10_000, mapOf(Signal.POSITION to 10_000))
        // 0 would say "delivered this very millisecond" — the opposite claim.
        assertEquals(0L, ages[Signal.POSITION])
        assertNull(ages[Signal.VELOCITY])
        assertNull(SampleAges.NONE[Signal.VELOCITY])
    }

    @Test
    fun `never delivered is never fresh — an absence is not a young reading`() {
        assertFalse(SampleAges.NONE.isFresh(Signal.VELOCITY))
        assertFalse(SampleAges.NONE.isFresh(Signal.VELOCITY, limitMs = 1_000_000))
        assertTrue(SampleAges.NONE.isStale(Signal.VELOCITY))
    }

    @Test
    fun `a clock that runs backwards cannot produce a negative age`() {
        // Both stamps come from the same monotonic clock, so this should not
        // happen; if it ever does, an age below zero would be worse than useless.
        val ages = SampleAges.since(nowMs = 900, deliveredAtMs = mapOf(Signal.POSITION to 1_000))
        assertEquals(0L, ages[Signal.POSITION])
    }

    // ── the freshness policy ──────────────────────────────────────────────────

    @Test
    fun `each signal's own limit decides, and it is at least four measured periods`() {
        // Measured over ~35 s of probe callbacks
        // (docs/measurements/2026-07-25-ground-probe.md): attitude 67 samples,
        // i.e. ~1.9 Hz, ~526 ms per delivery. 2000 ms is ~3.8 periods, so ordinary
        // jitter never trips it.
        assertEquals(2_000L, Signal.ATTITUDE.staleAfterMs)
        val ages = SampleAges.of(Signal.ATTITUDE to 1_999L)
        assertTrue(ages.isFresh(Signal.ATTITUDE))
        assertFalse(SampleAges.of(Signal.ATTITUDE to 2_001L).isFresh(Signal.ATTITUDE))
    }

    @Test
    fun `event-driven signals carry no limit, so a long silence is not a fault`() {
        // KeyFCFlightMode / KeyHomeLocation / KeyIsFailSafe arrive on change and
        // then stay quiet for the whole flight. A limit on those would manufacture
        // a warning out of an aircraft doing nothing unusual.
        assertNull(Signal.FLIGHT_MODE.staleAfterMs)
        assertNull(Signal.HOME.staleAfterMs)
        // Measured 2026-07-26: delivered once at connect and once more when a home
        // was actually recorded. Silence on it means nothing has happened.
        assertNull(Signal.HOME_SET.staleAfterMs)
        assertNull(Signal.FAILSAFE.staleAfterMs)
        // KeyGoHomeHeight is a *setting*, which is the strongest case of all: it changes only
        // when somebody changes it in DJI Fly, and it feeds a MAVLink parameter
        // (`RTL_RETURN_ALT`) that has no freshness channel on the wire to carry a warning even
        // if we raised one. A limit here would fire on every flight and mean nothing.
        assertNull(Signal.GO_HOME_HEIGHT.staleAfterMs)
        assertTrue(
            "a setting heard once, an hour ago, is still the setting",
            SampleAges.of(Signal.GO_HOME_HEIGHT to 3_600_000L).isFresh(Signal.GO_HOME_HEIGHT),
        )
        val hourOld = SampleAges.of(Signal.FLIGHT_MODE to 3_600_000L)
        assertTrue("heard once, an hour ago, is all we can say", hourOld.isFresh(Signal.FLIGHT_MODE))
        assertFalse(hourOld.isStale(Signal.FLIGHT_MODE))
        // ...but "never heard at all" is still not fresh.
        assertTrue(SampleAges.NONE.isStale(Signal.FLIGHT_MODE))
    }

    @Test
    fun `only the continuously-delivered signals are ever reported stale`() {
        // The set is small on purpose. Anything with a limit is something the probe
        // measured arriving repeatedly; everything else is event-driven.
        assertEquals(
            listOf(
                Signal.POSITION, Signal.ALTITUDE, Signal.TAKEOFF_ALTITUDE,
                Signal.ATTITUDE, Signal.VELOCITY,
            ),
            Signal.CONTINUOUS,
        )
        assertTrue(Signal.CONTINUOUS.all { it.staleAfterMs != null })
        assertTrue(Signal.entries.filter { it.staleAfterMs != null } == Signal.CONTINUOUS)
    }

    @Test
    fun `staleSignals names every continuous feed that is not usable as a live reading`() {
        val ages = SampleAges.of(
            Signal.POSITION to 40L,          // fresh
            Signal.ALTITUDE to 40L,          // fresh
            Signal.TAKEOFF_ALTITUDE to 80L,  // fresh
            Signal.ATTITUDE to 5_000L,       // gone quiet
            Signal.VELOCITY to 34_000L,      // gone quiet — the ground-probe case
            Signal.FLIGHT_MODE to 900_000L,  // event-driven — never listed
        )
        assertEquals(listOf(Signal.ATTITUDE, Signal.VELOCITY), ages.staleSignals())

        // Never delivered counts too. For a caller asking "which feeds can I not
        // trust" — which is what M3's controller is asking — "it stopped" and "it
        // never started" are the same answer.
        assertEquals(Signal.CONTINUOUS, SampleAges.NONE.staleSignals())
    }

    // ── the distinction the whole feature turns on ────────────────────────────

    @Test
    fun `a re-delivered identical value is fresh, and a value that stops arriving is stale`() {
        // This is the trap: the value is byte-for-byte the same in both cases. Only
        // the delivery time separates a live feed reporting a motionless aircraft
        // from a feed that died. Any implementation that watched for value changes
        // would score these two identically — and would be wrong.
        val stationary = AircraftState(
            velocityNorth = 0.0, velocityEast = 0.0, velocityDown = 0.0,
            ages = SampleAges.of(Signal.VELOCITY to 100L),
        )
        val frozen = stationary.copy(ages = SampleAges.of(Signal.VELOCITY to 34_000L))

        assertEquals("the values are identical", stationary.velocityNorth, frozen.velocityNorth)
        assertTrue(stationary.isFresh(Signal.VELOCITY))
        assertTrue(frozen.isStale(Signal.VELOCITY))
    }

    @Test
    fun `a fresh null is DJI saying the component is gone, not a missing sample`() {
        // A null newValue is the component-gone signal, and it is a *statement*,
        // delivered at a moment in time. Fresh-and-null and stale-and-null are two
        // different situations and the model must be able to hold both.
        val componentGone = AircraftState(
            velocityNorth = null,
            ages = SampleAges.of(Signal.VELOCITY to 20L),
        )
        assertNull(componentGone.velocityNorth)
        assertTrue("we were just told there is no reading", componentGone.isFresh(Signal.VELOCITY))

        val neverHeard = AircraftState(velocityNorth = null)
        assertNull(neverHeard.ageMs(Signal.VELOCITY))
        assertTrue(neverHeard.isStale(Signal.VELOCITY))
    }

    // ── ergonomics ────────────────────────────────────────────────────────────

    @Test
    fun `a bare AircraftState has heard nothing, which is the truth before registration`() {
        // Bridge and Recorder both use AircraftState() before the MSDK is
        // registered. That must not read as "everything arrived just now".
        val s = AircraftState()
        assertEquals(SampleAges.NONE, s.ages)
        for (signal in Signal.entries) {
            assertNull("$signal", s.ageMs(signal))
            assertTrue("$signal", s.isStale(signal))
        }
    }

    @Test
    fun `reading a value never requires unpacking its age`() {
        // The ergonomics constraint, asserted rather than asserted-in-prose: a
        // per-field wrapper would have made every one of these a tuple unpack, and
        // the project would have quietly stopped using the model.
        val s = AircraftState(
            latitude = 37.9938612,
            velocityNorth = 1.5,
            ages = SampleAges.of(Signal.POSITION to 20L, Signal.VELOCITY to 20L),
        )
        assertEquals(37.9938612, s.latitude!!, 1e-9)
        assertEquals(1.5, s.velocityNorth!!, 1e-9)
        assertEquals(20L, s.ageMs(Signal.VELOCITY))
    }

    @Test
    fun `an explicit limit overrides the default, for a caller with its own policy`() {
        // M3's controller will want a tighter bound than a logger does.
        val ages = SampleAges.of(Signal.VELOCITY to 500L)
        assertTrue(ages.isFresh(Signal.VELOCITY))                  // default 2000 ms
        assertFalse(ages.isFresh(Signal.VELOCITY, limitMs = 200))  // a controller's 200 ms
        assertTrue(ages.isStale(Signal.VELOCITY, limitMs = 200))
    }

    @Test
    fun `every AircraftState field is covered by some signal`() {
        // A field with no signal is a field whose freshness can never be asked
        // about — the gap this whole change exists to close. There is no reflection
        // here on purpose: this list is written by hand so that adding a field to
        // AircraftState and forgetting its Signal fails here.
        val covered = mapOf(
            Signal.FC_CONNECTION to "fcConnected",
            Signal.POSITION to "latitude, longitude",
            Signal.ALTITUDE to "relativeAltitude",
            Signal.TAKEOFF_ALTITUDE to "takeoffAltitudeAmsl",
            Signal.ATTITUDE to "rollDeg, pitchDeg, yawDeg",
            Signal.VELOCITY to "velocityNorth, velocityEast, velocityDown",
            Signal.SATELLITES to "satelliteCount",
            Signal.GPS_LEVEL to "gpsSignalLevel",
            Signal.HOME to "homeLatitude, homeLongitude",
            Signal.HOME_SET to "homeLocationSet",
            Signal.FLYING to "isFlying",
            Signal.MOTORS to "motorsOn",
            Signal.FLIGHT_MODE to "flightMode",
            Signal.MOTOR_START_VETO to "notAllowMotorStart",
            Signal.IMU_WARMUP to "imuWarmingUp",
            Signal.FAILSAFE to "inFailsafe",
            Signal.GO_HOME_HEIGHT to "goHomeHeightM",
            Signal.BATTERY_PERCENT to "batteryPercent",
            Signal.BATTERY_VOLTAGE to "voltageMv",
            Signal.BATTERY_CURRENT to "currentMa",
            Signal.CELL_COUNT to "cellCount",
            Signal.CELL_VOLTAGES to "cellVoltagesMv",
            Signal.BATTERY_TEMP to "batteryTempC",
        )
        assertEquals(Signal.entries.toSet(), covered.keys)
        // 27 declared properties on AircraftState: 26 readings above plus `ages`.
        val fields = AircraftState::class.java.declaredFields
            .filter { !it.isSynthetic }
            .map { it.name }
        val named = covered.values.flatMap { it.split(", ") }
        for (f in named) assertTrue("AircraftState has no field `$f`", fields.contains(f))
        assertEquals(
            "every AircraftState reading needs a Signal — add it to both",
            (named + "ages").toSet(), fields.toSet(),
        )
    }
}
