package com.dimensional.mini4pro.telemetry

import io.dronefleet.mavlink.common.GpsFixType
import io.dronefleet.mavlink.common.MavBatteryChargeState
import io.dronefleet.mavlink.common.MavBatteryFunction
import io.dronefleet.mavlink.common.MavBatteryType
import io.dronefleet.mavlink.common.MavLandedState
import io.dronefleet.mavlink.common.MavVtolState
import io.dronefleet.mavlink.minimal.MavAutopilot
import io.dronefleet.mavlink.minimal.MavState
import io.dronefleet.mavlink.minimal.MavType
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Every expectation here is worked out independently of the encoder — the
 * arithmetic is done by hand in the comments and the literal result asserted —
 * because a test that re-evaluates the implementation's own expression proves
 * only that Kotlin is deterministic.
 *
 * The primary fixture is the real reading set captured on the aircraft on
 * 2026-07-25 (docs/measurements/2026-07-25-ground-probe.md): sitting on the
 * ground, indoors near a window, motors off, never flown.
 *
 * The two home gates added 2026-07-26 ([TelemetryEncoder.homeCoordinate]) were
 * mutation-checked — each breakage made deliberately, failing tests counted, then
 * reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `homeLocationSet == false` gate dropped | 2 |
 *  | gate inverted so only `true` passes (an undelivered key vetoes home) | 18 |
 *  | gate kept, [Geo] coordinate validation replaced by a null check | 4 |
 */
class TelemetryEncoderTest {

    /** Arbitrary but fixed; the encoder must never read a clock itself. */
    private val t = 42_000L

    /**
     * Measured on device 2026-07-25. Velocity read {0,0,0} (stationary),
     * attitude {pitch 0, roll -1, yaw -121.1}, GPS LEVEL_5 with 14 satellites.
     */
    private val measured = AircraftState(
        fcConnected = true,
        latitude = 37.9938612,
        longitude = 23.7253298,
        relativeAltitude = 0.0,
        takeoffAltitudeAmsl = 103.1696,
        rollDeg = -1.0,
        pitchDeg = 0.0,
        yawDeg = -121.1,
        velocityNorth = 0.0,
        velocityEast = 0.0,
        velocityDown = 0.0,
        satelliteCount = 14,
        gpsSignalLevel = 5,
        homeLatitude = 37.9938872,
        homeLongitude = 23.7253295,
        isFlying = false,
        motorsOn = false,
        flightMode = "APAS",
        batteryPercent = 98,
        voltageMv = 8371,
        currentMa = -905,
        cellCount = 2,
        cellVoltagesMv = listOf(4186, 4183),
        batteryTempC = 37.5,
        // Not in the 2026-07-25 probe — these three keys were added later. False
        // here represents an aircraft with no blocker reported, i.e. the "ready
        // on the ground" case; individual blockers are exercised separately.
        notAllowMotorStart = false,
        imuWarmingUp = false,
        inFailsafe = false,
    )

    /** Nothing known at all — no aircraft, no keys ever fired. */
    private val empty = AircraftState()

    // 37.9938612 deg x 10^7, 23.7253298 deg x 10^7
    private val measuredLatE7 = 379_938_612
    private val measuredLonE7 = 237_253_298

    // Home read slightly north of the aircraft: 37.9938872, 23.7253295
    private val homeLatE7 = 379_938_872
    private val homeLonE7 = 237_253_295

    // 103.1696 m x 1000, rounded. relativeAltitude was 0, so AMSL == the datum.
    private val measuredAmslMm = 103_170

    // MavSysStatusSensor wire values, from @MavlinkEntryInfo in the generated
    // source: 3D_GYRO 1, 3D_ACCEL 2, 3D_MAG 4, ABSOLUTE_PRESSURE 8, GPS 32,
    // MOTOR_OUTPUTS 32768, PREARM_CHECK 268435456 (0x10000000).
    private val prearmBit = 268_435_456
    private val imuBaroBits = 1 + 2 + 4 + 8 // 15
    private val expectedSensorMask = imuBaroBits + 32 + prearmBit
    private val motorOutputsBit = 32768

    // MavModeFlag wire values, from @MavlinkEntryInfo in the generated source:
    // CUSTOM_MODE_ENABLED 1, AUTO_ENABLED 4, GUIDED_ENABLED 8,
    // STABILIZE_ENABLED 16, MANUAL_INPUT_ENABLED 64, SAFETY_ARMED 128.
    private val customModeBit = 1
    private val autoBit = 4
    private val guidedBit = 8
    private val stabilizeBit = 16
    private val manualInputBit = 64
    private val armedBit = 128

    /**
     * Pilot flying, motors off: MANUAL_INPUT | STABILIZE | CUSTOM_MODE_ENABLED
     * = 64 + 16 + 1 = 81. Under PX4 the custom-mode bit is unconditional, because
     * `PX4FirmwarePlugin::flightMode()` returns the bare string "Unknown" without
     * it and has no base_mode fallback (`PX4FirmwarePlugin.cc:133-142`).
     */
    private val baseModePilot = manualInputBit + stabilizeBit + customModeBit // 81

    // PX4 custom_mode values, hand-computed from (main << 16) | (sub << 24).
    // Derived independently in Px4ModeTest; repeated here as literals so this
    // file never asks the encoder what it thinks the answer is.
    private val px4Position = 196_608L // main 3, sub 0
    private val px4Offboard = 393_216L // main 6
    private val px4Takeoff = 33_816_576L // main 4, sub 2
    private val px4Rtl = 84_148_224L // main 4, sub 5
    private val px4Land = 100_925_440L // main 4, sub 6

    // ── HEARTBEAT ─────────────────────────────────────────────────────────────

    @Test
    fun heartbeat_measuredGroundState() {
        val hb = TelemetryEncoder.heartbeat(measured)

        assertEquals(MavType.MAV_TYPE_QUADROTOR, hb.type().entry())
        // PX4, not ARDUPILOTMEGA and no longer GENERIC. PX4 is the only identity
        // that gives QGC's Fly-view control buttons without demanding invented
        // calibration parameters: PX4FirmwarePlugin::isCapable reads nothing from
        // the parameter set (PX4FirmwarePlugin.cc:167-181).
        assertEquals(MavAutopilot.MAV_AUTOPILOT_PX4, hb.autopilot().entry())
        assertEquals(12, hb.autopilot().value())
        // APAS is normal GPS position flight, and is what the real aircraft
        // reports parked on the ground.
        assertEquals(baseModePilot, hb.baseMode().value())
        assertEquals(px4Position, hb.customMode())
        assertEquals(MavState.MAV_STATE_STANDBY, hb.systemStatus().entry())
        assertEquals(3, hb.mavlinkVersion())
    }

    /**
     * The custom-mode bit is unconditional under PX4 — including for modes we
     * cannot name. It is a statement about our encoding, not about QGC's lookup
     * table, and a real PX4 always sets it; letting it follow our translation
     * coverage would make the field's validity flicker for reasons no receiver
     * can infer.
     */
    @Test
    fun heartbeat_alwaysClaimsCustomModeEvenWhenTheModeIsUnnameable() {
        for (mode in listOf("APAS", "JOYSTICK", "NAVI_GO", "SOMETHING_DJI_ADDED", "UNKNOWN", null)) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            assertEquals("custom mode flag must be set for $mode", customModeBit, hb.baseMode().value() and customModeBit)
        }
    }

    /**
     * An unmapped or absent DJI mode encodes as custom_mode 0, which QGC renders
     * as "Unknown <base_mode>:0" (`PX4FirmwarePlugin.cc:138`). Main mode 0 is not
     * a member of PX4_CUSTOM_MAIN_MODE (which begins at MANUAL = 1), so it can
     * never be mistaken for a real mode.
     *
     * The thing this test really guards is the negative: an unknown mode must not
     * borrow a plausible neighbour's number.
     */
    @Test
    fun heartbeat_unnameableModesSendZeroNotAPlausibleSubstitute() {
        for (mode in listOf("SOMETHING_DJI_ADDED", "UNKNOWN", "FAULT_TOLERANT", "TAP_FLY", null)) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            assertEquals("custom_mode must be 0 for $mode", 0L, hb.customMode())
            // Specifically not any mode QGC can name.
            for (named in listOf(px4Position, px4Offboard, px4Takeoff, px4Rtl, px4Land)) {
                assertNotEquals("$mode must not encode as $named", named, hb.customMode())
            }
            // base_mode 0 would be read as an uninitialised vehicle
            // (Vehicle.cc:1302); we always assert at least manual input.
            assertNotEquals(0, hb.baseMode().value())
        }
    }

    /** The rows an operator's Takeoff/Land/RTL buttons depend on being reported back. */
    @Test
    fun heartbeat_carriesThePx4ModeNumbersQgcValidatesAgainst() {
        fun custom(mode: String) = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode)).customMode()

        assertEquals(px4Rtl, custom("GO_HOME"))
        assertEquals(px4Land, custom("AUTO_LANDING"))
        assertEquals(px4Takeoff, custom("AUTO_TAKE_OFF"))
        assertEquals(px4Offboard, custom("JOYSTICK"))
        assertEquals(px4Position, custom("APAS"))
    }

    @Test
    fun heartbeat_armedBitFollowsMotorsNotFlying() {
        val motors = TelemetryEncoder.heartbeat(measured.copy(motorsOn = true, isFlying = false))
        // 64 | 16 | 1 | 128
        assertEquals(209, motors.baseMode().value())
        assertEquals(MavState.MAV_STATE_ACTIVE, motors.systemStatus().entry())

        // Airborne but motorsOn unreported must not claim armed off the heartbeat.
        val flyingOnly = TelemetryEncoder.heartbeat(measured.copy(motorsOn = null, isFlying = true))
        assertEquals(baseModePilot, flyingOnly.baseMode().value())
        assertEquals(0, flyingOnly.baseMode().value() and armedBit)
        assertEquals(MavState.MAV_STATE_ACTIVE, flyingOnly.systemStatus().entry())
    }

    @Test
    fun heartbeat_guidedFlagForModesFlyingToATargetWeDesignate() {
        // Virtual stick (OFFBOARD) and hotpoint orbit are the only two: the goal
        // positions come from off the aircraft.
        for (mode in listOf("JOYSTICK", "GPS_HOTPOINT")) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            // 64 | 16 | 1 | 8
            assertEquals(mode, manualInputBit + stabilizeBit + customModeBit + guidedBit, hb.baseMode().value())
            assertEquals(mode, 0, hb.baseMode().value() and autoBit)
        }
    }

    @Test
    fun heartbeat_autoFlagForModesNavigatingTheirOwnPath() {
        for (mode in listOf("GO_HOME", "AUTO_LANDING", "ATTI_LANDING", "AUTO_TAKE_OFF", "NAVI_GO", "FOLLOW_ME")) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            // 64 | 16 | 1 | 4
            assertEquals(mode, manualInputBit + stabilizeBit + customModeBit + autoBit, hb.baseMode().value())
            assertEquals(mode, 0, hb.baseMode().value() and guidedBit)
        }
    }

    @Test
    fun heartbeat_pilotFlownModesClaimNoAutonomy() {
        for (mode in listOf("APAS", "GPS_ATTI", "GPS_SPORT", "TRIPOD_GPS", "ATTI", "MANUAL", "GPS_BRAKE")) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            assertEquals(mode, baseModePilot, hb.baseMode().value())
            assertEquals(mode, 0, hb.baseMode().value() and (guidedBit or autoBit))
        }
    }

    @Test
    fun heartbeat_unknownModeClaimsLeastCapability() {
        // An unrecognised or absent mode must not claim autonomy.
        for (mode in listOf("SOMETHING_DJI_ADDED", "UNKNOWN", "MOTOR_START", null)) {
            val hb = TelemetryEncoder.heartbeat(measured.copy(flightMode = mode))
            assertEquals(baseModePilot, hb.baseMode().value())
            assertEquals(0, hb.baseMode().value() and (guidedBit or autoBit))
        }
    }

    @Test
    fun heartbeat_emptyStateIsUnarmedAndNotReady() {
        val hb = TelemetryEncoder.heartbeat(empty)
        assertEquals(baseModePilot, hb.baseMode().value())
        assertEquals(0, hb.baseMode().value() and armedBit) // never armed on no data
        assertEquals(0L, hb.customMode())
        assertEquals(MavAutopilot.MAV_AUTOPILOT_PX4, hb.autopilot().entry())
        // No flight controller: the aircraft is not powered, so not STANDBY.
        assertEquals(MavState.MAV_STATE_BOOT, hb.systemStatus().entry())
        assertEquals(3, hb.mavlinkVersion())
    }

    // ── SYS_STATUS ────────────────────────────────────────────────────────────

    @Test
    fun sysStatus_measuredGroundState() {
        val ss = TelemetryEncoder.sysStatus(measured)

        assertEquals(expectedSensorMask, ss.onboardControlSensorsPresent().value())
        assertEquals(expectedSensorMask, ss.onboardControlSensorsEnabled().value())
        assertEquals(expectedSensorMask, ss.onboardControlSensorsHealth().value())
        assertEquals(8371, ss.voltageBattery())
        // -905 mA discharging -> +90.5 cA draw -> 91.
        assertEquals(91, ss.currentBattery())
        assertEquals(98, ss.batteryRemaining())
        assertEquals(0, ss.load())
        assertEquals(0, ss.dropRateComm())
        assertEquals(0, ss.errorsComm())
        assertEquals(0, ss.errorsCount1())
        assertEquals(0, ss.errorsCount2())
        assertEquals(0, ss.errorsCount3())
        assertEquals(0, ss.errorsCount4())
    }

    /**
     * QGC (Vehicle.cc:1291) prefers MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS over the
     * heartbeat for armed state on APM firmware. Setting it breaks arm
     * detection, so its absence is load-bearing, not an oversight.
     */
    @Test
    fun sysStatus_bitmaskNeverAdvertisesMotorOutputs() {
        for (s in listOf(measured, empty, measured.copy(motorsOn = true, isFlying = true))) {
            val ss = TelemetryEncoder.sysStatus(s)
            assertEquals(0, ss.onboardControlSensorsPresent().value() and motorOutputsBit)
            assertEquals(0, ss.onboardControlSensorsEnabled().value() and motorOutputsBit)
            assertEquals(0, ss.onboardControlSensorsHealth().value() and motorOutputsBit)
        }
    }

    @Test
    fun sysStatus_gpsHealthClearedWithoutAFix() {
        // Level 0 is a real level meaning "worst". No GPS health, and no prearm
        // either, since readiness needs a 3D fix.
        val noFix = TelemetryEncoder.sysStatus(measured.copy(gpsSignalLevel = 0))
        assertEquals(imuBaroBits, noFix.onboardControlSensorsHealth().value())
        assertEquals(expectedSensorMask, noFix.onboardControlSensorsPresent().value())

        // null means "no information", which is likewise not a fix.
        val unknown = TelemetryEncoder.sysStatus(measured.copy(gpsSignalLevel = null))
        assertEquals(imuBaroBits, unknown.onboardControlSensorsHealth().value())

        // Level 1 is a fix, however poor: the GPS sensor is healthy, but it is
        // not a 3D fix, so it is still not ready to fly.
        val poor = TelemetryEncoder.sysStatus(measured.copy(gpsSignalLevel = 1))
        assertEquals(imuBaroBits + 32, poor.onboardControlSensorsHealth().value())
        assertEquals(0, poor.onboardControlSensorsHealth().value() and prearmBit)

        // Level 3 with enough satellites is a 3D fix and is ready.
        val fix3d = TelemetryEncoder.sysStatus(measured.copy(gpsSignalLevel = 3))
        assertEquals(expectedSensorMask, fix3d.onboardControlSensorsHealth().value())
    }

    @Test
    fun sysStatus_emptyStateUsesSentinelsNotZero() {
        val ss = TelemetryEncoder.sysStatus(empty)
        // "Battery voltage, UINT16_MAX: Voltage not sent by autopilot"
        assertEquals(65535, ss.voltageBattery())
        // "Battery current, -1: Current not sent by autopilot"
        assertEquals(-1, ss.currentBattery())
        // "Battery energy remaining, -1: ... not sent by autopilot"
        assertEquals(-1, ss.batteryRemaining())
        // Nothing is healthy on an aircraft that is not connected.
        assertEquals(0, ss.onboardControlSensorsHealth().value())
        // A zero voltage would look like a dead pack; a zero percentage like an
        // empty one. Neither may appear.
        assertNotEquals(0, ss.voltageBattery())
        assertNotEquals(0, ss.batteryRemaining())
    }

    // ── SYS_STATUS prearm / "Ready" ───────────────────────────────────────────

    /** QGC reads `enabled & PREARM_CHECK` as "this vehicle reports prearm status". */
    @Test
    fun prearm_alwaysAdvertisedAsPresentAndEnabled() {
        for (s in listOf(measured, empty, measured.copy(inFailsafe = true))) {
            val ss = TelemetryEncoder.sysStatus(s)
            assertEquals(prearmBit, ss.onboardControlSensorsPresent().value() and prearmBit)
            assertEquals(prearmBit, ss.onboardControlSensorsEnabled().value() and prearmBit)
        }
        // present matters beyond readiness: QGC only lists sensors by name
        // ("Pre-Arm Check") for bits that are present (SysStatusSensorInfo.cc:26).
        assertEquals(
            prearmBit,
            TelemetryEncoder.sysStatus(empty).onboardControlSensorsPresent().value() and prearmBit,
        )
    }

    @Test
    fun prearm_healthyOnMeasuredGroundStateWithNoBlockers() {
        assertTrue(TelemetryEncoder.prearmHealthy(measured))
        val ss = TelemetryEncoder.sysStatus(measured)
        assertEquals(prearmBit, ss.onboardControlSensorsHealth().value() and prearmBit)
        // QGC's allSensorsHealthy is (enabled & health) == enabled, so a ready
        // vehicle must have every enabled bit healthy too.
        assertEquals(
            ss.onboardControlSensorsEnabled().value(),
            ss.onboardControlSensorsEnabled().value() and ss.onboardControlSensorsHealth().value(),
        )
    }

    @Test
    fun prearm_eachBlockerIndividuallySuppressesReadiness() {
        val blocked = mapOf(
            "DJI vetoes motor start" to measured.copy(notAllowMotorStart = true),
            "IMU still warming up" to measured.copy(imuWarmingUp = true),
            "aircraft in failsafe" to measured.copy(inFailsafe = true),
            "no 3D fix (level 2)" to measured.copy(gpsSignalLevel = 2),
            "3D fix claim with too few satellites" to measured.copy(satelliteCount = 4),
            "flight controller not connected" to measured.copy(fcConnected = false),
        )
        for ((why, s) in blocked) {
            assertFalse("must not be ready: $why", TelemetryEncoder.prearmHealthy(s))
            val ss = TelemetryEncoder.sysStatus(s)
            assertEquals(
                "prearm health bit must be clear: $why",
                0,
                ss.onboardControlSensorsHealth().value() and prearmBit,
            )
            // Under MAV_AUTOPILOT_GENERIC this bit is the only brake QGC has:
            // GenericAutoPilotPlugin's setupComplete is true with no joystick
            // attached, so the fallback readiness test reduces to
            // allSensorsHealthy == (enabled & health) == enabled
            // (Vehicle.cc:1094). An unhealthy prearm must break that too.
            assertNotEquals(
                "allSensorsHealthy must be false too: $why",
                ss.onboardControlSensorsEnabled().value(),
                ss.onboardControlSensorsEnabled().value() and ss.onboardControlSensorsHealth().value(),
            )
        }
    }

    /**
     * An absent reading is a blocker. "We have not heard" must never render as
     * "Ready" — that is the whole reason each condition tests for `false` rather
     * than for "not true".
     */
    @Test
    fun prearm_anyUnknownBlockerMeansNotReady() {
        val unknowns = mapOf(
            "notAllowMotorStart" to measured.copy(notAllowMotorStart = null),
            "imuWarmingUp" to measured.copy(imuWarmingUp = null),
            "inFailsafe" to measured.copy(inFailsafe = null),
            "gpsSignalLevel" to measured.copy(gpsSignalLevel = null),
            "satelliteCount and level" to measured.copy(satelliteCount = null, gpsSignalLevel = null),
        )
        for ((field, s) in unknowns) {
            assertFalse("unknown $field must not read as ready", TelemetryEncoder.prearmHealthy(s))
            assertEquals(
                0,
                TelemetryEncoder.sysStatus(s).onboardControlSensorsHealth().value() and prearmBit,
            )
        }
    }

    /**
     * The most important assertion in this section: an aircraft we know nothing
     * about must not be advertised as ready to fly.
     */
    @Test
    fun prearm_emptyStateNeverAdvertisesReady() {
        assertFalse(TelemetryEncoder.prearmHealthy(empty))
        val ss = TelemetryEncoder.sysStatus(empty)
        assertEquals(0, ss.onboardControlSensorsHealth().value() and prearmBit)
        // Nothing is healthy at all, so QGC's allSensorsHealthy is false too.
        assertEquals(0, ss.onboardControlSensorsHealth().value())
        assertNotEquals(
            ss.onboardControlSensorsEnabled().value(),
            ss.onboardControlSensorsHealth().value(),
        )
    }

    /**
     * `MAV_SYS_STATUS_PREARM_CHECK` is documented "Always healthy when armed":
     * prearm checks no longer apply once the motors are turning, and DJI having
     * started them is itself the evidence. Without this the in-flight
     * `allSensorsHealthy` would go false and raise a meaningless sensor alarm.
     */
    @Test
    fun prearm_healthyWhileArmedEvenWithAStaleBlocker() {
        val flyingWithStaleVeto = measured.copy(
            motorsOn = true,
            isFlying = true,
            notAllowMotorStart = true,
            gpsSignalLevel = 2,
        )
        assertTrue(TelemetryEncoder.prearmHealthy(flyingWithStaleVeto))
        assertEquals(
            prearmBit,
            TelemetryEncoder.sysStatus(flyingWithStaleVeto).onboardControlSensorsHealth().value() and prearmBit,
        )
        // Airborne with motorsOn unreported counts the same way.
        assertTrue(TelemetryEncoder.prearmHealthy(measured.copy(motorsOn = null, isFlying = true, imuWarmingUp = null)))
        // But "armed" cannot conjure an aircraft that is not connected.
        assertFalse(TelemetryEncoder.prearmHealthy(empty.copy(motorsOn = true, isFlying = true)))
    }

    // ── GPS_RAW_INT ───────────────────────────────────────────────────────────

    @Test
    fun gpsRawInt_measuredGroundState() {
        val g = TelemetryEncoder.gpsRawInt(measured, t)

        assertEquals(GpsFixType.GPS_FIX_TYPE_3D_FIX, g.fixType().entry())
        assertEquals(3, g.fixType().value())
        assertEquals(measuredLatE7, g.lat())
        assertEquals(measuredLonE7, g.lon())
        assertEquals(measuredAmslMm, g.alt())
        assertEquals(14, g.satellitesVisible())
        // No DOP source at all: "If unknown, set to: UINT16_MAX".
        assertEquals(65535, g.eph())
        assertEquals(65535, g.epv())
        // Stationary: a real 0 cm/s ground speed, not a sentinel.
        assertEquals(0, g.vel())
        // Course over ground is meaningless at 0 m/s.
        assertEquals(65535, g.cog())
        // 42000 ms since boot -> 42000000 us.
        assertEquals(42_000_000L, g.timeUsec().toLong())
        // Documented as "use 0 if this GPS does not provide yaw".
        assertEquals(0, g.yaw())
    }

    @Test
    fun gpsRawInt_fixTypeMappingBySignalLevel() {
        // null (LEVEL_NONE/UNKNOWN, or never reported) -> NO_GPS, not NO_FIX:
        // we do not know the receiver is reporting at all.
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_NO_GPS,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = null)),
        )
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_NO_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 0)),
        )
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_NO_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 1)),
        )
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_2D_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 2)),
        )
        for (level in 3..5) {
            assertEquals(
                "level $level with 14 satellites is a 3D fix",
                GpsFixType.GPS_FIX_TYPE_3D_FIX,
                TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = level)),
            )
        }
        // We never claim a precision the airframe cannot have.
        assertNotEquals(
            GpsFixType.GPS_FIX_TYPE_RTK_FIXED,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 5)),
        )
    }

    @Test
    fun gpsRawInt_fixDowngradedWhenTooFewSatellites() {
        // A stale level 5 with the satellite count collapsing must not keep
        // asserting a 3D fix; QGC would plot a position from it.
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_2D_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 5, satelliteCount = 3)),
        )
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_2D_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 5, satelliteCount = 5)),
        )
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_3D_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 5, satelliteCount = 6)),
        )
    }

    @Test
    fun gpsRawInt_fixTrustsLevelWhenSatelliteCountUnknown() {
        // Blanking the aircraft on a missing cross-check is the worse failure.
        assertEquals(
            GpsFixType.GPS_FIX_TYPE_3D_FIX,
            TelemetryEncoder.fixType(measured.copy(gpsSignalLevel = 5, satelliteCount = null)),
        )
    }

    @Test
    fun gpsRawInt_emptyStateReportsNoGpsAndNoPosition() {
        val g = TelemetryEncoder.gpsRawInt(empty, 0L)

        // fix_type < 3D_FIX is what makes QGC ignore the position entirely.
        assertEquals(GpsFixType.GPS_FIX_TYPE_NO_GPS, g.fixType().entry())
        assertEquals(0, g.fixType().value())
        assertEquals(0, g.lat())
        assertEquals(0, g.lon())
        // "If unknown, set to UINT8_MAX" - not 0, which would read as "no
        // satellites acquired" rather than "no report".
        assertEquals(255, g.satellitesVisible())
        assertEquals(65535, g.eph())
        assertEquals(65535, g.epv())
        assertEquals(65535, g.vel())
        assertEquals(65535, g.cog())
        // An unknown altitude must not read as sea level.
        assertEquals(Int.MIN_VALUE, g.alt())
        assertNotEquals(0, g.alt())
    }

    // ── GLOBAL_POSITION_INT ───────────────────────────────────────────────────

    @Test
    fun globalPositionIntOrNull_isSuppressedWhenThePositionIsUnknown() {
        // The message has no lat/lon sentinel, so an unknown position could only go
        // out as 0/0 — a real coordinate in the Gulf of Guinea that a GCS plots as a
        // confident fix. mavverify's pos.nonzero caught us emitting exactly that on
        // every frame while the aircraft was powered off (2026-07-25).
        assertNull(TelemetryEncoder.globalPositionIntOrNull(AircraftState(), t))
        // Latitude alone is not a position.
        assertNull(TelemetryEncoder.globalPositionIntOrNull(AircraftState(latitude = 37.99), t))
        assertNull(TelemetryEncoder.globalPositionIntOrNull(AircraftState(longitude = 23.72), t))
    }

    /**
     * An out-of-range *aircraft* position is worse than an out-of-range home: it
     * is the symbol on QGC's map. `KeyAircraftLocation3D` was never seen returning
     * DJI's filler — with no fix it delivered nothing at all, and
     * `GLOBAL_POSITION_INT` was correctly absent for the whole 2026-07-26 11:17
     * session — but it is the same kind of DJI coordinate as the home one that
     * did, so it gets the same gate. `degE7` saturates an out-of-range value to
     * `INT32_MAX`, i.e. 214.7°, which QGC would happily plot.
     */
    @Test
    fun globalPositionIntOrNull_isSuppressedWhenThePositionIsNotACoordinate() {
        val filler = 4.583662361046586E7
        assertNull(TelemetryEncoder.globalPositionIntOrNull(measured.copy(latitude = filler), t))
        assertNull(TelemetryEncoder.globalPositionIntOrNull(measured.copy(longitude = filler), t))
        assertNull(
            TelemetryEncoder.globalPositionIntOrNull(
                measured.copy(latitude = filler, longitude = filler), t,
            ),
        )
        // Out of range by a hair is still out of range.
        assertNull(
            TelemetryEncoder.globalPositionIntOrNull(measured.copy(latitude = 90.0001), t),
        )
        assertNull(
            TelemetryEncoder.globalPositionIntOrNull(measured.copy(longitude = -180.0001), t),
        )
        // GPS_RAW_INT carries a fix type, so it still goes out — with the 0/0
        // "do not plot me" pair rather than a saturated one.
        val raw = TelemetryEncoder.gpsRawInt(measured.copy(latitude = filler), t)
        assertEquals(0, raw.lat())
        assertEquals(0, raw.lon())
    }

    @Test
    fun globalPositionIntOrNull_isEmittedUnchangedWhenThePositionIsKnown() {
        val p = TelemetryEncoder.globalPositionIntOrNull(measured, t)
        assertNotNull(p)
        assertEquals(measuredLatE7, p!!.lat())
        assertEquals(measuredLonE7, p.lon())
        // Suppression must not alter the message we do send.
        assertEquals(TelemetryEncoder.globalPositionInt(measured, t), p)
    }

    @Test
    fun globalPositionInt_measuredGroundState() {
        val p = TelemetryEncoder.globalPositionInt(measured, t)

        assertEquals(42_000L, p.timeBootMs())
        assertEquals(measuredLatE7, p.lat())
        assertEquals(measuredLonE7, p.lon())
        // mm AMSL: (103.1696 + 0) * 1000, rounded.
        assertEquals(103_170, p.alt())
        // On the ground the relative altitude is a genuine 0, not a sentinel.
        assertEquals(0, p.relativeAlt())
        assertEquals(0, p.vx())
        assertEquals(0, p.vy())
        assertEquals(0, p.vz())
        // -121.1 deg -> 238.9 deg -> 23890 centi-degrees.
        assertEquals(23890, p.hdg())
    }

    @Test
    fun globalPositionInt_amslIsTakeoffDatumPlusRelative() {
        // 103.1696 + 12.5 = 115.6696 m -> 115669.6 mm -> 115670
        val s = measured.copy(relativeAltitude = 12.5)
        val p = TelemetryEncoder.globalPositionInt(s, t)
        assertEquals(115_670, p.alt())
        assertEquals(12_500, p.relativeAlt())
        assertEquals(115.6696, TelemetryEncoder.amslMetres(s)!!, 1e-9)

        // The relative altitude alone is never AMSL.
        assertNotEquals(12_500, p.alt())
    }

    @Test
    fun globalPositionInt_missingTakeoffDatumMeansAmslUnknown() {
        val s = measured.copy(takeoffAltitudeAmsl = null, relativeAltitude = 12.5)
        assertNull(TelemetryEncoder.amslMetres(s))

        val p = TelemetryEncoder.globalPositionInt(s, t)
        assertEquals(Int.MIN_VALUE, p.alt())
        // Specifically NOT the relative altitude passed off as AMSL, and not 0.
        assertNotEquals(12_500, p.alt())
        assertNotEquals(0, p.alt())
        // The relative altitude is still good and still reported: it is QGC's
        // default HUD altitude for an APM vehicle.
        assertEquals(12_500, p.relativeAlt())
    }

    @Test
    fun globalPositionInt_missingRelativeMeansAmslUnknown() {
        val s = measured.copy(relativeAltitude = null)
        assertNull(TelemetryEncoder.amslMetres(s))

        val p = TelemetryEncoder.globalPositionInt(s, t)
        assertEquals(Int.MIN_VALUE, p.alt())
        assertEquals(Int.MIN_VALUE, p.relativeAlt())
        // The takeoff datum on its own is not the aircraft's altitude.
        assertNotEquals(measuredAmslMm, p.alt())
    }

    @Test
    fun globalPositionInt_velocitiesAreNedCentimetresPerSecond() {
        val s = measured.copy(velocityNorth = 3.2, velocityEast = -1.5, velocityDown = 0.7)
        val p = TelemetryEncoder.globalPositionInt(s, t)
        // No axis remap: MSDK Velocity3D is already NED, like MAVLink.
        assertEquals(320, p.vx())
        assertEquals(-150, p.vy())
        // vz stays positive-down here (unlike VFR_HUD.climb).
        assertEquals(70, p.vz())
    }

    @Test
    fun globalPositionInt_emptyStateIsNotPlottable() {
        val p = TelemetryEncoder.globalPositionInt(empty, 0L)
        // QGC discards a GLOBAL_POSITION_INT whose lat AND lon are both 0
        // (Vehicle.cc:875). Exactly 0/0 is therefore the safe pair.
        assertEquals(0, p.lat())
        assertEquals(0, p.lon())
        assertEquals(Int.MIN_VALUE, p.alt())
        assertEquals(Int.MIN_VALUE, p.relativeAlt())
        // "Vehicle heading ... If unknown, set to: UINT16_MAX"
        assertEquals(65535, p.hdg())
    }

    @Test
    fun globalPositionInt_halfKnownPositionIsSuppressedEntirely() {
        // A valid latitude next to a zeroed longitude is a coordinate QGC would
        // happily plot, off the coast of Africa. Both must go to 0 together.
        val latOnly = TelemetryEncoder.globalPositionInt(measured.copy(longitude = null), t)
        assertEquals(0, latOnly.lat())
        assertEquals(0, latOnly.lon())

        val lonOnly = TelemetryEncoder.globalPositionInt(measured.copy(latitude = null), t)
        assertEquals(0, lonOnly.lat())
        assertEquals(0, lonOnly.lon())

        // Same rule in GPS_RAW_INT.
        val raw = TelemetryEncoder.gpsRawInt(measured.copy(longitude = null), t)
        assertEquals(0, raw.lat())
        assertEquals(0, raw.lon())
    }

    @Test
    fun globalPositionInt_nonFiniteReadingsAreTreatedAsAbsent() {
        val s = measured.copy(latitude = Double.NaN, relativeAltitude = Double.POSITIVE_INFINITY)
        val p = TelemetryEncoder.globalPositionInt(s, t)
        assertEquals(0, p.lat())
        assertEquals(0, p.lon())
        assertEquals(Int.MIN_VALUE, p.relativeAlt())
    }

    // ── the two altitude/heading unit traps ───────────────────────────────────

    @Test
    fun altitudeIsMillimetresInGlobalPositionButMetresInVfrHud() {
        val p = TelemetryEncoder.globalPositionInt(measured, t)
        val hud = TelemetryEncoder.vfrHud(measured)

        assertEquals(103_170, p.alt())
        assertEquals(103.1696f, hud.alt(), 1e-4f)
        // A factor of 1000 apart, and the HUD field is a float.
        assertEquals(p.alt().toDouble() / 1000.0, hud.alt().toDouble(), 1e-3)
        assertTrue(p.alt().toFloat() != hud.alt())
    }

    @Test
    fun headingIsCentiDegreesInGlobalPositionButDegreesInVfrHud() {
        val p = TelemetryEncoder.globalPositionInt(measured, t)
        val hud = TelemetryEncoder.vfrHud(measured)

        // -121.1 -> 238.9
        assertEquals(23890, p.hdg())
        assertEquals(238, hud.heading())
        assertNotEquals(p.hdg(), hud.heading())
    }

    @Test
    fun headingWrapsSignedYawIntoCompassRange() {
        // yawDeg to (expected hdg cdeg, expected VFR heading deg)
        val cases = listOf(
            Triple(0.0, 0, 0),
            Triple(90.0, 9000, 90),
            Triple(180.0, 18000, 180),
            // -180 is the same bearing as +180.
            Triple(-180.0, 18000, 180),
            Triple(-90.0, 27000, 270),
            // The measured reading.
            Triple(-121.1, 23890, 238),
            // Just west of north: 359.99 deg.
            Triple(-0.01, 35999, 359),
            Triple(-1.0, 35900, 359),
            Triple(179.99, 17999, 179),
        )
        for ((yaw, cdeg, deg) in cases) {
            val p = TelemetryEncoder.globalPositionInt(measured.copy(yawDeg = yaw), t)
            val hud = TelemetryEncoder.vfrHud(measured.copy(yawDeg = yaw))
            assertEquals("hdg for yaw $yaw", cdeg, p.hdg())
            assertEquals("VFR heading for yaw $yaw", deg, hud.heading())
        }
    }

    @Test
    fun headingStaysInRangeRightUpToTheBoundary() {
        // 359.999 deg rounds to 36000 cdeg, which is out of the documented
        // 0..35999 range, so it must wrap to 0 rather than overflow.
        val p = TelemetryEncoder.globalPositionInt(measured.copy(yawDeg = -0.001), t)
        assertEquals(0, p.hdg())
        // The HUD field truncates instead, so it reads 359 - a rounding-width
        // disagreement at the boundary, and both stay in range.
        assertEquals(359, TelemetryEncoder.vfrHud(measured.copy(yawDeg = -0.001)).heading())

        // Sweep the whole circle: never out of range, never 36000, never 360.
        var yaw = -180.0
        while (yaw <= 180.0) {
            val hdg = TelemetryEncoder.globalPositionInt(measured.copy(yawDeg = yaw), t).hdg()
            val deg = TelemetryEncoder.vfrHud(measured.copy(yawDeg = yaw)).heading()
            assertTrue("hdg $hdg out of range for yaw $yaw", hdg in 0..35999)
            assertTrue("heading $deg out of range for yaw $yaw", deg in 0..359)
            yaw += 0.37
        }
    }

    // ── ATTITUDE ──────────────────────────────────────────────────────────────

    @Test
    fun attitude_measuredGroundStateInRadians() {
        val a = TelemetryEncoder.attitude(measured, t)

        assertEquals(42_000L, a.timeBootMs())
        // -1 deg = -1 * pi/180 = -0.0174532925 rad
        assertEquals(-0.0174532925f, a.roll(), 1e-7f)
        assertEquals(0.0f, a.pitch(), 1e-7f)
        // -121.1 deg: 120 deg = 2.0943951 rad, 1.1 deg = 0.0191986 rad,
        // so -(2.0943951 + 0.0191986) = -2.1135937 rad
        assertEquals(-2.1135937f, a.yaw(), 1e-6f)
        // No angular-rate source; zeros rather than differentiated noise.
        assertEquals(0.0f, a.rollspeed(), 0.0f)
        assertEquals(0.0f, a.pitchspeed(), 0.0f)
        assertEquals(0.0f, a.yawspeed(), 0.0f)
    }

    @Test
    fun attitude_yawKeepsItsSignAndIsNotWrapped() {
        // ATTITUDE.yaw is (-pi..+pi) like DJI's [-180,180]: a 1:1 mapping. If
        // this were wrapped the way hdg is, it would come back near +4.17 rad.
        val a = TelemetryEncoder.attitude(measured, t)
        assertTrue("yaw must stay negative, was ${a.yaw()}", a.yaw() < 0f)
        assertEquals(-2.1135937f, a.yaw(), 1e-6f)

        // +180 and -180 stay distinct in sign here, unlike in the heading fields.
        assertEquals(3.14159265f, TelemetryEncoder.attitude(measured.copy(yawDeg = 180.0), t).yaw(), 1e-6f)
        assertEquals(-3.14159265f, TelemetryEncoder.attitude(measured.copy(yawDeg = -180.0), t).yaw(), 1e-6f)
    }

    @Test
    fun attitude_unknownIsNaNNotLevel() {
        val a = TelemetryEncoder.attitude(empty, 0L)
        // A missing attitude encoded as 0 is a confidently level aircraft.
        assertTrue(a.roll().isNaN())
        assertTrue(a.pitch().isNaN())
        assertTrue(a.yaw().isNaN())
        assertFalse(a.roll() == 0.0f)
    }

    // ── VFR_HUD ───────────────────────────────────────────────────────────────

    @Test
    fun vfrHud_measuredGroundState() {
        val hud = TelemetryEncoder.vfrHud(measured)

        assertEquals(0.0f, hud.groundspeed(), 1e-7f)
        // No airspeed sensor: ground speed reported as airspeed, same number.
        assertEquals(hud.groundspeed(), hud.airspeed(), 0.0f)
        assertEquals(238, hud.heading())
        assertEquals(0, hud.throttle())
        assertEquals(103.1696f, hud.alt(), 1e-4f)
        assertEquals(0.0f, hud.climb(), 1e-7f)
    }

    @Test
    fun vfrHud_groundspeedIsHorizontalHypotenuse() {
        // 3-4-5: vertical speed must not leak into ground speed.
        val hud = TelemetryEncoder.vfrHud(
            measured.copy(velocityNorth = 3.0, velocityEast = 4.0, velocityDown = 9.0),
        )
        assertEquals(5.0f, hud.groundspeed(), 1e-6f)
        assertEquals(5.0f, hud.airspeed(), 1e-6f)

        // And a negative component still contributes positively.
        val back = TelemetryEncoder.vfrHud(measured.copy(velocityNorth = -6.0, velocityEast = 8.0))
        assertEquals(10.0f, back.groundspeed(), 1e-6f)
    }

    @Test
    fun vfrHud_climbNegatesVelocityDown() {
        // NED down-positive -> MAVLink climb up-positive.
        assertEquals(
            -1.5f,
            TelemetryEncoder.vfrHud(measured.copy(velocityDown = 1.5)).climb(),
            1e-6f,
        )
        assertEquals(
            2.0f,
            TelemetryEncoder.vfrHud(measured.copy(velocityDown = -2.0)).climb(),
            1e-6f,
        )
        // Descending must never show as a climb.
        assertTrue(TelemetryEncoder.vfrHud(measured.copy(velocityDown = 4.25)).climb() < 0f)
        // GLOBAL_POSITION_INT.vz keeps the DJI sign; only VFR_HUD flips it.
        val s = measured.copy(velocityDown = 1.5)
        assertEquals(150, TelemetryEncoder.globalPositionInt(s, t).vz())
        assertEquals(-1.5f, TelemetryEncoder.vfrHud(s).climb(), 1e-6f)
    }

    @Test
    fun vfrHud_unknownScalarsAreNaN() {
        val hud = TelemetryEncoder.vfrHud(empty)
        // QGC tests these three for NaN explicitly (VehicleFactGroup.cc:215-217).
        assertTrue(hud.airspeed().isNaN())
        assertTrue(hud.groundspeed().isNaN())
        assertTrue(hud.climb().isNaN())
        // An unknown altitude must not read as sea level.
        assertTrue(hud.alt().isNaN())
        assertFalse(hud.alt() == 0.0f)
        assertEquals(65535, hud.heading())
        assertEquals(0, hud.throttle())
    }

    @Test
    fun vfrHud_altitudeUnknownWhenTakeoffDatumMissing() {
        val hud = TelemetryEncoder.vfrHud(measured.copy(takeoffAltitudeAmsl = null))
        assertTrue(hud.alt().isNaN())
        // Never the takeoff-relative altitude relabelled as MSL.
        assertFalse(hud.alt() == 0.0f)
    }

    // ── BATTERY_STATUS ────────────────────────────────────────────────────────

    @Test
    fun batteryStatus_measuredGroundState() {
        val b = TelemetryEncoder.batteryStatus(measured)

        assertEquals(0, b.id())
        assertEquals(MavBatteryFunction.MAV_BATTERY_FUNCTION_ALL, b.batteryFunction().entry())
        assertEquals(MavBatteryType.MAV_BATTERY_TYPE_LIPO, b.type().entry())
        // 37.5 C -> 3750 cdegC
        assertEquals(3750, b.temperature())
        // The measured per-cell readings, verbatim and in order, then UINT16_MAX
        // for cells 3..10. Note they sum to 8369, two millivolts under the 8371
        // the pack key reported - which is exactly the kind of detail an even
        // split would have hidden.
        assertEquals(
            listOf(4186, 4183, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535),
            b.voltages(),
        )
        // -905 mA discharging -> 90.5 cA draw -> 91.
        assertEquals(91, b.currentBattery())
        assertEquals(-1, b.currentConsumed())
        assertEquals(-1, b.energyConsumed())
        assertEquals(98, b.batteryRemaining())
        assertEquals(0, b.timeRemaining())
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK, b.chargeState().entry())
    }

    @Test
    fun batteryStatus_currentIsInvertedAndScaledToCentiamps() {
        // DJI mA, negative while discharging -> MAVLink cA, positive for draw.
        assertEquals(91, TelemetryEncoder.batteryStatus(measured.copy(currentMa = -905)).currentBattery())
        // -12340 mA = 12.34 A draw = 1234 cA
        assertEquals(1234, TelemetryEncoder.batteryStatus(measured.copy(currentMa = -12340)).currentBattery())
        // Scaling is /10, not *10: a 12 A draw must not read as 1234 A.
        assertNotEquals(123400, TelemetryEncoder.batteryStatus(measured.copy(currentMa = -12340)).currentBattery())
        // Charging: DJI positive -> MAVLink negative.
        assertEquals(-150, TelemetryEncoder.batteryStatus(measured.copy(currentMa = 1500)).currentBattery())
        assertTrue(TelemetryEncoder.batteryStatus(measured.copy(currentMa = 250)).currentBattery() < 0)
        // Exactly zero current is a real reading of zero.
        assertEquals(0, TelemetryEncoder.batteryStatus(measured.copy(currentMa = 0)).currentBattery())
        // Unknown is the documented -1, and SYS_STATUS agrees with BATTERY_STATUS.
        val unknown = measured.copy(currentMa = null)
        assertEquals(-1, TelemetryEncoder.batteryStatus(unknown).currentBattery())
        assertEquals(-1, TelemetryEncoder.sysStatus(unknown).currentBattery())
        assertEquals(
            TelemetryEncoder.sysStatus(measured).currentBattery(),
            TelemetryEncoder.batteryStatus(measured).currentBattery(),
        )
    }

    @Test
    fun batteryStatus_realCellArrayIsPaddedWithUint16Max() {
        // A 4S pack's real readings, unequal as real cells are.
        val b = TelemetryEncoder.batteryStatus(
            measured.copy(cellCount = 4, voltageMv = 15207, cellVoltagesMv = listOf(3801, 3799, 3812, 3795)),
        )
        assertEquals(
            listOf(3801, 3799, 3812, 3795, 65535, 65535, 65535, 65535, 65535, 65535),
            b.voltages(),
        )
        assertEquals(10, b.voltages().size)
        // Padding must be UINT16_MAX; a 0 there means "0 V cell" to a GCS.
        for (i in 4..9) assertEquals(65535, b.voltages()[i])
    }

    /**
     * Cell imbalance is a flight-safety signal, and averaging is what erases it.
     * The per-cell readings must arrive at the GCS exactly as measured.
     */
    @Test
    fun batteryStatus_cellImbalanceSurvivesEncoding() {
        // A badly imbalanced 2S pack: 676 mV of spread. Pack total 7696 mV.
        val imbalanced = measured.copy(
            voltageMv = 7696,
            cellCount = 2,
            cellVoltagesMv = listOf(4186, 3510),
        )
        val v = TelemetryEncoder.batteryStatus(imbalanced).voltages()

        assertEquals(4186, v[0])
        assertEquals(3510, v[1])
        assertEquals(65535, v[2])
        // 676 mV of spread still visible after encoding.
        assertEquals(676, v[0] - v[1])
        // An even split of 7696 over 2 cells would be 3848 + 3848: a perfectly
        // balanced pack that does not exist. Neither cell may read that.
        assertNotEquals(3848, v[0])
        assertNotEquals(3848, v[1])
        // And SYS_STATUS still carries the pack voltage unsplit.
        assertEquals(7696, TelemetryEncoder.sysStatus(imbalanced).voltageBattery())
    }

    @Test
    fun batteryStatus_evenSplitOnlyWhenCellVoltagesAreMissing() {
        // Fallback path: no per-cell readings, but pack voltage and cell count
        // are known. 11500 mV over 3 cells: 3834 + 3833 + 3833, remainder on the
        // leading cell so the entries still add up to the pack.
        val b = TelemetryEncoder.batteryStatus(
            measured.copy(cellCount = 3, voltageMv = 11500, cellVoltagesMv = null),
        )
        assertEquals(listOf(3834, 3833, 3833), b.voltages().take(3))
        assertEquals(11500, b.voltages().take(3).sum())
        assertEquals(65535, b.voltages()[3])

        // The measured 2S pack through the same fallback: 4186 + 4185 = 8371,
        // an estimate that is deliberately NOT the real 4186 + 4183.
        val estimated = TelemetryEncoder.batteryStatus(measured.copy(cellVoltagesMv = null))
        assertEquals(listOf(4186, 4185), estimated.voltages().take(2))
        assertEquals(8371, estimated.voltages().take(2).sum())
        // Real readings must win when both are available.
        assertEquals(
            listOf(4186, 4183),
            TelemetryEncoder.batteryStatus(measured).voltages().take(2),
        )

        // An empty list is no data either, so it falls back rather than emitting
        // an all-sentinel array.
        val emptyList = TelemetryEncoder.batteryStatus(measured.copy(cellVoltagesMv = emptyList()))
        assertEquals(listOf(4186, 4185), emptyList.voltages().take(2))
    }

    @Test
    fun batteryStatus_packVoltageGoesInCellZeroWhenCellCountUnknown() {
        // MAVLink's own fallback: "the overall battery voltage should be filled
        // in cell 0, with all others set to UINT16_MAX".
        val b = TelemetryEncoder.batteryStatus(measured.copy(cellCount = null, cellVoltagesMv = null))
        assertEquals(8371, b.voltages()[0])
        for (i in 1..9) assertEquals(65535, b.voltages()[i])

        // A nonsense cell count falls back the same way rather than dividing by it.
        val zero = TelemetryEncoder.batteryStatus(measured.copy(cellCount = 0, cellVoltagesMv = null))
        assertEquals(8371, zero.voltages()[0])
        assertEquals(65535, zero.voltages()[1])
    }

    /**
     * The array is the measurement; `cellCount` is metadata. When they disagree
     * the readings win — padding a short array invents a cell voltage, and
     * truncating a long one discards a measured one.
     */
    @Test
    fun batteryStatus_cellArrayLengthWinsOverCellCount() {
        // Three readings but cellCount says 2: emit all three, no truncation.
        val extra = TelemetryEncoder.batteryStatus(
            measured.copy(cellCount = 2, cellVoltagesMv = listOf(4186, 4183, 4190)),
        )
        assertEquals(listOf(4186, 4183, 4190, 65535), extra.voltages().take(4))

        // One reading but cellCount says 2: the second slot stays UINT16_MAX
        // rather than being padded with a fabricated cell voltage.
        val short = TelemetryEncoder.batteryStatus(
            measured.copy(cellCount = 2, cellVoltagesMv = listOf(4186)),
        )
        assertEquals(4186, short.voltages()[0])
        assertEquals(65535, short.voltages()[1])
        // Specifically not the pack, half the pack, or a repeat of cell 0.
        assertNotEquals(8371, short.voltages()[1])
        assertNotEquals(4185, short.voltages()[1])
        assertNotEquals(4186, short.voltages()[1])

        // More than ten cells could not fit the field; the first ten are kept and
        // nothing overflows into voltages_ext.
        val many = TelemetryEncoder.batteryStatus(
            measured.copy(cellCount = 12, cellVoltagesMv = List(12) { 3600 + it }),
        )
        assertEquals(10, many.voltages().size)
        assertEquals(3600, many.voltages()[0])
        assertEquals(3609, many.voltages()[9])
    }

    @Test
    fun batteryStatus_chargeStateFromPercentage() {
        fun state(percent: Int?) =
            TelemetryEncoder.batteryStatus(measured.copy(batteryPercent = percent)).chargeState().entry()

        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK, state(98))
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK, state(16))
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_LOW, state(15))
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_LOW, state(11))
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_CRITICAL, state(10))
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_CRITICAL, state(0))
        // Unknown must not read as OK.
        assertEquals(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_UNDEFINED, state(null))
    }

    @Test
    fun batteryStatus_emptyStateUsesSentinelsThroughout() {
        val b = TelemetryEncoder.batteryStatus(empty)

        // "INT16_MAX for unknown temperature" - 0 would read as freezing.
        assertEquals(32767, b.temperature())
        assertNotEquals(0, b.temperature())
        // Every cell unknown.
        assertEquals(List(10) { 65535 }, b.voltages())
        assertEquals(-1, b.currentBattery())
        assertEquals(-1, b.currentConsumed())
        assertEquals(-1, b.energyConsumed())
        // A 0 here is a flat battery; -1 is "not estimated".
        assertEquals(-1, b.batteryRemaining())
        assertEquals(0, b.timeRemaining())
        assertEquals(
            MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_UNDEFINED,
            b.chargeState().entry(),
        )
    }

    // ── EXTENDED_SYS_STATE (245) ──────────────────────────────────────────────

    @Test
    fun extendedSysState_measuredGroundState() {
        val e = TelemetryEncoder.extendedSysState(measured)
        // isFlying false, motors off, parked.
        assertEquals(MavLandedState.MAV_LANDED_STATE_ON_GROUND, e.landedState().entry())
        assertEquals(1, e.landedState().value())
        // A quadrotor is never in a VTOL configuration.
        assertEquals(MavVtolState.MAV_VTOL_STATE_UNDEFINED, e.vtolState().entry())
        assertEquals(0, e.vtolState().value())
        assertEquals(245, TelemetryEncoder.MESSAGE_ID_EXTENDED_SYS_STATE)
    }

    @Test
    fun extendedSysState_landedStateFromFlyingAndMotors() {
        fun landed(isFlying: Boolean?, motorsOn: Boolean?, mode: String? = "APAS") =
            TelemetryEncoder.landedState(
                measured.copy(isFlying = isFlying, motorsOn = motorsOn, flightMode = mode),
            )

        assertEquals(MavLandedState.MAV_LANDED_STATE_IN_AIR, landed(true, true))
        // Motors spinning on the ground is a real Mini 4 Pro state and is not
        // flight; the aircraft's own isFlying is what decides.
        assertEquals(MavLandedState.MAV_LANDED_STATE_ON_GROUND, landed(false, true))
        assertEquals(MavLandedState.MAV_LANDED_STATE_ON_GROUND, landed(false, false))
        // isFlying unknown but motors confirmed stopped: a quadrotor with no
        // motors running is not airborne. An inference from a known reading.
        assertEquals(MavLandedState.MAV_LANDED_STATE_ON_GROUND, landed(null, false))
        // Motors running with isFlying unknown says nothing about being airborne.
        assertEquals(MavLandedState.MAV_LANDED_STATE_UNDEFINED, landed(null, true))
    }

    @Test
    fun extendedSysState_takeoffAndLandingOnlyWhileFlying() {
        // Every FCFlightMode that Px4Mode calls a takeoff, and every one it calls
        // a landing — the two lists are the same source, so landed_state and the
        // heartbeat cannot disagree about what a landing is.
        for (mode in listOf("AUTO_TAKE_OFF", "ASSISTED_TAKE_OFF", "TAKEOFF", "QUICKTAKEOFF_ASSIST", "PALM_LAUNCH")) {
            assertEquals(
                mode,
                MavLandedState.MAV_LANDED_STATE_TAKEOFF,
                TelemetryEncoder.landedState(measured.copy(isFlying = true, flightMode = mode)),
            )
        }
        for (mode in listOf("AUTO_LANDING", "ATTI_LANDING", "CONFIRM_LANDING", "BASE_LANDING", "BACKUP_LANDING")) {
            assertEquals(
                mode,
                MavLandedState.MAV_LANDED_STATE_LANDING,
                TelemetryEncoder.landedState(measured.copy(isFlying = true, flightMode = mode)),
            )
        }
        // RTH ends in a landing but is not one: an aircraft crossing a field
        // homeward is IN_AIR, and "Return" is the mode the operator should see.
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_IN_AIR,
            TelemetryEncoder.landedState(measured.copy(isFlying = true, flightMode = "GO_HOME")),
        )
        // "FORCE_LANDING" is a member of DJI's public FlightMode enum and NOT of
        // FCFlightMode, which is what AircraftState.flightMode carries. The old
        // branch that tested for it could never match; it must not read as a
        // landing now either.
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_IN_AIR,
            TelemetryEncoder.landedState(measured.copy(isFlying = true, flightMode = "FORCE_LANDING")),
        )
        // QGC reads LANDING and TAKEOFF as *flying* (Vehicle.cc:1042-1050), so a
        // stale mode string after touchdown must not override "on the ground".
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_ON_GROUND,
            TelemetryEncoder.landedState(measured.copy(isFlying = false, flightMode = "AUTO_LANDING")),
        )
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_ON_GROUND,
            TelemetryEncoder.landedState(measured.copy(isFlying = false, flightMode = "AUTO_TAKE_OFF")),
        )
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_UNDEFINED,
            TelemetryEncoder.landedState(
                empty.copy(flightMode = "AUTO_LANDING"),
            ),
        )
    }

    @Test
    fun extendedSysState_unknownIsUndefinedNotOnGround() {
        val e = TelemetryEncoder.extendedSysState(empty)
        // "Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown", and
        // QGC ignores it rather than acting on it (Vehicle.cc:1051). Claiming
        // ON_GROUND on no data is how a GCS decides an airborne aircraft landed.
        assertEquals(MavLandedState.MAV_LANDED_STATE_UNDEFINED, e.landedState().entry())
        assertEquals(0, e.landedState().value())
        assertNotEquals(
            MavLandedState.MAV_LANDED_STATE_ON_GROUND,
            e.landedState().entry(),
        )
    }

    /**
     * On the ground the aircraft reports APAS, a flying mode — DJI has no
     * idle/standby mode — so QGC's mode string reads "Position" for a parked
     * aircraft. That must not be mistaken for being airborne: landed_state is the
     * on-ground signal, and it is the one to believe.
     */
    @Test
    fun groundStateReportsAFlyingModeButOnGroundLandedState() {
        assertEquals(baseModePilot, TelemetryEncoder.heartbeat(measured).baseMode().value())
        assertEquals(px4Position, TelemetryEncoder.heartbeat(measured).customMode())
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_ON_GROUND,
            TelemetryEncoder.extendedSysState(measured).landedState().entry(),
        )
        assertEquals(MavState.MAV_STATE_STANDBY, TelemetryEncoder.heartbeat(measured).systemStatus().entry())
    }

    // ── HOME_POSITION / GPS_GLOBAL_ORIGIN ─────────────────────────────────────

    @Test
    fun homePosition_measuredGroundState() {
        val h = TelemetryEncoder.homePosition(measured)!!

        assertEquals(homeLatE7, h.latitude())
        assertEquals(homeLonE7, h.longitude())
        // 103.1696 m x 1000 -> 103170 mm. Note this is the takeoff datum alone,
        // NOT the aircraft's AMSL.
        assertEquals(measuredAmslMm, h.altitude())
        assertEquals(0.0f, h.x(), 0.0f)
        assertEquals(0.0f, h.y(), 0.0f)
        assertEquals(0.0f, h.z(), 0.0f)
        // Identity quaternion, not the all-zero one an unset field would give.
        assertEquals(listOf(1.0f, 0.0f, 0.0f, 0.0f), h.q())
        assertEquals(0.0f, h.approachX(), 0.0f)
        assertEquals(0.0f, h.approachY(), 0.0f)
        assertEquals(0.0f, h.approachZ(), 0.0f)
    }

    @Test
    fun homePosition_altitudeIsTheDatumNotTheAircraftAltitude() {
        // Airborne 50 m above the takeoff point: home stays where it is.
        val flying = measured.copy(relativeAltitude = 50.0, isFlying = true, motorsOn = true)
        assertEquals(measuredAmslMm, TelemetryEncoder.homePosition(flying)!!.altitude())
        // 103.1696 + 50 = 153.1696 m -> 153170 mm
        assertEquals(153_170, TelemetryEncoder.globalPositionInt(flying, t).alt())
    }

    @Test
    fun homePosition_suppressedWhenAnyPartIsUnknown() {
        // No placeholder is safe: QGC plots the home symbol from this message and
        // measures altitude-above-home against it.
        assertNull(TelemetryEncoder.homePosition(measured.copy(homeLatitude = null)))
        assertNull(TelemetryEncoder.homePosition(measured.copy(homeLongitude = null)))
        assertNull(TelemetryEncoder.homePosition(measured.copy(takeoffAltitudeAmsl = null)))
        assertNull(TelemetryEncoder.homePosition(empty))
    }

    /**
     * The measured failure of 2026-07-26 11:17, at the encoder.
     *
     * DJI does not report "no home point" as null. With the aircraft powered,
     * linked and never flown, `KeyHomeLocation` returned a populated
     * `LocationCoordinate2D` with **both** fields set to `4.583662361046586E7`,
     * `KeyIsHomeLocationSet` read `false` beside it, and the bridge published 220
     * `HOME_POSITION` / `GPS_GLOBAL_ORIGIN` pairs whose latitude and longitude had
     * saturated to `INT32_MAX`. Neither message may exist at all in that state.
     */
    @Test
    fun homeEvents_suppressedForTheDjiNoHomeFiller() {
        val filler = 4.583662361046586E7
        val noHome = measured.copy(homeLatitude = filler, homeLongitude = filler)
        assertNull(TelemetryEncoder.homePosition(noHome))
        assertNull(TelemetryEncoder.gpsGlobalOrigin(noHome))
        assertTrue(TelemetryEncoder.eventMessages(noHome).isEmpty())
        // Not merely "no HOME_POSITION": nothing that could carry a home goes out.
        assertEquals(0, TelemetryEncoder.eventMessages(noHome).size)
        // One valid half is not a home either.
        assertTrue(TelemetryEncoder.eventMessages(measured.copy(homeLatitude = filler)).isEmpty())
        assertTrue(TelemetryEncoder.eventMessages(measured.copy(homeLongitude = filler)).isEmpty())
    }

    /**
     * DJI's own answer outranks the coordinates. `KeyIsHomeLocationSet` read
     * `false` for the whole 2026-07-26 11:17 session, and `true` the instant a real
     * home was recorded at 09:40 and at 19:16 the previous evening — it is the
     * authority, so a `false` beside a perfectly plausible coordinate still means
     * no home.
     */
    @Test
    fun homeEvents_suppressedWhenDjiSaysNoHomePointIsSet() {
        val notSet = measured.copy(homeLocationSet = false)
        assertNull(TelemetryEncoder.homePosition(notSet))
        assertNull(TelemetryEncoder.gpsGlobalOrigin(notSet))
        assertTrue(TelemetryEncoder.eventMessages(notSet).isEmpty())
        assertNull(TelemetryEncoder.homeCoordinate(notSet))
    }

    /**
     * The asymmetry, and the reason both gates have to stay. `null` is "the key
     * has never been delivered" — the first ~2.8 s of every measured session — and
     * that is not evidence of anything, so the coordinate check has to carry it.
     */
    @Test
    fun homeEvents_anUndeliveredHomeSetKeyLeavesTheCoordinateCheckInCharge() {
        val filler = 4.583662361046586E7
        assertNull(measured.homeLocationSet)
        // Never delivered + a good coordinate: send it. Silence on this key is not
        // a veto, or a bridge would go home-less on a firmware that omits the key.
        assertNotNull(TelemetryEncoder.homePosition(measured))
        // Never delivered + filler: the range check is the only thing standing
        // between DJI's 4.58e7 and a home symbol on the map.
        assertNull(
            TelemetryEncoder.homePosition(
                measured.copy(homeLatitude = filler, homeLongitude = filler),
            ),
        )
        // And `true` is not a licence to skip the coordinate check either: DJI
        // claiming a home does not make an impossible latitude into a place.
        assertNull(
            TelemetryEncoder.homePosition(
                measured.copy(homeLocationSet = true, homeLatitude = filler, homeLongitude = filler),
            ),
        )
        // A genuinely set home with genuine coordinates is the one case that sends.
        assertNotNull(TelemetryEncoder.homePosition(measured.copy(homeLocationSet = true)))
    }

    /**
     * The home gates are about home. An aircraft position is a different key with
     * a different source, and `KeyIsHomeLocationSet` says nothing about it.
     */
    @Test
    fun homeSetFlagDoesNotSuppressTheAircraftPosition() {
        assertNotNull(
            TelemetryEncoder.globalPositionIntOrNull(measured.copy(homeLocationSet = false), t),
        )
    }

    /** The boundaries of the validity rule, at the message level. */
    @Test
    fun homeEvents_boundariesOfTheValidityRule() {
        // Inclusive: the poles and the antimeridian are places.
        assertNotNull(
            TelemetryEncoder.homePosition(measured.copy(homeLatitude = 90.0, homeLongitude = 180.0)),
        )
        assertNotNull(
            TelemetryEncoder.homePosition(
                measured.copy(homeLatitude = -90.0, homeLongitude = -180.0),
            ),
        )
        // One ulp outside is out.
        assertNull(TelemetryEncoder.homePosition(measured.copy(homeLatitude = Math.nextUp(90.0))))
        assertNull(
            TelemetryEncoder.homePosition(measured.copy(homeLongitude = Math.nextDown(-180.0))),
        )
        // An in-range value repeated into both fields has the shape of a filler,
        // not of a fix.
        assertNull(
            TelemetryEncoder.homePosition(measured.copy(homeLatitude = 23.7253295)),
        )
        assertNull(TelemetryEncoder.homePosition(measured.copy(homeLatitude = 0.0, homeLongitude = 0.0)))
    }

    @Test
    fun gpsGlobalOrigin_measuredGroundState() {
        val o = TelemetryEncoder.gpsGlobalOrigin(measured)!!
        assertEquals(homeLatE7, o.latitude())
        assertEquals(homeLonE7, o.longitude())
        assertEquals(measuredAmslMm, o.altitude())
        assertEquals(0L, o.timeUsec().toLong())
    }

    @Test
    fun gpsGlobalOrigin_suppressedWhenHomeUnknown() {
        assertNull(TelemetryEncoder.gpsGlobalOrigin(measured.copy(homeLatitude = null)))
        assertNull(TelemetryEncoder.gpsGlobalOrigin(measured.copy(takeoffAltitudeAmsl = null)))
        assertNull(TelemetryEncoder.gpsGlobalOrigin(empty))
    }

    // ── the whole-tick contract ───────────────────────────────────────────────

    @Test
    fun periodicMessages_containsEveryRateMessageOnce() {
        val msgs = TelemetryEncoder.periodicMessages(measured, t)
        assertEquals(8, msgs.size)
        assertEquals(8, msgs.map { it.javaClass }.toSet().size)
        assertEquals(
            setOf(
                "Heartbeat", "SysStatus", "GpsRawInt", "GlobalPositionInt",
                "Attitude", "VfrHud", "BatteryStatus", "ExtendedSysState",
            ),
            msgs.map { it.javaClass.simpleName }.toSet(),
        )
    }

    @Test
    fun eventMessages_onlyOnceHomeIsKnown() {
        assertEquals(
            setOf("HomePosition", "GpsGlobalOrigin"),
            TelemetryEncoder.eventMessages(measured).map { it.javaClass.simpleName }.toSet(),
        )
        assertTrue(TelemetryEncoder.eventMessages(empty).isEmpty())
    }

    /**
     * The one that matters most: an aircraft we know nothing about must not be
     * described as being anywhere, at any altitude, with any charge.
     */
    @Test
    fun emptyState_neverLooksLikeValidTelemetry() {
        val p = TelemetryEncoder.globalPositionInt(empty, 0L)
        val g = TelemetryEncoder.gpsRawInt(empty, 0L)
        val hud = TelemetryEncoder.vfrHud(empty)
        val b = TelemetryEncoder.batteryStatus(empty)
        val ss = TelemetryEncoder.sysStatus(empty)
        val a = TelemetryEncoder.attitude(empty, 0L)
        val hb = TelemetryEncoder.heartbeat(empty)

        // Position: the only coordinate pair a GCS will refuse to plot is 0/0
        // exactly (QGC Vehicle.cc:875), paired with a sub-3D fix type so
        // GPS_RAW_INT is ignored too (Vehicle.cc:844).
        assertEquals(0, p.lat())
        assertEquals(0, p.lon())
        assertEquals(0, g.lat())
        assertEquals(0, g.lon())
        // GPS_FIX_TYPE_3D_FIX is wire value 3 (@MavlinkEntryInfo(3)).
        assertTrue(
            "fix type must be below 3D_FIX so QGC discards the position",
            g.fixType().value() < 3,
        )
        assertEquals(0, g.fixType().value())

        // Altitude: nothing that reads as "on the ground at sea level".
        assertNotEquals(0, p.alt())
        assertNotEquals(0, p.relativeAlt())
        assertNotEquals(0, g.alt())
        assertEquals(Int.MIN_VALUE, p.alt())
        assertEquals(Int.MIN_VALUE, p.relativeAlt())
        assertEquals(Int.MIN_VALUE, g.alt())
        assertTrue(hud.alt().isNaN())

        // Battery: no voltage, no charge, no current, no temperature.
        assertEquals(65535, ss.voltageBattery())
        assertEquals(-1, ss.currentBattery())
        assertEquals(-1, ss.batteryRemaining())
        assertEquals(-1, b.batteryRemaining())
        assertEquals(-1, b.currentBattery())
        assertEquals(32767, b.temperature())
        assertEquals(List(10) { 65535 }, b.voltages())

        // Attitude and motion: no level attitude, no confident stillness claim
        // beyond the velocity fields, which have no sentinel to use.
        assertTrue(a.roll().isNaN())
        assertTrue(a.pitch().isNaN())
        assertTrue(a.yaw().isNaN())
        assertTrue(hud.groundspeed().isNaN())
        assertTrue(hud.climb().isNaN())
        assertEquals(65535, p.hdg())
        assertEquals(65535, hud.heading())
        assertEquals(65535, g.vel())
        assertEquals(65535, g.cog())
        assertEquals(255, g.satellitesVisible())

        // Never armed, never "ready", never landed, and no home to fly back to.
        assertEquals(0, hb.baseMode().value() and armedBit)
        assertEquals(0, hb.baseMode().value() and (guidedBit or autoBit))
        assertEquals(0, ss.onboardControlSensorsHealth().value() and prearmBit)
        assertEquals(MavState.MAV_STATE_BOOT, hb.systemStatus().entry())
        assertEquals(
            MavLandedState.MAV_LANDED_STATE_UNDEFINED,
            TelemetryEncoder.extendedSysState(empty).landedState().entry(),
        )
        assertNull(TelemetryEncoder.homePosition(empty))
        assertNull(TelemetryEncoder.gpsGlobalOrigin(empty))
    }

    @Test
    fun encoderIsDeterministicAndClockFree() {
        // Same input, same output - nothing inside reads a clock.
        assertEquals(
            TelemetryEncoder.globalPositionInt(measured, t),
            TelemetryEncoder.globalPositionInt(measured, t),
        )
        assertEquals(TelemetryEncoder.batteryStatus(measured), TelemetryEncoder.batteryStatus(measured))
        // And the timestamp is the caller's, verbatim.
        assertEquals(7L, TelemetryEncoder.attitude(measured, 7L).timeBootMs())
        assertEquals(7_000L, TelemetryEncoder.gpsRawInt(measured, 7L).timeUsec().toLong())
    }

    // ── staleness: the wire is a function of values, not of ages ──────────────

    @Test
    fun staleFieldsDoNotChangeASingleByteOnTheWire() {
        // This pins a decision, not an accident:
        // docs/decisions/2026-07-25-per-field-staleness.md. Suppressing or
        // sentinel-ing a *stale* value is not the same act as suppressing an
        // *unknown* one (globalPositionIntOrNull), because a stale reading is a
        // real measurement that may still be correct — and because DJI's velocity
        // key is change-only, so a large age is the normal state of a parked
        // aircraft, not evidence of a fault.
        //
        // If this test fails, someone has started encoding staleness. That may be
        // right — but it changes what a ground station sees, so it must be a
        // deliberate change with this test updated and the decision doc revised,
        // never a side effect.
        val fresh = measured.copy(
            ages = SampleAges.of(
                Signal.POSITION to 20L,
                Signal.ALTITUDE to 20L,
                Signal.TAKEOFF_ALTITUDE to 100L,
                Signal.ATTITUDE to 500L,
                Signal.VELOCITY to 40L,
            ),
        )
        // Every continuous feed dead for 35 s — the ground-probe velocity case,
        // applied to everything.
        val frozen = measured.copy(
            ages = SampleAges.of(
                Signal.POSITION to 35_000L,
                Signal.ALTITUDE to 35_000L,
                Signal.TAKEOFF_ALTITUDE to 35_000L,
                Signal.ATTITUDE to 35_000L,
                Signal.VELOCITY to 35_000L,
            ),
        )
        assertEquals(Signal.CONTINUOUS, frozen.ages.staleSignals())
        assertTrue(fresh.ages.staleSignals().isEmpty())

        assertEquals(
            TelemetryEncoder.periodicMessages(fresh, t),
            TelemetryEncoder.periodicMessages(frozen, t),
        )
        assertEquals(TelemetryEncoder.eventMessages(fresh), TelemetryEncoder.eventMessages(frozen))
        // ...and identical to the ageless state the rest of this file asserts on,
        // so every expectation above still describes what goes out.
        assertEquals(
            TelemetryEncoder.periodicMessages(measured, t),
            TelemetryEncoder.periodicMessages(frozen, t),
        )
        // The suppression that *is* driven by knowledge is untouched: unknown
        // position still means no GLOBAL_POSITION_INT, whatever its age.
        assertNotNull(TelemetryEncoder.globalPositionIntOrNull(frozen, t))
        assertNull(
            TelemetryEncoder.globalPositionIntOrNull(
                empty.copy(ages = SampleAges.of(Signal.POSITION to 20L)), t,
            ),
        )
    }
}
