package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.AircraftState
import io.dronefleet.mavlink.common.Attitude
import io.dronefleet.mavlink.common.BatteryStatus
import io.dronefleet.mavlink.common.GlobalPositionInt
import io.dronefleet.mavlink.common.GpsRawInt
import io.dronefleet.mavlink.common.HomePosition
import io.dronefleet.mavlink.common.MavBatteryChargeState
import io.dronefleet.mavlink.common.MavBatteryFunction
import io.dronefleet.mavlink.common.MavBatteryType
import io.dronefleet.mavlink.common.SysStatus
import io.dronefleet.mavlink.common.VfrHud
import io.dronefleet.mavlink.minimal.Heartbeat
import io.dronefleet.mavlink.minimal.MavAutopilot
import io.dronefleet.mavlink.minimal.MavState
import io.dronefleet.mavlink.minimal.MavType
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sin

/**
 * Measures the recorder's byte rate by running a synthetic flight through the **real
 * writer**, rather than estimating it.
 *
 * The numbers this prints are the ones quoted in `docs/flight-recording.md`. Run it
 * and read them off:
 *
 * ```
 * nix develop --command bash -c 'cd android && ./gradlew :app:testDebugUnitTest \
 *     --tests "*ByteRateTest*" --no-daemon -i' | grep -A20 'FLIGHT LOG BYTE RATE'
 * ```
 *
 * The synthetic session is deliberately the *whole* flight-relevant set at once —
 * MAVLink both directions, state, sticks, RC, mode changes — because a rate measured
 * from telemetry alone would understate the file by a factor of three and be quoted
 * anyway.
 */
class ByteRateTest {

    /** Bridge's declared emitter schedule (docs/verification.md, EXPECTED_RATES). */
    private val mavSchedule = mapOf(
        "HEARTBEAT" to 1.0,
        "SYS_STATUS" to 2.0,
        "GLOBAL_POSITION_INT" to 5.0,
        "GPS_RAW_INT" to 5.0,
        "ATTITUDE" to 10.0,
        "VFR_HUD" to 5.0,
        "BATTERY_STATUS" to 1.0,
        "HOME_POSITION" to 0.2,
    )

    private class CountingSink(override val name: String) : LogSink {
        override var bytesWritten: Long = 0
            private set
        var lines = 0
        override fun writeLine(line: String) {
            lines++
            bytesWritten += line.length + 1
        }
        override fun flush(durable: Boolean) {}
        override fun close() {}
    }

    private class CountingFactory : SinkFactory {
        val sinks = ArrayList<CountingSink>()
        override fun open(session: String, part: Int): LogSink =
            CountingSink("$session.$part").also { sinks.add(it) }
        override fun prune(session: String, keep: Int) = 0
    }

    private class Clock(var nanos: Long = 0, var millis: Long = 1_753_000_000_000L) {
        val mono = MonotonicClock { nanos }
        val wall = WallClock { millis }
    }

    private data class Result(
        val seconds: Double,
        val bytes: Long,
        val lines: Int,
        val byKind: Map<String, Long>,
    ) {
        val bytesPerSecond get() = bytes / seconds
        val mbPerHour get() = bytesPerSecond * 3600 / (1024.0 * 1024.0)
    }

    /**
     * Plays [seconds] of flight through the recorder.
     *
     * @param stateHz `dji_state` and `rc_stick` sampling rate
     * @param controlHz `stick_cmd` rate (0 = M3 absent, which is today's reality)
     * @param mavOut whether Bridge's outbound telemetry is being tapped
     */
    private fun run(
        seconds: Int,
        stateHz: Double,
        controlHz: Double,
        mavOut: Boolean = true,
        mavInHz: Double = 2.0,
    ): Result {
        val factory = CountingFactory()
        val clock = Clock()
        val delta = StateDelta()
        val byKind = HashMap<String, Long>()
        // maxFileBytes high enough that rotation does not skew the measurement.
        val rec = FlightRecorder(
            "bench", factory, clock.mono, clock.wall,
            RecorderConfig(queueCapacity = 1 shl 20, maxFileBytes = Long.MAX_VALUE, statsIntervalMs = 10_000),
            headerJson = """{"note":"byte-rate benchmark"}""",
        )
        rec.start(startThread = false)

        val stepNs = 1_000_000L  // 1 ms resolution
        var nextState = 0.0
        var nextControl = 0.0
        var nextMavIn = 0.0
        val nextMav = HashMap(mavSchedule.mapValues { 0.0 })

        fun emit(e: LogEntry) {
            byKind[e.kind] = (byKind[e.kind] ?: 0L) + rec.render(e).length + 1L
            rec.record(e)
        }

        var t = 0.0
        while (t < seconds) {
            clock.nanos = (t * 1e9).toLong()
            clock.millis = 1_753_000_000_000L + (t * 1000).toLong()

            if (t >= nextState && stateHz > 0) {
                nextState += 1.0 / stateHz
                for (e in delta.entriesFor(clock.nanos, stateAt(t))) emit(e)
                emit(LogEntry.RcStick(clock.nanos, rcAt(t), 0, rcAt(t + 0.3), rcAt(t + 0.6)))
            }
            if (controlHz > 0 && t >= nextControl) {
                nextControl += 1.0 / controlHz
                emit(stickCmdAt(clock.nanos, t))
            }
            if (mavOut) {
                for ((name, hz) in mavSchedule) {
                    if (t >= nextMav[name]!!) {
                        nextMav[name] = nextMav[name]!! + 1.0 / hz
                        emit(MavlinkTap.outbound(clock.nanos, mavMessage(name, t), 1, 1))
                    }
                }
            }
            if (t >= nextMavIn) {
                nextMavIn += 1.0 / mavInHz
                emit(MavlinkTap.outbound(clock.nanos, gcsHeartbeat(), 255, 190))
            }
            rec.drainOnce()
            rec.maintenance()
            t += stepNs / 1e9
        }
        rec.stop()
        return Result(
            seconds.toDouble(),
            factory.sinks.sumOf { it.bytesWritten },
            factory.sinks.sumOf { it.lines },
            byKind,
        )
    }

    // ── synthetic flight: a 3 m/s climb then a 5 m/s eastward run ──

    private fun stateAt(t: Double): AircraftState {
        val climbing = t < 10
        return AircraftState(
            fcConnected = true,
            latitude = 37.9938612 + (if (climbing) 0.0 else (t - 10) * 4.5e-6),
            longitude = 23.7253298 + (if (climbing) 0.0 else (t - 10) * 5.7e-5),
            relativeAltitude = if (climbing) t * 3.0 else 30.0,
            takeoffAltitudeAmsl = 103.1696 + sin(t / 7) * 0.2,
            rollDeg = if (climbing) -1.0 else 8.0 + sin(t) * 0.5,
            pitchDeg = sin(t * 1.3) * 0.7,
            yawDeg = -121.1 + sin(t / 3) * 2,
            velocityNorth = if (climbing) 0.0 else 0.5,
            velocityEast = if (climbing) 0.0 else 5.0,
            velocityDown = if (climbing) -3.0 else 0.0,
            satelliteCount = 17,
            gpsSignalLevel = 5,
            homeLatitude = 37.9938872,
            homeLongitude = 23.7253295,
            isFlying = true,
            motorsOn = true,
            flightMode = if (t < 5) "APAS" else "VIRTUAL_STICK",
            notAllowMotorStart = false,
            imuWarmingUp = false,
            inFailsafe = false,
            batteryPercent = (98 - t / 30).toInt(),
            // Under load the pack sags and the current swings — the two fields whose
            // deadbands decide how much of the file is battery noise.
            voltageMv = (8371 - t * 4 + sin(t * 3) * 60).toInt(),
            currentMa = (-9000 + sin(t * 2) * 2500).toInt(),
            cellCount = 2,
            cellVoltagesMv = listOf((4186 - t * 2).toInt(), (4183 - t * 2).toInt()),
            batteryTempC = 37.5 + t / 60,
        )
    }

    private fun rcAt(t: Double): Int = (sin(t * 0.7) * 40).toInt()

    private fun stickCmdAt(nanos: Long, t: Double) = LogEntry.StickCmd(
        monoNanos = nanos,
        sequence = (t * 10).toLong(),
        setpoint = Setpoint(SetpointFrame.NED_VELOCITY, north = 0.5, east = 5.0, down = 0.0, yawRateDegPerS = 0.0),
        axes = StickAxes(pitch = 0.5, roll = 5.0, yaw = 0.0, verticalThrottle = 0.0),
        modes = StickModes("VELOCITY", "ANGULAR_VELOCITY", "VELOCITY", "GROUND", advanced = true),
        source = CommandSource("SET_POSITION_TARGET_LOCAL_NED", (t * 10).toInt(), nanos - 20_000_000),
        range = StickRange(15.0, 5.0, 100.0),
        path = StickPath.ADVANCED_PARAM,
        accepted = true,
    )

    private fun gcsHeartbeat() = Heartbeat.builder()
        .type(EnumValue.of(MavType.MAV_TYPE_GCS))
        .autopilot(EnumValue.of(MavAutopilot.MAV_AUTOPILOT_INVALID))
        .systemStatus(EnumValue.of(MavState.MAV_STATE_ACTIVE))
        .mavlinkVersion(3)
        .build()

    private fun mavMessage(name: String, t: Double): Any = when (name) {
        "HEARTBEAT" -> Heartbeat.builder()
            .type(EnumValue.of(MavType.MAV_TYPE_QUADROTOR))
            .autopilot(EnumValue.of(MavAutopilot.MAV_AUTOPILOT_ARDUPILOTMEGA))
            .customMode(4L)
            .systemStatus(EnumValue.of(MavState.MAV_STATE_ACTIVE))
            .mavlinkVersion(3).build()
        "SYS_STATUS" -> SysStatus.builder()
            .voltageBattery(8371).currentBattery(900).batteryRemaining(98).build()
        "GLOBAL_POSITION_INT" -> GlobalPositionInt.builder()
            .timeBootMs((t * 1000).toLong()).lat(379938612).lon(237253298)
            .alt(133169).relativeAlt(30000).vx(50).vy(500).vz(0).hdg(23890).build()
        "GPS_RAW_INT" -> GpsRawInt.builder()
            .lat(379938612).lon(237253298).alt(133169).satellitesVisible(17)
            .eph(65535).epv(65535).vel(502).cog(8400).build()
        "ATTITUDE" -> Attitude.builder()
            .timeBootMs((t * 1000).toLong()).roll(0.14f).pitch(0.01f).yaw(-2.11f).build()
        "VFR_HUD" -> VfrHud.builder()
            .airspeed(5.02f).groundspeed(5.02f).heading(239).throttle(0)
            .alt(133.169f).climb(0.0f).build()
        "BATTERY_STATUS" -> BatteryStatus.builder()
            .batteryFunction(EnumValue.of(MavBatteryFunction.MAV_BATTERY_FUNCTION_ALL))
            .type(EnumValue.of(MavBatteryType.MAV_BATTERY_TYPE_LIPO))
            .temperature(3750)
            .voltages(listOf(4186, 4183, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535))
            .currentBattery(900).batteryRemaining(98)
            .chargeState(EnumValue.of(MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK))
            .build()
        "HOME_POSITION" -> HomePosition.builder()
            .latitude(379938872).longitude(237253295).altitude(103169).build()
        else -> throw IllegalArgumentException(name)
    }

    /**
     * Bytes per second spent on the `"hex":"…"` member of `mav_out` entries at the
     * declared schedule. This is the price of the raw-bytes redundancy — the copy of
     * the evidence that survives a bug in our own decoding — and it is worth knowing
     * exactly, because "drop the hex" is the first thing anyone will propose if the
     * file ever needs to be smaller.
     */
    private fun hexBytesPerSecond(): Double {
        var total = 0.0
        for ((name, hz) in mavSchedule) {
            val entry = MavlinkTap.outbound(0, mavMessage(name, 0.0), 1, 1)
            val line = JsonObject.render { o -> entry.writeBody(o) }
            val hex = Regex("\"hex\":\"([0-9a-f]*)\"").find(line)?.groupValues?.get(1) ?: ""
            total += hz * (hex.length + "\"hex\":\"\",".length)
        }
        return total
    }

    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun `measure and report the byte rate at 5 Hz and 25 Hz`() {
        val scenarios = listOf(
            Triple("5 Hz, no M3 (today)", 5.0, 0.0),
            Triple("5 Hz + stick cmds at 5 Hz", 5.0, 5.0),
            Triple("25 Hz + stick cmds at 25 Hz", 25.0, 25.0),
            Triple("25 Hz, no MAVLink tap", 25.0, 25.0),
        )
        val sb = StringBuilder("\n=== FLIGHT LOG BYTE RATE (measured, 60 s synthetic flight) ===\n")
        sb.append("%-32s %10s %10s %10s %12s\n".format("scenario", "lines/s", "B/s", "MB/h", "h in 256 MB"))
        for ((i, s) in scenarios.withIndex()) {
            val (label, stateHz, controlHz) = s
            val r = run(60, stateHz, controlHz, mavOut = i != 3)
            sb.append(
                "%-32s %10.1f %10.0f %10.1f %12.1f\n".format(
                    label, r.lines / r.seconds, r.bytesPerSecond, r.mbPerHour,
                    256.0 / r.mbPerHour,
                )
            )
            if (i == 2) {
                sb.append("  per-kind bytes/s at 25 Hz:\n")
                for ((k, v) in r.byKind.entries.sortedByDescending { it.value }) {
                    sb.append("    %-12s %8.0f B/s  (%4.1f%%)\n".format(k, v / r.seconds, 100.0 * v / r.bytes))
                }
                // What the raw-hex redundancy costs, so "drop the hex" can be
                // costed rather than argued about.
                val hex = hexBytesPerSecond()
                sb.append(
                    "  of which raw MAVLink hex: %.0f B/s (%.1f%% of the whole file)\n"
                        .format(hex, 100.0 * hex / r.bytesPerSecond)
                )
            }
        }
        println(sb)

        // Guard rails, not predictions. If the recorder ever costs an order of
        // magnitude more than this, the doc's numbers are wrong and someone must
        // look rather than trust them.
        val today = run(60, 5.0, 0.0)
        assertTrue(
            "5 Hz session is ${today.bytesPerSecond} B/s — over the 20 kB/s budget",
            today.bytesPerSecond < 20_000,
        )
        val hot = run(60, 25.0, 25.0)
        assertTrue(
            "25 Hz session is ${hot.bytesPerSecond} B/s — over the 60 kB/s budget",
            hot.bytesPerSecond < 60_000,
        )
        // A 32 MB part must hold at least a few minutes at the hottest rate, or
        // rotation would churn.
        assertTrue(
            "a 32 MB part holds only ${32 * 1024 * 1024 / hot.bytesPerSecond} s at 25 Hz",
            32 * 1024 * 1024 / hot.bytesPerSecond > 300,
        )
    }

    @Test
    fun `nothing is dropped at either rate — the queue is sized for the load`() {
        val factory = CountingFactory()
        val clock = Clock()
        val rec = FlightRecorder("q", factory, clock.mono, clock.wall, RecorderConfig())
        rec.start(startThread = false)
        val delta = StateDelta()
        // 40 s of 25 Hz state without draining at all: the default 8192-entry queue
        // must absorb a filesystem stall of that length before it starts dropping.
        repeat(1000) { i ->
            clock.nanos = i * 40_000_000L
            for (e in delta.entriesFor(clock.nanos, stateAt(i / 25.0))) rec.record(e)
        }
        assertTrue("dropped ${rec.dropCount} entries with an idle writer", rec.dropCount == 0L)
        rec.stop()
    }
}
