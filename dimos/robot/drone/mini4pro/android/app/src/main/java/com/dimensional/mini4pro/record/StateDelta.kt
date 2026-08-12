package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Signal
import kotlin.math.abs

/**
 * Splits [AircraftState] into the half that is logged at rate and the half that is
 * logged on change.
 *
 * **Split by cadence, not by importance.** Position, attitude and velocity change
 * every sample and are only useful as a time series, so they go into
 * [LogEntry.DjiState] at the recorder's rate. Flight mode, failsafe, home point,
 * connection and battery change rarely, so re-writing them 25 times a second is
 * pure disk cost — and worse, it turns a mode change into something to be inferred
 * from a diff instead of a discrete timestamped line. Those go to
 * [LogEntry.Field] the moment they change.
 *
 * Two of the "slow" fields are not actually constant and need a deadband:
 *
 *  - `takeoffAltitudeAmsl` is a barometric estimate that drifted 0.7 m while the
 *    aircraft sat still (`docs/measurements/2026-07-25-ground-probe.md`). Without a
 *    deadband it would emit continuously.
 *  - `voltageMv` / `currentMa` / `batteryTempC` / `cellVoltagesMv` move constantly
 *    under load.
 *
 * A deadband is a deliberate loss of resolution, so each one is set an order of
 * magnitude finer than anything a diagnosis would turn on, and is stated here
 * rather than buried in the code.
 *
 * A small subset of changes are also worth a human-readable [LogEntry.Event]: the
 * ones that anchor a timeline. Flight mode, in-flight, motors, failsafe and FC
 * connection. Those five cost five extra lines per flight and mean the log reads as
 * a story top-to-bottom without a tool.
 *
 * ## Freezes, which are changes that do not look like changes
 *
 * The on-change logic above has one blind spot by construction: **a value that
 * stops arriving looks exactly like a value that is not changing.** A velocity feed
 * that dies mid-flight writes the same "nothing to report" as an aircraft holding
 * station. That is the bug class this class cannot see from values alone, so it
 * reads [AircraftState.ages] instead and emits a discrete [LogEntry.Event] on each
 * fresh↔stale transition of a continuously-delivered signal (see [Signal]).
 *
 * Transitions rather than a per-sample flag, because a stale signal is stale for
 * many consecutive samples and one line per sample would be the same disk cost the
 * cadence split exists to avoid. And only for signals we have actually heard from:
 * a signal that has never been delivered has a null value, which the log already
 * states, and calling that "stale" would put a warning on every session that starts
 * before the aircraft is powered — i.e. all of them.
 */
class StateDelta(
    private val deadbands: Deadbands = Deadbands(),
) {

    data class Deadbands(
        /** m. Barometric AMSL datum drifts ~0.7 m at rest; 0.25 m still shows a real change. */
        val takeoffAltitudeM: Double = 0.25,
        /** mV pack. Cells are ~4186 mV; 50 mV is well below any meaningful sag. */
        val voltageMv: Int = 50,
        /** mA. Idle draw is ~905 mA; 100 mA resolves a motor spooling up. */
        val currentMa: Int = 100,
        /** °C. */
        val batteryTempC: Double = 0.5,
        /** mV per cell. */
        val cellVoltageMv: Int = 25,
    )

    /** Field names on the wire. Stable — `tools/flightlog` keys on them. */
    object F {
        const val FC_CONNECTED = "fcConnected"
        const val TAKEOFF_ALT_AMSL = "takeoffAltitudeAmsl"
        const val SATELLITES = "satelliteCount"
        const val GPS_LEVEL = "gpsSignalLevel"
        const val HOME_LAT = "homeLatitude"
        const val HOME_LON = "homeLongitude"
        const val IS_FLYING = "isFlying"
        const val MOTORS_ON = "motorsOn"
        const val FLIGHT_MODE = "flightMode"
        const val NOT_ALLOW_MOTOR_START = "notAllowMotorStart"
        const val IMU_WARMING_UP = "imuWarmingUp"
        const val IN_FAILSAFE = "inFailsafe"
        const val BATTERY_PERCENT = "batteryPercent"
        const val VOLTAGE_MV = "voltageMv"
        const val CURRENT_MA = "currentMa"
        const val CELL_COUNT = "cellCount"
        const val CELL_VOLTAGES = "cellVoltagesMv"
        const val BATTERY_TEMP_C = "batteryTempC"
    }

    /** Changes to these also emit an [LogEntry.Event] — the timeline anchors. */
    private val narrative = setOf(
        F.FLIGHT_MODE, F.IS_FLYING, F.MOTORS_ON, F.IN_FAILSAFE, F.FC_CONNECTED,
    )

    private val last = HashMap<String, String?>()
    private val lastNumeric = HashMap<String, Double>()
    private var seenAny = false

    /**
     * Whether each continuous signal was stale at the previous sample. Absent
     * means "assumed fresh", so a signal that is already stale the first time we
     * see it announces itself once — that is a real finding, not a start-up
     * artefact — while a normally-updating one stays silent.
     */
    private val wasStale = HashMap<Signal, Boolean>()

    /**
     * Returns the entries to record for [state]: one [LogEntry.DjiState] plus a
     * [LogEntry.Field] for each slow field that changed, plus an [LogEntry.Event]
     * for each narrative change.
     *
     * Pure and order-stable, so a test can assert on the exact list.
     */
    fun entriesFor(monoNanos: Long, state: AircraftState): List<LogEntry> {
        val out = ArrayList<LogEntry>(4)
        out.add(LogEntry.DjiState.of(monoNanos, state))

        fun emit(name: String, value: String?, numeric: Double? = null, decimals: Int = 3) {
            val prev = last[name]
            if (seenAny && prev == value) return
            last[name] = value
            out.add(LogEntry.Field(monoNanos, name, value, if (seenAny) prev else null, numeric, decimals))
            if (name in narrative && seenAny) {
                out.add(
                    LogEntry.Event(
                        monoNanos = monoNanos,
                        code = if (name == F.FLIGHT_MODE) EventCode.MODE_CHANGE else "${name}_change",
                        severity = severityFor(name, value),
                        message = "$name: ${prev ?: "?"} -> ${value ?: "?"}",
                    )
                )
            }
        }

        /** Emits only when the value moved by more than [band] — see the class doc. */
        fun emitBanded(name: String, value: Double?, band: Double, decimals: Int) {
            if (value == null) {
                emit(name, null)
                return
            }
            val prev = lastNumeric[name]
            if (seenAny && prev != null && abs(prev - value) < band) return
            lastNumeric[name] = value
            emit(name, format(value, decimals), value, decimals)
        }

        emit(F.FC_CONNECTED, state.fcConnected.toString())
        emitBanded(F.TAKEOFF_ALT_AMSL, state.takeoffAltitudeAmsl, deadbands.takeoffAltitudeM, 3)
        emit(F.SATELLITES, state.satelliteCount?.toString(), state.satelliteCount?.toDouble(), 0)
        emit(F.GPS_LEVEL, state.gpsSignalLevel?.toString(), state.gpsSignalLevel?.toDouble(), 0)
        emit(F.HOME_LAT, state.homeLatitude?.let { format(it, 7) }, state.homeLatitude, 7)
        emit(F.HOME_LON, state.homeLongitude?.let { format(it, 7) }, state.homeLongitude, 7)
        emit(F.IS_FLYING, state.isFlying?.toString())
        emit(F.MOTORS_ON, state.motorsOn?.toString())
        emit(F.FLIGHT_MODE, state.flightMode)
        emit(F.NOT_ALLOW_MOTOR_START, state.notAllowMotorStart?.toString())
        emit(F.IMU_WARMING_UP, state.imuWarmingUp?.toString())
        emit(F.IN_FAILSAFE, state.inFailsafe?.toString())
        emit(F.BATTERY_PERCENT, state.batteryPercent?.toString(), state.batteryPercent?.toDouble(), 0)
        emitBanded(F.VOLTAGE_MV, state.voltageMv?.toDouble(), deadbands.voltageMv.toDouble(), 0)
        emitBanded(F.CURRENT_MA, state.currentMa?.toDouble(), deadbands.currentMa.toDouble(), 0)
        emit(F.CELL_COUNT, state.cellCount?.toString(), state.cellCount?.toDouble(), 0)
        emitCells(out, monoNanos, state.cellVoltagesMv)
        emitBanded(F.BATTERY_TEMP_C, state.batteryTempC, deadbands.batteryTempC, 2)

        emitStalenessTransitions(out, monoNanos, state)

        seenAny = true
        return out
    }

    /**
     * One [LogEntry.Event] per fresh↔stale transition — see the class doc for why
     * transitions and not a per-sample flag.
     *
     * The staleness question is asked of the **age**, never of the value: DJI
     * re-delivering an identical number is a live feed, and a value that has not
     * changed in ten seconds may be perfectly current. Deriving this from value
     * changes instead would report a motionless aircraft and a dead sensor
     * identically, which is the exact failure the entry exists to expose.
     */
    private fun emitStalenessTransitions(
        out: MutableList<LogEntry>,
        monoNanos: Long,
        state: AircraftState,
    ) {
        for (signal in Signal.CONTINUOUS) {
            val limit = signal.staleAfterMs ?: continue
            // Never delivered: the value is null and the log says so already.
            val age = state.ageMs(signal) ?: continue
            val stale = age > limit
            if (wasStale.getOrDefault(signal, false) == stale) continue
            wasStale[signal] = stale
            out.add(
                LogEntry.Event(
                    monoNanos = monoNanos,
                    code = if (stale) EventCode.SIGNAL_STALE else EventCode.SIGNAL_FRESH,
                    severity = if (stale) LogEntry.SEV_WARN else LogEntry.SEV_INFO,
                    message = if (stale) {
                        "$signal: no update for ${seconds(age)} s (limit ${seconds(limit)} s) — " +
                            "the value beside this line is cached, not measured now"
                    } else {
                        "$signal: updating again after ${seconds(age)} s"
                    },
                    dataJson = JsonObject.render { w ->
                        w.put("sig", signal.name)
                        w.put("age_ms", age)
                        w.put("limit_ms", limit)
                    },
                )
            )
        }
    }

    private fun seconds(ms: Long): String = format(ms / 1000.0, 3)

    /**
     * Cell voltages are a list, so the deadband applies to the largest per-cell
     * move. Recorded as the raw list — an even split of the pack voltage would hide
     * cell imbalance, which is the one thing the per-cell field exists to show.
     */
    private fun emitCells(out: MutableList<LogEntry>, monoNanos: Long, cells: List<Int>?) {
        val name = F.CELL_VOLTAGES
        val text = cells?.joinToString(",")
        val prev = last[name]
        if (seenAny && prev != null && text != null) {
            val prevCells = prev.split(',').mapNotNull { it.trim().toIntOrNull() }
            if (prevCells.size == cells.size &&
                prevCells.indices.all { abs(prevCells[it] - cells[it]) < deadbands.cellVoltageMv }
            ) return
        }
        if (seenAny && prev == text) return
        last[name] = text
        out.add(LogEntry.Field(monoNanos, name, text, if (seenAny) prev else null))
    }

    private fun severityFor(name: String, value: String?): String = when {
        name == F.IN_FAILSAFE && value == "true" -> LogEntry.SEV_ERROR
        name == F.FC_CONNECTED && value == "false" -> LogEntry.SEV_WARN
        else -> LogEntry.SEV_INFO
    }

    private fun format(v: Double, decimals: Int): String {
        val sb = StringBuilder(24)
        Json.num(sb, v, decimals)
        return sb.toString().removeSurrounding("\"")
    }

    /**
     * Forgets everything, so the next call re-emits every field.
     *
     * Called on reconnect. Without it a reconnected aircraft that happens to be in
     * the same mode as before would leave the log with no record of the mode at all
     * for the second half of the session.
     */
    fun reset() {
        last.clear()
        lastNumeric.clear()
        wasStale.clear()
        seenAny = false
    }
}
