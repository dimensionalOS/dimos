package com.dimensional.mini4pro.telemetry

/**
 * Physical bounds on the non-coordinate readings DJI hands us — the second half of
 * the rule [Geo] started.
 *
 * Plain Kotlin, no DJI and no Android, applied at the `StateCache` seam so that
 * [AircraftState]'s "null means no valid reading" contract is true rather than
 * merely intended.
 *
 * ## Why this exists, and why it is not a check on everything
 *
 * On 2026-07-26 `FlightControllerKey.KeyHomeLocation` was found to answer "I have
 * no home point" with a **populated struct full of filler** rather than with null,
 * and the bridge published 220 fabricated `HOME_POSITION` messages
 * (`docs/measurements/2026-07-26-home-position-sentinel.md`). That left one
 * generalisation to act on: every `?.` unwrap in `StateCache` rests on an
 * assumption now known false for one key.
 *
 * The audit that followed (`docs/measurements/2026-07-26-filler-value-audit.md`)
 * read every numeric field of 17 real sessions — 37 251 state ticks including two
 * flights with a 31-satellite fix — and found **exactly one field carrying a
 * value outside physical range: the home coordinate already guarded by [Geo]**.
 * Everything else stayed inside its bounds. So this file is not a sweep; it is a
 * short list chosen by *consequence*, plus one structural argument:
 *
 * **The key that failed was a struct.** DJI cannot deliver a null *member* of a
 * `LocationCoordinate2D`, so when it has nothing to say it must write something
 * into every field. The other two structs that reach the wire are `Attitude` and
 * `Velocity3D`, and they carry the same risk for the same reason — whereas a boxed
 * `Integer`/`Boolean`/enum key can be, and in the logs repeatedly is, delivered as
 * a plain null.
 *
 * What is guarded, and what each one would do wrong unguarded:
 *
 * | reading | source | consequence of filler |
 * |---|---|---|
 * | attitude | `Attitude` **struct** | QGC draws a wrong artificial horizon and a wrong heading |
 * | velocity | `Velocity3D` **struct** | wrong climb rate and course-over-ground; M3's only feedback |
 * | relative altitude | `Double` | QGC's primary altitude readout, and half of the AMSL sum |
 * | AMSL datum | `Double` | the other half — and takeoff/go-to/orbit altitudes are AMSL |
 * | battery percent | `Integer` | decides whether someone flies; drives `MAV_BATTERY_CHARGE_STATE` |
 * | satellite count | `Integer` | gates `fixType`'s upgrade to `3D_FIX`, which is what makes QGC plot the vehicle |
 *
 * ## What is deliberately **not** guarded
 *
 * - **The nine booleans** (`KeyConnection`, `KeyIsHomeLocationSet`, `KeyIsFlying`,
 *   `KeyAreMotorsOn`, `KeyNotAllowMotorStart`, `KeyIsIMUWarmingUp`,
 *   `KeyIsFailSafe`). A `Boolean?` has three states and all three already mean
 *   something. There is no fourth value to detect, so a guard here is not
 *   unnecessary — it is impossible.
 * - **The three enums** (`KeyProductType`, `KeyGPSSignalLevel`, `KeyFCFlightMode`).
 *   DJI's own unknown members are already handled by name: `gpsLevelToInt` maps
 *   `LEVEL_NONE`/`UNKNOWN` to null, and `KeyProductType` is documented to arrive as
 *   `UNRECOGNIZED` before it settles. An enum cannot be out of range.
 * - **`cellCount`** — `TelemetryEncoder.cellVoltages` already applies `it in 1..10`
 *   at the point of use. A second copy of the same rule in a second file is the
 *   noise this file is trying not to be.
 * - **Pack voltage, current and battery temperature.** Real hazard, but a smaller
 *   one: all three are read-outs with no command or gating path, all three have an
 *   explicit MAVLink unknown sentinel already wired, and the pack voltage is
 *   independently cross-checked against the per-cell array on every frame
 *   (`mavverify`'s `xscale.cells_vs_pack`, worst 7 mV over 189 samples). They are
 *   listed here so the omission is a decision rather than an oversight.
 *
 * ## Why the bounds are where they are
 *
 * Each is a fact about the world or the airframe, not a tuned threshold, and each
 * is orders of magnitude away from anything either the aircraft or the atmosphere
 * can produce. That is the point: a bound a real reading could reach would fire on
 * a good flight and teach an operator to ignore it, and a bound tuned to today's
 * filler constant lasts only until DJI changes the constant.
 */
object Plausible {

    /**
     * Euler angles are reported in degrees on `[-180, 180]`; this is the bound of
     * the representation itself, not a limit on the aircraft. Inclusive — 180° is
     * a real orientation. Widest observed in the logs: roll 65.4°, pitch 63.6°.
     */
    const val MAX_ATTITUDE_DEG = 180.0

    /**
     * Metres per second, per axis. The Mini 4 Pro's rated maximum is 16 m/s
     * horizontal and 6 m/s descending, and a free-falling one reaches roughly
     * 40–50 m/s. 100 m/s is beyond anything the airframe can do intact and still
     * five orders of magnitude below the observed filler.
     */
    const val MAX_SPEED_MS = 100.0

    /**
     * Metres, magnitude, for both the takeoff-relative altitude and the AMSL
     * datum. The lowest land surface on Earth is about −430 m and this airframe's
     * service ceiling is 4 km; ±10 km spans the whole troposphere in both
     * directions and is unreachable by either reading.
     */
    const val MAX_ALTITUDE_M = 10_000.0

    /** A percentage. Inclusive at both ends — 0 % and 100 % are both real. */
    const val MAX_BATTERY_PERCENT = 100

    /**
     * No consumer GNSS receiver tracks this many satellites at once across every
     * constellation it can see. Highest observed in the logs: 31, in flight, with
     * a level-5 fix.
     */
    const val MAX_SATELLITES = 64

    /** Roll, pitch and yaw in degrees, accepted as a set. */
    data class AttitudeDeg(val roll: Double, val pitch: Double, val yaw: Double)

    /** North/east/down velocity in metres per second, accepted as a set. */
    data class VelocityNed(val north: Double, val east: Double, val down: Double)

    /**
     * The three Euler angles when all three are present, finite and inside
     * [MAX_ATTITUDE_DEG], or null.
     *
     * **All-or-nothing, and that is the deliberate part.** DJI delivers roll,
     * pitch and yaw as one `Attitude` struct in one callback, so an impossible
     * value in any member is evidence about *the struct*, not about that member —
     * exactly as it was for the home coordinate. Keeping the survivors would mean
     * drawing a horizon from two thirds of a reading we have just decided not to
     * believe. Yaw goes with them even though it feeds a different display: a
     * confident wrong heading is no safer than a confident wrong horizon.
     */
    fun attitudeOrNull(rollDeg: Double?, pitchDeg: Double?, yawDeg: Double?): AttitudeDeg? {
        val roll = angleOrNull(rollDeg) ?: return null
        val pitch = angleOrNull(pitchDeg) ?: return null
        val yaw = angleOrNull(yawDeg) ?: return null
        return AttitudeDeg(roll, pitch, yaw)
    }

    /**
     * The three velocity components when all three are present, finite and inside
     * [MAX_SPEED_MS], or null.
     *
     * All-or-nothing for the same struct argument as [attitudeOrNull], and here it
     * is forced anyway: ground speed and course over ground are computed from
     * north *and* east together, so half a velocity is not a usable velocity.
     */
    fun velocityOrNull(north: Double?, east: Double?, down: Double?): VelocityNed? {
        val n = speedOrNull(north) ?: return null
        val e = speedOrNull(east) ?: return null
        val d = speedOrNull(down) ?: return null
        return VelocityNed(n, e, d)
    }

    /**
     * An altitude in metres when it is finite and inside [MAX_ALTITUDE_M], or null.
     *
     * Used for both halves of the AMSL sum — `KeyAltitude` (takeoff-relative) and
     * `KeyTakeoffLocationAltitude` (the datum). They share one bound because they
     * share one failure: `TelemetryEncoder.amslMetres` adds them, so filler in
     * either produces the same wrong absolute altitude, and that altitude is the
     * frame takeoff, go-to and orbit commands are expressed in.
     */
    fun altitudeMOrNull(metres: Double?): Double? {
        val m = metres ?: return null
        if (!m.isFinite()) return null
        if (m < -MAX_ALTITUDE_M || m > MAX_ALTITUDE_M) return null
        return m
    }

    /**
     * A battery percentage on `[0, 100]`, or null.
     *
     * The highest-consequence integer in the set: it drives QGC's battery
     * indicator, `MAV_BATTERY_CHARGE_STATE` and, through both, an operator's
     * decision to launch. Out of range it is not a percentage, and `-1` — MAVLink's
     * "the autopilot does not estimate this" — is the honest thing to send instead.
     */
    fun batteryPercentOrNull(percent: Int?): Int? {
        val p = percent ?: return null
        if (p < 0 || p > MAX_BATTERY_PERCENT) return null
        return p
    }

    /**
     * A satellite count on `[0, MAX_SATELLITES]`, or null.
     *
     * Guarded despite being a display number because it is not only a display
     * number: `TelemetryEncoder.fixType` refuses to call a DJI signal level of 3+
     * a `3D_FIX` while fewer than `MIN_SATS_FOR_3D` satellites are visible, and
     * **an unknown count skips that check deliberately** rather than downgrading.
     * So a filler count is the one value that can talk the bridge *into* claiming a
     * 3D fix, which is what makes QGC plot the vehicle at all.
     */
    fun satelliteCountOrNull(count: Int?): Int? {
        val n = count ?: return null
        if (n < 0 || n > MAX_SATELLITES) return null
        return n
    }

    private fun angleOrNull(deg: Double?): Double? {
        val d = deg ?: return null
        if (!d.isFinite()) return null
        if (d < -MAX_ATTITUDE_DEG || d > MAX_ATTITUDE_DEG) return null
        return d
    }

    private fun speedOrNull(ms: Double?): Double? {
        val v = ms ?: return null
        if (!v.isFinite()) return null
        if (v < -MAX_SPEED_MS || v > MAX_SPEED_MS) return null
        return v
    }
}
