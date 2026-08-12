package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.telemetry.Geo
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * One `DO_REPOSITION` (192) as it came off the wire, flattened to plain values so
 * [GuidedStickEngine.reposition] can be driven from a unit test without `handshake/` imports —
 * the same reason [GcsStickFrame] exists for `MANUAL_CONTROL`.
 *
 * The measured Go-to shape (`docs/measurements/2026-07-26-qgc-goto-validation.md`, off the wire):
 * `COMMAND_INT 192`, `frame = 0` (`MAV_FRAME_GLOBAL`), `param1 = -1` (default speed),
 * `param2 = 1` (`CHANGE_MODE` — a request, never a claim we echo), **`param4 = NaN`** (keep
 * heading), `x`/`y` 1e7 degrees, `z` AMSL composed by QGC from **our own last published AMSL**.
 * Anything off that shape is refused, never guessed at — the [StickMapping.read] rule.
 *
 * QGC's Pause arrives on the same command id as a `COMMAND_LONG` with NaN lat/lon/alt
 * (`PX4FirmwarePlugin.cc:272-284`, source) — [isCommandInt] false tells the two apart.
 */
data class RepositionCommand(
    val isCommandInt: Boolean,
    /** `MAV_FRAME`; only 0 (`MAV_FRAME_GLOBAL`) has been measured and only it is accepted. */
    val frame: Int,
    /** `COMMAND_INT.x`, 1e7-scaled degrees latitude. */
    val latE7: Int,
    /** `COMMAND_INT.y`, 1e7-scaled degrees longitude. */
    val lonE7: Int,
    /** `COMMAND_INT.z` / `COMMAND_LONG.param7` — AMSL metres in this bridge's own datum. */
    val zAmslM: Float,
    /** `param1` — ground speed; QGC sends −1 ("default"). A positive request is refused. */
    val groundSpeedMs: Float,
    /** `param4` — yaw in radians; QGC's Go-to sends NaN ("keep heading"). Finite is refused. */
    val yawRad: Float,
    /** `COMMAND_LONG.param5`/`param6` — float lat/lon; NaN in the Pause shape. */
    val param5: Float = Float.NaN,
    val param6: Float = Float.NaN,
) {
    companion object {
        /** `MAV_CMD_DO_REPOSITION`. A number for the reason `HandshakeResponder` spells ids as numbers. */
        const val MAV_CMD_DO_REPOSITION = 192

        /** The only frame ever measured off QGC's wire for this command (`MAV_FRAME_GLOBAL`). */
        const val FRAME_GLOBAL = 0
    }
}

/**
 * Stage B's arithmetic: lat/lon offsets to NED metres, and the guidance law. Pure functions,
 * no state, no DJI, no Android — every number that decides where the aircraft flies is in this
 * object and under `RepositionGuidanceTest`.
 *
 * ## The guidance law — deliberately the least code that can be correct
 *
 * ```
 * error      = target − position          (NED metres, from the ~12 Hz position feed)
 * v_desired  = k_p · error                (k_p ≈ 0.5 s⁻¹)
 * v_stopping = sqrt(2 · a_max · |error|)  (the braking envelope)
 * v_cmd      = clamp(v_desired, |v| ≤ min(envelope_max, v_stopping))
 * ```
 *
 * **No integral term** — DJI's own velocity controller sits underneath ours and has its own
 * integrator; stacking two is how a loop oscillates, and an integrator winds up against an
 * obstacle-avoidance brake (#594) and lurches on release. **No derivative term** — the feedback
 * is ~12 Hz and differentiating it amplifies noise into the stick command. Both reasons are the
 * design doc's (`docs/m3-guided-control.md` §1.4) and are binding.
 *
 * The horizontal law runs on the 2-D error **vector** (magnitude clamped, direction kept), so
 * the flight path is a straight line toward the target rather than a curve that converges one
 * axis first. Vertical runs the same law separately under its own Q1 limit.
 */
object RepositionGuidance {

    /** The proportional gain, 1/s — the design doc's k_p ≈ 0.5. */
    const val KP_PER_S = 0.5

    /**
     * The deceleration the stopping envelope guarantees the aircraft never needs to exceed,
     * m/s². 0.5 is deliberately gentle — braking from the full 3 m/s takes 9 m — and it is
     * chosen so the envelope actually *binds*: between ~4 m and ~9 m of error,
     * `sqrt(2·0.5·e)` is below both `k_p·e` and the 3 m/s cap, so the approach rides the
     * braking curve rather than the gain. (At 1.0 m/s² the sqrt term would sit above the
     * other two limits everywhere and the "safety property" would be dead code with a test
     * that can't fail.)
     */
    const val A_MAX_MS2 = 0.5

    /** Arrival conjunct 1: horizontal distance inside this, metres. GPS-grade, ~2× a good fix's error. */
    const val R_ACCEPT_M = 2.0

    /** Arrival's vertical acceptance, metres — see [GuidedStickEngine] for why it exists at all. */
    const val VERTICAL_ACCEPT_M = 1.0

    /** Arrival conjunct 2: measured speed below this, m/s. `k_p·R_ACCEPT = 1 m/s`, so a hover inside 1 m settles under it. */
    const val V_SETTLE_MS = 0.5

    /** Both conjuncts must hold this many *consecutive* 10 Hz ticks — 0.5 s. A fly-through cannot. */
    const val ARRIVE_TICKS = 5

    /**
     * How long the reposition tolerates a stale/absent position fix at zero commanded velocity
     * before releasing entirely. Deliberately many multiples of `Signal.POSITION`'s 1 s
     * staleness limit (design doc §3.2: the staleness limit is right for *detecting* a stall
     * and wrong as a trigger for handing over authority) — but bounded, because our feed being
     * dead says nothing about DJI's own GPS, and DJI's tested failsafes are the better pilot
     * of a blind bridge.
     */
    const val POSITION_LOST_MS = 10_000L

    /**
     * Metres per degree of latitude: `2πR/360` with R = 6 371 000 m (mean earth radius).
     *
     * Against WGS-84 this is wrong by <0.3%, i.e. <30 cm over the old Q1 100 m cap and **~6 m over
     * the 2 km cap of 2026-07-30** — which is worth stating precisely rather than re-describing as
     * "noise", because 6 m is not noise next to [R_ACCEPT_M]'s 2 m. What it is is a *scale* error,
     * not a *position* error, and the distinction is what makes it harmless at the new reach:
     *
     *  - **the target is not displaced.** [nedMetres] is used to compute the error *between* two
     *    coordinates, and it returns zero exactly when they are equal, whatever the scale factor.
     *    So the aircraft still stops where the operator clicked, to the accuracy of the fix; a 2 km
     *    goto is not 6 m short.
     *  - **the commanded speed is 0.3 % off** while it is far out, i.e. the braking curve is read at
     *    a distance 6 m wrong at 2 km and 6 mm wrong at 2 m. Below [R_ACCEPT_M] the error is
     *    millimetres.
     *  - **the refusal thresholds move by 0.3 %**, so a 2 km bound is enforced at ~2006 m of true
     *    distance. It is a refusal threshold; being 6 m generous on it is not a hazard, and it is
     *    the same ruler on both sides of the gate (`MissionItems.METRES_PER_DEG`'s KDoc makes that
     *    argument at length).
     *
     * **An alias for [Geo.METRES_PER_DEG], not a second definition** (unified 2026-07-27). The
     * name is kept because it is cited by tests and by `docs/m3-*`; the number lives in one
     * place so it cannot drift from the inverse conversion that has to share it.
     */
    const val METRES_PER_DEG = Geo.METRES_PER_DEG

    /**
     * The NED offset, metres, from (fromLat, fromLon) to (toLat, toLon), as `(north, east)` —
     * equirectangular small-offset conversion, valid far beyond the Q1 100 m cap.
     *
     * **The arithmetic itself is [Geo.nedMetres]**, and this is a name-preserving delegation:
     * the geodesy was unified into `telemetry/Geo` on 2026-07-27 so that the `cos(latitude)`
     * factor — invisible at the equator, a 21 % east error at this project's home latitude of
     * 38°N — exists in exactly one place ([Geo.longitudeScale]) shared with the mission layer,
     * the Zenoh encoder and the inverse below.
     *
     * The name stays here because it is cited by name in `docs/m3-guided-control.md`,
     * `docs/m4-mission-execution.md` and four test suites, and because "the geodesy the
     * reposition law runs on" is a sentence worth being able to point at.
     * `RepositionGuidanceTest` pins it at a non-equatorial latitude by name.
     */
    fun nedMetres(fromLatDeg: Double, fromLonDeg: Double, toLatDeg: Double, toLonDeg: Double): Pair<Double, Double> =
        Geo.nedMetres(fromLatDeg, fromLonDeg, toLatDeg, toLonDeg)

    /**
     * The inverse of [nedMetres]: the coordinate [northM]/[eastM] metres from (lat, lon).
     *
     * **Also [Geo.offsetCoordinate]** — and it lives next to the forward conversion there for
     * the reason that matters: the two must share the same constant and the same
     * `cos(latitude)` term or the round trip stops closing. Dropped here it places a computed
     * point 21 % short in east at 38°N, which for the orbit's join target is a point that is
     * not on the circle. `OrbitGuidanceTest` pins the round trip by name and `GeoMetresTest`
     * closes it to well inside a centimetre at several latitudes.
     */
    fun offsetCoordinate(latDeg: Double, lonDeg: Double, northM: Double, eastM: Double): Pair<Double, Double> =
        Geo.offsetCoordinate(latDeg, lonDeg, northM, eastM)

    /** Horizontal metres between two coordinates — the Q1 distance gate's ruler (2 km since 2026-07-30). */
    fun horizontalMetres(fromLatDeg: Double, fromLonDeg: Double, toLatDeg: Double, toLonDeg: Double): Double {
        val (n, e) = nedMetres(fromLatDeg, fromLonDeg, toLatDeg, toLonDeg)
        return hypot(n, e)
    }

    /**
     * The law: NED position error → commanded NED velocity. Yaw rate is always zero — QGC's
     * Go-to sends `param4 = NaN` (keep heading, measured), and commanding yaw during a
     * translation swings the camera and couples axes for no benefit.
     *
     * [errorDownM] null means "the vertical error is unknowable right now" (no fresh
     * altitude); the vertical axis commands zero, never a guess. A non-finite error on any
     * axis commands zero on every axis — garbage in, hover out.
     */
    fun velocity(errorNorthM: Double, errorEastM: Double, errorDownM: Double?): StickVelocities {
        if (!errorNorthM.isFinite() || !errorEastM.isFinite()) return StickVelocities.ZERO
        if (errorDownM != null && !errorDownM.isFinite()) return StickVelocities.ZERO

        var north = 0.0
        var east = 0.0
        val horizontal = hypot(errorNorthM, errorEastM)
        if (horizontal > 0.0) {
            val speed = clampedSpeed(horizontal, GuidedEnvelope.HORIZONTAL_MAX_MS)
            north = speed * errorNorthM / horizontal
            east = speed * errorEastM / horizontal
        }

        var down = 0.0
        if (errorDownM != null && errorDownM != 0.0) {
            down = clampedSpeed(abs(errorDownM), GuidedEnvelope.VERTICAL_MAX_MS) * sign(errorDownM)
        }
        return StickVelocities(north, east, down, yawRateDegPerS = 0.0)
    }

    /**
     * `min(k_p·e, envelope, sqrt(2·a_max·e))` — the whole law's magnitude, one place.
     *
     * Public because the orbit's **radial** axis is literally this law in one dimension, under its
     * own smaller cap ([OrbitGuidance.V_RADIAL_MAX_MS]) — same gain, same stopping envelope, same
     * `a_max`. Calling it rather than restating it is what makes that sentence checkable: a change
     * to the braking curve moves both, and there is no second copy to forget.
     */
    fun clampedSpeed(errorM: Double, envelopeMaxMs: Double): Double =
        min(KP_PER_S * errorM, min(envelopeMaxMs, sqrt(2.0 * A_MAX_MS2 * errorM)))

    /**
     * The M3 **arrival test**, both conjuncts, for one tick — distance inside [R_ACCEPT_M], the
     * vertical error inside [VERTICAL_ACCEPT_M] when it is knowable at all, and a *measured*
     * speed under [V_SETTLE_MS]. The caller counts the [ARRIVE_TICKS] consecutive ticks, because
     * that is state and this object has none.
     *
     * [speedMs] null means the velocity feed cannot vouch for a speed; that **withholds** arrival
     * rather than substituting zero, because on this airframe a dead velocity feed and a hover are
     * the same bytes. [errorDownM] null means the vertical error is unknowable, and the vertical
     * conjunct is then not asserted — the graduated treatment the vertical axis gets everywhere.
     *
     * Extracted so the orbit's **join** leg can reuse the arrival test unchanged rather than
     * restate it: `docs/m4-mission-execution.md` §8.2 requires the join to be an ordinary resting
     * leg under the M3 law and the M3 arrival test, and one function is the only way to be sure.
     */
    fun settled(horizontalErrorM: Double, errorDownM: Double?, speedMs: Double?): Boolean =
        horizontalErrorM <= R_ACCEPT_M &&
            (errorDownM == null || abs(errorDownM) <= VERTICAL_ACCEPT_M) &&
            speedMs != null && speedMs <= V_SETTLE_MS
}
