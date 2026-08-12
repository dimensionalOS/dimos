package com.dimensional.mini4pro.guided

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * One `MAV_CMD_DO_ORBIT` (34) as it came off the wire, flattened to plain values so
 * [GuidedStickEngine.orbit] can be driven from a unit test without `handshake/` imports — the same
 * reason [RepositionCommand] exists for `DO_REPOSITION` and [GcsStickFrame] for `MANUAL_CONTROL`.
 *
 * **Measured shape** (`docs/measurements/2026-07-27-orbit-wire-and-takeoff.md`, off the wire at
 * 00:34 on the bench, one command in the whole record):
 *
 * ```
 * COMMAND_INT 34   frame = 0 (MAV_FRAME_GLOBAL)
 * param1 = 30.766829   radius, metres, positive (= clockwise)
 * param2 = NaN         velocity — "vehicle default"
 * param3 = 5           ORBIT_YAW_BEHAVIOUR_UNCHANGED
 * param4 = NaN         turns — "vehicle default"
 * x, y   = 379938654, 237253314   centre, 1e7 degrees
 * z      = 102.064003             AMSL
 * ```
 *
 * Two measured facts shape everything below.
 *
 * **QGC sends this exactly once.** It does *not* stream while the operator drags the circle — the
 * drag is a local UI gesture and the wire sees only the confirmed result. So an arriving command is
 * always a deliberate press, and retargeting a live orbit is a second press rather than a stream.
 *
 * **The altitude is safe here, and for the opposite reason to a mission item's.** QGC computes `z`
 * as `homePosition.altitude + sliderMetres` (`GuidedActionsController.qml:645`) from *our own*
 * `HOME_POSITION`, so our datum enters its sum with one sign and leaves ours with the other and
 * cancels exactly — the Stage B argument verbatim. The button does not even appear without a valid
 * home altitude, so "no datum" cannot arise from this UI; it is still refused for anything else.
 */
data class OrbitCommand(
    val isCommandInt: Boolean,
    /** `MAV_FRAME`; only 0 (`MAV_FRAME_GLOBAL`) has been measured and only it is accepted. */
    val frame: Int,
    /**
     * `param1` — radius in metres, **signed**: QGC encodes direction in the sign,
     * `orbitMapCircle.radius() * (clockwiseRotation ? 1 : -1)`
     * (`GuidedActionsController.qml:641`). Positive is clockwise. The magnitude is bounded by
     * [OrbitGuidance.R_MIN_M]..[OrbitGuidance.R_MAX_M] and **refused, never clamped** — a clamped
     * circle is a different circle, drawn somewhere the operator did not click.
     *
     * The counter-clockwise case is **still unmeasured on the wire**: the bench session only ever
     * pressed the clockwise control. The sign handling here is source-derived, and it is on the
     * bench list.
     */
    val radiusM: Float,
    /** `param2` — tangential velocity, m/s. QGC sends NaN ("vehicle default"). A request is refused. */
    val velocityMs: Float,
    /** `param3` — `ORBIT_YAW_BEHAVIOUR`. QGC sends 5; see [OrbitCommand.YAW_BEHAVIOUR_UNCHANGED]. */
    val yawBehaviour: Float,
    /** `param4` — number of turns. QGC sends NaN ("vehicle default" → [OrbitGuidance.DEFAULT_TURNS]). */
    val turns: Float,
    /** `COMMAND_INT.x`, 1e7-scaled degrees latitude of the circle's centre. */
    val latE7: Int,
    /** `COMMAND_INT.y`, 1e7-scaled degrees longitude of the circle's centre. */
    val lonE7: Int,
    /** `COMMAND_INT.z` — AMSL metres, composed by QGC from our own published datum. */
    val zAmslM: Float,
) {
    companion object {
        /** `MAV_CMD_DO_ORBIT`. A number, for the reason `HandshakeResponder` spells its ids as numbers. */
        const val MAV_CMD_DO_ORBIT = 34

        /** The only frame ever measured off QGC's wire for this command (`MAV_FRAME_GLOBAL`). */
        const val FRAME_GLOBAL = 0

        /**
         * `ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER` — nose to the centre. The one value we
         * actually fly, and the only one for which no substitution has to be announced.
         */
        const val YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0

        /**
         * `ORBIT_YAW_BEHAVIOUR_UNCHANGED` — **a value our MAVLink dialect does not have.**
         *
         * `ORBIT_YAW_BEHAVIOUR` in `ref/mavlink/definition-xml/common.xml:811-827` stops at
         * `4 = RC_CONTROLLED`; `UNCHANGED = 5` exists only in QGC's newer definitions
         * (`_deps/mavlink-build/.../common.h:515`). This is PLAN.md's "anything added to MAVLink
         * after 2023 is absent from our dialect" showing up as an *enum value* rather than as a
         * message, and it is **measured**, not reasoned — QGC sent exactly this on the bench.
         *
         * So it is a named integer constant here, deliberately, and **never an enum lookup**: a
         * lookup would either fail to compile against our jar or — worse — resolve to whatever
         * meaning the number acquires in a future dialect.
         */
        const val YAW_BEHAVIOUR_UNCHANGED = 5
    }
}

/**
 * M3 Stage C's arithmetic: the circle, the two-axis decomposition, the curvature cap, the swept
 * angle, the yaw law and the gimbal solution. Pure functions, no state, no DJI, no Android — every
 * number that decides where the aircraft flies while circling is in this object and under
 * `OrbitGuidanceTest`.
 *
 * Design authority: `docs/m4-mission-execution.md` §8 (join-then-circle, the radial/tangential
 * decomposition, the curvature cap, termination) and §8.5 plus
 * `docs/decisions/2026-07-27-orbit-yaw.md` (yaw, and the gimbal holding the centre at takeoff
 * height).
 *
 * ## The decomposition
 *
 * In the same NED metres the reposition law uses, with the same `cos(latitude)` factor on east:
 *
 * ```
 * r    = |p − C|                  radial distance from the centre
 * e_r  = r − R                    radial error (positive = outside the circle)
 * û_r  = (p − C)/r                unit radial, outward
 * û_t  = s · rotCW(û_r)           unit tangential, s = sign(param1), rotCW(n,e) = (−e, n)
 *
 * v_radial     = −clampedSpeed(|e_r|, V_RADIAL_MAX) · sign(e_r)       // the M3 law, one dimension
 * v_tangential = min(v_t_target, previous + a_max · dt)               // ramped, never stepped
 * v_cmd        = v_radial · û_r + v_tangential · û_t
 * ```
 *
 * The radial axis is [RepositionGuidance.clampedSpeed] itself — same gain, same
 * `sqrt(2·a_max·e)` stopping envelope, same `a_max` — under [V_RADIAL_MAX_MS], because radial
 * motion is correction rather than travel.
 *
 * ## The one new safety property: the curvature cap
 *
 * > Circular motion at speed `v` on radius `R` requires centripetal acceleration `v²/R`. Bound it
 * > by the same `a_max` the stopping envelope uses:
 * > ```
 * > v_t_target = min(HORIZONTAL_MAX_MS, sqrt(a_max · R))
 * > ```
 *
 * It is the exact sibling of `sqrt(2·a_max·e)` and makes the same physical claim: **the aircraft
 * is never asked for more acceleration than we decided it may use.** At `a_max = 0.5`: `R = 10 m`
 * gives 2.24 m/s, `R = 18 m` gives 3.0 m/s and the envelope cap takes over, and at the 5 m floor
 * the speed is 1.58 m/s. Pinned as a **property over generated orbits** in `OrbitGuidanceTest`,
 * not as three examples.
 *
 * ## Why the radius floor is 5 m, stated where the number lives
 *
 * Not GPS noise alone. With `v ≤ sqrt(a_max·R)` the yaw feed-forward is `sqrt(a_max/R)·57.3`
 * °/s — 18.1 °/s at `R = 5`, inside the existing 30 °/s cap with room left for the correction
 * term, and *outside* it below about 1.8 m. The floor is what keeps the nose-to-centre loop from
 * saturating on the feed-forward alone.
 */
object OrbitGuidance {

    /**
     * The smallest circle this bridge will fly, metres. Below it the radial error signal is inside
     * GPS noise and — the binding reason — the yaw feed-forward `sqrt(a_max/R)` approaches
     * [GuidedEnvelope.YAW_RATE_MAX_DEGS] and leaves nothing for the heading correction.
     */
    const val R_MIN_M = 5.0

    /**
     * The largest circle this bridge will fly, metres. The whole circle must also fit inside the
     * Q1 leg bound ([GuidedEnvelope.MAX_REPOSITION_DISTANCE_M], 2 km since 2026-07-30) measured
     * from where the aircraft was when the command arrived — which is checked separately, and which
     * this constant alone does not guarantee.
     *
     * **Deliberately not raised with that bound.** The leg bound stopped being what binds this
     * number: what binds it is [ORBIT_MAX_S], because one turn at radius R costs `2*pi*R / 3 m/s`
     * and R = 50 m already spends 105 s of the 180 s cap. R = 100 m would be 210 s — a circle whose
     * time cap fires before it closes, which is a worse thing to offer than a refusal. See
     * `docs/decisions/2026-07-30-two-kilometre-envelope.md`.
     */
    const val R_MAX_M = 50.0

    /**
     * The radial axis's own cap, m/s. Deliberately far below [GuidedEnvelope.HORIZONTAL_MAX_MS]:
     * radial motion is *correction*, not travel, and a fast radial component on a circle is an
     * aircraft cutting across the middle of it.
     */
    const val V_RADIAL_MAX_MS = 1.0

    /**
     * The hard duration bound on one orbit, seconds. An orbit is the first manoeuvre in this
     * project with no natural completion, so **something must end it**; 180 s is three minutes of
     * circling, comfortably more than one turn at every radius in the band (the slowest, `R = 50 m`
     * at the 3 m/s envelope cap, is 105 s) and far short of a battery.
     */
    const val ORBIT_MAX_S = 180L

    /** `param4 = NaN` ("vehicle default") means this many turns. `docs/m4-mission-execution.md` §8.4. */
    const val DEFAULT_TURNS = 1.0

    /** One turn, in the degrees the swept-angle accumulator counts. */
    const val DEGREES_PER_TURN = 360.0

    /**
     * The nose-to-centre loop's proportional gain, °/s per ° of heading error.
     *
     * 1.0 means the correction term alone saturates [GuidedEnvelope.YAW_RATE_MAX_DEGS] at 30° of
     * error, so the aircraft turns at the cap while it is badly pointed and eases in over the last
     * half-turn — the same shape as the translation law's `k_p`, and chosen the same way: large
     * enough that the error actually closes, small enough that the loop does not chatter against a
     * ~12 Hz heading feed. **Chosen, not measured**; the first bench orbit is what confirms it.
     */
    const val K_YAW_PER_S = 1.0

    /**
     * Minimum spacing between gimbal commands during an orbit, milliseconds.
     *
     * On a steady orbit the solution is *constant* — radius and altitude are both held, so
     * `atan2` has two fixed arguments and the camera simply sits still. The angle only moves
     * during the join, on an altitude change, or on a retarget. So this is the least demanding
     * possible use of the gimbal path, and 500 ms is deliberately slower than
     * `GimbalManager.MIN_ROTATE_INTERVAL_MS`: there is nothing to track.
     */
    const val GIMBAL_MIN_INTERVAL_MS = 500L

    /**
     * How far the gimbal solution must move before it is re-commanded, degrees. 1.5° is below what
     * an operator can see in a frame and well above the jitter a GPS-derived horizontal distance
     * puts into `atan2` at close range.
     */
    const val GIMBAL_DEADBAND_DEG = 1.5

    /**
     * The curvature cap — **the new safety property**. `min(envelope, sqrt(a_max · R))`.
     *
     * Never call this with a radius outside the accepted band; a non-positive radius returns 0,
     * because the honest tangential speed on a circle of no radius is no speed at all.
     */
    fun tangentialCap(radiusM: Double): Double {
        if (!radiusM.isFinite() || radiusM <= 0.0) return 0.0
        return min(GuidedEnvelope.HORIZONTAL_MAX_MS, sqrt(RepositionGuidance.A_MAX_MS2 * radiusM))
    }

    /**
     * The tangential speed for this tick: the previous one plus at most `a_max · dt`, never above
     * [tangentialCap]. **Ramped, never stepped** — stepping the tangential speed asks for infinite
     * acceleration at the moment the circle begins, which is precisely the thing the cap exists to
     * bound.
     */
    fun rampedTangential(previousMs: Double, radiusM: Double, dtMs: Long): Double {
        val target = tangentialCap(radiusM)
        if (dtMs <= 0L) return min(previousMs, target)
        val step = RepositionGuidance.A_MAX_MS2 * (dtMs / 1000.0)
        return min(previousMs + step, target)
    }

    /**
     * The radial component, m/s along the **outward** unit radial: negative pulls the aircraft in
     * from outside the circle, positive pushes it out from inside.
     *
     * [RepositionGuidance.clampedSpeed] under [V_RADIAL_MAX_MS] — the M3 law in one dimension, with
     * nothing added and nothing removed.
     */
    fun radialSpeed(radialErrorM: Double): Double {
        if (!radialErrorM.isFinite() || radialErrorM == 0.0) return 0.0
        return -RepositionGuidance.clampedSpeed(abs(radialErrorM), V_RADIAL_MAX_MS) * sign(radialErrorM)
    }

    /**
     * The circling law for one tick: the NED offset **from the centre to the aircraft**, the
     * radius, the direction and an already-ramped tangential speed in, a velocity setpoint out.
     *
     * [direction] is `sign(param1)`: `+1` clockwise seen from above, `−1` counter-clockwise. The
     * tangential unit vector is `s · rotCW(û_r)` with `rotCW(n, e) = (−e, n)` — check it once and
     * never again: a clockwise aircraft due **north** of the centre must be flying **east**, and
     * `rotCW(1, 0) = (0, 1)` is exactly that.
     *
     * At the centre exactly (`r = 0`) there is no radial direction and no tangent, so the
     * horizontal axes command zero; the caller never reaches this state, because the join leg flies
     * to a point on the circle first.
     *
     * ## The composed magnitude is bounded, and that took a flight to notice
     *
     * The two components are **orthogonal**, so bounding each of them separately does *not* bound
     * their sum: `|v| = sqrt(v_r² + v_t²)`, which with the axis caps as written reaches
     * `sqrt(3.0² + 1.0²) = 3.162 m/s` — **5.4 % above [GuidedEnvelope.HORIZONTAL_MAX_MS]**.
     * `StickMapping.toDji` is a pure relabelling and clamps nothing, so nothing downstream caught
     * it either.
     *
     * Measured in flight on 2026-07-27 at 3.0021 m/s
     * (`docs/measurements/2026-07-27-takeoff-climb-and-ccw-orbit.md` §7.1) — a settled orbit spends
     * almost none of the headroom, which is exactly why it survived every previous orbit unnoticed.
     * A gust pushing the aircraft off the circle while the tangential speed is at its cap is what
     * would spend the rest.
     *
     * Ivan settled it as **S-3**: the constant bounds **ground speed**, which is how
     * [RepositionGuidance.velocity] has always read it — `hypot` first, then clamp the magnitude.
     * So the composed vector is scaled back to the envelope here, at the one point where the two
     * axes become one velocity.
     *
     * **Scaled, not truncated per axis.** Clipping the axes independently would rotate the
     * commanded direction — the aircraft would fly somewhere other than the sum asked for, which on
     * a circle means cutting the corner. Uniform scaling preserves the direction and pays for the
     * excess in speed, which is the axis with no safety argument attached. The visible consequence
     * is that during a radial correction at full tangential speed the tangential component dips
     * slightly, so a circle being blown off station takes marginally longer to come round.
     */
    fun circleVelocity(
        northFromCentreM: Double,
        eastFromCentreM: Double,
        radiusM: Double,
        direction: Int,
        tangentialMs: Double,
        downMs: Double,
    ): StickVelocities {
        if (!northFromCentreM.isFinite() || !eastFromCentreM.isFinite() || !downMs.isFinite()) {
            return StickVelocities.ZERO
        }
        val r = hypot(northFromCentreM, eastFromCentreM)
        if (r <= 0.0) return StickVelocities(0.0, 0.0, downMs, 0.0)
        val unitRadialNorth = northFromCentreM / r
        val unitRadialEast = eastFromCentreM / r
        // rotCW(n, e) = (−e, n), then the direction sign.
        val unitTangentNorth = -unitRadialEast * direction
        val unitTangentEast = unitRadialNorth * direction
        val radial = radialSpeed(r - radiusM)
        val north = radial * unitRadialNorth + tangentialMs * unitTangentNorth
        val east = radial * unitRadialEast + tangentialMs * unitTangentEast
        // S-3: the envelope bounds ground speed, and these two components are orthogonal, so it
        // has to be applied to the sum. Uniform scaling, never per-axis clipping — see the KDoc.
        val speed = hypot(north, east)
        val scale =
            if (speed > GuidedEnvelope.HORIZONTAL_MAX_MS) GuidedEnvelope.HORIZONTAL_MAX_MS / speed
            else 1.0
        return StickVelocities(
            north = north * scale,
            east = east * scale,
            down = downMs,
            // Yaw is added by the engine, and only by the engine, and only while circling.
            yawRateDegPerS = 0.0,
        )
    }

    /**
     * The bearing of an NED offset, degrees clockwise from north, in **(−180, 180]** — `atan2`'s
     * own range, which is where the swept-angle accumulator's wrap lives.
     *
     * Note the discontinuity is at **due south**, not due north:
     * `docs/m4-mission-execution.md` §8.4 says "a target that crosses due north", which is true of
     * a 0..360 convention and not of this one. `OrbitGuidanceTest` crosses **both**, because the
     * doc's sentence is about the class of bug rather than about one convention.
     */
    fun bearingDeg(northM: Double, eastM: Double): Double = Math.toDegrees(atan2(eastM, northM))

    /** An angle folded into (−180, 180]. The whole of the swept-angle wrap, in one place. */
    fun wrap180(degrees: Double): Double {
        if (!degrees.isFinite()) return 0.0
        var d = degrees % 360.0
        if (d > 180.0) d -= 360.0
        if (d <= -180.0) d += 360.0
        return d
    }

    /**
     * How much of the circle this tick swept, degrees, **always positive when the aircraft is
     * moving the way it was asked to**.
     *
     * The change in bearing from the centre, wrapped into ±180°, then multiplied by the direction
     * so a counter-clockwise orbit (whose bearing decreases) accumulates upward too. **The wrap is
     * the classic bug here**: without it, one tick that crosses the `atan2` discontinuity charges
     * the accumulator ~360° and the orbit "completes" instantly. It has its own test.
     *
     * A tick that drifts backwards returns a negative number and the accumulator goes down, which
     * is correct integration rather than a case to special-case: swept angle is a claim about where
     * the aircraft has been.
     */
    fun sweptDeltaDeg(previousBearingDeg: Double, bearingDeg: Double, direction: Int): Double =
        wrap180(bearingDeg - previousBearingDeg) * direction

    /**
     * The largest sweep that can fit inside [ORBIT_MAX_S] at this radius, degrees — the timeout
     * expressed as an angle, so a request for more turns than there is time for can be truncated
     * *and announced* rather than silently cut off by the clock.
     *
     * `ω = v/R` rad/s at the curvature cap, so `sweep = ORBIT_MAX_S · v_t_target / R` radians.
     */
    fun maxSweepDeg(radiusM: Double): Double {
        if (!radiusM.isFinite() || radiusM <= 0.0) return 0.0
        return Math.toDegrees(ORBIT_MAX_S * tangentialCap(radiusM) / radiusM)
    }

    /**
     * The point on the circle nearest the aircraft — the **join** target, computed once when the
     * command is accepted so the join is an ordinary resting leg toward a fixed lat/lon and not a
     * moving carrot.
     *
     * From the aircraft exactly at the centre there is no nearest point; due north of the centre is
     * chosen, arbitrarily and deliberately, because *some* point on the circle has to be picked and
     * a documented arbitrary choice beats a NaN.
     */
    fun joinPoint(
        centreLatDeg: Double,
        centreLonDeg: Double,
        radiusM: Double,
        fromLatDeg: Double,
        fromLonDeg: Double,
    ): Pair<Double, Double> {
        val (north, east) = RepositionGuidance.nedMetres(centreLatDeg, centreLonDeg, fromLatDeg, fromLonDeg)
        val r = hypot(north, east)
        val bearing = if (r <= 0.0) 0.0 else atan2(east, north)
        return RepositionGuidance.offsetCoordinate(
            centreLatDeg,
            centreLonDeg,
            radiusM * cos(bearing),
            radiusM * sin(bearing),
        )
    }

    /**
     * The nose-to-centre yaw rate, °/s clockwise-positive, clamped to
     * [GuidedEnvelope.YAW_RATE_MAX_DEGS] — **the existing constant, and no new envelope**.
     *
     * ```
     * ψ_target = bearing(p → C)                    // the centre, seen from the aircraft
     * ψ_error  = wrap180(ψ_target − heading)
     * ω_ff     = s · v_t / R · (180/π)             // the orbital rate, feed-forward
     * yawRate  = clamp(k_yaw · ψ_error + ω_ff, ±YAW_RATE_MAX_DEGS)
     * ```
     *
     * A **bounded closed loop on heading error**, not an open-loop rate: the feed-forward alone
     * would hold whatever pointing error the aircraft happened to arrive with, forever.
     *
     * [headingDeg] null — a stale or absent `yawDeg` — commands **zero**, never a guess. The caller
     * announces it; the lateral part of the orbit continues, which is the graduated treatment the
     * vertical axis already gets.
     */
    fun yawRate(
        northFromCentreM: Double,
        eastFromCentreM: Double,
        radiusM: Double,
        direction: Int,
        tangentialMs: Double,
        headingDeg: Double?,
    ): Double {
        if (headingDeg == null || !headingDeg.isFinite()) return 0.0
        if (!radiusM.isFinite() || radiusM <= 0.0) return 0.0
        // The centre seen from the aircraft is the negation of the aircraft seen from the centre.
        val target = bearingDeg(-northFromCentreM, -eastFromCentreM)
        val error = wrap180(target - headingDeg)
        val feedForward = direction * Math.toDegrees(tangentialMs / radiusM)
        return (K_YAW_PER_S * error + feedForward)
            .coerceIn(-GuidedEnvelope.YAW_RATE_MAX_DEGS, GuidedEnvelope.YAW_RATE_MAX_DEGS)
    }

    /**
     * The gimbal pitch that puts a target in frame, degrees, **negative down** (DJI's convention and
     * QGC's, and no sign flip happens anywhere between here and the camera).
     *
     * ```
     * pitch = −atan2(heightAboveTarget, horizontalDistance)
     * ```
     *
     * **Both quantities are in our own frame** — a height difference in *our* takeoff datum, a
     * distance from *our* position — so nothing is borrowed from a foreign datum and there is nothing
     * to convert. That is what the first argument's name means and it is the whole of the sign
     * discipline: the caller subtracts the target's height from the aircraft's, and a target the
     * aircraft is *below* therefore arrives here **negative** and solves to a positive, upward pitch —
     * a real case (a tower, a rooftop), clamped to the gimbal's reported travel and announced by the
     * caller, never silently flattened.
     *
     * For an orbit the height above the target *is* the relative altitude, because a circle's centre
     * is on the ground by construction (M4-6: *"orbit implies ROI at the circle's centre, at takeoff
     * height"*). For an ROI it depends on what the command's frame could tell us: a
     * `GLOBAL_RELATIVE_ALT` z is our own datum and is used; an AMSL or terrain-relative z is a number
     * whose reference moved 41.5 m between sessions and is discarded, leaving the announced
     * ground-level assumption. `guided/Roi`'s KDoc carries that decision in full; this function is
     * only the trigonometry, and it is deliberately the one copy of it.
     */
    fun gimbalPitchDeg(heightAboveTargetM: Double, horizontalToCentreM: Double): Double =
        -Math.toDegrees(atan2(heightAboveTargetM, horizontalToCentreM))
}
