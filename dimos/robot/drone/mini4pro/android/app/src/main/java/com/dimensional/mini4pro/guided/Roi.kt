package com.dimensional.mini4pro.guided

import kotlin.math.hypot

/**
 * One `MAV_CMD_DO_SET_ROI_LOCATION` (195), `MAV_CMD_DO_SET_ROI_NONE` (197) or legacy
 * `MAV_CMD_DO_SET_ROI` (201) as it came off the wire, flattened to plain values so
 * [GuidedStickEngine.roi] can be driven from a unit test without `handshake/` imports — the same
 * reason [RepositionCommand] and [OrbitCommand] exist.
 *
 * **Measured shape** (`HandshakeResponder.kt`'s table, off QGC's wire):
 *
 * ```
 * COMMAND_INT 195  frame = 0 (MAV_FRAME_GLOBAL)
 * param1..4 = NaN
 * x, y      = the map click, 1e7 degrees
 * z         = 0.0   ← a terrain-database AMSL that happened to resolve to zero
 * ```
 *
 * ## The z: **discarded for the frames that cannot be read, honoured for the frame that can**
 *
 * The original decision — *"the z is not used at all"* — was measured against QGC's **Fly-view** ROI
 * and it stands for that shape unchanged. For PX4, which is the identity we present, QGC does not
 * send the click's own altitude: it runs a **terrain-database query first**
 * (`Vehicle::guidedModeROI` → `TerrainQueryCoordinator::roiWithTerrain` → `sendROICommand`) and sends
 * the result as an **AMSL in `frame = 0`**. Our own AMSL datum is a barometer on the 1013.25 hPa
 * reference that moved **41.5 m between sessions**. Subtracting one from the other is not the exact
 * cancellation `DO_REPOSITION` and `DO_ORBIT` enjoy (where QGC composed `z` from *our own* published
 * datum and our datum leaves with the opposite sign); it is two unrelated numbers, and a 40 m
 * vertical error at 30 m of range is a pointing error of tens of degrees. So for frame 0 — and for
 * every other frame whose `z` is an AMSL or a terrain-relative height we have no database for — the
 * `z` is still **discarded**, the target is assumed to lie at the takeoff datum's ground level, and
 * that assumption is announced once per ROI as [GuidedStatusTexts.ROI_GROUND_LEVEL].
 *
 * **A `MAV_FRAME_GLOBAL_RELATIVE_ALT` (3) ROI is a different number entirely**, and discarding it was
 * over-broad (Ivan, 2026-07-30: *"ROI in the plan does have a height setting, it should be relative
 * height to takeoff height"*). Its `z` is metres **above the takeoff point** — our own datum, the same
 * one every waypoint altitude, the ceiling, the descent and the tag fixes are expressed in — so there
 * is nothing to convert and nothing to distrust: it is directly usable, and `MissionAdmission` already
 * refuses one that is not finite. That is the frame a **plan's** `DO_SET_ROI_LOCATION` item carries
 * (verified in `big1.plan`: item 6, `frame = 3`, params `[0,0,0,0, 37.99387681, 23.7257871, 0]`), and
 * it is the frame any GCS sending `COMMAND_INT` 195 in our own datum would use.
 *
 * With a usable height the pitch solve reads the **height difference**, `aircraft − target`, instead
 * of the aircraft's altitude alone ([RoiCommand.relativeAltMOrNull] → `GuidedStickEngine.roi` →
 * [OrbitGuidance.gimbalPitchDeg]). A target *above* the aircraft — a tower, a hillside subject, a
 * rooftop the aircraft is beneath — is then a negative depression, i.e. the camera aiming **up**,
 * which is a real case rather than an error: it is clamped to the gimbal's own reported travel and
 * announced as [GuidedStatusTexts.ROI_GIMBAL_RANGE] when the clamp bites, through the existing
 * `reachablePitch` path and no new one.
 *
 * Note **big1.plan's own ROI z is 0**, so this changes nothing for that plan: 0 metres above the
 * takeoff datum *is* ground level and the arithmetic is identical. It is the nonzero case the fix
 * exists for. `docs/m4-mission-execution.md` §9.1 and
 * `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-9, as amended.
 */
data class RoiCommand(
    /** The `MAV_CMD` id: [MAV_CMD_DO_SET_ROI_LOCATION], [MAV_CMD_DO_SET_ROI_NONE] or [MAV_CMD_DO_SET_ROI]. */
    val command: Int,
    val isCommandInt: Boolean,
    /** `MAV_FRAME`; `COMMAND_INT` only. See [GLOBAL_FRAMES]. */
    val frame: Int = 0,
    /** `COMMAND_INT.x`, 1e7-scaled degrees latitude. */
    val latE7: Int = 0,
    /** `COMMAND_INT.y`, 1e7-scaled degrees longitude. */
    val lonE7: Int = 0,
    /** `param1` — for [MAV_CMD_DO_SET_ROI] only, a `MAV_ROI` enum value. NaN everywhere else. */
    val param1: Float = Float.NaN,
    /** `param5` — the legacy `COMMAND_LONG` 201's latitude, degrees as a float. */
    val param5: Float = Float.NaN,
    /** `param6` — the legacy `COMMAND_LONG` 201's longitude, degrees as a float. */
    val param6: Float = Float.NaN,
    /**
     * `COMMAND_INT.z`, or `COMMAND_LONG.param7` — **the same field by MAVLink's definition**, and
     * whether it is a height this bridge can read is entirely [frame]'s business. See
     * [relativeAltMOrNull].
     */
    val z: Float = Float.NaN,
) {

    /**
     * The place this command points at, in degrees, or **null when the shape carries no usable
     * coordinate pair**. No validation beyond finiteness happens here; whether the pair is a
     * coordinate this bridge will point at is `Geo.coordinateOrNull`'s question and the engine's to
     * ask.
     *
     * The `COMMAND_INT` path is the measured one and the precise one. The legacy `COMMAND_LONG` 201
     * path carries degrees in **float32**, whose spacing at our latitudes is `38 · 2⁻²³ ≈ 4.5e-6°`,
     * about **0.5 m** on the ground. That is comfortably inside what a camera cares about and
     * comfortably outside what a flight path would, which is exactly why 201 is accepted for
     * *aiming* and why no manoeuvre in this project accepts float coordinates.
     */
    fun targetDegrees(): Pair<Double, Double>? {
        if (isCommandInt) return latE7 / 1e7 to lonE7 / 1e7
        if (!param5.isFinite() || !param6.isFinite()) return null
        return param5.toDouble() to param6.toDouble()
    }

    /**
     * The target's height **in our own datum** — metres above the takeoff point — or null when this
     * command's frame does not carry one we can read. The whole of the class KDoc's z decision, in one
     * function, so no call site can re-decide it.
     *
     * Null for:
     *
     *  - **every `COMMAND_LONG`** (including the legacy 201): no frame field at all, so the `param7`
     *    that arrives could be anything. Unknown is never zero;
     *  - **frames 0 and 5** (`GLOBAL`, `GLOBAL_INT`): AMSL, against a datum measured 41.5 m adrift
     *    between sessions — the *measured* discard, and it must survive;
     *  - **frames 10 and 11** (`GLOBAL_TERRAIN_ALT`): height above a terrain model this bridge does not
     *    have. Nothing to compare it to;
     *  - a non-finite value in any frame.
     */
    fun relativeAltMOrNull(): Double? {
        if (!isCommandInt) return null
        if (frame !in RELATIVE_FRAMES) return null
        return z.toDouble().takeIf { it.isFinite() }
    }

    /**
     * True when this command means **stop pointing**: `DO_SET_ROI_NONE`, or the legacy 201 carrying
     * [MAV_ROI_NONE]. Clearing is accepted in every state and behind no gate — turning something off
     * is always allowed.
     */
    fun isClear(): Boolean = command == MAV_CMD_DO_SET_ROI_NONE ||
        (command == MAV_CMD_DO_SET_ROI && param1.isFinite() && param1.toInt() == MAV_ROI_NONE)

    companion object {
        /** `MAV_CMD_DO_SET_ROI_LOCATION`. A number, for the reason `HandshakeResponder` spells ids as numbers. */
        const val MAV_CMD_DO_SET_ROI_LOCATION = 195

        /** `MAV_CMD_DO_SET_ROI_NONE`. */
        const val MAV_CMD_DO_SET_ROI_NONE = 197

        /** `MAV_CMD_DO_SET_ROI` — deprecated in MAVLink, still what imported plans and older GCSs send. */
        const val MAV_CMD_DO_SET_ROI = 201

        /** `MAV_ROI_NONE` — 201's `param1` spelling of "stop pointing". */
        const val MAV_ROI_NONE = 0

        /** `MAV_ROI_LOCATION` — 201's `param1` spelling of "point at this lat/lon". */
        const val MAV_ROI_LOCATION = 3

        /**
         * Every `MAV_FRAME` whose `COMMAND_INT.x`/`y` are 1e7-scaled degrees — `GLOBAL` (0),
         * `GLOBAL_RELATIVE_ALT` (3), `GLOBAL_INT` (5), `GLOBAL_RELATIVE_ALT_INT` (6),
         * `GLOBAL_TERRAIN_ALT` (10) and `GLOBAL_TERRAIN_ALT_INT` (11).
         *
         * **Wider than `DO_REPOSITION`'s and `DO_ORBIT`'s single measured frame, and that is an
         * argument rather than an oversight**: those two read `z` and therefore care intensely which
         * datum the frame names, so anything unmeasured is refused. An ROI *discards* `z`, so the
         * only thing the frame decides here is the encoding of the horizontal pair — identical
         * across all six. A `LOCAL_NED` frame, whose `x`/`y` would be centimetres, is refused,
         * because there the difference is real.
         *
         * Only frame 0 has been measured off QGC's Fly-view wire. The rest are the plan-item door
         * (`docs/m4-mission-transport.md` §6.2) arriving early.
         */
        val GLOBAL_FRAMES = setOf(0, 3, 5, 6, 10, 11)

        /**
         * **A plan's `DO_SET_ROI_LOCATION` item, spelled as the wire command it is** — the second
         * door onto [GuidedStickEngine.roi]'s machinery, and the reason there is no second ROI type.
         *
         * `mission/MissionStore.resolve` has already done the sticky walk and produced a
         * `RoiTarget(GeoPoint, relativeAltM)` for the leg; `MissionLaunch.routeOf` turns that back
         * into one of these for [MissionStep.roi], so what reaches the engine from a plan item is
         * **the same value type, through the same gates, into the same state** as the operator's
         * Fly-view click. The alternative — a `setRoi(lat, lon, alt)` entry beside the command one —
         * is the two-places-for-one-property failure with a fresh coat of paint.
         *
         * Three encoding choices, each forced rather than chosen:
         *
         *  - **[isCommandInt] is true** — the 1e7-integer shape, which is the only one this bridge
         *    accepts for 195 at all (the float-coordinate `COMMAND_LONG` shape is unmeasured and
         *    refused). A plan item *is* a `MISSION_ITEM_INT`, so integer degrees is what it carried.
         *  - **the frame is `GLOBAL_RELATIVE_ALT` (3)**, and it is not a guess: `MissionGeo.pointOrNull`
         *    resolves a coordinate **only** for `MissionFrames.RELATIVE` = {3, 6}, so an ROI item in
         *    any other frame is already refused at the desk as *"not a coordinate"* and cannot reach
         *    here. 3 and 6 mean the identical thing to [relativeAltMOrNull]; naming the first is
         *    honest about the pair rather than pretending to remember which of the two arrived.
         *  - **`z` is [relativeAltM] when there is one and `NaN` when there is not** — never 0.0.
         *    Unknown is never zero: a NaN comes back out of [relativeAltMOrNull] as null, which is the
         *    ground-level *assumption* with its own announcement, while a 0.0 would be a measurement
         *    nobody made. (`MissionAdmission` refuses a non-finite ROI `z` at the desk, so the null
         *    branch is unreachable through the plan door today and exists so that it cannot become a
         *    silent zero the day something else calls this.)
         *
         * The `e7` round trip is exact: `x → x/1e7 → round(·×1e7)` recovers the integer for every
         * coordinate MAVLink can express, so the item's own numbers survive into the pointing solution
         * unrounded.
         */
        fun pointingAt(latDeg: Double, lonDeg: Double, relativeAltM: Double?): RoiCommand = RoiCommand(
            command = MAV_CMD_DO_SET_ROI_LOCATION,
            isCommandInt = true,
            frame = 3,
            latE7 = Math.round(latDeg * 1e7).toInt(),
            lonE7 = Math.round(lonDeg * 1e7).toInt(),
            z = relativeAltM?.toFloat() ?: Float.NaN,
        )

        /**
         * **A plan's `DO_SET_ROI_NONE` item**, and the mission's own "stop pointing" — the same
         * command QGC's ROI-off button sends, so it lands on the same single clearing path
         * ([GuidedStickEngine.clearRoiTargetsLocked]) with the same record line and the same sentence.
         *
         * Carries no coordinate and no frame that matters: [isClear] answers on the command id alone,
         * ahead of every shape check, because turning something off is always allowed.
         */
        fun clearing(): RoiCommand = RoiCommand(
            command = MAV_CMD_DO_SET_ROI_NONE,
            isCommandInt = true,
        )

        /**
         * The two of [GLOBAL_FRAMES] whose `z` is **metres above the takeoff point** —
         * `GLOBAL_RELATIVE_ALT` (3) and `GLOBAL_RELATIVE_ALT_INT` (6), the same pair
         * `mission/MissionFrames.RELATIVE` admits and for the same reason: it is *our* datum, so there
         * is no conversion to get wrong and no foreign reference to distrust.
         *
         * Deliberately not spelled by importing that set: this package does not depend on `mission/`,
         * and the two definitions answer different questions (which plans we accept; which ROI heights
         * we can read). They agree, and they agree because MAVLink defines the frames, not because one
         * reads the other.
         */
        val RELATIVE_FRAMES = setOf(3, 6)
    }
}

/**
 * The region-of-interest arithmetic: pure functions, no state, no DJI, no Android, pinned by
 * `RoiGuidanceTest`.
 *
 * Design authority: `docs/m4-mission-execution.md` §9 (the pointing solution, the azimuth problem,
 * the update-rate rules and what an abort does) and `docs/decisions/2026-07-27-orbit-yaw.md` (the
 * yaw permission and its bounds, which name ROI tracking explicitly).
 *
 * ## What is deliberately **not** here
 *
 * **The pitch solution.** It is [OrbitGuidance.gimbalPitchDeg], called rather than restated: an
 * orbit pointing at its centre and an ROI pointing at a map click are the *same* problem —
 * `−atan2(relativeAltitude, horizontalDistance)` against a target assumed to sit at the takeoff
 * datum's ground level — and one of the design's standing instructions is that there be exactly one
 * of it. The rate limiter and the deadband are [OrbitGuidance.GIMBAL_MIN_INTERVAL_MS] and
 * [OrbitGuidance.GIMBAL_DEADBAND_DEG] for the same reason, even though an ROI leans on them harder:
 * an orbit's solution is *constant* on a steady circle, while an ROI's moves continuously as the
 * aircraft flies past the target.
 *
 * ## The azimuth half, and why it is a yaw
 *
 * This gimbal cannot yaw at all — DJI issue #527, where setting yaw or roll fails the entire
 * rotation command — so the only way to change what the camera is pointed *at* horizontally is to
 * turn the airframe. That is flight control, and the permission for it is narrow:
 *
 * > **We point the camera freely; we point the aircraft only when we are already the one flying
 * > it.** (`docs/m4-mission-execution.md` §9.3)
 *
 * The engine enforces the scope; this object supplies the law, which is the *same* bounded closed
 * loop the orbit's nose-to-centre uses — [OrbitGuidance.K_YAW_PER_S] on the heading error plus a
 * feed-forward, clamped to the existing [GuidedEnvelope.YAW_RATE_MAX_DEGS]. No new envelope, no new
 * gain.
 */
object RoiGuidance {

    /**
     * How close the aircraft may get, horizontally, before the pointing solution stops being worth
     * chasing — metres.
     *
     * Inside it the geometry is ill-conditioned in **both** axes at once. The pitch solution
     * approaches −90° and its derivative with respect to horizontal range blows up (at 20 m of
     * altitude, moving from 3 m to 2 m of range swings the pitch by 8.5°, and GPS noise alone is a
     * good fraction of a metre); the bearing is worse, because at 1 m of range a 1 m lateral drift
     * is a 45° step and the aircraft would be asked to spin at the yaw cap. So below this the
     * camera **holds its last angle** and the airframe is asked for no yaw at all, announced once.
     *
     * A camera hunting at the limit is worse than a camera pointing approximately down, and an
     * aircraft pirouetting under the operator's subject is worse than both.
     */
    const val MIN_RANGE_M = 3.0

    /**
     * How far the nose may be off the target's bearing before the operator is told we are not going
     * to fix it, degrees. `docs/m4-mission-execution.md` §9.3's `ROI_YAW_DEADBAND_DEG`.
     *
     * **A threshold for a sentence, not for the loop.** The yaw law itself has no deadband — it is
     * the orbit's law unchanged, and adding one would be a second behaviour to reason about. This
     * number exists so that an ROI we can only half-honour says so when the half actually matters:
     * a nose 2° off the target still has it in frame, and announcing that would be noise.
     */
    const val YAW_DEADBAND_DEG = 5.0

    /**
     * The rate at which the bearing to a **fixed** target changes because the aircraft is moving,
     * degrees per second, clockwise-positive — the ROI loop's feed-forward.
     *
     * With the line of sight `(N, E)` from aircraft to target and a ground velocity `(v_n, v_e)`,
     * the target is fixed so `Ṅ = −v_n`, `Ė = −v_e`, and differentiating `θ = atan2(E, N)` gives
     *
     * ```
     * θ̇ = (N·Ė − E·Ṅ)/(N² + E²) = (E·v_n − N·v_e)/d²
     * ```
     *
     * **This is exactly the orbit's own feed-forward, generalised.** Put the aircraft due north of
     * an orbit centre at radius R flying clockwise: `N = −R`, `E = 0`, `v = (0, v_t)`, and the
     * expression gives `+v_t/R` — which is `direction · v_t/R`, the term
     * [OrbitGuidance.yawRate] has always used, to the digit. The orbit's version is kept where it is
     * (it is written in terms of the circle it already knows) and this one is used wherever the path
     * is not a circle, which is every other ROI.
     *
     * Fed the **commanded** velocity rather than the measured one, deliberately: it is what the
     * aircraft has just been asked to do, it needs no freshness gate of its own, and it is the same
     * choice the orbit makes when it feeds forward its own ramped tangential speed rather than a
     * GPS-derived one.
     *
     * Zero inside [MIN_RANGE_M] and for any non-finite input — a feed-forward that can be enormous
     * is worse than none, and the closed loop still has the error term.
     */
    fun bearingRateDegPerS(
        losNorthM: Double,
        losEastM: Double,
        velocityNorthMs: Double,
        velocityEastMs: Double,
    ): Double {
        if (!losNorthM.isFinite() || !losEastM.isFinite()) return 0.0
        if (!velocityNorthMs.isFinite() || !velocityEastMs.isFinite()) return 0.0
        val d = hypot(losNorthM, losEastM)
        if (d < MIN_RANGE_M) return 0.0
        return Math.toDegrees((losEastM * velocityNorthMs - losNorthM * velocityEastMs) / (d * d))
    }

    /**
     * The yaw rate that swings the nose onto the ROI's bearing, °/s clockwise-positive, clamped to
     * [GuidedEnvelope.YAW_RATE_MAX_DEGS] — **the existing constant, and no new envelope**.
     *
     * ```
     * ψ_target = bearing(aircraft → ROI)
     * ψ_error  = wrap180(ψ_target − heading)
     * ω_ff     = bearingRateDegPerS(los, commanded velocity)
     * yawRate  = clamp(k_yaw · ψ_error + ω_ff, ±YAW_RATE_MAX_DEGS)
     * ```
     *
     * A **bounded closed loop on heading error**, exactly as the orbit's is and for the same reason:
     * the feed-forward alone would hold whatever pointing error the aircraft happened to arrive
     * with, forever.
     *
     * [headingDeg] null — a stale or absent `yawDeg` — commands **zero**, never a guess; the caller
     * announces it and the flight path continues, which is the graduated treatment every other feed
     * gets here. Inside [MIN_RANGE_M] it also commands zero: see that constant.
     *
     * The caller is responsible for the *scope* — this function has no idea who is flying, and the
     * one rule that matters about ROI yaw ("never on an aircraft somebody else is flying") is
     * enforced at the call sites, which are the two manoeuvre ticks and nowhere else.
     */
    fun yawRate(
        losNorthM: Double,
        losEastM: Double,
        headingDeg: Double?,
        commandedNorthMs: Double,
        commandedEastMs: Double,
    ): Double {
        if (headingDeg == null || !headingDeg.isFinite()) return 0.0
        if (!losNorthM.isFinite() || !losEastM.isFinite()) return 0.0
        if (hypot(losNorthM, losEastM) < MIN_RANGE_M) return 0.0
        val error = OrbitGuidance.wrap180(OrbitGuidance.bearingDeg(losNorthM, losEastM) - headingDeg)
        val feedForward = bearingRateDegPerS(losNorthM, losEastM, commandedNorthMs, commandedEastMs)
        return (OrbitGuidance.K_YAW_PER_S * error + feedForward)
            .coerceIn(-GuidedEnvelope.YAW_RATE_MAX_DEGS, GuidedEnvelope.YAW_RATE_MAX_DEGS)
    }

    /**
     * How far the nose is from the ROI's bearing, degrees in (−180, 180], or null when the heading
     * cannot be trusted or the geometry is degenerate. What [YAW_DEADBAND_DEG] is compared against.
     */
    fun azimuthErrorDeg(losNorthM: Double, losEastM: Double, headingDeg: Double?): Double? {
        if (headingDeg == null || !headingDeg.isFinite()) return null
        if (!losNorthM.isFinite() || !losEastM.isFinite()) return null
        if (hypot(losNorthM, losEastM) < MIN_RANGE_M) return null
        return OrbitGuidance.wrap180(OrbitGuidance.bearingDeg(losNorthM, losEastM) - headingDeg)
    }

    /** True when the aircraft is inside [MIN_RANGE_M] of the target, horizontally. */
    fun tooClose(horizontalM: Double): Boolean = !horizontalM.isFinite() || horizontalM < MIN_RANGE_M
}
