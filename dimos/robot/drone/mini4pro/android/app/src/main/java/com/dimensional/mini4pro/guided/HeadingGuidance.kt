package com.dimensional.mini4pro.guided

import kotlin.math.hypot

/**
 * **Heading follows course**: while this bridge is flying the aircraft somewhere, the nose points
 * that way.
 *
 * Design authority: `docs/decisions/2026-07-27-heading-follows-course.md`, requested by Ivan after
 * watching the first real mission-shaped flight — *"the drone is still not turning towards the
 * destination when it's flying a waypoint. No yaw change."* It is the omission that decision doc
 * describes, not a bug: until now the bridge generated yaw in exactly one place, the orbit's
 * nose-to-centre loop, so a goto and a mission leg flew sideways with the nose wherever it started.
 *
 * ## The law is the orbit's, with the feed-forward dropped
 *
 * [OrbitGuidance.yawRate] is a bounded closed loop on heading error plus an orbital feed-forward. A
 * straight leg has no orbital rate, so the feed-forward term is simply absent and everything else is
 * reused literally: the same [OrbitGuidance.K_YAW_PER_S] gain, the same
 * [GuidedEnvelope.YAW_RATE_MAX_DEGS] clamp, the same stale-heading-commands-zero rule. **No new
 * envelope and no new constant** — that is the whole of why this is safe to add to a manoeuvre that
 * has already flown.
 *
 * ## Bearing to the target, not course over ground
 *
 * Two readings of *"where it's going"* and they differ. Course over ground is derived from a
 * velocity that is noisy at low speed and **undefined at rest**, so the nose would hunt while the
 * aircraft settled. Bearing to the target is defined the moment a target exists, is stable, and is
 * what a human means by the phrase. On a straight leg in still air the two are identical; where they
 * diverge — crabbing across wind — facing the target is the more useful picture.
 *
 * ## Where it deliberately stops
 *
 *  - **Inside [RepositionGuidance.R_ACCEPT_M], hold heading.** No new constant: the arrival radius
 *    is exactly the distance at which a metre of GPS noise swings the bearing wildly, and the last
 *    two metres are flown with the nose already where it ended up.
 *  - **A vertical-only leg holds heading**, which falls out of the same rule rather than needing its
 *    own: a pure climb has no horizontal error, so it is inside the radius by construction.
 *  - **A stale or absent heading commands zero**, never a guess. The caller announces it and the
 *    translation continues — the graduated treatment every other feed gets here.
 *
 * ## What outranks it
 *
 * An **ROI wins**: if the operator has said "look at that", the camera is the point of the flight
 * and the nose serves the camera. Facing forward is the *default*, not a priority. An **orbit wins**
 * trivially, its nose-to-centre being a heading command already. And the **operator's own stick
 * wins**, as always — which is not this object's business, because the rule that matters there is
 * *generate versus relay*: Stage A has always relayed the pilot's own `r` axis to a yaw rate and
 * must continue to. This object is only ever consulted where the bridge is the one flying.
 *
 * ## It is a substitution, and it is announced
 *
 * QGC sends `DO_REPOSITION.param4 = NaN`, which MAVLink defines as *"unchanged or vehicle
 * default"*. Reading it as *our* default is legitimate — but the bridge has read it as "keep
 * heading" until now and that behaviour is flight-verified, so the change is announced
 * ([GuidedStatusTexts.HEADING_FOLLOWS]) once per manoeuvre. A **finite `param4` stays refused**: a
 * commanded absolute heading is a different feature with its own argument.
 */
object HeadingGuidance {

    /**
     * The yaw rate that swings the nose onto the bearing to the target, °/s clockwise-positive,
     * clamped to [GuidedEnvelope.YAW_RATE_MAX_DEGS].
     *
     * ```
     * ψ_target = bearing(aircraft → target)
     * ψ_error  = wrap180(ψ_target − heading)
     * yawRate  = clamp(k_yaw · ψ_error, ±YAW_RATE_MAX_DEGS)
     * ```
     *
     * [errorNorthM]/[errorEastM] are the NED offset **from the aircraft to the target** — the same
     * vector the translation law is given, so there is no second geodesy and no chance of the two
     * disagreeing about which way the target is.
     *
     * Returns exactly `0.0` when the heading cannot be trusted, when the horizontal error is inside
     * [RepositionGuidance.R_ACCEPT_M] (which includes a vertical-only leg, whose horizontal error is
     * zero), and for any non-finite input. Zero is the honest answer in every one of those cases:
     * there is no direction to face, or no measurement of which way we are facing.
     */
    fun yawRate(errorNorthM: Double, errorEastM: Double, headingDeg: Double?): Double {
        if (headingDeg == null || !headingDeg.isFinite()) return 0.0
        if (!errorNorthM.isFinite() || !errorEastM.isFinite()) return 0.0
        if (hypot(errorNorthM, errorEastM) <= RepositionGuidance.R_ACCEPT_M) return 0.0
        val error = OrbitGuidance.wrap180(
            OrbitGuidance.bearingDeg(errorNorthM, errorEastM) - headingDeg
        )
        return (OrbitGuidance.K_YAW_PER_S * error)
            .coerceIn(-GuidedEnvelope.YAW_RATE_MAX_DEGS, GuidedEnvelope.YAW_RATE_MAX_DEGS)
    }

    /**
     * True when [yawRate] would command nothing because the target is too close to have a bearing
     * worth facing — the hold-heading regime. Exposed so the engine can tell "we are holding
     * heading because we are nearly there" from "we are holding heading because the heading feed is
     * dead", which are different sentences.
     */
    fun holdingHeading(errorNorthM: Double, errorEastM: Double): Boolean {
        if (!errorNorthM.isFinite() || !errorEastM.isFinite()) return true
        return hypot(errorNorthM, errorEastM) <= RepositionGuidance.R_ACCEPT_M
    }
}
