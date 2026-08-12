package com.dimensional.mini4pro.gimbal

/**
 * **The pitch this bridge believes the camera holds, and where the belief comes from** — the one
 * implementation of the commanded/reported precedence, pure so `PitchBeliefTest` pins it without
 * an aircraft. Resolved in exactly one place ([MsdkGimbalAim.believedPitch], surfaced as
 * [GimbalManager.believedPitch]); every camera-pointing consumer reads that owner, so the arm
 * gate and the fix pipeline cannot disagree about where the camera points.
 *
 * ## Why two sources, and why this order
 *
 * The **commanded** angle ([CommandedGimbalPort.pitchDeg], via [GimbalPort.commandedPitchDeg])
 * is exact and never goes stale: it is a statement about what this bridge asked for,
 * `absolute = true`, remembered on DJI's success callback. It wins whenever it exists.
 *
 * The **reported** angle (`KeyGimbalAttitude`, [MsdkGimbalAim]'s own listener state) is the
 * fallback, and its existence here is not a weakening of the commanded-angle rule — it is the
 * other half of the rule as the old `commandedPitchDeg` KDoc mandated of every consumer: *"Null
 * is not 'level', it is 'we have not aimed it'… A consumer of this property must fall back to
 * the reported angle and say so, never substitute a zero."* The 2026-07-28 sessions (records
 * 20260728-213858 and -214210) are the measured cost of leaving that mandate to each consumer:
 * Ivan aimed the camera with the RC wheel, the reported pitch sat at −90° all approach, the
 * commanded pitch was null, and every descent arm died — several flights lost to a refusal that
 * named the wrong thing.
 *
 * ## What is deliberately absent: an age
 *
 * `KeyGimbalAttitude` is **change-driven** — silence means *unchanged*, not stale (median
 * delivery age 4.0 s holding still, max 25.3 s measured; the failing 2026-07-28 sessions show a
 * correct −90° at age 8.1 s). Freshness-gating the reported value by age is precisely the trap
 * this project has hit seven times, so this type cannot express one: it takes two values and
 * returns one, and the last-reported value is believed for as long as it is the last.
 *
 * The residual honesty gap is real and is carried rather than hidden: neither source proves the
 * camera is *there* (the commanded angle can be overridden by the RC wheel; the reported one lags
 * a moving gimbal by its delivery). [reported] rides every fix into the flight record
 * (`TagFix.pitchReported`) so a post-flight reader can tell which belief a fix rested on —
 * the `TagFix.bearingAssumed` pattern, applied to this assumption.
 */
data class PitchBelief(
    /** Degrees, −90 at nadir. Never fabricated: one of the two sources said this number. */
    val pitchDeg: Double,
    /** True when [pitchDeg] came from `KeyGimbalAttitude` because nothing was commanded. */
    val reported: Boolean,
) {
    companion object {
        /**
         * Resolve the two sources into one belief, or **null when there is neither** — and null
         * is the answer then, never zero: a level camera is a claim, and nobody made it.
         */
        fun of(commandedDeg: Double?, reportedDeg: Double?): PitchBelief? = when {
            commandedDeg != null -> PitchBelief(commandedDeg, reported = false)
            reportedDeg != null -> PitchBelief(reportedDeg, reported = true)
            else -> null
        }
    }
}
