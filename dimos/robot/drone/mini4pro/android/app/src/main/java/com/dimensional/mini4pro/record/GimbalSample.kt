package com.dimensional.mini4pro.record

import kotlin.math.abs

/**
 * One reading of where the camera is pointing, as the flight recorder wants it.
 *
 * A `record/`-owned type deliberately identical in content to the three angles and the age of
 * `gimbal/GimbalReading`, **so that this package imports nothing from `gimbal/`**. That is the
 * same choice `ZenohTelemetryEncoder.gimbalAttitudeOrNull` makes when it takes loose angles rather
 * than a gimbal type, and it is why the gimbal seam can be re-shaped by whoever owns it without a
 * recorder change.
 *
 * Every axis independently nullable: DJI's `Attitude` getters are boxed `Double`s and each can be
 * null on its own, and null means *no reading* rather than 0° — which is a real angle, and on this
 * airframe the resting one.
 */
data class GimbalSample(
    val pitchDeg: Double? = null,
    val rollDeg: Double? = null,
    val yawDeg: Double? = null,
    /** Milliseconds since DJI last delivered the attitude, or null if it never has. */
    val ageMs: Long? = null,
) {

    /**
     * Whether this reading differs from [previous] by more than [deadbandDeg] on any axis.
     *
     * **An axis appearing or disappearing is always a move**, however small the number beside it.
     * A gimbal that stops reporting yaw has told us something, and a deadband that compared only
     * the axes both readings happen to carry would record the last known yaw forever — the
     * absence-is-not-zero rule this format applies everywhere, at the one place where it is easy
     * to drop it.
     *
     * A non-finite reading is handled explicitly rather than by arithmetic. `abs(NaN - NaN) >=
     * band` is `false`, so it *looks* right by accident; `NaN != NaN` — which is what an ordinary
     * comparison gives, because IEEE says so — is `true`, so a gimbal stuck at `NaN` would report
     * a move on every sample and turn this entry into the firehose it was designed not to be.
     * Found by `GimbalSampleTest.anaNArrivingIsAMoveAndAPersistingNaNIsNot`, which was written
     * before the code was right. So: **`NaN` arriving is a move, `NaN` persisting is not**, and an
     * infinity is compared normally (`+∞ == +∞`, and `+∞` → `−∞` is a move).
     */
    fun movedFrom(previous: GimbalSample, deadbandDeg: Double): Boolean =
        axisMoved(pitchDeg, previous.pitchDeg, deadbandDeg) ||
            axisMoved(rollDeg, previous.rollDeg, deadbandDeg) ||
            axisMoved(yawDeg, previous.yawDeg, deadbandDeg)

    private fun axisMoved(now: Double?, before: Double?, band: Double): Boolean {
        if (now == null || before == null) return (now == null) != (before == null)
        if (now.isNaN() || before.isNaN()) return now.isNaN() != before.isNaN()
        if (!now.isFinite() || !before.isFinite()) return now != before
        return abs(now - before) >= band
    }
}
