package com.dimensional.mini4pro.gimbal

/**
 * **Remembers the angle the camera was last *told* to hold.**
 *
 * A `GimbalPort` decorator that observes and changes nothing, in the shape [RecordedGimbalPort]
 * already uses, and for a reason that is the whole of `vision/TagSighting`'s second rule.
 *
 * ## Why the reported angle cannot be used for this
 *
 * `KeyGimbalAttitude` is **change-driven**. Hold the camera still — which is the entire duration of
 * a nadir approach — and DJI stops delivering it. [GimbalReading]'s own KDoc records that this was
 * measured wrong twice in one day, and that six minutes of a flight record contain no attitude at
 * all while the camera was in perfect health. So a consumer asking *"is the camera pointing down
 * right now?"* off the reading gets "I do not know" precisely when the answer is "yes, and it has
 * been for a while".
 *
 * The commanded angle has the opposite property, and it is not a workaround: it is a statement about
 * what we asked for, so it has a timestamp and a value and **neither goes stale**. `MsdkGimbalAim`
 * sends `absolute = true`, so what was asked for is directly the angle, not an increment.
 *
 * `RecordedGimbalPort` already makes the same argument for putting the commanded angle in the flight
 * record; this makes it available *live*, which is what a detector deciding whether its geometry is
 * valid actually needs. The two are the same fact at two rates.
 *
 * ## What it deliberately does not claim
 *
 * **It does not claim the gimbal obeyed.** It records what was asked and when the ask was accepted,
 * and DJI accepting a rotate is not the camera having got there — the Mini 4 Pro's gimbal takes a
 * finite time to slew, and nothing here models it. A consumer using this to decide "the camera is at
 * nadir" is making a further assumption, and `vision/TagWorld.NADIR_TOLERANCE_DEG` is where that
 * assumption is stated and bounded.
 *
 * **A relative rotate clears it.** `rotateByAngle(absolute = false, …)` is an increment from
 * wherever the gimbal happens to be, and this class has no way to integrate that without the
 * reported angle it exists to avoid. Nothing in this project sends one today; if something starts
 * to, [pitchDeg] goes null rather than drifting, which is the honest failure.
 *
 * Thread-safe by being nothing but two volatile fields written from the caller's thread.
 */
class CommandedGimbalPort(private val inner: GimbalPort) : GimbalPort {

    /**
     * The last **absolute** pitch DJI accepted, degrees, or null if none has been.
     *
     * Written on DJI's success callback rather than at the ask, deliberately: a refused rotate left
     * the camera where it was, and remembering the angle we failed to reach would be exactly the
     * wrong number for a consumer that is about to do trigonometry with it.
     */
    @Volatile
    var pitchDeg: Double? = null
        private set

    /** When [pitchDeg] was accepted — `System.nanoTime` is not used; the caller's clock is. */
    @Volatile
    var acceptedAtNanos: Long? = null
        private set

    override fun unavailableReason(): String? = inner.unavailableReason()

    override fun canRotateByAngle(): Boolean = inner.canRotateByAngle()

    /** [pitchDeg] on the [GimbalPort] seam — the one override of the default; see the seam's KDoc. */
    override fun commandedPitchDeg(): Double? = pitchDeg

    override fun rotateByAngle(
        absolute: Boolean,
        pitchDeg: Double?,
        rollDeg: Double?,
        yawDeg: Double?,
        durationS: Double,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    ) {
        inner.rotateByAngle(
            absolute = absolute,
            pitchDeg = pitchDeg,
            rollDeg = rollDeg,
            yawDeg = yawDeg,
            durationS = durationS,
            onSuccess = {
                if (!absolute) {
                    // See the KDoc: an increment cannot be integrated without the reported angle,
                    // so the commanded angle becomes unknown rather than wrong.
                    this.pitchDeg = null
                    this.acceptedAtNanos = null
                } else if (pitchDeg != null && pitchDeg.isFinite()) {
                    this.pitchDeg = pitchDeg
                    this.acceptedAtNanos = nowNanos()
                }
                onSuccess()
            },
            onFailure = onFailure,
        )
    }

    override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) = inner.listenAttitude(onDelivery)

    override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) =
        inner.listenAttitudeRange(onDelivery)

    override fun listenWorkMode(onDelivery: (String?) -> Unit) = inner.listenWorkMode(onDelivery)

    override fun listenConnection(onDelivery: (Boolean?) -> Unit) = inner.listenConnection(onDelivery)

    override fun cancelListens() = inner.cancelListens()

    /**
     * The one clock. Injectable so a JVM test can hand-crank it — the same discipline
     * `record/VideoSidecar` uses, and for the same reason: a class that reads a real clock is a
     * class whose timing behaviour cannot be tested.
     */
    var nowNanos: () -> Long = { 0L }
}
