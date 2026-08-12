package com.dimensional.mini4pro.guided

/**
 * The whole of what a bridge-owned manoeuvre asks of the camera: **an absolute pitch, open loop.**
 *
 * Two callers, one seam, deliberately: the orbit holding the circle's centre in frame, and the
 * region of interest holding an operator's map click. They are the *same* pointing solution
 * — `−atan2(relativeAltitude, horizontalDistance)` against a target assumed to sit at the takeoff
 * datum's ground level — computed by [OrbitGuidance.gimbalPitchDeg] for both, so there is one
 * arithmetic, one rate limiter, one deadband and one route to DJI. It was named `OrbitGimbal` while
 * the orbit was the only caller; an interface that also aims the camera during a goto, or while the
 * RC pilot is flying, could not honestly keep that name.
 *
 * A seam in `guided/` rather than a direct call into `gimbal/`, for the reason every seam in this
 * project exists — the MSDK is `compileOnly` and off the unit-test classpath, so a decision that
 * reaches DJI cannot be tested unless something DJI-free sits in front of it. `Bridge` wires this
 * to `GimbalManager`, which already owns the DJI half; nothing here rewrites aiming, which works on
 * hardware.
 *
 * ## Open-loop, and there is deliberately no way to be otherwise
 *
 * **There is no attitude, no age, and no feedback of any kind in this interface, and that absence
 * is the safety property.** `KeyGimbalAttitude` is change-driven: a stabilised, motionless gimbal
 * stops delivering altogether — six minutes of silence in the record while the camera was in
 * perfect health — and DJI answers a direct `getValue` on it with `REQUEST_HANDLER_NOT_FOUND`. So
 * "the gimbal has not reported recently" cannot be distinguished from "the gimbal has not moved",
 * and any gate or timeout built on that age fails in exactly the situation an orbit produces: a
 * steady circle, where the solution is constant and the camera correctly sits still. An ROI held on
 * a hovering aircraft is the same silence for the same reason.
 *
 * That trap has bitten this project **three times** (the gimbal advertisement, the keep-fresh `get`,
 * and the relative-altitude hover deadlock), and the rule against it is binding. Making the
 * interface incapable of expressing an age is how the rule is enforced rather than remembered:
 * `GuidedOrbitTest` and `GuidedRoiTest` both assert that this type declares nothing an attitude age
 * could arrive through.
 *
 * The consequence, stated plainly: we command an angle and never learn whether the gimbal reached
 * it. The honest confirmation channel is the one that already exists and is nothing to do with the
 * manoeuvre — `GIMBAL_DEVICE_ATTITUDE_STATUS`, which QGC is already drawing five times a second.
 */
interface ManoeuvreGimbal {

    /**
     * DJI's own reported pitch travel, `pitchMin..pitchMax` in degrees, or **null when DJI has not
     * said**. Never invented: with no reported range nothing is clamped, and a request the gimbal
     * cannot honour comes back as DJI's own refusal, which is more informative than our guess at
     * the envelope.
     *
     * A *fact about the airframe*, not a measurement of where the camera is pointing — which is why
     * it may live in an interface that is otherwise feedback-free. On this airframe it has been
     * measured at −90°..+60°.
     */
    fun pitchRangeDeg(): ClosedFloatingPointRange<Double>?

    /**
     * Point the camera at [pitchDeg], absolute, negative down. Fire and forget: implementations
     * must not block, must not throw (the engine catches anyway, on the tick thread), and must
     * never be waited on.
     *
     * Called at most once per [OrbitGuidance.GIMBAL_MIN_INTERVAL_MS] and only when the solution has
     * moved by [OrbitGuidance.GIMBAL_DEADBAND_DEG].
     */
    fun aimPitch(pitchDeg: Double)
}
