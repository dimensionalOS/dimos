package com.dimensional.mini4pro.guided

/**
 * **Phase one of a mission's `NAV_TAKEOFF`: DJI's own takeoff, and nothing else.**
 *
 * One method, no parameters, no acknowledgement — because that is genuinely the whole of what the
 * aircraft offers. `FC.KeyStartTakeoff` is `EmptyMsg → EmptyMsg` (read from the jar's `<clinit>`)
 * and the aircraft climbs to its own ~1.2 m, so there is no altitude to pass, nothing to invert
 * against a datum, and no completion to wait on except `isFlying` becoming true.
 *
 * Deliberately **not** `FlightActions` itself: this engine has no business reaching the Return and
 * Land vocabulary, and an interface with one method is a seam a test can satisfy without a DJI
 * key-manager. Same reason [ManoeuvreGimbal] exists rather than the gimbal manager.
 *
 * ## What is *not* here, and where it goes
 *
 * The **commanded climb** to the plan's requested altitude — phase two of the two-phase takeoff
 * being built in parallel — is not on this interface and must not be added to it as a second call.
 * The climb is a *leg*, flown by the same law under the same ceiling gate and the same abort ladder
 * as every other leg, so it belongs inside the tick rather than behind a seam. Its attachment point
 * is named in `GuidedStickEngine.tickMissionTakeoffLocked`.
 */
fun interface MissionTakeoff {

    /**
     * Ask DJI to take off. Fire and forget in the same sense every other DJI call in this project
     * is: [onFailure] carries DJI's own error string verbatim when it refuses, and a call it accepts
     * and silently drops is covered by the caller's own timeout rather than by waiting here.
     *
     * A takeoff DJI refuses fails the mission outright — nothing is flown, and the operator gets
     * DJI's own error rather than our paraphrase of it.
     */
    fun startTakeoff(onFailure: (String) -> Unit)
}
