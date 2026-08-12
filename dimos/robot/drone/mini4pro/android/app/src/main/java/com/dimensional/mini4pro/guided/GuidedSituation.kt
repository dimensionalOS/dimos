package com.dimensional.mini4pro.guided

/**
 * An immutable, read-only picture of what [GuidedStickEngine] is currently flying — **a copy
 * taken under the engine's lock, not a handle on its state**.
 *
 * ## Why a copy, and why plain data
 *
 * The situation view needs to draw the circle the aircraft is on. It must not be able to
 * *change* that circle, and it must not be able to observe it half-written: the engine mutates
 * `OrbitState` from its own 10 Hz thread under a lock, and handing a `View` a reference to that
 * object would mean a paint at 60 Hz reading fields mid-update. So this type is flat immutable
 * data with no back-reference to the engine, produced by one synchronized read.
 *
 * That is the same shape as `AircraftState` and for the same reason: drawing is a **consumer**
 * of flight state and may never become a second source of it. Nothing in this file can be
 * written to, and nothing the engine reads back comes from here.
 *
 * ## What it deliberately omits
 *
 * The engine's control internals — swept angle, ramped tangential speed, arrival tick counters,
 * the release plan — are absent. They are the loop's working memory rather than facts about
 * where the aircraft is being sent, and a picture drawn from them would go stale and jittery for
 * reasons the operator cannot interpret. What is here is the *intent*: the place, the circle,
 * the point the camera is on.
 */
data class GuidedSituation(
    val phase: GuidedPhase,
    val goto: GuidedGoto? = null,
    val orbit: GuidedOrbit? = null,
    val roi: GuidedRoi? = null,
    val descent: GuidedDescent? = null,
) {
    /**
     * True when a manoeuvre of ours exists that a withdrawal would have something to act on.
     * A **shadow** descent is deliberately not one: it flies nothing, so Pause and Stop have
     * nothing of it to withdraw — shadow mode has its own off switch.
     */
    val hasManoeuvre: Boolean get() = goto != null || orbit != null || (descent != null && !descent.shadow)

    companion object {
        /** Nothing engaged, nothing accepted. What a stopped or absent engine looks like. */
        val IDLE = GuidedSituation(GuidedPhase.IDLE)
    }
}

/**
 * An accepted `DO_REPOSITION` target.
 *
 * @param arrived true once the M3 arrival test has passed and the loop is holding station —
 *   the same latch a Pause sets, so a paused goto and an arrived goto are the same picture,
 *   which is exactly what they are on the aircraft.
 */
data class GuidedGoto(
    val latDeg: Double,
    val lonDeg: Double,
    val relAltM: Double,
    val arrived: Boolean,
)

/**
 * An accepted `DO_ORBIT` circle.
 *
 * @param direction +1 clockwise, −1 anticlockwise — the sign of `OrbitCommand.radiusM`,
 *   normalised by the engine at acceptance.
 * @param circling false while flying to the join point, while coming to rest at the end, and
 *   while holding after a pause. The distinction is drawable — an aircraft heading *for* a
 *   circle is not on it — and it is the only orbit-phase fact this type carries.
 */
data class GuidedOrbit(
    val centreLatDeg: Double,
    val centreLonDeg: Double,
    val radiusM: Double,
    val direction: Int,
    val relAltM: Double,
    val circling: Boolean,
)

/**
 * An armed tag-tracked descent (M3 Stage D).
 *
 * Deliberately carries no target coordinate: the descent's target is wherever the newest fix
 * says the tag is, re-read every tick, and a coordinate snapshotted here would be a picture of a
 * moving estimate pretending to be a place. What the screen needs is *that* a descent is armed,
 * on which tag, and whether it has reached its ending — which is what the arm control renders.
 *
 * @param terminal true once the stage has completed and the aircraft is holding at the target
 *   height, waiting for the operator.
 * @param shadow true for a **shadow** run — the controller is computing everything and
 *   actuating nothing. Carried so the screen can never show a shadow as a live descent: an
 *   operator must never wonder whether the aircraft is about to move.
 * @param landing true once a full-autoland engagement has committed and handed the descent to
 *   DJI's own landing (Stage C, `TagDescentPhase.DJI_LANDING`). Never true for a plain Stage B
 *   descent or a shadow.
 * @param blind true while a committed landing's newest tag fix is older than the freshness
 *   bound — near the ground the *expected* shape (`landingdata.md`: DJI recenters the camera
 *   ~3 s before touchdown). The screen shows aligned/blind because the operator watching the
 *   last metres is owed the distinction.
 * @param approach true while the machine is still in its approach segment — armed above the
 *   7 m band, flying down into it (`TagDescentPhase.APPROACH`). Carried because the screen
 *   must not show "Descending" for a leg whose sensor weather (sparse decodes, baro height)
 *   is deliberately different from the tracking descent's; false from the band-entry handoff
 *   on, which is exactly when the face may claim the ordinary descent.
 */
data class GuidedDescent(
    val tagId: Int,
    val terminal: Boolean,
    val shadow: Boolean = false,
    val landing: Boolean = false,
    val blind: Boolean = false,
    val approach: Boolean = false,
)

/**
 * A `DO_SET_ROI` point.
 *
 * @param tracking false while the ROI is *remembered but suspended* — the state an abort leaves
 *   it in, where the camera stays exactly where it was pointing rather than being recentred
 *   (`GuidedStickEngine.abort`, §9.5). Drawing that as an active camera line would claim the
 *   camera is on a target it has drifted away from, so the two are separate facts here.
 */
data class GuidedRoi(
    val latDeg: Double,
    val lonDeg: Double,
    val tracking: Boolean,
)
