package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.Px4Mode
import kotlin.math.abs

/**
 * **Detects DJI's landing gimbal recenter and turns it into one record event** —
 * [EventCode.GIMBAL_RECENTERED] — instead of leaving it to be excavated from pitch samples.
 *
 * The behaviour it names is measured, not documented (`landingdata.md`): on entering a landing
 * mode this airframe's firmware slews the gimbal from wherever it is to level in ~0.32–0.36 s at
 * ≈250–300 °/s, and the pitch samples arrive *before* the mode-change key by 40–120 ms. Four
 * landings, four sessions, identical shape. Nothing in the SDK can switch it off (§3 of the doc,
 * checked exhaustively), so the honest thing a recorder can do is say it happened, when, and
 * from what angle.
 *
 * Decision-free by design — `landingdata.md` §5's "safe, useful and decision-free" change: this
 * class commands nothing, gates nothing, and is consulted by nothing but the recorder's gimbal
 * sampler. Pure JVM, no clock of its own, so `GimbalRecenterTest` hand-cranks it.
 *
 * The mode conjunct uses [Px4Mode.DJI_LANDING_MODES] — the one list of landing-mode names — and
 * tolerates the measured delivery skew: because the slew's samples can arrive *before* the mode
 * change, the detector triggers on the movement and lets the mode be satisfied by **either** end
 * of the window (the mode at the reference sample or at the current one). A slew that completes
 * entirely before the mode arrives is still caught on its later samples: the reference only
 * advances when the window expires, and the mode has arrived by then on every measured landing.
 */
class GimbalRecenter {

    companion object {
        /**
         * How far the pitch must move inside [WINDOW_MS] to count as a recenter, degrees.
         * 30° is an order of magnitude above the 0.5° sampling deadband and any measured
         * stabilisation jitter, and under half of the −90 → 0 slew, so the detector fires
         * mid-slew on every measured landing while no operator drag at QGC's 10 Hz closed-loop
         * rate (a few degrees per tick) can reach it.
         */
        const val MOVE_DEG = 30.0

        /** The slew is ~0.35 s end to end (measured, all four landings); 500 ms covers it. */
        const val WINDOW_MS = 500L

        /**
         * One event per recenter: after firing, the detector stays quiet until the pitch has
         * been still (no [MOVE_DEG] excursion) for a full window. DJI holds level through
         * touchdown, so in practice this is one line per landing.
         */
        private const val ARMED = 0
        private const val FIRED = 1
    }

    private var state = ARMED
    private var refTimeMs: Long? = null
    private var refPitchDeg = 0.0
    private var refModeLanding = false

    /**
     * One gimbal sample. Returns the event message to record ("pitch a -> b in n ms, mode m"),
     * or null. [pitchDeg] null (no reading) resets the reference — a gap is not a movement.
     */
    fun sample(tMs: Long, pitchDeg: Double?, flightMode: String?): String? {
        if (pitchDeg == null) {
            refTimeMs = null
            return null
        }
        val landingNow = Px4Mode.isDjiLandingMode(flightMode)
        val refAt = refTimeMs
        if (refAt == null || tMs - refAt > WINDOW_MS) {
            // The window expired without a trigger: this sample becomes the new reference, and
            // a fired detector that has been still for the whole window re-arms.
            if (state == FIRED && refAt != null && abs(pitchDeg - refPitchDeg) < MOVE_DEG) {
                state = ARMED
            }
            refTimeMs = tMs
            refPitchDeg = pitchDeg
            refModeLanding = landingNow
            return null
        }
        val moved = abs(pitchDeg - refPitchDeg)
        if (state == ARMED && moved > MOVE_DEG && (landingNow || refModeLanding)) {
            state = FIRED
            val message = "pitch %.1f -> %.1f in %dms, mode %s"
                .format(refPitchDeg, pitchDeg, tMs - refAt, flightMode ?: "?")
            refTimeMs = tMs
            refPitchDeg = pitchDeg
            refModeLanding = landingNow
            return message
        }
        return null
    }
}
