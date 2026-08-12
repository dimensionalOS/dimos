package com.dimensional.mini4pro.light

import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.Tap

/**
 * A [LightPort] that **records every mode written to the lamp**, on the way past.
 *
 * The exact shape of `command/RecordedActionPort` and `gimbal/RecordedGimbalPort`, for the reasons
 * the first of those argues: recording is a property of *reaching the wire*, so it lives in the
 * object standing between the decision and the wire. The ask is written **before** the SDK call,
 * so a `setValue` that throws still leaves the ask on the record.
 *
 * ## Why a lamp is worth recording at all
 *
 * Because the reason this feature exists is a measurement nobody has taken. The light was added to
 * find out whether it helps a camera see a tag after dark, and that question is answered by
 * comparing detections against *when the lamp was actually on* — which is only knowable if the
 * command is in the same record, on the same clock, as the frames and the altitude.
 *
 * A record that shows detection improving without showing the lamp being switched proves nothing.
 * This is the line that turns an impression into an experiment.
 *
 * [listenMode] is a pass-through with no recording: it is inbound state, and the observed mode
 * belongs in the telemetry stream rather than in the aircraft-outbound call log. The distinction
 * is the same one `RecordedGimbalPort` draws between a commanded and a reported angle.
 */
class RecordedLightPort(
    private val inner: LightPort,
    private val tap: Tap,
) : LightPort {

    override fun unavailableReason(): String? = inner.unavailableReason()

    override fun setMode(mode: AuxiliaryLight, onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.LIGHT_MODE, argsJson = "{\"mode\":\"$mode\"}")
        inner.setMode(
            mode,
            onSuccess = {
                call.accepted()
                onSuccess()
            },
            onFailure = { error ->
                call.refused(error)
                onFailure(error)
            },
        )
    }

    override fun listenMode(onDelivery: (AuxiliaryLight?) -> Unit) = inner.listenMode(onDelivery)

    override fun cancelListens() = inner.cancelListens()
}
