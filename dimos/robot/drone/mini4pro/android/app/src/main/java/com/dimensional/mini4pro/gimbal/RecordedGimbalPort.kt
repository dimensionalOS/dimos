package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.Tap

/**
 * A [GimbalPort] that **records every rotation asked of the camera**, on the way past.
 *
 * The exact shape of `command/RecordedActionPort`, for the same reasons — read that class's KDoc
 * for why the recording lives in a decorator rather than inside the DJI port or inside
 * [MsdkGimbalAim]. This one closes the last hole in that argument.
 *
 * ## What was missing, and why it was invisible
 *
 * [DjiOp.GIMBAL_ROTATE] has existed in `record/Tap.kt` since the seam was written, and **nothing
 * ever called it.** The gimbal has been aiming a real camera for weeks and no flight record
 * anywhere contains a single commanded angle — only the *reported* angle arriving later on
 * `KeyGimbalAttitude`, which is a different quantity.
 *
 * That gap is invisible from inside a flight. It surfaced when `tools/memexport` came to build
 * the `drone/base_link → drone/camera` edge of the TF tree and found it had nothing to build from:
 * `docs/mem2-converter.md` §2.2 says the edge must use the **commanded** angle, the converter
 * prefers it, and no record in this project carries one. It fell back to the reported angle held
 * forward causally — defensible on a nadir flight where the camera was aimed once and held, and
 * indefensible on any flight where the camera is being flown.
 *
 * ## Why the commanded angle is the one that belongs in a frame tree
 *
 * The reported angle is change-driven, like every DJI key. Hold the camera still — which is the
 * whole of a nadir approach — and it stops arriving, so a consumer cannot distinguish "the camera
 * is where it was" from "the gimbal feed died". This project has now hit that trap six times, and
 * the rule it settled on is in `vision/TagSighting`: **the camera's pointing is commanded and
 * known from the command, not read back.** A recorded command has a timestamp and a value and
 * neither goes stale, because it is a statement about what we asked for rather than a sample of
 * the world.
 *
 * The reported angle stays valuable and stays recorded — it is how we learn whether the gimbal
 * obeyed. The two are not substitutes.
 *
 * ## Faithfulness
 *
 * [rotateByAngle] forwards every argument untouched and invokes the caller's callbacks exactly
 * once each, exactly when DJI invokes ours, on the same thread. **The ask is recorded before the
 * SDK call**, so a `performAction` that throws still leaves the ask on the record — the ordering
 * `RecordedActionPort` establishes and the reason it matters.
 *
 * Everything else is a pass-through with no recording: [unavailableReason] and [canRotateByAngle]
 * are precondition reads rather than traffic, and the four `listen*` are inbound state the
 * recorder already samples through `GimbalAim.reading()`.
 */
class RecordedGimbalPort(
    private val inner: GimbalPort,
    private val tap: Tap,
) : GimbalPort {

    override fun unavailableReason(): String? = inner.unavailableReason()

    override fun canRotateByAngle(): Boolean = inner.canRotateByAngle()

    // Forwarded explicitly rather than left to the interface default, so the commanded record
    // survives whichever way the two decorators are ever nested — a default null here would
    // silently hide a CommandedGimbalPort wrapped inside this one.
    override fun commandedPitchDeg(): Double? = inner.commandedPitchDeg()

    override fun rotateByAngle(
        absolute: Boolean,
        pitchDeg: Double?,
        rollDeg: Double?,
        yawDeg: Double?,
        durationS: Double,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    ) {
        // Every axis, including the two that are always null on this airframe. A null is not
        // noise here: it is the #527 handling visible in the record, and the difference between
        // "we asked for no yaw" and "we asked for yaw 0" is exactly what that bug was about.
        val call = tap.aircraftOut(
            DjiOp.GIMBAL_ROTATE,
            argsJson = buildString {
                append("{\"absolute\":").append(absolute)
                append(",\"pitchDeg\":").append(jsonNumber(pitchDeg))
                append(",\"rollDeg\":").append(jsonNumber(rollDeg))
                append(",\"yawDeg\":").append(jsonNumber(yawDeg))
                append(",\"durationS\":").append(jsonNumber(durationS))
                append("}")
            },
        )
        inner.rotateByAngle(
            absolute = absolute,
            pitchDeg = pitchDeg,
            rollDeg = rollDeg,
            yawDeg = yawDeg,
            durationS = durationS,
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

    override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) =
        inner.listenAttitude(onDelivery)

    override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) =
        inner.listenAttitudeRange(onDelivery)

    override fun listenWorkMode(onDelivery: (String?) -> Unit) = inner.listenWorkMode(onDelivery)

    override fun listenConnection(onDelivery: (Boolean?) -> Unit) = inner.listenConnection(onDelivery)

    override fun cancelListens() = inner.cancelListens()

    private companion object {
        /**
         * A nullable angle as JSON: `null`, or the number.
         *
         * **A non-finite value is written `null` rather than as a bare `NaN`**, which is not JSON
         * and would make the whole record unreadable by `jq` — `record/Json`'s rule. Nothing here
         * can produce one ([MsdkGimbalAim] refuses a non-finite pitch before the port is reached)
         * but this is the layer that would serialise it if something ever did.
         */
        fun jsonNumber(v: Double?): String =
            if (v == null || !v.isFinite()) "null" else v.toString()
    }
}
