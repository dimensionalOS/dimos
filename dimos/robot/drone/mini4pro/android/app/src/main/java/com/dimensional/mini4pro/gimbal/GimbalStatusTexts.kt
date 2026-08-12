package com.dimensional.mini4pro.gimbal

import com.dimensional.mini4pro.command.StatusTexts

/**
 * The operator-facing sentences the gimbal produces, inside the one channel that exists for them.
 *
 * `STATUSTEXT.text` is **50 bytes, hard** — a fixed-width `char[50]`, silently cut on the wire —
 * and only `EMERGENCY`/`ALERT`/`CRITICAL`/`ERROR` are surfaced to an operator at all
 * (`StatusTextHandler.cc:18-24`, measured by this project on 2026-07-26). Both facts are
 * `command/StatusTexts`' already, so the byte-level clamp is **reused from there** rather than
 * re-derived: [StatusTexts.clamp] is the tricky part (a UTF-8 cut that never splits a character or
 * strands a surrogate) and there must be exactly one of it.
 *
 * What is *not* reused is the wording, because `command/StatusTexts` composes around
 * `FlightAction`, whose two members are Return and Land. Aiming a camera is neither, and the
 * project's naming discipline is the point of that type rather than an accident of it.
 *
 * ## The shortening rule, unchanged
 *
 * **DJI's word survives; ours does not.** When the framed sentence will not fit, the framing is
 * dropped and the bare DJI error goes out alone. That matters more here than it did in M2: the
 * error name this package expects to see most is
 * `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW`, which is **39 bytes on its own** and leaves room for
 * no framing whatsoever. An operator can search DJI's forums for that string; they cannot search
 * for our paraphrase of it.
 *
 * ## What is deliberately *not* announced
 *
 * **A successful aim.** M2 announces `"Return sent to DJI"` because, measured on 2026-07-26, all
 * three of its feedback channels can stay silent together and the operator cannot tell a sent
 * command from a broken bridge. The gimbal has no such gap: `GIMBAL_DEVICE_ATTITUDE_STATUS` is
 * streamed at 5 Hz from `KeyGimbalAttitude`, so an operator who aims the camera watches the
 * reported angle move — continuous, honest, and from the aircraft rather than from us. Adding a
 * red `STATUSTEXT` per aim on top of that, at up to 5 Hz during a drag, would be the alarm fatigue
 * that `HandshakeResponder.announceModeRefusal` argues against, in exchange for information the
 * operator already has.
 *
 * **A clamp to the gimbal's travel limits.** Same reason, plus [MsdkGimbalAim.clampedTarget]'s:
 * nothing was refused and the limit is visible in the attitude readout.
 */
object GimbalStatusTexts {

    /**
     * How the gimbal is named to the operator. Short on purpose — DJI's error name has first claim
     * on the 50 bytes, and its gimbal names are the longest this project has seen.
     */
    const val LABEL = "Gimbal"

    /**
     * What the operator reads when a QGroundControl command carried a yaw this airframe cannot
     * turn. 42 bytes, counted.
     *
     * It fires more often than "the operator asked for a yaw" suggests, and that is why it is
     * worded as a statement about the aircraft rather than about their input: QGC's **Center** and
     * **Tilt 90** buttons both hard-code `yaw = 0` into
     * `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` (`GimbalController.cc:424-431`,
     * `src/Toolbar/GimbalIndicator.qml:188`), so pressing either produces this line once even
     * though the operator only meant "point down". Telling them the airframe has no SDK yaw axis
     * is true in both cases and useful in both.
     *
     * There is no route by which QGC learns this on its own: our `GIMBAL_MANAGER_INFORMATION`
     * omits `HAS_YAW_AXIS`, but QGC reads that bit **nowhere** (`GimbalController.cc:114` is the
     * only read of `cap_flags`, and only `HAS_RETRACT` and `HAS_YAW_LOCK` are consulted). So this
     * sentence is the entire channel, and without it the yaw half of every Center press would
     * disappear in silence.
     */
    const val YAW_UNAVAILABLE = "Gimbal: pitch only, no yaw on this airframe"

    /**
     * What the operator reads when QGroundControl asks for a gimbal **rate** rather than an angle.
     * 42 bytes, counted.
     *
     * QGC emits rates from joystick buttons only, as raw `GIMBAL_MANAGER_SET_ATTITUDE` (282) with
     * an all-NaN quaternion and `angular_velocity_y`/`z` in rad/s, repeated at 2 Hz while the
     * button is held (`GimbalController.cc:594-639`). `KeyRotateBySpeed` exists and would serve —
     * but a rate is a control that runs until something stops it, and this project has measured
     * DJI swallowing a `performAction` with neither callback ever firing. A swallowed *stop*
     * leaves a camera slewing to its limit. See [GimbalPort]'s KDoc.
     *
     * So the rate is refused, and refused **out loud**: a joystick button that does nothing and
     * says nothing is precisely the silent no-op this layer exists to prevent.
     */
    const val RATE_UNSUPPORTED = "Gimbal rate control unsupported; use angles"

    /** "Gimbal refused by DJI: SDK_SERVICE_..." — the gimbal was asked and said no. */
    fun refusal(djiError: String): String = preferring("$LABEL refused by DJI: $djiError", djiError)

    /**
     * "Gimbal failed: NO_PRODUCT" — we could not put the question to the aircraft at all.
     *
     * A different verb from [refusal] on purpose, and the same distinction `command/StatusTexts`
     * draws: "refused" attributes a decision to the aircraft, and if nothing was reachable there
     * was no decision to attribute.
     */
    fun unavailable(reason: String): String = preferring("$LABEL failed: $reason", reason)

    /** "Gimbal failed: IllegalStateException" — the [GimbalAim] implementation threw. */
    fun threw(throwable: Throwable): String {
        val detail = throwable.message?.takeIf { it.isNotBlank() }
            ?: throwable::class.java.simpleName
        return preferring("$LABEL failed: $detail", detail)
    }

    /**
     * An asynchronous DJI error arriving from a rotation's `onFailure`, long after the command was
     * acknowledged. The prefix marks whose word it is; the name itself is untouched.
     */
    fun djiError(djiError: String): String = preferring("DJI: $djiError", djiError)

    /** The long form if it fits, otherwise the essential part, otherwise the essential part cut. */
    private fun preferring(full: String, essential: String): String =
        if (full.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES) {
            full
        } else {
            StatusTexts.clamp(essential)
        }
}
