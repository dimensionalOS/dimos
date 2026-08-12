package com.dimensional.mini4pro.gimbal

import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.sdk.keyvalue.value.gimbal.GimbalAttitudeRange
import dji.sdk.keyvalue.value.gimbal.GimbalMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager

/**
 * The production [GimbalPort]: the only class in this package that touches the MSDK.
 *
 * The third and last class in the project allowed to hold a `KeyManager` — after `StateCache`
 * (reading aircraft telemetry) and `KeyManagerActionPort` (performing flight actions) — and it is
 * justified the same way the second one was. It is deliberately **decision-free**: one method per
 * key, values unwrapped and nothing else, so everything worth testing lives in [MsdkGimbalAim]
 * where a fake port can drive it. It is verified the way `StateCache` is: on hardware, not by
 * tests, and it is kept this thin precisely so that verification is a short walk.
 *
 * Every key is `canPerformAction`/`canGet`+`canListen` as claimed, read on 2026-07-26 out of
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` by parsing `DJIGimbalKey.<clinit>` positionally — the
 * table is in [GimbalPort]'s KDoc, along with the reason a `<clinit>` is sound evidence where an
 * MSDK method body is not.
 *
 * ## Two things here that are easy to get wrong
 *
 * **The component index is explicit.** `StateCache` and `KeyManagerActionPort` both call
 * `KeyTools.createKey(info)`, whose index defaults to 0 — harmless for flight-controller keys,
 * which are not indexed. Gimbals *are* indexed components, so the index is spelled out as
 * [ComponentIndexType.LEFT_OR_MAIN]. On a single-gimbal airframe that is the same 0, and writing
 * it makes the difference visible rather than accidental.
 *
 * **A null axis becomes an `*Ignored` flag, not an unset field.** See [rotateByAngle].
 */
class KeyManagerGimbalPort : GimbalPort {

    /** One holder for every subscription, so teardown is a single cancelListen — as StateCache. */
    private val holder = Any()

    /**
     * Read fresh on every call rather than cached from a connect callback, exactly as
     * `KeyManagerActionPort` does: `Msdk.state` is the lifecycle's single source of truth, and a
     * per-call read means a product disconnect refuses the very next command with no listener
     * plumbing to go stale in between.
     */
    override fun unavailableReason(): String? {
        val s = com.dimensional.mini4pro.Msdk.state.value
        return when {
            !s.registered -> "SDK_NOT_REGISTERED"
            !s.productConnected -> "NO_PRODUCT"
            else -> null
        }
    }

    override fun canRotateByAngle(): Boolean =
        KeyTools.createKey(GimbalKey.KeyRotateByAngle, GIMBAL_INDEX).canPerformAction()

    /**
     * `performAction(Gimbal.KeyRotateByAngle, GimbalAngleRotation)`.
     *
     * The `*Ignored` booleans are set from whether the corresponding angle is null, and that is
     * the *entire* mechanism protecting this airframe from
     * [#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527). It has to be done here
     * because "unset" does not survive serialisation: `GimbalAngleRotation.toBytes` pushes every
     * field through `ByteStreamHelper.doubleToBytes(byte[], Double, int)`, whose first act is to
     * substitute `0.0` for a null — and `0.0` on the yaw or roll axis is exactly what makes a
     * Mini 4 Pro answer `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW`. So the value written for an
     * ignored axis is irrelevant and the flag is everything.
     *
     * That reading is of the **real** body, offsets 2-10, not of the `iconst_0; ireturn` stub the
     * provided jar prepends to every method — see [GimbalAim]'s KDoc for why the distinction is
     * load-bearing when reading this SDK.
     *
     * `jointReferenceUsed` and `timeout` are left unset. Neither is documented for this key and
     * neither has been measured; a value invented for them would be a claim about DJI's behaviour
     * that nobody here can support.
     *
     * Built with setters rather than the ten-argument constructor on purpose. That constructor is
     * `(mode, pitch, roll, yaw, pitchIgnored, rollIgnored, yawIgnored, duration, jointReferenceUsed,
     * timeout)` — three consecutive `Double?` followed by three consecutive `Boolean?` — and a
     * transposition of roll and yaw in it would compile, run, and point the camera somewhere
     * unintended.
     */
    override fun rotateByAngle(
        absolute: Boolean,
        pitchDeg: Double?,
        rollDeg: Double?,
        yawDeg: Double?,
        durationS: Double,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    ) {
        val rotation = GimbalAngleRotation().apply {
            mode = if (absolute) {
                GimbalAngleRotationMode.ABSOLUTE_ANGLE
            } else {
                GimbalAngleRotationMode.RELATIVE_ANGLE
            }
            pitch = pitchDeg
            roll = rollDeg
            yaw = yawDeg
            pitchIgnored = pitchDeg == null
            rollIgnored = rollDeg == null
            yawIgnored = yawDeg == null
            duration = durationS
        }
        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateByAngle, GIMBAL_INDEX),
            rotation,
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) = onSuccess()
                override fun onFailure(error: IDJIError) = onFailure(errorName(error))
            },
        )
    }

    override fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit) =
        listen(GimbalKey.KeyGimbalAttitude) { attitude: Attitude? ->
            onDelivery(
                attitude?.let { GimbalAngles(pitchDeg = it.pitch, rollDeg = it.roll, yawDeg = it.yaw) }
            )
        }

    override fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit) =
        listen(GimbalKey.KeyGimbalAttitudeRange) { range: GimbalAttitudeRange? ->
            onDelivery(
                range?.let {
                    GimbalLimits(
                        pitchMinDeg = it.pitch?.min,
                        pitchMaxDeg = it.pitch?.max,
                        rollMinDeg = it.roll?.min,
                        rollMaxDeg = it.roll?.max,
                        yawMinDeg = it.yaw?.min,
                        yawMaxDeg = it.yaw?.max,
                    )
                }
            )
        }

    /**
     * `KeyGimbalCMode` — DJI's own inner identifier for it is `GimbalMode`, and the value is one of
     * `FREE`, `FPV`, `YAW_FOLLOW`, `UNKNOWN`.
     *
     * The **name** crosses the seam, not the enum, so the decision about what `FREE` means for
     * MAVLink's `GIMBAL_DEVICE_FLAGS_YAW_LOCK` lives in [GimbalEncoder.isYawLocked] where a test
     * can reach it. Same idiom as `AircraftState.flightMode`.
     */
    override fun listenWorkMode(onDelivery: (String?) -> Unit) =
        listen(GimbalKey.KeyGimbalCMode) { mode: GimbalMode? -> onDelivery(mode?.name) }

    override fun listenConnection(onDelivery: (Boolean?) -> Unit) =
        listen(GimbalKey.KeyConnection) { connected: Boolean? -> onDelivery(connected) }

    override fun cancelListens() = KeyManager.getInstance().cancelListen(holder)

    /**
     * `getOnce = true`, the house rule for every subscription in this project: it primes the
     * listener with the current value, so a subscription made after the gimbal has already settled
     * still learns where it is pointing instead of waiting for it to move.
     */
    private fun <T> listen(info: DJIKeyInfo<T>, deliver: (T?) -> Unit) {
        KeyManager.getInstance().listen(
            KeyTools.createKey(info, GIMBAL_INDEX),
            holder,
            true,
        ) { _, newValue ->
            // Do no work in the listener beyond handing the value up; the lock and every decision
            // live in MsdkGimbalAim.
            deliver(newValue)
        }
    }

    /**
     * The string an operator will search DJI's forums for. `errorCode()` is the tier the flight
     * recorder caught live for the flight controller (`FC_AUTH_STATE` and friends) and is the tier
     * #527's reporter quotes for the gimbal (`SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW`). When DJI
     * leaves it blank the whole error's `toString` goes instead, because a blank is dropped
     * upstream and an unnamed failure the operator never hears about is worse than an ugly one.
     */
    private fun errorName(error: IDJIError): String =
        error.errorCode()?.takeIf { it.isNotBlank() } ?: error.toString()

    private companion object {
        /**
         * The Mini 4 Pro has one gimbal, and it is the main one. Written out rather than left to
         * `createKey`'s default because gimbal keys are component-indexed and the default being
         * correct here is a coincidence of there being only one.
         */
        val GIMBAL_INDEX: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN
    }
}
