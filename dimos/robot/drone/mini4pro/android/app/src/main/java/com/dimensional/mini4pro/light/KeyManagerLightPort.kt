package com.dimensional.mini4pro.light

import dji.sdk.keyvalue.key.FlightAssistantKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.flightassistant.AuxiliaryLightMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager

/**
 * The production [LightPort]: the only class in this package that touches the MSDK.
 *
 * The fourth class in the project allowed to hold a `KeyManager`, after `StateCache`,
 * `KeyManagerActionPort` and `KeyManagerGimbalPort`, and justified the same way: it is
 * **decision-free**. One key, values mapped across the seam and nothing else, so everything worth
 * testing lives in [LightControl] where a fake port can drive it. Verified on hardware rather than
 * by tests, and kept this thin precisely so that verification is a short walk.
 *
 * `FlightAssistant.KeyBottomAuxiliaryLightMode` is `canGet`+`canSet`+`canListen` and **not** an
 * action key, read on 2026-07-27 out of `dji-sdk-v5-aircraft-provided-5.18.0.jar` by parsing
 * `DJIFlightAssistantKey.<clinit>` positionally. The table and the reason a `<clinit>` is sound
 * evidence where an MSDK method body is not are in [LightPort]'s KDoc.
 *
 * No component index is spelled out here, unlike the gimbal port: the flight assistant is not an
 * indexed component, so `KeyTools.createKey(info)`'s default is the right one — the same call
 * `StateCache` and `KeyManagerActionPort` make.
 */
class KeyManagerLightPort : LightPort {

    /** One holder for the subscription, so teardown is a single cancelListen — as StateCache. */
    private val holder = Any()

    /**
     * Read fresh on every call rather than cached from a connect callback, exactly as the action
     * and gimbal ports do: `Msdk.state` is the lifecycle's single source of truth, and a per-call
     * read means a product disconnect refuses the very next command with no listener plumbing to
     * go stale in between.
     */
    override fun unavailableReason(): String? {
        val s = com.dimensional.mini4pro.Msdk.state.value
        return when {
            !s.registered -> "SDK_NOT_REGISTERED"
            !s.productConnected -> "NO_PRODUCT"
            else -> null
        }
    }

    /**
     * `setValue(FlightAssistant.KeyBottomAuxiliaryLightMode, AuxiliaryLightMode)`.
     *
     * [AuxiliaryLight.UNKNOWN] cannot arrive here — [LightControl] refuses it above this seam and
     * `AuxiliaryLight.fromParam` never produces it — but the mapping is total because a `when`
     * that is not exhaustive over the enum would compile with an `else` and silently write
     * something. `UNKNOWN` maps to DJI's own `UNKNOWN`, which is the value the aircraft would
     * reject rather than one it would act on.
     */
    override fun setMode(mode: AuxiliaryLight, onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        KeyManager.getInstance().setValue(
            KeyTools.createKey(FlightAssistantKey.KeyBottomAuxiliaryLightMode),
            djiMode(mode),
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() = onSuccess()
                override fun onFailure(error: IDJIError) = onFailure(errorName(error))
            },
        )
    }

    /**
     * `getOnce = true`, the house rule for every subscription in this project: it primes the
     * listener with the current value, so a subscription made after the light has already settled
     * reports where it is rather than waiting for it to change. That matters more here than
     * anywhere else, because a light that is not changing is the normal case.
     */
    override fun listenMode(onDelivery: (AuxiliaryLight?) -> Unit) {
        KeyManager.getInstance().listen(
            KeyTools.createKey(FlightAssistantKey.KeyBottomAuxiliaryLightMode),
            holder,
            true,
        ) { _, newValue ->
            // Do no work in the listener beyond handing the value up; every decision is above.
            onDelivery(newValue?.let(::ourMode))
        }
    }

    override fun cancelListens() = KeyManager.getInstance().cancelListen(holder)

    private fun djiMode(mode: AuxiliaryLight): AuxiliaryLightMode = when (mode) {
        AuxiliaryLight.AUTO -> AuxiliaryLightMode.AUTO
        AuxiliaryLight.ON -> AuxiliaryLightMode.ON
        AuxiliaryLight.OFF -> AuxiliaryLightMode.OFF
        AuxiliaryLight.BEACON -> AuxiliaryLightMode.BEACON
        AuxiliaryLight.UNKNOWN -> AuxiliaryLightMode.UNKNOWN
    }

    /**
     * DJI's mode into ours. A value this SDK version has no name for becomes
     * [AuxiliaryLight.UNKNOWN] rather than being guessed at — the `when` is over DJI's enum and
     * has no `else`, so a mode added in a future MSDK fails the build here instead of silently
     * reading as `OFF`.
     */
    private fun ourMode(mode: AuxiliaryLightMode): AuxiliaryLight = when (mode) {
        AuxiliaryLightMode.AUTO -> AuxiliaryLight.AUTO
        AuxiliaryLightMode.ON -> AuxiliaryLight.ON
        AuxiliaryLightMode.OFF -> AuxiliaryLight.OFF
        AuxiliaryLightMode.BEACON -> AuxiliaryLight.BEACON
        AuxiliaryLightMode.UNKNOWN -> AuxiliaryLight.UNKNOWN
    }

    /**
     * The string an operator will search DJI's forums for — `errorCode()`, the tier the flight
     * recorder caught live for the flight controller, falling back to the whole error's `toString`
     * when DJI leaves it blank, because a blank is dropped upstream and an unnamed failure the
     * operator never hears about is worse than an ugly one.
     */
    private fun errorName(error: IDJIError): String =
        error.errorCode()?.takeIf { it.isNotBlank() } ?: error.toString()
}
