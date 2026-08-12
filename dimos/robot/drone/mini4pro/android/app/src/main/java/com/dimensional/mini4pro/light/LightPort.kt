package com.dimensional.mini4pro.light

/**
 * The aircraft's **bottom auxiliary light**, as a seam.
 *
 * The counterpart of `command/ActionPort.kt` and `gimbal/GimbalPort.kt`, written to the same rule:
 * **no DJI types, ever**, and the `KeyManager` implementation below it is deliberately too dumb to
 * test. Any method here that wants a decision has the decision in the wrong place — it belongs in
 * [LightControl], where a fake port can drive it.
 *
 * ## What this light is, and what it is not
 *
 * `FlightAssistantKey.KeyBottomAuxiliaryLightMode`, read straight out of
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` on 2026-07-27 by parsing `DJIFlightAssistantKey.<clinit>`
 * — each key is built as `new DJIKeyInfo(...).canGet(b).canSet(b).canListen(b).canPerformAction(b)
 * .setIsEvent(b)` followed by `putstatic`, so the flags are read positionally rather than guessed:
 *
 * | key | flags actually set in 5.18.0 | value type |
 * |---|---|---|
 * | `FlightAssistant.KeyBottomAuxiliaryLightMode` | `canGet`, `canSet`, `canListen` | `AuxiliaryLightMode` |
 *
 * Not an action key — `canPerformAction` is false — so this is a *setting* that is written and
 * read back, not a request that succeeds or fails at the aircraft. That difference is why
 * [setMode] reports only whether the write was accepted, and why [listenMode] exists at all: the
 * only honest answer to "is the light on?" is what the aircraft says it is.
 *
 * **It is an aid to the downward vision sensors, not a floodlight.** DJI runs it in
 * [AuxiliaryLight.AUTO] near the ground, where the landing sensors need texture. Whether it does
 * anything useful for tag detection at three metres is **unmeasured** — see the note on
 * [AuxiliaryLight.ON]. Nothing in this package should be read as a claim that it does.
 *
 * ## What is deliberately absent
 *
 * **`KeyTopAuxiliaryLightMode`.** It exists with identical flags. It points at the sky, so it
 * illuminates nothing the camera can see and its only use is being seen by other people. That is
 * a real use, and it is a different feature with a different argument; wiring it here because the
 * key happened to be adjacent would be the reason nobody could later say why it was on.
 */
interface LightPort {

    /** Why the light cannot be commanded at all, or null when it can. */
    fun unavailableReason(): String?

    /**
     * Writes `KeyBottomAuxiliaryLightMode`.
     *
     * [onFailure] carries DJI's `IDJIError` name verbatim — the string an operator will search for
     * — and neither callback is invoked more than once.
     */
    fun setMode(mode: AuxiliaryLight, onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /**
     * Subscribes to `KeyBottomAuxiliaryLightMode`. [onDelivery] fires on **every** delivery,
     * repeats included, with `null` meaning DJI's component-gone signal — the same listener
     * contract `StateCache` documents.
     */
    fun listenMode(onDelivery: (AuxiliaryLight?) -> Unit)

    /** Cancels the subscription. Idempotent, and safe when nothing was ever subscribed. */
    fun cancelListens()
}

/**
 * `AuxiliaryLightMode` with the DJI type taken off.
 *
 * The four modes DJI declares, plus the one it returns when it does not know. Deliberately a
 * *closed* set mapped at the seam rather than an integer passed through: a mode this bridge does
 * not understand must be refused at the door, not written to an aircraft to find out.
 */
enum class AuxiliaryLight {
    /** DJI decides. The factory behaviour: on near the ground, off above it. */
    AUTO,

    /**
     * Forced on.
     *
     * **Whether this helps a camera see anything is unmeasured.** The lamp is aimed straight down
     * at close range for the landing sensors' benefit, and the tag work that prompted this feature
     * measured detection in afternoon sun, not at night
     * (`docs/measurements/2026-07-27-tag-detection-rate.md`). At night the binding constraint is
     * as likely to be exposure and motion blur as illumination. Forcing it on may also interact
     * with the vision positioning system, which is the thing the lamp is actually for.
     */
    ON,

    /** Forced off. */
    OFF,

    /**
     * **Strobe, not illumination.** For being seen, not for seeing.
     *
     * Offered because DJI offers it and an operator may want it for visibility or to satisfy a
     * night-flight rule. It is the wrong mode for anything that looks at video: a strobe alternates
     * the exposure frame to frame, so a detector gets a sequence of images that disagree about how
     * bright the world is. If tag detection is the goal, this is the mode that makes it worse.
     */
    BEACON,

    /** DJI reported a mode this SDK version has no name for. Never commandable — only observed. */
    UNKNOWN,
    ;

    companion object {
        /**
         * The mode named by a MAVLink `param1`, or null when the number names none.
         *
         * [UNKNOWN] is **not** reachable here on purpose: it is something the aircraft can tell
         * us, never something we can ask for, and accepting it from the wire would turn a
         * malformed command into a write.
         */
        fun fromParam(param: Float): AuxiliaryLight? = when {
            param == 0f -> AUTO
            param == 1f -> ON
            param == 2f -> OFF
            param == 3f -> BEACON
            else -> null
        }
    }
}
