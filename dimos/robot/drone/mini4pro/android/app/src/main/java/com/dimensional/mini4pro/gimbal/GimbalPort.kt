package com.dimensional.mini4pro.gimbal

/**
 * The thin seam between [MsdkGimbalAim] — every decision the DJI half of the gimbal makes — and
 * the `KeyManager` calls those decisions ride on.
 *
 * The counterpart of `command/ActionPort.kt`, and written to the same rule: **no DJI types, ever**,
 * and [KeyManagerGimbalPort] below it is *deliberately too dumb to test* — one method per key, no
 * branch that is not a mechanical unwrap. If a method here ever wants a decision, the decision
 * belongs in [MsdkGimbalAim] where a fake port can drive it.
 *
 * Every key below was read straight out of `dji-sdk-v5-aircraft-provided-5.18.0.jar` on
 * 2026-07-26 by parsing `DJIGimbalKey.<clinit>` — each key is built as
 * `new DJIKeyInfo(...).canGet(b).canSet(b).canListen(b).canPerformAction(b).setIsEvent(b)`
 * followed by `putstatic`, so the flags are read positionally rather than guessed. This is the
 * same method `docs/msdk/actions.md` used for the flight-controller keys, applied to the 151-key
 * gimbal class.
 *
 * That evidence is **the trustworthy kind**, and the distinction matters because the provided jar
 * is a compile-only ABI stub whose every *method body* has a `return <default>` prepended at
 * offsets 0-1 with the real implementation stranded behind it as dead code. A `<clinit>` builder
 * chain is not a method body being stubbed — it is the data that populates the key objects — and
 * for `KeyRotateByAngle` it agrees exactly with DJI's own offline HTML reference. Where this
 * package cites an MSDK *method body* instead, it says so and quotes from offset 2 onward.
 *
 * | method | key | flags actually set in 5.18.0 |
 * |---|---|---|
 * | [rotateByAngle] | `Gimbal.KeyRotateByAngle` | `canPerformAction` |
 * | [canRotateByAngle] | the same key | `DJIKey.canPerformAction()` |
 * | [listenAttitude] | `Gimbal.KeyGimbalAttitude` | `canGet`, `canListen` |
 * | [listenAttitudeRange] | `Gimbal.KeyGimbalAttitudeRange` | `canGet`, `canListen` |
 * | [listenWorkMode] | `Gimbal.KeyGimbalCMode` | `canGet`, `canSet`, `canListen` |
 * | [listenConnection] | `Gimbal.KeyConnection` | `canGet`, `canListen` |
 *
 * `KeyRotateByAngle` takes `DJIActionKeyInfo<GimbalAngleRotation, EmptyMsg>` and
 * `KeyGimbalAttitude` yields `dji.sdk.keyvalue.value.common.Attitude` — the same value type the
 * *aircraft* attitude uses, which is why the unwrap has to happen at the seam rather than being
 * passed around.
 *
 * ## What is deliberately absent
 *
 * - **`KeyRotateBySpeed`.** It exists (`canPerformAction`, `GimbalSpeedRotation`, units 0.1°/s)
 *   and QGroundControl does emit rate commands for joystick buttons. It is not wired, because a
 *   rate command is a control that keeps running until a second command stops it, and the one
 *   thing this project has measured about DJI is that a `performAction` can be swallowed without
 *   either callback ever firing (`docs/measurements/2026-07-26-m2-first-command.md`). A swallowed
 *   *stop* leaves a gimbal slewing to its limit. Angles are idempotent; rates are not.
 * - **`KeyGimbalReset`.** `GimbalResetType.RECENTER` would make QGC's Center button
 *   (`pitch = 0, yaw = 0`) genuinely centre both axes rather than only pitch. It is left out of
 *   the first cut on purpose: it is a *different DJI action* silently substituted for the one the
 *   protocol named, and this project's rule about substituting a nearby action for the requested
 *   one is the emergency-stop paragraph in `command/FlightActions.kt`. Worth revisiting with a
 *   deliberate decision; not worth doing quietly. See `docs/gimbal.md`.
 * - **`KeyGimbalCMode` as a setter.** It can be set (`canSet`), so we could switch the gimbal
 *   between FREE and YAW_FOLLOW. We only read it, because QGC's Yaw-Lock button is hidden by our
 *   capability flags and nothing else asks.
 * - **A `get` on `KeyGimbalAttitude`, which this interface briefly had and which the aircraft
 *   refused.** The key is `canGet`, so on 2026-07-26 a keep-fresh poll was built on
 *   `getValue(key, callback)` — the device read — to cover the key being change-driven. Measured
 *   the same evening on the aircraft: **every one of those gets came back
 *   `REQUEST_HANDLER_NOT_FOUND`** (`docs/measurements/2026-07-26-gimbal-keep-fresh-get.md`), while
 *   the listener on the same key was delivering normally. `DJICoreError.REQUEST_HANDLER_NOT_FOUND`
 *   is a core-layer error whose own hint string is *"not support"*, so **`canGet` describes the
 *   SDK's client-side key table, not what the device will serve.** Nothing here may treat a
 *   metadata flag as a capability again without a measurement behind it.
 *
 *   The cache-read overloads are no substitute: `JNIKeyValue.native_get_sync` returns bytes and no
 *   timestamp, so a cached value cannot be told from a six-minute-old one, and stamping it as
 *   fresh would invent the very liveness the reading is supposed to prove. It is also redundant —
 *   `getOnce = true` on [listenAttitude] already performs the device get on subscribe and already
 *   falls back to that cache read when it fails (`PushRecorder$1.onFailed`, 5.18.0 bytecode read
 *   from offset 2), delivering whatever it finds as an ordinary push. DJI does the one get worth
 *   doing, at the only moment it could help. See `GimbalReading.isAdvertisable` for what replaced
 *   the poll.
 */
interface GimbalPort {

    /**
     * Why the MSDK cannot be asked anything right now, or null if it can.
     *
     * The two production reasons are `SDK_NOT_REGISTERED` and `NO_PRODUCT`; both are statements
     * about *us*, which is why they surface as `ActionOutcome.Unavailable` rather than
     * `Refused` — no gimbal was consulted. Read fresh on every call, exactly as
     * `KeyManagerActionPort` does, so a product disconnect refuses the very next command.
     */
    fun unavailableReason(): String?

    /**
     * Whether `Gimbal.KeyRotateByAngle` declares itself performable — `DJIKey.canPerformAction()`.
     *
     * **Read `command/ActionPort.canStartGoHome`'s KDoc before trusting this.** It is the same
     * flag with the same limits: a constructor-chain boolean baked into the key at class-load
     * time, `true` here on 5.18.0, and **not** a pre-flight check. MSDK 5.18.0 offers no
     * synchronous way to ask whether a gimbal rotation will be honoured now — the only structured
     * refusal channel is `CompletionCallback.onFailure`, asynchronously.
     *
     * It is asked anyway and earns its place the same narrow way: it fails closed with a name if
     * an SDK upgrade ever demotes the key or someone wires a non-action key here.
     */
    fun canRotateByAngle(): Boolean

    /**
     * `performAction(Gimbal.KeyRotateByAngle)`.
     *
     * **A null axis is an axis this bridge is not commanding**, and the implementation must
     * express that with DJI's `pitchIgnored`/`rollIgnored`/`yawIgnored` booleans — *not* by
     * leaving the value unset. Leaving it unset does not work: `GimbalAngleRotation.toBytes`
     * serialises the field through `ByteStreamHelper.doubleToBytes`, which turns a `null Double`
     * into `0.0` before writing it (5.18.0 bytecode), and `0.0` on the yaw or roll axis is
     * precisely what makes a Mini 4 Pro answer `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW` —
     * [Mobile-SDK-Android-V5#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527).
     * The `*Ignored` flag is the only thing that suppresses an axis.
     *
     * The axes are separate parameters rather than a struct so that "which axes did we command?"
     * is a fact a unit test can assert about [MsdkGimbalAim] through a fake port. That assertion
     * is the whole defence against #527 regressing, and it has to live above this line because
     * nothing below it can be tested at all.
     *
     * [onFailure] carries the `IDJIError` name verbatim and arrives on DJI's thread — possibly
     * long after the call returns, and **possibly never**: on 2026-07-26 a flight-controller
     * action was called four times on a healthy aircraft and DJI invoked neither callback, ever.
     * Nothing above this seam may wait on it or read its absence as success.
     *
     * @param absolute `GimbalAngleRotationMode.ABSOLUTE_ANGLE` when true, `RELATIVE_ANGLE` when
     *   false.
     * @param durationS `GimbalAngleRotation.duration`, seconds — how long DJI should take over the
     *   move.
     */
    fun rotateByAngle(
        absolute: Boolean,
        pitchDeg: Double?,
        rollDeg: Double?,
        yawDeg: Double?,
        durationS: Double,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    )

    /**
     * Subscribes to `Gimbal.KeyGimbalAttitude`. [onDelivery] fires on **every** delivery, repeats
     * included, with `null` meaning DJI's component-gone signal — the same listener contract
     * `StateCache` documents. The subscriber stamps a delivery time on each call, so a repeated
     * value counts as a live feed.
     */
    fun listenAttitude(onDelivery: (GimbalAngles?) -> Unit)

    /** Subscribes to `Gimbal.KeyGimbalAttitudeRange`. Same delivery contract. */
    fun listenAttitudeRange(onDelivery: (GimbalLimits?) -> Unit)

    /**
     * Subscribes to `Gimbal.KeyGimbalCMode`, delivering DJI's enum **name** rather than a mapped
     * value — the `AircraftState.flightMode` idiom, so the meaning of `FREE` versus `YAW_FOLLOW`
     * is decided above the seam and tested.
     */
    fun listenWorkMode(onDelivery: (String?) -> Unit)

    /** Subscribes to `Gimbal.KeyConnection`. Same delivery contract. */
    fun listenConnection(onDelivery: (Boolean?) -> Unit)

    /**
     * Cancels every subscription. Idempotent, and must be safe to call when nothing was ever
     * subscribed — `Bridge.stop()` calls it unconditionally so no listener outlives the link.
     */
    fun cancelListens()

    /**
     * **The port stack's memory of the last absolute pitch DJI accepted**, degrees, or null when
     * the stack keeps none — which is the default, because tracking the commanded angle is
     * [CommandedGimbalPort]'s single job and no other implementation may grow a second copy of
     * it. Decorators forward; the DJI port answers null.
     *
     * On the seam so that [MsdkGimbalAim.believedPitch] — the one resolver of "where do we
     * believe the camera points?" — can reach both of its sources without a side channel: the
     * commanded record lives below this seam, the reported attitude lives above it in the aim's
     * own listener state, and a resolution done anywhere else would be the two-places-for-one-
     * property failure this project names. Not a measurement and never written to
     * [GimbalReading], whose "only measurements" rule stands.
     */
    fun commandedPitchDeg(): Double? = null
}

/**
 * A gimbal attitude as it left the DJI seam: DJI's `Attitude` with the DJI type taken off, degrees
 * throughout, every axis independently nullable because DJI's own fields are boxed `Double`s.
 */
data class GimbalAngles(
    val pitchDeg: Double? = null,
    val rollDeg: Double? = null,
    val yawDeg: Double? = null,
)
