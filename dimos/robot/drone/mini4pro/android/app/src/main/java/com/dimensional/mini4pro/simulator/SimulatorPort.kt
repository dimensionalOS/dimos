package com.dimensional.mini4pro.simulator

/**
 * The thin seam between [SimulatorControl] — every decision this bridge makes about the MSDK
 * aircraft simulator — and the four `KeyManager` calls those decisions ride on.
 *
 * The same split as `command/ActionPort.kt` + `KeyManagerActionPort.kt`, for the same reason: the
 * MSDK is not on the unit-test runtime classpath (`docs/architecture.md`), so anything below a
 * DJI seam cannot be tested, and the decisions that keep a *simulated* aircraft distinguishable
 * from a real one are exactly the code that must be. [KeyManagerSimulatorPort] is the production
 * implementation and is deliberately too dumb to test: one method per key, no branches beyond
 * unwrapping a callback.
 *
 * ## What the simulator API actually is — read this before changing anything
 *
 * DJI documents a manager, `ISimulatorManager` (`tools/djidoc ISimulatorManager`, supported since
 * MSDK 5.0.0), with `isSimulatorEnabled` / `enableSimulator` / `disableSimulator` /
 * `addSimulatorStateListener`. **This port does not use it.** It goes straight to `KeyManager`,
 * and that is a deliberate decision with two pieces of evidence behind it.
 *
 * ### 1. `SimulatorManager` is itself only a `KeyManager` wrapper
 *
 * Disassembled from `dji-sdk-v5-aircraft-provided-5.18.0.jar`, `SimulatorManager`'s entire
 * implementation is four key operations:
 *
 * | manager method | what it actually does |
 * |---|---|
 * | `enableSimulator(settings, cb)` | `performAction(FC.KeyStartSimulator, SimulatorInitializationSettings, cb)` |
 * | `disableSimulator(cb)` | `performAction(FC.KeyStopSimulator, cb)` |
 * | `isSimulatorEnabled()` | `getValue(FC.KeyIsSimulatorStarted)` |
 * | `addSimulatorStateListener(l)` | `listen(FC.KeySimulatorState, …)` |
 *
 * The keys, straight out of `FlightControllerKey` in the same jar:
 *
 * ```
 * DJIActionKeyInfo<SimulatorInitializationSettings, EmptyMsg> KeyStartSimulator   canPerformAction(true)
 * DJIActionKeyInfo<EmptyMsg, EmptyMsg>                        KeyStopSimulator    canPerformAction(true)
 * DJIKeyInfo<Boolean>                                         KeyIsSimulatorStarted  canGet(true) canListen(true)
 * DJIKeyInfo<SimulatorState>                                  KeySimulatorState      canGet(true) canListen(true)
 * ```
 *
 * So going direct costs nothing and buys the house `KeyManager` idiom — one holder, `getOnce`,
 * a single `cancelListen` — that `StateCache` and `KeyManagerActionPort` already use.
 *
 * ### 2. RETRACTED — `isSimulatorEnabled()` is **not** stubbed, and this was a misreading
 *
 * This section claimed `SimulatorManager.isSimulatorEnabled()` always returns `false`, on the
 * strength of `javap -c` showing:
 *
 * ```
 * 0: iconst_0
 * 1: ireturn                                        <-- read as "the method ends here"
 * 2: invokestatic  KeyManager.getInstance           <-- read as dead code
 * 5: getstatic     FlightControllerKey.KeyIsSimulatorStarted
 * ```
 *
 * **That shape is a property of the jar, not of the SDK.** `dji-sdk-v5-aircraft-provided-*.jar`
 * is a **compile-only ABI stub**: it prepends a return-default to *every* method body and leaves
 * the real implementation behind it. `IMediaDataCenter.getInstance()` shows the identical
 * `aconst_null; areturn` prologue over its real body. So the two instructions are a linking
 * artefact, the code at offset 2 is what actually ships, and DJI's documentation — *"To get
 * whether the simulator is turned on"* — was right all along.
 *
 * Corrected 2026-07-26, found while auditing `video/` against the same jar. **The general rule,
 * which cost more than this one method: a one- or two-instruction body in this jar tells you
 * nothing.** Key *metadata* is unaffected and still trustworthy — `canPerformAction`, `canGet`,
 * `canSet`, `canListen` and the converters are `DJIKeyInfo` instance fields assigned by a builder
 * in a static initialiser, i.e. real data rather than a stubbed body.
 *
 * **Reading `KeyIsSimulatorStarted` directly is still what this port does**, and is still right,
 * but for an ordinary reason rather than a dramatic one: `SimulatorManager` is a thin wrapper over
 * that same key, so going direct removes a layer at no cost. Everything the safety argument below
 * needs — observing a simulator we did not start — works either way. The argument stands; only
 * this justification for it was wrong.
 *
 * Kept rather than deleted because the reasoning error is worth more than the conclusion: an
 * unreachable-looking real body is exactly what a reverse-engineer expects to be meaningful, and
 * the next person to disassemble this jar will feel the same pull. Historic note follows.
 *
 * **A method that always answers "no simulator" would be the worst possible failure for this
 * feature.**
 * The whole safety argument below rests on being able to see a simulator we did not start; a
 * hard-coded `false` would report a simulated aircraft as a real one, silently, forever. So the
 * only source of truth this port will accept is `KeyIsSimulatorStarted`, read and *listened* to
 * through `KeyManager`, which is the same key the dead code was going to read.
 *
 * ### 3. It needs a real, connected aircraft — this is a bench tool, not a desk tool
 *
 * That follows from (1) rather than from any DJI statement. Every one of the four operations is a
 * `KeyManager` call on a `FlightControllerKey`, i.e. a request proxied over the RC link to the
 * aircraft's own flight controller. There is nothing here that runs on the phone. The simulation
 * is computed *by the flight controller*, which is why [SimulatedAircraft] comes back from the
 * aircraft rather than being integrated locally.
 *
 * Concretely: **phone + RC + a powered, connected aircraft, propellers off.** It removes the sky,
 * not the hardware. `unavailableReason` reports `SDK_NOT_REGISTERED` / `NO_PRODUCT` for exactly
 * this reason, in the same words `ActionPort` uses.
 *
 * ### 4. Three methods DJI's documentation does not mention
 *
 * `javap` on `ISimulatorManager` lists nine methods; `tools/djidoc ISimulatorManager` documents
 * five. The undocumented ones are `clearAllSimulatorStateListener()`,
 * `setFlyZoneLimitationEnabled(boolean, cb)`, `getFlyZoneLimitationEnabled(cb)` and
 * `setWindSpeed(SimulatorWindInfo)` — the last with a whole undocumented `SimulatorWindInfo`
 * class carrying `getWindSpeedX/Y/Z`. None are used here (wind and fly-zone limits are inputs to
 * a physics model we have not validated at all), but they are recorded because the pattern
 * "the jar has more than the docs" keeps repeating on this project.
 */
interface SimulatorPort {

    /**
     * Why the MSDK cannot be asked anything right now, or null if it can.
     *
     * Identical in shape and vocabulary to [com.dimensional.mini4pro.command.ActionPort]'s: the
     * two production reasons are `SDK_NOT_REGISTERED` and `NO_PRODUCT`, both statements about us
     * rather than about a flight controller that refused something.
     */
    fun unavailableReason(): String?

    /**
     * `performAction(FC.KeyStartSimulator, SimulatorInitializationSettings(lat, lon, sats))`.
     *
     * [onSuccess] means **DJI accepted the start request**, not that the aircraft is simulating.
     * Nothing above this seam may treat it as the latter — the simulator is "on" only when
     * [listenIsSimulatorStarted] says so. That is the same distinction `MsdkFlightActions` draws
     * for a landing start, and it exists for the same measured reason: on 2026-07-26 a
     * `performAction` on a healthy connected aircraft invoked **neither** callback, four times
     * (`docs/measurements/2026-07-26-m2-first-command.md`). Absence of a callback is not
     * evidence of anything.
     *
     * [satelliteCount] is passed through unclamped; DJI documents `@IntRange(from=0,to=20)` and
     * [SimulatorControl] refuses out-of-range values above the seam rather than silently
     * altering the operator's number here.
     */
    fun start(
        latitude: Double,
        longitude: Double,
        satelliteCount: Int,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    )

    /**
     * `performAction(FC.KeyStopSimulator)`. Same contract: [onSuccess] is acceptance, and the
     * simulator is off only when [listenIsSimulatorStarted] delivers `false`.
     */
    fun stop(onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /**
     * Subscribes to `FC.KeyIsSimulatorStarted` — **the only fact this feature's safety rests
     * on.**
     *
     * [onDelivery] fires on every delivery, repeats included, with `null` meaning DJI's
     * component-gone signal. A `null` is *not* `false`: it means we can no longer see whether a
     * simulator is running, which [SimulatorControl] reports as `UNKNOWN` and never as off.
     */
    fun listenIsSimulatorStarted(onDelivery: (Boolean?) -> Unit)

    /**
     * Subscribes to `FC.KeySimulatorState`. Diagnostics only — what the simulated aircraft is
     * doing. Nothing gates on it, because a state we can read tells us nothing about a simulator
     * we cannot; [listenIsSimulatorStarted] is the gate.
     */
    fun listenSimulatorState(onDelivery: (SimulatedAircraft?) -> Unit)

    /**
     * Cancels both subscriptions. Idempotent, and must be safe when nothing was subscribed.
     *
     * **Note who does *not* call this.** `Bridge.stop()` cancels `ActionPort`'s listens because a
     * command can only arrive over the link it owns. A simulator can outlive the link, the
     * bridge, and this process, so its subscription is tied to MSDK registration instead — see
     * [SimulatorControl.observe].
     */
    fun cancelListens()
}

/**
 * DJI's `SimulatorState`, flattened to plain Kotlin at the seam.
 *
 * Every field nullable for the usual reason (`docs/architecture.md`: null means no reading, never
 * zero), and additionally because the DJI value type's getters return boxed `Boolean`/`Double`
 * and can each be null independently — verified by `javap` on
 * `dji.sdk.keyvalue.value.flightcontroller.SimulatorState`, whose nine getters are
 * `getAreMotorsOn`, `getIsFlying`, `getPitch`, `getRoll`, `getYaw`, `getPositionX/Y/Z` and
 * `getLocation`.
 *
 * **The position triple's frame and units are undocumented.** DJI's page for `SimulatorState`
 * describes `getPositionX` only as *"To get the X-axis coordinate of the aircraft in the
 * simulator"* with no origin, no axis convention and no unit. Nothing in this project interprets
 * them; they are carried for display and for the flight log so that a future session can work
 * out what they mean by watching them move.
 */
data class SimulatedAircraft(
    val motorsOn: Boolean? = null,
    val flying: Boolean? = null,
    val pitchDeg: Double? = null,
    val rollDeg: Double? = null,
    val yawDeg: Double? = null,
    val positionX: Double? = null,
    val positionY: Double? = null,
    val positionZ: Double? = null,
    val latitude: Double? = null,
    val longitude: Double? = null,
)
