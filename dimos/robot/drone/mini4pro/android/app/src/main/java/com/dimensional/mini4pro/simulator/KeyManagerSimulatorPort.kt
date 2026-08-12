package com.dimensional.mini4pro.simulator

import com.dimensional.mini4pro.Msdk
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.SimulatorInitializationSettings
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.sdk.keyvalue.value.flightcontroller.SimulatorState as DjiSimulatorState

/**
 * The production [SimulatorPort]: the only class in this package that touches the MSDK.
 *
 * `KeyManagerActionPort`'s counterpart for the simulator, and deliberately the same shape — one
 * method per key, callbacks unwrapped, no branch that a test would want to reach. Every decision
 * lives in [SimulatorControl] where a fake port can drive it. Like `StateCache` and
 * `KeyManagerActionPort`, this class is verified on hardware rather than by tests, and it is kept
 * this thin precisely so that verification is a short walk.
 *
 * It sits in `simulator/` rather than in the root package alongside `KeyManagerActionPort`
 * because the feature is one unit and splitting four files across two packages to honour a
 * naming habit would make it harder to see, not easier. The rule that actually matters —
 * *"DJI types are unwrapped only at the seam"* — is unaffected: this file is the seam, and it is
 * the only file under `simulator/` that imports `dji.*`. `record/Recorder.kt` already sets the
 * precedent for a DJI-touching class inside a feature package.
 *
 * ## Why `KeyManager` and not `SimulatorManager`
 *
 * `SimulatorManager` is itself a `KeyManager` wrapper over these same four keys, so going direct
 * costs nothing and removes a layer.
 *
 * **CORRECTED 2026-07-26.** This doc previously said `isSimulatorEnabled()` is "stubbed to
 * `return false`" and treated that as the reason to bypass it. **That was a misreading of the
 * jar.** `dji-sdk-v5-aircraft-provided-*.jar` is a **compile-only ABI stub**: every method body
 * has a return-default *prepended*, with the real implementation left behind it as unreachable
 * dead code. `iconst_0; ireturn` at offsets 0–1 is a linking artefact, not shipped behaviour —
 * and the real body from offset 2 does exactly what the docs say, reading
 * `KeyIsSimulatorStarted`. So there was never a hard-coded "no simulator running" to avoid.
 *
 * Reading the key directly is still right, on the weaker but sufficient grounds above. The
 * general rule this cost us: **a one- or two-instruction body in this jar tells you nothing.**
 * Key *metadata* is different and remains trustworthy — `canPerformAction`, `canGet`, `canSet`,
 * `canListen` and the converters are `DJIKeyInfo` instance fields set by a builder in a static
 * initialiser, which is real data rather than a stubbed body. That is why the table below stands.
 *
 * ## Keys, verified by `javap` against `dji-sdk-v5-aircraft-provided-5.18.0.jar`
 *
 * | method | key | shape |
 * |---|---|---|
 * | [start] | `FC.KeyStartSimulator` | `DJIActionKeyInfo<SimulatorInitializationSettings, EmptyMsg>`, `canPerformAction(true)` |
 * | [stop] | `FC.KeyStopSimulator` | `DJIActionKeyInfo<EmptyMsg, EmptyMsg>`, `canPerformAction(true)` |
 * | [listenIsSimulatorStarted] | `FC.KeyIsSimulatorStarted` | `DJIKeyInfo<Boolean>`, `canGet(true) canListen(true)` |
 * | [listenSimulatorState] | `FC.KeySimulatorState` | `DJIKeyInfo<SimulatorState>`, `canGet(true) canListen(true)` |
 *
 * No `canPerformAction()` pre-check is made here, unlike `KeyManagerActionPort`. That guard earns
 * its place on Return and Land because those are the buttons an operator presses in flight and a
 * demoted key must fail closed with a name. The simulator is started once, deliberately, from a
 * screen the operator is looking at, and a refusal arrives on [start]'s `onFailure` with DJI's own
 * word — the same channel, one step later. Both keys read `canPerformAction(true)` in 5.18.0
 * (bytecode offsets 8513 and 8564 of `DJIFlightControllerKey.<clinit>`) and, as
 * `ActionPort.canStartGoHome` documents at length, the flag is a class-load constant rather than
 * a pre-flight check, so asking would prove nothing here.
 */
/**
 * **The only way to obtain a production [SimulatorPort], and it requires a
 * [com.dimensional.mini4pro.record.Tap].** The class below is private to this file, so an
 * unrecorded simulator port cannot be constructed — see `actionPort` in `KeyManagerActionPort.kt`
 * for the full argument.
 */
fun simulatorPort(tap: com.dimensional.mini4pro.record.Tap): SimulatorPort =
    RecordedSimulatorPort(KeyManagerSimulatorPort(), tap)

private class KeyManagerSimulatorPort : SimulatorPort {

    /** One holder for both subscriptions, so teardown is a single cancelListen — as StateCache. */
    private val holder = Any()

    /**
     * Read fresh on every call, as `KeyManagerActionPort.unavailableReason`: `Msdk.state` is the
     * lifecycle's single source of truth, and a per-call read means an aircraft unplugged between
     * two presses refuses the second one with no listener plumbing to go stale in between.
     *
     * This is also where the answer to *"does the simulator need real hardware?"* is enforced.
     * `NO_PRODUCT` is not a defensive check: every operation below is a `FlightControllerKey`
     * request proxied to the aircraft's own flight controller, so with no aircraft there is
     * nothing to simulate on.
     */
    override fun unavailableReason(): String? {
        val s = Msdk.state.value
        return when {
            !s.registered -> "SDK_NOT_REGISTERED"
            !s.productConnected -> "NO_PRODUCT"
            else -> null
        }
    }

    override fun start(
        latitude: Double,
        longitude: Double,
        satelliteCount: Int,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    ) {
        // The keyvalue settings type, not `simulator.InitializationSettings`: the latter is only
        // a holder that SimulatorManager copies into this one before performing the same action.
        val settings = SimulatorInitializationSettings(latitude, longitude, satelliteCount)
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStartSimulator),
            settings,
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) = onSuccess()
                override fun onFailure(error: IDJIError) = onFailure(errorName(error))
            },
        )
    }

    override fun stop(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyStopSimulator),
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) = onSuccess()
                override fun onFailure(error: IDJIError) = onFailure(errorName(error))
            },
        )
    }

    override fun listenIsSimulatorStarted(onDelivery: (Boolean?) -> Unit) =
        listen(FlightControllerKey.KeyIsSimulatorStarted, onDelivery)

    override fun listenSimulatorState(onDelivery: (SimulatedAircraft?) -> Unit) =
        listen(FlightControllerKey.KeySimulatorState) { onDelivery(it?.flatten()) }

    override fun cancelListens() = KeyManager.getInstance().cancelListen(holder)

    /**
     * `getOnce = true`, the house rule for every subscription in this project — and load-bearing
     * here rather than merely tidy. Without it a simulator started before this process launched
     * would only become visible when DJI next *changed* the flag, which for a running simulator
     * is never. The whole foreign-simulator detection depends on this boolean.
     */
    private fun <T> listen(info: DJIKeyInfo<T>, onDelivery: (T?) -> Unit) {
        KeyManager.getInstance().listen(KeyTools.createKey(info), holder, true) { _, newValue ->
            // Do no work in the listener beyond handing the value up; the decisions and their
            // lock live in SimulatorControl.
            onDelivery(newValue)
        }
    }

    /** DJI's boxed getters straight across. Nulls stay null — see [SimulatedAircraft]. */
    private fun DjiSimulatorState.flatten() = SimulatedAircraft(
        motorsOn = areMotorsOn,
        flying = isFlying,
        pitchDeg = pitch,
        rollDeg = roll,
        yawDeg = yaw,
        positionX = positionX,
        positionY = positionY,
        positionZ = positionZ,
        latitude = location?.latitude,
        longitude = location?.longitude,
    )

    /** The string the operator will search DJI's forums for — as `KeyManagerActionPort`. */
    private fun errorName(error: IDJIError): String =
        error.errorCode()?.takeIf { it.isNotBlank() } ?: error.toString()
}
