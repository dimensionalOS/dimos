package com.dimensional.mini4pro

import com.dimensional.mini4pro.command.ActionPort
import com.dimensional.mini4pro.command.RecordedActionPort
import com.dimensional.mini4pro.record.Tap
import dji.sdk.keyvalue.key.DJIActionKeyInfo
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager

/**
 * The production [ActionPort]: the only class that touches the MSDK's **action** surface, as
 * `StateCache` is the only one that unwraps its value types (`docs/architecture.md`).
 *
 * Deliberately decision-free — one method per key, callbacks unwrapped and nothing else — so
 * that everything worth testing lives in `MsdkFlightActions` where a fake port can drive it.
 * This class is verified the way `StateCache` is: on hardware, not by tests, and it is kept thin
 * precisely so that verification is a short walk.
 *
 * All six keys are `[class, verified twice]` in `docs/msdk/actions.md` and were re-checked by
 * `javap` against `dji-sdk-v5-aircraft-provided-5.18.0.jar` when this class was written: the
 * four actions are `DJIActionKeyInfo<EmptyMsg, EmptyMsg>` with `canPerformAction`, the two
 * booleans are get/listen. `KeyIsGoHomePathSupport` reads true on this airframe
 * (`docs/msdk/actions.md`), and landing and takeoff are both on the constraints doc's
 * confirmed-working list — takeoff with the caveat that it is flaky here (intermittent `-7`,
 * DJI [#783](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/783)).
 */
/**
 * **The only way to obtain a production [ActionPort], and it requires a [Tap].**
 *
 * This is where "recording is structural" stops being a sentence in a design doc. The class below
 * is `private` to this file, so `Bridge` — or anything else — literally cannot construct an
 * unrecorded flight-action port; the compiler refuses. There is no `KeyManagerActionPort()` to
 * call and therefore no version of this wiring that forgets to record.
 *
 * Compare what it replaces: `MavlinkLink` exposed `var onSent` and trusted `Bridge` to install a
 * tap into it, and `Bridge.onInbound` opened with a `Recorder.mavIn` that nothing obliged it to
 * keep. Both worked for months. The gimbal is what a convention like that is worth over weeks —
 * it aimed a real camera and left no trace in any flight record, and nobody decided that.
 *
 * The escape hatch that remains is writing a *new* class implementing [ActionPort] and passing it
 * to `MsdkFlightActions`. That cannot be closed by a type (the missing thing is an absence), and
 * it is closed instead by `record/RecordingSeamTest`, which walks the source tree and fails when a
 * file touches `KeyManager` without being declared.
 */
fun actionPort(tap: Tap): ActionPort = RecordedActionPort(KeyManagerActionPort(), tap)

private class KeyManagerActionPort : ActionPort {

    /** One holder for both subscriptions, so teardown is a single cancelListen — as StateCache. */
    private val holder = Any()

    /**
     * Read fresh on every call rather than cached from a connect callback: `Msdk.state` is the
     * lifecycle's single source of truth, and a per-call read means a product disconnect refuses
     * the very next action with no listener plumbing to go stale in between.
     */
    override fun unavailableReason(): String? {
        val s = Msdk.state.value
        return when {
            !s.registered -> "SDK_NOT_REGISTERED"
            !s.productConnected -> "NO_PRODUCT"
            else -> null
        }
    }

    override fun canStartGoHome(): Boolean = canPerform(FlightControllerKey.KeyStartGoHome)

    override fun canStartAutoLanding(): Boolean =
        canPerform(FlightControllerKey.KeyStartAutoLanding)

    override fun canStopAutoLanding(): Boolean =
        canPerform(FlightControllerKey.KeyStopAutoLanding)

    override fun canStartTakeoff(): Boolean = canPerform(FlightControllerKey.KeyStartTakeoff)

    override fun startGoHome(onFailure: (String) -> Unit) =
        perform(FlightControllerKey.KeyStartGoHome, {}, onFailure)

    /**
     * The key is `DJIActionKeyInfo<EmptyMsg, EmptyMsg>` and so takes the same
     * [CommonCallbacks.CompletionCallbackWithParam] shape as the other three. **Note what does
     * not appear in this line: an altitude.** There is no overload that takes one — see
     * [ActionPort.canStartTakeoff] for the bytecode.
     */
    override fun startTakeoff(onFailure: (String) -> Unit) =
        perform(FlightControllerKey.KeyStartTakeoff, {}, onFailure)

    override fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) =
        perform(FlightControllerKey.KeyStartAutoLanding, onSuccess, onFailure)

    override fun stopAutoLanding(onFailure: (String) -> Unit) =
        perform(FlightControllerKey.KeyStopAutoLanding, {}, onFailure)

    override fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) =
        perform(FlightControllerKey.KeyConfirmLanding, onSuccess, onFailure)

    override fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit) =
        listen(FlightControllerKey.KeyIsLandingConfirmationNeeded, onDelivery)

    override fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit) =
        listen(FlightControllerKey.KeyIsInLandingMode, onDelivery)

    override fun cancelListens() = KeyManager.getInstance().cancelListen(holder)

    /**
     * `DJIKey.canPerformAction()` on the very key [perform] would hand to `KeyManager`.
     *
     * Asked on the key rather than on the `DJIKeyInfo` so it is the same object the action rides,
     * and it touches no manager — `canPerformAction()` reads a field set in the key class's static
     * initialiser, so it is safe before registration and costs a field load. See
     * [ActionPort.canStartGoHome] for what the flag does and does not prove; the short version is
     * that it is a compile-time declaration and DJI ships no runtime pre-check.
     */
    private fun canPerform(info: DJIActionKeyInfo<EmptyMsg, EmptyMsg>): Boolean =
        KeyTools.createKey(info).canPerformAction()

    private fun perform(
        info: DJIActionKeyInfo<EmptyMsg, EmptyMsg>,
        accepted: () -> Unit,
        refused: (String) -> Unit,
    ) {
        KeyManager.getInstance().performAction(
            KeyTools.createKey(info),
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) = accepted()
                override fun onFailure(error: IDJIError) = refused(errorName(error))
            },
        )
    }

    /**
     * `getOnce = true`, the house rule for every subscription: it primes the listener with the
     * current value, so a subscription made mid-episode still learns the state it joined.
     */
    private fun listen(info: DJIKeyInfo<Boolean>, onDelivery: (Boolean?) -> Unit) {
        KeyManager.getInstance().listen(KeyTools.createKey(info), holder, true) { _, newValue ->
            // Do no work in the listener beyond handing the value up; the decisions and their
            // lock live in MsdkFlightActions.
            onDelivery(newValue)
        }
    }

    /**
     * The string the operator will search DJI's forums for. `errorCode()` is the name tier the
     * recorder caught live (`FC_AUTH_STATE` and friends); when DJI leaves it blank the whole
     * error's `toString` goes instead, because a blank is dropped upstream and an unnamed
     * failure the operator never hears about is worse than an ugly one.
     */
    private fun errorName(error: IDJIError): String =
        error.errorCode()?.takeIf { it.isNotBlank() } ?: error.toString()
}
