package com.dimensional.mini4pro.command

import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.Tap

/**
 * An [ActionPort] that **records every ask and every answer on the way past** — the aircraft-
 * outbound end of the [Tap] seam, for the four things this bridge can tell an aircraft to do.
 *
 * ## Why a decorator and not a `record(...)` call inside the port
 *
 * Three shapes were available and this is the third:
 *
 * 1. *A recorder call inside `KeyManagerActionPort`.* Rejected: that class is deliberately too
 *    dumb to test (`ActionPort`'s KDoc — *"if a method here ever wants an `if`, the `if` belongs
 *    in `MsdkFlightActions`"*), and it is the one class on this path a unit test cannot reach at
 *    all. Recording that cannot be tested is exactly what the gimbal gap was.
 * 2. *A recorder call inside `MsdkFlightActions`.* Rejected because it is beside the ask rather
 *    than on the way past: the decision layer already announces and already logs, and adding a
 *    third thing it must remember for each new action is the convention this seam exists to
 *    replace. It is also the wrong side of the seam — `MsdkFlightActions` cannot see whether the
 *    SDK call threw.
 * 3. **This.** The recording is a property of *reaching the wire*, so it lives in the object that
 *    stands between the decision and the wire. `MsdkFlightActions` is unchanged and untouched, and
 *    every one of its tests still passes a fake port — because this decorator is an `ActionPort`
 *    like any other, a test can equally wrap the fake and assert on what was recorded.
 *
 * The factory in `KeyManagerActionPort.kt` is what makes this structural rather than advisory: the
 * production port class is file-private there, and the only way to obtain one is a function that
 * **requires a [Tap]**. An unrecorded action port cannot be constructed.
 *
 * ## Faithfulness
 *
 * Every method forwards to [inner] with its arguments untouched and its callbacks intact — the
 * caller's `onSuccess`/`onFailure` are invoked exactly once each, exactly when DJI invokes ours,
 * in the same order, on the same thread. Nothing here can change the wire, delay a callback or
 * swallow one. `unavailableReason`, `canStart*` and the two `listen*` are pass-throughs with no
 * recording at all: a precondition read is not traffic, and the listens are inbound state that
 * `Recorder` already subscribes to itself.
 *
 * `cancelListens` is recorded — it is an ask to the MSDK and it is the last thing a session does,
 * so its presence at the end of a record is what says the teardown ran.
 *
 * ## Never perturbs the flight
 *
 * The tap contains its own throws ([Tap]'s contract, implemented once in `DjiCalls`), so no `try`
 * appears here — one would be dead code that made the next reader believe it was load-bearing.
 * What *is* load-bearing is the ordering: the ask is recorded **before** the SDK call, so a
 * `performAction` that throws still leaves the ask on the record.
 */
class RecordedActionPort(
    private val inner: ActionPort,
    private val tap: Tap,
) : ActionPort {

    override fun unavailableReason(): String? = inner.unavailableReason()

    override fun canStartGoHome(): Boolean = inner.canStartGoHome()

    override fun canStartAutoLanding(): Boolean = inner.canStartAutoLanding()

    override fun canStopAutoLanding(): Boolean = inner.canStopAutoLanding()

    override fun canStartTakeoff(): Boolean = inner.canStartTakeoff()

    override fun startGoHome(onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.GO_HOME)
        inner.startGoHome { error ->
            call.refused(error)
            onFailure(error)
        }
    }

    /**
     * Note there is no success path to record here, and that is not an omission: `startTakeoff`
     * deliberately has no `onSuccess` ([ActionPort.startTakeoff]). So a takeoff that DJI accepted
     * and a takeoff DJI swallowed are indistinguishable at this seam — and the record now says
     * which one it was anyway, because an ask with no answer is swept into a `none` after five
     * seconds. That is the first time this path has been able to state the difference.
     */
    override fun startTakeoff(onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.TAKEOFF)
        inner.startTakeoff { error ->
            call.refused(error)
            onFailure(error)
        }
    }

    override fun startAutoLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.LAND)
        inner.startAutoLanding(
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

    /** Rule 1's withdrawal of a committed autoland — whether DJI honours it is the measurement. */
    override fun stopAutoLanding(onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.STOP_LANDING)
        inner.stopAutoLanding { error ->
            call.refused(error)
            onFailure(error)
        }
    }

    /** The one call that tells a hovering aircraft to descend the last half metre. */
    override fun confirmLanding(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.CONFIRM_LANDING)
        inner.confirmLanding(
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

    override fun listenIsLandingConfirmationNeeded(onDelivery: (Boolean?) -> Unit) =
        inner.listenIsLandingConfirmationNeeded(onDelivery)

    override fun listenIsInLandingMode(onDelivery: (Boolean?) -> Unit) =
        inner.listenIsInLandingMode(onDelivery)

    /** Synchronous and void — `settled(null)` is the whole of what a clean return proves. */
    override fun cancelListens() {
        val call = tap.aircraftOut(DjiOp.CANCEL_LISTENS, urgent = false)
        try {
            inner.cancelListens()
        } catch (e: Throwable) {
            call.settled("${e.javaClass.simpleName}: ${e.message}")
            throw e
        }
        call.settled(null)
    }
}
