package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.Msdk
import com.dimensional.mini4pro.record.StickModes
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener

/**
 * The production [VirtualStickPort]: the only class in `guided/` that imports DJI, and the
 * counterpart of `KeyManagerActionPort` / `KeyManagerGimbalPort` — deliberately decision-free,
 * one method per SDK call, verified on hardware rather than by tests.
 *
 * ## The SDK surface, and how it was established
 *
 * `VirtualStickManager`'s method set was read with `javap` off the runtime classes in
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` when this file was written (the `-provided` jar's
 * *signatures* are trustworthy; its method bodies are ABI stubs —
 * `command/ActionPort.READING_THE_JAR`), and cross-checked against the 5.18.0 offline
 * reference (`tools/djidoc IVirtualStickManager`). The five calls used here:
 * `enableVirtualStick(cb)`, `disableVirtualStick(cb)`,
 * `setVirtualStickAdvancedModeEnabled(boolean)` (void setter),
 * `sendVirtualStickAdvancedParam(param)` (**void** — no acceptance signal exists), and the
 * state-listener pair, whose `setVirtualStickStateListener` **appends** to an internal list
 * (javap: `virtualStickStateListeners`; `Recorder.kt` relies on the same fact), so this port
 * and the flight recorder observe the same stream without displacing each other.
 *
 * ## The fixed Stage A configuration — why the modes are not parameters
 *
 * Every param goes out as `VELOCITY / ANGULAR_VELOCITY / VELOCITY / GROUND`, hard-coded at the
 * one place the object is built. Stage A has exactly one legal configuration
 * (`docs/m3-guided-control.md` §1.4 — also the only one DJI ever supports obstacle avoidance
 * in, and the one every `tools/flightlog` planted-fault profile assumes); making the modes
 * parameters would let a caller above the seam produce the `wrong-control-mode` fault that
 * `--diagnose-axis` case 5 exists to catch. The [SendReport.modes] are still read **off the
 * built object**, not off these constants, so the log records what was sent even if this file
 * drifts.
 *
 * **What `GROUND` means is UNVERIFIED** (the reference defines the enum's effect, not its
 * axes — `docs/m3-guided-control.md` §1.4). The design assumes earth-referenced north/east,
 * the same assumption `--diagnose-axis` prints with every diagnosis; bench item 9 settles it.
 */
class KeyManagerVirtualStickPort(
    /**
     * Narrative sink, decision-free like the rest of this file: the port reports what DJI did
     * (or failed to do), it never acts on it. Added after the 2026-07-26 bench session, where
     * a silently dead RC stick subscription was indistinguishable from a quiet one and cost
     * the session — the port must say when each axis first speaks and when a listen fails.
     */
    private val log: (String) -> Unit = {},
) : VirtualStickPort {

    /** One holder for the four stick subscriptions, so teardown is a single cancelListen. */
    private val holder = Any()

    /** Axes that have delivered at least once this instance — see [listen]'s first-word log. */
    private val axesHeard = java.util.concurrent.ConcurrentHashMap.newKeySet<String>()

    private var stateListener: VirtualStickStateListener? = null

    // Latest raw stick values, so each key delivery can hand a full snapshot upward. Written
    // only from KeyManager callbacks (main thread — docs/msdk-keys.md §A).
    @Volatile private var leftH: Int? = null
    @Volatile private var leftV: Int? = null
    @Volatile private var rightH: Int? = null
    @Volatile private var rightV: Int? = null

    /** Same two reasons, same per-call freshness rule as `KeyManagerActionPort`. */
    override fun unavailableReason(): String? {
        val s = Msdk.state.value
        return when {
            !s.registered -> "SDK_NOT_REGISTERED"
            !s.productConnected -> "NO_PRODUCT"
            else -> null
        }
    }

    override fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        VirtualStickManager.getInstance().enableVirtualStick(callback(onSuccess, onFailure))
    }

    override fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        VirtualStickManager.getInstance().disableVirtualStick(callback(onSuccess, onFailure))
    }

    override fun setAdvancedMode(enabled: Boolean) {
        VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(enabled)
    }

    override fun sendAdvancedParam(
        pitch: Double,
        roll: Double,
        yaw: Double,
        verticalThrottle: Double,
    ): SendReport {
        val param = VirtualStickFlightControlParam().apply {
            this.pitch = pitch
            this.roll = roll
            this.yaw = yaw
            this.verticalThrottle = verticalThrottle
            rollPitchControlMode = RollPitchControlMode.VELOCITY
            yawControlMode = YawControlMode.ANGULAR_VELOCITY
            verticalControlMode = VerticalControlMode.VELOCITY
            rollPitchCoordinateSystem = FlightCoordinateSystem.GROUND
        }
        // Off the object that was actually sent — the flight-recording rule.
        val modes = StickModes(
            rollPitch = param.rollPitchControlMode?.name,
            yaw = param.yawControlMode?.name,
            vertical = param.verticalControlMode?.name,
            coordinateSystem = param.rollPitchCoordinateSystem?.name,
            advanced = true,
        )
        return try {
            VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param)
            SendReport(modes, null)
        } catch (t: Throwable) {
            SendReport(modes, t.toString())
        }
    }

    override fun listenState(
        onState: (VirtualStickSnapshot) -> Unit,
        onAuthorityReason: (String) -> Unit,
    ) {
        val listener = object : VirtualStickStateListener {
            override fun onVirtualStickStateUpdate(stickState: VirtualStickState) {
                onState(
                    VirtualStickSnapshot(
                        enabled = stickState.isVirtualStickEnable,
                        advanced = stickState.isVirtualStickAdvancedModeEnabled,
                        authority = stickState.currentFlightControlAuthorityOwner?.name,
                    )
                )
            }

            override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
                onAuthorityReason(reason.name)
            }
        }
        stateListener = listener
        VirtualStickManager.getInstance().setVirtualStickStateListener(listener)
    }

    // The RC feed request, held until MSDK registration makes planting meaningful. A
    // KeyManager.listen made before registration is the silent pre-registration no-op
    // docs/architecture.md warns about (measured 2026-07-26: the bridge auto-start beats
    // registration by ~1 s on every fresh launch, and the feed stays deaf forever).
    @Volatile private var rcOnDelivery: ((RcSticks) -> Unit)? = null
    private var rcPlanted = false // guarded by synchronized(this)

    override fun listenRcSticks(onDelivery: (RcSticks) -> Unit) {
        rcOnDelivery = onDelivery
        ensureRcFeed()
    }

    override fun ensureRcFeed() {
        synchronized(this) {
            if (rcPlanted) return
            if (!Msdk.state.value.registered) return // Bridge.tick retries in 200 ms
            val onDelivery = rcOnDelivery ?: return
            rcPlanted = true
            // getOnce = true, the house rule: primes each axis with the current value, so a
            // subscription made mid-session learns the state it joined.
            listen(RemoteControllerKey.KeyStickLeftHorizontal, "leftH") { leftH = it; onDelivery(snapshot()) }
            listen(RemoteControllerKey.KeyStickLeftVertical, "leftV") { leftV = it; onDelivery(snapshot()) }
            listen(RemoteControllerKey.KeyStickRightHorizontal, "rightH") { rightH = it; onDelivery(snapshot()) }
            listen(RemoteControllerKey.KeyStickRightVertical, "rightV") { rightV = it; onDelivery(snapshot()) }
            log("RC stick listens planted (SDK registered)")
        }
    }

    override fun cancelListens() {
        try {
            KeyManager.getInstance().cancelListen(holder)
        } catch (_: Throwable) {
            // Teardown must not throw over a subscription that never existed.
        }
        try {
            stateListener?.let { VirtualStickManager.getInstance().removeVirtualStickStateListener(it) }
        } catch (_: Throwable) {
        }
        stateListener = null
    }

    private fun snapshot(): RcSticks = RcSticks(leftH, leftV, rightH, rightV)

    private fun listen(info: DJIKeyInfo<Int>, axis: String, sink: (Int?) -> Unit) {
        try {
            KeyManager.getInstance().listen(KeyTools.createKey(info), holder, true) { _, newValue ->
                // First word from each axis is logged so a dead subscription is visible in
                // logcat instead of masquerading as quiet sticks (2026-07-26 bench session).
                // Otherwise no work in the listener beyond handing the value up
                // (docs/architecture.md); the decisions and their lock live in GuidedStickEngine.
                if (axesHeard.add(axis)) log("RC stick feed: first delivery $axis=$newValue")
                sink(newValue)
            }
        } catch (t: Throwable) {
            // The recorder's rule: a listen that cannot be planted is a fact to report, not an
            // exception to die on — the engine's allPresent() gate already fails closed.
            log("RC stick listen($axis) FAILED to plant: $t")
        }
    }

    private fun callback(onSuccess: () -> Unit, onFailure: (String) -> Unit) =
        object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() = onSuccess()
            override fun onFailure(error: IDJIError) = onFailure(errorName(error))
        }

    /** DJI's own name for the refusal, verbatim — `KeyManagerActionPort.errorName`'s rule. */
    private fun errorName(error: IDJIError): String =
        error.errorCode()?.takeIf { it.isNotBlank() } ?: error.toString()
}
