package com.dimensional.mini4pro.health

import com.dimensional.mini4pro.warn.Warning

/**
 * The DJI seam for device health: everything [DeviceHealthWatch] needs from the SDK, and nothing
 * else. The only production implementation is [MsdkDeviceHealthPort]; the tests supply a fake and
 * never touch DJI.
 *
 * Same shape and same reasoning as `gimbal/GimbalPort`, `guided/VirtualStickPort` and
 * `simulator/SimulatorPort`.
 */
interface DeviceHealthPort {

    /**
     * Why device health cannot be subscribed to right now, or `null` when it can.
     *
     * **Read fresh on every attempt, never cached** — the rule `KeyManagerActionPort` states. The
     * only reason that matters here is `SDK_NOT_REGISTERED`; see [DeviceHealthWatch.ensure] for
     * why getting that wrong is silent rather than loud.
     */
    fun unavailableReason(): String?

    /**
     * Plant the health listener. Called at most once per successful [DeviceHealthWatch.ensure];
     * the watch owns the idempotence, not the port.
     *
     * @param onDelivery invoked with **the complete current list** of warnings, every time DJI
     *   decides something moved. On DJI's own callback thread, which is the Android main thread.
     */
    /**
     * Plant the listener.
     *
     * @return true if it is live. **A failure is reported, never thrown** — the caller runs on the
     * bridge's 200 ms tick, and an exception escaping there cancels the scheduled task for the rest
     * of the session, which would stop all telemetry. A listen that cannot be planted is a fact to
     * report, not an exception to die on (`guided/KeyManagerVirtualStickPort.listen`'s rule), and
     * returning false is what lets the tick retry it.
     */
    fun listen(onDelivery: (List<Warning>) -> Unit): Boolean

    /** Remove the listener. Must be safe to call when nothing was ever planted. */
    fun cancelListen()

    /**
     * DJI's current list, read directly rather than waited for.
     *
     * The reason this exists at all: a listener planted mid-session is not primed. DJI's own
     * sample (`SampleCode-V5/.../DiagnosticVm.kt`) calls `getCurrentDJIDeviceHealthInfos()`
     * alongside `addDJIDeviceHealthInfoChangeListener` for exactly that reason, and without it a
     * warning that was already standing when we subscribed stays invisible until it next changes
     * — which for a stuck overheat is never. That is precisely the failure this package exists to
     * end, so the priming read is not an optimisation.
     *
     * @return null if the read failed or DJI has nothing to say. Never throws.
     */
    fun current(): List<Warning>?
}
