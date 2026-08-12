package com.dimensional.mini4pro.health

import com.dimensional.mini4pro.warn.WarnSource
import com.dimensional.mini4pro.warn.Warning
import com.dimensional.mini4pro.warn.WarningBus

/**
 * **DJI's device-health feed, as a source of [WarningBus]** — the subscription, the retry, the
 * priming read, and nothing else.
 *
 * It used to be the whole feature: it planted the listener *and* decided severities *and* fanned
 * out to QGC, the record and logcat. On 2026-07-30 that fan-out moved to `warn/WarningBus`, which
 * owns it for every source, and this class kept exactly the half that is about DJI's health manager
 * specifically. The trade is deliberate and it is `CLAUDE.md`'s single-owner rule: adding the wind
 * warning to the old shape would have meant a second severity table, a second sentence builder and
 * a second set of sinks — the two-places-for-one-property failure, in advance.
 *
 * What it still owns is the part that is genuinely this subsystem's:
 *
 * ## The silent pre-registration no-op
 *
 * A DJI listener planted before MSDK registration completes **silently never delivers**. No error,
 * no exception, no callback — the subscription simply does not exist, and the symptom is
 * indistinguishable from an aircraft that never reports anything. This project has already been
 * bitten twice: `guided/KeyManagerVirtualStickPort.ensureRcFeed`'s comment records that the bridge
 * auto-start beats registration by ~1 s on every fresh launch, so the natural place to subscribe is
 * always the wrong one.
 *
 * The cure used there is the cure used here. [ensure] is **idempotent and cheap**, it refuses while
 * [DeviceHealthPort.unavailableReason] says the SDK is not ready, and `Bridge.tick()` calls it
 * every 200 ms until it takes. Nothing subscribes at construction time.
 *
 * It is assumed rather than measured that `DeviceHealthManager` shares the failure mode with
 * `KeyManager`, and that assumption is free: the retry costs one boolean read per tick, and being
 * wrong about it changes nothing. Being wrong the *other* way — subscribing once at construction
 * and never checking — is the bug that hides an overheat warning for a whole session.
 *
 * ## Priming
 *
 * The first successful [ensure] also reads [DeviceHealthPort.current] and feeds it through the same
 * path as a delivery. Without it, a warning that was already standing when we subscribed stays
 * invisible until DJI next changes it — and for the stuck overheat that started this package, "next
 * changes it" may be never.
 *
 * ## Threading
 *
 * DJI delivers on the Android main thread; [ensure] is called from `Bridge`'s telemetry thread.
 * Serialisation of the monitor's mutable state belongs to [WarningBus], on its own gate; what is
 * serialised here is only [planted].
 */
class DeviceHealthWatch(
    private val port: DeviceHealthPort,
    /** The one owner of what happens to a warning. See [WarningBus]. */
    private val warnings: WarningBus,
    /** Diagnostics about the subscription itself, not about any warning. */
    private val note: (String) -> Unit = {},
) {

    private val gate = Any()

    /** Guarded by [gate]. */
    private var planted = false

    /** True once the listener is live. For the status screen and for tests. */
    val isListening: Boolean get() = synchronized(gate) { planted }

    /**
     * The warnings DJI's health manager is currently reporting. Empty before the first delivery.
     *
     * Read out of the bus's own picture rather than kept a second time here — one owner of "what is
     * currently wrong", asked a narrower question.
     */
    fun snapshot(): List<Warning> =
        warnings.snapshot().filter { it.source == WarnSource.DEVICE_HEALTH }

    /**
     * Plant the listener if the SDK is ready and it is not planted already.
     *
     * Called from `Bridge.tick()` every 200 ms. Returns quickly and says nothing on the paths that
     * do nothing — a log line per tick would be its own flood.
     *
     * @return true if the listener is live after this call.
     */
    fun ensure(): Boolean {
        synchronized(gate) {
            if (planted) return true
            // Fresh on every attempt, never cached — `KeyManagerActionPort`'s rule.
            if (port.unavailableReason() != null) return false // Bridge.tick retries in 200 ms
        }
        // Outside the lock: `listen` reaches into the SDK, and a DJI callback that fires
        // synchronously from inside `addDJIDeviceHealthInfoChangeListener` would otherwise deadlock
        // against the bus's gate by way of this one.
        //
        // `planted` is committed only once the SDK has actually taken the listener. Setting it
        // before the call would make a failed plant permanent: the flag would suppress every
        // subsequent retry, and health would be silently dead for the whole session — which is
        // exactly the failure mode this package was built to end.
        if (!port.listen(::onDelivery)) return false // Bridge.tick retries in 200 ms
        synchronized(gate) { planted = true }
        note("device health listener planted (SDK registered)")
        // The listener is planted first, so a change landing between the two calls is seen by the
        // listener rather than falling into the gap. A warning present in both is deduped by the
        // monitor's diff, which is exactly what the diff is for.
        port.current()?.let { onDelivery(it) }
        return true
    }

    /**
     * Drop the subscription and forget this source's picture.
     *
     * Not called by `Bridge.stop()` — see `Bridge.deviceHealth` for why an observation-only
     * subscription is deliberately allowed to outlive a link. Provided because a seam with no
     * teardown is a seam that cannot be tested, and because the forget must exist for the reconnect
     * case whether or not anything calls it today.
     *
     * **An empty delivery, not a bus reset**: this source going away says nothing about any other
     * source's picture, and resetting the bus would drop those without a word. The empty picture
     * clears exactly this source's warnings, through the same diff as everything else, so the
     * operator gets a proper `cleared` for each one rather than silence.
     */
    fun stop() {
        synchronized(gate) {
            if (!planted) return
            planted = false
        }
        port.cancelListen()
        warnings.deliver(WarnSource.DEVICE_HEALTH, emptyList())
        note("device health listener removed")
    }

    /**
     * One delivery from DJI: the complete current list of health warnings.
     *
     * Contained end to end inside [WarningBus.deliver] — a throw here would land on DJI's callback
     * thread, the main thread, where this project has already killed the process once by letting an
     * exception escape a DJI callback (`Bridge.sendOffMain`).
     */
    internal fun onDelivery(items: List<Warning>) {
        warnings.deliver(WarnSource.DEVICE_HEALTH, items)
    }
}
