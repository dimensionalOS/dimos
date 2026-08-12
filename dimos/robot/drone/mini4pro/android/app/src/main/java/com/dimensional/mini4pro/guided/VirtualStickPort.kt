package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.record.StickModes

/**
 * The thin seam between [GuidedStickEngine] — every decision Stage A makes — and the MSDK's
 * virtual-stick surface plus the four RC stick keys the abort gesture reads.
 *
 * **This interface contains no DJI types and must never contain any**, for the reason every
 * seam in this project exists: the MSDK is not on the unit-test classpath
 * (`docs/architecture.md`), so anything on the DJI side cannot be tested — and the decisions
 * that keep a velocity command honest (engagement never latching on a request, the abort
 * gestures, the watchdog, the sign of a climb) are exactly the code that must be.
 * [KeyManagerVirtualStickPort] is the production implementation and is deliberately too dumb to
 * test: one method per SDK call. If a method here ever wants an `if`, the `if` belongs in the
 * engine where a fake port can reach it.
 *
 * The surface below was verified by `javap` against the runtime classes in
 * `dji-sdk-v5-aircraft-provided-5.18.0.jar` when this seam was written (per
 * `command/ActionPort.READING_THE_JAR` — signatures and static data are trustworthy there,
 * method bodies are not): `VirtualStickManager` carries exactly
 * `enableVirtualStick(CompletionCallback)`, `disableVirtualStick(CompletionCallback)`,
 * `setVirtualStickAdvancedModeEnabled(boolean)` (a **void synchronous setter** — nothing to
 * observe), `sendVirtualStickAdvancedParam(VirtualStickFlightControlParam)` (**void** — the SDK
 * reports acceptance only by not throwing), and the state listener pair.
 */
interface VirtualStickPort {

    /**
     * Why the MSDK cannot be asked anything right now, or null. Same two production reasons and
     * the same per-call freshness rule as `command/ActionPort.unavailableReason`.
     */
    fun unavailableReason(): String?

    /**
     * `enableVirtualStick`. Callbacks arrive on DJI's thread, **possibly never** — the measured
     * swallowed-`performAction` behaviour (`docs/measurements/2026-07-26-m2-first-command.md`)
     * applies to this whole callback family, which is why the engine's engagement never rests
     * on these firing. [onFailure] carries DJI's error name verbatim.
     */
    fun enable(onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /** `disableVirtualStick`. Same callback contract. Must be safe to call when never enabled. */
    fun disable(onSuccess: () -> Unit, onFailure: (String) -> Unit)

    /** `setVirtualStickAdvancedModeEnabled` — void, synchronous, unobservable except via [listenState]. */
    fun setAdvancedMode(enabled: Boolean)

    /**
     * One `sendVirtualStickAdvancedParam`, in the fixed Stage A configuration (VELOCITY /
     * ANGULAR_VELOCITY / VELOCITY / GROUND, advanced) — see [KeyManagerVirtualStickPort] for
     * why the modes are not parameters.
     *
     * Returns what was sent and whether the call itself threw. **`accepted` can only ever mean
     * "the SDK call returned"** — the method is void, so there is no acceptance signal at all,
     * and nothing above this seam may treat a clean return as the aircraft honouring anything
     * (`flight-recording.md`: *"stick_cmd.accepted is only whether the SDK call succeeded"*).
     */
    fun sendAdvancedParam(pitch: Double, roll: Double, yaw: Double, verticalThrottle: Double): SendReport

    /**
     * Subscribes to `VirtualStickState` and to DJI's authority-change reasons. [onState] fires
     * on every update with the three facts flattened to plain values; [onAuthorityReason]
     * carries `FlightControlAuthorityChangeReason.name` verbatim. Both on DJI's thread.
     *
     * The listener **appends** alongside the flight recorder's own (`Recorder.kt`:
     * `setVirtualStickStateListener` adds to a list), so subscribing here displaces nothing.
     */
    fun listenState(onState: (VirtualStickSnapshot) -> Unit, onAuthorityReason: (String) -> Unit)

    /**
     * Subscribes to the four `RemoteControllerKey.KeyStick*` keys and delivers the latest full
     * set on every delivery. A null field is a real signal — DJI's component-gone — and reaches
     * the engine as null, never as zero (`docs/architecture.md`).
     */
    fun listenRcSticks(onDelivery: (RcSticks) -> Unit)

    /**
     * Re-checks preconditions and plants any subscription [listenRcSticks] could not plant when
     * it was called. Idempotent and cheap; `Bridge.tick` calls it every 200 ms.
     *
     * Exists because a `KeyManager.listen` planted before MSDK registration is the **silent
     * pre-registration no-op** `docs/architecture.md` warns about — never an error, never a
     * delivery, not even the `getOnce` prime (measured 2026-07-26: the bridge auto-starts ~1 s
     * before registration completes on every fresh app launch, and the RC feed stayed deaf for
     * the whole session while the recorder's identical, later-planted listens flowed). The
     * default is a no-op for ports whose subscriptions have no such precondition (fakes).
     */
    fun ensureRcFeed() {}

    /** Cancels every subscription this port made. Idempotent; `Bridge.stop` calls it unconditionally. */
    fun cancelListens()
}

/**
 * What one `sendAdvancedParam` actually did: the control modes read **off the object that was
 * sent** (the flight-recording rule — recording configured constants would hide a re-set mode),
 * and the throwable's message if the call threw, else null.
 */
data class SendReport(val modes: StickModes, val error: String?)

/**
 * DJI's `VirtualStickState`, flattened at the seam. [authority] is
 * `FlightControlAuthority.name` — `"MSDK"` is the only value under which a stick command means
 * anything (`GcsMirror.authorityCode`, the recorder's own words).
 */
data class VirtualStickSnapshot(
    val enabled: Boolean,
    val advanced: Boolean,
    val authority: String?,
)

/**
 * The four RC stick positions, raw SDK units `[-660, 660]` (`record/LogEntry.RcStick`), null
 * where DJI has delivered null — component-gone, which while engaged is abort material, not a
 * zero.
 */
data class RcSticks(
    val leftHorizontal: Int?,
    val leftVertical: Int?,
    val rightHorizontal: Int?,
    val rightVertical: Int?,
) {
    fun allPresent(): Boolean =
        leftHorizontal != null && leftVertical != null && rightHorizontal != null && rightVertical != null

    /** Largest deflection as a fraction of full travel, or null if any axis is missing. */
    fun maxDeflection(): Double? {
        if (!allPresent()) return null
        val m = maxOf(
            kotlin.math.abs(leftHorizontal!!), kotlin.math.abs(leftVertical!!),
            kotlin.math.abs(rightHorizontal!!), kotlin.math.abs(rightVertical!!),
        )
        return m / FULL_DEFLECTION
    }

    companion object {
        /** `IStick`'s documented range. Mirrors `LogEntry.RcStick.FULL_DEFLECTION`. */
        const val FULL_DEFLECTION = 660.0
    }
}
