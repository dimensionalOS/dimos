package com.dimensional.mini4pro.simulator

import com.dimensional.mini4pro.record.DjiOp
import com.dimensional.mini4pro.record.JsonObject
import com.dimensional.mini4pro.record.Tap

/**
 * A [SimulatorPort] that records every ask and every answer on the way past — the same decorator
 * as `command/RecordedActionPort` and justified the same way, so read that one first.
 *
 * ## Why the simulator earns a call record at all
 *
 * A record whose session was simulated is not a measurement, and `Bridge.recordSimulatorEvent`
 * already writes every phase transition as a `SEV_WARN` event so a skim cannot miss it. That
 * covers *what the simulator did*. What it does not cover is **what we asked it to do and whether
 * DJI agreed** — and the two come apart in exactly the way that matters: `SimulatorControl` holds
 * a `STARTING` phase precisely because a `performAction` acceptance is not the simulator being on,
 * and DJI can answer neither callback. Until now a start request that DJI silently dropped left
 * the record showing a `STARTING` that never resolved and no statement anywhere about why.
 *
 * ## The arguments are recorded, and that is the difference from an event line
 *
 * [start] carries the origin the whole simulated flight is anchored to. A simulator started at the
 * wrong coordinates produces a plausible, entirely wrong flight, and the number that would prove
 * it was previously only in a logcat line that is gone by the time anyone asks. `sats` is there
 * for the same reason: it is the input to the GPS quality the session's telemetry claims.
 *
 * They ride as pre-rendered JSON because only the caller knows the units — the [Tap] contract.
 */
class RecordedSimulatorPort(
    private val inner: SimulatorPort,
    private val tap: Tap,
) : SimulatorPort {

    override fun unavailableReason(): String? = inner.unavailableReason()

    override fun start(
        latitude: Double,
        longitude: Double,
        satelliteCount: Int,
        onSuccess: () -> Unit,
        onFailure: (String) -> Unit,
    ) {
        val call = tap.aircraftOut(
            DjiOp.SIM_START,
            argsJson = JsonObject.render { o ->
                // 7 decimals of degree ≈ 1.1 cm, the resolution the rest of this format carries.
                o.put("lat", latitude, 7)
                o.put("lon", longitude, 7)
                o.put("sats", satelliteCount)
            },
        )
        inner.start(
            latitude = latitude,
            longitude = longitude,
            satelliteCount = satelliteCount,
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

    override fun stop(onSuccess: () -> Unit, onFailure: (String) -> Unit) {
        val call = tap.aircraftOut(DjiOp.SIM_STOP)
        inner.stop(
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

    override fun listenIsSimulatorStarted(onDelivery: (Boolean?) -> Unit) =
        inner.listenIsSimulatorStarted(onDelivery)

    override fun listenSimulatorState(onDelivery: (SimulatedAircraft?) -> Unit) =
        inner.listenSimulatorState(onDelivery)

    override fun cancelListens() = inner.cancelListens()
}
