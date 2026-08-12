package com.dimensional.mini4pro

import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.flightcontroller.FCFlightMode
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel
import dji.sdk.keyvalue.value.flightcontroller.HeightAboveSeaLevelMsg
import dji.sdk.keyvalue.value.product.ProductType
import android.os.SystemClock
import dji.v5.manager.KeyManager
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Plausible
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal

/**
 * The single source of aircraft state, fed only by KeyManager listeners.
 *
 * Three rules from docs/msdk-keys.md that shape this:
 *  - A `null` newValue is DJI's *component-gone* signal, not zero. So every
 *    field is nullable and null means "no valid reading", never 0.
 *    **But the converse does not hold: non-null is not the same as valid.** With
 *    no home point recorded, `KeyHomeLocation` returns a populated
 *    `LocationCoordinate2D` full of filler rather than null — measured
 *    2026-07-26, see [Geo]. Coordinates are therefore range-checked here, on the
 *    way into [AircraftState], which is where "null means unknown" has to become
 *    true rather than merely intended. [Plausible] extends the same rule to the
 *    other readings a filler value could reach the wire through — attitude,
 *    velocity, the two altitudes, battery percentage and satellite count — and
 *    records which keys are deliberately left unchecked.
 *  - Do no work in the listener: write and return. Encoding and sending happen
 *    on the telemetry thread, which snapshots via [snapshot].
 *  - Every key updates on its own schedule, and some barely update at all —
 *    `KeyAircraftVelocity` fired once in 35 s in the ground probe. So each
 *    callback also stamps a **delivery time** ([deliveredAtMs]), and
 *    [aircraftState] turns those into per-signal ages. Stamping on *delivery*
 *    rather than on *change* is the whole point: DJI re-sending an unchanged
 *    value is a live feed, and only a feed that stops is stale.
 */
object StateCache {

    /** Immutable snapshot so the telemetry thread never reads a half-updated set. */
    data class Snapshot(
        val fcConnected: Boolean? = null,
        val productType: ProductType? = null,
        val location: LocationCoordinate3D? = null,
        val altitude: Double? = null,
        val heightAboveSeaLevel: HeightAboveSeaLevelMsg? = null,
        val takeoffAltitude: Double? = null,
        val attitude: Attitude? = null,
        val velocity: Velocity3D? = null,
        val compassHeading: Double? = null,
        val satelliteCount: Int? = null,
        val gpsSignalLevel: GPSSignalLevel? = null,
        val homeLocation: LocationCoordinate2D? = null,
        val homeLocationSet: Boolean? = null,
        val isFlying: Boolean? = null,
        val motorsOn: Boolean? = null,
        val flightMode: FCFlightMode? = null,
        val notAllowMotorStart: Boolean? = null,
        val imuWarmingUp: Boolean? = null,
        val inFailsafe: Boolean? = null,
        val goHomeHeight: Int? = null,
        val batteryPercent: Int? = null,
        val voltageMv: Int? = null,
        val currentMa: Int? = null,
        val cellCount: Int? = null,
        val cellVoltagesMv: List<Int>? = null,
        val batteryTempC: Double? = null,
    )

    private val lock = Any()
    private var current = Snapshot()

    /**
     * The monotonic instant each [Signal] was last **delivered**, indexed by
     * ordinal, or [NEVER]. Guarded by [lock] alongside [current], so a snapshot
     * and its ages describe the same instant.
     *
     * A flat `LongArray` rather than a map because it is written from a
     * `KeyManager` callback at up to ~50 Hz: one array store is about as close to
     * "write and return" as the rule allows.
     */
    private val deliveredAtMs = LongArray(Signal.entries.size) { NEVER }

    /** Sentinel for "this signal has never been delivered". */
    private const val NEVER = Long.MIN_VALUE

    /**
     * The clock everything is stamped and aged against. Injected so an
     * instrumented test never has to sleep, and **monotonic on purpose**: a wall
     * clock stepped by NTP mid-flight would turn a fresh reading into an hour-old
     * one. This is the same clock family the flight recorder uses for `t`.
     */
    @Volatile
    var nowMs: () -> Long = { SystemClock.elapsedRealtime() }

    /**
     * Projects the DJI-typed snapshot into the plain-Kotlin model the encoder
     * consumes. This is the only place DJI value types are unwrapped, which is
     * what keeps the conversion arithmetic unit-testable.
     */
    fun aircraftState(): AircraftState {
        val now = nowMs()
        val s: Snapshot
        val stamps: LongArray
        // One critical section for both, so the ages describe the values beside
        // them rather than a set that moved on between two reads.
        synchronized(lock) {
            s = current
            stamps = deliveredAtMs.copyOf()
        }
        // Both coordinate pairs go through the same validity rule, and both are
        // resolved once so latitude and longitude cannot be accepted separately.
        val position = Geo.coordinateOrNull(s.location?.latitude, s.location?.longitude)
        val home = Geo.coordinateOrNull(s.homeLocation?.latitude, s.homeLocation?.longitude)
        // The same argument as the coordinates, applied to the readings that can
        // also carry a value DJI never measured — see [Plausible] for which ones
        // and why the rest are left alone. Attitude and velocity are resolved as
        // sets because DJI delivers each as one struct.
        val attitude = Plausible.attitudeOrNull(s.attitude?.roll, s.attitude?.pitch, s.attitude?.yaw)
        val velocity = Plausible.velocityOrNull(s.velocity?.x, s.velocity?.y, s.velocity?.z)
        return AircraftState(
            ages = agesFrom(now, stamps),
            fcConnected = s.fcConnected == true,
            latitude = position?.first,
            longitude = position?.second,
            relativeAltitude = Plausible.altitudeMOrNull(s.altitude),
            takeoffAltitudeAmsl = Plausible.altitudeMOrNull(s.takeoffAltitude),
            rollDeg = attitude?.roll,
            pitchDeg = attitude?.pitch,
            yawDeg = attitude?.yaw,
            velocityNorth = velocity?.north,
            velocityEast = velocity?.east,
            velocityDown = velocity?.down,
            satelliteCount = Plausible.satelliteCountOrNull(s.satelliteCount),
            gpsSignalLevel = s.gpsSignalLevel?.let(::gpsLevelToInt),
            homeLatitude = home?.first,
            homeLongitude = home?.second,
            homeLocationSet = s.homeLocationSet,
            isFlying = s.isFlying,
            motorsOn = s.motorsOn,
            flightMode = s.flightMode?.name,
            notAllowMotorStart = s.notAllowMotorStart,
            imuWarmingUp = s.imuWarmingUp,
            inFailsafe = s.inFailsafe,
            // NOT plausibility-checked, and that is a gap rather than a decision:
            // goHomeHeight landed while the filler-value audit was already sweeping, so it was
            // never classified (docs/measurements/2026-07-26-filler-value-audit.md). It is a
            // scalar, so it cannot be struct filler — the failure mode that audit found — but it
            // is published to the operator as RTL_RETURN_ALT and a wrong value there is a lie
            // about how high the aircraft will climb on its way home. Classify it next.
            goHomeHeightM = s.goHomeHeight,
            batteryPercent = Plausible.batteryPercentOrNull(s.batteryPercent),
            voltageMv = s.voltageMv,
            currentMa = s.currentMa,
            cellCount = s.cellCount,
            // Defensive copy: the SDK hands us a mutable List and AircraftState is
            // read on the telemetry thread.
            cellVoltagesMv = s.cellVoltagesMv?.toList(),
            batteryTempC = s.batteryTempC,
        )
    }

    /** Delivery instants → ages. A signal never delivered stays absent. */
    private fun agesFrom(nowMs: Long, stamps: LongArray): SampleAges {
        val delivered = HashMap<Signal, Long>(stamps.size * 2)
        for (signal in Signal.entries) {
            val at = stamps[signal.ordinal]
            if (at != NEVER) delivered[signal] = at
        }
        return SampleAges.since(nowMs, delivered)
    }

    /**
     * Measured range on the Mini 4 Pro is LEVEL_0..LEVEL_5 (LEVEL_5 seen with a
     * good fix). LEVEL_NONE/UNKNOWN mean "no information" and must map to null,
     * not to 0 — 0 is a real level meaning "worst", and conflating them would
     * report a bad fix as a good one's absence or vice versa.
     */
    private fun gpsLevelToInt(level: GPSSignalLevel): Int? = when (level) {
        GPSSignalLevel.LEVEL_0 -> 0
        GPSSignalLevel.LEVEL_1 -> 1
        GPSSignalLevel.LEVEL_2 -> 2
        GPSSignalLevel.LEVEL_3 -> 3
        GPSSignalLevel.LEVEL_4 -> 4
        GPSSignalLevel.LEVEL_5 -> 5
        else -> null // LEVEL_10, LEVEL_NONE, UNKNOWN
    }

    /** One holder for the whole group, so teardown is a single cancelListen. */
    private val holder = Any()

    fun snapshot(): Snapshot = synchronized(lock) { current }

    /**
     * Must be called only after MSDK registration completes — subscribing
     * earlier silently does nothing.
     */
    fun start() {
        // KeyProductType is the one subscription with no Signal: it does not reach
        // AircraftState, so there is no value whose age anyone could ask about.
        sub(ProductKey.KeyProductType, null) { copy(productType = it) }
        sub(FlightControllerKey.KeyConnection, Signal.FC_CONNECTION) { copy(fcConnected = it) }

        sub(FlightControllerKey.KeyAircraftLocation3D, Signal.POSITION) { copy(location = it) }
        sub(FlightControllerKey.KeyAltitude, Signal.ALTITUDE) { copy(altitude = it) }
        // Unusable in practice (quality VERY_BAD on the ground probe) and not
        // projected into AircraftState, so it carries no Signal either.
        sub(FlightControllerKey.KeyHeightAboveSeaLevel, null) { copy(heightAboveSeaLevel = it) }
        sub(FlightControllerKey.KeyTakeoffLocationAltitude, Signal.TAKEOFF_ALTITUDE) {
            copy(takeoffAltitude = it)
        }

        sub(FlightControllerKey.KeyAircraftAttitude, Signal.ATTITUDE) { copy(attitude = it) }
        sub(FlightControllerKey.KeyAircraftVelocity, Signal.VELOCITY) { copy(velocity = it) }
        sub(FlightControllerKey.KeyCompassHeading, null) { copy(compassHeading = it) }

        sub(FlightControllerKey.KeyGPSSatelliteCount, Signal.SATELLITES) { copy(satelliteCount = it) }
        sub(FlightControllerKey.KeyGPSSignalLevel, Signal.GPS_LEVEL) { copy(gpsSignalLevel = it) }
        sub(FlightControllerKey.KeyHomeLocation, Signal.HOME) { copy(homeLocation = it) }
        // DJI's own answer to "is there a home point?", and the reason the
        // coordinate above cannot be trusted on its own — see [Geo] and
        // AircraftState.homeLocationSet. Measured false for the whole of the
        // 2026-07-26 11:17 session while KeyHomeLocation returned filler.
        sub(FlightControllerKey.KeyIsHomeLocationSet, Signal.HOME_SET) {
            copy(homeLocationSet = it)
        }

        sub(FlightControllerKey.KeyIsFlying, Signal.FLYING) { copy(isFlying = it) }
        sub(FlightControllerKey.KeyAreMotorsOn, Signal.MOTORS) { copy(motorsOn = it) }
        sub(FlightControllerKey.KeyFCFlightMode, Signal.FLIGHT_MODE) { copy(flightMode = it) }
        sub(FlightControllerKey.KeyNotAllowMotorStart, Signal.MOTOR_START_VETO) {
            copy(notAllowMotorStart = it)
        }
        sub(FlightControllerKey.KeyIsIMUWarmingUp, Signal.IMU_WARMUP) { copy(imuWarmingUp = it) }
        sub(FlightControllerKey.KeyIsFailSafe, Signal.FAILSAFE) { copy(inFailsafe = it) }

        // The aircraft's configured return-to-home altitude, and the first *setting*
        // subscribed here rather than a measurement. getOnce is what makes it usable:
        // the key is not an event key (`setIsEvent(false)` in its static initialiser)
        // and nothing re-delivers it during a flight, so without getOnce it would stay
        // unknown until an operator happened to change it in DJI Fly.
        //
        // It is read by the parameter layer, not the telemetry encoder — see
        // `handshake/Parameters.AIRCRAFT_PARAMETERS`. Nothing on the telemetry wire
        // changes because of it.
        sub(FlightControllerKey.KeyGoHomeHeight, Signal.GO_HOME_HEIGHT) {
            copy(goHomeHeight = it)
        }

        sub(BatteryKey.KeyChargeRemainingInPercent, Signal.BATTERY_PERCENT) {
            copy(batteryPercent = it)
        }
        sub(BatteryKey.KeyVoltage, Signal.BATTERY_VOLTAGE) { copy(voltageMv = it) }
        sub(BatteryKey.KeyCurrent, Signal.BATTERY_CURRENT) { copy(currentMa = it) }
        sub(BatteryKey.KeyNumberOfCells, Signal.CELL_COUNT) { copy(cellCount = it) }
        sub(BatteryKey.KeyCellVoltages, Signal.CELL_VOLTAGES) { copy(cellVoltagesMv = it) }
        sub(BatteryKey.KeyBatteryTemperature, Signal.BATTERY_TEMP) { copy(batteryTempC = it) }
    }

    fun stop() {
        KeyManager.getInstance().cancelListen(holder)
        synchronized(lock) {
            current = Snapshot()
            // Back to "never delivered". Leaving old stamps would make the next
            // aircraftState() report ages measured against a previous session.
            deliveredAtMs.fill(NEVER)
        }
    }

    /**
     * Subscribes with getOnce so slow-changing keys (product type, home point,
     * cell count) report their current value instead of waiting for a change.
     *
     * [signal] is stamped on **every** callback — including one carrying the same
     * value as the last, and including one carrying `null`. A repeated value is a
     * live feed; a `null` is DJI telling us the component is gone, which is itself
     * a fresh fact. Pass null only for keys that do not reach `AircraftState`.
     */
    private fun <T> sub(info: DJIKeyInfo<T>, signal: Signal?, apply: Snapshot.(T?) -> Snapshot) {
        val key = KeyTools.createKey(info)
        KeyManager.getInstance().listen(key, holder, true) { _, newValue ->
            // Read the clock before taking the lock: it is a cheap monotonic call,
            // and it keeps the critical section to two stores.
            val at = nowMs()
            synchronized(lock) {
                current = current.apply(newValue)
                if (signal != null) deliveredAtMs[signal.ordinal] = at
            }
        }
    }
}
