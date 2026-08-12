package com.dimensional.mini4pro

import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.v5.manager.KeyManager

/**
 * One-shot dump of the keys whose semantics the DJI docs leave ambiguous —
 * altitude datums, velocity units, attitude sign conventions, battery current
 * sign. See docs/msdk-keys.md §E for the list this is meant to close.
 *
 * These questions cannot be answered from documentation; they need the real
 * aircraft. Trigger with:
 *   adb shell am start -n com.dimensional.mini4pro/.MainActivity --ez probe true
 * then read `adb logcat -s KeyProbe:V`.
 *
 * Values are logged raw, with their runtime class, so nothing is hidden behind
 * a conversion we are trying to verify.
 */
object KeyProbe {

    private const val TAG = "KeyProbe"

    /** Holder object so every probe subscription can be cancelled in one call. */
    private val holder = Any()

    fun run() {
        Log.i(TAG, "── probe start ── compare against: aircraft on the ground, known site elevation")

        // Identity, and telling "RC attached" from "aircraft powered" (§C).
        watch("ProductType", ProductKey.KeyProductType)
        watch("Product.Connection", ProductKey.KeyConnection)
        watch("FC.Connection  (aircraft powered)", FlightControllerKey.KeyConnection)
        watch("RC.Type", RemoteControllerKey.KeyRemoteControllerType)

        // §E-1..4: which altitude is which datum. On the ground, a takeoff-relative
        // value reads ~0 while an AMSL value reads roughly the site elevation.
        watch("FC.AircraftLocation3D", FlightControllerKey.KeyAircraftLocation3D)
        watch("FC.Altitude", FlightControllerKey.KeyAltitude)
        watch("FC.TakeoffLocationAltitude", FlightControllerKey.KeyTakeoffLocationAltitude)
        watch("FC.HeightAboveSeaLevel", FlightControllerKey.KeyHeightAboveSeaLevel)
        watch("FC.UltrasonicHeight", FlightControllerKey.KeyUltrasonicHeight)

        // §E-5..8: attitude signs, velocity units/frame, heading source.
        watch("FC.AircraftAttitude", FlightControllerKey.KeyAircraftAttitude)
        watch("FC.AircraftVelocity", FlightControllerKey.KeyAircraftVelocity)
        watch("FC.CompassHeading", FlightControllerKey.KeyCompassHeading)

        // §E-14: which flight modes this airframe actually reports.
        watch("FC.FCFlightMode", FlightControllerKey.KeyFCFlightMode)
        watch("FC.FlightModeString", FlightControllerKey.KeyFlightModeString)
        watch("FC.IsFlying", FlightControllerKey.KeyIsFlying)
        watch("FC.AreMotorsOn", FlightControllerKey.KeyAreMotorsOn)

        // GPS: mis-reading the level hides the aircraft, since QGC drops
        // GPS_RAW_INT positions below a 3D fix.
        watch("FC.GPSSatelliteCount", FlightControllerKey.KeyGPSSatelliteCount)
        watch("FC.GPSSignalLevel", FlightControllerKey.KeyGPSSignalLevel)
        watch("FC.HomeLocation", FlightControllerKey.KeyHomeLocation)

        // §E-10..13: battery scaling and current sign (discharge is negative on DJI).
        watch("B.Voltage", BatteryKey.KeyVoltage)
        watch("B.Current", BatteryKey.KeyCurrent)
        watch("B.ChargeRemainingInPercent", BatteryKey.KeyChargeRemainingInPercent)
        watch("B.CellVoltages", BatteryKey.KeyCellVoltages)
        watch("B.NumberOfCells", BatteryKey.KeyNumberOfCells)
        watch("B.BatteryTemperature", BatteryKey.KeyBatteryTemperature)
    }

    fun stop() {
        KeyManager.getInstance().cancelListen(holder)
        Log.i(TAG, "── probe stopped ──")
    }

    /**
     * Subscribes with getOnce so slow-changing keys report immediately rather
     * than waiting for a change that may never come.
     */
    private fun <T> watch(label: String, info: DJIKeyInfo<T>) {
        val key = KeyTools.createKey(info)
        val supported = runCatching { KeyManager.getInstance().isKeySupported(key) }
            .getOrElse { "?" }
        KeyManager.getInstance().listen(key, holder, true) { _, newValue ->
            Log.i(TAG, "$label = ${describe(newValue)}   [supported=$supported]")
        }
    }

    private fun describe(value: Any?): String = when (value) {
        null -> "null  (component absent, or no value yet)"
        else -> "$value  <${value.javaClass.simpleName}>"
    }
}
