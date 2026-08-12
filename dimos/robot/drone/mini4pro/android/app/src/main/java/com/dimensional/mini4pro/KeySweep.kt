package com.dimensional.mini4pro

import android.util.Log
import dji.sdk.keyvalue.key.AirLinkKey
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.KeyManager

/**
 * Read-only sweep of every declared key, so one powered-on session captures the
 * whole surface instead of only the keys we thought to ask for.
 *
 * SAFETY: this performs **cache reads only** — `KeyManager.getValue(key)`. It
 * never calls `setValue` and never calls `performAction`, so it cannot command
 * the aircraft. Action keys are skipped by construction: `DJIActionKeyInfo` is
 * not a `DJIKeyInfo`, and we only reflect over `DJIKeyInfo` fields.
 *
 *   adb shell am start -n com.dimensional.mini4pro/.MainActivity --ez sweep true
 *   adb logcat -s KeySweep:V
 */
object KeySweep {

    private const val TAG = "KeySweep"

    private val groups: List<Pair<String, Class<*>>> = listOf(
        "FC" to FlightControllerKey::class.java,
        "Battery" to BatteryKey::class.java,
        "Product" to ProductKey::class.java,
        "RC" to RemoteControllerKey::class.java,
        "AirLink" to AirLinkKey::class.java,
        "Gimbal" to GimbalKey::class.java,
        "Camera" to CameraKey::class.java,
    )

    /** Gimbal and camera keys are per-component; the Mini 4 Pro's single unit is the main one. */
    private val componentGroups = setOf("Gimbal", "Camera")

    fun run() {
        val km = KeyManager.getInstance()
        var total = 0
        var readable = 0

        for ((groupName, clazz) in groups) {
            // getFields() includes inherited public statics, which is where the
            // bulk of the keys live (FlightControllerKey declares ~15; the rest
            // are on DJIFlightControllerKey).
            val infos = clazz.fields
                .filter { DJIKeyInfo::class.java.isAssignableFrom(it.type) }
                .sortedBy { it.name }

            Log.i(TAG, "── $groupName: ${infos.size} keys ──")

            for (field in infos) {
                total++
                @Suppress("UNCHECKED_CAST")
                val info = runCatching { field.get(null) as? DJIKeyInfo<Any> }.getOrNull() ?: continue
                if (!info.isCanGet) continue // skip write-only / action-ish keys entirely

                val key = runCatching {
                    if (groupName in componentGroups) {
                        KeyTools.createKey(info, ComponentIndexType.LEFT_OR_MAIN)
                    } else {
                        KeyTools.createKey(info)
                    }
                }.getOrNull() ?: continue

                val supported = runCatching { km.isKeySupported(key) }.getOrDefault(false)
                val value = runCatching { km.getValue(key) }.getOrNull()
                if (value != null) {
                    readable++
                    Log.i(TAG, "$groupName.${field.name} = $value  <${value.javaClass.simpleName}>  supported=$supported")
                } else if (supported) {
                    // Supported but empty is itself information: the key exists on
                    // this airframe but has no cached value yet.
                    Log.i(TAG, "$groupName.${field.name} = <null>  supported=true")
                }
            }
        }
        Log.i(TAG, "── sweep done: $readable readable of $total declared ──")
    }
}
