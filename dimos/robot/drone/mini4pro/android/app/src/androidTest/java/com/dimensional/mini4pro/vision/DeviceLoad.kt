package com.dimensional.mini4pro.vision

import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import android.os.SystemClock
import java.io.File

/**
 * **What the phone is paying**, sampled from the phone itself.
 *
 * Detection is the first thing this project has proposed that burns CPU continuously, and the
 * thermal argument against it is not hypothetical: this airframe forced three overheat go-homes in
 * one afternoon (`docs/measurements/2026-07-26-stage-a-first-engagement.md`), and the phone that
 * would run the detector sits in the sun clamped to a controller. So "what does it cost" has to be
 * a measurement with a number and a unit, not a shrug — which is what makes it a design input to
 * the arming rule rather than a curiosity.
 *
 * ## Why these three quantities and not others
 *
 * **Process CPU** comes from `/proc/self/stat`, which is always readable by the process itself and
 * needs no permission. Fields 14 and 15 are `utime` and `stime` in clock ticks; the sum over an
 * interval, divided by the interval, is *cores used* — so 2.0 means two cores saturated, and on
 * this eight-core phone the ceiling is 8.0. That is the number that matters for headroom, and it
 * is deliberately not normalised to a percentage, because "300 % CPU" is a unit that has confused
 * every reader it has ever met.
 *
 * **Battery temperature** comes from the sticky `ACTION_BATTERY_CHANGED` broadcast, in tenths of a
 * degree Celsius. It is slow, it lags the die by minutes, and it is the *right* signal anyway: the
 * question is whether sustained detection cooks the phone over a flight, not what a core peaks at
 * for one frame. It is also the only temperature guaranteed readable without root or a permission.
 *
 * **Thermal zones** under `/sys/class/thermal` are read too, because on this device they expose
 * per-cluster and GPU die temperatures which lead the battery by minutes. They are attempted
 * rather than relied on: SELinux may refuse an app process what the shell is allowed, and a
 * measurement harness that dies because a sysfs file was unreadable would have told us nothing at
 * all. [thermalZones] returns empty when that happens and the caller reports the absence.
 */
internal class DeviceLoad(private val context: Context) {

    /** One instant's picture of what the device is doing. */
    data class Sample(
        val atMillis: Long,
        /** Clock ticks of CPU this process has used since it started (utime + stime). */
        val processTicks: Long,
        /** Battery temperature in °C, or null if the broadcast had nothing. */
        val batteryCelsius: Double?,
        /** Zone name → °C, empty if sysfs is not readable from this process. */
        val zones: Map<String, Double>,
    )

    /**
     * Cores used and degrees gained between two samples.
     *
     * `coresUsed` is `Δticks / HZ / Δseconds`. [android.os.Process] does not expose `HZ`, and it
     * has been 100 on every Android device ever shipped — but it is stated here as an assumption
     * rather than buried, because a wrong `HZ` would scale every CPU figure in the report.
     */
    data class Delta(
        val seconds: Double,
        val coresUsed: Double,
        val batteryRiseCelsius: Double?,
        val zoneRiseCelsius: Map<String, Double>,
    )

    fun sample(): Sample = Sample(
        atMillis = SystemClock.elapsedRealtime(),
        processTicks = processTicks(),
        batteryCelsius = batteryCelsius(),
        zones = thermalZones(),
    )

    fun delta(from: Sample, to: Sample): Delta {
        val seconds = (to.atMillis - from.atMillis) / 1000.0
        val cores =
            if (seconds <= 0.0) 0.0 else (to.processTicks - from.processTicks) / CLOCK_TICKS_PER_SECOND / seconds
        return Delta(
            seconds = seconds,
            coresUsed = cores,
            batteryRiseCelsius =
                if (from.batteryCelsius == null || to.batteryCelsius == null) null
                else to.batteryCelsius - from.batteryCelsius,
            zoneRiseCelsius = to.zones.mapNotNull { (k, v) ->
                from.zones[k]?.let { k to (v - it) }
            }.toMap(),
        )
    }

    private fun processTicks(): Long = runCatching {
        val stat = File("/proc/self/stat").readText()
        // The second field is the executable name in parentheses and may itself contain spaces,
        // so the split must start after the closing paren — the classic /proc/stat trap.
        val after = stat.substring(stat.lastIndexOf(')') + 2)
        val parts = after.split(' ')
        // After the trimmed prefix, field 14 (utime) is index 11 and field 15 (stime) index 12.
        parts[11].toLong() + parts[12].toLong()
    }.getOrDefault(0L)

    private fun batteryCelsius(): Double? = runCatching {
        @Suppress("DEPRECATION")
        val intent: Intent? =
            context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
        val tenths = intent?.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, Int.MIN_VALUE)
        if (tenths == null || tenths == Int.MIN_VALUE) null else tenths / 10.0
    }.getOrNull()

    /**
     * Every readable thermal zone whose type is one we care about, in °C.
     *
     * The filter is deliberate: this device publishes over sixty zones, most of them modem and
     * per-core duplicates, and a report listing all of them hides the three that matter.
     */
    fun thermalZones(): Map<String, Double> = runCatching {
        val root = File("/sys/class/thermal")
        val out = LinkedHashMap<String, Double>()
        root.listFiles { f -> f.name.startsWith("thermal_zone") }?.sortedBy { it.name }?.forEach { zone ->
            val type = runCatching { File(zone, "type").readText().trim() }.getOrNull() ?: return@forEach
            if (INTERESTING.none { type.startsWith(it) }) return@forEach
            val milli = runCatching { File(zone, "temp").readText().trim().toDouble() }.getOrNull()
                ?: return@forEach
            // Zones report milli-degrees on this device; some report degrees. 200 °C is not a
            // temperature a phone survives, so it is a safe discriminator.
            out[type] = if (milli > 200) milli / 1000.0 else milli
        }
        out
    }.getOrDefault(emptyMap())

    private companion object {
        /** `sysconf(_SC_CLK_TCK)`. 100 on every Android device; stated, not hidden. */
        const val CLOCK_TICKS_PER_SECOND = 100.0

        /** Big-core cluster, GPU, and battery/skin, which is what throttles a phone in the sun. */
        val INTERESTING = listOf("cpu-1-", "gpuss-0", "gpuss-1", "battery", "skin", "quiet-therm")
    }
}
