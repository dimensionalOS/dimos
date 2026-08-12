package com.dimensional.mini4pro.mavlink

import android.content.Context
import android.net.wifi.WifiManager
import android.os.PowerManager
import android.util.Log

/**
 * Keeps the WiFi radio and the CPU awake for as long as the MAVLink link is up.
 *
 * ## Why this exists
 *
 * Android puts the WiFi radio into power save by default. A station in power save
 * wakes only around beacons, so the access point buffers unicast traffic for it and
 * drops what does not fit. Outbound traffic is barely affected — a station can
 * transmit whenever it likes — which is what makes this so misleading: our
 * telemetry appears to flow while the ground station's messages vanish.
 *
 * ## What is and is not measured — read before trusting this
 *
 * These locks were added on 2026-07-25 while chasing QGroundControl's "Communication
 * Lost", and **that diagnosis was wrong**. The evidence is recorded honestly here so
 * nobody re-derives it:
 *
 * - The 90% ICMP loss that prompted this was dismissed at the time on the grounds
 *   that ICMP was blocked on that network. **That dismissal was itself wrong** —
 *   corrected 2026-07-25 ~22:20, the phone answers ping perfectly well. The 90%
 *   loss was real, and the cause was found much later: the WiFi access point
 *   isolates wireless clients from each other, so station-to-station traffic is
 *   dropped while anything via the gateway is clean. See the network section of
 *   `docs/dev-environment.md`. None of that is power save, and none of it is
 *   something a `WifiLock` can affect.
 * - The apparent drop in QGC's inbound heartbeats (16% → 3% after taking the locks)
 *   is **confounded** and shows nothing: QGC only emits heartbeats to a peer it
 *   believes is connected, and the app was being restarted repeatedly during both
 *   samples.
 * - Measured properly afterwards, the phone → ground path was **clean**: 607 frames
 *   in 607 datagrams over 25 s with HEARTBEAT at exactly 1.00 Hz, no loss. The real
 *   cause of the dropouts was restarting the app, which opens a new UDP source port
 *   and makes the GCS lose the peer it was tracking.
 *
 * So there is **no evidence these locks fixed anything**, and none that they hurt.
 * They are kept for a failure mode that is real but has not yet been tested: the
 * phone spends field sessions strapped to a controller with the screen off, and
 * `dumpsys wifi` confirmed `mPowerSaveDisableRequests 0` before and `2` after, so
 * the radio genuinely was subject to power save. Delete them if a screen-off session
 * shows they are unnecessary — do not keep them on the strength of the story above.
 *
 * ## Why both locks
 *
 * [WifiManager.WIFI_MODE_FULL_HIGH_PERF] is the only mode that actually disables
 * power save; `WIFI_MODE_FULL` has been a no-op since API 29. It keeps the radio
 * listening but says nothing about the CPU, so the partial wake lock is needed as
 * well or the emitter thread stops being scheduled once the screen finally does go
 * off — which is the normal state of a phone strapped to a controller.
 *
 * ## The cost, stated plainly
 *
 * This is a deliberate, large increase in power draw, and it is the right trade for
 * an aircraft link: a ground station that cannot hear the aircraft is worse than a
 * flat battery, and the phone is on the controller's USB power in flight anyway.
 * The locks are held **only while the bridge is running** — [release] is called from
 * `Bridge.stop`, so an idle app costs nothing.
 */
internal class LinkLocks(context: Context) {

    private val appContext = context.applicationContext

    private var wifiLock: WifiManager.WifiLock? = null
    private var wakeLock: PowerManager.WakeLock? = null

    /** Idempotent: acquiring twice would need releasing twice, so we never do. */
    @Synchronized
    fun acquire() {
        if (wifiLock != null || wakeLock != null) return
        try {
            val wifi = appContext.getSystemService(Context.WIFI_SERVICE) as? WifiManager
            wifiLock = wifi?.createWifiLock(
                WifiManager.WIFI_MODE_FULL_HIGH_PERF, "mini4pro:link",
            )?.apply {
                setReferenceCounted(false)
                acquire()
            }

            val power = appContext.getSystemService(Context.POWER_SERVICE) as? PowerManager
            wakeLock = power?.newWakeLock(
                PowerManager.PARTIAL_WAKE_LOCK, "mini4pro:link",
            )?.apply {
                setReferenceCounted(false)
                acquire()
            }
            Log.i(TAG, "held: wifi=${wifiLock != null} wake=${wakeLock != null}")
        } catch (e: Exception) {
            // Never take the link down over a lock: degraded comms beat no comms.
            Log.w(TAG, "could not acquire link locks", e)
        }
    }

    @Synchronized
    fun release() {
        try {
            wifiLock?.takeIf { it.isHeld }?.release()
            wakeLock?.takeIf { it.isHeld }?.release()
        } catch (e: Exception) {
            Log.w(TAG, "could not release link locks", e)
        } finally {
            wifiLock = null
            wakeLock = null
        }
    }

    private companion object {
        const val TAG = "LinkLocks"
    }
}
