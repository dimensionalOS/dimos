package com.dimensional.mini4pro.mavlink

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.util.Log
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * The Android edge of `wifi-fix.md` gotcha #2: watches for a WiFi [Network] via
 * ConnectivityManager so `Bridge` can bind the MAVLink socket to it.
 *
 * Deliberately as thin as `LinkLocks`: register a TRANSPORT_WIFI callback, remember
 * the latest network, forward events. Everything with a decision in it
 * lives in [WifiBind] / [WifiBindTracker] where the JVM tests are; this class is
 * not unit-tested and must stay dull enough for that to be honest.
 *
 * The request is transport-only — **not** `NET_CAPABILITY_VALIDATED` — because the
 * failure this exists to survive is precisely a WiFi network that flunks Android's
 * validation (gotcha #1: pfSense only validates the phone on 10.55.1.15). A
 * validated-only request would reject the network in exactly the situation the
 * binding is for.
 */
internal class WifiNetworkGate(
    context: Context,
    /** Every WiFi onAvailable, on the ConnectivityManager thread. The Network is
     *  passed whole because a rebind needs `Network.bindSocket`, not just the id. */
    private val onWifiAvailable: (Network) -> Unit,
    /** Every WiFi onLost, on the ConnectivityManager thread. */
    private val onWifiLost: (Network) -> Unit,
) {

    private val cm = context.applicationContext
        .getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager

    /**
     * Latest WiFi network seen — what a start() should bind to. Not necessarily the
     * one an already-bound socket is on; [WifiBindTracker] owns that comparison.
     */
    @Volatile private var latest: Network? = null

    private val firstAvailable = CountDownLatch(1)
    private var callback: ConnectivityManager.NetworkCallback? = null

    fun start() {
        if (callback != null) return
        val cb = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: Network) {
                latest = network
                firstAvailable.countDown()
                onWifiAvailable(network)
            }

            override fun onLost(network: Network) {
                if (network == latest) latest = null
                onWifiLost(network)
            }
        }
        callback = cb
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .build()
        cm.registerNetworkCallback(request, cb)
    }

    /**
     * The WiFi network to bind to, waiting up to [timeoutMs] for the callback.
     * ConnectivityManager reports even an already-up network asynchronously, so an
     * immediate null proves nothing — hence the wait: milliseconds in the normal
     * case, the full timeout only when there is genuinely no WiFi.
     *
     * Callbacks arrive on ConnectivityManager's own thread, so blocking the caller
     * here cannot deadlock the delivery that releases it.
     */
    fun awaitNetwork(timeoutMs: Long): Network? {
        try {
            firstAvailable.await(timeoutMs, TimeUnit.MILLISECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        return latest
    }

    fun stop() {
        callback?.let {
            try {
                cm.unregisterNetworkCallback(it)
            } catch (e: Exception) {
                Log.w(TAG, "unregister failed", e)
            }
        }
        callback = null
        latest = null
    }

    private companion object {
        const val TAG = "WifiNetworkGate"
    }
}
