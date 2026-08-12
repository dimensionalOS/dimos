package com.dimensional.mini4pro.mavlink

import java.io.IOException
import java.net.DatagramSocket

/**
 * The WiFi-binding policy for the MAVLink socket — the pure half of `wifi-fix.md`
 * gotcha #2.
 *
 * When the phone fails the router's network validation, Android 16 silently moves
 * the *default* network to cellular, and an unbound UDP socket follows it: flight
 * telemetry addressed to a LAN host leaves over LTE and dies ("net unreachable"
 * from a carrier IP — this has actually happened). Binding the socket to the WiFi
 * `android.net.Network` makes the route explicit, so a default-network flap can
 * never hijack the telemetry path.
 *
 * This file deliberately imports no `android.net`: the decisions and the status
 * strings live here, where the JVM tests can reach them, and `WifiNetworkGate` /
 * `Bridge` supply the actual `Network` at the edge — the same seam rule as
 * `TelemetryEncoder` vs `StateCache`.
 */
object WifiBind {

    /**
     * How long `Bridge.start` waits for the WiFi callback before refusing. With
     * WiFi associated the callback arrives within milliseconds, so this is only
     * ever paid in full on the refusal path. It exists because ConnectivityManager
     * reports the current network asynchronously — "no callback yet" and "no WiFi"
     * are indistinguishable until we have waited.
     */
    const val WAIT_MS = 2000L

    /**
     * Wraps [bind] — `Network::bindSocket` on the Android side — into the socket
     * factory `MavlinkLink` takes.
     *
     * A factory that throws must not leak: the socket is closed before the failure
     * propagates, because the caller only ever learns about a socket the factory
     * returned.
     */
    fun socketFactory(bind: (DatagramSocket) -> Unit): (Int) -> DatagramSocket = { localPort ->
        val socket = DatagramSocket(localPort)
        try {
            bind(socket)
        } catch (e: Exception) {
            socket.close()
            throw IOException("binding MAVLink socket to WiFi failed: ${e.message}", e)
        }
        socket
    }

    /**
     * Status line for a start that was refused because no WiFi network exists.
     *
     * Refusing is the safe *and* honest option: starting unbound would send
     * telemetry for a LAN host out over cellular — always wrong — and a link that
     * looks up while doing that is this project's least favourite failure.
     */
    fun refusalStatus(waitedMs: Long): String =
        "REFUSED: no WiFi network after ${waitedMs}ms — an unbound socket would send " +
            "telemetry over cellular, so the bridge did not start. Join WiFi, then start again."
}

/**
 * Turns ConnectivityManager events into rebind decisions and operator-facing
 * status lines, keyed on the one network the socket is actually bound to. Pure,
 * so all of it is unit-tested; `Bridge` feeds it events, performs whatever swap it
 * calls for, and publishes whatever string it returns.
 *
 * The two non-obvious facts it encodes:
 *
 * 1. A WiFi network that comes back after a loss is a NEW `Network` (netIds are
 *    not reused), so the socket — still bound to the dead one — will never
 *    transmit again. The link must swap sockets; nothing recovers by waiting.
 * 2. Rebinding is **link maintenance, not an autonomous aircraft action**. The Q4
 *    rule (`docs/decisions/2026-07-25-m2-command-safety.md`) forbids unrequested
 *    *aircraft commands*; keeping our own transport alive is the opposite — a
 *    bridge that silently stays dark after a 2 s AP blip, on a phone mounted to
 *    the RC where nobody can reach the screen, is the failure, not the fix.
 *
 * Networks are compared as strings (`Network.toString()` is the netId) so this
 * class never has to import `android.net`.
 */
class WifiBindTracker {

    // Written from the starter thread (bound) and the ConnectivityManager thread
    // (lost/shouldRebind/rebound); ConnectivityManager delivers its callbacks
    // sequentially, so volatile is enough.
    @Volatile private var bound: String? = null

    @Volatile private var lostBound = false

    /** Call once the socket is bound. Returns the status line to display. */
    fun bound(network: String): String {
        bound = network
        lostBound = false
        return "bound to WiFi network $network"
    }

    /**
     * A WiFi network disappeared. Returns the warning to surface, or null when the
     * lost network is not the one the socket is bound to.
     */
    fun lost(network: String): String? {
        if (network != bound) return null
        lostBound = true
        return "LOST WiFi network $network — telemetry is dark; will rebind " +
            "automatically when WiFi returns."
    }

    /**
     * A WiFi network appeared: should the link swap its socket onto it? True only
     * when the bound network was lost — the initial callback for the network we
     * are about to bind is not news — and only while the link runs: with the link
     * down there is no socket to maintain, and a rebind would resurrect one.
     *
     * Deciding false does NOT consume the pending loss: a stopped-link answer now
     * must not eat the rebind a running link is owed on the next event.
     */
    fun shouldRebind(network: String, linkRunning: Boolean): Boolean =
        linkRunning && lostBound

    /** The swap succeeded: [network] is now the bound one. Returns the status line. */
    fun rebound(network: String): String {
        val old = bound
        bound = network
        lostBound = false
        return "REBOUND to WiFi network $network after losing $old — telemetry path restored"
    }

    /**
     * The swap failed. The loss stays pending — a later network event may still
     * succeed — and the status falls back to honest report-only: the operator is
     * told the socket is dead and that restarting the bridge is the manual out.
     */
    fun rebindFailed(network: String, error: String?): String =
        "WiFi returned as network $network but rebinding failed (${error ?: "unknown"}) — " +
            "socket still bound to dead network $bound; restart the bridge."
}
