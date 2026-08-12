package com.dimensional.mini4pro.video

import java.net.Inet4Address
import java.net.NetworkInterface

/**
 * Which IP a GCS should dial to reach the RTSP server on this phone.
 *
 * MSDK's RTSP server binds a port but never tells you an address, so the URL has
 * to be assembled from the phone's own interfaces. Getting this wrong is another
 * silent failure: the server is genuinely up, the URL just points nowhere.
 *
 * The phone in the test rig has a mobile-data interface (`rmnet_data*`) as well
 * as WiFi, and the mobile address is useless to a GCS on the LAN — so interface
 * choice is a real decision, not a "take the first one".
 *
 * [pick] is pure and unit-tested; [current] is the thin `java.net` wrapper.
 */
object LocalAddress {

    data class Nic(val name: String, val address: String)

    /**
     * Best candidate for a LAN-reachable address, or null if the phone has none.
     *
     * Preference order:
     *  1. WiFi station (`wlan*`) — how the rig is wired: phone and QGC on the same /24.
     *  2. WiFi tethering / AP (`ap*`, `swlan*`) — works if QGC joins the phone's hotspot.
     *  3. USB tethering / other LAN-ish interfaces.
     *  4. Mobile data (`rmnet*`, `ccmni*`, `pdp*`) — carrier NAT, a GCS cannot reach it.
     *
     * Loopback, link-local (169.254/16) and non-IPv4 candidates are dropped
     * outright rather than ranked: none of them can serve a GCS.
     */
    fun pick(candidates: List<Nic>): Nic? =
        candidates.filter { usable(it.address) }
            .minByOrNull { score(it.name) }

    /** Enumerates this device's interfaces. Android needs no permission for this. */
    fun current(): Nic? = pick(enumerate())

    fun enumerate(): List<Nic> = runCatching {
        NetworkInterface.getNetworkInterfaces()?.toList().orEmpty()
            .filter { runCatching { it.isUp }.getOrDefault(false) }
            .flatMap { nif ->
                nif.inetAddresses.toList()
                    .filterIsInstance<Inet4Address>()
                    .map { Nic(nif.name ?: "?", it.hostAddress ?: "") }
            }
    }.getOrDefault(emptyList())

    /** Lower is better. */
    internal fun score(name: String): Int {
        val n = name.lowercase()
        return when {
            n.startsWith("wlan") -> 0
            n.startsWith("ap") || n.startsWith("swlan") || n.startsWith("softap") -> 10
            n.startsWith("rndis") || n.startsWith("usb") -> 20
            n.startsWith("eth") -> 30
            // Carrier-NATed: reachable from nothing. Last resort, but still better
            // than reporting "no address at all".
            n.startsWith("rmnet") || n.startsWith("ccmni") || n.startsWith("pdp") ||
                n.startsWith("tun") || n.startsWith("clat") -> 90
            else -> 50
        }
    }

    internal fun usable(address: String): Boolean {
        if (address.isBlank()) return false
        val octets = address.split('.')
        if (octets.size != 4) return false
        val nums = octets.map { it.toIntOrNull() ?: return false }
        if (nums.any { it !in 0..255 }) return false
        if (nums[0] == 127) return false // loopback
        if (nums[0] == 169 && nums[1] == 254) return false // link-local, no DHCP
        if (nums[0] == 0) return false
        return true
    }
}
