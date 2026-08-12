package com.dimensional.mini4pro.video

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/**
 * Interface choice is a real decision on this phone: it has mobile data as well
 * as WiFi, and a carrier-NATed address in the RTSP URL means a server that is
 * genuinely up and unreachable.
 */
class LocalAddressTest {

    private fun nic(name: String, address: String) = LocalAddress.Nic(name, address)

    @Test
    fun `wifi beats mobile data`() {
        val picked = LocalAddress.pick(
            listOf(
                nic("rmnet_data0", "10.171.44.9"),
                nic("wlan0", "10.55.1.15"),
            )
        )
        assertEquals(nic("wlan0", "10.55.1.15"), picked)
    }

    @Test
    fun `wifi beats tethering`() {
        val picked = LocalAddress.pick(
            listOf(nic("ap0", "192.168.43.1"), nic("wlan0", "10.55.1.15"))
        )
        assertEquals("wlan0", picked?.name)
    }

    @Test
    fun `tethering is used when there is no wifi station address`() {
        val picked = LocalAddress.pick(
            listOf(nic("rmnet_data0", "10.171.44.9"), nic("ap0", "192.168.43.1"))
        )
        assertEquals("ap0", picked?.name)
    }

    @Test
    fun `loopback alone yields nothing rather than a useless URL`() {
        assertNull(LocalAddress.pick(listOf(nic("lo", "127.0.0.1"))))
    }

    @Test
    fun `link-local addresses are rejected`() {
        assertNull(LocalAddress.pick(listOf(nic("wlan0", "169.254.7.9"))))
    }

    @Test
    fun `no interfaces yields null`() {
        assertNull(LocalAddress.pick(emptyList()))
    }

    @Test
    fun `mobile data is still better than nothing`() {
        assertEquals("rmnet_data0", LocalAddress.pick(listOf(nic("rmnet_data0", "10.171.44.9")))?.name)
    }

    @Test
    fun `malformed and non-ipv4 addresses are dropped`() {
        assertNull(LocalAddress.pick(listOf(nic("wlan0", "fe80::1"))))
        assertNull(LocalAddress.pick(listOf(nic("wlan0", "10.0.0"))))
        assertNull(LocalAddress.pick(listOf(nic("wlan0", "10.0.0.300"))))
        assertNull(LocalAddress.pick(listOf(nic("wlan0", "0.0.0.0"))))
        assertNull(LocalAddress.pick(listOf(nic("wlan0", ""))))
    }

    @Test
    fun `the rig address is usable`() {
        // Phone 10.55.1.15, QGC laptop 10.55.1.12 — same /24 (docs/dev-environment.md).
        assertEquals("10.55.1.15", LocalAddress.pick(listOf(nic("wlan0", "10.55.1.15")))?.address)
    }

    @Test
    fun `interface scoring order is wifi then tether then generic then mobile`() {
        val wlan = LocalAddress.score("wlan0")
        val ap = LocalAddress.score("ap0")
        val generic = LocalAddress.score("bond0")
        val mobile = LocalAddress.score("rmnet_data0")
        assert(wlan < ap && ap < generic && generic < mobile)
    }
}
