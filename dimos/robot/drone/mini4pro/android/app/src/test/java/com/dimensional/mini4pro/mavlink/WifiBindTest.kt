package com.dimensional.mini4pro.mavlink

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test
import java.io.IOException
import java.net.DatagramSocket

/**
 * The pure half of wifi-fix.md gotcha #2: the bound-socket factory and the
 * report-don't-act status logic.
 *
 * The Android half (`WifiNetworkGate`, `Bridge`'s use of `Network.bindSocket`) is
 * deliberately too thin to unit-test; what is pinned here is everything with a
 * decision in it — that a failed bind cannot leak a socket, that a refusal says
 * why, that a WiFi blip produces exactly one rebind and one announcement, and
 * that a failed rebind degrades to an honest report instead of a crash.
 */
class WifiBindTest {

    // ── socketFactory ────────────────────────────────────────────────────────

    @Test
    fun `factory hands the bind exactly the socket it returns`() {
        var handed: DatagramSocket? = null
        val socket = WifiBind.socketFactory { handed = it }(0)
        try {
            assertSame(socket, handed)
            assertTrue(socket.isBound)
            assertFalse(socket.isClosed)
        } finally {
            socket.close()
        }
    }

    @Test
    fun `factory respects the requested local port`() {
        // A port the OS just handed out is the closest thing to a known-free one.
        val probe = DatagramSocket(0)
        val port = probe.localPort
        probe.close()

        val socket = WifiBind.socketFactory { }(port)
        try {
            assertEquals(port, socket.localPort)
        } finally {
            socket.close()
        }
    }

    @Test
    fun `a failed bind closes the socket and propagates as IOException`() {
        // The caller only ever learns about a socket the factory returned, so a
        // factory that throws with the socket still open leaks it — and a leaked
        // UDP port is a port the next start() cannot have.
        var created: DatagramSocket? = null
        try {
            WifiBind.socketFactory {
                created = it
                throw IOException("network vanished")
            }(0)
            fail("expected IOException from the failed bind")
        } catch (e: IOException) {
            assertEquals("network vanished", e.cause?.message)
        }
        assertNotNull(created)
        assertTrue(created!!.isClosed)
    }

    // ── refusal ──────────────────────────────────────────────────────────────

    @Test
    fun `refusal status says refused, names the wait, and blames cellular`() {
        val status = WifiBind.refusalStatus(WifiBind.WAIT_MS)
        assertTrue(status, status.contains("REFUSED"))
        assertTrue(status, status.contains("${WifiBind.WAIT_MS}ms"))
        assertTrue(status, status.contains("WiFi"))
        // The reason must be on the line: this refusal exists because unbound
        // telemetry leaves over cellular. A bare "REFUSED" invites a workaround.
        assertTrue(status, status.contains("cellular"))
    }

    // ── WifiBindTracker ──────────────────────────────────────────────────────

    @Test
    fun `bound status names the network`() {
        assertTrue(WifiBindTracker().bound("104").contains("104"))
    }

    @Test
    fun `the initial available callback is not a rebind`() {
        // ConnectivityManager replays the current network to every new callback;
        // rebinding on it would swap a perfectly good socket at every start.
        assertFalse(WifiBindTracker().shouldRebind("104", linkRunning = true))
    }

    @Test
    fun `available with no prior loss never rebinds, even after binding`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        assertFalse(tracker.shouldRebind("104", linkRunning = true))
        assertFalse(tracker.shouldRebind("107", linkRunning = true))
    }

    @Test
    fun `losing the bound network warns and promises the auto-rebind`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        val message = tracker.lost("104")
        assertNotNull(message)
        assertTrue(message!!, message.contains("LOST"))
        // The operator must know recovery is automatic, or a 2 s blip triggers a
        // hands-on-aircraft intervention that is no longer needed.
        assertTrue(message, message.contains("rebind"))
    }

    @Test
    fun `losing a network the socket is not bound to is silent`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        assertNull(tracker.lost("105"))
        // ...and does not arm a rebind either.
        assertFalse(tracker.shouldRebind("106", linkRunning = true))
    }

    @Test
    fun `losing before anything was bound is silent`() {
        assertNull(WifiBindTracker().lost("104"))
    }

    @Test
    fun `a blip rebinds once and announces once`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        assertTrue(tracker.shouldRebind("107", linkRunning = true))
        val message = tracker.rebound("107")
        // The announcement names both networks: what we are on now, what died.
        assertTrue(message, message.contains("REBOUND"))
        assertTrue(message, message.contains("107"))
        assertTrue(message, message.contains("104"))
        // Consumed: the next available callback (replays happen) is not news.
        assertFalse(tracker.shouldRebind("107", linkRunning = true))
        assertFalse(tracker.shouldRebind("108", linkRunning = true))
    }

    @Test
    fun `after a rebind the tracker follows the NEW network`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        tracker.rebound("107")
        // Losing the dead old network again says nothing; losing the new one does.
        assertNull(tracker.lost("104"))
        assertNotNull(tracker.lost("107"))
        assertTrue(tracker.shouldRebind("109", linkRunning = true))
    }

    @Test
    fun `no rebind when the link is not running`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        assertFalse(tracker.shouldRebind("107", linkRunning = false))
        // A stopped-link answer must not consume the pending loss: if the link is
        // running at the next event, the rebind is still owed.
        assertTrue(tracker.shouldRebind("107", linkRunning = true))
    }

    @Test
    fun `rebind failure falls back to report-only and keeps the loss pending`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        assertTrue(tracker.shouldRebind("107", linkRunning = true))
        val message = tracker.rebindFailed("107", "bind refused")
        // Report-only: name the failure, the dead network, and the manual out.
        assertTrue(message, message.contains("failed"))
        assertTrue(message, message.contains("bind refused"))
        assertTrue(message, message.contains("104"))
        assertTrue(message, message.contains("restart"))
        // The loss stays pending, so a later network event can still recover.
        assertTrue(tracker.shouldRebind("108", linkRunning = true))
    }

    @Test
    fun `rebind failure with no exception message still reads as a sentence`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        val message = tracker.rebindFailed("107", null)
        assertFalse(message, message.contains("null"))
    }

    @Test
    fun `rebinding after a bridge restart clears the stale-loss state`() {
        val tracker = WifiBindTracker()
        tracker.bound("104")
        tracker.lost("104")
        tracker.bound("107")
        assertFalse(tracker.shouldRebind("107", linkRunning = true))
        // And losing the OLD network now says nothing — we are not on it.
        assertNull(tracker.lost("104"))
    }
}
