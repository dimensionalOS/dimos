package com.dimensional.mini4pro.mavlink

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference

/**
 * The socket-swap semantics a WiFi rebind depends on (`MavlinkLink.rebind`):
 * the input stream must survive the bound socket being replaced under it —
 * distinguishing "socket closed because we swapped" from "socket closed because
 * we stopped" — and the output stream must send each frame on exactly one socket.
 *
 * These run against real loopback UDP because the behaviour under test IS the
 * interaction with `DatagramSocket.close()` unblocking `receive()`; a fake socket
 * would test the fake.
 */
class DatagramStreamsTest {

    private val loopback: InetAddress = InetAddress.getLoopbackAddress()

    private fun sendTo(port: Int, payload: ByteArray) {
        DatagramSocket().use { s ->
            s.send(DatagramPacket(payload, payload.size, loopback, port))
        }
    }

    @Test(timeout = 10_000)
    fun `input stream survives a socket swap and keeps reading from the new socket`() {
        val a = DatagramSocket(0, loopback)
        val b = DatagramSocket(0, loopback)
        val current = AtomicReference(a)
        val input = DatagramInputStream({ current.get() })

        val executor = Executors.newSingleThreadExecutor()
        try {
            // Reader parked in receive() on A — the state a rebind interrupts.
            val first = executor.submit<Int> { input.read() }
            sendTo(a.localPort, byteArrayOf(11))
            assertEquals(11, first.get(5, TimeUnit.SECONDS))

            // The rebind ordering: install the new socket FIRST, then close the
            // old. The reader must come back around onto B, not report EOF.
            val second = executor.submit<Int> { input.read() }
            Thread.sleep(50) // let the reader block in receive(a) before the swap
            current.set(b)
            a.close()
            sendTo(b.localPort, byteArrayOf(22))
            assertEquals(22, second.get(5, TimeUnit.SECONDS))
        } finally {
            executor.shutdownNow()
            a.close()
            b.close()
        }
    }

    @Test(timeout = 10_000)
    fun `closing the current socket without replacing it is the stop signal`() {
        val a = DatagramSocket(0, loopback)
        val input = DatagramInputStream({ a })
        val executor = Executors.newSingleThreadExecutor()
        try {
            val read = executor.submit<Int> { input.read() }
            Thread.sleep(50) // let the reader block in receive(a) first
            a.close()
            // Same closed socket still current — that is stop, i.e. EOF, which is
            // what ends MavlinkLink's rx loop.
            assertEquals(-1, read.get(5, TimeUnit.SECONDS))
        } finally {
            executor.shutdownNow()
            a.close()
        }
    }

    @Test
    fun `a null socket reads as end of stream, not a crash`() {
        // stop() nulls the socket after joining the rx thread; if the join timed
        // out the reader can still come around and must end, not throw.
        assertEquals(-1, DatagramInputStream({ null }).read())
    }

    @Test
    fun `output stream drops the frame when between sockets, after tapping it`() {
        var tapped: ByteArray? = null
        val out = DatagramOutputStream(
            socket = { null },
            target = { InetSocketAddress(loopback, 14550) },
            onDatagram = { tapped = it },
        )
        out.write(byteArrayOf(1, 2, 3))
        out.flush() // must not throw: no socket is a drop, like no target
        // The tap fires before the drop — "we tried to send this" stays on record.
        assertTrue(tapped!!.contentEquals(byteArrayOf(1, 2, 3)))
    }

    @Test
    fun `output stream sends on whatever socket is current at flush time`() {
        val receiver = DatagramSocket(0, loopback)
        receiver.soTimeout = 5_000
        val a = DatagramSocket(0, loopback)
        val b = DatagramSocket(0, loopback)
        val current = AtomicReference(a)
        val out = DatagramOutputStream(
            socket = { current.get() },
            target = { InetSocketAddress(loopback, receiver.localPort) },
        )
        try {
            out.write(byteArrayOf(1))
            out.flush()
            val p1 = DatagramPacket(ByteArray(64), 64)
            receiver.receive(p1)
            assertEquals(a.localPort, p1.port)

            // The swap: subsequent frames leave from the new socket's port — which
            // is also how the GCS/relay learns our post-rebind return address.
            current.set(b)
            a.close()
            out.write(byteArrayOf(2))
            out.flush()
            val p2 = DatagramPacket(ByteArray(64), 64)
            receiver.receive(p2)
            assertEquals(b.localPort, p2.port)
        } finally {
            receiver.close()
            a.close()
            b.close()
        }
    }
}
