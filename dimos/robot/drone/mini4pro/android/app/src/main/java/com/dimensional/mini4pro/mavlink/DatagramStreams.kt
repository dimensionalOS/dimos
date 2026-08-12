package com.dimensional.mini4pro.mavlink

import java.io.ByteArrayOutputStream
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketException

/**
 * Adapters that let the stream-oriented dronefleet MAVLink library speak UDP.
 *
 * The library's send path is `out.write(packetBytes); out.flush()` (see
 * MavlinkConnection.send), so an OutputStream that transmits one datagram per
 * flush() yields exactly one UDP datagram per MAVLink message — which is what
 * every GCS expects.
 */

/**
 * Reads datagrams from whatever [socket] currently returns and presents their
 * payloads as one byte stream.
 *
 * The socket is a supplier, not a value, so `MavlinkLink.rebind` can swap the
 * underlying socket — WiFi came back as a new `Network`, the old bound socket is
 * permanently dead — without tearing down this stream, the `MavlinkConnection`
 * on top of it (whose sequence numbers must survive a rebind), or the rx thread.
 */
class DatagramInputStream(
    private val socket: () -> DatagramSocket?,
    /** Invoked with the source of each datagram, so callers can learn the peer. */
    private val onPeer: ((InetSocketAddress) -> Unit)? = null,
) : InputStream() {

    /** Reading a fixed socket, for callers that never rebind (e.g. FakeVehicle). */
    constructor(
        socket: DatagramSocket,
        onPeer: ((InetSocketAddress) -> Unit)? = null,
    ) : this({ socket }, onPeer)

    private val buffer = ByteArray(64 * 1024)
    private var offset = 0
    private var length = 0

    override fun read(): Int {
        if (!fill()) return -1
        return buffer[offset++].toInt() and 0xFF
    }

    override fun read(b: ByteArray, off: Int, len: Int): Int {
        if (len == 0) return 0
        if (!fill()) return -1
        val n = minOf(len, length - offset)
        System.arraycopy(buffer, offset, b, off, n)
        offset += n
        return n
    }

    override fun available(): Int = length - offset

    /** Blocks until buffered bytes are available. Returns false at end of link. */
    private fun fill(): Boolean {
        while (offset >= length) {
            val sock = socket() ?: return false
            val packet = DatagramPacket(buffer, buffer.size)
            try {
                sock.receive(packet)
            } catch (e: SocketException) {
                // close() on the socket unblocks receive() this way. Whether it means
                // stop or rebind is decided by the supplier: still the same socket —
                // stop signal, end the stream; a different (or no) socket — the link
                // swapped sockets under us, keep reading from the new one. The swap
                // installs the new socket BEFORE closing the old (MavlinkLink.rebind),
                // so this re-read cannot observe the closed socket as current.
                if (socket() === sock) return false
                continue
            }
            offset = 0
            length = packet.length
            onPeer?.invoke(InetSocketAddress(packet.address, packet.port))
        }
        return true
    }
}

/**
 * Buffers writes and sends one datagram per [flush], from whatever [socket]
 * currently returns to whatever [target] returns.
 *
 * The socket is a supplier for the same reason as [DatagramInputStream]'s: a WiFi
 * rebind swaps the socket underneath a live `MavlinkConnection`. Each datagram is
 * sent on the socket read *once* at flush time, so a frame goes out entirely on
 * the old socket or entirely on the new one — never split across a half-closed
 * socket.
 */
class DatagramOutputStream(
    private val socket: () -> DatagramSocket?,
    private val target: () -> InetSocketAddress?,
    /**
     * Tapped with the exact bytes of every datagram, just before it is sent.
     *
     * This exists so the flight recorder logs the **real frame** — genuine
     * sequence numbers and all — instead of a re-serialisation of the payload,
     * which is what makes outbound packet loss analysable from our own log.
     *
     * It is called on the sending thread and must not throw: the installer is
     * responsible for containing its own faults (see `MavlinkLink.onSent`). A
     * broken recorder is an evidence problem; a broken send is a flight-safety
     * problem, and the two must not be able to become the same problem.
     */
    private val onDatagram: ((ByteArray) -> Unit)? = null,
) : OutputStream() {

    /** Writing a fixed socket, for callers that never rebind (e.g. FakeVehicle). */
    constructor(
        socket: DatagramSocket,
        target: () -> InetSocketAddress?,
        onDatagram: ((ByteArray) -> Unit)? = null,
    ) : this({ socket }, target, onDatagram)

    private val pending = ByteArrayOutputStream(512)

    override fun write(b: Int) {
        pending.write(b)
    }

    override fun write(b: ByteArray, off: Int, len: Int) {
        pending.write(b, off, len)
    }

    override fun flush() {
        val bytes = pending.toByteArray()
        pending.reset()
        if (bytes.isEmpty()) return
        // Tapped before the send, so a datagram that fails to leave is still on the
        // record — "we tried to send this and the socket refused" is a fact worth
        // having, and it is invisible from a tap placed after a successful send.
        onDatagram?.invoke(bytes)
        val dest = target() ?: return // no peer configured yet — drop rather than throw
        val sock = socket() ?: return // link is between sockets or stopped — same rule
        try {
            sock.send(DatagramPacket(bytes, bytes.size, dest))
        } catch (e: SocketException) {
            // Includes a send that raced a rebind and drew the just-closed socket:
            // one lost frame, surfaced as lastError by MavlinkLink.send, and the tap
            // above already has it on the record. The next flush reads the supplier
            // again and gets the fresh socket.
            throw IOException("MAVLink datagram send failed", e)
        }
    }
}
