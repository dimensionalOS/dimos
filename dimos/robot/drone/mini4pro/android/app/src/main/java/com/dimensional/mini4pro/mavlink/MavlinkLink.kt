package com.dimensional.mini4pro.mavlink

import android.util.Log
import com.dimensional.mini4pro.record.Tap
import io.dronefleet.mavlink.MavlinkConnection
import io.dronefleet.mavlink.MavlinkMessage
import java.io.IOException
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.util.concurrent.atomic.AtomicLong
import kotlin.concurrent.thread

/**
 * The MAVLink wire link to the ground station: one UDP socket, MAVLink 2 out,
 * inbound messages handed to [onMessage].
 *
 * We are the vehicle, so we identify as system 1 / component 1 and send
 * unsolicited telemetry. The GCS learns our address from our first packet.
 *
 * ## Both directions are recorded here, and there is no way to build one that is not
 *
 * [tap] is a **non-null constructor parameter**, so a link that records nothing cannot exist. That
 * is a deliberate change from what this class used to be: a public `var onSent` that `Bridge`
 * remembered to install a recorder into, plus a `Recorder.mavIn` at the top of `Bridge.onInbound`
 * that nothing obliged anyone to keep. Both worked. Both were conventions, and the gimbal — which
 * aimed a real camera for weeks and left no trace in any flight record — is what a convention like
 * that is worth over time. Recording now happens *on the way past*, inside the class that owns the
 * wire, rather than beside it in the class that happens to own the wiring.
 *
 * Outbound is tapped at the **socket**, not at [send]: that records the datagram that actually
 * left, so sequence numbers in the log are the real ones and outbound loss is analysable. Inbound
 * is tapped **before** [onMessage], so what the GCS asked for is on the record even if routing it
 * throws.
 *
 * Neither tap can change the wire: both return `Unit`, both are contained by the [Tap]
 * implementation (that containment is the tap's job, not this class's — see [Tap]), and neither
 * sits between a byte and the socket.
 */
class MavlinkLink(
    private val targetHost: String,
    private val targetPort: Int = DEFAULT_GCS_PORT,
    private val localPort: Int = 0, // 0 = ephemeral; QGC replies to our source port
    private val onMessage: (MavlinkMessage<*>) -> Unit,
    /**
     * The flight recorder, as the seam every transport is constructed with. Required and non-null
     * on purpose — see the class doc. Tests pass a recording fake.
     */
    private val tap: Tap,
    /**
     * Builds the UDP socket. Injected so the Android side can hand in a socket
     * bound to the WiFi `Network` (`wifi-fix.md` gotcha #2) without this class
     * importing `android.net` — see [WifiBind.socketFactory]. A factory that
     * throws must not leak the socket; the default cannot throw after creating it.
     */
    private val socketFactory: (Int) -> DatagramSocket = { DatagramSocket(it) },
) {

    companion object {
        private const val TAG = "MavlinkLink"
        const val DEFAULT_GCS_PORT = 14550
        const val SYSTEM_ID = 1
        const val COMPONENT_ID = 1
    }

    // Volatile: the stream suppliers read it from the rx and tx threads, while
    // start/stop/rebind write it from theirs.
    @Volatile
    private var socket: DatagramSocket? = null
    private var connection: MavlinkConnection? = null
    private var output: DatagramOutputStream? = null
    private var rxThread: Thread? = null

    @Volatile
    private var target: InetSocketAddress? = null

    @Volatile
    private var running = false

    val sent = AtomicLong()
    val received = AtomicLong()

    @Volatile
    var lastError: String? = null
        private set

    fun start() {
        if (running) return
        running = true
        lastError = null

        val sock = socketFactory(localPort)
        socket = sock
        target = InetSocketAddress(targetHost, targetPort)

        // The socket is read through the field rather than captured, so rebind() can swap it
        // under a live connection. The tap is final for the life of the link — it is what this
        // class was constructed with, and there is no window in which a datagram leaves untapped.
        val out = DatagramOutputStream({ socket }, { target }, onDatagram = { tap.gcsOut(it) })
        output = out
        val input = DatagramInputStream({ socket })
        connection = MavlinkConnection.create(input, out)

        rxThread = thread(name = "mavlink-rx", isDaemon = true) {
            val conn = connection ?: return@thread
            try {
                while (running) {
                    val message = conn.next() ?: break
                    received.incrementAndGet()
                    // First, with the genuine wire bytes, and before routing: what the GCS asked
                    // for and when is half of any argument about what the aircraft then did, and
                    // it must survive a router that throws.
                    tap.gcsIn(message)
                    onMessage(message)
                }
            } catch (e: IOException) {
                if (running) {
                    Log.w(TAG, "rx loop ended", e)
                    lastError = e.message
                }
            }
        }
        Log.i(TAG, "link up: local=${sock.localPort} target=$targetHost:$targetPort")
    }

    /**
     * Replaces the UDP socket under the live link with one from [newSocketFactory],
     * leaving everything above it — `MavlinkConnection` and its outbound sequence
     * numbers, the rx thread, the caller's emitters — untouched. This is how the
     * link survives WiFi coming back as a *new* `Network` after a loss: the old
     * bound socket is permanently dead (netIds are not reused), so the socket is
     * the one thing that must be rebuilt.
     *
     * Ordering is the race-safety argument: the fresh socket is installed in
     * [socket] *before* the old one is closed, so when close() kicks the rx thread
     * out of `receive()`, `DatagramInputStream.fill` re-reads the supplier, sees a
     * different socket, and keeps reading — it can never observe the closed socket
     * as current and mistake a rebind for the stop signal. A tx flush that drew the
     * old socket just before the close loses at most that one frame (already
     * tapped into the recorder) and draws the fresh socket on the next flush.
     *
     * The new socket gets a new ephemeral source port. That is deliberate: reusing
     * the old port would require closing the old socket *first*, which reverses the
     * ordering above. The GCS/relay re-learns our address from our next outbound
     * datagram, exactly as it does after an app restart.
     *
     * Synchronized against [stop] so a rebind cannot resurrect a socket on a link
     * that is going down. Returns false if the link is not running (nothing done);
     * throws if the factory fails, in which case the old socket is untouched and
     * the link stays in its prior (dead-network but consistent) state.
     */
    @Synchronized
    fun rebind(newSocketFactory: (Int) -> DatagramSocket): Boolean {
        if (!running) return false
        val fresh = newSocketFactory(localPort) // throws → nothing was swapped
        val old = socket
        socket = fresh
        old?.close()
        Log.i(TAG, "rebound: local=${fresh.localPort} target=$targetHost:$targetPort")
        return true
    }

    @Synchronized
    fun stop() {
        running = false
        // Closing the socket is what unblocks the receive in the rx thread; fill()
        // then sees the same closed socket still current and reads it as stop.
        socket?.close()
        rxThread?.join(500)
        socket = null
        connection = null
        output = null
        rxThread = null
        Log.i(TAG, "link down")
    }

    val isRunning: Boolean get() = running

    /** Sends [payload] as an unsigned MAVLink 2 message. Safe to call from any thread. */
    fun send(payload: Any) {
        val conn = connection ?: return
        try {
            conn.send2(SYSTEM_ID, COMPONENT_ID, payload)
            sent.incrementAndGet()
        } catch (e: IOException) {
            Log.w(TAG, "send failed: ${payload.javaClass.simpleName}", e)
            lastError = e.message
        }
    }
}
