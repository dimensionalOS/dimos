package com.dimensional.mini4pro.record

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.annotations.MavlinkFieldInfo
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo
import io.dronefleet.mavlink.protocol.MavlinkPacket
import io.dronefleet.mavlink.ardupilotmega.ArdupilotmegaDialect
import io.dronefleet.mavlink.serialization.payload.reflection.ReflectionPayloadDeserializer
import io.dronefleet.mavlink.serialization.payload.reflection.ReflectionPayloadSerializer
import io.dronefleet.mavlink.util.EnumValue
import java.lang.reflect.Method
import java.math.BigInteger

/**
 * Turns MAVLink messages into [LogEntry.Mav] entries.
 *
 * **Log everything, both directions, with no filtering.** It is cheap: our
 * telemetry is eight messages at 5 Hz and QGC's inbound traffic is a trickle. The
 * cost of deciding what matters in advance is that the one message that turns out
 * to matter is the one that was filtered out.
 *
 * Both a decoded field map **and** the raw bytes as hex go into every entry, so a
 * bug in this file cannot destroy the evidence — the hex can always be re-read by
 * pymavlink, which shares no code with us.
 *
 * ## No DJI, no Android, and no reflection on a flying thread
 *
 * This object imports only the MAVLink library, so it is fully unit-testable. The
 * expensive parts — reflection over the payload's annotated getters, hex encoding,
 * re-serialisation — are deliberately **not** done at call time: [inbound] and
 * [outbound] hand [LogEntry.Mav] suppliers over immutable objects, and the work
 * happens when the writer thread renders the entry.
 */
object MavlinkTap {

    /** Cache of message class → annotated field getters, sorted MAVLink field order. */
    private val fieldCache = HashMap<Class<*>, List<Method>>()

    /**
     * One inbound message. The wire bytes are genuine: dronefleet's
     * `MavlinkMessage.getRawBytes()` returns a copy of the original packet, so
     * `hexsrc` is `"wire"` and sequence numbers in the log are the sender's real
     * ones.
     */
    fun inbound(monoNanos: Long, message: MavlinkMessage<*>): LogEntry.Mav {
        val payload = message.payload
        val info = infoOf(payload.javaClass)
        val raw = try {
            message.rawBytes
        } catch (e: Throwable) {
            null
        }
        return LogEntry.Mav(
            monoNanos = monoNanos,
            inbound = true,
            name = nameOf(payload.javaClass),
            messageId = info?.id ?: -1,
            systemId = message.originSystemId,
            componentId = message.originComponentId,
            sequence = message.sequence,
            hexSource = HEX_WIRE,
            hexProvider = { raw?.let(::hex) },
            fieldsProvider = { fieldsJson(payload) },
        )
    }

    /**
     * One outbound message, built from the payload object we handed to
     * `MavlinkLink`.
     *
     * The bytes are re-serialised here with the same library and the same
     * serialiser the link uses, so the **payload bytes are identical to what went
     * out**. The header is not: `MavlinkLink` owns the sequence counter privately,
     * so the sequence byte is our own count and the CRC follows from it. The entry
     * says `hexsrc: "reserialized"` and `tools/flightlog` refuses to draw
     * conclusions about sequence continuity from such a frame.
     *
     * Pass [wireBytes] instead if the outbound socket is ever tapped — the four-line
     * change is in `docs/flight-recording.md` — and the entry becomes
     * `hexsrc: "wire"`.
     */
    fun outbound(
        monoNanos: Long,
        payload: Any,
        systemId: Int,
        componentId: Int,
        sequence: Int? = null,
        wireBytes: ByteArray? = null,
    ): LogEntry.Mav {
        val info = infoOf(payload.javaClass)
        return LogEntry.Mav(
            monoNanos = monoNanos,
            inbound = false,
            name = nameOf(payload.javaClass),
            messageId = info?.id ?: -1,
            systemId = systemId,
            componentId = componentId,
            sequence = sequence,
            hexSource = if (wireBytes != null) HEX_WIRE else HEX_RESERIALIZED,
            hexProvider = {
                wireBytes?.let(::hex)
                    ?: reserialize(payload, systemId, componentId, sequence ?: 0)
            },
            fieldsProvider = { fieldsJson(payload) },
        )
    }

    /**
     * One outbound message reconstructed from the **actual datagram** that left the
     * socket, so `hexsrc` is `wire` and the sequence number is the real one.
     *
     * This is the path that makes outbound packet loss and sequence continuity
     * analysable from our own log rather than only from QGC's `.tlog`. It needs the
     * four-line `DatagramStreams` tap described in `docs/flight-recording.md`; without
     * it, use [outbound] and accept a re-serialised header.
     *
     * The payload is decoded here with the same dialect `MavlinkConnection` uses, so
     * the `f` map is as complete as [outbound]'s. If the message id is not in the
     * dialect the entry still carries the hex — which is the whole point of keeping
     * the raw bytes.
     */
    fun outboundWire(monoNanos: Long, datagram: ByteArray): LogEntry.Mav? {
        val packet = try {
            MavlinkPacket.fromV2Bytes(datagram)
        } catch (e: Throwable) {
            return null
        }
        val type: Class<*>? = try {
            DIALECT.resolve(packet.messageId)
        } catch (e: Throwable) {
            null
        }
        return LogEntry.Mav(
            monoNanos = monoNanos,
            inbound = false,
            name = type?.let(::nameOf) ?: "MSG_${packet.messageId}",
            messageId = packet.messageId,
            systemId = packet.systemId,
            componentId = packet.componentId,
            sequence = packet.sequence,
            hexSource = HEX_WIRE,
            hexProvider = { hex(datagram) },
            fieldsProvider = {
                type?.let { fieldsJson(DESERIALIZER.deserialize(packet.payload, it) as Any) }
            },
        )
    }

    // ─────────────────────────────────────────────────────────────────────────

    /**
     * `GLOBAL_POSITION_INT` from `GlobalPositionInt`. Derived rather than looked up
     * so a message we have never seen still gets a usable name, and so the name in
     * the log is the one pymavlink and QGC use.
     */
    fun nameOf(cls: Class<*>): String {
        val simple = cls.simpleName
        val sb = StringBuilder(simple.length + 6)
        for ((i, c) in simple.withIndex()) {
            if (c.isUpperCase() && i > 0 && !simple[i - 1].isUpperCase()) sb.append('_')
            sb.append(c.uppercaseChar())
        }
        return sb.toString()
    }

    private fun infoOf(cls: Class<*>): MavlinkMessageInfo? =
        cls.getAnnotation(MavlinkMessageInfo::class.java)

    private fun fieldsOf(cls: Class<*>): List<Method> = synchronized(fieldCache) {
        fieldCache.getOrPut(cls) {
            cls.methods
                .filter { it.isAnnotationPresent(MavlinkFieldInfo::class.java) && it.parameterCount == 0 }
                .sortedBy { it.getAnnotation(MavlinkFieldInfo::class.java)!!.position }
        }
    }

    fun hex(bytes: ByteArray): String {
        val sb = StringBuilder(bytes.size * 2)
        for (b in bytes) {
            val v = b.toInt() and 0xff
            sb.append(HEX_DIGITS[v ushr 4])
            sb.append(HEX_DIGITS[v and 0x0f])
        }
        return sb.toString()
    }

    /**
     * Renders every annotated field of a payload as a JSON object.
     *
     * Values are written in their **raw MAVLink units** — `lat` as degE7, `alt` as
     * millimetres, `hdg` as centidegrees — not prettified into human units. The
     * whole point of logging the converted value next to DJI's raw one is to be able
     * to check the conversion, and that only works if what is logged is the number
     * that actually went on the wire.
     */
    fun fieldsJson(payload: Any): String? = try {
        JsonObject.render { o ->
            for (m in fieldsOf(payload.javaClass)) {
                val name = snake(m.name)
                when (val v = m.invoke(payload)) {
                    null -> {}
                    is Int -> o.put(name, v)
                    is Long -> o.put(name, v)
                    is Short -> o.put(name, v.toInt())
                    is Byte -> o.put(name, v.toInt())
                    is Boolean -> o.put(name, v)
                    // 6 decimals: floats carry ~7 significant digits, so this
                    // loses nothing a float held in the first place.
                    is Float -> o.put(name, v.toDouble(), 6)
                    is Double -> o.put(name, v, 9)
                    is BigInteger -> o.put(name, v.toString())
                    is String -> o.put(name, v)
                    is EnumValue<*> -> {
                        // Both, always: the integer is what went on the wire and the
                        // name is what a human can read. A bad enum mapping shows up
                        // as an integer with no name.
                        o.put(name, v.value())
                        o.put("${name}_s", v.entry()?.toString())
                    }
                    is List<*> -> o.put(name, v.joinToString(",") { it?.toString() ?: "" })
                    is ByteArray -> o.put(name, hex(v))
                    else -> o.put(name, v.toString())
                }
            }
        }
    } catch (e: Throwable) {
        null
    }

    /**
     * Re-serialises a payload to MAVLink 2 wire bytes. See [outbound] for what is
     * and is not faithful about the result.
     */
    fun reserialize(payload: Any, systemId: Int, componentId: Int, sequence: Int): String? = try {
        val info = infoOf(payload.javaClass) ?: return null
        val body = SERIALIZER.serialize(payload)
        val packet = MavlinkPacket.createUnsignedMavlink2Packet(
            sequence and 0xff, systemId, componentId, info.id, info.crc, body,
        )
        hex(packet.rawBytes)
    } catch (e: Throwable) {
        null
    }

    /** `timeBootMs` → `time_boot_ms`, matching pymavlink's field names. */
    fun snake(camel: String): String {
        val sb = StringBuilder(camel.length + 4)
        for ((i, c) in camel.withIndex()) {
            if (c.isUpperCase()) {
                if (i > 0) sb.append('_')
                sb.append(c.lowercaseChar())
            } else {
                sb.append(c)
            }
        }
        return sb.toString()
    }

    private val SERIALIZER = ReflectionPayloadSerializer()
    private val DESERIALIZER = ReflectionPayloadDeserializer()

    /** The dialect we identify as, so a decoded outbound frame matches what QGC sees. */
    private val DIALECT = ArdupilotmegaDialect()
    private val HEX_DIGITS = "0123456789abcdef".toCharArray()

    const val HEX_WIRE = "wire"
    const val HEX_RESERIALIZED = "reserialized"
}
