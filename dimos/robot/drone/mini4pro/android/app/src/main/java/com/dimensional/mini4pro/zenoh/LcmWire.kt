package com.dimensional.mini4pro.zenoh

import java.nio.charset.StandardCharsets

/**
 * The LCM binary wire format, by hand.
 *
 * DiMOS carries ROS-shaped messages over Zenoh as **bare LCM bytes** — its
 * encoder is literally `msg.lcm_encode()` (`dimos/protocol/pubsub/encoders.py:104`),
 * with no CDR, no ROS serialisation and no envelope. A published payload is
 * therefore exactly:
 *
 * ```
 * [8-byte big-endian fingerprint][big-endian packed fields]
 * ```
 *
 * We write the codec rather than take LCM's runtime jar as a dependency for the
 * reasons in `docs/zenoh-dimos-transport.md` §2.5. The format is small enough to
 * hold in one file and the cost of getting it wrong is caught by fixtures
 * produced by DiMOS's own Python (`android/app/src/test/resources/lcm/`).
 *
 * ## The rules, all of them
 *
 * - **Everything is big-endian.** `int8/16/32/64`, `float`, `double`. There is no
 *   alignment and no padding: fields are packed back to back.
 * - **`boolean` is one byte**, 1 or 0.
 * - **Strings** are `int32` *byte* length **including a trailing NUL**, then the
 *   bytes, then the NUL. So the empty string is `00 00 00 01 00`, five bytes.
 *   The length is a count of encoded bytes, not of characters — a multi-byte
 *   UTF-8 string is longer on the wire than its `String.length`.
 * - **Fixed-size arrays** (`double[36]` covariance, `double[9]`) carry **no**
 *   length prefix: the size is in the type.
 * - **Variable-size arrays** carry their length in a *separate declared field*
 *   that lcm-gen hoists to the **front of the struct** — `nav_msgs.Path` encodes
 *   `poses_length` *before* `header`, and `sensor_msgs.BatteryState` encodes both
 *   `cell_voltage_length` and `cell_temperature_length` before everything else.
 *   This is the single most surprising thing in the format and the reason the
 *   fixtures exist.
 * - **Nested structs** are inlined with no fingerprint of their own. Only the
 *   outermost message carries one.
 *
 * ### UTF-8, and where the generated bindings disagree with each other
 *
 * lcm-gen's **Python** output encodes a string as `data.encode("utf-8")` and
 * length-prefixes the encoded byte count; its **Java** output writes each UTF-16
 * `char` through `DataOutput.write(int)`, which truncates to the low byte and so
 * mangles anything above U+00FF. We follow **Python**, because Python is what
 * DiMOS runs and therefore what is on the other end of the wire.
 */
object LcmWire {
    /** Every LCM message is prefixed with an 8-byte big-endian fingerprint. */
    const val FINGERPRINT_BYTES: Int = 8
}

/**
 * Appends LCM-encoded fields to a growable big-endian buffer.
 *
 * Not thread-safe and not meant to be: build one per message, call [toByteArray],
 * throw it away.
 */
class LcmWriter(initialCapacity: Int = 256) {
    private var buf = ByteArray(if (initialCapacity > 0) initialCapacity else 16)
    private var len = 0

    /** Number of bytes written so far. */
    val size: Int get() = len

    private fun ensure(extra: Int) {
        if (len + extra <= buf.size) return
        var cap = buf.size
        while (cap < len + extra) cap *= 2
        buf = buf.copyOf(cap)
    }

    fun writeByte(v: Int): LcmWriter {
        ensure(1)
        buf[len++] = v.toByte()
        return this
    }

    fun writeBoolean(v: Boolean): LcmWriter = writeByte(if (v) 1 else 0)

    /** `int16_t`, signed, big-endian. */
    fun writeShort(v: Int): LcmWriter {
        ensure(2)
        buf[len++] = (v ushr 8).toByte()
        buf[len++] = v.toByte()
        return this
    }

    /** `int32_t`, big-endian. */
    fun writeInt(v: Int): LcmWriter {
        ensure(4)
        buf[len++] = (v ushr 24).toByte()
        buf[len++] = (v ushr 16).toByte()
        buf[len++] = (v ushr 8).toByte()
        buf[len++] = v.toByte()
        return this
    }

    /** `int64_t`, big-endian. Also how the fingerprint is written. */
    fun writeLong(v: Long): LcmWriter {
        ensure(8)
        var shift = 56
        while (shift >= 0) {
            buf[len++] = (v ushr shift).toByte()
            shift -= 8
        }
        return this
    }

    fun writeFloat(v: Float): LcmWriter = writeInt(java.lang.Float.floatToRawIntBits(v))

    fun writeDouble(v: Double): LcmWriter = writeLong(java.lang.Double.doubleToRawLongBits(v))

    /**
     * `string`: int32 byte-count **including** the trailing NUL, the UTF-8 bytes,
     * then the NUL. The empty string is five bytes, never zero.
     */
    fun writeString(s: String): LcmWriter {
        val bytes = s.toByteArray(StandardCharsets.UTF_8)
        writeInt(bytes.size + 1)
        ensure(bytes.size + 1)
        System.arraycopy(bytes, 0, buf, len, bytes.size)
        len += bytes.size
        buf[len++] = 0
        return this
    }

    /**
     * A fixed-size `double[n]` with **no** length prefix. [expected] is the size
     * the type declares; a mismatch is a programming error, not a wire error.
     */
    fun writeDoubleArrayFixed(values: DoubleArray, expected: Int): LcmWriter {
        require(values.size == expected) {
            "fixed double[$expected] given ${values.size} elements"
        }
        for (v in values) writeDouble(v)
        return this
    }

    /** A variable-length `float[]` body. The length field is written separately. */
    fun writeFloatArrayBody(values: FloatArray): LcmWriter {
        for (v in values) writeFloat(v)
        return this
    }

    /**
     * A variable-length `byte[]` body — `foxglove_msgs.CompressedVideo.data`, and nothing else
     * in this catalogue. The length field is written separately, ahead of the struct.
     *
     * **A bulk copy, not a loop, and that is the whole reason this method exists.** An access unit
     * measured 17 kB and they arrive at 43 fps; writing 730 kB/s one `writeByte` at a time would
     * put a bounds check and a capacity check on every byte of the video path. `System.arraycopy`
     * on a pre-grown buffer is one memcpy — the same discipline `record/VideoSidecar` follows when
     * it hands MSDK's slice straight to a `FileOutputStream`.
     */
    fun writeBytes(data: ByteArray, offset: Int = 0, length: Int = data.size): LcmWriter {
        require(offset >= 0 && length >= 0 && offset + length <= data.size) {
            "byte window $offset..${offset + length} outside ${data.size} bytes"
        }
        ensure(length)
        System.arraycopy(data, offset, buf, len, length)
        len += length
        return this
    }

    /** The 8-byte big-endian fingerprint that opens every top-level message. */
    fun writeFingerprint(fingerprint: Long): LcmWriter = writeLong(fingerprint)

    fun toByteArray(): ByteArray = buf.copyOf(len)
}

/** Raised when bytes do not decode as the type that was asked for. */
class LcmDecodeException(message: String) : RuntimeException(message)

/**
 * Reads LCM-encoded fields out of a byte array.
 *
 * Every read is bounds-checked and every failure is an [LcmDecodeException] —
 * these bytes arrive from the network, so a short or corrupt payload must be a
 * refusal with a reason, never an `ArrayIndexOutOfBoundsException` up the stack.
 */
class LcmReader(private val bytes: ByteArray, offset: Int = 0, length: Int = bytes.size - offset) {
    private var pos = offset
    private val end = offset + length

    init {
        require(offset >= 0 && length >= 0 && offset + length <= bytes.size) {
            "reader window $offset..${offset + length} outside ${bytes.size} bytes"
        }
    }

    /** Bytes not yet consumed. */
    val remaining: Int get() = end - pos

    private fun need(n: Int) {
        if (remaining < n) {
            throw LcmDecodeException("truncated: need $n more bytes, have $remaining")
        }
    }

    fun readByte(): Byte {
        need(1)
        return bytes[pos++]
    }

    fun readBoolean(): Boolean = readByte().toInt() != 0

    /** `int16_t`, signed. */
    fun readShort(): Short {
        need(2)
        val v = ((bytes[pos].toInt() and 0xFF) shl 8) or (bytes[pos + 1].toInt() and 0xFF)
        pos += 2
        return v.toShort()
    }

    fun readInt(): Int {
        need(4)
        val v = ((bytes[pos].toInt() and 0xFF) shl 24) or
            ((bytes[pos + 1].toInt() and 0xFF) shl 16) or
            ((bytes[pos + 2].toInt() and 0xFF) shl 8) or
            (bytes[pos + 3].toInt() and 0xFF)
        pos += 4
        return v
    }

    fun readLong(): Long {
        need(8)
        var v = 0L
        for (i in 0 until 8) v = (v shl 8) or (bytes[pos + i].toLong() and 0xFF)
        pos += 8
        return v
    }

    fun readFloat(): Float = java.lang.Float.intBitsToFloat(readInt())

    fun readDouble(): Double = java.lang.Double.longBitsToDouble(readLong())

    /**
     * `string`. The declared length includes the trailing NUL, which is dropped;
     * a declared length of zero is impossible in a well-formed message and is
     * rejected rather than silently read as empty.
     */
    fun readString(): String {
        val declared = readInt()
        if (declared < 1) throw LcmDecodeException("string length $declared, must be >= 1")
        need(declared)
        val s = String(bytes, pos, declared - 1, StandardCharsets.UTF_8)
        pos += declared
        return s
    }

    /** A fixed-size `double[n]` with no length prefix. */
    fun readDoubleArrayFixed(n: Int): DoubleArray {
        val out = DoubleArray(n)
        for (i in 0 until n) out[i] = readDouble()
        return out
    }

    /** A variable-length `float[]` body whose length was read from its own field. */
    fun readFloatArrayBody(n: Int): FloatArray {
        if (n < 0) throw LcmDecodeException("negative array length $n")
        val out = FloatArray(n)
        for (i in 0 until n) out[i] = readFloat()
        return out
    }

    /**
     * A variable-length `byte[]` body whose length was read from its own field.
     *
     * Bounds-checked **before** allocating, which matters more here than anywhere else in this
     * file: `data_length` on an inbound `CompressedVideo` is an attacker-or-corruption-controlled
     * `int32`, and `ByteArray(n)` with a two-billion `n` is an `OutOfMemoryError` rather than a
     * refusal with a reason.
     */
    fun readBytes(n: Int): ByteArray {
        if (n < 0) throw LcmDecodeException("negative array length $n")
        need(n)
        val out = bytes.copyOfRange(pos, pos + n)
        pos += n
        return out
    }

    /**
     * Reads the 8-byte prefix and refuses anything that is not [expected].
     *
     * A fingerprint mismatch means the sender's type definition is not ours — the
     * only safe response is to drop the message, so this throws rather than
     * guessing.
     */
    fun expectFingerprint(expected: Long, typeName: String) {
        if (remaining < LcmWire.FINGERPRINT_BYTES) {
            throw LcmDecodeException("$typeName: ${remaining} bytes, too short for a fingerprint")
        }
        val actual = readLong()
        if (actual != expected) {
            throw LcmDecodeException(
                "$typeName: fingerprint 0x${java.lang.Long.toHexString(actual)}, " +
                    "expected 0x${java.lang.Long.toHexString(expected)}"
            )
        }
    }
}
