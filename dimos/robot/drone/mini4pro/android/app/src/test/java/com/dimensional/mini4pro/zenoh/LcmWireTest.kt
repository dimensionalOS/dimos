package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The primitives of the LCM wire format, asserted as literal bytes.
 *
 * These are the cases the format makes easy to get wrong and that a round-trip
 * test would happily agree with itself about: endianness, the trailing NUL, a
 * string length that counts *bytes* rather than characters, and a negative
 * double whose sign lives in the first byte written.
 */
class LcmWireTest {

    private fun hex(b: ByteArray) = b.joinToString("") { "%02x".format(it) }

    @Test
    fun `integers are big endian`() {
        assertEquals("01020304", hex(LcmWriter().writeInt(0x01020304).toByteArray()))
        assertEquals("0102030405060708", hex(LcmWriter().writeLong(0x0102030405060708L).toByteArray()))
        assertEquals("8000", hex(LcmWriter().writeShort(-32768).toByteArray()))
        assertEquals("ff", hex(LcmWriter().writeByte(-1).toByteArray()))
    }

    @Test
    fun `negative double keeps its sign bit in the first byte`() {
        // -1.5 is 0xBFF8000000000000. A little-endian bug puts the 0xBF last.
        assertEquals("bff8000000000000", hex(LcmWriter().writeDouble(-1.5).toByteArray()))
        assertEquals("bfc00000", hex(LcmWriter().writeFloat(-1.5f).toByteArray()))
        assertEquals(-1.5, LcmReader(LcmWriter().writeDouble(-1.5).toByteArray()).readDouble(), 0.0)
        assertEquals(-1.5f, LcmReader(LcmWriter().writeFloat(-1.5f).toByteArray()).readFloat(), 0.0f)
    }

    @Test
    fun `boolean is one byte`() {
        assertEquals("01", hex(LcmWriter().writeBoolean(true).toByteArray()))
        assertEquals("00", hex(LcmWriter().writeBoolean(false).toByteArray()))
    }

    @Test
    fun `the empty string is five bytes, not zero`() {
        assertEquals("0000000100", hex(LcmWriter().writeString("").toByteArray()))
        assertEquals("", LcmReader(LcmWriter().writeString("").toByteArray()).readString())
    }

    @Test
    fun `an ascii string is length plus one, bytes, NUL`() {
        assertEquals("0000000504474e5300", hex(LcmWriter().writeString("GNS").toByteArray()))
        assertEquals("00000005" + "6f646f6d" + "00", hex(LcmWriter().writeString("odom").toByteArray()))
    }

    @Test
    fun `a string length counts utf-8 bytes, not characters`() {
        // "µ°" is two characters but four UTF-8 bytes: c2 b5 c2 b0.
        val s = "µ°"
        assertEquals(2, s.length)
        val bytes = LcmWriter().writeString(s).toByteArray()
        assertEquals("00000005" + "c2b5c2b0" + "00", hex(bytes))
        assertEquals(s, LcmReader(bytes).readString())
    }

    @Test
    fun `a four byte utf-8 codepoint survives the round trip`() {
        // U+1F50B is a surrogate pair in Kotlin: length 2, four bytes on the wire.
        val s = "batt 🔋"
        val bytes = LcmWriter().writeString(s).toByteArray()
        assertEquals(4 + 5 + 4 + 1, bytes.size)
        assertEquals(s, LcmReader(bytes).readString())
    }

    @Test
    fun `fixed arrays carry no length prefix`() {
        val w = LcmWriter().writeDoubleArrayFixed(doubleArrayOf(1.0, 2.0), 2)
        assertEquals(16, w.size)
        assertArrayEquals(doubleArrayOf(1.0, 2.0), LcmReader(w.toByteArray()).readDoubleArrayFixed(2), 0.0)
    }

    @Test
    fun `an empty variable array writes nothing after its length`() {
        val w = LcmWriter().writeInt(0).writeFloatArrayBody(FloatArray(0))
        assertEquals("00000000", hex(w.toByteArray()))
        val r = LcmReader(w.toByteArray())
        assertEquals(0, r.readInt())
        assertEquals(0, r.readFloatArrayBody(0).size)
        assertEquals(0, r.remaining)
    }

    @Test
    fun `a wrong size fixed array is refused`() {
        assertThrows(IllegalArgumentException::class.java) {
            LcmWriter().writeDoubleArrayFixed(DoubleArray(8), 9)
        }
    }

    @Test
    fun `a truncated buffer throws rather than reading past the end`() {
        val e = assertThrows(LcmDecodeException::class.java) {
            LcmReader(byteArrayOf(1, 2)).readInt()
        }
        assertTrue(e.message!!.contains("truncated"))
    }

    @Test
    fun `a string longer than the buffer throws`() {
        assertThrows(LcmDecodeException::class.java) {
            LcmReader(byteArrayOf(0, 0, 0, 100, 65)).readString()
        }
    }

    @Test
    fun `a zero string length is rejected`() {
        assertThrows(LcmDecodeException::class.java) {
            LcmReader(byteArrayOf(0, 0, 0, 0)).readString()
        }
    }

    @Test
    fun `a fingerprint mismatch names both fingerprints`() {
        val bytes = LcmWriter().writeFingerprint(0x1122334455667788L).toByteArray()
        val e = assertThrows(LcmDecodeException::class.java) {
            LcmReader(bytes).expectFingerprint(0x0L, "std_msgs.String")
        }
        assertTrue(e.message!!.contains("1122334455667788"))
        assertTrue(e.message!!.contains("std_msgs.String"))
    }

    @Test
    fun `a payload too short for a fingerprint is refused`() {
        assertThrows(LcmDecodeException::class.java) {
            LcmReader(ByteArray(4)).expectFingerprint(1L, "std_msgs.String")
        }
    }

    @Test
    fun `the writer grows past its initial capacity`() {
        val w = LcmWriter(initialCapacity = 1)
        repeat(100) { w.writeDouble(it.toDouble()) }
        assertEquals(800, w.toByteArray().size)
        val r = LcmReader(w.toByteArray())
        repeat(100) { assertEquals(it.toDouble(), r.readDouble(), 0.0) }
    }
}
