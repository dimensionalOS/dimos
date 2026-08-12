package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.record.Json
import com.dimensional.mini4pro.record.JsonObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The reader against the writer it is the inverse of.
 *
 * Most of these do not assert against a hand-written JSON string. They render with
 * `record/JsonObject` — the code that actually writes every flight log — and then read the
 * result back, so what is pinned is *agreement between the two halves*, which is the only
 * property that matters. A hand-written literal would pass while both halves drifted
 * together.
 *
 * The exceptions are the cases the writer cannot produce: a literal `null`, whitespace,
 * escapes, and truncation. Those are hand-written because a real log is read by
 * `tools/flightlog` and by `jq` as well, and a reader that only accepts one writer's output
 * is a reader with a hidden dependency.
 */
class RecordJsonTest {

    private fun roundTrip(body: (JsonObject) -> Unit): Map<String, Any?> =
        RecordJson.parseObject(JsonObject.render(body))

    @Test
    fun `scalars survive the writer`() {
        val o = roundTrip { w ->
            w.put("s", "hello")
            w.put("b", true)
            w.put("i", 42)
            w.put("l", 47_973_515_798_721L)
            w.put("d", 37.9938461, 7)
        }
        assertEquals("hello", RecordJson.string(o["s"]))
        assertEquals(true, o["b"])
        assertEquals(42L, RecordJson.long(o["i"]))
        assertEquals(47_973_515_798_721L, RecordJson.long(o["l"]))
        assertEquals(37.9938461, RecordJson.number(o["d"])!!, 1e-12)
    }

    /**
     * `started_mono_ns` is the anchor the whole timeline hangs off, and it is
     * `SystemClock.elapsedRealtimeNanos` — past a `Double`'s exact-integer range after a few
     * months of uptime. Parsing it as a double would round it silently.
     */
    @Test
    fun `a large integer keeps every digit`() {
        val big = 9_007_199_254_740_993L // 2^53 + 1: the first integer a Double cannot hold
        val o = RecordJson.parseObject("""{"started_mono_ns":$big}""")
        assertEquals(big, RecordJson.long(o["started_mono_ns"]))
        assertTrue("must not have gone through a Double", o["started_mono_ns"] is Long)
    }

    /**
     * The writer's rule 1: a null is omitted, never written. So a reader must treat an absent
     * key and a literal `null` the same way — as "there was no valid reading", never as zero.
     */
    @Test
    fun `absent and explicit null are both nothing, and neither is zero`() {
        val written = JsonObject.render { w ->
            w.put("lat", null as Double?, 7)
            w.put("lon", 23.7, 7)
        }
        assertEquals("""{"lon":23.7}""", written)
        val o = RecordJson.parseObject(written)
        assertNull(RecordJson.number(o["lat"]))
        assertNull(o["lat"])

        val explicit = RecordJson.parseObject("""{"lat":null,"lon":23.7}""")
        assertNull(RecordJson.number(explicit["lat"]))
        assertEquals(23.7, RecordJson.number(explicit["lon"])!!, 1e-12)
    }

    /**
     * The writer's rule 2. Bare `NaN` is not JSON, so `record/Json` writes the three
     * non-finite values as quoted strings; a reader that did not coerce them back would turn
     * "we have no reading" into the literal text `"NaN"` and every numeric consumer into a
     * null.
     */
    @Test
    fun `NaN and both infinities survive as floats, not as text`() {
        val sb = StringBuilder()
        Json.num(sb, Double.NaN, 3)
        assertEquals("\"NaN\"", sb.toString())

        val o = RecordJson.parseObject(
            JsonObject.render { w ->
                w.put("nan", Double.NaN, 3)
                w.put("inf", Double.POSITIVE_INFINITY, 3)
                w.put("ninf", Double.NEGATIVE_INFINITY, 3)
            }
        )
        assertTrue(RecordJson.number(o["nan"])!!.isNaN())
        assertEquals(Double.POSITIVE_INFINITY, RecordJson.number(o["inf"])!!, 0.0)
        assertEquals(Double.NEGATIVE_INFINITY, RecordJson.number(o["ninf"])!!, 0.0)
    }

    @Test
    fun `a nested object round-trips`() {
        val o = roundTrip { w ->
            w.put("lat", 37.9938461, 7)
            w.obj("age") { a ->
                a.put("pos", 92L)
                a.put("relalt", 42052L)
                a.put("att", null as Long?)
            }
        }
        @Suppress("UNCHECKED_CAST")
        val age = o["age"] as Map<String, Any?>
        assertEquals(92L, RecordJson.long(age["pos"]))
        assertEquals(42052L, RecordJson.long(age["relalt"]))
        assertNull(RecordJson.long(age["att"]))
    }

    /** `JsonObject.obj` drops a nested object whose members were all null. */
    @Test
    fun `an all-null nested object is absent entirely`() {
        val o = roundTrip { w ->
            w.put("lat", 1.5, 3)
            w.obj("age") { a -> a.put("pos", null as Long?) }
        }
        assertNull(o["age"])
    }

    @Test
    fun `an int array round-trips`() {
        val o = roundTrip { w -> w.intArray("cells", listOf(4186, 4183)) }
        assertEquals(listOf(4186L, 4183L), o["cells"])
    }

    /**
     * The escapes the writer emits, plus the ones only another writer would: `\/`, `\b`,
     * `\f`, and a `\u` sequence. A flight log carries device model names and DJI's own
     * warning text, which is where a non-ASCII byte actually comes from.
     */
    @Test
    fun `escapes and multi-byte text survive`() {
        val text = "battery µ 41.5°C 🔋\n\ttab\"quote\\slash"
        val o = roundTrip { w -> w.put("msg", text) }
        assertEquals(text, RecordJson.string(o["msg"]))

        val hand = RecordJson.parseObject("""{"a":"x\/y","b":"\b\f","c":"µ"}""")
        assertEquals("x/y", RecordJson.string(hand["a"]))
        assertEquals("\b", RecordJson.string(hand["b"]))
        assertEquals("µ", RecordJson.string(hand["c"]))
    }

    @Test
    fun `whitespace between tokens is allowed`() {
        val o = RecordJson.parseObject("  { \"a\" : 1 , \"b\" : [ 1 , 2 ] }  ")
        assertEquals(1L, RecordJson.long(o["a"]))
        assertEquals(listOf(1L, 2L), o["b"])
    }

    /**
     * A truncated line is a real thing: a battery dies mid-write and the last line of the
     * file is half an entry. It must be an error rather than a half-parsed entry, because a
     * half-parsed `dji_state` is a position with no age.
     */
    @Test
    fun `a truncated line is refused, not half-read`() {
        for (bad in listOf(
            """{"k":"dji_state","lat":37.99""",
            """{"k":"dji_state","lat":""",
            """{"k":"dji_state",""",
            """{"k":"dji_state"} trailing""",
            """{"k":}""",
        )) {
            var threw = false
            try {
                RecordJson.parseObject(bad)
            } catch (e: RecordFormatException) {
                threw = true
            }
            assertTrue("should have refused: $bad", threw)
        }
    }

    /** A number where a string was expected is not silently stringified — see [RecordJson.string]. */
    @Test
    fun `a typed accessor returns null rather than coercing`() {
        val o = RecordJson.parseObject("""{"v":9,"f":"satelliteCount"}""")
        assertNull(RecordJson.string(o["v"]))
        assertNull(RecordJson.number(o["f"]))
        assertEquals("satelliteCount", RecordJson.string(o["f"]))
    }
}
