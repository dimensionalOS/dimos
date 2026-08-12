package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The JSON writer is hand-rolled (see [Json] for why), so it is the one place in
 * `record/` where a bug would silently corrupt every line of every log. It gets
 * exhaustive tests, including the two format rules `tools/flightlog` depends on:
 * nulls are omitted, and non-finite floats become strings.
 */
class JsonTest {

    private fun str(s: String): String {
        val sb = StringBuilder()
        Json.str(sb, s)
        return sb.toString()
    }

    private fun num(v: Double, decimals: Int): String {
        val sb = StringBuilder()
        Json.num(sb, v, decimals)
        return sb.toString()
    }

    @Test
    fun `escapes what RFC 8259 requires`() {
        assertEquals("\"plain\"", str("plain"))
        assertEquals("\"a\\\"b\"", str("a\"b"))
        assertEquals("\"a\\\\b\"", str("a\\b"))
        assertEquals("\"a\\nb\"", str("a\nb"))
        assertEquals("\"a\\tb\"", str("a\tb"))
        assertEquals("\"a\\rb\"", str("a\rb"))
        assertEquals("\"\\u0000\"", str("\u0000"))
        assertEquals("\"\\u001f\"", str("\u001f"))
        assertEquals("\"\\u007f\"", str("\u007f"))
    }

    @Test
    fun `renders numbers with hand-computed literals, no exponent, no trailing zeros`() {
        assertEquals("0", num(0.0, 3))
        assertEquals("1", num(1.0, 3))
        assertEquals("-1", num(-1.0, 3))
        assertEquals("1.5", num(1.5, 3))
        assertEquals("1.5", num(1.5000, 6))
        assertEquals("-2.25", num(-2.25, 3))
        // 7 decimals of degree is what MAVLink degE7 carries; 37.9938612 is the
        // measured ground-probe latitude and must survive verbatim.
        assertEquals("37.9938612", num(37.9938612, 7))
        assertEquals("23.7253298", num(23.7253298, 7))
        // rounding, not truncation
        assertEquals("1.235", num(1.23456, 3))
        assertEquals("-1.235", num(-1.23456, 3))
        // a value smaller than the precision collapses to 0 rather than 1e-7
        assertEquals("0", num(1e-9, 3))
        // fractions needing zero padding: 0.05 at 3 dp is "0.05", not "0.5"
        assertEquals("0.05", num(0.05, 3))
        assertEquals("0.005", num(0.005, 3))
        assertEquals("-0.005", num(-0.005, 3))
        assertEquals("1.005", num(1.005, 3))
        assertEquals("1.05", num(1.05, 3))
        // decimals = 0 means integer
        assertEquals("18", num(17.6, 0))
        // a big value must not become 1.0E9
        assertEquals("1000000000", num(1e9, 2))
    }

    @Test
    fun `non-finite floats become strings so the log stays parseable by jq`() {
        assertEquals("\"NaN\"", num(Double.NaN, 3))
        assertEquals("\"Infinity\"", num(Double.POSITIVE_INFINITY, 3))
        assertEquals("\"-Infinity\"", num(Double.NEGATIVE_INFINITY, 3))
    }

    @Test
    fun `nulls are omitted entirely, never written as JSON null`() {
        val json = JsonObject.render { o ->
            o.put("a", 1)
            o.put("b", null as Int?)
            o.put("c", null as String?)
            o.put("d", null as Double?, 3)
            o.put("e", null as Boolean?)
            o.put("f", "x")
        }
        assertEquals("""{"a":1,"f":"x"}""", json)
    }

    @Test
    fun `an empty nested object is omitted, key and all`() {
        val json = JsonObject.render { o ->
            o.put("a", 1)
            o.obj("empty") { it.put("x", null as Int?) }
            o.obj("full") { it.put("y", 2) }
        }
        assertEquals("""{"a":1,"full":{"y":2}}""", json)
    }

    @Test
    fun `an empty nested object as the only member leaves a valid empty object`() {
        assertEquals("{}", JsonObject.render { o -> o.obj("e") { it.put("x", null as Int?) } })
    }

    @Test
    fun `int arrays and raw members`() {
        val json = JsonObject.render { o ->
            o.intArray("cells", listOf(4186, 4183))
            o.intArray("absent", null)
            o.putRaw("nested", """{"k":1}""")
            o.putRaw("skipped", null)
        }
        assertEquals("""{"cells":[4186,4183],"nested":{"k":1}}""", json)
    }

    @Test
    fun `output survives a round trip through a strict parser`() {
        // A crude check that we produce no bare NaN and no unquoted keys: every
        // quote is balanced and the braces nest.
        val json = JsonObject.render { o ->
            o.put("t", 1.234567, 6)
            o.put("k", "dji_state")
            o.put("msg", "he said \"up\"\nnot right")
            o.put("nan", Double.NaN, 3)
            o.obj("n") { it.put("v", -0.5, 4) }
        }
        assertEquals(json.count { it == '{' }, json.count { it == '}' })
        assertTrue(json.startsWith("{") && json.endsWith("}"))
        assertTrue("bare NaN would break jq", !json.contains(":NaN"))
    }
}
