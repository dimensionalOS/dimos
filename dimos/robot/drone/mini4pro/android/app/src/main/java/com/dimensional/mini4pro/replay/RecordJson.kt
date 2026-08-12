package com.dimensional.mini4pro.replay

/**
 * A ~120-line JSON **reader**, the mirror image of `record/Json.kt`'s writer.
 *
 * It exists for the same reason that writer does: Android ships `org.json`, but only as a
 * stub in the unit-test `android.jar` where every method throws *"not mocked"*, and a
 * replay harness whose whole point is to run offline on the JVM cannot depend on a parser
 * that is not there. `kotlinx.serialization` or Moshi would mean a new Gradle dependency
 * for one file.
 *
 * It parses exactly what the recorder writes and nothing more — `mini4pro-flightlog-1` is
 * one flat object per line with scalars, one level of nested objects, and integer arrays —
 * so it is small enough to test exhaustively rather than trusted by reputation.
 *
 * ## The three rules it inherits from the writer
 *
 * 1. **Absent means "there was no valid reading"**, never zero. `Json` omits a null rather
 *    than writing `null`, so a missing key here becomes a Kotlin `null` and every consumer
 *    reads it as the recorder meant it. A literal `null` is accepted too, and means the
 *    same thing.
 * 2. **`"NaN"`, `"Infinity"` and `"-Infinity"` are floats wearing a string**, because bare
 *    `NaN` is not JSON and `jq` rejects it. [number] coerces those three back; anything
 *    else that is a string stays a string. `tools/flightlog` does the same coercion.
 * 3. **An integer stays a `Long`.** `started_mono_ns` is `SystemClock.elapsedRealtimeNanos`
 *    and on a device up for a month it is ~2.6e15 — inside a `Double`'s exact-integer range
 *    today, outside it after ~104 days of uptime. Parsing every number as a `Double` would
 *    quietly round the one field the whole timeline is anchored to.
 *
 * Duplicate keys take the last value, and trailing content after the closing brace is an
 * error rather than ignored: a truncated line is a real thing that happens to a log a
 * battery died in the middle of, and it must be reported rather than half-parsed.
 */
object RecordJson {

    /** One line of the record, as a map. Values are `String`, `Long`, `Double`, `Boolean`, `Map`, `List` or null. */
    fun parseObject(text: String): Map<String, Any?> {
        val p = Parser(text)
        p.skipWhitespace()
        val value = p.parseValue()
        p.skipWhitespace()
        if (!p.atEnd()) throw RecordFormatException("trailing content at offset ${p.offset}")
        @Suppress("UNCHECKED_CAST")
        return value as? Map<String, Any?>
            ?: throw RecordFormatException("expected an object, got ${value?.javaClass?.simpleName}")
    }

    /**
     * [raw] as a `Double`, coercing the writer's three non-finite spellings, or null when
     * it is absent or is not a number at all.
     */
    fun number(raw: Any?): Double? = when (raw) {
        null -> null
        is Double -> raw
        is Long -> raw.toDouble()
        "NaN" -> Double.NaN
        "Infinity" -> Double.POSITIVE_INFINITY
        "-Infinity" -> Double.NEGATIVE_INFINITY
        else -> null
    }

    /** [raw] as a `Long`, or null when it is absent or not an integer-valued number. */
    fun long(raw: Any?): Long? = when (raw) {
        null -> null
        is Long -> raw
        is Double -> if (raw.isFinite()) raw.toLong() else null
        else -> null
    }

    /** [raw] as a `String`, or null. Numbers and booleans are **not** stringified — a
     * field's type in the record is part of what the record says. */
    fun string(raw: Any?): String? = raw as? String

    private class Parser(private val s: String) {
        var offset = 0

        fun atEnd(): Boolean = offset >= s.length

        fun skipWhitespace() {
            while (offset < s.length && s[offset].isWhitespace()) offset++
        }

        fun parseValue(): Any? {
            if (atEnd()) throw RecordFormatException("unexpected end of input")
            return when (val c = s[offset]) {
                '{' -> parseObject()
                '[' -> parseArray()
                '"' -> parseString()
                't' -> literal("true", true)
                'f' -> literal("false", false)
                'n' -> literal("null", null)
                else -> if (c == '-' || c in '0'..'9') parseNumber()
                else throw RecordFormatException("unexpected '$c' at offset $offset")
            }
        }

        private fun literal(word: String, value: Any?): Any? {
            if (!s.startsWith(word, offset)) throw RecordFormatException("bad literal at offset $offset")
            offset += word.length
            return value
        }

        fun parseObject(): Map<String, Any?> {
            expect('{')
            val out = LinkedHashMap<String, Any?>()
            skipWhitespace()
            if (peek() == '}') { offset++; return out }
            while (true) {
                skipWhitespace()
                val key = parseString()
                skipWhitespace()
                expect(':')
                skipWhitespace()
                out[key] = parseValue()
                skipWhitespace()
                when (val c = next()) {
                    ',' -> continue
                    '}' -> return out
                    else -> throw RecordFormatException("expected ',' or '}' but got '$c' at offset ${offset - 1}")
                }
            }
        }

        private fun parseArray(): List<Any?> {
            expect('[')
            val out = ArrayList<Any?>()
            skipWhitespace()
            if (peek() == ']') { offset++; return out }
            while (true) {
                skipWhitespace()
                out.add(parseValue())
                skipWhitespace()
                when (val c = next()) {
                    ',' -> continue
                    ']' -> return out
                    else -> throw RecordFormatException("expected ',' or ']' but got '$c' at offset ${offset - 1}")
                }
            }
        }

        private fun parseString(): String {
            expect('"')
            val sb = StringBuilder()
            while (true) {
                val c = next()
                when {
                    c == '"' -> return sb.toString()
                    c == '\\' -> when (val esc = next()) {
                        '"' -> sb.append('"')
                        '\\' -> sb.append('\\')
                        '/' -> sb.append('/')
                        'b' -> sb.append('\b')
                        'f' -> sb.append('\u000C')
                        'n' -> sb.append('\n')
                        'r' -> sb.append('\r')
                        't' -> sb.append('\t')
                        'u' -> {
                            if (offset + 4 > s.length) throw RecordFormatException("truncated \\u escape")
                            sb.append(s.substring(offset, offset + 4).toInt(16).toChar())
                            offset += 4
                        }
                        else -> throw RecordFormatException("bad escape '\\$esc' at offset ${offset - 1}")
                    }
                    else -> sb.append(c)
                }
            }
        }

        /** Integer-looking numbers stay `Long` — see rule 3 in the object doc. */
        private fun parseNumber(): Any {
            val start = offset
            if (peek() == '-') offset++
            var isFloat = false
            while (offset < s.length) {
                val c = s[offset]
                when {
                    c in '0'..'9' -> offset++
                    c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-' -> { isFloat = true; offset++ }
                    else -> break
                }
            }
            val text = s.substring(start, offset)
            if (text.isEmpty() || text == "-") throw RecordFormatException("bad number at offset $start")
            return if (isFloat) {
                text.toDoubleOrNull() ?: throw RecordFormatException("bad number '$text' at offset $start")
            } else {
                text.toLongOrNull()
                    ?: text.toDoubleOrNull()
                    ?: throw RecordFormatException("bad number '$text' at offset $start")
            }
        }

        private fun peek(): Char =
            if (offset < s.length) s[offset] else throw RecordFormatException("unexpected end of input")

        private fun next(): Char =
            if (offset < s.length) s[offset++] else throw RecordFormatException("unexpected end of input")

        private fun expect(c: Char) {
            val got = next()
            if (got != c) throw RecordFormatException("expected '$c' but got '$got' at offset ${offset - 1}")
        }
    }
}

/**
 * A line the record reader could not make sense of — a truncated tail, a corrupt byte, a
 * schema that moved.
 *
 * Named rather than reusing `IllegalArgumentException` so a caller reading a 200 000-line
 * session can decide per line whether to skip and count it or to stop, which is a real
 * choice: a log whose last line was cut off by a dead battery is still worth replaying.
 */
class RecordFormatException(message: String) : RuntimeException(message)
