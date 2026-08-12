package com.dimensional.mini4pro.record

/**
 * A ~100-line JSON writer, because the flight recorder must not depend on
 * anything that is unavailable to a JVM unit test.
 *
 * Android ships `org.json`, but only as a stub in the unit-test `android.jar`:
 * every method throws "not mocked". `kotlinx.serialization` or Moshi would mean a
 * new Gradle dependency, and this agent is not allowed to edit `build.gradle`.
 * The output we need is one flat object per line with scalar and one-level-nested
 * values, which is small enough to write by hand and to test exhaustively.
 *
 * Two deliberate rules, both of which `tools/flightlog` relies on:
 *
 * 1. **A null value is omitted, never written as `null`.** `AircraftState` is
 *    null-per-field and a null there is DJI's *component-gone* signal. Omitting
 *    keeps a sparse state cheap on disk, and the reader's rule is simply "absent
 *    means there was no valid reading". Writing `null` would cost bytes to say the
 *    same thing.
 * 2. **Non-finite floats are written as the JSON strings `"NaN"`, `"Infinity"`,
 *    `"-Infinity"`.** Bare `NaN` is not JSON: `jq` rejects it outright, and while
 *    Python's `json` accepts it, a log we cannot `jq` is a log we will not read.
 *    `tools/flightlog` coerces those three strings back to floats.
 */
object Json {

    /** Appends [s] as a JSON string literal, escaping what RFC 8259 requires. */
    fun str(sb: StringBuilder, s: String) {
        sb.append('"')
        for (c in s) {
            when {
                c == '"' -> sb.append("\\\"")
                c == '\\' -> sb.append("\\\\")
                c == '\n' -> sb.append("\\n")
                c == '\r' -> sb.append("\\r")
                c == '\t' -> sb.append("\\t")
                c < ' ' || c == '\u007f' -> sb.append(String.format("\\u%04x", c.code))
                else -> sb.append(c)
            }
        }
        sb.append('"')
    }

    /**
     * Appends a double with at most [decimals] fractional digits and no trailing
     * zeros or exponent — so a log stays greppable and two logs diff cleanly.
     *
     * Rounding is a deliberate part of the format: an unrounded `Double.toString`
     * of a DJI velocity writes 17 significant digits of float noise, which is
     * pure disk cost. The per-field precision is chosen in [LogEntry] to be
     * finer than the sensor, so nothing measurable is lost.
     */
    fun num(sb: StringBuilder, v: Double, decimals: Int) {
        if (v.isNaN()) { sb.append("\"NaN\""); return }
        if (v == Double.POSITIVE_INFINITY) { sb.append("\"Infinity\""); return }
        if (v == Double.NEGATIVE_INFINITY) { sb.append("\"-Infinity\""); return }
        if (decimals <= 0) { sb.append(Math.round(v)); return }
        var scale = 1L
        repeat(decimals) { scale *= 10 }
        val scaled = scaled(v, scale)
        if (scaled == 0L) { sb.append('0'); return }
        val neg = scaled < 0
        val whole = (if (neg) -scaled else scaled) / scale
        var frac = (if (neg) -scaled else scaled) % scale
        if (neg) sb.append('-')
        sb.append(whole)
        if (frac == 0L) return
        sb.append('.')
        // zero-pad the fraction to `decimals`, then trim trailing zeros
        var pad = scale / 10
        while (pad > frac && pad > 1) { sb.append('0'); pad /= 10 }
        while (frac % 10 == 0L) frac /= 10
        sb.append(frac)
    }

    /**
     * **The format's one rounding rule**, as the scaled integer [num] renders: `Math.round(v ·
     * scale)` — half-up on the scaled value, Java's `Math.round` semantics (`floor(x + 0.5)`),
     * which is *not* Python's banker's rounding and is why `tools/memexport.unix_seconds`
     * spells it out by hand.
     *
     * Public because the record's precision is part of the record's *meaning*: a consumer that
     * must produce exactly what the record can reproduce — `zenoh/ZenohTelemetryEncoder`'s
     * `setpoint` path, whose wire bytes have to be derivable from the `stick_cmd` line alone —
     * quantises through this same function rather than keeping a second copy of the rule.
     * [roundTo] is the value-shaped convenience over it.
     */
    fun scaled(v: Double, scale: Long): Long = Math.round(v * scale)

    /**
     * [v] at the precision the record would write it: the double a reader gets back from
     * parsing [num]'s output. Non-finite values pass through untouched — [num] writes them as
     * strings and a reader coerces them back, so they round-trip by identity, not by rounding.
     */
    fun roundTo(v: Double, decimals: Int): Double {
        if (!v.isFinite()) return v
        var scale = 1L
        repeat(decimals) { scale *= 10 }
        return scaled(v, scale) / scale.toDouble()
    }
}

/**
 * Builder for one JSON object. Not thread-safe and not meant to be: every entry
 * is rendered on the recorder's writer thread, one at a time.
 */
class JsonObject internal constructor(private val sb: StringBuilder) {
    private var first = true

    private fun key(name: String) {
        if (!first) sb.append(',')
        first = false
        Json.str(sb, name)
        sb.append(':')
    }

    fun put(name: String, v: String?) { if (v != null) { key(name); Json.str(sb, v) } }
    fun put(name: String, v: Boolean?) { if (v != null) { key(name); sb.append(if (v) "true" else "false") } }
    fun put(name: String, v: Int?) { if (v != null) { key(name); sb.append(v) } }
    fun put(name: String, v: Long?) { if (v != null) { key(name); sb.append(v) } }

    /** Doubles always carry an explicit precision — see [Json.num]. */
    fun put(name: String, v: Double?, decimals: Int) {
        if (v != null) { key(name); Json.num(sb, v, decimals) }
    }

    /** Writes a value already rendered as JSON. Used for pre-encoded payloads. */
    fun putRaw(name: String, json: String?) { if (json != null) { key(name); sb.append(json) } }

    /** Nested object. Omitted entirely if [body] writes no members. */
    fun obj(name: String, body: (JsonObject) -> Unit) {
        val mark = sb.length
        val wasFirst = first
        key(name)
        sb.append('{')
        val nested = JsonObject(sb)
        body(nested)
        if (nested.first) {          // nothing was written — undo the key
            sb.setLength(mark)
            first = wasFirst
        } else {
            sb.append('}')
        }
    }

    fun intArray(name: String, values: List<Int>?) {
        if (values == null) return
        key(name)
        sb.append('[')
        values.forEachIndexed { i, v -> if (i > 0) sb.append(','); sb.append(v) }
        sb.append(']')
    }

    companion object {
        /** Renders one complete JSON object to a string. */
        fun render(body: (JsonObject) -> Unit): String {
            val sb = StringBuilder(256)
            sb.append('{')
            body(JsonObject(sb))
            sb.append('}')
            return sb.toString()
        }
    }
}
