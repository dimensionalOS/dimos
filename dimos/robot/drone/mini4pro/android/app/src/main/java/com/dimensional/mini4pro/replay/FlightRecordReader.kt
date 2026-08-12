package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.record.LogEntry

/**
 * The read side of `mini4pro-flightlog-1` — `record/LogEntry` is the write side, and that
 * file is the schema.
 *
 * Only the kinds a replay can use are decoded into types ([RecordEntry.DjiState],
 * [RecordEntry.Field], [RecordEntry.Event]); everything else — `mav_in`, `mav_out`,
 * `stick_cmd`, `rc_stick` — is either skipped before parsing or kept as
 * [RecordEntry.Other] with its raw map, so this reader never has to be edited when a new
 * kind is added to the recorder.
 *
 * **Skipping happens before the JSON parse, not after.** A real session is dominated by
 * MAVLink: the flight this harness was built against is 26 312 lines of which 21 303 are
 * `mav_in`/`mav_out` carrying hex payloads, 8.3 MB of a 8.3 MB file. Parsing those to throw
 * them away is most of the cost of a replay, so [Options.kinds] is applied with a substring
 * test on the raw line first. That is why the reader takes lines rather than a parsed
 * stream.
 */
object FlightRecordReader {

    /** The kinds a state replay needs: the header for its clock anchor, and the two DJI streams. */
    val REPLAY_KINDS: Set<String> = setOf(
        LogEntry.KIND_HEADER,
        LogEntry.KIND_DJI_STATE,
        LogEntry.KIND_DJI_FIELD,
    )

    /** [REPLAY_KINDS] plus the narrative, for a tool that wants to say what was happening. */
    val SUMMARY_KINDS: Set<String> = REPLAY_KINDS + setOf(
        LogEntry.KIND_EVENT,
        LogEntry.KIND_DJI_WARN,
        // Retired kind: sessions up to landing17 wrote `dji_health` for the same thing.
        LogEntry.KIND_DJI_HEALTH,
        LogEntry.KIND_VS_STATE,
    )

    data class Options(
        /** Kinds to decode. Null reads everything, including [RecordEntry.Other]. */
        val kinds: Set<String>? = SUMMARY_KINDS,
        /**
         * What to do with a line that will not parse.
         *
         * Default is to count and continue: a session whose last line was cut off by a dead
         * battery is still worth replaying, and refusing the whole file over its tail would
         * throw away the flight to protect the ending. A caller that wants strictness sets
         * this false and gets the [RecordFormatException].
         */
        val skipBadLines: Boolean = true,
    )

    /**
     * Parses [lines] into a [FlightRecord].
     *
     * Blank lines are skipped silently — a `.jsonl` ending in a newline has one, and it is
     * not damage.
     */
    fun read(lines: Sequence<String>, options: Options = Options()): FlightRecord {
        var header: RecordHeader? = null
        val entries = ArrayList<RecordEntry>(4096)
        var badLines = 0
        var skippedKinds = 0
        for (raw in lines) {
            val line = raw.trim()
            if (line.isEmpty()) continue
            val kind = peekKind(line)
            if (kind != null && options.kinds != null && kind !in options.kinds) {
                skippedKinds++
                continue
            }
            val obj = try {
                RecordJson.parseObject(line)
            } catch (e: RecordFormatException) {
                if (!options.skipBadLines) throw e
                badLines++
                continue
            }
            when (RecordJson.string(obj["k"])) {
                LogEntry.KIND_HEADER -> header = RecordHeader.of(obj)
                LogEntry.KIND_DJI_STATE -> entries.add(RecordEntry.DjiState.of(obj))
                LogEntry.KIND_DJI_FIELD -> entries.add(RecordEntry.Field.of(obj))
                LogEntry.KIND_EVENT -> entries.add(RecordEntry.Event.of(obj))
                null -> if (!options.skipBadLines) throw RecordFormatException("line has no 'k'")
                else -> entries.add(RecordEntry.Other.of(obj))
            }
        }
        return FlightRecord(header, entries, badLines = badLines, skippedByKind = skippedKinds)
    }

    /**
     * The `k` value without parsing the line, or null when the line does not obviously have
     * one. Exact-match on `"k":"…"` — the writer emits no spaces, so this is not a guess
     * about formatting, it is the format.
     */
    fun peekKind(line: String): String? {
        val at = line.indexOf("\"k\":\"")
        if (at < 0) return null
        val start = at + 5
        val end = line.indexOf('"', start)
        if (end < 0) return null
        return line.substring(start, end)
    }
}

/**
 * A parsed session: the header's clock anchor plus the entries, in file order.
 *
 * File order **is** time order. The recorder stamps every entry from one monotonic clock
 * and writes them from a single writer thread, so nothing here re-sorts; a replay that
 * sorted would hide a recorder bug rather than reveal one.
 */
class FlightRecord(
    val header: RecordHeader?,
    val entries: List<RecordEntry>,
    /** Lines that would not parse and were skipped. Zero on a clean file. */
    val badLines: Int = 0,
    /** Lines filtered out by [FlightRecordReader.Options.kinds] before parsing. */
    val skippedByKind: Int = 0,
) {
    val states: List<RecordEntry.DjiState> get() = entries.filterIsInstance<RecordEntry.DjiState>()
    val fields: List<RecordEntry.Field> get() = entries.filterIsInstance<RecordEntry.Field>()
    val events: List<RecordEntry.Event> get() = entries.filterIsInstance<RecordEntry.Event>()

    /** Seconds from the first entry to the last, or 0 for an empty record. */
    val durationSeconds: Double
        get() = if (entries.isEmpty()) 0.0 else entries.last().tSeconds - entries.first().tSeconds
}

/**
 * The session header — everything a replay needs to put the record back on a clock.
 *
 * `t` on every entry is *seconds since [startedMonoNanos]*, which makes entries totally
 * ordered but says nothing about when they happened. [startedUnixMillis] is the wall-clock
 * anchor, and it is the one a `LcmTime` stamp has to come from: LCM headers carry
 * seconds-since-epoch, so replaying with `t` alone would publish a fleet of messages
 * stamped 1970.
 */
data class RecordHeader(
    val format: String?,
    val schema: Long?,
    val session: String?,
    val part: Long?,
    val startedUnixMillis: Long?,
    val startedMonoNanos: Long?,
    /** The recorder's `state_hz` from `meta.recorder`, when it said. */
    val stateHz: Double?,
    val raw: Map<String, Any?>,
) {
    /** The absolute monotonic nanosecond value of an entry at [tSeconds]. */
    fun monoNanosAt(tSeconds: Double): Long? =
        startedMonoNanos?.let { it + Math.round(tSeconds * 1e9) }

    /** The wall-clock instant of an entry at [tSeconds], in Unix milliseconds. */
    fun unixMillisAt(tSeconds: Double): Long? =
        startedUnixMillis?.let { it + Math.round(tSeconds * 1e3) }

    companion object {
        fun of(o: Map<String, Any?>): RecordHeader {
            @Suppress("UNCHECKED_CAST")
            val meta = o["meta"] as? Map<String, Any?>
            @Suppress("UNCHECKED_CAST")
            val rec = meta?.get("recorder") as? Map<String, Any?>
            return RecordHeader(
                format = RecordJson.string(o["format"]),
                schema = RecordJson.long(o["schema"]),
                session = RecordJson.string(o["session"]),
                part = RecordJson.long(o["part"]),
                startedUnixMillis = RecordJson.long(o["started_unix_ms"]),
                startedMonoNanos = RecordJson.long(o["started_mono_ns"]),
                stateHz = RecordJson.number(rec?.get("state_hz")),
                raw = o,
            )
        }
    }
}

/** One decoded line. [tSeconds] is the record's own `t`: seconds since the session started. */
sealed class RecordEntry {

    abstract val tSeconds: Double

    /**
     * A `dji_state` sample — the fast half of `AircraftState` in DJI-native units, with the
     * per-signal ages that make it interpretable.
     *
     * **Absent is null, and null is not zero.** `Json` omits a null rather than writing one,
     * so a sample taken while the aircraft was disconnected has no `lat` key at all, and
     * this class carries that through as a Kotlin null. 509 of the 1855 samples in the
     * reference flight are like that — before takeoff and after the link dropped.
     */
    data class DjiState(
        override val tSeconds: Double,
        val latitude: Double?,
        val longitude: Double?,
        val relativeAltitudeM: Double?,
        val rollDeg: Double?,
        val pitchDeg: Double?,
        val yawDeg: Double?,
        val velocityNorth: Double?,
        val velocityEast: Double?,
        val velocityDown: Double?,
        val positionAgeMs: Long?,
        val altitudeAgeMs: Long?,
        val attitudeAgeMs: Long?,
        val velocityAgeMs: Long?,
    ) : RecordEntry() {
        companion object {
            fun of(o: Map<String, Any?>): DjiState {
                @Suppress("UNCHECKED_CAST")
                val age = o["age"] as? Map<String, Any?> ?: emptyMap()
                return DjiState(
                    tSeconds = RecordJson.number(o["t"]) ?: 0.0,
                    latitude = RecordJson.number(o["lat"]),
                    longitude = RecordJson.number(o["lon"]),
                    relativeAltitudeM = RecordJson.number(o["relalt"]),
                    rollDeg = RecordJson.number(o["roll"]),
                    pitchDeg = RecordJson.number(o["pitch"]),
                    yawDeg = RecordJson.number(o["yaw"]),
                    velocityNorth = RecordJson.number(o["vn"]),
                    velocityEast = RecordJson.number(o["ve"]),
                    velocityDown = RecordJson.number(o["vd"]),
                    positionAgeMs = RecordJson.long(age["pos"]),
                    altitudeAgeMs = RecordJson.long(age["relalt"]),
                    attitudeAgeMs = RecordJson.long(age["att"]),
                    velocityAgeMs = RecordJson.long(age["vel"]),
                )
            }
        }
    }

    /**
     * A `dji_field` line: one slow-changing value, **written only when it changed**.
     *
     * That last clause is the whole reason the [tSeconds] of one of these is not a delivery
     * time. `StateDelta` and `Recorder.field` both filter on value equality, so this stamp
     * says *when the value last became different*, and DJI may have re-delivered the same
     * number a hundred times since. [FlightReplay] reconstructs an age from it anyway,
     * because for an event-driven key an age carries no freshness meaning
     * (`Signal.staleAfterMs` is null) and only presence matters — but for the one deadbanded
     * signal that *does* carry a limit, `TAKEOFF_ALTITUDE`, the reconstruction is an upper
     * bound and is called out in [ReplayCoverage].
     */
    data class Field(
        override val tSeconds: Double,
        val name: String,
        /** The new value as the recorder rendered it. Null means DJI reported no value. */
        val value: String?,
        val previous: String?,
        val numeric: Double?,
    ) : RecordEntry() {
        companion object {
            fun of(o: Map<String, Any?>) = Field(
                tSeconds = RecordJson.number(o["t"]) ?: 0.0,
                name = RecordJson.string(o["f"]) ?: "",
                value = RecordJson.string(o["v"]),
                previous = RecordJson.string(o["prev"]),
                numeric = RecordJson.number(o["n"]),
            )
        }
    }

    /** An `event` line — the narrative. Replayed as context, never as state. */
    data class Event(
        override val tSeconds: Double,
        val code: String,
        val severity: String,
        val message: String?,
    ) : RecordEntry() {
        companion object {
            fun of(o: Map<String, Any?>) = Event(
                tSeconds = RecordJson.number(o["t"]) ?: 0.0,
                code = RecordJson.string(o["code"]) ?: "",
                severity = RecordJson.string(o["sev"]) ?: LogEntry.SEV_INFO,
                message = RecordJson.string(o["msg"]),
            )
        }
    }

    /** Any other kind, kept whole so a reader is never the reason a field is unavailable. */
    data class Other(
        override val tSeconds: Double,
        val kind: String,
        val raw: Map<String, Any?>,
    ) : RecordEntry() {
        companion object {
            fun of(o: Map<String, Any?>) = Other(
                tSeconds = RecordJson.number(o["t"]) ?: 0.0,
                kind = RecordJson.string(o["k"]) ?: "",
                raw = o,
            )
        }
    }
}
