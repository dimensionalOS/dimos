package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.record.FlightRecorder
import com.dimensional.mini4pro.record.LogSink
import com.dimensional.mini4pro.record.MonotonicClock
import com.dimensional.mini4pro.record.RecorderConfig
import com.dimensional.mini4pro.record.SinkFactory
import com.dimensional.mini4pro.record.StateDelta
import com.dimensional.mini4pro.record.WallClock
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.zenoh.Withheld
import com.dimensional.mini4pro.zenoh.ZenohChannel
import com.dimensional.mini4pro.zenoh.Gate
import com.dimensional.mini4pro.zenoh.LcmTime
import com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The harness against the recorder it is the inverse of — **a real round trip, not a
 * reading of the format**.
 *
 * Every case here builds `AircraftState` snapshots, runs them through the *actual*
 * `record/StateDelta` and `record/FlightRecorder` into JSONL lines, and reads those lines
 * back with [FlightRecordReader] and [FlightReplay]. Nothing asserts against a hand-written
 * log line, so a change to either half that the other does not follow fails here rather than
 * on a flight.
 *
 * The two properties worth the most:
 *
 * 1. **A folded `dji_field` value is actually applied** — not merely parsed, not applied one
 *    sample late. `every slow field survives the round trip` walks all nineteen.
 * 2. **A stale signal replays stale.** `SampleAges` is delivered-not-changed, so a value that
 *    sits still while its feed dies is the one thing a naive replay gets wrong, and it is
 *    exactly the case `ZenohTelemetryEncoder` withholds on.
 *
 * ## Mutation-checked, 2026-07-27
 *
 * A test that has never been shown to catch a planted fault is not known to work. Four faults
 * were planted in `replay/FlightReplay.kt`, one at a time, and **all four failed this class**:
 *
 * | planted fault | what it breaks |
 * |---|---|
 * | `fields[next].tSeconds <= entry.tSeconds` → `<` | every slow value lands one sample late |
 * | delete `fold.changedAtSeconds[binding.signal] = …` | event-driven signals never read as heard-from |
 * | delete the `batteryPercent` binding | a recorded value is parsed and then dropped |
 * | `ages[Signal.ALTITUDE] = it` → `= 0L` | a stale signal replays as fresh, and the encoder publishes on a dead feed |
 */
class FlightReplayTest {

    // ── a recorder that writes into memory ───────────────────────────────────

    private class MemorySink(override val name: String) : LogSink {
        val lines = ArrayList<String>()
        override var bytesWritten: Long = 0
            private set

        override fun writeLine(line: String) {
            lines.add(line)
            bytesWritten += line.length + 1
        }

        override fun flush(durable: Boolean) {}
        override fun close() {}
    }

    private class MemoryFactory : SinkFactory {
        val opened = ArrayList<MemorySink>()
        override fun open(session: String, part: Int): LogSink =
            MemorySink("$session.%03d.jsonl".format(part)).also { opened.add(it) }

        override fun prune(session: String, keep: Int): Int = 0
    }

    /**
     * Records [states] at 5 Hz through the real recorder and reads the file back.
     *
     * `startThread = false` and `drainOnce()` keep it synchronous, the same discipline
     * `FlightRecorderTest` uses: no sleeping, and the assertion is about content rather than
     * about timing.
     */
    private fun roundTrip(states: List<AircraftState>, periodMs: Long = 200): FlightRecord {
        var nanos = 47_973_515_798_721L
        var millis = 1_785_137_110_699L
        val factory = MemoryFactory()
        val recorder = FlightRecorder(
            session = "20260727-102510",
            sinks = factory,
            mono = MonotonicClock { nanos },
            wall = WallClock { millis },
            config = RecorderConfig(),
            headerJson = """{"recorder":{"state_hz":5}}""",
        )
        recorder.start(startThread = false)
        val delta = StateDelta()
        for (s in states) {
            for (e in delta.entriesFor(recorder.now(), s)) recorder.record(e)
            recorder.drainOnce()
            nanos += periodMs * 1_000_000
            millis += periodMs
        }
        recorder.drainOnce()
        val lines = factory.opened.flatMap { it.lines }
        return FlightRecordReader.read(lines.asSequence())
    }

    private fun replay(states: List<AircraftState>): List<ReplaySample> =
        FlightReplay.samples(roundTrip(states))

    // ── 1. the fold ──────────────────────────────────────────────────────────

    /**
     * Every slow field, through the recorder and back, distinctly valued so that a binding
     * wired to the wrong `Fold` member is caught rather than papered over by two fields that
     * happen to be equal.
     *
     * Mutation-checked: deleting any one binding from [FlightReplay.FIELD_MAP] leaves that
     * field null here and fails.
     */
    @Test
    fun `every slow field survives the round trip`() {
        val state = AircraftState(
            fcConnected = true,
            latitude = 37.9938461,
            longitude = 23.7253016,
            relativeAltitude = 29.3,
            takeoffAltitudeAmsl = 70.319,
            rollDeg = -1.6,
            pitchDeg = -1.4,
            yawDeg = -103.5,
            velocityNorth = 1.25,
            velocityEast = -2.5,
            velocityDown = 0.75,
            satelliteCount = 19,
            gpsSignalLevel = 4,
            homeLatitude = 37.9938399,
            homeLongitude = 23.7253078,
            isFlying = true,
            motorsOn = true,
            flightMode = "JOYSTICK",
            notAllowMotorStart = false,
            imuWarmingUp = false,
            inFailsafe = false,
            batteryPercent = 73,
            voltageMv = 7997,
            currentMa = -939,
            cellCount = 2,
            cellVoltagesMv = listOf(3999, 3997),
            batteryTempC = 30.6,
            ages = freshAges(),
        )
        val out = replay(listOf(state, state)).last().state

        assertEquals(true, out.fcConnected)
        assertEquals(37.9938461, out.latitude!!, 1e-9)
        assertEquals(23.7253016, out.longitude!!, 1e-9)
        assertEquals(29.3, out.relativeAltitude!!, 1e-6)
        assertEquals(70.319, out.takeoffAltitudeAmsl!!, 1e-6)
        assertEquals(-1.6, out.rollDeg!!, 1e-6)
        assertEquals(-1.4, out.pitchDeg!!, 1e-6)
        assertEquals(-103.5, out.yawDeg!!, 1e-6)
        assertEquals(1.25, out.velocityNorth!!, 1e-6)
        assertEquals(-2.5, out.velocityEast!!, 1e-6)
        assertEquals(0.75, out.velocityDown!!, 1e-6)
        assertEquals(19, out.satelliteCount)
        assertEquals(4, out.gpsSignalLevel)
        assertEquals(37.9938399, out.homeLatitude!!, 1e-9)
        assertEquals(23.7253078, out.homeLongitude!!, 1e-9)
        assertEquals(true, out.isFlying)
        assertEquals(true, out.motorsOn)
        assertEquals("JOYSTICK", out.flightMode)
        assertEquals(false, out.notAllowMotorStart)
        assertEquals(false, out.imuWarmingUp)
        assertEquals(false, out.inFailsafe)
        assertEquals(73, out.batteryPercent)
        assertEquals(7997, out.voltageMv)
        assertEquals(-939, out.currentMa)
        assertEquals(2, out.cellCount)
        assertEquals(listOf(3999, 3997), out.cellVoltagesMv)
        assertEquals(30.6, out.batteryTempC!!, 1e-6)
    }

    /**
     * `homeLocationSet` is the one bound field the round-trip rig above cannot produce, and
     * the reason is worth stating: `record/StateDelta` does not emit it — `record/Recorder`
     * subscribes to `KeyIsHomeLocationSet` directly and writes it under the DJI key's own
     * name, `isHomeLocationSet`. That path runs inside the MSDK adapter and is not on a JVM
     * test's classpath, so this feeds the line the adapter would have written.
     *
     * It matters because it is DJI's own answer to "is there a home point?", and without it
     * the 4.583662361046586E7 filler in `telemetry/Geo` has only the range check standing
     * against it.
     */
    @Test
    fun `isHomeLocationSet folds even though StateDelta never writes it`() {
        val record = FlightRecordReader.read(
            sequenceOf(
                """{"t":0.0,"k":"header","started_mono_ns":0,"started_unix_ms":1000}""",
                """{"t":0.1,"k":"dji_field","f":"isHomeLocationSet","v":"false"}""",
                """{"t":0.2,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""",
                """{"t":0.3,"k":"dji_field","f":"isHomeLocationSet","v":"true","prev":"false"}""",
                """{"t":0.4,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""",
            )
        )
        assertEquals(
            listOf(false, true),
            FlightReplay.samples(record).map { it.state.homeLocationSet },
        )
    }

    /**
     * A slow value that changes mid-flight is applied **from the sample it changed on**, not
     * from the next one.
     *
     * `StateDelta` writes the `dji_field` line *after* the `dji_state` line at the same `t`,
     * so a replay that folded in file order would be one sample — 200 ms — behind on every
     * mode change, every battery step, every home point. Mutation-checked: changing
     * [FlightReplay.samples]' `<=` to `<` makes the third sample read `GPS_ATTI`.
     */
    @Test
    fun `a field change applies to the sample it was recorded with`() {
        val base = AircraftState(fcConnected = true, flightMode = "GPS_ATTI", ages = freshAges())
        val samples = replay(
            listOf(base, base, base.copy(flightMode = "JOYSTICK"), base.copy(flightMode = "JOYSTICK"))
        )
        assertEquals(listOf("GPS_ATTI", "GPS_ATTI", "JOYSTICK", "JOYSTICK"), samples.map { it.state.flightMode })
    }

    /** A field DJI stops reporting comes back as null, not as its last value and not as zero. */
    @Test
    fun `a field going null replays as null, never as the previous value`() {
        val base = AircraftState(fcConnected = true, batteryPercent = 80, ages = freshAges())
        val samples = replay(listOf(base, base.copy(batteryPercent = null)))
        assertEquals(80, samples[0].state.batteryPercent)
        assertNull(samples[1].state.batteryPercent)
    }

    /**
     * Nothing is invented for a field the record does not carry. `goHomeHeightM` is the one
     * `AircraftState` field with no recorder line at all, and it replays as null rather than
     * as 0 — which would be an RTH altitude of ground level.
     */
    @Test
    fun `an unrecorded field replays as null`() {
        val state = AircraftState(fcConnected = true, goHomeHeightM = 60, ages = freshAges())
        assertNull(replay(listOf(state, state)).last().state.goHomeHeightM)
        assertEquals(listOf("goHomeHeightM"), FlightReplay.UNRECORDED)
    }

    /**
     * The binding table matches `record/StateDelta.F` exactly, plus the one name the recorder
     * subscribes to on its own (`isHomeLocationSet`, `record/Recorder.subscribeDji`).
     *
     * Read out of the compiled class rather than retyped, so adding a field to `StateDelta.F`
     * without a binding fails here instead of silently dropping it from every replay.
     */
    @Test
    fun `the binding table covers every recorded field name`() {
        val recorded = StateDelta.F::class.java.declaredFields
            .filter { it.type == String::class.java }
            .map { it.isAccessible = true; it.get(StateDelta.F) as String }
            .toSet()
        val bound = FlightReplay.FIELD_MAP.map { it.name }.toSet()
        assertEquals(
            "every StateDelta.F name needs a FlightReplay binding",
            emptySet<String>(),
            recorded - bound,
        )
        assertEquals(
            "bindings with no StateDelta.F name must come from Recorder.subscribeDji",
            setOf("isHomeLocationSet"),
            bound - recorded,
        )
    }

    /**
     * Every `AircraftState` field is accounted for: carried by `dji_state`, bound in
     * [FlightReplay.FIELD_MAP], or named in [FlightReplay.UNRECORDED].
     *
     * This is the test that keeps the coverage report honest. A new field on `AircraftState`
     * that nothing records is a new hole in offline testing, and it fails here the day it is
     * added rather than the day someone notices a replayed message is missing.
     */
    @Test
    fun `every AircraftState field is carried, bound or declared unrecorded`() {
        val kinematic = setOf(
            "latitude", "longitude", "relativeAltitude",
            "rollDeg", "pitchDeg", "yawDeg",
            "velocityNorth", "velocityEast", "velocityDown",
            "ages",
        )
        val bound = FlightReplay.FIELD_MAP.map { it.stateField }.toSet()
        val declared = FlightReplay.UNRECORDED.toSet()
        val all = AircraftState::class.java.declaredFields
            .filter { !it.isSynthetic && !java.lang.reflect.Modifier.isStatic(it.modifiers) }
            .map { it.name }
            .toSet()
        assertEquals(
            "an AircraftState field nothing accounts for",
            emptySet<String>(),
            all - kinematic - bound - declared,
        )
    }

    /** A name the record carries that no binding claims is counted, not silently swallowed. */
    @Test
    fun `an unmapped field name is reported`() {
        val record = FlightRecordReader.read(
            sequenceOf(
                """{"t":0.0,"k":"header","started_mono_ns":0,"started_unix_ms":1000}""",
                """{"t":0.1,"k":"dji_field","f":"windSpeedDmS","v":"0"}""",
                """{"t":0.2,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""",
            )
        )
        val fold = FlightReplay.Fold()
        record.fields.forEach { FlightReplay.applyField(fold, it) }
        assertEquals(mapOf("windSpeedDmS" to 1), fold.unmappedFields)
    }

    // ── 2. staleness ─────────────────────────────────────────────────────────

    /**
     * **The load-bearing case.** A value that does not change while its feed dies must replay
     * as *stale* — the replay's freshness has to come from the recorded age, never from whether
     * the number moved.
     *
     * The altitude here is 29.3 m on every sample and never moves. Only the recorded age
     * moves, ramping past `Signal.ALTITUDE`'s 1 s limit — which is exactly the shape
     * `KeyAltitude` has in the real flight. A replay that derived freshness from value changes
     * would call all five fresh.
     *
     * Mutation-checked: replacing the recorded `age.relalt` with 0 in [FlightReplay.sampleAt]
     * — i.e. "the value has not changed, so call it fresh" — makes every reason `PUBLISHED`,
     * and this test is what fails.
     *
     * **What the staleness then costs depends on the channel, and that is the second half.**
     * `odom` refuses it, because something closes a loop on `odom`. `gps_location` repeats it
     * for as long as the link is up, because this is precisely the case a change-driven key
     * cannot distinguish from a healthy aircraft that is holding still — and withholding here
     * is what measured 45.3 % pose coverage on a flight that was hovering over a tag.
     */
    @Test
    fun `a frozen value with a dying feed replays stale, and only the fresh gate refuses it`() {
        val ages = longArrayOf(100, 900, 1_100, 3_500, 5_700)
        val states = ages.map { age ->
            AircraftState(
                fcConnected = true,
                latitude = 37.9938461,
                longitude = 23.7253016,
                relativeAltitude = 29.3,
                ages = SampleAges.of(
                    Signal.POSITION to 10L,
                    Signal.ALTITUDE to age,
                ),
            )
        }
        val samples = replay(states)
        assertEquals(ages.toList(), samples.map { it.state.ageMs(Signal.ALTITUDE) })
        assertEquals(
            listOf(true, true, false, false, false),
            samples.map { it.state.isFresh(Signal.ALTITUDE) },
        )
        assertEquals(
            listOf(
                Withheld.PUBLISHED,
                Withheld.PUBLISHED,
                Withheld.ALTITUDE_STALE,
                Withheld.ALTITUDE_STALE,
                Withheld.ALTITUDE_STALE,
            ),
            samples.map { ZenohReplay.altitudeReason(it) },
        )
        // Under the held gate the same five samples are all publishable: the link is up, and
        // nothing here says the aircraft is gone.
        assertEquals(
            List(5) { Withheld.PUBLISHED },
            samples.map { ZenohReplay.altitudeReason(it, Gate.HELD) },
        )

        // And the encoder itself agrees, which is what the reasons are a mirror of.
        fun publishedCount(states: List<ReplaySample>) = states.count {
            ZenohTelemetryEncoder.gpsLocationOrNull(it.state, LcmTime.ZERO) != null
        }
        assertEquals("a live link repeats the last reading", 5, publishedCount(samples))
        assertEquals(
            "a dead link publishes nothing — the held value has lost its only witness",
            0,
            publishedCount(replay(states.map { it.copy(fcConnected = false) })),
        )
    }

    /**
     * A signal that was never delivered replays as never delivered — absent from
     * [SampleAges], not present with a large number. `isFresh` says false for both, but
     * `ageMs` must distinguish them: "we have never heard from the aircraft" and "the feed
     * stopped an hour ago" are different diagnoses.
     */
    @Test
    fun `a never-delivered signal stays absent rather than becoming old`() {
        val state = AircraftState(
            fcConnected = true,
            latitude = 37.99,
            longitude = 23.72,
            ages = SampleAges.of(Signal.POSITION to 10L),
        )
        val out = replay(listOf(state, state)).last().state
        assertEquals(10L, out.ageMs(Signal.POSITION))
        assertNull("VELOCITY was never delivered", out.ageMs(Signal.VELOCITY))
        assertFalse(out.isFresh(Signal.VELOCITY))
    }

    /**
     * An event-driven signal's reconstructed age is *time since it was last recorded*, and it
     * is only ever asked whether it was heard at all — `Signal.staleAfterMs` is null for all
     * of them, so `isFresh` reduces to presence. Both halves are pinned: the number is what
     * it claims to be, and the freshness answer does not depend on it.
     */
    @Test
    fun `an event-driven signal is fresh once heard, whatever its age`() {
        val base = AircraftState(fcConnected = true, batteryPercent = 80, ages = freshAges())
        val samples = replay(List(6) { base })
        val last = samples.last().state
        // 5 periods of 200 ms since the value was recorded on the first sample.
        assertEquals(1_000L, last.ageMs(Signal.BATTERY_PERCENT))
        assertTrue(last.isFresh(Signal.BATTERY_PERCENT))
        assertNull(Signal.BATTERY_PERCENT.staleAfterMs)
    }

    /** With reconstruction off, only the four recorded ages survive — the strict reading. */
    @Test
    fun `event-driven ages can be withheld entirely`() {
        val base = AircraftState(fcConnected = true, batteryPercent = 80, ages = freshAges())
        val samples = FlightReplay.samples(roundTrip(listOf(base, base)), eventDrivenAges = false)
        val last = samples.last().state
        assertNull(last.ageMs(Signal.BATTERY_PERCENT))
        assertEquals(80, last.batteryPercent)
        assertNotNull(last.ageMs(Signal.POSITION))
    }

    // ── 3. the clocks ────────────────────────────────────────────────────────

    /**
     * A sample carries the wall-clock instant it happened at, not the instant it was
     * replayed. An LCM header stamps seconds since the epoch, so a replay that used its own
     * clock would produce a fixture dated whenever it last ran.
     */
    @Test
    fun `samples carry the recording clock, not the replay clock`() {
        val base = AircraftState(fcConnected = true, ages = freshAges())
        val record = roundTrip(listOf(base, base, base))
        val samples = FlightReplay.samples(record)
        assertEquals(1_785_137_110_699L, record.header!!.startedUnixMillis)
        assertEquals(listOf(1_785_137_110_699L, 1_785_137_110_899L, 1_785_137_111_099L), samples.map { it.unixMillis })
        assertEquals(47_973_515_798_721L, samples.first().monoNanos)
        assertEquals(0.4, samples.last().tSeconds, 1e-6)
    }

    /** No header, no clock — and null rather than an invented epoch. */
    @Test
    fun `a headerless record still replays, with null stamps`() {
        val record = FlightRecordReader.read(
            sequenceOf("""{"t":1.5,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""")
        )
        val sample = FlightReplay.samples(record).single()
        assertNull(sample.unixMillis)
        assertNull(sample.monoNanos)
        assertEquals(1.5, sample.tSeconds, 1e-9)
    }

    // ── 4. the reader's own tolerance ────────────────────────────────────────

    /** A truncated tail costs the tail, not the flight. */
    @Test
    fun `a corrupt line is counted and skipped`() {
        val record = FlightRecordReader.read(
            sequenceOf(
                """{"t":0.0,"k":"header","started_mono_ns":0,"started_unix_ms":1000}""",
                """{"t":0.2,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""",
                """{"t":0.4,"k":"dji_state","lat":37.9""",
            )
        )
        assertEquals(1, record.badLines)
        assertEquals(1, record.states.size)
    }

    /** Kinds outside the filter never reach the parser at all — that is where the cost is. */
    @Test
    fun `unwanted kinds are skipped before parsing`() {
        val record = FlightRecordReader.read(
            sequenceOf(
                """{"t":0.0,"k":"header","started_mono_ns":0,"started_unix_ms":1000}""",
                """{"t":0.1,"k":"mav_out","name":"HEARTBEAT","hex":"not even valid json past here""",
                """{"t":0.2,"k":"dji_state","lat":37.9,"lon":23.7,"age":{"pos":10}}""",
            ),
            FlightRecordReader.Options(kinds = FlightRecordReader.REPLAY_KINDS),
        )
        assertEquals(0, record.badLines)
        assertEquals(1, record.skippedByKind)
        assertEquals(1, record.states.size)
    }

    private fun freshAges() = SampleAges.of(
        Signal.POSITION to 10L,
        Signal.ALTITUDE to 10L,
        Signal.ATTITUDE to 10L,
        Signal.VELOCITY to 10L,
    )
}
