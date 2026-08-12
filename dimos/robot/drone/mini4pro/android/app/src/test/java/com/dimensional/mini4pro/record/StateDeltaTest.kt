package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The cadence split: fast fields at rate, slow fields on change.
 *
 * This is where the file size is decided, and also where a mode change becomes a
 * discrete timestamped line rather than something to be inferred from a diff. Both
 * halves of that are tested — the first by counting entries when nothing changes,
 * the second by asserting the change carries its previous value.
 */
class StateDeltaTest {

    private val ground = AircraftState(
        fcConnected = true,
        latitude = 37.9938612,
        longitude = 23.7253298,
        relativeAltitude = 0.0,
        takeoffAltitudeAmsl = 103.1696,
        rollDeg = -1.0,
        pitchDeg = 0.0,
        yawDeg = -121.1,
        velocityNorth = 0.0,
        velocityEast = 0.0,
        velocityDown = 0.0,
        satelliteCount = 17,
        gpsSignalLevel = 5,
        homeLatitude = 37.9938872,
        homeLongitude = 23.7253295,
        isFlying = false,
        motorsOn = false,
        flightMode = "APAS",
        notAllowMotorStart = false,
        imuWarmingUp = false,
        inFailsafe = false,
        batteryPercent = 98,
        voltageMv = 8371,
        currentMa = -905,
        cellCount = 2,
        cellVoltagesMv = listOf(4186, 4183),
        batteryTempC = 37.5,
    )

    private fun fields(entries: List<LogEntry>) =
        entries.filterIsInstance<LogEntry.Field>().map { it.name }

    @Test
    fun `the first sample emits every field it has, so nothing is implicit`() {
        val d = StateDelta()
        val entries = d.entriesFor(0, ground)
        assertEquals(1, entries.count { it is LogEntry.DjiState })
        val names = fields(entries)
        assertTrue(names.contains(StateDelta.F.FLIGHT_MODE))
        assertTrue(names.contains(StateDelta.F.VOLTAGE_MV))
        assertTrue(names.contains(StateDelta.F.CELL_VOLTAGES))
        assertTrue(names.contains(StateDelta.F.TAKEOFF_ALT_AMSL))
        // no narrative events on the very first sample — there is nothing to change from
        assertEquals(0, entries.count { it is LogEntry.Event })
    }

    @Test
    fun `an unchanged aircraft costs exactly one dji_state per sample`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        val second = d.entriesFor(1, ground)
        assertEquals(1, second.size)
        assertTrue(second[0] is LogEntry.DjiState)
    }

    @Test
    fun `position and attitude are always in dji_state, never as slow fields`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        val moved = d.entriesFor(1, ground.copy(latitude = 37.99, rollDeg = 12.0, velocityEast = 2.0))
        // The whole fast set rides in the one dji_state entry, so a moving aircraft
        // still costs one line per sample.
        assertEquals(1, moved.size)
        val state = moved[0] as LogEntry.DjiState
        assertEquals(12.0, state.rollDeg!!, 1e-9)
        assertEquals(2.0, state.velocityEast!!, 1e-9)
    }

    @Test
    fun `a mode change is a discrete line carrying its previous value, plus an event`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        val entries = d.entriesFor(1, ground.copy(flightMode = "VIRTUAL_STICK"))
        val field = entries.filterIsInstance<LogEntry.Field>().single()
        assertEquals(StateDelta.F.FLIGHT_MODE, field.name)
        assertEquals("VIRTUAL_STICK", field.value)
        assertEquals("APAS", field.previous)
        val event = entries.filterIsInstance<LogEntry.Event>().single()
        assertEquals(EventCode.MODE_CHANGE, event.code)
        assertEquals("flightMode: APAS -> VIRTUAL_STICK", event.message)
    }

    @Test
    fun `failsafe is an error-severity event`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        val event = d.entriesFor(1, ground.copy(inFailsafe = true))
            .filterIsInstance<LogEntry.Event>().single()
        assertEquals(LogEntry.SEV_ERROR, event.severity)
    }

    @Test
    fun `the barometric AMSL datum's drift is absorbed by its deadband`() {
        // Measured: KeyTakeoffLocationAltitude drifted 102.49 -> 103.17 (0.68 m)
        // while the aircraft sat still. Without a deadband it would emit at rate.
        val d = StateDelta()
        d.entriesFor(0, ground.copy(takeoffAltitudeAmsl = 103.0))
        assertFalse(fields(d.entriesFor(1, ground.copy(takeoffAltitudeAmsl = 103.1)))
            .contains(StateDelta.F.TAKEOFF_ALT_AMSL))
        assertFalse(fields(d.entriesFor(2, ground.copy(takeoffAltitudeAmsl = 103.2)))
            .contains(StateDelta.F.TAKEOFF_ALT_AMSL))
        // ...but a real 1 m move is not absorbed
        assertTrue(fields(d.entriesFor(3, ground.copy(takeoffAltitudeAmsl = 104.0)))
            .contains(StateDelta.F.TAKEOFF_ALT_AMSL))
    }

    @Test
    fun `battery deadbands filter noise but not a motor spooling up`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        // 20 mV of pack ripple: below the 50 mV band
        assertFalse(fields(d.entriesFor(1, ground.copy(voltageMv = 8391))).contains(StateDelta.F.VOLTAGE_MV))
        // a 400 mV sag under load: reported
        assertTrue(fields(d.entriesFor(2, ground.copy(voltageMv = 7971))).contains(StateDelta.F.VOLTAGE_MV))
        // 905 mA idle -> 9 A of motor draw: reported
        assertTrue(fields(d.entriesFor(3, ground.copy(currentMa = -9000))).contains(StateDelta.F.CURRENT_MA))
    }

    @Test
    fun `cell voltages are recorded as the raw list so imbalance stays visible`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        // 10 mV per cell: inside the band
        assertFalse(fields(d.entriesFor(1, ground.copy(cellVoltagesMv = listOf(4176, 4173))))
            .contains(StateDelta.F.CELL_VOLTAGES))
        // one cell dropping 200 mV while the other holds — the condition per-cell
        // reporting exists to expose
        val entries = d.entriesFor(2, ground.copy(cellVoltagesMv = listOf(3986, 4183)))
        val field = entries.filterIsInstance<LogEntry.Field>().single { it.name == StateDelta.F.CELL_VOLTAGES }
        assertEquals("3986,4183", field.value)
        assertEquals("4186,4183", field.previous)
    }

    @Test
    fun `a field going null is recorded as a change, because null means component gone`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        val field = d.entriesFor(1, ground.copy(flightMode = null))
            .filterIsInstance<LogEntry.Field>().single()
        assertEquals(StateDelta.F.FLIGHT_MODE, field.name)
        assertEquals(null, field.value)
        assertEquals("APAS", field.previous)
    }

    @Test
    fun `reset re-emits everything, so a reconnect is never silently identical`() {
        val d = StateDelta()
        d.entriesFor(0, ground)
        assertEquals(1, d.entriesFor(1, ground).size)
        d.reset()
        assertTrue(
            "after a reconnect the mode must be restated even if it is unchanged",
            fields(d.entriesFor(2, ground)).contains(StateDelta.F.FLIGHT_MODE),
        )
    }

    // ── freezes: the change that does not look like a change ──────────────────

    /** Every continuous feed arriving normally, with velocity's age dialled in. */
    private fun ages(velocityAgeMs: Long) = SampleAges.of(
        Signal.POSITION to 20L,
        Signal.ALTITUDE to 20L,
        Signal.TAKEOFF_ALTITUDE to 100L,
        Signal.ATTITUDE to 520L,
        Signal.VELOCITY to velocityAgeMs,
    )

    private fun events(entries: List<LogEntry>) = entries.filterIsInstance<LogEntry.Event>()

    @Test
    fun `a velocity feed that silently freezes is announced, though no value changes`() {
        // THE bug class. A 5 Hz session in which KeyAircraftVelocity stops being
        // delivered two seconds in. Every AircraftState below carries exactly the
        // same numbers — a stationary aircraft and a dead feed are byte-identical
        // — so on values alone this session is one dji_state line per sample and
        // nothing else, forever.
        val d = StateDelta()
        var velocityAge = 100L
        var announced: LogEntry.Event? = null
        for (i in 0 until 50) {                       // 10 s at 5 Hz
            val t = i * 200_000_000L                  // ns
            velocityAge = if (i < 10) 100L else velocityAge + 200L
            val entries = d.entriesFor(t, ground.copy(ages = ages(velocityAge)))
            val stale = events(entries).firstOrNull { it.code == EventCode.SIGNAL_STALE }
            if (stale != null && announced == null) announced = stale
        }
        assertNotNull("a frozen velocity feed left no trace in the log", announced)
        assertEquals(LogEntry.SEV_WARN, announced!!.severity)
        assertTrue(announced.message!!.startsWith("VELOCITY: no update for "))
        // Caught within one sample of the limit: stale at 2000 ms, samples 200 ms
        // apart, first sample past the limit is the one at 2100 ms.
        assertEquals("""{"sig":"VELOCITY","age_ms":2100,"limit_ms":2000}""", announced.dataJson)
    }

    @Test
    fun `a re-delivered identical value is fresh, and is never announced`() {
        // The wrong-clock trap, pinned. These 50 samples carry the same zero
        // velocity as the test above — a genuinely motionless aircraft whose key
        // keeps arriving. An implementation that watched for value *changes*
        // rather than *deliveries* would flag this exactly as it flagged the
        // frozen feed, and would be wrong.
        val d = StateDelta()
        val all = ArrayList<LogEntry>()
        for (i in 0 until 50) all += d.entriesFor(i * 200_000_000L, ground.copy(ages = ages(100L)))
        assertTrue(events(all).none { it.code == EventCode.SIGNAL_STALE })
        assertTrue(events(all).none { it.code == EventCode.SIGNAL_FRESH })
    }

    @Test
    fun `staleness is a transition, not a per-sample flag`() {
        // A stale signal stays stale for the rest of the flight. One line per
        // sample would be the disk cost the cadence split exists to avoid.
        val d = StateDelta()
        d.entriesFor(0, ground.copy(ages = ages(100L)))
        val first = events(d.entriesFor(1, ground.copy(ages = ages(9_000L))))
        assertEquals(1, first.size)
        assertEquals(EventCode.SIGNAL_STALE, first.single().code)
        for (i in 2..20) {
            assertTrue(
                "sample $i re-announced a signal that was already stale",
                events(d.entriesFor(i.toLong(), ground.copy(ages = ages(9_000L + i * 200L)))).isEmpty(),
            )
        }
    }

    @Test
    fun `a feed coming back is announced too, at info severity`() {
        val d = StateDelta()
        d.entriesFor(0, ground.copy(ages = ages(100L)))
        d.entriesFor(1, ground.copy(ages = ages(9_000L)))
        val back = events(d.entriesFor(2, ground.copy(ages = ages(40L)))).single()
        assertEquals(EventCode.SIGNAL_FRESH, back.code)
        assertEquals(LogEntry.SEV_INFO, back.severity)
        assertEquals("""{"sig":"VELOCITY","age_ms":40,"limit_ms":2000}""", back.dataJson)
    }

    @Test
    fun `a signal already stale the first time we look says so once`() {
        // The recorder is started before the aircraft is; the first state it sees
        // with a live-but-quiet key is a real finding, not a start-up artefact.
        val d = StateDelta()
        val first = events(d.entriesFor(0, ground.copy(ages = ages(30_000L))))
        assertEquals(EventCode.SIGNAL_STALE, first.single().code)
        assertTrue(events(d.entriesFor(1, ground.copy(ages = ages(30_200L)))).isEmpty())
    }

    @Test
    fun `a signal never delivered is not called stale — its value is already absent`() {
        // Before MSDK registration every age is unknown. Warning about all five
        // continuous signals on every session start is the "train them to ignore
        // it" failure; the null values already say there is no reading.
        val d = StateDelta()
        val entries = d.entriesFor(0, AircraftState())
        assertTrue(events(entries).none { it.code == EventCode.SIGNAL_STALE })
        assertTrue(events(d.entriesFor(1, AircraftState())).isEmpty())
    }

    @Test
    fun `event-driven keys are never called stale, however long they stay quiet`() {
        // KeyFCFlightMode / KeyHomeLocation / KeyIsFailSafe arrive on change. An
        // hour of silence from those is an aircraft doing nothing unusual.
        val d = StateDelta()
        val quiet = SampleAges.of(
            Signal.FLIGHT_MODE to 3_600_000L,
            Signal.HOME to 3_600_000L,
            Signal.FAILSAFE to 3_600_000L,
            Signal.BATTERY_TEMP to 3_600_000L,
        )
        d.entriesFor(0, ground.copy(ages = quiet))
        assertTrue(events(d.entriesFor(1, ground.copy(ages = quiet))).isEmpty())
    }

    @Test
    fun `reset forgets staleness too, so a reconnect re-states it`() {
        val d = StateDelta()
        d.entriesFor(0, ground.copy(ages = ages(100L)))
        d.entriesFor(1, ground.copy(ages = ages(9_000L)))
        assertTrue(events(d.entriesFor(2, ground.copy(ages = ages(9_200L)))).isEmpty())
        d.reset()
        assertEquals(
            EventCode.SIGNAL_STALE,
            events(d.entriesFor(3, ground.copy(ages = ages(9_400L))))
                .single { it.code == EventCode.SIGNAL_STALE }.code,
        )
    }

    @Test
    fun `staleness events are urgent, so a freeze survives the crash that caused it`() {
        assertTrue(LogEntry.Event(0, EventCode.SIGNAL_STALE).urgent)
    }

    @Test
    fun `an all-null state still produces a dji_state line`() {
        // "The recorder produced nothing" and "the aircraft never connected" must be
        // distinguishable afterwards.
        val entries = StateDelta().entriesFor(0, AircraftState())
        assertTrue(entries.any { it is LogEntry.DjiState })
        assertTrue(fields(entries).contains(StateDelta.F.FC_CONNECTED))
    }
}
