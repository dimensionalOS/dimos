package com.dimensional.mini4pro.warn

import com.dimensional.mini4pro.zenoh.LcmTime
import com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **The owner's own suite: every surface is fed from one object, and none of them may be fed from
 * anywhere else.**
 *
 * This is the file that holds the rule `CLAUDE.md` states — that every DJI warning source joins one
 * owner and reaches all four surfaces. The properties are deliberately about *coincidence* rather
 * than about content: what a sentence says is `WarnStatusTextsTest`'s, what a level costs is
 * `WarningMonitorTest`'s, and what is tested here is that QGC, the phone, the record and the bus
 * cannot come apart.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-30, one breakage at a time applied to the shipped `src/main`, the
 * **whole** suite run per mutant with `test-results` deleted first, confirmed red, reverted.
 * Counts are failing tests across all 2701 — **measured, not estimated**. **No survivors.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a source bypasses the owner (device health returns before delivering its picture) | 7 |
 *  | the QGC sink dropped — the bus is told, the operator is not | 13 |
 *  | announced every delivery instead of on edges | 11 |
 *  | the severity mapping inverted (`WARNING`↔`NOTICE`, `SERIOUS_WARNING`↔`CAUTION`) | 8 |
 *  | the Zenoh sink dropped — QGC is told, the bus is not | 5 |
 *  | clears not announced (silence instead of a retraction) | 3 |
 *  | the measured speed omitted from the sentence when known | 2 |
 *  | the byte budget bypassed (the measurement appended past 50 bytes) | 5 |
 *  | `LEVEL_0` mapped to a forwardable level (every calm reading announced) | 5 |
 *
 * ### The counts worth reading rather than counting
 *
 * **The QGC sink scores 13 and the per-sample repeat scores 11**, which is the right shape for a
 * fan-out: the two mutants that change *what an operator experiences* — silence, or a flood — are
 * the ones the most tests are arranged around, and they fail across three packages (`warn/`,
 * `health/`, and the wind source's own file) because all three go through this one door.
 *
 * **A source bypassing the owner scores 7, and every one of them is somebody else's test.** The
 * health package's wiring suite fails, not this file, which is exactly what a single owner should
 * look like: the bypass is not detectable *here* — the bus simply never hears about it — and is
 * caught by the tests that assert a delivery reaches the surfaces at all. A row that failed only
 * in this file would mean the ownership was being tested by inspection.
 *
 * **Clears score 3 and the measurement scores 2, and both are small on purpose.** They are narrow
 * properties with one test each plus their reach into the wind source's own suite; a large number
 * here would mean the retraction rule had been restated in several places rather than living in
 * `WarningMonitor.event`.
 *
 * **The inverted severity table scores 8** — the mutant that changes nothing observable except how
 * loud each message is. It is silent in the worst way: every warning still arrives, the record
 * still has a line, and the only symptom is a pilot who did not react to a red modal that was
 * printed as a notice.
 */
class WarningBusTest {

    private class Surfaces {
        val qgc = mutableListOf<Pair<String, Int>>()
        val screen = mutableListOf<WarnEvent>()
        val record = mutableListOf<WarnEvent>()
        val bus = mutableListOf<WarnEvent>()
        val logged = mutableListOf<WarnEvent>()
        val notes = mutableListOf<String>()
        var qgcThrows = false
        var recordThrows = false
    }

    private var now = 0L

    private fun bus(s: Surfaces) = WarningBus(
        qgc = { text, severity ->
            if (s.qgcThrows) throw IllegalStateException("qgc")
            s.qgc += text to severity
        },
        screen = { s.screen += it },
        record = {
            if (s.recordThrows) throw IllegalStateException("record")
            s.record += it
        },
        bus = { s.bus += it },
        log = { s.logged += it },
        note = { s.notes += it },
        nowMs = { now },
    )

    private fun overheat(level: WarnLevel) = Warning(
        source = WarnSource.DEVICE_HEALTH,
        code = "0x1600A0",
        state = level.name,
        level = level,
        title = "Aircraft overheating",
        componentId = 0,
        sensorIndex = 0,
    )

    /**
     * **One event, four surfaces, one object.** The sentence QGC gets, the sentence the phone gets,
     * the sentence the record keeps and the sentence the bus carries are the *same string* from the
     * *same event* — not four renderings that happen to agree today.
     */
    @Test
    fun an_announced_warning_reaches_every_surface_from_the_one_event() {
        val s = Surfaces()
        bus(s).deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))

        val event = s.record.single()
        assertEquals(event.text to WarnLevel.MAV_SEVERITY_ERROR, s.qgc.single())
        assertEquals(event, s.screen.single())
        assertEquals(event, s.bus.single())
        assertEquals(event, s.logged.single())
        assertEquals("DJI: Aircraft overheating", event.text)
    }

    /**
     * **The record and the bus get what QGC does not.** A `NORMAL` reading is never sent to a
     * ground station by design and a rate-limited one never made it — and both are evidence, so
     * both are kept.
     *
     * This is the asymmetry `WarnEvent.announce` and `rateLimited` exist for, and losing it is the
     * failure the whole package was built against: a warning that reached nobody *and* left no
     * trace.
     */
    @Test
    fun the_record_and_the_bus_carry_what_the_operator_is_never_told() {
        val s = Surfaces()
        val b = bus(s)

        b.deliver(WarnSource.WIND, WindWarnings.picture("LEVEL_0", 61))
        assertTrue("a calm reading must not reach QGC", s.qgc.isEmpty())
        assertTrue("nor the pilot's screen", s.screen.isEmpty())
        assertEquals(1, s.record.size)
        assertEquals(1, s.bus.size)
        assertNull(s.bus.single().mavSeverity)
        assertEquals(WarnLevel.DIAGNOSTIC_OK, s.bus.single().diagnosticLevel)

        // And a flood: the wire is bounded, the evidence is not.
        val flood = (1..RateBound.CAPACITY + 3).map {
            Warning(
                source = WarnSource.DEVICE_HEALTH,
                code = "flap-$it",
                state = "CAUTION",
                level = WarnLevel.CAUTION,
                title = "Flapping $it",
            )
        }
        b.deliver(WarnSource.DEVICE_HEALTH, flood)
        assertEquals("the wire is bounded", RateBound.CAPACITY, s.qgc.size)
        assertEquals(1 + flood.size, s.record.size)
        assertEquals(s.record.size, s.bus.size)
        assertTrue(s.record.any { it.rateLimited })
    }

    /**
     * **Edge-triggered, never per sample.** DJI re-delivers its whole picture whenever anything in
     * it moves and a key listener fires on every value; fifty identical deliveries are one warning.
     */
    @Test
    fun a_repeated_delivery_reaches_no_surface() {
        val s = Surfaces()
        val b = bus(s)
        val picture = listOf(overheat(WarnLevel.WARNING))
        b.deliver(WarnSource.DEVICE_HEALTH, picture)
        val after = listOf(s.qgc.size, s.screen.size, s.record.size, s.bus.size)

        repeat(50) { b.deliver(WarnSource.DEVICE_HEALTH, picture) }

        assertEquals(after, listOf(s.qgc.size, s.screen.size, s.record.size, s.bus.size))
        assertEquals(1, after[0])
    }

    /**
     * **A clear is announced.** Silence does not retract a message: the operator's screen still
     * says the aircraft is overheating until something says otherwise, and it goes out at INFO so
     * the retraction never reads as a new emergency.
     */
    @Test
    fun a_clear_is_announced_on_every_surface_at_info() {
        val s = Surfaces()
        val b = bus(s)
        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))
        b.deliver(WarnSource.DEVICE_HEALTH, emptyList())

        assertEquals(2, s.qgc.size)
        assertEquals("DJI cleared: Aircraft overheating" to WarnLevel.MAV_SEVERITY_INFO, s.qgc.last())
        assertEquals(WarnChange.CLEARED, s.record.last().change)
        assertEquals(WarnLevel.DIAGNOSTIC_OK, s.bus.last().diagnosticLevel)
    }

    /**
     * The Zenoh message is built from the same event, and its level is the event's rather than a
     * second reading of the DJI word.
     *
     * `ZenohTelemetryEncoder.warning` is the only place that turns an event into bytes, and this
     * pins that it copies rather than decides — a second severity table on the bus side is exactly
     * how a robot and a pilot come to disagree about how bad something is.
     */
    @Test
    fun the_bus_message_is_the_events_own_decision() {
        val s = Surfaces()
        bus(s).deliver(WarnSource.WIND, WindWarnings.picture("LEVEL_2", 142))
        val event = s.bus.single()

        val message = ZenohTelemetryEncoder.warning(event, LcmTime(0, 0), seq = 1)
        val status = message.status.single()
        assertEquals(event.diagnosticLevel, status.level)
        assertEquals(event.text, status.message)
        assertEquals("wind/windWarning", status.name)
        assertEquals("LEVEL_2", status.values.single { it.key == "state" }.value)
        assertEquals("WARNING", status.values.single { it.key == "level" }.value)
        assertEquals("14.2 m/s", status.values.single { it.key == "measurement" }.value)
        assertEquals("true", status.values.single { it.key == "forwarded" }.value)
    }

    /**
     * A failing surface must not take the others with it, and must not throw.
     *
     * DJI delivers on the Android main thread. An exception escaping this path would kill the
     * process — which this project has already done once, by letting a DJI callback reach a UDP
     * send on the main thread.
     */
    @Test
    fun a_throwing_surface_does_not_stop_the_others() {
        val s = Surfaces()
        s.recordThrows = true
        val b = bus(s)
        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))
        assertEquals(1, s.qgc.size)
        assertEquals(1, s.bus.size)
        assertTrue(s.notes.any { it.contains("recorder") })

        s.recordThrows = false
        s.qgcThrows = true
        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.SERIOUS_WARNING)))
        assertEquals("the record survived a failing wire", 1, s.record.size)
        assertEquals(2, s.bus.size)
        assertTrue(s.notes.any { it.contains("STATUSTEXT") })
    }

    /** The screen reads the standing picture from the bus, so it cannot show a stale set. */
    @Test
    fun the_snapshot_is_every_sources_current_picture() {
        val s = Surfaces()
        val b = bus(s)
        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))
        b.deliver(WarnSource.WIND, WindWarnings.picture("LEVEL_2", 142))
        assertEquals(2, b.snapshot().size)
        assertEquals("DJI: Strong wind 14.2 m/s", b.lastText)

        b.deliver(WarnSource.DEVICE_HEALTH, emptyList())
        assertEquals(listOf(WarnSource.WIND), b.snapshot().map { it.source })
        assertFalse(b.snapshot().any { it.source == WarnSource.DEVICE_HEALTH })
    }

    /** A reset re-announces the picture rather than silently withholding it. */
    @Test
    fun a_reset_makes_the_next_delivery_a_fresh_appearance() {
        val s = Surfaces()
        val b = bus(s)
        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))
        b.reset()
        assertTrue(b.snapshot().isEmpty())
        assertNull(b.lastText)

        b.deliver(WarnSource.DEVICE_HEALTH, listOf(overheat(WarnLevel.WARNING)))
        assertEquals(WarnChange.APPEARED, s.record.last().change)
        assertEquals(2, s.qgc.size)
    }
}
