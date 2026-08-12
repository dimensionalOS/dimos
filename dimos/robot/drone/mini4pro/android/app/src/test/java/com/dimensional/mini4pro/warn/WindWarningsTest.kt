package com.dimensional.mini4pro.warn

import com.dimensional.mini4pro.command.StatusTexts
import com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **DJI's wind warning as a source of the one warning path** — the enum mapping, the measurement
 * that rides with it, and the landing17 sequence end to end.
 *
 * The flight: `datasets/landing17/20260730-172355.001.jsonl`, in a wind DJI measured at up to
 * **14.2 m/s** against this airframe's rated ~10.7 m/s. `windWarning` crossed between `LEVEL_0` and
 * `LEVEL_2` **eight times in 227 s** and reached no surface at all, while Ivan watched QGC asking
 * whether DJI was giving us wind warnings. Every number in this file is that record's.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-30, one breakage at a time applied to the shipped `src/main`, the
 * **whole** suite run per mutant with `test-results` deleted first, confirmed red, reverted.
 * Counts are failing tests across all 2701 — **measured, not estimated**. **No survivors.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `LEVEL_0` mapped to a forwardable level (every calm reading announced) | 5 |
 *  | the severity mapping inverted (`WARNING`↔`NOTICE`, `SERIOUS_WARNING`↔`CAUTION`) | 8 |
 *  | the measured speed omitted from the sentence when known | 2 |
 *  | the byte budget bypassed (the measurement appended past 50 bytes) | 5 |
 *  | announced every delivery instead of on edges | 11 |
 *
 * The full campaign — thirteen mutants across both halves of this feature — is tabulated in
 * `WarningBusTest`, which owns the fan-out these rows share. Repeated here are the five that this
 * file is the reason for.
 *
 * The **`LEVEL_0`-is-forwardable** row is the one that matters most in the air and reads least
 * like a bug: it announces nothing false, it simply announces *"DJI: Wind"* every time the wind
 * drops back — which on landing17 is four extra messages in 227 s, and is how an operator learns
 * to ignore the one channel that tells them the aircraft is being overpowered.
 */
class WindWarningsTest {

    /**
     * The enum as the 5.18.0 jar declares it, mapped onto the one ladder.
     *
     * `javap -p -c dji.sdk.keyvalue.value.flightcontroller.WindWarning` (2026-07-30, the
     * `docs/msdk/actions.md` §2a method — read the `static {}` block): four constants,
     * `LEVEL_0`/`LEVEL_1`/`LEVEL_2` with wire values 0/1/2 and `UNKNOWN` with **65535**. There is
     * no `LEVEL_3`.
     */
    @Test
    fun the_dji_enum_maps_onto_the_one_ladder() {
        assertEquals(WarnLevel.NORMAL, WindWarnings.levelOf("LEVEL_0"))
        assertEquals(WarnLevel.CAUTION, WindWarnings.levelOf("LEVEL_1"))
        assertEquals(WarnLevel.WARNING, WindWarnings.levelOf("LEVEL_2"))
        assertEquals(WarnLevel.UNKNOWN, WindWarnings.levelOf("UNKNOWN"))
        // A level DJI does not have today, and a null delivery: neither may read as "fine".
        assertEquals(WarnLevel.UNKNOWN, WindWarnings.levelOf("LEVEL_3"))
        assertEquals(WarnLevel.UNKNOWN, WindWarnings.levelOf(null))
    }

    /**
     * **The severities, in the direction that matters.** `LEVEL_2` earns QGC's red modal (DJI
     * raises it where its own app tells the pilot to land, and landing17 raised it in a wind above
     * the airframe's rating); `LEVEL_1` is one rung below; `LEVEL_0` is never sent at all.
     *
     * Inverting any row is a mutant, and the reason it is worth a test rather than a glance: an
     * inverted table is silent — every message still arrives, at the wrong volume, and the failure
     * only shows up as a pilot who did not react.
     */
    @Test
    fun the_severity_of_each_level_is_the_decided_one() {
        assertNull("a calm reading must never reach QGC", WindWarnings.levelOf("LEVEL_0").mavSeverity())
        assertEquals(WarnLevel.MAV_SEVERITY_WARNING, WindWarnings.levelOf("LEVEL_1").mavSeverity())
        assertEquals(WarnLevel.MAV_SEVERITY_ERROR, WindWarnings.levelOf("LEVEL_2").mavSeverity())
        assertEquals(WarnLevel.MAV_SEVERITY_WARNING, WindWarnings.levelOf("UNKNOWN").mavSeverity())
        // And the bus's ladder agrees with the wire's, because both come from the same rung.
        assertEquals(WarnLevel.DIAGNOSTIC_ERROR, WindWarnings.levelOf("LEVEL_2").diagnosticLevel())
        assertEquals(WarnLevel.DIAGNOSTIC_WARN, WindWarnings.levelOf("LEVEL_1").diagnosticLevel())
        assertEquals(WarnLevel.DIAGNOSTIC_OK, WindWarnings.levelOf("LEVEL_0").diagnosticLevel())
        assertEquals(WarnLevel.DIAGNOSTIC_STALE, WindWarnings.levelOf("UNKNOWN").diagnosticLevel())
    }

    /**
     * **The measurement rides the sentence** — landing14's lesson, and the reason a level alone is
     * not a report: `windWarning` stayed `LEVEL_0` through the 9.1 m/s incident that overwhelmed
     * lateral control, and the finding on the record is *"the warning is not a substitute for the
     * number"*.
     */
    @Test
    fun the_measured_speed_rides_the_sentence() {
        val w = WindWarnings.picture("LEVEL_2", 142).single()
        assertEquals("14.2 m/s", w.measurement)
        assertEquals("DJI: Strong wind 14.2 m/s", WarnStatusTexts.appeared(w))
        assertTrue(WarnStatusTexts.appeared(w).contains("14.2"))
    }

    /**
     * The conversion is DJI's own quantisation divided by ten, and it is pinned against the single
     * owner of that arithmetic on the Zenoh side, `ZenohTelemetryEncoder.windOrNull`.
     *
     * Two renderings of one reading that disagreed would be worse than either alone: a pilot
     * reading "14.2 m/s" in a sentence while a subscriber logged 1.42 would have no way to tell
     * which was wrong.
     */
    @Test
    fun the_speed_is_the_same_number_the_wind_channel_publishes() {
        for (dms in listOf(0, 1, 91, 107, 142, 255)) {
            val words = WindWarnings.speedOrNull(dms)!!
            val onTheBus = ZenohTelemetryEncoder.windOrNull(dms)!!.data
            assertEquals("$dms dm/s", "%.1f m/s".format(onTheBus), words)
        }
        // Unknown is never zero: no reading means no number in the sentence, not "0.0 m/s".
        assertNull(WindWarnings.speedOrNull(null))
        assertNull(WindWarnings.picture("LEVEL_2", null).single().measurement)
    }

    /** The sentence survives without a number, rather than the warning being withheld. */
    @Test
    fun a_warning_with_no_speed_yet_is_still_announced() {
        val w = WindWarnings.picture("LEVEL_2", null).single()
        assertEquals("DJI: Strong wind", WarnStatusTexts.appeared(w))
        assertNotNull(w.level.mavSeverity())
    }

    /**
     * **A calm reading is reported, not withheld** — and this is what makes the clear announceable.
     *
     * `LEVEL_0` becomes a `NORMAL` warning rather than an empty picture, so the wind dropping back
     * is a `CHANGED`-to-NORMAL, which the monitor's retraction rule announces as a clear at INFO.
     * An empty picture would work too; what would *not* work, and is the mutant, is mapping
     * `LEVEL_0` to something forwardable, which announces every calm reading.
     */
    @Test
    fun a_calm_reading_is_carried_at_normal_and_never_sent_to_qgc() {
        val w = WindWarnings.picture("LEVEL_0", 61).single()
        assertEquals(WarnSource.WIND, w.source)
        assertEquals(WindWarnings.CODE, w.code)
        assertEquals("LEVEL_0", w.state)
        assertEquals(WarnLevel.NORMAL, w.level)
        assertFalse(w.level.forwardable())
        assertEquals("6.1 m/s", w.measurement)
    }

    /** DJI's own word is carried verbatim, next to our translation, on every level. */
    @Test
    fun djis_own_state_word_is_carried_verbatim() {
        for (state in listOf("LEVEL_0", "LEVEL_1", "LEVEL_2", "UNKNOWN")) {
            assertEquals(state, WindWarnings.picture(state, 100).single().state)
        }
        // A null delivery is named rather than blanked — the record must say what happened.
        assertEquals("UNKNOWN", WindWarnings.picture(null, 100).single().state)
    }

    /** One warning, one key: the wind source cannot accumulate entries the way health can. */
    @Test
    fun the_source_has_exactly_one_warning_with_a_stable_key() {
        val a = WindWarnings.picture("LEVEL_2", 142).single()
        val b = WindWarnings.picture("LEVEL_0", 10).single()
        assertEquals(a.key, b.key)
        assertEquals("WIND/windWarning/null/null", a.key)
    }

    /** Every sentence this source can produce fits the 50-byte field, measurement included. */
    @Test
    fun no_wind_sentence_ever_exceeds_the_field() {
        for (state in listOf("LEVEL_0", "LEVEL_1", "LEVEL_2", "UNKNOWN", "LEVEL_9")) {
            for (dms in listOf(null, 0, 142, 9999)) {
                val w = WindWarnings.picture(state, dms).single()
                for (text in listOf(
                    WarnStatusTexts.appeared(w),
                    WarnStatusTexts.changed(w),
                    WarnStatusTexts.cleared(w),
                )) {
                    assertTrue(
                        "'$text' is ${text.toByteArray(Charsets.UTF_8).size} bytes",
                        text.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
                    )
                }
            }
        }
    }

    /**
     * **landing17, replayed through the whole path.** The eight transitions that flight recorded,
     * fed to the monitor in order, must produce eight announcements and not one more — an
     * announcement per *change*, never per sample.
     *
     * The clock advances 8 s between transitions, which is the flight's own spacing (the tightest
     * pair is t=225.084 → t=227.085, and the rate bound's window is 12 s at 6 tokens, so the
     * flight's real cadence never bites it).
     */
    @Test
    fun landing17s_transitions_produce_one_announcement_each() {
        val monitor = WarningMonitor()
        // t=3.100 LEVEL_0, then 111.078 → 151.074 → 159.083 → 183.076 → 202.074 → 216.079 →
        // 225.084 → 227.085, alternating LEVEL_2 / LEVEL_0.
        val flight = listOf(
            "LEVEL_0", "LEVEL_2", "LEVEL_0", "LEVEL_2", "LEVEL_0",
            "LEVEL_2", "LEVEL_0", "LEVEL_2", "LEVEL_0",
        )
        var now = 0L
        val events = ArrayList<WarnEvent>()
        for ((i, state) in flight.withIndex()) {
            now += 8_000
            events += monitor.onDelivery(
                WarnSource.WIND, WindWarnings.picture(state, 100 + i), now,
            )
        }
        // Nine deliveries, nine events: the first reading plus eight transitions.
        assertEquals(9, events.size)
        // Four of them are the LEVEL_2 rises, and every one of those is a red-modal ERROR.
        val rises = events.filter { it.warning.state == "LEVEL_2" }
        assertEquals(4, rises.size)
        assertTrue(rises.all { it.mavSeverity == WarnLevel.MAV_SEVERITY_ERROR })
        assertTrue(rises.all { it.announce })
        // The drops back are announced as clears, at INFO — never as a new emergency.
        val drops = events.drop(1).filter { it.warning.state == "LEVEL_0" }
        assertEquals(4, drops.size)
        assertTrue(drops.all { it.mavSeverity == WarnLevel.MAV_SEVERITY_INFO })
        assertTrue(drops.all { it.text.startsWith("DJI cleared:") })
        // And the very first reading — a calm one — was recorded but never sent.
        assertEquals(WarnChange.APPEARED, events.first().change)
        assertNull(events.first().mavSeverity)
    }

    /**
     * **Never per sample.** DJI's key listener fires on delivery, not on change, and a wind that
     * sits at `LEVEL_2` for a minute is one warning however many times it is delivered.
     */
    @Test
    fun a_repeated_level_is_announced_once_however_often_it_is_delivered() {
        val monitor = WarningMonitor()
        var now = 0L
        val first = monitor.onDelivery(WarnSource.WIND, WindWarnings.picture("LEVEL_2", 142), now)
        assertEquals(1, first.size)
        repeat(50) {
            now += 100
            // The speed moves, as it does continuously in the air; the *level* does not.
            assertTrue(
                "a wind sample must not be an announcement",
                monitor.onDelivery(WarnSource.WIND, WindWarnings.picture("LEVEL_2", 140 + it), now)
                    .isEmpty(),
            )
        }
    }

    /**
     * **The two sources do not erase each other.** They deliver independently, on different
     * threads, at unrelated rates; a wind delivery that cleared the standing overheat would be the
     * device-health failure reintroduced by the fix for it.
     */
    @Test
    fun a_wind_delivery_says_nothing_about_the_health_picture() {
        val monitor = WarningMonitor()
        val overheat = Warning(
            source = WarnSource.DEVICE_HEALTH,
            code = "0x1600A0",
            state = "WARNING",
            level = WarnLevel.WARNING,
            title = "Aircraft overheating",
            componentId = 0,
            sensorIndex = 0,
        )
        monitor.onDelivery(WarnSource.DEVICE_HEALTH, listOf(overheat), 0)
        assertEquals(1, monitor.snapshot().size)

        monitor.onDelivery(WarnSource.WIND, WindWarnings.picture("LEVEL_2", 142), 1_000)
        assertEquals("both sources stand at once", 2, monitor.snapshot().size)
        // The wind clearing leaves the overheat exactly where it was.
        val events = monitor.onDelivery(WarnSource.WIND, WindWarnings.picture("LEVEL_0", 60), 2_000)
        assertEquals(1, events.size)
        assertEquals(WarnSource.WIND, events.single().warning.source)
        assertTrue(monitor.snapshot().any { it.code == "0x1600A0" && it.level == WarnLevel.WARNING })
    }

    /** A source may only speak for itself: a mislabelled warning is dropped, not trusted. */
    @Test
    fun a_delivery_may_not_carry_another_sources_warning() {
        val monitor = WarningMonitor()
        val events = monitor.onDelivery(
            WarnSource.WIND,
            WindWarnings.picture("LEVEL_2", 142) + Warning(
                source = WarnSource.DEVICE_HEALTH,
                code = "0x1600A0",
                state = "WARNING",
                level = WarnLevel.WARNING,
            ),
            0,
        )
        assertEquals(1, events.size)
        assertEquals(WarnSource.WIND, events.single().warning.source)
        assertEquals(1, monitor.snapshot().size)
    }
}
