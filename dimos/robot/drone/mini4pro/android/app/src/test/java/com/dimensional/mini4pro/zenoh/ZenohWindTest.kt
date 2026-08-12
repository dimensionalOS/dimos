package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The `wind` channel — **DJI's own wind-speed estimate as a bare `std_msgs.Float32`, m/s**,
 * held to the byte against DiMOS's own encoder and to the record's dm/s integers.
 *
 * The channel is landing14's lesson (Ivan: *"log this value in zenoh/mem2? just as Float?"*):
 * the control-authority incident there was 9.1 m/s of wind, on the record all along as
 * `windSpeedDmS = 91`, and a compass theory survived a day because nothing downstream could
 * see it. The byte vectors below are that flight's own ramp — 22 dm/s as the aircraft left
 * the ground effect, 91 dm/s at the incident — encoded by **DiMOS's generated Python
 * bindings** (`Float32.lcm_encode()`, literally what a consumer decodes), not by a second
 * copy of our own arithmetic.
 *
 * Two properties carry the channel's honesty and both are pinned here:
 *
 *  - **One conversion, one owner.** The record and the SDK speak integer dm/s (DJI's own
 *    quantisation); the wire speaks m/s. `ZenohTelemetryEncoder.windOrNull` divides by ten
 *    exactly once — in double, then narrows, the same IEEE rounding `tools/memexport`'s
 *    `Float32(dms / 10.0)` performs — and nothing else anywhere converts. A ×10 error is
 *    0.91 m/s printed where 9.1 m/s blew the aircraft sideways.
 *  - **Event-driven, never resampled.** `KeyWindSpeed` is change-driven: silence means
 *    UNCHANGED, not stale. Wind is therefore deliberately absent from `AircraftState` and
 *    from [ZenohEmission.emit]'s per-sample output, so no sampler can republish a held
 *    reading as a fresh measurement — asserted below, beside
 *    `ZenohTelemetryPumpTest`'s NOT_DECIDED_HERE pin.
 *
 * ## Mutation campaign, 2026-07-30
 *
 * Whole suite per mutant (2629 tests), fresh `test-results` each run, reverted after each.
 * **Counts are failing tests across the whole suite, measured.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | dm/s→m/s conversion broken: `windOrNull` drops the ÷10 (`dms.toFloat()`) | 3 |
 *  | conversion re-spelled: `windOrNull` divides in float (`dms / 10.0f`) | 0 — see survivors |
 *  | unknown becomes zero: `windReason` publishes null as `PUBLISHED`, encoder pads `0.0f` | 1 |
 *  | cadence-resampled: `emit()` gains a per-sample WIND emission (held value as fresh) | 3 |
 *  | wrong wire width: `Float32Codec` writes the value as a double | 3 |
 *  | fingerprint drift: `STD_MSGS_FLOAT32` reuses `STD_MSGS_STRING`'s | 3 |
 *  | adapter wiring: `Recorder`'s direction listener records `value()` (the wire int), not the enum name | 0 — see survivors |
 *
 * **Two-owners drift is guarded outside this suite, and was measured there the same day**:
 * mutating `tools/memexport`'s `Messages.wind` off the shared expression (`dms / 10.0 * 10.0`)
 * fails 2 planted byte pins in `memexport --selftest` AND aborts a real landing14 conversion —
 * `kotlin FAILED`, every wind message flagged `bytes differ at 8` by the kotlinframes
 * cross-check, which runs the SHIPPING Kotlin encoder over the same record. The JVM suite
 * cannot see Python; the tool's own harness is where that owner is held.
 *
 * **Survivors section.** Two zeros, both closed by argument rather than left open:
 *
 *  - The float-division probe is **equivalent, not a gap**: for every integer the SDK can
 *    deliver (|dm/s| ≪ 2²⁴), `(dms / 10.0).toFloat()` and `dms / 10.0f` produce identical
 *    bits — the double quotient rounds once to 53 bits, and both spellings then round the
 *    same real value to the same nearest float. The double-then-narrow spelling stays
 *    because it is the expression `tools/memexport` mirrors verbatim (a CPython float IS a
 *    double), and cross-language byte identity should rest on identical expressions, not on
 *    an equivalence argument someone must re-derive.
 *  - The direction-listener mutant lives in `Recorder.kt`, **the declared untestable Android
 *    adapter** (its own KDoc: the thin layer between the recorder and "the two things that
 *    cannot be unit-tested"), one line of wiring the JVM suite structurally cannot reach —
 *    the identical exposure every one of the ~30 `listenField` lines has, not a wind-specific
 *    gap. There is no project-side conversion to test: the recorded value is Java's own
 *    `Enum.toString` (the NAME — `javap` on the 5.18.0 jar confirms `WindDirection` is a real
 *    enum), and the call site says in a comment why `value()` is deliberately not read.
 */
class ZenohWindTest {

    private fun hex(b: ByteArray) = b.joinToString("") { "%02x".format(it) }

    // ── byte identity against DiMOS's own encoder, on landing14's own ramp ───

    /**
     * Four `windSpeedDmS` values of `datasets/landing14/20260729-161058.001.jsonl`, each
     * encoded by `dimos-lcm`'s generated Python (`Float32(dms / 10.0).lcm_encode()` —
     * literally what DiMOS calls) from the same conversion this encoder claims to
     * implement. Fingerprint, width and the ÷10 all at once; `0.1f` and `9.1f` are both
     * inexact in binary, so a conversion done in the wrong precision has bits to trip on.
     */
    @Test
    fun `real landing14 deliveries encode byte-identically to DiMOS's Python`() {
        assertEquals("0adc26b8f0546dd300000000", hex(ZenohEmission.wind(0).bytes!!))
        assertEquals("0adc26b8f0546dd33dcccccd", hex(ZenohEmission.wind(1).bytes!!))
        // t=15.4 s: 22 dm/s, the bottom of the ramp the verification block reproduces.
        assertEquals("0adc26b8f0546dd3400ccccd", hex(ZenohEmission.wind(22).bytes!!))
        // t=25.4 s: 91 dm/s — the 9.1 m/s that explained the "flyaway".
        assertEquals("0adc26b8f0546dd34111999a", hex(ZenohEmission.wind(91).bytes!!))
    }

    // ── the conversion, as values ─────────────────────────────────────────────

    /** dm/s ÷ 10 = m/s, exactly once: 91 is 9.1, never 91.0 (dropped ÷10) or 0.91 (twice). */
    @Test
    fun `the record's dm per second becomes metres per second`() {
        assertEquals(9.1f, ZenohTelemetryEncoder.windOrNull(91)!!.data, 0.0f)
        assertEquals(2.2f, ZenohTelemetryEncoder.windOrNull(22)!!.data, 0.0f)
        assertEquals(0.0f, ZenohTelemetryEncoder.windOrNull(0)!!.data, 0.0f)
        // The rated envelope's neighbourhood, for scale: DJI rates this airframe ~10.7 m/s.
        assertEquals(10.7f, ZenohTelemetryEncoder.windOrNull(107)!!.data, 0.0f)
    }

    /** Decoding the wire gives back the narrowed value — twelve bytes, one single. */
    @Test
    fun `the message is twelve bytes and decodes to the same single`() {
        val bytes = ZenohEmission.wind(91).bytes!!
        assertEquals(12, bytes.size)
        assertEquals(LcmFloat32(9.1f), Float32Codec.decode(bytes))
    }

    // ── the refusals, by name ────────────────────────────────────────────────

    /**
     * A value-less delivery — DJI withdrawing the reading, landing14's own last wind line —
     * is refused as [Withheld.WIND_MISSING]. Zero is a calm day and publishes; null must
     * never become one, because on the day this channel exists for, the difference between
     * "0.0 m/s" and "nobody knows" was the difference between two flyaway theories.
     */
    @Test
    fun `a missing reading is refused by name and zero publishes`() {
        val missing = ZenohEmission.wind(null)
        assertEquals(Withheld.WIND_MISSING, missing.reason)
        assertNull(missing.bytes)
        assertEquals(Withheld.PUBLISHED, ZenohEmission.wind(0).reason)
    }

    // ── the cadence contract: on delivery, never per sample ─────────────────

    /**
     * [ZenohEmission.emit] must never answer for `wind`: the key is change-driven and a
     * per-sample emission would republish a held reading as a fresh measurement — the
     * freshness trap this project has hit 7+ times, inverted. Structurally closed by wind
     * being in no `AircraftState` at all; this pins the boundary from the channel's side,
     * `ZenohTelemetryPumpTest` pins the full NOT_DECIDED_HERE list.
     */
    @Test
    fun `wind is not decided from a state sample`() {
        val decided = ZenohEmission
            .emit(com.dimensional.mini4pro.telemetry.AircraftState(), null, LcmTime.ZERO, encode = false)
            .map { it.channel }
        assertFalse(ZenohChannel.WIND in decided)
        assertTrue(ZenohChannel.WIND in ZenohEmission.NOT_DECIDED_HERE)
    }
}
