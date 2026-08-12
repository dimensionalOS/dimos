package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The `tag_fix` channel — **the tag's believed place in `drone/world`, published from the
 * record's own fix values and from nothing else.**
 *
 * The channel exists to kill a composition artifact Ivan watched in mem2 replays
 * (2026-07-29): a consumer joining camera-optical `detections` against `tf` edges of a
 * different instant makes the tag ride the camera, and a descending body drags the last
 * sparse detection into the floor. `TagWorld.fix` already composes each sighting with the
 * pose at the sighting's own instant; this channel publishes that composition. The encoder
 * therefore takes **only** the `tag` line's fix half — it has no camera-frame inputs at all,
 * which is the structural half of the never-re-derive rule; the byte pins below are the
 * behavioural half.
 *
 * ## Mutation campaign, 2026-07-29
 *
 * Whole suite per mutant (2545 tests), fresh `test-results` each run, reverted after each,
 * driven by `tmp/mutants.py`. **Counts are failing tests across the whole suite, measured.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the artifact reintroduced: fix axes swapped at the encoder (`x = north`, `y = east` — the consumer-style error) | 2 |
 *  | z fabricated: [ZenohTelemetryEncoder.TAG_ON_DATUM_PLANE_Z] becomes a "measured-looking" height (1.0) | 2 |
 *  | provenance stripped: [ZenohTelemetryEncoder.tagFixId] returns the bare label always | 2 |
 *  | detections' solved fact made uncarryable: `detectionOrNull` writes [ZenohTelemetryEncoder.UNSOLVED_BOX] on solved messages too (id already bare) | 2 |
 *  | the fixless gate dropped: `tagFixReason` returns `PUBLISHED` unconditionally | 1 |
 *
 * No survivors. The last row's 1 is the [ZenohEmission] drift check doing its job: the gates
 * saying PUBLISHED while the encoder refuses is itself a throw, and the refusal test is the
 * one place a fixless sighting reaches the pair. The solved-fact mutant is `detections`'
 * channel, run from this campaign because the bare-id change made the NaN convention that
 * fact's *only* carrier — `theSolvedFactTravelsStructurallyAndTheIdStaysBare` and the box
 * test both die, which is the "must never become uncarryable" requirement holding.
 */
class ZenohTagFixTest {

    private fun stamp(unixMs: Long) = LcmTime.ofEpochSeconds(unixMs / 1000.0)

    private fun emit(
        north: Double?,
        east: Double?,
        tagId: Int = 0,
        margin: Double = 110.1,
        fixMetric: Boolean = false,
        rangeSource: String? = null,
        pitchReported: Boolean = false,
        seq: Int = 1,
        unixMs: Long = 1_785_315_837_716,
    ) = ZenohEmission.tagFix(
        tagId, margin, north, east, fixMetric, rangeSource, pitchReported,
        stamp(unixMs), seq,
    )

    private fun message(
        north: Double,
        east: Double,
        fixMetric: Boolean = false,
        rangeSource: String? = null,
        pitchReported: Boolean = false,
    ): LcmDetection3DArray =
        Detection3DArrayCodec.decode(
            emit(north, east, fixMetric = fixMetric, rangeSource = rangeSource,
                pitchReported = pitchReported).bytes!!,
        )

    // ── byte identity against DiMOS's own encoder, on real landing10 lines ───

    /**
     * Two `tag` lines of `datasets/landing10/20260729-120342.001.jsonl`
     * (`started_unix_ms = 1785315822708`), encoded by `dimos-lcm`'s generated Python from
     * the same construction this encoder claims. The first is a **metric** fix
     * (`fix_metric`, no `range_src` — the line predates the field, so the id's rung is the
     * `solve` fallback); the second a **bearing** fix, whose id is the bare label because
     * an absent rung means *unrecorded*, never a guess.
     */
    @Test
    fun `real landing10 fixes encode byte-identically to DiMOS's Python`() {
        // t=15.007509: n=-0.092, e=0.108, margin=110.1, fix_metric=true.
        val metric = emit(
            north = -0.092, east = 0.108, margin = 110.1,
            fixMetric = true, seq = 1, unixMs = 1_785_315_837_716,
        ).bytes!!
        assertEquals(546, metric.size)
        val metricHex = metric.toHex()
        assertTrue(metricHex.startsWith(
            "ea027f6760f2ebd200000001000000016a69c1fd2aad4b500000000c64726f6e652f776f726c64" +
                "0000000001000000016a69c1fd2aad4b500000000c64726f6e652f776f726c6400" +
                "0000000b74616733366831313a3000405b866666666666" +
                "3fbba5e353f7ced9bfb78d4fdf3b645a0000000000000000",
        ))
        assertTrue(metricHex.endsWith("0000001774616733366831313a303b72616e67653d736f6c766500"))
        // t=16.50649: n=0.228, e=0.256, margin=137.6, bearing fix — bare id.
        val bearing = emit(
            north = 0.228, east = 0.256, margin = 137.6,
            fixMetric = false, seq = 2, unixMs = 1_785_315_839_214,
        ).bytes!!
        assertEquals(534, bearing.size)
        val bearingHex = bearing.toHex()
        assertTrue(bearingHex.startsWith(
            "ea027f6760f2ebd200000001000000026a69c1ff0cc161720000000c64726f6e652f776f726c64" +
                "0000000001000000026a69c1ff0cc161720000000c64726f6e652f776f726c6400" +
                "0000000b74616733366831313a30004061333333333333" +
                "3fd0624dd2f1a9fc3fcd2f1a9fbe76c90000000000000000",
        ))
        assertTrue(bearingHex.endsWith("0000000b74616733366831313a3000"))
    }

    // ── the record's own values, in the world frame ──────────────────────────

    /**
     * ENU off the line's NED names: **`x = eastM`, `y = northM`** — the consumer-style swap
     * is the single most plausible re-derivation error and it is byte-visible here.
     */
    @Test
    fun `east lands on x and north on y`() {
        val p = message(12.345, -4.567).detections[0].results[0].pose.pose.position
        assertEquals(-4.567, p.x, 0.0)
        assertEquals(12.345, p.y, 0.0)
    }

    /**
     * **`z` is the datum-plane assumption, `0.0`, never a height that looks measured.** The
     * encoder has no baro and no range input to fabricate one from — `TagFix.fromHeightM` is
     * the *aircraft's* height and the tag might be standing on a box nobody measured
     * (`TagWorld.fix`'s own story; the 25 Hz flight's board was +0.33 m, found by accident).
     */
    @Test
    fun `z is the datum plane assumption`() {
        assertEquals(0.0, ZenohTelemetryEncoder.TAG_ON_DATUM_PLANE_Z, 0.0)
        assertEquals(0.0, message(1.0, 2.0).detections[0].results[0].pose.pose.position.z, 0.0)
    }

    /** The frame is `drone/world` on both headers, and they restate each other. */
    @Test
    fun `the frame is the world frame`() {
        val m = message(1.0, 2.0)
        assertEquals(ZenohTelemetryEncoder.FRAME_WORLD, m.header.frameId)
        assertEquals(m.header, m.detections[0].header)
    }

    /**
     * No world-frame tag orientation has ever been derived, so the orientation and the box
     * are NaN entire — the solved orientation is optical-frame and lives on `detections`.
     */
    @Test
    fun `orientation and box are refused entire`() {
        val d = message(1.0, 2.0).detections[0]
        assertTrue(d.results[0].pose.pose.orientation.w.isNaN())
        assertTrue(d.bbox.size.x.isNaN())
        assertTrue(d.bbox.center.position.x.isNaN())
    }

    /**
     * The wire carries the record's precision: `n`/`e` at 3 dp, `margin` at 1 dp — the same
     * `Json.roundTo` owner as `setpoint`, so a full-precision live `TagFix` and its record
     * rendering produce identical bytes.
     */
    @Test
    fun `full-precision input encodes identically to its record rendering`() {
        assertEquals(
            emit(-0.092, 0.108, margin = 110.1).bytes!!.toHex(),
            emit(-0.0920021, 0.1080499, margin = 110.09).bytes!!.toHex(),
        )
    }

    // ── the provenance, minimal ──────────────────────────────────────────────

    /**
     * The id grammar, [ZenohTelemetryEncoder.tagFixId]: bare label plus only what varies per
     * message and has no typed vehicle — the range-ladder rung (landing07-B measured a
     * baro-scaled and a solve-scaled fix disagreeing by ~1.2 m) and the pitch belief.
     * Ivan's bare-label rule (2026-07-29) is why there is nothing else.
     */
    @Test
    fun `the id carries the rung and the pitch belief, minimally`() {
        assertEquals("tag36h11:0", message(1.0, 2.0).detections[0].id)
        assertEquals("tag36h11:0;range=solve",
            message(1.0, 2.0, fixMetric = true).detections[0].id)
        assertEquals("tag36h11:0;range=size",
            message(1.0, 2.0, rangeSource = "size").detections[0].id)
        assertEquals("tag36h11:0;range=baro",
            message(1.0, 2.0, rangeSource = "baro").detections[0].id)
        assertEquals("tag36h11:0;range=solve;pitch=reported",
            message(1.0, 2.0, fixMetric = true, pitchReported = true).detections[0].id)
        assertEquals("tag36h11:0;pitch=reported",
            message(1.0, 2.0, pitchReported = true).detections[0].id)
    }

    // ── the refusal ──────────────────────────────────────────────────────────

    /** No fix on the line means nothing to say — the sighting still travels on `detections`. */
    @Test
    fun `a sighting without a fix is refused by name`() {
        for (e in listOf(
            emit(null, 2.0),
            emit(1.0, null),
            emit(Double.NaN, 2.0),
            emit(1.0, Double.POSITIVE_INFINITY),
        )) {
            assertEquals(Withheld.FIX_UNAVAILABLE, e.reason)
            assertNull(e.bytes)
        }
        assertEquals(Withheld.PUBLISHED, emit(0.0, 0.0).reason)
    }

    private fun ByteArray.toHex() = joinToString("") { "%02x".format(it) }
}
