package com.dimensional.mini4pro.vision

import com.dimensional.mini4pro.record.Channel
import com.dimensional.mini4pro.record.JsonObject
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Tap
import com.dimensional.mini4pro.record.TagTap
import io.dronefleet.mavlink.MavlinkMessage
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Every sighting reaches the record, on the way past.**
 *
 * The gimbal aimed a real camera for weeks with not one reading in any flight record; that is this
 * project's standing proof that recording cannot be a thing you remember to call. The detector is at
 * *higher* risk of the same failure than the gimbal was, because its natural consumer — the
 * `vision_msgs.Detection3D` channel on the Zenoh bus — does not exist yet. A design that recorded
 * through that channel would have recorded nothing at all and nobody would have noticed until the
 * first landing.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | the record is written after the consumer, not before | 1 |
 * | a throwing tap propagates to the producer | 1 |
 * | `TagTap` stamps the entry with the record time, not the frame time | 1 |
 * | `TagTap` claims a measured bearing when there is no fix | 1 |
 * | `metric` omitted from the rendered line when false | 1 |
 * | a missing fix renders as zero rather than being absent | 1 |
 *
 * The third row measured **zero** on the first pass, and that is why `record/TagTap` exists: the
 * mapping was inline in `Recorder`, which needs Android and cannot be unit-tested, so the rule was
 * stated in a comment and enforced by nothing. Eighteen lines moved into a pure function turned it
 * into a test.
 *
 * And 2026-07-28, when the corners and the pose solve joined the line (suite 2 260 tests;
 * the gate arithmetic's rows are `TagPoseTest`'s table, the encoder's are
 * `ZenohDetectionTest`'s):
 *
 * | mutation | failures |
 * |---|---|
 * | corners dropped from the record line | 2 |
 * | the solve's quaternion dropped from the record line | 1 |
 * | an infinite `err2` silently dropped from the record | 1 |
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`
 * (the 2026-07-28 rows: the same protocol, session scratchpad `mutate_pose.py`). Nothing in the
 * second table survived and nothing was NO-RUN.
 *
 * And 2026-07-29, when the metric fix joined the line (same protocol, suite 2 488 tests;
 * the geometry's rows are `TagWorldTest`'s table, the intake's `CameraCalibrationTest`'s):
 *
 * | mutation | failures |
 * |---|---|
 * | the record line drops `fix_metric` / `range_m` | 1 |
 * | `TagTap` claims a metric fix when there is no fix | 1 |
 */
class RecordedTagSinkTest {

    private class RecordingTap : Tap {
        val seen = ArrayList<Triple<TagSighting.Sighting, TagFix?, Boolean>>()
        var throwOnTag = false
        override fun gcsOut(datagram: ByteArray) = throw AssertionError("not a GCS path")
        override fun gcsIn(message: MavlinkMessage<*>) = throw AssertionError("not a GCS path")
        override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean): Tap.Call =
            throw AssertionError("not an action path")

        override fun tagSeen(sighting: TagSighting.Sighting, fix: TagFix?, latched: Boolean) {
            seen.add(Triple(sighting, fix, latched))
            if (throwOnTag) throw IllegalStateException("the disk is on fire")
        }
    }

    private fun sighting(at: Long = 42) = TagSighting.Sighting(
        tagId = 3, x = 1.0, y = -2.0, z = 9.0, atNanos = at, pixelSize = 77.5,
        metric = false, hamming = 1, decisionMargin = 33.25,
        centreX = 1000.0, centreY = 400.0, imageWidth = 1920, imageHeight = 1080,
    )

    private fun fix() = TagFix(
        tagId = 3, northM = 4.25, eastM = -1.5, fromHeightM = 6.0, atNanos = 42, pixelSize = 77.5,
    )

    @Test
    fun everySightingReachesTheTap() {
        val tap = RecordingTap()
        val sink = RecordedTagSink(tap)
        sink(sighting(), fix(), false)
        sink(sighting(43), null, true)
        assertEquals(2, tap.seen.size)
        assertEquals(3, tap.seen[0].first.tagId)
        assertTrue("the latching edge must travel", tap.seen[1].third)
    }

    /**
     * **The record goes first.** If a consumer throws, the record still has the sighting; if the
     * process dies between the two, the record still has it. Same ordering `RecordedActionPort`
     * establishes for an ask, and for the same reason — the evidence must not be contingent on what
     * was done with it.
     */
    @Test
    fun theRecordIsWrittenBeforeTheConsumerRuns() {
        val order = ArrayList<String>()
        val ordering = object : Tap {
            override fun gcsOut(datagram: ByteArray) = throw AssertionError("not a GCS path")
            override fun gcsIn(message: MavlinkMessage<*>) = throw AssertionError("not a GCS path")
            override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean): Tap.Call =
                throw AssertionError("not an action path")

            override fun tagSeen(s: TagSighting.Sighting, f: TagFix?, l: Boolean) {
                order.add("tap")
            }
        }
        RecordedTagSink(ordering) { _, _, _ -> order.add("downstream") }(sighting(), fix(), false)
        assertEquals(listOf("tap", "downstream"), order)
    }

    /** A tap on fire must not reach the detector's worker thread. */
    @Test
    fun aThrowingTapIsContained() {
        val tap = RecordingTap().apply { throwOnTag = true }
        var downstreamRan = false
        val sink = RecordedTagSink(tap) { _, _, _ -> downstreamRan = true }
        sink(sighting(), fix(), false)
        assertTrue("the consumer must still run after a broken tap", downstreamRan)
    }

    /** With no downstream wired it is still a complete recorder — which is today's shape. */
    @Test
    fun itWorksWithNoDownstreamAtAll() {
        val tap = RecordingTap()
        RecordedTagSink(tap)(sighting(), null, false)
        assertEquals(1, tap.seen.size)
    }

    // ─────────────────────────────────────────────────── the rendered line

    private fun render(entry: LogEntry): String = JsonObject.render { entry.writeBody(it) }

    /**
     * **Through the production mapping**, not a restatement of it.
     *
     * A first version of this helper built the `LogEntry.Tag` itself, field by field, and the
     * mutation "stamp the entry with the record time instead of the frame time" therefore killed
     * **zero** tests — the test was asserting its own arithmetic. `record/TagTap` was extracted from
     * `Recorder.tagSeen` so that the mapping is a pure function a test can actually call.
     */
    private fun tagEntry(fix: TagFix?, latched: Boolean = false): LogEntry.Tag =
        TagTap.entry(sighting(), fix, latched)

    @Test
    fun theChannelNamesTheEntryTheTagLineActuallyProduces() {
        assertEquals(LogEntry.KIND_TAG, Channel.AIRCRAFT_TAG.entryKind)
        assertEquals("tag", tagEntry(fix()).kind)
        // Not urgent: it can arrive at 10 Hz, and an fsync per line on a 10 Hz stream is what
        // `LogEntry.Gimbal` declines for the same reason.
        assertFalse(tagEntry(fix()).urgent)
    }

    @Test
    fun aFixRendersItsPositionAndTheAssumptionsBehindIt() {
        val line = render(tagEntry(fix()))
        assertTrue(line, line.contains("\"id\":3"))
        assertTrue(line, line.contains("\"n\":4.25"))
        assertTrue(line, line.contains("\"e\":-1.5"))
        assertTrue(line, line.contains("\"from_h\":6"))
        assertTrue("the uncertainty must travel with the number", line.contains("\"bearing_assumed\":true"))
        assertTrue("metric is always written", line.contains("\"metric\":false"))
    }

    /**
     * **A fix that could not be made is absent, not zero.** The project's standing rule, and here it
     * decides whether an offline reader thinks the tag was at the takeoff point.
     */
    @Test
    fun aMissingFixIsAbsentFromTheLineRatherThanZero() {
        val line = render(tagEntry(null))
        assertFalse(line, line.contains("\"n\":"))
        assertFalse(line, line.contains("\"e\":"))
        assertFalse(line, line.contains("\"from_h\":"))
        // But the sighting itself is all there — the camera did see a tag.
        assertTrue(line, line.contains("\"id\":3"))
        assertTrue(line, line.contains("\"px\":77.5"))
    }

    @Test
    fun theEvidenceFieldsAreOnTheLine() {
        val line = render(tagEntry(fix()))
        // The per-detection half of the maxhamming question, and the detector's own confidence.
        assertTrue(line, line.contains("\"ham\":1"))
        assertTrue(line, line.contains("\"margin\":33.2") || line.contains("\"margin\":33.3"))
        // The frame geometry, so a bearing can be re-derived under a real calibration.
        assertTrue(line, line.contains("\"w\":1920"))
        assertTrue(line, line.contains("\"h\":1080"))
    }

    @Test
    fun theLatchingEdgeIsOnTheLineAndOnlyThere() {
        assertTrue(render(tagEntry(fix(), latched = true)).contains("\"latched\":true"))
        assertFalse(render(tagEntry(fix(), latched = false)).contains("latched"))
    }

    /**
     * **The frame's arrival time, not the write time.** A sighting reaching the record is already
     * 40–160 ms old; stamping it at the write would be invisible in the file — the numbers would
     * still look plausible, just uniformly late — and would destroy every latency measurement anyone
     * could make from it afterwards.
     */
    @Test
    fun theEntryIsStampedWithTheFramesArrivalTime() {
        assertEquals(42L, TagTap.entry(sighting(at = 42), fix(), false).monoNanos)
        assertEquals(999L, TagTap.entry(sighting(at = 999), null, true).monoNanos)
    }

    /** No fix means the bearing is *assumed*, which is the conservative reading of an absence. */
    @Test
    fun aLineWithNoFixStillSaysTheBearingIsAssumed() {
        assertTrue(TagTap.entry(sighting(), null, false).bearingAssumed)
    }

    /**
     * **Which pitch belief the fix rested on travels with it** — `pitch_reported`, the
     * 2026-07-28 reported-pitch fallback's provenance, written only when true (absent is the
     * pre-fallback truth: every older line was a commanded-pitch fix). A line with no fix makes
     * no pitch claim either way, so the flag stays off it.
     */
    @Test
    fun theReportedPitchProvenanceRidesTheLineOnlyWhenTrue() {
        val reported = fix().copy(pitchReported = true)
        assertTrue(render(tagEntry(reported)).contains("\"pitch_reported\":true"))
        assertFalse(render(tagEntry(fix())).contains("pitch_reported"))
        assertFalse(render(tagEntry(null)).contains("pitch_reported"))
        assertFalse("no fix, no pitch claim", TagTap.entry(sighting(), null, false).pitchReported)
        assertTrue(TagTap.entry(sighting(), reported, false).pitchReported)
    }

    /** Every field of the sighting reaches the line — a mapping is only useful if it is total. */
    @Test
    fun theMappingCarriesEverySightingField() {
        val e = TagTap.entry(sighting(), fix(), latched = true)
        assertEquals(3, e.tagId)
        assertEquals(1000.0, e.centreX, 1e-12)
        assertEquals(400.0, e.centreY, 1e-12)
        assertEquals(77.5, e.pixelSize, 1e-12)
        assertEquals(1920, e.width)
        assertEquals(1080, e.height)
        assertEquals(1, e.hamming)
        assertEquals(33.25, e.decisionMargin, 1e-12)
        assertEquals(1.0, e.x, 1e-12)
        assertEquals(-2.0, e.y, 1e-12)
        assertEquals(9.0, e.z, 1e-12)
        assertFalse(e.metric)
        assertEquals(4.25, e.northM!!, 1e-12)
        assertEquals(-1.5, e.eastM!!, 1e-12)
        assertEquals(6.0, e.fromHeightM!!, 1e-12)
        assertTrue(e.latched)
    }

    /**
     * **A metric fix says so on the line, and carries its solved range** — `fix_metric` and
     * `range_m`, the switch the next flight's record must show (metric below ~1.8 m where the
     * 60 px gate passes, bearing above). Deliberately a *separate key* from `metric`, which
     * describes the sighting's camera-frame pose and whose always-written false is a published
     * convention. Written only when present, like every additive field: a bearing fix and a
     * no-fix line carry neither, and an older reader sees a valid line either way.
     */
    @Test
    fun aMetricFixSaysSoOnTheLineAndCarriesItsRange() {
        val metric = fix().copy(metric = true, rangeM = 1.234)
        val line = render(tagEntry(metric))
        assertTrue(line, line.contains("\"fix_metric\":true"))
        assertTrue(line, line.contains("\"range_m\":1.234"))
        // The bearing fix is the pre-metric line, byte for byte.
        assertFalse(render(tagEntry(fix())).contains("fix_metric"))
        assertFalse(render(tagEntry(fix())).contains("range_m"))
        // No fix makes no metric claim and has no range — the bearingAssumed rule's twin.
        assertFalse(render(tagEntry(null)).contains("fix_metric"))
        assertFalse(render(tagEntry(null)).contains("range_m"))
        assertFalse("no fix, no metric claim", TagTap.entry(sighting(), null, false).fixMetric)
        assertNull(TagTap.entry(sighting(), null, false).rangeM)
        // And through the mapping, not a restatement of it.
        assertTrue(TagTap.entry(sighting(), metric, false).fixMetric)
        assertEquals(1.234, TagTap.entry(sighting(), metric, false).rangeM!!, 1e-12)
    }

    // ─────────────────────────── the corners and the solve on the line (2026-07-28)

    private fun corners() = TagCorners(
        x0 = 961.25, y0 = 438.5, x1 = 1039.125, y1 = 440.0,
        x2 = 1038.0, y2 = 517.75, x3 = 960.5, y3 = 516.25,
    )

    private fun solve(err2: Double = 6.5e-7) = TagPoseSolve(
        qx = 0.028, qy = -0.012, qz = 0.612, qw = 0.79,
        tx = 0.71, ty = -0.31, tz = 3.46,
        err1 = 1.5e-7, err2 = err2, tagSizeM = 0.075,
    )

    private fun richSighting() = sighting().copy(corners = corners(), solve = solve())

    /**
     * **The corners reach the line, at 2 dp, and the solve reaches it raw.** Raw is the
     * decision worth pinning: the record carries every solve UNGATED so the gates stay
     * measurable from flight data — the same rule that keeps `decision_margin` recorded and
     * unthresholded. Belief is applied at the encoder (`TagPose.trusted`), not at the recorder.
     */
    @Test
    fun theCornersAndTheSolveAreOnTheLine() {
        val line = render(TagTap.entry(richSighting(), fix(), false))
        assertTrue(line, line.contains("\"c0x\":961.25"))
        assertTrue(line, line.contains("\"c1x\":1039.13"))   // 2 dp, rounded
        assertTrue(line, line.contains("\"c3y\":516.25"))
        assertTrue(line, line.contains("\"qz\":0.612"))
        assertTrue(line, line.contains("\"qw\":0.79"))
        assertTrue(line, line.contains("\"tx\":0.71"))
        assertTrue(line, line.contains("\"tz\":3.46"))
        // 12 dp on the errors: their RATIO is a published gate, and 1.5e-7 at the record's
        // ordinary 3 dp would be zero — the one quantity the line exists to preserve, gone.
        assertTrue(line, line.contains("\"e1\":0.00000015"))
        assertTrue(line, line.contains("\"e2\":0.00000065"))
        assertTrue(line, line.contains("\"tag_m\":0.075"))
    }

    /**
     * **A sighting without them writes a line without them** — the legacy line, byte-compatible
     * with every reader that predates the fields, and the exact line an old record contains.
     * Absent is "not measured", never zero.
     */
    @Test
    fun aLegacySightingWritesTheLegacyLine() {
        val line = render(tagEntry(fix()))
        for (key in listOf(
            "c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y",
            "qx", "qy", "qz", "qw", "tx", "ty", "tz", "e1", "e2", "tag_m",
        )) {
            assertFalse("\"$key\" must be absent from a legacy line: $line", line.contains("\"$key\":"))
        }
    }

    /**
     * **`err2 = +Infinity` survives the record as the format's own `"Infinity"`.** It is the
     * unambiguous case — no second PnP minimum — on 59 % of measured solves, and a writer that
     * dropped or coerced it would turn the best solves into partial ones on replay.
     */
    @Test
    fun anInfiniteErr2IsWrittenAsTheFormatsInfinityString() {
        val line = render(TagTap.entry(
            sighting().copy(solve = solve(err2 = Double.POSITIVE_INFINITY)), null, false,
        ))
        assertTrue(line, line.contains("\"e2\":\"Infinity\""))
        assertTrue(line, line.contains("\"e1\":0.00000015"))
    }

    /** The new fields ride the same mapping — total, like everything before them. */
    @Test
    fun theMappingCarriesTheCornersAndTheSolve() {
        val e = TagTap.entry(richSighting(), fix(), false)
        assertEquals(961.25, e.c0x!!, 1e-12)
        assertEquals(438.5, e.c0y!!, 1e-12)
        assertEquals(1039.125, e.c1x!!, 1e-12)
        assertEquals(516.25, e.c3y!!, 1e-12)
        assertEquals(0.028, e.qx!!, 1e-12)
        assertEquals(0.79, e.qw!!, 1e-12)
        assertEquals(0.71, e.tx!!, 1e-12)
        assertEquals(3.46, e.tz!!, 1e-12)
        assertEquals(1.5e-7, e.e1!!, 1e-20)
        assertEquals(6.5e-7, e.e2!!, 1e-20)
        assertEquals(0.075, e.tagM!!, 1e-12)
        // And a solve-less, corner-less sighting maps to nulls, not zeros.
        val legacy = TagTap.entry(sighting(), fix(), false)
        assertNull(legacy.c0x)
        assertNull(legacy.qx)
        assertNull(legacy.e2)
        assertNull(legacy.tagM)
    }
}
