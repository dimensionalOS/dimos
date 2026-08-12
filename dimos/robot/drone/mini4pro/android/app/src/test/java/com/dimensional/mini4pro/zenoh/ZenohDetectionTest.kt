package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.vision.TagPoseSolve
import com.dimensional.mini4pro.vision.TagSighting
import com.dimensional.mini4pro.vision.TagWorld
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **A sighting as a `vision_msgs.Detection3DArray`, and everything it refuses to claim.**
 *
 * The channel carried a bare `Detection3D` for one day and carries the array since 2026-07-28
 * (Ivan): ROS's own convention for a detection topic, and the type DiMOS consumes natively. The
 * wrapper is four lines and every claim a detection makes is still the **element's**, which is why
 * almost every test below still reads `detectionOrNull` — see the last section for the four that
 * are about the array itself.
 *
 * The `detections` channel is the only one on this bus whose payload is a number somebody might
 * fly on, and it is published with `metric = false` — a *fitted* focal length, an *assumed*
 * principal point and no distortion model. `docs/tag-detector.md` §7 names the hazard exactly:
 * *"publishing a coarse pose onto a bus where a consumer cannot see the flag is the one way this
 * could mislead somebody who never read this document."* So half of this file asserts that the
 * qualification is on the wire and that the unsolved fields stay unsolved.
 *
 * The **bytes** are pinned elsewhere and by an encoder we did not write: `LcmFixtureTest` checks
 * all five vision types — the array, the detection, and the three it nests — against fixtures
 * `tools/lcmfixtures/generate.py` produced from DiMOS's own Python bindings, including the exact
 * NaN bit pattern. This file is about which values go in.
 *
 * ## The substitution this file exists to catch
 *
 * `TagWorld` has two reference points and they are 2.99° apart. `cameraFrame` measures from the
 * **image centre**, standing in for the principal point, and returns a direction in the *optical*
 * frame; `nadirFrame` measures from the **measured nadir pixel** and is what `TagWorld.fix` needs,
 * because `fix` scales a pixel offset by altitude and that is only true of a ray pointing straight
 * down. A `Detection3D` is stamped `drone/camera_optical`, so its pose is referred to the
 * principal point and must come from `cameraFrame`.
 *
 * Nothing downstream could detect the wrong one: it is the right magnitude, in the right units,
 * 2.99° in the wrong direction — 42 cm at 8 m. [thePositionIsTheOpticalFrameOneAndNotTheNadirOne]
 * is the assertion, and it compares against both triples rather than against a literal, so it
 * fails if the encoder ever starts correcting.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | the encoder corrects the position to the nadir reference point | 1 |
 * | `score` carries `hamming` rather than `decisionMargin` | 1 |
 * | the orientation is the identity quaternion rather than NaN | 2 |
 * | the bounding box's `size` is `(0,0,0)` rather than NaN | 1 |
 * | the bounding box's centre is the detected position rather than NaN | 1 |
 * | `frame_id` is `drone/camera` rather than `drone/camera_optical` | 1 |
 * | `header.stamp` is ignored and left at zero | 1 |
 * | `class_id` drops the tag family | 1 |
 * | `id` drops the `metric` qualification | 1 |
 * | `id` drops the range provenance | 1 |
 * | a non-positive range is published rather than refused | 1 |
 * | a non-finite position is published rather than refused | 1 |
 * | a zero frame width is published rather than refused | 1 |
 * | the covariance is NaN rather than ROS's all-zero "unknown" | 1 |
 * | `metric = false` withholds the message | 3 |
 * | `hamming > 0` withholds the message | 1 |
 * | `detectionReason` returns `PUBLISHED` unconditionally | 4 |
 *
 * And 2026-07-28, on the wrapping, when the wire type became `Detection3DArray`. Suite 2229 tests;
 * again nothing survived and nothing was NO-RUN:
 *
 * | mutation | failures |
 * |---|---|
 * | the channel publishes the bare element rather than the array | 1 |
 * | the array's header is a fresh empty one rather than the element's | 1 |
 * | the array's header keeps the seq and frame but drops the stamp | 1 |
 * | a refused sighting becomes an empty array rather than nothing | 1 |
 * | the `detections` key keeps the bare `Detection3D` type | 2 |
 *
 * Every row is 1 or 2 and that is the honest shape of a four-line wrapper: there is very little of
 * it to get wrong, and each way of getting it wrong is pinned by exactly the test written for it.
 * The **codec's** side of the change — the hoisted length, the fingerprint — is `LcmFixtureTest`'s
 * table, where the counts are higher because the fixtures catch it three ways.
 *
 * And 2026-07-28 again, when the pose solve arrived — the encoder half of the gate work, run
 * against the 2 260-test suite. The gate arithmetic's own rows are `TagPoseTest`'s table and the
 * record's are `RecordedTagSinkTest`'s; these are the ways the *encoder* could lie about a solve:
 *
 * | mutation | failures |
 * |---|---|
 * | the encoder bypasses the gates entirely (uses the raw solve) | 3 |
 * | the orientation alone bypasses the gates (partial bypass) | 3 |
 * | an absent solve publishes the identity quaternion | 2 |
 * | a trusted solve still publishes the refused box | 1 |
 * | the bbox alone bypasses the gates (partial bypass) | 3 |
 * | the solved box's size becomes the confident zero `(0,0,0)` | 1 |
 * | the solved box claims a cube (`z = tagSize`) | 1 |
 * | the solved position replaces the apparent-size ray | 1 |
 * | the id marker is always present | 2 |
 * | the id marker is never present | 1 |
 * | the id marker is computed from the raw solve, not the gate | 4 |
 *
 * Every row of these tables is one mutation applied alone to `src/main`, with the **whole suite**
 * run against it and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that
 * fails to compile otherwise leaves the previous run's XML and reports a confident zero. Harness:
 * `tmp/mutate.py` (the solve rows: the same protocol, session scratchpad `mutate_pose.py`).
 * Nothing survived any of these runs and nothing was NO-RUN, all on the first pass, so unlike
 * `TagWorldTest` — and unlike `TagPoseTest`'s sign-normalisation row — none of these tables
 * records a hole that had to be closed.
 *
 * Most rows are 1 because each property is pinned by one test that asserts it and then asserts
 * several neighbouring things, and the first failing assertion ends that test. That is the honest
 * count and not a weaker one: what matters is that no mutation survived.
 *
 * The **4** on the last row is worth reading. `detectionReason` returning `PUBLISHED`
 * unconditionally does not merely mis-tally — it makes `ZenohEmission.emission` throw, because the
 * gate says published and the encoder returns nothing, and that is the drift check firing exactly
 * as designed. [theGateAndTheEncoderCannotDisagree] drives every refusal through
 * `ZenohEmission.detection` and not only through `detectionOrNull` for that reason: the encoder
 * refuses on its own, so a test that only called the encoder could not see a broken gate at all.
 *
 * ## What is not tested here, and is therefore untested
 *
 * **`ZenohBus.publishDetection`'s stamp.** It computes `header.stamp` as *now minus the sighting's
 * own age* so the message carries the frame's arrival rather than the send time (D-5), and
 * `ZenohBus` imports `android.os.SystemClock` and `android.util.Log`, so no JVM test reaches it.
 * The subtraction is three lines and it is asserted by nothing. What *is* asserted here is that
 * the encoder uses the stamp it is handed and invents none of its own.
 */
class ZenohDetectionTest {

    /** 2026-07-28T09:00:00Z, and a nine-digit nanosecond so a truncation would show. */
    private val stamp = LcmTime(1785574800, 123456789)

    /**
     * A sighting built the way `TagRecogniser.publish` builds one: the camera-frame triple from
     * [TagWorld.cameraFrame], scaled by the range the tag's apparent size implies.
     *
     * Deliberately **not** hand-written coordinates. The point of the position assertions is that
     * the encoder forwards what the geometry produced, and a literal would agree with a broken
     * encoder that happened to be broken the same way twice.
     */
    private fun sighting(
        centreX: Double = 700.0,
        centreY: Double = 300.0,
        range: Double = 3.5,
        tagId: Int = 7,
        hamming: Int = 0,
        decisionMargin: Double = 41.75,
        width: Int = 1920,
        height: Int = 1080,
        metric: Boolean = false,
    ): TagSighting.Sighting {
        val (x, y, z) = TagWorld.cameraFrame(centreX, centreY, width, height, range)
        return TagSighting.Sighting(
            tagId = tagId,
            x = x, y = y, z = z,
            atNanos = 1_234_000_000L,
            pixelSize = 112.5,
            metric = metric,
            hamming = hamming,
            decisionMargin = decisionMargin,
            centreX = centreX,
            centreY = centreY,
            imageWidth = width,
            imageHeight = height,
        )
    }

    private fun encode(s: TagSighting.Sighting = sighting(), seq: Int = 3) =
        ZenohTelemetryEncoder.detectionOrNull(s, stamp, seq)

    private fun result(s: TagSighting.Sighting = sighting()) = encode(s)!!.results.single()

    /** The message that actually goes on the wire: the element, wrapped. */
    private fun array(s: TagSighting.Sighting = sighting(), seq: Int = 3) =
        ZenohTelemetryEncoder.detectionsOrNull(s, stamp, seq)

    // ───────────────────────────────────────────────────────────── the frame

    /**
     * `drone/camera_optical`, and it is load-bearing rather than decorative: it is what tells a
     * consumer that x is right, y is **down** and z is along the optical axis, and it is the leaf
     * the `tf` tree already publishes a fixed edge to.
     */
    @Test
    fun theHeaderNamesTheOpticalFrameAndTheStampTheCallerGaveIt() {
        val d = encode()!!
        assertEquals("drone/camera_optical", d.header.frameId)
        assertEquals(ZenohTelemetryEncoder.FRAME_CAMERA_OPTICAL, d.header.frameId)
        // The stamp is the caller's, untouched. `ZenohBus` derives it from the sighting's own age
        // so it is the frame's arrival and not the send time; nothing here invents a clock.
        assertEquals(stamp, d.header.stamp)
        assertEquals(3, d.header.seq)
    }

    /**
     * **The one substitution nothing downstream could see.** See the class doc.
     *
     * `cameraFrame` and `nadirFrame` differ by 2.99° — 10.7 px in x and 75.2 px in y at 1920 wide
     * — so at 3.5 m along the axis they disagree by centimetres, in the right units, in a
     * plausible direction. The message says `camera_optical`, so it must be the first.
     */
    @Test
    fun thePositionIsTheOpticalFrameOneAndNotTheNadirOne() {
        val s = sighting()
        val p = result(s).pose.pose.position
        val optical = TagWorld.cameraFrame(700.0, 300.0, 1920, 1080, 3.5)
        val nadir = TagWorld.nadirFrame(700.0, 300.0, 1920, 1080, 3.5)

        assertEquals(optical.first, p.x, 0.0)
        assertEquals(optical.second, p.y, 0.0)
        assertEquals(optical.third, p.z, 0.0)

        // And they really are different numbers, so the assertion above is not vacuous. If this
        // ever fails, the nadir pixel has been set back to the image centre in `TagWorld`.
        assertNotEquals(nadir.first, p.x, 1e-9)
        assertNotEquals(nadir.second, p.y, 1e-9)
        assertTrue(
            "the two reference points must disagree by centimetres at this range",
            Math.hypot(nadir.first - p.x, nadir.second - p.y) > 0.05,
        )
    }

    /** `z` is the range along the optical axis, and it is the range the sighting carries. */
    @Test
    fun theRangeIsTheSightingsOwnAndIsNotRecomputed() {
        assertEquals(3.5, result(sighting(range = 3.5)).pose.pose.position.z, 1e-12)
        assertEquals(8.25, result(sighting(range = 8.25)).pose.pose.position.z, 1e-12)
    }

    // ────────────────────────────────────────────────────── the hypothesis

    /**
     * `class_id` is the family **and** the code. Tag 7 of `tag36h11` is not tag 7 of `tag25h9`,
     * and a consumer that landed on the wrong family's marker would have no way to tell.
     */
    @Test
    fun theClassIdCarriesTheFamilyAndTheTagId() {
        assertEquals("tag36h11:7", result(sighting(tagId = 7)).hypothesis.classId)
        assertEquals("tag36h11:0", result(sighting(tagId = 0)).hypothesis.classId)
        // The family is the one `android/app/src/apriltag/tag_jni.c:68` creates, and nothing else
        // is built into the APK.
        assertEquals("tag36h11", ZenohTelemetryEncoder.TAG_FAMILY)
    }

    /**
     * **`score` is the decision margin, not the hamming distance**, and the two were the only
     * candidates (`docs/tag-detector.md` §7).
     *
     * `hamming` is a count of corrected bit errors where **lower is better**. Writing it into a
     * field ROS documents as a confidence inverts the sense, so a consumer thresholding
     * `score > x` would keep the worse decodes and drop the clean ones. The test drives a sighting
     * where the two numbers are both present and unequal, so a swap cannot pass.
     */
    @Test
    fun theScoreIsTheDecisionMarginAndNeverTheHammingDistance() {
        val s = sighting(hamming = 1, decisionMargin = 41.75)
        assertEquals(41.75, result(s).hypothesis.score, 0.0)
        assertNotEquals(1.0, result(s).hypothesis.score, 1e-9)
        // A clean decode with a *low* margin must score low, which is the ordering a swap breaks.
        val weak = sighting(hamming = 0, decisionMargin = 4.5)
        assertTrue(result(weak).hypothesis.score < result(s).hypothesis.score)
    }

    /**
     * `hamming` and `pixelSize` are **refused**, not folded in. `Detection3D` has no column for
     * either; both are on every `LogEntry.Tag` line, which is where per-detection evidence lives.
     *
     * Asserted by their absence from the whole message rather than by a comment, so a future edit
     * that smuggles one into the `id` or the `class_id` has to change this test and say why.
     */
    @Test
    fun theHammingDistanceAndThePixelSizeAreNotSmuggledIntoAnyString() {
        val d = encode(sighting(hamming = 1))!!
        assertTrue("hamming must not appear in the id", !d.id.contains("hamming"))
        assertTrue("pixel size must not appear in the id", !d.id.contains("112"))
        assertTrue(!d.results.single().hypothesis.classId.contains("hamming"))
    }

    // ──────────────────────────────────────────── what is deliberately unsolved

    /**
     * **The orientation is NaN on all four components when nothing solved one** — a solve-less
     * sighting, which is every legacy record and every frame the solve is off for.
     *
     * The identity quaternion would say the marker is square-on to the image and unrotated in
     * it — wrong on essentially every frame, and wrong *invisibly*, because it is exactly what
     * a consumer expects to see. Since 2026-07-28 a *gated* solve fills this field instead —
     * the solved-pose section at the bottom of this file — and this test is now the shape every
     * gate failure must degrade to.
     */
    @Test
    fun theOrientationIsNaNBecauseNothingSolvesIt() {
        val q = result().pose.pose.orientation
        for (v in listOf(q.x, q.y, q.z, q.w)) assertTrue("orientation must be NaN", v.isNaN())
        assertNotEquals(LcmQuaternion.IDENTITY, q)
    }

    /**
     * **The bounding box is refused entire** — centre, orientation and size, ten NaN.
     *
     * A `size` of `(0,0,0)` claims the object is a point, which is the confident zero this project
     * refuses; a box also needs an orientation, which we do not have. The **centre** is NaN too
     * and that is the part worth pinning: the tag's position is known and it would have been easy
     * to put it here, but a centre beside a NaN size invites a reader to treat the box as the
     * detection. The pose lives in `results[0]`, where the covariance and the score that qualify
     * it also live.
     */
    @Test
    fun theBoundingBoxIsRefusedEntireAndNotZeroFilled() {
        val b = encode()!!.bbox
        val all = listOf(
            b.center.position.x, b.center.position.y, b.center.position.z,
            b.center.orientation.x, b.center.orientation.y,
            b.center.orientation.z, b.center.orientation.w,
            b.size.x, b.size.y, b.size.z,
        )
        for (v in all) assertTrue("every bbox field must be NaN, got $v", v.isNaN())
        // Not the detected position dressed up as a box, and not a zero size.
        assertNotEquals(result().pose.pose.position.z, b.center.position.z, 1e-9)
        assertEquals(ZenohTelemetryEncoder.UNSOLVED_BOX, b)
    }

    /**
     * The covariance is **zeros, not NaN**, and the difference is a rule rather than a taste.
     *
     * NaN is this project's word for a quantity with no feed at all (`ZenohTelemetryEncoder` §4);
     * ROS has its own documented word for an unknown covariance and it is an all-zero matrix. §4
     * says to use the documented convention where one exists, and `odom` and `pose` already do.
     * Nothing here has measured the pose's error: the focal length reproducing to 1.2 % across two
     * flights is a fit's spread, not a covariance.
     */
    @Test
    fun theCovarianceIsRosSAllZeroUnknownAndNotNaN() {
        val cov = result().pose.covariance
        assertEquals(36, cov.size)
        for (v in cov) assertEquals(0.0, v, 0.0)
    }

    // ─────────────────────────────────────── the label, bare

    /**
     * **The `id` is the bare label** — `tag36h11:<id>` and nothing else, on every message.
     *
     * Ivan, 2026-07-29: *"we shouldn't add all this stuff into the label… no need for
     * range=…, solved=…"* — reversing the 2026-07-28 suffix convention. The `metric` caveat
     * now lives in the contract row (`docs/zenoh-topics.md`), not per message, so the id is
     * identical whatever the sighting's metric flag says; per-message belief provenance is
     * the world-frame `tag_fix` channel's job. `id` is finally exactly what vision_msgs
     * intends: a stable tracking handle.
     */
    @Test
    fun theIdIsTheBareLabelWhateverTheSightingClaims() {
        assertEquals("tag36h11:7", encode()!!.id)
        assertEquals("tag36h11:7", encode(sighting(metric = true))!!.id)
        assertEquals("tag36h11:0", encode(sighting(tagId = 0))!!.id)
    }

    /** No number is formatted into the id: `String.format` is locale-sensitive and this phone is in Greece. */
    @Test
    fun theIdContainsNoFormattedDecimal() {
        assertTrue("a decimal separator in the id would differ by locale", !encode()!!.id.contains('.'))
        assertTrue(!encode()!!.id.contains(','))
    }

    // ────────────────────────────────────────────────────────── the refusals

    /**
     * **A range of zero is refused, not published.**
     *
     * `TagRecogniser.publish` substitutes `0.0` when the tag's apparent size implies no range, and
     * `TagWorld.cameraFrame` then returns `(0, 0, 0)` — the camera's own optical centre, which is
     * a place, and a confidently wrong one a landing controller cannot tell from a right one. The
     * sighting still reaches the flight record; what does not reach the bus is a position we do
     * not have.
     */
    @Test
    fun aSightingWithNoRangeIsRefusedRatherThanPublishedAtTheOrigin() {
        assertNull(encode(sighting(range = 0.0)))
        assertEquals(Withheld.RANGE_UNKNOWN, ZenohEmission.detectionReason(sighting(range = 0.0)))
        // Negative too, which would mirror the tag through the camera.
        assertNull(encode(sighting(range = -2.0)))
        assertEquals(Withheld.RANGE_UNKNOWN, ZenohEmission.detectionReason(sighting(range = -2.0)))
    }

    @Test
    fun aNonFinitePositionIsRefused() {
        val nan = sighting().copy(x = Double.NaN)
        assertNull(encode(nan))
        assertEquals(Withheld.RANGE_UNKNOWN, ZenohEmission.detectionReason(nan))
        val inf = sighting().copy(z = Double.POSITIVE_INFINITY)
        assertNull(encode(inf))
        assertEquals(Withheld.RANGE_UNKNOWN, ZenohEmission.detectionReason(inf))
    }

    /** A frame that stated no geometry cannot be reasoned about, and is not guessed at 1920×1080. */
    @Test
    fun aSightingWithNoFrameGeometryIsRefused() {
        val noWidth = sighting().copy(imageWidth = 0)
        assertNull(encode(noWidth))
        assertEquals(Withheld.RESOLUTION_UNKNOWN, ZenohEmission.detectionReason(noWidth))
        val noHeight = sighting().copy(imageHeight = 0)
        assertNull(encode(noHeight))
        assertEquals(Withheld.RESOLUTION_UNKNOWN, ZenohEmission.detectionReason(noHeight))
    }

    /**
     * **`metric = false` does not withhold**, and that is a decision rather than an oversight.
     *
     * The alternative considered was bearing-only. It was rejected: `metric` is about *range*,
     * which is the best-known quantity here — the focal length is fitted to 1.2 % — while it was
     * the *bearing* that used to be bad, and since the nadir pixel was measured on 2026-07-28 that
     * error is ±0.16° rather than a 2.99° bias. Withholding would ship the worse number and keep
     * the better one.
     *
     * Neither does a non-zero `hamming`: the detector already ships `maxhamming = 1` and the latch
     * already requires three sightings of one id inside two seconds. A second, unmeasured
     * threshold at a transport would be policy invented in the wrong place —
     * `vision/TagDetection` says the decision margin is "carried and **not** thresholded" and
     * `docs/tag-detector.md` §6.5 says what a useful cut-off would be is unmeasured.
     */
    @Test
    fun aCoarsePoseAndACorrectedDecodeAreBothPublished() {
        assertEquals(Withheld.PUBLISHED, ZenohEmission.detectionReason(sighting(metric = false)))
        assertNotNull(encode(sighting(metric = false)))
        assertEquals(Withheld.PUBLISHED, ZenohEmission.detectionReason(sighting(hamming = 1)))
        assertNotNull(encode(sighting(hamming = 1)))
    }

    // ─────────────────────────────────────────────── the gate and the encoder

    /**
     * **The gate and the encoder are two statements about the same sighting, and they must agree.**
     *
     * [ZenohEmission.emission] throws when the reason says published and the encoder returns
     * nothing, rather than publishing nothing and counting it as a publish. This drives every
     * refusal *through* `ZenohEmission.detection` — not only through `detectionOrNull` — because
     * that is the only path on which the drift check runs, and a mutation that made the gate
     * permissive scored zero until this test existed. See the class doc.
     */
    @Test
    fun theGateAndTheEncoderCannotDisagree() {
        val cases = listOf(
            sighting(),
            sighting(range = 0.0),
            sighting(range = -2.0),
            sighting().copy(x = Double.NaN),
            sighting().copy(y = Double.NEGATIVE_INFINITY),
            sighting().copy(z = Double.NaN),
            sighting().copy(imageWidth = 0),
            sighting().copy(imageHeight = -1),
            sighting(metric = true),
            sighting(hamming = 1),
        )
        for (s in cases) {
            val e = ZenohEmission.detection(s, stamp, 1)
            assertEquals(ZenohChannel.DETECTIONS, e.channel)
            assertEquals("$s", ZenohEmission.detectionReason(s), e.reason)
            if (e.published) {
                assertNotNull("published with no bytes: $s", e.bytes)
                // And the bytes are the **array** codec's — the channel's type — not a second
                // opinion about them, and not the bare element.
                assertEquals(
                    Detection3DArrayCodec.encode(
                        ZenohTelemetryEncoder.detectionsOrNull(s, stamp, 1)!!,
                    ).toList(),
                    e.bytes!!.toList(),
                )
            } else {
                assertNull("withheld with bytes: $s", e.bytes)
            }
        }
    }

    /** `encode = false` tallies without paying for the payload, as the telemetry path does. */
    @Test
    fun aTallyOnlyEmissionCarriesNoBytes() {
        val e = ZenohEmission.detection(sighting(), stamp, 1, encode = false)
        assertEquals(Withheld.PUBLISHED, e.reason)
        assertNull(e.bytes)
    }

    /**
     * A payload of the wrong type is refused rather than reinterpreted.
     *
     * `nav_msgs.Path` is what this most resembles on the wire — a hoisted length, a header, then
     * elements — so it is the type a missing fingerprint check would let through.
     */
    @Test
    fun aDetectionPayloadIsRefusedByAnotherCodec() {
        val bytes = Detection3DArrayCodec.encode(array()!!)
        val ex = assertThrows(LcmDecodeException::class.java) { PathCodec.decode(bytes) }
        assertTrue(ex.message!!.contains("nav_msgs.Path"))
        // And the array is not the element: neither decodes as the other.
        assertThrows(LcmDecodeException::class.java) { Detection3DCodec.decode(bytes) }
    }

    /** Exactly one hypothesis: one sighting is one object, and two would mean two candidates. */
    @Test
    fun oneSightingProducesExactlyOneHypothesis() {
        assertEquals(1, encode()!!.results.size)
    }

    // ────────────────────────────────────────────────────────────── the array

    /**
     * **The channel carries a `Detection3DArray`**, holding the detection unchanged.
     *
     * The type changed from a bare `Detection3D` on 2026-07-28 (Ivan): ROS's own convention for a
     * detection topic, and what DiMOS consumes natively. The wrapper is four lines, and everything
     * a detection *claims* is still the element's — which is why every test above is unchanged.
     */
    @Test
    fun theWireMessageIsAnArrayHoldingTheDetectionUnchanged() {
        val a = array()!!
        assertEquals(1, a.detections.size)
        assertEquals(encode(), a.detections.single())
        assertEquals("vision_msgs.Detection3DArray", Detection3DArrayCodec.typeName)
        assertEquals("vision_msgs.Detection3DArray", ZenohChannel.DETECTIONS.type)
        assertEquals(
            "dimos/drone/detections/vision_msgs.Detection3DArray",
            ZenohChannel.DETECTIONS.keyOrNull(),
        )
    }

    /**
     * **One element, and that is the recogniser's doing rather than this contract's.**
     *
     * `TagRecogniser.publish` takes `found.largest` and discards the rest of the frame — size is
     * the only ranking available without a pose — so a frame is already reduced to one tag by the
     * time a sighting exists. The cardinality is asserted here as an observation about today, and
     * the two- and zero-element fixtures in `LcmFixtureTest` are what stop the *codec* resting on
     * it.
     */
    @Test
    fun theArrayCarriesOneElementBecauseASightingIsOneTag() {
        assertEquals(1, array()!!.detections.size)
        assertEquals(1, array(sighting(tagId = 12))!!.detections.size)
    }

    /**
     * **The array's header restates the element's, and does not invent an envelope.**
     *
     * With one element there is one arrival time and one frame, so there is nothing else it could
     * honestly be. A second stamp read from a clock here would be the *send* time, which is what
     * D-5 forbids and what `Sighting.ageMillisAt` exists to keep out of the message. If the array
     * ever carries more than one frame's detections this stops being right, and it stops being
     * right loudly, because the elements will disagree with it.
     */
    @Test
    fun theArraysHeaderIsTheElementsAndNotAFreshEnvelope() {
        val a = array(seq = 11)!!
        assertEquals(a.detections.single().header, a.header)
        assertEquals(stamp, a.header.stamp)
        assertEquals(11, a.header.seq)
        assertEquals("drone/camera_optical", a.header.frameId)
    }

    /**
     * **Every refusal survives the wrapping.**
     *
     * The array must not become a way for a withheld sighting to reach the bus as an *empty*
     * array — a message a consumer reads as "the detector looked and saw nothing", which is the
     * opposite of what happened.
     */
    @Test
    fun aRefusedSightingProducesNoArrayRatherThanAnEmptyOne() {
        for (s in listOf(
            sighting(range = 0.0),
            sighting(range = -2.0),
            sighting().copy(x = Double.NaN),
            sighting().copy(imageWidth = 0),
        )) {
            assertNull("$s", ZenohTelemetryEncoder.detectionsOrNull(s, stamp, 1))
        }
    }

    // ─────────────────────────────────────────────── the solved pose (2026-07-28)

    /**
     * A solve as `AprilTagDetector` delivers one: whole, with the errors sorted and the
     * defaults comfortably inside both gates. The default sighting's `pixelSize` is 112.5 —
     * above the 60 px gate — so `sighting().copy(solve = solve())` is a publishable solve and
     * every gate test below moves exactly one thing.
     */
    private fun solve(
        qx: Double = 0.028, qy: Double = -0.012, qz: Double = 0.612, qw: Double = 0.79,
        tx: Double = 0.71, ty: Double = -0.31, tz: Double = 3.46,
        err1: Double = 1.5e-7, err2: Double = 6.5e-7,
        tagSizeM: Double = 0.075,
    ) = TagPoseSolve(qx, qy, qz, qw, tx, ty, tz, err1, err2, tagSizeM)

    private fun solvedSighting(px: Double = 112.5) = sighting().copy(
        pixelSize = px, solve = solve(),
    )

    /**
     * **When both gates pass, the orientation is the solve's quaternion** — in
     * `drone/camera_optical`, apriltag's own tag frame (x right, y down on the printed face,
     * z into the tag), and it replaces the NaN, nothing else.
     */
    @Test
    fun aTrustedSolvePublishesItsOrientation() {
        val q = result(solvedSighting()).pose.pose.orientation
        assertEquals(LcmQuaternion(0.028, -0.012, 0.612, 0.79), q)
    }

    /**
     * **The position does not change when the solve passes.** It stays the centre-ray ×
     * apparent-size range it has been since the channel shipped — existing consumers and every
     * recorded flight stay comparable — and the solve's own translation is visible in the bbox
     * instead. The solve's `t` here deliberately differs from the sighting's `x,y,z`, so an
     * encoder that substituted one for the other fails.
     */
    @Test
    fun theSolveDoesNotMoveThePublishedPosition() {
        val s = solvedSighting()
        val solvedPos = result(s).pose.pose.position
        val legacyPos = result(s.copy(solve = null)).pose.pose.position
        assertEquals(legacyPos, solvedPos)
        assertNotEquals(solvedPos.x, s.solve!!.tx, 1e-9)
    }

    /**
     * **The box carries the solved pose and the tag's flat square.** `size` is
     * `(tagSize, tagSize, 0)`: the zero z-extent is a *measurement of the marker* — a printed
     * square genuinely has no thickness worth claiming — not the confident zero this project
     * refuses elsewhere, and not the NaN of a refused box.
     */
    @Test
    fun theBoxIsTheSolvedPoseAndTheFlatSquare() {
        val b = encode(solvedSighting())!!.bbox
        assertEquals(LcmPoint(0.71, -0.31, 3.46), b.center.position)
        assertEquals(LcmQuaternion(0.028, -0.012, 0.612, 0.79), b.center.orientation)
        // The size is the TAG'S, not zero: (0,0,0) would claim a point, and NaN would un-solve
        // a box whose whole point is that it was solved.
        assertEquals(0.075, b.size.x, 0.0)
        assertEquals(0.075, b.size.y, 0.0)
        assertEquals(0.0, b.size.z, 0.0)
        assertNotEquals(ZenohTelemetryEncoder.UNSOLVED_BOX, b)
    }

    /**
     * **Below the pixel gate the message is byte-for-byte the legacy one.** Not "mostly NaN" —
     * *equal to the message a solve-less sighting produces*, id included. The gate's failure
     * mode is degradation to the pre-solve contract, never a partial claim.
     *
     * 14 px is what the 75 mm tag measures at the 8 m arming altitude: the gate is exactly the
     * "appropriate height" rule, and this is the frame it refuses.
     */
    @Test
    fun belowThePixelGateTheMessageIsExactlyTheLegacyOne() {
        val gated = solvedSighting(px = 14.0)
        val legacy = gated.copy(solve = null)
        assertEquals(encode(legacy), encode(gated))
        assertEquals(
            Detection3DArrayCodec.encode(array(legacy)!!).toList(),
            Detection3DArrayCodec.encode(array(gated)!!).toList(),
        )
        // And just under the boundary, same story.
        assertEquals(encode(solvedSighting(px = 59.9).copy(solve = null)),
            encode(solvedSighting(px = 59.9)))
    }

    /** **An ambiguous solve degrades identically** — err1/err2 near 1 is flip country. */
    @Test
    fun anAmbiguousSolveDegradesToTheLegacyMessage() {
        val ambiguous = sighting().copy(solve = solve(err1 = 9.4e-7, err2 = 1.0e-6))
        assertEquals(encode(ambiguous.copy(solve = null)), encode(ambiguous))
        val q = result(ambiguous).pose.pose.orientation
        for (v in listOf(q.x, q.y, q.z, q.w)) assertTrue("must stay NaN", v.isNaN())
        assertEquals(ZenohTelemetryEncoder.UNSOLVED_BOX, encode(ambiguous)!!.bbox)
    }

    /**
     * **A partial solve cannot reach the bus through any field.** One NaN component and the
     * whole solve is refused — the gates cannot be bypassed by a solve that is only partly
     * there, because a quaternion with three good components is not three-quarters of a
     * rotation.
     */
    @Test
    fun aPartialSolveDegradesToTheLegacyMessage() {
        val partials = listOf(
            solve(qw = Double.NaN),
            solve(tx = Double.NaN),
            solve(tz = -1.0),
            solve(err1 = Double.NaN),
            solve(err2 = Double.NaN),
            solve(tagSizeM = 0.0),
        )
        for (p in partials) {
            val s = sighting().copy(solve = p)
            assertEquals("$p", encode(s.copy(solve = null)), encode(s))
        }
    }

    /** No second PnP minimum (`err2 = +Inf`) is the unambiguous case and publishes. */
    @Test
    fun anUnambiguousSolveWithNoSecondMinimumPublishes() {
        val s = sighting().copy(solve = solve(err2 = Double.POSITIVE_INFINITY))
        assertEquals(LcmQuaternion(0.028, -0.012, 0.612, 0.79), result(s).pose.pose.orientation)
    }

    /**
     * **The id is bare on solved and unsolved alike, so the NaN convention is now the ONLY
     * carrier of the solved fact** — load-bearing, not belt-and-braces. Since 2026-07-29
     * (Ivan's bare-label rule deleted `;pose=solved`) a consumer learns whether the gates
     * passed exclusively from the structure: a real box and orientation, or NaN throughout.
     * This test pins both halves together so the fact can never become uncarryable — the
     * mutation "suffix gone AND NaN signalling broken" must die here.
     */
    @Test
    fun theSolvedFactTravelsStructurallyAndTheIdStaysBare() {
        val solved = encode(solvedSighting())!!
        assertEquals("tag36h11:7", solved.id)
        assertFalse(solved.bbox.size.x.isNaN())
        assertFalse(solved.results[0].pose.pose.orientation.w.isNaN())
        // Ungated, for every way of failing a gate: the identical bare id, and NaN structure.
        for (s in listOf(
            sighting(),                                              // no solve at all
            solvedSighting(px = 14.0),                               // pixel gate
            sighting().copy(solve = solve(err1 = 9.9e-7, err2 = 1.0e-6)), // ambiguity gate
            sighting().copy(solve = solve(qx = Double.NaN)),         // partial solve
        )) {
            val m = encode(s)!!
            assertEquals("$s", "tag36h11:7", m.id)
            assertTrue("$s", m.bbox.size.x.isNaN())
            assertTrue("$s", m.results[0].pose.pose.orientation.w.isNaN())
        }
    }

    /**
     * The other refusals still hold with a perfect solve aboard: a solve cannot buy back a
     * sighting the channel refuses for its own reasons, and the gate/encoder drift check
     * covers the solved shapes too.
     */
    @Test
    fun aTrustedSolveDoesNotOverrideTheChannelsOwnRefusals() {
        assertNull(encode(solvedSighting().copy(z = 0.0)))
        assertNull(encode(solvedSighting().copy(imageWidth = 0)))
        for (s in listOf(solvedSighting(), solvedSighting(px = 14.0), solvedSighting().copy(z = 0.0))) {
            val e = ZenohEmission.detection(s, stamp, 1)
            assertEquals("$s", ZenohEmission.detectionReason(s), e.reason)
            if (e.published) assertNotNull(e.bytes) else assertNull(e.bytes)
        }
    }
}
