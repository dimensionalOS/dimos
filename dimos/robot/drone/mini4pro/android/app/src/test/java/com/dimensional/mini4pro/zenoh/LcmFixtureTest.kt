package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Byte-equality against DiMOS's own encoder.
 *
 * Every fixture under `src/test/resources/lcm/` was produced by
 * `tools/lcmfixtures/generate.py`, which imports the generated Python bindings at
 * `~/coding/dimos-lcm/generated/python_lcm_msgs/` and calls each message's
 * `lcm_encode()` — literally the call DiMOS makes when it publishes
 * (`dimos/protocol/pubsub/encoders.py:104`). None of these bytes were written by
 * hand or derived from reading the format.
 *
 * That distinction is the entire value of this file. A round-trip test proves a
 * codec is self-consistent, which a codec with the fields in the wrong order also
 * is. Only an encoder we did not write can tell us we are wrong.
 *
 * Each type is checked twice: **encode** must reproduce the fixture byte for
 * byte, and **decode** must recover the same values from it.
 *
 * ## Mutation-checked 2026-07-27, on the six types added with the live publisher
 *
 * One breakage at a time, whole suite run, reverted after each, by `tmp/mutate.py`. **Counts are
 * failing tests across the whole 2068-test suite, measured, not estimated.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `TFMessage` writes a header before its edges | 4 |
 *  | `CameraInfo` writes `width` before `height` | 4 |
 *  | `CameraInfo` writes `D_length` after the header | 5 |
 *  | `CompressedVideo` length-prefixes its payload a second time | 2 |
 *  | `Transform` reuses the `Pose` fingerprint | 2 |
 *  | `CompressedVideo` nests `std_msgs.Time` rather than `builtin_interfaces.Time` | **0 — equivalent** |
 *
 * **The last row is the one worth reading.** Both time types are `int32 sec, int32 nsec`, and a
 * nested LCM struct carries **no fingerprint of its own** — so swapping them produces byte-identical
 * messages and no test anywhere could distinguish them. The distinction lives entirely in the
 * *outer* type's recursive hash, `LcmFingerprints.FOXGLOVE_MSGS_COMPRESSED_VIDEO`, which is a
 * pinned constant this file checks against a fixture DiMOS's own Python produced. That is the only
 * place it is checkable at all, and it is checked.
 *
 * The other five are the field orders nobody would guess, and they are exactly the rows that would
 * have been silent under a round-trip test: `CameraInfo`'s height-before-width in particular
 * decodes cleanly into a portrait camera at 1920×1080, and every focal length a consumer scales
 * from the width would then be scaled from the height.
 *
 * ## Mutation-checked 2026-07-28, on the four vision types
 *
 * Same harness, same rule, whole 2218-test suite per mutation, `test-results/testDebugUnitTest`
 * deleted before each run. **Nothing survived and nothing was NO-RUN.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `Detection3D` writes `results_length` after the header | 3 |
 *  | `Detection3D` writes a zero `results_length` while emitting the hypotheses | 2 |
 *  | `Detection3D` writes `id` before `bbox` | 3 |
 *  | `ObjectHypothesis` writes `score` before `class_id` | 4 |
 *  | `BoundingBox3D` reuses the `Pose` fingerprint | 1 |
 *  | `Detection3D` reuses `nav_msgs.Path`'s fingerprint | 4 |
 *
 * The second row is the one `Detection3D` needs three fixtures for. In a `nav_msgs.Path` a wrong
 * count truncates the message; here the same number of bytes go out and `bbox` is read out of the
 * tail of a hypothesis and `id` out of `bbox` — a message that decodes without complaint into a
 * box that is not NaN and an id that is not a qualification.
 *
 * ## And again 2026-07-28, when `Detection3DArray` became the wire type
 *
 * Suite 2229. Nothing survived, nothing NO-RUN.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `Detection3DArray` writes `detections_length` after the header | 3 |
 *  | `Detection3DArray` writes a zero `detections_length` while emitting the detections | 2 |
 *  | `Detection3DArray` reuses `nav_msgs.Path`'s fingerprint | 4 |
 *
 * The array's own hoisted length is a *second* one on the same wire, inside which each element has
 * its own — which is why the two-element fixture exists: with one element carrying one result the
 * two counts are both 1 and no test could tell them apart.
 */
class LcmFixtureTest {

    private fun hex(b: ByteArray) = b.joinToString("") { "%02x".format(it) }

    /**
     * Encode to the exact fixture bytes, and decode the fixture back to the exact
     * value. Failures print hex, because a byte offset is the only useful thing
     * to know when a field lands in the wrong place.
     */
    private fun <T> check(name: String, codec: LcmCodec<T>, value: T) {
        val expected = LcmFixtures.bytes(name)
        val actual = codec.encode(value)
        assertEquals("$name: encoded bytes", hex(expected), hex(actual))
        assertEquals("$name: decoded value", value, codec.decode(expected))
    }

    @Test
    fun `std_msgs Time`() = check("std_msgs_Time", TimeCodec, LcmFixtures.time())

    @Test
    fun `std_msgs Header`() = check("std_msgs_Header", HeaderCodec, LcmFixtures.header())

    @Test
    fun `std_msgs String carries multi-byte utf-8`() =
        check("std_msgs_String_utf8", StringCodec, LcmFixtures.UTF8_TEXT)

    @Test
    fun `std_msgs String empty is five bytes of body`() {
        check("std_msgs_String_empty", StringCodec, "")
        // 8 fingerprint + 4 length + 0 payload + 1 NUL.
        assertEquals(13, LcmFixtures.bytes("std_msgs_String_empty").size)
    }

    /**
     * The `wind` channel's whole message: 8 fingerprint + 4 float, twelve bytes. The fixture
     * value is landing14's measured 9.1 m/s peak, and `9.1f` here is the same narrowing
     * `windOrNull` performs — a codec that widened to double on the wire would double the
     * message and fail on length before it failed on bits.
     */
    @Test
    fun `std_msgs Float32 is a fingerprint and one single`() {
        check("std_msgs_Float32", Float32Codec, LcmFloat32(9.1f))
        assertEquals(12, LcmFixtures.bytes("std_msgs_Float32").size)
    }

    @Test
    fun `geometry_msgs Point`() = check("geometry_msgs_Point", PointCodec, LcmFixtures.point())

    @Test
    fun `geometry_msgs Vector3`() =
        check("geometry_msgs_Vector3", Vector3Codec, LcmFixtures.vector3())

    @Test
    fun `geometry_msgs Quaternion puts w last`() =
        check("geometry_msgs_Quaternion", QuaternionCodec, LcmFixtures.quat())

    @Test
    fun `geometry_msgs Pose`() = check("geometry_msgs_Pose", PoseCodec, LcmFixtures.pose())

    @Test
    fun `geometry_msgs PoseWithCovariance`() = check(
        "geometry_msgs_PoseWithCovariance", PoseWithCovarianceCodec, LcmFixtures.poseWithCov()
    )

    @Test
    fun `geometry_msgs Twist`() = check("geometry_msgs_Twist", TwistCodec, LcmFixtures.twist())

    @Test
    fun `geometry_msgs TwistWithCovariance`() = check(
        "geometry_msgs_TwistWithCovariance", TwistWithCovarianceCodec, LcmFixtures.twistWithCov()
    )

    @Test
    fun `geometry_msgs TwistStamped`() =
        check("geometry_msgs_TwistStamped", TwistStampedCodec, LcmFixtures.twistStamped())

    @Test
    fun `geometry_msgs PoseStamped`() =
        check("geometry_msgs_PoseStamped", PoseStampedCodec, LcmFixtures.poseStamped())

    @Test
    fun `geometry_msgs PointStamped`() =
        check("geometry_msgs_PointStamped", PointStampedCodec, LcmFixtures.pointStamped())

    @Test
    fun `nav_msgs Odometry`() =
        check("nav_msgs_Odometry", OdometryCodec, LcmFixtures.odometry())

    @Test
    fun `nav_msgs Path with three poses`() =
        check("nav_msgs_Path_three", PathCodec, LcmFixtures.pathThree())

    /**
     * An empty `Path` is 33 bytes: the fingerprint, the zero count, and a header
     * whose only variable part is `"odom"`. If the count were written *after* the
     * header the total would be the same and only the order would differ — which
     * is why this fixture exists rather than a hand-written assertion.
     */
    @Test
    fun `nav_msgs Path with no poses`() {
        check("nav_msgs_Path_empty", PathCodec, LcmFixtures.pathEmpty())
        assertEquals(33, LcmFixtures.bytes("nav_msgs_Path_empty").size)
    }

    @Test
    fun `nav_msgs Path writes its length before its header`() {
        // Bytes 8..11 are poses_length; bytes 12..15 would be header.seq if the
        // count were not hoisted. The three-pose fixture pins which is which.
        val bytes = LcmFixtures.bytes("nav_msgs_Path_three")
        val r = LcmReader(bytes)
        r.expectFingerprint(LcmFingerprints.NAV_MSGS_PATH, "nav_msgs.Path")
        assertEquals(3, r.readInt())
        assertEquals(11, r.readInt())
    }

    @Test
    fun `sensor_msgs NavSatStatus`() =
        check("sensor_msgs_NavSatStatus", NavSatStatusCodec, LcmFixtures.navSatStatus())

    /** `STATUS_NO_FIX` is −1: an `int8`, signed. A slip to `int` reads 255. */
    @Test
    fun `sensor_msgs NavSatStatus no fix is negative one`() {
        check(
            "sensor_msgs_NavSatStatus_nofix",
            NavSatStatusCodec,
            LcmFixtures.navSatStatus(LcmNavSatStatus.STATUS_NO_FIX, 4),
        )
        assertEquals(0xFF.toByte(), LcmFixtures.bytes("sensor_msgs_NavSatStatus_nofix")[8])
    }

    @Test
    fun `sensor_msgs NavSatFix`() =
        check("sensor_msgs_NavSatFix", NavSatFixCodec, LcmFixtures.navSatFix())

    @Test
    fun `sensor_msgs Imu`() = check("sensor_msgs_Imu", ImuCodec, LcmFixtures.imu())

    @Test
    fun `sensor_msgs BatteryState`() =
        check("sensor_msgs_BatteryState", BatteryStateCodec, LcmFixtures.battery())

    @Test
    fun `sensor_msgs BatteryState with no cells`() =
        check("sensor_msgs_BatteryState_nocells", BatteryStateCodec, LcmFixtures.batteryNoCells())

    /**
     * Both cell-array counts precede the header, and the bodies come far later.
     * Pinned directly because it is the least believable thing in the format.
     */
    @Test
    fun `sensor_msgs BatteryState writes both cell counts before its header`() {
        val r = LcmReader(LcmFixtures.bytes("sensor_msgs_BatteryState"))
        r.expectFingerprint(LcmFingerprints.SENSOR_MSGS_BATTERY_STATE, "sensor_msgs.BatteryState")
        assertEquals(3, r.readInt()) // cell_voltage_length
        assertEquals(2, r.readInt()) // cell_temperature_length
        assertEquals(77, r.readInt()) // header.seq
    }

    // ── the frame tree, the intrinsics and the video ─────────────────────────
    //
    // Added 2026-07-27 to bring the live publisher to the store's parity. Three of these six carry
    // a shape nothing above has, and each has its own byte-level assertion below the round trip.

    @Test
    fun `builtin_interfaces Time is std_msgs Time with a different fingerprint`() {
        check("builtin_interfaces_Time", BuiltinTimeCodec, LcmFixtures.builtinTime())
        // Identical bodies, different eight bytes in front. `CompressedVideo` nests this one and
        // every stamped type in the catalogue nests the other; nothing on the wire distinguishes
        // them and the recursive hash of their containers does.
        val builtin = LcmFixtures.bytes("builtin_interfaces_Time")
        val std = LcmFixtures.bytes("std_msgs_Time")
        assertEquals(hex(builtin.copyOfRange(8, 16)), hex(std.copyOfRange(8, 16)))
        org.junit.Assert.assertNotEquals(
            hex(builtin.copyOfRange(0, 8)),
            hex(std.copyOfRange(0, 8)),
        )
    }

    @Test
    fun `geometry_msgs Transform`() =
        check("geometry_msgs_Transform", TransformCodec, LcmFixtures.transform())

    /**
     * The bodies of a `Transform` and a `Pose` are the same bytes and the messages are not.
     *
     * lcm-gen hashes field *names* as well as layout, so `translation`/`rotation` and
     * `position`/`orientation` hash differently over identical wire shapes. A codec that reached
     * for `PoseCodec`'s body would produce exactly these bytes behind exactly the wrong
     * fingerprint, and no round trip against itself would notice.
     */
    @Test
    fun `a Transform body is a Pose body behind a different fingerprint`() {
        val transform = TransformCodec.encode(LcmFixtures.transform(LcmFixtures.vector3()))
        val pose = PoseCodec.encode(LcmFixtures.pose(LcmFixtures.point(-31.5, 0.125, 64.25)))
        assertEquals(hex(pose.copyOfRange(8, pose.size)), hex(transform.copyOfRange(8, transform.size)))
        org.junit.Assert.assertNotEquals(
            hex(pose.copyOfRange(0, 8)),
            hex(transform.copyOfRange(0, 8)),
        )
    }

    @Test
    fun `geometry_msgs TransformStamped`() = check(
        "geometry_msgs_TransformStamped",
        TransformStampedCodec,
        LcmFixtures.transformStamped("drone/base_link", "drone/camera"),
    )

    @Test
    fun `tf2_msgs TFMessage with three edges`() =
        check("tf2_msgs_TFMessage_three", TfMessageCodec, LcmFixtures.tfThree())

    /**
     * An empty tree is twelve bytes: the fingerprint and a zero count, and **no header at all**.
     *
     * `TFMessage` is the one type in this catalogue that is purely an array — unlike
     * `nav_msgs.Path`, there is nothing after the length but the elements. A codec that wrote a
     * header here would produce a message a `TFMessage` decoder reads as an absurd edge count.
     */
    @Test
    fun `tf2_msgs TFMessage with no edges is twelve bytes`() {
        check("tf2_msgs_TFMessage_empty", TfMessageCodec, LcmFixtures.tfEmpty())
        assertEquals(12, LcmFixtures.bytes("tf2_msgs_TFMessage_empty").size)
        val r = LcmReader(LcmFixtures.bytes("tf2_msgs_TFMessage_three"))
        r.expectFingerprint(LcmFingerprints.TF2_MSGS_TF_MESSAGE, "tf2_msgs.TFMessage")
        assertEquals(3, r.readInt())
        // Straight into the first edge's header.seq — no message-level header in between.
        assertEquals(1, r.readInt())
    }

    @Test
    fun `sensor_msgs RegionOfInterest`() = check(
        "sensor_msgs_RegionOfInterest", RegionOfInterestCodec, LcmFixtures.regionOfInterest(),
    )

    @Test
    fun `sensor_msgs CameraInfo with no distortion claim`() =
        check("sensor_msgs_CameraInfo", CameraInfoCodec, LcmFixtures.cameraInfo())

    @Test
    fun `sensor_msgs CameraInfo with a distortion array`() = check(
        "sensor_msgs_CameraInfo_distorted", CameraInfoCodec, LcmFixtures.cameraInfoDistorted(),
    )

    /**
     * `D_length` before the header, and **`height` before `width`**.
     *
     * Two orderings nobody expects, both lcm-gen's. The height/width swap is the dangerous one: at
     * 1920×1080 it is a message that decodes cleanly into a portrait camera, and every focal
     * length a consumer scales from the width would then be scaled from the height.
     */
    @Test
    fun `sensor_msgs CameraInfo hoists D_length and writes height before width`() {
        val r = LcmReader(LcmFixtures.bytes("sensor_msgs_CameraInfo_distorted"))
        r.expectFingerprint(LcmFingerprints.SENSOR_MSGS_CAMERA_INFO, "sensor_msgs.CameraInfo")
        assertEquals(5, r.readInt()) // D_length, ahead of everything
        assertEquals(43, r.readInt()) // header.seq
        r.readInt(); r.readInt(); r.readString() // stamp.sec, stamp.nsec, frame_id
        assertEquals(540, r.readInt()) // height first
        assertEquals(960, r.readInt()) // then width
    }

    @Test
    fun `foxglove_msgs CompressedVideo`() =
        check("foxglove_msgs_CompressedVideo", CompressedVideoCodec, LcmFixtures.compressedVideo())

    @Test
    fun `foxglove_msgs CompressedVideo with no payload`() = check(
        "foxglove_msgs_CompressedVideo_empty",
        CompressedVideoCodec,
        LcmFixtures.compressedVideoEmpty(),
    )

    /**
     * **The access unit goes on the wire as raw bytes**, in the middle of the message, between two
     * strings — not length-prefixed a second time, not escaped, not encoded as text.
     *
     * Asserted against the fixture's own offsets rather than against our encoder, because "we copy
     * the bytes" is the single load-bearing claim of the whole video path: the sidecar already
     * satisfies this schema byte for byte, and anything that transformed the payload here would
     * quietly break that equality while every round trip kept passing.
     */
    @Test
    fun `the video payload is the aircraft's own bytes, verbatim and in place`() {
        val bytes = LcmFixtures.bytes("foxglove_msgs_CompressedVideo")
        val payload = LcmFixtures.videoPayload()
        val r = LcmReader(bytes)
        r.expectFingerprint(
            LcmFingerprints.FOXGLOVE_MSGS_COMPRESSED_VIDEO, "foxglove_msgs.CompressedVideo",
        )
        assertEquals("data_length is the first field", payload.size, r.readInt())
        assertEquals(LcmFixtures.SEC, r.readInt())
        assertEquals(LcmFixtures.NSEC, r.readInt())
        assertEquals("drone/camera_optical", r.readString())
        assertEquals(hex(payload), hex(r.readBytes(payload.size)))
        assertEquals("h264", r.readString())
        assertEquals(0, r.remaining)
        // And the encoder can price a frame without building one — the arithmetic
        // `docs/measurements/` uses rather than estimating beside it.
        assertEquals(
            bytes.size,
            CompressedVideoCodec.encodedSize(payload.size, "drone/camera_optical", "h264"),
        )
    }

    // ── the tag detections ───────────────────────────────────────────────────
    //
    // Added 2026-07-28 with the `detections` channel. `Detection3D` is the only type in this
    // catalogue whose hoisted array length has more fields *after* the array, and the only one
    // whose real message carries NaN — so both get a byte-level assertion below the round trip.

    @Test
    fun `vision_msgs ObjectHypothesis`() =
        check("vision_msgs_ObjectHypothesis", ObjectHypothesisCodec, LcmFixtures.hypothesis())

    @Test
    fun `vision_msgs ObjectHypothesisWithPose`() = check(
        "vision_msgs_ObjectHypothesisWithPose",
        ObjectHypothesisWithPoseCodec,
        LcmFixtures.hypothesisWithPose(),
    )

    @Test
    fun `vision_msgs BoundingBox3D`() =
        check("vision_msgs_BoundingBox3D", BoundingBox3DCodec, LcmFixtures.unsolvedBox())

    @Test
    fun `vision_msgs Detection3D`() =
        check("vision_msgs_Detection3D", Detection3DCodec, LcmFixtures.detection3D())

    @Test
    fun `vision_msgs Detection3D with two results`() =
        check("vision_msgs_Detection3D_two", Detection3DCodec, LcmFixtures.detection3DTwo())

    @Test
    fun `vision_msgs Detection3D with no results`() =
        check("vision_msgs_Detection3D_empty", Detection3DCodec, LcmFixtures.detection3DEmpty())

    /**
     * `results_length` before the header, **and two fields after the array**.
     *
     * The shape `nav_msgs.Path` does not have, and it is the reason this type needs three
     * fixtures rather than one. A wrong count in a `Path` truncates the message; a wrong count
     * here leaves the same number of bytes and reads `bbox` out of the tail of a hypothesis and
     * `id` out of `bbox` — a message that decodes without complaint into a box that is not NaN
     * and an id that is not a qualification.
     */
    @Test
    fun `vision_msgs Detection3D hoists results_length ahead of its header`() {
        val r = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3D_two"))
        r.expectFingerprint(LcmFingerprints.VISION_MSGS_DETECTION_3D, "vision_msgs.Detection3D")
        assertEquals(2, r.readInt()) // results_length, ahead of everything
        assertEquals(7, r.readInt()) // header.seq
        // And the empty case, where a length written after the header would decode `bbox` out of
        // the header's own tail rather than failing.
        val e = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3D_empty"))
        e.expectFingerprint(LcmFingerprints.VISION_MSGS_DETECTION_3D, "vision_msgs.Detection3D")
        assertEquals(0, e.readInt())
        assertEquals(8, e.readInt())
    }

    /**
     * **The unsolved fields are the canonical quiet NaN on the wire**, all eight bytes of it.
     *
     * This is the assertion that makes "we refused to fill it" a wire fact rather than a comment.
     * Python's `struct.pack('>d', float('nan'))` wrote these bytes; a Kotlin codec that routed a
     * double through a float, normalised the payload, or substituted a zero or an identity
     * quaternion for a non-finite value fails here and passes every round trip.
     *
     * Read straight out of the fixture, at offsets counted from the schema, so a field order that
     * moved would fail too.
     */
    @Test
    fun `the unsolved orientation and the refused box are NaN on the wire`() {
        val r = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3D"))
        r.expectFingerprint(LcmFingerprints.VISION_MSGS_DETECTION_3D, "vision_msgs.Detection3D")
        assertEquals(1, r.readInt())
        r.readInt(); r.readInt(); r.readInt(); r.readString() // header
        assertEquals("tag36h11:7", r.readString())
        assertEquals(41.75, r.readDouble(), 0.0)
        // The pose: a real position, then four NaN.
        assertEquals(-0.375, r.readDouble(), 0.0)
        assertEquals(0.8125, r.readDouble(), 0.0)
        assertEquals(3.5, r.readDouble(), 0.0)
        repeat(4) { assertNaNBits(r.readDouble()) }
        // ROS's own "covariance unknown" — zeros, not NaN. The distinction is deliberate: NaN is
        // this project's word for a quantity with no feed, and ROS has its own word for this one.
        repeat(36) { assertEquals(0.0, r.readDouble(), 0.0) }
        // The box, refused entire: centre, orientation and size, ten NaN in a row.
        repeat(10) { assertNaNBits(r.readDouble()) }
        // And the qualification that travels with the pose.
        assertEquals(LcmFixtures.DETECTION_ID, r.readString())
        assertEquals(0, r.remaining)
    }

    /**
     * The solved variant, 2026-07-28: the fixture DiMOS's own bindings encoded for a message
     * whose gates both passed — a real orientation in `results[0]`, the solved pose and the
     * flat-square size in the bbox, the same bare id. The one message shape the NaN
     * assertions above cannot cover, pinned the same way they are.
     */
    @Test
    fun `vision_msgs Detection3D solved`() =
        check("vision_msgs_Detection3D_solved", Detection3DCodec, LcmFixtures.detection3DSolved())

    /**
     * The solved box's bytes, read at schema offsets: position untouched by the solve, the
     * planted quaternion where the NaNs sit on the unsolved wire, the tag's `(0.075, 0.075, 0)`
     * as the size — the zero z-extent is a measurement of a flat marker, not a refused zero —
     * and the marker on the id.
     */
    @Test
    fun `the solved orientation and box are real values on the wire`() {
        val r = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3D_solved"))
        r.expectFingerprint(LcmFingerprints.VISION_MSGS_DETECTION_3D, "vision_msgs.Detection3D")
        assertEquals(1, r.readInt())
        r.readInt(); r.readInt(); r.readInt(); r.readString() // header
        assertEquals("tag36h11:7", r.readString())
        assertEquals(41.75, r.readDouble(), 0.0)
        // The position: identical to the unsolved fixture's — the solve must not move it.
        assertEquals(-0.375, r.readDouble(), 0.0)
        assertEquals(0.8125, r.readDouble(), 0.0)
        assertEquals(3.5, r.readDouble(), 0.0)
        // The orientation: the planted solve, bit-exact, where NaN used to be.
        assertEquals(0.20056212, r.readDouble(), 0.0)
        assertEquals(-0.09442657, r.readDouble(), 0.0)
        assertEquals(0.47555491, r.readDouble(), 0.0)
        assertEquals(0.85078055, r.readDouble(), 0.0)
        repeat(36) { assertEquals(0.0, r.readDouble(), 0.0) }
        // The box: solved centre (NOT the position), solved orientation, flat-square size.
        assertEquals(0.0625, r.readDouble(), 0.0)
        assertEquals(-0.03125, r.readDouble(), 0.0)
        assertEquals(3.46875, r.readDouble(), 0.0)
        assertEquals(0.20056212, r.readDouble(), 0.0)
        assertEquals(-0.09442657, r.readDouble(), 0.0)
        assertEquals(0.47555491, r.readDouble(), 0.0)
        assertEquals(0.85078055, r.readDouble(), 0.0)
        assertEquals(0.075, r.readDouble(), 0.0)
        assertEquals(0.075, r.readDouble(), 0.0)
        assertEquals(0.0, r.readDouble(), 0.0)
        assertEquals(LcmFixtures.DETECTION_ID_SOLVED, r.readString())
        assertEquals(0, r.remaining)
    }

    @Test
    fun `vision_msgs Detection3DArray`() = check(
        "vision_msgs_Detection3DArray", Detection3DArrayCodec, LcmFixtures.detection3DArray(),
    )

    @Test
    fun `vision_msgs Detection3DArray solved`() = check(
        "vision_msgs_Detection3DArray_solved",
        Detection3DArrayCodec,
        LcmFixtures.detection3DArraySolved(),
    )

    @Test
    fun `vision_msgs Detection3DArray with two detections`() = check(
        "vision_msgs_Detection3DArray_two",
        Detection3DArrayCodec,
        LcmFixtures.detection3DArrayTwo(),
    )

    @Test
    fun `vision_msgs Detection3DArray with no detections`() = check(
        "vision_msgs_Detection3DArray_empty",
        Detection3DArrayCodec,
        LcmFixtures.detection3DArrayEmpty(),
    )

    /**
     * **Two hoisted lengths, nested, on one wire** — and the outer one is not the inner one.
     *
     * `Detection3DArray` is `nav_msgs.Path`'s shape one type further out, and each element is a
     * `Detection3D` with a hoisted length of its own. A one-element array carrying a one-result
     * detection cannot tell the two apart at all, which is why the two-element fixture exists: its
     * outer count is 2 and its first element's inner count is 1.
     */
    @Test
    fun `vision_msgs Detection3DArray hoists its own length ahead of the inner one`() {
        val r = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3DArray_two"))
        r.expectFingerprint(
            LcmFingerprints.VISION_MSGS_DETECTION_3D_ARRAY, "vision_msgs.Detection3DArray",
        )
        assertEquals("detections_length, ahead of everything", 2, r.readInt())
        assertEquals("the array's own header.seq", 7, r.readInt())
        r.readInt(); r.readInt(); r.readString() // the array's stamp and frame_id
        // Straight into the first element, whose own length field comes first in turn.
        assertEquals("the first element's results_length", 1, r.readInt())
        assertEquals("the first element's header.seq", LcmFixtures.SEQ, r.readInt())

        // And the empty case: fingerprint, a zero count, then a header and nothing else.
        val e = LcmReader(LcmFixtures.bytes("vision_msgs_Detection3DArray_empty"))
        e.expectFingerprint(
            LcmFingerprints.VISION_MSGS_DETECTION_3D_ARRAY, "vision_msgs.Detection3DArray",
        )
        assertEquals(0, e.readInt())
        assertEquals(8, e.readInt())
        assertEquals(49, LcmFixtures.bytes("vision_msgs_Detection3DArray_empty").size)
    }

    /**
     * **The array is the element, framed** — its body after the header is the inner message's
     * body, byte for byte, and the fingerprints in front of them differ.
     *
     * The property that makes the wrapping free: whatever `LcmFixtureTest` and `ZenohDetectionTest`
     * prove about a `Detection3D` is true of the bytes actually on the wire, in place.
     */
    @Test
    fun `the array's payload is the detection's own bytes behind a different fingerprint`() {
        val array = LcmFixtures.bytes("vision_msgs_Detection3DArray")
        val inner = LcmFixtures.bytes("vision_msgs_Detection3D")
        // 8 fingerprint + 4 count + the array's header, which is the element's header repeated.
        val headerBytes = 4 + 8 + 4 + "drone/camera_optical".length + 1
        val offset = 8 + 4 + headerBytes
        assertEquals(
            hex(inner.copyOfRange(8, inner.size)),
            hex(array.copyOfRange(offset, array.size)),
        )
        org.junit.Assert.assertNotEquals(hex(inner.copyOfRange(0, 8)), hex(array.copyOfRange(0, 8)))
        // The array's header really is the element's, restated rather than invented.
        assertEquals(
            hex(array.copyOfRange(12, offset)),
            hex(inner.copyOfRange(12, 12 + headerBytes)),
        )
    }

    // ── diagnostic_msgs: the `warnings` channel ──────────────────────────────

    /**
     * The three `diagnostic_msgs` types, against DiMOS's own encoder.
     *
     * The fixture is **landing17's own wind warning**: DJI's `LEVEL_2` at t=111.078 with
     * `windSpeedDmS` peaking at 142 — 14.2 m/s against this airframe's rated ~10.7 — mapped to
     * `WarnLevel.WARNING` and therefore to `DiagnosticStatus.ERROR`, one step up, the same
     * promotion the `STATUSTEXT` gets.
     *
     * What these bytes actually catch is the **hoisted `values_length`**: it is declared ahead of
     * `level`, not next to the array, so a codec that writes it where the array is produces a
     * message that still decodes — into three shifted strings and a plausible level. Nothing but
     * an encoder we did not write can tell us that.
     */
    @Test
    fun `diagnostic_msgs KeyValue is two strings`() =
        check("diagnostic_msgs_KeyValue", KeyValueCodec, LcmKeyValue("state", "LEVEL_2"))

    @Test
    fun `diagnostic_msgs DiagnosticStatus hoists its values length ahead of the level`() =
        check(
            "diagnostic_msgs_DiagnosticStatus",
            DiagnosticStatusCodec,
            LcmFixtures.windStatus(),
        )

    @Test
    fun `diagnostic_msgs DiagnosticArray is the warnings channel's message`() {
        check(
            "diagnostic_msgs_DiagnosticArray",
            DiagnosticArrayCodec,
            LcmDiagnosticArray(
                header = LcmFixtures.header("drone/base_link"),
                status = listOf(LcmFixtures.windStatus()),
            ),
        )
    }

    /**
     * A cleared health warning: `OK`, a hardware id, and **no values at all**.
     *
     * The empty nested array is the case a length written after the strings reads as garbage
     * rather than as nothing — the same shape `vision_msgs_Detection3DArray_empty` pins one level
     * out, and a real message: every `cleared` this bridge sends is this shape.
     */
    @Test
    fun `diagnostic_msgs DiagnosticArray carries an empty values list`() =
        check(
            "diagnostic_msgs_DiagnosticArray_empty",
            DiagnosticArrayCodec,
            LcmDiagnosticArray(
                header = LcmFixtures.header("drone/base_link", seq = 11),
                status = listOf(
                    LcmDiagnosticStatus(
                        level = LcmDiagnosticStatus.LEVEL_OK,
                        name = "health/0x1600A0",
                        message = "DJI cleared: Aircraft overheating",
                        hardwareId = "0/0",
                        values = emptyList(),
                    )
                ),
            ),
        )

    /**
     * Not merely `isNaN`: the exact bit pattern, because two NaNs with different payloads compare
     * equal under `isNaN` and are different bytes on a bus.
     */
    private fun assertNaNBits(v: Double) = assertEquals(
        "%016x".format(java.lang.Double.doubleToRawLongBits(Double.NaN)),
        "%016x".format(java.lang.Double.doubleToRawLongBits(v)),
    )
}
