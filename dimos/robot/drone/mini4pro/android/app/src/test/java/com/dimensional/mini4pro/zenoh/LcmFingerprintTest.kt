package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The pinned fingerprints, checked against the ones DiMOS actually sends.
 *
 * [LcmFingerprints] holds constants rather than a computation, so nothing inside
 * this codebase can tell you they are right. What can is the first eight bytes of
 * every fixture in `src/test/resources/lcm/`, which came out of
 * `_get_packed_fingerprint()` on the generated Python bindings.
 *
 * A fingerprint that drifts — because someone regenerates `dimos-lcm` from a
 * changed `.msg`, or edits a field name — must fail here. The alternative is a
 * receiver that silently drops every message, discovered in the air.
 */
class LcmFingerprintTest {

    /** Reads the 8-byte prefix a fixture actually carries. */
    private fun prefixOf(fixture: String): Long = LcmReader(LcmFixtures.bytes(fixture)).readLong()

    private fun check(fixture: String, pinned: Long) {
        assertEquals(
            "$fixture: pinned fingerprint disagrees with the bytes DiMOS produced",
            java.lang.Long.toHexString(prefixOf(fixture)),
            java.lang.Long.toHexString(pinned),
        )
    }

    @Test
    fun `every pinned fingerprint matches the fixture DiMOS generated`() {
        check("std_msgs_Time", LcmFingerprints.STD_MSGS_TIME)
        check("builtin_interfaces_Time", LcmFingerprints.BUILTIN_INTERFACES_TIME)
        check("std_msgs_Header", LcmFingerprints.STD_MSGS_HEADER)
        check("std_msgs_String_utf8", LcmFingerprints.STD_MSGS_STRING)
        check("std_msgs_Float32", LcmFingerprints.STD_MSGS_FLOAT32)
        check("geometry_msgs_Point", LcmFingerprints.GEOMETRY_MSGS_POINT)
        check("geometry_msgs_Vector3", LcmFingerprints.GEOMETRY_MSGS_VECTOR3)
        check("geometry_msgs_Quaternion", LcmFingerprints.GEOMETRY_MSGS_QUATERNION)
        check("geometry_msgs_Pose", LcmFingerprints.GEOMETRY_MSGS_POSE)
        check("geometry_msgs_PoseWithCovariance", LcmFingerprints.GEOMETRY_MSGS_POSE_WITH_COVARIANCE)
        check("geometry_msgs_Twist", LcmFingerprints.GEOMETRY_MSGS_TWIST)
        check("geometry_msgs_TwistStamped", LcmFingerprints.GEOMETRY_MSGS_TWIST_STAMPED)
        check("geometry_msgs_TwistWithCovariance", LcmFingerprints.GEOMETRY_MSGS_TWIST_WITH_COVARIANCE)
        check("geometry_msgs_PoseStamped", LcmFingerprints.GEOMETRY_MSGS_POSE_STAMPED)
        check("geometry_msgs_PointStamped", LcmFingerprints.GEOMETRY_MSGS_POINT_STAMPED)
        check("nav_msgs_Odometry", LcmFingerprints.NAV_MSGS_ODOMETRY)
        check("nav_msgs_Path_three", LcmFingerprints.NAV_MSGS_PATH)
        check("sensor_msgs_NavSatStatus", LcmFingerprints.SENSOR_MSGS_NAV_SAT_STATUS)
        check("sensor_msgs_NavSatFix", LcmFingerprints.SENSOR_MSGS_NAV_SAT_FIX)
        check("sensor_msgs_Imu", LcmFingerprints.SENSOR_MSGS_IMU)
        check("sensor_msgs_BatteryState", LcmFingerprints.SENSOR_MSGS_BATTERY_STATE)
        check("vision_msgs_ObjectHypothesis", LcmFingerprints.VISION_MSGS_OBJECT_HYPOTHESIS)
        check(
            "vision_msgs_ObjectHypothesisWithPose",
            LcmFingerprints.VISION_MSGS_OBJECT_HYPOTHESIS_WITH_POSE,
        )
        check("vision_msgs_BoundingBox3D", LcmFingerprints.VISION_MSGS_BOUNDING_BOX_3D)
        check("vision_msgs_Detection3D", LcmFingerprints.VISION_MSGS_DETECTION_3D)
        check(
            "vision_msgs_Detection3DArray",
            LcmFingerprints.VISION_MSGS_DETECTION_3D_ARRAY,
        )
    }

    /** Each codec must carry the constant it claims, and the right type name. */
    @Test
    fun `every codec advertises its own fingerprint and package-qualified name`() {
        val codecs: List<LcmCodec<*>> = listOf(
            TimeCodec, HeaderCodec, StringCodec, PointCodec, Vector3Codec, QuaternionCodec,
            PoseCodec, PoseWithCovarianceCodec, TwistCodec, TwistStampedCodec,
            TwistWithCovarianceCodec,
            PoseStampedCodec, PointStampedCodec, OdometryCodec, PathCodec,
            NavSatStatusCodec, NavSatFixCodec, ImuCodec, BatteryStateCodec,
        )
        assertEquals(19, codecs.size)
        for (c in codecs) {
            assertTrue("${c.typeName} is not package-qualified", c.typeName.contains('.'))
            assertNotEquals("${c.typeName} has a zero fingerprint", 0L, c.fingerprint)
        }
        // The type name is half of a Zenoh key: `<prefix>/<channel>/<pkg>.<Type>`.
        assertEquals("nav_msgs.Odometry", OdometryCodec.typeName)
        assertEquals("geometry_msgs.PoseStamped", PoseStampedCodec.typeName)
        assertEquals("sensor_msgs.NavSatFix", NavSatFixCodec.typeName)
    }

    /**
     * `Point` and `Vector3` really do hash the same — identical fields, identical
     * names — so nothing on the wire can tell them apart. Pinned so that the day
     * one of them changes, this fails loudly rather than the equality quietly
     * becoming a coincidence nobody rechecked.
     */
    @Test
    fun `Point and Vector3 share a fingerprint by construction`() {
        assertEquals(LcmFingerprints.GEOMETRY_MSGS_POINT, LcmFingerprints.GEOMETRY_MSGS_VECTOR3)
        assertEquals(prefixOf("geometry_msgs_Point"), prefixOf("geometry_msgs_Vector3"))
        // And therefore a Vector3 payload decodes as a Point without complaint.
        val asPoint = PointCodec.decode(LcmFixtures.bytes("geometry_msgs_Vector3"))
        assertEquals(LcmPoint(-31.5, 0.125, 64.25), asPoint)
    }

    /** All the others must be distinct, or a mis-typed publish would go unnoticed. */
    @Test
    fun `no two unrelated types share a fingerprint`() {
        val byFingerprint = mapOf(
            LcmFingerprints.STD_MSGS_TIME to "std_msgs.Time",
            LcmFingerprints.BUILTIN_INTERFACES_TIME to "builtin_interfaces.Time",
            LcmFingerprints.STD_MSGS_HEADER to "std_msgs.Header",
            LcmFingerprints.STD_MSGS_STRING to "std_msgs.String",
            LcmFingerprints.GEOMETRY_MSGS_POINT to "geometry_msgs.Point",
            LcmFingerprints.GEOMETRY_MSGS_QUATERNION to "geometry_msgs.Quaternion",
            LcmFingerprints.GEOMETRY_MSGS_POSE to "geometry_msgs.Pose",
            LcmFingerprints.GEOMETRY_MSGS_POSE_WITH_COVARIANCE to "geometry_msgs.PoseWithCovariance",
            LcmFingerprints.GEOMETRY_MSGS_TWIST to "geometry_msgs.Twist",
            LcmFingerprints.GEOMETRY_MSGS_TWIST_STAMPED to "geometry_msgs.TwistStamped",
            LcmFingerprints.GEOMETRY_MSGS_TWIST_WITH_COVARIANCE to "geometry_msgs.TwistWithCovariance",
            LcmFingerprints.GEOMETRY_MSGS_POSE_STAMPED to "geometry_msgs.PoseStamped",
            LcmFingerprints.GEOMETRY_MSGS_POINT_STAMPED to "geometry_msgs.PointStamped",
            LcmFingerprints.NAV_MSGS_ODOMETRY to "nav_msgs.Odometry",
            LcmFingerprints.NAV_MSGS_PATH to "nav_msgs.Path",
            LcmFingerprints.SENSOR_MSGS_NAV_SAT_STATUS to "sensor_msgs.NavSatStatus",
            LcmFingerprints.SENSOR_MSGS_NAV_SAT_FIX to "sensor_msgs.NavSatFix",
            LcmFingerprints.SENSOR_MSGS_IMU to "sensor_msgs.Imu",
            LcmFingerprints.SENSOR_MSGS_BATTERY_STATE to "sensor_msgs.BatteryState",
            LcmFingerprints.VISION_MSGS_OBJECT_HYPOTHESIS to "vision_msgs.ObjectHypothesis",
            LcmFingerprints.VISION_MSGS_OBJECT_HYPOTHESIS_WITH_POSE to
                "vision_msgs.ObjectHypothesisWithPose",
            LcmFingerprints.VISION_MSGS_BOUNDING_BOX_3D to "vision_msgs.BoundingBox3D",
            LcmFingerprints.VISION_MSGS_DETECTION_3D to "vision_msgs.Detection3D",
            LcmFingerprints.VISION_MSGS_DETECTION_3D_ARRAY to "vision_msgs.Detection3DArray",
        )
        assertEquals("two of these collide", 24, byFingerprint.size)
    }

    /**
     * `BoundingBox3D` is a `Pose` and a `Vector3`; `Transform` is a `Vector3` and a `Quaternion`.
     * Neither may share a fingerprint with the `Pose` whose bytes it partly is.
     *
     * `BoundingBox3D`'s body is a `Pose` followed by three doubles, so a codec that reached for
     * `PoseCodec` and then wrote the size would produce a decodable message behind the wrong eight
     * bytes — the same failure [GEOMETRY_MSGS_TRANSFORM] exists to catch, one type further out.
     */
    @Test
    fun `BoundingBox3D does not share a fingerprint with the Pose it contains`() {
        assertNotEquals(
            LcmFingerprints.GEOMETRY_MSGS_POSE,
            LcmFingerprints.VISION_MSGS_BOUNDING_BOX_3D,
        )
        assertNotEquals(
            LcmFingerprints.GEOMETRY_MSGS_POSE_WITH_COVARIANCE,
            LcmFingerprints.VISION_MSGS_OBJECT_HYPOTHESIS_WITH_POSE,
        )
    }

    /**
     * A payload of the wrong type is refused rather than reinterpreted. Both
     * `PoseStamped` and `PointStamped` start with a header, so a mis-routed one
     * would decode into plausible nonsense if the fingerprint were not checked.
     */
    @Test
    fun `a PoseStamped payload is refused by the PointStamped codec`() {
        val e = assertThrows(LcmDecodeException::class.java) {
            PointStampedCodec.decode(LcmFixtures.bytes("geometry_msgs_PoseStamped"))
        }
        assertTrue(e.message!!.contains("geometry_msgs.PointStamped"))
        assertTrue(e.message!!.contains("6a82696458c279a0"))
    }
}
