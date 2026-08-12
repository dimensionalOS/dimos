package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * What the codecs do at the edges, where the fixtures stop.
 *
 * The fixtures prove the happy path against DiMOS's own encoder. This file covers
 * the cases a byte-for-byte comparison cannot: values a well-behaved publisher
 * would never send, payloads that arrive damaged, and the sizes that must be
 * refused rather than truncated.
 *
 * A decoder here is reading bytes off a network, so every failure has to be a
 * refusal with a reason. Silence, a partial value, or a runtime exception from
 * deep inside an array index are all worse than a dropped message — a
 * half-decoded `cmd_vel` is a command.
 */
class LcmCodecTest {

    // ---- arrays -----------------------------------------------------------

    @Test
    fun `an empty Path round-trips and its bytes are the header alone`() {
        val path = LcmPath(LcmHeader(3, LcmTime(5, 6), "odom"), emptyList())
        val bytes = PathCodec.encode(path)
        // 8 fingerprint + 4 count + 4 seq + 8 stamp + (4 + 4 + 1) frame_id
        assertEquals(33, bytes.size)
        assertEquals(path, PathCodec.decode(bytes))
        assertTrue(PathCodec.decode(bytes).poses.isEmpty())
    }

    @Test
    fun `a Path grows by exactly one PoseStamped per pose`() {
        val header = LcmHeader(1, LcmTime(2, 3), "odom")
        fun path(n: Int) = LcmPath(header, List(n) { LcmPoseStamped(header, LcmPose()) })
        val one = PathCodec.encode(path(1)).size
        val two = PathCodec.encode(path(2)).size
        val five = PathCodec.encode(path(5)).size
        assertEquals(one - PathCodec.encode(path(0)).size, two - one)
        assertEquals(one + 4 * (two - one), five)
    }

    @Test
    fun `a Path of many poses keeps them in order and distinct`() {
        val header = LcmHeader(1, LcmTime(2, 3), "odom")
        val poses = List(17) {
            LcmPoseStamped(LcmHeader(it, LcmTime(it, -it), "odom"), LcmPose(LcmPoint(it * -1.5, it + 0.25, 0.0)))
        }
        val decoded = PathCodec.decode(PathCodec.encode(LcmPath(header, poses)))
        assertEquals(poses, decoded.poses)
        assertEquals(-24.0, decoded.poses[16].pose.position.x, 0.0)
    }

    @Test
    fun `a BatteryState with no cells still writes both zero counts`() {
        val bat = LcmBatteryState(header = LcmHeader(1, LcmTime(2, 3), "base_link"), voltage = 11.1f)
        val bytes = BatteryStateCodec.encode(bat)
        val r = LcmReader(bytes)
        r.expectFingerprint(LcmFingerprints.SENSOR_MSGS_BATTERY_STATE, "sensor_msgs.BatteryState")
        assertEquals(0, r.readInt())
        assertEquals(0, r.readInt())
        assertEquals(bat, BatteryStateCodec.decode(bytes))
    }

    @Test
    fun `the two BatteryState cell arrays may have different lengths`() {
        val bat = LcmBatteryState(
            cellVoltage = listOf(3.7f, 3.8f, 3.9f, 4.0f),
            cellTemperature = listOf(-5.5f),
        )
        val decoded = BatteryStateCodec.decode(BatteryStateCodec.encode(bat))
        assertEquals(4, decoded.cellVoltage.size)
        assertEquals(1, decoded.cellTemperature.size)
        assertEquals(-5.5f, decoded.cellTemperature[0], 0f)
        assertEquals(bat, decoded)
    }

    // ---- signs and magnitudes ---------------------------------------------

    @Test
    fun `negative doubles survive every level of nesting`() {
        val odom = LcmOdometry(
            header = LcmHeader(-1, LcmTime(-2, -3), "odom"),
            childFrameId = "base_link",
            pose = LcmPoseWithCovariance(
                LcmPose(LcmPoint(-1.5, -2.25, -0.0625), LcmQuaternion(-0.5, -0.5, -0.5, -0.5)),
                List(36) { -(it + 1) * 0.125 },
            ),
            twist = LcmTwistWithCovariance(
                LcmTwist(LcmVector3(-9.5, -0.125, -1e-9), LcmVector3(-3.25, -0.5, -180.0)),
                List(36) { -1.0 },
            ),
        )
        val decoded = OdometryCodec.decode(OdometryCodec.encode(odom))
        assertEquals(odom, decoded)
        assertEquals(-1e-9, decoded.twist.twist.linear.z, 0.0)
    }

    @Test
    fun `a negative float in BatteryState is not read as a large positive`() {
        // Discharging current is negative in ROS. Sign-extension slips show here.
        val bat = LcmBatteryState(current = -8.25f, temperature = -12.5f, percentage = 0.001f)
        val decoded = BatteryStateCodec.decode(BatteryStateCodec.encode(bat))
        assertEquals(-8.25f, decoded.current, 0f)
        assertEquals(-12.5f, decoded.temperature, 0f)
        assertEquals(0.001f, decoded.percentage, 0f)
    }

    @Test
    fun `a southern and western fix keeps both signs`() {
        val fix = LcmNavSatFix(latitude = -33.8688, longitude = -151.2093, altitude = -3.5)
        val decoded = NavSatFixCodec.decode(NavSatFixCodec.encode(fix))
        assertEquals(-33.8688, decoded.latitude, 0.0)
        assertEquals(-151.2093, decoded.longitude, 0.0)
        assertEquals(-3.5, decoded.altitude, 0.0)
    }

    @Test
    fun `NavSatStatus no-fix is minus one on the way back, not 255`() {
        val decoded = NavSatStatusCodec.decode(
            NavSatStatusCodec.encode(LcmNavSatStatus(LcmNavSatStatus.STATUS_NO_FIX, 15))
        )
        assertEquals(-1, decoded.status.toInt())
        assertEquals(15, decoded.service.toInt())
    }

    // ---- strings ----------------------------------------------------------

    @Test
    fun `a multi-byte frame_id round-trips inside a nested header`() {
        val text = "odom·µ·🛰"
        val ps = LcmPoseStamped(LcmHeader(1, LcmTime(2, 3), text), LcmPose())
        assertEquals(text, PoseStampedCodec.decode(PoseStampedCodec.encode(ps)).header.frameId)
    }

    @Test
    fun `an empty string field is legal everywhere it appears`() {
        val odom = LcmOdometry(header = LcmHeader(0, LcmTime.ZERO, ""), childFrameId = "")
        val decoded = OdometryCodec.decode(OdometryCodec.encode(odom))
        assertEquals("", decoded.header.frameId)
        assertEquals("", decoded.childFrameId)
    }

    @Test
    fun `a std_msgs String holding JSON is not parsed, only carried`() {
        val json = """{"id":"b17c","verb":"orbit","radius_m":20.0,"note":"20 m ± 0.5"}"""
        assertEquals(json, StringCodec.decode(StringCodec.encode(json)))
    }

    // ---- refusals ---------------------------------------------------------

    @Test
    fun `a truncated payload is refused, not half-decoded`() {
        val full = OdometryCodec.encode(LcmFixtures.odometry())
        val e = assertThrows(LcmDecodeException::class.java) {
            OdometryCodec.decode(full.copyOf(full.size - 1))
        }
        assertTrue(e.message!!.contains("truncated"))
    }

    @Test
    fun `trailing bytes after a complete message are refused`() {
        val full = TwistCodec.encode(LcmFixtures.twist())
        val e = assertThrows(LcmDecodeException::class.java) {
            TwistCodec.decode(full + byteArrayOf(0))
        }
        assertTrue(e.message!!.contains("trailing"))
    }

    @Test
    fun `a covariance of the wrong length is refused at encode time`() {
        val e = assertThrows(IllegalArgumentException::class.java) {
            PoseWithCovarianceCodec.encode(LcmPoseWithCovariance(LcmPose(), List(35) { 0.0 }))
        }
        assertTrue(e.message!!.contains("36"))
        assertThrows(IllegalArgumentException::class.java) {
            NavSatFixCodec.encode(LcmNavSatFix(positionCovariance = List(10) { 0.0 }))
        }
        assertThrows(IllegalArgumentException::class.java) {
            ImuCodec.encode(LcmImu(angularVelocityCovariance = emptyList()))
        }
    }

    @Test
    fun `a Path claiming a negative pose count is refused`() {
        val w = LcmWriter()
        w.writeFingerprint(LcmFingerprints.NAV_MSGS_PATH)
        w.writeInt(-1)
        HeaderCodec.encodeInto(w, LcmHeader(0, LcmTime.ZERO, "odom"))
        val e = assertThrows(LcmDecodeException::class.java) { PathCodec.decode(w.toByteArray()) }
        assertTrue(e.message!!.contains("negative poses_length"))
    }

    @Test
    fun `a Path claiming more poses than it carries is refused`() {
        val w = LcmWriter()
        w.writeFingerprint(LcmFingerprints.NAV_MSGS_PATH)
        w.writeInt(1000)
        HeaderCodec.encodeInto(w, LcmHeader(0, LcmTime.ZERO, "odom"))
        assertThrows(LcmDecodeException::class.java) { PathCodec.decode(w.toByteArray()) }
    }

    @Test
    fun `a BatteryState claiming a negative cell count is refused`() {
        val w = LcmWriter()
        w.writeFingerprint(LcmFingerprints.SENSOR_MSGS_BATTERY_STATE)
        w.writeInt(2)
        w.writeInt(-3)
        val e = assertThrows(LcmDecodeException::class.java) {
            BatteryStateCodec.decode(w.toByteArray())
        }
        assertTrue(e.message!!.contains("negative cell array length"))
    }

    @Test
    fun `an empty payload is refused by every codec`() {
        for (codec in listOf<LcmCodec<*>>(StringCodec, TwistCodec, PathCodec, NavSatFixCodec)) {
            assertThrows(LcmDecodeException::class.java) { codec.decode(ByteArray(0)) }
        }
    }

    // ---- LcmTime ----------------------------------------------------------

    @Test
    fun `LcmTime splits an epoch time into whole seconds and nanoseconds`() {
        val t = LcmTime.ofEpochSeconds(1753622400.25)
        assertEquals(1753622400, t.sec)
        assertEquals(250000000, t.nsec)
        assertEquals(t, TimeCodec.decode(TimeCodec.encode(t)))
    }

    @Test
    fun `LcmTime floors rather than truncates before the epoch`() {
        // -0.25 s is second -1 plus 750 ms, not second 0 minus 250 ms. Truncation
        // toward zero would produce a negative nsec, which is not representable.
        val t = LcmTime.ofEpochSeconds(-0.25)
        assertEquals(-1, t.sec)
        assertEquals(750000000, t.nsec)
    }
}
