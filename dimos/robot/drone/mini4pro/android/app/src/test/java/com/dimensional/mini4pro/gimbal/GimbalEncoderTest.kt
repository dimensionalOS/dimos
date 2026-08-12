package com.dimensional.mini4pro.gimbal

import io.dronefleet.mavlink.common.GimbalDeviceFlags
import io.dronefleet.mavlink.common.GimbalManagerCapFlags
import io.dronefleet.mavlink.util.EnumValue
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2

/**
 * The three messages that make a gimbal exist in QGroundControl, and the arithmetic inside them.
 *
 * This suite exists because everything [GimbalEncoder] does is the kind of error that produces a
 * plausible wrong answer rather than a crash: a sign flip that points the camera at the sky, a
 * degrees-for-radians slip, a quaternion whose elements QGC reads in a different order, a frame
 * flag that puts the azimuth 180° out. The defence is to decode our own output with **QGC's own
 * algorithm**, transcribed here from the C library QGroundControl compiles against
 * (`ref/qgc-build-sys/_deps/mavlink-build/include/mavlink/mavlink_conversions.h`), rather than
 * with a nicer inverse of our own.
 *
 * Mutation-checked 2026-07-26 — each breakage was applied to the source, the whole gimbal suite
 * run, the failing tests counted, and the source restored. **Every one was caught.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `quaternionFromEulerDeg` takes degrees as radians (drop `Math.toRadians`) | 2 |
 *  | quaternion elements `x` and `y` transposed | 7 |
 *  | pitch sign inverted in `deviceAttitudeStatus` | 2 |
 *  | `YAW_IN_EARTH_FRAME` replaced with `YAW_IN_VEHICLE_FRAME` | 1 |
 *  | `GIMBAL_DEVICE_ID` set to 0 | 2 |
 *  | `HAS_YAW_AXIS` added to `CAP_FLAGS` | 1 |
 *  | `isYawLocked` matches `YAW_FOLLOW` instead of `FREE` | 1 |
 *  | unknown limits encoded as 0 instead of NaN | 1 |
 *  | `radiansOrNaN` returns degrees | 1 |
 *  | `withinAsinDomain` removed (the gimbal-lock NaN fix) | 2 |
 *  | `GimbalReading.isAdvertisable` stops checking the aircraft link | 3 |
 *  | `GimbalReading.isAdvertisable` stops checking `connected` | 2 |
 */
class GimbalEncoderTest {

    // ------------------------------------------------------------------- rig

    /** A gimbal that is present, level and freshly reported — the baseline for every test. */
    private val live = GimbalReading(
        pitchDeg = 0.0,
        rollDeg = 0.0,
        yawDeg = 0.0,
        workMode = "YAW_FOLLOW",
        connected = true,
        attitudeAgeMs = 0L,
        aircraftLinked = true,
    )

    /**
     * `mavlink_quaternion_to_euler`, transcribed: quaternion → DCM (`:39-58`) → euler
     * (`:68-92`), singular branches and all. **Degrees out**, because everything on our side of
     * the seam is degrees.
     *
     * The two `1.0e-3` gimbal-lock branches are copied rather than smoothed over. They are why
     * `pitch -90 loses the yaw` below is a test and not a bug.
     */
    private fun qgcEulerDeg(q: List<Float>): Triple<Double, Double, Double> {
        val a = q[0].toDouble()
        val b = q[1].toDouble()
        val c = q[2].toDouble()
        val d = q[3].toDouble()
        val aSq = a * a
        val bSq = b * b
        val cSq = c * c
        val dSq = d * d
        // `mavlink_quaternion_to_dcm` computes in double and stores into a `float dcm[3][3]`, so
        // every entry is narrowed before `mavlink_dcm_to_euler` ever sees it. Reproducing the
        // narrowing is not pedantry: it is the difference between an `asinf` argument of
        // 1.0000000000001 and one of exactly 1.0, i.e. between NaN and -90°.
        fun f(x: Double) = x.toFloat().toDouble()
        val dcm = arrayOf(
            doubleArrayOf(f(aSq + bSq - cSq - dSq), f(2 * (b * c - a * d)), f(2 * (a * c + b * d))),
            doubleArrayOf(f(2 * (b * c + a * d)), f(aSq - bSq + cSq - dSq), f(2 * (c * d - a * b))),
            doubleArrayOf(f(2 * (b * d - a * c)), f(2 * (a * b + c * d)), f(aSq - bSq - cSq + dSq)),
        )
        val theta = asin(-dcm[2][0])
        val phi: Double
        val psi: Double
        if (abs(theta - Math.PI / 2) < 1.0e-3) {
            phi = 0.0
            psi = atan2(dcm[1][2] - dcm[0][1], dcm[0][2] + dcm[1][1]) + phi
        } else if (abs(theta + Math.PI / 2) < 1.0e-3) {
            phi = 0.0
            psi = atan2(dcm[1][2] - dcm[0][1], dcm[0][2] + dcm[1][1] - phi)
        } else {
            phi = atan2(dcm[2][1], dcm[2][2])
            psi = atan2(dcm[1][0], dcm[0][0])
        }
        return Triple(Math.toDegrees(phi), Math.toDegrees(theta), Math.toDegrees(psi))
    }

    /**
     * Whether one `GIMBAL_DEVICE_FLAGS` bit is set in the attitude message built from [reading].
     *
     * `EnumValue.flagsEnabled` is a predicate rather than a getter in this library, so the flags
     * are interrogated one at a time — which is also how QGC reads them
     * (`GimbalController.cc:233-241` masks each bit individually).
     */
    private fun hasFlag(reading: GimbalReading, flag: GimbalDeviceFlags): Boolean =
        GimbalEncoder.deviceAttitudeStatus(reading, 0L)!!.flags().flagsEnabled(flag)

    private fun capBit(flag: GimbalManagerCapFlags): Int = EnumValue.of(flag).value()

    // ------------------------------------------------------- the quaternion round trip

    @Test
    fun `QGC recovers every attitude we encode`() {
        // A spread that covers all four sign combinations plus the two axes Ivan named, avoiding
        // only the ±90 pitch singularity which has its own test below.
        val cases = listOf(
            Triple(0.0, 0.0, 0.0),
            Triple(0.0, -30.0, 0.0),
            Triple(0.0, -89.0, 0.0),
            Triple(0.0, 25.0, 0.0),
            Triple(0.0, 0.0, 121.0),
            Triple(0.0, 0.0, -121.0),
            Triple(-12.0, -45.0, 90.0),
            Triple(7.5, 33.25, -179.0),
        )
        for ((roll, pitch, yaw) in cases) {
            val (r, p, y) = qgcEulerDeg(GimbalEncoder.quaternionFromEulerDeg(roll, pitch, yaw))
            assertEquals("roll of $roll/$pitch/$yaw", roll, r, 1e-3)
            assertEquals("pitch of $roll/$pitch/$yaw", pitch, p, 1e-3)
            assertEquals("yaw of $roll/$pitch/$yaw", yaw, y, 1e-3)
        }
    }

    @Test
    fun `the quaternion is w x y z and normalised`() {
        // Element order is the single most consequential convention in this file: MAVLink is
        // [w, x, y, z] Hamilton, and a [x, y, z, w] quaternion decodes to a completely different
        // and entirely plausible attitude. A pure pitch rotation puts the whole imaginary part in
        // element 2 and nowhere else, which pins the order without appealing to the decoder.
        val q = GimbalEncoder.quaternionFromEulerDeg(rollDeg = 0.0, pitchDeg = -90.0, yawDeg = 0.0)
        assertEquals(4, q.size)
        assertEquals("w", 0.70710678, q[0].toDouble(), 1e-6)
        assertEquals("x is zero for a pure pitch", 0.0, q[1].toDouble(), 1e-9)
        assertEquals("y carries the pitch, and it is negative for nose-down", -0.70710678, q[2].toDouble(), 1e-6)
        assertEquals("z is zero for a pure pitch", 0.0, q[3].toDouble(), 1e-9)
        // Unit to within float precision. Not exactly 1: the components are rounded to float,
        // and `withinAsinDomain` may shave a further ulp off all four to keep QGC's `asinf` in
        // domain — a scale change that QGC's roll and yaw (both `atan2` ratios) cannot see.
        val norm = q.sumOf { it.toDouble() * it.toDouble() }
        assertEquals(1.0, norm, 1e-6)
    }

    @Test
    fun `pitch straight down survives the round trip and does not produce NaN`() {
        // Ivan's "down", and QGC's Tilt-90 button. It sits exactly on `mavlink_dcm_to_euler`'s
        // singularity, where `asinf` is evaluated at the edge of its domain: a quaternion rounded
        // even slightly the wrong way makes -dcm[2][0] exceed 1 and QGC's pitch display becomes
        // NaN. Narrowing to Float after the trigonometry is what keeps it inside.
        val q = GimbalEncoder.quaternionFromEulerDeg(0.0, -90.0, 0.0)
        val a = q[0].toDouble()
        val b = q[1].toDouble()
        val c = q[2].toDouble()
        val d = q[3].toDouble()
        val sinArgument = -(2.0 * (b * d - a * c)).toFloat()
        assertTrue("asin argument $sinArgument must stay in [-1, 1]", abs(sinArgument) <= 1.0f)
        // 0.05° rather than 1e-3: `asin` is infinitely ill-conditioned at ±1, so a float
        // quaternion cannot carry a pitch of exactly -90 to better than a few hundredths of a
        // degree — and `withinAsinDomain` spends a little more of that budget to stay in domain.
        // QGC prints one decimal, so both round to the "-90.0" the operator reads.
        assertEquals(-90.0, qgcEulerDeg(q).second, 0.05)
    }

    @Test
    fun `no yaw at all makes QGC's asin overflow at the down position`() {
        // The defect this pins was found by this suite and is entirely real: at exactly ±90°
        // pitch, `dcm[2][0]` is ±1 in exact arithmetic, and float rounding pushed it to
        // ±1.0000001 for 744 of the 3601 yaw values below. `asinf` of that is NaN, which reaches
        // the operator as a NaN pitch readout *and* kills QGC's on-screen drag, which is
        // closed-loop on that same value. `GimbalEncoder.withinAsinDomain` is the fix.
        var worstError = 0.0
        var yawTenths = -1800
        while (yawTenths <= 1800) {
            val yaw = yawTenths / 10.0
            for (pitch in listOf(-90.0, 90.0)) {
                val recovered = qgcEulerDeg(GimbalEncoder.quaternionFromEulerDeg(0.0, pitch, yaw)).second
                assertFalse("pitch $pitch yaw $yaw decoded to NaN", recovered.isNaN())
                worstError = maxOf(worstError, abs(recovered - pitch))
            }
            yawTenths++
        }
        // Measured: 0.034°. Asserted loosely enough not to be brittle, tightly enough that
        // a heavier-handed fix would fail here.
        assertTrue("worst pitch error was $worstError deg", worstError < 0.05)
    }

    @Test
    fun `pitch straight down loses the yaw, and that is QGC's decoder rather than ours`() {
        // Pinned deliberately. At ±90° pitch `mavlink_dcm_to_euler` forces roll to 0 and computes
        // yaw from a degenerate expression (`mavlink_conversions.h:73-81`), so a camera looking at
        // its own feet reports an azimuth that is not the one we encoded. There is nothing to fix:
        // heading is genuinely undefined there. This test exists so that a future reader who
        // notices the round-trip failure does not "correct" it by rotating the frame, which would
        // break every other angle.
        val q = GimbalEncoder.quaternionFromEulerDeg(rollDeg = 0.0, pitchDeg = -90.0, yawDeg = 45.0)
        val (roll, pitch, yaw) = qgcEulerDeg(q)
        assertEquals("pitch is still right", -90.0, pitch, 0.05)
        assertEquals("roll is forced to zero by the singular branch", 0.0, roll, 1e-9)
        assertNotEquals("yaw does not survive", 45.0, yaw, 1e-3)
    }

    // --------------------------------------------------------------- advertisability

    @Test
    fun `nothing is advertised before DJI says where the camera points`() {
        val silent = GimbalReading()
        assertFalse(silent.isAdvertisable())
        assertNull(GimbalEncoder.managerInformation(silent, 0L))
        assertNull(GimbalEncoder.managerStatus(silent, 0L, 0, 0))
        assertNull(GimbalEncoder.deviceAttitudeStatus(silent, 0L))
    }

    @Test
    fun `a disconnected gimbal is not advertised even with a remembered angle`() {
        val gone = live.copy(connected = false)
        assertFalse(gone.isAdvertisable())
        assertNull(GimbalEncoder.deviceAttitudeStatus(gone, 0L))
    }

    @Test
    fun `an unasked connection does not block advertising`() {
        // null is "we have not asked", not "there is no gimbal". The attitude arriving is itself
        // evidence a gimbal exists, and refusing to advertise on a null would mean a working
        // camera stays invisible because one slow key never fired.
        assertTrue(live.copy(connected = null).isAdvertisable())
    }

    @Test
    fun `an old attitude is not a reason to stop advertising, a dead link is`() {
        // Reversed on 2026-07-26 against two hardware sessions. `KeyGimbalAttitude` only delivers
        // on change, so a minute-old angle is a camera holding still — the previous rule withdrew
        // the advertisement on exactly that and QGC lost the gimbal. Liveness comes from the
        // aircraft link now; see GimbalReading.isAdvertisable.
        assertTrue(live.copy(attitudeAgeMs = 600_000).isAdvertisable())
        assertNotNull(GimbalEncoder.deviceAttitudeStatus(live.copy(attitudeAgeMs = 600_000), 0L))

        assertFalse(live.copy(aircraftLinked = false).isAdvertisable())
        assertFalse("an unasked link is not a link", live.copy(aircraftLinked = null).isAdvertisable())
        assertNull(GimbalEncoder.deviceAttitudeStatus(live.copy(aircraftLinked = false), 0L))
    }

    @Test
    fun `an attitude that has never been delivered is absent, and absence is not advertisable`() {
        assertFalse(live.copy(pitchDeg = null).isAdvertisable())
    }

    // -------------------------------------------------------- GIMBAL_MANAGER_INFORMATION

    @Test
    fun `the device id is non-zero, or QGC discards the message outright`() {
        // GimbalController.cc:99-103 and :137-140 both `return` on gimbal_device_id == 0, with
        // nothing but a log line. A zero here is a gimbal that never appears and never explains.
        assertTrue(GimbalEncoder.GIMBAL_DEVICE_ID != 0)
        assertEquals(GimbalEncoder.GIMBAL_DEVICE_ID, GimbalEncoder.managerInformation(live, 0L)!!.gimbalDeviceId())
        assertEquals(GimbalEncoder.GIMBAL_DEVICE_ID, GimbalEncoder.managerStatus(live, 0L, 0, 0)!!.gimbalDeviceId())
    }

    @Test
    fun `the device id equals our component id, because 285 cannot carry one`() {
        // io.dronefleet.mavlink 1.1.11 predates gimbal_device_id on GIMBAL_DEVICE_ATTITUDE_STATUS,
        // so QGC reads 0 there and falls back to `pairId.deviceId = message.compid`
        // (GimbalController.cc:198-212). Everything we send leaves as compid 1
        // (MavlinkLink.COMPONENT_ID), so declaring anything else in 280/281 makes QGC drop every
        // attitude message and the gimbal never completes registration.
        assertEquals(1, GimbalEncoder.GIMBAL_DEVICE_ID)
    }

    @Test
    fun `we claim pitch and nothing we cannot perform`() {
        val claimed = GimbalEncoder.managerInformation(live, 0L)!!.capFlags()
        // Asserted as an exact bitmask, not as "these two are present": the whole point of the
        // honesty boundary is the bits that are *absent*, and a set-membership check would pass
        // just as happily with five more.
        val expected = capBit(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS) or
            capBit(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK)
        assertEquals(expected, claimed.value())
        // The two that would put controls in front of an operator that always fail. HAS_YAW_LOCK
        // shows QGC's Yaw Lock/Follow toggle and HAS_RETRACT shows a Retract button
        // (GimbalIndicator.qml:165-173, :202-210); this airframe can honour neither.
        assertFalse(claimed.flagsEnabled(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK))
        assertFalse(claimed.flagsEnabled(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT))
        // And the axes we cannot command, whatever the hardware physically contains.
        assertFalse(claimed.flagsEnabled(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS))
        assertFalse(claimed.flagsEnabled(GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS))
    }

    @Test
    fun `limits are DJI's own, in radians`() {
        val info = GimbalEncoder.managerInformation(
            live.copy(limits = GimbalLimits(pitchMinDeg = -90.0, pitchMaxDeg = 60.0)),
            0L,
        )!!
        assertEquals(-Math.PI / 2, info.pitchMin().toDouble(), 1e-6)
        assertEquals(Math.toRadians(60.0), info.pitchMax().toDouble(), 1e-6)
    }

    @Test
    fun `an unreported limit is NaN, never zero`() {
        // 0 is a real angle and a limit of 0 describes a gimbal that cannot move — the same
        // sentinel discipline TelemetryEncoder applies to altitudes.
        val info = GimbalEncoder.managerInformation(live, 0L)!!
        assertTrue(info.pitchMin().isNaN())
        assertTrue(info.pitchMax().isNaN())
        assertTrue(info.yawMin().isNaN())
        assertTrue(info.rollMax().isNaN())
    }

    // ------------------------------------------------------------ GIMBAL_MANAGER_STATUS

    @Test
    fun `nobody holds control until someone asks for it`() {
        val status = GimbalEncoder.managerStatus(live, 0L, 0, 0)!!
        assertEquals(0, status.primaryControlSysid())
        assertEquals(0, status.primaryControlCompid())
        // A non-zero controller that is not this QGC makes `othersHaveControl` true, and
        // `_tryGetGimbalControl` then refuses to send any operator command at all
        // (GimbalController.cc:179-180, :346-352) — the operator is locked out of their own
        // camera behind a popup. 0/0 is the only safe default.
    }

    @Test
    fun `whoever configured control is reported back`() {
        val status = GimbalEncoder.managerStatus(live, 0L, 255, 190)!!
        assertEquals(255, status.primaryControlSysid())
        assertEquals(190, status.primaryControlCompid())
    }

    // ---------------------------------------------------- GIMBAL_DEVICE_ATTITUDE_STATUS

    @Test
    fun `the reported attitude is DJI's, converted and not adjusted`() {
        val reading = live.copy(pitchDeg = -33.5, rollDeg = 1.25, yawDeg = -121.0)
        val (roll, pitch, yaw) = qgcEulerDeg(GimbalEncoder.deviceAttitudeStatus(reading, 0L)!!.q())
        assertEquals(-33.5, pitch, 1e-3)
        assertEquals(1.25, roll, 1e-3)
        assertEquals(-121.0, yaw, 1e-3)
    }

    @Test
    fun `yaw is declared earth-referenced, because that is what DJI reports`() {
        // DJI's own documentation for KeyGimbalAttitude: "The yaw angle uses the north east down
        // coordinate system." QGC branches on this flag to decide whether to add or subtract the
        // vehicle heading (GimbalController.cc:365-375), so the wrong bit is a silent azimuth
        // error of up to 180°.
        assertTrue(hasFlag(live, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME))
        assertFalse(hasFlag(live, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME))
    }

    @Test
    fun `roll and pitch are declared locked, which is what a stabilised gimbal does`() {
        assertTrue(hasFlag(live, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_ROLL_LOCK))
        assertTrue(hasFlag(live, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_PITCH_LOCK))
    }

    @Test
    fun `yaw lock reports DJI's actual mode, and nothing when DJI has not said`() {
        assertTrue(GimbalEncoder.isYawLocked("FREE"))
        assertFalse(GimbalEncoder.isYawLocked("YAW_FOLLOW"))
        assertFalse(GimbalEncoder.isYawLocked("FPV"))
        assertFalse("DJI's own UNKNOWN is not evidence of a lock", GimbalEncoder.isYawLocked("UNKNOWN"))
        assertFalse("a key that never arrived is not evidence either", GimbalEncoder.isYawLocked(null))

        assertTrue(hasFlag(live.copy(workMode = "FREE"), GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_LOCK))
        assertFalse(hasFlag(live.copy(workMode = null), GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_LOCK))
    }

    @Test
    fun `retract and neutral are never claimed`() {
        val locked = live.copy(workMode = "FREE")
        assertFalse(hasFlag(locked, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_RETRACT))
        assertFalse(hasFlag(locked, GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_NEUTRAL))
    }

    @Test
    fun `unmeasured quantities are NaN rather than zero`() {
        val status = GimbalEncoder.deviceAttitudeStatus(live, 0L)!!
        // DJI reports no gimbal rates at all. A zero here would claim a motionless gimbal.
        assertTrue(status.angularVelocityX().isNaN())
        assertTrue(status.angularVelocityY().isNaN())
        assertTrue(status.angularVelocityZ().isNaN())
        assertTrue(status.deltaYaw().isNaN())
    }

    @Test
    fun `the attitude message is broadcast rather than addressed`() {
        val status = GimbalEncoder.deviceAttitudeStatus(live, 0L)!!
        assertEquals(0, status.targetSystem())
        assertEquals(0, status.targetComponent())
    }

    @Test
    fun `time_boot_ms is carried through`() {
        assertEquals(12_345L, GimbalEncoder.deviceAttitudeStatus(live, 12_345L)!!.timeBootMs())
        assertEquals(12_345L, GimbalEncoder.managerStatus(live, 12_345L, 0, 0)!!.timeBootMs())
        assertEquals(12_345L, GimbalEncoder.managerInformation(live, 12_345L)!!.timeBootMs())
    }

    @Test
    fun `a missing roll or yaw does not stop the pitch being reported`() {
        // A quaternion has no sentinel for "this axis is unknown", so an absent roll or yaw is
        // encoded as level. That is the one place in this project where a null becomes a zero,
        // and it is confined to the two axes QGC's pitch readout does not use.
        val pitchOnly =
            GimbalReading(pitchDeg = -45.0, connected = true, attitudeAgeMs = 0L, aircraftLinked = true)
        val status = GimbalEncoder.deviceAttitudeStatus(pitchOnly, 0L)
        assertNotNull(status)
        assertEquals(-45.0, qgcEulerDeg(status!!.q()).second, 1e-3)
    }

    private fun assertNotEquals(message: String, unexpected: Double, actual: Double, delta: Double) {
        assertTrue(message, abs(unexpected - actual) > delta)
    }
}
