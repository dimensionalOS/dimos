package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos

/**
 * [ZenohTelemetryEncoder] — `AircraftState` into the LCM messages of `docs/zenoh-topics.md`'s
 * telemetry half. Every conversion is asserted against a hand-computed value.
 *
 * Written to fail loudly for the four things this layer exists to get right:
 *
 *  - **an NED axis leaking onto an ENU bus** — a swapped X/Y flies east when told north, an
 *    unflipped Z descends when told to climb. Pinned on all three axes in both signs, and again
 *    end-to-end through `Odometry`
 *  - **a missing `cos(latitude)`** — exactly zero error at the equator, 21 % at this site.
 *    Pinned at 38°N by name
 *  - **a zero standing in for a missing reading** — 0,0 is a real place in the Atlantic and
 *    0,0,0 is the takeoff pad. Every message is checked to disappear when any field it needs is
 *    null or stale
 *  - **the two codec traps** — `BatteryState.percentage` is a 0..1 fraction, and
 *    `NavSatStatus.status` is a *signed* int8 whose "no fix" is −1, which a byte slip reads as a
 *    confident 255
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted in this suite, code
 * reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | ENU axes not swapped (x=north, y=east) | 3 |
 *  | ENU z not negated (down published as up) | 3 |
 *  | cos(latitude) dropped from the east metre | 6 |
 *  | quaternion yaw not turned from north-clockwise to east-counterclockwise | 5 |
 *  | quaternion pitch sign not flipped for FLU | 2 |
 *  | quaternion roll sign flipped | 2 |
 *  | battery percentage published as 0..100 | 1 |
 *  | no fix reported as a fix | 2 |
 *  | no fix encoded as 255 rather than −1 | **0 — see below** |
 *  | satellite floor removed from the fix claim | 1 |
 *  | position freshness dropped (a cached fix published as live) | 2 |
 *  | altitude freshness dropped from the local position | 2 |
 *  | attitude freshness dropped | 3 |
 *  | velocity freshness dropped (a stale reading republished as live) | 2 |
 *  | unknown position zero-filled in the local frame | 3 |
 *  | unknown position zero-filled in the global fix | 1 |
 *  | unknown altitude zero-filled in the local frame | 2 |
 *  | a takeoff origin recorded from a cached fix | 1 |
 *  | odom and base_link swapped on Odometry | 1 |
 *  | no gyro reported as a confident zero rate | 2 |
 *  | imu's not-reported covariance sentinel dropped | 1 |
 *  | battery percent range check dropped | 1 |
 *  | an unmeasured battery voltage zero-filled instead of NaN | 1 |
 *  | battery current sign inverted (the MAVLink habit) | 1 |
 *  | cell voltages averaged into an even split | 1 |
 *  | an absent gimbal pitch published as level | 1 |
 *  | a blank flight mode published as a mode | 1 |
 *
 * The 6 and the 5 are the two landmines the docs name — the missing cosine and the yaw
 * convention — and most of the geometry refuses to run past either.
 *
 * ## The liveness gate, mutation-checked 2026-07-27
 *
 * [Gate] splits the channels in two: `odom` requires a reading that *arrived*, the dense streams
 * repeat a held one for as long as the link is up. Both halves are load-bearing in opposite
 * directions, so both are broken here — counts across this suite plus `replay/`:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | link liveness dropped — a dead link keeps publishing held values | 4 |
 *  | freshness dropped from the FRESH gate — odom flies on a held reading | 4 |
 *  | pose regressed to a freshness gate (the 45.3 % coverage bug) | 4 |
 *  | gps_location regressed to a freshness gate | 6 |
 *  | imu regressed to a freshness gate | 3 |
 *  | odom's default gate weakened to HELD | 4 |
 *
 * The direction matters as much as the count. Weakening `odom` and strengthening `pose` are both
 * caught, which is the property that keeps someone from "fixing" one channel's coverage by
 * loosening the gate the flight controller reads.
 *
 * **The 255-for-−1 mutant is alive on purpose, because it cannot be written here.** Kotlin's
 * `Byte` has no 255: `255.toUByte().toByte()` *is* −1, so the mutation compiles to the correct
 * code and no test can tell them apart. The slip is only expressible one layer down, where the
 * field is written to the wire, and `LcmFixtureTest`'s *"NavSatStatus no fix is negative one"*
 * pins that byte against DiMOS's own Python output. What this suite can
 * check — and does — is that the value reaching the codec is negative rather than 255, so the two
 * layers together leave nowhere for it to hide.
 */
class ZenohTelemetryEncoderTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        /** The takeoff barometric altitude this session. Another session saw it 41.5 m away. */
        const val TAKEOFF_ALT = 100.0

        val DATUM = OdomDatum(
            latitudeDeg = LAT,
            longitudeDeg = LON,
            takeoffAltitudeM = TAKEOFF_ALT,
        )

        val STAMP = LcmTime(1_753_600_000, 250_000_000)

        /** Every continuous signal delivered a moment ago. */
        val FRESH: SampleAges = SampleAges.of(
            Signal.POSITION to 50L,
            Signal.ALTITUDE to 50L,
            Signal.ATTITUDE to 50L,
            Signal.VELOCITY to 50L,
            Signal.TAKEOFF_ALTITUDE to 50L,
            Signal.BATTERY_PERCENT to 50L,
            Signal.FLIGHT_MODE to 50L,
        )

        /** [FRESH] with one signal pushed past its own `staleAfterMs`. */
        fun stale(signal: Signal): SampleAges =
            SampleAges.of(FRESH.asMap() + (signal to (signal.staleAfterMs!! + 1)))

        fun latNorthOf(metres: Double): Double = LAT + metres / Geo.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))
    }

    /** A fully-populated, fully-fresh aircraft: 30 m east, 40 m north, 12 m up, moving. */
    private fun flying(): AircraftState = AircraftState(
        fcConnected = true,
        latitude = latNorthOf(40.0),
        longitude = lonEastOf(30.0),
        relativeAltitude = 12.0,
        takeoffAltitudeAmsl = TAKEOFF_ALT,
        rollDeg = 20.0,
        pitchDeg = -15.0,
        yawDeg = 118.0,
        velocityNorth = 3.0,
        velocityEast = 4.0,
        velocityDown = -1.5,
        satelliteCount = 14,
        gpsSignalLevel = 5,
        isFlying = true,
        flightMode = "GPS_ATTI",
        batteryPercent = 62,
        voltageMv = 8371,
        currentMa = -4200,
        cellCount = 2,
        cellVoltagesMv = listOf(4186, 4183),
        batteryTempC = 27.5,
        ages = FRESH,
    )

    // ── the geodesy: cos(latitude), pinned at 38°N ────────────────────────────

    @Test
    fun `local metres carry the cosine of latitude on east at 38 degrees north`() {
        // One degree of longitude at 38°N is METRES_PER_DEG * cos(38°) = 87 631 m, not
        // METRES_PER_DEG. Dropping the factor is a 21% error here and zero at the equator.
        val (east, north) = Geo.enuMetres(LAT, LON, LAT, LON + 1.0)
        assertEquals(Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)), east, 1e-6)
        assertEquals(0.0, north, 1e-9)
        assertTrue("cos(38) must shorten the east metre", east < Geo.METRES_PER_DEG * 0.79)
    }

    @Test
    fun `local metres are east then north, and north carries no cosine`() {
        val (east, north) = Geo.enuMetres(LAT, LON, LAT + 1.0, LON)
        assertEquals(0.0, east, 1e-9)
        assertEquals(Geo.METRES_PER_DEG, north, 1e-9)
    }

    @Test
    fun `local metres are signed the ENU way`() {
        val (west, south) = Geo.enuMetres(LAT, LON, latNorthOf(-40.0), lonEastOf(-30.0))
        assertEquals(-30.0, west, 1e-6)
        assertEquals(-40.0, south, 1e-6)
    }

    @Test
    fun `a longitude delta across the antimeridian is a short offset`() {
        val (east, _) = Geo.enuMetres(LAT, 179.999, LAT, -179.999)
        assertEquals(0.002 * Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)), east, 1e-6)
    }

    // ── the origin ────────────────────────────────────────────────────────────
    //
    // [OdomDatum] is no longer published as a message. It survives as the local frame's origin,
    // held in this process and consumed only by [ZenohTelemetryEncoder.localPositionOrNull]. The
    // four tests that pinned its wire form are deleted rather than adapted: there is no wire form
    // to pin. What remains is that the origin is captured correctly, which is the only property a
    // consumer can still be hurt by — every local metre on every channel is measured from it.

    @Test
    fun `a takeoff origin needs a valid, fresh fix`() {
        val s = flying()
        assertNotNull(OdomDatum.atTakeoff(s))
        assertNull("no position is no origin", OdomDatum.atTakeoff(s.copy(latitude = null)))
        assertNull(
            "a cached fix anchors the whole flight to where we last were",
            OdomDatum.atTakeoff(s.copy(ages = stale(Signal.POSITION))),
        )
    }

    @Test
    fun `a takeoff origin records the position and its barometric altitude`() {
        val d = OdomDatum.atTakeoff(flying())!!
        assertEquals(latNorthOf(40.0), d.latitudeDeg, 0.0)
        assertEquals(lonEastOf(30.0), d.longitudeDeg, 0.0)
        assertEquals(TAKEOFF_ALT, d.takeoffAltitudeM!!, 0.0)
    }

    // ── the NED → ENU boundary ────────────────────────────────────────────────

    @Test
    fun `east becomes x and north becomes y - the axes swap`() {
        val eastOnly = ZenohTelemetryEncoder.enuFromNed(north = 0.0, east = 7.0, down = 0.0)
        assertEquals(7.0, eastOnly.x, 0.0)
        assertEquals(0.0, eastOnly.y, 0.0)

        val northOnly = ZenohTelemetryEncoder.enuFromNed(north = 7.0, east = 0.0, down = 0.0)
        assertEquals(0.0, northOnly.x, 0.0)
        assertEquals(7.0, northOnly.y, 0.0)
    }

    @Test
    fun `down becomes minus z - climbing is positive up`() {
        // The sign that ends in the ground: NED down is negative while climbing.
        val climbing = ZenohTelemetryEncoder.enuFromNed(north = 0.0, east = 0.0, down = -2.5)
        assertEquals(2.5, climbing.z, 0.0)

        val descending = ZenohTelemetryEncoder.enuFromNed(north = 0.0, east = 0.0, down = 2.5)
        assertEquals(-2.5, descending.z, 0.0)
    }

    @Test
    fun `every axis keeps its sign through the swap`() {
        val v = ZenohTelemetryEncoder.enuFromNed(north = -3.0, east = -4.0, down = 5.0)
        assertEquals(-4.0, v.x, 0.0)
        assertEquals(-3.0, v.y, 0.0)
        assertEquals(-5.0, v.z, 0.0)
    }

    // ── the quaternion ────────────────────────────────────────────────────────

    @Test
    fun `the ENU quaternion matches the frame change computed independently`() {
        // Non-trivial on every axis: no zero, no symmetry, nothing a swapped pair would survive.
        val roll = 20.0
        val pitch = -15.0
        val yaw = 118.0

        // T_enu<-ned . Rz(yaw)Ry(pitch)Rx(roll) . T_frd<-flu, from the axis permutations alone.
        val nedToEnu = arrayOf(
            doubleArrayOf(0.0, 1.0, 0.0),
            doubleArrayOf(1.0, 0.0, 0.0),
            doubleArrayOf(0.0, 0.0, -1.0),
        )
        val fluToFrd = arrayOf(
            doubleArrayOf(1.0, 0.0, 0.0),
            doubleArrayOf(0.0, -1.0, 0.0),
            doubleArrayOf(0.0, 0.0, -1.0),
        )
        val nedFrd = mul(mul(rotZ(yaw), rotY(pitch)), rotX(roll))
        val expected = mul(mul(nedToEnu, nedFrd), fluToFrd)

        val actual = matrixOf(ZenohTelemetryEncoder.enuQuaternion(roll, pitch, yaw))
        for (r in 0..2) {
            for (c in 0..2) {
                assertEquals("R[$r][$c]", expected[r][c], actual[r][c], 1e-12)
            }
        }
    }

    @Test
    fun `a level aircraft pointing north is a quarter turn about up`() {
        // ENU yaw is counterclockwise from east, so north is +90 degrees about z.
        val q = ZenohTelemetryEncoder.enuQuaternion(0.0, 0.0, 0.0)
        assertEquals(0.0, q.x, 1e-12)
        assertEquals(0.0, q.y, 1e-12)
        assertEquals(Math.sin(Math.PI / 4), q.z, 1e-12)
        assertEquals(Math.cos(Math.PI / 4), q.w, 1e-12)
    }

    @Test
    fun `a level aircraft pointing east is the identity rotation`() {
        val q = ZenohTelemetryEncoder.enuQuaternion(0.0, 0.0, 90.0)
        assertEquals(0.0, q.x, 1e-12)
        assertEquals(0.0, q.y, 1e-12)
        assertEquals(0.0, q.z, 1e-12)
        assertEquals(1.0, q.w, 1e-12)
    }

    @Test
    fun `nose up in NED points the body x axis up in ENU`() {
        // Pitch +30 nose-up, heading north: forward is 0.866 north and 0.5 up.
        val forward = mul(matrixOf(ZenohTelemetryEncoder.enuQuaternion(0.0, 30.0, 0.0)), UNIT_X)
        assertEquals(0.0, forward[0], 1e-12)
        assertEquals(Math.cos(Math.toRadians(30.0)), forward[1], 1e-12)
        assertEquals(Math.sin(Math.toRadians(30.0)), forward[2], 1e-12)
    }

    @Test
    fun `right wing down in NED puts the left wing up in ENU`() {
        // Roll +30 right-wing-down, heading north: body y is *left* in FLU, so it goes up.
        val left = mul(matrixOf(ZenohTelemetryEncoder.enuQuaternion(30.0, 0.0, 0.0)), UNIT_Y)
        assertEquals(-Math.cos(Math.toRadians(30.0)), left[0], 1e-12)
        assertEquals(0.0, left[1], 1e-12)
        assertEquals(Math.sin(Math.toRadians(30.0)), left[2], 1e-12)
    }

    @Test
    fun `the quaternion is a unit quaternion`() {
        val q = ZenohTelemetryEncoder.enuQuaternion(20.0, -15.0, 118.0)
        val norm = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        assertEquals(1.0, norm, 1e-12)
    }

    // ── the local position ────────────────────────────────────────────────────

    @Test
    fun `local position is ENU metres from the datum, with altitude above takeoff`() {
        val p = ZenohTelemetryEncoder.localPositionOrNull(flying(), DATUM)!!
        assertEquals(30.0, p.x, 1e-6)
        assertEquals(40.0, p.y, 1e-6)
        assertEquals(12.0, p.z, 0.0)
    }

    @Test
    fun `local position is withheld when position, altitude or the datum is missing or stale`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.localPositionOrNull(s.copy(latitude = null), DATUM))
        assertNull(ZenohTelemetryEncoder.localPositionOrNull(s.copy(longitude = null), DATUM))
        assertNull(ZenohTelemetryEncoder.localPositionOrNull(s.copy(relativeAltitude = null), DATUM))
        assertNull(
            ZenohTelemetryEncoder.localPositionOrNull(s.copy(ages = stale(Signal.POSITION)), DATUM)
        )
        assertNull(
            ZenohTelemetryEncoder.localPositionOrNull(s.copy(ages = stale(Signal.ALTITUDE)), DATUM)
        )
        assertNull(
            ZenohTelemetryEncoder.localPositionOrNull(s, DATUM.copy(latitudeDeg = 4.5836623E7))
        )
    }

    // ── odom ──────────────────────────────────────────────────────────────────

    @Test
    fun `odometry is the world to base_link edge, pose and twist together`() {
        val o = ZenohTelemetryEncoder.odometryOrNull(flying(), DATUM, STAMP)!!
        assertEquals("drone/world", o.header.frameId)
        assertEquals("drone/base_link", o.childFrameId)
        assertEquals(STAMP, o.header.stamp)
        assertEquals(30.0, o.pose.pose.position.x, 1e-6)
        assertEquals(40.0, o.pose.pose.position.y, 1e-6)
        assertEquals(12.0, o.pose.pose.position.z, 0.0)
    }

    @Test
    fun `odometry twist is the velocity in ENU`() {
        // NED 3 north, 4 east, -1.5 down (climbing) -> ENU 4 east, 3 north, +1.5 up.
        val t = ZenohTelemetryEncoder.odometryOrNull(flying(), DATUM, STAMP)!!.twist.twist
        assertEquals(4.0, t.linear.x, 0.0)
        assertEquals(3.0, t.linear.y, 0.0)
        assertEquals(1.5, t.linear.z, 0.0)
        assertTrue("no gyro on this airframe - never a confident zero", t.angular.x.isNaN())
        assertTrue(t.angular.y.isNaN())
        assertTrue(t.angular.z.isNaN())
    }

    @Test
    fun `odometry is withheld when any of its four inputs is missing or stale`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.odometryOrNull(s.copy(latitude = null), DATUM, STAMP))
        assertNull(
            ZenohTelemetryEncoder.odometryOrNull(s.copy(relativeAltitude = null), DATUM, STAMP)
        )
        assertNull(ZenohTelemetryEncoder.odometryOrNull(s.copy(yawDeg = null), DATUM, STAMP))
        assertNull(ZenohTelemetryEncoder.odometryOrNull(s.copy(velocityEast = null), DATUM, STAMP))
        for (signal in listOf(Signal.POSITION, Signal.ALTITUDE, Signal.ATTITUDE, Signal.VELOCITY)) {
            assertNull(
                "a stale $signal must not be republished as a live one",
                ZenohTelemetryEncoder.odometryOrNull(s.copy(ages = stale(signal)), DATUM, STAMP),
            )
        }
    }

    @Test
    fun `the pose channel survives a velocity gap that stops odometry`() {
        val noVelocity = flying().copy(ages = stale(Signal.VELOCITY))
        assertNull(ZenohTelemetryEncoder.odometryOrNull(noVelocity, DATUM, STAMP))
        val p = ZenohTelemetryEncoder.poseStampedOrNull(noVelocity, DATUM, STAMP)!!
        assertEquals("drone/world", p.header.frameId)
        assertEquals(30.0, p.pose.position.x, 1e-6)
        assertEquals(
            ZenohTelemetryEncoder.enuQuaternion(20.0, -15.0, 118.0),
            p.pose.orientation,
        )
    }

    @Test
    fun `the pose channel is withheld without a position or an attitude`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.poseStampedOrNull(s.copy(latitude = null), DATUM, STAMP))
        assertNull(ZenohTelemetryEncoder.poseStampedOrNull(s.copy(rollDeg = null), DATUM, STAMP))
        assertNull(
            ZenohTelemetryEncoder.poseStampedOrNull(
                s.copy(relativeAltitude = null), DATUM, STAMP
            )
        )
    }

    /**
     * **A hovering aircraft publishes nothing, and that is not a fault.** Position, altitude and
     * attitude are all change-driven keys: hold still and all three fall silent together, which is
     * exactly why one of them cannot be used to vouch for another. Under a freshness gate this
     * measured **45.3 % pose coverage** on the 2026-07-27 tag flight — the aircraft was hovering
     * over the tag, which is when the pose matters most.
     */
    @Test
    fun `the pose channel holds a hover while the link is up, and stops when it drops`() {
        val hovering = flying().copy(
            ages = SampleAges.of(
                FRESH.asMap() + listOf(Signal.POSITION, Signal.ALTITUDE, Signal.ATTITUDE)
                    .associateWith { it.staleAfterMs!! + 1 }
            )
        )
        assertNotNull(ZenohTelemetryEncoder.poseStampedOrNull(hovering, DATUM, STAMP))
        assertNull(
            ZenohTelemetryEncoder.poseStampedOrNull(
                hovering.copy(fcConnected = false), DATUM, STAMP
            )
        )
    }

    // ── gps_location ──────────────────────────────────────────────────────────

    @Test
    fun `the global fix is latitude, longitude and height above takeoff`() {
        val fix = ZenohTelemetryEncoder.gpsLocationOrNull(flying(), STAMP)!!
        assertEquals(latNorthOf(40.0), fix.latitude, 0.0)
        assertEquals(lonEastOf(30.0), fix.longitude, 0.0)
        // Not AMSL, and not the 100 m barometric datum either.
        assertEquals(12.0, fix.altitude, 0.0)
        assertEquals("drone/base_link", fix.header.frameId)
    }

    @Test
    fun `no fix is minus one, the signed int8, never 255`() {
        val noFix = flying().copy(gpsSignalLevel = 0)
        val status = ZenohTelemetryEncoder.gpsLocationOrNull(noFix, STAMP)!!.status
        assertEquals("STATUS_NO_FIX is -1; read unsigned it is 255, a confident fix", -1, status.status.toInt())
        assertTrue("a byte slip shows up here", status.status < 0)
    }

    @Test
    fun `a fix is claimed only on DJI's own 3D thresholds`() {
        val s = flying()
        fun statusOf(state: AircraftState): Int =
            ZenohTelemetryEncoder.navSatStatus(state).status.toInt()

        assertEquals(0, statusOf(s))
        assertEquals("level unknown is no fix", -1, statusOf(s.copy(gpsSignalLevel = null)))
        assertEquals("a 2D-grade level is not a fix", -1, statusOf(s.copy(gpsSignalLevel = 2)))
        assertEquals(0, statusOf(s.copy(gpsSignalLevel = 3)))
        assertEquals(
            "a good level with collapsing satellites is not a fix",
            -1,
            statusOf(s.copy(satelliteCount = 5)),
        )
        assertEquals("an unknown count does not downgrade", 0, statusOf(s.copy(satelliteCount = null)))
        assertEquals(LcmNavSatStatus.SERVICE_GPS, ZenohTelemetryEncoder.navSatStatus(s).service)
    }

    @Test
    fun `the global fix is withheld without a position or an altitude`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.gpsLocationOrNull(s.copy(latitude = null), STAMP))
        assertNull(ZenohTelemetryEncoder.gpsLocationOrNull(s.copy(longitude = null), STAMP))
        assertNull(
            "0,0 is a real place in the Atlantic",
            ZenohTelemetryEncoder.gpsLocationOrNull(s.copy(latitude = 0.0, longitude = 0.0), STAMP),
        )
        assertNull(ZenohTelemetryEncoder.gpsLocationOrNull(s.copy(relativeAltitude = null), STAMP))
        assertNull(
            "a fix from a link that is gone is the last place we were, not where we are",
            ZenohTelemetryEncoder.gpsLocationOrNull(s.copy(fcConnected = false), STAMP),
        )
    }

    @Test
    fun `the global fix holds a stationary reading while the link is up`() {
        val parked = flying().copy(ages = stale(Signal.POSITION))
        assertNotNull(ZenohTelemetryEncoder.gpsLocationOrNull(parked, STAMP))
        assertNotNull(
            ZenohTelemetryEncoder.gpsLocationOrNull(
                flying().copy(ages = stale(Signal.ALTITUDE)), STAMP
            )
        )
    }

    // ── imu ───────────────────────────────────────────────────────────────────

    @Test
    fun `the imu carries the ENU orientation in base_link`() {
        val imu = ZenohTelemetryEncoder.imuOrNull(flying(), STAMP)!!
        assertEquals("drone/base_link", imu.header.frameId)
        assertEquals(ZenohTelemetryEncoder.enuQuaternion(20.0, -15.0, 118.0), imu.orientation)
        assertEquals(ZERO_COVARIANCE_9, imu.orientationCovariance)
    }

    @Test
    fun `the imu says it has no gyro and no accelerometer, twice over`() {
        val imu = ZenohTelemetryEncoder.imuOrNull(flying(), STAMP)!!
        assertTrue(imu.angularVelocity.x.isNaN())
        assertTrue(imu.linearAcceleration.z.isNaN())
        // ROS's own convention for "not reported", for a consumer that never looks at NaN.
        assertEquals(-1.0, imu.angularVelocityCovariance[0], 0.0)
        assertEquals(-1.0, imu.linearAccelerationCovariance[0], 0.0)
        assertEquals(9, imu.angularVelocityCovariance.size)
    }

    @Test
    fun `the imu is withheld when the attitude was never read`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.imuOrNull(s.copy(rollDeg = null), STAMP))
        assertNull(ZenohTelemetryEncoder.imuOrNull(s.copy(pitchDeg = null), STAMP))
        assertNull(
            "an identity quaternion is a confidently level aircraft",
            ZenohTelemetryEncoder.imuOrNull(s.copy(yawDeg = null), STAMP),
        )
    }

    @Test
    fun `the imu holds a still attitude while the link is up, and stops when it drops`() {
        val still = flying().copy(ages = stale(Signal.ATTITUDE))
        assertNotNull(
            "an aircraft holding an attitude publishes no ATTITUDE key — silence here is the " +
                "gimbal-lock trap, not a dead feed",
            ZenohTelemetryEncoder.imuOrNull(still, STAMP),
        )
        assertNull(
            "the link is the only independent witness; without it the held value is a fiction",
            ZenohTelemetryEncoder.imuOrNull(still.copy(fcConnected = false), STAMP),
        )
    }

    // ── battery ───────────────────────────────────────────────────────────────

    @Test
    fun `battery percentage is a fraction of one, not a percent`() {
        val b = ZenohTelemetryEncoder.batteryOrNull(flying(), STAMP)!!
        assertEquals("62 percent is 0.62, not 62.0", 0.62f, b.percentage, 1e-6f)
        assertTrue("a fraction can never exceed 1", b.percentage <= 1.0f)
    }

    @Test
    fun `battery units are volts, amps and celsius, and the current keeps DJI's sign`() {
        val b = ZenohTelemetryEncoder.batteryOrNull(flying(), STAMP)!!
        assertEquals(8.371f, b.voltage, 1e-6f)
        // Negative while discharging in both conventions — unlike MAVLink's cA, which inverts.
        assertEquals(-4.2f, b.current, 1e-6f)
        assertEquals(27.5f, b.temperature, 1e-6f)
        assertEquals(
            LcmBatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            b.powerSupplyStatus,
        )
        assertEquals(LcmBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO, b.powerSupplyTechnology)
        assertTrue(b.present)
    }

    @Test
    fun `cell voltages are per cell volts, never averaged`() {
        val b = ZenohTelemetryEncoder.batteryOrNull(flying(), STAMP)!!
        assertEquals(listOf(4.186f, 4.183f), b.cellVoltage)
        assertTrue("cell imbalance must stay visible", b.cellVoltage[0] != b.cellVoltage[1])
        assertEquals(emptyList<Float>(), b.cellTemperature)
    }

    @Test
    fun `an unmeasured battery field is NaN, per ROS's own convention`() {
        val bare = AircraftState(batteryPercent = 62, ages = FRESH)
        val b = ZenohTelemetryEncoder.batteryOrNull(bare, STAMP)!!
        assertTrue(b.voltage.isNaN())
        assertTrue(b.current.isNaN())
        assertTrue(b.temperature.isNaN())
        assertTrue(b.charge.isNaN())
        assertTrue(b.capacity.isNaN())
        assertTrue(b.designCapacity.isNaN())
        assertEquals(LcmBatteryState.POWER_SUPPLY_STATUS_UNKNOWN, b.powerSupplyStatus)
        assertEquals(emptyList<Float>(), b.cellVoltage)
    }

    @Test
    fun `the battery is withheld without a charge reading, and when it is not a percentage`() {
        val s = flying()
        assertNull(ZenohTelemetryEncoder.batteryOrNull(s.copy(batteryPercent = null), STAMP))
        assertNull(
            "clamping 150 to 100 invents a full battery",
            ZenohTelemetryEncoder.batteryOrNull(s.copy(batteryPercent = 150), STAMP),
        )
        assertNull(ZenohTelemetryEncoder.batteryOrNull(s.copy(batteryPercent = -1), STAMP))
        assertEquals(0f, ZenohTelemetryEncoder.batteryOrNull(s.copy(batteryPercent = 0), STAMP)!!.percentage, 0f)
    }

    @Test
    fun `the health channel owns the overheat, not this one`() {
        val hot = flying().copy(batteryTempC = 85.0)
        val b = ZenohTelemetryEncoder.batteryOrNull(hot, STAMP)!!
        assertEquals(LcmBatteryState.POWER_SUPPLY_HEALTH_UNKNOWN, b.powerSupplyHealth)
        assertEquals(85f, b.temperature, 0f)
    }

    // ── mode and gimbal ───────────────────────────────────────────────────────

    @Test
    fun `the flight mode is DJI's own word, verbatim`() {
        assertEquals("GPS_ATTI", ZenohTelemetryEncoder.flightModeOrNull(flying()))
        assertEquals(
            "JOYSTICK",
            ZenohTelemetryEncoder.flightModeOrNull(flying().copy(flightMode = "JOYSTICK")),
        )
    }

    @Test
    fun `no flight mode is no message, and a blank name is no name`() {
        assertNull(ZenohTelemetryEncoder.flightModeOrNull(flying().copy(flightMode = null)))
        assertNull(ZenohTelemetryEncoder.flightModeOrNull(flying().copy(flightMode = "  ")))
    }

    @Test
    fun `the gimbal carries pitch in x, in degrees`() {
        val v = ZenohTelemetryEncoder.gimbalAttitudeOrNull(-90.0, 0.0, 12.0)!!
        assertEquals(-90.0, v.x, 0.0)
        assertEquals(0.0, v.y, 0.0)
        assertEquals(12.0, v.z, 0.0)
    }

    @Test
    fun `an unreported gimbal roll or yaw is NaN, and no pitch is no message`() {
        val v = ZenohTelemetryEncoder.gimbalAttitudeOrNull(-30.0)!!
        assertEquals(-30.0, v.x, 0.0)
        assertTrue("0 degrees is a real angle", v.y.isNaN())
        assertTrue(v.z.isNaN())
        assertNull(ZenohTelemetryEncoder.gimbalAttitudeOrNull(null))
        assertNull(ZenohTelemetryEncoder.gimbalAttitudeOrNull(Double.NaN))
    }

    // ── hand-rolled linear algebra, so the quaternion is checked against something
    //    that shares no code with it ─────────────────────────────────────────────

    private fun rotX(deg: Double): Array<DoubleArray> {
        val c = cos(Math.toRadians(deg))
        val s = Math.sin(Math.toRadians(deg))
        return arrayOf(
            doubleArrayOf(1.0, 0.0, 0.0),
            doubleArrayOf(0.0, c, -s),
            doubleArrayOf(0.0, s, c),
        )
    }

    private fun rotY(deg: Double): Array<DoubleArray> {
        val c = cos(Math.toRadians(deg))
        val s = Math.sin(Math.toRadians(deg))
        return arrayOf(
            doubleArrayOf(c, 0.0, s),
            doubleArrayOf(0.0, 1.0, 0.0),
            doubleArrayOf(-s, 0.0, c),
        )
    }

    private fun rotZ(deg: Double): Array<DoubleArray> {
        val c = cos(Math.toRadians(deg))
        val s = Math.sin(Math.toRadians(deg))
        return arrayOf(
            doubleArrayOf(c, -s, 0.0),
            doubleArrayOf(s, c, 0.0),
            doubleArrayOf(0.0, 0.0, 1.0),
        )
    }

    private fun mul(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> =
        Array(3) { r -> DoubleArray(3) { c -> (0..2).sumOf { k -> a[r][k] * b[k][c] } } }

    private fun mul(a: Array<DoubleArray>, v: DoubleArray): DoubleArray =
        DoubleArray(3) { r -> (0..2).sumOf { k -> a[r][k] * v[k] } }

    /** The rotation matrix of a `(x, y, z, w)` quaternion, textbook form. */
    private fun matrixOf(q: LcmQuaternion): Array<DoubleArray> = arrayOf(
        doubleArrayOf(
            1 - 2 * (q.y * q.y + q.z * q.z),
            2 * (q.x * q.y - q.z * q.w),
            2 * (q.x * q.z + q.y * q.w),
        ),
        doubleArrayOf(
            2 * (q.x * q.y + q.z * q.w),
            1 - 2 * (q.x * q.x + q.z * q.z),
            2 * (q.y * q.z - q.x * q.w),
        ),
        doubleArrayOf(
            2 * (q.x * q.z - q.y * q.w),
            2 * (q.y * q.z + q.x * q.w),
            1 - 2 * (q.x * q.x + q.y * q.y),
        ),
    )
}

private val UNIT_X = doubleArrayOf(1.0, 0.0, 0.0)
private val UNIT_Y = doubleArrayOf(0.0, 1.0, 0.0)
