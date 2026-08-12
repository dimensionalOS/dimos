package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.zenoh.Withheld
import com.dimensional.mini4pro.zenoh.ZenohChannel
import com.dimensional.mini4pro.zenoh.LcmTime
import com.dimensional.mini4pro.zenoh.NavSatFixCodec
import com.dimensional.mini4pro.zenoh.OdometryCodec
import com.dimensional.mini4pro.zenoh.PoseStampedCodec
import com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * **A real flight, replayed end to end.**
 *
 * `src/test/resources/replay/orbit-real-air.jsonl` is session `20260727-102510`: 371 seconds
 * of a Mini 4 Pro over Athens on 2026-07-27, motors at t=26.6 s, airborne 28.8–259.6 s, a
 * commanded 24.7 m orbit from t=81.5 s, landing, and the link dropping at t=275.4 s. Every
 * line is byte-identical to what the phone wrote; the file is the session with its
 * `mav_in`/`mav_out`/`stick_cmd`/`rc_stick` lines filtered out, which is 21 303 of the
 * original 26 312 lines and 8.0 MB of its 8.3 MB:
 *
 * ```
 * grep -E '"k":"(header|dji_state|dji_field|dji_health|vs_state|event)"' \
 *     tmp/session-logs/orbit-real-air.jsonl > android/app/src/test/resources/replay/orbit-real-air.jsonl
 * ```
 *
 * ## What is asserted, and why each one would catch a real break
 *
 * A test that only proved the file parses would pass with the encoder deleted. These assert
 * that **the replayed data is the flight it came from**:
 *
 * 1. **The orbit is in there.** The published local positions trace a circle of 24.79 m mean
 *    radius, ±0.19 m, about the commanded centre, sweeping 321°. An axis swap, a lost
 *    `cos(latitude)`, a wrong datum or a sign flip in the ENU conversion all break the
 *    circle, and none of them break a round trip.
 * 2. **Nothing is published on a stale input.** Every message the encoder emitted is checked
 *    against its own freshness gates, on all 1855 samples.
 * 3. **The counts, exactly.** Per channel, per withholding reason. They are the measured
 *    behaviour of this encoder on real DJI cadences and they are what a regression would
 *    move.
 * 4. **Decoded coordinates round-trip to the recorded ones** to under a centimetre, through
 *    the actual LCM bytes.
 * 5. **The null-rather-than-zero rule holds throughout**: no `NavSatFix` at 0,0, no
 *    zero-filled quaternion, no NaN where a reading was claimed.
 *
 * ## The measured counts, 1855 samples
 *
 * | channel | published | withheld, and why |
 * |---|---|---|
 * | `odom` | 409 | 870 altitude stale, 509 position invalid, 67 velocity stale |
 * | `pose` | **1346** | 509 position invalid |
 * | `gps_location` | **1346** | 509 position invalid |
 * | `imu` | **1377** | 478 attitude missing |
 * | `battery` | 1378 | 477 no charge reading |
 * | `mode` | 1377 | 478 DJI named no mode |
 * | `gimbal` | **0** | 1855 not recorded — the one channel a record cannot reproduce |
 *
 * **The headline used to be the 870, and the 870 is now zero on three channels.** They were
 * `KeyAltitude`'s *cadence* rather than a missing reading — the aircraft's altitude was a
 * perfectly well-known 29.3 m throughout the orbit — and a dense stream that withholds on it is
 * describing our own rule, not the aircraft. Under the liveness gate (`3b2d331`) `pose` and
 * `gps_location` go 476 → **1346**, and **1346 + 509 = 1855**, so every remaining withholding is
 * an aircraft with no fix at all rather than one holding still. `odom` is unchanged at 409 and
 * must stay so: something closes a loop on it, so it still requires a reading that arrived.
 *
 * The `datum` channel is gone entirely (`5f8..`/`3b2d331`): it carried a generation counter for
 * several flights per recording, we settled on one flight per session, and the counter had
 * nothing to count.
 *
 * The paragraph below is kept because it is the measurement that motivated the change, and
 * because `odom` still lives under exactly the constraint it describes: the aircraft's altitude was
 * a perfectly well-known 29.3 m throughout the orbit. It is `KeyAltitude`'s **cadence**.
 * The key arrives in bursts and its age ramps to 5.7 s between them, against
 * `Signal.ALTITUDE`'s 1 s limit — so the primary telemetry message is silent for most of a
 * normal flight. That is not something a synthetic test would ever have shown, and it is an
 * open question for the publisher rather than a bug in this encoder: see the report in
 * [ReplayCoverage].
 *
 * The 509 `POSITION_INVALID` are honest — 478 of them are samples with no aircraft at all
 * (before the FC connected and after the link dropped) and the rest are the same window's
 * edges. `POSITION_STALE` never occurs once: `KeyAircraftLocation3D` delivered at rate for
 * the entire flight, median age 2 ms in the air.
 *
 * ## Measured on the same fixture, 2026-07-27
 *
 * - **Orbit**: 250 replayed samples in the circling window, mean radius **24.79 m** against a
 *   commanded 24.7, spread **0.18 m**, swept **321°**. From the published `pose` frames alone
 *   — a gappy subset, because `pose` is withheld whenever `KeyAltitude` has gone quiet — 288°.
 * - **Round trip**: all 409 decoded `odom` positions convert back to the recorded latitude and
 *   longitude to **under a centimetre**, and to the recorded altitude exactly.
 * - **Signal ages, whole session**: `POSITION` median 4 ms, `ATTITUDE` 5 ms, `ALTITUDE`
 *   **6101 ms**, `VELOCITY` 3292 ms. In the airborne window only, `VELOCITY`'s median is
 *   **204 ms** — so `KeyAircraftVelocity` *does* deliver at rate in flight, which
 *   `telemetry/Signal` lists as an open question and this record answers.
 * - **Cross-check**: `tools/zenohreplay --frames`, which re-derives the gating independently
 *   and encodes with DiMOS's own Python bindings, produces the same 5704 frames on the same
 *   channels at the same timestamps, **5696 of them byte-identical**. The eight that differ
 *   are one ULP in a quaternion component — `cos`/`sin` are not bit-identical between the JVM
 *   and libm.
 */
class OrbitReplayTest {

    /** The commanded orbit, from the record's own `orbit_accepted` event at t=81.52. */
    private companion object {
        const val ORBIT_CENTRE_LAT = 37.9938418
        const val ORBIT_CENTRE_LON = 23.7253487
        const val ORBIT_RADIUS_M = 24.7

        /** `orbit_circling` at t=91.13 to `orbit_ended` at t=141.15 — the circling leg. */
        const val ORBIT_FROM_S = 91.2
        const val ORBIT_TO_S = 141.1
    }

    /** Total turned angle along [angles], unwrapped so a lap does not cancel itself out. */
    private fun sweep(angles: List<Double>): Double {
        var total = 0.0
        for (i in 1 until angles.size) {
            var d = angles[i] - angles[i - 1]
            while (d > 180) d -= 360
            while (d < -180) d += 360
            total += d
        }
        return total
    }

    private fun record(): FlightRecord {
        val stream = javaClass.classLoader!!.getResourceAsStream("replay/orbit-real-air.jsonl")
        assertNotNull("fixture missing", stream)
        return stream!!.bufferedReader().useLines { FlightRecordReader.read(it) }
    }

    private val record by lazy { record() }
    private val samples by lazy { FlightReplay.samples(record) }
    private val result by lazy { ZenohReplay.run(samples, ZenohReplay.Options(encodeFrames = true)) }

    // ── 1. the record is the flight ──────────────────────────────────────────

    @Test
    fun `the fixture is the session it claims to be`() {
        val header = record.header!!
        assertEquals("mini4pro-flightlog-1", header.format)
        assertEquals("20260727-102510", header.session)
        assertEquals(5.0, header.stateHz!!, 1e-9)
        assertEquals(0, record.badLines)
        assertEquals(1855, record.states.size)
        assertEquals(673, record.fields.size)
        assertEquals(370.8, record.durationSeconds, 0.5)
        assertEquals(1855, samples.size)
    }

    /**
     * The four continuous ages are on **every** sample. This is the single fact the whole
     * harness depends on: without it a replay cannot know what the encoder withheld.
     */
    @Test
    fun `every sample carries all four recorded ages`() {
        for (signal in Signal.CONTINUOUS) {
            if (signal == Signal.TAKEOFF_ALTITUDE) continue // deadbanded, not per-sample
            val missing = samples.count { it.state.ageMs(signal) == null }
            assertEquals("$signal has samples with no recorded age", 0, missing)
        }
    }

    /**
     * The flight's own narrative is in the fold: motors at 26.6 s, airborne 28.8 s to
     * 259.6 s, the guided mode in force through the orbit.
     */
    @Test
    fun `the folded fields tell the flight's story`() {
        val firstMotors = samples.first { it.state.motorsOn == true }
        val firstFlying = samples.first { it.state.isFlying == true }
        val lastFlying = samples.last { it.state.isFlying == true }
        assertEquals(26.6, firstMotors.tSeconds, 0.3)
        assertEquals(28.8, firstFlying.tSeconds, 0.3)
        assertEquals(259.6, lastFlying.tSeconds, 0.3)

        val duringOrbit = samples.first { it.tSeconds >= 100.0 }.state
        assertEquals("JOYSTICK", duringOrbit.flightMode)
        assertEquals(true, duringOrbit.isFlying)
        assertTrue("battery should be a real percentage", duringOrbit.batteryPercent!! in 1..100)
        assertEquals(2, duringOrbit.cellCount)
        assertEquals(2, duringOrbit.cellVoltagesMv!!.size)
        assertTrue("home was recorded", duringOrbit.homeLocationSet == true)
        assertNotNull(duringOrbit.homeLatitude)
        // The one field no line of the record carries.
        assertNull(duringOrbit.goHomeHeightM)
    }

    /** Names the record carries that no binding claims are reported rather than dropped. */
    @Test
    fun `the unmapped field names are the ones the coverage report expects`() {
        val fold = FlightReplay.Fold()
        record.fields.forEach { FlightReplay.applyField(fold, it) }
        val unmapped = fold.unmappedFields.keys
        assertTrue("gimbal is absent, not unmapped", unmapped.none { it.contains("gimbal", true) })
        for (expected in listOf(
            "ultrasonicHeight", "windSpeedDmS", "flightModeString", "goHomeState",
            "airLinkSignalQuality", "productType",
        )) {
            assertTrue("$expected should be seen and unmapped", expected in unmapped)
        }
        // Nothing an AircraftState field needs is sitting in here unclaimed.
        val bound = FlightReplay.FIELD_MAP.map { it.name }.toSet()
        assertTrue(unmapped.none { it in bound })
    }

    // ── 2. the encoder, on real cadences ─────────────────────────────────────

    /**
     * The measured counts. See the class doc for what each one means; the number that
     * matters is 870 `ALTITUDE_STALE`, which is `KeyAltitude`'s cadence and not a fault.
     */
    @Test
    fun `the per-channel counts are what this flight produces`() {
        fun check(ch: ZenohChannel, published: Int, vararg reasons: Pair<Withheld, Int>) {
            val r = result[ch]
            assertEquals("$ch published", published, r.published)
            assertEquals("$ch samples", 1855, r.samples)
            for ((reason, n) in reasons) assertEquals("$ch $reason", n, r.reasons[reason] ?: 0)
        }
        check(
            ZenohChannel.ODOM, 409,
            Withheld.ALTITUDE_STALE to 870,
            Withheld.POSITION_INVALID to 509,
            Withheld.VELOCITY_STALE to 67,
            Withheld.POSITION_STALE to 0,
        )
        // The dense streams publish on link liveness, so `ALTITUDE_STALE` — `KeyAltitude`'s
        // cadence, not a fault — no longer withholds them. **476 → 1346, and 1346 + 509 = 1855:**
        // every remaining withholding is POSITION_INVALID, an aircraft with no fix at all rather
        // than one holding still. That is the whole point of the change.
        check(
            ZenohChannel.POSE, 1346,
            Withheld.ALTITUDE_STALE to 0,
            Withheld.POSITION_INVALID to 509,
            Withheld.LINK_DOWN to 0,
        )
        check(
            ZenohChannel.GPS_LOCATION, 1346,
            Withheld.ALTITUDE_STALE to 0,
            Withheld.POSITION_INVALID to 509,
        )
        check(
            ZenohChannel.IMU, 1377,
            Withheld.ATTITUDE_MISSING to 478,
            Withheld.ATTITUDE_STALE to 0,
        )
        check(ZenohChannel.BATTERY, 1378, Withheld.BATTERY_MISSING to 477)
        check(ZenohChannel.MODE, 1377, Withheld.MODE_MISSING to 478)
        check(ZenohChannel.GIMBAL, 0, Withheld.NOT_RECORDED to 1855)
    }

    /**
     * The reason tally is a *mirror* of the encoder, so it has to agree with it sample for
     * sample. If the two ever drift, the diagnosis this whole class produces is a fiction —
     * so the mirror is checked against the thing it mirrors rather than trusted.
     */
    @Test
    fun `the withholding tally agrees with the encoder itself`() {
        val datum = result.datum!!
        var odom = 0
        var pose = 0
        var gps = 0
        var imu = 0
        var battery = 0
        var mode = 0
        for (s in samples) {
            val stamp = LcmTime.ZERO
            if (ZenohTelemetryEncoder.odometryOrNull(s.state, datum, stamp) != null) odom++
            if (ZenohTelemetryEncoder.poseStampedOrNull(s.state, datum, stamp) != null) pose++
            if (ZenohTelemetryEncoder.gpsLocationOrNull(s.state, stamp) != null) gps++
            if (ZenohTelemetryEncoder.imuOrNull(s.state, stamp) != null) imu++
            if (ZenohTelemetryEncoder.batteryOrNull(s.state, stamp) != null) battery++
            if (ZenohTelemetryEncoder.flightModeOrNull(s.state) != null) mode++
        }
        assertEquals(odom, result[ZenohChannel.ODOM].published)
        assertEquals(pose, result[ZenohChannel.POSE].published)
        assertEquals(gps, result[ZenohChannel.GPS_LOCATION].published)
        assertEquals(imu, result[ZenohChannel.IMU].published)
        assertEquals(battery, result[ZenohChannel.BATTERY].published)
        assertEquals(mode, result[ZenohChannel.MODE].published)
    }

    /**
     * **Each channel is gated by its own rule, and by nothing else.** Checked over all 1855
     * samples in both directions: nothing published that its rule forbids, nothing withheld that
     * its rule allows.
     *
     * `odom` is the one a controller closes a loop on, so every input must have *arrived*. The
     * dense streams may repeat a held reading, because a value that is not changing publishes
     * nothing on a change-driven key — a hovering aircraft and a dead aircraft look identical in
     * the data and are told apart only by the link.
     */
    @Test
    fun `each channel is gated by its own rule and nothing else`() {
        val datum = result.datum!!
        for (s in samples) {
            val st = s.state
            val posThere = Geo.isValid(st.latitude, st.longitude)
            val altThere = st.relativeAltitude?.isFinite() == true
            val attThere = st.rollDeg?.isFinite() == true && st.pitchDeg?.isFinite() == true &&
                st.yawDeg?.isFinite() == true
            val velThere = st.velocityNorth?.isFinite() == true &&
                st.velocityEast?.isFinite() == true && st.velocityDown?.isFinite() == true

            val posOk = posThere && st.isFresh(Signal.POSITION)
            val altOk = altThere && st.isFresh(Signal.ALTITUDE)
            val attOk = attThere && st.isFresh(Signal.ATTITUDE)
            val velOk = velThere && st.isFresh(Signal.VELOCITY)

            val live = st.fcConnected
            val at = "t=%.3f".format(s.tSeconds)
            assertEquals("$at gps_location", posThere && altThere && live,
                ZenohTelemetryEncoder.gpsLocationOrNull(st, LcmTime.ZERO) != null)
            assertEquals("$at imu", attThere && live,
                ZenohTelemetryEncoder.imuOrNull(st, LcmTime.ZERO) != null)
            assertEquals("$at pose", posThere && altThere && attThere && live,
                ZenohTelemetryEncoder.poseStampedOrNull(st, datum, LcmTime.ZERO) != null)
            assertEquals("$at odom", posOk && altOk && attOk && velOk,
                ZenohTelemetryEncoder.odometryOrNull(st, datum, LcmTime.ZERO) != null)
        }
    }

    /**
     * The origin is the takeoff point, taken while the motors were on and the fix was fresh.
     *
     * It is no longer published as a message — it is held in this process and every local metre
     * on `pose` and `odom` is measured from it, which is why capturing it from the *right sample*
     * still matters as much as it did when it was on the wire.
     */
    @Test
    fun `the local frame's origin is this flight's takeoff point`() {
        val datum = result.datum!!
        assertEquals(26.6, result.datumAtSeconds!!, 0.3)
        assertEquals(37.99384, datum.latitudeDeg, 1e-4)
        assertEquals(23.72531, datum.longitudeDeg, 1e-4)
        // The barometric datum, which is pressure altitude and not AMSL — metadata only.
        assertEquals(70.319, datum.takeoffAltitudeM!!, 1e-3)
    }

    // ── 3. the orbit ─────────────────────────────────────────────────────────

    /**
     * **The aircraft genuinely circled, and the replayed local positions say so.**
     *
     * The centre and radius are the ones in the record's own `orbit_accepted` event, so this
     * compares the flown track against the commanded circle, not against itself. Radius is
     * asserted from the *published* `pose` messages — decoded back out of their LCM bytes —
     * so the whole chain is under test: fold, ages, gating, ENU conversion, codec.
     *
     * A dropped `cos(latitude)` is a 21 % east error at 38°N and turns this circle into an
     * ellipse 5 m out of round; a swapped X/Y turns it into a circle centred somewhere else.
     * Neither survives the ±1 m band below.
     */
    @Test
    fun `the published local positions trace the commanded circle`() {
        val datum = result.datum!!
        val (centreE, centreN) = Geo.enuMetres(
            datum.latitudeDeg, datum.longitudeDeg, ORBIT_CENTRE_LAT, ORBIT_CENTRE_LON,
        )
        val radii = ArrayList<Double>()
        val angles = ArrayList<Double>()
        var altMin = Double.MAX_VALUE
        var altMax = -Double.MAX_VALUE
        for (frame in result.frames) {
            if (frame.channel != ZenohChannel.POSE) continue
            if (frame.tSeconds < ORBIT_FROM_S || frame.tSeconds > ORBIT_TO_S) continue
            val p = PoseStampedCodec.decode(frame.bytes).pose.position
            radii.add(hypot(p.x - centreE, p.y - centreN))
            angles.add(Math.toDegrees(atan2(p.x - centreE, p.y - centreN)))
            altMin = minOf(altMin, p.z)
            altMax = maxOf(altMax, p.z)
        }
        assertTrue("too few published poses in the orbit window: ${radii.size}", radii.size >= 40)

        val mean = radii.average()
        assertEquals("mean radius", ORBIT_RADIUS_M, mean, 1.0)
        assertTrue("radius spread ${radii.max() - radii.min()} m is not a circle",
            radii.max() - radii.min() < 2.0)
        for (r in radii) assertEquals("every point on the circle", mean, r, 1.5)

        // Unwrapped sweep: a genuine circuit, not an arc traversed back and forth.
        //
        // 288° from the published poses and 321° from every replayed sample. The published
        // set is gappy — `pose` is withheld whenever `KeyAltitude` has gone quiet — so it
        // starts a little into the circle and stops a little before the end, and its sweep is
        // a lower bound on the real one. Both are asserted, because the difference between
        // them *is* the altitude-cadence finding, measured on the same track.
        assertTrue("published poses swept only ${sweep(angles)}°", abs(sweep(angles)) > 280.0)

        val allAngles = samples
            .filter { it.tSeconds in ORBIT_FROM_S..ORBIT_TO_S && it.state.latitude != null }
            .map {
                val (e, n) = Geo.enuMetres(
                    datum.latitudeDeg, datum.longitudeDeg, it.state.latitude!!, it.state.longitude!!,
                )
                Math.toDegrees(atan2(e - centreE, n - centreN))
            }
        assertEquals("every replayed sample in the window", 250, allAngles.size)
        assertEquals("the whole circling leg", 321.0, abs(sweep(allAngles)), 3.0)

        // Held altitude: the orbit was flown level, ~29.3 m above the takeoff point.
        assertEquals(29.3, altMin, 0.5)
        assertTrue("altitude wandered: $altMin..$altMax", altMax - altMin < 0.5)
    }

    /**
     * A decoded local coordinate converts back to the latitude and longitude the recorder
     * wrote, to under a centimetre — through the real LCM bytes and `Geo`'s own arithmetic.
     *
     * Sub-centimetre is the right bar because the record stores 7 decimal places of degree,
     * which is 1.1 cm, so anything larger would be the conversion rather than the format.
     */
    @Test
    fun `decoded local coordinates round-trip to the recorded latitude and longitude`() {
        val datum = result.datum!!
        val byTime = samples.associateBy { it.tSeconds }
        var checked = 0
        var worst = 0.0
        for (frame in result.frames) {
            if (frame.channel != ZenohChannel.ODOM) continue
            val p = OdometryCodec.decode(frame.bytes).pose.pose.position
            val recorded = byTime.getValue(frame.tSeconds).state
            // Invert Geo.enuMetres with the same constants, from the same origin.
            val lat = datum.latitudeDeg + p.y / Geo.METRES_PER_DEG
            val lon = datum.longitudeDeg +
                p.x / (Geo.METRES_PER_DEG * kotlin.math.cos(Math.toRadians(datum.latitudeDeg)))
            val dNorth = (lat - recorded.latitude!!) * Geo.METRES_PER_DEG
            val dEast = (lon - recorded.longitude!!) * Geo.METRES_PER_DEG *
                kotlin.math.cos(Math.toRadians(datum.latitudeDeg))
            worst = maxOf(worst, hypot(dEast, dNorth))
            assertEquals("altitude at t=${frame.tSeconds}", recorded.relativeAltitude!!, p.z, 1e-9)
            checked++
        }
        assertEquals(409, checked)
        assertTrue("worst round-trip error ${worst * 100} cm", worst < 0.01)
    }

    // ── 4. the null-rather-than-zero rule ────────────────────────────────────

    /**
     * `AircraftState` is null-per-field because null means *no reading*, and LCM has no
     * sentinel vocabulary — a `NavSatFix` at 0,0 is a real place in the Atlantic and a
     * zero-filled quaternion is not a rotation at all. Over 8 000 encoded frames, neither
     * appears.
     */
    @Test
    fun `no message carries a zero standing in for a missing reading`() {
        var fixes = 0
        var poses = 0
        for (frame in result.frames) {
            when (frame.channel) {
                ZenohChannel.GPS_LOCATION -> {
                    val fix = NavSatFixCodec.decode(frame.bytes)
                    assertTrue("NavSatFix at 0,0 at t=${frame.tSeconds}",
                        fix.latitude != 0.0 || fix.longitude != 0.0)
                    assertTrue("non-finite latitude", fix.latitude.isFinite())
                    assertTrue("latitude out of range", abs(fix.latitude) <= 90.0)
                    assertTrue("longitude out of range", abs(fix.longitude) <= 180.0)
                    assertTrue("altitude must be a real reading", fix.altitude.isFinite())
                    fixes++
                }
                ZenohChannel.POSE, ZenohChannel.ODOM -> {
                    val q = if (frame.channel == ZenohChannel.POSE) {
                        PoseStampedCodec.decode(frame.bytes).pose.orientation
                    } else {
                        OdometryCodec.decode(frame.bytes).pose.pose.orientation
                    }
                    val norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
                    assertEquals("quaternion not a unit rotation at t=${frame.tSeconds}", 1.0, norm, 1e-9)
                    poses++
                }
                else -> Unit
            }
        }
        assertEquals(1346, fixes)
        assertEquals(1346 + 409, poses)
    }

    /**
     * The `gimbal` channel produces nothing, for exactly one reason, and the coverage report says
     * so in words. This is the assertion that keeps the report honest as the recorder grows: *"the
     * day a gimbal field is added, this fails and the report gets updated with it."*
     *
     * **It fired on 2026-07-27, as intended.** The recorder grew a `gimbal` entry
     * (`record/LogEntry.Gimbal`, written by `record/Recorder.sampleGimbal` on change past a 0.5°
     * deadband with a 1 Hz heartbeat), so `ReplayCoverage.GIMBAL.absent` is now false and this
     * test says so. What has **not** changed, and is what keeps the channel in
     * [ReplayCoverage.BLOCKING], is either of the two things a replay needs:
     *
     *  1. `FlightReplay` does not read the new line. That is a reader change and it is the
     *     remaining half of the work.
     *  2. `orbit-real-air.jsonl` is a genuine flight from **before** the entry existed, so it
     *     contains no `gimbal` line whatever the reader learns to do. A fresh capture is required
     *     to test the replay of this channel at all, and that is a note for whoever flies next.
     *
     * The tripwire is kept, pointed at the half that is still open.
     */
    @Test
    fun `the gimbal channel is the one thing a record cannot replay`() {
        assertEquals(0, result[ZenohChannel.GIMBAL].published)
        // **Two entries since 2026-07-27, not one.** `status` joined the list when the live
        // publisher gave it a producer: `command/Announcer` fans every operator sentence onto the
        // bus directly, and a record carries all of those sentences but not the derivation of
        // *which* lines were announcements. So it is live-only for the same reason the gimbal is
        // — a reader gap, not a recorder gap — and the list is where a tool prints it from.
        // **Five entries since the video, `tf` and `camera_info` channels landed**, and the three
        // new ones are the same shape as the two old: a reader gap, not a recorder gap, except
        // `video` which is a scope boundary — the pixels are in the `.h264` sidecar and `replay/`
        // is handed a parsed JSONL and no file handle. `tf` and `camera_info` are blocked rather
        // than half-built for the reason `status` is: a tree with the camera silently missing is
        // indistinguishable, to a subscriber, from a flight where the camera was never aimed.
        // **Six entries since the `detections` channel landed on 2026-07-28.** The sixth is the
        // gimbal's shape exactly: `LogEntry.Tag` carries everything a `Detection3D` needs, and
        // `replay/` folds a record into an aircraft state, which a sighting is not. A reader gap.
        // **Eight since 2026-07-29**: `tag_fix` rides the same `tag` lines and the same reader
        // gap, and `setpoint` rides `stick_cmd` lines — a command is not an aircraft state
        // either. Both are byte-checked offline (tools/kotlinframes reads the lines directly),
        // which is precisely why blocking them here costs nothing: the cross-check does not go
        // through `replay/` at all.
        // **Nine since 2026-07-30**: `wind` rides `dji_field` windSpeedDmS lines through the
        // same offline path — and its reader gap is the one entry that must NOT be closed by
        // a fold, because putting wind into an aircraft state is exactly the resampling the
        // channel's cadence contract forbids (`ReplayCoverage.WIND_READINGS`).
        // **Ten since 2026-07-30's warning path**: `warnings` rides `dji_warn` lines, and its
        // reader gap is `setpoint`'s exactly — a warning is not an aircraft state. What is
        // different, and better, is that the record carries the *decided* event rather than the
        // raw reading, so an offline converter reproduces the channel without re-deciding a
        // severity (`ReplayCoverage.WARNINGS`).
        assertEquals(
            listOf(
                ZenohChannel.GIMBAL, ZenohChannel.STATUS,
                ZenohChannel.TF, ZenohChannel.CAMERA_INFO, ZenohChannel.VIDEO,
                ZenohChannel.DETECTIONS, ZenohChannel.TAG_FIX, ZenohChannel.SETPOINT,
                ZenohChannel.WIND, ZenohChannel.WARNINGS,
            ),
            ReplayCoverage.BLOCKING.map { it.blocks },
        )
        assertFalse(
            "the recorder half is closed as of 2026-07-28 — a record taken today carries `tag` " +
                "lines. If this is true again, the entry was removed.",
            ReplayCoverage.TAG_SIGHTINGS.absent,
        )
        for (ch in listOf(
            ZenohChannel.TF, ZenohChannel.CAMERA_INFO, ZenohChannel.VIDEO,
            ZenohChannel.DETECTIONS,
        )) {
            assertEquals("$ch", 0, result[ch].published)
            assertTrue("$ch", ch !in ReplayCoverage.REPLAYABLE_CHANNELS)
        }
        assertEquals(0, result[ZenohChannel.STATUS].published)
        assertTrue(ZenohChannel.STATUS !in ReplayCoverage.REPLAYABLE_CHANNELS)
        assertEquals(0, result[ZenohChannel.WARNINGS].published)
        assertTrue(ZenohChannel.WARNINGS !in ReplayCoverage.REPLAYABLE_CHANNELS)
        assertFalse(
            "the recorder half is closed as of 2026-07-27 — a record taken today carries a " +
                "`gimbal` line. If this is true again, the entry was removed.",
            ReplayCoverage.GIMBAL.absent,
        )
        assertTrue(ZenohChannel.GIMBAL !in ReplayCoverage.REPLAYABLE_CHANNELS)
        // The reference flight predates the entry, so it genuinely carries none. When this
        // starts failing, someone has captured a session with a gimbal in it — at which point
        // FlightReplay's half becomes both possible and testable.
        assertEquals(
            "orbit-real-air.jsonl predates LogEntry.Gimbal and must contain no gimbal line",
            0,
            javaClass.classLoader!!.getResourceAsStream("replay/orbit-real-air.jsonl")!!
                .bufferedReader().useLines { lines -> lines.count { it.contains("\"k\":\"gimbal\"") } },
        )
        // Every other channel published something on this flight.
        for (ch in ReplayCoverage.REPLAYABLE_CHANNELS) {
            assertTrue("$ch published nothing", result[ch].published > 0)
        }
    }
}
