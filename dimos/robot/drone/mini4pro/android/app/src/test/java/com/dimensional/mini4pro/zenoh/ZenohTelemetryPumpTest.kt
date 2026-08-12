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
 * [ZenohTelemetryPump] — the two things a live publisher decides that an offline converter cannot:
 * **where the local origin is**, and **when a channel is due**.
 *
 * Everything else about a sample is [ZenohEmission]'s, which `replay/OrbitReplayTest` already
 * holds to account against 1855 real samples of a real flight. What is tested here is only what
 * the live path adds — plus the one property that has to be re-asserted at this level because it
 * is the whole reason the encoder has two gates: **publishing stops when the link goes down.**
 *
 * ## Why the origin rule gets this much attention
 *
 * Every local coordinate on this bus is *GPS now minus GPS at takeoff*. If the origin is wrong,
 * nothing errors: `pose` keeps arriving, `odom` keeps arriving, the numbers stay smooth and
 * plausible, and they are all offset by however far the aircraft was from its takeoff point when
 * the origin was taken. Measured across twenty sessions on the same patch of ground, session
 * origins separate by a median of **2.5 m** and a maximum of **13.4 m** — so this is not a
 * rounding error, it is the difference between landing on a target and landing beside it.
 *
 * The three cases below are the three ways it can be taken, and only one of them is right.
 *
 * ## Mutation-checked 2026-07-27
 *
 * One breakage at a time, whole suite run, reverted after each, by `tmp/mutate.py`. **Counts are
 * failing tests across the whole 1994-test suite, measured, not estimated.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the origin is taken from the first usable fix, ignoring `motorsOn` | 3 |
 *  | the origin may be taken from a **stale** fix (the freshness gate dropped) | 2 |
 *  | the origin is re-taken on every sample instead of latched | 1 |
 *  | a mid-flight origin is taken silently — reported as `MOTORS_ON` | 1 |
 *  | a mid-flight origin is refused outright (a restarted bridge publishes no pose) | 1 |
 *  | `reset` does not clear the origin — one flight's frame carried into the next | 1 |
 *  | the cadence ceiling is dropped — every channel publishes at the sample rate | 3 |
 *  | `battery` given the 5 Hz ceiling rather than 1 Hz | 1 |
 *  | `mode` published only on the heartbeat, never on change | 1 |
 *  | `mode` published only on change, never on the heartbeat | 2 |
 *  | a rate-limited channel reported as withheld rather than `RATE_LIMITED` | 2 |
 *  | the sink's refusal counted as a publish | 1 |
 *  | the stamp taken from the monotonic clock rather than the wall clock | 2 |
 *
 * And the two gates, broken in [ZenohTelemetryEncoder] and [ZenohEmission] themselves — so these
 * counts include the encoder's and the replay's own suites, which is the point of running them:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | link liveness dropped — a dead link keeps publishing held values | 4 |
 *  | `odom` weakened to the held gate | **9** |
 *  | `pose` regressed to a freshness gate (the 45.3 % coverage bug) | 5 |
 *  | [ZenohEmission]'s gates-vs-encoder agreement check removed | **0 — see below** |
 *
 * ### What the numbers say, and one place they say nothing
 *
 * **`odom` weakened to the held gate kills 9 — the largest number in either table.** That is the
 * right shape: `odom` is the only channel a controller closes a loop on, and loosening it is the
 * mutation that would let an aircraft be flown on a reading that never arrived. Nine assertions,
 * across three suites, refuse.
 *
 * **The cadence rows kill 1–3 each and that asymmetry is deliberate rather than a gap.**
 * Publishing too often is bandwidth; publishing from the wrong origin is a wrong answer that looks
 * right. The origin gets three tests and the cadence one apiece because that is the ratio of the
 * consequences.
 *
 * **The agreement check is an alive mutant, on purpose, and it cannot be killed from here.**
 * [ZenohEmission] throws when its gates say `PUBLISHED` and the encoder returns nothing, because
 * that disagreement means this whole layer's diagnosis is a fiction. No test kills the mutation
 * that removes it — and none can, because the two *are* consistent, which is exactly the property
 * the guard exists to notice being lost. It is the same kind of deliberately-alive mutant as
 * `ZenohTelemetryEncoderTest`'s 255-for-−1, and it is recorded rather than quietly dropped.
 */
class ZenohTelemetryPumpTest {

    private companion object {
        const val LAT = 38.0
        const val LON = 23.7
        const val PAD_LAT = 38.0001
        const val PAD_LON = 23.7001

        val FRESH: SampleAges = SampleAges.of(
            Signal.POSITION to 50L,
            Signal.ALTITUDE to 50L,
            Signal.ATTITUDE to 50L,
            Signal.VELOCITY to 50L,
            Signal.TAKEOFF_ALTITUDE to 50L,
            Signal.BATTERY_PERCENT to 50L,
            Signal.FLIGHT_MODE to 50L,
        )

        fun stale(vararg signals: Signal): SampleAges =
            SampleAges.of(FRESH.asMap() + signals.associateWith { it.staleAfterMs!! + 1 })

        fun latNorthOf(base: Double, metres: Double): Double = base + metres / Geo.METRES_PER_DEG

        fun lonEastOf(base: Double, metres: Double): Double =
            base + metres / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))
    }

    /** Everything present and fresh, on the pad, motors off. */
    private fun onPad(
        motorsOn: Boolean? = false,
        isFlying: Boolean? = false,
        fcConnected: Boolean = true,
        ages: SampleAges = FRESH,
        lat: Double = PAD_LAT,
        lon: Double = PAD_LON,
        alt: Double = 0.0,
        mode: String? = "GPS_ATTI",
    ) = AircraftState(
        fcConnected = fcConnected,
        latitude = lat,
        longitude = lon,
        relativeAltitude = alt,
        takeoffAltitudeAmsl = 100.0,
        rollDeg = 1.0,
        pitchDeg = -2.0,
        yawDeg = 118.0,
        velocityNorth = 3.0,
        velocityEast = 4.0,
        velocityDown = -1.5,
        satelliteCount = 14,
        gpsSignalLevel = 5,
        motorsOn = motorsOn,
        isFlying = isFlying,
        flightMode = mode,
        batteryPercent = 62,
        voltageMv = 8371,
        currentMa = -4200,
        cellCount = 2,
        cellVoltagesMv = listOf(4186, 4183),
        batteryTempC = 27.5,
        ages = ages,
    )

    private val sent = ArrayList<Pair<ZenohChannel, ByteArray>>()
    private val said = ArrayList<String>()

    /** The gimbal and the resolution the pump is told about, swappable mid-test. */
    private var gimbal: GimbalEarthAttitude? = null
    private var resolution: Pair<Int, Int>? = null

    private fun pump(
        cadence: ZenohTelemetryPump.Cadence = ZenohTelemetryPump.Cadence(),
        accept: Boolean = true,
    ) = ZenohTelemetryPump(
        sink = { ch, bytes -> sent += ch to bytes; accept },
        cadence = cadence,
        gimbal = { gimbal },
        resolution = { resolution },
        announce = { said += it },
    )

    /** Far enough apart in mono time that nothing is ever rate-limited by accident. */
    private var mono = 0L
    private var wall = 1_753_600_000_000L

    private fun ZenohTelemetryPump.step(s: AircraftState, dtMs: Long = 10_000): Int {
        mono += dtMs
        wall += dtMs
        return sample(s, mono, wall)
    }

    private fun channels() = sent.map { it.first }

    // ── 1. the origin ────────────────────────────────────────────────────────

    @Test
    fun `no origin means no local channels, and the global ones publish regardless`() {
        val p = pump()
        p.step(onPad())
        assertEquals(ZenohTelemetryPump.DatumOrigin.NONE, p.datumOrigin)
        assertNull(p.datum)
        // The two that need an origin say so by name, rather than publishing a zero.
        assertEquals(Withheld.NO_DATUM, p.lastReasons[ZenohChannel.POSE])
        assertEquals(Withheld.NO_DATUM, p.lastReasons[ZenohChannel.ODOM])
        // The four that do not, publish. A `NavSatFix` needs no local frame to mean something.
        assertEquals(
            listOf(
                ZenohChannel.GPS_LOCATION, ZenohChannel.IMU,
                ZenohChannel.BATTERY, ZenohChannel.MODE,
            ),
            channels(),
        )
    }

    @Test
    fun `the origin is taken at motor start, and it is that point`() {
        val p = pump()
        p.step(onPad())
        assertNull("still on the pad with the motors off", p.datum)

        p.step(onPad(motorsOn = true))
        assertEquals(ZenohTelemetryPump.DatumOrigin.MOTORS_ON, p.datumOrigin)
        val datum = p.datum!!
        assertEquals(PAD_LAT, datum.latitudeDeg, 1e-12)
        assertEquals(PAD_LON, datum.longitudeDeg, 1e-12)
        assertTrue(ZenohChannel.POSE in channels())
        assertTrue(ZenohChannel.ODOM in channels())
    }

    /**
     * An origin recorded from a cached fix anchors the whole flight's frame to wherever the
     * aircraft was the last time the GPS spoke, and every coordinate afterwards inherits that
     * error with no way to see it. **A datum we could not record is an absence**, and an absence
     * is publishable — a wrong origin is not.
     */
    @Test
    fun `a stale fix cannot become an origin, even with the motors running`() {
        val p = pump()
        p.step(onPad(motorsOn = true, ages = stale(Signal.POSITION)))
        assertNull(p.datum)
        assertEquals(ZenohTelemetryPump.DatumOrigin.NONE, p.datumOrigin)
        // And it is taken the moment a fresh fix arrives.
        p.step(onPad(motorsOn = true))
        assertNotNull(p.datum)
    }

    @Test
    fun `an origin taken in flight is announced and is not called the takeoff point`() {
        val p = pump()
        // The bridge comes up with the aircraft already airborne — motors reported off, or simply
        // never seen to turn on, but DJI says it is flying.
        p.step(onPad(motorsOn = false, isFlying = true, lat = latNorthOf(PAD_LAT, 60.0), alt = 30.0))
        assertEquals(ZenohTelemetryPump.DatumOrigin.MID_FLIGHT, p.datumOrigin)
        assertNotNull(p.datum)
        assertEquals(1, said.size)
        assertTrue(
            "the sentence must say the origin is not the takeoff point: ${said[0]}",
            said[0].contains("NOT the takeoff point"),
        )
        // Refusing outright would leave an airborne aircraft publishing no pose for the rest of
        // the battery, which is worse than an origin that says what it is.
        assertTrue(ZenohChannel.POSE in channels())
    }

    @Test
    fun `the origin never moves once taken, however far the aircraft goes`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        val first = p.datum!!
        repeat(5) { i ->
            p.step(
                onPad(
                    motorsOn = true,
                    isFlying = true,
                    lat = latNorthOf(PAD_LAT, 20.0 * (i + 1)),
                    lon = lonEastOf(PAD_LON, 15.0 * (i + 1)),
                    alt = 5.0 * (i + 1),
                ),
            )
        }
        assertEquals(first, p.datum)
        assertEquals(ZenohTelemetryPump.DatumOrigin.MOTORS_ON, p.datumOrigin)
        assertEquals("only the one sentence, and only for a mid-flight origin", 0, said.size)
    }

    @Test
    fun `reset forgets the flight, because a coordinate from another one is meaningless`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        assertNotNull(p.datum)
        p.reset()
        assertNull(p.datum)
        assertEquals(ZenohTelemetryPump.DatumOrigin.NONE, p.datumOrigin)
        // And the cadence is forgotten with it, so a new session does not start rate-limited
        // against the previous one's clock.
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 1)
        assertTrue(ZenohChannel.BATTERY in channels())
    }

    // ── 2. cadence ───────────────────────────────────────────────────────────

    @Test
    fun `the ceiling holds each channel to its own rate`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()

        // 200 ms later: the 5 Hz set is due, the 1 Hz set is not.
        p.step(onPad(motorsOn = true), dtMs = 200)
        assertEquals(
            listOf(
                ZenohChannel.POSE, ZenohChannel.ODOM,
                ZenohChannel.GPS_LOCATION, ZenohChannel.IMU,
                // `tf` is in the 5 Hz set and not the 1 Hz one, because two of its three edges
                // move continuously — including the camera edge, which is composed against a live
                // attitude and therefore changes even while the gimbal command does not.
                ZenohChannel.TF,
            ),
            channels(),
        )
        // `camera_info` is in neither set here: this pump has no resolution supplier, so the
        // intrinsics are RESOLUTION_UNKNOWN rather than rate-limited. Absence because nothing has
        // stated a geometry is a different fact from absence because the ceiling has not elapsed,
        // and the two must not be reported as one.
        assertEquals(
            Withheld.RESOLUTION_UNKNOWN,
            p.lastReasons[ZenohChannel.CAMERA_INFO],
        )
        assertEquals(Withheld.RATE_LIMITED, p.lastReasons[ZenohChannel.BATTERY])
        assertEquals(Withheld.RATE_LIMITED, p.lastReasons[ZenohChannel.MODE])

        // A further 800 ms: a full second since the last battery, so it is due again.
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 800)
        assertTrue(ZenohChannel.BATTERY in channels())
        assertTrue(ZenohChannel.MODE in channels())
    }

    @Test
    fun `a channel not yet due is rate-limited and not withheld`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        p.step(onPad(motorsOn = true), dtMs = 40)
        // Every one of them is inside its ceiling, and none of them is a *withholding* — the
        // difference the status screen and the coverage report are built on.
        for (ch in listOf(ZenohChannel.POSE, ZenohChannel.ODOM, ZenohChannel.BATTERY)) {
            assertEquals("$ch", Withheld.RATE_LIMITED, p.lastReasons[ch])
        }
    }

    /**
     * A mode transition is the most operationally interesting thing on this bus. Waiting up to a
     * second to say `GO_HOME` is a second of an operator not knowing the aircraft is leaving.
     */
    @Test
    fun `mode publishes on change as well as on its heartbeat`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()
        // Well inside the 1 s ceiling, but the mode moved.
        p.step(onPad(motorsOn = true, mode = "GO_HOME"), dtMs = 40)
        assertTrue("a mode change must not wait for the heartbeat", ZenohChannel.MODE in channels())

        // Unchanged and inside the ceiling: nothing.
        sent.clear()
        p.step(onPad(motorsOn = true, mode = "GO_HOME"), dtMs = 40)
        assertTrue(ZenohChannel.MODE !in channels())

        // Unchanged, past the ceiling: the heartbeat. A steady cruise never changes mode, and a
        // subscriber joining mid-flight would otherwise never learn what mode it is in.
        sent.clear()
        p.step(onPad(motorsOn = true, mode = "GO_HOME"), dtMs = 1_000)
        assertTrue(ZenohChannel.MODE in channels())
    }

    // ── 3. the link, and the two gates ───────────────────────────────────────

    /**
     * **The property that makes the dense channels honest.** `pose`, `gps_location` and `imu` may
     * repeat a held reading for as long as the link is up, because a change-driven key goes silent
     * exactly when the value is not changing — a hovering aircraft and a dead one look identical
     * in the data. Only `fcConnected`, which is not a data key, can tell them apart, and when it
     * says the link is down every held channel must stop.
     */
    @Test
    fun `a dead link stops every held channel and publishes nothing local`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()

        p.step(onPad(motorsOn = true, fcConnected = false))
        for (ch in listOf(ZenohChannel.POSE, ZenohChannel.GPS_LOCATION, ZenohChannel.IMU)) {
            assertEquals("$ch must stop with the link", Withheld.LINK_DOWN, p.lastReasons[ch])
        }
        assertTrue(ZenohChannel.POSE !in channels())
        assertTrue(ZenohChannel.GPS_LOCATION !in channels())
        assertTrue(ZenohChannel.IMU !in channels())
    }

    /**
     * `odom` is the only telemetry channel a controller closes a loop on, so every input must have
     * *arrived*. It must never be published from a held reading — the opposite direction from the
     * test above, and both are load-bearing.
     */
    @Test
    fun `odom stops on a stale reading while the dense channels carry on`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()

        p.step(onPad(motorsOn = true, ages = stale(Signal.ALTITUDE)))
        assertEquals(Withheld.ALTITUDE_STALE, p.lastReasons[ZenohChannel.ODOM])
        assertTrue(ZenohChannel.ODOM !in channels())
        // …and `pose` is unaffected, which is the whole 267 → 752 samples of the liveness gate.
        assertEquals(Withheld.PUBLISHED, p.lastReasons[ZenohChannel.POSE])
        assertTrue(ZenohChannel.POSE in channels())
    }

    @Test
    fun `a stale velocity stops odom and nothing else`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()
        p.step(onPad(motorsOn = true, ages = stale(Signal.VELOCITY)))
        assertEquals(Withheld.VELOCITY_STALE, p.lastReasons[ZenohChannel.ODOM])
        assertEquals(Withheld.PUBLISHED, p.lastReasons[ZenohChannel.POSE])
    }

    // ── 4. the sink, and the stamp ───────────────────────────────────────────

    @Test
    fun `a refused message is not counted as published`() {
        val p = pump(accept = false)
        val n = p.step(onPad(motorsOn = true))
        assertEquals("the sink dropped everything", 0, n)
        assertTrue("but it was offered", sent.isNotEmpty())
    }

    /**
     * D-5: `header.stamp` is the **reading's** time, in Unix seconds at millisecond resolution.
     * The monotonic clock is for cadence only — a stamp taken from it would put every message
     * near the epoch, which decodes fine and is nonsense.
     */
    @Test
    fun `the stamp is the wall clock, decoded back out of the bytes`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        val imu = sent.first { it.first == ZenohChannel.IMU }.second
        val decoded = ImuCodec.decode(imu)
        assertEquals(wall / 1000L, decoded.header.stamp.sec.toLong())
        assertEquals(((wall % 1000L) * 1_000_000L).toInt(), decoded.header.stamp.nsec)
        assertEquals(ZenohTelemetryEncoder.FRAME_BASE_LINK, decoded.header.frameId)
    }

    /**
     * The bytes on the bus are the encoder's bytes, unwrapped — D-4. Checked here rather than
     * assumed, because the whole point of the seam is that nothing between the encoder and the
     * transport may add an envelope.
     */
    @Test
    fun `the payload is exactly what the encoder produces and nothing around it`() {
        val p = pump()
        val state = onPad(motorsOn = true)
        p.step(state)
        val datum = p.datum!!
        val stamp = LcmTime.ofEpochSeconds(wall / 1000.0)
        val expected = PoseStampedCodec.encode(
            ZenohTelemetryEncoder.poseStampedOrNull(state, datum, stamp)!!,
        )
        val actual = sent.first { it.first == ZenohChannel.POSE }.second
        assertTrue("the payload must be the encoder's bytes", expected.contentEquals(actual))
    }

    // ── 5. tf and camera_info — the two the pump decides, not ZenohEmission ───

    /**
     * **Why these two are here and not in [ZenohEmission].**
     *
     * `emit` is a pure function of one `AircraftState` and one origin, and neither of these can be
     * answered from those: the camera edge needs the gimbal, which lives behind `gimbal/`'s seam,
     * and the intrinsics need the video stream's stated geometry, which arrives on MSDK's decode
     * thread. The boundary is stated in [ZenohEmission.NOT_DECIDED_HERE] and asserted here.
     */
    @Test
    fun `the event and frame driven channels are not decided by a state sample`() {
        val decided = ZenohEmission.emit(onPad(motorsOn = true), null, LcmTime.ZERO, encode = false)
            .map { it.channel }
        for (ch in ZenohEmission.NOT_DECIDED_HERE) {
            assertTrue("$ch must not be decided from an AircraftState", ch !in decided)
        }
        assertEquals(
            listOf(
                ZenohChannel.TF,
                ZenohChannel.CAMERA_INFO,
                ZenohChannel.VIDEO,
                // Driven by what the camera can see, not by what the aircraft reports. Nothing in
                // an `AircraftState` says whether a tag was in frame — and `tag_fix` rides the
                // same sightings.
                ZenohChannel.DETECTIONS,
                ZenohChannel.TAG_FIX,
                // Driven by what this bridge commanded, which is in no AircraftState at all.
                ZenohChannel.SETPOINT,
                // Driven by DJI delivering a change-driven key. Wind is deliberately in no
                // AircraftState, so a per-sample answer here is structurally impossible —
                // which is the channel's whole cadence contract: a sampler republishing a
                // held wind reading as a fresh measurement is the failure this line closes.
                ZenohChannel.WIND,
            ),
            ZenohEmission.NOT_DECIDED_HERE,
        )
    }

    /** The tree rides the 5 Hz set, and its bytes are the encoder's. */
    @Test
    fun `the frame tree is published at the state rate with the gimbal the pump was given`() {
        gimbal = GimbalEarthAttitude(0.0, -90.0, 118.0, GimbalEarthAttitude.Source.COMMANDED)
        val p = pump()
        val state = onPad(motorsOn = true)
        p.step(state)
        assertTrue(ZenohChannel.TF in channels())

        val stamp = LcmTime.ofEpochSeconds(wall / 1000.0)
        val expected = TfMessageCodec.encode(
            ZenohTelemetryEncoder.tfOrNull(state, p.datum!!, stamp, gimbal, Gate.HELD)!!,
        )
        val actual = sent.first { it.first == ZenohChannel.TF }.second
        assertTrue("the payload must be the encoder's bytes", expected.contentEquals(actual))
        // Three edges with a gimbal, two without — and the tree is still published either way.
        assertEquals(3, TfMessageCodec.decode(actual).transforms.size)
    }

    @Test
    fun `with no gimbal supplier the tree is still published, without the camera edge`() {
        gimbal = null
        val p = pump()
        p.step(onPad(motorsOn = true))
        val tree = TfMessageCodec.decode(sent.first { it.first == ZenohChannel.TF }.second)
        assertEquals(
            listOf(
                "drone/world" to "drone/base_link",
                "drone/camera" to "drone/camera_optical",
            ),
            tree.transforms.map { it.header.frameId to it.childFrameId },
        )
    }

    /** `tf` follows `pose`, not `odom`: a stale reading is held, a dead link withholds. */
    @Test
    fun `the tree is withheld for the link and not for a stale reading`() {
        val p = pump()
        p.step(onPad(motorsOn = true))
        sent.clear()
        p.step(onPad(motorsOn = true, ages = stale(Signal.ALTITUDE)))
        assertTrue("a stale altitude must not withhold the tree", ZenohChannel.TF in channels())

        sent.clear()
        p.step(onPad(motorsOn = true, fcConnected = false))
        assertTrue(ZenohChannel.TF !in channels())
        assertEquals(Withheld.LINK_DOWN, p.lastReasons[ZenohChannel.TF])
    }

    /**
     * **Before a frame has stated a geometry there is nothing to hold**, and the reason says so by
     * name rather than reporting a rate limit.
     *
     * The distinction is the one the whole status screen is built on: "nothing has told us the
     * resolution" and "the ceiling has not elapsed" are different problems, and only one of them
     * means a subscriber will never see the stream.
     */
    @Test
    fun `camera_info is absent until a frame states a resolution`() {
        resolution = null
        val p = pump()
        p.step(onPad(motorsOn = true))
        assertTrue(ZenohChannel.CAMERA_INFO !in channels())
        assertEquals(Withheld.RESOLUTION_UNKNOWN, p.lastReasons[ZenohChannel.CAMERA_INFO])

        resolution = 1920 to 1080
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 200)
        assertTrue(ZenohChannel.CAMERA_INFO in channels())
        val info = CameraInfoCodec.decode(
            sent.first { it.first == ZenohChannel.CAMERA_INFO }.second,
        )
        assertEquals(1920, info.width)
        assertEquals("drone/camera_optical", info.header.frameId)
    }

    /**
     * On change, and at least once a second — the same second rule `mode` has, for the sharper
     * version of the same reason.
     *
     * A resolution change invalidates the intrinsics, so waiting out the heartbeat would be up to
     * a second of a consumer solving a pose against the wrong focal length. Two matrices are never
     * averaged into one; the new one goes out immediately.
     */
    @Test
    fun `camera_info repeats on its heartbeat and republishes immediately on a change`() {
        resolution = 1920 to 1080
        val p = pump()
        p.step(onPad(motorsOn = true))
        assertTrue(ZenohChannel.CAMERA_INFO in channels())

        // Well inside the 1 s ceiling and unchanged: nothing.
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 200)
        assertTrue(ZenohChannel.CAMERA_INFO !in channels())
        assertEquals(Withheld.RATE_LIMITED, p.lastReasons[ZenohChannel.CAMERA_INFO])

        // A full second: the heartbeat, so a late joiner is blind for at most that long.
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 800)
        assertTrue(ZenohChannel.CAMERA_INFO in channels())

        // A change, well inside the ceiling: immediately, and with the new focal length.
        resolution = 960 to 540
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 40)
        assertTrue("a resolution change must not wait for the heartbeat", ZenohChannel.CAMERA_INFO in channels())
        val info = CameraInfoCodec.decode(
            sent.first { it.first == ZenohChannel.CAMERA_INFO }.second,
        )
        assertEquals(960, info.width)
        assertEquals(ZenohTelemetryEncoder.FOCAL_OVER_WIDTH * 960, info.k[0], 1e-9)
    }

    /**
     * The intrinsics are a property of the camera, not a reading off the aircraft, so they neither
     * go stale nor need a link. A subscriber that has lost the aircraft still wants to know what
     * the lens was.
     */
    @Test
    fun `camera_info survives a dead link and an absent origin`() {
        resolution = 1920 to 1080
        val p = pump()
        p.step(onPad(fcConnected = false))
        assertTrue(ZenohChannel.CAMERA_INFO in channels())
        assertEquals(Withheld.NO_DATUM, p.lastReasons[ZenohChannel.POSE])
        assertEquals(Withheld.PUBLISHED, p.lastReasons[ZenohChannel.CAMERA_INFO])
    }

    /** A new session must not start rate-limited against the last one's clock. */
    @Test
    fun `reset forgets the held resolution as well as the cadence`() {
        resolution = 1920 to 1080
        val p = pump()
        p.step(onPad(motorsOn = true))
        p.reset()
        sent.clear()
        p.step(onPad(motorsOn = true), dtMs = 1)
        assertTrue(ZenohChannel.CAMERA_INFO in channels())
    }

    /**
     * **`pose` and `tf` run at the measured rate of the position feed — 100 ms — and the
     * number is a measurement, not a preference.** Landing10 (2026-07-29): the `dji_state`
     * position age resets 2884 times in 291.9 s — a fresh position every 101 ms, 9.88 Hz.
     * The old 200 ms ceiling published every *second* position the aircraft delivered, and
     * at the measured 1–1.5 m/s flight speeds that was 20–30 cm of travel per published
     * sample — seen by Ivan as ~25 cm steps in mem2 replays. Faster than the feed would
     * re-publish held positions (the change-driven-key rule: silence means unchanged), so
     * 100 ms is not a floor to beat but the feed's own cadence, matched.
     *
     * Pinned as the exact constants because both directions of drift are wrong: 200 ms is
     * the measured stepping regression, and a made-up 50 ms would claim a feed rate nobody
     * measured. `tf` matches `pose` because the tree's translation IS the pose's position
     * and must not step coarser than it.
     */
    @Test
    fun `pose and tf cadence match the measured 10 Hz position feed`() {
        val c = ZenohTelemetryPump.Cadence()
        assertEquals(100L, c.poseMs)
        assertEquals(100L, c.tfMs)
        assertEquals(100L, c.intervalMs(ZenohChannel.POSE))
        assertEquals(100L, c.intervalMs(ZenohChannel.TF))
    }
}
