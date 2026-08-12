package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.cos

/**
 * **The frame tree and the camera intrinsics** — `tf` and `camera_info`, the two streams
 * `tools/memexport` had and the live bus did not, until the same evening.
 *
 * The whole file exists for one arithmetic fact and one honesty rule.
 *
 * **The arithmetic**: DJI reports the gimbal *earth-referenced*, so the `base_link` → `camera`
 * edge is not the commanded angle — it is the commanded angle with the airframe's own attitude
 * composed away. Writing the obvious number gives a wrong answer that no round trip can see, and
 * the error is 6 cm at a 1 m tag range typically and 60 cm at the flight's measured worst roll of
 * 31.7°. `tools/memexport`'s self-test plants a nadir gimbal under 30° roll and 20° pitch;
 * [the same case is planted here][`the camera edge composes the airframe attitude away`], on the
 * instruction not to re-derive it differently.
 *
 * **The honesty rule**: `camera_info` carries a *fitted* focal length and no distortion claim at
 * all. `docs/mem2-converter.md` §0.2 argues both rejections — not five zeros with `plumb_bob`,
 * which asserts a rectilinear lens nobody has measured, and not NaN, which would poison the one
 * number the message exists to hand over. Every one of those is a test below rather than a comment.
 *
 * ## Mutation-checked 2026-07-27
 *
 * One breakage at a time, whole suite run, reverted after each, by `tmp/mutate.py`, with
 * `app/build/test-results/testDebugUnitTest` deleted before every run — a mutation that fails to
 * compile otherwise leaves the previous run's XML on disk and the harness reports a confident zero.
 * **Counts are failing tests across the whole suite, measured, not estimated** — 2067 tests when
 * the table was taken, 2068 after the `seq` row below added the test that catches it.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the camera edge is the commanded angle, with no airframe composition | 3 |
 *  | the composition is applied the wrong way round (`q ⊗ conj` for `conj ⊗ q`) | 3 |
 *  | the airframe is composed away without conjugating (a double rotation) | 2 |
 *  | the optical rotation is the identity | 2 |
 *  | the optical rotation's y and z axes swapped | 2 |
 *  | **the fixed optical edge is omitted** | **7** |
 *  | an absent gimbal published as a level camera | 3 |
 *  | `world`→`base_link` published with the parent and child swapped | 5 |
 *  | the camera translation zeroed (the 8 cm Ivan measured) | 1 |
 *  | `tf` regressed to a freshness gate | 2 |
 *  | `tfReason` uses `odom`'s strict gate rather than `pose`'s | 1 |
 *  | the tree's edges carry their own stamps rather than the sample's | 1 |
 *  | `tf` headers carry `seq = 0` like every other message | 1 |
 *  | the `tf` key expression joins its type with a slash | 1 |
 *  | `fx` taken from DJI's published 82.1° field of view | 2 |
 *  | `fx` scaled by height rather than width | 3 |
 *  | the principal point put at (0, 0) rather than the image centre | 1 |
 *  | `D` published as five zeros with `plumb_bob` | 1 |
 *  | `D` published as five NaNs | 1 |
 *  | `R` published as zeros rather than the identity | 1 |
 *  | `P` given a non-zero fourth column | 1 |
 *  | `camera_info` stamped `drone/camera` rather than `drone/camera_optical` | 2 |
 *  | a zero or negative resolution accepted | 1 |
 *  | `camera_info` published only on change, never on the heartbeat | 1 |
 *  | `camera_info` published only on the heartbeat, never on a change | 1 |
 *
 * ### The three findings in that table
 *
 * **Omitting the fixed optical edge kills 7 — by far the largest number here, and it is the cheapest
 * thing in the whole tree.** A constant rotation, 100 bytes a second, needing no measurement and no
 * reading. It kills seven because it is the edge everything else is *composed through*: without it
 * `drone/camera_optical` does not exist, and that is the frame the video stamps itself with, so
 * every end-to-end projection assertion fails at once. The number is the argument against ever
 * "optimising" it into a latched publish.
 *
 * **The uncomposed camera edge kills 3, and all three are geometry rather than plumbing.** That is
 * the shape to want: the mutant survives every structural check — three edges, right frames, right
 * stamps, decodes cleanly, round-trips — and dies only where a rotation is actually applied to a
 * vector. Its two variants (conjugating the wrong operand, not conjugating at all) kill 3 and 2 for
 * the same reason. It is the exact failure `docs/mem2-converter.md` calls *"the most subtle thing
 * left in the contract"*, and no byte-level check anywhere would have caught any of the three.
 *
 * **The spec focal length kills only 2, and that is worth knowing rather than fixing.** It is a
 * 13.2 % error in one number, and 13.2 % is not visibly wrong in anything: it is the difference
 * between a consumer solving a pose and solving a confidently wrong one, at a true 3 m reading
 * 2.6 m. Nothing about the message's *shape* changes, so only a test that **knows the fitted
 * number** catches it — which is why the fit's two flights are named in the code as a list rather
 * than folded into the constant. The single-kill rows below it (`D` as zeros, `R` as zeros, the
 * principal point) are the same shape and are worth the same amount: each is a claim that looks
 * exactly like a fact.
 */
class ZenohFramesTest {

    private companion object {
        /** Athens, where every flight in this project has happened. */
        const val LAT = 37.9838096
        const val LON = 23.7275383
        const val TAKEOFF_ALT = 88.5

        val DATUM = OdomDatum(LAT, LON, TAKEOFF_ALT)
        val STAMP = LcmTime(1_753_600_000, 250_000_000)

        val FRESH: SampleAges = SampleAges.of(
            Signal.POSITION to 50L,
            Signal.ALTITUDE to 50L,
            Signal.ATTITUDE to 50L,
            Signal.VELOCITY to 50L,
        )

        fun stale(signal: Signal): SampleAges =
            SampleAges.of(FRESH.asMap() + (signal to (signal.staleAfterMs!! + 1)))

        fun latNorthOf(m: Double) = LAT + m / Geo.METRES_PER_DEG
        fun lonEastOf(m: Double) = LON + m / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        /** Foxglove's own words for the optical frame, quoted so the axes below are checkable. */
        const val OPTICAL_X = "+x points to the right in the video"
        const val OPTICAL_Y = "+y points down"
        const val OPTICAL_Z = "+z points into the plane of the video"
    }

    /**
     * The airframe of `memexport`'s planted case: **roll 30°, pitch 20°**, well away from every
     * degenerate value, with a heading that is not a multiple of ninety.
     */
    private fun tilted(
        rollDeg: Double = 30.0,
        pitchDeg: Double = 20.0,
        yawDeg: Double = 118.0,
    ): AircraftState = AircraftState(
        fcConnected = true,
        latitude = latNorthOf(40.0),
        longitude = lonEastOf(30.0),
        relativeAltitude = 12.0,
        takeoffAltitudeAmsl = TAKEOFF_ALT,
        rollDeg = rollDeg,
        pitchDeg = pitchDeg,
        yawDeg = yawDeg,
        satelliteCount = 14,
        gpsSignalLevel = 5,
        isFlying = true,
        ages = FRESH,
    )

    /** A nadir camera: pitch −90°, roll zero, yaw following the nose. */
    private fun nadir(yawDeg: Double = 118.0) = GimbalEarthAttitude(
        rollDeg = 0.0,
        pitchDeg = -90.0,
        yawDeg = yawDeg,
        source = GimbalEarthAttitude.Source.COMMANDED,
    )

    // ── quaternion helpers, written out so the assertions are about vectors ───

    private fun rotate(q: LcmQuaternion, v: Triple<Double, Double, Double>):
        Triple<Double, Double, Double> {
        val (vx, vy, vz) = v
        val tx = 2.0 * (q.y * vz - q.z * vy)
        val ty = 2.0 * (q.z * vx - q.x * vz)
        val tz = 2.0 * (q.x * vy - q.y * vx)
        return Triple(
            vx + q.w * tx + (q.y * tz - q.z * ty),
            vy + q.w * ty + (q.z * tx - q.x * tz),
            vz + q.w * tz + (q.x * ty - q.y * tx),
        )
    }

    private fun assertVector(
        what: String,
        expected: Triple<Double, Double, Double>,
        actual: Triple<Double, Double, Double>,
        tol: Double = 1e-9,
    ) {
        assertEquals("$what x", expected.first, actual.first, tol)
        assertEquals("$what y", expected.second, actual.second, tol)
        assertEquals("$what z", expected.third, actual.third, tol)
    }

    private fun edges(m: LcmTfMessage) = m.transforms.map { it.header.frameId to it.childFrameId }

    private fun edge(m: LcmTfMessage, parent: String, child: String): LcmTransformStamped =
        m.transforms.single { it.header.frameId == parent && it.childFrameId == child }

    // ─────────────────────────────────────────────────────────────────────────
    // 1. the fixed optical edge — exact, free, and the schema's own definition
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The rotation is asserted **by its axes, not by its four numbers**, because the numbers are a
     * derivation and the axes are the claim. `CompressedVideo` defines its own `frame_id` in these
     * words, so publishing this edge is what makes the video's frame right by construction.
     */
    @Test
    fun `the optical edge maps the schema's own axes onto the camera's`() {
        val q = ZenohTelemetryEncoder.OPTICAL_ROTATION
        // Rotating an optical-frame axis by this edge expresses it in the parent camera frame,
        // whose convention is x forward, y left, z up.
        assertVector(
            "$OPTICAL_Z is the camera's forward axis",
            Triple(1.0, 0.0, 0.0),
            rotate(q, Triple(0.0, 0.0, 1.0)),
        )
        assertVector(
            "$OPTICAL_Y is the camera's down axis",
            Triple(0.0, 0.0, -1.0),
            rotate(q, Triple(0.0, 1.0, 0.0)),
        )
        assertVector(
            "$OPTICAL_X is the camera's right axis",
            Triple(0.0, -1.0, 0.0),
            rotate(q, Triple(1.0, 0.0, 0.0)),
        )
        // And it is a unit quaternion, exactly — the components are ±½ and not the output of a
        // matrix decomposition, so this is an equality rather than a tolerance.
        assertEquals(1.0, q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 0.0)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 2. the camera edge — the one place the obvious number is wrong
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **`memexport`'s planted case, not a re-derivation of it.** A nadir gimbal under 30° roll and
     * 20° pitch: composing the airframe back onto the edge must recover the earth-referenced
     * attitude DJI reported, to floating point.
     *
     * This is the assertion the whole edge exists to satisfy. It fails for a mutant that writes
     * the commanded angle straight in, for one that conjugates the wrong operand, and for one that
     * forgets to conjugate at all — three different ways to be wrong that all produce a tree that
     * decodes perfectly.
     */
    @Test
    fun `the camera edge composes the airframe attitude away`() {
        val s = tilted(rollDeg = 30.0, pitchDeg = 20.0)
        val worldFromBase = ZenohTelemetryEncoder.orientationOrNull(s, Gate.HELD)!!
        val gimbal = nadir(yawDeg = s.yawDeg!!)
        val baseFromCamera = ZenohTelemetryEncoder.cameraEdge(worldFromBase, gimbal)

        // R(world→base) · R(base→camera) must be R(world→camera), which is exactly the
        // earth-referenced angle DJI reported.
        val recomposed = ZenohTelemetryEncoder.qMul(worldFromBase, baseFromCamera)
        val expected = ZenohTelemetryEncoder.enuQuaternion(
            gimbal.rollDeg, gimbal.pitchDeg, gimbal.yawDeg,
        )
        for ((name, pair) in listOf(
            "x" to (expected.x to recomposed.x),
            "y" to (expected.y to recomposed.y),
            "z" to (expected.z to recomposed.z),
            "w" to (expected.w to recomposed.w),
        )) {
            assertEquals("recomposed $name", pair.first, pair.second, 1e-12)
        }
    }

    /**
     * The consequence stated in the contract, and the reason a *reported* roll of zero must not be
     * copied into the edge: **the joint roll is approximately minus the aircraft's roll**, because
     * that is what the stabiliser is doing.
     *
     * Asserted as a vector rather than as an angle: the camera's own down axis, expressed in the
     * *airframe*, must lean 30° away from the airframe's down axis when the airframe is rolled 30°
     * under a level camera.
     */
    @Test
    fun `the joint roll is minus the airframe's, even though the report says zero`() {
        val level = tilted(rollDeg = 0.0, pitchDeg = 0.0, yawDeg = 0.0)
        val rolled = tilted(rollDeg = 30.0, pitchDeg = 0.0, yawDeg = 0.0)
        val gimbal = nadir(yawDeg = 0.0)

        val levelEdge = ZenohTelemetryEncoder.cameraEdge(
            ZenohTelemetryEncoder.orientationOrNull(level, Gate.HELD)!!, gimbal,
        )
        val rolledEdge = ZenohTelemetryEncoder.cameraEdge(
            ZenohTelemetryEncoder.orientationOrNull(rolled, Gate.HELD)!!, gimbal,
        )
        assertNotEquals(
            "an airframe roll must move the joint even though DJI reports the gimbal level",
            levelEdge.x,
            rolledEdge.x,
            1e-6,
        )
        // The camera's forward axis (optical +z, so camera +x) in the airframe's own frame. With
        // the airframe rolled 30° and the camera held at nadir, it must sit 30° off the airframe's
        // −z, in the roll plane.
        val forwardInBody = rotate(rolledEdge, Triple(1.0, 0.0, 0.0))
        val expectedTiltDeg = 30.0
        val tiltDeg = Math.toDegrees(kotlin.math.acos(-forwardInBody.third.coerceIn(-1.0, 1.0)))
        assertEquals("the joint leans by the airframe's own roll", expectedTiltDeg, tiltDeg, 1e-6)
    }

    /**
     * End to end: with the camera commanded to nadir, the optical axis must point **at the
     * ground** in `drone/world` whatever the airframe is doing.
     *
     * That is the property a consumer actually uses — it projects a pixel through the whole chain
     * — and it is the one an edge built from the raw commanded pitch gets wrong by the airframe's
     * tilt while every structural check still passes.
     */
    @Test
    fun `the optical axis points at the ground through the whole chain, at any attitude`() {
        for (attitude in listOf(
            Triple(0.0, 0.0, 0.0),
            Triple(30.0, 20.0, 118.0),
            Triple(-31.7, 27.3, 250.0),
        )) {
            val s = tilted(attitude.first, attitude.second, attitude.third)
            val tree = ZenohTelemetryEncoder.tfOrNull(s, DATUM, STAMP, nadir(attitude.third))!!
            val worldFromBase = edge(tree, "drone/world", "drone/base_link").transform.rotation
            val baseFromCamera =
                edge(tree, "drone/base_link", "drone/camera").transform.rotation
            val cameraFromOptical =
                edge(tree, "drone/camera", "drone/camera_optical").transform.rotation
            val worldFromOptical = ZenohTelemetryEncoder.qMul(
                ZenohTelemetryEncoder.qMul(worldFromBase, baseFromCamera), cameraFromOptical,
            )
            // Optical +z is the viewing direction. In ENU, straight down is (0, 0, −1).
            assertVector(
                "the view direction at attitude $attitude",
                Triple(0.0, 0.0, -1.0),
                rotate(worldFromOptical, Triple(0.0, 0.0, 1.0)),
                tol = 1e-9,
            )
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 3. the tree — which edges, when, and with what stamp
    // ─────────────────────────────────────────────────────────────────────────

    @Test
    fun `the tree is three edges in parent-to-child order, all at one stamp`() {
        val tree = ZenohTelemetryEncoder.tfOrNull(tilted(), DATUM, STAMP, nadir())!!
        assertEquals(
            listOf(
                "drone/world" to "drone/base_link",
                "drone/base_link" to "drone/camera",
                "drone/camera" to "drone/camera_optical",
            ),
            edges(tree),
        )
        // One snapshot of one tree at one time, not three independently aged facts. A consumer
        // that had to re-join edges across stamps would be doing a merge that can be stale in one
        // and fresh in another.
        for (t in tree.transforms) assertEquals(STAMP, t.header.stamp)
    }

    /**
     * **`seq = 1` on every `TransformStamped`, and `seq = 0` on everything else — DiMOS's own
     * defaults, not ours.**
     *
     * Found by `tools/zenohparity/check` and by nothing else: all twelve planted trees differed
     * from `tools/memexport`'s at byte 15 and nowhere else. The cause is upstream —
     * `dimos.msgs.geometry_msgs.Transform` builds its header as `Header(ts, frame_id)`, and that
     * overload of `dimos.msgs.std_msgs.Header.__init__` declares `seq: int = 1`, while every other
     * wrapper this catalogue uses comes out 0.
     *
     * Pinned in both directions, because the interesting claim is the *asymmetry*: a future edit
     * that "tidied" this by making everything 1, or everything 0, would break parity with the store
     * on one side or the other.
     */
    @Test
    fun `tf headers carry DiMOS's seq of one, and every other message carries zero`() {
        val tree = ZenohTelemetryEncoder.tfOrNull(tilted(), DATUM, STAMP, nadir())!!
        for (t in tree.transforms) {
            assertEquals("${t.header.frameId} -> ${t.childFrameId}", 1, t.header.seq)
        }
        assertEquals(1, ZenohTelemetryEncoder.TF_HEADER_SEQ)
        // And the rest of the catalogue is 0, measured against `dimos.msgs` the same way.
        assertEquals(0, ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!.header.seq)
        assertEquals(
            0,
            ZenohTelemetryEncoder.poseStampedOrNull(tilted(), DATUM, STAMP)!!.header.seq,
        )
        assertEquals(0, ZenohTelemetryEncoder.imuOrNull(tilted(), STAMP)!!.header.seq)
    }

    /**
     * **The camera edge is absent when nothing knows where the camera is pointing**, and the tree
     * is still published without it.
     *
     * The absence is the statement. A tree with a fabricated level camera would be worse than one
     * with no camera in it, and a tree withheld entirely would lose the aircraft's own pose for
     * want of a gimbal reading that has nothing to do with it.
     */
    @Test
    fun `with no gimbal angle the camera edge is omitted and the rest is published`() {
        val tree = ZenohTelemetryEncoder.tfOrNull(tilted(), DATUM, STAMP, gimbal = null)!!
        assertEquals(
            listOf(
                "drone/world" to "drone/base_link",
                "drone/camera" to "drone/camera_optical",
            ),
            edges(tree),
        )
    }

    /**
     * The fixed edge is in **every** message, not published once.
     *
     * *"Once"* has no meaning on a bus a subscriber can join at any moment: a late joiner that
     * missed the single announcement could never resolve `drone/camera_optical`, which is the
     * frame the video stamps itself with. It costs about 100 bytes a second.
     */
    @Test
    fun `the fixed optical edge rides in every message, including one with no camera`() {
        for (gimbal in listOf(nadir(), null)) {
            val tree = ZenohTelemetryEncoder.tfOrNull(tilted(), DATUM, STAMP, gimbal)!!
            val fixed = edge(tree, "drone/camera", "drone/camera_optical")
            assertEquals(LcmVector3.ZERO, fixed.transform.translation)
            assertEquals(ZenohTelemetryEncoder.OPTICAL_ROTATION, fixed.transform.rotation)
        }
    }

    /** Ivan, with a ruler: 8 cm forward. Small enough to be tempting to zero, and it is 8 % of an
     * AprilTag landing's error budget at a metre. */
    @Test
    fun `the camera sits eight centimetres forward of the body origin`() {
        val tree = ZenohTelemetryEncoder.tfOrNull(tilted(), DATUM, STAMP, nadir())!!
        assertEquals(
            LcmVector3(0.08, 0.0, 0.0),
            edge(tree, "drone/base_link", "drone/camera").transform.translation,
        )
        // And the vertical component is published as zero because nobody has measured it. Stated
        // as a test so that the day someone does, this is what they change.
        assertEquals(0.0, ZenohTelemetryEncoder.CAMERA_OFFSET_M.z, 0.0)
    }

    /** The aircraft's edge carries `pose`'s own position, in ENU metres from the datum. */
    @Test
    fun `the world edge carries the local position and the airframe attitude`() {
        val s = tilted()
        val tree = ZenohTelemetryEncoder.tfOrNull(s, DATUM, STAMP, nadir())!!
        val t = edge(tree, "drone/world", "drone/base_link").transform
        val expected = ZenohTelemetryEncoder.localPositionOrNull(s, DATUM, Gate.HELD)!!
        assertEquals(expected.x, t.translation.x, 1e-6)
        assertEquals(expected.y, t.translation.y, 1e-6)
        assertEquals(expected.z, t.translation.z, 1e-9)
        assertEquals(ZenohTelemetryEncoder.orientationOrNull(s, Gate.HELD), t.rotation)
    }

    /**
     * **`tf` follows `pose`'s gate, not `odom`'s.**
     *
     * The tree's aircraft edge carries the same position and the same attitude a `pose` does, so
     * there is no defensible reason for the two to disagree about whether that is publishable: a
     * stale altitude is a value that has not changed, and a dead link is a value we no longer know.
     */
    @Test
    fun `the tree is held under liveness and withheld only when the link dies`() {
        val s = tilted()
        // Stale readings on a live link: published, because quiet-because-unchanged is not
        // quiet-because-dead.
        for (signal in listOf(Signal.POSITION, Signal.ALTITUDE, Signal.ATTITUDE)) {
            assertTrue(
                "$signal stale must not withhold the tree",
                ZenohTelemetryEncoder.tfOrNull(
                    s.copy(ages = stale(signal)), DATUM, STAMP, nadir(),
                ) != null,
            )
            assertEquals(
                Withheld.PUBLISHED,
                ZenohEmission.tfReason(s.copy(ages = stale(signal)), DATUM),
            )
        }
        // A dead link: withheld, and the reason names the link rather than a datum on it.
        val dead = s.copy(fcConnected = false)
        assertNull(ZenohTelemetryEncoder.tfOrNull(dead, DATUM, STAMP, nadir()))
        assertEquals(Withheld.LINK_DOWN, ZenohEmission.tfReason(dead, DATUM))
        // And with no origin there is no local frame to express anything in.
        assertEquals(Withheld.NO_DATUM, ZenohEmission.tfReason(s, null))
    }

    /** A tree with no aircraft in it is not a tree. */
    @Test
    fun `no position and no attitude means no tree at all`() {
        assertNull(
            ZenohTelemetryEncoder.tfOrNull(
                tilted().copy(latitude = null, longitude = null), DATUM, STAMP, nadir(),
            ),
        )
        assertNull(
            ZenohTelemetryEncoder.tfOrNull(
                tilted().copy(yawDeg = null), DATUM, STAMP, nadir(),
            ),
        )
    }

    /** All-or-nothing: a partial gimbal attitude is a camera confidently pointing somewhere it is
     * not, and two good axes plus a zeroed third is not a smaller error than none. */
    @Test
    fun `a partial or non-finite gimbal attitude produces no edge`() {
        val src = GimbalEarthAttitude.Source.REPORTED
        assertNull(GimbalEarthAttitude.of(0.0, null, 118.0, src))
        assertNull(GimbalEarthAttitude.of(null, -90.0, 118.0, src))
        assertNull(GimbalEarthAttitude.of(0.0, -90.0, null, src))
        assertNull(GimbalEarthAttitude.of(0.0, Double.NaN, 118.0, src))
        val partial = ZenohTelemetryEncoder.tfOrNull(
            tilted(), DATUM, STAMP,
            GimbalEarthAttitude.of(0.0, null, 118.0, src),
        )!!
        assertEquals(2, partial.transforms.size)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 4. camera_info — a fit, and two things it must not pretend
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The fitted focal length, and the number a consumer would otherwise reach for.
     *
     * Two independent flights fitted **1465.9 px** and **1448.3 px** at 1920 wide, agreeing to
     * 1.2 % while the same fits' intercepts disagreed entirely — which is the evidence the fit
     * separated a camera error from a scene error. DJI's published 82.1° diagonal implies 0.659·W
     * and is **13.2 % low**.
     */
    @Test
    fun `fx is the two flights' fit and not DJI's published field of view`() {
        val info = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        val mean = ZenohTelemetryEncoder.FOCAL_FITS_PX_AT_1920.average()
        assertEquals("the fit's mean at its own width", mean, info.k[0], 1e-9)
        assertEquals("fx == fy assumes square pixels, which nothing has measured", info.k[0], info.k[4], 0.0)
        // 13.2 % apart, and both are plausible-looking numbers. Only knowing the fit tells them
        // apart, which is why the two flights are named in the code.
        val spec = ZenohTelemetryEncoder.FOCAL_OVER_WIDTH_FROM_SPEC * 1920
        assertTrue(
            "the spec figure must be well below the fit — it was, by 13.2 %",
            abs(info.k[0] - spec) / info.k[0] > 0.12,
        )
    }

    /** Scaled by **width**, so the intrinsics follow a resolution change rather than being pinned
     * to one — and a 960-wide stream gets half the focal length, not the same one. */
    @Test
    fun `the focal length scales with the width and not the height`() {
        val full = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        val half = ZenohTelemetryEncoder.cameraInfo(960, 540, STAMP)!!
        assertEquals(full.k[0] / 2.0, half.k[0], 1e-9)
        // A 4:3 stream at the same width must have the same focal length. A mutant scaling by
        // height would move it by a third here and nowhere else.
        val fourThree = ZenohTelemetryEncoder.cameraInfo(1920, 1440, STAMP)!!
        assertEquals(full.k[0], fourThree.k[0], 0.0)
    }

    /** The principal point is **assumed** to be the image centre. Nothing has measured it, and
     * writing it down as a test is what keeps the assumption visible. */
    @Test
    fun `the principal point is the image centre`() {
        val info = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        assertEquals(960.0, info.k[2], 0.0)
        assertEquals(540.0, info.k[5], 0.0)
        assertEquals(1.0, info.k[8], 0.0)
        assertEquals(0.0, info.k[1], 0.0)
        assertEquals(0.0, info.k[3], 0.0)
    }

    /**
     * **Distortion is published as an absence**, and the two rejected alternatives matter as much
     * as the choice.
     *
     * Five zeros with `plumb_bob` is an affirmative claim that a 66.8° lens is rectilinear, which
     * nothing has measured. NaN is this project's sentinel for *"there is no feed at all"* and
     * would poison every projection including the focal length the message exists to deliver. An
     * empty `D` with an empty model is ROS's own "no model is offered", and it forces a consumer
     * that wants zero distortion to assume it visibly.
     */
    @Test
    fun `distortion is an empty array and an empty model name, never zeros and never NaN`() {
        val info = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        assertEquals(emptyList<Double>(), info.d)
        assertEquals("", info.distortionModel)
        // And it survives the wire: `D` is length-prefixed, so "no model offered" is expressible.
        val decoded = CameraInfoCodec.decode(CameraInfoCodec.encode(info))
        assertEquals(emptyList<Double>(), decoded.d)
        assertEquals("", decoded.distortionModel)
    }

    /**
     * `R` is the identity and that is **exact, not an assumption**: an unrectified monocular
     * camera's rectification *is* the identity. `P` is `K` with a zero fourth column, derived
     * rather than claimed.
     */
    @Test
    fun `R is the identity and P is K with a zero fourth column`() {
        val info = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        assertEquals(listOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), info.r)
        assertEquals(12, info.p.size)
        // Row-major 3×4: P[j] == K[i] with a zero in every fourth column.
        for (row in 0 until 3) {
            for (col in 0 until 3) {
                assertEquals("P[$row][$col]", info.k[row * 3 + col], info.p[row * 4 + col], 0.0)
            }
            assertEquals("P[$row][3] must be zero", 0.0, info.p[row * 4 + 3], 0.0)
        }
    }

    /**
     * **The optical frame, not the camera frame.** The projection this matrix describes is
     * expressed in the optical convention, and it is the same leaf `CompressedVideo` stamps itself
     * with — naming the parent body frame looks fine until someone composes a transform with it.
     */
    @Test
    fun `camera_info is stamped in the optical frame`() {
        val info = ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!
        assertEquals("drone/camera_optical", info.header.frameId)
        assertEquals(STAMP, info.header.stamp)
        assertEquals(1920, info.width)
        assertEquals(1080, info.height)
        // Nothing crops and nothing bins on this camera, so the honest values are zeros.
        assertEquals(0, info.binningX)
        assertEquals(0, info.binningY)
        assertEquals(LcmRegionOfInterest(), info.roi)
    }

    /** Before a frame has stated a geometry there is nothing to build intrinsics for, and a
     * guessed 1920×1080 would be exactly the plausible artefact this project keeps meeting. */
    @Test
    fun `a resolution that was never stated produces no intrinsics`() {
        assertNull(ZenohTelemetryEncoder.cameraInfo(0, 1080, STAMP))
        assertNull(ZenohTelemetryEncoder.cameraInfo(1920, 0, STAMP))
        assertNull(ZenohTelemetryEncoder.cameraInfo(-1920, -1080, STAMP))
    }

    /**
     * ROS documents `K[0] == 0` as the flag for an uncalibrated camera, and ours is not zero —
     * deliberately, because we have a usable focal length and zeroing it would throw away the
     * correction that is the whole reason for the stream.
     *
     * The qualification therefore has to travel somewhere else, and this test is where that fact
     * is recorded: it is in the message's documentation and in `docs/mem2-converter.md` §0.2, not
     * on the wire. A consumer reading `K[0] != 0` as "calibrated" is reading more than we said.
     */
    @Test
    fun `K is not zeroed, so the uncalibrated flag is deliberately not set`() {
        assertTrue(ZenohTelemetryEncoder.cameraInfo(1920, 1080, STAMP)!!.k[0] > 0.0)
    }
}
