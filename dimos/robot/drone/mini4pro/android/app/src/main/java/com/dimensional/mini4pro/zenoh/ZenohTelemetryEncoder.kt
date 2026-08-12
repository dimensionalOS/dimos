package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.record.Json
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.SetpointFrame
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.vision.TagSighting
import kotlin.math.cos
import kotlin.math.sin

/**
 * Where the local frame's origin sits on Earth: this flight's takeoff point.
 *
 * **Held internally, and no longer published.** The `datum` channel and its generation counter
 * were deleted on 2026-07-27, and the reason is arithmetic rather than taste: our local position
 * is *GPS now minus GPS at takeoff*, so any consumer holding both `pose` and `gps_location` —
 * both of which we publish, both timestamped — recovers this origin by subtraction, using every
 * sample rather than the one instant we chose to announce. Measured over 267 paired samples of the
 * 2026-07-27 mission: recovered `37.9938403, 23.7252870` against published `37.9938403,
 * 23.7252870`, **difference 0.0000 m**, worst per-sample residual 4.4 µm — floating-point noise
 * from applying `cos(latitude)` per sample. The channel was asserting, once a second, something
 * the data already proved to the width of a double.
 *
 * The generation counter went with it. It existed so that a local coordinate authored against one
 * origin could not be flown against another — origins measured a median 2.5 m apart, p90 8.7 m,
 * max 13.4 m across twenty sessions on the same patch of ground, so the error it guarded was real.
 * What removed it was **one flight per recording, one flight per session**: there is now no way to
 * express a coordinate from another flight at all, so the failure is not defended against, it is
 * unreachable. `docs/zenoh-frames-and-paths.md`.
 *
 * @param takeoffAltitudeM the barometric altitude DJI reported at the origin
 *   ([AircraftState.takeoffAltitudeAmsl]), or null when it never arrived. It is **pressure
 *   altitude on the 1013.25 hPa reference, not AMSL** — measured to move 44 m at a stationary
 *   site over two days — so it is metadata about where the vertical zero sits and never a number
 *   to compute with. Every Z this encoder publishes is metres above it, by construction.
 */
data class OdomDatum(
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val takeoffAltitudeM: Double? = null,
) {

    companion object {
        /**
         * **The local frame: origin at takeoff, and not what the ROS convention implies.**
         *
         * Called `world` rather than `odom` because it does not accumulate error: our local
         * position is *GPS now minus GPS at takeoff*, and there is no odometry anywhere in this
         * system — no VIO, no wheel odometry, no filter. The horizontal error is bounded by GPS
         * accuracy and does not grow with time or distance, which is the property `odom` exists
         * to warn about and `world`/`map` exist to promise.
         *
         * Two things travel with that name and must not be separated from it:
         *
         *  - **the vertical is barometric, and it does drift** — 2.3 m in twelve minutes measured,
         *    and `takeoffAltitudeAmsl` has moved 41.5 m between sessions. Horizontal may be
         *    trusted over a flight; vertical may not.
         *  - **nothing is smoothed.** A GPS jump of two metres appears as a two-metre jump in
         *    local position, because it is the same measurement with a constant subtracted.
         *
         * `docs/mem2-converter.md` §2.1 is the argument; `docs/zenoh-frames-and-paths.md` is the
         * arithmetic. GPS scatter while stationary measured at 0.03 m median — which is
         * *precision* and says nothing about absolute accuracy, a distinction that decides
         * whether anyone tries to land on this frame. They should not; that is what the tag is
         * for.
         */
        const val FRAME_WORLD = "drone/world"

        /** The aircraft body. Namespaced so a second aircraft on one bus is expressible. */
        const val FRAME_BASE_LINK = "drone/base_link"

        /**
         * The camera, as the gimbal points it. A **joint** rotation: DJI reports gimbal attitude
         * earth-referenced, so this edge composes the airframe's own attitude away rather than
         * carrying DJI's number through. Measured 2026-07-27: reported roll stayed within
         * {−0.1, 0, 0.4}° while the aircraft rolled to 22.8°, and writing the commanded pitch
         * straight into this edge is wrong by a median 3.6° and a maximum 31.7°.
         *
         * Translation `(0.08, 0, 0)` m — measured with a ruler, 2026-07-27.
         */
        const val FRAME_CAMERA = "drone/camera"

        /**
         * The optical frame: +x right in the image, +y down, +z along the view. A fixed rotation
         * from [FRAME_CAMERA], exact and free, and **the frame Foxglove's `CompressedVideo`
         * defines for itself** — so a recorded frame's `frame_id` and this leaf are the same
         * thing by construction rather than by our assertion.
         */
        const val FRAME_CAMERA_OPTICAL = "drone/camera_optical"

        /**
         * The datum for a takeoff happening **now**, or null when the aircraft's position is
         * not something we are willing to repeat ([Geo.coordinateOrNull]) or is not fresh.
         *
         * The freshness gate is the point: a datum recorded from a cached fix anchors the whole
         * flight's local frame to wherever the aircraft was the last time the GPS spoke, and
         * every local coordinate afterwards inherits that error with no way to see it. A datum
         * we could not record is an absence — the caller publishes no datum and no local
         * message, which is the honest state of affairs.
         */
        fun atTakeoff(s: AircraftState): OdomDatum? {
            val (lat, lon) = Geo.coordinateOrNull(s.latitude, s.longitude) ?: return null
            if (!s.isFresh(Signal.POSITION)) return null
            return OdomDatum(
                latitudeDeg = lat,
                longitudeDeg = lon,
                takeoffAltitudeM = s.takeoffAltitudeAmsl?.takeIf { it.isFinite() },
            )
        }
    }
}

/**
 * Turns an [AircraftState] into the LCM messages the Zenoh bus carries — the telemetry half of
 * `docs/zenoh-topics.md`. Pure arithmetic: no Zenoh, no Android, no DJI, no clock reads. The
 * caller passes the stamp, so every message is a deterministic function of its inputs and can be
 * asserted field by field on the JVM. [com.dimensional.mini4pro.telemetry.TelemetryEncoder] is
 * this class's sibling for the MAVLink wire.
 *
 * ## 1. ENU, not NED — the conversion, in one place
 *
 * The aircraft and every existing line of this codebase speak **NED**: X north, Y east, Z
 * **down**. The Zenoh bus speaks **ENU**: X east, Y north, Z **up** (Z-6). Every position and
 * every velocity crosses that boundary here, in [enuFromNed] and nowhere else. Get a sign wrong
 * and the drone descends when told to climb; the axis mapping is stated in that function's KDoc
 * and pinned by a test on all three axes in both signs.
 *
 * ## 2. `odom`, `base_link`, and the datum
 *
 * Local position is metres from **this flight's takeoff point**, in ENU, in the frame ROS calls
 * `odom`; the body is `base_link`. Where that origin sits on Earth is published separately, as
 * its own `NavSatFix` carrying a generation counter — see [OdomDatum]. Local metres come from
 * [Geo.enuMetres], including the `cos(latitude)` factor that is a 21 % error at this site if
 * dropped.
 *
 * ## 3. Altitude is always height above **this flight's** takeoff point
 *
 * In every message here, never AMSL, including inside [gpsLocationOrNull] where ROS's own
 * documentation asks for a WGS-84 ellipsoid height. `docs/zenoh-frames-and-paths.md` §4 has the
 * argument: the only vertical reference this aircraft knows is `relativeAltitude`, the
 * barometric datum under any "AMSL" we could compute moved 41.5 m between sessions, and a
 * vertical error is the one that ends in the ground. The deviation is deliberate and is
 * announced here rather than discovered by a consumer.
 *
 * ## 4. Null means no reading — so a message is withheld, not zero-filled
 *
 * `AircraftState` is null-per-field because null means *no reading*, never zero, and LCM has no
 * sentinel vocabulary — a `NavSatFix` at 0,0 is a real place in the Atlantic and a `Point` at
 * 0,0,0 is the takeoff pad. **Every function here returns null rather than a zero-filled
 * message**, and absence is the signal a Zenoh subscriber is well equipped to notice.
 *
 * That is the difference from `TelemetryEncoder`, which must keep a stream flowing to a GCS and
 * therefore uses MAVLink's documented sentinels. The two consequently share no code below
 * `AircraftState`, and that is correct rather than duplication.
 *
 * The line between *withhold* and *encode as unknown* is drawn once, here:
 *
 * - **Withheld** when a quantity the aircraft does report is missing or stale — position,
 *   altitude, attitude, velocity, battery charge. There is no honest encoding of "we had this a
 *   moment ago and do not now"; silence is it.
 * - **`NaN`** only for quantities this airframe **never** reports at all — angular rate and
 *   linear acceleration, for which there is no feed to go stale. Zero would read as a
 *   confidently motionless aircraft. Where ROS documents its own "unmeasured" convention the
 *   documented one is used as well (`Imu`'s `covariance[0] = -1`, `BatteryState`'s NaN fields),
 *   so a convention-following consumer and a NaN-checking one both get the truth.
 *
 * Freshness is checked with [AircraftState.isFresh] only for the four signals that have a
 * measured continuous cadence and therefore a `Signal.staleAfterMs`. For the event-driven keys —
 * battery, flight mode — a long silence means "nothing has changed", so a non-null value *is*
 * the whole check and a staleness gate would suppress on normal operation.
 */
/**
 * **How a reading is judged usable**, and the one place this project's oldest trap is answered
 * twice, with two different answers, on purpose.
 *
 * Every telemetry key on this airframe is *change-driven*: it publishes when its value changes and
 * says nothing when it does not. So a quantity that is genuinely constant — an altitude held in
 * level flight, an attitude held in a hover, a velocity held at cruise — looks exactly like a feed
 * that has died. Reading that silence as "unknown" is what produced 45.3 % pose coverage on the
 * first real mission, with a 22-second hole in the middle of it.
 *
 * The two answers, and why they differ:
 *
 *  - [FRESH] — the value arrived within its own staleness limit. What **flight control** requires
 *    everywhere without exception, and what `odom` requires, because a twist is a claim about
 *    motion *now*. An aircraft must never be flown on a held value: there, "unchanged" and "dead"
 *    genuinely do deserve the same conservative answer, because acting on the wrong one moves an
 *    aircraft.
 *  - [HELD] — the value is present and **the link is alive now**. What the dense published streams
 *    use. Quiet-because-unchanged is not quiet-because-dead, and the only signal that can separate
 *    them is one about the *link* rather than about any datum, because every data key falls silent
 *    together. Measured effect on the 2026-07-27 mission: `pose` from 267 to 752 of 902 samples,
 *    and video frames with a position within 100 ms from 45.3 % to 100.0 %.
 *
 * The load-bearing assumption, recorded so that it is discovered on purpose rather than by
 * accident: **`fcConnected` proves the link, not any individual key's subscription.** A
 * subscription dying silently while the link stayed up would be held forever and published
 * confidently. We have never observed that — the measured failure is an FC blackout delivering
 * nulls across every key at once, which liveness catches by construction — but nobody has looked
 * for the other shape on purpose. `docs/mem2-converter.md` §3.4.
 */
enum class Gate {
    FRESH,
    HELD,
    ;

    fun allows(s: AircraftState, signal: Signal): Boolean = when (this) {
        FRESH -> s.isFresh(signal)
        HELD -> s.fcConnected
    }
}

object ZenohTelemetryEncoder {

    /** `docs/zenoh-frames-and-paths.md` §6a — the local ENU frame anchored at takeoff. */
    const val FRAME_WORLD = OdomDatum.FRAME_WORLD

    /** The aircraft body. `Odometry.child_frame_id`, and the sensor frame of `imu`/`gps_location`. */
    const val FRAME_BASE_LINK = OdomDatum.FRAME_BASE_LINK

    /** The camera as the gimbal points it — the `tf` tree's third frame. */
    const val FRAME_CAMERA = OdomDatum.FRAME_CAMERA

    /** The optical leaf, and `CompressedVideo`'s own `frame_id`. */
    const val FRAME_CAMERA_OPTICAL = OdomDatum.FRAME_CAMERA_OPTICAL

    /**
     * A three-vector this airframe has **no feed for at all**, ever — not a reading that went
     * missing. `NaN` on every axis, because 0 would read as a confidently motionless aircraft
     * and there is no reading to wait for. See §4 of the class doc for the line this sits on.
     */
    val UNMEASURED_VECTOR3 = LcmVector3(Double.NaN, Double.NaN, Double.NaN)

    // ── 1. the NED → ENU boundary ─────────────────────────────────────────────

    /**
     * An NED triple as ENU. **The axis mapping, stated once:**
     *
     * ```
     * ENU x (east)  =  NED y (east)
     * ENU y (north) =  NED x (north)
     * ENU z (up)    = −NED z (down)      ← the sign that ends in the ground
     * ```
     *
     * X and Y **swap**; Z **negates**. Both mistakes are silent: a swap flies east when told
     * north, and an unnegated Z descends when told to climb. Applies unchanged to a position
     * offset and to a velocity, which is why both go through this one function.
     *
     * Takes the values through untouched otherwise — no clamping, no finiteness check. A
     * non-finite input is the caller's to refuse, and every caller here does.
     */
    fun enuFromNed(north: Double, east: Double, down: Double): LcmVector3 =
        LcmVector3(x = east, y = north, z = -down)

    /**
     * DJI's aircraft attitude in degrees → the ENU orientation of `base_link` in `odom`, as a
     * quaternion.
     *
     * ## The two conventions, both stated because both are assumptions
     *
     * **What DJI is assumed to report** (`KeyAircraftAttitude`): standard aerospace Euler angles
     * in a body **FRD** frame — X forward, Y right, Z down — composed `Rz(yaw)·Ry(pitch)·Rx(roll)`
     * against a world **NED** frame, with `yaw = 0` at true north and increasing clockwise
     * (eastward), positive `pitch` nose-up, positive `roll` right-wing-down.
     * **`AircraftState.rollDeg`/`pitchDeg` record the sign convention as UNVERIFIED — it needs a
     * tilt test** — so the roll and pitch *signs* here inherit that doubt. Yaw does not: it is
     * documented and used by the whole MAVLink path.
     *
     * **What ROS wants**: the rotation from body **FLU** — X forward, Y left, Z up — into world
     * **ENU**, quaternion `(x, y, z, w)` with **w last** on the wire.
     *
     * ## The conversion
     *
     * Composing the two frame changes leaves a rotation that is again `Rz·Ry·Rx`, in ENU/FLU,
     * with the angles renamed:
     *
     * ```
     * roll_enu  =  roll          (right-wing-down stays right-wing-down)
     * pitch_enu = −pitch         (Y flips from right to left, so nose-up flips sign)
     * yaw_enu   =  90° − yaw     (from clockwise-from-north to counterclockwise-from-east)
     * ```
     *
     * That identity is not taken on trust: `ZenohTelemetryEncoderTest` builds the rotation matrix
     * of this quaternion at a non-trivial attitude (roll 20°, pitch −15°, yaw 118° — no axis
     * degenerate, no symmetry to hide a swap) and compares it, element by element, against
     * `T_enu←ned · Rz(yaw)Ry(pitch)Rx(roll) · T_frd←flu` computed independently from the axis
     * permutations. The 90° yaw term and both sign flips are each pinned by a separate test as
     * well, because a matrix identity that fails tells you less than a case that names the axis.
     */
    fun enuQuaternion(rollDeg: Double, pitchDeg: Double, yawDeg: Double): LcmQuaternion {
        val halfRoll = Math.toRadians(rollDeg) / 2.0
        val halfPitch = Math.toRadians(-pitchDeg) / 2.0
        val halfYaw = Math.toRadians(90.0 - yawDeg) / 2.0
        val cr = cos(halfRoll)
        val sr = sin(halfRoll)
        val cp = cos(halfPitch)
        val sp = sin(halfPitch)
        val cy = cos(halfYaw)
        val sy = sin(halfYaw)
        return LcmQuaternion(
            x = sr * cp * cy - cr * sp * sy,
            y = cr * sp * cy + sr * cp * sy,
            z = cr * cp * sy - sr * sp * cy,
            w = cr * cp * cy + sr * sp * sy,
        )
    }

    // ── 2. the local frame ────────────────────────────────────────────────────

    /**
     * The aircraft's position in `odom`: **ENU metres from [datum]**, or null when we do not
     * know where the aircraft is, how high it is, or where the origin was.
     *
     * `z` is [AircraftState.relativeAltitude] — height above *this flight's* takeoff point —
     * straight through, which is what the `odom` origin's Z is by construction. Never AMSL; see
     * §3 of the class doc.
     *
     * Both coordinates go through [Geo], the aircraft's *and* the datum's: a local metre against
     * an origin that is DJI's no-home filler is not a smaller error than a bad latitude, it is
     * the same error moved somewhere harder to see.
     */
    fun localPositionOrNull(s: AircraftState, datum: OdomDatum, gate: Gate = Gate.FRESH): LcmPoint? {
        val origin = Geo.coordinateOrNull(datum.latitudeDeg, datum.longitudeDeg) ?: return null
        val here = Geo.coordinateOrNull(s.latitude, s.longitude) ?: return null
        if (!gate.allows(s, Signal.POSITION)) return null
        val up = s.relativeAltitude?.takeIf { it.isFinite() } ?: return null
        if (!gate.allows(s, Signal.ALTITUDE)) return null
        val (east, north) = Geo.enuMetres(origin.first, origin.second, here.first, here.second)
        return LcmPoint(x = east, y = north, z = up)
    }

    /** The ENU orientation of `base_link`, or null when the attitude is missing or stale. */
    fun orientationOrNull(s: AircraftState, gate: Gate = Gate.FRESH): LcmQuaternion? {
        val roll = s.rollDeg?.takeIf { it.isFinite() } ?: return null
        val pitch = s.pitchDeg?.takeIf { it.isFinite() } ?: return null
        val yaw = s.yawDeg?.takeIf { it.isFinite() } ?: return null
        if (!gate.allows(s, Signal.ATTITUDE)) return null
        return enuQuaternion(roll, pitch, yaw)
    }

    /**
     * The aircraft's velocity in ENU m/s, or null when it is missing or stale.
     *
     * **An aged velocity is not a measured zero.** `KeyAircraftVelocity` is change-driven —
     * it fired once in 35 s on the ground probe — so a stale reading is genuinely
     * indistinguishable from a stationary aircraft, and publishing the cached number as if it
     * were live is the one thing that makes those two look different when they are not.
     */
    fun velocityEnuOrNull(s: AircraftState): LcmVector3? {
        val north = s.velocityNorth?.takeIf { it.isFinite() } ?: return null
        val east = s.velocityEast?.takeIf { it.isFinite() } ?: return null
        val down = s.velocityDown?.takeIf { it.isFinite() } ?: return null
        if (!s.isFresh(Signal.VELOCITY)) return null
        return enuFromNed(north = north, east = east, down = down)
    }

    // ── the `odom` channel ────────────────────────────────────────────────────

    /**
     * The `odom` channel — **the primary telemetry message**: pose *and* twist together, in
     * `odom`, for the body `base_link`. Null unless position, altitude, attitude *and* velocity
     * are all present and fresh.
     *
     * That conjunction is strict, and the cost is real and worth naming: `VELOCITY` is
     * change-driven on this airframe and reads stale for most of any period the aircraft is not
     * accelerating, so on the ground this message is mostly absent, and in flight it may gap.
     * [poseStampedOrNull] carries the pose half through those gaps. The alternative — a cached
     * or zeroed twist — publishes "not moving" on a dead feed, which is the failure this whole
     * layer is built to refuse.
     *
     * **`twist` is world ENU, not body frame.** ROS's `Odometry` conventionally expresses twist
     * in `child_frame_id`; `docs/zenoh-topics.md` specifies *"velocity is ENU m/s"* for this
     * channel and that is what is published. The deviation is deliberate — the body-frame
     * rotation would need a live heading, so a stale attitude would take the velocity with it —
     * and it is recorded here because a consumer following ROS to the letter would read it wrong.
     *
     * `twist.angular` is [UNMEASURED_VECTOR3]: this airframe reports no angular rate at all.
     */
    fun odometryOrNull(s: AircraftState, datum: OdomDatum, stamp: LcmTime): LcmOdometry? {
        val position = localPositionOrNull(s, datum) ?: return null
        val orientation = orientationOrNull(s) ?: return null
        val velocity = velocityEnuOrNull(s) ?: return null
        return LcmOdometry(
            header = LcmHeader(stamp = stamp, frameId = FRAME_WORLD),
            childFrameId = FRAME_BASE_LINK,
            pose = LcmPoseWithCovariance(LcmPose(position, orientation)),
            twist = LcmTwistWithCovariance(LcmTwist(velocity, UNMEASURED_VECTOR3)),
        )
    }

    /**
     * The `pose` channel — the pose half of [odometryOrNull] alone, for DiMOS's existing drone
     * module, which expects `odom` to be a `PoseStamped`. Free to publish and costs nothing to
     * ignore; it also survives a velocity gap, which `Odometry` does not.
     */
    fun poseStampedOrNull(s: AircraftState, datum: OdomDatum, stamp: LcmTime): LcmPoseStamped? {
        val position = localPositionOrNull(s, datum, Gate.HELD) ?: return null
        val orientation = orientationOrNull(s, Gate.HELD) ?: return null
        return LcmPoseStamped(
            header = LcmHeader(stamp = stamp, frameId = FRAME_WORLD),
            pose = LcmPose(position, orientation),
        )
    }

    // ── the global fix ────────────────────────────────────────────────────────

    /**
     * Satellites required before DJI's signal level 3+ is called a fix — the same floor
     * `TelemetryEncoder.MIN_SATS_FOR_3D` applies, deliberately duplicated rather than shared.
     */
    const val MIN_SATS_FOR_FIX = 6

    /**
     * DJI's GPS signal level and satellite count → `NavSatStatus.status`.
     *
     * **[LcmNavSatStatus.STATUS_NO_FIX] is −1, and it is a *signed* `int8`.** Read as an unsigned
     * byte it is 255, which is not "no fix" but a value no consumer has a name for, and which a
     * lenient one may well treat as a confident augmented fix. The codec writes it signed; this
     * is the value that makes that matter.
     *
     * `NavSatStatus` has no way to say *2D*, so the question it answers is only fix / no fix, and
     * it is answered on the same thresholds `TelemetryEncoder.fixType` uses for a 3D fix: level
     * ≥ 3, and at least [MIN_SATS_FOR_FIX] satellites when the count is known. A DJI level of 2 is
     * a marginal 2D solution — enough for a position, not enough to assert one — and it reports
     * no fix here rather than being rounded up. **The two wires therefore never disagree about
     * whether this aircraft has a fix**, which is the property worth having; the duplicated
     * thresholds are the price, and they are named in both places.
     *
     * Never SBAS or GBAS: nothing in the MSDK reports either, and claiming one is claiming a
     * precision the airframe does not have.
     */
    fun navSatStatus(s: AircraftState): LcmNavSatStatus {
        val level = s.gpsSignalLevel
        val sats = s.satelliteCount
        val fixed = level != null &&
            level >= 3 &&
            (sats == null || sats >= MIN_SATS_FOR_FIX)
        return LcmNavSatStatus(
            status = if (fixed) LcmNavSatStatus.STATUS_FIX else LcmNavSatStatus.STATUS_NO_FIX,
            service = LcmNavSatStatus.SERVICE_GPS,
        )
    }

    /**
     * The `gps_location` channel — the global fix, unmediated — or null when the position is
     * missing, stale, or not a coordinate we are willing to repeat.
     *
     * **`altitude` is metres above this flight's takeoff point, not the WGS-84 ellipsoid height
     * ROS asks for.** That is the deliberate deviation of §3 in the class doc, and it is the one
     * field on this message a consumer could be surprised by. The alternative is a barometric
     * "AMSL" whose datum moved 41.5 m between sessions, i.e. a number that looks like an
     * elevation and is not one.
     *
     * The altitude is required, not defaulted: a `NavSatFix` with `altitude = 0` reads as an
     * aircraft sitting on its takeoff point, which is a specific and wrong claim.
     */
    fun gpsLocationOrNull(s: AircraftState, stamp: LcmTime): LcmNavSatFix? {
        val (lat, lon) = Geo.coordinateOrNull(s.latitude, s.longitude) ?: return null
        if (!Gate.HELD.allows(s, Signal.POSITION)) return null
        val up = s.relativeAltitude?.takeIf { it.isFinite() } ?: return null
        if (!Gate.HELD.allows(s, Signal.ALTITUDE)) return null
        return LcmNavSatFix(
            header = LcmHeader(stamp = stamp, frameId = FRAME_BASE_LINK),
            status = navSatStatus(s),
            latitude = lat,
            longitude = lon,
            altitude = up,
        )
    }

    // ── imu ───────────────────────────────────────────────────────────────────

    /**
     * ROS's documented "this quantity is not reported": **−1 in element 0** of the covariance.
     * The rest stay zero; nothing reads them once element 0 is negative.
     */
    val UNREPORTED_COVARIANCE_9: List<Double> =
        List(COVARIANCE_SIZE_9) { if (it == 0) -1.0 else 0.0 }

    /**
     * The `imu` channel — orientation only, or null when the attitude is missing or stale.
     *
     * `header.frame_id` is `base_link`: the ROS convention is that an `Imu`'s frame is the
     * sensor's own, and `orientation` is that frame's rotation into a world frame — here `odom`,
     * which is earth-aligned ENU and so is the same rotation `Odometry` carries.
     *
     * **Angular velocity and linear acceleration are `NaN`, with `covariance[0] = -1`.** This
     * airframe exposes no gyro or accelerometer feed at all, so there is nothing to go stale and
     * nothing to wait for — see §4 of the class doc for why that is `NaN` here and a withheld
     * message elsewhere. The −1 covariance is ROS's own convention for the same statement, so a
     * consumer that follows ROS and one that checks for `NaN` both learn the truth; most ROS
     * drivers pair the −1 with a zero value, and a zero angular rate is exactly the confident
     * lie this project refuses.
     *
     * `orientation_covariance` is left at zeros — "unknown, and not claiming otherwise" — rather
     * than −1: the orientation *is* reported, we simply have no uncertainty figure for it.
     *
     * Note the rate this channel can honestly carry. Attitude was measured arriving at ~2 Hz and
     * change-driven with it (`docs/measurements/2026-07-26-attitude-and-staleness.md`), so a
     * publisher asking for 5 Hz will get the same reading twice; that is the caller's decision
     * and open item 1 of `docs/zenoh-topics.md`, not something this encoder can paper over.
     */
    fun imuOrNull(s: AircraftState, stamp: LcmTime): LcmImu? {
        val orientation = orientationOrNull(s, Gate.HELD) ?: return null
        return LcmImu(
            header = LcmHeader(stamp = stamp, frameId = FRAME_BASE_LINK),
            orientation = orientation,
            orientationCovariance = ZERO_COVARIANCE_9,
            angularVelocity = UNMEASURED_VECTOR3,
            angularVelocityCovariance = UNREPORTED_COVARIANCE_9,
            linearAcceleration = UNMEASURED_VECTOR3,
            linearAccelerationCovariance = UNREPORTED_COVARIANCE_9,
        )
    }

    // ── battery ───────────────────────────────────────────────────────────────

    /**
     * The `battery` channel, or null when the charge remaining is unknown or is not a
     * percentage.
     *
     * **[LcmBatteryState.percentage] is ROS's 0..1 fraction, not a 0..100 percent.** DJI reports
     * whole percent, so this is the one field on this message that is divided rather than copied,
     * and getting it wrong publishes 62 for a 62 % battery — a value 62× over full that a
     * consumer's low-battery logic will read as fine forever. A reading outside 0..100 is not a
     * percentage and the message is withheld rather than clamped: clamping 150 to 100 invents a
     * full battery out of a broken reading.
     *
     * Units, all of which differ from the MAVLink side and none of which are shared with it:
     *
     * | field | DJI | here |
     * |---|---|---|
     * | voltage | whole-pack mV | **V** |
     * | current | mA, **negative discharging** | **A**, negative discharging — the sign passes straight through, where MAVLink needs it inverted |
     * | temperature | °C | °C, unchanged |
     * | cell voltages | mV each, in cell order | **V** each, in cell order, **never averaged** — a spread between cells is a flight-safety signal and any averaging erases it |
     *
     * Unmeasured fields are `NaN`, which is `sensor_msgs/BatteryState`'s **own documented**
     * convention (*"If unmeasured NaN"* on temperature, current, charge, capacity,
     * design_capacity, percentage) — one of the few places ROS names a sentinel, so it is used
     * rather than an invented one. `voltage` is documented "Mandatory value" and is still NaN
     * when DJI has not reported it: a known charge with an unknown voltage is worth publishing,
     * and NaN is ROS's own word for the gap.
     *
     * `power_supply_health` stays `UNKNOWN` even though this airframe's characteristic failure is
     * a battery overheat. That fault is DJI's own device-health warning and it has its own
     * `health` channel; inferring `OVERHEAT` from a temperature threshold here would be this
     * bridge inventing a diagnosis DJI did not make.
     */
    fun batteryOrNull(s: AircraftState, stamp: LcmTime): LcmBatteryState? {
        val percent = s.batteryPercent?.takeIf { it in 0..100 } ?: return null
        return LcmBatteryState(
            header = LcmHeader(stamp = stamp, frameId = FRAME_BASE_LINK),
            voltage = s.voltageMv?.let { it / 1000f } ?: Float.NaN,
            temperature = s.batteryTempC?.takeIf { it.isFinite() }?.toFloat() ?: Float.NaN,
            // Negative while discharging in both conventions — no inversion, unlike MAVLink's cA.
            current = s.currentMa?.let { it / 1000f } ?: Float.NaN,
            charge = Float.NaN,
            capacity = Float.NaN,
            designCapacity = Float.NaN,
            // 0..1. Not 0..100.
            percentage = percent / 100f,
            powerSupplyStatus = powerSupplyStatus(s.currentMa),
            powerSupplyHealth = LcmBatteryState.POWER_SUPPLY_HEALTH_UNKNOWN,
            powerSupplyTechnology = LcmBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
            // A charge reading is evidence of a battery. `false` would contradict it.
            present = true,
            cellVoltage = s.cellVoltagesMv?.map { it / 1000f } ?: emptyList(),
            // DJI reports one pack temperature, not per-cell; an empty list is the honest length.
            cellTemperature = emptyList(),
        )
    }

    /** DJI's signed milliamps → ROS's charging/discharging enum. Null current claims nothing. */
    private fun powerSupplyStatus(currentMa: Int?): Byte = when {
        currentMa == null -> LcmBatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        currentMa < 0 -> LcmBatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        currentMa > 0 -> LcmBatteryState.POWER_SUPPLY_STATUS_CHARGING
        else -> LcmBatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
    }

    // ── mode and gimbal ───────────────────────────────────────────────────────

    /**
     * The `mode` channel — **DJI's own flight-mode name, verbatim** (`GPS_ATTI`, `JOYSTICK`,
     * `GO_HOME`), or null when DJI has not said.
     *
     * Deliberately not a translation into PX4's vocabulary the way `Px4Mode` is for the MAVLink
     * wire: the DJI word is the one that appears in DJI's logs, DJI's forums and this project's
     * flight records, which makes it the searchable one. A blank name is treated as no name —
     * an empty `std_msgs.String` is indistinguishable from a mode called "".
     *
     * No freshness gate: `KeyFCFlightMode` is event-driven, so an hour of silence means the mode
     * has not changed. The channel is "on change, ≥1 Hz" in `docs/zenoh-topics.md`, which is the
     * publisher's business rather than this encoder's.
     */
    fun flightModeOrNull(s: AircraftState): String? = s.flightMode?.takeIf { it.isNotBlank() }

    /**
     * The `gimbal` channel — the gimbal's attitude in **degrees**, or null without a pitch.
     *
     * `x` is pitch and **it is the only axis this airframe means anything by**; the Mini 4 Pro's
     * gimbal has no usable yaw or roll, and the inbound `gimbal_cmd` contract refuses a non-zero
     * `y`/`z` for that reason. They are still carried here as *reported* rather than written to
     * zero, so a future airframe with a real yaw axis is not silently flattened, and `NaN` when
     * DJI has not reported them, because 0° is a real angle.
     *
     * Degrees, not radians, matching `gimbal_cmd`'s *"x is pitch in degrees"*. A bare `Vector3`
     * carries no units of its own and ROS would ordinarily imply radians, so the two directions
     * being spelled the same way in `docs/zenoh-topics.md` is what settles it.
     *
     * Takes loose angles rather than a gimbal reading type: the gimbal state lives behind
     * `gimbal/`'s own seam, and this encoder deliberately imports nothing from it.
     */
    fun gimbalAttitudeOrNull(
        pitchDeg: Double?,
        rollDeg: Double? = null,
        yawDeg: Double? = null,
    ): LcmVector3? {
        val pitch = pitchDeg?.takeIf { it.isFinite() } ?: return null
        return LcmVector3(
            x = pitch,
            y = rollDeg?.takeIf { it.isFinite() } ?: Double.NaN,
            z = yawDeg?.takeIf { it.isFinite() } ?: Double.NaN,
        )
    }

    // ── 8. the frame tree ─────────────────────────────────────────────────────

    /**
     * `drone/base_link` → `drone/camera`, in metres. **Measured with a ruler, 2026-07-27.**
     *
     * Ivan: *"camera sits 8 cm forward from origin"*. Small enough to have been tempting to call
     * zero and wrong enough to matter — at 20 m it is nothing, at the 1 m an AprilTag landing ends
     * in it is roughly 8 % of the error budget.
     *
     * **The vertical component has never been measured and is published as zero.** The Mini 4
     * Pro's gimbal hangs at the nose and slightly below the body, so a downward term almost
     * certainly exists; on a nadir camera it lies *along* the viewing axis, where it biases range
     * directly rather than laterally. `docs/mem2-converter.md` §2.2 lists it as owed, together
     * with the prior question of where `drone/base_link`'s origin is — this takes it as the body
     * centre, which is ROS's convention and nobody's measurement.
     */
    val CAMERA_OFFSET_M = LcmVector3(x = 0.08, y = 0.0, z = 0.0)

    /**
     * `drone/camera` → `drone/camera_optical`: **exact, constant, and free.**
     *
     * The rotation from a body-style camera frame (x forward, y left, z up) into the optical
     * convention (x right, y down, z into the image). It needs no measurement and no reading —
     * it is a property of how the two frames are defined, and the quaternion's components are
     * exactly ±½ rather than the output of a decomposition.
     *
     * It earns its own edge for one reason. Foxglove defines `CompressedVideo.frame_id` as
     * *"the origin of the frame is the optical center of the camera. +x points to the right in
     * the video, +y points down, and +z points into the plane of the video"* — so publishing this
     * rotation and stamping the video with the leaf makes the video's frame match the schema's own
     * definition **by construction rather than by our assertion**. `ZenohFramesTest` asserts the
     * three axes rather than the four numbers, because the numbers are the derivation and the axes
     * are the claim.
     */
    val OPTICAL_ROTATION = LcmQuaternion(x = 0.5, y = -0.5, z = 0.5, w = -0.5)

    /**
     * **`seq = 1` on a `TransformStamped`'s header, and it is DiMOS's default rather than ours.**
     *
     * Every other header this encoder writes carries `seq = 0`, and
     * `LcmHeader`'s KDoc says why that is honest: nothing in the catalogue reads the field, so a
     * constant is as good as a counter. `tf` is the one exception, and it was **found by the
     * cross-check rather than by reading** — `tools/zenohparity/check` reported all twelve planted
     * trees differing at byte 15 and nowhere else.
     *
     * The cause is upstream: `dimos.msgs.geometry_msgs.Transform` builds its header as
     * `Header(self.ts, self.frame_id)`, and that overload of `dimos.msgs.std_msgs.Header.__init__`
     * declares `seq: int = 1`. So a `Transform` DiMOS publishes itself carries 1, and so does every
     * `tf` row `tools/memexport` writes — while `PoseStamped`, `Imu`, `NavSatFix`, `BatteryState`
     * and `CameraInfo` all come out 0, measured.
     *
     * **Matching it is the point rather than a concession.** `docs/zenoh-replay-contract.md`'s whole
     * premise is that *"a subscriber must not be able to tell which publisher it is talking to"*, and
     * the only way to be indistinguishable from a `Transform` DiMOS produced is to carry the value
     * DiMOS produces. Writing 0 here would make our trees the odd ones on a shared TF tree, on a
     * field nobody reads, for the sake of an internal consistency no consumer can observe.
     *
     * This is the same shape of finding as `odom_1`-versus-`local_N` in that contract's §9: the wire
     * settles this kind of question and the documents do not.
     */
    const val TF_HEADER_SEQ: Int = 1

    /** Hamilton product, `(x, y, z, w)` in and out. `a ⊗ b` is "rotate by b, then by a". */
    fun qMul(a: LcmQuaternion, b: LcmQuaternion): LcmQuaternion = LcmQuaternion(
        x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )

    /** The inverse of a unit quaternion. */
    fun qConjugate(q: LcmQuaternion): LcmQuaternion = LcmQuaternion(-q.x, -q.y, -q.z, q.w)

    /**
     * `drone/base_link` → `drone/camera` — **a joint rotation, and not DJI's number.**
     *
     * This is the subtlest arithmetic in the whole catalogue: the one place where writing down the
     * obvious value gives a wrong answer, and where the argument is carried by measurement rather
     * than by convention.
     *
     * **DJI reports the gimbal earth-referenced.** Commanded to −90° the camera points at the
     * ground *regardless of how the aircraft is leaning*, so the angle is measured from the horizon
     * and not from the airframe. That is measured, not assumed: over 271 `gimbal` lines of the
     * 2026-07-27 mission the reported roll took only the values {−0.1, 0, 0.4}° and was non-zero
     * on 3 of 244 valued lines, **while the aircraft rolled to 22.8°**. A body-relative report
     * would have counter-rotated through every one of those degrees. Yaw says the same differently:
     * gimbal yaw tracks absolute heading with a stable offset rather than sitting near zero.
     *
     * An edge between two *body* frames is not an earth-referenced angle, so the airframe's own
     * attitude has to be composed away:
     *
     * ```
     * R(base_link → camera) = R(world → base_link)⁻¹ · R(world → camera)
     * ```
     *
     * Writing the commanded pitch straight in is wrong by the airframe's tilt. Measured over 525
     * in-flight samples: |roll| median 3.6°, p90 6.5°, **max 31.7°**; |pitch| median 2.0°, p90
     * 6.8°, max 27.3°. At a 1 m tag range that is **6 cm typically and 60 cm at worst** — larger
     * than the unmeasured translation above.
     *
     * **Two consequences that must not be conflated.** In the earth-referenced *report* the roll is
     * zero, and that is a property of this airframe: the Mini 4 Pro's gimbal has no usable
     * independent yaw or roll axis, yaw follows the nose and roll is stabilised level. In the
     * *edge* the roll is approximately minus the aircraft's roll, because that is exactly what the
     * stabiliser is doing. "Roll and yaw are zero" is true of the camera's attitude in the world
     * and false of the joint.
     *
     * @param worldFromBase the airframe's own ENU orientation — [orientationOrNull]'s output, and
     *   the reason this edge inherits the attitude's holding rule rather than having one of its
     *   own.
     * @param gimbal the earth-referenced camera attitude in DJI's own convention, degrees.
     */
    fun cameraEdge(worldFromBase: LcmQuaternion, gimbal: GimbalEarthAttitude): LcmQuaternion =
        qMul(
            qConjugate(worldFromBase),
            enuQuaternion(gimbal.rollDeg, gimbal.pitchDeg, gimbal.yawDeg),
        )

    /**
     * The whole tree at one instant, or null when the aircraft's own edge cannot be built.
     *
     * **Which edges, and what their absence means.** `world` → `base_link` needs a position and an
     * attitude under the same held-under-liveness rule `pose` follows, and when it is missing there
     * is no tree to publish at all — a `camera` edge hanging off a `base_link` nobody can locate
     * says nothing. `base_link` → `camera` is present only when a gimbal angle is available, and
     * its absence is honest: the camera is pointing somewhere and we cannot say where.
     * `camera` → `camera_optical` is in **every** message, because it is constant and free and
     * *"once"* has no meaning on a bus a subscriber joins at any moment — 100 bytes a second
     * against a late joiner being unable to resolve the video's own frame.
     *
     * Every `TransformStamped` carries the same [stamp]. They are a snapshot of one tree at one
     * time, not three independently aged facts.
     */
    fun tfOrNull(
        s: AircraftState,
        datum: OdomDatum,
        stamp: LcmTime,
        gimbal: GimbalEarthAttitude? = null,
        gate: Gate = Gate.HELD,
    ): LcmTfMessage? {
        val position = localPositionOrNull(s, datum, gate) ?: return null
        val orientation = orientationOrNull(s, gate) ?: return null
        val edges = ArrayList<LcmTransformStamped>(3)
        edges += LcmTransformStamped(
            header = LcmHeader(seq = TF_HEADER_SEQ, stamp = stamp, frameId = FRAME_WORLD),
            childFrameId = FRAME_BASE_LINK,
            transform = LcmTransform(
                translation = LcmVector3(position.x, position.y, position.z),
                rotation = orientation,
            ),
        )
        if (gimbal != null) {
            edges += LcmTransformStamped(
                header = LcmHeader(seq = TF_HEADER_SEQ, stamp = stamp, frameId = FRAME_BASE_LINK),
                childFrameId = FRAME_CAMERA,
                transform = LcmTransform(
                    translation = CAMERA_OFFSET_M,
                    rotation = cameraEdge(orientation, gimbal),
                ),
            )
        }
        edges += LcmTransformStamped(
            header = LcmHeader(seq = TF_HEADER_SEQ, stamp = stamp, frameId = FRAME_CAMERA),
            childFrameId = FRAME_CAMERA_OPTICAL,
            transform = LcmTransform(
                translation = LcmVector3.ZERO,
                rotation = OPTICAL_ROTATION,
            ),
        )
        return LcmTfMessage(edges)
    }

    // ── 9. the camera, as far as anything has actually measured it ─────────────

    /**
     * **`f / W` — a fitted focal length, not a calibration**, and the difference is the whole
     * reason this stream exists.
     *
     * A consumer holding video and no intrinsics has to invent them, and the obvious invention is
     * DJI's published 82.1° diagonal field of view — which implies `f = 0.659 · W` and is
     * **13.2 % wrong for this stream**. A range computed on the spec figure under-reads by 13 %:
     * at a true 3 m it says 2.6 m.
     *
     * `docs/measurements/2026-07-27-tag-detection-rate.md` §3 fitted the focal length against
     * recorded altitude on two independent flights and got **1465.9 px** and **1448.3 px** at 1920
     * wide — reproducing to 1.2 %, while the *intercept* of the same fit did not reproduce at all
     * (+0.328 m against −0.063 m), because that is the height of the board the tag stood on and
     * the two sessions were set up on different days. **Slope agreeing while intercept disagrees
     * is the evidence the fit separated a camera error from a scene error** rather than trading
     * one off against the other.
     *
     * Expressed as a ratio to the width rather than as pixels so the intrinsics follow a
     * resolution change instead of being pinned to one.
     */
    const val FOCAL_OVER_WIDTH: Double = 0.75890625

    /** The two fits, at 1920 wide, kept so the constant above is checkable by hand. */
    val FOCAL_FITS_PX_AT_1920: List<Double> = listOf(1465.9, 1448.3)

    /** The width the two fits were taken at. */
    const val FOCAL_FIT_WIDTH: Double = 1920.0

    /**
     * What DJI's published 82.1° diagonal implies. **Never published** — carried only so a test
     * can show the two are far apart, and so the number a consumer would otherwise reach for has
     * a name.
     */
    const val FOCAL_OVER_WIDTH_FROM_SPEC: Double = 0.659

    /**
     * `sensor_msgs.CameraInfo` in the **optical** frame, with no distortion claim.
     *
     * ## The frame
     *
     * `header.frame_id` is [FRAME_CAMERA_OPTICAL] and not [FRAME_CAMERA]. The projection this
     * matrix describes is expressed in the optical convention — z forward along the optical axis,
     * x right, y down — which is the leaf `CompressedVideo` already stamps itself with. Naming the
     * parent body frame looks fine right up until someone composes a transform with it.
     *
     * ## What each field claims, and what it does not
     *
     * | field | value | why |
     * |---|---|---|
     * | `K` | `fx = fy = 0.7589 · width`, principal point at the image centre | the fit's mean, scaled by width. **`fx == fy` assumes square pixels and the centre is assumed; nothing has measured either** |
     * | `R` | identity | *exact*, not an assumption: an unrectified monocular camera's rectification is the identity |
     * | `P` | `K` with a zero fourth column | derived from `K`, not a second claim |
     * | `D` | **empty** | below |
     * | `distortion_model` | **empty string** | below |
     * | `binning`, `roi` | zero | nothing crops and nothing bins |
     *
     * ## Distortion is unmeasured, and is published as an absence
     *
     * Three candidate representations, and the two rejections matter as much as the choice:
     *
     * - **Not five zeros with `plumb_bob`.** That is an affirmative claim that this lens is
     *   rectilinear, and nothing has measured that. A 66.8° lens will have real barrel distortion.
     * - **Not `NaN`**, which is this project's sentinel for *"there is no feed at all"* (§4 of the
     *   class doc). NaN here would poison every projection **including the focal length we do
     *   have**, and handing that focal length over is the entire point of the message.
     * - **An empty `D` with an empty `distortion_model`** — ROS's own "no model is offered",
     *   expressible on the wire because `D` is length-prefixed, and it forces a consumer that
     *   wants to assume zero distortion to make that assumption itself, visibly.
     *
     * ## The one place this could be read as claiming more than it has
     *
     * ROS documents `K[0] == 0` as the flag for an uncalibrated camera, and `K[0]` here is not
     * zero — because we do have a usable focal length and zeroing it would throw away the
     * correction that is the reason for the stream. **This is a two-flight fit against a
     * 0.1 m-quantised altitude over ranges of 0.6–5.2 m, with no chessboard and no distortion
     * term.** `vision/TagSighting.Sighting.metric` stays false for exactly the same reason, and a
     * `CameraInfo` that looked authoritative would be the plausible-artifact failure this project
     * keeps meeting.
     *
     * @param width from the video stream's own `StreamInfo`, **never assumed**. A resolution
     *   change invalidates the intrinsics, so the caller holds the last stated one and republishes
     *   immediately on a change rather than averaging two matrices into one.
     */
    fun cameraInfo(width: Int, height: Int, stamp: LcmTime): LcmCameraInfo? {
        if (width <= 0 || height <= 0) return null
        val f = FOCAL_OVER_WIDTH * width
        val cx = width / 2.0
        val cy = height / 2.0
        return LcmCameraInfo(
            header = LcmHeader(stamp = stamp, frameId = FRAME_CAMERA_OPTICAL),
            height = height,
            width = width,
            distortionModel = "",
            d = emptyList(),
            k = listOf(f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0),
            r = listOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            p = listOf(f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0),
        )
    }

    // ── 8. the tag detector ───────────────────────────────────────────────────

    /**
     * The one AprilTag family `android/app/src/apriltag/tag_jni.c:68` creates.
     *
     * Pinned here rather than read from `vision/AprilTagDetector` on purpose: that class loads
     * `libapriltagjni.so` in its companion, and a JVM unit test that touched it would fail on a
     * desktop for a reason that has nothing to do with encoding. It is the same discipline as
     * [LcmFingerprints] — a constant with its provenance in the comment, checked by a test.
     */
    const val TAG_FAMILY = "tag36h11"

    /**
     * **An orientation that was not solved, or did not deserve belief** — the message's state
     * whenever [com.dimensional.mini4pro.vision.TagPose.trusted] returns null.
     *
     * `NaN` on all four components, by §4's line: not a reading that went missing but a quantity
     * there is no feed for on this frame, so there is nothing to wait for and nothing to
     * withhold *for*. The two plausible alternatives are both claims:
     *
     *  - **The identity quaternion** says the tag's axes are the camera's — that the marker is
     *    square-on to the image and unrotated in it. It is wrong on essentially every frame and
     *    it is wrong *invisibly*, because it is exactly what a consumer expects to see.
     *  - **`(0,0,0,0)`** is not a rotation at all, which is honest, but it is also the value tf2
     *    rejects with a message about a malformed transform — a complaint about our encoding
     *    rather than about our knowledge.
     *
     * Until 2026-07-28 this was every frame's orientation — no corner set crossed the JNI
     * boundary, so nothing could solve one. The solve exists now (`vision/TagPoseSolve`,
     * apriltag's own estimator), and this value is what the gates degrade to: a solve that is
     * absent, partial, too small ([TagPose.MIN_SOLVE_PIXELS]) or ambiguous
     * ([TagPose.MAX_AMBIGUITY_RATIO]) publishes **exactly the pre-solve message**, never a
     * best-effort orientation. The gates' evidence:
     * `docs/measurements/2026-07-28-pose-solve-stability.md`.
     */
    val UNSOLVED_ORIENTATION = LcmQuaternion(Double.NaN, Double.NaN, Double.NaN, Double.NaN)

    /**
     * **A `BoundingBox3D` refused entire.** NaN centre, NaN orientation, NaN size.
     *
     * `BoundingBox3D` is a `Pose` and a `Vector3` and there is no way in it to say "no box". A
     * `size` of `(0,0,0)` claims the object is a point, which is the confident-zero this project
     * refuses; and a box needs an orientation, which [UNSOLVED_ORIENTATION] explains we do not
     * have on an ungated frame.
     *
     * The **centre** is NaN too, and that is the part worth arguing. The tag's position *is*
     * known and it would have been easy to put it here — but a `bbox.center` beside a NaN `size`
     * invites a reader to treat the box as the detection, and the pose belongs in `results[0]`
     * where the schema puts it and where the covariance and the score that qualify it also live.
     * One field, one answer.
     *
     * Since 2026-07-28 a solve that passes both gates fills the box instead — see [solvedBox] —
     * and this is what every gate failure degrades to, whole: a box is either fully solved or
     * fully refused, never a real centre with a NaN size.
     */
    val UNSOLVED_BOX = LcmBoundingBox3D(
        center = LcmPose(
            position = LcmPoint(Double.NaN, Double.NaN, Double.NaN),
            orientation = UNSOLVED_ORIENTATION,
        ),
        size = UNMEASURED_VECTOR3,
    )

    /**
     * The bounding box of a **trusted** solve: the solved pose at its centre, and a size that
     * says what a tag is — `(tagSize, tagSize, 0)`. A flat square. The zero z-extent is a
     * *measurement of the object*, not the refused-zero this project guards against: the marker
     * genuinely has no thickness worth claiming, and NaN here would un-solve a box whose whole
     * point is that it was solved.
     *
     * The centre is the **estimator's own translation**, which is a second opinion beside
     * `results[0].pose.position` (the apparent-size ray) — deliberately not forced equal.
     * The two agree to millimetres on the measured flight (same focal, same corners) and
     * publishing both is what lets a consumer see when they stop agreeing.
     */
    fun solvedBox(solve: com.dimensional.mini4pro.vision.TagPoseSolve): LcmBoundingBox3D =
        LcmBoundingBox3D(
            center = LcmPose(
                position = LcmPoint(solve.tx, solve.ty, solve.tz),
                orientation = LcmQuaternion(solve.qx, solve.qy, solve.qz, solve.qw),
            ),
            size = LcmVector3(solve.tagSizeM, solve.tagSizeM, 0.0),
        )

    /**
     * One sighting as a `vision_msgs.Detection3D`, or **null when the pose would be three zeros**.
     *
     * **The element, not the message.** [detectionsOrNull] wraps this in the
     * `vision_msgs.Detection3DArray` the channel carries. The wrapping is four lines and is argued
     * there; everything about *what a detection claims* is here, and the two fixtures pin both.
     *
     * ## The frame, and the one substitution that would have been silent
     *
     * `header.frame_id` is [FRAME_CAMERA_OPTICAL], so the position is referred to the **principal
     * point** and to the optical axes. That is what [TagSighting.Sighting] already carries:
     * `TagRecogniser.publish` builds `x, y, z` from `TagWorld.cameraFrame`, which measures from
     * the image centre standing in for the principal point.
     *
     * `TagWorld` has a second reference point — `nadirPointX/Y`, the **measured** pixel a point
     * directly beneath the aircraft appears at, 2.99° from the image centre — and it is used by
     * `TagWorld.fix` because that function scales a pixel offset by altitude and needs a ray that
     * really points down. **It must not be used here.** A vector measured from the nadir pixel and
     * stamped `camera_optical` is a nadir-referred vector wearing an optical-frame label, and
     * nothing downstream could detect the substitution: it is the right magnitude, in the right
     * units, 2.99° in the wrong direction. The world-frame answer is `TagFix`'s and reaches the
     * bus, if it ever does, on a `tf`-relative channel and not on this one.
     *
     * ## Field by field
     *
     * | field | what goes in it |
     * |---|---|
     * | `header.stamp` | the **frame's arrival**, on the wall clock, never the send time (D-5) |
     * | `header.seq` | a count of detections published this session |
     * | `header.frame_id` | `drone/camera_optical` — see above |
     * | `results[0].hypothesis.class_id` | `tag36h11:<id>`. Family *and* code: tag 7 of one family is not tag 7 of another |
     * | `results[0].hypothesis.score` | `decision_margin`, raw and uncalibrated. See below |
     * | `results[0].pose.pose.position` | metres, optical frame, from `TagWorld.cameraFrame` — **never** the solve's translation, so history stays comparable |
     * | `results[0].pose.pose.orientation` | the **gated** solve (`TagPose.trusted`), else [UNSOLVED_ORIENTATION] |
     * | `results[0].pose.covariance` | [ZERO_COVARIANCE_36] — ROS's own documented "unknown" |
     * | `bbox` | [solvedBox] when the same gate passes, else [UNSOLVED_BOX] — never a mixture |
     * | `id` | the bare label, [detectionId] — no suffixes since 2026-07-29 (Ivan). See below |
     *
     * **`score` is the decision margin and not the hamming distance**, and the two were the only
     * candidates (`docs/tag-detector.md` §7). `hamming` is the count of bit errors the decoder
     * corrected, `0..maxhamming`, and **lower is better** — writing it into a field ROS documents
     * as a confidence inverts the sense, so a consumer thresholding `score > x` would keep the
     * worse decodes and drop the clean ones. `decision_margin` is apriltag's own confidence in
     * this decode and is the only continuous number of the two. It is **not a probability**: this
     * project thresholds nothing on it and has measured no cut-off, which `vision/TagDetection`
     * says at the field. It is forwarded, not interpreted.
     *
     * **`hamming` and `pixelSize` are refused rather than folded in.** `Detection3D` has no
     * column for either and there is no honest place to put them — a class is not a per-detection
     * count, and an id is not a size. Both are on every `LogEntry.Tag` line of the flight record,
     * which is where per-detection evidence lives. A future `Detection3DArray` with a second
     * hypothesis would be worse, not better: two hypotheses means two candidate objects.
     *
     * **`id` is the bare label** — `tag36h11:<id>` — since 2026-07-29, by Ivan's decision;
     * [detectionId] carries the full story of what each deleted suffix was replaced by. The
     * caveat those suffixes stated is unchanged and now lives in the contract row: the range
     * still comes from the tag's apparent size through `TagWorld.rangeFromSize` (*"a much less
     * reliable range"* than the aircraft's altitude, *"for cross-checking rather than for
     * flying on"*), and the pose still rests on a fitted focal length and an assumed principal
     * point. A consumer that needs per-message belief provenance reads `tag_fix`.
     *
     * ## What is refused
     *
     * Null when the frame stated no geometry, and null when the position is not finite or `z` is
     * not positive. `TagRecogniser.publish` falls back to a range of `0.0` when
     * `TagWorld.rangeFromSize` declines, and `TagWorld.cameraFrame` then returns `(0, 0, 0)` — a
     * tag at the camera's optical centre, which is a place, and a confidently wrong one. Silence
     * is the honest encoding of "the size implied no range", exactly as §4 draws the line.
     *
     * @param seq `header.seq`. A counter, per the caller; nothing in `docs/zenoh-topics.md` reads
     *   it, so this is diagnostic and not a contract.
     */
    fun detectionOrNull(
        sighting: TagSighting.Sighting,
        stamp: LcmTime,
        seq: Int,
    ): LcmDetection3D? {
        if (sighting.imageWidth <= 0 || sighting.imageHeight <= 0) return null
        if (!sighting.x.isFinite() || !sighting.y.isFinite()) return null
        // Not merely finite: `z` **is** the range along the optical axis, and a range of zero is
        // the value `TagRecogniser` substitutes when the tag's apparent size implied none.
        if (!sighting.z.isFinite() || sighting.z <= 0.0) return null
        // **The gates, and the one place a solved orientation can reach the bus.** `trusted`
        // returns the solve only when it is whole (no partial claim), large enough
        // (MIN_SOLVE_PIXELS) and unambiguous (MAX_AMBIGUITY_RATIO) — measured bounds,
        // `docs/measurements/2026-07-28-pose-solve-stability.md`. Null degrades every solved
        // field at once to exactly the pre-solve message: NaN orientation, refused box, plain
        // id. The raw solve is deliberately not consulted anywhere below this line.
        val trusted = com.dimensional.mini4pro.vision.TagPose.trusted(
            sighting.pixelSize, sighting.solve,
        )
        return LcmDetection3D(
            header = LcmHeader(seq = seq, stamp = stamp, frameId = FRAME_CAMERA_OPTICAL),
            results = listOf(
                LcmObjectHypothesisWithPose(
                    hypothesis = LcmObjectHypothesis(
                        classId = "$TAG_FAMILY:${sighting.tagId}",
                        score = sighting.decisionMargin,
                    ),
                    pose = LcmPoseWithCovariance(
                        pose = LcmPose(
                            // **Unchanged by the solve, deliberately.** This stays the
                            // centre-ray × apparent-size range it has been since the channel
                            // shipped, so every existing consumer and every recorded flight
                            // stays comparable. The solve's own translation is in the bbox.
                            position = LcmPoint(sighting.x, sighting.y, sighting.z),
                            orientation = trusted
                                ?.let { LcmQuaternion(it.qx, it.qy, it.qz, it.qw) }
                                ?: UNSOLVED_ORIENTATION,
                        ),
                        // ROS's own documented flag for an unknown covariance, and the same
                        // choice `odom` and `pose` already make. Nothing here has measured the
                        // pose's error: the focal length reproduces to 1.2 % across two flights
                        // and that is a fit's spread, not a covariance.
                        covariance = ZERO_COVARIANCE_36,
                    ),
                ),
            ),
            bbox = trusted?.let(::solvedBox) ?: UNSOLVED_BOX,
            id = detectionId(sighting),
        )
    }

    /**
     * One sighting as the **`vision_msgs.Detection3DArray` that goes on the wire**, or null when
     * [detectionOrNull] refuses.
     *
     * The channel's type since 2026-07-28 (Ivan): ROS's own convention for a detection topic, and
     * what DiMOS consumes natively. `detectionOrNull` still builds the element and still carries
     * the whole field-by-field argument; this only wraps it.
     *
     * ## The array has one element, and that is not a promise
     *
     * It is a fact about the **recogniser**, not about this contract. `TagRecogniser.publish`
     * takes `found.largest` and discards the rest of the frame before anything downstream sees it
     * — size being the only ranking available without a pose — so by the time a sighting reaches
     * `RecordedTagSink` the frame has already been reduced to one tag. A consumer that assumed a
     * single element would be resting on that reduction, which lives two packages away and exists
     * for reasons that have nothing to do with the bus. The type says array because the truthful
     * statement is *"these are the detections in this frame"*, and one is how many there are.
     *
     * This is also why the type is an array rather than the channel simply publishing twice: the
     * `tf` channel's argument, unchanged — one message per instant, because a consumer that has to
     * re-join across messages cannot tell one frame's detections from two frames'.
     *
     * ## The header restates the element's, and does not invent an envelope
     *
     * Same `seq`, same `stamp`, same `frame_id`. There is nothing else it could honestly be: with
     * one element there is one arrival time and one frame, and a second stamp taken from a clock
     * here would be the send time — which is exactly what D-5 forbids and what
     * `TagSighting.Sighting.ageMillisAt` exists to keep out of the message. If the array ever
     * carries detections from more than one frame this stops being right, and it will stop being
     * right *loudly*, because the elements will disagree with it.
     */
    fun detectionsOrNull(
        sighting: TagSighting.Sighting,
        stamp: LcmTime,
        seq: Int,
    ): LcmDetection3DArray? {
        val detection = detectionOrNull(sighting, stamp, seq) ?: return null
        return LcmDetection3DArray(header = detection.header, detections = listOf(detection))
    }

    /**
     * `Detection3D.id`: **the label, bare** — `tag36h11:<id>`, family and code, nothing else.
     *
     * **The suffix stuffing is gone, and that is Ivan's decision** (2026-07-29: *"we
     * shouldn't add all this stuff into the label… no need for range=…, solved=…"*),
     * reversing the 2026-07-28 convention that rode `;metric=false;range=apparent_size` and
     * `;pose=solved` on every message. What replaces each piece, honestly:
     *
     *  - **`pose=solved` was redundant with the message's own structure and remains fully
     *    carried**: the orientation and the box are [UNSOLVED_ORIENTATION]/[UNSOLVED_BOX] —
     *    NaN — exactly when the gates fail, and real values exactly when they pass. That NaN
     *    convention is now the *only* carrier of the solved fact, so it is load-bearing
     *    rather than belt-and-braces, and the tests that pin it pin the contract.
     *  - **`metric=false` and `range=apparent_size` stop travelling on this channel.** The
     *    caveat itself is unchanged — the pose still rests on a fitted focal length and an
     *    assumed principal point — but it now lives in the contract row
     *    (`docs/zenoh-topics.md`) rather than in every message. A consumer that needs
     *    belief-grade provenance per message reads the world-frame `tag_fix` channel, whose
     *    id carries the range-ladder rung.
     *
     * Still no number formatted into it (`String.format` is locale-sensitive and this phone
     * runs in Greece), and still stable per tag, so it still serves as vision_msgs' intended
     * tracking handle — better, in fact, now that it is only that.
     */
    fun detectionId(sighting: TagSighting.Sighting): String = "$TAG_FAMILY:${sighting.tagId}"

    // ── 9a. the tag's world-frame fix — the `tag_fix` channel ─────────────────

    /**
     * **The tag's z in `drone/world`: zero, and it is an assumption with a name, not a
     * measurement.** `TagWorld.fix` assumes the tag lies on the datum plane — the whole
     * bearing×altitude construction rests on it, and `TagFix.fromHeightM`'s KDoc tells the
     * story: the 25 Hz flight's tag stood on a board measured at +0.33 m *by accident*, and
     * nothing has measured a tag's height since. Every message carries `z=datum_plane` in its
     * `id` so the zero cannot be read as a survey. The alternatives are both worse: a z from
     * the baro or the solved range would *look* measured (the fabrication this project
     * refuses), and NaN would deny the assumption the lateral numbers already rest on.
     */
    const val TAG_ON_DATUM_PLANE_Z: Double = 0.0

    /**
     * One `TagWorld.fix` as the `vision_msgs.Detection3DArray` the `tag_fix` channel carries,
     * or null when the sighting produced no fix.
     *
     * ## Why this channel exists — the composition artifact, named
     *
     * Ivan, watching mem2 replays (2026-07-29): the camera-frame `detections`, composed by a
     * consumer against `tf` edges of a different instant, make the tag *ride the camera* —
     * camera up, tag up — and with detections sparser than poses a descending body drags the
     * last detection into the floor. Both are join artifacts, and the app already owns the
     * correct join: `TagWorld.fix` composes each sighting with the aircraft pose **at the
     * sighting's own instant**. This function publishes that fix — **the record's own `n`/`e`
     * values, never a re-derivation**: re-composing here from the camera-frame pose and a
     * current aircraft state would reintroduce at the source exactly the artifact the channel
     * exists to kill, and would put numbers on the wire the `tag` line cannot reproduce.
     *
     * ## Frame, and the one caveat a consumer must hold
     *
     * `frame_id` is [FRAME_WORLD], and the fix's numbers are anchored to **DJI's home point**,
     * which DJI re-records at every takeoff at the then-current wandering GPS reading, while
     * `drone/world`'s datum is latched once per session. Measured on landing10 (5 takeoffs):
     * the home sits 0.41–1.37 m from the session datum. That offset is *not* corrected here —
     * correcting it would be a state join the tag line alone cannot reproduce — and it is the
     * same class of GPS-anchoring error `drone/world` already carries (absolute accuracy
     * 1–2 m, "the final descent cannot close on this frame; that is what the AprilTag is
     * for"). This channel is for putting a belief on a map beside the poses, not for landing.
     *
     * ## Field by field, against [detectionOrNull]'s precedent
     *
     * | field | what goes in it |
     * |---|---|
     * | `header` | `seq` counts published fixes, `stamp` the **frame's arrival** (D-5), `frame_id` `drone/world` |
     * | `results[0].hypothesis.class_id` | `tag36h11:<id>` — same vocabulary as `detections` |
     * | `results[0].hypothesis.score` | the sighting's `decision_margin`, **quantised to the record's own 1 dp** so the wire never carries a digit the `tag` line cannot reproduce |
     * | `results[0].pose.pose.position` | ENU: `x = eastM`, `y = northM`, `z =` [TAG_ON_DATUM_PLANE_Z] — the record's fix, at the record's 3 dp |
     * | `results[0].pose.pose.orientation` | [UNSOLVED_ORIENTATION], always: no world-frame tag orientation has ever been derived — the solved orientation is optical-frame and lives on `detections`; composing it out here would be the forbidden re-derivation |
     * | `results[0].pose.covariance` | zeros — ROS's "unknown" |
     * | `bbox` | [UNSOLVED_BOX], always — a world box needs a world orientation |
     * | `id` | the provenance, [tagFixId] — the part of this message a consumer must not lose |
     *
     * ## What is refused
     *
     * Null exactly when the record's line has no fix (`n`/`e` absent — no position, no
     * heading, no believed camera angle, or a camera not near nadir at the frame) or a
     * non-finite one. The sighting itself still travels on `detections`; what does not travel
     * is a place we do not have.
     *
     * @param rangeSource the ladder rung off the record (`solve`/`size`/`baro`), or null on a
     *   line that predates the field — [tagFixId] then falls back to what [fixMetric] alone
     *   can vouch for.
     */
    fun tagFixOrNull(
        tagId: Int,
        decisionMargin: Double,
        northM: Double?,
        eastM: Double?,
        fixMetric: Boolean,
        rangeSource: String?,
        pitchReported: Boolean,
        stamp: LcmTime,
        seq: Int,
    ): LcmDetection3DArray? {
        val north = northM?.takeIf { it.isFinite() } ?: return null
        val east = eastM?.takeIf { it.isFinite() } ?: return null
        val detection = LcmDetection3D(
            header = LcmHeader(seq = seq, stamp = stamp, frameId = FRAME_WORLD),
            results = listOf(
                LcmObjectHypothesisWithPose(
                    hypothesis = LcmObjectHypothesis(
                        classId = "$TAG_FAMILY:$tagId",
                        score = Json.roundTo(decisionMargin, 1),
                    ),
                    pose = LcmPoseWithCovariance(
                        pose = LcmPose(
                            // The record's own numbers, at the record's own precision
                            // (Json.num, 3 dp) — the byte-identity owner, as `setpoint`'s.
                            position = LcmPoint(
                                x = Json.roundTo(east, 3),
                                y = Json.roundTo(north, 3),
                                z = TAG_ON_DATUM_PLANE_Z,
                            ),
                            orientation = UNSOLVED_ORIENTATION,
                        ),
                        covariance = ZERO_COVARIANCE_36,
                    ),
                ),
            ),
            bbox = UNSOLVED_BOX,
            id = tagFixId(tagId, fixMetric, rangeSource, pitchReported),
        )
        return LcmDetection3DArray(header = detection.header, detections = listOf(detection))
    }

    /**
     * The `tag_fix` message's `id` — **minimal by decision, not by accident.**
     *
     * Ivan's bare-label rule for `detections` (2026-07-29) applies here too: no suffix
     * stuffing, typed fields preferred. But this schema has **no typed vehicle** for a
     * categorical belief grade — every non-string field is a number a value would lie in —
     * and landing07-B measured why the grade must travel per message: the barometer read
     * ~1.2 m wrong within a minute while the size range was right, so a `baro`-scaled and a
     * `solve`-scaled fix are *different beliefs* and a consumer must be able to tell. So the
     * id stays the vehicle, cut to the two qualifications that vary per message:
     *
     * ```
     * tag36h11:<id>[;range=solve|size|baro][;pitch=reported]
     * ```
     *
     *  - `range=` — the ladder rung that scaled this fix, from the record's `range_src`;
     *    `solve` on a pre-`range_src` line that `fix_metric` vouches for (that is the
     *    field's definition); **omitted** on a legacy bearing line — absent means
     *    *unrecorded*, never a guess.
     *  - `pitch=reported` — the camera pitch under this fix was DJI's reported angle rather
     *    than a commanded one; present only when true.
     *
     * What deliberately does **not** travel per message, because it does not vary per
     * message: `bearing=assumed` (true on every fix this project has ever made, until the
     * hover-translate manoeuvre measures the camera-to-body rotation — a channel-level
     * caveat, in the contract row) and `z=datum_plane` (every message's z is the same named
     * assumption, [TAG_ON_DATUM_PLANE_Z]). If either ever becomes per-message, its token is
     * additive here. No number is formatted in — the locale argument of [detectionId].
     */
    fun tagFixId(
        tagId: Int,
        fixMetric: Boolean,
        rangeSource: String?,
        pitchReported: Boolean,
    ): String {
        val rung = rangeSource ?: if (fixMetric) "solve" else null
        return "$TAG_FAMILY:$tagId" +
            (rung?.let { ";range=$it" } ?: "") +
            (if (pitchReported) ";pitch=reported" else "")
    }

    // ── 10. the commanded velocity — the `setpoint` channel ───────────────────

    /**
     * One virtual-stick send as the `geometry_msgs.TwistStamped` the `setpoint` channel
     * carries, or null when the setpoint cannot be expressed honestly.
     *
     * **`Twist` is Ivan's decision** (2026-07-29: *"we can actually publish twist messages,
     * that's the correct thing to publish"*); stamped, because on this bus every published
     * reading travels with its instant (D-5) and its frame, and a bare `Twist` can carry
     * neither.
     *
     * ## The record is the source of truth, and the quantisation is why
     *
     * This message must be reproducible **from the `stick_cmd` line alone** — the
     * replay-indistinguishability rule: `tools/memexport` rebuilds the same channel from the
     * record and the two must be byte-identical for the same flight. The record writes the
     * setpoint at [Setpoint.VELOCITY_DECIMALS]/[Setpoint.YAW_RATE_DECIMALS] digits
     * (`Json.num`'s half-up rule), so this encoder quantises through the *same* owner
     * ([Json.roundTo]) before converting. Skipping that would put full-precision doubles on
     * the live wire that no record-derived store could ever match — a divergence measured in
     * the last bits of every message, invisible until the cross-check runs. Quantising twice
     * is idempotent, which is what lets `tools/kotlinframes` feed already-rounded record
     * values back through this same function.
     *
     * ## The frame, and the two signs
     *
     * The setpoint is flown in world NED (`SetpointFrame.NED_VELOCITY` — DJI's `GROUND`
     * coordinate system); the bus speaks ENU (`docs/zenoh-topics.md`). One conversion, one
     * owner, two callers: `linear` crosses at [enuFromNed] exactly as every velocity on this
     * bus does, and `frame_id` is [FRAME_WORLD]. The yaw rate crosses on its own line:
     * DJI/NED yaw rate is **clockwise-positive in deg/s**, ROS's `angular.z` about ENU's
     * z-up is **counterclockwise-positive in rad/s**, so `angular.z = −toRadians(rate)` —
     * a negation *and* a unit change, both silent if dropped and both pinned by test.
     *
     * ## `angular.x`/`angular.y` are NaN, not zero
     *
     * The advanced virtual-stick surface commands four degrees of freedom; a roll or pitch
     * *rate* is not among them — there is no feed and no command, ever, which is §4's NaN
     * case, the same statement `odom`'s `twist.angular` and `imu` make. Zero would claim we
     * commanded the aircraft not to rotate about those axes, and no such command exists.
     *
     * ## What is refused
     *
     * Null for a frame other than `NED_VELOCITY` — today's engine only flies that frame, and
     * translating any other here would be a second guidance implementation hiding in a
     * transport. Null when any of the four values is absent or non-finite: a partial twist
     * with invented zeros is the confidently-wrong shape this project refuses everywhere.
     * The caller ([ZenohEmission.setpointReason]) names each refusal.
     *
     * @param stamp the send's own instant. The tap runs synchronously inside the send call,
     *   so unlike a detection there is no pipeline age to subtract — "now" at the tap IS the
     *   send time, to well under the wire's millisecond quantisation.
     */
    fun setpointOrNull(
        frame: String?,
        northMps: Double?,
        eastMps: Double?,
        downMps: Double?,
        yawRateDegPerS: Double?,
        stamp: LcmTime,
    ): LcmTwistStamped? {
        if (frame != SetpointFrame.NED_VELOCITY) return null
        val vn = northMps?.takeIf { it.isFinite() } ?: return null
        val ve = eastMps?.takeIf { it.isFinite() } ?: return null
        val vd = downMps?.takeIf { it.isFinite() } ?: return null
        val rate = yawRateDegPerS?.takeIf { it.isFinite() } ?: return null
        val qn = Json.roundTo(vn, Setpoint.VELOCITY_DECIMALS)
        val qe = Json.roundTo(ve, Setpoint.VELOCITY_DECIMALS)
        val qd = Json.roundTo(vd, Setpoint.VELOCITY_DECIMALS)
        val qr = Json.roundTo(rate, Setpoint.YAW_RATE_DECIMALS)
        return LcmTwistStamped(
            header = LcmHeader(stamp = stamp, frameId = FRAME_WORLD),
            twist = LcmTwist(
                linear = enuFromNed(north = qn, east = qe, down = qd),
                // NED clockwise-positive deg/s → ENU counterclockwise-positive rad/s.
                //
                // `Math.toRadians` is a single multiply by `DEGREES_TO_RADIANS`
                // (0.017453292519943295) — NOT `deg / 180.0 * PI`, which this comment claimed
                // until landing16 and which is a different double on some inputs (−84.54°:
                // bff79ba74d9e3f86 vs …85, measured on jdk17.0.20). `tools/memexport` mirrors
                // it with `math.radians`, whose constant is the same double; the golden bytes
                // live in `ZenohSetpointTest.a yaw rate the two radian spellings disagree on`.
                // Believing the comment instead of measuring cost 246 differing messages on
                // landing16 and 361 on landing17, invisible on every earlier flight because
                // they all commanded `yawrate: 0` and −0.0 is −0.0 under either expression.
                angular = LcmVector3(Double.NaN, Double.NaN, -Math.toRadians(qr)),
            ),
        )
    }

    /**
     * The `wind` channel's message, or null when there is nothing measured to say.
     *
     * ## The one conversion owner: the record's dm/s → the wire's m/s, here and nowhere else
     *
     * The input is **the record's own integer** — DJI's quantisation, not ours: `KeyWindSpeed`
     * is `DJIKeyInfo<Integer>` in dm/s (javap on the 5.18.0 jar; `docs/msdk/actions.md`), the
     * recorder writes it verbatim as `windSpeedDmS`, and the live tap hands this function the
     * same integer the line was written from. There is therefore no quantisation step to own —
     * the source is already discrete — only the division by ten, and it happens **once, in
     * double, then narrows**: `(dmS / 10.0).toFloat()`. `tools/memexport`'s Python mirrors the
     * exact expression (`Float32(dms / 10.0)` — a CPython float *is* a double, and the binding
     * packs `>f` with the identical IEEE round-to-nearest narrowing), which is what the
     * kotlinframes byte cross-check holds on every conversion, `ZenohSetpointTest`'s
     * record-vs-live rule reapplied to a channel where the record's rendering is an integer.
     * A ×10 error here is 0.91 m/s printed where 9.1 m/s blew the aircraft sideways
     * (landing14) — the byte pins in `ZenohWindTest` are that flight's own ramp.
     *
     * Null in means null out: a `windSpeedDmS` line with no value is DJI withdrawing the
     * reading, and zero is a calm day, not an unknown. No stamp parameter — `std_msgs.Float32`
     * carries none (see [LcmFloat32] for why that is honest here).
     */
    fun windOrNull(speedDmS: Int?): LcmFloat32? {
        val dms = speedDmS ?: return null
        return LcmFloat32((dms / 10.0).toFloat())
    }

    // ── warnings ──────────────────────────────────────────────────────────────

    /**
     * The `warnings` channel's message: **one decided warning, one `DiagnosticStatus`, no
     * opinions added.**
     *
     * Every field is copied from the [com.dimensional.mini4pro.warn.WarnEvent] the monitor
     * produced — including `level`, whose single owner is `warn/WarnLevel.diagnosticLevel` and
     * which is deliberately not recomputed here from the DJI word. A second mapping in this file
     * is exactly how a bus subscriber and a QGC operator come to disagree about how bad something
     * is, which is the failure the whole warning path is arranged against.
     *
     * `frame_id` is [FRAME_BASE_LINK]: a warning is about the aircraft, not about the world or the
     * camera. `hardware_id` carries DJI's component/sensor pair where the source has one and is
     * **empty rather than `"null/null"`** where it does not — LCM has no null string, and a
     * literal "null" on the wire is the unknown-is-never-zero failure spelled in ASCII.
     *
     * The `values` are the facts ROS's own fields have nowhere to put, each one DJI's word or DJI's
     * number, never a paraphrase:
     *
     *  - `source` — which DJI subsystem (`health`, `wind`);
     *  - `state` — DJI's own state name, verbatim (`WARNING`, `LEVEL_2`);
     *  - `level` — our ladder's name for it, so a consumer can see both halves of the translation;
     *  - `change` — `appeared` / `changed` / `cleared`, the edge itself;
     *  - `previous` — the level it came from, on a change only;
     *  - `measurement` — the number that belongs with it (`14.2 m/s`), when the source has one;
     *  - `forwarded` — whether the operator's ground station was actually told, so a replay can
     *    tell "the robot knew and the pilot did not" from "nobody knew".
     */
    fun warning(
        event: com.dimensional.mini4pro.warn.WarnEvent,
        stamp: LcmTime,
        seq: Int,
    ): LcmDiagnosticArray {
        val w = event.warning
        val values = ArrayList<LcmKeyValue>(7)
        values += LcmKeyValue("source", w.source.label)
        values += LcmKeyValue("state", w.state)
        values += LcmKeyValue("level", w.level.name)
        values += LcmKeyValue("change", event.change.name.lowercase())
        event.previousLevel?.let { values += LcmKeyValue("previous", it.name) }
        w.measurement?.let { values += LcmKeyValue("measurement", it) }
        values += LcmKeyValue("forwarded", if (event.announce) "true" else "false")
        return LcmDiagnosticArray(
            header = LcmHeader(seq = seq, stamp = stamp, frameId = FRAME_BASE_LINK),
            status = listOf(
                LcmDiagnosticStatus(
                    level = event.diagnosticLevel,
                    name = "${w.source.label}/${w.code}",
                    message = event.text,
                    hardwareId = hardwareId(w),
                    values = values,
                )
            ),
        )
    }

    /** `"<componentId>/<sensorIndex>"`, or empty when the source has no components. */
    private fun hardwareId(w: com.dimensional.mini4pro.warn.Warning): String {
        val component = w.componentId
        val sensor = w.sensorIndex
        if (component == null && sensor == null) return ""
        return "${component ?: ""}/${sensor ?: ""}"
    }

    // ── datum ─────────────────────────────────────────────────────────────────

}

/**
 * The camera's attitude **as DJI reports it: earth-referenced, in degrees, DJI's own convention.**
 *
 * Deliberately not `gimbal/GimbalAngles`: this package imports nothing from `gimbal/`, whose seam
 * owns the DJI half, and the two types mean different things anyway. `GimbalAngles` is *what a
 * reading contained*, all three axes independently nullable because DJI's fields are boxed. This
 * is *what we are willing to build an edge from*, which requires all three and requires them
 * finite — see [ZenohTelemetryEncoder.cameraEdge] for what happens to them.
 *
 * @param source which angle this is, and it is not a detail. The **commanded** angle is exact and
 *   always fresh; the **reported** one is change-driven and goes silent exactly when the camera is
 *   held still — measured delivery age while stationary: median **4.0 s**, p90 16.5 s, max 25.3 s,
 *   against 33–51 ms while moving. A large age means "nothing moved" rather than "the feed died",
 *   which is good news and useless news: a pose computed against a four-second-old angle is
 *   confidently wrong and nothing errors.
 * @param ageMs how old the reading behind this is, or null when it is a command and therefore has
 *   no age. Carried so a consumer auditing a held edge can see what it was held from — the
 *   diagnostic `docs/mem2-converter.md` §6.1 says matters *more* under a holding rule, not less.
 */
data class GimbalEarthAttitude(
    val rollDeg: Double,
    val pitchDeg: Double,
    val yawDeg: Double,
    val source: Source,
    val ageMs: Long? = null,
) {
    enum class Source {
        /** What we asked for. Exact, always fresh, and the one §2.2 of the contract wants. */
        COMMANDED,

        /**
         * What DJI last said. Used when nothing has commanded the gimbal this session — which is
         * the normal case when the camera is being aimed from the RC, and is what both AprilTag
         * datasets contain.
         */
        REPORTED,
    }

    companion object {
        /**
         * The three angles, or null when any is missing or not finite.
         *
         * All-or-nothing on purpose. A partial attitude is not a smaller error than none: two
         * good axes and a zeroed third is a camera confidently pointing somewhere it is not, and
         * the absence of the edge is a statement a consumer can act on.
         */
        fun of(
            rollDeg: Double?,
            pitchDeg: Double?,
            yawDeg: Double?,
            source: Source,
            ageMs: Long? = null,
        ): GimbalEarthAttitude? {
            val roll = rollDeg?.takeIf { it.isFinite() } ?: return null
            val pitch = pitchDeg?.takeIf { it.isFinite() } ?: return null
            val yaw = yawDeg?.takeIf { it.isFinite() } ?: return null
            return GimbalEarthAttitude(roll, pitch, yaw, source, ageMs)
        }
    }
}
