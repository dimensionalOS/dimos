package com.dimensional.mini4pro.zenoh

/**
 * The ROS-shaped message types DiMOS carries, as Kotlin data.
 *
 * One data class per LCM type in `docs/zenoh-topics.md`, plus every type they
 * nest. They hold values and nothing else — no encoding, no clocks, no frames,
 * no unit conversion. [LcmCodecs] turns them into bytes; whoever fills them in
 * decides what the numbers mean.
 *
 * ## Choices worth knowing about
 *
 * - **Field order here mirrors the wire, not ROS's `.msg` files.** Where lcm-gen
 *   hoists an array-length field to the front of the struct (`nav_msgs.Path`,
 *   `sensor_msgs.BatteryState`) the length is *not* a field here at all: it is
 *   derived from the list at encode time, so the two can never disagree.
 * - **Covariance is a `List<Double>`, not a `DoubleArray`.** Data classes give
 *   array fields reference equality, which would make every `assertEquals` in a
 *   round-trip test pass for the wrong reason. Lists compare by value. At 5 Hz
 *   the boxing is not worth a hand-written `equals`.
 * - **Nothing is nullable.** `AircraftState` is null-per-field because null means
 *   *no reading*; LCM has no way to say that, which is why a message is published
 *   only once the fields it needs are non-null (`docs/zenoh-topics.md`). By the
 *   time a value reaches these classes the decision to publish has been made.
 */

/** The size of the `double[36]` covariance on a pose or a twist. */
const val COVARIANCE_SIZE_36: Int = 36

/** The size of the `double[9]` covariance on a fix or an IMU reading. */
const val COVARIANCE_SIZE_9: Int = 9

/** A 6×6 of zeros — ROS's "unknown, and I am not claiming otherwise". */
val ZERO_COVARIANCE_36: List<Double> = List(COVARIANCE_SIZE_36) { 0.0 }

/** A 3×3 of zeros. */
val ZERO_COVARIANCE_9: List<Double> = List(COVARIANCE_SIZE_9) { 0.0 }

/**
 * `std_msgs.Time` — the time inside every [LcmHeader].
 *
 * Not `builtin_interfaces.Time`: lcm-gen's `std_msgs.Header` nests this one, and
 * the second field is `nsec`, not `nanosec`. Both are `int32`, so [sec] wraps in
 * 2038 — that is the format's problem, faithfully reproduced.
 */
data class LcmTime(val sec: Int, val nsec: Int) {
    companion object {
        val ZERO = LcmTime(0, 0)

        /** Splits a Unix epoch time in seconds into the two `int32` halves. */
        fun ofEpochSeconds(seconds: Double): LcmTime {
            val whole = kotlin.math.floor(seconds)
            val nanos = ((seconds - whole) * 1e9).toLong()
            return LcmTime(whole.toLong().toInt(), nanos.toInt())
        }
    }
}

/**
 * `std_msgs.Header` — `seq`, `stamp`, `frame_id`.
 *
 * [seq] survives here from ROS 1; ROS 2 dropped it and lcm-gen did not. Nothing
 * in `docs/zenoh-topics.md` reads it, so a monotonic counter or a constant zero
 * are both honest.
 *
 * [frameId] is load-bearing. It names the frame the payload is expressed in —
 * `odom`, `base_link`, or on the `datum` channel the datum *generation*
 * (`docs/zenoh-frames-and-paths.md` §3). A wrong `frame_id` is worse than a
 * missing message.
 */
data class LcmHeader(
    val seq: Int = 0,
    val stamp: LcmTime = LcmTime.ZERO,
    val frameId: String = "",
)

/**
 * `std_msgs.Float32` — `float data`, nothing else. The `wind` channel's whole message.
 *
 * **A `Float`, deliberately, twice over.** Once by schema — the LCM type is single-precision
 * and encoding a Kotlin `Double` would be a different message — and once by honesty: the value
 * is DJI's own wind estimate, delivered as an *integer* in dm/s (`KeyWindSpeed` is
 * `DJIKeyInfo<Integer>`, javap on the 5.18.0 jar), so a double's extra 29 bits could only ever
 * carry the artefacts of a division by ten. Float32 represents every dm/s reading DJI can
 * produce without collision; nothing about the source earns more.
 *
 * **No header, no stamp, no frame.** `std_msgs.Float32` carries none, and that makes `wind`
 * the one published channel whose message states no time of its own: live, a subscriber gets
 * the arrival instant, which is honest for an event-driven delivery; offline, the mem2 store's
 * own `ts` column carries the record's clock. D-5 ("the reading's own time, never the send
 * time") is satisfied by there being no field to get wrong.
 */
data class LcmFloat32(val data: Float)

/** `geometry_msgs.Point` — `double x, y, z`. */
data class LcmPoint(val x: Double, val y: Double, val z: Double) {
    companion object {
        val ZERO = LcmPoint(0.0, 0.0, 0.0)
    }
}

/**
 * `geometry_msgs.Vector3` — `double x, y, z`.
 *
 * Byte-identical to [LcmPoint], fingerprint included; separate because the
 * `gimbal` and `gimbal_cmd` channels are typed `Vector3` and reading `Point`
 * there would be a lie about the contract.
 */
data class LcmVector3(val x: Double, val y: Double, val z: Double) {
    companion object {
        val ZERO = LcmVector3(0.0, 0.0, 0.0)
    }
}

/** `geometry_msgs.Quaternion` — `double x, y, z, w`, in that order. **`w` last.** */
data class LcmQuaternion(val x: Double, val y: Double, val z: Double, val w: Double) {
    companion object {
        /** No rotation. `w = 1`, and it is the *last* number on the wire. */
        val IDENTITY = LcmQuaternion(0.0, 0.0, 0.0, 1.0)
    }
}

/** `geometry_msgs.Pose` — a position and an orientation. */
data class LcmPose(
    val position: LcmPoint = LcmPoint.ZERO,
    val orientation: LcmQuaternion = LcmQuaternion.IDENTITY,
)

/**
 * `geometry_msgs.PoseWithCovariance` — a [LcmPose] and a row-major 6×6.
 *
 * [covariance] must be exactly [COVARIANCE_SIZE_36] long: it is a fixed-size LCM
 * array with no length on the wire, so a short list would silently shorten the
 * message.
 */
data class LcmPoseWithCovariance(
    val pose: LcmPose = LcmPose(),
    val covariance: List<Double> = ZERO_COVARIANCE_36,
)

/** `geometry_msgs.Twist` — linear and angular velocity. */
data class LcmTwist(
    val linear: LcmVector3 = LcmVector3.ZERO,
    val angular: LcmVector3 = LcmVector3.ZERO,
)

/** `geometry_msgs.TwistWithCovariance` — a [LcmTwist] and a row-major 6×6. */
data class LcmTwistWithCovariance(
    val twist: LcmTwist = LcmTwist(),
    val covariance: List<Double> = ZERO_COVARIANCE_36,
)

/**
 * `geometry_msgs.TwistStamped` — the `setpoint` channel: **the velocity command this bridge
 * actually handed the aircraft**, stamped and framed.
 *
 * A stamped [LcmTwist], nothing more. The inbound `cmd_vel` channel carries a bare `Twist`
 * because that is ROS's command convention; the outbound echo of what was commanded is
 * *telemetry* and follows the bus's stamping pattern — every published reading travels with the
 * instant it claims (`header.stamp`, D-5) and the frame it is expressed in (`frame_id`), which a
 * bare `Twist` cannot carry.
 */
data class LcmTwistStamped(
    val header: LcmHeader = LcmHeader(),
    val twist: LcmTwist = LcmTwist(),
)

/** `geometry_msgs.PoseStamped` — the `goal` channel, and each element of a `path`. */
data class LcmPoseStamped(
    val header: LcmHeader = LcmHeader(),
    val pose: LcmPose = LcmPose(),
)

/** `geometry_msgs.PointStamped` — the `roi` channel. Its `frame_id` says which frame. */
data class LcmPointStamped(
    val header: LcmHeader = LcmHeader(),
    val point: LcmPoint = LcmPoint.ZERO,
)

/**
 * `nav_msgs.Odometry` — the primary telemetry message: pose *and* twist together.
 *
 * `header.frame_id` is `odom`, [childFrameId] is `base_link`.
 */
data class LcmOdometry(
    val header: LcmHeader = LcmHeader(),
    val childFrameId: String = "",
    val pose: LcmPoseWithCovariance = LcmPoseWithCovariance(),
    val twist: LcmTwistWithCovariance = LcmTwistWithCovariance(),
)

/**
 * `nav_msgs.Path` — a header plus a list of [LcmPoseStamped]. A mission.
 *
 * On the wire the *count* comes first, before the header. That is not visible
 * here on purpose: it is written from `poses.size`.
 */
data class LcmPath(
    val header: LcmHeader = LcmHeader(),
    val poses: List<LcmPoseStamped> = emptyList(),
)

/**
 * `sensor_msgs.NavSatStatus` — the fix quality inside a [LcmNavSatFix].
 *
 * [status] is an `int8` and **[STATUS_NO_FIX] is −1**, so it is signed on the
 * wire and a byte-vs-int slip changes "no fix" into 255. [service] is an `int16`
 * bitmask of the constellations.
 */
data class LcmNavSatStatus(
    val status: Byte = STATUS_NO_FIX,
    val service: Short = SERVICE_GPS,
) {
    companion object {
        /** Unable to fix position. **Negative.** */
        const val STATUS_NO_FIX: Byte = -1

        /** Unaugmented fix. */
        const val STATUS_FIX: Byte = 0

        /** With satellite-based augmentation. */
        const val STATUS_SBAS_FIX: Byte = 1

        /** With ground-based augmentation. */
        const val STATUS_GBAS_FIX: Byte = 2

        const val SERVICE_GPS: Short = 1
        const val SERVICE_GLONASS: Short = 2
        const val SERVICE_COMPASS: Short = 4
        const val SERVICE_GALILEO: Short = 8
    }
}

/**
 * `sensor_msgs.NavSatFix` — the `datum`, `gps_location` and `gps_goal` channels.
 *
 * [positionCovariance] is a fixed `double[9]`, row-major ENU.
 */
data class LcmNavSatFix(
    val header: LcmHeader = LcmHeader(),
    val status: LcmNavSatStatus = LcmNavSatStatus(),
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val positionCovariance: List<Double> = ZERO_COVARIANCE_9,
    val positionCovarianceType: Byte = COVARIANCE_TYPE_UNKNOWN,
) {
    companion object {
        const val COVARIANCE_TYPE_UNKNOWN: Byte = 0
        const val COVARIANCE_TYPE_APPROXIMATED: Byte = 1
        const val COVARIANCE_TYPE_DIAGONAL_KNOWN: Byte = 2
        const val COVARIANCE_TYPE_KNOWN: Byte = 3
    }
}

/**
 * `sensor_msgs.Imu` — orientation, angular rate, linear acceleration, each with
 * its own fixed `double[9]`.
 *
 * ROS's convention for "this quantity is not reported" is `−1` in element 0 of
 * the relevant covariance. This project's rule is not to publish at all in that
 * case (`docs/zenoh-topics.md`), so the sentinel is available but should be rare.
 */
data class LcmImu(
    val header: LcmHeader = LcmHeader(),
    val orientation: LcmQuaternion = LcmQuaternion.IDENTITY,
    val orientationCovariance: List<Double> = ZERO_COVARIANCE_9,
    val angularVelocity: LcmVector3 = LcmVector3.ZERO,
    val angularVelocityCovariance: List<Double> = ZERO_COVARIANCE_9,
    val linearAcceleration: LcmVector3 = LcmVector3.ZERO,
    val linearAccelerationCovariance: List<Double> = ZERO_COVARIANCE_9,
)

/**
 * `geometry_msgs.Transform` — a translation and a rotation, and **not** a [LcmPose].
 *
 * Byte-identical to `Pose` in field count and order, and a different type with a different
 * fingerprint, because the two mean different things: a `Pose` is where something *is*, a
 * `Transform` is how to get from one frame to another. `translation` is a [LcmVector3] where
 * `Pose` has a `Point`, which is the wire-level trace of that distinction.
 */
data class LcmTransform(
    val translation: LcmVector3 = LcmVector3.ZERO,
    val rotation: LcmQuaternion = LcmQuaternion.IDENTITY,
)

/**
 * `geometry_msgs.TransformStamped` — one edge of a TF tree.
 *
 * [LcmHeader.frameId] is the **parent** and [childFrameId] is the child. Getting those the wrong
 * way round produces a tree that resolves, with every transform inverted, which is the failure
 * mode a round-trip test cannot see and a composition test can.
 */
data class LcmTransformStamped(
    val header: LcmHeader = LcmHeader(),
    val childFrameId: String = "",
    val transform: LcmTransform = LcmTransform(),
)

/**
 * `tf2_msgs.TFMessage` — **every edge valid at one instant, in one message.**
 *
 * One message rather than one per edge, per `docs/zenoh-topics.md`: the edges are a snapshot of
 * one tree at one timestamp, and a consumer that has to re-join them across messages is doing a
 * merge that can be stale in one edge and fresh in another.
 *
 * The count is hoisted ahead of the array on the wire and is **not** a field here — it is derived
 * from [transforms] at encode time, so the two cannot disagree.
 */
data class LcmTfMessage(val transforms: List<LcmTransformStamped> = emptyList())

/**
 * `sensor_msgs.RegionOfInterest` — the sub-window a [LcmCameraInfo] describes.
 *
 * All zeros with [doRectify] false is ROS's "the whole image", which is what this camera reports:
 * nothing crops, nothing bins, and the default is the honest value rather than a placeholder.
 */
data class LcmRegionOfInterest(
    val xOffset: Int = 0,
    val yOffset: Int = 0,
    val height: Int = 0,
    val width: Int = 0,
    val doRectify: Boolean = false,
)

/**
 * `sensor_msgs.CameraInfo` — the intrinsics, in the **optical** frame.
 *
 * See `ZenohTelemetryEncoder.cameraInfo` for what goes in each field and, more importantly, what
 * deliberately does not: [d] and [distortionModel] are empty because distortion is unmeasured, and
 * an empty `D` is ROS's own "no model is offered" rather than an affirmative claim that this lens
 * is rectilinear. `docs/mem2-converter.md` §0.2 is the argument and it is not boilerplate.
 *
 * [d] is variable-length and its count is hoisted to the **front of the struct**, ahead of the
 * header — the same lcm-gen shape as `nav_msgs.Path` and `sensor_msgs.BatteryState`, and derived
 * from the list rather than carried as a field. [k], [r] and [p] are fixed `double[9]`,
 * `double[9]` and `double[12]` with no length on the wire at all.
 */
data class LcmCameraInfo(
    val header: LcmHeader = LcmHeader(),
    val height: Int = 0,
    val width: Int = 0,
    val distortionModel: String = "",
    val d: List<Double> = emptyList(),
    val k: List<Double> = ZERO_MATRIX_9,
    val r: List<Double> = ZERO_MATRIX_9,
    val p: List<Double> = ZERO_MATRIX_12,
    val binningX: Int = 0,
    val binningY: Int = 0,
    val roi: LcmRegionOfInterest = LcmRegionOfInterest(),
)

/** The size of `CameraInfo.K` and `CameraInfo.R` — 3×3, row-major. */
const val MATRIX_SIZE_9: Int = 9

/** The size of `CameraInfo.P` — 3×4, row-major. */
const val MATRIX_SIZE_12: Int = 12

/** A 3×3 of zeros, which is ROS's flag for an uncalibrated camera. We do not publish one. */
val ZERO_MATRIX_9: List<Double> = List(MATRIX_SIZE_9) { 0.0 }

/** A 3×4 of zeros. */
val ZERO_MATRIX_12: List<Double> = List(MATRIX_SIZE_12) { 0.0 }

/**
 * `foxglove_msgs.CompressedVideo` — **one H.264 access unit, verbatim, never decoded.**
 *
 * The one type in this file that is not a `data class`, and the reason is [data]: a data class
 * gives an array field reference equality, so every generated `equals` would compare frames by
 * identity and every round-trip assertion would pass for the wrong reason. [LcmMessages]' own
 * header states that rule for covariances and solves it there by using a `List<Double>`; a
 * 17 kB access unit is the case where boxing every byte is not an option, so the equality is
 * written out instead.
 *
 * ## Two things about this type that are not like the others
 *
 * - **The timestamp is `builtin_interfaces.Time`, not a `std_msgs.Header`.** Foxglove's schema
 *   carries a bare stamp and a bare `frame_id` with no `seq`. The two time types have the same
 *   two `int32` fields and different fingerprints — which is invisible here, because a nested
 *   struct carries no fingerprint of its own, and visible in [LcmFingerprints] where the outer
 *   type's recursive hash is computed over it.
 * - **[data] is the aircraft's own bytes and is never re-encoded.** Annex-B, start codes
 *   included, exactly the span `record/VideoSidecar` writes to its `.h264` sidecar. Measured over
 *   6952 real frames: every keyframe is `(SPS, PPS, IDR)`, every inter frame is one NAL, and there
 *   are **zero B-frames** — which is what makes a byte copy schema-conformant and a decode
 *   pointless. `docs/mem2-converter.md` §4.1 lists the four requirements and the measurement
 *   behind each.
 *
 * @param frameId `drone/camera_optical`, which is the frame the schema defines for itself — so
 *   the video's frame is right by construction rather than by our assertion.
 * @param format `"h264"`, lower case, as the schema names it.
 */
class LcmCompressedVideo(
    val timestamp: LcmTime = LcmTime.ZERO,
    val frameId: String = "",
    val data: ByteArray = EMPTY,
    val format: String = "",
) {
    override fun equals(other: Any?): Boolean =
        this === other || (
            other is LcmCompressedVideo &&
                timestamp == other.timestamp &&
                frameId == other.frameId &&
                format == other.format &&
                data.contentEquals(other.data)
            )

    override fun hashCode(): Int {
        var h = timestamp.hashCode()
        h = 31 * h + frameId.hashCode()
        h = 31 * h + format.hashCode()
        h = 31 * h + data.contentHashCode()
        return h
    }

    /** Never the bytes: a frame in a stack trace must not be 17 kB of hex. */
    override fun toString(): String =
        "LcmCompressedVideo($format, $frameId, ${data.size} bytes, $timestamp)"

    companion object {
        private val EMPTY = ByteArray(0)

        /** The only format this project publishes, and the only one the measurements cover. */
        const val FORMAT_H264 = "h264"
    }
}

/**
 * `vision_msgs.ObjectHypothesis` — `string class_id, double score`.
 *
 * [classId] is the *class* of thing seen, not the instance: for this project
 * `"tag36h11:7"`, family and code together, because tag 7 of one family is a
 * different object from tag 7 of another.
 *
 * [score] is documented by ROS as a confidence and is **not required to be a
 * probability by anything on the wire**. What this project puts there, and why
 * `hamming` is not it, is argued at `ZenohTelemetryEncoder.detectionOrNull`.
 */
data class LcmObjectHypothesis(val classId: String = "", val score: Double = 0.0)

/** `vision_msgs.ObjectHypothesisWithPose` — one hypothesis and where it is. */
data class LcmObjectHypothesisWithPose(
    val hypothesis: LcmObjectHypothesis = LcmObjectHypothesis(),
    val pose: LcmPoseWithCovariance = LcmPoseWithCovariance(),
)

/**
 * `vision_msgs.BoundingBox3D` — `Pose center, Vector3 size`.
 *
 * **Nothing in this project solves one**, and the type has no way to say so: a
 * [size] of `(0, 0, 0)` is an affirmative claim that the object is a point.
 * `ZenohTelemetryEncoder.UNSOLVED_BOX` is what goes on the wire instead and it is
 * NaN throughout — the sentinel §4 of that class's doc already reserves for a
 * quantity there is no feed for at all.
 */
data class LcmBoundingBox3D(
    val center: LcmPose = LcmPose(),
    val size: LcmVector3 = LcmVector3.ZERO,
)

/**
 * `vision_msgs.Detection3D` — **one detected object, at one instant, in one frame.**
 *
 * The `detections` channel. `header.frame_id` is `drone/camera_optical`, so
 * every pose in [results] is referred to the **principal point** and to the
 * optical axes — x right, y down, z along the axis.
 *
 * `results_length` is hoisted to the front of the struct on the wire and is
 * **not** a field here: it is derived from [results] at encode time, so the two
 * cannot disagree. Unlike `nav_msgs.Path` the array is followed by two more
 * fields, so a wrong length shifts [bbox] and [id] rather than merely truncating.
 *
 * [id] is the field vision_msgs intends as a tracking handle, and since 2026-07-29 it is
 * exactly that: the bare label, `tag36h11:<id>` (Ivan — no suffix stuffing). See
 * `ZenohTelemetryEncoder.detectionId` for what carried the old suffixes' information after
 * their deletion.
 */
data class LcmDetection3D(
    val header: LcmHeader = LcmHeader(),
    val results: List<LcmObjectHypothesisWithPose> = emptyList(),
    val bbox: LcmBoundingBox3D = LcmBoundingBox3D(),
    val id: String = "",
)

/**
 * `vision_msgs.Detection3DArray` — **the type the `detections` channel actually carries.**
 *
 * One frame's detections in one message, exactly as `tf2_msgs.TFMessage` carries one instant's
 * edges: a consumer that had to re-join detections across messages could not tell one frame's from
 * two frames'. ROS's own convention for a detection topic, and the one DiMOS consumes natively.
 *
 * `detections_length` is hoisted to the front of the struct and, as everywhere else in this file,
 * is **not** a field here — it is derived from [detections] at encode time.
 *
 * [header] restates the inner detection's `seq`, `stamp` and `frame_id` rather than carrying an
 * envelope of its own. See `ZenohTelemetryEncoder.detectionsOrNull` for why, and for why the array
 * has exactly one element today without that being a promise.
 */
data class LcmDetection3DArray(
    val header: LcmHeader = LcmHeader(),
    val detections: List<LcmDetection3D> = emptyList(),
)

/**
 * `sensor_msgs.BatteryState` — the `battery` channel.
 *
 * Everything numeric is a **`float`**, not a double, which is the one thing about
 * this type that catches people out. [percentage] is ROS's 0..1 fraction, not a
 * 0..100 percent. [current] is negative while discharging, per ROS.
 *
 * The two cell arrays are variable-length; their counts are written first on the
 * wire, ahead of the header, and are derived from the lists here.
 */
data class LcmBatteryState(
    val header: LcmHeader = LcmHeader(),
    val voltage: Float = 0f,
    val temperature: Float = 0f,
    val current: Float = 0f,
    val charge: Float = 0f,
    val capacity: Float = 0f,
    val designCapacity: Float = 0f,
    /** 0..1, not 0..100. */
    val percentage: Float = 0f,
    val powerSupplyStatus: Byte = POWER_SUPPLY_STATUS_UNKNOWN,
    val powerSupplyHealth: Byte = POWER_SUPPLY_HEALTH_UNKNOWN,
    val powerSupplyTechnology: Byte = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
    val present: Boolean = false,
    val cellVoltage: List<Float> = emptyList(),
    val cellTemperature: List<Float> = emptyList(),
    val location: String = "",
    val serialNumber: String = "",
) {
    companion object {
        const val POWER_SUPPLY_STATUS_UNKNOWN: Byte = 0
        const val POWER_SUPPLY_STATUS_CHARGING: Byte = 1
        const val POWER_SUPPLY_STATUS_DISCHARGING: Byte = 2
        const val POWER_SUPPLY_STATUS_NOT_CHARGING: Byte = 3
        const val POWER_SUPPLY_STATUS_FULL: Byte = 4

        const val POWER_SUPPLY_HEALTH_UNKNOWN: Byte = 0
        const val POWER_SUPPLY_HEALTH_GOOD: Byte = 1

        /** The failure this airframe actually has. See `docs/device-health.md`. */
        const val POWER_SUPPLY_HEALTH_OVERHEAT: Byte = 2
        const val POWER_SUPPLY_HEALTH_DEAD: Byte = 3
        const val POWER_SUPPLY_HEALTH_OVERVOLTAGE: Byte = 4
        const val POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: Byte = 5
        const val POWER_SUPPLY_HEALTH_COLD: Byte = 6
        const val POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: Byte = 7
        const val POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: Byte = 8

        const val POWER_SUPPLY_TECHNOLOGY_UNKNOWN: Byte = 0
        const val POWER_SUPPLY_TECHNOLOGY_NIMH: Byte = 1
        const val POWER_SUPPLY_TECHNOLOGY_LION: Byte = 2

        /** What a Mini 4 Pro flies on. */
        const val POWER_SUPPLY_TECHNOLOGY_LIPO: Byte = 3
        const val POWER_SUPPLY_TECHNOLOGY_LIFE: Byte = 4
        const val POWER_SUPPLY_TECHNOLOGY_NICD: Byte = 5
        const val POWER_SUPPLY_TECHNOLOGY_LIMN: Byte = 6
    }
}

// ---------------------------------------------------------------------------
// diagnostic_msgs — the `warnings` channel
// ---------------------------------------------------------------------------

/**
 * `diagnostic_msgs.KeyValue` — `string key, string value`.
 *
 * The escape hatch that lets [LcmDiagnosticStatus] carry DJI's own words without inventing a
 * message type: `state=LEVEL_2`, `level=WARNING`, `change=appeared`, `measurement=14.2 m/s`.
 * Ivan's standing rule is to use standard types rather than invent them, and a diagnostic's
 * `values` is where ROS itself puts exactly this kind of per-source detail.
 */
data class LcmKeyValue(
    val key: String = "",
    val value: String = "",
)

/**
 * `diagnostic_msgs.DiagnosticStatus` — one warning's current state.
 *
 * `values_length` is hoisted to the front of the struct and, as everywhere else in this file, is
 * **not** a field here — it is derived from [values] at encode time.
 *
 * [level] is ROS's own four-value ladder (`OK`, `WARN`, `ERROR`, `STALE`), and its single owner is
 * `warn/WarnLevel.diagnosticLevel` — not this class, which is a wire shape and holds no opinion.
 */
data class LcmDiagnosticStatus(
    val level: Byte = LEVEL_OK,
    /** `"<source>/<code>"` — the same identity the flight record's `src` and `code` carry. */
    val name: String = "",
    /** The operator's sentence, byte-identical to the one QGC was given. */
    val message: String = "",
    /** DJI's component/sensor, where the source has them; empty where it does not. */
    val hardwareId: String = "",
    val values: List<LcmKeyValue> = emptyList(),
) {
    companion object {
        /** The constants the `.lcm` file itself declares. */
        const val LEVEL_OK: Byte = 0
        const val LEVEL_WARN: Byte = 1
        const val LEVEL_ERROR: Byte = 2
        const val LEVEL_STALE: Byte = 3
    }
}

/**
 * `diagnostic_msgs.DiagnosticArray` — the `warnings` channel.
 *
 * **One status per message, and the message is a *change*, not a picture.** ROS publishes
 * DiagnosticArrays periodically with every component's current state; this channel publishes one
 * whenever a DJI warning appears, changes level or clears, carrying exactly the warning that
 * moved. Two reasons, both of them this project's rules rather than ROS's:
 *
 *  - **Edge-triggered everywhere.** `warn/WarningMonitor` is the single owner of "is this news?",
 *    and a periodic full picture would put a second answer to that question on the bus — the
 *    per-sample repeat this whole path exists to prevent.
 *  - **Absence is the honest signal.** Publishing "nothing is wrong" at 1 Hz would make a dead
 *    publisher and a healthy aircraft look identical, which is the trap `docs/zenoh-topics.md`
 *    states for every other channel here.
 *
 * A consumer that wants the standing picture accumulates it from the stream, which is what the
 * `appeared`/`cleared` pairing is for. `docs/zenoh-topics.md`'s row says so.
 *
 * `status_length` is hoisted to the front of the struct and is derived from [status] at encode
 * time.
 */
data class LcmDiagnosticArray(
    val header: LcmHeader = LcmHeader(),
    val status: List<LcmDiagnosticStatus> = emptyList(),
)
