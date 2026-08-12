package com.dimensional.mini4pro.zenoh

/**
 * The Kotlin half of the fixture pair.
 *
 * Every value here is mirrored by hand from `tools/lcmfixtures/generate.py`, and
 * the duplication is deliberate. The fixtures in
 * `android/app/src/test/resources/lcm/` were produced by DiMOS's own Python
 * bindings calling `lcm_encode()`; if a number below is mistyped, the bytes stop
 * matching. A fixture that carried its own field values alongside the bytes
 * would let a shared typo through, which is the one thing this cannot afford:
 * the whole point is an encoder we did not write disagreeing with the one we did.
 *
 * Nothing here is a zero, an empty string or an identity quaternion unless a
 * fixture exists *specifically* to pin that degenerate case — those hide swapped
 * field order and wrong endianness, which are the bugs this format invites.
 */
object LcmFixtures {

    /** 2025-07-27T12:00:00Z. Comfortably inside `int32`. */
    const val SEC: Int = 1753622400

    /** Nine digits, so a truncation to milliseconds would be visible. */
    const val NSEC: Int = 987654321

    const val SEQ: Int = 4242

    /**
     * `µ` and `°` are two UTF-8 bytes each and the battery emoji is four, so this
     * string is 47 characters and 55 bytes. A codec that length-prefixes
     * *character* counts fails here and nowhere else.
     */
    const val UTF8_TEXT: String =
        "{\"code\":9,\"title\":\"battery µ 41.5°C 🔋\",\"level\":\"WARNING\"}"

    /** A 6×6 that is different in every cell and negative in a third of them. */
    fun cov36(): List<Double> = List(36) { it * 0.25 - 3.0 }

    /** A 3×3, likewise. */
    fun cov9(): List<Double> = List(9) { it * 1.5 - 5.0 }

    fun time() = LcmTime(SEC, NSEC)

    fun header(frameId: String = "odom", seq: Int = SEQ) = LcmHeader(seq, time(), frameId)

    fun point(x: Double = 12.5, y: Double = -4.25, z: Double = 31.125) = LcmPoint(x, y, z)

    fun vector3(x: Double = -31.5, y: Double = 0.125, z: Double = 64.25) = LcmVector3(x, y, z)

    /**
     * A real rotation, none of whose components is 0 or 1 — roughly yaw 60°,
     * pitch −20°, roll 15°. An identity quaternion would let a swapped `w`
     * through unnoticed.
     */
    fun quat() = LcmQuaternion(0.20056212, -0.09442657, 0.47555491, 0.85078055)

    fun pose(position: LcmPoint = point()) = LcmPose(position, quat())

    fun poseWithCov() = LcmPoseWithCovariance(pose(), cov36())

    fun twist() = LcmTwist(vector3(3.5, -1.25, 0.75), vector3(-0.125, 0.0625, -0.5))

    /** The `setpoint` channel's type; the frame is the one it publishes in. */
    fun twistStamped() = LcmTwistStamped(header("drone/world", 77), twist())

    fun twistWithCov() = LcmTwistWithCovariance(twist(), cov36().map { it * -1.0 })

    fun poseStamped(frameId: String = "odom", seq: Int = SEQ, position: LcmPoint = point()) =
        LcmPoseStamped(header(frameId, seq), pose(position))

    fun pointStamped() = LcmPointStamped(header("base_link", 9), point(-100.5, 250.25, 8.0))

    fun odometry() = LcmOdometry(header("odom"), "base_link", poseWithCov(), twistWithCov())

    /** Three poses, each distinguishable, so an off-by-one shows as a value mismatch. */
    fun pathThree() = LcmPath(
        header("odom", 11),
        listOf(
            poseStamped("odom", 1, point(1.5, -2.5, 3.5)),
            poseStamped("odom", 2, point(-10.25, 20.75, 30.125)),
            poseStamped("odom", 3, point(0.0625, 0.125, -0.25)),
        ),
    )

    fun pathEmpty() = LcmPath(header("odom", 12), emptyList())

    fun navSatStatus(status: Byte = 1, service: Short = 11) = LcmNavSatStatus(status, service)

    fun navSatFix() = LcmNavSatFix(
        header = header("datum/3"),
        status = navSatStatus(),
        latitude = 37.9838096,
        longitude = 23.7275383,
        altitude = -12.75,
        positionCovariance = cov9(),
        positionCovarianceType = LcmNavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
    )

    fun imu() = LcmImu(
        header = header("base_link"),
        orientation = quat(),
        orientationCovariance = cov9(),
        angularVelocity = vector3(0.25, -0.5, 1.75),
        angularVelocityCovariance = cov9().map { it * 2.0 },
        linearAcceleration = vector3(-0.75, 0.375, 9.80665),
        linearAccelerationCovariance = cov9().map { it * -0.5 },
    )

    fun battery() = LcmBatteryState(
        header = header("base_link", 77),
        voltage = 11.7f,
        temperature = 41.5f,
        current = -8.25f,
        charge = 1.35f,
        capacity = 2.59f,
        designCapacity = 2.59f,
        percentage = 0.63f,
        powerSupplyStatus = LcmBatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
        powerSupplyHealth = LcmBatteryState.POWER_SUPPLY_HEALTH_OVERHEAT,
        powerSupplyTechnology = LcmBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
        present = true,
        cellVoltage = listOf(3.91f, 3.89f, 3.90f),
        cellTemperature = listOf(40.5f, 42.5f),
        location = "bay µ1",
        serialNumber = "DJI-MINI4PRO-0001",
    )

    fun batteryNoCells() = LcmBatteryState(
        header = header("base_link", 78),
        voltage = 12.05f,
        temperature = 22.0f,
        current = -1.5f,
        charge = 2.0f,
        capacity = 2.59f,
        designCapacity = 2.59f,
        percentage = 0.87f,
        powerSupplyStatus = LcmBatteryState.POWER_SUPPLY_STATUS_FULL,
        powerSupplyHealth = LcmBatteryState.POWER_SUPPLY_HEALTH_GOOD,
        powerSupplyTechnology = LcmBatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
        present = false,
        cellVoltage = emptyList(),
        cellTemperature = emptyList(),
        location = "",
        serialNumber = "",
    )

    // ── the frame tree, the intrinsics and the video ─────────────────────────

    fun builtinTime() = LcmTime(SEC, NSEC)

    /**
     * A `Transform`, whose translation is a `Vector3` where a `Pose`'s position is a `Point` —
     * byte-identical, different type, different fingerprint.
     */
    fun transform(translation: LcmVector3 = vector3(0.08, 0.0, -0.035)) =
        LcmTransform(translation, quat())

    fun transformStamped(
        parent: String,
        child: String,
        translation: LcmVector3 = vector3(0.08, 0.0, -0.035),
        seq: Int = SEQ,
    ) = LcmTransformStamped(header(parent, seq), child, transform(translation))

    /** Three edges, in the order the publisher emits them. */
    fun tfThree() = LcmTfMessage(
        listOf(
            transformStamped(
                "drone/world", "drone/base_link", LcmVector3(12.5, -4.25, 31.125), 1,
            ),
            transformStamped("drone/base_link", "drone/camera", LcmVector3(0.08, 0.0, 0.0), 2),
            transformStamped(
                "drone/camera", "drone/camera_optical", LcmVector3(0.0, 0.0, 0.0), 3,
            ),
        ),
    )

    /** Five bytes of fingerprint-plus-zero — the shape a hoisted length field gets wrong. */
    fun tfEmpty() = LcmTfMessage(emptyList())

    fun regionOfInterest() = LcmRegionOfInterest(
        xOffset = 16, yOffset = 32, height = 540, width = 960, doRectify = true,
    )

    /** The message the publisher sends: the fitted focal length, and no distortion claim. */
    fun cameraInfo() = LcmCameraInfo(
        header = header("drone/camera_optical"),
        height = 1080,
        width = 1920,
        distortionModel = "",
        d = emptyList(),
        k = listOf(FITTED_F, 0.0, 960.0, 0.0, FITTED_F, 540.0, 0.0, 0.0, 1.0),
        r = listOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p = listOf(FITTED_F, 0.0, 960.0, 0.0, 0.0, FITTED_F, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    )

    /**
     * A `CameraInfo` with a non-empty `D`, so the array body's position — after
     * `distortion_model`, far from its own length field — is pinned too.
     *
     * **These are not values this project publishes.** Nothing has measured this lens, which is
     * why the real message above carries an empty `D` and an empty model name.
     */
    fun cameraInfoDistorted() = LcmCameraInfo(
        header = header("drone/camera_optical", 43),
        height = 540,
        width = 960,
        distortionModel = "plumb_bob",
        d = listOf(-0.25, 0.0625, -0.001, 0.002, 0.0),
        k = listOf(728.55, 0.0, 480.0, 0.0, 728.55, 270.0, 0.0, 0.0, 1.0),
        r = listOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p = listOf(728.55, 0.0, 480.0, 0.0, 0.0, 728.55, 270.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        binningX = 2,
        binningY = 2,
        roi = regionOfInterest(),
    )

    /** `0.75890625 · 1920` — the fitted focal length at the stream's own width. */
    const val FITTED_F: Double = 0.75890625 * 1920

    /**
     * An access unit shaped like a real one: a four-byte Annex-B start code, an SPS NAL, another
     * start code, an IDR. Short, and every byte value matters.
     */
    fun videoPayload(): ByteArray = byteArrayOf(
        0, 0, 0, 1, 0x67.toByte(), 0x42, 0xC0.toByte(), 0x1F,
        0, 0, 0, 1, 0x65, 0x88.toByte(), 0x84.toByte(), 0x00,
    )

    fun compressedVideo() = LcmCompressedVideo(
        timestamp = builtinTime(),
        frameId = "drone/camera_optical",
        data = videoPayload(),
        format = "h264",
    )

    /** Zero bytes of payload — where a length-first field with a body in the middle goes wrong. */
    fun compressedVideoEmpty() = LcmCompressedVideo(
        timestamp = builtinTime(),
        frameId = "drone/camera_optical",
        data = ByteArray(0),
        format = "h264",
    )

    // ── the tag detections ───────────────────────────────────────────────────

    /**
     * An orientation and an extent nothing here solves, as the wire says it: **NaN**.
     *
     * Mirrored by hand from `generate.py`'s `float("nan")`, and it is the one value in this file
     * whose *bits* are the assertion rather than its magnitude. `struct.pack('>d')` writes the
     * canonical quiet NaN `7ff8000000000000` and `Double.doubleToRawLongBits(Double.NaN)` is the
     * same eight bytes; a codec that routed a double through a float, or normalised the payload,
     * or refused a non-finite value, fails here and on no other fixture.
     */
    fun unsolvedQuaternion() = LcmQuaternion(Double.NaN, Double.NaN, Double.NaN, Double.NaN)

    fun hypothesis(classId: String = "tag36h11:7", score: Double = 41.75) =
        LcmObjectHypothesis(classId, score)

    /**
     * A real detection's shape: the tag left of and above centre, 3.5 m down the optical axis,
     * with an orientation nothing solved and ROS's own all-zero "covariance unknown".
     */
    fun hypothesisWithPose(
        classId: String = "tag36h11:7",
        score: Double = 41.75,
        position: LcmPoint = point(-0.375, 0.8125, 3.5),
    ) = LcmObjectHypothesisWithPose(
        hypothesis = hypothesis(classId, score),
        pose = LcmPoseWithCovariance(
            pose = LcmPose(position, unsolvedQuaternion()),
            covariance = ZERO_COVARIANCE_36,
        ),
    )

    /** A `BoundingBox3D` refused entire — NaN centre, NaN orientation, NaN size. */
    fun unsolvedBox() = LcmBoundingBox3D(
        center = LcmPose(LcmPoint(Double.NaN, Double.NaN, Double.NaN), unsolvedQuaternion()),
        size = LcmVector3(Double.NaN, Double.NaN, Double.NaN),
    )

    /**
     * `Detection3D.id`, verbatim: the bare label since 2026-07-29 (Ivan — no suffix
     * stuffing; solvedness travels structurally, the metric caveat lives in the contract row).
     */
    const val DETECTION_ID: String = "tag36h11:7"

    /** The message the publisher actually sends: one result, a refused box, a bare id. */
    fun detection3D() = LcmDetection3D(
        header = header("drone/camera_optical"),
        results = listOf(hypothesisWithPose()),
        bbox = unsolvedBox(),
        id = DETECTION_ID,
    )

    /**
     * Two results, so `bbox` and `id`'s position after a *variable* number of hypotheses is pinned
     * by more than one length.
     *
     * **Not a message the publisher sends.** One sighting is one object, and two hypotheses would
     * mean two candidate objects rather than one object described twice.
     */
    fun detection3DTwo() = LcmDetection3D(
        header = header("drone/camera_optical", 7),
        results = listOf(
            hypothesisWithPose("tag36h11:7", 41.75, point(-0.375, 0.8125, 3.5)),
            hypothesisWithPose("tag36h11:12", 18.5, point(0.125, -0.0625, 6.25)),
        ),
        bbox = unsolvedBox(),
        id = DETECTION_ID,
    )

    /**
     * The id when both gates passed — **the same bare label**. Since 2026-07-29 the solved
     * fact is carried only by the message's structure (real box and orientation against NaN),
     * which is exactly what the solved fixture differs from [detection3D] by.
     */
    const val DETECTION_ID_SOLVED: String = "tag36h11:7"

    /**
     * The **solved** message: same position as [detection3D] (the solve must not move it), the
     * solved orientation where the NaNs were, the solved pose as the bbox centre and the tag's
     * flat square — `(0.075, 0.075, 0.0)` — as its size. The solved translation deliberately
     * differs from the position so an encoder writing one where the other belongs fails here.
     */
    fun detection3DSolved() = LcmDetection3D(
        header = header("drone/camera_optical", 9),
        results = listOf(
            LcmObjectHypothesisWithPose(
                hypothesis = hypothesis(),
                pose = LcmPoseWithCovariance(
                    pose = LcmPose(point(-0.375, 0.8125, 3.5), quat()),
                    covariance = ZERO_COVARIANCE_36,
                ),
            ),
        ),
        bbox = LcmBoundingBox3D(
            center = LcmPose(point(0.0625, -0.03125, 3.46875), quat()),
            size = LcmVector3(0.075, 0.075, 0.0),
        ),
        id = DETECTION_ID_SOLVED,
    )

    /** [detection3DSolved] on the wire, wrapped exactly as the encoder wraps it. */
    fun detection3DArraySolved() = LcmDetection3DArray(
        header = header("drone/camera_optical", 9),
        detections = listOf(detection3DSolved()),
    )

    /**
     * No results at all — where a hoisted length with a trailing body goes wrong most quietly.
     *
     * The encoder never produces one: it refuses rather than publishing an empty detection.
     */
    fun detection3DEmpty() = LcmDetection3D(
        header = header("drone/camera_optical", 8),
        results = emptyList(),
        bbox = unsolvedBox(),
        id = "",
    )

    /**
     * **The message the publisher actually sends**: one detection, wrapped.
     *
     * The array's header restates the element's — same seq, same stamp, same frame — which is what
     * the encoder does and what a two-element array could not do.
     */
    fun detection3DArray() = LcmDetection3DArray(
        header = header("drone/camera_optical"),
        detections = listOf(detection3D()),
    )

    /**
     * Two detections, so the **outer** count is pinned against a value the inner one is not.
     *
     * Not a message the publisher sends: `TagRecogniser` reduces each frame to its largest tag
     * before anything downstream sees a sighting.
     */
    fun detection3DArrayTwo() = LcmDetection3DArray(
        header = header("drone/camera_optical", 7),
        detections = listOf(detection3D(), detection3DTwo()),
    )

    /** No detections at all — the shape a length written after the header reads as garbage. */
    fun detection3DArrayEmpty() = LcmDetection3DArray(
        header = header("drone/camera_optical", 8),
        detections = emptyList(),
    )

    /**
     * **landing17's wind warning as the `warnings` channel carries it**, mirroring
     * `tools/lcmfixtures/generate.py` field for field.
     *
     * The values list is the `warn/` path's own: DJI's source and state word, our level, the edge,
     * the measured speed, and whether the operator's ground station was actually told. Duplicated
     * by hand from the generator on purpose — a fixture that carried its own field values would
     * let a shared typo pass on both sides.
     */
    fun windStatus() = LcmDiagnosticStatus(
        level = LcmDiagnosticStatus.LEVEL_ERROR,
        name = "wind/windWarning",
        message = "DJI: Strong wind 14.2 m/s",
        hardwareId = "",
        values = listOf(
            LcmKeyValue("source", "wind"),
            LcmKeyValue("state", "LEVEL_2"),
            LcmKeyValue("level", "WARNING"),
            LcmKeyValue("change", "appeared"),
            LcmKeyValue("measurement", "14.2 m/s"),
            LcmKeyValue("forwarded", "true"),
        ),
    )

    /** Reads a fixture from `src/test/resources/lcm/`. */
    fun bytes(name: String): ByteArray {
        val stream = LcmFixtures::class.java.getResourceAsStream("/lcm/$name.bin")
            ?: error(
                "missing fixture /lcm/$name.bin — regenerate with " +
                    "`python3 tools/lcmfixtures/generate.py`"
            )
        return stream.use { it.readBytes() }
    }
}
