package com.dimensional.mini4pro.zenoh

/**
 * The 8-byte LCM fingerprints, pinned.
 *
 * An LCM fingerprint is a hash of a type's *entire* field layout, computed
 * recursively over every nested type. It is not a version number anyone chooses:
 * it falls out of the `.lcm` definition, and any change to a field's name, type
 * or order moves it. A receiver that sees an unexpected fingerprint drops the
 * message — so a drifted fingerprint is a silent transport outage, discovered in
 * the air.
 *
 * They are therefore **constants here and asserted against fixtures in tests**,
 * not computed at runtime. A codec that derived its own fingerprint could never
 * disagree with itself, and that disagreement is exactly what is worth catching.
 *
 * ## Provenance
 *
 * Every value below was lifted on **2026-07-27** from the generated Python
 * bindings at `~/coding/dimos-lcm/generated/python_lcm_msgs/lcm_msgs/<pkg>/<Type>.py`,
 * by calling each class's `_get_packed_fingerprint()` — the same call DiMOS makes
 * when it encodes (`dimos/protocol/pubsub/encoders.py:104`). The generator
 * `tools/lcmfixtures/generate.py` re-derives them next to the byte fixtures, and
 * the first eight bytes of every fixture in
 * `android/app/src/test/resources/lcm/` are one of these — so a regeneration that
 * moves a fingerprint fails a test.
 *
 * The Java bindings under `generated/java_lcm_msgs/<pkg>/<Type>.java` compute the
 * identical values from the `LCM_FINGERPRINT_BASE` constants quoted in each KDoc
 * below. The base is the one number visible by reading rather than by running,
 * which is why it is recorded; the pinned value is the *recursive* hash, which is
 * a different number.
 *
 * Kotlin has no unsigned `Long` literal usable in a `const val`, so values above
 * `Long.MAX_VALUE` appear as their negative two's-complement form. The unsigned
 * hex — what you will see in a packet dump — is in the comment on every line.
 *
 * ## Two types share a fingerprint, and that is correct
 *
 * `geometry_msgs.Point` and `geometry_msgs.Vector3` are both `double x, y, z`
 * with identical field names, so lcm-gen hashes them identically. An LCM
 * fingerprint identifies a *layout*, not a name. Nothing on the wire can tell
 * them apart and nothing needs to: the Zenoh key carries the type name
 * (`docs/zenoh-topics.md`, "The shape of a key").
 */
object LcmFingerprints {

    // ---- builtin_interfaces ------------------------------------------------

    /**
     * `builtin_interfaces.Time` — `int32 sec, int32 nanosec`.
     * Base `0x263f39efa2a9af63` (`java_lcm_msgs/builtin_interfaces/Time.java:27`).
     *
     * **Nothing in the catalogue nests this.** Despite the ROS 2 lineage of these
     * types, lcm-gen's `std_msgs.Header` nests `std_msgs.Time` — field `nsec`,
     * not `nanosec` — and every stamped type in `docs/zenoh-topics.md` reaches
     * time through `Header`. Pinned so the distinction is recorded rather than
     * rediscovered; see [STD_MSGS_TIME] for the one that is actually used.
     */
    const val BUILTIN_INTERFACES_TIME: Long = 0x4C7E73DF45535EC6L // 0x4C7E73DF45535EC6

    // ---- std_msgs ----------------------------------------------------------

    /**
     * `std_msgs.Time` — `int32 sec, int32 nsec`.
     * Base `0xde1d24a3a8ecb648` (`java_lcm_msgs/std_msgs/Time.java:27`).
     * This is the time type inside [STD_MSGS_HEADER].
     */
    const val STD_MSGS_TIME: Long = -0x43C5B6B8AE26936FL // 0xBC3A494751D96C91

    /**
     * `std_msgs.Header` — `int32 seq, std_msgs.Time stamp, string frame_id`.
     * Base `0xdbb33f5b4c19b8ea` (`java_lcm_msgs/std_msgs/Header.java:27`).
     *
     * Note `seq`, which ROS 2 dropped from its own `Header` and lcm-gen kept.
     */
    const val STD_MSGS_HEADER: Long = 0x2FDB11453BE64AF7L // 0x2FDB11453BE64AF7

    /**
     * `std_msgs.String` — `string data`.
     * Base `0x90df9b84cdceaf0a` (`java_lcm_msgs/std_msgs/String.java:23`).
     *
     * Carries `mode`, `status`, `health` and `command_ack` — the last three as
     * JSON in `data`, per `docs/zenoh-topics.md`.
     */
    const val STD_MSGS_STRING: Long = 0x21BF37099B9D5E15L // 0x21BF37099B9D5E15

    /**
     * `std_msgs.Float32` — `float data`. **The one single-precision scalar in the catalogue.**
     * Base `0x856e135c782a36e9` (`java_lcm_msgs/std_msgs/Float32.java:24`).
     *
     * Carries `wind` — DJI's own wind-speed estimate in m/s, per `docs/zenoh-topics.md`.
     * Lifted 2026-07-30 by the same `_get_packed_fingerprint()` call as everything above,
     * when the wind reading reached the bus (Ivan: a 1-D reading rides a Float; landing14's
     * 9.1 m/s control-authority incident is why it rides at all). No nested types, so the
     * recursive hash is just the base rotated left one bit — visible here because the value
     * starts `0x0A`, one of the few in this file that stayed positive.
     */
    const val STD_MSGS_FLOAT32: Long = 0x0ADC26B8F0546DD3L // 0x0ADC26B8F0546DD3

    // ---- geometry_msgs -----------------------------------------------------

    /**
     * `geometry_msgs.Point` — `double x, y, z`.
     * Base `0x573f2fdd2f76508f` (`java_lcm_msgs/geometry_msgs/Point.java:26`).
     */
    const val GEOMETRY_MSGS_POINT: Long = -0x5181A045A1135EE2L // 0xAE7E5FBA5EECA11E

    /**
     * `geometry_msgs.Vector3` — `double x, y, z`.
     * Base `0x573f2fdd2f76508f` (`java_lcm_msgs/geometry_msgs/Vector3.java:26`).
     *
     * **Deliberately equal to [GEOMETRY_MSGS_POINT]** — same layout, same field
     * names, same hash. Kept as its own name so the `gimbal` and `gimbal_cmd`
     * channels read as what they are.
     */
    const val GEOMETRY_MSGS_VECTOR3: Long = -0x5181A045A1135EE2L // 0xAE7E5FBA5EECA11E

    /**
     * `geometry_msgs.Quaternion` — `double x, y, z, w`. **`w` is last.**
     * Base `0x9b1dee9dfc8c0515` (`java_lcm_msgs/geometry_msgs/Quaternion.java:27`).
     */
    const val GEOMETRY_MSGS_QUATERNION: Long = 0x363BDD3BF9180A2BL // 0x363BDD3BF9180A2B

    /**
     * `geometry_msgs.Pose` — `Point position, Quaternion orientation`.
     * Base `0x2d70dd60bd541272` (`java_lcm_msgs/geometry_msgs/Pose.java:25`).
     */
    const val GEOMETRY_MSGS_POSE: Long = 0x245634AE2AB17B76L // 0x245634AE2AB17B76

    /**
     * `geometry_msgs.PoseWithCovariance` — `Pose pose, double[36] covariance`.
     * Base `0x42dbdfaa69371237` (`java_lcm_msgs/geometry_msgs/PoseWithCovariance.java:28`).
     */
    const val GEOMETRY_MSGS_POSE_WITH_COVARIANCE: Long = -0x319BD74ED82EE4A6L // 0xCE6428B127D11B5A

    /**
     * `geometry_msgs.Twist` — `Vector3 linear, Vector3 angular`.
     * Base `0x3a4144772922add7` (`java_lcm_msgs/geometry_msgs/Twist.java:25`).
     */
    const val GEOMETRY_MSGS_TWIST: Long = 0x2E7C07D7CDF7E027L // 0x2E7C07D7CDF7E027

    /**
     * `geometry_msgs.TwistWithCovariance` — `Twist twist, double[36] covariance`.
     * Base `0xaba0a9d55e98da5d` (`java_lcm_msgs/geometry_msgs/TwistWithCovariance.java:28`).
     */
    const val GEOMETRY_MSGS_TWIST_WITH_COVARIANCE: Long = -0x4BC69CA5A6DE8AF7L // 0xB439635A59217509

    /**
     * `geometry_msgs.TwistStamped` — `Header header, Twist twist`.
     * Base `0xf01245422c7b28c2` (`java_lcm_msgs/geometry_msgs/TwistStamped.java:25`).
     *
     * Lifted 2026-07-29, by the same `_get_packed_fingerprint()` call as everything above, when
     * the commanded velocity reached the bus as the `setpoint` channel.
     */
    const val GEOMETRY_MSGS_TWIST_STAMPED: Long = -0x632D4341934D5840L // 0x9CD2BCBE6CB2A7C0

    /**
     * `geometry_msgs.PoseStamped` — `Header header, Pose pose`.
     * Base `0xe10feebec5c97663` (`java_lcm_msgs/geometry_msgs/PoseStamped.java:25`).
     */
    const val GEOMETRY_MSGS_POSE_STAMPED: Long = 0x6A82696458C279A0L // 0x6A82696458C279A0

    /**
     * `geometry_msgs.PointStamped` — `Header header, Point point`.
     * Base `0xf012413a2c8028c2` (`java_lcm_msgs/geometry_msgs/PointStamped.java:25`).
     */
    const val GEOMETRY_MSGS_POINT_STAMPED: Long = -0x63289B8C7159D651L // 0x9CD764738EA629AF

    /**
     * `geometry_msgs.Transform` — `Vector3 translation, Quaternion rotation`.
     * Base `0xf694f4a6d8779002` (`java_lcm_msgs/geometry_msgs/Transform.java:25`).
     *
     * **Not equal to [GEOMETRY_MSGS_POSE]**, despite the same two nested layouts in the same
     * order: lcm-gen hashes the *field names* too, and `translation`/`rotation` are not
     * `position`/`orientation`. So the wire bytes of a `Pose` and a `Transform` are identical and
     * the eight bytes in front of them are not, which is exactly the discrimination a fingerprint
     * exists to provide.
     */
    const val GEOMETRY_MSGS_TRANSFORM: Long = -0x11A00BD9B840C06FL // 0xEE5FF42647BF3F91

    /**
     * `geometry_msgs.TransformStamped` — `Header header, string child_frame_id,
     * Transform transform`.
     * Base `0x448d6658328c5ebc` (`java_lcm_msgs/geometry_msgs/TransformStamped.java:26`).
     */
    const val GEOMETRY_MSGS_TRANSFORM_STAMPED: Long = 0x299FF424B83A3514L // 0x299FF424B83A3514

    // ---- tf2_msgs ----------------------------------------------------------

    /**
     * `tf2_msgs.TFMessage` — `int32 transforms_length, TransformStamped[transforms_length]`.
     * Base `0x398a869d05983f0e` (`java_lcm_msgs/tf2_msgs/TFMessage.java:26`).
     *
     * The length field is encoded **before** the array and there is no header at all — this type
     * is the array and nothing else. See `LcmWire`.
     */
    const val TF2_MSGS_TF_MESSAGE: Long = -0x3D475E3CC576DC14L // 0xC2B8A1C33A8923EC

    // ---- foxglove_msgs -----------------------------------------------------

    /**
     * `foxglove_msgs.CompressedVideo` — `int32 data_length,
     * builtin_interfaces.Time timestamp, string frame_id, byte[data_length] data, string format`.
     * Base `0xb4ea6258bc6d0702` (`java_lcm_msgs/foxglove_msgs/CompressedVideo.java:29`).
     *
     * The recursive hash is taken over **[BUILTIN_INTERFACES_TIME]**, not [STD_MSGS_TIME]: this is
     * the one type in the catalogue that carries a bare stamp rather than a `std_msgs.Header`, and
     * it is why that constant is pinned at all rather than merely recorded.
     */
    const val FOXGLOVE_MSGS_COMPRESSED_VIDEO: Long = 0x2217B46EEFBF7B05L // 0x2217B46EEFBF7B05

    // ---- nav_msgs ----------------------------------------------------------

    /**
     * `nav_msgs.Odometry` — `Header header, string child_frame_id,
     * PoseWithCovariance pose, TwistWithCovariance twist`.
     * Base `0x97f82279756d9d18` (`java_lcm_msgs/nav_msgs/Odometry.java:29`).
     */
    const val NAV_MSGS_ODOMETRY: Long = -0x6B1E806B9B730F1CL // 0x94E17F94648CF0E4

    /**
     * `nav_msgs.Path` — `int32 poses_length, Header header, PoseStamped[poses_length] poses`.
     * Base `0xc779b6acc503055a` (`java_lcm_msgs/nav_msgs/Path.java:29`).
     *
     * The length field is encoded **before** the header. See `LcmWire`.
     */
    const val NAV_MSGS_PATH: Long = -0x3C519D534CA86C1EL // 0xC3AE62ACB35793E2

    // ---- sensor_msgs -------------------------------------------------------

    /**
     * `sensor_msgs.NavSatStatus` — `int8 status, int16 service`.
     * Base `0x76b236592075c1db` (`java_lcm_msgs/sensor_msgs/NavSatStatus.java:26`).
     */
    const val SENSOR_MSGS_NAV_SAT_STATUS: Long = -0x129B934DBF147C4AL // 0xED646CB240EB83B6

    /**
     * `sensor_msgs.NavSatFix` — `Header header, NavSatStatus status,
     * double latitude, longitude, altitude, double[9] position_covariance,
     * int8 position_covariance_type`.
     * Base `0x4a84d20526d9067a` (`java_lcm_msgs/sensor_msgs/NavSatFix.java:37`).
     */
    const val SENSOR_MSGS_NAV_SAT_FIX: Long = -0x30776006B8AA55B2L // 0xCF889FF94755AA4E

    /**
     * `sensor_msgs.Imu` — `Header header, Quaternion orientation, double[9],
     * Vector3 angular_velocity, double[9], Vector3 linear_acceleration, double[9]`.
     * Base `0x55c1e238541325f6` (`java_lcm_msgs/sensor_msgs/Imu.java:34`).
     */
    const val SENSOR_MSGS_IMU: Long = 0x31AB205C8DD57AA8L // 0x31AB205C8DD57AA8

    /**
     * `sensor_msgs.BatteryState` — two array-length `int32`s first, then
     * `Header`, seven `float`s, three `int8`s, a `boolean`, the two `float[]`
     * bodies, and two `string`s.
     * Base `0x8f419fb94c3b774d` (`java_lcm_msgs/sensor_msgs/BatteryState.java:52`).
     */
    const val SENSOR_MSGS_BATTERY_STATE: Long = 0x7E3961FD10438489L // 0x7E3961FD10438489

    /**
     * `sensor_msgs.RegionOfInterest` — `int32 x_offset, y_offset, height, width, boolean
     * do_rectify`.
     * Base `0x37bc5cbce50a5ce2` (`java_lcm_msgs/sensor_msgs/RegionOfInterest.java:29`).
     */
    const val SENSOR_MSGS_REGION_OF_INTEREST: Long = 0x73150D3A0B307E1CL // 0x73150D3A0B307E1C

    /**
     * `sensor_msgs.CameraInfo` — `int32 D_length` first, then `Header`, `int32 height, width`,
     * `string distortion_model`, `double[D_length] D`, `double[9] K`, `double[9] R`,
     * `double[12] P`, `int32 binning_x, binning_y`, `RegionOfInterest roi`.
     * Base `0x1275bd1ccbdaf47f` (`java_lcm_msgs/sensor_msgs/CameraInfo.java:42`).
     *
     * Note the field order the ROS `.msg` does **not** have: `height` before `width`, and the
     * variable array's length hoisted ahead of the header. Both are lcm-gen's, both are pinned by
     * a fixture generated from DiMOS's own Python.
     */
    const val SENSOR_MSGS_CAMERA_INFO: Long = -0x504AFE4FF8F85FD6L // 0xAFB501B00707A02A

    // ---- vision_msgs -------------------------------------------------------
    //
    // Lifted 2026-07-28 from the same bindings and by the same call as everything above, when the
    // on-board AprilTag detector's sightings reached the bus.

    /**
     * `vision_msgs.ObjectHypothesis` — `string class_id, double score`.
     * Base `0x38c9acd543138b22` (`java_lcm_msgs/vision_msgs/ObjectHypothesis.java:25`).
     */
    const val VISION_MSGS_OBJECT_HYPOTHESIS: Long = 0x719359AA86271644L // 0x719359AA86271644

    /**
     * `vision_msgs.ObjectHypothesisWithPose` — `ObjectHypothesis hypothesis,
     * geometry_msgs.PoseWithCovariance pose`.
     * Base `0x65e1d44b451e8a8b` (`java_lcm_msgs/vision_msgs/ObjectHypothesisWithPose.java:25`).
     */
    const val VISION_MSGS_OBJECT_HYPOTHESIS_WITH_POSE: Long =
        0x4BB2AD4DE62D7853L // 0x4BB2AD4DE62D7853

    /**
     * `vision_msgs.BoundingBox3D` — `geometry_msgs.Pose center, geometry_msgs.Vector3 size`.
     * Base `0xe10feec5cba89663` (`java_lcm_msgs/vision_msgs/BoundingBox3D.java:25`).
     */
    const val VISION_MSGS_BOUNDING_BOX_3D: Long = 0x67C9065CAA8D65EFL // 0x67C9065CAA8D65EF

    /**
     * `vision_msgs.Detection3D` — `int32 results_length, std_msgs.Header header,
     * ObjectHypothesisWithPose[results_length] results, BoundingBox3D bbox, string id`.
     * Base `0x7c62020c10a78d22` (`java_lcm_msgs/vision_msgs/Detection3D.java:34`).
     *
     * The length field is encoded **before** the header — lcm-gen's hoisting again, the same shape
     * as [NAV_MSGS_PATH] and [SENSOR_MSGS_CAMERA_INFO]. Unlike either of those the array here is
     * followed by two more fields, so a length written from anywhere but the list shifts `bbox`
     * and `id` rather than merely truncating.
     */
    const val VISION_MSGS_DETECTION_3D: Long = -0x408E7208456E934AL // 0xBF718DF7BA916CB6

    /**
     * `vision_msgs.Detection3DArray` — `int32 detections_length, std_msgs.Header header,
     * Detection3D[detections_length] detections`.
     * Base `0x85b4a076ba01be3c` (`java_lcm_msgs/vision_msgs/Detection3DArray.java:30`).
     *
     * **This is the type on the wire**, since 2026-07-28; [VISION_MSGS_DETECTION_3D] is what it
     * carries and is still checked in its own right. Exactly [NAV_MSGS_PATH]'s shape — a hoisted
     * length, a header, and nothing after the array — which is what makes the wrapping cheap and
     * the mistake to fear the same one: a count written from anywhere but the list.
     */
    const val VISION_MSGS_DETECTION_3D_ARRAY: Long = -0x15FD80989F0D142EL // 0xEA027F6760F2EBD2

    // ---- diagnostic_msgs ---------------------------------------------------
    //
    // Lifted 2026-07-30, by the same `_get_packed_fingerprint()` call as everything above, run
    // against the bindings DiMOS itself imports
    // (`~/coding/dimos/.venv/lib/python3.12/site-packages/dimos_lcm/diagnostic_msgs/`), when the
    // `warnings` channel was added. The family exists upstream, which is why this channel carries
    // a typed diagnostic rather than the JSON-in-a-String shape `status` and `command_ack` use:
    // Ivan's standing rule is standard types where the family has one.

    /**
     * `diagnostic_msgs.KeyValue` — `string key, string value`.
     * Base `0x97574015d52eedde` (`DiagnosticStatus`'s only nested type).
     */
    const val DIAGNOSTIC_MSGS_KEY_VALUE: Long = 0x2EAE802BAA5DDBBDL // 0x2EAE802BAA5DDBBD

    /**
     * `diagnostic_msgs.DiagnosticStatus` — `int32 values_length, int8 level, string name, string
     * message, string hardware_id, KeyValue[values_length] values`.
     * Base `0x3e3fb00c69778dfb`.
     *
     * **The hoisted length sits ahead of `level`, not next to the array** — lcm-gen's usual
     * hoisting, and the one shape mistake this type invites: writing the count late shifts three
     * strings rather than truncating a list, so the bytes stay plausible.
     */
    const val DIAGNOSTIC_MSGS_DIAGNOSTIC_STATUS: Long = -0x26239F8FD8552C90L // 0xD9DC607027AAD370

    /**
     * `diagnostic_msgs.DiagnosticArray` — `int32 status_length, std_msgs.Header header,
     * DiagnosticStatus[status_length] status`.
     * Base `0x163c308c500b94d`.
     *
     * **This is the type on the wire** for `warnings`, since 2026-07-30. Exactly
     * [VISION_MSGS_DETECTION_3D_ARRAY]'s shape — a hoisted length, a header, and nothing after the
     * array.
     */
    const val DIAGNOSTIC_MSGS_DIAGNOSTIC_ARRAY: Long = 0x1636697C5123AF68L // 0x1636697C5123AF68
}
