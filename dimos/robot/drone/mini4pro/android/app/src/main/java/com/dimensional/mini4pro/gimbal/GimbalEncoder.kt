package com.dimensional.mini4pro.gimbal

import io.dronefleet.mavlink.common.GimbalDeviceAttitudeStatus
import io.dronefleet.mavlink.common.GimbalDeviceFlags
import io.dronefleet.mavlink.common.GimbalManagerCapFlags
import io.dronefleet.mavlink.common.GimbalManagerInformation
import io.dronefleet.mavlink.common.GimbalManagerStatus
import kotlin.math.cos
import kotlin.math.sin

/**
 * [GimbalReading] → the three MAVLink messages QGroundControl needs before it will admit a gimbal
 * exists. Pure: no DJI, no Android, no clock it was not handed, and therefore entirely under test.
 *
 * The counterpart of `telemetry/TelemetryEncoder`, and it exists for the same reason: the failure
 * modes worth fearing here are silent unit and frame errors — degrees for radians, a sign
 * inversion that points the camera at the sky, a quaternion whose element order QGC reads
 * differently from the one we wrote. Every one of those is arithmetic, and arithmetic has to be
 * testable.
 *
 * ## What QGroundControl requires, and what it actually reads
 *
 * Read out of QGC `da14fad28` (`src/Gimbal/GimbalController.cc`) on 2026-07-26. **Source-derived,
 * not measured against a running QGC** — the aircraft is powered down and the operator's QGC is
 * live on 14550 with a real aircraft, so no end-to-end run was made. Every line reference below is
 * a quote, not an inference.
 *
 * QGC registers a gimbal only when **all three** of these have arrived (`GimbalController.cc:329`):
 *
 * | message | id | when we send it |
 * |---|---|---|
 * | `GIMBAL_MANAGER_INFORMATION` | 280 | on `MAV_CMD_REQUEST_MESSAGE` param1 = 280 |
 * | `GIMBAL_MANAGER_STATUS` | 281 | streamed, 1 Hz |
 * | `GIMBAL_DEVICE_ATTITUDE_STATUS` | 285 | streamed, 5 Hz |
 *
 * `GIMBAL_DEVICE_INFORMATION` (283) and `AUTOPILOT_STATE_FOR_GIMBAL_DEVICE` (286) are **never**
 * parsed by QGC — zero references in its source — so we never send them.
 *
 * The discovery trigger is our **heartbeat**: `GimbalController::_handleHeartbeat` probes every
 * component id that heartbeats, up to six times, once the initial-connect state machine has
 * finished (`:55-59`, `:79-94`, `:270-280`). Nothing has to be done to invite it.
 *
 * Of the fields, QGC reads far fewer than the messages carry, and the ones it ignores are still
 * filled honestly here because another ground station may not ignore them:
 *
 *  - **280:** `gimbal_device_id` (must be non-zero or the message is discarded outright,
 *    `:99-103`) and `cap_flags` (only two bits do anything, see [CAP_FLAGS]). `pitch_min`/`max`,
 *    `roll_*`, `yaw_*` and `time_boot_ms` are read **nowhere**.
 *  - **281:** `gimbal_device_id` (again must be non-zero, `:137-140`) and
 *    `primary_control_sysid`/`compid`. `flags`, the secondary control ids and `time_boot_ms` are
 *    read nowhere.
 *  - **285:** `flags` and `q`. Angular velocities, `failure_flags`, `delta_yaw`,
 *    `target_system`/`target_component` and `time_boot_ms` are all ignored (`:233-263`).
 *
 * ## Why `gimbal_device_id` is 1, and why that is forced rather than chosen
 *
 * [GIMBAL_DEVICE_ID] appears in 280 and 281. It does *not* appear in the 285 we send, and it
 * cannot: **`io.dronefleet.mavlink:mavlink:1.1.11` has no `gimbalDeviceId` on
 * `GimbalDeviceAttitudeStatus`.** Upstream MAVLink added that field as a third extension after the
 * library's last release (2023-03-01; the project appears unmaintained — see PLAN.md). Omitting a
 * trailing extension is legal and does not disturb the CRC, so QGC parses our message fine and
 * zero-extends the missing field.
 *
 * QGC then takes the `gimbal_device_id == 0` branch (`:198-212`):
 *
 * ```cpp
 * if (attitude_status.gimbal_device_id == 0) {
 *     pairId.deviceId = message.compid;          // the compid we sent 285 FROM
 *     // reverse lookup: find a known gimbal whose deviceId == that compid
 *     if (foundGimbal == _potentialGimbals.constEnd()) { return; }   // else DROPPED
 * ```
 *
 * So the device id we declare in 280/281 **must numerically equal the component id we transmit
 * 285 from**, or every attitude message is silently dropped and the gimbal never completes
 * registration. `MavlinkLink` sends everything as `sysid 1, compid 1` (`MavlinkLink.kt:36-37`),
 * so the device id must be **1**. Choosing the conventional `MAV_COMP_ID_GIMBAL` (154) would
 * require a second component id on the wire — which this bridge does not have — and putting 154
 * in 285's field is not an option either: QGC drops any value above 6 outright (`:213-217`).
 *
 * If a `gimbal_device_id` is ever needed in 285, hand-roll that one annotated message class rather
 * than porting the library's 2017-era Gradle build — the same conclusion PLAN.md reached for
 * `AVAILABLE_MODES`.
 */
object GimbalEncoder {

    /** `MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION`, for `MAV_CMD_REQUEST_MESSAGE`. */
    const val MESSAGE_ID_GIMBAL_MANAGER_INFORMATION = 280

    /** `MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS`. Spelled out because we stream it unsolicited. */
    const val MESSAGE_ID_GIMBAL_MANAGER_STATUS = 281

    /** `MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS`. */
    const val MESSAGE_ID_GIMBAL_DEVICE_ATTITUDE_STATUS = 285

    /**
     * The gimbal device this bridge manages. **1, and forced to be** — see the class KDoc: it must
     * equal `MavlinkLink.COMPONENT_ID`, because the MAVLink library cannot set the field in
     * `GIMBAL_DEVICE_ATTITUDE_STATUS` and QGC therefore infers it from our component id.
     *
     * It must also be non-zero: QGC discards `GIMBAL_MANAGER_INFORMATION` and
     * `GIMBAL_MANAGER_STATUS` whose device id is 0 (`GimbalController.cc:99-103`, `:137-140`), and
     * it discards `GIMBAL_DEVICE_ATTITUDE_STATUS` whose id is above 6 (`:213-217`). 1 satisfies
     * every constraint at once.
     */
    const val GIMBAL_DEVICE_ID = 1

    /**
     * What we claim this gimbal can do — **exactly two bits, and the omissions are the point.**
     *
     * `PLAN.md`'s honesty boundary "capability flags claim only what we honour" has an unusually
     * direct consequence here, because two of these bits are the *only* thing `cap_flags` does in
     * QGroundControl. QGC reads `cap_flags` once (`GimbalController.cc:114`) and uses it for
     * nothing but two visibility bindings (`Gimbal.h:63-64`,
     * `src/Toolbar/GimbalIndicator.qml:165-173`, `:202-210`):
     *
     *  - `HAS_RETRACT` (1) → shows a **Retract** button.
     *  - `HAS_YAW_LOCK` (1024) → shows a **Yaw Lock / Yaw Follow** toggle.
     *
     * Neither is claimed:
     *
     *  - **Retract** has no MSDK equivalent on this airframe at all.
     *  - **Yaw lock** would put a control in front of the operator that this bridge cannot
     *    perform — the Mini 4 Pro refuses gimbal yaw through the SDK entirely
     *    ([#527](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/527)). A button that
     *    always fails is worse than no button, which is the same argument
     *    `command/FlightActions.kt` makes for having no Cancel.
     *
     * `HAS_YAW_AXIS` (256) and `HAS_ROLL_AXIS` (4) are likewise absent, for the same reason: this
     * gimbal has those axes physically, but **we cannot command them**, and `cap_flags` describes
     * what the *manager* offers, not what the hardware contains. Leaving them out changes no QGC
     * UI (it ignores those bits) and is the true statement.
     *
     * What is left is `HAS_PITCH_AXIS` (32) — we can command pitch — and `HAS_PITCH_LOCK` (128) —
     * the commanded pitch is held against the earth rather than against the airframe, which is
     * what a stabilised gimbal does and what `GIMBAL_DEVICE_FLAGS_PITCH_LOCK` reports back.
     *
     * The two buttons Ivan asked for survive all of this: QGC's **Center** (pitch 0) and
     * **Tilt 90** (pitch −90) are unconditionally visible whenever a gimbal is registered
     * (`GimbalIndicator.qml:175-191`), gated on no capability bit at all.
     */
    val CAP_FLAGS: List<GimbalManagerCapFlags> = listOf(
        GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS,
        GimbalManagerCapFlags.GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK,
    )

    /**
     * `GIMBAL_MANAGER_INFORMATION` (280), or **null when we have no gimbal to describe**.
     *
     * Null is the honest answer before DJI has said where the camera points, and it travels well:
     * `HandshakeResponder`'s message-provider contract turns a null into `MAV_RESULT_UNSUPPORTED`
     * for the `MAV_CMD_REQUEST_MESSAGE`, which is exactly what we want to tell a ground station
     * that asked about a gimbal we cannot see. QGC re-asks (six heartbeat-driven attempts, then up
     * to three more once any gimbal message arrives), so answering "not yet" is not answering
     * "never".
     *
     * The angle limits are DJI's own reported range converted to radians, or `NaN` where DJI has
     * not said — MAVLink's documented "unknown", and the same discipline
     * `TelemetryEncoder` applies to altitudes and attitudes. QGC reads none of these six fields,
     * so nothing here is load-bearing for it; it is filled correctly for anyone else.
     */
    fun managerInformation(reading: GimbalReading, timeBootMs: Long): GimbalManagerInformation? {
        if (!reading.isAdvertisable()) return null
        val limits = reading.limits
        return GimbalManagerInformation.builder()
            .timeBootMs(timeBootMs)
            .capFlags(CAP_FLAGS)
            .gimbalDeviceId(GIMBAL_DEVICE_ID)
            .rollMin(radiansOrNaN(limits?.rollMinDeg))
            .rollMax(radiansOrNaN(limits?.rollMaxDeg))
            .pitchMin(radiansOrNaN(limits?.pitchMinDeg))
            .pitchMax(radiansOrNaN(limits?.pitchMaxDeg))
            .yawMin(radiansOrNaN(limits?.yawMinDeg))
            .yawMax(radiansOrNaN(limits?.yawMaxDeg))
            .build()
    }

    /**
     * `GIMBAL_MANAGER_STATUS` (281), or null when there is no gimbal to report.
     *
     * [primarySysid]/[primaryCompid] are **whoever last took control through
     * `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`**, and 0/0 until someone has. That default is
     * load-bearing rather than lazy. QGC computes
     *
     * ```cpp
     * othersHaveControl = !haveControl && (primary_control_sysid != 0 && primary_control_compid != 0)
     * ```
     *
     * (`GimbalController.cc:179-180`) and `_tryGetGimbalControl` **refuses to send any operator
     * command at all** while `othersHaveControl` is true, replacing it with a "Request Gimbal
     * Control?" modal (`:346-363`). So reporting a plausible-looking non-zero controller that
     * happens not to be this QGC — 1/1, say — would lock the operator out of their own camera
     * with a dialog. 0/0 means "nobody has claimed it", QGC quietly claims it on the first
     * command, and we echo back what it claimed.
     *
     * `flags` is left empty. QGC never reads it, and the manager-level lock/follow flags describe
     * modes this bridge does not offer.
     */
    fun managerStatus(
        reading: GimbalReading,
        timeBootMs: Long,
        primarySysid: Int,
        primaryCompid: Int,
    ): GimbalManagerStatus? {
        if (!reading.isAdvertisable()) return null
        return GimbalManagerStatus.builder()
            .timeBootMs(timeBootMs)
            .gimbalDeviceId(GIMBAL_DEVICE_ID)
            .primaryControlSysid(primarySysid)
            .primaryControlCompid(primaryCompid)
            .secondaryControlSysid(0)
            .secondaryControlCompid(0)
            .build()
    }

    /**
     * `GIMBAL_DEVICE_ATTITUDE_STATUS` (285), or null when we do not know where the camera points.
     *
     * **This is the one message in the package that reports reality**, and the rule from
     * `PLAN.md` applies to it exactly as it applies to the flight mode: it carries what DJI says
     * the gimbal *is* doing, never what anyone asked it to do. [GimbalReading] has no field that
     * could hold a request, so the guarantee is structural rather than a convention — the same
     * trick `CommandDispatcher` uses to make it impossible to echo a requested flight mode.
     *
     * ### The frame flags, which are not decoration
     *
     * `GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME` (64) is set because DJI's own documentation for
     * `KeyGimbalAttitude` says *"The yaw angle uses the north east down coordinate system"*. QGC
     * branches on exactly this (`GimbalController.cc:365-375`): with the earth-frame bit set it
     * takes our yaw as an absolute azimuth and derives the body-relative one by subtracting the
     * vehicle heading; without it, it assumes the opposite and the toolbar's azimuth is wrong by
     * the aircraft's heading. Getting this bit wrong is a silent error of up to 180°.
     *
     * `ROLL_LOCK` (4) and `PITCH_LOCK` (8) are set because a DJI gimbal is stabilised against the
     * earth on both axes — that is what the mechanism is for — so a reported roll or pitch means
     * the same thing whatever the airframe is doing.
     *
     * `YAW_LOCK` (16) is set **only** when DJI reports the gimbal in `FREE` mode, i.e. holding an
     * earth-referenced heading rather than following the nose. It is read from
     * `KeyGimbalCMode`, so QGC's "Yaw locked"/"Yaw follow" status line reports the gimbal's actual
     * mode instead of an assumption. When DJI has not said, the bit is left off — see
     * [isYawLocked]. Note this cannot disturb the frame decision above: QGC only falls back to
     * inferring the frame from `YAW_LOCK` when *neither* frame bit is present (`:373`), and we
     * always set one.
     *
     * `failure_flags` is empty and the angular velocities are NaN, both honestly: DJI reports no
     * gimbal fault bits we have found, and no gimbal rates at all. QGC reads neither.
     */
    fun deviceAttitudeStatus(reading: GimbalReading, timeBootMs: Long): GimbalDeviceAttitudeStatus? {
        if (!reading.isAdvertisable()) return null
        val flags = mutableListOf(
            GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_ROLL_LOCK,
            GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_PITCH_LOCK,
            GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,
        )
        if (isYawLocked(reading.workMode)) {
            flags.add(GimbalDeviceFlags.GIMBAL_DEVICE_FLAGS_YAW_LOCK)
        }
        return GimbalDeviceAttitudeStatus.builder()
            // Broadcast. QGC ignores both fields, and there is no single ground station this
            // message is *for* — it describes the aircraft to whoever is listening.
            .targetSystem(0)
            .targetComponent(0)
            .timeBootMs(timeBootMs)
            // .toList(): the builder's Java signature is `flags(Collection<Enum>)`, which Kotlin
            // maps to an invariant mutable projection that a MutableList<GimbalDeviceFlags> does
            // not satisfy. The read-only copy does.
            .flags(flags.toList())
            .q(
                quaternionFromEulerDeg(
                    // A missing roll or yaw becomes 0 here, and only here. Everywhere else in
                    // this project a null is preserved as "unknown" — but a quaternion has no
                    // sentinel for a missing axis, and the field QGC actually uses is pitch,
                    // which isAdvertisable() has already required to be present. Reporting a
                    // level roll for an unknown roll is the smallest available lie and it is
                    // confined to this one expression.
                    rollDeg = reading.rollDeg ?: 0.0,
                    pitchDeg = reading.pitchDeg ?: 0.0,
                    yawDeg = reading.yawDeg ?: 0.0,
                )
            )
            .angularVelocityX(Float.NaN)
            .angularVelocityY(Float.NaN)
            .angularVelocityZ(Float.NaN)
            .deltaYaw(Float.NaN)
            .deltaYawVelocity(Float.NaN)
            .build()
    }

    /**
     * DJI's `GimbalMode` name → "is this yaw locked to the earth?".
     *
     * `FREE` is DJI's earth-locked mode: the gimbal holds its heading while the aircraft rotates.
     * `YAW_FOLLOW` follows the nose. `FPV` follows the airframe's attitude outright, which is the
     * opposite of locked. Anything else — including DJI's own `UNKNOWN` and a `null` from a key
     * that has not arrived — reports **not locked**, because the alternative is claiming a
     * specific gimbal behaviour on no evidence.
     *
     * Matched on the enum's name rather than the enum, because the name is what crosses the seam
     * (`AircraftState.flightMode`'s idiom) and because a DJI SDK upgrade that adds a mode then
     * reaches this `else` instead of failing to compile in a file that cannot be tested.
     */
    fun isYawLocked(workMode: String?): Boolean = workMode == "FREE"

    /**
     * Euler angles in degrees → a MAVLink `[w, x, y, z]` quaternion.
     *
     * A transcription of `mavlink_euler_to_quaternion` from the C library QGroundControl itself
     * compiles against (`mavlink_conversions.h:119-135`), because the only thing that matters
     * about this function is that QGC's `mavlink_quaternion_to_euler` gives back what we put in.
     * `GimbalEncoderTest` pins exactly that, by round-tripping through a Kotlin transcription of
     * QGC's *inverse* — quaternion → DCM → euler, singular branches included.
     *
     * **One thing the round trip does not survive, and it is the angle Ivan asked for first.**
     * `mavlink_dcm_to_euler` has an explicit gimbal-lock branch within 1 mrad of ±90° pitch
     * (`mavlink_conversions.h:73-81`) which forces `roll = 0` and computes yaw from a degenerate
     * expression. At pitch −90 — camera straight down, QGC's Tilt-90 button — **pitch comes back
     * exactly right and yaw does not**. That is a property of the Euler representation and of
     * QGC's decoder, not of this code, and there is nothing to fix: the camera's heading is
     * genuinely undefined when it is looking at its own feet. It is pinned by a test so that
     * nobody later "fixes" the round-trip failure by rotating the frame.
     *
     * Computed in `Double` and narrowed once at the end. The C version works in `float`
     * throughout; doing the trigonometry in double and rounding once is strictly closer to the
     * exact value, and the difference is far below the ~1e-7 the wire format can carry anyway.
     */
    fun quaternionFromEulerDeg(rollDeg: Double, pitchDeg: Double, yawDeg: Double): List<Float> {
        val halfRoll = Math.toRadians(rollDeg) / 2.0
        val halfPitch = Math.toRadians(pitchDeg) / 2.0
        val halfYaw = Math.toRadians(yawDeg) / 2.0
        val cr = cos(halfRoll)
        val sr = sin(halfRoll)
        val cp = cos(halfPitch)
        val sp = sin(halfPitch)
        val cy = cos(halfYaw)
        val sy = sin(halfYaw)
        return withinAsinDomain(
            floatArrayOf(
                (cr * cp * cy + sr * sp * sy).toFloat(),
                (sr * cp * cy - cr * sp * sy).toFloat(),
                (cr * sp * cy + sr * cp * sy).toFloat(),
                (cr * cp * sy - sr * sp * cy).toFloat(),
            )
        )
    }

    /**
     * Shrinks [q] by the smallest amount that keeps QGroundControl's `asinf` inside its domain,
     * and returns it unchanged whenever it already is.
     *
     * **This exists because of a real defect found by testing, at exactly the angle Ivan asked for
     * first.** QGC recovers pitch as `asinf(-dcm[2][0])` where `dcm[2][0] = 2(q1·q3 − q0·q2)`,
     * stored as a `float` (`mavlink_conversions.h:39-58`, `:72`). At a pitch of exactly ±90° that
     * quantity is exactly ±1 in real arithmetic — the very edge of `asin`'s domain — and `float`
     * rounding of the quaternion pushes it to `±1.0000001` for a large fraction of yaw values.
     * `asinf` of that is **NaN**, and a NaN reaches the operator twice over: the toolbar's pitch
     * readout becomes `NaN`, and QGC's on-screen drag is closed-loop on that same value
     * (`GimbalController.cc:449-450`) so the control stops working. Swept over every 0.1° of yaw:
     * **744 of 3601 yaw values at ±90° pitch produced an out-of-domain argument** before this fix,
     * and none do after it.
     *
     * Scaling is the right lever rather than nudging the *angle*, and the distinction is the
     * project's usual one. The angle is DJI's measurement and is not ours to alter; the
     * quaternion's magnitude is an encoding detail of a value that only represents a direction.
     * Scaling every component by `k` multiplies every DCM entry by `k²`, and QGC's roll and yaw
     * come from `atan2` of two DCM entries — scale-invariant — so **only the pitch moves, and only
     * at the singularity**.
     *
     * The residual cost is 0.034° of pitch at ±90°, measured across the same sweep. That is the
     * same order as the ~0.02° that float quaternions lose there anyway (`asin` is infinitely
     * ill-conditioned at ±1: a one-ulp representation error is already ~0.03° of angle), so the
     * fix costs nothing that was not already gone, and it rounds to the same "-90.0" QGC prints.
     */
    private fun withinAsinDomain(q: FloatArray): List<Float> {
        repeat(SHRINK_ATTEMPTS) {
            if (kotlin.math.abs(qgcSinPitch(q)) <= 1f) return q.toList()
            for (i in q.indices) q[i] *= SHRINK
        }
        return q.toList()
    }

    /**
     * `dcm[2][0]` exactly as QGroundControl computes and stores it: the products in `double`, the
     * result narrowed to `float`. Reproducing the narrowing matters — the value only leaves the
     * domain once it has been rounded to `float`.
     */
    private fun qgcSinPitch(q: FloatArray): Float =
        (2.0 * (q[1].toDouble() * q[3].toDouble() - q[0].toDouble() * q[2].toDouble())).toFloat()

    /**
     * One shrink step, a little under one ulp at 1.0 (`2⁻²³ ≈ 1.19e-7`) per component, i.e. about
     * two ulps on the squared term that overflows. Small enough that the pitch error stays inside
     * the precision already lost at the singularity; large enough that one step is almost always
     * sufficient.
     */
    private const val SHRINK = 1f - 1e-7f

    /** Give up after this many steps rather than loop. Two has always been enough in the sweep. */
    private const val SHRINK_ATTEMPTS = 8

    /**
     * Degrees → radians, or `NaN` for "we do not know".
     *
     * NaN rather than 0 for the reason `PLAN.md` gives for every other sentinel in this project: 0
     * is a real angle, and a limit of 0 would describe a gimbal that cannot move.
     */
    private fun radiansOrNaN(degrees: Double?): Float =
        degrees?.let { Math.toRadians(it).toFloat() } ?: Float.NaN
}
