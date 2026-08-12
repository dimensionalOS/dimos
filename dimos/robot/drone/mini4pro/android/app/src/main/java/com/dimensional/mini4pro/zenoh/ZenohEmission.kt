package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.vision.TagSighting

/**
 * Why a channel produced nothing for one sample. [PUBLISHED] is the not-withheld case, carried in
 * the same enum so a per-channel tally always sums to the sample count.
 *
 * The names say **which input and which failure**, because "no position" and "a position we are
 * not willing to repeat" are different bugs and one of them is [Geo]'s one-number-in-both-fields
 * filler — the thing that produced 220 fabricated `HOME_POSITION` messages on 2026-07-26.
 */
enum class Withheld {
    PUBLISHED,

    /** No latitude/longitude at all, or a pair [Geo] refuses. */
    POSITION_INVALID,
    POSITION_STALE,
    ALTITUDE_MISSING,
    ALTITUDE_STALE,
    ATTITUDE_MISSING,
    ATTITUDE_STALE,
    VELOCITY_MISSING,
    VELOCITY_STALE,

    /** No charge percentage, or one outside 0..100 — not a percentage, so not clamped. */
    BATTERY_MISSING,

    /** DJI never named a flight mode, or named it with a blank string. */
    MODE_MISSING,

    /** No `odom` origin has been recorded yet, so nothing local can be expressed. */
    NO_DATUM,

    /**
     * The flight controller link was down, so a held reading has nothing vouching for it.
     *
     * The counterpart of the `_STALE` reasons, and the only one that can withhold a dense
     * channel. A stale key means *the value has not changed*; a dead link means *we no longer
     * know*. Every data key falls silent together, so only [AircraftState.fcConnected] — which is
     * not a data key — can tell the two apart.
     */
    LINK_DOWN,

    /**
     * Nothing was wrong — the channel simply was not due. `battery` is 1 Hz against a 5 Hz
     * sampler and `mode` is on-change, so most samples of those two land here. Kept distinct from
     * every other reason precisely because it is not a withholding.
     */
    RATE_LIMITED,

    /** The record carries no gimbal attitude at all. Not a gap in the flight — a gap in the schema. */
    NOT_RECORDED,

    /**
     * No video frame has yet stated a width and a height, so there is nothing to build intrinsics
     * for.
     *
     * `camera_info`'s only unknown. It is a real absence rather than a rate limit: `LogEntry.Frame`
     * writes the geometry *on change plus once at the start*, precisely because a resolution change
     * invalidates the intrinsics and so has to be a discrete stated fact rather than something a
     * reader infers. Before any resolution has been stated there is nothing to hold and nothing is
     * published — never a guessed 1920×1080.
     */
    RESOLUTION_UNKNOWN,

    /**
     * A tag was seen and its **range** could not be had, so there is no pose to publish.
     *
     * `detections`' only unknown, and it is a real absence rather than a rate limit.
     * `TagRecogniser.publish` substitutes a range of `0.0` when the tag's apparent size implies
     * none, and `TagWorld.cameraFrame` then returns `(0, 0, 0)` — the camera's own optical centre,
     * which is a place and a confidently wrong one. The sighting still reaches the flight record;
     * what does not reach the bus is a position we do not have.
     */
    RANGE_UNKNOWN,

    /**
     * A tag was seen and **no world fix could be made of it** — `TagWorld.fix` returned null
     * at the frame (no position, no heading, no believed camera angle, or a camera not near
     * nadir), so the record's `tag` line carries no `n`/`e` and the `tag_fix` channel has
     * nothing to say. The sighting itself still travels on `detections`; what does not travel
     * is a place we do not have — `tag_fix`'s only unknown, and a real absence.
     */
    FIX_UNAVAILABLE,

    /**
     * A `stick_cmd` that was **never handed to the SDK** — shadow mode's would-be commands,
     * recorded with `accepted = null` because no SDK call happened and there is no verdict to
     * record. The channel is *"commands we are sending to the drone"* (Ivan), and a shadow
     * command is precisely one we did not send; publishing it would put a phantom setpoint on
     * a bus other software may correlate against real motion. The record keeps the full
     * would-be stream — that is shadow mode's entire purpose — and the discriminator is the
     * record's own: a real send always carries a verdict (`performSend` writes `accepted =
     * (error == null)`), a shadow line never does.
     */
    NOT_SENT,

    /**
     * A `stick_cmd` in a frame the `setpoint` channel has no conversion for — anything but
     * `NED_VELOCITY`, which is the only frame the engine flies today. Refused by name rather
     * than converted here, because a frame translation inside a transport would be a second
     * guidance implementation; if the engine ever flies another frame, the conversion belongs
     * in the encoder with its own tests, not in a fallback.
     */
    SETPOINT_FRAME_UNSUPPORTED,

    /**
     * A `stick_cmd` whose setpoint is missing an axis, non-finite, or absent entirely. A
     * partial twist padded with zeros would claim commands that were never given; silence is
     * the honest encoding, exactly as §4 of the encoder's doc draws the line.
     */
    SETPOINT_INCOMPLETE,

    /**
     * A `windSpeedDmS` delivery with no value — DJI withdrawing the reading, which the record
     * writes as a line with `prev` and no `v` (landing14's last wind line is one). Zero is a
     * calm day and a real measurement; this is the absence of one, and encoding it as `0.0`
     * would be the project's characteristic failure. `wind`'s only refusal.
     */
    WIND_MISSING,
}

/**
 * **The one place a per-sample publish decision is taken**, shared by the live Android publisher
 * and by `replay/ZenohReplay`'s offline driver.
 *
 * ## Why this is one object and not two
 *
 * `ZenohReplay` was written first, before any transport existed, and its KDoc said so: *"There is
 * no Zenoh session here and no publisher: the transport does not exist yet. What exists is every
 * decision a publisher would make."* When the transport arrived, those decisions had exactly two
 * possible fates — be re-derived beside the live path, or be lifted out and called from both. They
 * are lifted out, because a re-derivation is a second implementation that agrees today and drifts
 * silently, and the whole diagnostic value of the offline driver is that its counts describe what
 * the *live* publisher would have done.
 *
 * The direction of the dependency is deliberate and is not reversible: **`replay/` depends on
 * `zenoh/`, never the other way round.** `replay/ReplayAdmission` exists so that a recorded flight
 * cannot reach a live path, and a live publisher importing the replay package would be that rule
 * pointing the wrong way.
 *
 * ## What it does and does not decide
 *
 * It decides *whether* and *why*, from one immutable [AircraftState] and one origin. It does not
 * decide *when* — cadence is [ZenohCadence]'s, because a replay schedules from a recorded `t`
 * column and a live publisher publishes when a reading arrives, and `docs/zenoh-replay-contract.md`
 * §8 is explicit that only the *observable* timing contract is shared and the mechanism is not.
 *
 * It never reads a clock, never touches Zenoh, never touches Android and never touches DJI, so
 * every rule below is a JVM assertion.
 *
 * ## The precedence, and why it is stated once
 *
 * Each channel's reason is the **first** of its inputs that failed, in the encoder's own order.
 * That ordering is not cosmetic: on the reference flight `odom` is withheld for 78 % of samples
 * and the honest diagnosis is `ALTITUDE_STALE` — `KeyAltitude` arriving in 5–6 s bursts against
 * `Signal.ALTITUDE`'s 1 s limit — not `POSITION_INVALID`, even on the samples where both are true.
 * A tally that named the wrong input would send somebody to fix a fix that was never broken.
 *
 * `ZenohReplayTest` and `OrbitReplayTest` assert the mirror is exact against 1855 real samples:
 * for every channel, the count this class calls published equals the count for which
 * [ZenohTelemetryEncoder] actually returned a message.
 */
object ZenohEmission {

    /**
     * One channel's outcome for one sample: the reason, and the LCM bytes when there are any.
     *
     * [bytes] is null whenever [reason] is not [Withheld.PUBLISHED], and also when the caller
     * asked for reasons only — an offline run over a 371 s flight is ~8000 frames and several
     * megabytes, and a test that wants counts should not pay for them.
     */
    class Emission(val channel: ZenohChannel, val reason: Withheld, val bytes: ByteArray?) {
        val published: Boolean get() = reason == Withheld.PUBLISHED
    }

    /**
     * Every telemetry channel's outcome for one sample, in catalogue order.
     *
     * @param datum this flight's origin, or null when none has been recorded yet — which makes
     *   the two local channels [Withheld.NO_DATUM] and leaves the global ones untouched, because
     *   `gps_location`, `imu`, `battery` and `mode` need no origin to mean something.
     * @param stamp the reading's own time, never the send time (D-5). The caller supplies it so
     *   this function reads no clock and every message is a deterministic function of its inputs.
     * @param encode false to tally without encoding.
     */
    fun emit(
        s: AircraftState,
        datum: OdomDatum?,
        stamp: LcmTime,
        encode: Boolean = true,
    ): List<Emission> {
        // The four gates, each evaluated under both rules, in the encoder's own order. A channel
        // something *flies* on (Gate.FRESH) may only carry a reading that arrived; a dense
        // published stream (Gate.HELD) may repeat the last one for as long as the link is up.
        val positionFresh = positionReason(s, Gate.FRESH)
        val altitudeFresh = altitudeReason(s, Gate.FRESH)
        val attitudeFresh = attitudeReason(s, Gate.FRESH)
        val velocityFresh = velocityReason(s)

        val positionHeld = positionReason(s, Gate.HELD)
        val altitudeHeld = altitudeReason(s, Gate.HELD)
        val attitudeHeld = attitudeReason(s, Gate.HELD)

        val out = ArrayList<Emission>(ZenohChannel.entries.size)

        // pose = position + altitude + attitude + an origin, all held.
        val poseReason = when {
            datum == null -> Withheld.NO_DATUM
            positionHeld != Withheld.PUBLISHED -> positionHeld
            altitudeHeld != Withheld.PUBLISHED -> altitudeHeld
            else -> attitudeHeld
        }
        out += emission(ZenohChannel.POSE, poseReason, encode) {
            ZenohTelemetryEncoder.poseStampedOrNull(s, datum!!, stamp)?.let(PoseStampedCodec::encode)
        }

        // odom = the same four inputs as pose plus velocity, but every one of them fresh: it is
        // the only telemetry channel a controller closes a loop on.
        val odomReason = when {
            datum == null -> Withheld.NO_DATUM
            positionFresh != Withheld.PUBLISHED -> positionFresh
            altitudeFresh != Withheld.PUBLISHED -> altitudeFresh
            attitudeFresh != Withheld.PUBLISHED -> attitudeFresh
            else -> velocityFresh
        }
        out += emission(ZenohChannel.ODOM, odomReason, encode) {
            ZenohTelemetryEncoder.odometryOrNull(s, datum!!, stamp)?.let(OdometryCodec::encode)
        }

        // gps_location = position + altitude. No datum: it is the global fix.
        val gpsReason = if (positionHeld != Withheld.PUBLISHED) positionHeld else altitudeHeld
        out += emission(ZenohChannel.GPS_LOCATION, gpsReason, encode) {
            ZenohTelemetryEncoder.gpsLocationOrNull(s, stamp)?.let(NavSatFixCodec::encode)
        }

        out += emission(ZenohChannel.IMU, attitudeHeld, encode) {
            ZenohTelemetryEncoder.imuOrNull(s, stamp)?.let(ImuCodec::encode)
        }

        val batteryReason =
            if (s.batteryPercent?.takeIf { it in 0..100 } != null) Withheld.PUBLISHED
            else Withheld.BATTERY_MISSING
        out += emission(ZenohChannel.BATTERY, batteryReason, encode) {
            ZenohTelemetryEncoder.batteryOrNull(s, stamp)?.let(BatteryStateCodec::encode)
        }

        val modeReason =
            if (ZenohTelemetryEncoder.flightModeOrNull(s) != null) Withheld.PUBLISHED
            else Withheld.MODE_MISSING
        out += emission(ZenohChannel.MODE, modeReason, encode) {
            ZenohTelemetryEncoder.flightModeOrNull(s)?.let(StringCodec::encode)
        }

        // `status` is event-driven and has no per-sample answer; the gimbal is retired from the
        // bus and absent from every flight record. Both are named rather than omitted so a tally
        // over the channels this function decides always sums to the sample count.
        out += Emission(ZenohChannel.STATUS, Withheld.RATE_LIMITED, null)
        out += Emission(ZenohChannel.GIMBAL, Withheld.NOT_RECORDED, null)
        return out
    }

    /**
     * **The four channels this function does not decide, and why the boundary is here.**
     *
     * `tf`, `camera_info`, `video` and `detections` are absent from [emit]'s output — not
     * withheld, not tallied, simply not this function's to answer. The rule is not a convenience:
     * **[emit] is a pure function of one [AircraftState] and one origin**, and none of the four
     * can be answered from those.
     *
     *  - **`tf`** needs the gimbal's angle, which lives behind `gimbal/`'s own seam and is in no
     *    flight-state snapshot. Its `world` → `base_link` edge alone *would* be derivable here,
     *    and publishing a tree with the camera edge silently missing whenever the gimbal is
     *    unknown is exactly the half-truth this project refuses elsewhere.
     *  - **`camera_info`** needs the video stream's stated geometry, which arrives on MSDK's
     *    decode thread and not in a state sample.
     *  - **`video`** is frame-driven at ~43 fps against this function's 5 Hz and is published by
     *    [ZenohVideoPublisher] on its own queue, thread and session.
     *  - **`detections`** is driven by what the camera can see, not by what the aircraft reports.
     *    Nothing in an [AircraftState] says whether a tag was in frame, and a channel gated on
     *    the aircraft would publish on a schedule rather than on evidence — which is the whole
     *    difference `docs/tag-detector.md` §3 draws between arming on geometry and arming on a
     *    detection. Its per-sighting rule is [detection], below.
     *
     * [ZenohTelemetryPump] owns the first two and holds the inputs for them; the video publisher
     * owns the third; [ZenohBus] holds the detector's lambda for the fourth. This is stated here
     * rather than left to be noticed because a future edit that "completes" [emit] by adding them
     * would be reaching for state this function is defined not to have.
     */
    val NOT_DECIDED_HERE: List<ZenohChannel> = listOf(
        ZenohChannel.TF,
        ZenohChannel.CAMERA_INFO,
        ZenohChannel.VIDEO,
        ZenohChannel.DETECTIONS,
        // `tag_fix` is `detections`' world-frame companion and shares its shape exactly:
        // driven by what the camera saw, not by what the aircraft reports. Its per-sighting
        // rule is [tagFix], below.
        ZenohChannel.TAG_FIX,
        // `setpoint` is driven by what this bridge *commanded*, which is in no AircraftState
        // at all — the send is the event, exactly `detections`' shape. Its per-send rule is
        // [setpoint], below; [ZenohBus.publishSetpoint] is the live caller and the flight
        // record's `stick_cmd` line is the offline one (`tools/memexport`).
        ZenohChannel.SETPOINT,
        // `wind` is driven by DJI *delivering* a change-driven key, and wind is deliberately
        // in no AircraftState: a per-sample answer here would be exactly the resampling the
        // channel's contract forbids — a held reading republished as a fresh measurement on a
        // key whose silence means UNCHANGED. Its per-delivery rule is [wind], below;
        // [ZenohBus.publishWind] is the live caller (fed by the recorder's own on-change tap)
        // and the record's `windSpeedDmS` line is the offline one (`tools/memexport`).
        ZenohChannel.WIND,
    )

    /**
     * One sighting's outcome on the `detections` channel.
     *
     * Written here rather than in [ZenohBus] for [ZenohEmission]'s founding reason: the decision
     * *whether* to publish and the reason *why not* belong in the one object both the live path
     * and any offline driver can call, and the Android adapter is deliberately the file that
     * supplies clocks and forwards bytes and takes no decisions.
     *
     * The **switch** is not asked here. `detections` is off by default and that is a session-level
     * configuration, the same shape as `video`'s — a channel that is off does not exist, so there
     * is nothing to withhold and no reason to record. [ZenohBus] returns before it reaches this.
     *
     * @param stamp the **frame's arrival** on the wall clock, never the send time (D-5). The
     *   caller derives it from `Sighting.ageMillisAt`, which exists precisely because the number
     *   is 60–160 ms and a consumer assuming currency would be wrong in the direction that hurts.
     */
    fun detection(
        sighting: TagSighting.Sighting,
        stamp: LcmTime,
        seq: Int,
        encode: Boolean = true,
    ): Emission = emission(ZenohChannel.DETECTIONS, detectionReason(sighting), encode) {
        // The **array**, which is what the channel is typed as. `detectionsOrNull` refuses exactly
        // when `detectionOrNull` does — it is a wrapper and nothing else — so [detectionReason]
        // stands in front of both and the drift check in [emission] covers the pair.
        ZenohTelemetryEncoder.detectionsOrNull(sighting, stamp, seq)
            ?.let(Detection3DArrayCodec::encode)
    }

    /**
     * Why a sighting produced no message, or [Withheld.PUBLISHED].
     *
     * **Mirrors [ZenohTelemetryEncoder.detectionOrNull] exactly**, in its order, as every gate in
     * this file mirrors the encoder it stands in front of — and [emission] throws if the two ever
     * disagree, rather than counting a publish that did not happen.
     *
     * Note what is *not* a reason. `metric = false` does not withhold: that decision was taken
     * deliberately (`docs/tag-detector.md` §7) and it is a statement about the *quality* of a
     * range which is nevertheless the best-known quantity in the message, not about its absence.
     * The caveat lives in the contract row (since 2026-07-29; before that it rode the id as a
     * suffix). Neither does a non-zero `hamming`:
     * the detector already ships `maxhamming = 1` and the latch already requires three sightings,
     * and a second, unmeasured threshold here would be policy invented at a transport.
     */
    fun detectionReason(sighting: TagSighting.Sighting): Withheld = when {
        sighting.imageWidth <= 0 || sighting.imageHeight <= 0 -> Withheld.RESOLUTION_UNKNOWN
        !sighting.x.isFinite() || !sighting.y.isFinite() -> Withheld.RANGE_UNKNOWN
        !sighting.z.isFinite() || sighting.z <= 0.0 -> Withheld.RANGE_UNKNOWN
        else -> Withheld.PUBLISHED
    }

    /**
     * One sighting's outcome on the `tag_fix` channel — the world-frame companion of
     * [detection], deciding from **the record's own fix values** and nothing else.
     *
     * The parameters are exactly the `tag` line's fix half, which is the rule that keeps the
     * live caller honest: [ZenohBus.publishTagFix] hands the same `TagFix` fields the line was
     * written from, `tools/kotlinframes` hands the line's own values back, and neither can
     * publish a fix the other would not — re-deriving a world position here (from the
     * camera-frame pose and whatever state is current) is the composition artifact this
     * channel exists to kill, reintroduced at the source.
     *
     * @param stamp the **frame's arrival** (D-5), same instant the sighting's `detections`
     *   message carries — one frame, one time, two frames of reference.
     */
    fun tagFix(
        tagId: Int,
        decisionMargin: Double,
        northM: Double?,
        eastM: Double?,
        fixMetric: Boolean,
        rangeSource: String?,
        pitchReported: Boolean,
        stamp: LcmTime,
        seq: Int,
        encode: Boolean = true,
    ): Emission = emission(ZenohChannel.TAG_FIX, tagFixReason(northM, eastM), encode) {
        ZenohTelemetryEncoder.tagFixOrNull(
            tagId, decisionMargin, northM, eastM,
            fixMetric, rangeSource, pitchReported, stamp, seq,
        )?.let(Detection3DArrayCodec::encode)
    }

    /**
     * Why a sighting produced no `tag_fix` message, or [Withheld.PUBLISHED]. Mirrors
     * [ZenohTelemetryEncoder.tagFixOrNull] exactly: the fix is present and finite, or there
     * is nothing to say — [Withheld.FIX_UNAVAILABLE] is the record's own null, not a judgment
     * this transport makes.
     */
    fun tagFixReason(northM: Double?, eastM: Double?): Withheld = when {
        northM?.takeIf { it.isFinite() } == null ||
            eastM?.takeIf { it.isFinite() } == null -> Withheld.FIX_UNAVAILABLE
        else -> Withheld.PUBLISHED
    }

    /**
     * One virtual-stick send's outcome on the `setpoint` channel.
     *
     * Written here for the founding reason [detection] restates: the decision *whether* and the
     * reason *why not* live in the one object every caller shares — [ZenohBus.publishSetpoint]
     * live, `tools/kotlinframes` offline over the record's `stick_cmd` lines — so the live bus
     * and a record-derived store cannot come to different verdicts about the same command.
     *
     * @param accepted the SDK's verdict as the record carries it: true/false for a send that
     *   actually reached DJI (refusals included — a refused send *was* sent), **null for a
     *   command that was never handed to the SDK at all** (shadow mode), which is refused as
     *   [Withheld.NOT_SENT]. The verdict itself is not on the message — the record holds it —
     *   only the fact that a send happened gates publishing.
     * @param stamp the send's own instant, never the publish time (D-5).
     */
    fun setpoint(
        frame: String?,
        northMps: Double?,
        eastMps: Double?,
        downMps: Double?,
        yawRateDegPerS: Double?,
        accepted: Boolean?,
        stamp: LcmTime,
        encode: Boolean = true,
    ): Emission = emission(
        ZenohChannel.SETPOINT,
        setpointReason(frame, northMps, eastMps, downMps, yawRateDegPerS, accepted),
        encode,
    ) {
        ZenohTelemetryEncoder.setpointOrNull(frame, northMps, eastMps, downMps, yawRateDegPerS, stamp)
            ?.let(TwistStampedCodec::encode)
    }

    /**
     * Why a send produced no `setpoint` message, or [Withheld.PUBLISHED].
     *
     * **Mirrors [ZenohTelemetryEncoder.setpointOrNull] exactly, in its order**, plus the one
     * gate the encoder deliberately does not own: [Withheld.NOT_SENT] comes first, because a
     * command that never went to the SDK must not be published however well-formed it is —
     * the encoder answers "can this be expressed", this answers "did it happen". [emission]
     * throws if the two ever disagree on the expressible half.
     */
    fun setpointReason(
        frame: String?,
        northMps: Double?,
        eastMps: Double?,
        downMps: Double?,
        yawRateDegPerS: Double?,
        accepted: Boolean?,
    ): Withheld = when {
        accepted == null -> Withheld.NOT_SENT
        // A line with no setpoint object at all is an absence, not an unsupported frame.
        frame == null -> Withheld.SETPOINT_INCOMPLETE
        frame != com.dimensional.mini4pro.record.SetpointFrame.NED_VELOCITY ->
            Withheld.SETPOINT_FRAME_UNSUPPORTED
        northMps?.takeIf { it.isFinite() } == null ||
            eastMps?.takeIf { it.isFinite() } == null ||
            downMps?.takeIf { it.isFinite() } == null ||
            yawRateDegPerS?.takeIf { it.isFinite() } == null -> Withheld.SETPOINT_INCOMPLETE
        else -> Withheld.PUBLISHED
    }

    /**
     * One wind delivery's outcome on the `wind` channel.
     *
     * Written here for the founding reason [detection] and [setpoint] restate: the decision
     * *whether* and the reason *why not* live in the one object every caller shares —
     * [ZenohBus.publishWind] live, `tools/kotlinframes` offline over the record's
     * `windSpeedDmS` lines — so the live bus and a record-derived store cannot come to
     * different verdicts about the same delivery.
     *
     * **The delivery is the record line.** The recorder's on-change dedup
     * (`Recorder.field`) is the single owner of "this delivery is a new fact", and the live
     * tap fires this exactly when a line was written — so one line, one message, on both
     * sides, which is what the kotlinframes cross-check counts. No stamp parameter:
     * `std_msgs.Float32` carries none (see [LcmFloat32]).
     */
    fun wind(speedDmS: Int?, encode: Boolean = true): Emission =
        emission(ZenohChannel.WIND, windReason(speedDmS), encode) {
            ZenohTelemetryEncoder.windOrNull(speedDmS)?.let(Float32Codec::encode)
        }

    /**
     * Why a delivery produced no `wind` message, or [Withheld.PUBLISHED]. Mirrors
     * [ZenohTelemetryEncoder.windOrNull] exactly: a value, or nothing — zero publishes
     * (a calm day is a measurement), null is [Withheld.WIND_MISSING] (DJI withdrew the
     * reading, and fabricating a 0.0 for it is the unknown-is-never-zero failure).
     */
    fun windReason(speedDmS: Int?): Withheld =
        if (speedDmS == null) Withheld.WIND_MISSING else Withheld.PUBLISHED

    /**
     * **One decided warning as `diagnostic_msgs.DiagnosticArray`** — the `warnings` channel.
     *
     * Nothing is decided here. The event arrives from `warn/WarningMonitor` with its severity, its
     * sentence and its diagnostic level already chosen, and this function's whole job is to express
     * it: **the same object QGC's `STATUSTEXT`, the phone's line and the record's `dji_warn` entry
     * are rendered from**, which is the property that keeps four surfaces from disagreeing. There
     * is no [Withheld] branch and no null return, and that is the point — every event reaches the
     * bus, including the ones QGC is never told about (`NORMAL`) and the ones the rate bound
     * suppressed. A subscriber is a machine, and a machine is not protected by being told less.
     *
     * @param stamp wall-clock, "now": a warning's delivery *is* its event, unlike a detection,
     *   whose stamp is the frame's arrival (see [ZenohBus.publishDetection]'s subtraction).
     * @param seq the header's own counter, so a subscriber can see a gap.
     */
    fun warning(
        event: com.dimensional.mini4pro.warn.WarnEvent,
        stamp: LcmTime,
        seq: Int,
        encode: Boolean = true,
    ): Emission = emission(ZenohChannel.WARNINGS, Withheld.PUBLISHED, encode) {
        DiagnosticArrayCodec.encode(ZenohTelemetryEncoder.warning(event, stamp, seq))
    }

    /**
     * Whether the frame tree can be published for this sample, and why not when it cannot.
     *
     * **Exactly `pose`'s rule**, and deliberately the same three gates in the same order rather
     * than a second opinion about them: the tree's `world` → `base_link` edge carries the same
     * position and the same attitude the pose does, so there is no defensible reason for the two
     * to disagree about whether that is publishable. The camera edge inherits it too, because it
     * composes the attitude away and cannot exist without one.
     */
    fun tfReason(s: AircraftState, datum: OdomDatum?): Withheld {
        if (datum == null) return Withheld.NO_DATUM
        val position = positionReason(s, Gate.HELD)
        if (position != Withheld.PUBLISHED) return position
        val altitude = altitudeReason(s, Gate.HELD)
        if (altitude != Withheld.PUBLISHED) return altitude
        return attitudeReason(s, Gate.HELD)
    }

    /**
     * Builds one outcome, and **fails loudly rather than quietly** if the reason and the encoder
     * disagree.
     *
     * The two are independent statements about the same sample: [emit] composes the reason from
     * the gates, and the encoder decides for itself whether to return a message. They must agree
     * on every sample of every flight, and when they do not it is this class's diagnosis that is a
     * fiction — so a published reason with no bytes throws instead of publishing nothing and
     * counting it as a publish.
     */
    private inline fun emission(
        channel: ZenohChannel,
        reason: Withheld,
        encode: Boolean,
        bytes: () -> ByteArray?,
    ): Emission {
        if (reason != Withheld.PUBLISHED || !encode) return Emission(channel, reason, null)
        val encoded = bytes()
            ?: throw IllegalStateException(
                "$channel: the gates said PUBLISHED and the encoder returned nothing — " +
                    "ZenohEmission has drifted from ZenohTelemetryEncoder",
            )
        return Emission(channel, reason, encoded)
    }

    // ── the four gates, mirroring ZenohTelemetryEncoder exactly ──────────────
    //
    // Each gate is asked twice per sample, because the encoder asks it twice. The missing/invalid
    // checks are identical in both cases — a value that was never read is not held, it is absent.

    /** Which reason [gate] gives for a signal that is present: published, stale, or link-down. */
    private fun gated(s: AircraftState, gate: Gate, signal: Signal, stale: Withheld): Withheld =
        when (gate) {
            Gate.FRESH -> if (s.isFresh(signal)) Withheld.PUBLISHED else stale
            Gate.HELD -> if (s.fcConnected) Withheld.PUBLISHED else Withheld.LINK_DOWN
        }

    fun positionReason(s: AircraftState, gate: Gate = Gate.FRESH): Withheld {
        if (Geo.coordinateOrNull(s.latitude, s.longitude) == null) return Withheld.POSITION_INVALID
        return gated(s, gate, Signal.POSITION, Withheld.POSITION_STALE)
    }

    fun altitudeReason(s: AircraftState, gate: Gate = Gate.FRESH): Withheld {
        if (s.relativeAltitude?.takeIf { it.isFinite() } == null) return Withheld.ALTITUDE_MISSING
        return gated(s, gate, Signal.ALTITUDE, Withheld.ALTITUDE_STALE)
    }

    fun attitudeReason(s: AircraftState, gate: Gate = Gate.FRESH): Withheld {
        if (s.rollDeg?.takeIf { it.isFinite() } == null ||
            s.pitchDeg?.takeIf { it.isFinite() } == null ||
            s.yawDeg?.takeIf { it.isFinite() } == null
        ) {
            return Withheld.ATTITUDE_MISSING
        }
        return gated(s, gate, Signal.ATTITUDE, Withheld.ATTITUDE_STALE)
    }

    /**
     * Velocity has **no held form**, and that asymmetry is the point: a twist is a claim about
     * motion *now*, `KeyAircraftVelocity` is change-driven, and a stale reading is genuinely
     * indistinguishable from a stationary aircraft. Publishing the cached number is the one thing
     * that makes those two look different when they are not.
     */
    fun velocityReason(s: AircraftState): Withheld {
        if (s.velocityNorth?.takeIf { it.isFinite() } == null ||
            s.velocityEast?.takeIf { it.isFinite() } == null ||
            s.velocityDown?.takeIf { it.isFinite() } == null
        ) {
            return Withheld.VELOCITY_MISSING
        }
        return gated(s, Gate.FRESH, Signal.VELOCITY, Withheld.VELOCITY_STALE)
    }
}
