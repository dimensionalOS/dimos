package com.dimensional.mini4pro.vision

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * **Where the aircraft was when it saw the tag**, in the terms this geometry needs and no others.
 *
 * DJI-free by construction — the wiring layer projects `AircraftState` onto this — for the reason
 * the whole project follows: a pure function over primitives is testable, and a function that
 * reaches into a state cache is not.
 *
 * **[cameraPitchDeg] is the angle the bridge *believes*, commanded first, reported as the
 * fallback** — `gimbal/PitchBelief`'s resolution, never a raw readback age-gated by anyone.
 * The commanded angle wins when it exists because it is exact and never goes stale; DJI's
 * `KeyGimbalAttitude` is change-driven and goes silent exactly when the camera is holding still,
 * which is the whole of a nadir approach — the trap this project has hit seven times, and the
 * reason no consumer of the reported value may gate on its age (silence means *unchanged*).
 * When nothing has been commanded this session, the last-*reported* angle is the honest fallback
 * — the RC-wheel sessions of 2026-07-28, where a camera genuinely at −90° produced no fixes at
 * all because this field was fed from the command alone — and [cameraPitchReported] says so, so
 * every fix carries which belief it rested on. Null means neither source has said anything,
 * which is not the same as nadir and must not be treated as it.
 */
data class CameraPose(
    /** Metres north of the takeoff datum. Null when position is unknown. */
    val northM: Double?,
    /** Metres east of the takeoff datum. */
    val eastM: Double?,
    /** Height above the takeoff datum, metres. Null when unknown — and unknown is not zero. */
    val relativeAltitudeM: Double?,
    /** Aircraft heading, degrees clockwise from true north. */
    val headingDeg: Double?,
    /** Believed camera pitch, degrees, −90 at nadir. See the class KDoc. */
    val cameraPitchDeg: Double?,
    /**
     * True when [cameraPitchDeg] is the **reported** gimbal attitude rather than a commanded
     * angle — `gimbal/PitchBelief.reported`, carried onto every fix as [TagFix.pitchReported].
     */
    val cameraPitchReported: Boolean = false,
)

/**
 * **Which instrument a range rests on** — the provenance of every range this pipeline resolves,
 * in ladder order, strongest first. Carried on every [TagFix] ([TagFix.rangeSource]) and onto
 * the descent's own height resolution (`TagDescentGuidance.descentHeight`), so a post-flight
 * reader is never left inferring which instrument a number was.
 *
 * The ladder itself — who outranks whom, and why — is [TagWorld.fix]'s (the fix's scale) and
 * `descentHeight`'s (the law's height); this enum only names the rungs, once, for both.
 */
enum class RangeSource {
    /** A trusted PnP solve's translation — the tag's printed geometry, [TagPose.trusted]-gated. */
    SOLVE,

    /** `f·S/pixels` from the tag's apparent size — [TagWorld.rangeFromSize], pixel-gated. */
    SIZE,

    /** The barometric relative altitude — the fallback, and the measured liar (landing07). */
    BARO,
}

/**
 * **A tag's position in `drone/world`**, and everything that is wrong with it, on the same object.
 *
 * The fields that say what it is worth are not optional decoration. A landing controller reading
 * this is reading a number derived from a focal length that was *fitted* from two flights, a
 * principal point assumed to be the image centre, a distortion model that does not exist, and a
 * camera-to-body rotation nobody has measured. Every one of those is a real error and every one is
 * named here rather than in a document somebody may not have read.
 */
data class TagFix(
    val tagId: Int,
    /** Metres north of the takeoff datum. */
    val northM: Double,
    /** Metres east of the takeoff datum. */
    val eastM: Double,
    /**
     * The height above the datum the aircraft was at when it made this fix. Not the tag's height:
     * the tag is *assumed* to be on the datum plane, and if it is standing on a board it is not.
     * The 25 Hz flight measured that board at +0.33 m by accident; a tape measure would beat it.
     *
     * **Barometric on every fix, whatever [rangeSource] says** — this field is the record of
     * what the barometer claimed at the frame, kept even now that the range ladder outranks it
     * (landing07 measured it lying by ~1.2 m within a minute), precisely so a post-flight
     * reader can hold the two instruments against each other on every fix ever made. What it
     * is *not*, since landing07, is what the descent laws fly: the fix's own scale is the
     * ladder's ([rangeSource]), and the law's height is `TagDescentGuidance.descentHeight`'s.
     */
    val fromHeightM: Double,
    /** Monotonic nanoseconds, from the frame's arrival — not from when the fix was computed. */
    val atNanos: Long,
    /** The tag's longest edge in pixels in the frame this came from. The trust proxy. */
    val pixelSize: Double,
    /**
     * **True when this fix's scale is the tag's own printed geometry** — a solved pose that
     * passed [TagPose.trusted], its translation carried into the world instead of a pixel ray
     * scaled by the barometer. False is the bearing×altitude fix every flight before 2026-07-28
     * flew on.
     *
     * What "metric" honestly buys **today** is *altitude-independence*, not lateral accuracy:
     * the baro is measured dishonest exactly where landings end (−0.1 m at rest on the ground,
     * `landingdata.md` §4; landing06's own final metres put the solved range up to 0.46 m away
     * from the baro), and this fix's lateral does not ride it. What it does **not** buy yet is
     * precision — the intrinsics under the solve are still the assumed prior, whose measured
     * cost is a ~3° systematic tilt floor (≈5 cm lateral at 1 m,
     * `2026-07-28-pose-solve-stability.md` §5) until the chessboard session replaces
     * [CameraCalibration.ASSUMED]. A consumer that needs to know which camera model a metric
     * fix rested on reads the session's `camera_calibration` record event.
     */
    val metric: Boolean = false,
    /**
     * **True while the camera-to-body rotation is an assumption rather than a measurement —
     * and that is every fix, metric ones included.**
     *
     * A bearing in the camera's frame is solid: it needs only a focal length and a pixel offset.
     * Turning it into north and east needs to know how the image is oriented on the airframe when
     * the gimbal is at nadir, and **nothing in this project has measured that.** [TagWorld] assumes
     * the convention in [TagWorld.NADIR_IMAGE_UP_IS_NOSE] and marks every fix it produces. The
     * metric path changes where the *scale* comes from, not this rotation: the solved translation
     * crosses camera→body through the identical assumed convention, so a trusted solve does not
     * clear this flag — the one manoeuvre that would (hover nadir, translate, watch the tag slide)
     * still has not been flown.
     */
    val bearingAssumed: Boolean = true,
    /**
     * **True when the camera pitch this fix's geometry rested on was the *reported* gimbal
     * attitude rather than an angle this bridge commanded** — [CameraPose.cameraPitchReported],
     * carried with the number it qualifies exactly as [bearingAssumed] is.
     *
     * The two beliefs fail differently, which is why a reader must be able to tell them apart:
     * a commanded angle can be overridden by the RC wheel after the ask, while a reported one is
     * DJI's own last delivery and lags a moving gimbal by at most its change-driven delivery.
     * False is the normal case once anything (the takeoff sequence, QGC's Tilt 90) has aimed the
     * camera; true is the RC-wheel session, which is what both 2026-07-28 denial records are.
     */
    val pitchReported: Boolean = false,
    /**
     * **The solved range to the tag along the optical axis, metres — present exactly when
     * [metric] is true.** The trusted solve's `tz`, from the tag's printed size and nothing
     * else: no barometer in it, which makes it the only range this project has that stays
     * honest through a landing's last metre.
     *
     * The withholding this field shipped with — *"not fed to the descent laws: a range-fed
     * height law is a separate, flight-gated decision"* — was **resolved by landing07**
     * (`datasets/landing07/20260729-095413.001.jsonl`, 2026-07-29): the barometer drifted a
     * full metre during a ~40 s flight (read 0.00 on the ground pre-takeoff, t≈101–104, then
     * "0.7 m" at a terminal hold whose tag measured 51–57 px ⇒ true range ~1.9–2.1 m), and
     * every descent law downstream was faithful to the poisoned height — a 46 cm miss through
     * a 10.6 s blind final. This range is tag-anchored and carries none of that drift, so
     * `TagDescentGuidance.descentHeight` now flies on it when the fix is fresh and metric,
     * with the baro as the fallback. Null on a bearing fix — the range there *was* the
     * altitude, and duplicating [fromHeightM] here would let a reader believe a measurement
     * that never happened.
     */
    val rangeM: Double? = null,
    /**
     * **The size-implied range along the optical axis, metres** — `f·S/pixels` through
     * [TagWorld.rangeFromSize], computed once per frame by the recogniser and carried here so
     * the fix and its scale ride the same frame. When [rangeSource] is [RangeSource.SIZE]
     * this IS the range the fix's lateral was scaled by; on a metric fix it is the solve's
     * independent sibling, kept for the record's cross-reading. Independent of the barometer
     * by construction — landing07's landing B measured this instrument *right* (1.93–2.13 m
     * at 51–57 px) while the baro was ~1.2 m wrong. Null only on a fix whose producer carried
     * none (degenerate pixels, or a fix built by code older than the field).
     */
    val sizeRangeM: Double? = null,
    /**
     * **Which rung of the range ladder scaled this fix's geometry** — [RangeSource.SOLVE] for
     * a metric fix (the trusted solve's translation), [RangeSource.SIZE] for a bearing fix
     * whose pixel offset was scaled by the tag's own apparent size, [RangeSource.BARO] for a
     * bearing fix that fell all the way to the altitude (tag too small for its size to be a
     * range — [TagWorld.SIZE_RANGE_MIN_PIXELS]). The provenance every consumer of this fix's
     * *scale* reads, and the record's `height_source` speaks in these names.
     */
    val rangeSource: RangeSource = RangeSource.BARO,
) {
    /**
     * **The tag-derived range this fix rests on, or null when it rests on the barometer** —
     * the ladder's answer, read off the provenance rather than copied into a fourth field
     * (single owner per property: [rangeM] holds the solve's number, [sizeRangeM] the size's,
     * [fromHeightM] the baro's; this maps [rangeSource] to the one that scaled the fix).
     * Null means "this fix has no range the tag itself vouches for": a consumer resolving a
     * height (the descent's ladder) must fall back to its own current baro rather than to
     * [fromHeightM], which is the *frame's* baro reading, already aging.
     */
    fun tagRangeM(): Double? = when (rangeSource) {
        RangeSource.SOLVE -> rangeM
        RangeSource.SIZE -> sizeRangeM
        RangeSource.BARO -> null
    }
}

/**
 * **Pixels to a place, with every assumption named.**
 *
 * Pure, and every input a primitive, so the whole of it runs in a plain JVM test. That matters more
 * here than anywhere else in the package: this is the only part whose output is a number a
 * controller might fly on, and it is the part an aircraft cannot check — an aircraft can tell you
 * the detector saw a tag, and it cannot tell you the tag was really 1.4 m north.
 *
 * Every camera number it computes with comes from one [CameraCalibration] — the same object the
 * JNI pose solve's intrinsics come from, resolved once per session in `Bridge` — so the solve and
 * this projection cannot disagree about the instrument. The functions default to
 * [CameraCalibration.ASSUMED] because that *is* the shipped camera model; the recogniser passes
 * its session's calibration explicitly.
 *
 * ## What is solid and what is not
 *
 * | step | rests on | status |
 * |---|---|---|
 * | pixel offset → bearing | focal length, and the pixel that means "straight down" | focal length **fitted**, ±1.2 % across two flights; the nadir pixel **measured** 2026-07-28 to ±2.3/±4.0 px — [CameraCalibration.nadirPointX] |
 * | bearing → range | the range ladder: the tag's apparent size when ≥ [SIZE_RANGE_MIN_PIXELS], else the aircraft's height above the tag plane | size range **fitted** (5–8 cm rms in-band); baro **measured telemetry**, 0.1 m-quantised — and measured lying by ~1.2 m within a minute (landing07) |
 * | solved pose → range | the tag's printed size through the PnP solve | **trusted-gated** ([TagPose.trusted]); the metric path, no altitude in it |
 * | camera frame → body frame | how the image sits on the airframe at nadir | **assumed**, see [NADIR_IMAGE_UP_IS_NOSE] — both paths |
 * | body frame → world | aircraft heading | measured telemetry |
 *
 * ## The one place a sighting becomes a place: [fix] decides metric or bearing
 *
 * A detection that carries a solve which [TagPose.trusted] believes produces a **metric** fix —
 * the solved translation carried through the frame chain below. Anything else — no solve, solve
 * below the pixel gate, ambiguous solve, partial solve — produces the bearing×altitude fix,
 * byte-identically the pre-metric behaviour. The gate is *called*, never restated: its
 * thresholds ([TagPose.MIN_SOLVE_PIXELS], [TagPose.MAX_AMBIGUITY_RATIO]) live in one object with
 * their measurements, and a copy here would be a second gate that could drift. With the shipped
 * 75 mm marker the pixel gate means the switch happens at ~1.8 m: a landing's final metres fly
 * on metric fixes, the approach above flies on bearing ones, and the record's `fix_metric` flag
 * shows exactly where.
 *
 * ## The metric frame chain, explicitly
 *
 * ```
 * (tx, ty, tz)  tag centre, metres, drone/camera_optical — x right, y down-image, z out the
 *               lens; scale from the printed tag ([TagPoseSolve], apriltag's own estimator)
 *   ↓ nadir-ray correction (the same measured 2.99° the bearing path corrects)
 * camX = tx − tz·(nadirX − cx)/fx      camY = ty − tz·(nadirY − cy)/fy      (at-1920 numbers)
 *   ↓ camera → body, the ASSUMED rotation, same two lines as the bearing path
 * forward = −camY   right = camX      ([NADIR_IMAGE_UP_IS_NOSE])
 *   ↓ body → world, aircraft heading clockwise from north
 * north += forward·cos(yaw) − right·sin(yaw)   east += forward·sin(yaw) + right·cos(yaw)
 * ```
 *
 * The correction step exists because the solve's translation is referred to the calibration's
 * optical axis (the assumed image centre) while "beneath the aircraft" is the **measured** nadir
 * pixel, 2.99° away — the identical correction [fix]'s bearing path applies via
 * [CameraCalibration.nadirPointX], applied here in the tangent plane at range `tz`. Without it
 * the two paths would disagree by a systematic 5.2 cm/m and the switch between them would step.
 * The small-angle form (`tan θ` for the rotation) is exact to 0.15 % at 2.99°, far under the
 * solve's own ~3° tilt floor. When `tz` equals the altitude the metric lateral therefore equals
 * the bearing lateral algebraically — landing06's real solves reproduce it to millimetres with
 * the ranges disagreeing by up to 0.46 m, which is the whole point: same direction, honest scale.
 *
 * ## The range ladder: the tag in view is the truth-teller, the barometer is the fallback
 *
 * The bearing path's range comes from a **ladder** — [RangeSource], resolved here and only here
 * for the fix's scale: the tag's apparent size (`z = f · S / pixels`) when the tag is
 * comfortably measurable ([SIZE_RANGE_MIN_PIXELS]), the relative altitude only when the tag
 * offers no usable range. The original design flew the altitude ("measured telemetry, does not
 * degrade with tag size") and **landing07 refuted it in the air**
 * (`datasets/landing07/20260729-095413.001.jsonl`, landing B, 2026-07-29): the baro drifted a
 * full metre during a ~40 s flight and read "0.7 m" at a hold whose tag measured 51–57 px —
 * size range 1.93–2.13 m, correct — so every bearing fix's lateral was under-scaled ~2.7×, the
 * hold wandered on errors it could not see at true size, and the commit fired "in-cone at
 * 0.7 m" while genuinely ~0.5 m off at ~1.9 m: a 46 cm miss through a 10.6 s blind final. The
 * instrument that was *right* was carried on every sighting and flown on by nothing. Design
 * authority for the flip: Ivan, 2026-07-29 — *"we should land if we see the tag, don't be
 * fragile insisting on baro."*
 *
 * **The consistency requirement the ladder exists to keep** (what actually dissolves landing
 * B): the range that scales a bearing fix's lateral offset and the height the descent law
 * flies must be the **same number** — one owner for "the range this sighting rests on". The
 * fix resolves it once, carries the provenance ([TagFix.rangeSource]), and
 * `TagDescentGuidance.descentHeight` reads the same resolution off the fix. Two consumers
 * reading two instruments for one property is the two-places-for-one-property failure, and
 * landing B is what it costs in the air.
 */
object TagWorld {

    /**
     * **The assumption this file cannot get rid of by thinking.**
     *
     * With the gimbal at nadir, is the top of the image the direction the nose points? It is the
     * conventional mounting and it is what DJI's own preview looks like, and **nobody here has
     * flown the one manoeuvre that would prove it**: hover nadir over a tag, translate north, and
     * see which way the tag slides in the frame.
     *
     * Every [TagFix] carries [TagFix.bearingAssumed] until that is done — the metric path passes
     * through this same constant, so a trusted solve does not retire it. Naming the constant
     * means the fix is one edit and one line of a measurement document rather than a hunt
     * through trigonometry.
     */
    const val NADIR_IMAGE_UP_IS_NOSE = true

    /**
     * How far from nadir the **believed** camera pitch may be before a fix is refused, in degrees.
     *
     * A tilted camera does not make the geometry impossible — it makes it depend on the pitch angle,
     * on the gimbal's lever arm from the aircraft's centre (§1.4 of the recording document, also
     * unmeasured), and on the ground being flat under a slanted ray rather than under a plumb line.
     * Each of those is a further unmeasured term stacked on the ones already here. Refusing is
     * honest; a fix computed through four assumptions and labelled with one is not. The metric
     * path is judged by the same tolerance: a solved translation crossed into the world through
     * [NADIR_IMAGE_UP_IS_NOSE] assumes a nadir camera exactly as a pixel ray does.
     */
    const val NADIR_TOLERANCE_DEG = 12.0

    /**
     * **The size rung's admission floor: below this many pixels of longest edge, the tag's
     * apparent size is not a range and the ladder falls to the barometer.**
     *
     * From the fit the size range rests on (`2026-07-27-tag-detection-rate.md` §3, two
     * flights): the `range ∝ f·S/pixels` model fits at 5–8 cm rms across the measured band,
     * and the document's own warning is that the residual **triples once tags below about
     * 20 px are included** — at and below ~5 px per cell the size is "a reliable presence
     * signal and a much less reliable range". 20 px of 75 mm tag is ~5.5 m of range, so in
     * practice: the approach above ~5.5 m flies baro (where the baro's errors matter least,
     * and the 7 m arm ceiling still consumes it), the descent from ~5.5 m down flies the
     * tag's own size, and below ~1.8 m ([TagPose.MIN_SOLVE_PIXELS] = 60 px) the trusted solve
     * outranks both. Landing07's landing B sat at 51–57 px — comfortably above this floor,
     * which is why its size range was right while its baro was a metre wrong.
     */
    const val SIZE_RANGE_MIN_PIXELS = 20.0

    /**
     * The focal length in pixels for a frame [width] px across — [CameraCalibration.fxPx] of the
     * session's calibration, here as a convenience for callers that hold none. The numbers and
     * their provenance live on [CameraCalibration]; this object deliberately owns no copy.
     */
    fun focalPx(width: Int, cal: CameraCalibration = CameraCalibration.ASSUMED): Double =
        cal.fxPx(width)

    /** The pixel straight-down appears at, x. [CameraCalibration.nadirPointX], measured. */
    fun nadirPointX(width: Int, cal: CameraCalibration = CameraCalibration.ASSUMED): Double =
        cal.nadirPointX(width)

    /** The pixel straight-down appears at, y. See [nadirPointX]. */
    fun nadirPointY(
        width: Int,
        height: Int,
        cal: CameraCalibration = CameraCalibration.ASSUMED,
    ): Double = cal.nadirPointY(width, height)

    /**
     * The tag's offset from the point directly beneath the aircraft, metres, at range [rangeM].
     *
     * `first` is to the image's right, `second` is down the image — the same axes [cameraFrame]
     * uses and deliberately not north/east, for the reason [cameraFrame] gives. The difference from
     * [cameraFrame] is only the reference point, and that difference is the whole correction: this
     * one is measured against true nadir, the other against an assumed optical axis.
     */
    fun nadirFrame(
        centreX: Double,
        centreY: Double,
        width: Int,
        height: Int,
        rangeM: Double,
        cal: CameraCalibration = CameraCalibration.ASSUMED,
    ): Pair<Double, Double> {
        val f = cal.fxPx(width)
        if (f <= 0.0 || !rangeM.isFinite()) return Pair(0.0, 0.0)
        val dx = centreX - cal.nadirPointX(width)
        val dy = centreY - cal.nadirPointY(width, height)
        return Pair(dx / f * rangeM, dy / f * rangeM)
    }

    /**
     * The tag's direction in the **camera's** frame, as a unit-range triple scaled by [rangeM].
     *
     * x right, y down in the image, z along the optical axis — the frame `TagSighting.Sighting`
     * documents, and deliberately not converted to north/east here, because that conversion needs
     * the aircraft's yaw and the camera's pointing and burying two assumptions inside a number that
     * looks like a measurement is exactly what this package is arranged to avoid.
     *
     * Referred to the calibration's principal point — the image centre while
     * [CameraCalibration.ASSUMED] stands (nothing has measured it; 20 px of error at 1457 px
     * focal length is a 0.8° bearing error — small against everything else here, and real), and
     * the chessboard's answer the day one lands in the calibration file. Deliberately **not**
     * the nadir pixel: that is `principal + gimbal error` summed, the wrong reference for an
     * optical-frame direction — [CameraCalibration.nadirPointX] documents the trap.
     */
    fun cameraFrame(
        centreX: Double,
        centreY: Double,
        width: Int,
        height: Int,
        rangeM: Double,
        cal: CameraCalibration = CameraCalibration.ASSUMED,
    ): Triple<Double, Double, Double> {
        val f = cal.fxPx(width)
        if (f <= 0.0 || !rangeM.isFinite()) return Triple(0.0, 0.0, 0.0)
        val dx = centreX - cal.cxPx(width)
        val dy = centreY - cal.cyPx(width, height)
        return Triple(dx / f * rangeM, dy / f * rangeM, rangeM)
    }

    /**
     * The range implied by the tag's **apparent size**, metres, or null when it cannot be had.
     *
     * The ladder's SIZE rung ([fix] flies it when the tag is measurable — the class KDoc's
     * argument, landing07's lesson) and the divergence record's tag-side number. [tagSizeM] is
     * the printed marker's black square, side to side.
     */
    fun rangeFromSize(
        longestEdgePixels: Double,
        width: Int,
        tagSizeM: Double,
        cal: CameraCalibration = CameraCalibration.ASSUMED,
    ): Double? {
        if (longestEdgePixels <= 0.0 || tagSizeM <= 0.0) return null
        return cal.fxPx(width) * tagSizeM / longestEdgePixels
    }

    /**
     * The tag's place in `drone/world`, or **null when any of the things it rests on is missing**.
     *
     * Null is returned, rather than a fix with a fabricated component, whenever position, altitude,
     * heading or a believed camera angle is unknown, or the camera is not pointing near nadir. That
     * follows the project's standing rule that unknown is never zero: a fix computed with a heading
     * of 0 because none had arrived is a confidently wrong place, and a landing controller cannot
     * tell it from a right one.
     *
     * **The refusal set is identical on both paths, altitude included.** The metric path does not
     * geometrically need the barometer, but [TagFix.fromHeightM] still carries it for every
     * consumer that reads height off the fix, and producing metric fixes at baro ≤ 0 — where the
     * bearing path has always refused — would move the *when-fixes-exist* behaviour of arm gates
     * and staleness ladders in the same pass that changes the arithmetic. Freeing the metric path
     * from the baro's sign is real (the last 10 cm of a landing sit exactly there) and it is a
     * flight-gated follow-up beside the range-fed height law, not a rider on this change.
     */
    fun fix(
        detection: TagDetection,
        width: Int,
        height: Int,
        pose: CameraPose,
        atNanos: Long,
        cal: CameraCalibration = CameraCalibration.ASSUMED,
        /**
         * The frame's own size-implied range ([rangeFromSize]) — the ladder's SIZE rung's
         * input, and carried onto [TagFix.sizeRangeM]. Supplied by the caller rather than
         * recomputed here because the recogniser already computes it once per frame (it needs
         * the tag's printed size, which this function deliberately does not take) and a second
         * computation would be a second place for the same fact to drift. Null is carried as
         * null — absent, never 0 — and an absent size range simply cannot win the ladder.
         */
        sizeRangeM: Double? = null,
    ): TagFix? {
        val north = pose.northM ?: return null
        val east = pose.eastM ?: return null
        val alt = pose.relativeAltitudeM ?: return null
        val heading = pose.headingDeg ?: return null
        val pitch = pose.cameraPitchDeg ?: return null
        // Nadir is −90°. Anything else stacks unmeasured terms — see NADIR_TOLERANCE_DEG. The
        // tolerance judges the believed pitch whatever its source: a reported −60° is refused
        // exactly as a commanded −60° is, because the geometry does not care who said the number.
        if (abs(pitch - (-90.0)) > NADIR_TOLERANCE_DEG) return null
        // At or below the tag plane there is no downward ray to intersect, and a negative range
        // would silently mirror the offset through the aircraft. Applied before the path split —
        // see the KDoc for why the metric path keeps this gate it does not geometrically need.
        if (!alt.isFinite() || alt <= 0.0) return null

        // **The switch, and the only copy of it.** A solve the measured gates believe makes the
        // fix metric; anything less and the fix is the bearing×altitude one, byte-identical to
        // every fix flown before the metric path existed.
        val trusted = TagPose.trusted(detection.longestEdgePixels, detection.solve)

        val camX: Double
        val camY: Double
        val rangeSource: RangeSource
        if (trusted != null) {
            // The solved translation, corrected from the calibration's optical axis to the
            // measured nadir ray — the class KDoc's frame chain, step by step. At-1920 numbers
            // throughout: the correction is an angle, and angles do not scale with width.
            camX = trusted.tx - trusted.tz * (cal.nadirXAt1920 - cal.cxAt1920) / cal.fxAt1920
            camY = trusted.ty - trusted.tz * (cal.nadirYAt1920 - cal.cyAt1920) / cal.fyAt1920
            rangeSource = RangeSource.SOLVE
        } else {
            // **The ladder's SIZE rung — the class KDoc's argument, landing07's lesson.** The
            // pixel offset is scaled by the tag's own size-implied range when the tag is
            // comfortably measurable, and by the altitude only when it is not: the tag in
            // view is the truth-teller, the baro the measured liar. The floor is the fit's
            // own admission bound (SIZE_RANGE_MIN_PIXELS), so a 10 px smudge's "range" is
            // never believed over an instrument that is merely drifting.
            val sizeUsable = sizeRangeM != null && sizeRangeM.isFinite() && sizeRangeM > 0.0 &&
                detection.longestEdgePixels >= SIZE_RANGE_MIN_PIXELS
            val range = if (sizeUsable) sizeRangeM!! else alt
            rangeSource = if (sizeUsable) RangeSource.SIZE else RangeSource.BARO
            // **[nadirFrame], not [cameraFrame].** The next step multiplies a pixel offset by
            // a range and calls the answer a ground offset, which is only true of a ray
            // pointing straight down. Straight down is the measured nadir pixel, 2.99° from
            // the image centre this used to subtract.
            val (x, y) = nadirFrame(detection.centreX, detection.centreY, width, height, range, cal)
            camX = x
            camY = y
        }

        // Camera frame → body frame. **The assumption**, isolated to these two lines so that one
        // measurement changes one constant — and shared by both paths, which is why a metric fix
        // still carries bearingAssumed. Up in the image is the nose, so a tag *above* centre
        // (smaller y) is *ahead*; right in the image is the starboard wing.
        val forwardM = if (NADIR_IMAGE_UP_IS_NOSE) -camY else camY
        val rightM = camX

        // Body frame → world. Heading is clockwise from north, so a heading of 90° puts the nose
        // east and the starboard side south.
        val yaw = Math.toRadians(heading)
        val dNorth = forwardM * cos(yaw) - rightM * sin(yaw)
        val dEast = forwardM * sin(yaw) + rightM * cos(yaw)

        return TagFix(
            tagId = detection.id,
            northM = north + dNorth,
            eastM = east + dEast,
            fromHeightM = alt,
            atNanos = atNanos,
            pixelSize = detection.longestEdgePixels,
            metric = trusted != null,
            bearingAssumed = true,
            pitchReported = pose.cameraPitchReported,
            rangeM = trusted?.tz,
            sizeRangeM = sizeRangeM,
            rangeSource = rangeSource,
        )
    }
}
