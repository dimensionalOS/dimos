package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.vision.RangeSource
import com.dimensional.mini4pro.vision.TagWorld
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max

/**
 * **What the tag pipeline can tell the engine right now**, flattened to plain values at the
 * `Bridge` seam so [GuidedStickEngine.armTagDescent] and the descent tick can be driven from a
 * unit test with no detector, no camera and no aircraft — the same reason [RepositionCommand]
 * and [GcsStickFrame] exist.
 *
 * The frame of [fixNorthM]/[fixEastM] is the fix's own: **metres from the home point**, exactly
 * as `vision/TagFix` carries them, because `Bridge` builds the detector's `CameraPose` from
 * `nedMetres(home, here)`. The engine places the aircraft in the *same* frame with the *same*
 * function before subtracting, so the datum enters both sides and cancels — the Stage B altitude
 * argument, horizontally.
 *
 * [fixAgeMs] is computed **at the moment the supplier is read**, from the sighting's own
 * monotonic stamp, because a sighting handed to a control loop is 60–160 ms old before any
 * staleness even begins (`vision/TagSighting.Sighting.ageMillisAt` — measured 2026-07-28) and a
 * consumer that assumed currency would be wrong in the direction that hurts.
 */
data class TagDescentSense(
    /** True while `TagLatch` holds evidence for this flight. The latch, not a single frame. */
    val latched: Boolean,
    /** The latched tag's id, or null when nothing is latched. One id per flight, by design. */
    val latchedTagId: Int?,
    /** The id the newest world fix decoded, or null with no fix. Matched against the latch. */
    val fixTagId: Int?,
    /** Newest world fix, metres north of home, or null when the newest sighting could not be placed. */
    val fixNorthM: Double?,
    /** Newest world fix, metres east of home. */
    val fixEastM: Double?,
    /** How old the newest world fix is, milliseconds, at the moment this snapshot was taken. */
    val fixAgeMs: Long?,
    /**
     * The newest fix's **tag-derived range**, metres — `TagFix.tagRangeM()`: the trusted
     * solve's range on a metric fix, the size-implied range on a SIZE-scaled bearing fix,
     * null when the fix rests on the barometer alone. This is the number the fix's own
     * lateral was scaled by, so the descent's height law flying the same number
     * ([TagDescentGuidance.descentHeight]) is what keeps one property in one place —
     * landing07's consistency requirement, argued at `TagWorld`'s ladder KDoc.
     */
    val fixTagRangeM: Double? = null,
    /**
     * The newest fix's range provenance — `TagFix.rangeSource`, carried beside the number it
     * qualifies so the engine's `height_source` record lines and the commit's `hsrc` speak
     * the instrument's name rather than leaving a reader to infer it.
     */
    val fixRangeSource: RangeSource? = null,
)

/** Where the tag-tracked descent is in its life. See [TagDescentGuidance] for the transitions. */
enum class TagDescentPhase {
    /**
     * **Armed above the detection band, flying down into it** — the segment between an arm
     * taken at [TagDescentGuidance.ARM_CEILING_M]–[TagDescentGuidance.APPROACH_CEILING_M] and
     * the one-way handoff into [TRACKING] at [TagDescentGuidance.APPROACH_BAND_ENTRY_M].
     *
     * Why the segment exists — the measured friction it absorbs: the phone's takeoff climbs to
     * 10 m (`CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M`, Ivan's default) and the descent arm
     * refused everything above 7 m, so every flight was hover → `tag_descent_denied above 7m
     * tag band` (landing13 t=41.8, landing06 t=124.1) → a fully **blind manual descent** →
     * press again. The machine now flies that leg itself, centring as it goes, under the whole
     * abort ladder — strictly more guarded than the manual descent it replaces.
     *
     * What the sensor offers up here, decomposed (the design rests on this): the detector's
     * **bearing is the robust quantity** — the centroid of even an 11–14 px tag holds
     * ~pixel-level accuracy, and one pixel through the 1457 px focal is ~0.04°, centimetres of
     * lateral information at 10 m — while **range-from-size does not exist** (below
     * `TagWorld.SIZE_RANGE_MIN_PIXELS` = 20 px the fix carries no tag range, so the height
     * ladder answers BARO by construction — [TagDescentGuidance.descentHeight], verified not
     * duplicated) and **decodes are sparse** (the measured per-frame rate is 92.6 % at 7–8 m
     * falling to 1.3 % at 9–10 m — `2026-07-27-apriltag-c-vs-opencv.md` §1). The approach
     * needs only the first: lateral centring on the believed fix, height on the barometer,
     * whose fractional error matters least exactly where this phase flies.
     *
     * The phase is **entered only at arm time** (a constructor fact, like the autoland
     * option): a machine armed below the ceiling starts at [TRACKING] as it always has, and
     * nothing transitions into APPROACH — the handoff at the band is one-way, so the boundary
     * cannot oscillate however the barometer jitters.
     */
    APPROACH,

    /**
     * The tag is fresh: lateral velocity toward the fix, and descent **only while the lateral
     * error is inside the alignment cone** — outside it the aircraft holds altitude and keeps
     * centring, in this same phase, because "centring first" is the point of the stage.
     */
    TRACKING,

    /** Sighting older than [TagDescentGuidance.T_HOLD_MS]: descent stops, centring continues. */
    HOLDING,

    /**
     * Sighting older than [TagDescentGuidance.T_CLIMB_MS]: climb slowly back toward the
     * reacquisition band, bounded by [TagDescentGuidance.ARM_CEILING_M], still centring on the
     * last fix — the tag is a fixed object in the world and the last fix stays the best estimate
     * of where it is.
     */
    CLIMBING,

    /**
     * At [TagDescentGuidance.TARGET_HEIGHT_M] with the lateral error inside the innermost cone,
     * held for [TagDescentGuidance.TERMINAL_TICKS] consecutive ticks. The safe ending of plain
     * Stage B: hold over the tag, announced, until the operator takes over or an abort rung
     * fires. **Never exits back to a descent.** With the machine built `fullAutoland = false`
     * (the default) this is the last phase there is; with full autoland enabled the one exit is
     * [DJI_LANDING], and only on a fresh in-cone fix — see [TagDescent.step].
     */
    TERMINAL,

    /**
     * **Stage C — committed to DJI's own landing**, entered only with full autoland enabled and
     * only on a fresh in-cone fix: from [TERMINAL], or from [TRACKING] when the engine reports
     * the FC's descent floor ([TagDescent.step]'s `floorStalled`) at or below
     * [TagDescentGuidance.COMMIT_CEILING_M].
     *
     * Why a commit and not a held stick — **measured, landing04 (2026-07-28,
     * `datasets/landing04/20260728-174923.001.jsonl`), which refuted the first design the same
     * day it flew.** A sustained virtual-stick descent is floored by downward obstacle sensing
     * at ~1.4 m (scene-dependent) and held indefinitely: 12 s of commanded `vd = +0.4` moved
     * the aircraft nothing, and `KeyIsLandingConfirmationNeeded` never fired — the FC does not
     * route a virtual-stick descent into its landing flow at all. The identical gesture from
     * the **physical RC** (lv −660, ~1.5 s) produced `CONFIRM_LANDING`, the gimbal recenter
     * and touchdown 2.9 s later. Same command, different channel, opposite result: the
     * confirmation gesture is honoured only from the pilot's hand, and the only landing DJI
     * permits from software is its own — `KeyStartAutoLanding`.
     *
     * So this phase asks for exactly that, once (the engine issues the one-shot `land()` on the
     * entry edge), and then flies **lateral odometric steering and nothing else**: every step
     * commands exactly 0.0 vertical and 0.0 yaw — DJI owns the descent — while the LATERAL law
     * flies `target − position` on the [TagDescentGuidance.landingTarget] of the engagement's
     * believed fixes, **frozen at the commit edge** (the window ends at the newest PRE-commit
     * sample — the engine stops ingesting at the commit; landing07 measured why at the
     * engine's committed branch), a world-frame constant, under
     * [TagDescentGuidance.V_LAND_LATERAL_MAX_MS].
     *
     * **The lineage, cited because the design predates its own evidence.** The pre-pivot
     * `LAND_FRESH_MS` KDoc (5d95cd1) recorded *"odometric continuation"* as the anticipated
     * pivot verbatim — the lateral law on `fix − position` with the fix a world-frame
     * constant — and named frozen-at-zero as the conservative first-flight policy, chosen "to
     * measure DJI's blind-final behaviour before trusting centimetre-scale odometry at ankle
     * height", with two trigger conditions. Landing06
     * (`datasets/landing06/20260728-205913.001.jsonl`) measured both: the gimbal is
     * unholdable (DJI recentred the camera ~0.2 s after commit, twice, against the watchdog's
     * re-commands — the tag is unrecoverable there) and the blind drift matters (~12 cm/s of
     * handover momentum decaying at τ≈1.7 s ⇒ ~21 cm of the 31 cm measured miss). This phase
     * is that pivot, not new machinery.
     *
     * **The central unknown, stated:** whether DJI HONOURS nonzero lateral virtual-stick input
     * during its own landing is UNMEASURED. Landing06 proved only that VS *authority* survives
     * the whole AUTO_LANDING → CONFIRM_LANDING window (zeros streamed to touchdown, no
     * authority loss). If DJI ignores the input, ignored commands and zeros are behaviourally
     * identical — strictly no worse than the frozen design by construction — and the next
     * flight's record measures which world we are in (commanded nonzero in `stick_cmd` vs
     * achieved in GLOBAL_POSITION_INT).
     *
     * Tag loss here is EXPECTED, so the staleness ladder's climb and hand-back rungs do not
     * exist in this phase — steering deliberately continues on the frozen target however old
     * the fix (the tag does not move) — but it **dies to zeros the moment the aircraft
     * position feed goes stale** (the engine's gate: an error against a stale position is not
     * steering, it is a random walk; DJI keeps landing, we just stop helping). There is no
     * exit inside the law: the engagement's ends are the engine's — rule 1 (any manual stick,
     * which now also sends `KeyStopAutoLanding`), the abort ladder, the never-engaged
     * timeout, and touchdown observed as motors-off.
     */
    DJI_LANDING,
}

/**
 * M3 Stage D's arithmetic: the alignment cone, the staleness ladder, the lateral and vertical
 * laws and the terminal test. Pure functions, no state, no DJI, no Android — every number that
 * decides where the aircraft flies while descending onto a tag is in this object and under
 * `TagDescentGuidanceTest`, exactly as [OrbitGuidance] holds the circle's and
 * [RepositionGuidance] holds the goto's. Mutation-checked 2026-07-28, one breakage at a time
 * against the whole suite; the **measured** kill tables are in `TagDescentGuidanceTest` (this
 * file's arithmetic and machine) and `GuidedTagDescentTest` (the engine's half).
 *
 * ## The law, in the M3 house style
 *
 * ```
 * e         = fix − position                    (N/E metres, home frame both sides)
 * v_lateral = clampedSpeed(|e|, V_LATERAL_MAX)  (the M3 law, direction kept)
 * cone(h)   = max(CONE_FLOOR, CONE_TAN · h)
 * v_down    = |e| ≤ cone(h) ? clampedSpeed(h − TARGET, V_DESCENT_MAX) : 0
 * ```
 *
 * [RepositionGuidance.clampedSpeed] is **called, not restated** — same gain, same
 * `sqrt(2·a_max·e)` stopping envelope — under this manoeuvre's own smaller caps, for the orbit's
 * radial-axis reason verbatim: centring is *correction*, not travel.
 *
 * ## Why the descent flies on the world fix and not on the camera-frame offset
 *
 * A sighting reaching this loop is 60–160 ms old (measured, `TagSighting.ageMillisAt`), and at
 * descent speeds that is centimetres to decimetres of unaccounted motion if the offset is read
 * as "where the tag is relative to me *now*". `TagWorld.fix` converts each sighting into a
 * position **in the world**, using the pose at (approximately) the frame's own time — and the
 * tag does not move. Subtracting a *current* position fix from a *slightly old but world-frame*
 * tag position bounds the staleness error to the aircraft's own pose-feed lag rather than to the
 * whole detection latency. That, plus the nadir-point correction inside `fix()` (measured
 * 2026-07-28: the assumed image centre was 2.99° off true nadir — 15.6 cm of standing bias at
 * 3 m, larger than the project's best recorded arrival of 0.01 m), is what makes the centring
 * honest.
 *
 * ## What error the geometry still carries, stated because Stage E will land on it
 *
 * The nadir point is measured to ±2.3/±4.0 px (±0.09°/±0.16°) but it is the **sum** of the
 * principal point and the gimbal's nadir error, unseparated (chessboard pending,
 * `docs/measurements/2026-07-28-nadir-image-point.md`). For *this* stage that sum is exactly the
 * right number — centring drives the tag to the pixel that means "beneath me", whichever effect
 * put it there. What remains: the focal length is a two-flight fit (±1.2 %), the camera-to-body
 * rotation is assumed ([TagWorld.NADIR_IMAGE_UP_IS_NOSE], unflown), and `bearingAssumed` is
 * carried on every fix. At the heights this stage descends through (≤ 7 m) the focal-length term
 * is ≤ ~1 % of the lateral offset — centimetres — and the rotation assumption, if wrong, is a
 * *sign/quadrant* error that the first bench flight of this stage must look for before anything
 * descends over ground that matters.
 */
object TagDescentGuidance {

    /**
     * **7.0 m** — the top of the reliable-detection band over the takeoff datum. An arm below
     * it is today's arm, straight into [TagDescentPhase.TRACKING]; an arm above it (and under
     * [APPROACH_CEILING_M]) is accepted into [TagDescentPhase.APPROACH], which flies down to
     * [APPROACH_BAND_ENTRY_M] and hands off into the same ladder.
     *
     * The measured edge of the band where the shipped detector is a sensor rather than a coin
     * toss: 100 % per-frame detection to the 6.5–7.0 m band and a collapse of 92.3 % → 5.9 %
     * across the half-metre above it, reproduced across two flights
     * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1 — the shipped `apriltag`
     * library's curve, which superseded OpenCV's 3 m cliff in
     * `2026-07-27-tag-detection-rate.md`).
     *
     * Deliberately **not** `TagArming.ACQUIRE_CEILING_M` (8 m): that constant decides where it
     * is worth *spending CPU looking*, and a metre of margin above the cliff is cheap there.
     * Arming a descent needs the sensor to be *reliable*, so this one is the measured band
     * itself, with no margin in the unreliable direction — and `TagArming`'s own KDoc says the
     * grant is not its to make: *"whether a descent may arm on what it sees … belongs to whoever
     * writes that law"*. This is that law.
     */
    const val ARM_CEILING_M = 7.0

    /**
     * **12.0 m — no arm above this height, even into an approach.** Between [ARM_CEILING_M]
     * and here an arm with a latched tag and a fresh believed fix is accepted into
     * [TagDescentPhase.APPROACH]; above here it is refused by name.
     *
     * The bound is the **measured decode reach plus the measured baro error scale**, and it is
     * a backstop, not the working sensor gate — the arm's latch and ARM_FRESH conjuncts are
     * what prove the tag is decodable from the arm height *right now*. What this ceiling
     * refuses is the reading that contradicts every decode measurement this project has:
     *
     *  - **Decode reach, measured three ways.** The profiling descent holds 92.6 % per-frame
     *    at 7–8 m, 2.7 % at 8–9 m, 1.3 % at 9–10 m and 0 % in all 60 frames of the 10–11 m
     *    band (`2026-07-27-apriltag-c-vs-opencv.md` §1); landing06's 8.0 m hover delivered a
     *    solid decode stream (110 fixes at ~8.0 m, median 13.9 px, the t=124.1 denial's own
     *    evidence) and its record's highest believed fix stands at 8.4 m; landing13's decodes
     *    reach 8.1 m and its 9.5–9.7 m hover was decode-silent for 7.1 s. So genuine fixes
     *    exist — sporadically — to the top of the 9–10 m band, and nothing has EVER decoded
     *    above it: at 10 m the 75 mm tag is ~11 px through the 1457 px focal — under the
     *    ~14 px cliff where the *rate* collapses, yet exactly where the 9–10 m band's
     *    measured singles (median 11.3 px) say occasional decodes still land.
     *  - **The margin above the reach is the barometer's own measured lie scale**: ~1.2 m of
     *    within-session drift (landing07, `2026-07-27-altitude-datum-wander.md` family), so a
     *    true 10 m hover may *read* ~11 m and must still arm. 12.0 = the 10 m reach edge +
     *    ~2× that wander.
     *  - **Ivan's use case sits inside with room**: `PHONE_TAKEOFF_HEIGHT_M` = 10 m, the
     *    hover this feature exists for (landing13 t=41.8's denial is the friction on record).
     *
     * Above 12 m a "fresh fix" means an instrument is lying — a baro reading far under the
     * true height, or a decode of a tag no measurement says is decodable — and arming a
     * descent on a lie is exactly what a ceiling is for. Deliberately **not**
     * `GuidedEnvelope`'s Q1 ceiling (100 m — an airspace bound, not a sensor statement) and
     * not the 20 px size-range floor (~5.5 m — a *range* bound; the approach flies bearings,
     * [TagDescentPhase.APPROACH]'s decomposition).
     */
    const val APPROACH_CEILING_M = 12.0

    /**
     * The band-entry hysteresis: an approach hands off into the [TagDescentPhase.TRACKING]
     * ladder at [ARM_CEILING_M] **minus this margin**, metres — 6.5 m, not 7.0.
     *
     * Two measurements size it:
     *
     *  - **It is the detection measurement's own bin width.** The collapse above the band was
     *    located at half-metre resolution — the last 100 % band is 6.5–7.0 m and the two
     *    bands above fall 92.9 % → 92.3 % → 5.9 % (`2026-07-27-apriltag-c-vs-opencv.md` §1) —
     *    so entering at 6.5 m puts the whole measured-100 % half metre between the handoff
     *    and the cliff: the TRACKING ladder's freshness semantics (T_HOLD as "tripped by the
     *    first genuinely missing pair of detections") become true at the moment they take
     *    over, not half a metre later.
     *  - **It is 5 baro quanta.** The altitude feed is 0.1 m-quantised and change-driven; a
     *    margin of one quantum would let the feed's own step noise sit on the boundary. Five
     *    cannot chatter — and the handoff is one-way anyway (the phase latches), so the
     *    margin buys honesty of the ladder at entry, not oscillation protection alone.
     */
    const val APPROACH_ENTRY_MARGIN_M = 0.5

    /** Where the approach ends and today's machine begins: [ARM_CEILING_M] − the margin. */
    const val APPROACH_BAND_ENTRY_M = ARM_CEILING_M - APPROACH_ENTRY_MARGIN_M

    /**
     * How fresh the newest world fix must be for arming, milliseconds — and the first rung of
     * the staleness ladder ([T_HOLD_MS] aliases it, deliberately: "fresh enough to arm on" and
     * "fresh enough to keep descending on" are the same claim about the same sighting).
     *
     * From the measured cadence (`2026-07-28-addframelistener-on-the-aircraft.md`, and the
     * ageMillisAt KDoc): frames arrive every 41.5 ms (p50; 47 p90), the detector takes 35–50 ms
     * at the shipped two threads, and the 10 Hz cap adds up to 125 ms of waiting for the next
     * admitted frame — so a visible tag delivers a fix every ~100–225 ms. 400 ms is roughly two
     * whole cap periods beyond the worst normal path: it cannot be tripped by the pipeline's own
     * jitter, and it is tripped by the first genuinely missing pair of detections.
     */
    const val ARM_FRESH_MS = 400L

    /**
     * Older than this: **stop descending, keep centring** ([TagDescentPhase.HOLDING]).
     *
     * Same number and same argument as [ARM_FRESH_MS]. What it bounds is blind *descent*: at
     * [V_DESCENT_MAX_MS] the aircraft moves at most 16 cm toward the ground between the last
     * believed fix and the hold — under the cone floor, and under the tag's own board-height
     * uncertainty.
     */
    const val T_HOLD_MS = ARM_FRESH_MS

    /**
     * Older than this: **climb slowly back toward the reacquisition band**
     * ([TagDescentPhase.CLIMBING]).
     *
     * **Two seconds — `TagLatch.DEFAULT_WINDOW_NANOS`' twin, on the same measurement.** At the
     * 10 Hz detection cap two seconds is up to twenty chances, and the measured per-frame rate
     * everywhere inside the band is ≥ 92 % — so two seconds of nothing is not a gap in a healthy
     * stream (the latch's own reasoning, run backwards), it is a tag that has genuinely left the
     * frame or the light. Climbing widens the footprint the camera covers, which is the only
     * thing this bridge can do about that.
     */
    const val T_CLIMB_MS = 2_000L

    /**
     * Older than this: **give the aircraft back** — hover, hand back, announced.
     *
     * **Ten seconds — [RepositionGuidance.POSITION_LOST_MS]'s twin, on the same argument**: a
     * bounded window of benign commands while blind, then release, because this bridge without
     * its sensor has no business holding stick authority and DJI's own failsafes have eyes of
     * their own. The eight seconds between [T_CLIMB_MS] and here recover 2.4 m of height at
     * [V_REACQUIRE_MS] — from mid-band back toward the top of the detection band — so the climb
     * gets a real chance before the handback.
     */
    const val T_ABORT_MS = 10_000L

    /**
     * The lateral centring cap, m/s — [OrbitGuidance.V_RADIAL_MAX_MS]'s twin, for its reason
     * verbatim: centring is *correction*, not travel, and a fast lateral component over a
     * landing pad is an aircraft overshooting the thing it is trying to sit on.
     */
    const val V_LATERAL_MAX_MS = 1.0

    /**
     * The **committed landing's** lateral cap, m/s — a fifth of [V_LATERAL_MAX_MS], sized from
     * landing06's 32 cm miss budget (`datasets/landing06/20260728-205913.001.jsonl`) in three
     * directions:
     *
     *  - **It never binds inside the design envelope.** The error the blind-final steering must
     *    remove is bounded by the commit cone (0.21 m at the measured 1.4 m floor, 0.375 m at
     *    the 2.5 m ceiling) plus the measured handover-momentum drift (~12 cm/s decaying at
     *    τ≈1.7 s ⇒ ~21 cm); at the worst credible ~0.4 m of error the clamped-speed law asks
     *    `KP_PER_S · 0.4 = 0.2 m/s` — the cap sits exactly at the largest command the measured
     *    budget can produce, so within it the law is the plain [RepositionGuidance.clampedSpeed]
     *    braking curve, undistorted.
     *  - **It bounds the wrong-world case.** Odometric steering on a frozen fix at ankle height
     *    is only as good as the position feed; if the fix or the GPS is wrong, the worst runaway
     *    is `0.2 m/s · 3.2 s = 0.64 m` over the measured blind final — the same scale as the
     *    32 cm miss it exists to correct, not a new hazard class. The full tracking cap would
     *    allow 3.2 m.
     *  - **It out-pulls the disturbance it corrects**: 0.2 m/s against the measured 12 cm/s of
     *    handover momentum.
     */
    const val V_LAND_LATERAL_MAX_MS = 0.2

    /**
     * **The commit gate's lateral-speed bound, m/s — no `KeyStartAutoLanding` while moving
     * sideways faster than this.** Landing06 crossed the commit carrying ~12 cm/s of lateral
     * velocity (GLOBAL_POSITION_INT slope 147.6→148.4 s, decaying at τ≈1.7 s under DJI's own
     * braking), which alone contributed ~21 cm of the measured 31 cm touchdown miss. Gating the
     * commit at ≤ 0.05 m/s cuts that term to ≤ ~8 cm.
     *
     * **Why 0.05 and not 0.1 — the quantisation argument.** The velocity feed is 0.1 m/s
     * quantised, so the only *reading* that can pass a 0.05 m/s bound is 0.0, and a reading of
     * 0.0 means a true speed ≤ 0.05 m/s (plus sensor error). A bound at or above the quantum
     * would pass a reading of 0.1 whose truth may be 0.15 m/s — a gate looser than its own
     * name, letting through more momentum than tonight's whole budget. Deliberately below the
     * quantum, so quantisation can never let a fast commit through; the price is that the gate
     * is conservative by up to one quantum, which only delays a commit by the τ≈1.7 s the
     * braking takes — the safe direction. A tick failing this conjunct keeps holding or
     * descending exactly as a tick failing the cone does.
     *
     * Defence-in-depth, stated honestly: the committed phase's own odometric steering
     * ([V_LAND_LATERAL_MAX_MS]) makes handover momentum partially self-correcting, so this gate
     * matters most in the world where DJI *ignores* our lateral input during its landing —
     * which is unmeasured, and exactly the world the gate protects.
     */
    const val LAND_COMMIT_SPEED_MS = 0.05

    /**
     * **The tag/baro divergence record's threshold — a measurement bound, never a gate.** When
     * the newest believed fix's tag-derived range and the barometric height disagree beyond
     * this factor, the engine writes one `range_baro_divergence` line per episode, so a
     * post-flight read shows the barometer's lies against the instrument the descent actually
     * flew. Behaviour already chose the tag ([descentHeight]'s ladder) — recording is all
     * that is left to do, and refusing to land because the measured liar disagrees with the
     * truth-teller would be exactly backwards (Ivan, 2026-07-29: *"we should land if we see
     * the tag, don't be fragile insisting on baro"*).
     *
     * The factor is picked from landing07's two measured brackets
     * (`datasets/landing07/20260729-095413.001.jsonl`, one session, two autolands):
     *
     *  - **Healthy (landing A, commit t=74.23):** tag ranges 0.56–0.60 m against baro 0.8 m —
     *    ratio **1.36**, the known composite of the near-ground baro over-read (~0.2 m) and
     *    the 0.1 m quantum. The healthy pair never agrees much better than ~1.3 at terminal
     *    heights, so a tighter threshold would flag every honest landing.
     *  - **Failure (landing B, commit t=134.93):** baro "0.7 m" against a size range of
     *    1.93–2.13 m at 51–57 px — ratio **2.76**, the ~1.2 m drift the record must call out.
     *
     * 1.8 sits 1.3× above the healthy ratio and 1.5× under the failure. Symmetric
     * (`max(r, 1/r)`): an instrument lying in either direction is worth a line.
     */
    const val RANGE_DIVERGENCE_FACTOR = 1.8

    /**
     * **The divergence check** — the named detail with both numbers when [tagRangeM] and
     * [baroHeightM] disagree beyond [RANGE_DIVERGENCE_FACTOR], null when they agree *or when
     * either is unknown*: an absent instrument is an absence, not a divergence, and warning
     * about it every tick the tag flies baro-free would bury the signal this line exists to
     * carry. For the record only — nothing consumes this as a gate (the constant's KDoc).
     */
    fun rangeDivergence(tagRangeM: Double?, baroHeightM: Double?): String? {
        if (tagRangeM == null || !tagRangeM.isFinite() || tagRangeM <= 0.0 ||
            baroHeightM == null || !baroHeightM.isFinite() || baroHeightM <= 0.0
        ) {
            return null
        }
        val ratio = max(tagRangeM / baroHeightM, baroHeightM / tagRangeM)
        if (ratio <= RANGE_DIVERGENCE_FACTOR) return null
        return "tag=%.2f baro=%.2f ratio=%.1f bound=%.1f".format(
            tagRangeM, baroHeightM, ratio, RANGE_DIVERGENCE_FACTOR,
        )
    }

    /** [descentHeight]'s answer: the height the descent laws fly on, and which instrument said it. */
    data class DescentHeight(val heightM: Double, val source: RangeSource)

    /**
     * **The descent's height, resolved by the range ladder: a fresh believed fix's
     * tag-derived range — solved or size-implied, the fix already chose which
     * (`TagFix.rangeSource`) — outranks the barometer; the baro is the fallback** (no usable
     * tag range, stale fix), and null means no instrument can vouch — unknown, never zero.
     *
     * The flight-gated decision `TagFix.rangeM` shipped waiting for, now flight-justified —
     * **landing07's landing B is the measurement** (`datasets/landing07/20260729-095413.001.jsonl`,
     * 2026-07-29): the baro drifted a full metre within a ~40 s flight and every law consuming
     * it (cone, terminal test, commit ceiling) was faithful to the poisoned height — a 46 cm
     * miss through a 10.6 s blind final — while the tag's own size-implied range, carried on
     * every sighting, was right (1.93–2.13 m at 51–57 px) and flown on by nothing. In the same
     * session landing A's solve read 0.59 m against baro 0.8 m at the commit — the known
     * ~0.2 m near-ground baro bias, now simply not flown on either.
     *
     * **The consistency requirement this function keeps** (what actually dissolves landing B):
     * [tagRangeM] must be the newest believed fix's own `TagFix.tagRangeM()` — the number that
     * scaled that fix's lateral offset — so the lateral error and the height the law flies
     * rest on the SAME instrument. Landing B's cascade was precisely their divergence: the
     * bearing lateral rode the lying baro, under-reported the error 2.7×, let the hold wander
     * and the commit fire falsely in-cone. With one owner for the range, a lying baro changes
     * nothing the descent believes: the error reads true, the law sees ~1.9 m, keeps
     * descending toward the target, the tag grows past the solve gate, and the commit happens
     * at a true low height — the failure self-heals instead of being refused.
     *
     * **The identity this rests on: at nadir, a tag-derived range IS the height above the tag
     * plane.** Its assumptions, named: the camera within [NADIR_TOLERANCE_DEG] of plumb —
     * already gated at fix production (`TagWorld.fix` refuses beyond it) and by the descent's
     * own camera gate, and the cosine error at the 12° tolerance edge is ≤ 2.2 %, under the
     * baro's own quantum at these heights — and the tag on the takeoff datum plane
     * (`TagWorld`'s standing assumption; a tag on a raised board makes this height "above the
     * tag", which is the height a descent onto the tag actually wants).
     *
     * The freshness bound is [LAND_FRESH_MS] — the same "fresh enough to believe" every other
     * claim uses, made once. A stale tag range falls back to baro rather than being believed
     * on: the tag does not move but the aircraft does, and a range measured before the last
     * [LAND_FRESH_MS] of descent is a height from another place. The high approach lives on
     * baro too — below ~20 px ([TagWorld.SIZE_RANGE_MIN_PIXELS], ~5.5 m of range) the tag
     * offers no usable range — which is fine: baro errors matter least up high, and the arm
     * ceiling still consumes the baro on purpose.
     *
     * **The seam, thought through** (this function can *step* the height at every rung
     * transition — BARO→SIZE where the tag passes 20 px on the way down, SIZE→SOLVE at the
     * 60 px solve gate, and back on any loss):
     *
     *  - Cone, descent rate, terminal test, commit ceiling all follow the step — the honest
     *    direction: a step from baro 0.7 to tag 1.9 (landing B's shape) re-opens the descent
     *    the false height had closed, at the wider cone the true height earns.
     *  - The **floor detector deliberately does not consume this height** — it stays on the
     *    baro alone (`floorStalledLocked`). It measures *progress* (successive deltas), and
     *    baro deltas stay honest under the measured drift (~1.2 m over ~40 s ≈ 0.03 m/s —
     *    0.06 m across the 2 s stall window, under the one-quantum threshold), while this
     *    ladder steps at every rung transition, all of which live inside the descent: a step
     *    at any seam would fake either a 0.1 m "drop" (resetting the window and deferring the
     *    commit landing04 promoted) or phantom stalls. One instrument per detector; the
     *    argument lives at the detector.
     *  - A divergence between the flown tag range and the baro is **recorded, never refused**
     *    ([rangeDivergence], the engine's `range_baro_divergence` line) — and every rung
     *    switch writes its `height_source` line with both instruments' numbers, so the record
     *    always shows which instrument the descent flew on and when it changed its mind.
     */
    fun descentHeight(
        baroHeightM: Double?,
        tagRangeM: Double?,
        tagRangeSource: RangeSource?,
        fixAgeMs: Long,
    ): DescentHeight? {
        if (tagRangeM != null && tagRangeM.isFinite() && tagRangeM > 0.0 &&
            tagRangeSource != null && tagRangeSource != RangeSource.BARO &&
            fixAgeMs <= LAND_FRESH_MS
        ) {
            return DescentHeight(tagRangeM, tagRangeSource)
        }
        val baro = baroHeightM ?: return null
        if (!baro.isFinite()) return null
        return DescentHeight(baro, RangeSource.BARO)
    }

    /**
     * The window over which the blind-final steering target is a robust (median) average of
     * believed fixes, milliseconds — **short on purpose, from landing06's own error anatomy.**
     *
     * 75 world-frame fix reconstructions over the last 8 s before landing06's commit measured
     * frame-to-frame jitter of σ≈3.5 cm N (3–5 cm over short windows) — but a 12 cm E spread
     * over the full 8 s that is **not noise**: it is a height-correlated systematic walk (the
     * fix migrated ~9 cm as the aircraft descended 2.5 → 1.0 m; the ~3° unseparated intrinsics
     * tilt is ~5 cm of bias per metre of height, plus the bearing-path baro scale shrinking
     * with range). So the window must be short enough that every sample sits at essentially
     * one height: 500 ms is ~5 fixes at the 10 Hz cap — enough to average the 3–5 cm random
     * part down by ~2×, and short enough (≤ 0.2 m of height change at [V_DESCENT_MAX_MS])
     * that the walk cannot enter. A long window would average in the more-biased
     * high-altitude fixes and move the target the *wrong* way.
     *
     * The expected gain is honest: ~2–3 cm off the random part of the target. The systematic
     * walk is the chessboard calibration's job (`2026-07-28-nadir-image-point.md`), not this
     * window's.
     */
    const val LAND_TARGET_WINDOW_MS = 500L

    /** One believed fix with its own timestamp — [landingTarget]'s input, the engine's ingest out. */
    data class FixSample(val atMs: Long, val northM: Double, val eastM: Double)

    /**
     * The descent cap, m/s — deliberately about a quarter of [GuidedEnvelope.VERTICAL_MAX_MS].
     *
     * Chosen against the sensor's latency, not the airframe's ability: a fix is 60–160 ms old
     * on arrival and believed for up to [T_HOLD_MS] more, so the worst blind descent inside the
     * ladder is `0.4 · 0.4 s = 16 cm` — below the cone floor, and comparable to the 0.1 m
     * quantisation of the altitude it is descending against. Both measured descents that
     * produced the detection-rate curve were flown at comparable rates.
     */
    const val V_DESCENT_MAX_MS = 0.4

    /**
     * The reacquisition climb, m/s — slower than the descent, on purpose. Climbing away from
     * the ground is the safe direction, so speed buys nothing but motion blur, and the one
     * thing the detection-rate document says about climbs is that nobody has measured one
     * (`2026-07-27-tag-detection-rate.md` §6: "the climb is not a comparison").
     */
    const val V_REACQUIRE_MS = 0.3

    /**
     * The alignment cone's slope: descend only while the lateral error is inside
     * `CONE_TAN · height` — a half-angle of **8.5°**.
     *
     * The geometry, from the two measurements it must sit between:
     *
     *  - **It must dominate the pointing residual.** The nadir point is known to the sum of
     *    principal point and gimbal error, 2.99° unseparated
     *    (`2026-07-28-nadir-image-point.md`); a cone comparable to that residual would gate
     *    descent on calibration noise. 8.5° is 2.8× the residual: a fix at the cone edge is
     *    genuinely off-centre.
     *  - **It must keep the tag well inside the frame while descending.** The half-width of the
     *    field of view is `atan(960/1457) = 33.4°`; at 8.5° the tag sits within the central
     *    quarter of the image, so the descent itself never pushes the tag toward the edge — and
     *    the edge is where detection dies (the measured 0–1 m failure is *framing*, 41 % on both
     *    flights, unfixable by any detector).
     *
     * The cone **tightens with height by construction**: 1.05 m of allowed error at 7 m,
     * 0.30 m at 2 m, the floor below ~1.3 m. Centring accuracy is the point of the stage —
     * later stages land on this tag — so the aircraft may not buy altitude with alignment.
     */
    const val CONE_TAN = 0.15

    /**
     * The cone's floor, metres. `CONE_TAN · h` at low heights falls under what the sensor pair
     * can resolve: the altitude is 0.1 m-quantised, and the nadir-point bias table reads
     * 5.2 cm at 1 m. A floor of 0.20 m — 2× the quantisation step, ~4× the low-height bias —
     * is the tightest gate that will not chatter descent on and off on measurement noise.
     */
    const val CONE_FLOOR_M = 0.20

    /**
     * **Where Stage B ends: hold ~0.6 m above the tag.** The landing below it is Stage C
     * ([TagDescentPhase.DJI_LANDING]) — behind its own explicit operator toggle, committed on
     * a fresh in-cone fix from this hold or from the FC's measured descent floor (landing04:
     * ~1.4 m, so in practice the floor commit fires first and this hold is the fallback for
     * scenes whose floor sits lower). Without that toggle this height is still where
     * everything ends.
     *
     * Why 0.6 m and not lower: the measured detection collapse in the last metre is *framing* —
     * the tag runs off the frame edge — and it reached 41 % over the whole 0–1 m band. A
     * **centred** tag at 0.6 m is ~180 px across (f·S/h = 1457·0.075/0.6) and wholly inside the
     * central frame, which is exactly the situation this stage's cone maintains; below ~0.5 m
     * even a centred 75 mm tag starts to leave the frame. 0.6 m holds the last height at which
     * the sensor that flew the descent can still vouch for where the aircraft is.
     *
     * **The height source, and its honesty.** Height is `relativeAltitude` above the takeoff
     * datum, delivered **0.1 m-quantised and change-driven** (measured,
     * `2026-07-26-attitude-and-staleness` family), and the tag is *assumed to sit on the datum
     * plane* — `TagWorld.fix` says so at the field. A tag standing on a raised board is higher
     * than the datum by the board (measured once, by accident, at +0.33 m), and this stage's
     * "0.6 m above the tag" is then really "0.6 m above the takeoff plane" — *closer* to the tag
     * by the board height. That error is in the direction that hurts, it cannot be measured from
     * the aircraft, and it is the first number Stage C's measurement flight must produce
     * (a tape measure beats everything this project has).
     */
    const val TARGET_HEIGHT_M = 0.6

    /**
     * The terminal height band, metres: within this of [TARGET_HEIGHT_M] counts as "at the
     * target". 0.15 m is 1.5× the altitude feed's 0.1 m quantisation step — the tightest band
     * the sensor can honestly resolve; tighter would be asserting a digit the feed does not
     * carry.
     */
    const val HEIGHT_ACCEPT_M = 0.15

    /**
     * Both terminal conjuncts must hold this many **consecutive** ticks — 0.5 s at the 10 Hz
     * tick. [RepositionGuidance.ARRIVE_TICKS]'s twin, for its reason: a fly-through (a descent
     * blowing past the target height at speed, or a gust carrying the aircraft across the cone)
     * must not declare the stage complete.
     */
    const val TERMINAL_TICKS = 5

    /**
     * How far the **commanded** camera pitch may sit from nadir, degrees — an alias for
     * [TagWorld.NADIR_TOLERANCE_DEG], not a second number. Beyond it `TagWorld.fix` refuses to
     * produce fixes at all, so a looser gate here would arm a descent whose sensor is about to
     * go silent, and a tighter one would refuse geometry the fix pipeline accepts. One
     * constant, one meaning. (Commanded, never read back: `KeyGimbalAttitude` is change-driven
     * and goes silent exactly during a nadir hold — the trap this project has hit seven times.)
     */
    const val NADIR_TOLERANCE_DEG = TagWorld.NADIR_TOLERANCE_DEG

    /** Straight down, in the commanded-pitch convention every gimbal path here uses. */
    const val NADIR_PITCH_DEG = -90.0

    /**
     * **The commit ceiling: no `KeyStartAutoLanding` above this height, metres.**
     *
     * The floor commit ([TagDescentPhase.DJI_LANDING] via `floorStalled`) exists because the
     * FC's downward obstacle sensing floors a virtual-stick descent — measured at **1.4 m** on
     * landing04, and **scene-dependent by DJI's own design**, so the floor height is a reading,
     * never a constant. This ceiling bounds what reading is believable:
     *
     *  - **2.5 m is comfortably above the one measured floor** (1.8×), so scene variation has
     *    room without the commit being refused over the pad it was armed on.
     *  - **Above it, a stalled descent stops meaning "ground proximity".** The obstacle sensor
     *    that pins a descent at 2.5+ m is seeing something that is *not* the landing surface —
     *    a person, a table, a wire — and `KeyStartAutoLanding` onto an obstacle is precisely
     *    the command this gate exists to refuse. DJI's own landing then descends ~2.5 m blind
     *    (~8 s at its measured 0.2–0.4 m/s), the longest blind final this feature could sign
     *    up for; higher would be longer still, over a surface the sensor just objected to.
     *  - The tag geometry stays healthy at 2.5 m: 100 % per-frame detection band, cone radius
     *    0.375 m, ~43 px tag — the fix vouching for "over the pad" at the commit is real.
     *
     * A floor met above the ceiling simply keeps holding (centred, announced by the stall
     * line), leaving the decision with the operator — the safe direction.
     */
    const val COMMIT_CEILING_M = 2.5

    /**
     * How long a committed [TagDescentPhase.DJI_LANDING] may wait for the FC to actually enter
     * a landing mode before the engine gives the engagement back to a hold, milliseconds.
     *
     * Whether `KeyStartAutoLanding` is honoured at all while MSDK virtual stick holds authority
     * is Stage C's next open unknown — DJI accepts commands it does not enact (measured,
     * `2026-07-26-m2-first-command.md`), so "asked" proves nothing and an unbounded wait would
     * hover the aircraft at the FC floor forever with a screen claiming DJI is landing. 15 s is
     * 3× `MsdkFlightActions.LANDING_MODE_GRACE_MS` (itself 2.8× the one measured accept→mode
     * delay of 1.8 s), i.e. generous against every measured latency on this path. The bound
     * applies **only while the flight mode is not a landing mode**: once the FC is measurably
     * landing (`CONFIRM_LANDING` et al.), the landing takes as long as it takes and touchdown
     * (motors-off) is the ending.
     */
    const val DJI_LAND_TIMEOUT_MS = 15_000L

    /**
     * The freshness bound the committed landing's remaining *claims* stand on, milliseconds —
     * an alias of [T_HOLD_MS], deliberately: "fresh enough to believe" is the same claim in
     * every phase, made once.
     *
     * [TagDescentPhase.DJI_LANDING]'s steering deliberately does NOT gate on this bound — the
     * odometric pivot's whole point (see the phase's KDoc): the target is a world-frame
     * constant and only the aircraft's own position feed can invalidate the error. What this
     * still gates is honesty about the sensor while DJI lands: the screen's aligned/blind flag
     * (`GuidedDescent.blind`), and the engine's `lastFreshFixInCone` — the cone half of the
     * guarded auto-confirm — which is updated only from fixes inside this bound, so the confirm
     * gate's "was over the tag" is never a re-derivation from stale numbers. Going blind is
     * the landing's expected shape (DJI recenters the camera ~3 s before touchdown, measured
     * on every landing including landing04's: −79° → −33° in 120 ms).
     */
    const val LAND_FRESH_MS = T_HOLD_MS

    /**
     * How far the **reported** gimbal pitch may sit from [NADIR_PITCH_DEG] during a LANDING
     * before the watchdog re-commands nadir, degrees.
     *
     * 5° sits between the two numbers that bound it: far above the ±0.2° jitter the held
     * samples show (`landing02` gimbal lines read −90.0/−89.8 for minutes at a time), so a
     * healthy nadir hold can never trip it — and far below the 30° the recenter detector uses
     * to *declare* a DJI recenter, so the watchdog fires early in the ~250–300 °/s slew
     * (`landingdata.md` §2: 5° is crossed within ~20 ms of the slew starting) rather than
     * after the camera is already level.
     */
    const val GIMBAL_WATCHDOG_DEG = 5.0

    /**
     * The watchdog's rate limit: at most one nadir re-command per second, at most
     * [GIMBAL_WATCHDOG_MAX] per landing. A watchdog with no bound is a command loop fighting
     * the firmware over a camera while the aircraft is half a metre off the ground; whether
     * the FC honours a nadir command in `CONFIRM_LANDING` at all is exactly the open unknown
     * this contingency measures (`landingdata.md` §4 Option 2), and ten polite attempts answer
     * it as well as a thousand. Past the bound the landing simply continues blind, logged.
     */
    const val GIMBAL_WATCHDOG_MIN_MS = 1_000L

    /** See [GIMBAL_WATCHDOG_MIN_MS]. */
    const val GIMBAL_WATCHDOG_MAX = 10

    /**
     * **The floor window — the commit trigger's temporal conjunct, and load-bearing since
     * landing04.** An autoland-armed descent whose altitude has not fallen one quantum
     * ([LAND_STALL_DROP_M]) in this long, under a sustained down command, has met the FC's
     * downward-obstacle floor: the measured shape is 12 s of commanded `vd = +0.4` at a pinned
     * 1.4 m (landing04, t≈81–94 s), against a nominal descent that drops a quantum every
     * 250 ms. 2 s is 8× the healthy inter-quantum time — jitter cannot trip it — and ~6× less
     * than the measured hold, so the commit fires early in the stall rather than after the
     * operator has started wondering.
     *
     * The window tolerates the cone's own gating: at the floor the lateral error wobbles across
     * the cone edge and the down command alternates 0/0.4 (measured, landing04's last
     * commands), so ticks that command no descent neither advance nor reset the window — only
     * altitude *progress* resets it. Each detected stall still writes its `landing_stall` line
     * once (the measurement half survives the promotion to trigger).
     */
    const val LAND_STALL_MS = 2_000L

    /** The altitude drop that counts as progress for the floor detector — one 0.1 m quantum. */
    const val LAND_STALL_DROP_M = 0.1

    /** The staleness ladder, as a function so the thresholds live in exactly one place. */
    enum class Rung { FRESH, HOLD, CLIMB, GONE }

    /**
     * Which rung of the ladder a fix of [ageMs] sits on. Boundaries are inclusive on the fresh
     * side: an age of exactly [T_HOLD_MS] is still fresh, exactly [T_CLIMB_MS] still holds, and
     * exactly [T_ABORT_MS] still climbs — the gate fires on *exceeding* the bound, so a clock
     * that lands on the boundary errs toward the milder rung it was already on.
     */
    fun rung(ageMs: Long): Rung = when {
        ageMs <= T_HOLD_MS -> Rung.FRESH
        ageMs <= T_CLIMB_MS -> Rung.HOLD
        ageMs <= T_ABORT_MS -> Rung.CLIMB
        else -> Rung.GONE
    }

    /** The alignment cone's radius at [heightM] above the tag plane. */
    fun coneRadiusM(heightM: Double): Double {
        if (!heightM.isFinite() || heightM <= 0.0) return CONE_FLOOR_M
        return max(CONE_FLOOR_M, CONE_TAN * heightM)
    }

    /**
     * The lateral centring law: the N/E error in, an envelope-bounded velocity toward the tag
     * out — [RepositionGuidance.clampedSpeed] on the error vector's magnitude with the direction
     * kept, under [V_LATERAL_MAX_MS]. Garbage in, hover out, exactly as the M3 law behaves.
     */
    fun lateral(errorNorthM: Double, errorEastM: Double): Pair<Double, Double> =
        cappedLateral(errorNorthM, errorEastM, V_LATERAL_MAX_MS)

    /**
     * The committed landing's lateral law — **the same arithmetic as [lateral]**, the same
     * gains and braking curve, under [V_LAND_LATERAL_MAX_MS]. One law, two caps: the tracking
     * loop corrects toward a fix it can still see; this corrects toward a frozen target at
     * ankle height, and the smaller cap carries that difference (see the cap's own KDoc for
     * the budget arithmetic).
     */
    fun landingLateral(errorNorthM: Double, errorEastM: Double): Pair<Double, Double> =
        cappedLateral(errorNorthM, errorEastM, V_LAND_LATERAL_MAX_MS)

    private fun cappedLateral(
        errorNorthM: Double,
        errorEastM: Double,
        capMs: Double,
    ): Pair<Double, Double> {
        if (!errorNorthM.isFinite() || !errorEastM.isFinite()) return Pair(0.0, 0.0)
        val distance = hypot(errorNorthM, errorEastM)
        if (distance <= 0.0) return Pair(0.0, 0.0)
        val speed = RepositionGuidance.clampedSpeed(distance, capMs)
        return Pair(speed * errorNorthM / distance, speed * errorEastM / distance)
    }

    /**
     * **The blind-final steering target**: a per-axis median of the believed fixes within
     * [LAND_TARGET_WINDOW_MS] of the *newest* sample — anchored to the samples' own clock, not
     * the caller's, so the target stays put once fixes stop arriving (the camera is gone; the
     * window must not empty itself into a dead stick while the position feed still lives).
     *
     * Median rather than mean because it costs nothing and bounds any straggler — the id gate
     * upstream already kills false decodes (2 in 1978 frames, measured), so this is a bound,
     * not a filter doing load-bearing work. **Fewer than two samples in the window: the single
     * newest fix, unaveraged** — which is exactly the pre-averaging design, and fine. Empty:
     * null, and the caller must treat "no believed samples" as "no target" (a dead stick,
     * never a guess).
     *
     * Deliberately never used by the live tracking loop: that loop is latency-limited and
     * steers on the newest fix alone — filtering there lowers the stable gain for zero benefit,
     * because a visible tag refreshes the fix faster than any averaging horizon.
     */
    fun landingTarget(samples: List<FixSample>): Pair<Double, Double>? {
        if (samples.isEmpty()) return null
        val anchor = samples.maxOf { it.atMs }
        val window = samples.filter { it.atMs >= anchor - LAND_TARGET_WINDOW_MS }
        if (window.size < 2) {
            val newest = samples.maxBy { it.atMs }
            return Pair(newest.northM, newest.eastM)
        }
        return Pair(median(window.map { it.northM }), median(window.map { it.eastM }))
    }

    private fun median(values: List<Double>): Double {
        val sorted = values.sorted()
        val mid = sorted.size / 2
        return if (sorted.size % 2 == 1) sorted[mid] else (sorted[mid - 1] + sorted[mid]) / 2.0
    }

    /**
     * The descent rate toward [TARGET_HEIGHT_M], m/s down-positive, **never negative** — this
     * function can only descend or stand still; the climb rungs have their own bounded law. The
     * same clamped-speed shape as everything else in M3, so the approach to the terminal height
     * rides the braking curve rather than arriving at speed.
     */
    fun descentRate(heightM: Double): Double {
        if (!heightM.isFinite()) return 0.0
        val above = heightM - TARGET_HEIGHT_M
        if (above <= 0.0) return 0.0
        return RepositionGuidance.clampedSpeed(above, V_DESCENT_MAX_MS)
    }

    /**
     * The reacquisition climb, m/s down-positive (so negative = up), bounded by
     * [ARM_CEILING_M]: at or above the ceiling the climb stops and the aircraft holds, because
     * above the band there is nothing a climb can buy — the measured rate above the cliff is
     * not a sensor.
     */
    fun reacquireRate(heightM: Double): Double {
        if (!heightM.isFinite()) return 0.0
        if (heightM >= ARM_CEILING_M) return 0.0
        return -V_REACQUIRE_MS
    }

    /**
     * The terminal test, one tick's worth: at the target height (within [HEIGHT_ACCEPT_M]) with
     * the lateral error inside the **innermost** cone — the cone evaluated at the target height
     * itself, which is the floor. The caller counts [TERMINAL_TICKS] consecutive ticks, because
     * that is state and this object has none.
     */
    fun terminalNow(heightM: Double, lateralErrorM: Double): Boolean {
        if (!heightM.isFinite() || !lateralErrorM.isFinite()) return false
        return abs(heightM - TARGET_HEIGHT_M) <= HEIGHT_ACCEPT_M &&
            lateralErrorM <= coneRadiusM(TARGET_HEIGHT_M)
    }
}

/**
 * **The tag-tracked descent's state machine** — the phase, the terminal counter, and one [step]
 * per engine tick. Pure JVM, no DJI, no Android, no clock of its own (ages arrive as inputs), so
 * the whole machine hand-cranks in `TagDescentGuidanceTest` the way [TakeoffClimb] does.
 *
 * What lives here and what does not, drawn once: **everything the sighting decides** — the
 * ladder, the cone, the terminal latch — is in this class; everything the *engagement* decides —
 * the interlock, the RC sticks, DJI's authority, the link watchdog, the position feed — stays in
 * `GuidedStickEngine`'s abort ladder and descent tick, where every other manoeuvre's copy of
 * those properties already lives and is already mutation-tested. A second copy here would be the
 * two-places-for-one-property failure the engine's KDoc names.
 *
 * Not thread-safe; the engine touches it only under its own lock, exactly as it touches
 * `OrbitState`.
 *
 * @param fullAutoland whether the machine may commit to [TagDescentPhase.DJI_LANDING] (from
 *   TERMINAL, or from TRACKING at the FC's measured descent floor). **Off by default, and a
 *   constructor value on purpose**: the option is the operator's, taken at arm time on the
 *   phone's own explicit toggle, and a machine built without it is exactly yesterday's Stage B —
 *   terminal is final, byte for byte. Design authority: Ivan's 2026-07-28 "go for it" on the
 *   commit-via-SDK pivot after landing04 refuted the held-stick design (the FC floors a
 *   virtual-stick descent at ~1.4 m and honours the stick-down confirmation only from the
 *   physical RC) — the deliberate decision `docs/apriltag-landing.md` §3 requires
 *   touchdown-becomes-ours to rest on, now resting on DJI's own landing flow.
 * @param approach whether the machine starts in [TagDescentPhase.APPROACH] — the arm was taken
 *   above [TagDescentGuidance.ARM_CEILING_M] (and under [TagDescentGuidance.APPROACH_CEILING_M];
 *   the engine's gate owns that refusal). A constructor value for the same reason the option
 *   flag is: whether this engagement began above the band is a fact about the arm, pinned at
 *   the operator's act, and a machine built without it is byte-identical to yesterday's — it
 *   starts at [TagDescentPhase.TRACKING] and can never visit APPROACH. Design authority: Ivan's
 *   approach brief (2026-07-29), after landing13 t=41.8 measured the friction (`tag_descent_denied
 *   above 7m tag band` from the 10 m takeoff hover, resolved by a blind manual descent).
 */
class TagDescent(
    private val fullAutoland: Boolean = false,
    approach: Boolean = false,
) {

    /**
     * The current phase. Starts [TagDescentPhase.TRACKING] — arming requires a fresh fix — or
     * [TagDescentPhase.APPROACH] when the arm was taken above the band.
     */
    var phase: TagDescentPhase =
        if (approach) TagDescentPhase.APPROACH else TagDescentPhase.TRACKING
        private set

    private var terminalTicks = 0

    /** What one tick decided. */
    sealed interface Step {
        /**
         * Fly [velocities]. [entered] is non-null exactly once per phase transition — the edge
         * the engine announces and records — and null on every tick that stays put.
         */
        data class Fly(val velocities: StickVelocities, val entered: TagDescentPhase?) : Step

        /**
         * The fix is older than [TagDescentGuidance.T_ABORT_MS]: the descent is over and the
         * aircraft must be handed back. The machine is dead after returning this — the engine
         * drops it, and a fresh arm builds a fresh machine, which is what makes "no automatic
         * re-engagement" structural rather than remembered.
         */
        object HandBack : Step
    }

    /**
     * One tick. [heightM] null means the altitude is unusable this tick (the engine's
     * `usableAltitude` said so): lateral centring continues, but nothing vertical is commanded
     * in either direction and the terminal test is not evaluated — the graduated treatment the
     * vertical axis gets everywhere in M3, because a cone with no height and a target with no
     * measurement are not things to act on.
     *
     * [fixAgeMs] is the age of the newest **believed** fix (id-matched against the latch, by the
     * engine); the error is measured against that same fix.
     *
     * [floorStalled] is the engine's floor verdict — the altitude has not fallen for
     * [TagDescentGuidance.LAND_STALL_MS] under a sustained down command (the measured FC
     * obstacle floor, landing04). An input rather than computed here because it is a claim
     * about *time and a sensor* (the engine owns both clocks), while this machine deliberately
     * has neither; the machine owns what the claim may *do*, which is commit — and only with
     * every other conjunct fresh on the same tick.
     *
     * [lateralSpeedM] is the aircraft's measured lateral ground speed, m/s, or null when the
     * velocity feed cannot vouch for one — the same engine-owns-the-sensors split as
     * [floorStalled]: the engine reads the feed and its freshness, this machine owns what the
     * fact may do, which is refuse a commit above [TagDescentGuidance.LAND_COMMIT_SPEED_MS]
     * (landing06's ~21 cm momentum term — the constant's KDoc has the budget). **Null defaults
     * closed**: an unknown speed cannot vouch for a slow one, so it holds, exactly as an
     * unknown height does.
     *
     * [heightM] is, since landing07, the **range ladder's** height
     * ([TagDescentGuidance.descentHeight]): the newest believed fix's own tag-derived range
     * when fresh, the baro as the fallback — resolved by the engine, which owns both sensors,
     * exactly as [floorStalled] and [lateralSpeedM] are. This machine neither knows nor cares
     * which instrument spoke; the laws it applies to the number are the same either way, and
     * the record's `height_source` lines carry the provenance.
     *
     * In [TagDescentPhase.DJI_LANDING] the error inputs are the ENGINE's steering error —
     * `landingTarget − position` when the position feed is fresh, exactly (0, 0) when it is
     * not (a dead stick, never a random walk against a cached position). This machine turns
     * them into the capped lateral and structurally nothing else.
     */
    fun step(
        heightM: Double?,
        errorNorthM: Double,
        errorEastM: Double,
        fixAgeMs: Long,
        floorStalled: Boolean = false,
        lateralSpeedM: Double? = null,
    ): Step {
        val rung = TagDescentGuidance.rung(fixAgeMs)

        if (phase == TagDescentPhase.DJI_LANDING) {
            // Committed: DJI owns the descent, so vertical and yaw are the literal constants
            // 0.0 — fighting the FC's own landing on the axis it is flying is structurally
            // impossible here, not merely untested. The lateral is the odometric pivot (the
            // phase's KDoc): the same clamped-speed arithmetic as TRACKING under the landing's
            // own smaller cap, on the error the engine computed against the frozen believed
            // target — or exactly zero when the engine could not vouch for a position this
            // tick. Before the GONE check, deliberately: tag loss here is the EXPECTED shape
            // (DJI recenters the camera off the pad, measured on every landing), so the
            // hand-back rung does not exist — a machine that released mid-landing because its
            // camera was taken away would abandon the very landing it just asked for. No exit
            // inside the law; the engagement's ends are the engine's.
            val (north, east) = TagDescentGuidance.landingLateral(errorNorthM, errorEastM)
            return Step.Fly(StickVelocities(north, east, 0.0, 0.0), entered = null)
        }

        if (rung == TagDescentGuidance.Rung.GONE) return Step.HandBack

        val (north, east) = TagDescentGuidance.lateral(errorNorthM, errorEastM)
        val lateralError = hypot(errorNorthM, errorEastM)

        // The APPROACH segment — above the band, flying down into it. After the GONE check on
        // purpose: T_ABORT's hand-back is unchanged up here (a 10 s decode silence at the
        // measured reach edge — landing13's 9.7 m hover was silent for 7.1 s — means the tag
        // genuinely cannot be seen from this height, and the hand-back is a benign hover the
        // operator re-arms lower from; a longer bound would hold stick authority blind with
        // nothing measured to justify it).
        var enteredBand = false
        if (phase == TagDescentPhase.APPROACH) {
            if (heightM != null && heightM <= TagDescentGuidance.APPROACH_BAND_ENTRY_M) {
                // **The band entry — the one-way handoff.** From here the machine IS today's:
                // the same tick falls through into the ladder below, whose rung decides the
                // phase it lands on (a stale fix at entry announces HOLDING, not a false
                // TRACKING), and nothing below ever transitions back to APPROACH. The edge is
                // forced onto the record via `enteredBand` because a FRESH entry would
                // otherwise compute next == phase and announce nothing — a silent regime
                // change, the bug class "refusal by name" exists to forbid.
                enteredBand = true
                phase = TagDescentPhase.TRACKING
            } else {
                // Above the band the vertical law is: descend in-cone through SHORT staleness
                // — the FRESH and HOLD rungs, i.e. a fix believed within T_CLIMB_MS — and hold
                // beyond it. The bound's arithmetic: 2 s at V_DESCENT_MAX is 0.8 m of
                // odometric descent per decode gap, under the cone radius at the entry height
                // (0.98 m at 6.5 m) and in open air ≥ 5.7 m over the datum — while the
                // measured weather it is sized for is decode gaps of ~3.7 s (8–9 m, 2.7 %
                // per-frame at the 10 Hz cap) to ~7.7 s (9–10 m, 1.3 %): descending only on
                // FRESH would move 16 cm per sporadic fix and park the aircraft above 8 m for
                // minutes, which is the friction this segment exists to absorb. The odometric
                // half of the argument is TRACKING's own: the tag does not move, and a
                // *vertical* descent does not consume the fix's lateral scale — the one part
                // of an above-band fix that is metre-grade — while the bearing it does
                // consume is the robust quantity (the phase's KDoc decomposition). The
                // lateral keeps centring on the believed fix at every rung, exactly as
                // HOLDING/CLIMBING do below; the climb rung's *climb* deliberately does not
                // exist up here (reacquireRate is zero at the ceiling already — there is
                // nothing above the band a climb can buy). Height unknown: nothing vertical,
                // never a guess — the band entry cannot be evaluated blind, same as the
                // engine's climb gate.
                val down = if (
                    heightM != null &&
                    (rung == TagDescentGuidance.Rung.FRESH || rung == TagDescentGuidance.Rung.HOLD) &&
                    lateralError <= TagDescentGuidance.coneRadiusM(heightM)
                ) {
                    TagDescentGuidance.descentRate(heightM)
                } else {
                    0.0
                }
                return Step.Fly(StickVelocities(north, east, down, 0.0), entered = null)
            }
        }

        if (phase == TagDescentPhase.TERMINAL) {
            // TERMINAL's one exit, and only with the operator's explicit option: commit to
            // DJI's landing on a tick whose fix is FRESH and inside the cone at a known height
            // under the commit ceiling, MOVING SLOWLY (landing06: momentum carried across the
            // commit becomes touchdown miss — the speed constant's KDoc has the arithmetic) —
            // the commitment is taken over the tag, demonstrably and at rest, never on a
            // memory of having been there or through it. Any tick that fails a conjunct simply
            // keeps holding; the hold is safe and the next fresh centred slow tick commits.
            // Since landing07 the height is the range ladder's, so inside the ceiling this
            // conjunct set is in practice tag-vouched twice over: the FRESH fix says the tag
            // is in view, and at any height under 2.5 m a visible 75 mm tag is ≥ ~44 px —
            // above the size rung's floor — so heightM is tag-derived. If the impossible
            // happens and the ladder answered baro anyway (a fix whose size range was lost),
            // the commit is deliberately NOT refused: the fresh in-cone fix already vouches
            // the tag is there, its scale is then the baro's — the pre-metric world, which
            // flew — and refusing to land on the fallback instrument while looking at the tag
            // would be the fragility landing07's redesign removed (Ivan, 2026-07-29).
            // (The ceiling conjunct is vacuous from a 0.6 m hold and stated anyway: one commit
            // condition, spelled once per entry point, both under it.)
            if (fullAutoland && heightM != null && rung == TagDescentGuidance.Rung.FRESH &&
                heightM <= TagDescentGuidance.COMMIT_CEILING_M &&
                lateralError <= TagDescentGuidance.coneRadiusM(heightM) &&
                lateralSpeedM != null && lateralSpeedM <= TagDescentGuidance.LAND_COMMIT_SPEED_MS
            ) {
                phase = TagDescentPhase.DJI_LANDING
                return Step.Fly(StickVelocities.ZERO, entered = TagDescentPhase.DJI_LANDING)
            }
            // The safe ending, kept safe: re-descend toward the target only on a fresh fix
            // inside the cone (a gust that lifted the aircraft is corrected), climb back up to
            // the target if it sank below (climbing is always the safe direction), and never
            // command anything below TARGET_HEIGHT_M — descentRate() is zero at and below it by
            // construction. A stale tag in TERMINAL holds; T_ABORT still hands back, through the
            // rung check above, because a hold nobody can see is not a hold to trust forever.
            val down = when {
                heightM == null -> 0.0
                rung != TagDescentGuidance.Rung.FRESH -> 0.0
                heightM < TagDescentGuidance.TARGET_HEIGHT_M -> {
                    // Below the target: climb gently back to it. The reacquire rate is the
                    // right magnitude (slow, safe direction); the ceiling bound is irrelevant
                    // this close to the ground but harmless.
                    -RepositionGuidance.clampedSpeed(
                        TagDescentGuidance.TARGET_HEIGHT_M - heightM,
                        TagDescentGuidance.V_REACQUIRE_MS,
                    )
                }

                lateralError <= TagDescentGuidance.coneRadiusM(heightM) ->
                    TagDescentGuidance.descentRate(heightM)

                else -> 0.0
            }
            return Step.Fly(StickVelocities(north, east, down, 0.0), entered = null)
        }

        val next = when (rung) {
            TagDescentGuidance.Rung.FRESH -> TagDescentPhase.TRACKING
            TagDescentGuidance.Rung.HOLD -> TagDescentPhase.HOLDING
            TagDescentGuidance.Rung.CLIMB -> TagDescentPhase.CLIMBING
            TagDescentGuidance.Rung.GONE -> throw IllegalStateException("handled above")
        }

        // **The floor commit — landing04's lesson made law.** The FC has pinned a commanded
        // descent (floorStalled, the engine's measured verdict) while the machine is TRACKING
        // — i.e. the fix is FRESH — inside the cone, at a known height under the commit
        // ceiling, moving slowly (landing06's momentum lesson, same conjunct as TERMINAL's),
        // with the operator's autoland option armed: every conjunct about "over the pad, near
        // the ground, at rest, allowed to land" holds on this same tick, so commit to DJI's
        // landing. TRACKING is required rather than incidental: a stalled altitude with a
        // stale fix is an FC floor over ground nobody can vouch for, and it keeps holding.
        // The height is the range ladder's since landing07 — tag-derived at any height this
        // ceiling admits (TERMINAL's commit comment carries the argument, including why a
        // ladder that fell to baro is accepted rather than refused).
        if (next == TagDescentPhase.TRACKING && fullAutoland && floorStalled &&
            heightM != null && heightM <= TagDescentGuidance.COMMIT_CEILING_M &&
            lateralError <= TagDescentGuidance.coneRadiusM(heightM) &&
            lateralSpeedM != null && lateralSpeedM <= TagDescentGuidance.LAND_COMMIT_SPEED_MS
        ) {
            phase = TagDescentPhase.DJI_LANDING
            return Step.Fly(StickVelocities.ZERO, entered = TagDescentPhase.DJI_LANDING)
        }

        val down = when (next) {
            TagDescentPhase.TRACKING ->
                if (heightM != null && lateralError <= TagDescentGuidance.coneRadiusM(heightM)) {
                    TagDescentGuidance.descentRate(heightM)
                } else {
                    0.0
                }

            TagDescentPhase.HOLDING -> 0.0

            // Height unknown while the tag is lost: hold rather than climb, because the climb's
            // ceiling bound cannot be enforced blind — the same reason the engine's climb gate
            // blocks an unknown-altitude climb.
            TagDescentPhase.CLIMBING ->
                if (heightM != null) TagDescentGuidance.reacquireRate(heightM) else 0.0

            TagDescentPhase.APPROACH -> throw IllegalStateException("approach handled above")
            TagDescentPhase.TERMINAL -> throw IllegalStateException("terminal handled above")
            TagDescentPhase.DJI_LANDING -> throw IllegalStateException("landing handled above")
        }

        // The terminal latch: only from TRACKING with a usable height, counted consecutively,
        // reset by any tick that fails either conjunct — a fly-through cannot complete.
        if (next == TagDescentPhase.TRACKING && heightM != null &&
            TagDescentGuidance.terminalNow(heightM, lateralError)
        ) {
            terminalTicks++
            if (terminalTicks >= TagDescentGuidance.TERMINAL_TICKS) {
                phase = TagDescentPhase.TERMINAL
                // Entering the hold at zero vertical: the aircraft is inside the accept band and
                // the hold's own logic takes over next tick.
                return Step.Fly(
                    StickVelocities(north, east, 0.0, 0.0),
                    entered = TagDescentPhase.TERMINAL,
                )
            }
        } else {
            terminalTicks = 0
        }

        // `enteredBand` forces the edge on the handoff tick: the approach set phase = TRACKING
        // above, so a FRESH entry would read next == phase and record nothing.
        val entered = if (next != phase || enteredBand) next else null
        phase = next
        return Step.Fly(StickVelocities(north, east, down, 0.0), entered)
    }
}
