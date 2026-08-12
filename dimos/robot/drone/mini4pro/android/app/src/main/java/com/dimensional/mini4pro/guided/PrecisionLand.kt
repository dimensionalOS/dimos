package com.dimensional.mini4pro.guided

import kotlin.math.min

/**
 * **A plan's `NAV_LAND` with "Precision Land" set lands the aircraft on the AprilTag** — the
 * sequence's own two numbers, its gates, and the phase vocabulary the engine's tick flies. Pure
 * functions and constants, no state, no DJI, no Android, pinned by `PrecisionLandTest`.
 *
 * Design authority: Ivan, 2026-07-30, on `/home/lesh/Documents/QGroundControl Daily/Missions/big1.plan`
 * — takeoff 10 m, six waypoints to 60 m, final item `NAV_LAND` frame 3 at 15 m with `param2 = 2`
 * (Required) at 37.99388856 / 23.72531951, which is 0.3 m from that plan's own planned home.
 *
 * ## What this file is, and what it deliberately is not
 *
 * It is **the sequencing decision** — where the aircraft goes, how low it gets, and the two
 * conditions under which the whole thing is refused before it starts. It contains **no descent law
 * and no landing law at all**: below the arm height every metre is
 * [TagDescent]/[TagDescentGuidance]'s, flown by the engine's existing tag-descent tick, armed
 * through the same door the phone's arm uses, with every gate that door applies binding unchanged.
 * Ivan's instruction was *"trigger a full landing including the last half a metre"*, and the way to
 * honour it is to **call** the machine that has flown thirteen recorded landings rather than to
 * grow a second one here.
 *
 * ## The sequence, in the order the aircraft flies it
 *
 * ```
 * cursor reaches the NAV_LAND item (param2 >= 1)
 *   ├─ [gate] — refuse, and the mission ENDS holding, if any of it fails
 *   ├─ any still-active ROI is cleared (see "The ROI" below)
 *   ├─ TRANSIT : fly to (recorded takeoff lat/lon, the ITEM's own altitude)   ← translates and descends
 *   ├─ AIMING  : command the camera to nadir and wait for the belief, bounded
 *   ├─ LOWER   : descend to min([LAND_TAG_ARM_HEIGHT_M], current) at the same lat/lon
 *   ├─ ACQUIRE : keep descending, gently, to min([LAND_TAG_ACQUIRE_FLOOR_M], the arm height),
 *   │            asking the descent's own arm gate on every tick and arming the instant it clears
 *   ├─ HOLD    : at the floor, stop descending and keep asking, for [LAND_TAG_ACQUIRE_HOLD_MS]
 *   │            — landing17's 0.27 s, and the refusal that beat it
 *   └─ arm the tag descent with full autoland — APPROACH, band entry, tracking,
 *      terminal, commit, KeyStartAutoLanding, touchdown at motors-off
 * ```
 *
 * ## The reference point: **the recorded takeoff position, never the drawn one**
 *
 * Ivan corrected an earlier reading of this and the correction is the load-bearing part of the
 * feature: *"plan's lat/lng will slightly differ from executed one, can be 5 m away for example"*.
 * The tag is physically on **the pad the aircraft actually left**, so:
 *
 *  - the **XY of both flown phases** is the recorded takeoff position;
 *  - the plan's own `NAV_TAKEOFF` coordinate and the Land item's drawn coordinate are **never
 *    targets**. The Land item's coordinate serves exactly one purpose — the sanity cross-check in
 *    [gate], which a 5 m disagreement passes harmlessly and a plan drawn at another site fails by
 *    name.
 *
 * **The single owner of "where we actually took off" is DJI's own home point** — `KeyHomeLocation`
 * as `AircraftState.homeLatitude`/`homeLongitude`, admitted only when `homeLocationSet` is DJI's own
 * `true` (the 2026-07-26 sentinel measurement: before a home exists the key returns a *populated*
 * `4.583662361046586E7`, so the coordinate cannot answer the question by itself). Three reasons it
 * is the right owner rather than a new field of our own:
 *
 *  - **DJI re-records it at every takeoff**, at the then-current GPS reading — measured across
 *    landing09's session at ~1 m of wander (`HOME_POSITION`, and CLAUDE.md's analysis trap). So it
 *    tracks the pad the aircraft actually left, per flight, with no bookkeeping of ours to go stale.
 *  - **It is already the origin of the frame the tag fixes live in.** `TagWorld.fix` places a
 *    sighting north/east **of home**, and the descent's own arm gate refuses without it. Using a
 *    second definition of the launch point would mean the leg that puts the pad in frame and the
 *    fix that centres on it were measured from different origins.
 *  - **A moved home is already refused upstream.** `MissionLaunch` blocks a mission start after any
 *    `DO_SET_HOME` this session (M4-14), which is exactly the case where "home" would stop meaning
 *    "where we took off".
 *
 * The ~1 m of session wander is harmless here, and this is the paragraph that says why: **"straight
 * down" ends at the arm.** Below [LAND_TAG_ARM_HEIGHT_M] the aircraft centres on what the camera
 * *sees*, not on any coordinate, so the recorded takeoff position only has to be good enough to put
 * the pad **in frame** at 8 m. Through the [com.dimensional.mini4pro.vision.CameraCalibration]
 * prior's fitted focal length (fx = 1457 px at 1920×1080, two flights, ±1.2 %) the 1920×1080 frame
 * at 8 m covers `8 × 960/1457 ≈ ±5.3 m` across and `8 × 540/1457 ≈ ±3.0 m` up the image — some five
 * times the wander in the narrow axis.
 *
 * ## The camera, and why it is aimed at 15 m rather than at 8 m
 *
 * Ivan asked the question directly (*"do we point down at 8 m or at 15 in this case?"*) and the
 * answer is **on arrival at the item's altitude, before the descent to the arm height**:
 *
 *  - **Aiming early costs nothing.** At the item altitude the tag is undecodable anyway: 75 mm
 *    through 1457 px is `0.075 × 1457 / 15 ≈ 7 px` at 15 m, half the ~14 px cliff where the
 *    detection rate collapses and below even the 11.3 px median of the sporadic 9–10 m singles
 *    (`2026-07-27-apriltag-c-vs-opencv.md` §1, and [TagDescentGuidance.APPROACH_CEILING_M]'s KDoc).
 *  - **Aiming late costs the arm.** With the camera already settled at nadir the detector latches
 *    and fixes start flowing on the way down the moment the marker is big enough (~12 px at 9 m), so
 *    the arm at 8 m usually passes on its first tick instead of hovering while the gimbal slews and
 *    the belief settles — and the descent itself becomes *observed*: the tag lines from 15 m down are
 *    free evidence for the next flight's record.
 *  - **The measured-good shape is exactly this**: landing06 aimed `pitchDeg=-90 absolute` while
 *    airborne (t=19.77) and every descent afterwards armed clean.
 *
 * The aim is not instant, so [AIMING] is a phase rather than a line: the camera is commanded through
 * the same [ManoeuvreGimbal] path the takeoff sequence uses (one path to DJI, one rate window, one
 * recorded `dji_call op=gimbal_rotate`), and the sequence then **waits for the belief** —
 * `gimbal/PitchBelief`'s commanded-or-reported answer, the identical number
 * `GuidedStickEngine.descentGateLocked` and `TagWorld.fix` judge — for at most
 * [NADIR_AIM_LIMIT_MS], after which the sequence is refused by name rather than descending with a
 * camera nobody can vouch for.
 *
 * ## The ROI
 *
 * `updateRoiCameraLocked` runs on **every** engaged tick, so an ROI still in force would re-aim the
 * gimbal at its own depression angle on every tick and fight the nadir command forever — a deadlock
 * ending in a phase timeout, with the camera pointing at the operator's subject the whole way down.
 * `DO_SET_ROI_NONE` deliberately does **not** re-point the camera (a default is a lie about where the
 * target is), so a clear alone would not be enough either: the nadir command has to be explicit *and*
 * unopposed. The sequence therefore **clears the ROI when it begins**, through the same single owner
 * `DO_SET_ROI_NONE` uses, and records the fact by name — the operator's ROI ending is something they
 * are owed an account of.
 *
 * **Which ROIs can be in force here is narrower than it looks, and the narrowness is a gap rather
 * than a comfort:** a *plan's* own `DO_SET_ROI_LOCATION` item never reaches this engine today —
 * `MissionStore.resolve` carries it into `ResolvedLeg.roi` and `MissionLaunch.routeOf` drops it,
 * because a [MissionStep] has no ROI field — so big1.plan's items 6 and 8 currently point nothing
 * (`MissionBig1PlanTest` asserts exactly that rather than leaving it assumed). What *can* be live at
 * the Land item is a Fly-view `DO_SET_ROI_LOCATION` sent mid-mission, or an orbit's implied centre.
 * The clear is written for both, and it is what will keep a plan's ROI items from deadlocking this
 * landing on the day they are wired.
 *
 * ## What refusal means here
 *
 * Ivan: *"just error out"*. A failed [gate], a camera that never reaches nadir, or a refused arm all
 * mean: **the mission ends, the aircraft holds where it is, and the reason is named on the record and
 * on the operator's screen.** There is no fallback landing and no DJI go-home substitution on any
 * path — the aircraft is left hovering, which is the state M4-5 already chose for every other way a
 * mission can finish.
 */
object PrecisionLand {

    /**
     * **How far from the recorded takeoff point the sequence may be asked to run, metres.**
     *
     * Two things are measured against it in [gate], both at the moment the item begins: where the
     * *aircraft* is, and where the Land item was *drawn*.
     *
     * **A bound Ivan set, not a fitted number** — said plainly, because this file's neighbours carry
     * measurements and this one carries a decision. What the evidence contributes is that 20 m is
     * comfortably above every legitimate disagreement and far below the hazard:
     *
     *  - the pad-vs-plan disagreement Ivan named is metres (*"can be 5 m away for example"*); in
     *    big1.plan it is 0.3 m, and the last waypoint before the Land item is ~8 m away;
     *  - DJI's own home wander across a session is ~1 m (landing09);
     *  - the hazard this refuses — a plan drawn at another site, or an aircraft that has ended up
     *    somewhere else entirely — is hundreds of metres, not tens.
     *
     * For scale within the same layer: [MissionGuidance.MAX_HOME_DIST_M] is 2000 m (Ivan's
     * 2026-07-30 limit) and [MissionGuidance.REJOIN_MAX_M] is 50 m (a leg nobody drew). This is the
     * tightest of the three, which is right: it gates a leg that ends with the aircraft on the
     * ground. **Untouched by the envelope expansion, deliberately** — the disagreements it bounds
     * are metres of GPS wander and pad-versus-plan, which do not grow because the mission may now
     * fly further.
     */
    const val LAND_TAG_RADIUS_M = 20.0

    /**
     * **The height the sequence starts *looking* from, metres above the takeoff datum — the top of
     * an acquisition band, not a commit point.**
     *
     * It was a commit point until 2026-07-30 ("descend to 8 m, arm once, refuse if the arm fails")
     * and landing16 measured why that was wrong. See [LAND_TAG_ACQUIRE_FLOOR_M] for the band's
     * other end and the measurement that put it there; what stays true of *this* number is why the
     * looking begins here rather than higher or lower.
     *
     * Chosen against the descent's own measured band rather than picked round:
     *
     *  - **Above [TagDescentGuidance.ARM_CEILING_M] (7.0 m), so the approach segment does the
     *    centring.** That is deliberate and it is Ivan's spec: an arm between 7 and 12 m enters
     *    [TagDescentPhase.APPROACH], which descends on baro height while centring on the sparse fixes
     *    and hands off into the tracking ladder at [TagDescentGuidance.APPROACH_BAND_ENTRY_M]. Arming
     *    *inside* the band instead would throw that segment away and demand the sequence deliver a
     *    centred aircraft, which is work the descent already does better.
     *  - **Well under [TagDescentGuidance.APPROACH_CEILING_M] (12.0 m)**, with 4 m of margin against
     *    the ~1.2 m of within-session barometric wander (landing07, the
     *    `2026-07-27-altitude-datum-wander.md` family) that decides whether a true 8 m hover *reads*
     *    as 8 m. An arm height with less margin than the instrument's own lie scale would be a
     *    sequence that refuses itself on a warm afternoon.
     *  - **It is a height with measured decodes.** landing06's 8.0 m hover delivered 110 fixes at a
     *    13.9 px median — this is not an extrapolation to the edge of the sensor, it is the height a
     *    recorded flight sat at while the pipeline worked. It is *under*
     *    [com.dimensional.mini4pro.vision.TagArming.ACQUIRE_CEILING_M] (10 m since 2026-07-30, the
     *    measured decode reach), which is what makes the detector's own ceiling a non-issue here —
     *    it was 8.0 m until that day, and landing16 arrived at 8.8 m, above it.
     *
     * The arrival test's vertical tolerance is [RepositionGuidance.VERTICAL_ACCEPT_M] = 1 m, so the
     * looking begins somewhere in **7–9 m** — and that spread is exactly why an arm here cannot be
     * a single attempt. landing16 settled at **8.8 m** (`datasets/landing16/20260730-161329.001.jsonl`,
     * t=188.8) where the tag measured ~11 px, and the profiling descent's per-frame decode rate is
     * 92.6 % at 7–8 m but **2.7 % at 8–9 m** (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md`
     * §1). Against [TagDescentGuidance.ARM_FRESH_MS] = 400 ms that is a coin flip at best, so the
     * sequence keeps descending until the gate clears rather than betting the landing on one tick.
     */
    const val LAND_TAG_ARM_HEIGHT_M = 8.0

    /**
     * **The bottom of the acquisition band, metres above the takeoff datum: the lowest the sequence
     * will descend hunting for an arm before it refuses by name.** 5.0 m.
     *
     * ## Why the band exists at all — landing16's video, measured
     *
     * Written up in full in `docs/measurements/2026-07-30-landing16-arm-height.md`; the short form:
     *
     * The sequence's first design armed once, on arrival at [LAND_TAG_ARM_HEIGHT_M]. landing16
     * (`datasets/landing16/20260730-161329.001.jsonl`) arrived at 8.8 m and held there for ~14 s
     * with nothing to arm on. Two separate faults produced that, and both are fixed independently:
     * the detector was disarmed (the mission takeoff never aimed the camera, so nothing latched —
     * `vision/TagArming`), **and** the height itself was marginal. The video says so directly:
     * frame 4710 of `20260730-161329.v002.h264` (t ≈ 195 s), the tag at pixel (923, 512) against
     * `CameraCalibration`'s measured nadir point (970.7, 615.2) — 48 px across and 103 px up, which
     * at 8.8 m through fx = 1457 is **0.29 m and 0.62 m: the pad lay 0.69 m from directly beneath
     * the aircraft**. That 0.69 m is also the whole justification for this feature — a GPS-flown
     * approach put the aircraft two thirds of a metre off a pad it must land *on*, comfortably
     * inside the tag descent's own cone but nowhere near good enough without it.
     *
     * At that height the marker measured ~11 px. The decode-rate curve
     * (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1) reads 92.6 % per frame at 7–8 m
     * and **2.7 % at 8–9 m**; [TagDescentGuidance.ARM_FRESH_MS] demands a fix under 400 ms old. So
     * arming *once* at 8–9 m is a coin flip with the coin weighted against, and the honest fix is to
     * keep looking on the way down rather than to move the start height into the reliable band and
     * lose the approach segment.
     *
     * ## Why 5.0 m and not lower
     *
     *  - **It is a whole measured band below the cliff, with margin.** 7–8 m decodes at 92.6 % and
     *    the 100 % band reaches 7.0 m, so anything under ~6.5 m is the reliable region rather than
     *    its edge; 5.0 m is 2 m of margin under the cliff, which is the same shape of margin
     *    [TagDescentGuidance.APPROACH_ENTRY_MARGIN_M] uses at the other handoff.
     *  - **It leaves the descent's own work untouched.** [TagDescentGuidance.ARM_CEILING_M] is
     *    7.0 m, so an arm taken anywhere in this band below 7.0 m enters
     *    [TagDescentPhase.TRACKING] directly and the ladder gets the whole of its own descent;
     *    [TagDescentGuidance.COMMIT_CEILING_M] (2.5 m) and [TagDescentGuidance.TARGET_HEIGHT_M]
     *    (0.6 m) are far below.
     *  - **It is far above the firmware's own floor.** The FC floors a virtual-stick descent at
     *    ~1.0–1.4 m (landing04/landing12), which is the height at which *this* machine's descent
     *    would stop meaning anything; 5.0 m never gets near it, so a failed acquisition ends in a
     *    hover the operator can act on rather than in a stand-off with obstacle sensing.
     *  - **It keeps the size cross-check honest.** The 20 px size-range floor sits at ~5.5 m
     *    ([TagDescentGuidance.APPROACH_CEILING_M]'s KDoc), so the arm taken at the bottom of this
     *    band has a size range worth cross-checking the barometer against.
     *
     * **What happens at the floor is a bounded wait, and then a refusal — never a landing.**
     * Until landing17 the floor was an *instant* refusal and that was the bug: see
     * [LAND_TAG_ACQUIRE_HOLD_MS], which owns the wait and the flight that measured it. Nothing
     * here descends on the hope that something will appear; the hold is a hover over the pad at a
     * height a whole measured band inside the reliable one, and it ends in Ivan's *"just error
     * out"* exactly as the instant refusal did.
     *
     * ## This floor is barometric, and the barometer lies by a metre or two
     *
     * **Measured, landing17** (`datasets/landing17/20260730-172355.001.jsonl`): the descent was
     * commanded to this 5.0 m floor and the arrival test fired at t=262.192 with `relalt` reading
     * **5.8 m** and 0.4 m/s of descent still on it (the vertical accept band is
     * [RepositionGuidance.VERTICAL_ACCEPT_M] = 1 m, so "arrived" legitimately happens up to a
     * metre high and still moving). The first sighting 0.27 s later, at an indicated 5.7 m,
     * solved the marker at **z = 7.39 m** — 14.8 px through fx = 1457 and a 75 mm tag is
     * `1457 x 0.075 / 14.8 = 7.38 m`, the two agreeing to a centimetre. The same flight later
     * recorded `range_baro_divergence tag=1.54 baro=0.10 ratio=15.4` at the other end of the
     * descent.
     *
     * So **a floor expressed in barometric metres can sit one to two metres from the truth in
     * either direction**, and on that flight it sat *high*: the sequence refused at a true 7.4 m,
     * inside the 2.7 %-per-frame decode band, believing it had spent its whole 8→5 m ladder.
     *
     * This is deliberately **not** fixed by flying the ladder on tag range: before acquisition
     * there is no tag range, by definition — that is what acquisition means, and
     * `TagDescent`'s own height-source ladder only exists once fixes are flowing. The honest
     * mitigation is the hold: a wait at the floor costs nothing and covers a floor that is a
     * metre or two off, in the direction that matters (too high), without pretending the number
     * is better known than it is.
     *
     * The height at which the arm actually succeeded goes on the record (`land_tag_phase armed
     * at=… held=…ms`), because it is a measurement: accumulated over flights it says where this
     * marker's real acquisition band is, and it is the number that would justify moving either
     * end of this one. The same two numbers go on the *refusal* (`land_tag_refused … held …`),
     * because a flight that waited the whole window and never armed measures the band's other
     * edge just as sharply.
     */
    const val LAND_TAG_ACQUIRE_FLOOR_M = 5.0

    /**
     * **How long the sequence hovers at [LAND_TAG_ACQUIRE_FLOOR_M] still asking the arm gate,
     * before it refuses by name.** 10 s.
     *
     * ## The flight that made this a constant instead of nothing — landing17, to the millisecond
     *
     * `datasets/landing17/20260730-172355.001.jsonl`, and the sequence is worth reading in order
     * because every part of the fix is in it:
     *
     * | t (s) | what the record says |
     * |---|---|
     * | 258.887 | `land_tag_phase acquiring from=8.0 to=5.0` — the band begins |
     * | 262.192 | `land_tag_refused seq=9 no tag by 5m … still said TAG_NOT_IN_VIEW: newest fix 193188ms old` |
     * | **262.459** | **the first sighting: px 14.8, `from_h` 5.7 m, solved range 7.39 m** |
     * | 262.6 → 288.5 | the aircraft sits there and the detector produces **133 more fixes** |
     * | 288.607 | Ivan, flying it manually, arms by hand at 3.5 m with `fixAge=85` |
     * | 302.291 | `landing_commit height=1.4` — and the landing works, first time |
     * | 311.642 | motors off |
     *
     * **The refusal beat the tag into view by 267 ms.** Everything the sequence needed arrived a
     * quarter of a second after it gave up, and then kept arriving for twenty-six seconds. The
     * detector was armed and correct throughout (the tag was latched at takeoff, id 0); the
     * marker simply is not decodable until it reaches ~15 px, which is exactly where it was when
     * the floor was reached. The floor's height was not wrong. **The refusal's timing was.**
     *
     * ## Why ten seconds, when the evidence says one would have done
     *
     * Three separate things have to fit inside the window, and only the first is measured at
     * 0.27 s:
     *
     *  - **The tag becoming decodable.** landing17: 267 ms after the refusal. One second covers it
     *    with a factor of four.
     *  - **A fix surviving [TagDescentGuidance.ARM_FRESH_MS]** (400 ms) at the tick the gate is
     *    asked. Sightings at this height are *intermittent*, not continuous: across landing17's
     *    26 s hover the median gap between fixes was **116 ms**, but **13 gaps exceeded 400 ms**
     *    and the largest was **2.20 s** (t=272.276→274.473). A window sized on the median would
     *    be a window that refuses on a two-second dry spell in the middle of a perfectly good
     *    acquisition. Ten seconds is four and a half times the worst dry spell measured.
     *  - **The aircraft settling.** The arrival test fires up to
     *    [RepositionGuidance.VERTICAL_ACCEPT_M] = 1 m early and, measured on landing17, with
     *    0.4 m/s of descent still on the aircraft; `vd` reached 0 at t≈262.68, half a second
     *    later. The hold is not a freeze — the leg keeps flying to the floor — so the last of the
     *    descent happens inside the window, and the window has to be long enough that the
     *    aircraft is *asked* while at rest rather than only while still moving.
     *
     * And the cost of the window is close to nothing, which is the other half of the argument:
     * **it is a hover over the pad**, the safest place this sequence ever puts the aircraft —
     * station-held at 5 m over the point it took off from, with an operator watching, [Phase.HOLD]
     * announced on the screen and on the wire, and a stick grab that ends it instantly like any
     * other phase. Ten seconds of that is roughly 3 % of a battery and it is bounded: the
     * whole-mission cap and Q1's idle window still bind above it, and the refusal at the end is
     * the same refusal, by the same name, carrying the same gate's last word.
     *
     * **Not longer than ten**, because a window that outlasts an operator's patience stops being
     * a wait and becomes a hang: past ten seconds the honest thing is to say the acquisition
     * failed and let a human decide, which is exactly what landing17's operator did — he took
     * over, descended, and landed on the first try.
     */
    const val LAND_TAG_ACQUIRE_HOLD_MS = 10_000L

    /**
     * How long the sequence waits for the **believed** camera pitch to reach nadir after commanding
     * it, milliseconds, before refusing by name.
     *
     * Normally one tick: `gimbal/PitchBelief` prefers the **commanded** angle once this bridge has
     * aimed the camera, so the belief is nadir on the tick after [AIMING] begins. The wait exists for
     * the session where the belief can only come from the *reported* attitude — no gimbal path wired,
     * or a command DJI never enacted — and it is sized against the only slew rate this project has
     * measured: DJI's own landing recenter, ~300 °/s (landing06), which crosses the gimbal's entire
     * travel in well under a second. Three seconds is five times that, and short enough that a
     * refusal still arrives while the aircraft has fuel to do something about it.
     *
     * Deliberately **not** an excuse to descend anyway: the sequence's next act is to fly toward the
     * ground and then hand over to a machine whose every fix is trigonometry on this angle.
     */
    const val NADIR_AIM_LIMIT_MS = 3_000L

    /**
     * Which part of the sequence is flying. Five values, so a `when` breaks loudly the day a sixth
     * arrives — and the arm is deliberately not one of them: the arm ends the mission run, so there
     * is no phase in which "we are landing" is a state of this machine. That belongs to
     * [TagDescentPhase], which owns it.
     *
     * It was four until 2026-07-30, when landing17 split the acquisition in two: [ACQUIRE] is the
     * descent that looks, [HOLD] is the wait that looks. See [HOLD] for why that is a phase and not
     * a longer [ACQUIRE].
     */
    enum class Phase {
        /**
         * The leg to `(recorded takeoff lat/lon, the item's own altitude)`. **Ivan's instruction
         * verbatim**: *"the way we should treat this land message height is the actual height you
         * create a waypoint to and just go there"* — so this leg both translates and changes
         * altitude, which in big1.plan is 60 m → 15 m while moving ~8 m laterally. Climb is
         * permitted (the plan cleared that airspace, and [gate] is what forbids starting the
         * sequence from below it); [GuidedEnvelope.CEILING_M] still binds through the engine's
         * existing climb gate, and `MissionAdmission`/`MissionLaunch` have already checked the
         * item's altitude against the M3 envelope twice.
         */
        TRANSIT,

        /** The camera commanded to nadir, and the bounded wait for the belief. See the object KDoc. */
        AIMING,

        /**
         * The pure altitude change to the arm height, **at the same recorded lat/lon** — station
         * held, nothing lateral asked for beyond keeping the pad under the aircraft.
         */
        LOWER,

        /**
         * **The acquisition descent**: the same station-held altitude change, continued to
         * [acquireFloorTargetM], with the descent's own arm gate asked on every tick and the arm
         * taken the instant it clears.
         *
         * It is a separate phase from [LOWER] rather than a longer one, and the difference is what
         * each is waiting for. [LOWER] is waiting to *arrive* — an arrival test, a leg timeout
         * scaled to a known leg. This one is waiting to *acquire*, and its success has nothing to
         * do with arriving anywhere: arriving is how it **fails**. Folding them together would give
         * one phase two success conditions and one deadline for both, which is how a leg timeout
         * ends up meaning two different things on the same record.
         *
         * The aircraft is doing exactly what it did in [LOWER] — same law, same caps, same target
         * lat/lon, only a lower target height — so nothing about the *flying* is new here. See
         * [LAND_TAG_ACQUIRE_FLOOR_M] for the measurement that put the floor where it is.
         *
         * **Reaching the floor is no longer how this ends** — it is how [HOLD] begins.
         */
        ACQUIRE,

        /**
         * **The wait at the floor**: the leg still commanded to [acquireFloorTargetM], the arm gate
         * still asked on every tick, for [LAND_TAG_ACQUIRE_HOLD_MS] — and *then* the refusal.
         *
         * It is a phase rather than a longer [ACQUIRE] for the reason [ACQUIRE] is a phase rather
         * than a longer [LOWER], applied once more: **what the bound means is different.**
         * [ACQUIRE]'s deadline is a leg timeout, scaled by [MissionGuidance.legTimeoutMs] to the
         * distance it has to fly, and it means *this descent is not making progress*. This one's
         * deadline is not about progress at all — the aircraft is exactly where it wants to be —
         * and expiring is not a fault in the flying but the whole answer: **the tag never became
         * decodable here.** One phase carrying both would have one clock meaning two things, and a
         * record on which a stuck descent and a failed acquisition look the same.
         *
         * The separation is also what makes the two announcements honest ("looking for the tag" on
         * the way down, "holding, still looking" at the bottom) and what lets the record time the
         * wait: `land_tag_phase holding at=…` starts the clock a post-flight reader can measure
         * the acquisition against.
         *
         * **The aircraft is not frozen.** The leg keeps flying to the floor, so the last of the
         * descent — landing17 stopped 0.8 m above it, still moving — happens inside this phase and
         * the marker keeps growing while the gate is asked. What ends is *hunting lower*: there is
         * nothing below the floor this sequence will go looking at.
         */
        HOLD,
    }

    /** What [gate] decided. A place to fly to and a height, or a refusal with a word for the wire. */
    sealed interface Decision {

        /**
         * @param takeoffLatDeg the recorded takeoff point — the XY of **both** flown phases.
         * @param transitRelAltM the Land item's own altitude, metres above the takeoff datum.
         */
        data class Clear(
            val takeoffLatDeg: Double,
            val takeoffLonDeg: Double,
            val transitRelAltM: Double,
        ) : Decision

        /**
         * @param reason the word the `STATUSTEXT` carries — short, because the framing has to fit
         *   round it.
         * @param detail the sentence for the log and the flight record, where there is room to say
         *   which numbers failed.
         */
        data class Refused(val reason: String, val detail: String) : Decision
    }

    /**
     * **The two gates Ivan asked for, plus the three things that must be known before either can be
     * evaluated, plus the cross-check that catches a plan drawn at another site.** Measured at the
     * moment the item begins, which is the only moment at which "where we actually took off" and
     * "how high are we" are facts rather than predictions.
     *
     * Ordered cheapest-and-most-fundamental first, exactly as `MissionLaunch.evaluate` is, and
     * **first failure only** — a list of five reasons is a list nobody reads:
     *
     *  1. **the recorded takeoff point exists** — without it there is no reference point, no target
     *     for either phase, and no origin for the frame the tag fixes will arrive in;
     *  2. **the item names a height** — unreachable, because [MissionLaunch.routeOf] refuses to build
     *     the step without one; kept as a fail-closed branch, on the
     *     `tickMissionTakeoffLocked`-with-no-takeoff precedent;
     *  3. **the plan names a takeoff height** — the number gate 6 compares against. A plan with no
     *     `NAV_TAKEOFF` item is refused **by name** rather than waved through: the gate's whole
     *     content is *"do not start this from below the height the plan cleared"*, and a plan that
     *     cleared nothing supplies no such height. Unknown is never zero (CLAUDE.md), so it is not
     *     silently treated as 0 m — which would make the gate vacuous exactly when nobody checked;
     *  4. **the current height is knowable** — the gate below cannot be evaluated blind;
     *  5. **the site cross-check**: the recorded takeoff point within [LAND_TAG_RADIUS_M] of the Land
     *     item **as drawn**. This is the "plan drawn at another site" hazard, and it is judged before
     *     the aircraft's own position because it is a statement about the *plan* — the operator's fix
     *     is to redraw it, not to fly somewhere else;
     *  6. **the aircraft within [LAND_TAG_RADIUS_M] of the recorded takeoff point**, so the sequence
     *     never begins with a long unsupervised leg toward the ground;
     *  7. **the aircraft at or above the plan's `NAV_TAKEOFF` altitude**, so the sequence never
     *     begins from below the height the plan cleared.
     *
     * Distances are [RepositionGuidance.horizontalMetres] — the **flight law's own ruler**, not the
     * mission layer's [com.dimensional.mini4pro.mission.MissionGeo.distanceM]. The two differ by
     * where they evaluate the `cos(latitude)` term and by 0.11 % of scale
     * (`MissionGeo.METRES_PER_DEG`'s KDoc has the whole history), and this gate measures **the same
     * two points the transit leg is about to fly between**: a gate that clears a leg should measure it
     * with the ruler that will fly it.
     *
     * Pure: no clock, no aircraft, no store — so every row above has its own test with no fakes in
     * it, exactly as `MissionLaunchTest` does for §7.2.
     *
     * @param aircraft the aircraft's own fresh position, or null. The caller has already established
     *   freshness — a stale fix returns before this is called at all, because the mission tick's
     *   position block sits above every completion test (§3.4).
     * @param takeoff the recorded takeoff point ([com.dimensional.mini4pro.telemetry.Geo]-validated,
     *   and admitted only on DJI's own `homeLocationSet == true`), or null when there is none.
     * @param currentRelAltM `usableAltitude` — metres above the takeoff datum, or null when the
     *   altitude cannot be trusted.
     * @param planTakeoffRelAltM the height the plan's own `NAV_TAKEOFF` item named, or null when the
     *   plan has no takeoff item.
     */
    fun gate(
        aircraft: Pair<Double, Double>?,
        takeoff: Pair<Double, Double>?,
        itemLatDeg: Double,
        itemLonDeg: Double,
        itemRelAltM: Double?,
        currentRelAltM: Double?,
        planTakeoffRelAltM: Double?,
    ): Decision {
        if (takeoff == null) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_NO_TAKEOFF_POINT,
                "no recorded takeoff point — the tag is on the pad the aircraft left, and " +
                    "nothing says where that was",
            )
        }
        if (itemRelAltM == null || !itemRelAltM.isFinite()) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_NO_HEIGHT,
                "the land item names no height to fly to (refused at Start; unreachable)",
            )
        }
        if (planTakeoffRelAltM == null || !planTakeoffRelAltM.isFinite()) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_NO_PLAN_TAKEOFF,
                "the plan has no takeoff item, so there is no cleared height to compare against",
            )
        }
        if (currentRelAltM == null || !currentRelAltM.isFinite()) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_NO_DATUM,
                "no usable height above the takeoff datum — the plan-height gate cannot be evaluated",
            )
        }
        val itemToTakeoff = RepositionGuidance.horizontalMetres(
            takeoff.first, takeoff.second, itemLatDeg, itemLonDeg,
        )
        if (!itemToTakeoff.isFinite() || itemToTakeoff > LAND_TAG_RADIUS_M) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_SITE,
                "the land item is drawn %.1f m from where we took off, past the %.0f m bound"
                    .format(itemToTakeoff, LAND_TAG_RADIUS_M),
            )
        }
        if (aircraft == null) {
            // Unreachable: the tick's position block returns above this. Fail-closed all the same.
            return Decision.Refused(
                GuidedStatusTexts.REASON_NO_FIX, "no position fix to measure the takeoff radius from",
            )
        }
        val aircraftToTakeoff = RepositionGuidance.horizontalMetres(
            aircraft.first, aircraft.second, takeoff.first, takeoff.second,
        )
        if (!aircraftToTakeoff.isFinite() || aircraftToTakeoff > LAND_TAG_RADIUS_M) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_TOO_FAR,
                "%.1f m from where we took off, past the %.0f m bound"
                    .format(aircraftToTakeoff, LAND_TAG_RADIUS_M),
            )
        }
        if (currentRelAltM < planTakeoffRelAltM) {
            return Decision.Refused(
                GuidedStatusTexts.REASON_LAND_TOO_LOW,
                "%.1f m is below the %.1f m the plan's takeoff item cleared"
                    .format(currentRelAltM, planTakeoffRelAltM),
            )
        }
        return Decision.Clear(takeoff.first, takeoff.second, itemRelAltM)
    }

    /**
     * The height the [Phase.LOWER] leg flies to: **`min(`[LAND_TAG_ARM_HEIGHT_M]`, current)`**, so
     * the sequence **never climbs to reach the arm height**.
     *
     * Ivan's rule, and the case it exists for is real rather than hypothetical: a Land item authored
     * at 4 m (QGC's altitude field is the operator's, and a low number there is a common way to say
     * "come down") leaves the transit at 4 m, and a fixed 8 m target would then fly the aircraft
     * *back up* four metres — above the band, into the approach, for no reason anybody asked for.
     * With the clamp the sequence simply arms where it is, which is inside the tracking band and
     * exactly what the descent wants.
     *
     * Evaluated **once, at the moment [Phase.LOWER] begins**, rather than per tick. A per-tick `min`
     * would be a target that ratchets down with the aircraft's own noise — a gust that dips the
     * aircraft to 6 m would pin the arm height there for good — and a frozen number is the one a
     * flight record can be read against. The never-climb property is preserved either way, because
     * the frozen value is by construction no higher than the altitude the phase started at.
     *
     * A non-finite input returns [LAND_TAG_ARM_HEIGHT_M] rather than propagating a NaN into the
     * vertical law; the caller has already refused a null altitude, so this is the fail-closed answer
     * to an impossible argument.
     */
    fun armHeightTargetM(currentRelAltM: Double): Double =
        if (!currentRelAltM.isFinite()) LAND_TAG_ARM_HEIGHT_M
        else min(LAND_TAG_ARM_HEIGHT_M, currentRelAltM)

    /**
     * The height [Phase.ACQUIRE] descends to: **`min(`[LAND_TAG_ACQUIRE_FLOOR_M]`, the arm
     * height)`** — [armHeightTargetM]'s never-climb rule applied a second time, at the second end
     * of the band.
     *
     * The clamp is not decoration. A Land item authored at 4 m leaves [Phase.LOWER] arming at 4 m
     * (the first clamp), and a fixed 5 m floor would then fly the aircraft **back up** a metre to
     * go looking — a climb, in a sequence whose whole remaining job is to go down. With the clamp
     * the acquisition phase simply has nowhere to descend to, holds where it is for its own bound,
     * and either arms or refuses by name. That is the correct behaviour for an item that asked to
     * be landed from below the band: the operator gets the tries, not a climb they did not ask for.
     *
     * Frozen once, when the phase begins, for [armHeightTargetM]'s reason verbatim: a per-tick
     * `min` is a target that ratchets down with the aircraft's own noise.
     *
     * A non-finite input returns [LAND_TAG_ACQUIRE_FLOOR_M] — the fail-closed answer to an
     * impossible argument, matching [armHeightTargetM]'s.
     */
    fun acquireFloorTargetM(armHeightM: Double): Double =
        if (!armHeightM.isFinite()) LAND_TAG_ACQUIRE_FLOOR_M
        else min(LAND_TAG_ACQUIRE_FLOOR_M, armHeightM)
}
