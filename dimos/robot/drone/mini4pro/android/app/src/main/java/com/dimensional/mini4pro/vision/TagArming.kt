package com.dimensional.mini4pro.vision

/**
 * The operator's override. **AUTO is the design; the other two exist because a design is not an
 * emergency plan.**
 */
enum class TagArm {
    /**
     * Ivan's rule: acquire on takeoff, and re-arm for the descent only if something was acquired —
     * **plus** the one warrant that does not need the acquisition to have worked, a tag landing
     * this bridge has committed to ([FlightView.landingOnTag]).
     */
    AUTO,

    /**
     * Run the detector whatever the aircraft is doing — including on the ground, which is how a
     * bench experiment gets frames at all.
     */
    ON,

    /**
     * Never run it. The reason this exists is thermal: 0.68 cores on a phone clamped to a
     * controller in the sun, on an airframe whose characteristic failure is an overheat go-home.
     * An operator who can feel the phone is hot must be able to stop it without stopping the bridge.
     */
    OFF,
}

/**
 * What the arming rule is looking at. DJI-free primitives, projected by the wiring layer.
 *
 * **There is no gimbal attitude here and there never will be.** `vision/TagSighting` states the rule
 * and this file is one of the two places it would be tempting to break: DJI's `KeyGimbalAttitude` is
 * change-driven and goes silent precisely when the camera is holding still, which is the whole of a
 * nadir approach. Arming on it would disarm the detector at the exact moment the tag came into view.
 * Seven times, this project.
 */
data class FlightView(
    /** DJI's own `isFlying`. */
    val flying: Boolean,
    /** Height above the takeoff datum, metres. Null when unknown, and unknown is not zero. */
    val relativeAltitudeM: Double?,
    /** Return-to-home is running. A commanded state, not a sensed one. */
    val returning: Boolean,
    /** A landing is running. Also commanded. */
    val landing: Boolean,
    /**
     * **This bridge has committed to a tag landing and is on its way down to the pad.**
     *
     * Commanded, like [returning] and [landing], and for the same reason they are: it is a
     * statement about what *we* have decided, never about what an instrument reports. What makes
     * it a separate field rather than a widening of [landing] is that the two read different
     * things — [landing] is DJI's own `AUTO_LAND` family through `Px4Mode`, and our tag descent
     * flies on virtual stick, where DJI's mode is `JOYSTICK` from the arm to the Stage C commit.
     * So on a bridge-flown landing [landing] is false for the whole of the descent that matters.
     *
     * **The measured consequence of not having it** — landing16
     * (`datasets/landing16/20260730-161329.001.jsonl`), the first fully autonomous mission: the
     * precision-`NAV_LAND` sequence aimed the camera at nadir at t=182.0 (`dji_call
     * gimbal_rotate pitchDeg=-90`), lowered to the arm height, and asked for the descent at
     * t=188.8 — at 8.8 m, with the detector disarmed since the climb crossed the ceiling at
     * t=62.0. `tag_descent_denied NO_TAG_LATCHED`, and **zero `tag` lines in 241 seconds** of
     * record. The camera was pointing straight down at the pad and nothing was looking.
     *
     * There is no field for *where in* the landing we are, deliberately: the arming rule has no
     * use for the phase, and the one thing it must know — that this ends when the landing run
     * ends — is a property of whoever projects this flag, not of the rule that reads it.
     * `GuidedStickEngine.landingOnTag` is that owner, and it is derived rather than latched.
     */
    val landingOnTag: Boolean,
)

/**
 * The acquisition window's memory. Explicit and immutable so the whole rule is a pure step function,
 * which is what makes every row of it a one-line test.
 */
data class TagArmingState(
    /** When the current flight began, monotonic nanos. Null while on the ground. */
    val flightStartedNanos: Long? = null,
    /** True once this flight has climbed out of the band where the tag is detectable. */
    val leftAcquisitionBand: Boolean = false,
)

/** What the rule decided, and the one-line reason, which goes on the screen and in the record. */
data class TagArmingDecision(
    val state: TagArmingState,
    val armed: Boolean,
    val why: String,
    /** True on the transition into a new flight — the signal to reset the latch. */
    val newFlight: Boolean = false,
)

/**
 * **When the detector is allowed to burn CPU**, as a pure step function.
 *
 * ## Ivan's rule, and why it is evidence and not geometry
 *
 * > Run the detector during takeoff. If the tag is recognised then, latch it for the flight — and
 * > the recogniser auto-enables for the return and descent.
 *
 * The alternative — arm whenever the aircraft is near the home coordinate — is worse for a reason
 * that is easy to miss: it spends the CPU on the strength of somebody's belief about where the pad
 * is. This spends it on the strength of the camera having seen one. A site with no pad never pays,
 * and a pad that has moved is still found.
 *
 * ## The three windows
 *
 * 1. **Acquisition.** From the moment the aircraft is flying, unconditionally, until either the
 *    climb takes it out of the band where a tag is detectable at all or [ACQUIRE_WINDOW_NANOS]
 *    expires. This is the one chance to see the tag from close range with good framing — the
 *    aircraft passes through 1–2 m, where detection was 100 % on both measured flights.
 * 2. **Cruise.** Not armed. Nothing above the band is detectable (0 % in all 60 frames of the
 *    10–11 m band), so this is CPU that could not buy a detection even in principle.
 * 3. **Approach.** Armed when a tag landing has been *commanded* ([FlightView.landingOnTag]) —
 *    with or without a latch — and otherwise only if something was latched, and then only when
 *    descending back into the band or when DJI's own return or landing is running. All three
 *    commanded cases arm it *regardless of altitude*, deliberately: the detector should already be
 *    looking by the time the aircraft is low enough to see anything, not start looking when it
 *    gets there.
 *
 * ## Why a commanded tag landing outranks the latch
 *
 * The latch rule spends CPU on the strength of the camera having seen a pad. A commanded tag
 * landing is a stronger warrant than that: *this bridge has decided to put the aircraft on a
 * marker* and is already flying toward it. Refusing to look then is refusing to look at exactly
 * the moment the whole feature exists for — which is what landing16 measured
 * (`datasets/landing16/20260730-161329.001.jsonl`, 2026-07-30: mission takeoff with the camera
 * never commanded to nadir, so the 1–2 m acquisition pass saw nothing, no latch, the one-way band
 * exit at t=62.0, and a `NO_TAG_LATCHED` refusal at t=188.8 over a pad the camera was pointing
 * straight at). Both halves of that flight are fixed: the mission takeoff now aims the camera like
 * every other takeoff door, *and* the landing can arm the detector without the takeoff's evidence.
 * The two are deliberately independent — a fix that needed both to work would have neither's
 * failure named.
 *
 * The CPU rule survives whole for ordinary flight: with no landing commanded, no latch is still no
 * detector.
 *
 * ## Why the ceiling is the measured decode *reach* and not the reliable band
 *
 * The question this constant answers is *"could this CPU buy a detection at all?"* — reach, not
 * rate. The old 8.0 m answered the other one (one metre of margin over the 7.0 m cliff where the
 * per-frame *rate* collapses) and that mismatch had a measured cost: landing16's arm fired at
 * 8.8 m, inside the precision-land sequence's own ±1 m arrival window, where the detector was off
 * by construction. See [ACQUIRE_CEILING_M].
 *
 * **This is a rule about spending CPU, not a rule about landing.** It says where it is worth
 * looking. Whether a descent may *arm on* what it sees is a different decision, it belongs to
 * whoever writes that law, and nothing here grants it.
 */
object TagArming {

    /**
     * **10.0 m — the measured decode *reach*, which is the top of the band where spending CPU can
     * buy a detection at all.** Was 8.0 m until 2026-07-30; the raise, its evidence, and the flight
     * that forced it are written up in `docs/measurements/2026-07-30-landing16-arm-height.md`:
     *
     *  - **The reach is 10 m and the rate collapses at 7.** The profiling descent holds 92.6 %
     *    per-frame at 7–8 m, **2.7 % at 8–9 m, 1.3 % at 9–10 m and 0 % in all 60 frames of the
     *    10–11 m band** (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1). landing06's
     *    record carries a believed fix at 8.4 m and landing13's decodes reach 8.1 m. So decodes
     *    exist — sporadically — to the top of the 9–10 m band, and **nothing has ever decoded
     *    above it**. This is the identical sentence
     *    [com.dimensional.mini4pro.guided.TagDescentGuidance.APPROACH_CEILING_M] is built on, and
     *    that is the point: the two constants now rest on **one** measured number rather than two
     *    unrelated ones (see "the two ceilings" below).
     *  - **The old 8.0 m answered the wrong question, measurably.** It was the 100 %-detection
     *    cliff (7.0 m) plus a metre — a statement about where the detector is *reliable*, which is
     *    [com.dimensional.mini4pro.guided.TagDescentGuidance.ARM_CEILING_M]'s question, not this
     *    one's. The cost was on the record: `PrecisionLand.LAND_TAG_ARM_HEIGHT_M` is 8.0 m and the
     *    arrival tolerance is [com.dimensional.mini4pro.guided.RepositionGuidance.VERTICAL_ACCEPT_M]
     *    = 1 m, so a precision landing arms anywhere in **7–9 m** — and landing16 armed at **8.8 m**
     *    (`datasets/landing16/20260730-161329.001.jsonl`, t=188.8), above a ceiling that had just
     *    turned the detector off. An arm gate whose fresh-fix conjunct is evaluated *before* any
     *    landing exists cannot be served by a detector ceiling below the heights arms are taken at.
     *  - **What the extra 2 m costs.** The detector is 0.68 cores over the floor against 0.06
     *    disarmed (measured on the aircraft, 2026-07-28). This spends it across two more metres of
     *    descent on a latched flight, and — the honest case — for the whole of a latched flight
     *    that loiters between 8 and 10 m. [ACQUIRE_WINDOW_NANOS]'s two-minute bound does not cover
     *    that case, so it is named rather than hidden; the operator's OFF override is the answer,
     *    and it is why that override exists.
     *
     * ## The two ceilings, and why they no longer contradict each other
     *
     * [com.dimensional.mini4pro.guided.TagDescentGuidance.APPROACH_CEILING_M] is 12.0 m: no descent
     * may be armed above it, even into the approach segment. It is **this number plus ~2 m of
     * measured within-session barometric wander** (landing07, the
     * `2026-07-27-altitude-datum-wander.md` family), because a true 10 m hover may *read* 11 m and
     * must still be allowed to arm. So the 2 m gap between the two constants is the *barometer's*
     * error bar and nothing else — not a disagreement about the camera.
     *
     * **What an approach armed between 10 and 12 m can and cannot expect, stated plainly:** it will
     * get no fix stream from this rule until it descends to 10 m, because under [TagArm.AUTO] the
     * detector is not running there. It can be armed at all only because something else armed the
     * detector — the operator's [TagArm.ON], an acquisition window still open, or a commanded tag
     * landing ([FlightView.landingOnTag]) — and above ~10 m no measured decode exists to arm it
     * with in any case. That is a documented gap, not an accident: the alternative is spending
     * 0.68 cores where 60 measured frames produced nothing.
     */
    const val ACQUIRE_CEILING_M = 10.0

    /**
     * **Two minutes.** How long the unconditional acquisition window stays open after the aircraft
     * starts flying, if the climb never takes it out of the band.
     *
     * It exists for the flight that hovers low for a long time — a bench run, a slow survey — where
     * "still taking off" stops being true long before any altitude threshold says so. Two minutes is
     * about a quarter of this airframe's battery, so it cannot quietly become "always on".
     */
    const val ACQUIRE_WINDOW_NANOS = 120_000_000_000L

    /**
     * One step. Returns the next state and whether the detector may run.
     *
     * @param latched whether [TagLatch] holds evidence for this flight.
     */
    fun step(
        mode: TagArm,
        state: TagArmingState,
        view: FlightView,
        latched: Boolean,
        nowNanos: Long,
    ): TagArmingDecision {
        // A new flight before anything else, so the latch is reset even when the mode is OFF —
        // otherwise switching the detector on mid-flight would inherit the previous flight's fix,
        // which is a fix for a different takeoff point.
        val takingOff = view.flying && state.flightStartedNanos == null
        var next = when {
            takingOff -> TagArmingState(flightStartedNanos = nowNanos, leftAcquisitionBand = false)
            !view.flying -> TagArmingState()
            else -> state
        }
        val alt = view.relativeAltitudeM
        if (view.flying && alt != null && alt > ACQUIRE_CEILING_M && !next.leftAcquisitionBand) {
            next = next.copy(leftAcquisitionBand = true)
        }

        // The overrides are read after the state has been stepped, never instead of it: an operator
        // holding the detector off must not also be freezing the acquisition window, or turning it
        // back on mid-flight would resume a window that expired minutes ago.
        return when (mode) {
            TagArm.OFF -> TagArmingDecision(next, false, "off (operator)", takingOff)
            TagArm.ON -> TagArmingDecision(next, true, "on (operator)", takingOff)
            TagArm.AUTO -> {
                val armed: Boolean
                val why: String
                val started = next.flightStartedNanos
                val windowOpen = view.flying &&
                    started != null &&
                    !next.leftAcquisitionBand &&
                    nowNanos - started <= ACQUIRE_WINDOW_NANOS
                val inBand = alt != null && alt <= ACQUIRE_CEILING_M
                when {
                    !view.flying -> { armed = false; why = "on the ground" }
                    windowOpen -> { armed = true; why = "acquiring on takeoff" }
                    // **Above the latch, and above the closed acquisition band.** The only rung
                    // that arms on no evidence at all once the takeoff window has gone, because it
                    // is the only one whose warrant is a decision of ours rather than a reading:
                    // a tag landing is *being flown*, toward a marker, and the camera is already
                    // pointing at it. landing16 is what this rung costs when it is missing.
                    // It stays under `!view.flying`: on the ground there is nothing to land onto,
                    // and a run that outlived its flight would be a bug in its owner, not a reason
                    // to burn CPU here.
                    view.landingOnTag -> { armed = true; why = "landing on the tag" }
                    !latched -> { armed = false; why = "no tag seen this flight" }
                    view.returning -> { armed = true; why = "returning, tag latched" }
                    view.landing -> { armed = true; why = "landing, tag latched" }
                    inBand -> { armed = true; why = "below ${ACQUIRE_CEILING_M.toInt()} m, tag latched" }
                    // Altitude unknown while latched: not armed, and it is a deliberate refusal
                    // rather than an oversight. "Unknown" is not "low" — this project's standing
                    // rule — and a detector that armed whenever telemetry went quiet would run for
                    // the whole of any flight with a bad altitude feed.
                    else -> { armed = false; why = "above the band" }
                }
                TagArmingDecision(next, armed, why, takingOff)
            }
        }
    }
}
