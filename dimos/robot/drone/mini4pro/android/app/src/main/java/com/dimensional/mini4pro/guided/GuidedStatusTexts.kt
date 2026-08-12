package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.command.StatusTexts

/**
 * Every operator-facing sentence Stage A can produce, counted against `STATUSTEXT`'s hard
 * 50-byte field and clamped by the same [StatusTexts.clamp] the rest of the project uses.
 *
 * M3 Q5's closed answer is why these exist at all: the heartbeat **never** claims a mode for
 * guided control — it keeps reporting whatever DJI reports — so the only sign an operator gets
 * that this bridge holds (or lost) stick authority is these sentences. Engagement is announced;
 * every disengagement is announced **with the reason on the way out** (released, sticks,
 * interlock, authority, link-lost, idle — the Q5 list, plus the Q1 envelope's timeout and the
 * bridge's own stop).
 *
 * The DJI-word rule holds here as in `command/StatusTexts`: where a sentence carries DJI's
 * refusal or DJI's authority-change reason, the name goes verbatim, and if the framing does not
 * fit the frame is dropped rather than the name.
 */
object GuidedStatusTexts {

    /** DJI has confirmed enabled + advanced + MSDK authority; setpoints are flowing. 39 bytes. */
    const val ENGAGED = "Virtual stick engaged - GCS sticks live"

    /** Input resumed during a link-loss/released wind-down, before release ran. 39 bytes. */
    const val RESUMED = "Virtual stick resumed - GCS sticks live"

    /**
     * "Virtual stick off: <reason>" — the Q5 disengagement announcement. [reason] is one of
     * [GuidedStickEngine.DisengageReason]'s wire words, plus DJI's own name when there is one
     * (authority NEAR_BOUNDARY and family). Longest fixed form is
     * `"Virtual stick off: link-lost"`, 28 bytes; a DJI reason may push past 50 and is clamped
     * with the prefix intact — the reason category survives, the tail of DJI's word is what
     * gets cut, and DJI's longest reason name today (`BATTERY_SUPER_LOW_LANDING`) still fits
     * whole at 45 bytes.
     */
    fun off(reason: String, detail: String? = null): String =
        StatusTexts.clamp(if (detail == null) "Virtual stick off: $reason" else "Virtual stick off: $reason $detail")

    /**
     * A wind-down has begun (ramp → hold → release). Announced at its start as well as at the
     * final [off], because the shipped sequence takes ~1.5 s and the FreezeAndHold alternative
     * never reaches [off] at all — a policy that held position silently would be the display
     * failing to say what the aircraft is doing. `"Virtual stick stopping: link-lost"` is
     * 32 bytes.
     */
    fun stopping(reason: String): String = StatusTexts.clamp("Virtual stick stopping: $reason")

    /**
     * Engagement was attempted and did not complete — DJI refused the enable (name verbatim),
     * or confirmed nothing inside the window ([NO_CONFIRM]). The engagement never latched, so
     * there is nothing to hand back; this is the whole of what happened.
     */
    fun refused(reason: String): String = preferring("Virtual stick refused: $reason", reason)

    /**
     * The engage window expired with DJI confirming nothing. Deliberately its own word rather
     * than silence: the measured swallowed-`performAction` case
     * (`docs/measurements/2026-07-26-m2-first-command.md`) means "no callback" is a real
     * outcome, and an operator whose sticks do nothing is owed the reason.
     */
    const val NO_CONFIRM = "NO_CONFIRM"

    /** Deliberate deflection with the interlock off. The switch is on the phone's screen. 29 bytes. */
    const val IGNORED_INTERLOCK = "Sticks ignored: interlock off"

    /**
     * Deliberate deflection from a stream never (recently) seen at rest — which includes every
     * stream whose resting throttle is not at the 500 centre, i.e. QGC's opt-in centre-zero
     * regime and any −1000..1000-convention sender. The instruction is the fix: centring the
     * sticks (throttle included) is exactly the observation the gate needs. 40 bytes.
     */
    const val CENTER_FIRST = "Sticks ignored: center sticks to engage"

    /**
     * Deliberate deflection while the RC stick feed has not delivered — abort gesture 1 would
     * be blind, so engagement is refused rather than flown without it. 31 bytes.
     */
    const val NO_RC_FEED = "Sticks ignored: no RC stick feed"

    /** A frame this bridge refuses to interpret — see [StickMapping.read]. 30 bytes. */
    const val BAD_AXES = "Sticks refused: unreadable axes"

    /** Deliberate deflection while the SDK/product cannot be asked ("NO_PRODUCT" etc.). */
    fun ignored(reason: String): String = preferring("Sticks ignored: $reason", reason)

    /**
     * At or above the Q1 ceiling with a climb commanded: the climb component is zeroed and the
     * operator is told; lateral and descent pass through unchanged. Announced rather than
     * silently clamped — the one thing Q1 forbids is the operator believing a command was
     * obeyed — and announcing is the only channel there is, since `MANUAL_CONTROL` carries no
     * ack to refuse through. 27 bytes.
     */
    val CEILING = "Ceiling ${GuidedEnvelope.CEILING_M.toInt()}m - climb blocked"

    /**
     * A climb was commanded and the bridge has no fresh height above its takeoff datum, so the
     * ceiling cannot be enforced; the climb is blocked, everything else passes. The Q1 ceiling
     * is the only altitude bound guided authority has, and flying up without the number it is
     * defined against would make it unenforceable in exactly the direction it exists for.
     * 27 bytes.
     */
    const val NO_ALTITUDE = "No altitude - climb blocked"

    // ── Heading follows course ───────────────────────────────────────────────

    /**
     * **The substitution, announced once per manoeuvre.** 22 bytes.
     *
     * QGC sends `DO_REPOSITION.param4 = NaN`, which MAVLink defines as *"unchanged or vehicle
     * default"*. Reading it as *our* default — face the way you are going — is legitimate, but the
     * bridge read it as "keep heading" until 2026-07-27 and **that behaviour is flight-verified**.
     * Changing what a flight-verified `NaN` means is a substitution, and this project announces
     * substitutions: nothing exceeds the envelope, and the operator is never left believing
     * something else was obeyed. A *finite* `param4` stays refused — a commanded absolute heading is
     * a different feature.
     *
     * `docs/decisions/2026-07-27-heading-follows-course.md`.
     */
    const val HEADING_FOLLOWS = "Heading follows course"

    /**
     * The heading feed went stale while the nose was being turned toward a target: **zero yaw rate,
     * never a guess**, and the translation continues. 27 bytes.
     *
     * The same graduated treatment [ROI_NO_HEADING] gives a tracked target, said in its own words
     * because the operator's question differs — an orbit that stops yawing is still circling, while
     * a leg that stops yawing simply flies sideways again, which is the behaviour they just asked to
     * be rid of.
     */
    const val HEADING_NO_HEADING = "No heading - nose not turning"

    // ── Stage B: commanded reposition ────────────────────────────────────────
    //
    // Q5's closed answer applies with extra force here: the heartbeat never claims a mode, so
    // these sentences are the *only* sign the operator gets that a manoeuvre began, arrived,
    // or ended — plus the COMMAND_ACK, which is the honest channel for every refusal.

    /** A `DO_REPOSITION` was validated and the target taken; the controller is (or is becoming) engaged. 29 bytes. */
    const val GOTO_STARTED = "Goto accepted - repositioning"

    /** DJI confirmed authority for an engagement begun by a reposition, not by sticks. 38 bytes. */
    const val ENGAGED_GOTO = "Virtual stick engaged - repositioning"

    /** Both arrival conjuncts held for the required consecutive ticks; keeping station, still engaged. 31 bytes. */
    const val GOTO_ARRIVED = "Goto arrived - holding position"

    /**
     * The commanded altitude was above the Q1 ceiling and was capped to it — capped, not
     * refused, and *announced*, the JC-5 reading of Q1's "refused, never silently clamped":
     * the lateral half of the operator's intent is still honoured and the operator is told
     * exactly what was not. 27 bytes.
     */
    val GOTO_CAPPED = "Goto altitude capped at ${GuidedEnvelope.CEILING_M.toInt()}m"

    /** QGC's Pause: the target is dropped and the aircraft keeps station where it is. 30 bytes. */
    const val GOTO_PAUSED = "Goto paused - holding position"

    /** A deliberate GCS stick deflection interrupted the reposition; passthrough has it now. 27 bytes. */
    const val GOTO_STICKS = "Goto cancelled - GCS sticks"

    /**
     * The position feed went stale mid-reposition: zero velocity is commanded and the loop
     * stops closing — a cached fix is never flown on (landmine: a ~12 Hz feed that the bench
     * proved flaps). 25 bytes.
     */
    const val NO_POSITION_HOLD = "No position fix - holding"

    /** `"Goto refused: <reason>"` — rides the `COMMAND_ACK`'s `DENIED`, which QGC shows as a modal. */
    fun gotoRefused(reason: String): String = preferring("Goto refused: $reason", reason)

    // ── M4: the mission ──────────────────────────────────────────────────────
    //
    // Deliberately few. A five-waypoint mission must not produce five announcements: leg progress
    // lives in `MISSION_CURRENT`, which is what QGC actually draws, and this 50-byte channel at
    // severity ERROR exists for the things an operator must **act** on. Start, finish, every pause
    // and every refusal are announced; arrivals are not.

    /** The launch check passed and the route was taken. 25 bytes. */
    const val MISSION_STARTED = "Mission started - flying plan"

    /**
     * A resume flies a **resting** leg back to the cursor, and the operator should know that the
     * first thing the aircraft does is not a leg they drew. 31 bytes. §6.3.
     */
    const val MISSION_REJOINING = "Rejoining the plan at the cursor"

    /** Phase one of a `NAV_TAKEOFF` item: DJI's own takeoff, which stops at its own ~1.2 m. 30 bytes. */
    const val MISSION_TAKEOFF = "Mission takeoff - DJI's own hop"

    /**
     * **The most important sentence in M4.** 37 bytes, and its wording is the decision doc's own.
     *
     * `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-5 half-reversed the ground-to-ground
     * answer — *"just hover for now, I'll land manually"* — so a mission ends with the aircraft **in
     * the air**, holding station, holding authority, waiting for a human on the sticks. An operator
     * who reads "Mission complete" and walks away from a hovering aircraft is the failure this
     * sentence exists to prevent, which is why it says what to do rather than what happened.
     */
    const val MISSION_DONE_HOLDING = "Mission done - holding, land manually"

    /** QGC's Pause, applied to a plan: the aircraft keeps station and the cursor is kept. 33 bytes. */
    const val MISSION_PAUSED = "Mission paused - holding position"

    /** A deliberate GCS stick deflection took the aircraft; the plan waits at its cursor. 30 bytes. */
    const val MISSION_STICKS = "Mission paused - GCS sticks"

    // ── A plan's precision NAV_LAND: the tag landing ─────────────────────────
    //
    // Four sentences and eight reason words, on the M4 principle above: this sequence is the one part
    // of a mission that ends with the aircraft on the ground, so each of its three phases says its
    // word (the operator must know which of "flying there", "pointing the camera" and "coming down"
    // they are watching), the hand-off to the descent says its word, and every way it can refuse
    // names itself. Everything after the arm speaks in the descent's own sentences, which are the
    // DESCENT_* family below and are deliberately not duplicated here.

    /**
     * The gates passed and the leg to the recorded takeoff point is flying — at the item's own
     * altitude, which in big1.plan means 60 m → 15 m. 31 bytes.
     */
    const val LAND_TAG_TRANSIT = "Tag landing - flying to the pad"

    /**
     * At the item's altitude, camera commanded to nadir, waiting for the belief
     * ([PrecisionLand.NADIR_AIM_LIMIT_MS]). Said because a hovering aircraft that has stopped
     * translating and is not yet descending otherwise looks like a stall. 29 bytes.
     */
    const val LAND_TAG_AIMING = "Tag landing - camera to nadir"

    /** Lowering to [PrecisionLand.LAND_TAG_ARM_HEIGHT_M] above the pad, never climbing. 32 bytes. */
    const val LAND_TAG_LOWERING = "Tag landing - down to arm height"

    /**
     * The acquisition descent ([PrecisionLand.Phase.ACQUIRE]): still coming down, toward
     * [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M], asking the arm gate every tick. Said because an
     * aircraft that reached the arm height and is *still* descending would otherwise read as a
     * sequence that had lost its target. 33 bytes.
     */
    const val LAND_TAG_ACQUIRING = "Tag landing - looking for the tag"

    /**
     * [PrecisionLand.Phase.HOLD]: the floor is reached, the descent has stopped, and the arm gate
     * is still being asked for [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS] before anything is
     * refused. Said because it is the one phase in which an aircraft that looks *stuck* is doing
     * exactly what it was built to do — landing17 sat at this height for 26 s with the tag in
     * view and nothing on the screen said so, because the sequence had already given up. The
     * sentence deliberately says **holding**, not "arrived": arriving is not what this phase is
     * waiting for. 36 bytes.
     */
    const val LAND_TAG_HOLDING = "Tag landing - holding, still looking"

    /**
     * The mission's last item is done and the tag descent has been armed for full autoland: from
     * here the aircraft is flying the descent's own ladder and every further sentence is one of the
     * `DESCENT_*` family. 32 bytes.
     */
    const val LAND_TAG_ARMED = "Tag landing - descent has it now"

    /**
     * `"Tag land refused: <reason>"` — every way the sequence declines, including a refused arm.
     * **The aircraft is holding where it is when this is said**; there is no fallback landing on any
     * path, which is why the sentence names the refusal rather than announcing an alternative.
     */
    fun tagLandRefused(reason: String): String = preferring("Tag land refused: $reason", reason)

    /** No recorded takeoff point (DJI's home), so there is no pad to fly to and no frame to fix in. */
    const val REASON_LAND_NO_TAKEOFF_POINT = "no takeoff point"

    /** The land item names no height to fly to. Refused at Start; a fail-closed branch here. */
    const val REASON_LAND_NO_HEIGHT = "land item has no height"

    /** The plan has no `NAV_TAKEOFF`, so there is no cleared height the low gate could compare to. */
    const val REASON_LAND_NO_PLAN_TAKEOFF = "no takeoff in plan"

    /** The land item is drawn further than [PrecisionLand.LAND_TAG_RADIUS_M] from where we took off. */
    val REASON_LAND_SITE = "plan drawn ${PrecisionLand.LAND_TAG_RADIUS_M.toInt()}m+ from pad"

    /** The aircraft is further than [PrecisionLand.LAND_TAG_RADIUS_M] from where we took off. */
    val REASON_LAND_TOO_FAR = "over ${PrecisionLand.LAND_TAG_RADIUS_M.toInt()}m from pad"

    /** Below the height the plan's own takeoff item cleared — the sequence must not start from there. */
    const val REASON_LAND_TOO_LOW = "below plan takeoff height"

    /**
     * A phase of the sequence did not finish inside its own bound — the leg timeout for the two flying
     * phases, [PrecisionLand.NADIR_AIM_LIMIT_MS] for the camera. **The aiming case is the important
     * one**: it means the *believed* camera pitch never reached nadir, so the descent would have been
     * refused for the camera anyway and would have flown on trigonometry nobody can vouch for.
     */
    const val REASON_LAND_PHASE_TIMEOUT = "landing phase timed out"

    /**
     * The tag descent's own arm door refused, at the arm height, over the pad. Deliberately a *second*
     * sentence rather than a paraphrase of the first: the descent has already announced which of its
     * gates said no (`Tag descent refused: <gate>`), and this one says what became of the **mission** —
     * it is over, and the aircraft is holding.
     */
    const val REASON_LAND_ARM = "arm refused"

    /**
     * The acquisition looked all the way down its band to [PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M]
     * **and then held there for [PrecisionLand.LAND_TAG_ACQUIRE_HOLD_MS]**, and the arm gate never
     * cleared — the tries are genuinely exhausted, which is what makes this an honest refusal
     * rather than landing17's premature one. Distinct from [REASON_LAND_ARM], which is a *single*
     * refused arm at the end of the old one-shot design. The detail sentence carries the wait it
     * spent and the gate's own last words, so the record says which conjunct was still failing at
     * the floor and for how long it kept saying so.
     */
    val REASON_LAND_NO_ACQUIRE = "no tag by ${PrecisionLand.LAND_TAG_ACQUIRE_FLOOR_M.toInt()}m"

    /** `"Mission refused: <reason>"` — every launch-check failure, naming the first one only. */
    fun missionRefused(reason: String): String = preferring("Mission refused: $reason", reason)

    /** `"Resume refused: <reason>"` — a Start that failed while paused, including the rejoin gate. */
    fun resumeRefused(reason: String): String = preferring("Resume refused: $reason", reason)

    /** No AMSL datum published this link — an inbound AMSL is uninterpretable (landmine 1). */
    const val REASON_NO_DATUM = "NO_ALT_DATUM"

    /** No usable, fresh position fix to define the target against or fly from. */
    const val REASON_NO_FIX = "NO_POSITION_FIX"

    /** Target farther than the Q1 cap. Names the limit, per Q1's rule. */
    val REASON_TOO_FAR = "beyond ${GuidedEnvelope.MAX_REPOSITION_DISTANCE_M.toInt()}m limit"

    /** Target coordinates that are not a coordinate this bridge will fly toward. */
    const val REASON_BAD_TARGET = "BAD_TARGET"

    /** A commanded relative altitude below the takeoff datum — refused, see the engine's KDoc. */
    const val REASON_BELOW_DATUM = "target below takeoff"

    /** `param4` finite — a commanded yaw. Stage B never yaws (the measured Go-to sends NaN). */
    const val REASON_YAW = "YAW_UNSUPPORTED"

    /** `param1` a positive speed request — the Q1 envelope is not negotiable from MAVLink. */
    const val REASON_SPEED = "SPEED_UNSUPPORTED"

    /** A Pause with no manoeuvre to pause. */
    const val REASON_NOTHING_TO_PAUSE = "nothing to pause"

    /** A `COMMAND_LONG` 192 carrying coordinates — an unmeasured shape, refused not guessed at. */
    const val REASON_LONG_FORM = "COMMAND_LONG goto"

    // ── The takeoff's second phase ───────────────────────────────────────────
    //
    // A takeoff is two phases (`docs/m4-mission-execution.md` §3.6): DJI's own ~1.2 m hop, then a
    // commanded vertical-only climb to the height the operator asked for. The first phase is
    // announced by `command/StatusTexts`; these are the three things that can happen to the
    // second, and all three are said, because between the phases the aircraft is in the air with
    // an intention pending and the operator is the only one who can act on any of it.

    /**
     * DJI reported its takeoff finished and the climb is being started — the seam between the two
     * phases, said at the instant authority changes hands. Without it the operator sees a
     * `Goto accepted` for a goto they never commanded. 36 bytes.
     */
    const val TAKEOFF_CLIMB_ENGAGING = "Takeoff done - climbing to set height"

    /**
     * [TakeoffClimb.WAIT_LIMIT_MS] elapsed with the aircraft never reporting a finished takeoff,
     * so the pending climb was dropped. The measured case it exists for: DJI answering a
     * `performAction` on a channel we cannot hear — an RC beep, neither callback, nothing on any
     * key (`docs/measurements/2026-07-26-m2-first-command.md`). Silence would leave an operator
     * waiting for a climb that is never coming. 33 bytes.
     */
    const val TAKEOFF_CLIMB_EXPIRED = "Takeoff climb dropped - never flew"

    /**
     * An abort rung fired while a climb was armed but not yet flying, so it was cancelled —
     * `"Takeoff climb cancelled: sticks"`, 30 bytes for the longest reason word.
     *
     * Said for the same reason every disengagement is said: the pending climb is a thing this
     * bridge was going to do to the aircraft, and an operator who grabbed the sticks between the
     * phases must be able to read that it will not happen rather than infer it from nothing
     * happening.
     */
    fun takeoffClimbCancelled(reason: String): String =
        preferring("Takeoff climb cancelled: $reason", reason)

    // ── Stage C: orbit ───────────────────────────────────────────────────────
    //
    // The `DO_ORBIT` refusal measured on 2026-07-27 was a bare `MAV_RESULT_UNSUPPORTED` with **no
    // STATUSTEXT** — the only refusal in this project that never told the operator why, so the
    // operator saw QGC's generic dialog and nothing else. Every sentence below exists so that
    // cannot happen again for this command: each refusal pairs its result with a reason, and each
    // *substitution* says which part of the request was not obeyed.

    /** A `DO_ORBIT` was validated and the circle taken; the join leg is flying. 30 bytes. */
    const val ORBIT_STARTED = "Orbit accepted - joining circle"

    /** DJI confirmed authority for an engagement begun by an orbit, not by sticks or a goto. 32 bytes. */
    const val ENGAGED_ORBIT = "Virtual stick engaged - orbiting"

    /** The join leg arrived and the tangential ramp has begun — the circle proper. 15 bytes. */
    const val ORBIT_CIRCLING = "Orbit: circling"

    /**
     * **The `param3` substitution, announced.** QGC's Orbit sends
     * `ORBIT_YAW_BEHAVIOUR_UNCHANGED` (measured: 5) and we deliberately fly nose-to-centre
     * instead, because on this airframe the gimbal cannot yaw at all and pointing the camera at
     * the centre is achievable only by turning the aircraft. Nothing exceeds the envelope and the
     * operator is never left believing something else was obeyed. 26 bytes.
     */
    const val ORBIT_NOSE_TO_CENTRE = "Orbit: nose held to centre"

    /** The swept angle reached the requested turns; coming to rest at the completion point. 24 bytes. */
    const val ORBIT_COMPLETE = "Orbit complete - holding"

    /** [OrbitGuidance.ORBIT_MAX_S] elapsed; coming to rest where the circle stopped. */
    val ORBIT_TIME_LIMIT = "Orbit ${OrbitGuidance.ORBIT_MAX_S}s limit - holding"

    /** QGC's Pause during an orbit: the circle is dropped and the aircraft keeps station. 31 bytes. */
    const val ORBIT_PAUSED = "Orbit paused - holding position"

    /** A deliberate GCS stick deflection interrupted the orbit; passthrough has it now. 29 bytes. */
    const val ORBIT_STICKS = "Orbit cancelled - GCS sticks"

    /**
     * `param4 = 0` — MAVLink's "orbit forever". Nothing in this bridge runs forever, so the only
     * bound left is [OrbitGuidance.ORBIT_MAX_S] and the operator is told which one they got.
     */
    val ORBIT_UNBOUNDED = "Orbit: no turn limit - ${OrbitGuidance.ORBIT_MAX_S}s cap"

    /** More turns were asked for than fit inside the time cap; the sweep was truncated. */
    val ORBIT_TURNS_CAPPED = "Orbit: turns cut to the ${OrbitGuidance.ORBIT_MAX_S}s cap"

    /** The commanded altitude was above the Q1 ceiling and was capped to it — as a goto's is. */
    val ORBIT_CAPPED = "Orbit altitude capped at ${GuidedEnvelope.CEILING_M.toInt()}m"

    // **Two sentences retired on 2026-07-27**, when the orbit stopped having a pointing system of
    // its own: `ORBIT_NO_HEADING` and `ORBIT_GIMBAL_RANGE` said, in the orbit's words, exactly what
    // [ROI_NO_HEADING] and [ROI_GIMBAL_RANGE] say in the ROI's. A circle now points at its own
    // centre by *implying an ROI there* (M4-6), so one target has one vocabulary and the operator
    // does not have to learn that two sentences mean one thing. Removed rather than left dead: an
    // unused operator-facing string is a sentence nobody can ever be shown, and keeping it invites
    // the next reader to wire it back up.

    /** `"Orbit refused: <reason>"` — rides the `COMMAND_ACK`'s `DENIED`, which QGC shows as a modal. */
    fun orbitRefused(reason: String): String = preferring("Orbit refused: $reason", reason)

    /**
     * A radius outside [OrbitGuidance.R_MIN_M]..[OrbitGuidance.R_MAX_M]. **Refused, not clamped**,
     * and this sentence is why the distinction is worth a constant: a clamped circle is a
     * *different* circle, drawn somewhere the operator did not click.
     */
    val REASON_ORBIT_RADIUS =
        "radius ${OrbitGuidance.R_MIN_M.toInt()}-${OrbitGuidance.R_MAX_M.toInt()}m only"

    /** The circle would leave the Q1 leg bound at its far side. */
    val REASON_ORBIT_TOO_FAR = "circle beyond ${GuidedEnvelope.MAX_REPOSITION_DISTANCE_M.toInt()}m"

    /** A `COMMAND_LONG` 34 — an unmeasured shape (QGC sends `COMMAND_INT`), refused not guessed at. */
    const val REASON_ORBIT_LONG_FORM = "COMMAND_LONG orbit"

    // ── M3 Stage D: the tag-tracked descent ──────────────────────────────────
    //
    // Q5's closed answer applies here with the most force of any manoeuvre yet: the heartbeat
    // never claims a mode, the arm surface is the phone rather than QGC, and the aircraft is
    // *descending*. These sentences are the only running commentary the operator gets, so every
    // engagement, every rung of the staleness ladder, the ending and every cancel says its word.

    /** The arm gates all passed and the descent was taken; centring begins. 28 bytes. */
    const val DESCENT_ARMED = "Tag descent armed - centring"

    /**
     * The arm was taken **above the band** and the machine is flying the approach: descending
     * toward the 7 m band on baro height, centring on the sparse fixes as they come. Said
     * beside the armed sentence (the RESUMED pattern) so an operator arming from the 10 m
     * takeoff hover hears that the height was accepted and absorbed, not refused — the exact
     * press that used to answer `above 7m tag band` (landing13 t=41.8). 28 bytes.
     */
    const val DESCENT_APPROACH = "Above band - approaching tag"

    /**
     * The approach reached [TagDescentGuidance.APPROACH_BAND_ENTRY_M] with a fresh fix and
     * handed off into the ordinary tracking ladder — said in its own words because
     * [DESCENT_TRACKING] claims a *re*acquisition and the band entry is not one. (An entry on
     * a stale fix announces the rung it lands on instead — holding/climbing — which is the
     * honest sentence for that tick.) 26 bytes.
     */
    const val DESCENT_BAND_ENTRY = "In band - tracking descent"

    /** DJI confirmed authority for an engagement begun by a descent arm. 35 bytes. */
    const val ENGAGED_DESCENT = "Virtual stick engaged - tag descent"

    /** The fix aged past [TagDescentGuidance.T_HOLD_MS]: descent stops, centring continues. */
    const val DESCENT_HOLDING = "Tag stale - descent holding"

    /** The fix aged past [TagDescentGuidance.T_CLIMB_MS]: climbing to reacquire. 32 bytes. */
    const val DESCENT_CLIMBING = "Tag lost - climbing to reacquire"

    /** The tag came back after a hold or a climb; descending again. 25 bytes. */
    const val DESCENT_TRACKING = "Tag reacquired - tracking"

    /**
     * **Plain Stage B's ending, verbatim from the design brief.** At the target height,
     * centred, held for the required consecutive ticks; the aircraft holds over the tag until
     * the operator takes over. With full autoland armed the hold is instead the launch point
     * of [DESCENT_LANDING], which supersedes this sentence on the commit. 25 bytes.
     */
    const val DESCENT_COMPLETE = "Stage B complete, holding"

    /** The operator disarmed from the phone: descent dead, keeping station. 25 bytes. */
    const val DESCENT_DISARMED = "Tag descent off - holding"

    /** A deliberate GCS stick deflection cancelled the descent; passthrough has it now. 34 bytes. */
    const val DESCENT_STICKS = "Tag descent cancelled - GCS sticks"

    /** The latch died mid-descent (a new flight began, or the detector was stopped). 32 bytes. */
    const val DESCENT_LATCH_LOST = "Tag descent off - tag latch lost"

    /**
     * The **commanded** camera pitch left the nadir tolerance mid-descent — refusal-grade
     * inconsistency: whatever moved the camera, the fixes this descent flies on are about to
     * stop arriving (`TagWorld.fix` refuses beyond the same tolerance), so the descent ends
     * before it can fly blind. 35 bytes.
     */
    const val DESCENT_GIMBAL = "Tag descent off - camera left nadir"

    /** `"Tag descent refused: <reason>"` — the arm gate's named refusals. */
    fun descentRefused(reason: String): String = preferring("Tag descent refused: $reason", reason)

    // ── Stage C: full autoland ───────────────────────────────────────────────
    //
    // The same Q5 discipline, at the lowest heights this project commands. The landing's
    // running commentary is three sentences: the arm said it would land, the commit says it is
    // landing, and the ending says the wheels are down — plus rule 1's own cancel words, which
    // are the engine's existing ones and deliberately not duplicated here.

    /** The arm took a **full autoland** — this descent will not stop at the hold. 33 bytes. */
    const val DESCENT_ARMED_AUTOLAND = "Tag autoland armed - to touchdown"

    /**
     * The commit fired (FC floor or terminal, fresh in-cone fix) and `KeyStartAutoLanding` is
     * being asked: DJI flies the landing from here, our sticks are neutral, and any manual
     * stick cancels our engagement AND withdraws the landing. Said because the operator must
     * know which regime they are watching — and who is flying. 34 bytes.
     */
    const val DESCENT_DJI_LANDING = "DJI landing on tag - sticks cancel"

    /** Motors off observed during the landing: the engagement is over, wheels down. 29 bytes. */
    const val DESCENT_TOUCHDOWN = "Touchdown - autoland complete"

    /**
     * Rule 1 (or a disarm/pause) fired mid-landing and `KeyStopAutoLanding` was asked — said
     * because whether DJI honours it is a measurement, and an operator whose stop might not
     * stop is owed the fact that it was at least sent. 24 bytes.
     */
    const val DESCENT_STOP_SENT = "Stop landing sent to DJI"

    /**
     * A committed landing whose flight mode never became a landing mode within
     * [TagDescentGuidance.DJI_LAND_TIMEOUT_MS]: DJI accepted a command it did not enact (the
     * measured silent-accept failure), the engagement is back to holding, and the decision is
     * the operator's. 36 bytes.
     */
    const val DESCENT_DJI_TIMEOUT = "DJI never took the landing - holding"

    /** `"Autoland commit failed: <reason>"` — the one-shot land() refused synchronously. */
    fun autolandCommitFailed(reason: String): String =
        preferring("Autoland commit failed: $reason", reason)

    // ── Shadow mode ──────────────────────────────────────────────────────────
    //
    // The validation gate before the first real engagement: the whole controller runs — every
    // gate, the ladder, the machine, the velocities — and actuates NOTHING; its would-be
    // commands go only to the flight record, its would-be cancels are recorded as edges while
    // it keeps running, and it re-arms itself whenever the gates hold again, so one manual
    // landing produces a continuous timeline. The operator must never wonder whether the
    // aircraft is about to move, so the spoken sentences all say so, and the per-segment
    // transitions deliberately stay off this 50-byte ERROR-severity channel — they are not
    // actionable, and every one is in the record (`tag_descent_*`, shadow-marked).

    /** Shadow mode enabled: computing everything, actuating nothing. 30 bytes. */
    const val DESCENT_SHADOW_ON = "Shadow descent on - not flying"

    /** The shadow reached its terminal state — what a real run would have held at. 33 bytes. */
    const val DESCENT_SHADOW_COMPLETE = "Shadow descent complete - no hold"

    /** `"Shadow descent off: <reason>"` — the ways the *mode* ends, reason named. */
    fun shadowOff(reason: String): String = preferring("Shadow descent off: $reason", reason)

    /** No latch: the camera has not seen a tag at this site this flight. */
    const val REASON_NO_TAG = "NO_TAG_LATCHED"

    /**
     * Latched, camera believed at nadir, but the newest world fix is older than the arm bound —
     * the tag is not in view *now*. Judged **after** [REASON_NOT_NADIR] since 2026-07-28, so
     * this sentence can no longer be manufactured by a camera problem: `TagWorld.fix` refuses
     * every sighting without a believed nadir pitch, and two real sessions that day read
     * TAG_NOT_IN_VIEW off a camera that was the actual fault.
     */
    const val REASON_TAG_STALE = "TAG_NOT_IN_VIEW"

    /**
     * Above [TagDescentGuidance.APPROACH_CEILING_M] — beyond every measured decode. Names the
     * limit, per Q1's rule. The old `above 7m tag band` refusal is gone on purpose: between
     * 7 m and here the arm is now *accepted* into the approach segment (the friction landing13
     * t=41.8 measured), so the only ceiling left to refuse at is the decode-reach one.
     */
    val REASON_APPROACH_CEILING = "above ${TagDescentGuidance.APPROACH_CEILING_M.toInt()}m decode reach"

    /**
     * The believed camera pitch — commanded, or last-reported when nothing was commanded — is
     * not near nadir, or no belief exists at all. Judged before [REASON_TAG_STALE]: a camera
     * that is wrong makes the fix absent, and the refusal must name the cause.
     */
    const val REASON_NOT_NADIR = "CAMERA_NOT_NADIR"

    /** Another manoeuvre of ours is flying; a descent does not silently cancel it. */
    const val REASON_DESCENT_BUSY = "MANOEUVRE_ACTIVE"

    /** No on-board detector wired this session — there is no sensor to descend on. */
    const val REASON_NO_DETECTOR = "NO_DETECTOR"

    /** The GCS link is not up. The descent inherits the engine's link watchdog, so it needs one. */
    const val REASON_LINK_DOWN = "LINK_DOWN"

    /** No home coordinate: the fix's north/east frame has no origin to place the aircraft in. */
    const val REASON_NO_HOME = "NO_HOME_POINT"

    // ── Region of interest ───────────────────────────────────────────────────
    //
    // An ROI is a *modifier*, not a manoeuvre: it changes where the camera looks and — only while
    // we are already the one flying — where the nose points, and it changes the flight path not at
    // all. So its sentences are about what the operator can and cannot expect to see, and there are
    // four of them because there are four ways the honest answer is "less than you asked for":
    // the height was discarded, the aircraft is not ours to turn, the target is too close to track,
    // and the gimbal has run out of travel. Every one of them is said rather than hidden — the
    // `DO_ORBIT` gap of 2026-07-27 (a refusal with no `STATUSTEXT` at all) is not to be repeated.

    /** A `DO_SET_ROI_LOCATION` was validated and the target taken. 31 bytes. */
    const val ROI_STARTED = "ROI accepted - camera on target"

    /**
     * **The discarded altitude, announced once per ROI.** For PX4 — our identity — QGC runs a
     * terrain-database query and sends the result as the ROI's `z`. Our datum is a barometer that
     * moved 41.5 m between sessions, so the two are unrelated numbers whose difference means
     * nothing. We use the click's lat/lon and assume the target is on the ground at our own takeoff
     * datum, which is right for the common case and wrong for a rooftop by the height of the
     * building — an error the operator is told about rather than left to discover. 43 bytes.
     */
    const val ROI_GROUND_LEVEL = "ROI at ground level - target height ignored"

    /**
     * **The honoured altitude, announced once per ROI** — the other half of [ROI_GROUND_LEVEL] since
     * 2026-07-30. A `MAV_FRAME_GLOBAL_RELATIVE_ALT` ROI carries its `z` in *our own* datum (metres
     * above the takeoff point), so it is used, and the operator is told the number that is now in the
     * pointing solution: `"ROI target 12.5m above takeoff"`, 30 bytes at two digits.
     *
     * Said even for `0.0`, deliberately, and that is not noise: "your target is at takeoff level
     * because you said so" and "your target is assumed to be at takeoff level because nobody could
     * tell" are different facts about the same geometry, and the second one is a warning.
     */
    fun roiTargetHeight(relAltM: Double): String =
        StatusTexts.clamp("ROI target %.1fm above takeoff".format(relAltM))

    /**
     * `DO_SET_ROI_NONE`, or the legacy 201 with `MAV_ROI_NONE`. **The camera is left exactly where
     * it is** — no recentring, no stowing. `docs/gimbal.md` already argues that a silent slew is
     * the wrong default, and a camera that whips to level the moment tracking ends is startling in
     * precisely the way a flying aircraft cannot afford. 37 bytes.
     */
    const val ROI_CLEARED = "ROI cleared - camera left where it is"

    /**
     * An ROI is set, the nose is more than [RoiGuidance.YAW_DEADBAND_DEG] off its bearing, and
     * **nothing of ours is flying** — so the camera is being tilted and the aircraft is not being
     * turned. `docs/m4-mission-execution.md` §9.3, verbatim: we do not yaw an aircraft somebody
     * else is flying, and the operator is owed the reason their subject is off-frame. 23 bytes.
     */
    const val ROI_PITCH_ONLY = "ROI: pitch only, no yaw"

    /**
     * Inside [RoiGuidance.MIN_RANGE_M] horizontally: the camera holds its last angle and no yaw is
     * commanded. A camera hunting at the limit is worse than one pointing approximately down.
     * 36 bytes.
     */
    const val ROI_TOO_CLOSE = "ROI too close - holding camera angle"

    /**
     * The pointing solution is outside the travel DJI reported for this gimbal: the camera is at its
     * limit and the target is not in frame. The ROI is **not** refused for this — a
     * partially-correct aim the operator knows about is more useful than none. 39 bytes.
     */
    const val ROI_GIMBAL_RANGE = "ROI: gimbal at its limit, not on target"

    /**
     * `yawDeg` is stale or absent while an ROI is being tracked by a manoeuvre of ours, so the
     * heading loop commands **zero** rather than guessing. The camera keeps tilting and the flight
     * path continues — the same graduated treatment the vertical axis gets when the altitude goes
     * unknowable. **Also the orbit's**, since 2026-07-27: a circle points at its own centre through
     * the ROI system rather than through a second law of its own, so there is one sentence for
     * "cannot yaw, do not know which way we face" instead of two. 28 bytes.
     */
    const val ROI_NO_HEADING = "ROI: no heading - not yawing"

    /**
     * An abort fired while an ROI was being tracked. The gimbal **stops being driven and stays where
     * it is**; the target is remembered, so the next manoeuvre re-acquires it, and
     * `DO_SET_ROI_NONE` clears it in any state. `docs/m4-mission-execution.md` §9.5. 20 bytes.
     */
    const val ROI_TRACKING_STOPPED = "ROI tracking stopped"

    /**
     * **A mission ended while an ROI its own plan had set was still in force** — no `DO_SET_ROI_NONE`
     * before the plan ran out, or the plan was paused or aborted mid-ROI. The plan's target is dropped
     * (it dies with the plan that asked for it, `docs/m4-mission-transport.md` §6.3) and **the camera is
     * left exactly where the plan left it** — no recentring, no stowing, no nadir.
     *
     * Its own sentence rather than [ROI_CLEARED]'s, because the two are different news: `ROI cleared` is
     * something the operator asked for, and this is something that *happened to* the camera while their
     * attention was on a mission ending. An operator who reads only "mission complete" and then finds
     * the camera at 40° of depression has been told nothing, which is the silent half-application this
     * project treats as a bug. 42 bytes.
     *
     * An ROI **the operator set by hand** never produces this line and is never dropped here: theirs
     * outlives every manoeuvre, and `abort` says [ROI_TRACKING_STOPPED] for it as it always has.
     */
    const val MISSION_ROI_ENDED = "ROI ended with the plan - camera left as is"

    /** `"ROI refused: <reason>"` — rides the `COMMAND_ACK`'s `DENIED`, which QGC shows as a modal. */
    fun roiRefused(reason: String): String = preferring("ROI refused: $reason", reason)

    /** No camera is attached at all, so there is nothing that could point anywhere. */
    const val REASON_ROI_NO_GIMBAL = "NO_GIMBAL"

    /** A `COMMAND_LONG` 195/197 — QGC sends `COMMAND_INT`; the float-coordinate shape is unmeasured. */
    const val REASON_ROI_LONG_FORM = "COMMAND_LONG ROI"

    /** A legacy `DO_SET_ROI` asking for a `MAV_ROI` mode that is not a fixed location or "none". */
    const val REASON_ROI_MODE = "ROI mode unsupported"

    /** The long form if it fits, else the essential part alone — `command/StatusTexts`' rule. */
    private fun preferring(full: String, essential: String): String =
        if (full.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES) full
        else StatusTexts.clamp(essential)
}
