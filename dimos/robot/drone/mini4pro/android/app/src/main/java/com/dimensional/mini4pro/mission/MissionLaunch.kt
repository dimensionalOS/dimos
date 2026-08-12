package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.guided.GuidedEnvelope
import com.dimensional.mini4pro.guided.MissionGuidance
import com.dimensional.mini4pro.guided.MissionRoute
import com.dimensional.mini4pro.guided.MissionStep
import com.dimensional.mini4pro.guided.MissionStepKind
import com.dimensional.mini4pro.guided.ResumeBlock

/**
 * Everything a Start needs to know that a desk cannot: facts about the world, gathered once at the
 * moment the operator presses the button.
 *
 * A plain data class with no clock and no aircraft handle, so [MissionLaunch.evaluate] is pure and
 * every one of §7.2's checks has its own unit test with no fakes in it.
 */
data class LaunchInputs(
    val interlockOn: Boolean,
    val plan: MissionPlan?,
    /** True when this Start is a **resume** — the lifecycle is `PAUSED` and a cursor is held. */
    val resuming: Boolean,
    /** The wire `seq` the paused cursor points at, for a resume. */
    val cursorSeq: Int?,
    /** The plan id the paused cursor was taken against. A change invalidates it. */
    val planIdAtPause: Int?,
    /** A standing block from the row of §6.2 that caused the pause, or null. */
    val resumeBlock: ResumeBlock?,
    /** The aircraft's own position, already range-checked, or null. */
    val fix: Pair<Double, Double>?,
    val positionFresh: Boolean,
    /**
     * Whether the aircraft is **talking to us at all** — `AircraftState.fcConnected`, the SDK's own
     * link state.
     *
     * Deliberately not another data key. Every telemetry key on this airframe is *change-driven*
     * (`telemetry/SampleAges`), so on a parked aircraft they all go quiet together and none of them
     * can distinguish "nothing is happening" from "nobody is there". The connection state is driven
     * by the SDK's connection callbacks rather than by data changing, which is exactly what makes it
     * usable as the liveness anchor [positionUsable] needs. See its KDoc.
     */
    val linkAlive: Boolean,
    /** `relativeAltitude` — metres above **our** takeoff point. Null when it has never been delivered. */
    val relativeAltitudeM: Double?,
    /** `takeoffAltitudeAmsl` — the datum. Null when nothing has been published this link. */
    val amslDatumM: Double?,
    val homeSet: Boolean?,
    val homeLatDeg: Double?,
    val homeLonDeg: Double?,
    val batteryPercent: Int?,
    val isFlying: Boolean?,
    /**
     * Whether a `DO_SET_HOME` has been seen this session. M4-14: **any** of them blocks mission
     * start until the aircraft lands and takes off again.
     */
    val homeMovedThisSession: Boolean,
    /** Whether anything is wired that can start DJI's own takeoff. */
    val takeoffAvailable: Boolean,
)

/** The answer to a Start. Either a refusal with a reason word, or a route and where to begin it. */
sealed interface LaunchVerdict {

    /** [reason] is the word the `STATUSTEXT` carries — short, because the framing has to fit round it. */
    data class Refused(val reason: String) : LaunchVerdict

    data class Cleared(
        val route: MissionRoute,
        val startIndex: Int,
        /** True for a resume: the first leg is a **resting** rejoin whatever the step says. */
        val rejoining: Boolean,
    ) : LaunchVerdict
}

/**
 * §7.2's launch admission, plus the translation from a stored plan into the route the engine flies.
 *
 * **Pure**: no clock, no aircraft, no DJI, no store. Everything it needs is in [LaunchInputs], which
 * is what lets every check below have its own named test with no fakes at all.
 *
 * ## The rule these checks exist to keep
 *
 * > A mission that violates a bound is **refused before it flies**, not aborted halfway.
 *
 * A halfway abort leaves an aircraft somewhere nobody planned, which is the worst of both outcomes.
 * Admission is therefore in two halves: `MissionAdmission` runs at upload against the things a desk
 * can know, and this runs at every Start — including every resume — against the things it cannot.
 *
 * **A failed check changes nothing.** No engage, no setpoint, no mode claim; the answer is a
 * `STATUSTEXT` naming the **single first** failure, because a list of five reasons is a list nobody
 * reads and the first one is the one to fix.
 */
object MissionLaunch {

    // The reason words. Short on purpose — `GuidedStatusTexts.missionRefused` prefers the reason
    // over its own framing when the 50 bytes will not stretch.
    const val REASON_INTERLOCK = "interlock off"
    const val REASON_NO_PLAN = "no plan loaded"
    const val REASON_NO_FIX = "no position fix"
    const val REASON_NO_DATUM = "NO_ALT_DATUM"
    const val REASON_HOME_UNKNOWN = "home unknown"
    const val REASON_HOME_MOVED = "home was moved"
    const val REASON_BATTERY = "battery too low"
    const val REASON_ON_THE_GROUND = "aircraft not flying"
    const val REASON_ALREADY_FLYING = "already flying"
    const val REASON_NO_TAKEOFF = "cannot take off"
    const val REASON_PLAN_CHANGED = "plan changed"
    const val REASON_TOO_FAR_TO_REJOIN = "too far to rejoin"
    const val REASON_NOTHING_TO_FLY = "nothing to fly"

    /**
     * Whether the fix in [inputs] may be launched on — **and the one place this bridge admits that
     * a position which has stopped changing is not the same thing as a position it has lost.**
     *
     * `KeyAircraftLocation` is change-driven, like every other key on this airframe. A parked
     * aircraft does not move, so it publishes nothing, so `POSITION` reads stale past its 1 s limit
     * within a second of the aircraft being set down — **permanently**. Measured on 2026-07-27
     * (`tmp/session-logs/takeoff-climb-ccw-orbit.jsonl`): while parked, 38 consecutive
     * `GLOBAL_POSITION_INT` messages carried a perfectly good coordinate and **exactly one distinct
     * value**; airborne and translating, 232 distinct values in 400 messages.
     *
     * With the flat freshness rule that made a mission beginning with a takeoff **unstartable by
     * construction**, which is exactly what happened on the bench: check 3 here wants a fresh
     * position, which is only true in flight, and check 5 below wants an aircraft that is *not*
     * flying, which is only true on the ground. The two can never both hold, and QGC's Start
     * answered `no position fix` with a full-strength fix on the screen.
     *
     * So the rule is split by what the two situations actually mean:
     *
     *  - **in the air, unchanged**: a position feed that has gone quiet is a controller flying
     *    blind, and this refuses. Nothing about the in-flight case is relaxed here.
     *  - **on the ground, with the link alive**: the aircraft not moving is *correct information*
     *    about an aircraft that is not moving. What has to be established instead is that anyone is
     *    still there to hear us, and [LaunchInputs.linkAlive] is the only signal on this airframe
     *    that answers that without going quiet along with everything else.
     *
     * `isFlying` is required to be an actual **false** rather than "not true": a null is DJI
     * declining to say, and the relaxation is only sound for an aircraft DJI has told us is on the
     * ground.
     */
    fun positionUsable(inputs: LaunchInputs): Boolean = when {
        inputs.positionFresh -> true
        inputs.isFlying == false && inputs.linkAlive -> true
        else -> false
    }

    /** `Item 4 too far from home` — the bound that keeps the flown path inside line of sight. */
    fun reasonTooFar(seq: Int): String = "item $seq too far from home"

    /** `Item 3 not flyable` — an item this executor refuses to sequence. See [routeOf]. */
    fun reasonNotFlyable(seq: Int): String = "item $seq not flyable"

    /** `Item 2 above the ceiling` — re-checked at Start, because the datum drifts. */
    fun reasonTooHigh(seq: Int): String = "item $seq above ceiling"

    /**
     * The whole launch check, in order, first failure only.
     *
     * The order is not arbitrary: **cheapest and most fundamental first**, so that an operator with
     * the interlock off is told that rather than being told about an item's altitude, and so that no
     * check downstream has to cope with an input an earlier one would have rejected.
     */
    fun evaluate(inputs: LaunchInputs): LaunchVerdict {
        // 1. Consent. Without it nothing engages, so every check after this would be theatre.
        if (!inputs.interlockOn) return LaunchVerdict.Refused(REASON_INTERLOCK)

        val plan = inputs.plan ?: return LaunchVerdict.Refused(REASON_NO_PLAN)

        // 2. A resume's two extra questions, asked before the world's, because a cursor against a
        //    plan that no longer exists is not a mission at all.
        if (inputs.resuming) {
            if (inputs.planIdAtPause != null && inputs.planIdAtPause != plan.planId) {
                return LaunchVerdict.Refused(REASON_PLAN_CHANGED)
            }
            // A standing block refuses **and is spent by the refusal** when its persistence says so
            // — §6.1's "spent as soon as its condition clears or the operator is told about it".
            // Spending is the caller's, because it is a state change and this function has no state.
            inputs.resumeBlock?.let { return LaunchVerdict.Refused(it.reason) }
        }

        // 3. The world, in the order a failure is most likely and most actionable.
        val fix = inputs.fix
        if (fix == null || !positionUsable(inputs)) return LaunchVerdict.Refused(REASON_NO_FIX)
        if (inputs.amslDatumM == null || !inputs.amslDatumM.isFinite() ||
            inputs.relativeAltitudeM == null
        ) {
            return LaunchVerdict.Refused(REASON_NO_DATUM)
        }
        // M4-14: QGC's plan altitudes are relative to **home** and our `relativeAltitude` is relative
        // to the **takeoff point**. Identical in the normal case and different after a
        // `DO_SET_HOME` — silently, in the vertical axis, in the direction that ends in the ground.
        // A cheap absolute block removes the whole class.
        if (inputs.homeMovedThisSession) return LaunchVerdict.Refused(REASON_HOME_MOVED)
        if (inputs.homeSet != true) return LaunchVerdict.Refused(REASON_HOME_UNKNOWN)
        val home = homeOrNull(inputs) ?: return LaunchVerdict.Refused(REASON_HOME_UNKNOWN)
        val battery = inputs.batteryPercent
        if (battery == null || battery < MissionGuidance.BATTERY_START_MIN_PCT) {
            return LaunchVerdict.Refused(REASON_BATTERY)
        }

        // 4. The plan, translated. Anything this executor cannot fly is a refusal naming the item.
        val route = when (val built = routeOf(plan, home, inputs.takeoffAvailable)) {
            is RouteResult.Refused -> return LaunchVerdict.Refused(built.reason)
            is RouteResult.Built -> built.route
        }

        val startIndex = if (inputs.resuming) {
            route.steps.indexOfFirst { it.seq == inputs.cursorSeq }
                .takeIf { it >= 0 } ?: return LaunchVerdict.Refused(REASON_PLAN_CHANGED)
        } else {
            0
        }

        // 5. Does the aircraft's own state match what the plan starts with? A plan beginning with a
        //    takeoff needs an aircraft that is not flying; anything else needs one that is.
        val first = route[startIndex]
        if (first.kind == MissionStepKind.TAKEOFF) {
            if (inputs.isFlying == true) return LaunchVerdict.Refused(REASON_ALREADY_FLYING)
        } else if (inputs.isFlying != true) {
            return LaunchVerdict.Refused(REASON_ON_THE_GROUND)
        }

        // 6. Every item inside line of sight of home, **with the corner-cutting allowance added**, so
        //    that a plan cleared here cannot be violated by the corner-cutting the design chose.
        val bound = MissionGuidance.MAX_HOME_DIST_M - MissionGuidance.CORNER_ALLOWANCE_M
        for (step in route.steps) {
            val distance = MissionGeo.distanceM(home, GeoPoint(step.latDeg, step.lonDeg))
            if (distance > bound) return LaunchVerdict.Refused(reasonTooFar(step.seq))
            // Re-checked here and not only at upload: the barometric datum drifts 2.3 m in twelve
            // minutes and a plan admitted twenty minutes ago was admitted against a different number.
            val alt = step.relAltM
            if (alt != null && (!alt.isFinite() || alt > GuidedEnvelope.CEILING_M)) {
                return LaunchVerdict.Refused(reasonTooHigh(step.seq))
            }
        }

        // 7. The rejoin gate. §6.3: a resume is a leg nobody drew, so it is bounded far more
        //    tightly than a fresh goto — someone pressing Continue is picturing the aircraft roughly
        //    where it stopped.
        if (inputs.resuming) {
            val toCursor = MissionGeo.distanceM(
                GeoPoint(fix.first, fix.second), GeoPoint(first.latDeg, first.lonDeg),
            )
            if (toCursor > MissionGuidance.REJOIN_MAX_M) {
                return LaunchVerdict.Refused(REASON_TOO_FAR_TO_REJOIN)
            }
        }

        return LaunchVerdict.Cleared(route, startIndex, rejoining = inputs.resuming)
    }

    private fun homeOrNull(inputs: LaunchInputs): GeoPoint? {
        val lat = inputs.homeLatDeg ?: return null
        val lon = inputs.homeLonDeg ?: return null
        val pair = com.dimensional.mini4pro.telemetry.Geo.coordinateOrNull(lat, lon) ?: return null
        return GeoPoint(pair.first, pair.second)
    }

    private sealed interface RouteResult {
        data class Built(val route: MissionRoute) : RouteResult
        data class Refused(val reason: String) : RouteResult
    }

    /**
     * The stored plan's navigable items, translated into the route the engine flies — the *one*
     * place a plan's vocabulary becomes the executor's, so the 10 Hz tick has no interpretation left
     * to do.
     *
     * ## What is refused here, and why refusing at Start rather than at upload
     *
     * `MissionAdmission` accepts a wider vocabulary than this executor flies, and the gap is
     * deliberate rather than an oversight — the transport was built first and its allow-list is what
     * QGC can author. Three item kinds are accepted into the store and refused here:
     *
     *  - **`DO_ORBIT`** — an orbit inside a mission is real machinery (`GuidedStickEngine.orbit`
     *    already flies one) but sequencing it needs a completion signal the mission cursor does not
     *    have, and building it half-way would give the cursor a source of advances that is neither
     *    of §3.1's two tests. Deferred, out loud.
     *  - **`NAV_DELAY` and `NAV_LOITER_TIME`** — both advance the cursor on a **clock**, and §3.4's
     *    rule is that the cursor advances only on an observation. Rather than carve an exception into
     *    the one rule that keeps a cursor honest, they are refused.
     *
     * Refusing at Start costs the operator a legible sentence at the moment they press the button,
     * which is better than the alternative on offer today (a `when` branch discovered at item 14) and
     * honest about where the gap is.
     *
     * ## What each accepted item becomes
     *
     * `NAV_TAKEOFF` is a takeoff step; `NAV_WAYPOINT` is a fly-through unless it is last;
     * `NAV_RETURN_TO_LAUNCH`, `NAV_LOITER_UNLIM` and `NAV_LAND` **with precision landing Disabled**
     * are **holds that do not descend** (M4-5, M4-7) and must therefore be last. An RTL has no
     * coordinate of its own and is resolved to [home] here, which is the one place this bridge decides
     * what "return to launch" means: *fly to home and hold*, never DJI's own RTH.
     *
     * **`NAV_LAND` with `param2 >= 1`** ([PrecisionLandMode]) becomes a
     * [MissionStepKind.PRECISION_LAND] step instead — the tag landing,
     * [com.dimensional.mini4pro.guided.PrecisionLand]. Two things are decided here and nowhere else:
     *
     *  - **the item's altitude is kept**, because for this step it is an ordinary waypoint altitude
     *    (Ivan: *"the way we should treat this land message height is the actual height you create a
     *    waypoint to and just go there"*). A step built without one is **refused by name** rather than
     *    flown at whatever height the aircraft happens to have — an unflyable item at the desk beats
     *    a `null` discovered above the pad. `MissionStore.relativeAltOf` is the other half of this
     *    decision and the two are deliberately one sentence apart;
     *  - **the drawn coordinate is carried unchanged**, and it is not a target: the flown XY is the
     *    *recorded takeoff point*, resolved at the moment the item begins, because the plan's
     *    coordinates and the executed takeoff can differ by metres (Ivan: *"can be 5 m away for
     *    example"*). What the drawn coordinate is for is the site cross-check in `PrecisionLand.gate`
     *    and the geometry this layer already does with every step's position — the home-distance bound
     *    below, and [MissionRoute.restAheadM].
     *
     * ## The sticky state: `DO_SET_ROI_*`, carried rather than dropped (2026-07-30)
     *
     * A plan's `DO_SET_ROI_LOCATION` / `DO_SET_ROI_NONE` items produce no step of their own — they are
     * state, not places — and until 2026-07-30 this function **dropped them**, so a plan's ROI items
     * pointed nothing in flight (pinned as a gap by `MissionBig1PlanTest`, now pinned as a behaviour).
     * They are now carried onto every step as [MissionStep.roi], which is
     * [ResolvedLeg.roi] — the store's own sticky walk, the same one `DO_CHANGE_SPEED` rides — re-spelled
     * as the wire command the engine's live ROI door already takes
     * ([com.dimensional.mini4pro.guided.RoiCommand.pointingAt]). No sequencing is decided here: *when*
     * an ROI applies was decided at commit by the walk, and *how* it is applied is the engine's one ROI
     * owner. This function only stops throwing the answer away.
     *
     * The two flight gates are deliberately **not** evaluated here. They are facts about the moment
     * the item begins — where the aircraft *is*, how high it *is*, and where DJI has *since* recorded
     * home — and this function runs before the takeoff that sets the last of them. Checking them at
     * Start as well would be a second implementation of a property that can then disagree with the
     * first; what Start does check, and still checks for this step unchanged, is the item's altitude
     * against the ceiling and its distance from home.
     */
    private fun routeOf(plan: MissionPlan, home: GeoPoint, takeoffAvailable: Boolean): RouteResult {
        val steps = ArrayList<MissionStep>(plan.legs.size)
        for ((index, leg) in plan.legs.withIndex()) {
            val last = index == plan.legs.size - 1
            // The sticky ROI, translated once per step: `MissionStore.resolve` already decided *which*
            // ROI is in force at this item (the same walk that resolves `DO_CHANGE_SPEED`), and this is
            // the only place that answer becomes the executor's vocabulary. Attached to every step kind
            // uniformly — a plan item that applied to five of six step kinds would be a rule nobody
            // could state. See `MissionStep.roi`.
            val roi = leg.roi?.let {
                com.dimensional.mini4pro.guided.RoiCommand.pointingAt(
                    latDeg = it.target.latDeg,
                    lonDeg = it.target.lonDeg,
                    relativeAltM = it.relativeAltM,
                )
            }
            when (leg.kind) {
                LegKind.ORBIT, LegKind.DELAY -> return RouteResult.Refused(reasonNotFlyable(leg.seq))

                LegKind.HOVER -> {
                    // `NAV_LOITER_UNLIM` resolves to an infinite hold and is terminal; anything else
                    // in this family carries a timed hold, which is the clock §3.4 refuses.
                    if (!leg.holdSeconds.isInfinite() || !last) {
                        return RouteResult.Refused(reasonNotFlyable(leg.seq))
                    }
                    val point = leg.target ?: return RouteResult.Refused(reasonNotFlyable(leg.seq))
                    steps += MissionStep(
                        seq = leg.seq,
                        kind = MissionStepKind.HOLD,
                        latDeg = point.latDeg,
                        lonDeg = point.lonDeg,
                        relAltM = leg.relativeAltM,
                        switchRadiusM = MissionGuidance.switchRadiusM(leg.acceptRadiusM),
                        rest = true,
                        roi = roi,
                    )
                }

                LegKind.TAKEOFF -> {
                    if (!takeoffAvailable) return RouteResult.Refused(REASON_NO_TAKEOFF)
                    if (index != 0) return RouteResult.Refused(reasonNotFlyable(leg.seq))
                    // A takeoff has no lateral target: DJI climbs where the aircraft stands. The
                    // step's coordinate is home, which is where it is, so the geometry downstream
                    // (the home-distance bound, the route's leg walk) has a real point to use.
                    steps += MissionStep(
                        seq = leg.seq,
                        kind = MissionStepKind.TAKEOFF,
                        latDeg = leg.target?.latDeg ?: home.latDeg,
                        lonDeg = leg.target?.lonDeg ?: home.lonDeg,
                        relAltM = leg.relativeAltM,
                        switchRadiusM = MissionGuidance.R_SWITCH_M,
                        rest = true,
                        roi = roi,
                    )
                }

                LegKind.LAND, LegKind.RTL -> {
                    if (!last) return RouteResult.Refused(reasonNotFlyable(leg.seq))
                    val precision = leg.kind == LegKind.LAND &&
                        PrecisionLandMode.landsOnTag(leg.precisionLandMode ?: PrecisionLandMode.DISABLED)
                    if (precision) {
                        // The tag landing. The altitude is a real commanded height here, so a step
                        // without one is refused rather than built — see this function's KDoc.
                        val alt = leg.relativeAltM
                        if (alt == null || !alt.isFinite()) {
                            return RouteResult.Refused(reasonNotFlyable(leg.seq))
                        }
                        val drawn = leg.target ?: return RouteResult.Refused(reasonNotFlyable(leg.seq))
                        steps += MissionStep(
                            seq = leg.seq,
                            kind = MissionStepKind.PRECISION_LAND,
                            // As drawn: the cross-check's subject, never a target. The flown XY is
                            // the recorded takeoff point, resolved when the item begins.
                            latDeg = drawn.latDeg,
                            lonDeg = drawn.lonDeg,
                            relAltM = alt,
                            switchRadiusM = MissionGuidance.R_SWITCH_M,
                            rest = true,
                            // On the record, and only on the record: Opportunistic and Required fly
                            // identically today and the difference must still be readable afterwards.
                            precisionLandMode = leg.precisionLandMode ?: PrecisionLandMode.REQUIRED,
                            // Carried, and then cleared by the landing's own first tick — see
                            // `MissionStep.roi`. A visible take-and-give-up beats a silent drop.
                            roi = roi,
                        )
                        continue
                    }
                    // Neither descends. An RTL has no coordinate of its own; a land item's is where
                    // the aircraft goes to *hold*, not where it goes to touch down.
                    val point = leg.target ?: home
                    steps += MissionStep(
                        seq = leg.seq,
                        kind = MissionStepKind.HOLD,
                        latDeg = point.latDeg,
                        lonDeg = point.lonDeg,
                        // Deliberately null: the aircraft holds whatever altitude it arrives with.
                        // `MissionStore.resolve` already refuses to project a height for these two,
                        // and honouring one here would be this layer quietly promising a descent.
                        relAltM = null,
                        switchRadiusM = MissionGuidance.R_SWITCH_M,
                        rest = true,
                        roi = roi,
                    )
                }

                LegKind.WAYPOINT -> {
                    val point = leg.target ?: return RouteResult.Refused(reasonNotFlyable(leg.seq))
                    steps += MissionStep(
                        seq = leg.seq,
                        kind = MissionStepKind.WAYPOINT,
                        latDeg = point.latDeg,
                        lonDeg = point.lonDeg,
                        relAltM = leg.relativeAltM,
                        switchRadiusM = MissionGuidance.switchRadiusM(leg.acceptRadiusM),
                        // Rest only at the last one — Ivan's fourth answer, and the whole reason
                        // §3.1 has two completion tests.
                        rest = last,
                        roi = roi,
                    )
                }
            }
        }
        if (steps.isEmpty()) return RouteResult.Refused(REASON_NOTHING_TO_FLY)
        return RouteResult.Built(MissionRoute.of(plan.planId, steps))
    }
}
