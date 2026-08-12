package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.telemetry.Geo

/**
 * What one step of a route *is*, for the executor's tick. Four values, because there are exactly
 * four shapes of thing the tick has to do, and a `when` over this breaks loudly the day a fifth
 * arrives.
 */
enum class MissionStepKind {
    /**
     * **DJI's own takeoff, then the climb the two-phase design is building.** The cursor leaves
     * this step when DJI reports the aircraft flying *and* the climb phase has completed — and the
     * climb phase is deliberately **not built here**: see [MissionRoute] and
     * `GuidedStickEngine.missionTakeoffClimb` for the seam another agent's two-phase takeoff work
     * attaches to.
     */
    TAKEOFF,

    /**
     * A place to fly to. [MissionStep.rest] decides whether the aircraft passes through it on the
     * geometric crossing test or settles on it under the flight-verified M3 arrival test.
     */
    WAYPOINT,

    /**
     * A place to fly to, settle on, and **hold** — `NAV_RETURN_TO_LAUNCH`, `NAV_LOITER_UNLIM`, and
     * `NAV_LAND` **with "Precision Land" Disabled** (`param2 = 0`, QGC's default). None of them
     * descends.
     *
     * `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-5 reversed the ground-to-ground answer:
     * *"just hover for now, I'll land manually"*. So a land item that does not ask for precision
     * means fly there, come to rest, and wait for a human on the sticks; an RTL item (M4-7) means the
     * same thing at home. The mission reaches its terminal state **with the aircraft in the air**.
     *
     * That is unchanged by [PRECISION_LAND] and the fact that it is unchanged is a property with its
     * own mutation row: every plan authored before 2026-07-30 carries `param2 = 0`, and a plan that
     * does not ask to land must not land.
     */
    HOLD,

    /**
     * `NAV_LAND` with QGC's **"Precision Land"** set to Opportunistic or Required (`param2 >= 1`):
     * fly to the recorded takeoff point at the item's own altitude, lower to the arm height, and hand
     * the last stretch to the tag descent with full autoland. The whole sequence — its two gates, its
     * two numbers, its phases, and the reason the reference point is the *recorded* takeoff position
     * rather than any drawn coordinate — is [PrecisionLand].
     *
     * The step's own [MissionStep.latDeg]/[MissionStep.lonDeg] are the item **as drawn**, and they are
     * deliberately not a target: they serve [PrecisionLand.gate]'s site cross-check and the route's
     * geometry (the home-distance bound at Start, [MissionRoute.restAheadM]). [MissionStep.relAltM] is
     * the item's altitude and **is** flown to, as an ordinary waypoint altitude.
     *
     * Always last and always resting, on the same structural rule as [HOLD]: `MissionAdmission`
     * refuses a land item that is not last, and [MissionRoute.of] coerces the last step to rest.
     */
    PRECISION_LAND,
}

/**
 * One step of a route: a place, a height, and the two facts that decide how the leg to it ends.
 *
 * @param seq the item's **wire** sequence number, which is what `MISSION_CURRENT` and
 *   `MISSION_ITEM_REACHED` carry. Not an index into [MissionRoute.steps] — a plan holds
 *   non-navigable items too.
 * @param relAltM metres above this bridge's own takeoff datum, already ceiling-checked, or null
 *   for a step that commands no height — an RTL, or a `NAV_LAND` with precision landing Disabled:
 *   neither descends, so neither names a height we fly to. A
 *   [MissionStepKind.PRECISION_LAND] step **does** carry one (the item's altitude, flown as an
 *   ordinary waypoint altitude), and `MissionLaunch` refuses to build one without it.
 * @param rest true when the mission requires the aircraft to be **stationary** here. This is the
 *   flag the stopping envelope is computed against (§3.3) and the flag that chooses between the two
 *   completion tests (§3.1).
 *
 * There is deliberately **no hold-time field**. A `NAV_LOITER_TIME` or `NAV_DELAY` item advances the
 * cursor on a *clock*, and §3.4's rule is that the cursor advances only on an observation; rather
 * than carve an exception, those items are refused at Start with a sentence. `NAV_LOITER_UNLIM` is
 * a terminal hold and needs no timer at all.
 */
data class MissionStep(
    val seq: Int,
    val kind: MissionStepKind,
    val latDeg: Double,
    val lonDeg: Double,
    val relAltM: Double?,
    val switchRadiusM: Double,
    val rest: Boolean,
    /**
     * `NAV_LAND.param2` for a [MissionStepKind.PRECISION_LAND] step — 1 (Opportunistic) or 2
     * (Required), the values [com.dimensional.mini4pro.mission.PrecisionLandMode] names. **0 for
     * every other step**, which is exactly what "this step is not a tag landing" reads as.
     *
     * Carried for the flight record rather than for behaviour: 1 and 2 fly identically today
     * (`PrecisionLandMode`'s KDoc has the reason — *"otherwise land normally"* is a landing nobody has
     * decided to build), so the record is the only place the difference exists, and it must be there
     * or the day that decision is taken there will be no evidence of which plans asked for which.
     */
    val precisionLandMode: Int = 0,
    /**
     * **The ROI in force while this step is the cursor**, as the plan's own `DO_SET_ROI_LOCATION`
     * (195) / `DO_SET_ROI` (201) items set it, or **null when the plan asks for no ROI here** —
     * before the first ROI item, or after a `DO_SET_ROI_NONE` (197).
     *
     * ## Sequencing: a `DO_` item acts when the sequence reaches it, and this field is that rule
     *
     * A plan's `DO_` items are **sticky state**, not places: they occupy sequence numbers, produce no
     * leg, and apply to every navigable item that follows until something changes them. That
     * resolution has exactly one implementation — `MissionStore.resolve`, which walks the plan once at
     * commit carrying the speed limit and the ROI and stamps each `ResolvedLeg` with the values *as
     * they stand at that item* — and this field is that projection carried into the executor's
     * vocabulary by `MissionLaunch.routeOf`. **The precedent followed is `DO_CHANGE_SPEED`'s**, which
     * is resolved by the same walk in the same loop; nothing about sequencing is invented here, and in
     * particular there is no second backwards scan at 10 Hz, which is the thing that is correct in
     * review and wrong at item 14.
     *
     * Concretely, on the plan this was built for (`big1.plan`, read 2026-07-30 — item 6 is
     * `DO_SET_ROI_LOCATION` at 37.99387681/23.7257871 in frame 3 with `z = 0`, item 8 is
     * `DO_SET_ROI_NONE`): the ROI is null on the steps for items 0–5, **item 6's target on the step
     * for item 7**, and null again on the step for item 9. So the camera comes onto the target as the
     * leg to item 7 begins and comes off it when item 7 is reached — which is Ivan's expectation
     * verbatim (*"after wp6 it looks at roi, at wp8 it should stop"*, in QGC's display numbering,
     * which is the wire `seq` plus one for the deleted planned-home marker).
     *
     * ## What the engine does with it, and what it deliberately does not
     *
     * `GuidedStickEngine` applies a **change**, never a value per tick: on Start and on every cursor
     * advance it compares this field with the last thing the mission itself applied and, if they
     * differ, drives the live ROI door — [RoiCommand.pointingAt] for a set and
     * [RoiCommand.clearing] for the `DO_SET_ROI_NONE`. An unchanged ROI across a corner is therefore
     * *not* re-taken: re-taking it would reset the camera's rate limiter and re-announce the target on
     * every leg of a five-leg mission.
     *
     * Two consequences worth stating because they look like omissions:
     *
     *  - **a step whose kind is [MissionStepKind.PRECISION_LAND] still carries its ROI**, and the
     *    landing sequence then clears it in its first tick (`beginLandTagLocked` — the camera is the
     *    landing's). Both acts are on the record. Dropping it here instead would be a plan item
     *    silently half-applied, which is worse than a target taken and given up in one visible pair.
     *  - **the height is the item's own**, in our own datum, and reaches the pointing solution
     *    unchanged; `RoiTarget.relativeAltM` → [RoiCommand.pointingAt] → `RoiCommand.relativeAltMOrNull`
     *    → the depression angle, with no second interpretation of `z` anywhere on the plan path.
     */
    val roi: RoiCommand? = null,
)

/**
 * A committed plan, flattened into exactly what the 10 Hz tick needs: places, heights, and where
 * the aircraft is required to be stationary.
 *
 * **Built once, at Start, and never mutated.** The executor hands one of these to the engine and
 * the engine flies it; an upload arriving mid-flight replaces the *store*, not this
 * (`docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-12 — *"the running mission continues on
 * the plan it started with"*), and [planId] is what makes a resume across that change refusable.
 *
 * ## Why the route is a separate type from `MissionPlan`
 *
 * `mission/MissionStore`'s `ResolvedLeg` is the *transport's* projection: it carries a plan's
 * vocabulary, including items this executor refuses to fly. This type carries the *executor's*
 * vocabulary, in which every step is a place with a height and a completion rule, so the tick has
 * no interpretation left to do. The translation happens once, in the launch check, where a step
 * this bridge cannot fly is a refusal the operator reads at the desk rather than a `when` branch
 * discovered at item 14.
 *
 * ## The invariant the stopping envelope rests on
 *
 * **The last step always rests.** [restAheadM] walks forward looking for the next step at which the
 * mission requires zero speed, and a route whose last step did not rest would leave that walk with
 * no terminator — which is precisely the "`e_stop` quietly becomes infinite" mutation §3.3 names.
 * [of] enforces it rather than trusting callers, and a test asserts the enforcement.
 */
class MissionRoute private constructor(
    val planId: Int,
    val steps: List<MissionStep>,
) {

    val size: Int get() = steps.size

    operator fun get(index: Int): MissionStep = steps[index]

    /**
     * **The height this plan's own `NAV_TAKEOFF` item cleared**, metres above the takeoff datum, or
     * null when the plan has no takeoff item at all.
     *
     * The single owner of that number for the whole executor, because exactly one thing reads it and
     * a second spelling could disagree with this one: [PrecisionLand.gate]'s *"do not start the
     * landing sequence from below the height the plan cleared"*. Null is a real answer and is refused
     * by name there rather than defaulted to zero — a plan that cleared nothing has no such height,
     * and unknown is never zero.
     *
     * Read from the route rather than from the plan, so a resume — which rebuilds the route and starts
     * at a later cursor — still sees the takeoff item's height: the route holds every step, always,
     * whatever the cursor points at.
     */
    val takeoffRelAltM: Double?
        get() = steps.firstOrNull { it.kind == MissionStepKind.TAKEOFF }?.relAltM

    /**
     * The horizontal distance, metres, from step [index] forward to the **next point at which the
     * mission requires the aircraft to be stationary**, inclusive — the Σ term of §3.3's
     * `e_stop`.
     *
     * Zero when step [index] itself rests, which is the case that makes the fly-through law reduce
     * *literally* to M3's: the final leg's braking term then sees the current leg's own error and
     * nothing else.
     *
     * Walks forward rather than caching, because a route is at most twenty items long (admission's
     * `MAX_ITEMS`) and a cache is a second place for the invariant to be wrong.
     *
     * **One approximation, stated:** a [MissionStepKind.PRECISION_LAND] step is measured at its
     * *drawn* coordinate, while the leg to it is flown to the recorded takeoff point — up to
     * [PrecisionLand.LAND_TAG_RADIUS_M] apart, and 0.3 m apart in the plan the feature was built for.
     * The route is built at Start, before the takeoff this mission performs has re-recorded home, so
     * the flown point is not knowable here. It costs nothing that matters: the precision-land step
     * *rests*, so its own leg's braking term reads its own error to the real target
     * ([MissionGuidance.stopDistanceM] with `restAhead = 0`), and the only number affected is the
     * speed carried through the *preceding* corner.
     */
    fun restAheadM(index: Int): Double {
        if (index !in steps.indices) return 0.0
        if (steps[index].rest) return 0.0
        var total = 0.0
        var i = index
        while (i + 1 < steps.size) {
            total += Geo.nedMetres(
                steps[i].latDeg, steps[i].lonDeg, steps[i + 1].latDeg, steps[i + 1].lonDeg,
            ).let { (n, e) -> kotlin.math.hypot(n, e) }
            i++
            if (steps[i].rest) return total
        }
        // Unreachable while `of` holds its invariant; returning what we walked is the fail-*short*
        // answer, which under-states `e_stop` and therefore brakes earlier. Never the fail-long one.
        return total
    }

    companion object {
        /**
         * Builds a route and **forces the last step to rest**, whatever the caller said.
         *
         * The coercion is deliberate rather than a `require`: this is called on the Start path, and
         * a route that could not be built would have to become a refusal with a sentence nobody can
         * act on ("the last item does not rest" is not an operator's problem). Resting one step
         * earlier than a caller intended is safe in the only direction that matters — it brakes.
         */
        fun of(planId: Int, steps: List<MissionStep>): MissionRoute {
            require(steps.isNotEmpty()) { "a route needs at least one step" }
            val fixed = steps.toMutableList()
            fixed[fixed.lastIndex] = fixed.last().copy(rest = true)
            return MissionRoute(planId, fixed)
        }
    }
}
