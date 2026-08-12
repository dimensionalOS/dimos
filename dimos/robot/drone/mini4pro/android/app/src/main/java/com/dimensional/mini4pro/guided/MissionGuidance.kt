package com.dimensional.mini4pro.guided

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * M4's arithmetic: the two leg-completion tests, the fly-through stopping envelope, and the bounds
 * a mission is refused against. Pure functions, no state, no DJI, no Android — every number that
 * decides where a *mission* flies the aircraft is in this object and under `MissionGuidanceTest`.
 *
 * Design authority: `docs/m4-mission-execution.md` §3 and §7, as amended by
 * `docs/decisions/2026-07-27-m4-eighteen-answers.md` (whose answers win wherever they disagree).
 *
 * ## Two completion tests, because there are two kinds of waypoint
 *
 * Ivan's fourth answer — *fly through intermediate waypoints, come to rest only at the last* —
 * splits the problem cleanly, and the split is the whole reason this object exists:
 *
 *  - **The final waypoint is arrived at**, and there the M3 test is reused by **calling
 *    [RepositionGuidance.settled]**, not by restating it. That predicate has arrived twice on a
 *    simulator and repeatedly in real air (0.09 m / 0.01 m / 0.68 m / 0.74 m), and re-deriving it
 *    would throw away the only arrival evidence this project owns. `MissionGuidanceTest` fails if
 *    the mission path stops calling the same function the reposition path calls.
 *  - **Intermediate waypoints are passed, not arrived at.** The M3 test is deliberately
 *    unsatisfiable by a fly-through — that is stated in `Reposition.kt` as a feature — so reusing
 *    it here would be reusing it *wrongly*. [passed] is geometric, not kinematic.
 *
 * ## The fly-through, and the safety property it must not break
 *
 * M3's law brakes to zero at its target by construction: `v ≤ sqrt(2·a_max·e)` goes to zero as the
 * error does. Retargeting at 3 m out would therefore give a "slow to 1.5 m/s, turn, accelerate
 * again" corner. The envelope is a **safety property**, not a tuning parameter, so it is restated
 * precisely enough to survive:
 *
 * > **The commanded speed never exceeds the speed from which the aircraft can stop, at `a_max`,
 * > before the next point at which the mission requires it to be stationary.**
 *
 * That point is not the current waypoint. So exactly one substitution enters the law:
 *
 * ```
 * e_stop = d(current waypoint) + Σ(remaining leg lengths up to and including the next rest point)
 * v_cmd  = clamp(k_p · e_leg,  |v| ≤ min(HORIZONTAL_MAX_MS, sqrt(2 · a_max · e_stop)))
 * ```
 *
 * The **P-term still uses the current leg's error**, which keeps the corner behaviour sane (the
 * aircraft is at `k_p · 3 m = 1.5 m/s` when it switches, so the rounded corner is metres rather
 * than tens of metres), while the **braking term uses the distance to the true stop**. On a
 * straight-line mission the two terms coincide; on the last leg `e_stop == e_leg` and the law is
 * *literally* [RepositionGuidance.clampedSpeed] — pinned by a test that compares them digit for
 * digit, because "it reduces to M3's" is the kind of claim that rots silently.
 *
 * `e_stop` is the one number a mutant can quietly make infinite, so the property test walks
 * generated missions and asserts the bound at **every** tick against an independently computed
 * distance-to-rest.
 */
object MissionGuidance {

    // ------------------------------------------------------------------ the switch radius

    /**
     * The default switch radius, metres — how close is "we got there" for a waypoint the aircraft
     * flies *through*.
     *
     * 3 m rather than [RepositionGuidance.R_ACCEPT_M]'s 2 m: the arrival test's radius is chosen so
     * a settled aircraft is inside it, while this one is chosen so a *moving* aircraft crosses it
     * on a 10 Hz tick — at the 3 m/s envelope cap the aircraft covers 0.3 m per tick, so a 3 m ball
     * is entered for about twenty ticks even on a grazing pass.
     */
    const val R_SWITCH_M = 3.0

    /** The smallest per-item acceptance radius a plan may ask for, metres. §3.5. */
    const val R_SWITCH_MIN_M = 2.0

    /** The largest per-item acceptance radius a plan may ask for, metres. §3.5. */
    const val R_SWITCH_MAX_M = 10.0

    /**
     * The corridor the half-plane term is bounded by, metres.
     *
     * The half-plane term exists because a waypoint the aircraft cannot quite reach — wind, a GPS
     * bias, an obstacle brake — would otherwise be circled until the leg timeout, turning a 1 m
     * error into an aborted mission. But an aircraft blown far off track must not declare a
     * waypoint passed from 60 m to the side, so outside this corridor **only the radius counts**.
     */
    const val R_MISS_M = 10.0

    /**
     * How much wider than the drawn plan the *flown* path may be, metres — the corner-cutting
     * allowance added to every distance-from-home check at Start.
     *
     * `v/k_p` at the speed the aircraft carries through a switch: it enters the ball at
     * `k_p · R_SWITCH = 1.5 m/s` and the turn's radius is that divided by the gain, i.e. 3 m. Six
     * is double it, on the standing rule that an allowance which decides a refusal should be
     * generous in the direction of refusing. **A plan admitted at the desk must not be violated by
     * the corner-cutting the design itself chose** (§3.3).
     */
    const val CORNER_ALLOWANCE_M = 6.0

    // ------------------------------------------------------------------ the bounds

    /**
     * How far from home any item may be at Start, metres.
     *
     * **2 km since 2026-07-30, by Ivan** — *"2 km is our new limit"*,
     * `docs/decisions/2026-07-30-two-kilometre-envelope.md`. That decision **supersedes M4-3's
     * reasoning for this constant**: it was 150 m for *"line of sight at this site"*
     * (`docs/decisions/2026-07-27-m4-eighteen-answers.md`), and the aircraft's owner has replaced
     * that judgement with a number of his own. Line of sight at 2 km is not something this bridge
     * can check — it has no idea where the operator is standing, or whether they can see — so it is
     * now the pilot's to keep rather than a bound this code pretends to enforce.
     *
     * It is deliberately **the same number** as [GuidedEnvelope.MAX_REPOSITION_DISTANCE_M] and not
     * an alias of it: the two answer different questions (the farthest a *single leg* may reach
     * versus the farthest from *home* the mission may go) and a future decision may well move one
     * without the other, exactly as 100/150 differed before today.
     *
     * Checked **with [CORNER_ALLOWANCE_M] added**, so the number that is actually enforced against
     * a waypoint is `MAX_HOME_DIST_M − CORNER_ALLOWANCE_M` = 1994 m of drawn distance.
     */
    const val MAX_HOME_DIST_M = 2000.0

    /**
     * How far the aircraft may be from the cursor when a paused mission is resumed, metres.
     *
     * **Deliberately left at 50 m by the 2026-07-30 envelope expansion**, which is a decision and
     * not an oversight. A rejoin is *the one leg nobody drew*: someone pressing Continue is
     * picturing the aircraft roughly where it stopped, and that mental picture did not get larger
     * because the plan may now reach further. Ivan's decision was about how far a *drawn* plan may
     * go; nothing in it touches an undrawn leg, so this bound keeps its own argument. Beyond it the
     * resume is refused and the honest suggestion is to fly the aircraft back manually or restart
     * the mission.
     *
     * It is now 1/40th of a fresh goto's reach rather than 1/2 of it. That asymmetry is the point:
     * one of those two legs is on a map.
     */
    const val REJOIN_MAX_M = 50.0

    /**
     * The whole path a plan may draw, metres — **an out-and-back at the envelope's reach**.
     *
     * Derived, not chosen, so that Ivan's 2 km cannot be admitted by the leg bound and then refused
     * by a total that predates it: at a 100 m reach the total was his own flat 500 m (M4-3,
     * *"~2.8 min of flying at cruise"*), and a single 2 km leg would have been refused by it on the
     * way out. Two times the reach is the smallest total that makes the new limit usable in both
     * directions, and it stays a **refusal** threshold — 20 items at 2 km each could otherwise draw
     * 40 km of path.
     *
     * [com.dimensional.mini4pro.mission.MissionAdmission.MAX_TOTAL_M] is this number; the desk
     * imports it rather than restating it.
     */
    const val MAX_PATH_M = 2.0 * GuidedEnvelope.MAX_REPOSITION_DISTANCE_M

    /**
     * The whole-mission time cap, seconds — **the time the admitted path needs, under the same
     * deadline law every other manoeuvre is bounded by** ([GuidedEnvelope.manoeuvreDeadlineMs] read
     * at [MAX_PATH_M]).
     *
     * It was a flat 600 s (M4-3: *"a mission cannot outlive the battery through a sequence of
     * nearly-timing-out legs"*) and that number cannot survive the 2 km reach: **one** 2 km leg is
     * ~667 s of travel, so a flat 600 s would have aborted a mission for being exactly as long as
     * the desk admitted — the mid-leg abort this expansion exists to avoid, one layer up.
     *
     * Derived rather than re-chosen so the two cannot disagree: whatever path admission accepts,
     * this clock allows it. It computes to **~2668 s (44 minutes)**.
     *
     * **And that is the honest cost, stated rather than hidden: at 44 minutes this constant is no
     * longer a battery bound.** No Mini 4 Pro flies it. The clock's original two jobs have come
     * apart — it still catches a mission that has stopped making progress, and it no longer catches
     * one that will outlive its battery, because nothing in this repo models battery against
     * distance and this layer sees only percentages at Start ([BATTERY_START_MIN_PCT]). What
     * catches that instead is the pilot and DJI's own RTH failsafe.
     * `docs/decisions/2026-07-30-two-kilometre-envelope.md` carries the argument and names the
     * measurement that would let this become a real bound again.
     */
    val MISSION_MAX_S: Long =
        (GuidedEnvelope.manoeuvreDeadlineMs(MAX_PATH_M, 0.0) + 999L) / 1_000L

    /**
     * The battery percentage below which a mission may not **begin**, and the one place in this
     * layer a percentage is allowed to be a flight-control input.
     *
     * Refusing to begin on a low number is a *refusal*, not an intervention, so a conservative
     * refusal built on a coarse signal is safe in the direction that matters. The in-flight
     * low-battery policy is emphatically **not** built on this — it waits on DJI's own warning
     * keys, which `AircraftState` does not yet carry (§6.5), and until they exist rows 12 and 13 of
     * the abort table degrade to row 7's authority reasons. That degradation is stated in
     * [MissionAbortPolicy] rather than quietly relied upon.
     */
    const val BATTERY_START_MIN_PCT = 30

    /** The per-leg timeout's fixed slack, milliseconds. Half a minute on top of the travel time. */
    const val LEG_TIMEOUT_BASE_MS = 30_000L

    /**
     * The per-leg timeout's travel allowance, milliseconds per metre — a generous worst case of
     * 1 m/s average against a nominal 3 m/s.
     *
     * **Why this stays a second, differently-shaped deadline after 2026-07-30**, when the guided
     * manoeuvre deadline became distance-derived ([GuidedEnvelope.manoeuvreDeadlineMs]) and the
     * "one owner per property" rule would otherwise fold them together: a mission leg and a goto
     * bound *different* properties with different consequences. A goto's deadline ends the
     * manoeuvre and hands the aircraft back; a leg's deadline **pauses the plan at its cursor**
     * (row 15, [MissionAbortPolicy]), which is a resumable hold rather than a withdrawal — and a
     * fly-through leg must not be ended for being slow when the next leg is what it is heading for.
     * Folding one into the other would change mission behaviour on the strength of a shape
     * coincidence.
     *
     * The relationship that does have to hold is checked rather than asserted in prose: **a leg is
     * never timed out before the law could plausibly have flown it** (`MissionGuidanceTest`). At the
     * extremes: a 5 m leg gets 35 s for 2.3 s of travel; a 2 km leg gets 2030 s for 667 s, where the
     * goto deadline would be 1334 s — looser, in the direction of not aborting.
     */
    const val LEG_TIMEOUT_PER_METRE_MS = 1_000L

    /** `LEG_TIMEOUT_MS = 30 000 + 1000 × legMetres`. A 100 m leg gets 130 s against a nominal ~40 s. */
    fun legTimeoutMs(legMetres: Double): Long {
        if (!legMetres.isFinite() || legMetres <= 0.0) return LEG_TIMEOUT_BASE_MS
        return LEG_TIMEOUT_BASE_MS + (LEG_TIMEOUT_PER_METRE_MS * legMetres).toLong()
    }

    /**
     * The switch radius in force for one item: the plan's `param2` when it asked for one inside the
     * band, otherwise [R_SWITCH_M].
     *
     * A radius **outside** the band is refused at upload rather than clamped here (§3.5, and
     * `MissionAdmission` is where that happens), so a value arriving out of band means the
     * admission gate has been bypassed — this function refuses to honour it and falls back to our
     * own, which is the fail-safe direction.
     *
     * Why honour it at all, when the house instinct is *our thresholds always win*? Because the
     * switch radius is genuinely a **path** parameter rather than a safety one: it decides how
     * tightly the corner is cut, and the person who drew the plan knows whether they are flying
     * between two trees or across a field. It cannot weaken a safety property, because the stopping
     * envelope, the speed cap, the ceiling and the arrival test all live elsewhere.
     */
    fun switchRadiusM(requestedM: Double?): Double {
        if (requestedM == null || !requestedM.isFinite()) return R_SWITCH_M
        if (requestedM < R_SWITCH_MIN_M || requestedM > R_SWITCH_MAX_M) return R_SWITCH_M
        return requestedM
    }

    // ------------------------------------------------------------------ leg completion

    /**
     * Metres still to go **projected on the leg** — `dot(W − p, unit(W − P))`, negative once the
     * aircraft has crossed the plane through `W` perpendicular to the leg.
     *
     * @param toTargetNorthM the NED offset from the aircraft to the target, `W − p` — **the same
     *   vector the translation law is given**, so the two cannot disagree about where the target is.
     * @param legNorthM the NED offset from the leg's origin to the target, `W − P`.
     *
     * A zero-length or non-finite leg has no direction, so this returns the plain distance: with no
     * projection available the half-plane term can never fire, and only the radius counts. (Zero-
     * length legs are refused at upload; this is the fail-closed answer if one ever arrives.)
     */
    fun alongTrackRemainingM(
        toTargetNorthM: Double,
        toTargetEastM: Double,
        legNorthM: Double,
        legEastM: Double,
    ): Double {
        if (!toTargetNorthM.isFinite() || !toTargetEastM.isFinite()) return 0.0
        val distance = hypot(toTargetNorthM, toTargetEastM)
        val legLength = hypot(legNorthM, legEastM)
        if (!legLength.isFinite() || legLength <= 0.0) return distance
        return (toTargetNorthM * legNorthM + toTargetEastM * legEastM) / legLength
    }

    /**
     * **The pass-through test** — the completion predicate for a waypoint the mission flies through.
     *
     * ```
     * passed = (d ≤ R_switch) OR (alongTrack ≤ 0 AND d ≤ R_MISS)
     * ```
     *
     * The first term is the normal case: we got there. The second is the honest statement *"this
     * waypoint is behind us now"* for a waypoint the aircraft could not quite reach, bounded by
     * [R_MISS_M] so that being blown off track cannot declare a distant waypoint passed.
     *
     * **Vertical error deliberately does not gate a pass-through.** The vertical axis is the weakest
     * one on this airframe (the altitude key is change-driven and 0.1 m-quantised) and a mission
     * that stalls at a waypoint because the climb has not finished is a mission that hovers until
     * its timeout. The next leg's altitude is commanded from the moment the cursor advances, so the
     * climb continues; it simply stops being a gate. The resting waypoint keeps M3's vertical
     * conjunct, because that is where the aircraft comes to rest.
     */
    fun passed(distanceM: Double, alongTrackRemainingM: Double, switchRadiusM: Double): Boolean {
        if (!distanceM.isFinite()) return false
        if (distanceM <= switchRadiusM) return true
        return alongTrackRemainingM <= 0.0 && distanceM <= R_MISS_M
    }

    // ------------------------------------------------------------------ the law

    /**
     * §3.3's `e_stop`: the distance to the next point at which the mission requires zero speed.
     *
     * @param legErrorM how far the aircraft is from the waypoint it is currently flying to.
     * @param restAheadM the summed lengths of the legs from that waypoint forward to the next
     *   resting one — [MissionRoute.restAheadM], and zero when the current waypoint rests.
     *
     * Non-finite or negative inputs collapse to zero, which commands **no speed at all**. That is
     * the fail-closed direction and it is the reason this function exists rather than the addition
     * being inline: a NaN reaching `sqrt(2·a·e)` would produce a NaN clamp that compares false
     * against everything, i.e. an *unbounded* speed.
     */
    fun stopDistanceM(legErrorM: Double, restAheadM: Double): Double {
        if (!legErrorM.isFinite() || legErrorM < 0.0) return 0.0
        if (!restAheadM.isFinite() || restAheadM < 0.0) return legErrorM
        return legErrorM + restAheadM
    }

    /**
     * The fly-through speed magnitude: **M3's own law, read at the distance to the next point the
     * mission requires zero speed at** rather than at the distance to the next waypoint.
     *
     * `RepositionGuidance.clampedSpeed(e_stop)`, and that is the whole function. With
     * `stopDistanceM == legErrorM` — the final leg, where the mission does stop at the point it is
     * flying to — it is [RepositionGuidance.clampedSpeed] **by construction** rather than by
     * arithmetic that happens to agree, which is what lets the final leg inherit M3's flight
     * evidence unchanged.
     *
     * ## Why it is not `min(k_p·e_leg, …)`, which is what it was until 2026-07-27
     *
     * It used to take the proportional term against the **current leg** and the braking term
     * against `e_stop`. The braking half was right and the proportional half quietly undid it: with
     * `KP_PER_S = 0.5` the P-term binds below 6 m, so the commanded speed decayed linearly from the
     * envelope to `0.5 × R_SWITCH_M` = **1.45 m/s** at every crossing and snapped back to 3.0 the
     * instant the cursor advanced. The first mission ever flown measured exactly that — minima of
     * 1.46, 1.47 and 1.44 m/s at the three intermediate waypoints
     * (`docs/measurements/2026-07-27-first-mission-flown.md` §7.1) — an aircraft braking into every
     * corner and accelerating out of it, for a stop the mission never asked for.
     *
     * The proportional term exists to make a *final approach* smooth. On a leg the aircraft is
     * flying **through**, there is no approach to smooth, and `e_stop` already knows where the
     * real one is.
     *
     * **What this cost, stated because it is a trade and not a free win:** a corner is now taken at
     * up to the full envelope instead of at half of it, so the flown path cuts wider. The measured
     * worst deviation from the drawn polyline before the change was 2.71 m against
     * [CORNER_ALLOWANCE_M]'s 6.0 m, and that allowance is what [MAX_HOME_DIST_M] subtracts so the
     * *flown* path cannot leave a bound the *drawn* path was admitted against. **It must be
     * re-measured on the next flight rather than assumed to still hold** — and note that the corner
     * allowance is a property of the switch radius and the gain, not of the leg length, so the
     * 2026-07-30 move to a 2 km envelope does not change it.
     */
    fun flyThroughSpeed(legErrorM: Double, stopDistanceM: Double): Double {
        if (!legErrorM.isFinite() || legErrorM <= 0.0) return 0.0
        val stop = if (stopDistanceM.isFinite() && stopDistanceM > 0.0) stopDistanceM else 0.0
        return RepositionGuidance.clampedSpeed(stop, GuidedEnvelope.HORIZONTAL_MAX_MS)
    }

    /**
     * The mission leg's law: NED position error → commanded NED velocity, with the braking term
     * evaluated against [stopDistanceM] rather than against the current leg.
     *
     * Deliberately the same shape as [RepositionGuidance.velocity] — the horizontal law runs on the
     * 2-D error **vector** (magnitude clamped, direction kept) so the path is a straight line rather
     * than a curve that converges one axis first, the vertical axis runs its own law under its own
     * limit, and a null [errorDownM] means the vertical error is unknowable and commands zero rather
     * than a guess. Yaw is zero here and is added by the engine, which is the only thing that knows
     * whether an ROI or heading-follows-course is in force.
     */
    fun velocity(
        errorNorthM: Double,
        errorEastM: Double,
        errorDownM: Double?,
        stopDistanceM: Double,
    ): StickVelocities {
        if (!errorNorthM.isFinite() || !errorEastM.isFinite()) return StickVelocities.ZERO
        if (errorDownM != null && !errorDownM.isFinite()) return StickVelocities.ZERO

        var north = 0.0
        var east = 0.0
        val horizontal = hypot(errorNorthM, errorEastM)
        if (horizontal > 0.0) {
            val speed = flyThroughSpeed(horizontal, stopDistanceM)
            north = speed * errorNorthM / horizontal
            east = speed * errorEastM / horizontal
        }

        var down = 0.0
        if (errorDownM != null && errorDownM != 0.0) {
            down = RepositionGuidance.clampedSpeed(abs(errorDownM), GuidedEnvelope.VERTICAL_MAX_MS) *
                sign(errorDownM)
        }
        return StickVelocities(north, east, down, yawRateDegPerS = 0.0)
    }
}
