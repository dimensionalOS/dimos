package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.guided.GuidedSituation
import com.dimensional.mini4pro.mission.MissionPlan
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Geo
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.telemetry.TelemetryEncoder

/**
 * **What may be drawn, given what is known** — the honesty gate of the situation view, and the
 * only place in this package that decides it.
 *
 * Pure Kotlin. Takes an `AircraftState`, the engine's `GuidedSituation` and a plan, and returns
 * a [Situation] carrying **only the things it could establish**. Everything downstream — the
 * projection, the scene, the `View` — draws what it is handed without asking any further
 * questions, because by then there is nothing left to ask.
 *
 * ## The rule this file exists to enforce
 *
 * A picture can lie faster than text. The status dump has to spell out "position: stale" and an
 * operator has to read the word; a triangle sitting in the middle of a map says *the aircraft
 * is there* in no time at all, with no way to qualify it. So the display rule this project
 * applies to sentences is applied here to symbols, and it is applied by **removal**:
 *
 *  - **No aircraft symbol without a fresh fix.** `Signal.POSITION` carries a 1 s limit against
 *    a measured ~12 Hz feed, so a stale position is a dead feed rather than a quiet one, and
 *    the symbol goes away. Not greyed — *away*. A grey aircraft in a plausible place is still
 *    an aircraft in a place.
 *  - **No heading without a fresh attitude.** This one degrades rather than disappears: the
 *    symbol becomes a dot with no orientation, because we still know *where* it is. That
 *    distinction is not cosmetic on this airframe — `Signal.ATTITUDE` is change-driven and was
 *    measured going 15.3 s between deliveries on a motionless aircraft, so a frozen triangle
 *    would be the normal case rather than the exceptional one.
 *  - **No coordinate `Geo` will not accept.** DJI's no-home filler (4.583662361046586E7 in both
 *    fields) is a populated coordinate that a map would plot, and it once produced 220
 *    fabricated `HOME_POSITION` messages. Home additionally honours DJI's own
 *    `KeyIsHomeLocationSet` through [TelemetryEncoder.homeCoordinate], so this file and the
 *    MAVLink wire cannot disagree about whether there is a home.
 *  - **No manoeuvre the engine is not holding.** The circle, the goto target and the ROI come
 *    from `GuidedStickEngine.situation()` and nowhere else. There is no cache here and no
 *    "last commanded" anything, so a manoeuvre that ended cannot survive on screen.
 *  - **No plan that is being flown**, because none is. See [planMarkOf].
 *
 * Every removal leaves a [Situation.notes] line, so an empty frame is never mistaken for a
 * quiet situation.
 */
object SituationReading {

    /** Said when there is no position we would repeat, so the aircraft is not on the picture. */
    const val NOTE_NO_FIX = "no fresh fix — aircraft not drawn"

    /** Said when the fix is good but the attitude is not: a dot, not a triangle. */
    const val NOTE_NO_HEADING = "attitude stale — heading not shown"

    /** Said when a plan is loaded. It is not being flown, and nothing in this build could fly it. */
    const val NOTE_PLAN_NOT_FLYING = "plan loaded — nothing is flying it"

    /** Said when an ROI is remembered but the camera is not being driven at it. */
    const val NOTE_ROI_SUSPENDED = "ROI remembered — camera not tracking"

    /**
     * The picture, from the three things that know anything.
     *
     * @param guided null when there is no engine — the bridge is stopped, or this is a replay,
     *   where there is no engine by construction. Null yields no manoeuvre geometry at all,
     *   which is the truth in both cases.
     */
    fun read(
        source: SituationSource,
        state: AircraftState,
        guided: GuidedSituation? = null,
        plan: PlanMark? = null,
    ): Situation {
        val notes = ArrayList<String>(3)

        val fix = Geo.coordinateOrNull(state.latitude, state.longitude)
            ?.takeIf { state.isFresh(Signal.POSITION) }
        if (fix == null) notes.add(NOTE_NO_FIX)

        val heading = state.yawDeg
            ?.takeIf { it.isFinite() }
            ?.takeIf { state.isFresh(Signal.ATTITUDE) }
        // Only worth saying when there is a symbol for it to be missing from. With no fix at
        // all, NOTE_NO_FIX already covers it and a second line would be noise.
        if (fix != null && heading == null) notes.add(NOTE_NO_HEADING)

        val aircraft = fix?.let { AircraftMark(Fix(it.first, it.second), heading) }
        val home = TelemetryEncoder.homeCoordinate(state)?.let { Fix(it.first, it.second) }

        val orbit = guided?.orbit
            ?.takeIf { it.radiusM.isFinite() && it.radiusM > 0.0 }
            ?.let { o ->
                Geo.coordinateOrNull(o.centreLatDeg, o.centreLonDeg)?.let { c ->
                    OrbitMark(Fix(c.first, c.second), o.radiusM, if (o.direction < 0) -1 else 1)
                }
            }
        val goto = guided?.goto?.let { g ->
            Geo.coordinateOrNull(g.latDeg, g.lonDeg)?.let { t ->
                GotoMark(Fix(t.first, t.second), g.arrived)
            }
        }
        val roi = guided?.roi?.let { r ->
            Geo.coordinateOrNull(r.latDeg, r.lonDeg)?.let { t ->
                RoiMark(Fix(t.first, t.second), r.tracking)
            }
        }
        if (roi != null && !roi.tracking) notes.add(NOTE_ROI_SUSPENDED)
        if (plan != null && plan.points.isNotEmpty() && !plan.flying) notes.add(NOTE_PLAN_NOT_FLYING)

        return Situation(
            source = source,
            aircraft = aircraft,
            home = home,
            orbit = orbit,
            goto = goto,
            roi = roi,
            plan = plan?.takeIf { it.points.isNotEmpty() },
            notes = notes,
        )
    }

    /**
     * A committed plan's navigable legs as drawable points — **and it is never marked as
     * flying.**
     *
     * `PlanMark.flying` is hard-coded false and `currentSeq` hard-coded null here, not because
     * the fields are unused but because there is nothing in this build that could truthfully
     * set them: the mission *executor* does not exist, and
     * `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-2 puts Start in QGC. A plan uploaded
     * from QGC sits in `MissionStore` and the aircraft does not move. Drawing it with a
     * highlighted "current" leg would be the picture inventing a capability, which is the
     * failure this whole package is arranged against — so the plan is drawn as what it is, a
     * loaded intention, with [NOTE_PLAN_NOT_FLYING] beside it.
     *
     * Legs with no place of their own — `RTL` and `DELAY` carry a null `target` by construction
     * — contribute no point, because there is no point to contribute. They are not silently
     * given the previous waypoint's position.
     */
    fun planMarkOf(plan: MissionPlan?): PlanMark? {
        if (plan == null) return null
        val points = ArrayList<PlanPoint>(plan.legs.size)
        for (leg in plan.legs) {
            val target = leg.target ?: continue
            val accepted = Geo.coordinateOrNull(target.latDeg, target.lonDeg) ?: continue
            points.add(PlanPoint(leg.seq, Fix(accepted.first, accepted.second), leg.kind.name))
        }
        if (points.isEmpty()) return null
        return PlanMark(points = points, currentSeq = null, flying = false)
    }
}
