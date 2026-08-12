package com.dimensional.mini4pro.situation

/**
 * Where the picture on screen came from — **the only two answers there will ever be**, and the
 * single most important field in this package.
 *
 * A top-down view of an aircraft, painted on a phone strapped beside an armed interlock, is a
 * claim about *now*. A replayed flight painted the same way is the same picture of a different
 * afternoon, and the failure mode is not subtle: an operator glances down, sees an aircraft
 * moving, and believes it. So the distinction is carried in the data rather than in a variable
 * somewhere in the Activity, every [Situation] has one, and `SituationScene` refuses to build
 * a scene without deciding what to do about it.
 */
enum class SituationSource {
    /** The aircraft, now, through `StateCache`. */
    LIVE,

    /** A recording off the phone's own flight-log directory. Never commands anything. */
    REPLAY,
}

/** A place on the earth we are willing to draw. Degrees, WGS-84. */
data class Fix(val latDeg: Double, val lonDeg: Double)

/**
 * The aircraft symbol's two facts, kept separate because they are known separately.
 *
 * [headingDeg] is null when the attitude feed is stale or absent — and null means the symbol
 * is drawn **without orientation** rather than pointing north. `Signal.ATTITUDE` is
 * change-driven on this airframe and goes quiet for tens of seconds on the ground; a triangle
 * frozen at the last heading is exactly the lie this package is shaped to prevent.
 */
data class AircraftMark(val fix: Fix, val headingDeg: Double?)

/**
 * An orbit in progress: the circle, its centre, and which way round.
 *
 * @param direction +1 clockwise, −1 anticlockwise, matching `guided/OrbitCommand`'s sign of
 *   `param1` and `OrbitGuidance.circleVelocity`'s parameter of the same name.
 */
data class OrbitMark(val centre: Fix, val radiusM: Double, val direction: Int)

/** A `DO_REPOSITION` target in progress. */
data class GotoMark(val target: Fix, val arrived: Boolean)

/**
 * A region of interest.
 *
 * @param tracking true only while the camera is actually being driven at it. A remembered ROI
 *   whose engagement has ended is still the place the operator asked about — `GuidedStickEngine`
 *   keeps it deliberately — but the camera is *not* on it, and the picture must not say it is.
 */
data class RoiMark(val target: Fix, val tracking: Boolean)

/** One waypoint of a loaded plan. [seq] is the wire sequence number, as `mission/ResolvedLeg`. */
data class PlanPoint(val seq: Int, val fix: Fix, val kind: String)

/**
 * A plan in the store.
 *
 * [flying] is **false in every build that exists today**, and the field is here so the picture
 * can say so out loud. `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-2 puts mission
 * Start in QGC and the executor is not written; a plan drawn as though something were flying it
 * would be the picture claiming a capability the bridge does not have. [currentSeq] is null for
 * the same reason — nothing knows which leg is current, because nothing is on one.
 */
data class PlanMark(
    val points: List<PlanPoint>,
    val currentSeq: Int? = null,
    val flying: Boolean = false,
)

/**
 * Everything the situation view is allowed to draw, and nothing else — **the honesty boundary
 * of this feature, as a type**.
 *
 * Every field is nullable and a null means *we do not know*, exactly as `AircraftState` means
 * it. There is no "last known" anything here: a value that has gone stale is removed by
 * [SituationReading] before it reaches this object, so a consumer cannot paint a stale symbol
 * even by accident, because it is not holding one.
 *
 * ## What this type deliberately does not contain
 *
 * No `AircraftState`, no engine, no port, no callback, no context — nothing but coordinates,
 * numbers, strings and enums. That is not tidiness. It is the structural half of the replay
 * safety argument: a replayed flight is turned into one of these *before* it leaves the replay
 * controller, and this object has no member that could reach a command path, so there is
 * nothing for a replayed state to be routed into. `SituationHonestyTest` asserts that property
 * by reflection rather than by review, so adding such a field fails the build.
 */
data class Situation(
    val source: SituationSource,
    val aircraft: AircraftMark? = null,
    val home: Fix? = null,
    val orbit: OrbitMark? = null,
    val goto: GotoMark? = null,
    val roi: RoiMark? = null,
    val plan: PlanMark? = null,
    /**
     * Where the aircraft has **been** — the only field here that is not about this instant.
     *
     * Null until something has been flown, and cut into segments wherever the position was not
     * known, so a dropout is a hole in the line rather than a straight leg across it. It is
     * accumulated by [FlownTrack] from these same vetted [aircraft] fixes and nothing else,
     * which is what keeps history under the rule the rest of this file applies to now: nothing
     * drawn that we do not know.
     */
    val track: TrackMark? = null,
    /**
     * Short operator-facing sentences about what is *missing* — "no fresh fix", "heading
     * stale". The picture degrades by dropping symbols, and these are how it says why rather
     * than leaving an empty frame to be read as "nothing is happening".
     */
    val notes: List<String> = emptyList(),
    /**
     * True when this recording is being **published** — put on the MAVLink link, the Zenoh bus,
     * or both — rather than only drawn.
     *
     * A `Boolean` and nothing else, deliberately, because this type is flat value data and
     * `SituationHonestyTest` asserts that by reflection: there is nothing here for a replayed
     * state to be routed into, and a flag does not change that. What it buys is the one indicator
     * an operator sees without looking for it — [SituationView] paints a different watermark
     * across the whole picture. A drawn replay is contained by the screen; a published one has
     * reached people who did not open anything, and the two must not look identical to the person
     * who can still switch it off.
     *
     * Always false for [SituationSource.LIVE]. There is nothing to publish.
     */
    val publishing: Boolean = false,
) {
    /** True when there is at least one thing on the earth to draw. */
    val hasAnything: Boolean
        get() = aircraft != null || home != null || orbit != null ||
            goto != null || roi != null || (plan?.points?.isNotEmpty() == true) ||
            (track?.isEmpty == false)

    companion object {
        /** Nothing known, live. What the screen shows before an aircraft is connected. */
        val UNKNOWN = Situation(SituationSource.LIVE)
    }
}
