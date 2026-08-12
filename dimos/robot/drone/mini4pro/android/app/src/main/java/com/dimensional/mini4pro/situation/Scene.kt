package com.dimensional.mini4pro.situation

import kotlin.math.cos
import kotlin.math.sin

/**
 * What a shape *means*, never what colour it is.
 *
 * The `View` owns the palette because a colour is a resource and a resource needs a `Context`.
 * This layer owns the meaning, so the question "is a suspended ROI drawn differently from a
 * tracked one?" is answered in a unit test rather than by looking at a phone.
 */
enum class Ink {
    AIRCRAFT,

    /**
     * The recent part of the flown track — the window the frame was fitted around, and the part
     * that still describes what the aircraft is doing.
     */
    TRACK,

    /**
     * The rest of the flown track: older than the fit window, drawn to recede. Separate from
     * [TRACK] because the difference is meaning — "where it is coming from" against "where it
     * has been" — and the `View` is left to decide only how faint that looks.
     */
    TRACK_PAST,
    /** A drawn thing whose input is present but not live — a remembered ROI, a plan nobody flies. */
    GHOST,
    ORBIT,
    GOTO,
    PLAN,
    /** The leg being flown. Unreachable today; see `SituationReading.planMarkOf`. */
    PLAN_CURRENT,
    ROI,
    HOME,
    /** The replay watermark and frame. Never used by anything that describes an aircraft. */
    REPLAY,
    /** Scale bar, labels, the honesty notes. */
    CHROME,
}

/** One primitive the `View` paints. Screen pixels, already projected — no geodesy downstream. */
sealed class Shape {
    abstract val ink: Ink

    /** An open path: plan legs, the line to a goto, the camera's line of sight. */
    data class Path(
        val points: List<ScreenPoint>,
        override val ink: Ink,
        /** True for a line that means "intended", drawn broken so it cannot read as flown. */
        val dashed: Boolean = false,
    ) : Shape()

    /** A filled outline: the aircraft triangle, a direction chevron. */
    data class Polygon(val points: List<ScreenPoint>, override val ink: Ink) : Shape()

    /** A stroked circle: the orbit. */
    data class Circle(
        val centre: ScreenPoint,
        val radiusPx: Double,
        override val ink: Ink,
        val dashed: Boolean = false,
    ) : Shape()

    /** A small filled marker: a waypoint, a centre, a home, an ROI point, a heading-less aircraft. */
    data class Dot(
        val centre: ScreenPoint,
        val radiusPx: Double,
        override val ink: Ink,
        val hollow: Boolean = false,
    ) : Shape()

    /** Text anchored at a point, drawn left-aligned just below it. */
    data class Label(
        val at: ScreenPoint,
        val text: String,
        override val ink: Ink,
    ) : Shape()
}

/**
 * A finished picture: primitives in paint order, plus the two things the chrome needs.
 *
 * [scaleBarMetres]/[scaleBarPx] are a matched pair — a round number of metres and how long that
 * is on this canvas — so the operator can tell a 20 m orbit from a 200 m one. Without it an
 * auto-scaled view is unreadable: every picture fills the frame, so the frame says nothing.
 */
data class Scene(
    val shapes: List<Shape>,
    val source: SituationSource,
    val notes: List<String>,
    val scaleBarMetres: Double? = null,
    val scaleBarPx: Double? = null,
    /**
     * Carried through from [Situation.publishing], untouched, so the `View` can pick a watermark
     * without knowing what a replay is.
     *
     * It is not a shape because it is not *on* the earth — it is a fact about the whole canvas,
     * like [source] beside it, and turning it into geometry here would put the same string in
     * twelve `Shape`s and make the wash a thing this file laid out rather than a thing the view
     * paints across itself.
     */
    val publishing: Boolean = false,
) {
    val isEmpty: Boolean get() = shapes.isEmpty()
}

/**
 * [Situation] + [Viewport] → [Scene]: the last pure step, and the one that makes the `View`
 * dumb.
 *
 * Everything with an opinion happens here — where the triangle's corners are, which way the
 * orbit chevron points, whether a line is dashed, what the scale bar says. The `View` receives
 * pixel coordinates and an [Ink] and does nothing but stroke and fill, which is what lets all
 * of it be tested on the JVM with no `Context`, no `Canvas` and no phone.
 *
 * Paint order is deliberate and is the order things are added below: **context first, aircraft
 * last**. The aircraft is the thing being looked for, so nothing is ever drawn over it.
 */
object SituationScene {

    /** Half-height of the aircraft triangle, in pixels. Sized for a glance, not for beauty. */
    const val AIRCRAFT_PX = 18.0

    /** Radius of the dot the aircraft degrades to when heading is unknown. */
    const val AIRCRAFT_DOT_PX = 7.0

    /** Half-angle from the nose to each rear corner. 145° gives a clearly pointed dart. */
    const val AIRCRAFT_SPREAD_DEG = 145.0

    /** Radius of a waypoint / centre / home / ROI marker. */
    const val MARKER_PX = 6.0

    /** Length of the orbit's direction chevron arms. */
    const val CHEVRON_PX = 13.0

    /** Fraction of the viewport width the scale bar aims for before rounding to a 1/2/5. */
    const val SCALE_BAR_FRACTION = 0.22

    /** Radius of the marker an isolated one-point track segment degrades to. */
    const val TRACK_POINT_PX = 2.5

    /**
     * **How much of the flown track the frame is fitted around.**
     *
     * The recent window, not the whole track, and this is the load-bearing framing decision of
     * the whole picture: a long approach followed by a small orbit, fitted end to end, reduces a
     * 30 m orbit to about a tenth of the frame, at which point the manoeuvre you are watching is
     * a smudge. Measured both ways in `SituationTrackTest`: with a 500 m approach behind it, a
     * 30 m orbit spans **11.3 %** of the shorter axis fitted to the whole track and **50.0 %**
     * fitted to this window.
     *
     * **The unit is time**, [FlownTrack.RECENT_MS] — 120 s, Ivan's call on 2026-07-27 after
     * looking at the real thing: an operator counts history in seconds, and the 120 m this
     * replaced quietly showed less of a fast flight than of a slow one. [FlownTrack.RECENT_M]
     * survives as a floor under it, because the thinner works in distance and a hover keeps one
     * point — without it, holding station for two minutes would collapse the frame onto a dot.
     *
     * That floor is three times [Projection.MIN_SPAN_M], so the track can widen the frame but
     * never by an order of magnitude. The older track is still **drawn**; it simply does not get a
     * vote on the scale, and runs off the edge of the frame like every other moving map's history.
     */
    const val TRACK_RECENT_MS = FlownTrack.RECENT_MS

    /** The distance floor under [TRACK_RECENT_MS]. See [FlownTrack.RECENT_M]. */
    const val TRACK_RECENT_M = FlownTrack.RECENT_M

    /** The 1-2-5 ladder a scale bar is rounded down onto, in metres. */
    val SCALE_STEPS_M = listOf(
        1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0,
        1_000.0, 2_000.0, 5_000.0, 10_000.0,
    )

    /**
     * Everything that must be inside the frame — the input to [Projection.fit].
     *
     * Public because it is the answer to "why is the view zoomed like that", and because a test
     * can assert the framing decision without going near a projection.
     */
    fun extents(situation: Situation): List<Extent> {
        val out = ArrayList<Extent>(8)
        situation.aircraft?.let { out.add(Extent(it.fix.latDeg, it.fix.lonDeg)) }
        // Only the recent tail of the track, never the whole of it — see TRACK_RECENT_M. This is
        // the one extent that grows without limit if it is taken naively, and the failure is
        // quiet: everything stays on screen, and everything becomes too small to read.
        situation.track?.recentTail()
            ?.forEach { out.add(Extent(it.latDeg, it.lonDeg)) }
        // The circle, not the centre: an orbit fitted by its centre alone puts the flown path
        // outside the frame, which is the one framing mistake that would matter.
        situation.orbit?.let { out.add(Extent(it.centre.latDeg, it.centre.lonDeg, it.radiusM)) }
        situation.goto?.let { out.add(Extent(it.target.latDeg, it.target.lonDeg)) }
        situation.roi?.let { out.add(Extent(it.target.latDeg, it.target.lonDeg)) }
        situation.plan?.points?.forEach { out.add(Extent(it.fix.latDeg, it.fix.lonDeg)) }
        // Home is the one thing that does **not** always claim a place in the frame. On a
        // transit it can be kilometres away, and fitting it would shrink a 40 m orbit to a
        // smudge — losing the picture to gain a marker. So it joins the extents only when it
        // is the *only* thing known, and otherwise is drawn if and only if it happens to fall
        // inside the frame the rest of the picture chose (`build`, which has the scale).
        if (out.isEmpty()) situation.home?.let { out.add(Extent(it.latDeg, it.lonDeg)) }
        return out
    }

    /**
     * The finished picture, or a [Scene] with no shapes when nothing is known.
     *
     * An empty scene is a real and common answer — bridge stopped, aircraft off, no plan — and
     * it is returned rather than substituted for, so the `View` paints nothing but its notes and
     * an operator sees an honest blank instead of a map of somewhere.
     */
    fun build(situation: Situation, viewport: Viewport): Scene {
        val extents = extents(situation)
        val projection = Projection.fit(extents, viewport)
            ?: return Scene(
                emptyList(), situation.source, situation.notes,
                publishing = situation.publishing,
            )

        val shapes = ArrayList<Shape>(16)

        // ── the flown track, first and therefore furthest back ────────────────
        //
        // Context, not the subject: it is under the plan, under the orbit, under the aircraft
        // and under the operator's text, and it is the only thing here drawn in two inks — the
        // recent window bright, everything older receding. Each segment is its own path, and
        // **nothing joins two of them**, which is the entire point of the segment: a gap in
        // knowledge is left as a gap on the screen.
        situation.track?.let { track ->
            val recent = track.recentCount
            track.segments.forEachIndexed { i, segment ->
                if (segment.points.isEmpty()) return@forEachIndexed
                val newest = i == track.segments.lastIndex
                val screen = segment.points.map { projection.toScreen(it.latDeg, it.lonDeg) }
                if (screen.size == 1) {
                    // A single fix between two gaps. A one-point path draws nothing at all, and
                    // a position we genuinely had must not disappear because we did not get the
                    // next one — that would turn a dropout into a lie by omission.
                    shapes.add(
                        Shape.Dot(screen[0], TRACK_POINT_PX, if (newest) Ink.TRACK else Ink.TRACK_PAST),
                    )
                    return@forEachIndexed
                }
                if (!newest) {
                    shapes.add(Shape.Path(screen, Ink.TRACK_PAST))
                    return@forEachIndexed
                }
                // The window is measured along the path and never crosses a break, so the split
                // is always inside this, the newest segment. The newest leg is always in the
                // bright half — a track whose head is dim reads as history, and its head is now.
                val cut = (screen.size - maxOf(recent, 2)).coerceAtLeast(0)
                // The boundary point is in both halves, so the fade has no seam at it.
                if (cut >= 1) shapes.add(Shape.Path(screen.subList(0, cut + 1), Ink.TRACK_PAST))
                shapes.add(Shape.Path(screen.subList(cut, screen.size), Ink.TRACK))
            }
        }

        // ── home ─────────────────────────────────────────────────────────────
        // Included only if it lands inside the frame the rest of the picture chose. Refitting
        // to include a home 2 km away would compress an orbit to a dot; a home off screen is
        // better said in the status text than paid for with the whole picture.
        situation.home?.let { h ->
            val at = projection.toScreen(h.latDeg, h.lonDeg)
            if (inside(at, viewport)) {
                shapes.add(Shape.Dot(at, MARKER_PX, Ink.HOME, hollow = true))
                shapes.add(Shape.Label(at, "H", Ink.HOME))
            }
        }

        // ── the plan ─────────────────────────────────────────────────────────
        situation.plan?.let { plan ->
            val points = plan.points.map { projection.toScreen(it.fix.latDeg, it.fix.lonDeg) }
            if (points.size >= 2) {
                // Dashed unless something is actually flying it — which nothing is, today.
                shapes.add(Shape.Path(points, Ink.PLAN, dashed = !plan.flying))
            }
            plan.points.forEachIndexed { i, p ->
                val ink = if (plan.currentSeq != null && p.seq == plan.currentSeq) {
                    Ink.PLAN_CURRENT
                } else {
                    Ink.PLAN
                }
                shapes.add(Shape.Dot(points[i], MARKER_PX, ink))
                shapes.add(Shape.Label(points[i], "${p.seq}", ink))
            }
        }

        // ── the orbit ────────────────────────────────────────────────────────
        situation.orbit?.let { orb ->
            val centre = projection.toScreen(orb.centre.latDeg, orb.centre.lonDeg)
            val radiusPx = projection.lengthPx(orb.radiusM)
            shapes.add(Shape.Circle(centre, radiusPx, Ink.ORBIT))
            shapes.add(Shape.Dot(centre, MARKER_PX, Ink.ORBIT, hollow = true))
            shapes.add(chevron(centre, radiusPx, orb.direction))
        }

        // ── the goto ─────────────────────────────────────────────────────────
        situation.goto?.let { g ->
            val target = projection.toScreen(g.target.latDeg, g.target.lonDeg)
            // The leg only when we know where it starts. No aircraft, no line — a line drawn
            // from nowhere is a claim about a position we just refused to draw.
            situation.aircraft?.let { a ->
                val from = projection.toScreen(a.fix.latDeg, a.fix.lonDeg)
                // Solid while still going somewhere, dashed once it is only holding station:
                // an arrived goto is a place we are at, not a place we are heading for.
                shapes.add(Shape.Path(listOf(from, target), Ink.GOTO, dashed = g.arrived))
            }
            shapes.add(Shape.Dot(target, MARKER_PX, Ink.GOTO, hollow = true))
        }

        // ── the region of interest ───────────────────────────────────────────
        situation.roi?.let { r ->
            val ink = if (r.tracking) Ink.ROI else Ink.GHOST
            val target = projection.toScreen(r.target.latDeg, r.target.lonDeg)
            situation.aircraft?.let { a ->
                val from = projection.toScreen(a.fix.latDeg, a.fix.lonDeg)
                // The line *is* the claim "the camera is on that". It is drawn only while the
                // camera is genuinely being driven; a suspended ROI keeps its marker and loses
                // its line, because the camera stayed where it was and is no longer aimed.
                if (r.tracking) shapes.add(Shape.Path(listOf(from, target), Ink.ROI))
            }
            shapes.add(Shape.Dot(target, MARKER_PX, ink))
            shapes.add(Shape.Label(target, "ROI", ink))
        }

        // ── the aircraft, last, over everything ──────────────────────────────
        situation.aircraft?.let { a ->
            val at = projection.toScreen(a.fix.latDeg, a.fix.lonDeg)
            val heading = a.headingDeg
            if (heading != null) {
                shapes.add(Shape.Polygon(triangle(at, heading, AIRCRAFT_PX), Ink.AIRCRAFT))
            } else {
                // Degraded, not frozen: we know where it is and not which way it faces, so the
                // symbol says exactly that by having no direction to read off it.
                shapes.add(Shape.Dot(at, AIRCRAFT_DOT_PX, Ink.AIRCRAFT))
            }
        }

        val (barM, barPx) = scaleBar(projection, viewport)
        return Scene(
            shapes = shapes,
            source = situation.source,
            notes = situation.notes,
            scaleBarMetres = barM,
            scaleBarPx = barPx,
            publishing = situation.publishing,
        )
    }

    /**
     * The aircraft dart: nose along [headingDeg], two rear corners at ±[AIRCRAFT_SPREAD_DEG].
     *
     * Compass convention throughout — 0° is north, which on a north-up canvas is **−y**, and 90°
     * is east, which is +x. Hence `sin` on x and `−cos` on y. Getting that pair the wrong way
     * round produces a symbol that is wrong by a reflection and looks entirely plausible, which
     * is why `SituationSceneTest` pins all four cardinal headings rather than one.
     */
    fun triangle(at: ScreenPoint, headingDeg: Double, sizePx: Double): List<ScreenPoint> {
        val rear = sizePx * 0.72
        return listOf(
            offset(at, headingDeg, sizePx),
            offset(at, headingDeg + AIRCRAFT_SPREAD_DEG, rear),
            offset(at, headingDeg - AIRCRAFT_SPREAD_DEG, rear),
        )
    }

    /**
     * The chevron that says which way round the orbit goes, placed at the circle's north point
     * and pointing along the tangent there.
     *
     * The tangent is taken from the same construction `OrbitGuidance.circleVelocity` flies:
     * `tangent = (−radialEast, radialNorth) · direction`. At the north point the radial is
     * (1, 0), so the tangent is (0, direction) — due east for a clockwise circle. Deriving it
     * rather than hard-coding "right for clockwise" is what keeps the picture and the flown
     * circle from disagreeing if the sign convention is ever revisited.
     */
    fun chevron(centre: ScreenPoint, radiusPx: Double, direction: Int): Shape.Polygon {
        val sign = if (direction < 0) -1 else 1
        // North point of the circle on a north-up canvas.
        val tip = ScreenPoint(centre.x, centre.y - radiusPx)
        // Tangent bearing there: 90° (east) clockwise, 270° (west) anticlockwise.
        val bearing = if (sign > 0) 90.0 else 270.0
        return Shape.Polygon(
            listOf(
                offset(tip, bearing, CHEVRON_PX),
                offset(tip, bearing + 140.0, CHEVRON_PX * 0.8),
                offset(tip, bearing - 140.0, CHEVRON_PX * 0.8),
            ),
            Ink.ORBIT,
        )
    }

    /**
     * A round ground distance and its length on this canvas, rounded **down** onto the 1-2-5
     * ladder.
     *
     * Down rather than nearest, so the bar is never longer than the fraction of the frame it
     * was budgeted; a scale bar that overhangs the picture is worse than a short one. Null when
     * even the smallest step would not fit, which happens only at absurd scales.
     */
    fun scaleBar(projection: Projection, viewport: Viewport): Pair<Double?, Double?> {
        val budgetPx = viewport.widthPx * SCALE_BAR_FRACTION
        if (budgetPx <= 0.0) return null to null
        val budgetM = projection.metres(budgetPx)
        val step = SCALE_STEPS_M.lastOrNull { it <= budgetM } ?: return null to null
        return step to projection.lengthPx(step)
    }

    /** True when a point is within the viewport rectangle — the test `build` applies to home. */
    fun inside(p: ScreenPoint, viewport: Viewport): Boolean =
        p.x >= 0.0 && p.x <= viewport.widthPx && p.y >= 0.0 && p.y <= viewport.heightPx

    /** A point [lengthPx] away from [from] on compass bearing [bearingDeg], north-up canvas. */
    fun offset(from: ScreenPoint, bearingDeg: Double, lengthPx: Double): ScreenPoint {
        val rad = Math.toRadians(bearingDeg)
        return ScreenPoint(
            x = from.x + sin(rad) * lengthPx,
            y = from.y - cos(rad) * lengthPx,
        )
    }
}
