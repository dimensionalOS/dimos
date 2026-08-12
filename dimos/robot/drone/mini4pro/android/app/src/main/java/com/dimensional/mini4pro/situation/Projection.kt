package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.telemetry.Geo

/**
 * The rectangle the picture is painted into, in pixels.
 *
 * [padPx] is inset on all four sides before anything is fitted, so a symbol at the extreme
 * edge of the drawn world still has room for its own body — an aircraft triangle centred
 * exactly on the boundary would otherwise be half outside the view.
 */
data class Viewport(
    val widthPx: Double,
    val heightPx: Double,
    val padPx: Double = 0.0,
) {
    /** Drawable width after padding; never negative. */
    val usableWidthPx: Double get() = (widthPx - 2.0 * padPx).coerceAtLeast(0.0)

    /** Drawable height after padding; never negative. */
    val usableHeightPx: Double get() = (heightPx - 2.0 * padPx).coerceAtLeast(0.0)

    /** True when there is any room at all to draw in. */
    val isUsable: Boolean get() = usableWidthPx > 0.0 && usableHeightPx > 0.0
}

/** A point on the canvas. Origin top-left, x right, y **down** — Android's convention. */
data class ScreenPoint(val x: Double, val y: Double)

/**
 * One thing that must end up inside the picture: a place, and how far around it the drawing
 * of that thing reaches.
 *
 * [radiusM] is what makes an orbit fit. A circle is not its centre — fitting only the centre
 * point puts the operator's actual flight path off-screen, which is the one framing error
 * that would matter here.
 */
data class Extent(val latDeg: Double, val lonDeg: Double, val radiusM: Double = 0.0)

/**
 * North-up, top-down, equirectangular world→screen projection with a **uniform** scale on both
 * axes, and the auto-fit that chooses it.
 *
 * Pure Kotlin: no Android, no DJI, no `View`. The whole of this file is arithmetic over
 * [Geo], which is the bridge's single copy of the `cos(latitude)` term — **nothing here
 * recomputes it**, on purpose (`telemetry/Geo.longitudeScale` documents why that term is this
 * project's most expensive recurring mistake: zero error at the equator, 21 % east error at
 * the home site).
 *
 * ## Why the scale is uniform
 *
 * A circle stays a circle. The orbit is the one geometry on this screen whose *shape* carries
 * information — a stretched ellipse would read as a manoeuvre nobody commanded — so the fit
 * takes the worse of the two axis ratios rather than filling the rectangle. The cost is empty
 * space beside a long thin plan, which is honest; the alternative is a picture that lies about
 * the shape of the path.
 *
 * ## Why there is a minimum span
 *
 * With one known point and nothing else, the bounding box is zero metres across and the scale
 * is a division by zero — or, worse, a finite but absurd number that zooms a stationary hover
 * to centimetres per pixel and turns GPS noise into a picture of violent motion. [MIN_SPAN_M]
 * floors the shorter screen axis at 40 m, so a hovering aircraft sits still in a frame with a
 * believable size to it.
 */
class Projection private constructor(
    /** Latitude the canvas centre corresponds to. */
    val centreLatDeg: Double,
    /** Longitude the canvas centre corresponds to. */
    val centreLonDeg: Double,
    /** Ground metres per pixel — the same on both axes, see the class doc. */
    val metresPerPx: Double,
    val viewport: Viewport,
) {

    /**
     * Where a coordinate lands on the canvas. North is up, east is right.
     *
     * The offset is taken from the projection *centre*, so [Geo.nedMetres]'s `cos(latitude)`
     * is evaluated at one fixed latitude for every point in the picture. That is what keeps
     * the scale uniform across the frame — evaluating it per point would shear the picture.
     */
    fun toScreen(latDeg: Double, lonDeg: Double): ScreenPoint {
        val (north, east) = Geo.nedMetres(centreLatDeg, centreLonDeg, latDeg, lonDeg)
        return ScreenPoint(
            x = viewport.widthPx / 2.0 + east / metresPerPx,
            y = viewport.heightPx / 2.0 - north / metresPerPx,
        )
    }

    /** A ground distance as a length on the canvas. An orbit radius, an accept radius. */
    fun lengthPx(metres: Double): Double = metres / metresPerPx

    /** A canvas length back as ground metres — what the scale bar is labelled from. */
    fun metres(lengthPx: Double): Double = lengthPx * metresPerPx

    companion object {

        /**
         * The narrowest ground span the shorter screen axis is ever scaled to, in metres.
         *
         * 40 m rather than something tighter because the two single-point cases are a hover
         * and a plan of one waypoint, and at 40 m across the frame a metre of GPS jitter is a
         * couple of pixels rather than a lurch. It is a *floor*, so any genuine spread larger
         * than this wins.
         */
        const val MIN_SPAN_M = 40.0

        /**
         * The projection that fits every [extents] entry inside [viewport], or **null when
         * there is nothing to draw** — an empty list, or a viewport with no room in it.
         *
         * Null rather than an identity projection on purpose: "we know of no place" and "we
         * know of a place and it is here" are the two things this whole feature exists not to
         * confuse, and a caller that gets null has nothing it could accidentally paint.
         */
        fun fit(extents: List<Extent>, viewport: Viewport): Projection? {
            if (extents.isEmpty() || !viewport.isUsable) return null
            // The first *usable* extent, not the first one: an unreadable coordinate at the head
            // of the list must cost that one symbol, never the whole picture.
            val origin = extents.firstOrNull { it.latDeg.isFinite() && it.lonDeg.isFinite() }
                ?: return null
            var minNorth = Double.POSITIVE_INFINITY
            var maxNorth = Double.NEGATIVE_INFINITY
            var minEast = Double.POSITIVE_INFINITY
            var maxEast = Double.NEGATIVE_INFINITY
            for (e in extents) {
                if (!e.latDeg.isFinite() || !e.lonDeg.isFinite()) continue
                val r = if (e.radiusM.isFinite() && e.radiusM > 0.0) e.radiusM else 0.0
                val (north, east) = Geo.nedMetres(origin.latDeg, origin.lonDeg, e.latDeg, e.lonDeg)
                if (north - r < minNorth) minNorth = north - r
                if (north + r > maxNorth) maxNorth = north + r
                if (east - r < minEast) minEast = east - r
                if (east + r > maxEast) maxEast = east + r
            }
            if (!minNorth.isFinite() || !minEast.isFinite()) return null

            val centreNorth = (minNorth + maxNorth) / 2.0
            val centreEast = (minEast + maxEast) / 2.0
            val (centreLat, centreLon) =
                Geo.offsetCoordinate(origin.latDeg, origin.lonDeg, centreNorth, centreEast)

            val spanNorth = maxNorth - minNorth
            val spanEast = maxEast - minEast
            val w = viewport.usableWidthPx
            val h = viewport.usableHeightPx
            // The worse of the two ratios — see "why the scale is uniform" — floored so a
            // single point cannot divide by zero.
            val metresPerPx = maxOf(
                spanEast / w,
                spanNorth / h,
                MIN_SPAN_M / minOf(w, h),
            )
            if (!metresPerPx.isFinite() || metresPerPx <= 0.0) return null
            return Projection(centreLat, centreLon, metresPerPx, viewport)
        }
    }
}
