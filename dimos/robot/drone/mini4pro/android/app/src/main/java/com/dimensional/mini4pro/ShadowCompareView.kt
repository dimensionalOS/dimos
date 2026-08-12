package com.dimensional.mini4pro

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import com.dimensional.mini4pro.guided.GuidedEnvelope
import com.dimensional.mini4pro.guided.GuidedStickEngine
import com.dimensional.mini4pro.guided.StickVelocities
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt

/**
 * **The shadow-validation trust instrument**: two 3-D velocity arrows from one origin — the
 * operator's RC sticks and the shadow controller's would-be command — drawn in an oblique
 * projection chosen so the **vertical** component is visible as an angled line instead of
 * vanishing into the screen plane. Ivan's ask, verbatim: *"represent it as arrows... at an
 * angle so we can see the downward arrow as well. Both my controls and shadow's controls at
 * the same time to compare."*
 *
 * ## What it draws, and in whose units
 *
 * Both arrows are envelope-scaled NE/vertical velocities out of
 * [GuidedStickEngine.shadowComparison]: the operator's side goes through
 * `StickMapping.rcVelocities` — **the** mapping, so the comparison cannot lie by using a
 * second one — and is rotated from the sticks' body frame into north/east by the aircraft's
 * heading, inside the engine's snapshot, before this view ever sees it. Full scale on the
 * horizontal ring is [GuidedEnvelope.HORIZONTAL_MAX_MS] (3 m/s); the vertical axis is drawn
 * against [GuidedEnvelope.VERTICAL_MAX_MS] (1.5 m/s) over the same ring radius, so a
 * max-descent line and a max-lateral line are the same length on screen.
 *
 * The at-a-glance cue: the 3-D angle between the two vectors, shown in degrees when both are
 * moving, with the shadow arrow tinted red past [DIVERGE_DEG] — agreement is the whole thing
 * the operator is out there to see, and one number plus one colour says it.
 *
 * ## What it can never do
 *
 * **Display-only, by construction.** It holds a flat [GuidedStickEngine.ShadowComparison]
 * with no method that reaches the engine; nothing in the controller knows this view exists;
 * and it is visible **whenever shadow mode is on** — while the gates block a segment it
 * prints the current blocker where the arrows will appear, because on 2026-07-28 (flight
 * 152922) an operator staring at empty screen space could not tell shadow-broken from
 * shadow-blocked. In live mode it is GONE — the operator's sticks are a cancel there, so the
 * pair this view compares cannot coexist, and its absence is the truthful signal of which
 * mode is running. A shadow command older than the engine's own staleness bound arrives here
 * as null and is drawn as **absence** (greyed label, no arrow), never as a frozen arrow — a
 * frozen arrow reads as a live opinion.
 */
class ShadowCompareView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    /** The frame to draw. Set from the 200 ms render tick; null clears the panel. */
    var comparison: GuidedStickEngine.ShadowComparison? = null
        set(value) {
            field = value
            invalidate()
        }

    private companion object {
        /** Beyond this many degrees between the vectors the shadow arrow tints red. */
        const val DIVERGE_DEG = 45.0

        /** Below this speed (m/s) a vector has no meaningful direction and no angle is shown. */
        const val MOVING_MS = 0.15

        const val OPERATOR_COLOR = 0xFF58D68D.toInt() // green — the hand
        const val SHADOW_COLOR = 0xFF5DADE2.toInt() // blue — the controller
        const val SHADOW_DIVERGED = 0xFFE74C3C.toInt() // red — the controller, disagreeing
        const val AXIS_COLOR = 0x50FFFFFF
        const val TEXT_COLOR = 0xC0FFFFFF.toInt()
        const val MUTED_COLOR = 0x80FFFFFF.toInt()
        const val PANEL_COLOR = 0xB0101418.toInt()

        // The oblique basis, screen units per unit of world axis. East right; north up-right
        // at a slope, so lateral motion reads like the map; **up steeply up**, so the descent
        // component is an unmistakable angled line — the reason this projection exists.
        const val EAST_X = 1.0f
        const val EAST_Y = 0.0f
        const val NORTH_X = 0.45f
        const val NORTH_Y = -0.32f
        const val UP_X = 0.0f
        const val UP_Y = -0.90f
    }

    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)

    override fun onDraw(canvas: Canvas) {
        val cmp = comparison ?: return
        val w = width.toFloat()
        val h = height.toFloat()
        val cx = w * 0.46f
        val cy = h * 0.56f
        val ring = min(w, h) * 0.36f

        paint.style = Paint.Style.FILL
        paint.color = PANEL_COLOR
        canvas.drawRoundRect(0f, 0f, w, h, 12f, 12f, paint)

        // The axes, faint, with their letters — the frame both arrows are drawn in.
        paint.strokeWidth = 2f
        paint.textSize = h * 0.09f
        axis(canvas, cx, cy, ring, EAST_X, EAST_Y, "E")
        axis(canvas, cx, cy, ring, NORTH_X, NORTH_Y, "N")
        axis(canvas, cx, cy, ring, UP_X, UP_Y, "U")

        val you = cmp.operator
        val shadow = cmp.shadow
        val angle = angleDeg(you, shadow)
        val shadowColor =
            if (angle != null && angle > DIVERGE_DEG) SHADOW_DIVERGED else SHADOW_COLOR

        // The operator first, the shadow on top: the question is what the controller says.
        if (you != null) arrow(canvas, cx, cy, ring, you, OPERATOR_COLOR)
        if (shadow != null) arrow(canvas, cx, cy, ring, shadow, shadowColor)

        // The legend, colour-keyed, with absence written out rather than left blank.
        paint.textSize = h * 0.10f
        paint.style = Paint.Style.FILL
        paint.color = if (you != null) OPERATOR_COLOR else MUTED_COLOR
        canvas.drawText(if (you != null) "YOU" else "YOU —", 8f, h * 0.13f, paint)
        paint.color = if (shadow != null) shadowColor else MUTED_COLOR
        canvas.drawText(
            when {
                shadow != null -> "SHADOW"
                cmp.segmentArmed -> "SHADOW …"
                else -> "SHADOW gated"
            },
            8f, h * 0.25f, paint,
        )
        // The blocker, written where the arrows will appear: an operator staring at empty
        // screen space cannot tell shadow-broken from shadow-blocked, and on 2026-07-28 that
        // ambiguity is exactly what flew (mode on above the band, tag out of view, nothing on
        // screen). The engine already records blocker *changes*; this is the current one.
        cmp.blocker?.let { blocker ->
            paint.color = MUTED_COLOR
            paint.textSize = h * 0.085f
            canvas.drawText("shadow: $blocker", 8f, h * 0.40f, paint)
        }
        paint.color = TEXT_COLOR
        if (angle != null) {
            canvas.drawText("Δ %.0f°".format(angle), 8f, h * 0.96f, paint)
        }
        paint.color = MUTED_COLOR
        paint.textSize = h * 0.075f
        canvas.drawText(
            "ring %.0f m/s".format(GuidedEnvelope.HORIZONTAL_MAX_MS), w * 0.55f, h * 0.96f, paint,
        )
    }

    private fun axis(
        canvas: Canvas, cx: Float, cy: Float, ring: Float, ux: Float, uy: Float, label: String,
    ) {
        paint.style = Paint.Style.STROKE
        paint.color = AXIS_COLOR
        val ex = cx + ux * ring
        val ey = cy + uy * ring
        canvas.drawLine(cx, cy, ex, ey, paint)
        paint.style = Paint.Style.FILL
        canvas.drawText(label, ex + 4f, ey - 4f, paint)
    }

    /** One velocity as an oblique 3-D arrow. Horizontal against 3 m/s, vertical against 1.5. */
    private fun arrow(
        canvas: Canvas, cx: Float, cy: Float, ring: Float, v: StickVelocities, color: Int,
    ) {
        val n = (v.north / GuidedEnvelope.HORIZONTAL_MAX_MS).toFloat()
        val e = (v.east / GuidedEnvelope.HORIZONTAL_MAX_MS).toFloat()
        val up = (-v.down / GuidedEnvelope.VERTICAL_MAX_MS).toFloat()
        val x = cx + (e * EAST_X + n * NORTH_X + up * UP_X) * ring
        val y = cy + (e * EAST_Y + n * NORTH_Y + up * UP_Y) * ring
        paint.color = color
        paint.strokeWidth = 5f
        paint.style = Paint.Style.STROKE
        canvas.drawLine(cx, cy, x, y, paint)
        // The vertical component's ground shadow: the lateral part alone, thin — which is what
        // makes a descending arrow readable as "down" rather than as a shorter lateral one.
        paint.strokeWidth = 2f
        val gx = cx + (e * EAST_X + n * NORTH_X) * ring
        val gy = cy + (e * EAST_Y + n * NORTH_Y) * ring
        canvas.drawLine(gx, gy, x, y, paint)
        paint.style = Paint.Style.FILL
        canvas.drawCircle(x, y, 6f, paint)
    }

    /** The 3-D angle between the two commands, degrees, or null when either is not moving. */
    private fun angleDeg(a: StickVelocities?, b: StickVelocities?): Double? {
        if (a == null || b == null) return null
        val magA = magnitude(a)
        val magB = magnitude(b)
        if (magA < MOVING_MS || magB < MOVING_MS) return null
        val dot = a.north * b.north + a.east * b.east + a.down * b.down
        val cos = (dot / (magA * magB)).coerceIn(-1.0, 1.0)
        return Math.toDegrees(acos(cos))
    }

    private fun magnitude(v: StickVelocities): Double =
        sqrt(v.north * v.north + v.east * v.east + v.down * v.down)
}
