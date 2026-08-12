package com.dimensional.mini4pro

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import com.dimensional.mini4pro.situation.Ink
import com.dimensional.mini4pro.situation.Scene
import com.dimensional.mini4pro.situation.Shape
import com.dimensional.mini4pro.situation.Situation
import com.dimensional.mini4pro.situation.SituationScene
import com.dimensional.mini4pro.replay.ReplayPublication
import com.dimensional.mini4pro.situation.SituationSource
import com.dimensional.mini4pro.situation.Viewport

/**
 * The situation picture, painted **behind** the status text.
 *
 * Deliberately the dumbest class in this feature — the same way `KeyManagerActionPort` is the
 * dumbest class in the command path. It holds a [Situation], asks `SituationScene` what that
 * means in pixels, and strokes what it is handed. There is no geodesy here, no freshness rule,
 * no decision about what may be drawn, and consequently nothing in this file that would need a
 * phone to test: all of that lives in `situation/`, on the JVM, under
 * `ProjectionTest` / `SituationReadingTest` / `SituationSceneTest`.
 *
 * The one thing it *does* own is the palette, because a colour is a resource and a resource
 * needs a `Context`. [Ink] carries the meaning; [colourOf] is the only mapping.
 *
 * ## Behind the text, and readable through it
 *
 * The text is the operator's primary channel and stays that way: everything here is drawn at
 * low alpha over the terminal background, and nothing is filled solid except the aircraft
 * symbol itself. The picture is context; the words are the message.
 *
 * ## Cadence
 *
 * This view never polls, never subscribes and never times anything. `MainActivity` hands it a
 * [Situation] on the cadence it already renders the status text at, and each hand-off is one
 * `invalidate`. Nothing here can stall the 25 Hz setpoint loop or a DJI callback because
 * nothing here is ever called from either.
 */
class SituationView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0,
) : View(context, attrs, defStyle) {

    /**
     * What to draw. Assigning triggers exactly one repaint.
     *
     * There is no setter that takes an `AircraftState`, a `Bridge`, or anything else that could
     * make this view a second source of truth — see `situation/Situation`. It is handed a
     * finished, already-vetted picture or it is handed nothing.
     */
    var situation: Situation = Situation.UNKNOWN
        set(value) {
            field = value
            invalidate()
        }

    private val density = resources.displayMetrics.density

    private val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 2.0f * density
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 11.0f * density
        isFakeBoldText = true
    }
    private val dash = DashPathEffect(floatArrayOf(9.0f * density, 7.0f * density), 0f)
    private val path = Path()

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val viewport = Viewport(
            widthPx = width.toDouble(),
            heightPx = height.toDouble(),
            padPx = (PAD_DP * density).toDouble(),
        )
        val scene = SituationScene.build(situation, viewport)
        for (shape in scene.shapes) paint(canvas, shape)
        paintScaleBar(canvas, scene)
        if (scene.source == SituationSource.REPLAY) paintReplayWash(canvas, scene.publishing)
    }

    private fun paint(canvas: Canvas, shape: Shape) {
        val colour = colourOf(shape.ink)
        // One shared stroke paint, so every shape that uses it sets its own width rather than
        // inheriting the previous shape's.
        stroke.strokeWidth = strokeWidthOf(shape.ink)
        when (shape) {
            is Shape.Path -> {
                if (shape.points.size < 2) return
                stroke.color = colour
                stroke.alpha = alphaOf(shape.ink)
                stroke.pathEffect = if (shape.dashed) dash else null
                path.reset()
                path.moveTo(shape.points[0].x.toFloat(), shape.points[0].y.toFloat())
                for (i in 1 until shape.points.size) {
                    path.lineTo(shape.points[i].x.toFloat(), shape.points[i].y.toFloat())
                }
                canvas.drawPath(path, stroke)
                stroke.pathEffect = null
            }

            is Shape.Polygon -> {
                if (shape.points.size < 3) return
                fill.color = colour
                fill.alpha = alphaOf(shape.ink)
                path.reset()
                path.moveTo(shape.points[0].x.toFloat(), shape.points[0].y.toFloat())
                for (i in 1 until shape.points.size) {
                    path.lineTo(shape.points[i].x.toFloat(), shape.points[i].y.toFloat())
                }
                path.close()
                canvas.drawPath(path, fill)
            }

            is Shape.Circle -> {
                stroke.color = colour
                stroke.alpha = alphaOf(shape.ink)
                stroke.pathEffect = if (shape.dashed) dash else null
                canvas.drawCircle(
                    shape.centre.x.toFloat(),
                    shape.centre.y.toFloat(),
                    shape.radiusPx.toFloat(),
                    stroke,
                )
                stroke.pathEffect = null
            }

            is Shape.Dot -> {
                val paint = if (shape.hollow) stroke else fill
                paint.color = colour
                paint.alpha = alphaOf(shape.ink)
                canvas.drawCircle(
                    shape.centre.x.toFloat(),
                    shape.centre.y.toFloat(),
                    (shape.radiusPx * density / DESIGN_DENSITY).toFloat(),
                    paint,
                )
            }

            is Shape.Label -> {
                text.color = colour
                text.alpha = alphaOf(shape.ink)
                canvas.drawText(
                    shape.text,
                    (shape.at.x + LABEL_OFFSET_DP * density).toFloat(),
                    (shape.at.y + LABEL_OFFSET_DP * density).toFloat() + text.textSize,
                    text,
                )
            }
        }
    }

    /** Bottom-left, out of the status text's way, so the auto-scaled frame has a size to it. */
    private fun paintScaleBar(canvas: Canvas, scene: Scene) {
        val metres = scene.scaleBarMetres ?: return
        val lengthPx = (scene.scaleBarPx ?: return).toFloat()
        val y = height - BAR_MARGIN_DP * density
        val x = BAR_MARGIN_DP * density
        stroke.strokeWidth = strokeWidthOf(Ink.CHROME)
        stroke.color = colourOf(Ink.CHROME)
        stroke.alpha = alphaOf(Ink.CHROME)
        canvas.drawLine(x, y, x + lengthPx, y, stroke)
        canvas.drawLine(x, y - 5f * density, x, y + 5f * density, stroke)
        canvas.drawLine(x + lengthPx, y - 5f * density, x + lengthPx, y + 5f * density, stroke)
        text.color = colourOf(Ink.CHROME)
        text.alpha = alphaOf(Ink.CHROME)
        val label = if (metres >= 1000.0) "${(metres / 1000.0).toInt()} km" else "${metres.toInt()} m"
        canvas.drawText(label, x, y - 8f * density, text)
    }

    /**
     * The replay watermark: repeated diagonal text across the whole picture.
     *
     * Loud on purpose, and *inside the picture* rather than beside it. A banner above the frame
     * can be scrolled past, cropped by a screenshot, or simply not looked at; a word written
     * across the aircraft symbol cannot be. This is the third of three independent indicators —
     * the command strip changes colour and wording, the withdrawal controls grey out, and this.
     * Any one of them alone is a thing an operator can miss.
     */
    private fun paintReplayWash(canvas: Canvas, publishing: Boolean) {
        text.color = colourOf(Ink.REPLAY)
        // Denser and darker while the recording is going out to a ground station or a bus. The
        // wash is the one indicator that cannot be scrolled past or cropped out of a screenshot,
        // so it is where the difference between "on my screen" and "on somebody else's" is said
        // loudest. `ReplayPublication` owns the wording; this owns how hard it is to miss.
        text.alpha = if (publishing) REPLAY_WASH_ALPHA_PUBLISHING else REPLAY_WASH_ALPHA
        val size = 26f * density
        text.textSize = size
        canvas.save()
        canvas.rotate(-24f, width / 2f, height / 2f)
        var y = -height.toFloat()
        while (y < height * 2f) {
            var x = -width.toFloat()
            while (x < width * 2f) {
                canvas.drawText(ReplayPublication.watermark(publishing), x, y, text)
                x += size * 12f
            }
            y += size * 3.2f
        }
        canvas.restore()
        text.textSize = 11.0f * density
    }

    private fun colourOf(ink: Ink): Int = when (ink) {
        Ink.AIRCRAFT -> Color.parseColor("#7FD4A8")
        // The track is the aircraft's own history, so it is the aircraft's own colour, cooled
        // and — through alphaOf — much fainter. Same hue family, so it reads as belonging to the
        // symbol at the head of it without ever competing with it.
        Ink.TRACK -> Color.parseColor("#6FBE96")
        Ink.TRACK_PAST -> Color.parseColor("#5E8C7C")
        Ink.GHOST -> Color.parseColor("#7A8894")
        Ink.ORBIT -> Color.parseColor("#6FA8DC")
        Ink.GOTO -> Color.parseColor("#E0B25B")
        Ink.PLAN -> Color.parseColor("#9C87C4")
        Ink.PLAN_CURRENT -> Color.parseColor("#D0BCF0")
        Ink.ROI -> Color.parseColor("#D98C6A")
        Ink.HOME -> Color.parseColor("#8FA3B0")
        Ink.REPLAY -> Color.parseColor("#C77DD8")
        Ink.CHROME -> Color.parseColor("#5C6B77")
    }

    /**
     * How present each ink is. The picture sits *under* the operator's primary channel, so
     * everything is deliberately dim; the aircraft is the only symbol allowed to be nearly
     * solid, because it is the thing being looked for.
     */
    private fun alphaOf(ink: Ink): Int = when (ink) {
        Ink.AIRCRAFT -> 235
        Ink.GHOST -> 110
        Ink.CHROME -> 130
        Ink.REPLAY -> 200
        // Dimmer than anything else on the screen, and the older half dimmer again. The track is
        // the longest thing in the picture and would otherwise be the most visible; it sits under
        // the status text, which is the operator's primary channel and stays that way.
        Ink.TRACK -> 120
        Ink.TRACK_PAST -> 60
        else -> 170
    }

    /**
     * How wide each ink strokes. Only the track differs: it is a long line rather than a symbol,
     * so it is drawn thinner as well as fainter — a hairline of history behind a solid present.
     */
    private fun strokeWidthOf(ink: Ink): Float = when (ink) {
        Ink.TRACK, Ink.TRACK_PAST -> 1.4f * density
        else -> 2.0f * density
    }

    private companion object {
        const val PAD_DP = 28f
        const val BAR_MARGIN_DP = 14f
        const val LABEL_OFFSET_DP = 7f

        /** `Shape.Dot`'s radii are quoted in the scene's own design pixels; this rescales them. */
        const val DESIGN_DENSITY = 2.0f

        /**
         * The quiet watermark, kept here so the existing view tests read one constant.
         *
         * The text itself now comes from `replay/ReplayPublication`, which owns both wordings
         * because the published one is also the sentence the bus and the banner carry, and three
         * copies of a sentence is two too many.
         */
        const val REPLAY_WATERMARK = ReplayPublication.WATERMARK_QUIET

        const val REPLAY_WASH_ALPHA = 46

        /** Nearly twice as present, for a recording that has left the phone. See [paintReplayWash]. */
        const val REPLAY_WASH_ALPHA_PUBLISHING = 84
    }
}
