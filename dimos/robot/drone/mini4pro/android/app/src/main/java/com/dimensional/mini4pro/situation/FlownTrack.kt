package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.telemetry.Geo
import kotlin.math.hypot

/**
 * One unbroken run of positions the bridge actually knew, oldest first.
 *
 * The *segment* is the unit rather than the point because the only interesting property of a
 * flown track is where it is **not**: two consecutive points inside one segment were sampled
 * closely enough in time that the straight line between them is a fair account of the flight,
 * and two points in different segments were not. Nothing downstream may join them — see
 * [FlownTrack] for the rule that decides where a segment ends.
 */
data class TrackSegment(val points: List<Fix>)

/**
 * The flown track, as something drawable: the path so far, cut wherever knowledge was.
 *
 * A [Situation] field like any other, and value data like any other — a list of lists of
 * coordinates, with no clock, no buffer and nothing to call. The mutable accumulator that
 * produces it is [FlownTrack], and it is deliberately not this type: the picture is handed a
 * finished, immutable account of the past exactly as it is handed a finished account of now.
 */
data class TrackMark(
    val segments: List<TrackSegment>,
    /**
     * How many of the newest segment's trailing points count as **recent** — what the auto-fit
     * frames and what is drawn brightest. At least 1 whenever there is a point at all, and never
     * more than the newest segment holds.
     *
     * **A number, computed by [FlownTrack], rather than a method on this type.** The window is
     * primarily a span of *time* ([FlownTrack.RECENT_MS]) and this type deliberately has no clock:
     * it is a finished account of where the aircraft has been, handed to the picture exactly as
     * the rest of `Situation` is. The accumulator owns the clock, so the accumulator counts.
     */
    val recentCount: Int,
) {

    /** No segment carries a point. */
    val isEmpty: Boolean get() = segments.all { it.points.isEmpty() }

    /** Every point in every segment. The bound [FlownTrack.MAX_POINTS] applies to this. */
    val pointCount: Int get() = segments.sumOf { it.points.size }

    /** The most recent position on the track, or null when there is none. */
    val newest: Fix? get() = segments.lastOrNull()?.points?.lastOrNull()

    /** The points [recentCount] counted, in order. */
    fun recentTail(): List<Fix> {
        val points = segments.lastOrNull()?.points ?: return emptyList()
        return points.takeLast(recentCount)
    }

    companion object {
        /** A track with nothing in it. Never handed to a [Situation] — null is used for that. */
        val EMPTY = TrackMark(emptyList(), recentCount = 0)
    }
}

/**
 * **Where the aircraft has been** — the one thing in this package that remembers, and therefore
 * the one thing in it that can lie about more than a single instant.
 *
 * Pure Kotlin, no Android, no clock of its own: every call that advances anything takes the
 * current millisecond as an argument, exactly as `ReplayPlayer` does, so a twenty-minute flight
 * is testable by hand-cranking a number. `MainActivity` owns the ticking and hands each finished
 * [Situation] here on the drawing cadence it already runs at.
 *
 * ## The honesty rules, which are the whole of this file
 *
 * The rest of `situation/` refuses to draw what it does not know *now*. A track extends that
 * refusal backwards, where it is easier to get wrong, because a polyline is a claim about every
 * point *between* its vertices as well as about the vertices themselves.
 *
 *  1. **Only positions we actually knew become points.** [SituationReading] has already removed
 *     the aircraft symbol when `Signal.POSITION` is stale, and this file takes its input from
 *     the same place — [Situation.aircraft]. A sample with no fresh fix contributes no point,
 *     and it does not contribute a *time* either: [lastKnownMs] is left where it was, so a
 *     dropout accumulates towards rule 2 rather than being papered over.
 *  2. **A gap in knowledge is drawn as a gap.** Two consecutive known fixes continue the same
 *     segment only if they are within [GAP_MS] of one another. Beyond that the track breaks and
 *     the next fix starts a new segment, because a straight line across fifteen silent seconds
 *     is a picture of a flight that may never have happened — the aircraft could have gone
 *     anywhere and come back. This is measured between *known* samples and **not** between kept
 *     points: a stationary hover keeps no points for a minute while knowing exactly where the
 *     aircraft is the whole time, and breaking that would be the opposite mistake.
 *  3. **A step no aircraft could have flown is a break, not a leg.** [JUMP_M] within [GAP_MS] is
 *     beyond twice this airframe's sport-mode ceiling, so a step that large is not motion: it is
 *     a new flight, a datum change, a replay looping, or a fix this bridge should not have
 *     believed. Drawing the line would invent a transit across whatever lies between.
 *  4. **A long silence forgets rather than breaks.** Past [FORGET_MS] the track is *cleared*.
 *     A minute of not knowing where the aircraft is, on a twenty-five-minute battery, means what
 *     is on screen is very likely a different flight or a different site, and history from the
 *     last one has no business framing this one. See [FORGET_MS].
 *  5. **A change of source clears everything.** A live track and a replayed track are pictures of
 *     two different afternoons — the distinction `Situation.source` exists for — and mixing them
 *     into one polyline would be the single worst thing this file could produce.
 *
 * ## Thinning, and why by distance
 *
 * `StateCache` delivers position at roughly 12 Hz and this is asked at 5 Hz; a twenty-minute
 * flight is six thousand samples. Storing them all is a leak on a phone that is also flying an
 * aircraft, and it is not even a better picture: a hover produces thousands of points inside a
 * two-metre circle of GPS noise, and drawing them is drawing the noise.
 *
 * So a point is kept only when it is [STEP_M] from the last kept one — **by distance, never by
 * time**. A hover therefore costs one point no matter how long it lasts, and a fast leg is
 * sampled as densely as a slow one, which is the property a time-based thinner does not have.
 * At 5 m the worst-case chord error against a 20 m orbit is 16 cm, i.e. invisible at any scale
 * this view uses, and a 15 m/s transit still yields a point every third of a second.
 *
 * ## The bound
 *
 * A fixed ring of [MAX_POINTS] coordinates, allocated once, oldest dropped first. At [STEP_M]
 * that is at least 10 km of flown path — more than a Mini 4 Pro battery of ordinary flying — and
 * it costs 34 kB that never grows and never churns the collector. Dropping the *oldest* is the
 * right end to lose: it agrees with the fit window and with the fading, both of which already
 * say that the recent part of the track is the part doing work.
 */
class FlownTrack(
    /** Metres between kept points. See "thinning, and why by distance". */
    val stepM: Double = STEP_M,
    /** Longest silence that still continues a segment, ms. Rule 2. */
    val gapMs: Long = GAP_MS,
    /** Longest silence that is still the same flight, ms. Rule 4. */
    val forgetMs: Long = FORGET_MS,
    /** A step this large between consecutive known fixes is not flight. Rule 3. */
    val jumpM: Double = JUMP_M,
    /** Ring size. See "the bound". */
    val capacity: Int = MAX_POINTS,
    /** Seconds of flying that count as recent, ms. See [RECENT_MS]. */
    val recentMs: Long = RECENT_MS,
    /** The floor under the time window, metres of path. See [RECENT_M]. */
    val recentM: Double = RECENT_M,
) {

    private val lat = DoubleArray(capacity)
    private val lon = DoubleArray(capacity)

    /** When each point was sampled, for the recent window. See [RECENT_MS]. */
    private val times = LongArray(capacity)

    /** True when the point at this slot begins a new segment — i.e. a gap precedes it. */
    private val breaks = BooleanArray(capacity)

    /** Ring index of the oldest live point. */
    private var head = 0

    /** How many slots are live, `0..capacity`. */
    private var size = 0

    /** Monotonic time of the last sample that carried a fix. Null before the first one. */
    private var lastKnownMs: Long? = null

    /** Which picture the points belong to, so a change of it can clear them. Rule 5. */
    private var source: SituationSource? = null

    /** Rebuilt only when the ring changes, so an unchanging track allocates nothing per frame. */
    private var cached: TrackMark? = null

    /** Points currently held. Test-facing; the bound is asserted on it. */
    val pointCount: Int get() = size

    /**
     * Offers [situation] to the track and returns what should now be drawn, or null when there
     * is nothing.
     *
     * @param atMs the sample's own time in monotonic milliseconds — `SystemClock.elapsedRealtime`
     *   for a live picture, and the **recording's own** timestamp for a replay, so that a
     *   recording played at four times speed is still thinned and broken by the times the
     *   aircraft actually flew rather than by the operator's playback rate.
     */
    fun accept(situation: Situation, atMs: Long): TrackMark? {
        if (situation.source != source) {
            source = situation.source
            clear()
        }
        val fix = situation.aircraft?.fix
        // Rule 1. A sample with no fresh fix is not a point *and not a time*: leaving
        // lastKnownMs alone is what lets the dropout accumulate towards rules 2 and 4.
        if (fix == null) return mark()

        val since = lastKnownMs?.let { atMs - it }
        lastKnownMs = atMs
        when {
            // First fix of a picture, a clock that ran backwards (a replay looping), or a
            // silence long enough that this is probably not the same flight. Rule 4.
            since == null || since < 0L || since > forgetMs -> {
                clearPoints()
                add(fix, startsSegment = true, atMs = atMs)
            }
            // Rule 2: known too long ago to join up.
            since > gapMs -> add(fix, startsSegment = true, atMs = atMs)
            else -> {
                val moved = newestFix()?.let { metresBetween(it, fix) } ?: Double.MAX_VALUE
                when {
                    // Rule 3: too far for the time available.
                    moved > jumpM -> add(fix, startsSegment = true, atMs = atMs)
                    // Thinning: near enough to the last kept point to be the same place.
                    moved < stepM -> Unit
                    else -> add(fix, startsSegment = false, atMs = atMs)
                }
            }
        }
        return mark()
    }

    /**
     * Forgets everything, including which source the points came from.
     *
     * Called on a change of source by [accept], and explicitly by `MainActivity` when a *new*
     * recording is opened — two recordings in a row are both `REPLAY`, so nothing about the
     * source could tell them apart, and one flight's path drawn under another's is precisely
     * the confusion this feature is arranged against.
     */
    fun clear() {
        clearPoints()
        lastKnownMs = null
    }

    private fun clearPoints() {
        head = 0
        size = 0
        cached = null
    }

    private fun add(fix: Fix, startsSegment: Boolean, atMs: Long) {
        val slot = (head + size) % capacity
        lat[slot] = fix.latDeg
        lon[slot] = fix.lonDeg
        times[slot] = atMs
        breaks[slot] = startsSegment
        if (size == capacity) head = (head + 1) % capacity else size++
        cached = null
    }

    private fun newestFix(): Fix? {
        if (size == 0) return null
        val slot = (head + size - 1) % capacity
        return Fix(lat[slot], lon[slot])
    }

    /** The drawable account of the ring, rebuilt only when the ring has moved. */
    private fun mark(): TrackMark? {
        cached?.let { return it }
        if (size == 0) return null
        val segments = ArrayList<TrackSegment>(4)
        var current = ArrayList<Fix>(size.coerceAtMost(64))
        for (i in 0 until size) {
            val slot = (head + i) % capacity
            // The oldest live point always starts a segment: the ring may have dropped whatever
            // it used to continue, and a segment that begins at a point we no longer hold would
            // claim a leg from nowhere.
            if (i > 0 && breaks[slot]) {
                segments.add(TrackSegment(current))
                current = ArrayList(32)
            }
            current.add(Fix(lat[slot], lon[slot]))
        }
        segments.add(TrackSegment(current))
        val out = TrackMark(segments, recentCount = countRecent(current.size))
        cached = out
        return out
    }

    /**
     * How many of the newest segment's trailing points are **recent** — the window the auto-fit
     * frames and the drawing brightens.
     *
     * [newestSegmentSize] bounds the walk *and* the result, which is how *"never crosses a
     * break"* is enforced: beyond a break we do not know what the path was, so it cannot be part
     * of a window measured along the path. A track that has just resumed after a dropout therefore
     * frames itself on the resumed part, which is the part an operator cares about. The two are
     * belt and braces — mutation T3 removes the loop bound and nothing fails, because the
     * `coerceAtMost` still clips the answer. The loop bound stays as the cheaper of the two and
     * because a walk that steps across a break is wrong even when its result is then repaired.
     *
     * ## Mutation-checked 2026-07-27
     *
     * | # | mutation | kills |
     * |---|---|---|
     * | T1 | the time window ignored, distance floor only — the pre-2026-07-27 rule | 2 |
     * | T2 | the distance floor removed | **0, then 1** |
     * | T3 | the walk unbounded by the segment (`until size`) | **0, masked by `coerceAtMost`** |
     * | T4 | the age's lower bound dropped (`age <= recentMs`) | **0, prevented by `accept`** |
     * | T5 | `maxOf` replaced by `minOf` — the smaller window wins | 4 |
     *
     * T2 scored 0 first, and the reason was a wrong belief rather than a thin test: the floor was
     * justified as protecting a *hover*, and a hover needs no protecting, because this window is
     * anchored on the newest **point's** time and a stationary aircraft adds no points — so the
     * 120 s of flying that led up to the hover simply stays in the window for as long as the
     * aircraft sits there. What actually needs the floor is **moving off again**: the first step
     * re-anchors the window on a fresh timestamp and ages the whole approach out in one tick. The
     * test was rewritten around that, and a second one pins the hover behaviour it replaced so the
     * distinction cannot quietly rot.
     *
     * ## Time, with a distance floor under it
     *
     * `max(the last recentMs, the last recentM of path)`, and both halves earn their place.
     *
     * **Time is the operator's unit**, which is Ivan's call after watching it on the phone: 120 s
     * of history means the same thing to a person whatever the aircraft is doing, where 120 m
     * means two minutes on a slow orbit and forty seconds on a transit. Under a pure distance
     * window a fast leg silently shows less history than a slow one.
     *
     * **The distance floor exists because thinning is by distance and a hover keeps one point.**
     * Hover for longer than [recentMs] and a pure time window contains exactly that one point, so
     * the frame would collapse to a dot and the approach that led there would drop out of the
     * picture at the moment the aircraft stopped — the opposite of useful. The floor keeps the last
     * [recentM] visible however long the aircraft sits still. In flight it never binds: at 3 m/s
     * the time window is 360 m, three times the floor.
     *
     * At least 1 whenever the segment has a point: the newest point is always recent.
     */
    private fun countRecent(newestSegmentSize: Int): Int {
        if (newestSegmentSize == 0 || size == 0) return 0
        val newestSlot = (head + size - 1) % capacity
        val newestMs = times[newestSlot]
        var walked = 0.0
        var byTime = 1
        var byDistance = 1
        var timeStillOpen = true
        var distanceStillOpen = true
        for (back in 1 until newestSegmentSize) {
            val slot = (head + size - 1 - back + capacity) % capacity
            val next = (slot + 1) % capacity
            walked += metresBetween(Fix(lat[slot], lon[slot]), Fix(lat[next], lon[next]))
            // Written on the age, with the lower bound, rather than as `diff <= recentMs`. It is
            // belt-and-braces: `accept` clears the ring whenever the clock runs backwards, so the
            // stored times are monotonic within one mark and a negative age cannot arise. Measured
            // as such — mutation T4 scores 0 (table below) — and kept because the guarantee lives
            // in a different method, and a walk that reads times has no business assuming it.
            val age = newestMs - times[slot]
            if (timeStillOpen && age in 0..recentMs) byTime++ else timeStillOpen = false
            if (distanceStillOpen && walked <= recentM) byDistance++ else distanceStillOpen = false
            if (!timeStillOpen && !distanceStillOpen) break
        }
        return maxOf(byTime, byDistance).coerceAtMost(newestSegmentSize)
    }

    companion object {

        /**
         * Metres between kept points.
         *
         * 5 m. Above the metre-or-two of GPS noise a stationary aircraft produces — so a hover
         * genuinely costs one point — and small enough that the polygon it draws round a 20 m
         * orbit departs from the true circle by 16 cm, which is a fraction of a pixel at every
         * scale this view chooses.
         */
        const val STEP_M = 5.0

        /**
         * The longest silence two known fixes may span and still be joined by a line, ms.
         *
         * 2 s. `Signal.POSITION` carries a 1 s freshness limit against a measured ~12 Hz feed, so
         * a 2 s hole is at least two consecutive expired windows and can never be tripped by one
         * dropped sample. It is the deliberate compromise in this file: a line is still drawn
         * across up to 2 s of not knowing, which at 16 m/s is 32 m of assumption. Shorter and
         * ordinary telemetry jitter would shred the track into hundreds of one-point segments,
         * which is a *less* readable kind of dishonesty — a picture nobody can read says nothing
         * true either.
         */
        const val GAP_MS = 2_000L

        /**
         * The silence past which the track is forgotten entirely rather than broken, ms.
         *
         * 60 s. The aircraft was off, out of range, or the app was in the background with DJI Fly
         * in front for a minute — on a twenty-five-minute battery that is long enough that what
         * is on screen next is most likely a different flight, and drawing an old one behind it
         * would frame the new picture around a place the aircraft is not.
         */
        const val FORGET_MS = 60_000L

        /**
         * A step between consecutive known fixes that no flight could have made, metres.
         *
         * 60 m ≈ [GAP_MS] × 30 m/s, twice the Mini 4 Pro's 16 m/s sport-mode ceiling. Anything
         * larger inside the continuity window is not motion — it is a new flight, a datum change,
         * or a fix that should not have been believed — so it breaks the track instead of drawing
         * a transit across ground the aircraft never crossed.
         */
        const val JUMP_M = 60.0

        /**
         * Points held, at most. Fixed ring, allocated once, oldest dropped first.
         *
         * 2048 × [STEP_M] is at least 10.2 km of flown path, more than a battery of ordinary
         * flying, for 34 kB that never grows. The alternative — an unbounded list on a phone that
         * is also flying an aircraft — is a leak whose symptom would first appear on the longest
         * flight anyone had yet made.
         */
        const val MAX_POINTS = 2048

        /**
         * **Seconds of flying that count as *recent***: what the auto-fit frames, and what is drawn
         * brightest.
         *
         * 120 s, chosen by Ivan on 2026-07-27 after looking at the track on the phone — the picture
         * was right and the window was the wrong unit. The reasoning is that history is something an
         * operator counts in seconds: "the last two minutes" means the same thing whatever the
         * aircraft is doing, whereas the 120 m this replaced meant two minutes on a slow orbit and
         * forty seconds on a transit, so the faster the flight the less of it you were shown.
         *
         * At the 3 m/s envelope this is up to 360 m of path — well inside the [MAX_POINTS] ring,
         * which holds 10.2 km.
         */
        const val RECENT_MS = 120_000L

        /**
         * The floor under [RECENT_MS], in flown metres.
         *
         * Needed because the thinner works in distance and a hover keeps **one point** — but not
         * for the reason it first appears. During a hover nothing goes wrong: the window is
         * anchored on the newest *point's* time, and a stationary aircraft adds no points, so the
         * 120 s of flying that led up to the hover stays in view for as long as it sits there.
         * The problem is **moving off again**. The first step after a long hold stamps a fresh
         * time, the window re-anchors on it, and the entire approach ages out in a single tick —
         * leaving the frame fitted to the last few metres. This floor keeps 120 m of path visible
         * through that transition.
         *
         * In flight it never binds — at 3 m/s the time window is three times this — so it is a
         * floor rather than a second policy. It was the whole window until 2026-07-27, and the
         * framing measurement in `SituationTrackTest` is still quoted against it.
         */
        const val RECENT_M = 120.0

        /**
         * Ground distance between two fixes, metres.
         *
         * Through [Geo.nedMetres], which is the bridge's single copy of the `cos(latitude)`
         * term — nothing in this package recomputes it, on purpose. At 38°N a track thinned
         * without that factor would keep points every 5 m north and every 6.3 m east, which is
         * not a bug anyone would ever see and is exactly the class of mistake `Geo` exists to
         * make impossible.
         */
        fun metresBetween(a: Fix, b: Fix): Double {
            val (north, east) = Geo.nedMetres(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg)
            return hypot(north, east)
        }
    }
}
