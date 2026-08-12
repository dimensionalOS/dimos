package com.dimensional.mini4pro.replay

/**
 * A cursor over a replayed flight — **play, pause, seek, and where we are**, with no clock of
 * its own.
 *
 * Pure Kotlin: every method that advances time takes the current monotonic millisecond as an
 * argument, so the whole of this class is testable by hand-cranking a number, exactly as
 * `GuidedStickEngine` is. The Android side owns the actual ticking.
 *
 * It holds [samples] — the output of [FlightReplay.samples], which reconstructs each instant's
 * `AircraftState` **including its recorded staleness**. That last part is what makes a replayed
 * picture worth looking at: the situation view's honesty rules see the same stale altitude and
 * the same silent attitude key the aircraft actually had, so the symbols drop out of the picture
 * at the same moments they did in flight rather than sliding smoothly through a fiction.
 *
 * ## What this class cannot do
 *
 * It has no reference to anything that commands. It reads a list and returns entries from it.
 * The safety argument for replay is not built here — it is built by the *type* of what leaves:
 * see `situation/Situation` and `ReplayAdmission`.
 */
class ReplayPlayer(
    /** In time order, as [FlightReplay.samples] produces them. May be empty. */
    val samples: List<ReplaySample>,
    /** The recording's own name, for the indicator that must never be missable. */
    val name: String = "",
    /**
     * The tag detector's sightings, in time order, as [FlightReplay.sightings] produces them.
     *
     * A second stream rather than a column on [samples], for the reason [TimedSighting] gives:
     * they arrive at a different rate, for a few seconds of a flight, and never otherwise.
     * Defaulted empty so every existing caller and test is untouched, and so a recording made
     * before the detector existed is simply a replay with no detections.
     */
    val sightings: List<TimedSighting> = emptyList(),
) {

    /** Seconds from the first sample to the last; 0 for an empty or single-sample record. */
    val durationSeconds: Double =
        if (samples.size < 2) 0.0 else samples.last().tSeconds - samples.first().tSeconds

    /** The record's own `t` of the first sample — the offset [positionSeconds] is measured from. */
    private val startSeconds: Double = samples.firstOrNull()?.tSeconds ?: 0.0

    /** Where the cursor is, in seconds from the start of the recording. */
    var positionSeconds: Double = 0.0
        private set

    /** True while [onClock] advances the cursor. Starts **false**: a replay is opened, then run. */
    var playing: Boolean = false
        private set

    /** Playback rate. 1.0 is real time; the UI offers a small set of multiples. */
    var rate: Double = 1.0
        private set

    private var lastClockMs: Long? = null

    /** True once the cursor has reached the end and stopped there. */
    val atEnd: Boolean get() = positionSeconds >= durationSeconds && durationSeconds > 0.0

    /** Nothing to replay — an empty file, or one with no `dji_state` lines in it. */
    val isEmpty: Boolean get() = samples.isEmpty()

    /**
     * The sample in force at the cursor: the **last one at or before** it, never an
     * interpolation between two.
     *
     * Interpolation would be inventing a state the aircraft never reported, at the exact moment
     * the recorded staleness says we had no fresh reading — which is the one thing this whole
     * replay path exists not to do. A held sample is what the app itself was looking at.
     */
    fun current(): ReplaySample? {
        if (samples.isEmpty()) return null
        val t = startSeconds + positionSeconds
        var lo = 0
        var hi = samples.size - 1
        var found = 0
        while (lo <= hi) {
            val mid = (lo + hi) ushr 1
            if (samples[mid].tSeconds <= t) {
                found = mid
                lo = mid + 1
            } else {
                hi = mid - 1
            }
        }
        return samples[found]
    }

    /**
     * How far the cursor had got the last time [takeSightings] was asked. See it.
     *
     * Starts **below** the first sighting rather than at it, so a recording whose very first
     * `tag` line is at `t = 0` still emits it. `-1.0` is safe because the record's `t` is
     * seconds since the session started and is never negative.
     */
    private var sightingsUpTo: Double = -1.0

    /**
     * **The sightings the cursor has crossed since this was last asked**, in time order.
     *
     * The shape a *stream* wants rather than the one [current] gives, and the difference is the
     * point. A state is a level — "what was true at this instant" — so a cursor sitting between
     * two samples honestly reports the earlier one, repeatedly, for as long as it sits there. A
     * sighting is an **event**: the detector saw a tag once, at one instant, and reporting it
     * again on the next tick would publish a detection that never happened. So this drains rather
     * than reads, and each sighting leaves exactly once.
     *
     * **A seek emits nothing, in either direction** — [seekFraction] re-arms this to wherever it
     * put the cursor, so the jump itself is never an emission and what follows it is whatever is
     * played from there. Dragging *back* therefore replays that stretch's detections, which is
     * what dragging back means; dragging *forward* does not deliver the burst that was jumped
     * over, because those detections did not happen on this playback and a subscriber receiving
     * forty at once with the aircraft ten seconds further on would be reading a lie about when
     * they were seen.
     *
     * Empty for a paused cursor, for a recording with no `tag` lines, and after the end.
     */
    fun takeSightings(): List<TimedSighting> {
        if (sightings.isEmpty()) return emptyList()
        val to = startSeconds + positionSeconds
        val from = sightingsUpTo
        sightingsUpTo = to
        if (to <= from) return emptyList()
        return sightings.filter { it.tSeconds > from && it.tSeconds <= to }
    }

    /** Re-arms [takeSightings] at the cursor, so a jump is never itself an emission. */
    private fun armSightings() {
        sightingsUpTo = startSeconds + positionSeconds
    }

    /**
     * Advances the cursor to [nowMs], if playing.
     *
     * A single step is clamped to [MAX_STEP_MS] and a clock that runs backwards moves nothing.
     * The gap being defended against is real and routine: this app spends whole sessions in the
     * background with DJI Fly in front, the drawing tick stops when the Activity does, and the
     * first frame after it comes back can be minutes later. Treating that gap as playback time
     * would jump the replay across most of a flight in one frame — which on a top-down view
     * looks exactly like an aircraft teleporting, the single most alarming thing this picture
     * could do. The time was not played; it is not counted.
     */
    fun onClock(nowMs: Long) {
        val last = lastClockMs
        lastClockMs = nowMs
        if (!playing || last == null) return
        val elapsed = (nowMs - last).coerceIn(0L, MAX_STEP_MS) / 1000.0
        positionSeconds = (positionSeconds + elapsed * rate).coerceIn(0.0, durationSeconds)
        if (atEnd) playing = false
    }

    /** Starts running. From the end, restarts from the beginning rather than sitting still. */
    fun play(nowMs: Long) {
        if (isEmpty) return
        if (atEnd) {
            positionSeconds = 0.0
            // A restart is a jump backwards, and a jump is never an emission — the run that
            // follows it carries the detections, one at a time, as they are reached.
            armSightings()
        }
        playing = true
        lastClockMs = nowMs
    }

    /** Stops the cursor where it is. The picture stays; only time stops. */
    fun pause() {
        playing = false
        lastClockMs = null
    }

    /** [play] or [pause], whichever is not the current state. */
    fun toggle(nowMs: Long) {
        if (playing) pause() else play(nowMs)
    }

    /**
     * Moves the cursor to a fraction of the recording, clamped. Does not change [playing].
     *
     * **Re-arms [takeSightings] at the new position.** A seek is a jump rather than playback, and
     * the detections on either side of it were not seen on this run — see [takeSightings] for why
     * delivering the ones jumped over would misdate them, and why the ones behind a backward drag
     * are then replayed by the playback that follows rather than by the drag itself.
     */
    fun seekFraction(fraction: Double, nowMs: Long) {
        if (!fraction.isFinite()) return
        positionSeconds = (fraction.coerceIn(0.0, 1.0) * durationSeconds)
        armSightings()
        lastClockMs = nowMs
    }

    /** Sets the playback rate, ignoring anything that is not a positive finite number. */
    fun setRate(value: Double, nowMs: Long) {
        if (!value.isFinite() || value <= 0.0) return
        rate = value
        lastClockMs = nowMs
    }

    /** How far through, 0..1. Zero-length records read 0 rather than dividing by zero. */
    fun fraction(): Double =
        if (durationSeconds <= 0.0) 0.0 else (positionSeconds / durationSeconds).coerceIn(0.0, 1.0)

    companion object {
        /**
         * The largest gap between two [onClock] calls that is counted as playback time, in
         * milliseconds.
         *
         * 500 ms is well over twice the drawing tick this is driven at, so an ordinary frame is
         * never clipped, and far under any gap a backgrounded Activity produces. See [onClock].
         */
        const val MAX_STEP_MS = 500L
    }
}
