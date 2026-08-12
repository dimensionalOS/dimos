package com.dimensional.mini4pro.vision

/**
 * **What the latch is holding**: which tag this site has, where it is, and the evidence for both.
 */
data class Latched(
    val tagId: Int,
    /**
     * Where the tag is, or **null when it was seen but could not be placed** — no position fix, no
     * heading, no commanded camera angle, or a camera not near nadir.
     *
     * Null is a real and useful state and is deliberately not collapsed into "not latched". "We
     * have seen the tag at this site on this flight" is by itself a reason to spend CPU on the
     * descent, which is Ivan's argument for the whole mechanism; knowing *where* is a bonus that
     * makes the descent better, not a precondition for looking.
     */
    val fix: TagFix?,
    val firstSeenNanos: Long,
    val lastSeenNanos: Long,
    /** How many frames of this id were counted before it latched, and since. */
    val sightings: Int,
)

/**
 * **"We have actually seen a tag at this site, this flight."**
 *
 * Ivan's trigger, and the argument for it is worth restating because it is the reason this is not a
 * geofence: *"we have seen a tag here, this flight"* is a far stronger reason to spend a third of a
 * phone's CPU on a descent than *"we are near a coordinate"*. A coordinate says where somebody
 * thought the pad was. This says the camera saw it.
 *
 * ## Why it takes more than one frame
 *
 * Because a false id is worse than a miss. The comparison measured **2 false ids in 1978 frames**
 * for this detector against OpenCV's zero — both single frames, both *alongside* the correct id, and
 * both where the real tag was small. Latching on one frame would let a landing arm on a tag that was
 * never there; a two-of-three temporal filter erases exactly that failure, and the measurement
 * document says so and then says *"this measurement did not test one"*. [minSightings] within
 * [windowNanos] is that filter, and the defaults are chosen against those two numbers.
 *
 * The window matters as much as the count. Three sightings spread over a whole flight are three
 * independent chances to be wrong; three inside two seconds are the same tag seen three times.
 *
 * ## Which fix it keeps
 *
 * The one from the **largest** tag seen, not the newest and not an average. Size is the only ranking
 * available without a real pose, and the measurement is unambiguous about what it ranks: restricted
 * to tags at least 40 px across, the focal-length fit's residual is 0.051 m, and including
 * everything down to 14 px triples it to 0.130 m. Averaging would let the many poor fixes outvote
 * the few good ones. A running mean is the obvious thing to do here and it is the wrong one.
 *
 * ## Lifetime
 *
 * One flight. [reset] is called when a new flight starts — not when one ends, so that a fix survives
 * touchdown to be looked at, which is when somebody is most likely to want it.
 *
 * Not thread-safe; it is touched only from [TagRecogniser]'s single worker thread, and the published
 * result is an immutable snapshot.
 */
class TagLatch(
    private val minSightings: Int = DEFAULT_MIN_SIGHTINGS,
    private val windowNanos: Long = DEFAULT_WINDOW_NANOS,
) {

    /** Per-id evidence, only for ids not yet latched. Cleared the moment one wins. */
    private val candidates = HashMap<Int, Candidate>()

    private var latched: Latched? = null

    private class Candidate(var firstNanos: Long, var lastNanos: Long, var count: Int)

    /**
     * Feed one detection. Returns true **on the frame it latches** and never again — so a caller can
     * announce the event without keeping its own edge-detection state.
     *
     * Sightings after the latch still arrive here: they refresh [Latched.lastSeenNanos], count
     * towards [Latched.sightings], and replace the fix when they are from a larger tag. A latch that
     * stopped listening would freeze on the worst view of the tag it ever had, because the first
     * three frames of an approach are the most distant three.
     */
    fun observe(detection: TagDetection, fix: TagFix?, atNanos: Long): Boolean {
        val held = latched
        if (held != null) {
            if (detection.id != held.tagId) return false
            // Strictly greater, so a stream of equal-sized detections does not churn the fix.
            val better = fix != null && (held.fix == null || fix.pixelSize > held.fix.pixelSize)
            latched = held.copy(
                fix = if (better) fix else held.fix,
                lastSeenNanos = atNanos,
                sightings = held.sightings + 1,
            )
            return false
        }

        val c = candidates.getOrPut(detection.id) { Candidate(atNanos, atNanos, 0) }
        // Outside the window the evidence is stale and the count restarts from this frame. It does
        // not decay gradually: the question is "three sightings close together", and a decay would
        // let a trickle of isolated false ids accumulate into a latch, which is the exact failure
        // the window exists to prevent.
        if (atNanos - c.firstNanos > windowNanos) {
            c.firstNanos = atNanos
            c.count = 0
        }
        c.lastNanos = atNanos
        c.count++
        if (c.count < minSightings) return false

        latched = Latched(
            tagId = detection.id,
            fix = fix,
            firstSeenNanos = c.firstNanos,
            lastSeenNanos = atNanos,
            sightings = c.count,
        )
        // Every other candidate's evidence is discarded rather than kept. Two latched tags is not a
        // state anything downstream knows how to read, and the board this is aimed at
        // (`docs/apriltag-landing-recording.md` §6.3) is one pad with two markers on it — a second
        // id is the *same* site, not a competing one.
        candidates.clear()
        return true
    }

    /**
     * The frame had no tag in it. Nothing to forget: a gap between sightings is normal — the
     * detection rate is 92 % in the best band above 7 m, and a control law that treated one missed
     * frame as evidence against would never latch at all.
     *
     * Present as a named no-op so that "what happens on a miss" is a decision in the code rather
     * than an absence a reader has to notice. The window in [observe] is what expires stale
     * evidence, and it does so by time rather than by counting misses, because time is what makes
     * three sightings *one* observation.
     */
    fun observedNothing() {
        // Deliberately empty. See the KDoc.
    }

    /** The latch, or null. Immutable — safe to hand to any thread. */
    fun latched(): Latched? = latched

    /** True once the flight has evidence a tag is here. The arming rule's input. */
    fun isLatched(): Boolean = latched != null

    /** A new flight. Everything, including the evidence that has not yet latched. */
    fun reset() {
        candidates.clear()
        latched = null
    }

    companion object {
        /**
         * **Three.** Both measured false ids were single frames, so two would already be enough;
         * three is one frame of margin, and it costs 3/24 s ≈ 125 ms at the aircraft's frame rate
         * and 300 ms at the 10 Hz detection cap. That is a price worth paying against the one
         * failure mode this sensor has that a controller cannot see.
         */
        const val DEFAULT_MIN_SIGHTINGS = 3

        /**
         * **Two seconds.** At the 10 Hz cap that is up to twenty chances, so a tag detected in even
         * 15 % of frames still latches — comfortably below the measured rate anywhere the tag is
         * usable (100 % to 7 m, 41 % in the last metre where framing fails). Short enough that three
         * isolated false ids would have to arrive within two seconds of each other, which in 1978
         * frames happened zero times.
         */
        const val DEFAULT_WINDOW_NANOS = 2_000_000_000L
    }
}
