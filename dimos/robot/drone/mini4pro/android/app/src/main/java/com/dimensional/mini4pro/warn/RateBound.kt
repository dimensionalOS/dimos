package com.dimensional.mini4pro.warn

/**
 * A sliding-window cap on how many `STATUSTEXT`s device health may put on the wire.
 *
 * ## Why there is a second brake at all
 *
 * [WarningMonitor]'s diff is the brake that matters: an aircraft that is merely hot produces one
 * event no matter how often DJI repeats the news, and repeats are the overwhelmingly common case.
 * This exists for the case the diff cannot help with — a code that genuinely **flaps**, appearing
 * and clearing and appearing again. Each of those is a real transition, so the diff passes every
 * one, and a marginal sensor could produce them at whatever rate DJI notices them at.
 *
 * The precedent is not hypothetical. This project once put 25 lines a second into a log monitor
 * from a refusal path that logged per loop iteration, and it buried everything else in the file.
 * The rule that came out of it is that **anything driven by an external event source gets a
 * bound**, chosen so that the worst case is merely noisy rather than useless.
 *
 * ## The numbers
 *
 * [CAPACITY] events per [WINDOW_MS]: a burst of six goes straight out — which covers every
 * realistic simultaneous-failure picture, since DJI's own list rarely exceeds a handful — and the
 * sustained ceiling is 0.5 Hz. Against `Bridge`'s 5 Hz telemetry that is negligible traffic, and
 * against the 25 Hz disaster it is fifty times quieter.
 *
 * ## What suppression is not
 *
 * A suppressed event is **not dropped**. [WarningMonitor] marks it [WarnEvent.rateLimited] and
 * [DeviceHealthWatch] still writes it to the flight record and logcat. The bound protects the
 * operator's screen and the link; it never protects them from the evidence.
 *
 * Not thread-safe; every caller is serialised by [DeviceHealthWatch].
 */
class RateBound(
    private val capacity: Int = CAPACITY,
    private val windowMs: Long = WINDOW_MS,
) {

    /**
     * When each of the last [capacity] grants happened. Bounded by construction — nothing is ever
     * added without an eviction pass first — so this cannot grow with the length of a session.
     */
    private val grants = ArrayDeque<Long>()

    /**
     * Take a token if one is free, at monotonic time [nowMs].
     *
     * @return true if the caller may transmit. Call **once per event**, and only for events that
     *   would otherwise go on the wire: a token spent on an event that was never forwardable
     *   would let `NORMAL`-level chatter starve a real warning.
     */
    fun take(nowMs: Long): Boolean {
        while (grants.isNotEmpty() && nowMs - grants.first() >= windowMs) grants.removeFirst()
        if (grants.size >= capacity) return false
        grants.addLast(nowMs)
        return true
    }

    /** Forget the window. Pairs with [WarningMonitor.reset] — a new session starts with full credit. */
    fun reset() = grants.clear()

    companion object {
        /** Events allowed inside one [WINDOW_MS]. Sized to pass a whole realistic failure picture at once. */
        const val CAPACITY = 6

        /** The window, in milliseconds. With [CAPACITY] this is a sustained ceiling of 0.5 Hz. */
        const val WINDOW_MS = 12_000L
    }
}
