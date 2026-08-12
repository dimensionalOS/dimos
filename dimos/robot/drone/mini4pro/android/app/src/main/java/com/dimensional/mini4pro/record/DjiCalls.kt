package com.dimensional.mini4pro.record

import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * **"We asked DJI to do X, and DJI answered Y"** — the aircraft-outbound half of [Tap], as one
 * correlated record rather than two lines a reader has to pair by eye.
 *
 * ## Why this exists at all
 *
 * The 25 Hz setpoint stream has been recorded in forensic detail since M3 Stage A. The *discrete*
 * asks — takeoff, land, confirm-landing, return, virtual-stick enable, gimbal rotate, simulator
 * start — were narrated, when they were narrated at all, as free-text `event` lines composed at
 * whichever call site happened to think of it. So the record said "Return sent to DJI" and then,
 * separately and in different words, sometimes, that something failed. It did not say *which ask*
 * the failure belonged to, how long DJI took, or — the one that has cost this project the most
 * time — that an ask was made and **nothing ever came back**.
 *
 * `CONTROL_AUTH_HAS_NO_CONTROL_AUTH`, `REQUEST_HANDLER_NOT_FOUND` and `SYSTEM_ERROR` have each
 * sent someone down a wrong path in a single week. All three are strings DJI hands us on a
 * callback thread and all three were, until now, one `Log.w` away from being lost.
 *
 * ## The three properties, and the one that is not obvious
 *
 * 1. **The ask is written before the SDK is touched.** A `performAction` that throws, or a process
 *    that dies between the call and the callback, still leaves the ask on the record.
 * 2. **A refusal is always urgent**, whatever the caller asked for. An accepted gimbal aim can
 *    wait for the next 500 ms flush; a refusal is one of the lines a post-mortem is built from and
 *    goes to the flash immediately. This is why [urgentFor] is a function of the phase and not
 *    only of the op.
 * 3. **A swallowed callback becomes a line, not an absence.** On 2026-07-26 an operator's Return
 *    reached `performAction` four times on a connected, healthy, grounded aircraft and DJI invoked
 *    **neither** callback, ever (`docs/measurements/2026-07-26-m2-first-command.md`). A record
 *    that shows the ask and no answer is technically complete and practically invisible — it is
 *    the same shape as the change-driven-key trap this project has hit three times, where silence
 *    means two entirely different things. [sweep] turns it into a [DjiPhase.NONE] entry naming the
 *    op and how long we waited.
 *
 * ## Never perturbs the flight
 *
 * Every public method is total: it contains its own throws (including a throwing [sink]) and
 * returns normally. That containment lives here, once, rather than at each port decorator —
 * a decorator obliged to wrap every tap call in a `try` will eventually forget one, and the
 * forgotten one will be on the path that turns a recorder bug into a refused landing.
 *
 * No locks. [ConcurrentHashMap] plus an [AtomicLong], because `begin` is called from the
 * `mavlink-rx` thread and the UI thread while `accepted`/`refused` arrive on DJI's callback thread
 * and [sweep] runs on the recorder's sampler. Nothing here blocks and nothing here does I/O; the
 * [sink] is `FlightRecorder.record`, which is a bounded `offer`.
 *
 * ## Bounded, because callbacks that never arrive would otherwise accumulate
 *
 * Outstanding asks are evicted by [sweep] at [Config.unansweredAfterMs], so the map is bounded by
 * rate × timeout in normal operation. [Config.maxOutstanding] is the backstop for the case where
 * nobody calls [sweep]: the oldest ask is evicted and recorded as [DjiPhase.NONE] with
 * `overflow` set, so the bound is visible rather than silent.
 *
 * No Android imports and no DJI imports, so all of the above is unit-testable — the same rule
 * every file in `record/` except `Recorder` follows.
 */
class DjiCalls(
    /** Where entries go. `FlightRecorder.record` in the app; a list in the tests. */
    private val sink: (LogEntry) -> Unit,
    /** The one clock, `SystemClock.elapsedRealtimeNanos` on the phone. */
    private val nowNanos: () -> Long,
    private val config: Config = Config(),
) {

    data class Config(
        /**
         * How long an ask may go unanswered before it is recorded as [DjiPhase.NONE].
         *
         * 5 s, and the number is a judgement rather than a measurement: DJI's own answers on this
         * airframe arrive in tens of milliseconds when they arrive at all, and the observed
         * failure is not lateness but total silence. Long enough that a slow link never produces a
         * false `none`; short enough that the line lands while the operator is still looking at
         * the screen they pressed the button on.
         *
         * A late answer after a `none` is **still recorded**, with its true elapsed time, so the
         * pair reads "we gave up at 5 s" / "DJI replied at 9 s" rather than the second being
         * dropped as unexpected.
         */
        val unansweredAfterMs: Long = 5_000,

        /**
         * Hard cap on outstanding asks, for the case where [sweep] is never called. 256 is far
         * above anything the discrete-action rate can reach (the whole gimbal aim path is ≤5/s and
         * answers in milliseconds) and small enough that the map is a rounding error in memory.
         */
        val maxOutstanding: Int = 256,
    )

    private class Outstanding(
        val sequence: Long,
        val op: String,
        val startNanos: Long,
        val urgent: Boolean,
    )

    private val sequence = AtomicLong()
    private val outstanding = ConcurrentHashMap<Long, Outstanding>()

    /** How many asks are waiting for an answer. For tests and for the stats line. */
    val outstandingCount: Int get() = outstanding.size

    /**
     * Records the ask and returns the handle its answer is reported through.
     *
     * Total: if anything in here fails, a handle that records nothing is returned rather than a
     * throw reaching the port that is one line away from commanding an aircraft.
     */
    fun begin(op: String, argsJson: String?, urgent: Boolean): Tap.Call {
        return try {
            val t = nowNanos()
            val seq = sequence.incrementAndGet()
            evictOldestIfFull(t)
            outstanding[seq] = Outstanding(seq, op, t, urgent)
            emit(
                LogEntry.DjiCall(
                    monoNanos = t,
                    sequence = seq,
                    op = op,
                    phase = DjiPhase.ASK,
                    argsJson = argsJson,
                    urgent = urgent,
                )
            )
            Handle(seq, op, t)
        } catch (e: Throwable) {
            DEAF
        }
    }

    /**
     * Emits a [DjiPhase.NONE] for every ask older than [Config.unansweredAfterMs].
     *
     * Called from `Recorder`'s existing 5 Hz sampler rather than from a thread of its own: this
     * needs no better resolution than the sampler's, and a recorder that spawns a thread to notice
     * that a recorder thread noticed nothing is one thread too many.
     */
    fun sweep() {
        try {
            val t = nowNanos()
            val limitNanos = config.unansweredAfterMs * 1_000_000L
            for (entry in outstanding.values) {
                if (t - entry.startNanos < limitNanos) continue
                // remove() before emitting, so two concurrent sweeps cannot both report it.
                if (outstanding.remove(entry.sequence) == null) continue
                emitNone(entry, t, overflow = false)
            }
        } catch (e: Throwable) {
            // Nothing to do and nowhere to say it. The next sweep tries again.
        }
    }

    /**
     * Drops the oldest outstanding ask when the map is at [Config.maxOutstanding], recording it as
     * a [DjiPhase.NONE] with `overflow`. Only reachable when [sweep] is not being called.
     */
    private fun evictOldestIfFull(nowN: Long) {
        if (outstanding.size < config.maxOutstanding) return
        val oldest = outstanding.values.minByOrNull { it.startNanos } ?: return
        if (outstanding.remove(oldest.sequence) == null) return
        emitNone(oldest, nowN, overflow = true)
    }

    private fun emitNone(entry: Outstanding, nowN: Long, overflow: Boolean) {
        emit(
            LogEntry.DjiCall(
                monoNanos = nowN,
                sequence = entry.sequence,
                op = entry.op,
                phase = DjiPhase.NONE,
                elapsedMs = elapsedMs(entry.startNanos, nowN),
                overflow = overflow,
                // Always: "DJI never answered" is one of the highest-value lines in the file, and
                // the whole point of it is that it survives the crash that followed.
                urgent = true,
            )
        )
    }

    /**
     * One outstanding ask.
     *
     * [answered] makes the three report methods idempotent. DJI's callbacks are not ours to trust
     * — nothing documents that `onSuccess` and `onFailure` are mutually exclusive or fire once —
     * and a second report would otherwise write a second answer for one ask, which is precisely
     * the correlation this class exists to keep honest.
     */
    private inner class Handle(
        private val seq: Long,
        private val op: String,
        private val startNanos: Long,
    ) : Tap.Call {
        private val answered = AtomicBoolean(false)

        override fun accepted() = finish(DjiPhase.OK, null)
        override fun refused(error: String) = finish(DjiPhase.ERR, error)
        override fun settled(error: String?) =
            finish(DjiPhase.SYNC, error)

        private fun finish(phase: String, error: String?) {
            try {
                if (!answered.compareAndSet(false, true)) return
                val t = nowNanos()
                // May already be gone — swept as `none`, or evicted. The answer is still recorded,
                // with its true elapsed time, so a late reply reads as a late reply.
                val entry = outstanding.remove(seq)
                emit(
                    LogEntry.DjiCall(
                        monoNanos = t,
                        sequence = seq,
                        op = op,
                        phase = phase,
                        error = error,
                        elapsedMs = elapsedMs(startNanos, t),
                        urgent = urgentFor(phase, error, entry?.urgent ?: true),
                    )
                )
            } catch (e: Throwable) {
                // Contained here so a recorder fault cannot propagate into a DJI callback.
            }
        }
    }

    private fun emit(entry: LogEntry) {
        try {
            sink(entry)
        } catch (e: Throwable) {
            // A throwing sink is an evidence problem. It must never become a flight problem.
        }
    }

    companion object {
        /** Returned when [begin] itself failed. Records nothing and cannot throw. */
        private val DEAF = object : Tap.Call {
            override fun accepted() {}
            override fun refused(error: String) {}
            override fun settled(error: String?) {}
        }

        /**
         * Whether an answer goes to the flash immediately.
         *
         * **A refusal is always urgent**, even for an op whose ask was not. That asymmetry is the
         * whole of property 2 in the class doc: an accepted gimbal aim is one of thousands and can
         * wait for the next flush; the one that came back `SDK_SERVICE_GIMBAL_ROTATE_YAW_NOT_ALLOW`
         * is the line someone will be reading in an hour, and it must survive the crash that
         * follows it.
         */
        fun urgentFor(phase: String, error: String?, askWasUrgent: Boolean): Boolean =
            phase == DjiPhase.ERR || phase == DjiPhase.NONE || error != null || askWasUrgent

        /** Nanoseconds to whole milliseconds, never negative. */
        fun elapsedMs(startNanos: Long, nowNanos: Long): Long =
            ((nowNanos - startNanos) / 1_000_000L).coerceAtLeast(0L)
    }
}
