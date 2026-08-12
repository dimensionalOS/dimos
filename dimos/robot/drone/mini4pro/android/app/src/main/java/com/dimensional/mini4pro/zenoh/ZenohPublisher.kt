package com.dimensional.mini4pro.zenoh

import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * **The live Zenoh publisher: a bounded queue, one thread, and a rule that nothing in the flight
 * path may ever wait on this class.**
 *
 * ## The safety requirement, stated first because it is the only one that is not negotiable
 *
 * The phone is the pilot. It holds the abort ladder and it owns the setpoint stream, and
 * `guided/GuidedStickEngine` runs that stream on its own thread with a watchdog measured in tens
 * of milliseconds. Publishing telemetry to a bus is best-effort diagnostics. **If those two ever
 * share a stall, the diagnostics win and an aircraft is flown by nobody**, so they are built so
 * that they cannot:
 *
 *  - [offer] does exactly one bounded-time thing — `offer` onto an array-backed queue. No
 *    allocation beyond the item, no I/O, no lock held across anything, no `synchronized` block a
 *    slow thread could be holding. It cannot block and it cannot throw.
 *  - Every call that can take time — opening a session, declaring a publisher, putting a payload,
 *    closing — happens on **this class's own thread and nowhere else**.
 *  - A full queue **drops and counts**. It does not grow, it does not wait, and it does not
 *    silently forget: the drop count is a number on the operator's screen and a line in the flight
 *    record. This is `record/FlightRecorder`'s rule and the argument is identical — a queue exists
 *    to absorb a stall, not to buffer a flight, and if it is full then no capacity fixes it.
 *  - **Nothing the transport does can propagate out of this class.** A throw from the seam is
 *    caught, counted, and turned into a reopen. Losing the bus is a diagnostics problem; taking
 *    the bridge down with it would make it a control problem.
 *
 * `NEVER_DROP` is the case that makes the shape necessary rather than tidy. `status` is
 * reliable + **block** by definition, so a subscriber that stops reading can wedge a `put`
 * indefinitely. That block lands on this thread, behind this queue, and the guided engine
 * announcing a refusal never learns it happened — it offered, the offer succeeded or was dropped,
 * and it went back to flying.
 *
 * ## The session lifecycle, and why it is lazy
 *
 * The bridge starts and stops repeatedly, the aircraft connects and disconnects, and WiFi comes
 * and goes. So a session is not something [start] produces; it is something the thread keeps
 * trying to have. [start] returns immediately, having done nothing but launch a thread.
 *
 * Opening is retried with exponential backoff between [Settings.reopenDelayMs] and
 * [Settings.maxReopenDelayMs], forever, because a router that is down at takeoff may be up in a
 * minute and the alternative is an operator who has to remember to restart the bridge. While there
 * is no session the queue is **drained and discarded**, counted separately from a queue-full drop,
 * because "the bus is not there" and "the bus cannot keep up" are different problems and the
 * screen should not conflate them.
 *
 * Failure degrades to no publishing and never to a stopped bridge: `docs/zenoh-dimos-transport.md`
 * §5 draws exactly that line against the WiFi gate's refusal, and the difference is that an
 * unbound socket is a *positive hazard* (telemetry over LTE) while an absent Zenoh session is
 * simply a bridge doing less.
 *
 * There is deliberately **no auto-re-engage of anything**. This class publishes; it has no inbound
 * half and cannot command. When the command half arrives, a returning session must be sent a fresh
 * command, because a resumed engagement nobody asked for is the unrequested aircraft action §Q4 of
 * the M2 safety decisions forbids.
 *
 * ## Testability
 *
 * No Android, no DJI, no zenoh imports. The transport is [ZenohSinkFactory], the clock is a
 * lambda, and [pumpOnce] drives the whole worker synchronously with no thread at all — the same
 * three affordances `record/FlightRecorder` uses, chosen because they are what made the recorder's
 * drop accounting testable. What is **not** covered here is everything below the seam; see
 * [ZenohSink]'s KDoc, which names it.
 */
class ZenohPublisher(
    private val config: ZenohConfig,
    private val factory: ZenohSinkFactory,
    private val settings: Settings = Settings(),
    private val nowMs: () -> Long,
    private val log: (String) -> Unit = {},
    /**
     * Called on every [Phase] change, from the publisher's thread.
     *
     * Deliberately **not** the `Announcer`. A sentence about Zenoh being down would be published
     * on the `status` channel, which is on the bus that is down — and the failure of that publish
     * would produce another sentence. The adapter takes this callback and decides what to do with
     * it (logcat and a flight-record event), which keeps the loop unreachable rather than merely
     * unlikely.
     */
    private val onPhase: (Phase, String) -> Unit = { _, _ -> },
) {

    /**
     * Every number here is a decision about what we are willing to lose.
     *
     * @param queueCapacity 512 items is about 17 s of the six telemetry channels at 5 Hz, or a
     *   long burst of operator sentences. Smaller than the recorder's 8192 on purpose: the
     *   recorder is the forensic record and its entries are the only copy, while everything on
     *   this bus is *also* in the flight log. Buffering a Zenoh backlog past the point a
     *   subscriber could use it is memory spent on stale telemetry.
     * @param reopenDelayMs first retry after a failed open or a broken session.
     * @param maxReopenDelayMs the backoff ceiling. 30 s, so a phone sitting in a field with no
     *   router is not dialling continuously for a whole battery.
     * @param pollMs how long the thread blocks on an empty queue, so an idle publisher costs
     *   nothing and a [stop] is still prompt.
     */
    data class Settings(
        val queueCapacity: Int = 512,
        val reopenDelayMs: Long = 1_000,
        val maxReopenDelayMs: Long = 30_000,
        val pollMs: Long = 100,
    )

    /** What the publisher is doing, for the status screen and for the flight record. */
    enum class Phase {
        /** Not started, or stopped. */
        STOPPED,

        /** Running, with no session — either never opened or dropped and being retried. */
        CONNECTING,

        /** Running, with a session, putting payloads on it. */
        PUBLISHING,
    }

    /**
     * What has happened, in numbers an operator can read.
     *
     * [dropped] and [discarded] are separate and that separation is the point: the first says the
     * bus could not keep up with us, the second says there was no bus. One is a capacity problem
     * and the other is a network problem, and a single "lost" counter would send somebody to look
     * at the wrong one.
     */
    data class Counters(
        val phase: Phase,
        val published: Long,
        val dropped: Long,
        val discarded: Long,
        val failures: Long,
        val opens: Long,
        val queued: Int,
        val peakQueued: Int,
        val lastError: String?,
    )

    private class Item(val channel: ZenohChannel, val key: String, val payload: ByteArray)

    private val queue = ArrayBlockingQueue<Item>(settings.queueCapacity)
    private val running = AtomicBoolean(false)
    private val published = AtomicLong()
    private val dropped = AtomicLong()
    private val discarded = AtomicLong()
    private val failures = AtomicLong()
    private val opens = AtomicLong()
    private val dropLock = Any()
    private val dropByChannel = HashMap<ZenohChannel, Long>()

    @Volatile private var phase = Phase.STOPPED
    @Volatile private var lastError: String? = null
    @Volatile private var peakQueued = 0
    private var thread: Thread? = null

    // ── worker-thread state; single-threaded, no locking needed ──
    private var sink: ZenohSink? = null
    private var declared = HashSet<String>()
    private var nextOpenAtMs = 0L
    private var backoffMs = 0L

    /**
     * The key expression for each channel under this session's prefix, resolved once.
     *
     * Held rather than recomputed so [offer] does no string work: it is called from the sampler,
     * from the guided engine's announcements, and potentially from a DJI callback thread, and the
     * whole point of this class is that those callers pay as little as possible.
     */
    private val keys: Map<ZenohChannel, String> =
        ZenohChannel.entries.mapNotNull { ch -> ch.keyOrNull(config.prefix)?.let { ch to it } }.toMap()

    val isRunning: Boolean get() = running.get()

    fun counters(): Counters = Counters(
        phase = phase,
        published = published.get(),
        dropped = dropped.get(),
        discarded = discarded.get(),
        failures = failures.get(),
        opens = opens.get(),
        queued = queue.size,
        peakQueued = peakQueued,
        lastError = lastError,
    )

    /** Per-channel drop counts, for the flight record's closing line. */
    fun dropsByChannel(): Map<ZenohChannel, Long> = synchronized(dropLock) { HashMap(dropByChannel) }

    // ─────────────────────────────────────────────────────────────────────────
    // lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Starts the worker. **Returns immediately and opens nothing** — the session is the thread's
     * problem, not the caller's, because `Zenoh.open` dials a router and a peer's connect timeout
     * defaults to an infinite retry. On the UI thread that is an ANR.
     *
     * [startThread] false is for tests and drives the worker through [pumpOnce].
     */
    @Synchronized
    fun start(startThread: Boolean = true) {
        if (!running.compareAndSet(false, true)) return
        setPhase(Phase.CONNECTING, "starting → ${config.connectEndpoint}")
        nextOpenAtMs = 0L
        backoffMs = 0L
        if (startThread) {
            thread = Thread({ workerLoop() }, "zenoh-publish").apply {
                isDaemon = true
                // Below the telemetry and control threads, exactly as the flight recorder's
                // writer is: if the phone is short of CPU, publishing must yield to flying.
                priority = Thread.MIN_PRIORITY + 1
                start()
            }
        }
    }

    /**
     * Stops the worker and closes the session.
     *
     * Whatever is still queued is **abandoned, not flushed**. The recorder does the opposite — it
     * drains synchronously on stop, because its entries are evidence and the only copy. These are
     * not: everything on this bus is also in the flight log, and a stop that blocked for however
     * long a `NEVER_DROP` publisher takes to unwedge would put the wedge back on the caller's
     * thread, which is the one thing this class exists to prevent.
     */
    @Synchronized
    fun stop() {
        if (!running.compareAndSet(true, false)) return
        val t = thread
        thread = null
        t?.interrupt()
        // Bounded, and short. A thread wedged inside a blocking `put` is not something we wait
        // out; it is a daemon and it dies with the process. What the join buys is the ordinary
        // case, where the session closes tidily.
        t?.join(JOIN_MS)
        // Only when the thread is gone, or never ran. Closing a session under a thread still
        // using it is the one way this class could crash something.
        if (t == null || !t.isAlive) closeSink()
        queue.clear()
        setPhase(Phase.STOPPED, "stopped")
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the producer side — safe from any thread, never blocks, never throws
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Enqueues one encoded message. Returns false when it was dropped.
     *
     * The **entire** flight-path contract is in this method's body: a volatile read, a map lookup
     * on a pre-built map, and `ArrayBlockingQueue.offer`, which takes a lock it holds for a few
     * instructions and never waits. Everything a caller could be hurt by is on the other side of
     * the queue.
     *
     * A channel with no key expression ([ZenohChannel.GIMBAL], retired) is refused here rather
     * than at the seam, so a retired channel cannot reach a bus even if something asks.
     */
    fun offer(channel: ZenohChannel, payload: ByteArray): Boolean {
        if (!running.get()) return false
        val key = keys[channel] ?: return false
        if (queue.offer(Item(channel, key, payload))) return true
        dropped.incrementAndGet()
        synchronized(dropLock) {
            dropByChannel[channel] = (dropByChannel[channel] ?: 0L) + 1L
        }
        return false
    }

    // ─────────────────────────────────────────────────────────────────────────
    // the worker
    // ─────────────────────────────────────────────────────────────────────────

    private fun workerLoop() {
        while (running.get()) {
            try {
                pumpOnce(settings.pollMs)
            } catch (e: InterruptedException) {
                break
            } catch (e: Throwable) {
                // Belt and braces. `pumpOnce` contains its own throws; this catches the ones a
                // future edit forgets, because a dead worker looks exactly like a quiet bus.
                noteFailure(e)
            }
        }
        closeSink()
    }

    /**
     * One iteration of the worker: wait briefly for something to send, make sure there is a
     * session, then drain everything queued onto it.
     *
     * Public and synchronous so a test can drive the whole lifecycle — open, publish, fail,
     * reopen, drop — with no thread and no sleeping, which is the only way the timing-dependent
     * parts of this class are assertable at all. [waitMs] `0` polls without blocking, which is
     * what every test uses.
     */
    fun pumpOnce(waitMs: Long = 0) {
        peakQueued = maxOf(peakQueued, queue.size)
        val head = if (waitMs > 0) queue.poll(waitMs, TimeUnit.MILLISECONDS) else queue.poll()
        try {
            ensureSink()
        } catch (e: Throwable) {
            noteFailure(e)
        }
        val s = sink
        val batch = ArrayList<Item>(BATCH_HINT)
        head?.let { batch.add(it) }
        queue.drainTo(batch)
        if (s == null) {
            // No session: discard loudly rather than let the queue stand full, so that when a
            // session does open it carries what is happening now and not a minute of history.
            if (batch.isNotEmpty()) discarded.addAndGet(batch.size.toLong())
            return
        }
        for (item in batch) {
            if (!running.get()) return
            try {
                if (declared.add(item.key)) s.declare(item.key, item.channel.qos)
                s.put(item.key, item.payload)
                published.incrementAndGet()
            } catch (e: Throwable) {
                // One bad put condemns the session, not the message: a zenoh error here means the
                // link to the router is gone, and putting the rest of the batch onto a dead
                // session would just be the same exception several hundred more times.
                noteFailure(e)
                closeSink()
                return
            }
        }
    }

    /** Opens a session if there is none and the backoff has expired. Throws on a failed open. */
    private fun ensureSink() {
        if (sink != null) return
        val now = nowMs()
        if (now < nextOpenAtMs) return
        // Set the *next* attempt before trying, so a throw out of `factory.open` still backs off.
        backoffMs = if (backoffMs == 0L) {
            settings.reopenDelayMs
        } else {
            (backoffMs * 2).coerceAtMost(settings.maxReopenDelayMs)
        }
        nextOpenAtMs = now + backoffMs
        val opened = factory.open(config)
        sink = opened
        declared = HashSet()
        opens.incrementAndGet()
        backoffMs = 0L
        nextOpenAtMs = 0L
        lastError = null
        setPhase(Phase.PUBLISHING, "session open → ${config.connectEndpoint}")
    }

    private fun closeSink() {
        val s = sink ?: return
        sink = null
        declared = HashSet()
        try {
            s.close()
        } catch (e: Throwable) {
            log("zenoh: close failed: ${e.javaClass.simpleName}: ${e.message}")
        }
        if (running.get()) setPhase(Phase.CONNECTING, "session closed, retrying")
    }

    private fun noteFailure(e: Throwable) {
        failures.incrementAndGet()
        lastError = "${e.javaClass.simpleName}: ${e.message}"
        log("zenoh: $lastError")
        if (running.get() && sink == null) setPhase(Phase.CONNECTING, lastError!!)
    }

    private fun setPhase(next: Phase, why: String) {
        if (phase == next) return
        phase = next
        onPhase(next, why)
    }

    private companion object {
        /** How long [stop] waits for a tidy close before giving up on it. */
        const val JOIN_MS = 1_000L

        /** Initial size of a drain batch. One tick of every channel, plus room. */
        const val BATCH_HINT = 16
    }
}
