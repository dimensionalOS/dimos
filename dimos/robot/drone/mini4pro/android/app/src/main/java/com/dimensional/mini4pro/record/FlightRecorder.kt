package com.dimensional.mini4pro.record

import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * How the recorder behaves. Every number here is a decision about what we are
 * willing to lose, so each carries its reasoning.
 */
data class RecorderConfig(
    /**
     * Bounded queue between the producing threads and the writer.
     *
     * 8192 entries is about 40 s of everything at 25 Hz. The queue exists to
     * absorb a filesystem stall, not to buffer a flight: if it fills, the writer
     * is not keeping up and no capacity will fix that, so we would rather drop
     * loudly than grow without limit inside an app that is flying an aircraft.
     */
    val queueCapacity: Int = 8192,

    /**
     * How often buffered bytes are pushed into the kernel. **This is the window
     * that a process crash can lose**, and it is the number to quote when someone
     * asks how much of the tail survives.
     */
    val flushIntervalMs: Long = 500,

    /**
     * How often the kernel is asked to put bytes on the flash. **This is the
     * window a battery pull or a hard reboot can lose.** `fsync` on a phone's
     * flash costs tens of milliseconds, which is why it is not every flush; it
     * happens anyway, immediately, after any urgent entry (an event, a mode
     * change, a virtual-stick state change, a drop record), so the discrete
     * moments are on the flash before the next one arrives.
     */
    val syncIntervalMs: Long = 5_000,

    /** Rotate at this size. 32 MB is ~3 h of a 25 Hz session (see the doc). */
    val maxFileBytes: Long = 32L * 1024 * 1024,

    /** Parts kept per session. 8 × 32 MB = 256 MB, the whole disk budget. */
    val maxParts: Int = 8,

    /** Whole sessions kept on disk. */
    val maxSessions: Int = 20,

    /** How often the recorder writes its own health line. */
    val statsIntervalMs: Long = 10_000,

    /** Drain at least this often even when idle, so a flush is never late. */
    val pollMs: Long = 100,
)

/** Wall clock, isolated so tests are deterministic. */
fun interface WallClock {
    /** Milliseconds since the Unix epoch. */
    fun millis(): Long
}

/**
 * Monotonic clock. On the phone this is `SystemClock.elapsedRealtimeNanos()` and
 * nothing else — see [FlightRecorder]'s note on clocks.
 */
fun interface MonotonicClock {
    fun nanos(): Long
}

/**
 * The on-phone flight recorder: an append-only JSONL record of everything at our
 * boundaries, written by a dedicated thread behind a bounded queue.
 *
 * ## Why this is the primary record and not the MAVLink mirror
 *
 * The MAVLink link is precisely what fails when things go wrong. A record that
 * exists only by being transmitted loses exactly the moments worth investigating:
 * link loss, RC authority takeback, a geofence takeback at the edge of a zone. The
 * phone-local file is also unconstrained by a radio that shares its bandwidth with
 * video. So the file is the forensic record; `GcsMirror` is a small, deliberate
 * cross-check that lands in QGC's `.tlog` for free.
 *
 * ## One clock, and one wall-clock anchor
 *
 * Every entry is timestamped from a single monotonic source. The header records
 * both that source's value and the wall clock at file open, and those two numbers
 * are the only bridge to the other two records (QGC's `.tlog`, DJI's flight
 * records). Mixing clocks — some entries from `currentTimeMillis`, some from
 * `elapsedRealtime` — makes cross-source alignment impossible the moment NTP steps
 * the wall clock mid-flight, and it will not be recoverable afterwards. This is
 * the single most important design detail in the file.
 *
 * ## Never perturb the flight
 *
 * [record] does one bounded-time thing: `offer` onto an array-backed queue. No
 * allocation of strings, no reflection, no I/O, no blocking. All rendering and all
 * writing happen on the writer thread. If the queue is full the entry is **dropped
 * and counted**, and the writer emits a [LogEntry.Drop] naming the count and the
 * time span it covers — because a recorder that silently swallows the interesting
 * moment is worse than one that admits it.
 *
 * ## Testability
 *
 * There are no Android and no DJI imports in this file. The clocks and the sink
 * are injected, and [drainOnce] lets a test drive the writer synchronously with no
 * thread at all. The adapter that supplies the real clocks and the real DJI
 * callbacks is `Recorder`.
 */
class FlightRecorder(
    private val session: String,
    private val sinks: SinkFactory,
    private val mono: MonotonicClock,
    private val wall: WallClock,
    private val config: RecorderConfig = RecorderConfig(),
    /** Pre-rendered JSON members describing app, device, aircraft and RC identity. */
    private val headerJson: String? = null,
) {

    private val queue = ArrayBlockingQueue<LogEntry>(config.queueCapacity)
    private val running = AtomicBoolean(false)
    private var writerThread: Thread? = null

    // ── drop accounting ──
    private val dropped = AtomicLong()
    private val dropReported = AtomicLong()
    @Volatile private var firstDropNanos = 0L
    @Volatile private var lastDropNanos = 0L
    private val dropKindCounts = HashMap<String, Long>()
    private val dropLock = Any()

    // ── writer-thread state (single-threaded, no locking needed) ──
    private var sink: LogSink? = null
    private var part = 0
    private var rotations = 0
    private var written = 0L
    private var partLines = 0
    private var bytes = 0L
    private var flushes = 0L
    private var syncs = 0L
    private var peakQueued = 0
    private var lastFlushMs = 0L
    private var lastSyncMs = 0L
    private var lastStatsMs = 0L

    /** Monotonic reading at session start; the origin of every entry's `t`. */
    var startedMonoNanos: Long = 0L
        private set

    /** Wall clock at session start, the anchor for cross-source alignment. */
    var startedUnixMs: Long = 0L
        private set

    val isRunning: Boolean get() = running.get()
    val dropCount: Long get() = dropped.get()
    val writtenCount: Long get() = written
    val byteCount: Long get() = bytes
    val currentPart: Int get() = part
    val currentSinkName: String? get() = sink?.name

    // ─────────────────────────────────────────────────────────────────────────
    // lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Opens part 1 and writes the header. [startThread] `false` is for tests and
     * for the byte-rate benchmark, which drive [drainOnce] themselves.
     */
    fun start(startThread: Boolean = true) {
        if (!running.compareAndSet(false, true)) return
        startedMonoNanos = mono.nanos()
        startedUnixMs = wall.millis()
        lastFlushMs = startedUnixMs
        lastSyncMs = startedUnixMs
        lastStatsMs = startedUnixMs
        openNextPart()
        if (startThread) {
            writerThread = Thread({ writerLoop() }, "flight-recorder").apply {
                isDaemon = true
                // Below the telemetry and control threads on purpose: if the phone
                // is short of CPU, writing the log must yield to flying.
                priority = Thread.MIN_PRIORITY + 1
                start()
            }
        }
    }

    /** Drains what is queued, writes a final stats line, and closes the file. */
    fun stop() {
        if (!running.compareAndSet(true, false)) return
        writerThread?.interrupt()
        writerThread?.join(2_000)
        writerThread = null
        // Whether or not the thread was running, drain synchronously so the tail is
        // on disk before the file closes.
        drainOnce()
        reportDropsIfAny()
        // Written directly rather than enqueued: `record` refuses once `running` is
        // false, and this line is the thing that distinguishes "the recorder was
        // stopped" from "the app died". Its absence at the end of a file is itself a
        // finding, so it must not depend on the queue.
        writeEntry(LogEntry.Event(mono.nanos(), EventCode.RECORDER_STOP))
        writeStats()
        sink?.close()
        sink = null
    }

    // ─────────────────────────────────────────────────────────────────────────
    // producers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Enqueues one entry. Safe from any thread, never blocks, never throws.
     * Returns `false` if the entry was dropped.
     */
    fun record(entry: LogEntry): Boolean {
        if (!running.get()) return false
        if (queue.offer(entry)) return true
        val n = dropped.incrementAndGet()
        val t = entry.monoNanos
        if (n == dropReported.get() + 1) firstDropNanos = t
        lastDropNanos = t
        synchronized(dropLock) {
            dropKindCounts[entry.kind] = (dropKindCounts[entry.kind] ?: 0L) + 1
        }
        return false
    }

    /** Convenience for the common case of stamping now. */
    fun now(): Long = mono.nanos()

    // ─────────────────────────────────────────────────────────────────────────
    // writer
    // ─────────────────────────────────────────────────────────────────────────

    private fun writerLoop() {
        while (running.get()) {
            try {
                // Block briefly so an idle recorder costs nothing, then drain the
                // rest in one go.
                val head = queue.poll(config.pollMs, TimeUnit.MILLISECONDS)
                if (head != null) writeEntry(head)
                drainOnce()
                maintenance()
            } catch (e: InterruptedException) {
                break
            } catch (e: Throwable) {
                // A recorder must never take the app down with it. Losing the log
                // is bad; losing control of an airborne aircraft is worse.
                reportSelfFailure(e)
            }
        }
    }

    /**
     * Writes everything currently queued. Public so a test can drive the writer
     * without a thread; also called by [stop] to flush the tail.
     */
    fun drainOnce(): Int {
        val batch = ArrayList<LogEntry>(64)
        val n = queue.drainTo(batch)
        peakQueued = maxOf(peakQueued, n)
        for (e in batch) writeEntry(e)
        return n
    }

    /** Flushes, syncs, writes stats and rotates when due. Called from the loop. */
    fun maintenance() {
        val s = sink ?: return
        val nowMs = wall.millis()
        reportDropsIfAny()
        if (nowMs - lastFlushMs >= config.flushIntervalMs) {
            s.flush(durable = false)
            flushes++
            lastFlushMs = nowMs
        }
        if (nowMs - lastSyncMs >= config.syncIntervalMs) {
            s.flush(durable = true)
            syncs++
            lastSyncMs = nowMs
            lastFlushMs = nowMs
        }
        if (nowMs - lastStatsMs >= config.statsIntervalMs) {
            writeStats()
            lastStatsMs = nowMs
        }
        // `partLines > 1` guards against a maxFileBytes smaller than a header, which
        // would otherwise rotate on every pass and write nothing but headers.
        if (s.bytesWritten >= config.maxFileBytes && partLines > 1) rotate()
    }

    private fun writeEntry(entry: LogEntry) {
        val s = sink ?: return
        val line = render(entry)
        s.writeLine(line)
        written++
        partLines++
        bytes += line.length + 1
        if (entry.urgent) {
            // Durable immediately: these are the lines a post-mortem is built from,
            // and they are rare enough that the fsync cost is irrelevant.
            s.flush(durable = true)
            syncs++
            lastSyncMs = wall.millis()
            lastFlushMs = lastSyncMs
        }
    }

    /** Renders one entry as a JSONL line. Visible for tests. */
    fun render(entry: LogEntry): String = JsonObject.render { o ->
        o.put("t", secondsSinceStart(entry.monoNanos), 6)
        o.put("k", entry.kind)
        entry.writeBody(o)
    }

    private fun secondsSinceStart(nanos: Long): Double = (nanos - startedMonoNanos) / 1e9

    /**
     * Emits a [LogEntry.Drop] whenever entries have been lost since the last
     * report. Written straight to the sink rather than enqueued — a drop record
     * that can itself be dropped would defeat the purpose.
     */
    private fun reportDropsIfAny() {
        val total = dropped.get()
        val reported = dropReported.get()
        if (total <= reported) return
        val kinds = synchronized(dropLock) {
            val snapshot = HashMap(dropKindCounts)
            dropKindCounts.clear()
            snapshot
        }
        val kindsJson = if (kinds.isEmpty()) null else JsonObject.render { o ->
            kinds.entries.sortedBy { it.key }.forEach { o.put(it.key, it.value) }
        }
        dropReported.set(total)
        writeEntry(
            LogEntry.Drop(
                monoNanos = mono.nanos(),
                count = total - reported,
                firstMonoNanos = firstDropNanos,
                lastMonoNanos = lastDropNanos,
                total = total,
                kindCountsJson = kindsJson,
            )
        )
    }

    private fun writeStats() {
        writeEntry(
            LogEntry.Stats(
                monoNanos = mono.nanos(),
                written = written,
                dropped = dropped.get(),
                queued = queue.size,
                peakQueued = peakQueued,
                bytes = bytes,
                flushes = flushes,
                syncs = syncs,
                rotations = rotations,
                part = part,
            )
        )
    }

    private fun reportSelfFailure(e: Throwable) {
        try {
            writeEntry(
                LogEntry.Event(
                    monoNanos = mono.nanos(),
                    code = EventCode.SDK_ERROR,
                    severity = LogEntry.SEV_ERROR,
                    message = "recorder writer failed: ${e.javaClass.simpleName}: ${e.message}",
                )
            )
        } catch (ignored: Throwable) {
            // Nothing left to do; the queue keeps draining and dropping loudly.
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // rotation
    // ─────────────────────────────────────────────────────────────────────────

    /** Closes the current part and opens the next. Visible for tests. */
    fun rotate() {
        val previous = sink
        val previousName = previous?.name
        val previousBytes = previous?.bytesWritten ?: 0L
        previous?.let {
            writeEntry(
                LogEntry.Event(
                    monoNanos = mono.nanos(),
                    code = EventCode.ROTATE,
                    message = "continues in part ${part + 1}",
                )
            )
            it.close()
        }
        rotations++
        openNextPart(continuesFrom = previousName, previousBytes = previousBytes)
        sinks.prune(session, config.maxParts)
    }

    private fun openNextPart(continuesFrom: String? = null, previousBytes: Long = 0L) {
        part++
        sink = sinks.open(session, part)
        writeHeader(continuesFrom, previousBytes)
    }

    private fun writeHeader(continuesFrom: String?, previousBytes: Long) {
        // Every part carries a full header, so any single file is self-describing:
        // pulling one part off the phone is enough to read it.
        val line = JsonObject.render { o ->
            o.put("t", secondsSinceStart(mono.nanos()), 6)
            o.put("k", LogEntry.KIND_HEADER)
            o.put("format", FORMAT)
            o.put("schema", SCHEMA)
            o.put("session", session)
            o.put("part", part)
            o.put("continues", continuesFrom)
            o.put("prev_bytes", if (continuesFrom != null) previousBytes else null)
            o.put("started_unix_ms", startedUnixMs)
            o.put("started_mono_ns", startedMonoNanos)
            o.put("opened_unix_ms", wall.millis())
            o.put("opened_mono_ns", mono.nanos())
            o.obj("clock") { w ->
                w.put("mono", MONO_CLOCK_NAME)
                w.put("wall", WALL_CLOCK_NAME)
                w.put("t_field", "seconds since started_mono_ns")
            }
            o.obj("config") { w ->
                w.put("queue_capacity", config.queueCapacity)
                w.put("flush_interval_ms", config.flushIntervalMs)
                w.put("sync_interval_ms", config.syncIntervalMs)
                w.put("max_file_bytes", config.maxFileBytes)
                w.put("max_parts", config.maxParts)
                w.put("stats_interval_ms", config.statsIntervalMs)
            }
            o.putRaw("meta", headerJson)
        }
        sink?.writeLine(line)
        sink?.flush(durable = true)
        written++
        partLines = 1
        bytes += line.length + 1
    }

    companion object {
        const val FORMAT = "mini4pro-flightlog-1"
        const val SCHEMA = 1
        const val MONO_CLOCK_NAME = "SystemClock.elapsedRealtimeNanos"
        const val WALL_CLOCK_NAME = "System.currentTimeMillis"
    }
}
