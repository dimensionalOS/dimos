package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.video.RawFrameInfo
import com.dimensional.mini4pro.video.RawFrameSink
import com.dimensional.mini4pro.video.RtpH264
import java.io.File
import java.io.FileOutputStream

/**
 * Where the encoded video bytes go — one file per part, beside the JSONL, sharing its session id.
 *
 * Deliberately the same shape as [LogSink]: one interface so the whole of [VideoSidecar] —
 * rotation, the budget, the parameter-set re-injection — can be driven by an in-memory
 * implementation in a JVM test. Those are the parts most likely to be wrong and least likely to be
 * exercised on a phone before a flight, which is the argument [LogSink] already makes.
 */
interface VideoSink {

    /** Human-readable name of this part, for the log line that says a part opened. */
    val name: String

    /** Bytes written to this part so far, which is also the offset the next write lands at. */
    val bytesWritten: Long

    /** Appends [length] bytes from [data] at [offset]. Returns the byte offset they landed at. */
    fun append(data: ByteArray, offset: Int, length: Int): Long

    fun flush()

    fun close()
}

/** Opens sidecar parts and deletes old ones. Mirrors [LogSinkFactory] and for the same reasons. */
interface VideoSinkFactory {

    /** Opens part [part] (1-based) of session [session]. */
    fun open(session: String, part: Int): VideoSink

    /**
     * Deletes the oldest parts of [session] beyond [keep], returning how many went.
     *
     * Oldest-first, exactly as the JSONL prunes: an approach is the *end* of a flight, so the tail
     * is what a post-mortem reads and the head is what can be spared.
     */
    fun prune(session: String, keep: Int): Int
}

/**
 * **The video half of the flight record**: encoded frames to a sidecar file, and one index line per
 * frame into the JSONL that already exists.
 *
 * `docs/apriltag-landing-recording.md` §2 is the argument; this is the build. The short version of
 * why the bytes are not in the record and their *arrival* is: 5 Mbit/s of H.264 is two orders of
 * magnitude larger than everything else the record holds, so it goes beside rather than inside —
 * but the frame's timestamp is stamped on the **same monotonic clock** as the gimbal's, the
 * altitude's and the setpoint's, first thing in the callback, so an offline tool joins them by `t`
 * with no second clock to reconcile. That property is the whole reason this is an index and not an
 * MP4.
 *
 * ## What this class guarantees
 *
 *  - **A keyframe is a seek point.** DJI's stream sends SPS/PPS ahead of some IDRs and not others.
 *    An offline tool seeking to an arbitrary keyframe is a late joiner in exactly the sense
 *    `RtpVideoSink` already handles for a late-joining receiver, so the cached parameter sets are
 *    re-injected ahead of any bare IDR and the frame is marked `psi`. Without it a record is only
 *    decodable from its first frame, which defeats the point of an index.
 *  - **The budget is spent loudly.** Silent truncation of a video record looks exactly like an
 *    aircraft that stopped delivering frames, and those two need different diagnoses. Running out
 *    is an `event` at warning severity, said once.
 *  - **An evidence problem never becomes a flight problem.** This runs on MSDK's callback thread.
 *    Every failure path here swallows, counts and carries on, exactly as [Tap]'s contract requires
 *    of everything in this package.
 *
 * ## What it deliberately does not do
 *
 * It does not decode, inspect, or validate anything beyond finding NAL boundaries. Whether those
 * bytes are a *good* frame is the decoder's business, offline, where being wrong is free.
 *
 * **Not thread-safe by construction, and it does not need to be**: MSDK delivers frames on one
 * callback thread. [close] is the exception and takes the same lock the writes do, because it can
 * arrive from the UI thread while a frame is in flight.
 */
class VideoSidecar(
    private val session: String,
    private val sinks: VideoSinkFactory,
    /**
     * The **whole session's** video budget in bytes. Once spent, frames stop being written and the
     * index stops growing — see [budgetSpent].
     *
     * There is no defensible default here, which is why it has none. `docs/video.md` works from
     * ~5 Mbit/s and that figure has never been measured; at face value it is 625 kB/s, so a
     * three-minute approach is 113 MB and an hour is 2.25 GB against a JSONL budget of 256 MB. The
     * caller has to choose, and [VideoSidecar] makes the consequence visible rather than guessing.
     */
    private val budgetBytes: Long,
    /** Bytes per part before rotating. Parts exist so that pruning can drop coarse chunks. */
    private val partBytes: Long,
    /** How many parts survive a prune. */
    private val keepParts: Int,
    /** Where index lines go — [Recorder]'s bounded queue, never a blocking write. */
    private val emit: (LogEntry) -> Unit,
    /** The one clock. `SystemClock.elapsedRealtimeNanos` in the app, hand-cranked in tests. */
    private val nowNanos: () -> Long,
    private val log: (String) -> Unit = {},
) : RawFrameSink {

    private val lock = Any()

    private var sink: VideoSink? = null
    private var part = 0
    private var frames = 0L
    private var written = 0L
    private var budgetSpent = false

    /**
     * **Whether frames are being written right now** — the operator's switch, and the only thing
     * that decides whether a frame reaches disk.
     *
     * Deliberately *inside* this class rather than gating whether the sidecar is handed the stream
     * at all. Two reasons, and the second was found the hard way. It makes the switch testable with
     * everything else it interacts with. And a sidecar that saw nothing while off could not know
     * what it had missed — it has to watch the stream continuously to know when the **next
     * keyframe** arrives, which is the only moment a file may start.
     */
    var recording: Boolean = false
        set(value) {
            synchronized(lock) {
                if (value && !field) awaitingKey = true
                field = value
            }
        }

    /**
     * True until the next keyframe, during which frames are counted and **thrown away**.
     *
     * `docs/apriltag-landing-recording.md` said a keyframe must be a seek point and built the wrong
     * mechanism for it: cached SPS/PPS re-injected ahead of a *bare* IDR. The first flight showed
     * DJI never sends one — `psi injected 0 of 16` — so that branch never fires, while the two
     * cases that matter went uncovered. Part 1 of that record began where the operator flipped the
     * switch, mid-GOP, and **90 frames were undecodable**; part 2 began where rotation happened to
     * land, and 12 were.
     *
     * Ivan's rule, and it is better than injecting anything: **start at a keyframe and accept the
     * delay.** Every byte in the file then came from the aircraft, every part is decodable from
     * byte 0, and there are no synthesised parameter sets to be subtly wrong. The cost is up to one
     * keyframe interval — 5.3 s on the measured stream — paid once when recording starts and again
     * whenever it resumes.
     */
    private var awaitingKey = true

    /** Frames dropped while waiting for a keyframe, so the delay is a number rather than a mystery. */
    private var skipped = 0L

    /**
     * Set when the current part has passed [partBytes] and is waiting for a keyframe to cut on.
     *
     * Rotation waits rather than cutting where the budget runs out, and that asymmetry is the
     * point: cutting mid-GOP would make the *next* part start undecodable, and waiting costs only
     * an overshoot — about 1.5 MB on a 32 MB part at the measured rate. A start delay is a
     * reasonable price for a file that begins; a mid-flight gap is not.
     */
    private var rotateDue = false

    /** The most recent parameter sets seen in the stream, for re-injection ahead of a bare IDR. */
    private var sps: ByteArray? = null
    private var pps: ByteArray? = null

    /** Last reported geometry, so `w`/`h`/`mime` are written on change rather than on every frame. */
    private var lastWidth: Int? = null
    private var lastHeight: Int? = null
    private var lastMime: String? = null

    /** Frames dropped because something threw. Reported at [close] rather than per frame. */
    private var failures = 0L

    /**
     * **Every frame the aircraft delivered this session, whether or not it was written.**
     *
     * The gap this closes: `video_phase` reaching `SERVING` means *we asked MSDK to start*, not
     * that anything arrived. `CameraStreamTap` has counted frames since M1 but sends them only to
     * logcat and the status screen, so a record could never answer "did video actually start" —
     * which is exactly the question a passthrough that sometimes needs a bridge restart raises
     * (Ivan, 2026-07-27). The sidecar is handed the stream regardless of the switch, so it is the
     * one thing already in a position to witness it.
     */
    private var seen = 0L
    private var lastSeenNanos = 0L
    private var announcedFirst = false
    private var stalled = false

    override fun onEncodedFrame(data: ByteArray, offset: Int, length: Int, info: RawFrameInfo) {
        // **First, before anything can fail.** The whole timestamp story is this line: the frame's
        // time is taken on the same clock every other entry uses, at the moment the bytes arrived,
        // and nothing downstream has to reconcile two clocks.
        val monoNanos = nowNanos()
        if (length <= 0) return
        synchronized(lock) {
            seen++
            lastSeenNanos = monoNanos
            if (!announcedFirst) {
                announcedFirst = true
                // The single most useful line about video in a session: it separates "the camera
                // never delivered" from "it delivered and we did not record it", which are the two
                // explanations for an empty index and want opposite responses.
                emit(
                    LogEntry.Event(
                        monoNanos, EventCode.VIDEO_FIRST_FRAME, LogEntry.SEV_INFO,
                        "first frame from the aircraft: %dx%d %s, %d bytes"
                            .format(info.width, info.height, info.mime ?: "?", length),
                    )
                )
            }
            if (stalled) {
                stalled = false
                emit(
                    LogEntry.Event(
                        monoNanos, EventCode.VIDEO_RESUMED, LogEntry.SEV_INFO,
                        "frames from the aircraft again after $seen seen",
                    )
                )
            }
            runCatching { writeLocked(monoNanos, data, offset, length, info) }
                .onFailure {
                    failures++
                    // Never rethrown: this is MSDK's decode thread, and an evidence problem here
                    // must not become a video problem. Counted, and said once at close.
                    if (failures == 1L) log("video sidecar write failed (first of possibly many): $it")
                }
        }
    }

    /** Must hold [lock]. */
    private fun writeLocked(monoNanos: Long, data: ByteArray, offset: Int, length: Int, info: RawFrameInfo) {
        if (budgetSpent) return

        // **Nothing is parsed while idle**, and nothing needs to be: whether a frame is a keyframe
        // comes from DJI's own `StreamInfo`, so the rule that decides when a file may start costs
        // one boolean rather than a walk over every NAL of every frame on the decode thread.
        if (!recording) return

        // **Nothing is written until a keyframe.** See [awaitingKey].
        if (awaitingKey) {
            if (!info.keyFrame) {
                skipped++
                return
            }
            awaitingKey = false
            if (skipped > 0) log("video sidecar started at a keyframe after skipping $skipped frame(s)")
        }

        val target = ensurePart(info.keyFrame) ?: return

        // **The only frames this parses are keyframes** — 16 in 84 s on the measured stream, about
        // 0.2 Hz — and one walk answers both questions at once: what the parameter sets are, and
        // whether this frame already carries them.
        //
        // The injection below is now belt-and-braces rather than the mechanism: since every part
        // begins at a keyframe, and DJI puts SPS/PPS in front of every one it sends, there is
        // nothing to inject. Kept because "DJI always does X" is a measurement of one airframe on
        // one firmware, and the cost of being wrong is a file that cannot be opened.
        var injected = false
        if (info.keyFrame) {
            var carries = false
            for (nal in RtpH264.nalUnits(data, offset, length)) {
                when (RtpH264.nalType(data, nal)) {
                    RtpH264.NAL_SPS -> { sps = annexB(data, nal); carries = true }
                    RtpH264.NAL_PPS -> { pps = annexB(data, nal); carries = true }
                }
            }
            if (!carries) {
                val s = sps
                val p = pps
                if (s != null && p != null) {
                    target.append(s, 0, s.size)
                    target.append(p, 0, p.size)
                    written += s.size + p.size
                    injected = true
                }
            }
        }

        val at = target.append(data, offset, length)
        written += length
        val n = ++frames

        val geometryChanged =
            info.width != lastWidth || info.height != lastHeight || info.mime != lastMime
        if (geometryChanged) {
            lastWidth = info.width
            lastHeight = info.height
            lastMime = info.mime
        }

        emit(
            LogEntry.Frame(
                monoNanos = monoNanos,
                n = n,
                part = part,
                offset = at,
                length = length + if (injected) (sps?.size ?: 0) + (pps?.size ?: 0) else 0,
                keyframe = info.keyFrame,
                parameterSets = injected,
                presentationMs = info.presentationTimeMs.takeIf { it != 0L },
                width = if (geometryChanged) info.width else null,
                height = if (geometryChanged) info.height else null,
                mime = if (geometryChanged) info.mime else null,
            )
        )

        if (written >= budgetBytes) {
            budgetSpent = true
            closeSinkLocked()
            val message = "video budget spent after $n frames / $written bytes — recording video stopped"
            log(message)
            emit(LogEntry.Event(monoNanos, EventCode.VIDEO_BUDGET_SPENT, LogEntry.SEV_WARN, message))
        }
    }

    /**
     * Must hold [lock]. Opens the first part, or rotates when this one is full **and the frame
     * about to be written is a keyframe** — so that every part begins where a decoder can start.
     */
    private fun ensurePart(keyFrame: Boolean): VideoSink? {
        val current = sink
        if (current != null) {
            if (current.bytesWritten >= partBytes) rotateDue = true
            // Not yet: cutting here would leave the next part starting mid-GOP, which is precisely
            // the defect this rule exists to fix. The part runs over its budget until a keyframe.
            if (!rotateDue || !keyFrame) return current
        }
        rotateDue = false
        current?.let {
            runCatching { it.flush() }
            runCatching { it.close() }
        }
        part += 1
        val opened = runCatching { sinks.open(session, part) }.getOrElse {
            failures++
            log("video sidecar could not open part $part: $it")
            sink = null
            return null
        }
        sink = opened
        log("video sidecar part ${opened.name} open")
        // Pruned *after* the new part exists, so a prune that goes wrong cannot leave the recorder
        // with nowhere to write.
        runCatching { sinks.prune(session, keepParts) }
            .onSuccess { if (it > 0) log("video sidecar pruned $it old part(s)") }
        return opened
    }

    /**
     * One NAL, **with its four-byte start code back on the front**. Must hold [lock].
     *
     * `RtpH264.nalUnits` reports NALs *without* their start codes, because RTP packetisation
     * strips them — the right shape for the packetiser and exactly the wrong shape here. A sidecar
     * is an Annex-B elementary stream: parameter sets written bare would leave a file whose
     * keyframes are preceded by 19 bytes no decoder can find, which is worse than not writing them
     * at all because it looks like it worked.
     */

    private fun annexB(data: ByteArray, nal: RtpH264.Nal): ByteArray =
        byteArrayOf(0, 0, 0, 1) + data.copyOfRange(nal.offset, nal.offset + nal.length)
    /** Must hold [lock]. */
    private fun closeSinkLocked() {
        sink?.let {
            runCatching { it.flush() }
            runCatching { it.close() }
        }
        sink = null
    }

    /** Ends the sidecar. Safe from any thread and safe twice. */
    fun close() {
        synchronized(lock) {
            if (sink != null || frames > 0L) {
                log("video sidecar closed: $frames frames / $written bytes in $part part(s), $failures failure(s)")
            }
            closeSinkLocked()
        }
    }

    /**
     * **The stream has gone quiet.** Called from the recorder's own sampler, because this class is
     * callback-driven and a stream that stops delivering stops calling it — the one state it
     * cannot notice by itself.
     *
     * [quietForMs] is deliberately generous. The measured stream runs at 34 fps, so three seconds
     * is a hundred missing frames: long past a hiccup, and still prompt enough to land beside the
     * `video_phase` line that will be blamed for it.
     */
    fun checkStall(nowNanos: Long, quietForMs: Long = STALL_MS) {
        synchronized(lock) {
            if (seen == 0L || stalled) return
            val quietMs = (nowNanos - lastSeenNanos) / 1_000_000
            if (quietMs < quietForMs) return
            stalled = true
            emit(
                LogEntry.Event(
                    nowNanos, EventCode.VIDEO_STALLED, LogEntry.SEV_WARN,
                    "no frame from the aircraft for %.1fs after %d seen".format(quietMs / 1000.0, seen),
                )
            )
        }
    }

    /** What was written and what was seen, for the status screen and the tests. */
    fun counters(): Counters = synchronized(lock) {
        Counters(
            frames = frames, bytes = written, parts = part, failures = failures,
            budgetSpent = budgetSpent, seen = seen, stalled = stalled,
        )
    }

    data class Counters(
        val frames: Long,
        val bytes: Long,
        val parts: Int,
        val failures: Long,
        val budgetSpent: Boolean,
        /** Frames the aircraft delivered, whether or not they were written. */
        val seen: Long,
        val stalled: Boolean,
    )

    companion object {
        /** How long the stream may be quiet before it is reported. ~100 frames at 34 fps. */
        const val STALL_MS = 3_000L
    }
}

/** Writes sidecar parts as files next to the JSONL. */
class FileVideoSinkFactory(private val dir: File) : VideoSinkFactory {

    override fun open(session: String, part: Int): VideoSink {
        dir.mkdirs()
        return FileVideoSink(File(dir, fileName(session, part)))
    }

    override fun prune(session: String, keep: Int): Int {
        val parts = dir.listFiles { f -> f.name.startsWith("$session.v") && f.name.endsWith(SUFFIX) }
            ?.sortedBy { it.name }
            ?: return 0
        if (parts.size <= keep) return 0
        var gone = 0
        for (f in parts.take(parts.size - keep)) {
            if (f.delete()) gone++
        }
        return gone
    }

    private class FileVideoSink(private val file: File) : VideoSink {
        private val out = FileOutputStream(file, true)
        override val name: String get() = file.name
        override var bytesWritten: Long = file.length()
            private set

        override fun append(data: ByteArray, offset: Int, length: Int): Long {
            val at = bytesWritten
            out.write(data, offset, length)
            bytesWritten = at + length
            return at
        }

        override fun flush() = out.flush()

        override fun close() = out.close()
    }

    companion object {
        /**
         * `.h264` rather than `.mp4` or `.bin`: it is a raw Annex-B elementary stream, which is
         * what `ffmpeg`, `ffprobe` and OpenCV all open without being told anything.
         */
        const val SUFFIX = ".h264"

        /**
         * `<session>.v001.h264`, beside `<session>.001.jsonl`. The `v` is what keeps the video
         * parts from being confused with the JSONL parts by a glob, a sort, or a person.
         */
        fun fileName(session: String, part: Int) = "%s.v%03d%s".format(session, part, SUFFIX)
    }
}
