package com.dimensional.mini4pro.vision

import android.content.Context
import android.os.SystemClock
import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import com.dimensional.mini4pro.Msdk
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.junit.Assume.assumeTrue
import org.junit.Test
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.atomic.AtomicLong

/**
 * **The measurement the whole tag design rests on, and the one nothing had ever taken.**
 *
 * `ICameraStreamManager.addFrameListener` has never been called by this project. Everything the
 * frame budget assumes about it — that it is dispatched from MSDK's decode looper rather than from
 * the passthrough thread, and that it avoids the 94 ms buffer-mapping cost measured on our own
 * MediaCodec path — is **read from bytecode, not observed**
 * (`docs/measurements/2026-07-27-tag-detection-rate.md` §6). This test is what turns that into
 * numbers, and it is deliberately capable of producing an answer that says stop.
 *
 * ## The four questions, in the order they decide things
 *
 * | phase | what it settles |
 * |---|---|
 * | `arrive` | does it deliver at all, at what rate, at what geometry, on which thread — and is the RTP passthrough still flowing beside it? |
 * | `copy` | what it costs to take the luminance plane out of MSDK's buffer, which is the only unavoidable per-frame cost the design pays |
 * | `detect` | the real pipeline: a single-slot mailbox and one worker at the shipped thread count, against a live 52–56 fps stream |
 * | `block` | **the safety question.** Occupy the frame-listener callback for 100 ms a frame and watch what happens to the passthrough's frame count |
 *
 * `block` is the one that can veto the design. If holding the frame callback also holds the encoded
 * receive stream, then the two are the same thread, and a detector on that thread is a detector
 * that can stall the video a pilot is flying on — no queue depth fixes that, because the stall would
 * be *upstream* of the queue. The design would have to move to a Surface or be abandoned.
 *
 * ## Why an instrumented test rather than a screen
 *
 * The reasons `TagProfileTest` gives, plus one this inherits: **it needs an aircraft**, and a
 * measurement that needs an aircraft must be a single command that runs to completion without
 * anyone touching the phone, because the phone's screen and its USB port both belong to the rig.
 *
 * ## What it does not touch
 *
 * `KeyManager`, `VirtualStickManager`, the flight-control surface, and anything that could move
 * anything. It subscribes to two read-only camera listeners and removes them. The aircraft may sit
 * on the ground with its props off for the whole run.
 */
class FrameListenerProbeTest {

    private val context: Context get() = InstrumentationRegistry.getInstrumentation().targetContext
    private val args get() = InstrumentationRegistry.getArguments()

    @Test
    fun frameListener() {
        val report = Report(context, "framelistener")
        val seconds = args.getString("frameprobe.seconds")?.toIntOrNull() ?: PHASE_SECONDS
        val threads = args.getString("frameprobe.nthreads")?.toIntOrNull()
            ?: AprilTagDetector.DEFAULT_THREADS
        val blockMs = args.getString("frameprobe.blockMs")?.toLongOrNull() ?: BLOCK_MS

        report.line("waiting for MSDK registration and an aircraft (up to ${READY_MS / 1000}s)")
        val ready = awaitReady(READY_MS)
        report.line("msdk: ${Msdk.state.value}")
        // Skipped, not failed: a phone with no aircraft attached has not failed this measurement,
        // it has not been asked to take one, and a red test here would train people to ignore it.
        assumeTrue("no registered MSDK with a connected aircraft — attach the RC and power on", ready)

        val manager = MediaDataCenter.getInstance().cameraStreamManager
        // The passthrough's own subscription, running for the whole test beside the frame listener.
        // This is the control: every number about the frame listener is only interesting relative to
        // what the *encoded* stream — the one QGroundControl is fed from — was doing at the time.
        val encoded = AtomicLong()
        val encodedListener = ICameraStreamManager.ReceiveStreamListener { _, _, _, _ ->
            encoded.incrementAndGet()
        }
        runCatching { manager.setKeepAliveDecoding(true) }
            .onFailure { report.line("setKeepAliveDecoding(true) failed: $it") }
        manager.addReceiveStreamListener(CAMERA, encodedListener)
        report.line("encoded receive-stream listener attached (the control)")
        // A stream that has not started yet would make the first phase measure a warm-up. Wait for
        // the passthrough to be alive before asking the decoded tap anything.
        val began = SystemClock.elapsedRealtime()
        while (encoded.get() == 0L && SystemClock.elapsedRealtime() - began < STREAM_MS) {
            SystemClock.sleep(200)
        }
        report.line("encoded frames before the first phase: ${encoded.get()}")

        val load = DeviceLoad(context)
        // First sample discarded: the sticky battery broadcast's first `registerReceiver` and the
        // first sysfs walk cost seconds, and on the first run that time lands inside whatever phase
        // asked for it. `TagProfileTest` learned this the same way.
        load.sample()

        try {
            // The floor every other number sits on: the process with no listener attached, and the
            // aircraft's stream still arriving into MSDK. Without it "0.78 cores" is not a cost,
            // because nobody knows what it is a cost *above*.
            phaseIdle(report, load, encoded, seconds)
            phaseArrive(report, load, manager, encoded, seconds)
            phaseCopy(report, load, manager, encoded, seconds)
            // The thread sweep is the whole reason to be on an aircraft twice. The offline table
            // was measured on an idle phone replaying a file; these are the same knobs against a
            // live airlink, a live decode and a live passthrough, which is the contention §6 of the
            // comparison says was never measured.
            for (n in threadSweep()) {
                phaseDetect(report, load, manager, encoded, seconds, n, capHz = 0.0)
            }
            // The shipped arrangement: the chosen thread count, capped at 10 Hz. What it costs here
            // is what it costs in flight, and the gap between this and the uncapped run above is
            // exactly what the cap buys.
            phaseDetect(report, load, manager, encoded, seconds, threads, capHz = CAP_HZ)
            phaseBlock(report, load, manager, encoded, seconds, blockMs)
        } finally {
            runCatching { manager.removeReceiveStreamListener(encodedListener) }
            // The flag is left as `CameraStreamTap` would leave it if the bridge were running; this
            // test never owned it exclusively and does not clear it out from under anything.
            report.close()
        }
    }

    // ───────────────────────────────────────────────────────────────── phases

    /**
     * **The floor.** No frame listener at all, the passthrough still running.
     *
     * Every cores figure below is only meaningful as a difference from this one, because the
     * process is never doing nothing: MSDK is decoding, the airlink is up and the encoded control
     * listener is counting. Reporting a detector's cost without this would report the aircraft's.
     */
    private fun phaseIdle(report: Report, load: DeviceLoad, encoded: AtomicLong, seconds: Int) {
        val before = load.sample()
        val encodedBefore = encoded.get()
        SystemClock.sleep(seconds * 1000L)
        val delta = load.delta(before, load.sample())
        report.line(
            "idle (no frame listener): encoded ${encoded.get() - encodedBefore} frames; " +
                "cpu ${"%.2f".format(delta.coresUsed)} cores over ${"%.1f".format(delta.seconds)}s; " +
                "battery ${before.batteryCelsius} C",
        )
    }

    /** Does it deliver, how fast, at what geometry, and on which thread. */
    private fun phaseArrive(
        report: Report,
        load: DeviceLoad,
        manager: ICameraStreamManager,
        encoded: AtomicLong,
        seconds: Int,
    ) {
        val stats = Stats()
        val listener = ICameraStreamManager.CameraFrameListener { data, offset, length, w, h, fmt ->
            stats.frame(w, h, length, offset, fmt?.name, data?.size ?: -1)
        }
        run(report, "arrive", load, manager, listener, encoded, seconds, stats)
    }

    /** What it costs to take plane 0 out of MSDK's buffer — the design's one unavoidable per-frame cost. */
    private fun phaseCopy(
        report: Report,
        load: DeviceLoad,
        manager: ICameraStreamManager,
        encoded: AtomicLong,
        seconds: Int,
    ) {
        val stats = Stats()
        var buffer = ByteArray(0)
        val listener = ICameraStreamManager.CameraFrameListener { data, offset, length, w, h, fmt ->
            val t0 = System.nanoTime()
            val need = w * h
            if (data != null && length >= need) {
                if (buffer.size != need) buffer = ByteArray(need)
                System.arraycopy(data, offset, buffer, 0, need)
            }
            stats.work(System.nanoTime() - t0)
            stats.frame(w, h, length, offset, fmt?.name, data?.size ?: -1)
        }
        run(report, "copy", load, manager, listener, encoded, seconds, stats)
    }

    /**
     * **The shipped pipeline, against a live stream.** One single-slot mailbox, one worker, the
     * shipped detector at the shipped thread count — the arrangement `TagRecogniser` builds, wired
     * by hand here so this test measures the parts rather than the wiring.
     */
    private fun phaseDetect(
        report: Report,
        load: DeviceLoad,
        manager: ICameraStreamManager,
        encoded: AtomicLong,
        seconds: Int,
        threads: Int,
        capHz: Double,
    ) {
        if (!AprilTagDetector.available) {
            report.line("detect: skipped — libapriltagjni.so did not load")
            return
        }
        val stats = Stats()
        val mailbox = LatestFrame()
        val gate = if (capHz > 0.0) RateCap(capHz) else null
        val detector = AprilTagDetector(nthreads = threads)
        val detected = AtomicLong()
        val detectNanos = AtomicLong()
        val worker = Thread({
            while (!Thread.currentThread().isInterrupted) {
                val frame = mailbox.take(500) ?: continue
                val t0 = System.nanoTime()
                val found = runCatching { detector.detect(frame.luma, frame.width, frame.height) }
                    .getOrDefault(Found.NOTHING)
                detectNanos.addAndGet(System.nanoTime() - t0)
                detected.incrementAndGet()
                if (found.any) stats.hit()
            }
        }, "tag-probe-detect").apply { isDaemon = true; start() }

        val listener = ICameraStreamManager.CameraFrameListener { data, offset, length, w, h, fmt ->
            val t0 = System.nanoTime()
            // The cap lives on the **producer** side, exactly where `TagRecogniser` puts it: a frame
            // rejected here costs one clock read, while a frame accepted and then thrown away by the
            // consumer would already have cost the 2 MB copy.
            if (data != null && length >= w * h &&
                (gate == null || gate.admit(SystemClock.elapsedRealtimeNanos()))
            ) {
                mailbox.offer(data, offset, w, h, SystemClock.elapsedRealtimeNanos())
            }
            stats.work(System.nanoTime() - t0)
            stats.frame(w, h, length, offset, fmt?.name, data?.size ?: -1)
        }
        val label = "detect(nthreads=$threads" + (if (capHz > 0) ", cap ${capHz}Hz" else "") + ")"
        run(report, label, load, manager, listener, encoded, seconds, stats)
        worker.interrupt()
        worker.join(2_000)
        detector.close()
        val n = detected.get()
        report.line(
            "  detector: $n frames in ${seconds}s = ${"%.1f".format(n.toDouble() / seconds)} Hz, " +
                (if (n > 0) "${"%.1f".format(detectNanos.get() / 1e6 / n)} ms/frame" else "—") +
                ", tag in ${stats.hits} of them",
        )
        report.line("  mailbox: ${mailbox.offered} offered, ${mailbox.dropped} dropped oldest")
    }

    /**
     * **The veto.** Hold the frame-listener callback for [blockMs] a frame and watch the *encoded*
     * stream's rate.
     *
     * If the passthrough's frames per second falls with it, the two share a thread and a detector on
     * that callback can stall the video a pilot is flying on. No queue depth can fix that, because
     * the stall is upstream of the queue.
     */
    private fun phaseBlock(
        report: Report,
        load: DeviceLoad,
        manager: ICameraStreamManager,
        encoded: AtomicLong,
        seconds: Int,
        blockMs: Long,
    ) {
        val stats = Stats()
        val listener = ICameraStreamManager.CameraFrameListener { _, offset, length, w, h, fmt ->
            val t0 = System.nanoTime()
            SystemClock.sleep(blockMs)
            stats.work(System.nanoTime() - t0)
            stats.frame(w, h, length, offset, fmt?.name, -1)
        }
        run(report, "block(${blockMs}ms)", load, manager, listener, encoded, seconds, stats)
    }

    /** Thread counts to sweep, `-e frameprobe.sweep 1,2,4`. */
    private fun threadSweep(): List<Int> =
        args.getString("frameprobe.sweep")?.split(',')?.mapNotNull { it.trim().toIntOrNull() }
            ?.takeIf { it.isNotEmpty() }
            ?: listOf(1, 2, 4)

    // ──────────────────────────────────────────────────────────────── harness

    private fun run(
        report: Report,
        label: String,
        load: DeviceLoad,
        manager: ICameraStreamManager,
        listener: ICameraStreamManager.CameraFrameListener,
        encoded: AtomicLong,
        seconds: Int,
        stats: Stats,
    ) {
        val encodedBefore = encoded.get()
        val before = load.sample()
        val t0 = SystemClock.elapsedRealtime()
        manager.addFrameListener(CAMERA, ICameraStreamManager.FrameFormat.NV21, listener)
        SystemClock.sleep(seconds * 1000L)
        manager.removeFrameListener(listener)
        val wallMs = SystemClock.elapsedRealtime() - t0
        val delta = load.delta(before, load.sample())
        val encodedFrames = encoded.get() - encodedBefore
        report.line(
            "$label: decoded ${stats.frames} in ${wallMs} ms = " +
                "${"%.1f".format(stats.frames * 1000.0 / wallMs)} fps; " +
                "encoded (passthrough) $encodedFrames = " +
                "${"%.1f".format(encodedFrames * 1000.0 / wallMs)} fps",
        )
        report.line(
            "  cpu ${"%.2f".format(delta.coresUsed)} cores; " +
                "battery ${before.batteryCelsius} -> ${load.sample().batteryCelsius} C",
        )
        report.line("  geometry ${stats.geometry}, formats ${stats.formats}, offsets ${stats.offsets}")
        report.line("  threads ${stats.threads}")
        report.line("  in-callback work ${stats.workSummary()}")
        report.line("  gaps ${stats.gapSummary()}")
    }

    /** True once MSDK is registered and an aircraft is connected, or [timeoutMs] has passed. */
    private fun awaitReady(timeoutMs: Long): Boolean {
        val until = SystemClock.elapsedRealtime() + timeoutMs
        while (SystemClock.elapsedRealtime() < until) {
            val s = Msdk.state.value
            if (s.registered && s.productConnected) return true
            SystemClock.sleep(500)
        }
        val s = Msdk.state.value
        return s.registered && s.productConnected
    }

    /**
     * What one phase saw. Every field is written from the callback thread and read after it has
     * been removed, so no synchronisation is needed and none is claimed.
     */
    private class Stats {
        var frames = 0L; private set
        var hits = 0L; private set
        val geometry = LinkedHashSet<String>()
        val formats = LinkedHashSet<String>()
        val offsets = LinkedHashSet<String>()
        val threads = LinkedHashSet<String>()
        private var lastNanos = 0L
        private val gaps = ArrayList<Long>(4096)
        private var workNanos = 0L
        private var workCount = 0L

        fun frame(w: Int, h: Int, length: Int, offset: Int, format: String?, bufferSize: Int) {
            frames++
            val now = System.nanoTime()
            if (lastNanos != 0L && gaps.size < 100_000) gaps.add(now - lastNanos)
            lastNanos = now
            if (geometry.size < 8) {
                // `len` against `w*h*3/2` is the check that says NV21 is really NV21 and that the
                // luminance plane is tightly packed. A stride MSDK never mentions would show up
                // here as a length that is not the one the format implies.
                geometry.add("${w}x$h len=$length expect_nv21=${w * h * 3 / 2} buf=$bufferSize")
            }
            if (formats.size < 8) formats.add(format ?: "null")
            if (offsets.size < 8) offsets.add(offset.toString())
            if (threads.size < 8) threads.add(Thread.currentThread().name)
        }

        fun hit() { hits++ }

        fun work(nanos: Long) { workNanos += nanos; workCount++ }

        fun workSummary(): String =
            if (workCount == 0L) "none" else "${"%.3f".format(workNanos / 1e6 / workCount)} ms/frame"

        /**
         * The gap distribution, not the mean. A mean inter-arrival time cannot tell a steady 55 fps
         * from a stream that arrives in bursts with 300 ms holes in it, and a detector sampling at
         * 10 Hz cares which.
         */
        fun gapSummary(): String {
            if (gaps.isEmpty()) return "none"
            val sorted = gaps.sorted()
            fun pct(p: Double) = "%.1f".format(sorted[(sorted.size * p).toInt().coerceAtMost(sorted.lastIndex)] / 1e6)
            return "p50 ${pct(0.5)} p90 ${pct(0.9)} p99 ${pct(0.99)} max ${"%.1f".format(sorted.last() / 1e6)} ms"
        }
    }

    /** Same file discipline as `TagProfileTest.Report`, and the same reason: logcat rotates. */
    private class Report(context: Context, name: String) {
        private val out = StringBuilder()
        private val file = File(
            File(context.filesDir, "tagprofile").also { it.mkdirs() },
            "probe-$name-${SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())}.txt",
        )

        fun line(text: String) {
            Log.i(TAG, text)
            out.append(text).append('\n')
        }

        fun close() {
            runCatching { file.writeText(out.toString()) }
                .onSuccess { Log.i(TAG, "report written to $file") }
                .onFailure { Log.w(TAG, "could not write $file", it) }
        }
    }

    private companion object {
        const val TAG = "FrameProbe"
        val CAMERA: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN

        /**
         * Per phase. Long enough for a gap distribution and a CPU average, short enough that eight
         * phases stay inside two minutes of an aircraft that heats up sitting on the ground.
         */
        const val PHASE_SECONDS = 15

        /** The shipped cap, measured as shipped. */
        const val CAP_HZ = RateCap.DEFAULT_HZ

        /** How long to wait for registration and an aircraft. Registration is an online activation. */
        const val READY_MS = 60_000L

        /** How long to wait for the first encoded frame before starting anyway, and saying so. */
        const val STREAM_MS = 20_000L

        /** The deliberate stall. Comfortably longer than one frame interval at 55 fps. */
        const val BLOCK_MS = 100L
    }
}
