package com.dimensional.mini4pro.vision

import android.content.Context
import android.media.MediaCodecList
import android.os.Bundle
import android.util.Log
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assume.assumeTrue
import org.junit.Test
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * **The on-phone profiler.** What decoding and detecting a real flight's video actually costs on
 * the device that would have to do it, measured rather than reasoned about.
 *
 * Run it with `tools/tagprofile`, which stages the dataset and collects the report.
 *
 * ## Why this is an instrumented test and not a debug screen
 *
 * Three reasons, in the order they decided it:
 *
 * 1. **It cannot ship.** `src/androidTest` compiles into a separate APK. A profiler that burns
 *    every core for minutes at a time is exactly the thing that must be structurally incapable of
 *    running during a flight, and "in a different APK" is a stronger guarantee than any flag.
 * 2. **It is driven entirely over adb.** The phone's one USB-C port belongs to the RC and its
 *    screen belongs to the operator; a measurement that needs neither can be run while the rig is
 *    set up for something else.
 * 3. **The input is deterministic.** The same 2374 frames, in the same order, every run — so two
 *    numbers from two designs are comparable, which is the whole reason to measure instead of
 *    guess. A live camera feed would have made every run a different experiment.
 *
 * ## What it deliberately does not do
 *
 * It never touches `MediaDataCenter`, `KeyManager`, or anything that could reach an aircraft. The
 * app process it runs in does start MSDK (`Mini4ProApp.onCreate`), which is unavoidable and
 * harmless with no RC attached — but nothing here subscribes, commands, or registers a listener.
 */
class TagProfileTest {

    private val context: Context get() = InstrumentationRegistry.getInstrumentation().targetContext
    private val args: Bundle get() = InstrumentationRegistry.getArguments()

    /**
     * Throughput at three working resolutions, plus the two baselines that make them readable:
     * an idle process, and a decoder with nothing attached.
     *
     * The five runs answer one question each, and only together:
     *
     * | run | what it isolates |
     * |---|---|
     * | `idle` | what the process costs doing nothing — the floor every other number sits on |
     * | `decode-only` | the hardware decoder alone, output buffers released unmapped |
     * | `luma-full` | + copying 1920×1080 luminance out of the decoder |
     * | `luma-half` | + copying it at 960×540 |
     * | `luma-quarter` | + copying it at 480×270 |
     */
    @Test
    fun throughput() {
        val dataset = dataset()
        val frames = frames(dataset)
        // `-e tagprofile.colour SEMI_PLANAR` runs one format instead of all three. FLEXIBLE takes
        // four minutes per run against four seconds, so being able to re-measure the fast ones
        // without re-measuring the slow one is the difference between iterating and waiting.
        val only = args.getString("tagprofile.colour")
        val colours = H264Replay.Colour.entries.filter { only == null || it.name == only }
        val codec = args.getString("tagprofile.codec")

        val load = DeviceLoad(context)
        // Take and discard one sample first. The first `registerReceiver` for the sticky battery
        // broadcast, the first sysfs walk and the class loading behind them cost seconds, and on
        // the first run that time landed inside the idle baseline and made it look like a 22 s
        // sleep. Warming it up costs nothing and stops the floor being wrong.
        load.sample()
        val report = Report(context, "throughput")
        report.line("frames in index: ${frames.size}")
        report.line("stream: ${dataset.stream.length()} bytes")
        report.line("thermal zones visible to the app: ${load.thermalZones().keys}")

        val replay = H264Replay(dataset.stream.absolutePath, frames)

        report.measured(load, "idle") {
            // Same wall clock as a decode run, so the CPU and temperature baselines compare.
            Thread.sleep(IDLE_MILLIS)
            null
        }

        report.measured(load, "decode-only") {
            replay.run("decode-only", codecName = codec, consumer = null)
        }

        for (colour in colours) {
            for (d in H264Replay.Decimation.entries) {
                val label = "luma-${colour.name.lowercase()}-${d.name.lowercase()}"
                report.measured(load, label) {
                    var checksum = 0L
                    replay.run(label, decimation = d, colour = colour, codecName = codec) { frame ->
                        // Touch the buffer so no optimiser and no lazy mapping can make the copy
                        // free. One byte from the middle is enough and costs nothing measurable.
                        checksum += frame.luma[frame.luma.size / 2].toLong()
                    }.also { report.line("  (checksum $checksum)") }
                }
            }
        }

        report.close()
    }

    /**
     * **What this phone's H.264 decoders actually offer**, printed once so the rest of the profile
     * is not guesswork.
     *
     * This exists because two plausible ways of getting pixels out of `MediaCodec` both failed on
     * this device in ways no documentation predicts — `COLOR_FormatYUV420Flexible` in ByteBuffer
     * mode works but at 10.6 fps, and an `ImageReader` fed straight from the decoder **aborts the
     * process** in `Image.getPlanes` because the buffer it hands over is a vendor-compressed
     * surface that cannot be mapped, `USAGE_CPU_READ_OFTEN` or not. When the two obvious paths
     * both fail, the next move is to read the capability table rather than guess a third time.
     */
    @Test
    fun codecs() {
        val report = Report(context, "codecs")
        val list = MediaCodecList(MediaCodecList.ALL_CODECS)
        for (info in list.codecInfos) {
            if (info.isEncoder) continue
            if (!info.supportedTypes.any { it.equals("video/avc", ignoreCase = true) }) continue
            val caps = info.getCapabilitiesForType("video/avc")
            report.line(
                "${info.name} hardware=${info.isHardwareAccelerated} " +
                    "software=${info.isSoftwareOnly} colorFormats=${caps.colorFormats.toList()}",
            )
        }
        report.close()
    }

    /**
     * **What detection costs, and how far it reaches.**
     *
     * Runs the candidate detector over every frame of the reference flight at each working
     * resolution, and writes one line per frame — index, flight time, whether the tag was found,
     * and how big it was in pixels. That file is the input to the only question that decides
     * whether a tag landing is possible at all: *at what height does this stop working.* The join
     * to altitude is done off-device, because the flight record already has it on the same clock
     * and re-deriving it here would be a second implementation of something `replay/` owns.
     *
     * It is a separate test from [throughput] because it answers a different question and takes
     * much longer, and because a detector that is not yet chosen should not be able to make the
     * decode numbers unreproducible.
     */
    @Test
    fun detect() {
        val dataset = dataset()
        val frames = frames(dataset)

        // **OpenCV's ArUco was the other candidate and it lost**, on both axes at once —
        // 108.8 ms against 182.0 at 1080p, and 100 % detection to 7 m where ArUco reached 3
        // (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md`). The dependency left the build
        // with it: 9.96 MB of APK for a comparison that is finished. The `aruco` branch and
        // `ArucoProbe` are in git at `54ef7dd` and re-running the comparison means restoring the
        // dependency line and that file, which the measurement doc spells out.
        //
        // apriltag's knobs. Defaults here are the *comparable* ones, not upstream's: one thread
        // because OpenCV's detector was single-threaded, and quad_decimate 1.0 because upstream's
        // 2.0 halves the resolution of the quad search and gives back the height that was the
        // whole reason to switch. Upstream's own defaults are measured too, by passing them.
        val nthreads = args.getString("tagprofile.nthreads")?.toIntOrNull() ?: 1
        val decimate = args.getString("tagprofile.decimate")?.toFloatOrNull() ?: 1.0f
        val refine = args.getString("tagprofile.refine")?.toBoolean() ?: true
        // **The lever the comparison left untested.** apriltag's error-correction budget: how
        // many corrected bit errors still count as a decode. Upstream's default is 2, which is
        // where `docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` measured **2 false ids in
        // 1978 frames** against OpenCV's zero — the only axis on which the incumbent was better,
        // and the axis a landing controller is least tolerant of. §6 of that document named this
        // as the first thing to measure next and said the profiler "simply does not pass it yet".
        // Now it does.
        val maxHamming = args.getString("tagprofile.maxhamming")?.toIntOrNull()
            ?: AprilTagDetector.DEFAULT_MAX_HAMMING

        val load = DeviceLoad(context)
        load.sample()
        val report = Report(context, "detect")
        assumeTrue("libapriltagjni.so did not load", AprilTagDetector.available)
        report.line(
            "detector: apriltag C library, tag36h11, nthreads=$nthreads " +
                "quad_decimate=$decimate refine_edges=$refine maxhamming=$maxHamming",
        )
        report.line("dataset: ${dataset.stream.name}, ${frames.size} frames")

        // The decode path is a knob here because detection does not care how fast the frames
        // arrive, only what is in them. FLEXIBLE is four minutes a run and known to work;
        // SEMI_PLANAR should be four seconds and is not yet proven. A detection rate must not
        // wait on a decode question.
        val colour = args.getString("tagprofile.colour")
            ?.let { name -> H264Replay.Colour.entries.first { it.name == name } }
            ?: H264Replay.Colour.SEMI_PLANAR
        val only = args.getString("tagprofile.decimation")
        val steps = H264Replay.Decimation.entries.filter { only == null || it.name == only }
        // The window is the descent, given in the record's own seconds. Frames outside it are
        // decoded and dropped: a recording is not all measurement, and this one spends 22 s on
        // the ground before it starts answering the question.
        val from = args.getString("tagprofile.from")?.toDoubleOrNull() ?: Double.NEGATIVE_INFINITY
        val to = args.getString("tagprofile.to")?.toDoubleOrNull() ?: Double.POSITIVE_INFINITY
        report.line("decode path: $colour, window ${from}..${to} s")

        val replay = H264Replay(dataset.stream.absolutePath, frames)
        for (d in steps) {
            val suffix = args.getString("tagprofile.tag")?.let { "-$it" } ?: ""
            val label = "detect-${d.name.lowercase()}$suffix"
            val april = AprilTagDetector(
                nthreads = nthreads, quadDecimate = decimate, refineEdges = refine,
                maxHamming = maxHamming,
            )
            val perFrame = StringBuilder()
            var hits = 0
            // Timed here rather than read off `consumerMeanMillis`, because the consumer also
            // checksums the frame and formats a CSV line, and neither of those is detection.
            var detectNanos = 0L
            report.measured(load, label) {
                replay.run(
                    label, decimation = d, colour = colour,
                    windowFrom = from, windowTo = to,
                ) { frame ->
                    val t0 = System.nanoTime()
                    val found = april.detect(frame.luma, frame.width, frame.height)
                    detectNanos += System.nanoTime() - t0
                    if (found.any) hits++
                    // **The proof that two detectors saw the same pixels.** A previous run of this
                    // harness read a tiled MediaCodec buffer as if it were linear and produced a
                    // detection rate that measured nothing (fixed in 351cf55). A rate is not
                    // self-validating, so the sum of the luminance bytes goes in the sidecar and
                    // the two runs' columns are diffed off-device. Identical sums mean identical
                    // input; different sums mean the comparison is void, whatever the rates say.
                    var sum = 0L
                    for (b in frame.luma) sum += (b.toInt() and 0xFF).toLong()
                    // **The two new columns are appended, not inserted.** `tools/tagcompare`
                    // and `tools/tagrate` read by position, and every published number in
                    // `docs/measurements/` came out of the first five. A column on the end is
                    // invisible to both; a column in the middle would silently make `luma_sum`
                    // read as a pixel size, and the gate that makes the comparison trustworthy
                    // would start passing on nonsense.
                    perFrame.append(frame.n).append(',')
                        .append("%.6f".format(frame.tSeconds)).append(',')
                        .append(found.ids.joinToString(" ")).append(',')
                        .append("%.2f".format(found.longestEdgePixels)).append(',')
                        .append(sum).append(',')
                        .append(found.tags.joinToString(" ") { it.hamming.toString() })
                        .append(',')
                        .append(found.tags.joinToString(" ") { "%.1f".format(it.decisionMargin) })
                        .append('\n')
                }.also {
                    report.line(
                        "  detected in $hits / ${it.framesConsumed} frames in window " +
                            "(${it.framesDecoded} decoded)",
                    )
                    val n = it.framesConsumed
                    if (n > 0) {
                        report.line(
                            "  detector alone: ${"%.2f".format(detectNanos / 1e6 / n)} ms/frame " +
                                "over $n frames",
                        )
                    }
                }
            }
            april.close()
            val session = dataset.stream.name.substringBefore('.')
            report.sidecar(
                "$session-$label.csv",
                "n,t,ids,longest_px,luma_sum,hamming,margin\n$perFrame",
            )
        }
        report.close()
    }

    /**
     * **The soak.** The working configuration held at the aircraft's real frame rate for several
     * minutes, which is the only run that can say anything about heat.
     *
     * Throughput runs finish in seconds and tell you nothing thermal — a phone takes minutes to
     * warm up, and the failure this is looking for (a sustained load that quietly throttles the
     * device the abort ladder runs on) only exists over minutes. Duration is
     * `-e tagprofile.soakSeconds`, default [SOAK_SECONDS].
     */
    @Test
    fun soak() {
        val dataset = dataset()
        val frames = frames(dataset)

        val seconds = args.getString("tagprofile.soakSeconds")?.toIntOrNull() ?: SOAK_SECONDS
        val fps = args.getString("tagprofile.soakFps")?.toDoubleOrNull() ?: RECORDED_FPS
        val clipSeconds = frames.last().tSeconds - frames.first().tSeconds
        val loops = maxOf(1, Math.ceil(seconds / clipSeconds).toInt())

        val load = DeviceLoad(context)
        val report = Report(context, "soak")
        report.line("soak: ${seconds}s requested, $loops loops of ${"%.1f".format(clipSeconds)}s at ${fps} fps")

        val replay = H264Replay(dataset.stream.absolutePath, frames)
        // The soak runs the path the design would actually ship, not the slowest one available.
        // `-e tagprofile.colour` overrides it, so the same soak can be pointed at either.
        val colour = args.getString("tagprofile.colour")
            ?.let { name -> H264Replay.Colour.entries.first { it.name == name } }
            ?: H264Replay.Colour.SEMI_PLANAR
        val detecting = args.getString("tagprofile.detect")?.toBoolean() ?: false
        // Spelt out rather than left to the constructor's defaults. `AprilTagDetector`'s defaults
        // are the *shipped* ones and they moved to 2 threads when it left this package; a soak that
        // silently followed them would stop being comparable with the runs already published at
        // one thread. `-e tagprofile.nthreads` is how the shipped configuration gets soaked.
        val soakThreads = args.getString("tagprofile.nthreads")?.toIntOrNull() ?: 1
        val probe = if (detecting && AprilTagDetector.available) {
            AprilTagDetector(nthreads = soakThreads)
        } else {
            null
        }
        report.line("soak: colour=$colour detector=${probe != null} nthreads=$soakThreads")
        report.measured(load, "soak-${colour.name.lowercase()}-half-paced") {
            var checksum = 0L
            replay.run(
                "soak-${colour.name.lowercase()}-half-paced",
                decimation = H264Replay.Decimation.HALF,
                colour = colour,
                pacedFps = fps,
                loops = loops,
            ) { frame ->
                checksum += frame.luma[frame.luma.size / 2].toLong()
                probe?.detect(frame.luma, frame.width, frame.height)
            }
        }
        probe?.close()
        report.close()
    }

    // ---------------------------------------------------------------- fixtures

    private class Dataset(val index: File, val stream: File)

    /**
     * The recorded frames, optionally truncated by `-e tagprofile.frames N`.
     *
     * Truncation is a **speed knob for iterating on the harness, not a way to take a
     * measurement**: a short run never reaches a steady state and its fps is dominated by the
     * codec's start-up. Any number quoted anywhere comes from the whole 2374.
     */
    private fun frames(dataset: Dataset): List<FrameIndex.Frame> {
        val all = FrameIndex.read(dataset.index)
        assumeTrue("no frame lines in ${dataset.index}", all.isNotEmpty())
        val limit = args.getString("tagprofile.frames")?.toIntOrNull() ?: all.size
        return if (limit < all.size) all.subList(0, limit) else all
    }

    /**
     * The staged dataset, or a skipped test saying exactly what is missing.
     *
     * Skipping rather than failing is the right verdict: a phone with no dataset pushed has not
     * failed a measurement, it has not been asked to take one, and a red test would train whoever
     * runs the suite to ignore this file.
     *
     * **Two locations are searched, and internal storage wins.** `adb push` into the app's
     * *external* files dir succeeds — the shell may write there — but the resulting files were
     * measured on 2026-07-27 to be **invisible to the app itself**: `listFiles` returns nothing
     * for a directory `ls` shows as populated. So the dataset is staged into internal storage
     * through `run-as` instead, which the app reads as its own. External is still searched second
     * because it is the convenient place and may work on another device; **reports are always
     * written to external**, since those the app creates itself and they need to be pullable.
     */
    private fun dataset(): Dataset {
        val candidates = listOf(
            File(context.filesDir, DIR_NAME),
            File(context.getExternalFilesDir(null), DIR_NAME),
        )
        // `-e tagprofile.session 20260727-192417` picks one of several staged recordings. More
        // than one lives on the phone at a time on purpose: a rate curve measured against two
        // flights that disagree is worth more than either alone, and re-pushing 35 MB over the
        // tunnel to switch between them would discourage exactly that.
        val session = args.getString("tagprofile.session")
        fun pick(dir: File, suffix: String) = dir.listFiles { f ->
            f.name.endsWith(suffix) && (session == null || f.name.startsWith(session))
        }?.minByOrNull { it.name }
        for (dir in candidates) {
            val index = pick(dir, ".jsonl")
            val stream = pick(dir, ".h264")
            if (index != null && stream != null) return Dataset(index, stream)
        }
        assumeTrue(
            "no dataset in ${candidates.map { "$it=${it.list()?.toList()}" }} " +
                "— run tools/tagprofile stage",
            false,
        )
        throw IllegalStateException("unreachable")
    }

    /**
     * Accumulates lines, prints them to logcat as it goes, and leaves a file behind.
     *
     * Both outputs exist for a reason: logcat is what a person watching a run reads, and the file
     * is what survives to be pulled, diffed, and pasted into a measurement doc. Neither is a
     * substitute for the other — logcat rotates.
     */
    private class Report(context: Context, name: String) {
        private val out = StringBuilder()

        /**
         * Internal storage, beside the dataset — **not** the external files dir.
         *
         * The obvious place is `getExternalFilesDir`, because `adb pull` reaches it directly. On
         * this device the app cannot write there at all: measured 2026-07-27,
         * `ErrnoException: open failed: EACCES`, because the directory was created by the shell
         * while staging and the app is not its owner. Chasing that ownership is a losing game —
         * internal storage is the app's by construction, and `tools/tagprofile pull` reads it
         * through `run-as`, which is the same route the dataset went in by.
         */
        private val file: File = File(
            File(context.filesDir, DIR_NAME).also { it.mkdirs() },
            "profile-$name-${SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())}.txt",
        )

        fun line(text: String) {
            Log.i(TAG, text)
            out.append(text).append('\n')
        }

        /** Run [block], bracketed by device samples, and report both the result and the cost. */
        fun measured(load: DeviceLoad, label: String, block: () -> H264Replay.Result?) {
            val before = load.sample()
            val result = block()
            val after = load.sample()
            val delta = load.delta(before, after)
            if (result == null) {
                line(
                    "$label: ${"%.1f".format(delta.seconds)}s " +
                        "cpu ${"%.2f".format(delta.coresUsed)} cores",
                )
            } else {
                line(
                    "$label: ${result.framesDecoded} frames in ${result.wallMillis} ms = " +
                        "${"%.1f".format(result.fps)} fps at ${result.outputWidth}x${result.outputHeight}; " +
                        "copy ${"%.2f".format(result.sinkMeanMillis)} ms/frame, " +
                        "consume ${"%.2f".format(result.consumerMeanMillis)} ms/frame; " +
                        "cpu ${"%.2f".format(delta.coresUsed)} cores over ${"%.1f".format(delta.seconds)}s",
                )
            }
            line("  battery ${before.batteryCelsius} -> ${after.batteryCelsius} C")
            if (after.zones.isNotEmpty()) line("  zones ${before.zones} -> ${after.zones}")
        }

        /** A bulk result too large for logcat — per-frame detections, for joining off-device. */
        fun sidecar(name: String, text: String) {
            val f = File(file.parentFile, name)
            runCatching { f.writeText(text) }
                .onSuccess { line("  wrote ${f.name} (${text.length} bytes)") }
                .onFailure { Log.w(TAG, "could not write $f", it) }
        }

        fun close() {
            runCatching { file.writeText(out.toString()) }
                .onSuccess { Log.i(TAG, "report written to $file") }
                .onFailure { Log.w(TAG, "could not write $file", it) }
        }
    }

    private companion object {
        const val TAG = "TagProfile"

        /** Where `tools/tagprofile` stages the dataset, under the app's own external files dir. */
        const val DIR_NAME = "tagprofile"

        /** The reference flight's measured rate — `datasets/…/README.md`: 2374 frames over 42.4 s. */
        const val RECORDED_FPS = 55.9

        const val SOAK_SECONDS = 300

        const val IDLE_MILLIS = 10_000L
    }
}
