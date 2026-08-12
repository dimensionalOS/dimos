package com.dimensional.mini4pro.zenoh

import android.os.SystemClock
import android.util.Log
import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.Severity
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Recorder
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.StateSource
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/**
 * The **thin adapter** between the Zenoh publisher and the two things that cannot be unit-tested:
 * Android and the DJI SDK.
 *
 * Written to `record/Recorder`'s shape, deliberately and almost line for line, because the problem
 * is the same one: a periodic sampler, a bounded queue, a consumer that must never block a
 * producer, and drops that are counted rather than swallowed. Everything in this package except
 * this file and [ZenohKotlinSink] imports neither `android.*` nor `dji.*`, which is what keeps the
 * catalogue, the gating, the origin rule, the cadence and the queue all testable on the JVM. This
 * file is deliberately dull: it supplies clocks, reads a snapshot, and forwards bytes.
 *
 * ## What it does, in order
 *
 *  1. Holds a [ZenohPublisher] — one bounded queue and one thread, which owns the session.
 *  2. Runs a sampler at [Config.hz], reading `telemetry/StateSource` — the aircraft, or a
 *     recording an operator switched onto this bus — and hands each snapshot to
 *     [ZenohTelemetryPump]. The pump cannot tell which, deliberately.
 *  3. Attaches an [Announcer.Sink] so every operator sentence reaches the bus as well as QGC.
 *
 * ## Why the sampler is its own thread and not `Bridge.tick`
 *
 * Three reasons, and the first is the one that matters. **`Bridge.tick` is the MAVLink emitter**:
 * it is what draws the aircraft moving on a ground station, and hanging a second transport's work
 * off it means any cost here is paid by the thing an operator is watching. Second, the cadences
 * differ — `Bridge` ticks at 5 Hz because that is MAVLink's fast set, and this bus's rates are
 * per-channel ceilings that `ZenohTelemetryPump.Cadence` owns. Third, `Bridge.tick` stops when the
 * link stops, and the Zenoh bus has no reason to stop because a UDP socket to QGC did.
 *
 * Nothing on this thread can reach the flight-control path. It reads immutable snapshots
 * (`StateSource.read`), calls pure functions, and `offer`s onto a bounded queue. It
 * calls nothing that can command an aircraft, and there is no path by which it could — this class
 * has no inbound half at all, which is the whole of the argument that the live publisher cannot
 * move anything.
 *
 * ## The announcement sink, and the one containment this project did not previously need
 *
 * `command/Announcer` says of itself: *"**No containment.** A throwing sink propagates… with one
 * sink it cannot matter, and with two it wants a decision that belongs with the transport that
 * raises the question."* This is that transport, and this is the decision: **a Zenoh failure must
 * never cost the MAVLink sink its sentence.** So [statusSink] contains its own throws. The reason
 * is not tidiness — `Announcer.say` is called from the guided engine's 10 Hz thread, from the
 * `mavlink-rx` thread and from DJI's own callback threads, and every one of those is a thread on
 * which an exception is somebody's flight.
 *
 * It cannot in fact throw: [ZenohPublisher.offer] is a queue offer and contains everything. The
 * `try` is there because *"it cannot throw today"* is exactly the sentence that precedes the
 * commit where it can.
 */
object ZenohBus {

    private const val TAG = "ZenohBus"

    data class Config(
        /**
         * How often `AircraftState` is sampled, in hertz.
         *
         * 10 Hz — the measured rate of the DJI position feed (9.88 Hz on landing10, the
         * measurement `ZenohTelemetryPump.Cadence.poseMs` carries), because a sampler slower
         * than the feed makes every faster ceiling unreachable: `pose`/`tf` at their 100 ms
         * ceilings can only publish as often as they are *asked*, and the old 5 Hz sampler was
         * exactly the 20–30 cm-per-sample stepping Ivan measured in mem2 replays. The
         * per-channel ceilings in [ZenohTelemetryPump.Cadence] still decide the rates; this is
         * only how often they are offered a sample.
         */
        val hz: Double = 10.0,
        val publisher: ZenohPublisher.Settings = ZenohPublisher.Settings(),
        val cadence: ZenohTelemetryPump.Cadence = ZenohTelemetryPump.Cadence(),
        /**
         * Whether to bring the video publisher up at all.
         *
         * **False builds no second publisher and opens no second session**, which is the point:
         * off means the transport does not exist, not that it exists and is quiet. `ZenohSettings`
         * resolves it from a saved switch and a launch extra, both defaulting off.
         */
        val video: Boolean = false,
        val videoSettings: ZenohVideoPublisher.Settings = ZenohVideoPublisher.Settings(),
        /**
         * Whether the `detections` channel is wanted this session.
         *
         * Unlike [video] this opens **no second session and builds nothing**: a detection is
         * 552 bytes at 10 Hz and rides the telemetry publisher's existing queue. False means
         * [publishDetection] returns before it encodes, so an off channel costs one volatile read
         * per sighting and the detector carries on feeding the flight record exactly as before.
         *
         * Off by default. `ZenohSettings.PREF_DETECTIONS` argues why a channel this cheap is
         * still a switch.
         */
        val detections: Boolean = false,
    )

    @Volatile private var publisher: ZenohPublisher? = null
    @Volatile private var pump: ZenohTelemetryPump? = null
    @Volatile private var sampler: ScheduledExecutorService? = null
    @Volatile private var announcer: Announcer? = null
    @Volatile private var target: String? = null

    /**
     * The video half, or null when the bus is off. **A separate publisher with a separate session**
     * — see [ZenohVideoPublisher] for why that is structural and not an accident of layering.
     */
    @Volatile private var video: ZenohVideoPublisher? = null

    /**
     * The `detections` switch, and the counters behind it.
     *
     * Written under [start]/[stop]'s lock and read on the `tag-detect` worker, hence volatile.
     * [detectionSeq] is `header.seq` and is a plain count of messages put on the channel this
     * session; it is incremented only on the worker thread, which is the only thread that ever
     * produces a sighting, so no atomic is needed and none is claimed.
     */
    @Volatile private var detections = false

    @Volatile private var detectionSeq = 0

    /** Sightings offered, and the reason the last one gave. For the status screen. */
    @Volatile private var detectionsSeen = 0L

    @Volatile private var detectionsPublished = 0L

    @Volatile private var lastDetectionReason: Withheld? = null

    /**
     * The `tag_fix` channel's counters — same worker, same switch, same discipline as the
     * `detections` counters above. `tagFixSeq` counts published fixes and is touched only on
     * the `tag-detect` worker; `seen` counts sightings *offered*, so `seen − published` is the
     * count of sightings the record itself had no fix for.
     */
    @Volatile private var tagFixSeq = 0

    @Volatile private var tagFixesSeen = 0L

    @Volatile private var tagFixesPublished = 0L

    @Volatile private var lastTagFixReason: Withheld? = null

    /** The `setpoint` channel's counters — sends offered, messages published. */
    @Volatile private var setpointsSeen = 0L

    @Volatile private var setpointsPublished = 0L

    @Volatile private var lastSetpointReason: Withheld? = null

    /**
     * The `wind` channel's counters — key deliveries offered, messages published.
     * `seen − published` is the count of value-less deliveries (DJI withdrawing the
     * reading, [Withheld.WIND_MISSING]), which is a normal disconnect, not a fault.
     */
    @Volatile private var windSeen = 0L

    @Volatile private var windPublished = 0L

    @Volatile private var lastWindReason: Withheld? = null

    /** Samples taken, and samples on which the publisher took at least one message. */
    @Volatile private var samples = 0L

    @Volatile private var samplesPublished = 0L

    val isRunning: Boolean get() = publisher?.isRunning == true

    /** Where the bus is, for the status screen. Null when stopped. */
    fun target(): String? = target

    fun counters(): ZenohPublisher.Counters? = publisher?.counters()

    fun phase(): ZenohPublisher.Phase = publisher?.counters()?.phase ?: ZenohPublisher.Phase.STOPPED

    /** The reason each channel gave on the most recent sample, for the status screen. */
    fun lastReasons(): Map<ZenohChannel, Withheld> = pump?.lastReasons ?: emptyMap()

    fun datumOrigin(): ZenohTelemetryPump.DatumOrigin =
        pump?.datumOrigin ?: ZenohTelemetryPump.DatumOrigin.NONE

    fun sampleCount(): Long = samples

    fun publishedSampleCount(): Long = samplesPublished

    /**
     * The video sink to hand MSDK's frames, or null when the bus or the video switch is off.
     *
     * Read fresh on every frame by `Bridge.startVideo`'s `rawSink`, for the reason that lambda
     * already gives about the recorder's sidecar: the bus starts and stops independently of the
     * video loop, and a streamer holding a publisher from a finished session would offer frames
     * into a queue nobody drains.
     */
    fun videoSink(): com.dimensional.mini4pro.video.RawFrameSink? = video

    /** What the video channel has done this session, or null when it is not running. */
    fun videoCounters(): ZenohVideoPublisher.Counters? = video?.counters()

    /**
     * What the `detections` channel has done this session, or null when the switch is off.
     *
     * Null rather than a zeroed record, for the reason the whole bus follows: *"off means the
     * transport does not exist, not that it exists and is quiet"*, and a zero beside a detector
     * that has been seeing tags all flight would say the opposite.
     */
    fun detectionCounters(): DetectionCounters? =
        if (!detections) null
        else DetectionCounters(detectionsSeen, detectionsPublished, lastDetectionReason)

    /**
     * @param seen sightings the detector handed us, whether or not they reached the bus.
     * @param published messages actually offered to the publisher's queue. `seen - published` is
     *   the count refused by [ZenohEmission.detectionReason], and [lastReason] says why the most
     *   recent one was — not a queue drop, which is [ZenohPublisher.Counters.dropped]'s.
     */
    data class DetectionCounters(
        val seen: Long,
        val published: Long,
        val lastReason: Withheld?,
    )

    /**
     * The camera geometry the intrinsics are built for, held across frames.
     *
     * Written on MSDK's decode thread and read on the sampler's, hence volatile. It is held here
     * rather than read out of `VideoStreamer` so that `zenoh/` keeps importing nothing from
     * `video/` but the two plain seam types — and so that the holding rule is one line, in the
     * place the contract argues for it.
     */
    @Volatile private var geometry: Pair<Int, Int>? = null

    /**
     * Records a frame's stated geometry. Called from the same fan-out that feeds [videoSink].
     *
     * **Always, whether or not video is being published**, because `camera_info` is deliberately
     * not gated on the video switch: a consumer that has the pixels from QGC's RTP stream still
     * needs to know that DJI's published field of view is 13.2 % wrong for them.
     */
    fun noteVideoGeometry(width: Int, height: Int) {
        if (width <= 0 || height <= 0) return
        val next = width to height
        if (geometry != next) geometry = next
    }

    /**
     * Where the camera is pointing, in DJI's own earth-referenced convention — installed by
     * `Bridge` and cleared with the link.
     *
     * A supplier rather than a push, exactly as `Recorder.gimbalSource` is and for the same three
     * reasons that comment gives: `gimbal/` needs no change at all, `GimbalAim.reading()` is
     * already the public immutable snapshot that package chose to expose, and the decision about
     * *which* angle to prefer belongs beside the two things that hold them rather than inside a
     * transport.
     *
     * Null — the default, and the state a stopped bridge is in — omits the `base_link` → `camera`
     * edge from the tree. The tree is still published; the camera is simply not in it.
     */
    @Volatile
    var gimbalSource: (() -> GimbalEarthAttitude?)? = null

    // ─────────────────────────────────────────────────────────────────────────
    // lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Brings the bus up. **Opens nothing and can refuse nothing** — see [ZenohPublisher.start].
     *
     * Safe to call when the MSDK is not registered and when the aircraft is not connected: the
     * sampler reads an all-null [AircraftState] in that case and [ZenohEmission] withholds
     * everything, which is the honest state of affairs and is exactly what a subscriber should
     * see. A bus that published zeros before the aircraft arrived would be worse than a quiet one.
     *
     * @param announcer the fan-out to attach the `status` channel to, or null to publish telemetry
     *   only. Attached here rather than at construction because the sink must go away with the
     *   session — an `Announcer` holding a sink for a stopped publisher would offer every sentence
     *   into a queue nobody drains.
     * @param factory the transport. Defaulted to the real one; a test or a bench harness passes
     *   its own.
     */
    @Synchronized
    fun start(
        config: ZenohConfig,
        cfg: Config = Config(),
        announcer: Announcer? = null,
        factory: ZenohSinkFactory = ZenohKotlinSink.FACTORY,
    ) {
        if (publisher != null) return
        // The one Android-version gate, taken here rather than inside the transport so the
        // refusal is a sentence an operator can read instead of an `UnsatisfiedLinkError` out of
        // a static initialiser. The AAR declares `minSdkVersion 30` and this app declares 24; the
        // manifest overrides that rather than raising the whole app's floor for an optional,
        // off-by-default transport, and this is the other half of that decision.
        if (factory === ZenohKotlinSink.FACTORY && !ZenohKotlinSink.supportedHere()) {
            val why = "Zenoh needs Android ${ZenohKotlinSink.SUPPORTED_SDK}; this device is " +
                "${android.os.Build.VERSION.SDK_INT}. Publishing is off for this session."
            Log.w(TAG, why)
            recordEvent(EventCode.ZENOH_ERROR, why, LogEntry.SEV_ERROR)
            announcer?.say(Severity.ERROR, why)
            return
        }
        samples = 0L
        samplesPublished = 0L
        detectionSeq = 0
        detectionsSeen = 0L
        detectionsPublished = 0L
        lastDetectionReason = null
        tagFixSeq = 0
        tagFixesSeen = 0L
        tagFixesPublished = 0L
        lastTagFixReason = null
        setpointsSeen = 0L
        setpointsPublished = 0L
        lastSetpointReason = null
        windSeen = 0L
        windPublished = 0L
        lastWindReason = null
        target = config.connectEndpoint

        val pub = ZenohPublisher(
            config = config,
            factory = factory,
            settings = cfg.publisher,
            nowMs = { SystemClock.elapsedRealtime() },
            log = { msg -> Log.w(TAG, msg) },
            onPhase = ::onPhase,
        )
        val p = ZenohTelemetryPump(
            sink = { channel, bytes -> pub.offer(channel, bytes) },
            cadence = cfg.cadence,
            gimbal = { gimbalSource?.invoke() },
            resolution = { geometry },
            // The datum announcements, and only those. Nothing per-sample reaches an operator.
            announce = { text -> this.announcer?.say(Severity.ERROR, text) },
            log = { msg -> Log.i(TAG, msg) },
        )
        p.reset()
        publisher = pub
        pump = p
        pub.start()

        // **After the publisher exists, never before.** The detector's worker is already running
        // — `Bridge.startTagRecogniser` starts it with the link, not with the bus — so a sighting
        // can land on `publishDetection` at any instant, and the switch is the only thing standing
        // between it and a null publisher. Set last so that "on" always implies "there is
        // somewhere for it to go"; cleared first in `stop()` for the mirror-image reason.
        detections = cfg.detections
        if (cfg.detections) {
            recordEvent(
                EventCode.ZENOH_START,
                "detections + tag_fix channels on: camera-frame sightings and world-frame " +
                    "fixes; the pose is coarse (fitted focal length, assumed principal " +
                    "point) per the contract row",
                LogEntry.SEV_INFO,
            )
        }

        // The wind tap, installed by the same rule as the `detections` flag above: after the
        // publisher exists, never before, because DJI's key callback can deliver at any
        // instant and the sink is what stands between it and a null publisher. The sink lives
        // on `Recorder` because that is where the one `KeyWindSpeed` listener and its
        // on-change dedup already are — a second listener here would be two owners of one
        // delivery stream. Cleared first in `stop()`, mirror-image.
        Recorder.windSink = ::publishWind

        // **The second publisher, and only when it is asked for.** Its own queue, its own thread
        // and its own zenoh session — `ZenohVideoPublisher`'s KDoc argues each of the three, and
        // the property they buy is that no video frame can delay, evict or block a telemetry one.
        // Built after the telemetry publisher has already started, so that a failure constructing
        // it cannot cost the bus its telemetry.
        if (cfg.video) {
            val v = ZenohVideoPublisher(
                config = config,
                factory = factory,
                settings = cfg.videoSettings,
                nowMs = { SystemClock.elapsedRealtime() },
                nowUnixMs = { System.currentTimeMillis() },
                log = { msg -> Log.w(TAG, msg) },
                onPhase = { phase, why -> Log.i(TAG, "zenoh video phase → $phase ($why)") },
            )
            video = v
            v.enabled = true
            v.start()
            recordEvent(
                EventCode.ZENOH_START,
                "video channel on: a second session to ${config.connectEndpoint}, " +
                    "queue ${cfg.videoSettings.queueCapacity} frames",
                LogEntry.SEV_INFO,
            )
        }

        announcer?.let {
            this.announcer = it
            it.attach(statusSink)
        }

        recordEvent(
            EventCode.ZENOH_START,
            "publishing to ${config.connectEndpoint} as ${config.prefix} at ${cfg.hz} Hz",
            LogEntry.SEV_INFO,
        )

        val periodMs = (1000.0 / cfg.hz).toLong().coerceAtLeast(1)
        sampler = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "zenoh-sample").apply { isDaemon = true }
        }.also {
            it.scheduleAtFixedRate(::sample, 0, periodMs, TimeUnit.MILLISECONDS)
        }
        Log.i(TAG, "zenoh bus up → ${config.connectEndpoint} (${config.prefix}) at ${cfg.hz} Hz")
    }

    /**
     * Takes the bus down.
     *
     * The order is the recorder's, and for the recorder's reason: **detach the producers first**,
     * so nothing is still offering into a publisher that is closing. The sink comes off the
     * announcer before the publisher stops, and the sampler is shut down before either.
     */
    @Synchronized
    fun stop() {
        sampler?.shutdownNow()
        sampler = null
        // First among the producers, and before the publisher is touched: the detector's worker
        // reads this on every sighting and the flag is what keeps it away from a closing
        // publisher. Same order, same reason, as the announcer and the video sink below.
        detections = false
        // Beside the flag, for the flag's reason: DJI's key callback reads this on every
        // wind delivery, and the null is what keeps it away from a closing publisher.
        Recorder.windSink = null
        val windCounters = DetectionCounters(windSeen, windPublished, lastWindReason)
        val tagCounters = DetectionCounters(detectionsSeen, detectionsPublished, lastDetectionReason)
        val fixCounters = DetectionCounters(tagFixesSeen, tagFixesPublished, lastTagFixReason)
        val spCounters = DetectionCounters(setpointsSeen, setpointsPublished, lastSetpointReason)
        announcer?.detach(statusSink)
        announcer = null
        // Detached before it is stopped, and before the telemetry publisher is touched: MSDK's
        // decode thread reads `videoSink()` on every frame, so the null has to land first or a
        // frame can arrive at a publisher that is closing. Same order, same reason, as the
        // announcer above.
        val vid = video
        video = null
        vid?.enabled = false
        val videoCounters = vid?.counters()
        vid?.stop()
        geometry = null
        val pub = publisher
        publisher = null
        val counters = pub?.counters()
        pub?.stop()
        pump = null
        target = null
        if (tagCounters.seen > 0) {
            recordEvent(
                EventCode.ZENOH_STOP,
                "detections: seen ${tagCounters.seen}, published ${tagCounters.published}" +
                    (tagCounters.lastReason?.let { ", last reason $it" } ?: ""),
                if (tagCounters.published < tagCounters.seen) LogEntry.SEV_WARN
                else LogEntry.SEV_INFO,
            )
        }
        if (fixCounters.seen > 0) {
            recordEvent(
                EventCode.ZENOH_STOP,
                "tag_fix: seen ${fixCounters.seen}, published ${fixCounters.published}" +
                    (fixCounters.lastReason?.let { ", last reason $it" } ?: ""),
                // `seen − published` here is FIX_UNAVAILABLE — sightings the record itself has
                // no fix for — which is a normal flight, not a fault.
                LogEntry.SEV_INFO,
            )
        }
        if (spCounters.seen > 0) {
            recordEvent(
                EventCode.ZENOH_STOP,
                "setpoint: sends seen ${spCounters.seen}, published ${spCounters.published}" +
                    (spCounters.lastReason?.let { ", last reason $it" } ?: ""),
                if (spCounters.published < spCounters.seen) LogEntry.SEV_WARN
                else LogEntry.SEV_INFO,
            )
        }
        if (windCounters.seen > 0) {
            recordEvent(
                EventCode.ZENOH_STOP,
                "wind: deliveries seen ${windCounters.seen}, published " +
                    "${windCounters.published}" +
                    (windCounters.lastReason?.let { ", last reason $it" } ?: ""),
                // `seen − published` is WIND_MISSING — DJI withdrawing the reading on
                // disconnect — which is a normal end of session, not a fault.
                LogEntry.SEV_INFO,
            )
        }
        videoCounters?.let {
            recordEvent(
                EventCode.ZENOH_STOP,
                "video: seen ${it.seen}, published ${it.published} frames / " +
                    "${it.bytesPublished} bytes, dropped ${it.framesDropped} frames in " +
                    "${it.gopsDropped} GOP(s), ${it.framesAwaitingKey} skipped awaiting a " +
                    "keyframe, ${it.framesRefused} refused, peak queue ${it.peakQueued}",
                if (it.gopsDropped > 0) LogEntry.SEV_WARN else LogEntry.SEV_INFO,
            )
        }
        counters?.let {
            recordEvent(
                EventCode.ZENOH_STOP,
                "published ${it.published}, dropped ${it.dropped}, discarded ${it.discarded}, " +
                    "failures ${it.failures}, opens ${it.opens}, peak queue ${it.peakQueued}",
                if (it.dropped > 0) LogEntry.SEV_WARN else LogEntry.SEV_INFO,
            )
        }
        Log.i(TAG, "zenoh bus down")
    }

    // ─────────────────────────────────────────────────────────────────────────
    // producers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One sample of the aircraft onto the bus.
     *
     * The `try` around the whole body is the same rule `Recorder.sample` and `Bridge.tick` state:
     * **a bad value must not kill the loop**, because a dead sampler looks exactly like a
     * disconnected aircraft. It is also where [ZenohEmission]'s drift check lands if it ever fires
     * on a real flight — loudly, in logcat and the flight record, without stopping anything.
     */
    private fun sample() {
        val p = pump ?: return
        try {
            // **The outbound seam.** The aircraft, or a recording an operator switched onto this
            // bus — `telemetry/StateSource` decides, and this loop deliberately cannot tell.
            // Everything after this line is unchanged: the same gates, the same cadences, the
            // same withholding, because a replayed sample carries the staleness the aircraft
            // actually had and the pump has no reason to treat it differently.
            val state = StateSource.read(StateSource.Sink.ZENOH)
            samples++
            val sent = p.sample(
                s = state,
                monoMs = SystemClock.elapsedRealtime(),
                unixMs = System.currentTimeMillis(),
            )
            if (sent > 0) samplesPublished++
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh sample failed", e)
            recordEvent(
                EventCode.ZENOH_ERROR,
                "sample failed: ${e.javaClass.simpleName}: ${e.message}",
                LogEntry.SEV_ERROR,
            )
        }
    }

    /**
     * The `status` channel: every operator sentence, as a `std_msgs.String`, `NEVER_DROP`.
     *
     * Fed by the same `Announcer` that writes QGC's `STATUSTEXT`, which is what makes the two
     * interfaces structurally unable to disagree — §4.4 of `docs/zenoh-dimos-transport.md` wants
     * *"a QGC operator must not be blind to a DiMOS goto"* and the reverse, and a fan-out is how
     * that becomes a property rather than a rule people follow.
     *
     * **The severity is not carried**, and that is the contract's choice rather than an omission:
     * `docs/zenoh-topics.md` types this channel `std_msgs.String` and a subscriber decoding it
     * gets a sentence. The severity is on the MAVLink side because `STATUSTEXT` has a field for
     * it. A structured form belongs on `health`, which is JSON and has one.
     */
    /**
     * **One tag sighting onto the bus**, called on the detector's `tag-detect` worker thread.
     *
     * This is the far end of `vision/RecordedTagSink`'s `downstream` lambda and there is
     * deliberately no second tap on the detector: the record is written *before* this is called
     * and does not depend on it, which is the ordering that file establishes so that evidence is
     * never contingent on what was done with it.
     *
     * ## What runs here, and why it is safe on that thread
     *
     * A volatile read, a clock read, a pure encode of ~550 bytes, and a bounded-time queue
     * offer. No I/O, no lock, nothing that can block — [ZenohPublisher.offer]'s contract. The
     * worker is not a flight-control thread and the detector already spends 25–50 ms a frame on
     * it, so an encode is not a cost worth measuring; what matters is that nothing here can
     * *wait*, because a stalled worker stops the detector and a stopped detector is a landing
     * that has nothing to look at.
     *
     * **The throw is contained**, exactly as [statusSink]'s is and for a sharper version of the
     * same reason. `RecordedTagSink` deliberately does not swallow a downstream throw — the
     * recogniser's worker does, and logs it — so an uncontained failure here would print once per
     * detected frame, ten times a second, and bury the log an operator reads. A Zenoh problem must
     * not become a vision problem.
     *
     * ## The stamp is the frame's, not the send's
     *
     * D-5, and it costs a subtraction. The sighting's `atNanos` is monotonic, `header.stamp` must
     * be wall-clock, and the two are related only through *now* — so the stamp is now minus the
     * sighting's own age, using the method `TagSighting.Sighting.ageMillisAt` exists to provide.
     * By the time this runs the detection is 40–160 ms old and stamping it "now" would silently
     * destroy every latency measurement anyone could make from the bus.
     */
    fun publishDetection(sighting: com.dimensional.mini4pro.vision.TagSighting.Sighting) {
        if (!detections) return
        val pub = publisher ?: return
        try {
            detectionsSeen++
            val ageMs = sighting.ageMillisAt(SystemClock.elapsedRealtimeNanos())
            val stamp = LcmTime.ofEpochSeconds((System.currentTimeMillis() - ageMs) / 1000.0)
            val emission = ZenohEmission.detection(sighting, stamp, detectionSeq + 1)
            lastDetectionReason = emission.reason
            val bytes = emission.bytes ?: return
            detectionSeq++
            detectionsPublished++
            pub.offer(ZenohChannel.DETECTIONS, bytes)
        } catch (e: Throwable) {
            // Contained here and nowhere else — see the KDoc. Logged at warn rather than counted
            // silently, because the one thing that could reach this is ZenohEmission's own drift
            // check, and that is a bug rather than a condition.
            Log.w(TAG, "zenoh detection publish failed", e)
        }
    }

    /**
     * **One tag fix onto the bus** — the `tag_fix` channel's live half, called on the
     * `tag-detect` worker beside [publishDetection], from the same `RecordedTagSink`
     * downstream and after the same record write, so the line the fix was written from is
     * already on disk before the bus sees it.
     *
     * **Publishes the `TagFix`'s own fields and nothing derived** — the exact values
     * `TagTap.entry` writes to the `tag` line, which is what makes the live message
     * reproducible from the record byte for byte (`tools/memexport`'s cross-check). The
     * containment, the thread argument and the stamp arithmetic are [publishDetection]'s,
     * unchanged: the stamp is the frame's arrival, not the send.
     *
     * Gated on the same [Config.detections] switch as the camera-frame channel, deliberately
     * — two views of one detector, one operator decision.
     */
    fun publishTagFix(
        sighting: com.dimensional.mini4pro.vision.TagSighting.Sighting,
        fix: com.dimensional.mini4pro.vision.TagFix?,
    ) {
        if (!detections) return
        val pub = publisher ?: return
        try {
            tagFixesSeen++
            val ageMs = sighting.ageMillisAt(SystemClock.elapsedRealtimeNanos())
            val stamp = LcmTime.ofEpochSeconds((System.currentTimeMillis() - ageMs) / 1000.0)
            val emission = ZenohEmission.tagFix(
                tagId = sighting.tagId,
                decisionMargin = sighting.decisionMargin,
                northM = fix?.northM,
                eastM = fix?.eastM,
                fixMetric = fix?.metric == true,
                rangeSource = fix?.rangeSource?.name?.lowercase(),
                pitchReported = fix?.pitchReported ?: false,
                stamp = stamp,
                seq = tagFixSeq + 1,
            )
            lastTagFixReason = emission.reason
            val bytes = emission.bytes ?: return
            tagFixSeq++
            tagFixesPublished++
            pub.offer(ZenohChannel.TAG_FIX, bytes)
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh tag_fix publish failed", e)
        }
    }

    /**
     * **One virtual-stick send onto the bus** — the `setpoint` channel's live half, called
     * from `Bridge`'s recorder tap immediately after `Recorder.stickCmd` writes the line the
     * message must be reproducible from.
     *
     * Runs on the guided engine's own send path, which is why the body is [publishDetection]'s
     * shape exactly: a volatile read, a pure encode of ~90 bytes, a bounded queue offer, and a
     * containment — a Zenoh problem must never become a *control* problem, and this thread is
     * the one flying the aircraft. No switch: the channel is on whenever the bus is, per the
     * catalogue (`ZenohChannel.SETPOINT` argues why there is nothing to weigh).
     *
     * The stamp is "now": the tap is synchronous inside the send call, so unlike a detection
     * there is no pipeline age to subtract — the send *is* this instant, to well under the
     * wire's millisecond quantisation.
     */
    fun publishSetpoint(
        setpoint: com.dimensional.mini4pro.record.Setpoint?,
        accepted: Boolean?,
    ) {
        val pub = publisher ?: return
        try {
            setpointsSeen++
            val stamp = LcmTime.ofEpochSeconds(System.currentTimeMillis() / 1000.0)
            val emission = ZenohEmission.setpoint(
                frame = setpoint?.frame,
                northMps = setpoint?.north,
                eastMps = setpoint?.east,
                downMps = setpoint?.down,
                yawRateDegPerS = setpoint?.yawRateDegPerS,
                accepted = accepted,
                stamp = stamp,
            )
            lastSetpointReason = emission.reason
            val bytes = emission.bytes ?: return
            setpointsPublished++
            pub.offer(ZenohChannel.SETPOINT, bytes)
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh setpoint publish failed", e)
        }
    }

    /**
     * **One wind delivery onto the bus** — the `wind` channel's live half, called through
     * [com.dimensional.mini4pro.record.Recorder.windSink] from the recorder's `KeyWindSpeed`
     * tap, **after the `windSpeedDmS` line is written and only when one was** — the record
     * first, the bus second, `RecordedTagSink`'s ordering, and the recorder's on-change dedup
     * is the single owner of "this delivery is a new fact". One line, one message: what
     * `tools/memexport` derives from the record is what a live subscriber saw.
     *
     * Runs on DJI's key-callback thread (the main thread), which is why the body is
     * [publishSetpoint]'s shape exactly: a volatile read, a pure encode of twelve bytes, a
     * bounded queue offer, and a containment — a Zenoh problem must never reach a DJI
     * callback. No switch: always on with the bus, per the catalogue ([ZenohChannel.WIND]).
     *
     * No stamp anywhere: `std_msgs.Float32` carries none ([LcmFloat32] argues why that is
     * honest for an event-driven delivery).
     */
    fun publishWind(speedDmS: Int?) {
        val pub = publisher ?: return
        try {
            windSeen++
            val emission = ZenohEmission.wind(speedDmS)
            lastWindReason = emission.reason
            val bytes = emission.bytes ?: return
            windPublished++
            pub.offer(ZenohChannel.WIND, bytes)
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh wind publish failed", e)
        }
    }

    /**
     * **One decided DJI warning onto the bus** — the `warnings` channel, called from
     * `warn/WarningBus`'s bus sink for **every** event, announced or not.
     *
     * Body shape is [publishWind]'s exactly: a volatile read, a pure encode, a bounded queue
     * offer, and a containment — a Zenoh problem must never reach a DJI callback thread, which is
     * where the health manager and the wind key both deliver. No switch: always on with the bus,
     * per the catalogue ([ZenohChannel.WARNINGS]), because a handful of messages a flight is
     * nothing to weigh and the one you drop is the one that mattered.
     *
     * The stamp is **now**, unlike [publishDetection]'s: a warning has no upstream frame whose
     * arrival it should be dated to — the delivery *is* the event.
     */
    fun publishWarning(event: com.dimensional.mini4pro.warn.WarnEvent) {
        val pub = publisher ?: return
        try {
            warningsSeen++
            val stamp = LcmTime.ofEpochSeconds(System.currentTimeMillis() / 1000.0)
            val emission = ZenohEmission.warning(event, stamp, warningsSeen)
            val bytes = emission.bytes ?: return
            warningsPublished++
            pub.offer(ZenohChannel.WARNINGS, bytes)
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh warning publish failed", e)
        }
    }

    /** How many warnings reached [publishWarning], and how many became messages. Diagnostics only. */
    @Volatile
    var warningsSeen: Int = 0
        private set

    @Volatile
    var warningsPublished: Int = 0
        private set

    /**
     * **Puts a sentence on `status` saying this stream is a recording.**
     *
     * Called by the replay controller when Zenoh-out is switched on, and again when it goes off.
     * It is the only thing on the bus a subscriber can read to learn what it is looking at, and
     * that is deliberate: neither `Odometry` nor `Detection3D` has an honest field for a
     * provenance flag, and inventing one would be a schema change every consumer has to be
     * taught. A sentence on the channel built for sentences is discoverable by anything already
     * subscribed. `replay/ReplayPublication.announcement` is the wording, and it is pure so a
     * test asserts it.
     *
     * `NEVER_DROP`, like every other sentence: this one is said once and losing it would leave a
     * subscriber ingesting a flight it believes is happening. Offered directly to the publisher
     * rather than through the `Announcer` fan-out, because the MAVLink half of that fan-out
     * has its own sentence to make about its own sink and the two are switched separately.
     *
     * Silent when the bus is not up — there is no subscriber to tell.
     */
    fun announceReplay(sentence: String) {
        val pub = publisher ?: return
        try {
            pub.offer(ZenohChannel.STATUS, StringCodec.encode(sentence))
        } catch (e: Throwable) {
            Log.w(TAG, "zenoh replay announcement failed", e)
        }
    }

    private val statusSink = Announcer.Sink { _, text ->
        try {
            publisher?.offer(ZenohChannel.STATUS, StringCodec.encode(text))
        } catch (e: Throwable) {
            // Contained here and nowhere else. `Announcer` deliberately does not contain, so a
            // throw would reach the guided engine's 10 Hz thread. See the class doc.
            Log.w(TAG, "zenoh status publish failed", e)
        }
    }

    private fun onPhase(phase: ZenohPublisher.Phase, why: String) {
        Log.i(TAG, "zenoh phase → $phase ($why)")
        recordEvent(
            EventCode.ZENOH_PHASE,
            "$phase — $why",
            // A publisher that cannot connect while the operator asked for one is somebody
            // staring at a screen with nothing on it — the same severity, and the same argument,
            // as a video failure.
            if (phase == ZenohPublisher.Phase.PUBLISHING) LogEntry.SEV_INFO else LogEntry.SEV_WARN,
        )
    }

    /** Contained like every recorder tap: an evidence problem must never become a bus problem. */
    private fun recordEvent(code: String, message: String, severity: String) {
        try {
            Recorder.event(code, message, severity)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder zenoh event failed", e)
        }
    }
}
