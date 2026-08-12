package com.dimensional.mini4pro.record

import android.content.Context
import android.os.Build
import android.os.SystemClock
import android.util.Log
import com.dimensional.mini4pro.Msdk
import com.dimensional.mini4pro.StateCache
import com.dimensional.mini4pro.warn.WarnEvent
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.video.RawFrameSink
import dji.sdk.keyvalue.key.AirLinkKey
import dji.sdk.keyvalue.key.DJIKeyInfo
import dji.sdk.keyvalue.key.FlightAssistantKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.flightrecord.FlightLogManager
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.PerceptionInfo
import dji.v5.manager.aircraft.perception.listener.PerceptionInformationListener
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import io.dronefleet.mavlink.MavlinkMessage
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong

/**
 * The **thin adapter** between the flight recorder and the two things that cannot
 * be unit-tested: Android and the DJI SDK.
 *
 * Everything in `record/` except this file and `LogSink`'s `java.io` use imports
 * neither `android.*` nor `dji.*`, which is what keeps the schema, the queue, the
 * drop accounting, the rotation policy, the on-change logic and the GCS mirror all
 * testable on the JVM. This file is deliberately dull: it supplies clocks, opens a
 * directory, subscribes to keys and listeners, and forwards immutable snapshots.
 *
 * ## What it subscribes to on its own
 *
 * `StateCache` is not modified — the recorder samples it. Beyond that, it owns
 * subscriptions to the things a post-mortem needs and `AircraftState` does not
 * carry, each chosen because it answers a specific question about why the aircraft
 * did not do what we asked:
 *
 * | source | question it answers |
 * |---|---|
 * | `RemoteControllerKey.KeyStick*` | were the human's sticks fighting us? |
 * | `IVirtualStickManager` state listener | was virtual stick engaged, and who held authority? |
 * | `FlightControlAuthorityChangeReason` | *why* did authority move — geofence, RC switch, low battery? |
 * | `KeyIsNearHeightLimit` / `KeyIsNearDistanceLimit` / `KeyOutOfDistanceLimit` / `KeyIsInLimitHeightArea` | were we inside the ~30 m geofence margin where virtual stick stops working? |
 * | `PerceptionManager` per-direction "working" flags | was obstacle avoidance braking? |
 * | `KeyIsActivelyAvoidingObstacle` / `KeyIsAscentLimitedByObstacle` / `KeyIsNearObstacle` | ditto, from the key side |
 * | `KeyMotorStopReason` / `KeyMotorStartFailureError` / `KeyTakeoffFailureError` | why did it refuse? |
 * | `KeyGoHomeState` | did RTH take over? |
 * | `KeyFlightModeString` | what the aircraft calls its own mode, for auditing our mapping |
 * | `AirLinkKey.KeySignalQuality` | was the link degrading before it failed? |
 * | `IFlightLogManager` paths | where DJI's *independent* record of the same flight is |
 *
 * ## The M3 hook is connected (Stage A, 2026-07-26)
 *
 * `stickCmd` and the mirror's commanded-velocity fields now have their producer:
 * `guided/GuidedStickEngine` records every virtual-stick send through `Bridge`'s
 * contained `GuidedRecord` adapter, in exactly the shape `docs/flight-recording.md`
 * §8 specified — modes off the sent object, `source` only for passthrough sends,
 * the envelope as `StickRange` once per engagement. A session with no engagement
 * still contains no `stick_cmd` entries, and `--diagnose-axis` says so rather than
 * guess.
 */
object Recorder : Tap {

    private const val TAG = "Recorder"

    /** Subdirectory under the app's external files dir. `adb pull`-able. */
    const val DIR_NAME = "flightlogs"

    data class Config(
        /**
         * How often `AircraftState` is sampled. 5 Hz matches `Bridge`'s emitter
         * rate, so every `dji_state` entry has a `mav_out` beside it. 25 Hz is for
         * a control-tuning session, where the question is what happened between two
         * telemetry frames; see the byte-rate table in `docs/flight-recording.md`.
         */
        val stateHz: Double = 5.0,
        val recorder: RecorderConfig = RecorderConfig(),
        val mirror: GcsMirror.MirrorConfig = GcsMirror.MirrorConfig(),
        /** Free-text note stored in the header — the site, the test, the intent. */
        val note: String? = null,
        /**
         * **Record the encoded video beside the flight log.** Off by default, and that default is
         * the honest one rather than the timid one.
         *
         * At `docs/video.md`'s assumed ~5 Mbit/s this writes **625 kB/s**, which is twenty times
         * everything else the recorder produces put together: a ten-minute flight is 375 MB against
         * a 256 MB budget for the whole JSONL, and pulling an hour of it back through the adb tunnel
         * would take about half an hour (`docs/apriltag-landing-recording.md` §2.4–2.5). A default
         * that quietly filled the phone would be discovered on the flight it mattered.
         *
         * Turned on for tag work, and windowed by [videoBudgetBytes] when it is.
         */
        val video: Boolean = false,
        /**
         * The **whole session's** video budget. 256 MB is one JSONL budget's worth, which at the
         * assumed rate is about seven minutes — long enough for an approach and its run-in, short
         * enough to pull back in four minutes.
         *
         * Reaching it is announced (`video_budget_spent`) rather than silent, because a video
         * record that simply stops looks exactly like an aircraft that stopped sending.
         */
        val videoBudgetBytes: Long = 2L * 1024 * 1024 * 1024,
        /** Bytes per sidecar part. Parts exist so pruning can drop coarse chunks of the head. */
        val videoPartBytes: Long = 64L * 1024 * 1024,
        /**
         * Parts kept when pruning — **enough to hold the whole budget**, so nothing is thrown away
         * inside it.
         *
         * This deliberately differs from the JSONL's rule, and the difference is what the two
         * records are *for*. The flight log is evidence: it may run unbounded, so it keeps the tail
         * and drops the head, because a fault is diagnosed from the seconds before it. A video
         * recording is a deliberate act with a purpose — a tag approach, a survey — and losing its
         * beginning to keep its end is exactly backwards.
         *
         * The numbers come from the first measurement of this stream (2026-07-27): **649 kB/s**, so
         * 2 GB is about 51 minutes, longer than any battery this airframe has. The phone had 394 GB
         * free when that was chosen, so the binding limit is deliberately the operator's attention
         * rather than the disk.
         */
        val videoKeepParts: Int = 32,
    )

    @Volatile private var recorder: FlightRecorder? = null

    /**
     * The video sidecar for this session, or null when [Config.video] is off — which is the usual
     * case. Exposed through [videoSink] so `VideoStreamer` can hang it off the stream it already
     * subscribes to, rather than this class learning anything about DJI's camera.
     */
    @Volatile private var video: VideoSidecar? = null

    /**
     * The sink `VideoStreamer` hangs off the camera stream, or null when video recording is off.
     *
     * Read fresh rather than held, exactly as `Bridge` reads `guidedStick`: the recorder starts and
     * stops independently of the video loop, and a streamer holding a stale sidecar would write
     * frames into a session that had ended.
     */
    /**
     * The sink `VideoStreamer` hangs off the camera stream, or null when no session is open.
     *
     * **Handed over whether or not recording is on**, because the sidecar has to watch the stream
     * to know when the next keyframe arrives — the only moment a file may start. It costs one
     * boolean per frame when idle; see [VideoSidecar.recording].
     */
    fun videoSink(): RawFrameSink? = video

    /**
     * **The operator's switch for recording video**, read fresh on every frame.
     *
     * Separate from [Config.video], which decides whether a session *has* a sidecar at all, because
     * the two answer different questions and change on different schedules. The config is a
     * property of the session; this is a hand on a switch during it, and turning it off mid-flight
     * has to take effect on the next frame rather than the next session.
     *
     * Costs nothing when off: [VideoSidecar] opens no file until its first write, so a session that
     * never records writes no bytes and leaves no empty part behind.
     *
     * Deliberately **not** persisted across launches. 625 kB/s is not something to inherit from a
     * previous session's mood; an operator who wants video says so each time, which is the same
     * reasoning `CommandInterlock` uses for a switch with much larger consequences.
     */
    val videoEnabled: Boolean get() = video?.recording == true

    /** Returns what it is now, so a caller can put the switch back if it refuses. */
    @Synchronized
    fun setVideoEnabled(on: Boolean): Boolean {
        if (video == null || on == videoEnabled) return videoEnabled
        video?.recording = on
        val counters = video?.counters()
        Log.i(TAG, "video recording ${if (on) "ON" else "OFF"} (sidecar ${if (video == null) "absent" else "ready"})")
        recorder?.record(
            LogEntry.Event(
                monoNanos = SystemClock.elapsedRealtimeNanos(),
                code = EventCode.VIDEO_RECORDING,
                severity = LogEntry.SEV_INFO,
                message = if (on) {
                    "video recording ON by the operator"
                } else {
                    "video recording OFF by the operator after ${counters?.frames ?: 0} frames"
                },
            )
        )
        return videoEnabled
    }

    /** What the sidecar has written this session, for the status screen. Null when it is off. */
    fun videoCounters(): VideoSidecar.Counters? = video?.counters()
    @Volatile private var config: Config = Config()
    @Volatile private var sampler: ScheduledExecutorService? = null
    private val delta = StateDelta()

    /** Latest values, for the mirror and for the sampled `rc_stick` entry. */
    @Volatile private var rcLeftH: Int? = null
    @Volatile private var rcLeftV: Int? = null
    @Volatile private var rcRightH: Int? = null
    @Volatile private var rcRightV: Int? = null
    @Volatile private var vsEnabled: Boolean? = null
    @Volatile private var vsAdvanced: Boolean? = null
    @Volatile private var vsAuthority: String? = null
    @Volatile private var lastSetpoint: Setpoint? = null
    @Volatile private var lastAxes: StickAxes? = null
    @Volatile private var lastModes: StickModes? = null
    @Volatile private var lastState: AircraftState? = null
    private val stickSequence = AtomicLong()

    /** Last emitted rc_stick, so a stationary transmitter is not logged 25×/s. */
    private var lastRcKey: String? = null
    private var lastRcNanos = 0L

    /** Last emitted gimbal attitude, so a motionless camera is not logged at the sample rate. */
    private var lastGimbal: GimbalSample? = null
    private var lastGimbalNanos = 0L

    /** The DJI landing-recenter detector — see [GimbalRecenter]. Fed by [sampleGimbal] only. */
    private val gimbalRecenter = GimbalRecenter()

    /**
     * Where the camera is pointing, or null when nothing can say.
     *
     * **Installed by `Bridge` with the link, and cleared with it**, exactly as `flightActions` and
     * `gimbalAim` are — the reading comes from `MsdkGimbalAim`, which lives as long as the link
     * does. A supplier rather than a push, and read from [sample] rather than from `Bridge.tick`,
     * for two reasons that are both about the trap this entry exists for:
     *
     *  - The deadband and the idle heartbeat belong on the recorder's own clock, beside
     *    [sampleRcSticks] which solves the identical problem for the RC transmitter. A gimbal
     *    recorded from the telemetry tick would sample at the *link's* rate, so an operator
     *    running the recorder at 25 Hz for a control-tuning session would still get 5 Hz of
     *    camera.
     *  - It keeps recording when the ground station is not being talked to.
     *
     * A `record/`-owned type so this package still imports nothing from `gimbal/` — that seam
     * belongs to the gimbal, and `ZenohTelemetryEncoder.gimbalAttitudeOrNull` takes loose angles
     * for the same reason.
     */
    @Volatile
    var gimbalSource: (() -> GimbalSample?)? = null

    /**
     * The aircraft-outbound half of [Tap]. Constructed once and re-pointed at each session's
     * recorder through the same `recorder ?: return` idiom every producer here uses, so an ask
     * made with no session running is dropped rather than queued for the next one.
     */
    private val calls = DjiCalls(
        sink = { entry -> recorder?.record(entry) },
        nowNanos = { SystemClock.elapsedRealtimeNanos() },
    )

    /** On-change filter for the keys this file owns (not the ones in `AircraftState`). */
    private val keyLast = HashMap<String, String?>()
    private val keyLock = Any()

    private val listenHolder = Any()
    private var vsListener: VirtualStickStateListener? = null
    private var perceptionListener: PerceptionInformationListener? = null

    val isRunning: Boolean get() = recorder?.isRunning == true
    val dropCount: Long get() = recorder?.dropCount ?: 0
    val writtenCount: Long get() = recorder?.writtenCount ?: 0
    val byteCount: Long get() = recorder?.byteCount ?: 0
    val currentFile: String? get() = recorder?.currentSinkName

    /**
     * Where sessions are written. Surfaced so the status screen can show it.
     *
     * The app's external files dir first, because that is `adb pull`-able without
     * root and survives an uninstall prompt; `filesDir` as a fallback for the case
     * where external storage is unavailable, since a log in an awkward place beats
     * no log.
     */
    fun directory(context: Context): File =
        File(context.getExternalFilesDir(null) ?: context.filesDir, DIR_NAME)

    // ─────────────────────────────────────────────────────────────────────────
    // lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Opens a session and starts sampling. Safe to call when the MSDK is not yet
     * registered: the DJI subscriptions are attempted and their failure is recorded
     * as an event rather than thrown, so a log always exists even when the aircraft
     * never came up. That matters — "the recorder produced nothing" and "the
     * aircraft never connected" must be distinguishable afterwards.
     */
    @Synchronized
    fun start(context: Context, cfg: Config = Config()) {
        if (recorder != null) return
        config = cfg
        val dir = directory(context)
        val factory = FileSinkFactory(dir)
        val session = sessionId(System.currentTimeMillis())
        val rec = FlightRecorder(
            session = session,
            sinks = factory,
            mono = MonotonicClock { SystemClock.elapsedRealtimeNanos() },
            wall = WallClock { System.currentTimeMillis() },
            config = cfg.recorder,
            headerJson = headerMeta(context, cfg),
        )
        recorder = rec
        // Built for every session, because it costs nothing until something is written to it and
        // the operator's switch can be flipped at any moment. `Config.video` seeds the switch
        // rather than gating the object.
        video = run {
            VideoSidecar(
                session = session,
                sinks = FileVideoSinkFactory(dir),
                budgetBytes = cfg.videoBudgetBytes,
                partBytes = cfg.videoPartBytes,
                keepParts = cfg.videoKeepParts,
                // Straight onto the same bounded queue every other entry takes, so a frame index
                // can never block MSDK's decode thread — `Tap`'s second non-negotiable.
                emit = { rec.record(it) },
                nowNanos = { SystemClock.elapsedRealtimeNanos() },
                log = { Log.i(TAG, "video: $it") },
            ).also { it.recording = cfg.video }
        }
        delta.reset()
        lastGimbal = null
        lastGimbalNanos = 0L
        lastRcKey = null
        lastRcNanos = 0L
        synchronized(keyLock) { keyLast.clear() }
        rec.start()
        factory.pruneSessions(cfg.recorder.maxSessions)

        rec.record(
            LogEntry.Event(
                rec.now(), EventCode.RECORDER_START,
                message = "session $session in ${dir.absolutePath}",
            )
        )

        subscribeDji()

        val periodMs = (1000.0 / cfg.stateHz).toLong().coerceAtLeast(1)
        sampler = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "flight-recorder-sample").apply { isDaemon = true }
        }.also {
            it.scheduleAtFixedRate(::sample, 0, periodMs, TimeUnit.MILLISECONDS)
        }
        Log.i(TAG, "recording to ${dir.absolutePath}/$session at ${cfg.stateHz} Hz")
    }

    @Synchronized
    fun stop() {
        sampler?.shutdownNow()
        sampler = null
        unsubscribeDji()
        djiAttached = false
        // The sidecar closes **before** the recorder, so its closing line and any last frame index
        // still have somewhere to go. The other way round loses the one summary that says how much
        // video this session actually holds.
        video?.close()
        video = null
        recorder?.stop()
        recorder = null
        Log.i(TAG, "recording stopped")
    }

    /**
     * `20260725-161200` — wall-clock, so files sort chronologically in `adb shell
     * ls` and in the phone's file manager, which is how they will be found.
     */
    fun sessionId(unixMs: Long): String =
        java.text.SimpleDateFormat("yyyyMMdd-HHmmss", java.util.Locale.US)
            .format(java.util.Date(unixMs))

    // ─────────────────────────────────────────────────────────────────────────
    // producers — all safe from any thread, all non-blocking
    // ─────────────────────────────────────────────────────────────────────────

    // ── the Tap seam ─────────────────────────────────────────────────────────
    //
    // The three verbs a transport is constructed with. Each is total: it contains its own
    // throws, because the contract in `Tap`'s KDoc says containment is the implementation's job
    // and not the transport's. A transport obliged to wrap every tap call in a `try` will
    // eventually forget one, and the forgotten one will be in the send path.

    /** [Tap.gcsOut] — one datagram that actually left the socket. */
    override fun gcsOut(datagram: ByteArray) {
        try {
            mavOutWire(datagram)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder tap failed on send", e)
        }
    }

    /** [Tap.gcsIn] — one MAVLink message that arrived, with its genuine wire bytes. */
    override fun gcsIn(message: MavlinkMessage<*>) {
        try {
            mavIn(message)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder tap failed on receive", e)
        }
    }

    /**
     * [Tap.aircraftOut] — one discrete ask to the aircraft, and the handle its answer rides.
     *
     * [DjiCalls] contains its own throws, including a throwing sink, so this needs no `try` of its
     * own and could not usefully have one: the point of the return value is that the caller always
     * gets a handle.
     */
    override fun aircraftOut(op: String, argsJson: String?, urgent: Boolean): Tap.Call =
        calls.begin(op, argsJson, urgent)

    /**
     * [Tap.tagSeen] — one tag the on-board detector saw.
     *
     * Contained like every other verb here, and for a sharper reason than most: this runs on the
     * detector's worker thread, which also holds the only reference to the native detector. An
     * uncontained throw here would kill that thread and the detector would stop with no message
     * anywhere, which is the failure mode this project's whole recording seam exists to prevent.
     */
    override fun tagSeen(
        sighting: com.dimensional.mini4pro.vision.TagSighting.Sighting,
        fix: com.dimensional.mini4pro.vision.TagFix?,
        latched: Boolean,
    ) {
        try {
            // The mapping itself is `TagTap.entry`, a pure function next door, for the reason
            // `MavlinkTap.inbound` is one: this file needs Android and therefore cannot be
            // unit-tested, and a rule that lives only here is a rule no test can hold. Measured:
            // mutating the timestamp to the write time while the mapping was inline killed **zero**
            // tests, and moving it out is what closed that.
            recorder?.record(TagTap.entry(sighting, fix, latched))
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder tap failed on a tag sighting", e)
        }
    }

    /** Outstanding asks with no answer yet. Surfaced for tests and the status screen. */
    val outstandingCalls: Int get() = calls.outstandingCount

    // ── producers ────────────────────────────────────────────────────────────

    /** One inbound MAVLink message, with its genuine wire bytes. */
    fun mavIn(message: MavlinkMessage<*>) {
        val rec = recorder ?: return
        rec.record(MavlinkTap.inbound(rec.now(), message))
    }

    /**
     * One outbound MAVLink message. Pass [wireBytes] if the socket is ever tapped;
     * without it the hex is re-serialised and labelled as such — see
     * [MavlinkTap.outbound].
     */
    fun mavOut(payload: Any, systemId: Int = 1, componentId: Int = 1, wireBytes: ByteArray? = null) {
        val rec = recorder ?: return
        rec.record(MavlinkTap.outbound(rec.now(), payload, systemId, componentId, wireBytes = wireBytes))
    }

    /**
     * One outbound MAVLink message, from the **actual datagram** that left the
     * socket. Install as `MavlinkLink.onSent` once the `DatagramStreams` tap in
     * `docs/flight-recording.md` exists; it replaces [mavOut] and makes outbound
     * sequence numbers real rather than ours.
     */
    fun mavOutWire(datagram: ByteArray) {
        val rec = recorder ?: return
        MavlinkTap.outboundWire(rec.now(), datagram)?.let { rec.record(it) }
    }

    /**
     * One virtual-stick command. **The M3 hook.**
     *
     * Call this from `GuidedController` immediately after handing the numbers to the
     * SDK, with all four axes and all three control modes, and with the setpoint
     * that produced them. Recording the modes on every command rather than once at
     * configuration time is what makes a control-mode fault distinguishable from an
     * axis fault — the two look identical in the numbers alone.
     */
    fun stickCmd(
        setpoint: Setpoint?,
        axes: StickAxes,
        modes: StickModes,
        source: CommandSource? = null,
        range: StickRange? = null,
        path: String = StickPath.ADVANCED_PARAM,
        accepted: Boolean? = null,
        error: String? = null,
    ) {
        lastSetpoint = setpoint
        lastAxes = axes
        lastModes = modes
        val rec = recorder ?: return
        rec.record(
            LogEntry.StickCmd(
                monoNanos = rec.now(),
                sequence = stickSequence.incrementAndGet(),
                setpoint = setpoint,
                axes = axes,
                modes = modes,
                source = source,
                range = range,
                path = path,
                accepted = accepted,
                error = error,
            )
        )
    }

    fun event(code: String, message: String? = null, severity: String = LogEntry.SEV_INFO) {
        val rec = recorder ?: return
        rec.record(LogEntry.Event(rec.now(), code, severity, message))
    }

    /**
     * One DJI warning changing state, from any source. **The record sink `warn/WarningBus` calls.**
     *
     * Takes the already-decided [WarnEvent] rather than loose fields, so the record, the
     * `STATUSTEXT`, the phone screen and the Zenoh message cannot come apart: all four are rendered
     * from the one object `warn/WarningMonitor` produced, and there is no second place where a
     * level could be mapped to a severity differently.
     *
     * Change-driven, never per-delivery — see [LogEntry.Warn].
     */
    fun warn(event: WarnEvent) {
        val rec = recorder ?: return
        val w = event.warning
        rec.record(
            LogEntry.Warn(
                monoNanos = rec.now(),
                source = w.source.label,
                change = event.change.name.lowercase(),
                code = w.code,
                state = w.state,
                level = w.level.name,
                previousLevel = event.previousLevel?.name,
                title = w.title?.takeIf { it.isNotBlank() },
                description = w.description?.takeIf { it.isNotBlank() },
                componentId = w.componentId,
                sensorIndex = w.sensorIndex,
                measurement = w.measurement,
                severity = event.recordSeverity,
                forwarded = event.announce,
                rateLimited = event.rateLimited,
                text = event.text,
            )
        )
    }

    /**
     * The messages to mirror to the GCS this cycle, or an empty list. Call from the
     * emitter loop at [GcsMirror.MirrorConfig.hz] and send each one.
     *
     * Kept as a pull rather than a push so the recorder never touches the link and
     * `Bridge` stays the only thing that owns sending.
     */
    fun mirrorMessages(timeBootMs: Long): List<Any> {
        val s = lastState
        val sample = GcsMirror.Sample(
            timeBootMs = timeBootMs,
            commandedNorth = lastSetpoint?.north,
            commandedEast = lastSetpoint?.east,
            commandedDown = lastSetpoint?.down,
            commandedYawRate = lastSetpoint?.yawRateDegPerS,
            achievedNorth = s?.velocityNorth,
            achievedEast = s?.velocityEast,
            achievedDown = s?.velocityDown,
            axes = lastAxes,
            modes = lastModes,
            vsEnabled = vsEnabled,
            vsAdvanced = vsAdvanced,
            authority = vsAuthority,
            rcLeftHorizontal = rcLeftH,
            rcLeftVertical = rcLeftV,
            rcRightHorizontal = rcRightH,
            rcRightVertical = rcRightV,
        )
        return GcsMirror.cycle(sample, config.mirror)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // sampling
    // ─────────────────────────────────────────────────────────────────────────

    private fun sample() {
        val rec = recorder ?: return
        try {
            val state = if (Msdk.state.value.registered) StateCache.aircraftState() else AircraftState()
            lastState = state
            val t = rec.now()
            for (e in delta.entriesFor(t, state)) rec.record(e)
            sampleRcSticks(rec, t)
            sampleGimbal(rec, t)
            // The video stream's liveness, from the one thing watching it. A stream that stops
            // delivering stops calling the sidecar, so the sidecar cannot notice by itself — this
            // is the tick that lets it.
            video?.checkStall(SystemClock.elapsedRealtimeNanos())
            // Turns a swallowed DJI callback into a line instead of an absence. Cheap: it walks a
            // map that is empty except in the milliseconds an action is in flight.
            calls.sweep()
        } catch (e: Throwable) {
            // Same rule as Bridge's tick: a bad value must not kill the loop, because
            // a dead sampler looks exactly like a disconnected aircraft.
            Log.w(TAG, "recorder sample failed", e)
        }
    }

    /**
     * Emits one `rc_stick` per sample while the sticks are moving, and once a second
     * while they are still.
     *
     * Recording four separate key callbacks would produce four entries per twitch
     * and none at all while the transmitter is untouched — and "the sticks were
     * centred" is exactly the fact that exonerates the pilot, so it has to be on the
     * record rather than inferred from an absence.
     */
    private fun sampleRcSticks(rec: FlightRecorder, t: Long) {
        val key = "$rcLeftH,$rcLeftV,$rcRightH,$rcRightV"
        val moved = key != lastRcKey
        val stale = t - lastRcNanos >= IDLE_RC_INTERVAL_NS
        if (!moved && !stale) return
        lastRcKey = key
        lastRcNanos = t
        rec.record(LogEntry.RcStick(t, rcLeftH, rcLeftV, rcRightH, rcRightV))
    }

    /**
     * Emits one `gimbal` entry when the camera has moved past [GIMBAL_DEADBAND_DEG], and once a
     * second while it has not — **closing the one Zenoh channel a flight record could not
     * reproduce** (`replay/ReplayCoverage.GIMBAL`).
     *
     * ## The deadband, and why 0.5°
     *
     * A deadband is a deliberate loss of resolution, so it is set against what a *replay of the
     * camera's behaviour* would turn on rather than against what the sensor can resolve. The Mini
     * 4 Pro's 24 mm-equivalent lens covers roughly 44° vertically, so 0.5° is about **1 % of frame
     * height** — a framing error nobody could see in the picture, and an order finer than the
     * gimbal solution an orbit commands (`guided/OrbitGimbal` aims in whole-degree steps at ~2 Hz).
     *
     * Applied to the largest move on any axis, not to pitch alone. Pitch is the only axis this
     * airframe means anything by, but roll and yaw are recorded as reported rather than assumed
     * (`LogEntry.Gimbal`), and a deadband that ignored them would be the assumption smuggled back
     * in.
     *
     * ## The heartbeat, and why it is not optional
     *
     * `Gimbal.KeyGimbalAttitude` is change-driven — it goes silent when the camera is still, which
     * during a steady orbit is exactly when the camera *should* be still. Emitting only on change
     * would make "the camera held −30° for the whole circle" and "the attitude feed died as the
     * circle began" produce identical records: nothing. So a still camera is stated once a second,
     * the same interval and the same argument as [sampleRcSticks] uses for a transmitter nobody is
     * touching, and the `age` on every line separates a live repeat from a cached one.
     *
     * ## Cost
     *
     * ~60 bytes a line. At the 5 Hz ceiling that is ~300 B/s against a file that already runs at
     * ~22 kB/s — **1.4 %**, and about a fifth of that in practice, since the camera is motionless
     * for most of any flight and the heartbeat is 1 Hz. Not urgent, so it adds no `fsync`.
     */
    private fun sampleGimbal(rec: FlightRecorder, t: Long) {
        val sample = gimbalSource?.invoke() ?: return
        // The DJI landing recenter, named as an event rather than left as a pitch-sample shape
        // (`landingdata.md` §5). Fed every sample — before the deadband, because the slew is
        // exactly the kind of movement the deadband passes anyway and the detector's own window
        // does its de-duplication. Decision-free: nothing reads the verdict but this line.
        gimbalRecenter.sample(t / 1_000_000, sample.pitchDeg, lastState?.flightMode)?.let {
            rec.record(LogEntry.Event(t, EventCode.GIMBAL_RECENTERED, LogEntry.SEV_WARN, it))
        }
        val previous = lastGimbal
        val moved = previous == null || sample.movedFrom(previous, GIMBAL_DEADBAND_DEG)
        val stale = t - lastGimbalNanos >= IDLE_GIMBAL_INTERVAL_NS
        if (!moved && !stale) return
        lastGimbal = sample
        lastGimbalNanos = t
        rec.record(
            LogEntry.Gimbal(
                monoNanos = t,
                pitchDeg = sample.pitchDeg,
                rollDeg = sample.rollDeg,
                yawDeg = sample.yawDeg,
                ageMs = sample.ageMs,
            )
        )
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DJI subscriptions
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Attaches the DJI-side sources. Idempotent.
     *
     * [start] calls this, but registration usually completes *after* the recorder is
     * started — and subscribing to a `KeyManager` key before registration silently
     * does nothing, with no error and no callback (`docs/msdk-keys.md` §A). So the
     * lead must also call this from the `Msdk.state` observer once `registered` turns
     * true. Until then the session records our MAVLink boundary and nothing else, and
     * says so in an event.
     */
    @Synchronized
    fun attachDjiSources() {
        if (recorder == null || djiAttached) return
        subscribeDji()
    }

    @Volatile private var djiAttached = false

    private fun subscribeDji() {
        if (!Msdk.state.value.registered) {
            event(
                EventCode.SDK_ERROR,
                "MSDK not registered yet — DJI sources are not attached. Call " +
                    "Recorder.attachDjiSources() once Msdk.state.registered turns true.",
                LogEntry.SEV_WARN,
            )
            return
        }
        djiAttached = true
        // Say so on the record. Without this the log's only statement about DJI
        // sources is the warning above, which is written on every session that
        // starts before registration — i.e. all of them — and is never retracted.
        // A reader then sees "not attached" and no contradiction, and concludes the
        // DJI half of the session is missing when it is present.
        event(EventCode.SDK_ATTACHED, "DJI sources attached")

        // RC sticks. Held in fields and emitted by the sampler — see sampleRcSticks.
        listenRaw(RemoteControllerKey.KeyStickLeftHorizontal) { rcLeftH = it }
        listenRaw(RemoteControllerKey.KeyStickLeftVertical) { rcLeftV = it }
        listenRaw(RemoteControllerKey.KeyStickRightHorizontal) { rcRightH = it }
        listenRaw(RemoteControllerKey.KeyStickRightVertical) { rcRightV = it }

        // Identity, once. Product type settles late (UNRECOGNIZED first), so it is
        // listened to rather than read: see docs/measurements/2026-07-25-ground-probe.md.
        listenField(ProductKey.KeyProductType, "productType")
        listenField(RemoteControllerKey.KeyRemoteControllerType, "rcType")
        listenField(ProductKey.KeyFirmwareVersion, "firmwareVersion")

        // What the aircraft calls its own mode, so our FlightMode mapping can be
        // audited against reality rather than against the docs.
        listenField(FlightControllerKey.KeyFlightModeString, "flightModeString")

        // Refusals and takeovers — case 7.
        listenField(FlightControllerKey.KeyGoHomeState, "goHomeState")
        listenField(FlightControllerKey.KeyMotorStopReason, "motorStopReason")
        listenField(FlightControllerKey.KeyMotorStartFailureError, "motorStartFailureError")
        listenField(FlightControllerKey.KeyTakeoffFailureError, "takeoffFailureError")

        // Geofence proximity. djidoc: IVirtualStickManager — "when the aircraft is
        // close (about 30 meters) to the restricted flight zone or restricted
        // distance, the remote controller will get the control of the aircraft, and
        // the virtual stick cannot be used". These four are the only warning.
        listenField(FlightControllerKey.KeyIsNearHeightLimit, "isNearHeightLimit")
        listenField(FlightControllerKey.KeyIsNearDistanceLimit, "isNearDistanceLimit")
        listenField(FlightControllerKey.KeyOutOfDistanceLimit, "outOfDistanceLimit")
        listenField(FlightControllerKey.KeyIsInLimitHeightArea, "isInLimitHeightArea")
        listenField(FlightControllerKey.KeyFlightLimitHeight, "flightLimitHeight")
        listenField(FlightControllerKey.KeyDistanceLimit, "distanceLimit")
        listenField(FlightControllerKey.KeyDistanceLimitEnabled, "distanceLimitEnabled")

        // Obstacle avoidance, from the key side.
        listenField(FlightAssistantKey.KeyObstacleAvoidanceEnabled, "obstacleAvoidanceEnabled")
        listenField(FlightAssistantKey.KeyIsActivelyAvoidingObstacle, "isActivelyAvoidingObstacle")
        listenField(FlightAssistantKey.KeyIsAscentLimitedByObstacle, "isAscentLimitedByObstacle")
        listenField(FlightAssistantKey.KeyIsNearObstacle, "isNearObstacle")

        // Environment and link. Wind is landing14's lesson made structural: 9.1 m/s of it
        // explained a "flyaway" that a compass theory survived a day of, because the value
        // sat here in dm/s and nothing downstream could see it. The speed rides its own tap
        // (windDelivery) so the Zenoh bus publishes the same on-change stream this record
        // keeps; direction and warning stay record-only.
        listenRaw(FlightControllerKey.KeyWindSpeed) { v -> windDelivery(v) }
        // The warning level rides its own tap for the same reason the speed does, and since
        // landing17 it has a consumer: `warn/WindWarnings` turns it into the one warning
        // vocabulary and `warn/WarningBus` puts it in front of the pilot. Recorded first, exactly
        // as before, so the `dji_field windWarning` line a reader has been able to grep since
        // landing14 is unchanged and the announcement is reproducible from it.
        listenRaw(FlightControllerKey.KeyWindWarning) { v -> windWarningDelivery(v?.toString()) }
        // The 8-way compass enum (WINDLESS, NORTH … NORTH_WEST, UNKNOWN) — verified against
        // the 5.18.0 jar by javap 2026-07-30: `DJIKeyInfo<WindDirection> KeyWindDirection` on
        // `DJIFlightControllerKey`, a real Java enum, so `toString()` below is the NAME and
        // never the wire int (`value()` exists on the class and is deliberately not read: a
        // "3" in the record would need this comment to be legible, "EAST" needs nothing).
        // Record-only: a Float channel cannot carry an 8-way enum honestly, and synthesizing
        // a vector from speed+compass would fabricate precision — see `ZenohChannel.WIND`.
        listenField(FlightControllerKey.KeyWindDirection, "windDirection")
        listenField(FlightControllerKey.KeyUltrasonicHeight, "ultrasonicHeight")
        listenField(FlightControllerKey.KeyIsHomeLocationSet, "isHomeLocationSet")
        listenField(AirLinkKey.KeySignalQuality, "airLinkSignalQuality")

        subscribeVirtualStick()
        subscribePerception()
        recordDjiLogPaths()
    }

    /**
     * The virtual-stick state listener. Engagement, advanced mode, authority owner,
     * and DJI's own reason for an authority change.
     *
     * `setVirtualStickStateListener` appends to a list inside `VirtualStickManager`
     * (javap: `virtualStickStateListeners`), so registering here does not displace
     * the one M3 will add.
     */
    private fun subscribeVirtualStick() {
        try {
            val listener = object : VirtualStickStateListener {
                override fun onVirtualStickStateUpdate(stickState: VirtualStickState) {
                    val rec = recorder ?: return
                    vsEnabled = stickState.isVirtualStickEnable
                    vsAdvanced = stickState.isVirtualStickAdvancedModeEnabled
                    vsAuthority = stickState.currentFlightControlAuthorityOwner?.name
                    rec.record(
                        LogEntry.VsState(
                            rec.now(),
                            enabled = vsEnabled,
                            advanced = vsAdvanced,
                            authority = vsAuthority,
                        )
                    )
                }

                override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
                    val rec = recorder ?: return
                    rec.record(
                        LogEntry.VsState(
                            rec.now(),
                            enabled = vsEnabled,
                            advanced = vsAdvanced,
                            authority = vsAuthority,
                            changeReason = reason.name,
                        )
                    )
                    rec.record(
                        LogEntry.Event(
                            rec.now(), EventCode.VS_AUTHORITY_CHANGE,
                            severity = if (reason.name == "MSDK_REQUEST") LogEntry.SEV_INFO else LogEntry.SEV_WARN,
                            message = "flight-control authority changed: ${reason.name}",
                        )
                    )
                }
            }
            vsListener = listener
            VirtualStickManager.getInstance().setVirtualStickStateListener(listener)
        } catch (e: Throwable) {
            event(EventCode.SDK_ERROR, "virtual stick state listener failed: ${e.message}", LogEntry.SEV_WARN)
        }
    }

    /**
     * Per-direction obstacle-avoidance state.
     *
     * `PerceptionManager` is an industry-oriented API and its per-direction
     * "working" flags may simply be absent on a Mini 4 Pro — the jar carries a
     * `Mini3ProPerceptionDelegate` and a `ConsumePerceptionDelegate`, so consumer
     * airframes take a different path. Failure is recorded as an event rather than
     * swallowed, so a log that lacks obstacle data says *why* it lacks it. The
     * `KeyIsActivelyAvoidingObstacle` family above is the independent fallback.
     */
    private fun subscribePerception() {
        try {
            val listener = PerceptionInformationListener { info: PerceptionInfo ->
                field("oaWorkingForward", info.forwardObstacleAvoidanceWorking?.toString())
                field("oaWorkingBackward", info.backwardObstacleAvoidanceWorking?.toString())
                field("oaWorkingLeft", info.leftSideObstacleAvoidanceWorking?.toString())
                field("oaWorkingRight", info.rightSideObstacleAvoidanceWorking?.toString())
                field("oaWorkingUp", info.upwardObstacleAvoidanceWorking?.toString())
                field("oaWorkingDown", info.downwardObstacleAvoidanceWorking?.toString())
                field("oaBrakingDistanceH", info.horizontalObstacleAvoidanceBrakingDistance.toString())
            }
            perceptionListener = listener
            PerceptionManager.getInstance().addPerceptionInformationListener(listener)
        } catch (e: Throwable) {
            event(
                EventCode.SDK_ERROR,
                "perception listener unavailable (${e.javaClass.simpleName}) — obstacle-avoidance " +
                    "state for this session comes only from KeyIsActivelyAvoidingObstacle",
                LogEntry.SEV_WARN,
            )
        }
    }

    /**
     * Records where DJI's own flight records live, so they can be pulled with adb
     * afterwards.
     *
     * We deliberately do not parse them: they are DJI's format, they change, and
     * their value here is precisely that they are **not derived from our code**. A
     * third independent record is the tiebreaker when our log and QGC's tlog
     * disagree. `docs/flight-recording.md` has the `adb pull` commands.
     */
    private fun recordDjiLogPaths() {
        try {
            val mgr = FlightLogManager.getInstance()
            field("djiFlightRecordPath", mgr.flightRecordPath)
            field("djiFlyClogPath", mgr.flyClogPath)
        } catch (e: Throwable) {
            event(EventCode.SDK_ERROR, "flight log paths unavailable: ${e.message}", LogEntry.SEV_WARN)
        }
    }

    private fun unsubscribeDji() {
        try {
            KeyManager.getInstance().cancelListen(listenHolder)
        } catch (e: Throwable) {
            Log.w(TAG, "cancelListen failed", e)
        }
        try {
            vsListener?.let { VirtualStickManager.getInstance().removeVirtualStickStateListener(it) }
        } catch (e: Throwable) {
            Log.w(TAG, "removeVirtualStickStateListener failed", e)
        }
        try {
            perceptionListener?.let { PerceptionManager.getInstance().removePerceptionInformationListener(it) }
        } catch (e: Throwable) {
            Log.w(TAG, "removePerceptionInformationListener failed", e)
        }
        vsListener = null
        perceptionListener = null
    }

    /** Subscribes and hands the raw value to [sink]. No work in the callback. */
    private fun <T> listenRaw(info: DJIKeyInfo<T>, sink: (T?) -> Unit) {
        try {
            KeyManager.getInstance().listen(KeyTools.createKey(info), listenHolder, true) { _, v -> sink(v) }
        } catch (e: Throwable) {
            event(EventCode.SDK_ERROR, "listen failed: ${e.message}", LogEntry.SEV_WARN)
        }
    }

    /** Subscribes and records the value as a [LogEntry.Field] on change. */
    private fun <T> listenField(info: DJIKeyInfo<T>, name: String) {
        listenRaw(info) { v -> field(name, v?.toString()) }
    }

    /**
     * Where a wind-speed delivery goes after the record has it, or null when no bus wants one.
     *
     * Installed by `ZenohBus.start` and cleared by `ZenohBus.stop` — the sink must go away
     * with the session, `Announcer.attach`'s lifecycle and `ZenohBus.gimbalSource`'s shape.
     * It lives here, not as a second `KeyWindSpeed` listener in `zenoh/`, because the
     * delivery-dedup below ([field]'s on-change rule) must have exactly one owner: the bus
     * publishes precisely the lines this record writes, which is what lets `tools/memexport`
     * reproduce the live stream from the record alone. The value handed over is the raw dm/s
     * integer the line was written from — the dm/s→m/s conversion belongs to
     * `ZenohTelemetryEncoder.windOrNull`, its single owner, on the far side.
     */
    @Volatile
    var windSink: ((Int?) -> Unit)? = null

    /**
     * One `KeyWindSpeed` delivery: the record line first, the bus second, and the bus **only
     * when a line was written** — no session or no change means no message, because a wind
     * message must be reproducible from a `windSpeedDmS` line that exists. Contained like
     * every recorder tap: an evidence consumer's problem must never reach DJI's callback
     * thread (the sink's own containment is `publishWind`'s, but *"it cannot throw today"*
     * is exactly the sentence that precedes the commit where it can).
     */
    private fun windDelivery(speedDmS: Int?) {
        // Latched whether or not a line was written, because it is the *measurement* a wind
        // warning is announced with and the warning may arrive on any tick. `null` stays null:
        // unknown is never zero, and a sentence without a number is honest where "0.0 m/s" is not.
        lastWindSpeedDmS = speedDmS
        if (!field("windSpeedDmS", speedDmS?.toString())) return
        val sink = windSink ?: return
        try {
            sink(speedDmS)
        } catch (e: Throwable) {
            Log.w(TAG, "wind sink failed", e)
        }
    }

    /**
     * The newest `KeyWindSpeed` reading in dm/s, or null before the first one.
     *
     * Not a second owner of the wind speed — the record's `windSpeedDmS` line and the `wind` Zenoh
     * channel are still fed by [windDelivery] from the same delivery. This is the *latch* the
     * warning path reads when it needs the number that goes with the level, and it exists because
     * the two keys arrive independently: DJI raised `LEVEL_2` at t=111.078 on landing17 and the
     * speed had last changed a second earlier. Reading "the newest speed" is the only honest
     * answer available at the instant the warning changes.
     */
    @Volatile
    private var lastWindSpeedDmS: Int? = null

    /**
     * **The sink `warn/WindWarnings` is fed through** — one `KeyWindWarning` delivery as DJI's own
     * enum *name*, paired with the newest wind speed, or null when nothing is wired.
     *
     * Same shape and same rule as [windSink]: the record line is written first and the sink runs
     * **only when a line was written**, so an announcement is always reproducible from a
     * `windWarning` field line that exists, and a repeated identical delivery reaches nothing.
     * That on-change dedup is the recorder's own single owner of "this delivery is a new fact";
     * the bus's diff is the second brake behind it, not a substitute for it.
     */
    @Volatile
    var windWarningSink: ((stateName: String?, speedDmS: Int?) -> Unit)? = null

    /** One `KeyWindWarning` delivery: the record line first, the warning path second. */
    private fun windWarningDelivery(stateName: String?) {
        if (!field("windWarning", stateName)) return
        val sink = windWarningSink ?: return
        try {
            sink(stateName, lastWindSpeedDmS)
        } catch (e: Throwable) {
            Log.w(TAG, "wind warning sink failed", e)
        }
    }

    /**
     * Records one named value, but only when it changed; true exactly when a line was
     * written. Called from KeyManager callbacks — which arrive on the main thread — so it
     * must stay allocation-light and lock-brief. See `docs/msdk-keys.md` §A on the callback
     * thread.
     */
    private fun field(name: String, value: String?): Boolean {
        val rec = recorder ?: return false
        var previous: String? = null
        val changed = synchronized(keyLock) {
            if (keyLast.containsKey(name) && keyLast[name] == value) {
                false
            } else {
                previous = keyLast[name]
                keyLast[name] = value
                true
            }
        }
        if (changed) rec.record(LogEntry.Field(rec.now(), name, value, previous))
        return changed
    }

    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Identity for the header: app, device, and the app-side view of the aircraft.
     * Aircraft identity from the SDK arrives later, as `dji_field` entries, because
     * `KeyProductType` fires `UNRECOGNIZED` before it settles.
     */
    private fun headerMeta(context: Context, cfg: Config): String = JsonObject.render { o ->
        o.obj("app") { w ->
            val pkg = context.packageName
            w.put("package", pkg)
            try {
                @Suppress("DEPRECATION")
                val pi = context.packageManager.getPackageInfo(pkg, 0)
                w.put("version_name", pi.versionName)
                @Suppress("DEPRECATION")
                w.put("version_code", pi.versionCode)
            } catch (e: Throwable) {
                w.put("version_name", "unknown")
            }
        }
        o.obj("device") { w ->
            w.put("manufacturer", Build.MANUFACTURER)
            w.put("model", Build.MODEL)
            w.put("android_sdk", Build.VERSION.SDK_INT)
            w.put("android_release", Build.VERSION.RELEASE)
            w.put("abi", Build.SUPPORTED_ABIS.firstOrNull())
        }
        o.obj("recorder") { w ->
            w.put("state_hz", cfg.stateHz, 3)
            w.put("mirror_enabled", cfg.mirror.enabled)
            w.put("mirror_hz", cfg.mirror.hz, 3)
            w.put("mirror_only_when_engaged", cfg.mirror.onlyWhenEngaged)
        }
        o.put("note", cfg.note)
        o.put("msdk_registered", Msdk.state.value.registered)
    }

    /** While the sticks are still, one entry a second is enough to prove it. */
    private const val IDLE_RC_INTERVAL_NS = 1_000_000_000L

    /** Same interval and the same argument, for a camera that is not moving. */
    private const val IDLE_GIMBAL_INTERVAL_NS = 1_000_000_000L

    /** Degrees. See [sampleGimbal] for why 0.5° and why it is applied to all three axes. */
    const val GIMBAL_DEADBAND_DEG = 0.5
}
