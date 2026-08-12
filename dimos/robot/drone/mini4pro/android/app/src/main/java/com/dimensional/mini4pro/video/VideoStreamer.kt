package com.dimensional.mini4pro.video

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import com.dimensional.mini4pro.Msdk
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.datacenter.livestream.LiveStreamSettings
import dji.v5.manager.datacenter.livestream.LiveStreamStatus
import dji.v5.manager.datacenter.livestream.LiveStreamStatusListener
import dji.v5.manager.datacenter.livestream.LiveStreamType
import dji.v5.manager.datacenter.livestream.LiveVideoBitrateMode
import dji.v5.manager.datacenter.livestream.StreamQuality
import dji.v5.manager.datacenter.livestream.settings.RtspSettings
import dji.v5.manager.interfaces.ICameraStreamManager
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.atomic.AtomicLong

/**
 * The video path, plus enough instrumentation to tell the silent failures apart.
 *
 * ## Two ways out of the phone
 *
 * **RTP passthrough ([VideoMode.RTP], preferred).** The aircraft already sends
 * H.264. `ICameraStreamManager.addReceiveStreamListener` hands those bytes over
 * untouched, [RtpVideoSink] wraps them in RTP and pushes them to the GCS over
 * UDP. Nothing is decoded, scaled or re-encoded, and the packets travel *out* of
 * the phone — the direction that works on this network (`wifi-fix.md`).
 *
 * **MSDK's RTSP server ([VideoMode.RTSP]).** `ILiveStreamManager` with
 * `LiveStreamType.RTSP`. Convenient — QGC takes a URL — but it re-encodes:
 * `setLiveStreamQuality` pins the resolution, `setLiveVideoBitrate` the rate and
 * `setLiveStreamScaleType` scales, which is only possible after a decode. It also
 * has to be dialled *into*, so on this network a GCS on the laptop cannot reach
 * it without a tunnel. See docs/video.md.
 *
 * ## What the caller does
 *
 * ```kotlin
 * VideoStreamer.start()                 // idempotent; safe to call before registration
 * VideoStreamer.state                   // StateFlow<VideoStatus>, 1 Hz
 * VideoStreamer.state.value.gcsHint     // what to configure in the GCS
 * VideoStreamer.stop()
 * ```
 *
 * [start] never blocks and never needs the SDK to be ready — it records the wish
 * and a 1 Hz control loop brings the stream up the moment `Msdk.state.registered`,
 * the aircraft and the camera are all present. That ordering is the whole point:
 * touching the MSDK earlier silently does nothing (docs/architecture.md).
 *
 * ## Rules this file follows
 *
 *  - **DJI types stop here.** [VideoStatus], [VideoLoop], [RtpH264],
 *    [RtpVideoSink], [RtspUrl], [StreamSource], [VideoGate] and [LocalAddress]
 *    are DJI-free and unit-tested; this class is the thin adapter.
 *  - **No work in SDK callbacks.** Every callback stores primitives (or posts to
 *    the main looper) and returns. All MSDK calls happen on the main looper.
 *  - **Positive counters over booleans.** `running == true` was the failure mode
 *    that wasted time here; `tapFrames == 0` after 10 s is a diagnosis.
 */
object VideoStreamer {

    private const val TAG = "Video"

    /** Control-loop period. MSDK's own `LiveStreamStatus` arrives about this often. */
    private const val TICK_MS = 1_000L

    /**
     * The Mini 4 Pro's only camera. `LEFT_OR_MAIN` is the index the key sweep read
     * every camera key through (docs/measurements/2026-07-25-key-sweep.md).
     */
    private val CAMERA_INDEX = ComponentIndexType.LEFT_OR_MAIN

    /** Where the encoded video is sent, and by whom. */
    enum class VideoMode {
        /** Our own RTP/UDP push of the aircraft's H.264. No re-encode. */
        RTP,

        /** MSDK's built-in RTSP server. Re-encodes; must be dialled into. */
        RTSP,

        /** Both at once, for a side-by-side latency measurement. */
        BOTH,
    }

    /** Where [RtpVideoSink] pushes packets. */
    data class RtpTarget(
        /** GCS address. On this network, a wired relay host is safer than the laptop. */
        val host: String,
        /** QGroundControl's default `udpUrl` is `0.0.0.0:5600`. */
        val port: Int = RtpVideoSink.DEFAULT_PORT,
        val mtu: Int = RtpH264.DEFAULT_MTU,
        val payloadType: Int = RtpH264.PAYLOAD_TYPE,
    )

    /**
     * @param frameTap subscribe to the raw stream. Always on for [VideoMode.RTP] —
     *   it *is* the video path. In [VideoMode.RTSP] it is the diagnostic that
     *   distinguishes "wrong lens / no frames" from "no client connected".
     * @param advertiseHost overrides the host in the advertised RTSP URL. The
     *   server always binds on the phone; this only changes the address handed to
     *   a GCS, which is what you need when the GCS reaches the phone through a
     *   tunnel rather than directly (`wifi-fix.md`).
     * @param advertisePort likewise for the port.
     * @param rawSink an extra sink alongside the RTP one. Null today.
     */
    data class Config(
        val mode: VideoMode = VideoMode.RTP,
        val rtp: RtpTarget? = null,
        val user: String = RtspUrl.DEFAULT_USER,
        val password: String = RtspUrl.DEFAULT_PASSWORD,
        val port: Int = RtspUrl.DEFAULT_PORT,
        val quality: VideoQuality = VideoQuality.HD,
        /** null = MSDK AUTO bitrate. Ignored in [VideoMode.RTP]. */
        val bitrateBps: Int? = null,
        val frameTap: Boolean = true,
        val advertiseHost: String? = null,
        val advertisePort: Int? = null,
        val rawSink: RawFrameSink? = null,
    ) {
        val wantsRtsp: Boolean get() = mode == VideoMode.RTSP || mode == VideoMode.BOTH
        val wantsRtp: Boolean get() = mode == VideoMode.RTP || mode == VideoMode.BOTH
    }

    private val _state = MutableStateFlow(VideoStatus())
    val state: StateFlow<VideoStatus> = _state.asStateFlow()

    /** Convenience for the GCS-facing UI. Null until the server is configured. */
    val rtspUrl: String? get() = _state.value.rtspUrl

    /**
     * Pins the RTP socket to a particular network. Set once, at wiring time,
     * *before* [start]; `Bridge` supplies the same `android.net.Network` the
     * MAVLink socket is bound to.
     *
     * Deliberately a property rather than a field on [Config]. [Config] is
     * compared for equality to decide whether a [start] call is a reconfiguration,
     * and a lambda compares by identity — a binder in there would make every
     * `start()` look like a config change and tear the stream down each time.
     *
     * Null means an unbound socket, which is correct only on a JVM. See
     * [RtpVideoSink.udp] for what it costs on this phone.
     */
    @Volatile
    var socketBinder: ((java.net.DatagramSocket) -> Unit)? = null

    /**
     * Where phase changes and failures go besides logcat — the flight recorder, in
     * practice.
     *
     * Video is the one subsystem whose complete failure is indistinguishable from
     * "the operator did not turn it on", so a session that produced nothing has to
     * say which of those it was in the file, not only in a logcat ring buffer that
     * has usually wrapped by the time anyone asks. Typed as primitives so this
     * package still imports nothing from `record/`; see [VideoEvents] for the codes.
     */
    @Volatile
    var eventSink: ((code: String, message: String, error: Boolean) -> Unit)? = null

    private val main by lazy { Handler(Looper.getMainLooper()) }

    // ---- control-loop state; all touched on the main looper only ---------------

    private var wanted: Config? = null
    private var ticking = false
    private var serving = false
    private var startInFlight = false
    private var attemptStartedAtMs = 0L
    private var servingSinceMs = 0L
    private var nextAttemptAtMs = 0L
    private var attempts = 0L

    /**
     * Failures since the last healthy stretch — the index into `VideoLoop.retryDelayMs`'s ladder.
     *
     * Reset in the tick once the stream has served with frames still arriving for
     * `VideoLoop.HEALTHY_RESET_MS`, not merely for being `serving`: a stalled stream is serving
     * too, and resetting on that would make the ladder unreachable exactly when it is needed.
     */
    private var consecutiveFailures = 0

    private var phase = VideoPhase.STOPPED
    private var phaseSinceMs = 0L

    private var url: String? = null
    private var urlRedacted: String? = null
    private var localNic: LocalAddress.Nic? = null

    private var sourceRange: List<String> = emptyList()
    private var sourceCurrent: String? = null
    private var sourceWarning: String? = null
    private var cameraKeyConnected: Boolean? = null

    private var zeroBitrateSamples = 0L
    private var lastError: String? = null
    private var tap: CameraStreamTap? = null

    /**
     * The live RTP sink, swappable underneath a running tap — see [liveSink].
     * Volatile because it is written on the main looper and read on MSDK's
     * callback thread.
     */
    @Volatile private var rtpSink: RtpVideoSink? = null

    @Volatile private var extraSink: RawFrameSink? = null

    /**
     * Totals from sinks that have been closed, so [rtpCounters] stays monotonic
     * across a socket rebuild. See [RtpVideoSink.Counters.plus].
     */
    private var rtpCarry: RtpVideoSink.Counters? = null

    /** Remembered across [stop] so the final counters still name their destination. */
    private var lastRtpTarget: String? = null

    /**
     * The one sink the tap ever sees, for the whole life of a session.
     *
     * An indirection rather than the concrete sink, because [rebindTransport] has
     * to be able to replace the socket underneath a *running* subscription. The
     * alternative — rebuilding the tap — means an
     * `addReceiveStreamListener`/`removeReceiveStreamListener` cycle against DJI,
     * which throws `DJICheckException` if the pair is ever mis-ordered
     * (docs/video.md) and drops frames while it happens. This costs one nullable
     * read per frame instead.
     */
    private val liveSink = RawFrameSink { data, offset, length, info ->
        rtpSink?.onEncodedFrame(data, offset, length, info)
        extraSink?.onEncodedFrame(data, offset, length, info)
    }

    // ---- written from SDK callbacks; read on the loop --------------------------

    @Volatile private var availableCameras: List<String> = emptyList()
    @Volatile private var lastLiveStatus: LiveStreamStatus? = null
    @Volatile private var streamingReported = false
    private val statusUpdates = AtomicLong()

    private var liveListenerRegistered = false
    private var cameraListenerRegistered = false

    /**
     * Stores and returns. No work here — see docs/architecture.md.
     *
     * Note `addLiveStreamStatusListener` is a *setter*, not an adder:
     * `LiveStreamManager.addLiveStreamStatusListener` calls
     * `MRTCManager.setLiveStreamStatusListener` and `remove…` calls
     * `MRTCManager.clearCallback()` ignoring its argument (`javap -c`). There is
     * exactly one such listener per process, and it is this one.
     */
    private val liveStatusListener = object : LiveStreamStatusListener {
        override fun onLiveStreamStatusUpdate(status: LiveStreamStatus?) {
            lastLiveStatus = status
            statusUpdates.incrementAndGet()
        }

        override fun onError(error: IDJIError?) {
            val text = error?.let { "liveStream: ${it.description()} (${it.errorCode()})" }
            main.post { lastError = text; publish() }
        }
    }

    private val availableCameraListener = object : ICameraStreamManager.AvailableCameraUpdatedListener {
        override fun onAvailableCameraUpdated(list: MutableList<ComponentIndexType>) {
            availableCameras = list.map { it.name }
        }

        override fun onCameraStreamEnableUpdate(map: MutableMap<ComponentIndexType, Boolean>) {
            // Industry airframes only: `CameraStreamManager.enableStream` is a no-op
            // unless `ProductUtil.isIndustryMachine()` (javap -c). Nothing to do here
            // on a Mini 4 Pro.
        }
    }

    private val tickRunnable = object : Runnable {
        override fun run() {
            if (!ticking) return
            try {
                tick()
            } catch (t: Throwable) {
                // A throw here would kill the loop and the stream would just stop
                // updating — the exact silence this class exists to prevent.
                Log.e(TAG, "control loop threw", t)
                lastError = "control loop: ${t.javaClass.simpleName}: ${t.message}"
                publish()
            }
            main.postDelayed(this, TICK_MS)
        }
    }

    // ---- public API -----------------------------------------------------------

    /**
     * Ask for video. Safe to call at any time, from any thread, repeatedly.
     * Calling it before MSDK registration is the expected case.
     */
    @JvmOverloads
    fun start(config: Config = Config()) {
        main.post {
            val restart = wanted != null && wanted != config
            wanted = config
            if (restart) {
                Log.i(TAG, "config changed, tearing down to re-apply")
                tearDown("config changed")
                // Rebuild the session: the old tap holds the old sink and the old
                // counters, and reusing it makes a changed config silently partial.
                resetSession(config)
            }
            if (!ticking) {
                if (!restart) resetSession(config)
                ticking = true
                Log.i(
                    TAG,
                    "video requested: mode=${config.mode} quality=${config.quality} " +
                        "port=${config.port} rtp=${config.rtp?.let { "${it.host}:${it.port}" } ?: "-"}",
                )
                main.post(tickRunnable)
            } else {
                publish()
            }
        }
    }

    /** Stop the video path and the control loop. Idempotent. */
    fun stop() {
        main.post {
            if (wanted == null && !ticking) return@post
            wanted = null
            ticking = false
            main.removeCallbacks(tickRunnable)
            tearDown("stop requested")
            // Keep the session tally readable after the stream is down: "we sent
            // 41 000 packets and QGC showed nothing" is a different bug report
            // from "we sent none", and the screen should still be able to tell
            // them apart once an operator has pressed stop.
            rtpCarry = rtpCounters()
            rtpSink?.close()
            rtpSink = null
            removeListeners()
            setPhase(VideoPhase.STOPPED)
            publish()
            Log.i(TAG, "video stopped")
        }
    }

    // ---- control loop ---------------------------------------------------------

    private fun tick() {
        val cfg = wanted
        val msdk = Msdk.state.value
        val now = SystemClock.elapsedRealtime()

        // Registration gates every SDK call, including the ones that read inputs.
        if (cfg != null && msdk.registered) {
            ensureAvailableCameraListener()
            if (msdk.productConnected) cameraKeyConnected = readCameraConnected()
            if (serving && cfg.wantsRtsp) pollStreamHealth()
        }

        val cameraAvailable = CAMERA_INDEX.name in availableCameras || cameraKeyConnected == true
        val tapNow = tap?.counters()
        // **Forgive the failure history once the feed has been genuinely healthy**, so the backoff
        // ladder is not a one-way ratchet across a long flight. "Healthy" is frames still arriving,
        // not merely `serving` — a stalled stream is serving too, which is the whole problem.
        val frameFresh = tapNow?.lastFrameAtMs?.let { now - it < VideoLoop.STALL_AFTER_MS } == true
        if (serving && frameFresh && now - servingSinceMs > VideoLoop.HEALTHY_RESET_MS) {
            consecutiveFailures = 0
        }
        val decision = VideoLoop.decide(
            VideoLoop.Input(
                wanted = cfg != null,
                registered = msdk.registered,
                aircraftConnected = msdk.productConnected,
                cameraAvailable = cameraAvailable,
                serving = serving,
                startInFlight = startInFlight,
                nowMs = now,
                attemptStartedAtMs = attemptStartedAtMs,
                nextAttemptAtMs = nextAttemptAtMs,
                servingSinceMs = servingSinceMs,
                // Only MSDK's RTSP server has an isStreaming() to disagree with us.
                streamingReported = if (cfg?.wantsRtsp == true) streamingReported else true,
                framesSeen = tapNow?.frames ?: 0L,
                lastFrameAtMs = tapNow?.lastFrameAtMs ?: 0L,
                consecutiveFailures = consecutiveFailures,
            ),
        )

        decision.tearDown?.let { tearDown(it) }

        when (decision) {
            VideoLoop.Stop -> setPhase(VideoPhase.STOPPED)
            is VideoLoop.Wait -> setPhase(decision.gate.toPhase())
            VideoLoop.AwaitStart -> setPhase(VideoPhase.STARTING)
            VideoLoop.AwaitRetry -> Unit
            VideoLoop.Poll -> setPhase(VideoPhase.SERVING)
            is VideoLoop.Fail -> fail(decision.message)
            VideoLoop.BringUp -> cfg?.let { bringUp(it) }
        }

        publishAndLog()
    }

    private fun bringUp(cfg: Config) {
        attempts++
        attemptStartedAtMs = SystemClock.elapsedRealtime()
        setPhase(VideoPhase.STARTING)

        // 0. The passthrough IS the RTP socket. Without one there is nothing to
        //    bring up, and the old code would have gone all the way to "RTP
        //    passthrough up" with no socket behind it — SERVING, tapFrames
        //    climbing, and not one packet on the wire, which is the single most
        //    misleading state this class can reach.
        if (cfg.wantsRtp && rtpSink == null) {
            fail(
                lastError
                    ?: "RTP mode with no target configured — VideoStreamer.Config.rtp is null",
            )
            return
        }

        // 1. Where a GCS should dial, when MSDK is serving. MSDK never tells us this.
        if (cfg.wantsRtsp && !prepareRtspUrl(cfg)) return

        // 2. The single-lens trap. Ask the airframe, then pin DEFAULT_CAMERA.
        applyStreamSource()

        // 3. The tap. In RTP mode this *is* the video path, so it comes first and a
        //    failure here is fatal rather than cosmetic.
        if (cfg.frameTap || cfg.wantsRtp) {
            val ok = runCatching { tap?.start() }
                .onFailure { Log.e(TAG, "frame tap failed to register", it) }
                .isSuccess
            if (!ok && cfg.wantsRtp) {
                fail("could not subscribe to the camera stream; RTP has no source")
                return
            }
        }

        if (!cfg.wantsRtsp) {
            // Passthrough only: there is no asynchronous server to wait for. The
            // tap is subscribed, so packets flow as soon as the aircraft sends.
            startInFlight = false
            serving = true
            servingSinceMs = SystemClock.elapsedRealtime()
            lastError = null
            setPhase(VideoPhase.SERVING)
            Log.i(TAG, "RTP passthrough up. GCS: ${_state.value.gcsHint ?: describeRtp(cfg)}")
            return
        }

        startRtspServer(cfg)
    }

    /** Builds and validates the URL a GCS will dial. False means we already failed. */
    private fun prepareRtspUrl(cfg: Config): Boolean {
        val nic = LocalAddress.current()
        localNic = nic
        val host = cfg.advertiseHost ?: nic?.address
        val port = cfg.advertisePort ?: cfg.port
        val problems = RtspUrl.problems(host, port, cfg.user, cfg.password)
        if (problems.isNotEmpty()) {
            fail(problems.joinToString("; "))
            return false
        }
        url = RtspUrl.build(host!!, port, cfg.user, cfg.password)
        urlRedacted = RtspUrl.redact(url!!)
        if (cfg.advertiseHost != null) {
            Log.i(TAG, "advertising $urlRedacted; server binds ${nic?.address ?: "?"}:${cfg.port}")
        }
        return true
    }

    private fun startRtspServer(cfg: Config) {
        startInFlight = true
        val mgr = MediaDataCenter.getInstance().liveStreamManager

        // A leftover session from a previous run would ignore our new settings.
        if (runCatching { mgr.isStreaming }.getOrDefault(false)) {
            Log.w(TAG, "live stream already running; stopping it before reconfiguring")
            mgr.stopStream(completion("stopStream(pre-existing)") {})
            startInFlight = false
            nextAttemptAtMs = SystemClock.elapsedRealtime() + 2_000
            return
        }

        // Status listener before starting, or the first updates are lost.
        ensureLiveStreamListener()

        // Configure, in the order StreamConfigPlan fixes. That order is
        // load-bearing: `setLiveStreamQuality` silently forces bitrate mode back to
        // MANUAL (javap -c LiveStreamManager), so quality must be applied first.
        runCatching {
            for (step in StreamConfigPlan.of(cfg.quality, cfg.bitrateBps).steps) when (step) {
                StreamConfigPlan.Step.CAMERA_INDEX -> mgr.cameraIndex = CAMERA_INDEX
                StreamConfigPlan.Step.SETTINGS -> mgr.liveStreamSettings = LiveStreamSettings.Builder()
                    .setLiveStreamType(LiveStreamType.RTSP)
                    .setRtspSettings(
                        RtspSettings.Builder()
                            .setUserName(cfg.user)
                            .setPassWord(cfg.password)
                            .setPort(cfg.port)
                            .build()
                    )
                    .build()
                StreamConfigPlan.Step.QUALITY -> mgr.liveStreamQuality = cfg.quality.toDji()
                StreamConfigPlan.Step.BITRATE_MODE_AUTO ->
                    mgr.liveVideoBitrateMode = LiveVideoBitrateMode.AUTO
                StreamConfigPlan.Step.BITRATE_MODE_MANUAL ->
                    mgr.liveVideoBitrateMode = LiveVideoBitrateMode.MANUAL
                StreamConfigPlan.Step.BITRATE_VALUE ->
                    mgr.liveVideoBitrate = cfg.bitrateBps ?: return@runCatching
            }
        }.onFailure {
            fail("configuring live stream: ${it.javaClass.simpleName}: ${it.message}")
            return
        }

        Log.i(TAG, "startStream attempt #$attempts → $urlRedacted")
        mgr.startStream(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                main.post {
                    startInFlight = false
                    serving = true
                    servingSinceMs = SystemClock.elapsedRealtime()
                    lastError = null
                    setPhase(VideoPhase.SERVING)
                    // Printed in full exactly once, because this is the string the
                    // operator has to paste into QGroundControl. The credentials are
                    // a fixed LAN default, not a secret; the 1 Hz line redacts them.
                    Log.i(TAG, "RTSP server up. GCS URL: $url")
                    publish()
                }
            }

            override fun onFailure(error: IDJIError) {
                val text = "startStream: ${error.description()} (${error.errorCode()}/${error.errorType()})"
                main.post { fail(text) }
            }
        })
    }

    /**
     * Reads the supported stream-source range off the airframe and pins the lens.
     *
     * The Mini 4 Pro reported `[DEFAULT_CAMERA]` in our sweep, so normally this
     * writes nothing at all — a pointless `setValue` on a live camera is itself a
     * way to break the stream.
     */
    private fun applyStreamSource() {
        val km = KeyManager.getInstance()
        val sourceKey = KeyTools.createKey(CameraKey.KeyCameraVideoStreamSource, CAMERA_INDEX)
        val rangeKey = KeyTools.createKey(CameraKey.KeyCameraVideoStreamSourceRange, CAMERA_INDEX)

        sourceRange = runCatching { km.getValue(rangeKey) }.getOrNull()
            ?.mapNotNull { it?.name }.orEmpty()
        sourceCurrent = runCatching { km.getValue(sourceKey) }.getOrNull()?.name

        val plan = StreamSource.plan(sourceRange, sourceCurrent)
        sourceWarning = plan.warning
        plan.warning?.let { Log.w(TAG, "stream source: $it") }
        Log.i(TAG, "stream source: current=${sourceCurrent ?: "?"} range=$sourceRange action=${plan.action}")

        if (plan.action != StreamSource.Action.SWITCH) return
        val target = CameraVideoStreamSourceType.values().firstOrNull { it.name == plan.desired }
        if (target == null) {
            Log.e(TAG, "no CameraVideoStreamSourceType named ${plan.desired}")
            return
        }
        Log.w(TAG, "switching stream source ${sourceCurrent ?: "?"} → ${target.name}")
        km.setValue(sourceKey, target, completion("setValue(KeyCameraVideoStreamSource=${target.name})") {
            sourceCurrent = target.name
        })
    }

    private fun pollStreamHealth() {
        val mgr = MediaDataCenter.getInstance().liveStreamManager
        streamingReported = runCatching { mgr.isStreaming }.getOrDefault(false)

        val vbps = lastLiveStatus?.vbps ?: 0
        zeroBitrateSamples = if (vbps == 0) zeroBitrateSamples + 1 else 0

        // Read the source back so a lens change behind our back is visible.
        sourceCurrent = runCatching {
            KeyManager.getInstance()
                .getValue(KeyTools.createKey(CameraKey.KeyCameraVideoStreamSource, CAMERA_INDEX))
                ?.name
        }.getOrNull() ?: sourceCurrent
    }

    // ---- listener plumbing ----------------------------------------------------

    private fun ensureLiveStreamListener() {
        if (liveListenerRegistered) return
        runCatching {
            MediaDataCenter.getInstance().liveStreamManager
                .addLiveStreamStatusListener(liveStatusListener)
        }.onSuccess {
            liveListenerRegistered = true
            Log.i(TAG, "live stream status listener registered")
        }.onFailure { Log.e(TAG, "addLiveStreamStatusListener failed", it) }
    }

    private fun ensureAvailableCameraListener() {
        if (cameraListenerRegistered) return
        runCatching {
            MediaDataCenter.getInstance().cameraStreamManager
                .addAvailableCameraUpdatedListener(availableCameraListener)
        }.onSuccess {
            cameraListenerRegistered = true
            Log.i(TAG, "available-camera listener registered")
        }.onFailure { Log.e(TAG, "addAvailableCameraUpdatedListener failed", it) }
    }

    private fun removeListeners() {
        if (liveListenerRegistered) {
            runCatching {
                MediaDataCenter.getInstance().liveStreamManager
                    .removeLiveStreamStatusListener(liveStatusListener)
            }
            liveListenerRegistered = false
        }
        if (cameraListenerRegistered) {
            runCatching {
                MediaDataCenter.getInstance().cameraStreamManager
                    .removeAvailableCameraUpdatedListener(availableCameraListener)
            }
            cameraListenerRegistered = false
        }
    }

    // ---- helpers -------------------------------------------------------------

    private fun readCameraConnected(): Boolean? = runCatching {
        KeyManager.getInstance().getValue(KeyTools.createKey(CameraKey.KeyConnection, CAMERA_INDEX))
    }.getOrNull()

    private fun describeRtp(cfg: Config): String =
        cfg.rtp?.let { "UDP h.264 Video Stream, udpUrl 0.0.0.0:${it.port}" } ?: "no RTP target set"

    private fun resetSession(cfg: Config) {
        serving = false
        startInFlight = false
        attempts = 0
        consecutiveFailures = 0
        attemptStartedAtMs = 0
        nextAttemptAtMs = 0
        zeroBitrateSamples = 0
        lastError = null
        url = null
        urlRedacted = null
        localNic = null
        sourceRange = emptyList()
        sourceCurrent = null
        sourceWarning = null
        cameraKeyConnected = null
        availableCameras = emptyList()
        lastLiveStatus = null
        streamingReported = false
        statusUpdates.set(0)

        tap?.stop()
        rtpSink?.close()
        rtpCarry = null
        extraSink = cfg.rawSink
        lastRtpTarget = cfg.rtp?.takeIf { cfg.wantsRtp }?.let { "${it.host}:${it.port}" }
        rtpSink = cfg.rtp?.takeIf { cfg.wantsRtp }?.let { openSink(it) }
        tap = CameraStreamTap(CAMERA_INDEX, liveSink)
        setPhase(VideoPhase.WAITING_REGISTRATION)
    }

    /** A fresh RTP sink on a freshly bound socket, or null with [lastError] set. */
    private fun openSink(target: RtpTarget): RtpVideoSink? = runCatching {
        RtpVideoSink(
            // The binder is what keeps this off the carrier network; see
            // RtpVideoSink.udp and wifi-fix.md gotcha #2.
            transport = RtpVideoSink.udp(target.host, target.port, socketBinder),
            mtu = target.mtu,
            payloadType = target.payloadType,
        )
    }.onFailure { t ->
        lastError = "RTP socket to ${target.host}:${target.port}: " +
            "${t.javaClass.simpleName}: ${t.message}"
        Log.e(TAG, "could not open the RTP socket", t)
        event(VideoEvents.FAILED, lastError!!, error = true)
    }.getOrNull()

    /** Live totals plus anything a previous socket sent. Null when RTP was never on. */
    private fun rtpCounters(): RtpVideoSink.Counters? {
        val live = rtpSink?.counters()
        val carry = rtpCarry
        return when {
            live == null -> carry
            carry == null -> live
            else -> carry + live
        }
    }

    /**
     * Rebuild the RTP socket on whatever [socketBinder] now resolves to, keeping
     * the DJI subscription and the counters intact.
     *
     * Called when the MAVLink link rebinds after a WiFi flap. A `Network` that
     * comes back is a *new* `Network` — netIds are not reused — so a socket bound
     * to the old one will never transmit again, and nothing recovers by waiting
     * (`WifiBindTracker`). Telemetry already handles this; without the same
     * treatment video would go dark for the rest of the session after a two-second
     * AP blip, on a phone nobody can reach, with `rtpErrors` as the only clue.
     *
     * Idempotent and safe when video is off: no wish, no RTP target, nothing to do.
     */
    fun rebindTransport(reason: String) {
        main.post {
            val cfg = wanted ?: return@post
            val target = cfg.rtp?.takeIf { cfg.wantsRtp } ?: return@post
            // Fold the outgoing socket's totals in *before* it is dropped, or
            // rtpPkts would fall back to zero on a working path.
            val carried = rtpCounters()
            val fresh = openSink(target) ?: run { publish(); return@post }
            val old = rtpSink
            rtpCarry = carried
            rtpSink = fresh
            old?.close()
            val message = "RTP socket rebuilt on a new network: $reason"
            Log.w(TAG, message)
            event(VideoEvents.REBIND, message, error = false)
            publish()
        }
    }

    private fun tearDown(reason: String) {
        Log.i(TAG, "tearing down live stream: $reason")
        if (wanted?.wantsRtsp != false) {
            runCatching {
                val mgr = MediaDataCenter.getInstance().liveStreamManager
                if (mgr.isStreaming) mgr.stopStream(completion("stopStream") {})
            }.onFailure { Log.w(TAG, "stopStream threw", it) }
        }
        tap?.stop()
        serving = false
        startInFlight = false
        streamingReported = false
        zeroBitrateSamples = 0
    }

    private fun fail(message: String) {
        Log.e(TAG, "video failed: $message")
        lastError = message
        serving = false
        startInFlight = false
        // The progressive retry: each failure with no healthy stretch between doubles the wait, to
        // a one-minute cap, and a healthy stretch in the tick above forgives the history. See
        // `VideoLoop.retryDelayMs` for why the interval is capped but the attempts are not.
        val delay = VideoLoop.retryDelayMs(consecutiveFailures)
        consecutiveFailures++
        nextAttemptAtMs = SystemClock.elapsedRealtime() + delay
        event(VideoEvents.FAILED, message, error = true)
        setPhase(VideoPhase.FAILED)
        publish()
    }

    private fun setPhase(next: VideoPhase) {
        if (phase == next) return
        val held = if (phaseSinceMs == 0L) 0 else (SystemClock.elapsedRealtime() - phaseSinceMs) / 1000
        Log.i(TAG, "phase ${phase.name} → ${next.name}")
        // Recorded as well as logged. A phase *trail* is what distinguishes "the
        // camera never reported" from "it reported and then went away", and only
        // the transitions carry that — the 1 Hz snapshot does not.
        event(VideoEvents.PHASE, "${phase.name} → ${next.name} after ${held}s", error = false)
        phase = next
        phaseSinceMs = SystemClock.elapsedRealtime()
    }

    /**
     * Hands an event to whoever is recording, and never lets that matter.
     *
     * Same containment rule as every other recorder tap in this project: an
     * evidence problem must not become a video problem, and a throwing sink here
     * would reach the 1 Hz control loop.
     */
    private fun event(code: String, message: String, error: Boolean) {
        val sink = eventSink ?: return
        try {
            sink(code, message, error)
        } catch (t: Throwable) {
            Log.w(TAG, "video event sink threw", t)
        }
    }

    /** Wraps a completion callback so nothing but bookkeeping runs in it. */
    private fun completion(what: String, onOk: () -> Unit) =
        object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                main.post {
                    Log.i(TAG, "$what ok")
                    onOk()
                    publish()
                }
            }

            override fun onFailure(error: IDJIError) {
                val text = "$what: ${error.description()} (${error.errorCode()})"
                main.post {
                    Log.e(TAG, text)
                    lastError = text
                    publish()
                }
            }
        }

    private fun publishAndLog() {
        publish()
        Log.i(TAG, _state.value.oneLine())
    }

    private fun publish() {
        val st = lastLiveStatus
        val cfg = wanted
        _state.value = VideoStatus(
            phase = phase,
            mode = cfg?.mode?.name,
            rtspUrl = url,
            rtspUrlRedacted = urlRedacted,
            // Survives stop() so the final tally still says where it went.
            rtpTarget = cfg?.rtp?.let { "${it.host}:${it.port}" } ?: lastRtpTarget,
            localAddress = localNic?.address,
            localInterface = localNic?.name,
            cameraIndex = CAMERA_INDEX.name,
            streamSource = sourceCurrent,
            streamSourceRange = sourceRange,
            sourceWarning = sourceWarning,
            quality = cfg?.quality,
            requestedBitrateBps = cfg?.bitrateBps,
            liveStreamListenerRegistered = liveListenerRegistered,
            availableCameraListenerRegistered = cameraListenerRegistered,
            frameTapRegistered = tap?.registered == true,
            availableCameras = availableCameras,
            cameraKeyConnected = cameraKeyConnected,
            statusUpdates = statusUpdates.get(),
            streamingReported = streamingReported,
            fps = st?.fps,
            bitrateBps = st?.vbps,
            width = st?.resolution?.width,
            height = st?.resolution?.height,
            packetLossPercent = st?.packetLoss,
            rttMs = st?.rtt,
            packetCacheLen = st?.packetCacheLen,
            zeroBitrateSamples = zeroBitrateSamples,
            tap = tap?.counters() ?: TapCounters(),
            rtp = rtpCounters(),
            startAttempts = attempts,
            lastError = lastError,
            phaseHeldSeconds = if (phaseSinceMs == 0L) 0
            else (SystemClock.elapsedRealtime() - phaseSinceMs) / 1000,
        )
    }

    private fun VideoQuality.toDji(): StreamQuality = when (this) {
        VideoQuality.SD -> StreamQuality.SD
        VideoQuality.HD -> StreamQuality.HD
        VideoQuality.FULL_HD -> StreamQuality.FULL_HD
        VideoQuality.ORIGINAL -> StreamQuality.ORIGINAL
    }
}
