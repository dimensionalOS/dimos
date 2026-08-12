package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.telemetry.AircraftState

/**
 * **One sample of the aircraft into whatever the bus should carry for it** — the live counterpart
 * of `replay/ZenohReplay`'s loop, and the only part of the publishing path that had to be written
 * rather than shared.
 *
 * [ZenohEmission] decides *whether* each channel publishes and *why*. This class decides the two
 * things a replay cannot decide, because both require not being able to see the future:
 *
 *  1. **where the local frame's origin is** ([datum]), and
 *  2. **when a channel is due** ([Cadence]).
 *
 * Everything else it does is hand the bytes to a sink and count what came back. It reads no clock
 * — the caller passes both — touches no Android, no DJI and no Zenoh, and is therefore assertable
 * sample by sample on the JVM.
 *
 * ## The origin, and the one place this genuinely differs from the offline converter
 *
 * `ZenohReplay.takeoffDatum` scans a whole record for the first sample with the motors on, and
 * falls back to the first usable fix if the motors never came on. **Both passes look ahead**, and
 * a live publisher has no later samples. So the rule here is causal, and it is stated as a rule
 * rather than left to emerge:
 *
 *  - While there is no origin, every sample is offered one. It is accepted **only** when DJI says
 *    the motors are on and [OdomDatum.atTakeoff] returns a coordinate — which requires a *fresh*
 *    fix, not a cached one, because an origin recorded from the last place the GPS spoke anchors
 *    the whole flight's local frame to the wrong point and every coordinate afterwards inherits
 *    that error invisibly.
 *  - **Except** when the aircraft is already flying and we still have none. That is the bridge
 *    being restarted mid-flight, and refusing an origin there would mean an aircraft in the air
 *    publishing no `pose` and no `odom` for the rest of the battery. The origin is taken from the
 *    first usable fix instead, and it is [DatumOrigin.MID_FLIGHT] — announced, logged, and
 *    distinguishable, because it is *not* the takeoff point and a consumer measuring "height above
 *    where it took off" would be measuring height above wherever the bridge came up.
 *  - Once taken it **never moves**, for the session. A datum that shifted mid-flight would move
 *    every local coordinate under a consumer that has no way to notice.
 *
 * Until an origin exists, `pose` and `odom` are withheld as [Withheld.NO_DATUM] and the four
 * channels that need no origin — `gps_location`, `imu`, `battery`, `mode` — publish normally.
 * That is the honest shape: absence is the signal, and a `Point` at 0,0,0 is the takeoff pad.
 *
 * ## Cadence
 *
 * `docs/zenoh-replay-contract.md` D-10: the rates in the catalogue are **ceilings, not
 * guarantees**, and nothing downstream may assume one. So this class enforces the ceiling and
 * nothing else — a channel that is due publishes if [ZenohEmission] allows it, and a channel that
 * is not due is [Withheld.RATE_LIMITED], which is distinct from every withholding reason precisely
 * because it is not one.
 */
class ZenohTelemetryPump(
    /** Where an encoded message goes. Returns false when it was dropped, which is counted. */
    private val sink: (ZenohChannel, ByteArray) -> Boolean,
    private val cadence: Cadence = Cadence(),
    /**
     * The camera's earth-referenced attitude right now, or null when nothing knows where it is
     * pointing.
     *
     * **A supplier rather than a value, and read fresh on every sample**, because the gimbal is
     * commanded and reported on threads this class never touches — the same indirection
     * `Bridge.startVideo` uses to reach the recorder's sidecar, and for the same reason.
     *
     * The caller decides which angle it is. `docs/mem2-converter.md` §2.2 wants the **commanded**
     * one: it is exact, always fresh, and already carries DJI's verdict, while `KeyGimbalAttitude`
     * is change-driven and goes silent exactly when the camera is held still. When nothing has
     * commanded the gimbal this session — the camera being aimed from the RC, which is what both
     * AprilTag datasets contain — the reported angle with its age is the honest fallback, and
     * [GimbalEarthAttitude.source] says which arrived.
     *
     * Null omits the `base_link` → `camera` edge. The tree is still published; the camera is
     * simply not in it, which is a statement rather than a gap.
     */
    private val gimbal: () -> GimbalEarthAttitude? = { null },
    /**
     * The video stream's stated geometry, or null before any frame has stated one.
     *
     * Held by the caller rather than by this class, because the statement arrives on MSDK's decode
     * thread and the holding rule belongs with whatever is watching that stream. `LogEntry.Frame`
     * writes `w`/`h` **on change plus once at the start** on exactly the argument that applies
     * here: a resolution change invalidates the intrinsics, so it is a discrete fact and not
     * something to infer.
     */
    private val resolution: () -> Pair<Int, Int>? = { null },
    /** One sentence to the operator, for the two datum events. Never called per sample. */
    private val announce: (String) -> Unit = {},
    private val log: (String) -> Unit = {},
) {

    /**
     * The **ceiling** for each channel, in milliseconds between messages.
     *
     * `docs/zenoh-topics.md`'s table, with one deviation named in it and left alone here: `imu` is
     * listed at 5 Hz while DJI's attitude was *measured* arriving at ~2 Hz, so a 5 Hz ceiling
     * publishes the same reading twice. That is open item 1 of that document — publish at the
     * measured rate and let consumers interpolate, or keep the padding — and it is a decision
     * about the contract, not something a publisher should quietly settle by choosing a number.
     * The padding stays, because it is what `tools/zenohpublish` replays and a subscriber must not
     * be able to tell the two publishers apart.
     */
    data class Cadence(
        /**
         * **100 ms — the measured rate of the DJI position feed, not a round number.**
         *
         * Measured on landing10 (`datasets/landing10/20260729-120342.001.jsonl`, 2026-07-29):
         * over 291.9 s the `dji_state` position age resets 2884 times — a fresh position
         * every 101 ms, **9.88 Hz** — with delivery age median 45 ms, p90 85 ms, max 169 ms.
         * Publishing faster than the feed re-publishes cached positions (the change-driven-key
         * lesson: silence means unchanged, and a resend adds information-free rows); publishing
         * slower discards positions the aircraft delivered. The old 200 ms ceiling did exactly
         * that, and it was *seen*: at the measured 1–1.5 m/s flight speeds, 5 Hz is 20–30 cm of
         * travel per published sample, and mem2 replays showed the aircraft jumping in ~25 cm
         * steps (Ivan, 2026-07-29). At the feed's own 10 Hz the step is the feed's, 10–15 cm,
         * and no cadence can do better without inventing positions.
         */
        val poseMs: Long = 100,
        val odomMs: Long = 200,
        val gpsMs: Long = 200,
        val imuMs: Long = 200,
        val batteryMs: Long = 1_000,
        /**
         * `mode` is **on change, and at least this often**. The heartbeat is not padding: a
         * subscriber that joins mid-flight would otherwise wait for the aircraft to change mode
         * before learning what mode it is in, and a steady `GPS_ATTI` cruise never changes.
         */
        val modeMs: Long = 1_000,
        /**
         * `tf` at the state rate, matching `pose` — [poseMs] carries the measurement (the
         * position feed delivers at 9.88 Hz, landing10) — because the tree's translation is the
         * same position the pose carries and must not step coarser than it.
         *
         * **The tree's *orientation* cannot honestly be 10 Hz and this does not pretend it is:**
         * attitude was measured arriving at ~2 Hz
         * (`docs/measurements/2026-07-26-attitude-and-staleness.md`), so between attitude
         * samples the rotation half of `world → base_link` — and the camera edge composed
         * against it — repeats the held value while the translation moves. That is the honest
         * shape: position at the position feed's rate, orientation at the attitude feed's,
         * both inside one message stamped with the sample's instant.
         *
         * The **fixed** `camera` → `camera_optical` edge rides inside the same message rather than
         * having a rate of its own, which is what makes the catalogue's *"fixed edges ≥1 Hz"* true
         * by construction: at 10 Hz it is republished ten times more often than the floor.
         */
        val tfMs: Long = 100,
        /**
         * `camera_info` is **on change, and at least this often** — the same rule `mode` follows
         * and for the same reason. The intrinsics change only when the resolution does, and
         * *"once"* has no meaning on a bus a subscriber joins at any moment. 200 bytes a second.
         */
        val cameraInfoMs: Long = 1_000,
    ) {
        fun intervalMs(channel: ZenohChannel): Long = when (channel) {
            ZenohChannel.POSE -> poseMs
            ZenohChannel.ODOM -> odomMs
            ZenohChannel.GPS_LOCATION -> gpsMs
            ZenohChannel.IMU -> imuMs
            ZenohChannel.BATTERY -> batteryMs
            ZenohChannel.MODE -> modeMs
            ZenohChannel.TF -> tfMs
            ZenohChannel.CAMERA_INFO -> cameraInfoMs
            // Event-driven or frame-driven; never offered by this class. `video` is
            // `ZenohVideoPublisher`'s, on its own queue and its own thread; `detections` is the
            // tag detector's, on the `tag-detect` worker, and its rate is already capped at 10 Hz
            // on the *producer* side by `vision/RateCap` — a second ceiling here would drop
            // sightings the detector had already paid 2.07 MB and 25–50 ms to produce.
            // `tag_fix` rides the same sightings and the same cap; `setpoint` is the send
            // itself, ~10 Hz while guided flight is engaged, and a ceiling on it would fake
            // quiet where a command flowed. `wind` is one DJI key delivery — a few per
            // minute, change-driven — and any interval here would be the resampling its
            // contract forbids.
            // `warnings` is the same shape as `wind` and more so: it is published on a *change*
            // decided by `warn/WarningMonitor`, and an interval here would either resample a
            // change (the per-sample repeat the whole path forbids) or invent a heartbeat that
            // made a dead publisher look like a healthy aircraft.
            ZenohChannel.STATUS, ZenohChannel.GIMBAL, ZenohChannel.VIDEO,
            ZenohChannel.DETECTIONS, ZenohChannel.TAG_FIX, ZenohChannel.SETPOINT,
            ZenohChannel.WIND, ZenohChannel.WARNINGS,
            -> Long.MAX_VALUE
        }
    }

    /** Where this session's local origin came from — see the class doc. */
    enum class DatumOrigin {
        /** No origin yet. `pose` and `odom` are withheld. */
        NONE,

        /** The normal case: DJI reported the motors on, with a fresh fix, and this is that point. */
        MOTORS_ON,

        /**
         * The bridge came up with the aircraft already airborne, so the origin is where the
         * *bridge* started, not where the aircraft took off. Announced, because every altitude and
         * every local metre published afterwards is measured from it.
         */
        MID_FLIGHT,
    }

    @Volatile
    var datum: OdomDatum? = null
        private set

    @Volatile
    var datumOrigin: DatumOrigin = DatumOrigin.NONE
        private set

    /** The reason each channel gave on the last sample. For the status screen, and for tests. */
    @Volatile
    var lastReasons: Map<ZenohChannel, Withheld> = emptyMap()
        private set

    private val lastSentMs = HashMap<ZenohChannel, Long>()
    private var lastMode: String? = null

    /**
     * The resolution the last `camera_info` was built for, so a change publishes **immediately**
     * rather than waiting out the heartbeat.
     *
     * The same second rule `mode` has, for the sharper version of the same reason: a mode an
     * operator learns a second late is a second of not knowing, while intrinsics a second stale
     * are a second of a consumer solving a pose against the wrong focal length.
     */
    private var lastResolution: Pair<Int, Int>? = null

    /** Per-channel counts of messages handed to the sink, and of what the sink refused. */
    private val offeredByChannel = HashMap<ZenohChannel, Long>()

    fun offered(channel: ZenohChannel): Long = synchronized(lastSentMs) {
        offeredByChannel[channel] ?: 0L
    }

    /**
     * Forgets everything about this flight.
     *
     * Called when the publisher starts, never while it runs. **The origin is per session and
     * per flight** — "one flight per recording, one flight per session" — because across twenty
     * measured sessions on the same patch of ground the origins separate by a median of 2.5 m and
     * up to 13.4 m, so a coordinate carried over from a previous session is not a small error, it
     * is a meaningless number.
     */
    @Synchronized
    fun reset() {
        datum = null
        datumOrigin = DatumOrigin.NONE
        lastSentMs.clear()
        offeredByChannel.clear()
        lastMode = null
        lastResolution = null
        lastReasons = emptyMap()
    }

    /**
     * Publishes whatever this sample warrants, and returns how many messages reached the sink.
     *
     * @param monoMs a monotonic reading, for cadence only. Never a wall clock: an NTP step
     *   mid-flight would otherwise either stall every channel for the length of the step or
     *   release them all at once.
     * @param unixMs the wall-clock instant **of this sample**, which becomes `header.stamp`.
     *   D-5: the reading's time in Unix seconds at millisecond resolution. For a held value that
     *   is the sample's time rather than the key's delivery time, which is exactly what the
     *   offline converter stamps a held reading with — the two must agree or a subscriber could
     *   tell a replay from an aircraft, and §7 of the contract lists what may differ.
     */
    @Synchronized
    fun sample(s: AircraftState, monoMs: Long, unixMs: Long): Int {
        latchDatum(s)
        val stamp = LcmTime.ofEpochSeconds(unixMs / 1000.0)
        val reasons = HashMap<ZenohChannel, Withheld>()
        var sent = 0
        // `encode = true`, and the encoding of a rate-limited channel is thrown away. That looks
        // wasteful and is deliberate: it is what makes [ZenohEmission.emit]'s agreement check
        // run on the live path too, so a drift between the gates and the encoder fails on an
        // aircraft the same way it fails in a test rather than silently publishing nothing and
        // counting it as a publish. The cost is at most six small LCM serialisations per sample
        // — a few hundred bytes at 5 Hz, against a video path that moves 649 kB/s.
        for (e in ZenohEmission.emit(s, datum, stamp, encode = true)) {
            val bytes = e.bytes
            if (bytes == null) {
                reasons[e.channel] = e.reason
                continue
            }
            if (!due(e.channel, s, monoMs)) {
                reasons[e.channel] = Withheld.RATE_LIMITED
                continue
            }
            reasons[e.channel] = Withheld.PUBLISHED
            lastSentMs[e.channel] = monoMs
            if (e.channel == ZenohChannel.MODE) lastMode = s.flightMode
            offeredByChannel[e.channel] = (offeredByChannel[e.channel] ?: 0L) + 1L
            if (sink(e.channel, bytes)) sent++
        }
        sent += sampleTf(s, stamp, monoMs, reasons)
        sent += sampleCameraInfo(stamp, monoMs, reasons)
        lastReasons = reasons
        return sent
    }

    /**
     * The frame tree for this sample.
     *
     * Decided here rather than in [ZenohEmission] because the camera edge needs the gimbal, which
     * is not in an [AircraftState] — [ZenohEmission.NOT_DECIDED_HERE] is where that boundary is
     * argued. The *gates* are still `ZenohEmission`'s, through [ZenohEmission.tfReason], so this
     * method decides nothing about freshness or liveness on its own.
     */
    private fun sampleTf(
        s: AircraftState,
        stamp: LcmTime,
        monoMs: Long,
        reasons: HashMap<ZenohChannel, Withheld>,
    ): Int {
        val d = datum
        val reason = ZenohEmission.tfReason(s, d)
        if (reason != Withheld.PUBLISHED) {
            reasons[ZenohChannel.TF] = reason
            return 0
        }
        if (!due(ZenohChannel.TF, s, monoMs)) {
            reasons[ZenohChannel.TF] = Withheld.RATE_LIMITED
            return 0
        }
        // `d` is non-null: `tfReason` returns NO_DATUM otherwise, and we returned above.
        val message = ZenohTelemetryEncoder.tfOrNull(s, d!!, stamp, gimbal(), Gate.HELD)
            ?: throw IllegalStateException(
                "tf: the gates said PUBLISHED and the encoder returned nothing — " +
                    "ZenohEmission.tfReason has drifted from ZenohTelemetryEncoder.tfOrNull",
            )
        reasons[ZenohChannel.TF] = Withheld.PUBLISHED
        lastSentMs[ZenohChannel.TF] = monoMs
        offeredByChannel[ZenohChannel.TF] = (offeredByChannel[ZenohChannel.TF] ?: 0L) + 1L
        return if (sink(ZenohChannel.TF, TfMessageCodec.encode(message))) 1 else 0
    }

    /**
     * The camera intrinsics, on change and at least once a second.
     *
     * Gated on nothing but the resolution: intrinsics are a property of the camera rather than a
     * reading off the aircraft, so they neither go stale nor need a link. A subscriber that has
     * lost the aircraft still wants to know what the lens was.
     */
    private fun sampleCameraInfo(
        stamp: LcmTime,
        monoMs: Long,
        reasons: HashMap<ZenohChannel, Withheld>,
    ): Int {
        val res = resolution()
        if (res == null || res.first <= 0 || res.second <= 0) {
            reasons[ZenohChannel.CAMERA_INFO] = Withheld.RESOLUTION_UNKNOWN
            return 0
        }
        val changed = res != lastResolution
        val last = lastSentMs[ZenohChannel.CAMERA_INFO]
        if (!changed && last != null && monoMs - last < cadence.cameraInfoMs) {
            reasons[ZenohChannel.CAMERA_INFO] = Withheld.RATE_LIMITED
            return 0
        }
        val message = ZenohTelemetryEncoder.cameraInfo(res.first, res.second, stamp)
            ?: run {
                reasons[ZenohChannel.CAMERA_INFO] = Withheld.RESOLUTION_UNKNOWN
                return 0
            }
        if (changed && lastResolution != null) {
            log("zenoh: camera_info resolution changed ${lastResolution!!.first}x" +
                "${lastResolution!!.second} -> ${res.first}x${res.second}; " +
                "intrinsics republished immediately")
        }
        lastResolution = res
        reasons[ZenohChannel.CAMERA_INFO] = Withheld.PUBLISHED
        lastSentMs[ZenohChannel.CAMERA_INFO] = monoMs
        offeredByChannel[ZenohChannel.CAMERA_INFO] =
            (offeredByChannel[ZenohChannel.CAMERA_INFO] ?: 0L) + 1L
        return if (sink(ZenohChannel.CAMERA_INFO, CameraInfoCodec.encode(message))) 1 else 0
    }

    /**
     * Whether a channel may publish now.
     *
     * `mode` is the one with a second rule: it publishes **on change** as well as on the
     * heartbeat, because a mode transition is the single most operationally interesting thing on
     * this bus and waiting up to a second to say `GO_HOME` would be a second of an operator not
     * knowing the aircraft is leaving.
     */
    private fun due(channel: ZenohChannel, s: AircraftState, monoMs: Long): Boolean {
        if (channel == ZenohChannel.MODE && s.flightMode != lastMode) return true
        val last = lastSentMs[channel] ?: return true
        return monoMs - last >= cadence.intervalMs(channel)
    }

    /** The causal origin rule. See the class doc; this is only its mechanism. */
    private fun latchDatum(s: AircraftState) {
        if (datum != null) return
        val candidate = OdomDatum.atTakeoff(s) ?: return
        when {
            s.motorsOn == true -> {
                datum = candidate
                datumOrigin = DatumOrigin.MOTORS_ON
                val line = "zenoh: local origin set at motor start, " +
                    "%.7f %.7f".format(candidate.latitudeDeg, candidate.longitudeDeg)
                log(line)
            }
            s.isFlying == true -> {
                datum = candidate
                datumOrigin = DatumOrigin.MID_FLIGHT
                val line = "Zenoh local origin taken in flight — drone/world is where the bridge " +
                    "started, NOT the takeoff point"
                log("zenoh: $line")
                announce(line)
            }
            // On the ground with the motors off: no origin, and none is wanted. `pose` and `odom`
            // stay withheld until the aircraft is actually going somewhere.
            else -> Unit
        }
    }
}
