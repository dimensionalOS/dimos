package com.dimensional.mini4pro.zenoh

/**
 * **The channels of `docs/zenoh-topics.md`, as a machine copy of the catalogue.**
 *
 * The prose lives in `docs/zenoh-topics.md`, the subscriber-facing contract in
 * `docs/zenoh-replay-contract.md` §1, and the third implementation in `tools/zenohpublish`'s
 * `CATALOGUE` dict — which prints itself with `--keys` for exactly this reason. Three copies of a
 * table is two too many, so the rule that keeps them honest is the one that keeps the encoder and
 * the Python converter honest: each is written from the specification and then checked against the
 * others. [ZenohKeysTest] is this copy's half of that check, key expression by key expression.
 *
 * ## What a channel is *not*
 *
 * A channel is a name and a type. It is **not** a decision about whether anything is published:
 * that lives entirely in [ZenohEmission], which asks [ZenohTelemetryEncoder] and reports the
 * reason. Nothing here knows what an aircraft is.
 *
 * ## Two entries that carry a history
 *
 * [GIMBAL] is **retired from the bus** — 2026-07-27 replaced it with an edge in the `tf` tree,
 * built from the *commanded* angle rather than the reported one, because `KeyGimbalAttitude` is
 * change-driven and goes silent exactly while the camera is held still (measured median age 4.0 s
 * stationary, max 25.3 s). It survives here with **no key expression at all**, because
 * [ZenohEmission] still tallies it: a flight record carries no gimbal attitude, and "we withheld
 * this because the schema has no column for it" is a different statement from "we did not publish
 * it because it is retired". [keyOrNull] returns null for it and [PUBLISHED] excludes it, so it
 * cannot reach a bus by accident.
 *
 * `datum` is absent entirely and is not a `ZenohChannel` at all. It was deleted on 2026-07-27:
 * `pose` and `gps_location` are the same measurement expressed two ways, so a consumer recovers
 * the origin by subtracting them over every sample rather than trusting the one instant we chose
 * to announce it — measured agreement 0.0000 m, worst per-sample residual 4.4 µm.
 * `tools/zenohpublish` still carries a `datum` row because that tool replays frame files that
 * predate the deletion; **this side must never publish one.**
 */
enum class ZenohChannel(
    /** The logical channel, the middle segment of the key. DiMOS's own `drone/<channel>`. */
    val channel: String,
    /**
     * The LCM type, the last segment, joined with a **dot** and not a slash — `Topic.key_expr` in
     * `dimos/protocol/pubsub/impl/zenohpubsub.py`. Null for a channel with no key ([GIMBAL]).
     */
    val type: String?,
    val qos: ZenohQos,
) {
    ODOM("odom", "nav_msgs.Odometry", ZenohQos.DEFAULT),
    POSE("pose", "geometry_msgs.PoseStamped", ZenohQos.DEFAULT),
    GPS_LOCATION("gps_location", "sensor_msgs.NavSatFix", ZenohQos.DEFAULT),
    IMU("imu", "sensor_msgs.Imu", ZenohQos.DEFAULT),
    BATTERY("battery", "sensor_msgs.BatteryState", ZenohQos.DEFAULT),
    MODE("mode", "std_msgs.String", ZenohQos.DEFAULT),

    /**
     * **The frame tree** — every edge valid at one instant, in one message, at the state rate.
     *
     * Not derivable from [AircraftState] alone, which is why [ZenohEmission] does not decide it
     * and [ZenohTelemetryPump] does: the `base_link` → `camera` edge needs the gimbal's commanded
     * angle, and the gimbal lives behind `gimbal/`'s own seam and is in no flight-state snapshot.
     */
    TF("tf", "tf2_msgs.TFMessage", ZenohQos.DEFAULT),

    /**
     * **The camera intrinsics, and the two things they must not pretend.**
     *
     * A *fitted* focal length from two flights, no distortion claim at all, and a principal point
     * nobody has measured — `ZenohTelemetryEncoder.cameraInfo` carries the argument. Published on
     * change and at least once a second against a held resolution, because "once" has no meaning
     * on a bus a subscriber joins at any moment.
     *
     * Deliberately **not** gated on whether video is being published: a consumer that has our
     * pixels from somewhere else still needs to know that DJI's published 82.1° diagonal is 13.2 %
     * wrong for this stream, and the intrinsics cost 200 bytes a second.
     */
    CAMERA_INFO("camera_info", "sensor_msgs.CameraInfo", ZenohQos.DEFAULT),

    /**
     * **Encoded H.264, one access unit per message, never decoded and never re-encoded.**
     *
     * `DEFAULT` — reliable + **drop** — and that is the only defensible QoS here. `NEVER_DROP`
     * would let a slow subscriber wedge a 5.85 Mbit/s stream behind a blocking `put`, and
     * `LATEST_WINS` would discard the P frames a GOP needs while keeping the newest, which is the
     * one way to turn a gap into garbage.
     *
     * **Off by default and on its own publisher**, because the bandwidth is fifty times the rest
     * of this bus combined and it is the operator's to spend — see [ZenohVideoPublisher], which
     * owns the queue, the thread, the session and the GOP-aware drop policy that keeps this
     * channel from ever touching the telemetry one.
     */
    VIDEO("video", "foxglove_msgs.CompressedVideo", ZenohQos.DEFAULT),

    /**
     * **What the on-board AprilTag detector saw** — one message per detected frame, at most 10 a
     * second, in `drone/camera_optical`.
     *
     * **`Detection3DArray`, not a bare `Detection3D`** (Ivan, 2026-07-28). ROS's own convention
     * for a detection topic, and the type DiMOS consumes natively. The array carries exactly one
     * element today, and that is a fact about `vision/TagRecogniser` rather than a promise of this
     * channel — see `ZenohTelemetryEncoder.detectionsOrNull`. A consumer that assumed one would be
     * assuming something the *detector's* ranking rule happens to guarantee and this contract does
     * not.
     *
     * **Off by default**, like [VIDEO], and for a different cost. Video's switch is bandwidth;
     * this one is 5.5 kB/s — 552 bytes a message (bare id, 2026-07-29), measured off the committed fixture, of which
     * 288 are the all-zero covariance — and the reason is `docs/tag-detector.md` §6.1: the pose
     * rests on a *fitted* focal length, an *assumed* principal point and no distortion model, and
     * `docs/tag-detector.md` §7 names the hazard — *"publishing a coarse pose onto a bus where a
     * consumer cannot see the flag is the one way this could mislead somebody who never read this
     * document"*. Since 2026-07-29 that caveat lives in the contract row rather than as id
     * suffixes on every message (Ivan: the label stays bare — `tag36h11:<id>`); the switch is
     * so that spending it is still a deliberate act.
     *
     * `DEFAULT` — reliable + **drop** — because a detection repeats: a tag in view produces
     * another one 100 ms later, so a dropped message is a gap and not a lost fact. `NEVER_DROP`
     * would let a slow subscriber wedge the publisher's thread on behalf of a channel whose next
     * message says nearly the same thing.
     *
     * Shares [ZenohPublisher]'s queue with the telemetry channels rather than taking a second one
     * as [VIDEO] does. At 10 Hz against a 512-item queue that is about 2 % of the capacity; the
     * separate session video has exists for 5.87 Mbit/s, which this is not.
     */
    DETECTIONS("detections", "vision_msgs.Detection3DArray", ZenohQos.DEFAULT),

    /**
     * **Where the tag is believed to be in `drone/world`** — the record's own `TagWorld.fix`,
     * one message per fix, ≤10 Hz, `vision_msgs.Detection3DArray` with one element.
     *
     * **Exists to kill a composition artifact Ivan watched happen** (2026-07-29, replaying
     * mem2 stores): a consumer joining `detections` (camera-optical, sparse) against `tf`
     * edges of a *different instant* makes the tag visually ride the camera — camera up, tag
     * up — and a descending body drags the last sparse detection into the floor. The app
     * already solves that at source: `TagWorld.fix` composes each sighting with the aircraft
     * pose **at the sighting's own instant**, and this channel publishes that composition
     * rather than leaving every consumer to redo it against mismatched samples. One truth per
     * channel: [DETECTIONS] stays byte-identical, the camera-frame evidence; this is the
     * world-frame belief, and the two cross-reference rather than replace each other.
     *
     * Same type as [DETECTIONS], and the same bare-label rule — except that this schema has
     * no typed vehicle for a categorical belief grade and landing07-B measured why the grade
     * must travel (the baro ~1.2 m wrong within a minute while the size range was right), so
     * the id keeps a **minimal** documented tail: `tag36h11:<id>[;range=solve|size|baro]`
     * `[;pitch=reported]` — see `ZenohTelemetryEncoder.tagFixId`. Channel-level constants
     * (the assumed camera-to-body rotation, the z-on-datum-plane assumption) live in the
     * contract row, not on every message. Same switch as [DETECTIONS], deliberately: two
     * views of one detector, one operator decision. Same QoS argument: a fix repeats
     * ~100 ms later while the tag is in view.
     */
    TAG_FIX("tag_fix", "vision_msgs.Detection3DArray", ZenohQos.DEFAULT),

    /**
     * **The velocity command this bridge actually handed the aircraft** — one message per
     * virtual-stick send, `geometry_msgs.TwistStamped` in `drone/world` ENU, zeros included.
     *
     * **`Twist` is Ivan's decision** (2026-07-29: *"we can actually publish twist messages,
     * that's the correct thing to publish"*), stamped because every published reading on this
     * bus travels with its instant and its frame. Event-driven like [DETECTIONS] — the send IS
     * the event, ~10 Hz while guided flight is engaged and **nothing at all otherwise**, because
     * a zero setpoint is a command and no message means no command flowed. The distinction
     * matters: a subscriber watching this channel learns when software had its hands on the
     * sticks, and faking the quiet with resampled zeros would erase exactly that.
     *
     * **Always on when the bus is** — no switch, unlike [VIDEO] (5.87 Mbit/s) and [DETECTIONS]
     * (a coarse pose others might fly on). This is ~90 bytes at 10 Hz of our *own* decision,
     * already public on the record; there is no cost and no hazard to weigh.
     *
     * `DEFAULT` — reliable + **drop** — by [DETECTIONS]' argument: a setpoint repeats 100 ms
     * later, so a dropped message is a gap and not a lost fact, and `NEVER_DROP` would let a
     * slow subscriber wedge the publisher's thread behind a stream that says nearly the same
     * thing ten times a second. The flight record's `stick_cmd` line is the lossless copy.
     */
    SETPOINT("setpoint", "geometry_msgs.TwistStamped", ZenohQos.DEFAULT),

    /**
     * **DJI's own wind-speed estimate, in m/s** — one `std_msgs.Float32` per key delivery,
     * nothing at all between deliveries.
     *
     * **A bare Float is Ivan's decision** (2026-07-30: 1-D data rides a Float, 3-D a Vector3
     * — and this source is genuinely 1-D: `KeyWindSpeed` is a scalar in dm/s, and the only
     * direction DJI offers is `KeyWindDirection`'s 8-way compass enum, which has no components
     * and no vertical. Synthesizing a Vector3 from the pair would fabricate a 22.5°-quantised
     * bearing as continuous and a z=0 that means *unmeasured* — the unknown-is-never-zero
     * failure — so direction stays on the flight record until a genuinely vectorial source
     * exists). Why it is on the bus at all is landing14: a "flyaway" — lateral stick ignored,
     * 1 m/s drift against full deflection — was 9.1 m/s wind, already on the record, read a
     * day late; a DiMOS consumer should not have to wait for the post-mortem.
     *
     * **Event-driven on key delivery, and the doc row forbids freshness-gating it.**
     * `KeyWindSpeed` is change-driven like every DJI telemetry key (the trap this project has
     * hit 7+ times): silence means UNCHANGED, not stale. So the channel publishes exactly when
     * DJI delivers — the record's `windSpeedDmS` line and the bus message are the same event,
     * `[ZenohBus.publishWind]` fed by the recorder's own on-change tap — and is deliberately
     * **not** in [ZenohEmission.emit]'s per-sample output: wind is in no [AircraftState], so
     * no sampler can ever republish a held reading as a fresh measurement.
     *
     * Always on when the bus is, like [SETPOINT]: twelve bytes a few times a minute of a
     * reading already public on the record — nothing to weigh. `DEFAULT` QoS by the same
     * argument as every repeating reading: the next change supersedes a dropped one, and
     * `NEVER_DROP` would let a slow subscriber block the publisher for a value the record
     * keeps losslessly.
     */
    WIND("wind", "std_msgs.Float32", ZenohQos.DEFAULT),

    /**
     * **Operator sentences** — every refusal, every engage/disengage reason, every substitution
     * notice, fed by the same `command/Announcer` that writes QGC's `STATUSTEXT`, so the two
     * interfaces can never disagree about what an operator was told.
     *
     * `NEVER_DROP`, and that is the one QoS on this bus with a cost worth naming: reliable +
     * **block**. The block happens on the publisher's own thread, behind the bounded queue, and
     * never on the thread that composed the sentence — see [ZenohPublisher].
     */
    STATUS("status", "std_msgs.String", ZenohQos.NEVER_DROP),

    /**
     * **Every warning the aircraft gives us** — DJI's device-health messages, its wind warning, and
     * whatever source is added next — one message per *change*, never per sample.
     *
     * Added 2026-07-30, on Ivan's instruction after landing17: *"make sure to pass all of these DJI
     * warnings into both Zenoh and QGroundControl, of course. Just so I don't need to ask you again
     * for the next warning."* The flight that prompted it recorded four `LEVEL_2` wind warnings in
     * a 14.2 m/s wind and told nobody on any surface.
     *
     * **`diagnostic_msgs.DiagnosticArray`, the standard type, not a JSON string.** The family
     * exists in `dimos_lcm` (`diagnostic_msgs/{DiagnosticArray,DiagnosticStatus,KeyValue}`), and
     * Ivan's standing rule is to use standard types where upstream has one; `status` and
     * `command_ack` carry JSON in a `std_msgs.String` only because nothing upstream fits them. DJI's
     * own words that ROS has no field for — the DJI state name, the change, the measured wind speed
     * — ride the status's `values`, which is exactly what `KeyValue` is for.
     *
     * `NEVER_DROP`, with `status`'s argument and one addition of its own: a warning is said once.
     * `pose` dropping a sample loses a position that the next sample supersedes; `warnings`
     * dropping a message loses *the fact that the aircraft is in trouble*, and nothing later
     * repeats it — the diff upstream is deliberately silent about things it has already said.
     *
     * Always on with the bus. It is a handful of messages a flight: landing17, the windiest
     * recorded, would have produced nine.
     */
    WARNINGS("warnings", "diagnostic_msgs.DiagnosticArray", ZenohQos.NEVER_DROP),

    /**
     * Retired 2026-07-27, kept as a tally reason only. **Has no key and is never published.**
     * See the class doc.
     */
    GIMBAL("gimbal", null, ZenohQos.DEFAULT),
    ;

    /**
     * `<prefix>/<channel>/<pkg>.<Type>`, or null for a channel with no wire presence.
     *
     * Not a property, because the prefix is configuration: `dimos/drone` is the default and
     * `Q1` of `docs/zenoh-dimos-transport.md` §7 is still open on whether a second aircraft gets
     * its own.
     */
    fun keyOrNull(prefix: String = DEFAULT_PREFIX): String? =
        type?.let { "$prefix/$channel/$it" }

    companion object {

        /** DiMOS's `transport_topic()` is literally `"dimos/" + name`, and our name is `drone`. */
        const val DEFAULT_PREFIX = "dimos/drone"

        /** Every channel that has a key expression, in catalogue order. */
        val PUBLISHED: List<ZenohChannel> = entries.filter { it.type != null }
    }
}

/**
 * The three delivery classes, named as DiMOS names them (`zenohpubsub.py`).
 *
 * **Every QoS on this bus is set explicitly, by the publisher, and none of it is inherited** —
 * `docs/zenoh-replay-contract.md` D-1. DiMOS's automatic rule matches the *logical channel name*
 * against `("human_input", "agent", "agent_idle", "command")`, and ours are all `drone/…`, so
 * none of them match and all of them would silently get zenoh's defaults. A subscriber cannot
 * observe QoS and cannot request it; this enum exists so two publishers agree, not because
 * anything downstream can check.
 *
 * `LATEST_WINS` is deliberately absent: nothing this project publishes uses it. It exists in
 * DiMOS for `Image` and `PointCloud2`, and Z-10 keeps video off this bus from the phone.
 */
enum class ZenohQos {
    /** Zenoh's publisher default: reliable + **drop**. Everything periodic. */
    DEFAULT,

    /** Reliable + **block**. Discrete, non-repeating facts: a refusal, a warning, a verdict. */
    NEVER_DROP,
}
