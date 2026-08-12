package com.dimensional.mini4pro.record

import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.Signal

/**
 * Every line the flight recorder can write. This file **is** the schema; the
 * matching reader is `tools/flightlog` and the prose is `docs/flight-recording.md`.
 * Changing a field name here means changing both.
 *
 * ## Why entries are objects rather than strings
 *
 * An entry is constructed on whichever thread produced the data — the telemetry
 * loop, the guided-control loop, a DJI callback — and **rendered to JSON on the
 * recorder's writer thread**. Rendering is where the reflection over MAVLink
 * payloads and the string building happen, so none of that cost lands on a thread
 * that is flying an aircraft. Everything an entry captures is therefore an
 * immutable snapshot taken at construction; nothing here may hold a reference to
 * mutable SDK state.
 *
 * ## The one clock
 *
 * [monoNanos] is `SystemClock.elapsedRealtimeNanos()` for every entry without
 * exception — one clock, so entries are totally ordered and differences are
 * meaningful even across a wall-clock step. The header carries the wall-clock
 * anchor. On the wire the field is `t`: **seconds since the session's first
 * monotonic reading**, six decimals. That matches `mavcapture-1`'s `t` so the two
 * formats align without unit thought, and the header's `started_mono_ns` recovers
 * the absolute monotonic value if it is ever needed.
 */
sealed class LogEntry {

    abstract val monoNanos: Long

    /** The `k` discriminator. */
    abstract val kind: String

    /**
     * True for entries that must reach the disk immediately rather than at the
     * next flush interval: the discrete moments a post-mortem is built from.
     */
    open val urgent: Boolean get() = false

    /** Writes this entry's own members. `t` and `k` are written by the writer. */
    abstract fun writeBody(o: JsonObject)

    // ─────────────────────────────────────────────────────────────────────────
    // MAVLink, both directions
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One MAVLink message crossing our boundary.
     *
     * Both a **decoded field map** and the **raw bytes as hex** are recorded, and
     * that redundancy is the point: a bug in our own decoding cannot destroy the
     * evidence, because the hex is still there to be re-read by pymavlink.
     *
     * [hexSource] is honest about where the bytes came from:
     *  - `"wire"` — the exact datagram. Inbound always achieves this, because
     *    dronefleet's `MavlinkMessage.getRawBytes()` hands back the original packet.
     *  - `"reserialized"` — we re-encoded the payload ourselves because nothing
     *    taps the outbound socket. The payload bytes are identical to what went
     *    out (same library, same serializer); the **sequence number and therefore
     *    the CRC are not**, because `MavlinkLink` keeps its counter private. Never
     *    use a `reserialized` frame to reason about sequence continuity or loss —
     *    use QGC's `.tlog` or a `tools/mavcapture` man-in-the-middle for that.
     *    `docs/flight-recording.md` has the four-line tap that turns this into
     *    `"wire"`.
     *
     * [hexProvider] and [fieldsProvider] are **suppliers, not strings**: hex
     * encoding, reflection over a dozen annotated getters and re-serialisation would
     * otherwise all land on the telemetry thread forty times a second. They are
     * invoked exactly once, on the writer thread, in [writeBody]. Everything they
     * close over is immutable.
     */
    class Mav(
        override val monoNanos: Long,
        val inbound: Boolean,
        val name: String,
        val messageId: Int,
        val systemId: Int?,
        val componentId: Int?,
        val sequence: Int?,
        val hexSource: String?,
        private val hexProvider: () -> String?,
        private val fieldsProvider: () -> String?,
    ) : LogEntry() {
        override val kind: String get() = if (inbound) KIND_MAV_IN else KIND_MAV_OUT

        override fun writeBody(o: JsonObject) {
            val hex = try { hexProvider() } catch (e: Throwable) { null }
            var err: String? = null
            val fields = try {
                fieldsProvider()
            } catch (e: Throwable) {
                // A decode failure is itself evidence, and the hex above survives it.
                err = "${e.javaClass.simpleName}: ${e.message}"
                null
            }
            o.put("name", name)
            o.put("id", messageId)
            o.put("sys", systemId)
            o.put("comp", componentId)
            o.put("seq", sequence)
            o.put("len", hex?.let { it.length / 2 })
            o.put("hex", hex)
            o.put("hexsrc", if (hex != null) hexSource else null)
            o.putRaw("f", fields)
            o.put("err", err)
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DJI state
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * The fast-changing half of [AircraftState], at the recorder's rate, in
     * **DJI-native units, unconverted**. That is deliberate: if only the converted
     * MAVLink value were logged, a units bug in `TelemetryEncoder` and a genuine
     * oddity from the aircraft would be indistinguishable afterwards. The
     * converted value is on the same timeline in the neighbouring `mav_out` entry.
     *
     * The slow-changing half goes to [Field] on change instead — see [StateDelta].
     *
     * ## `age` — the answer to "was this value real?"
     *
     * The numbers above are read out of a cache at the recorder's rate, and DJI
     * updates each key on its own schedule: `KeyAircraftVelocity` fired **once in
     * 35 s** on the ground probe. So a 25 Hz `dji_state` line is frequently
     * re-recording a value that arrived long ago, and a flat velocity trace could
     * equally be "not moving" or "not updating".
     *
     * The `age` object carries **milliseconds since that reading was last
     * delivered by DJI**, so the two are separable on the same line as the value
     * they describe. It is written for the four readings this entry carries; the
     * remaining signals show up through the `signal_stale` / `signal_fresh`
     * events, which are the discrete version of the same fact.
     *
     * | member | reading | `Signal` |
     * |---|---|---|
     * | `age.pos` | `lat`/`lon` | `POSITION` |
     * | `age.relalt` | `relalt` | `ALTITUDE` |
     * | `age.att` | `roll`/`pitch`/`yaw` | `ATTITUDE` |
     * | `age.vel` | `vn`/`ve`/`vd` | `VELOCITY` |
     *
     * An absent age means the signal has **never** been delivered — the same rule
     * the format uses everywhere: absent is not zero. An age of 0 is the opposite
     * claim, that it arrived in this very millisecond.
     *
     * Cost, against the measured 25 Hz per-kind table in
     * `docs/flight-recording.md`: `dji_state` lines average ~136 B, and a full
     * `age` object is ~40 B, so this is roughly +3 % of the whole file. It buys
     * the difference between a diagnosable and an undiagnosable velocity trace.
     */
    class DjiState(
        override val monoNanos: Long,
        val latitude: Double?,
        val longitude: Double?,
        val relativeAltitudeM: Double?,
        val rollDeg: Double?,
        val pitchDeg: Double?,
        val yawDeg: Double?,
        val velocityNorth: Double?,
        val velocityEast: Double?,
        val velocityDown: Double?,
        val positionAgeMs: Long? = null,
        val altitudeAgeMs: Long? = null,
        val attitudeAgeMs: Long? = null,
        val velocityAgeMs: Long? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_DJI_STATE

        override fun writeBody(o: JsonObject) {
            // 7 decimals of degree ≈ 1.1 cm, the resolution MAVLink degE7 carries.
            o.put("lat", latitude, 7)
            o.put("lon", longitude, 7)
            o.put("relalt", relativeAltitudeM, 3)
            o.put("roll", rollDeg, 2)
            o.put("pitch", pitchDeg, 2)
            o.put("yaw", yawDeg, 2)
            // DJI velocity carries ~10 cm precision (see docs/msdk-keys.md); 3 dp
            // is two orders finer than the sensor.
            o.put("vn", velocityNorth, 3)
            o.put("ve", velocityEast, 3)
            o.put("vd", velocityDown, 3)
            // Omitted entirely when no age is known — see JsonObject.obj.
            o.obj("age") { w ->
                w.put("pos", positionAgeMs)
                w.put("relalt", altitudeAgeMs)
                w.put("att", attitudeAgeMs)
                w.put("vel", velocityAgeMs)
            }
        }

        companion object {
            fun of(monoNanos: Long, s: AircraftState) = DjiState(
                monoNanos = monoNanos,
                latitude = s.latitude,
                longitude = s.longitude,
                relativeAltitudeM = s.relativeAltitude,
                rollDeg = s.rollDeg,
                pitchDeg = s.pitchDeg,
                yawDeg = s.yawDeg,
                velocityNorth = s.velocityNorth,
                velocityEast = s.velocityEast,
                velocityDown = s.velocityDown,
                positionAgeMs = s.ageMs(Signal.POSITION),
                altitudeAgeMs = s.ageMs(Signal.ALTITUDE),
                attitudeAgeMs = s.ageMs(Signal.ATTITUDE),
                velocityAgeMs = s.ageMs(Signal.VELOCITY),
            )
        }
    }

    /**
     * One slow-changing value, written **only when it changes**: flight mode,
     * failsafe, home point, battery, obstacle-avoidance flags, connection state.
     *
     * Split by cadence rather than by importance. A mode change is then a discrete
     * timestamped line rather than something to be inferred from the diff between
     * two state samples, and the file stays small without losing signal.
     *
     * [previous] is carried so a change reads at a glance without scanning
     * backwards, which matters when the interesting line is one of 200 000.
     */
    class Field(
        override val monoNanos: Long,
        val name: String,
        val value: String?,
        val previous: String?,
        val numeric: Double? = null,
        val decimals: Int = 3,
    ) : LogEntry() {
        override val kind: String get() = KIND_DJI_FIELD
        override val urgent: Boolean get() = true

        override fun writeBody(o: JsonObject) {
            o.put("f", name)
            o.put("v", value)
            o.put("prev", previous)
            o.put("n", numeric, decimals)
        }
    }

    /**
     * One **DJI warning changing state**, from any source: the device-health messages ("Aircraft
     * overheating", "Compass interference"), which this bridge did not see at all until 2026-07-26,
     * and the flight controller's wind warning, which it saw but never said anything about until
     * landing17.
     *
     * **`dji_warn` since 2026-07-30; it was `dji_health` before that.** The old kind is kept as
     * [KIND_DJI_HEALTH] so records written up to and including landing17 still read; nothing writes
     * it any more. The rename is the record half of `warn/WarningBus` becoming the single owner of
     * every warning — one line shape for every source, with [source] saying which, rather than a
     * new kind per subsystem.
     *
     * Written on **change only**, never per delivery: DJI hands over the whole current list every
     * time anything in it moves, and a key listener fires on every value, so a per-delivery entry
     * would re-record a standing overheat forever. `warn/WarningMonitor` owns that diff and its
     * reasoning.
     *
     * The fields that make this worth having over a plain [Event]:
     *
     *  - [source], so "the wind warning" and "the ESC warning" are one query apart.
     *  - [state] and [level] together — DJI's own word and our translation of it.
     *  - [level] and [previousLevel] together, so an escalation is one line rather than a
     *    subtraction across two.
     *  - [componentId]/[sensorIndex], because DJI reports the same [code] per component — four
     *    ESCs, two IMUs — and without them "all four motors" reads as one motor.
     *  - [measurement], because landing14 measured that a level without its number is not enough.
     *  - [forwarded] and [rateLimited], which answer the question a post-mortem actually asks:
     *    **did the operator see this?** `forwarded=false, rate_limited=false` means this level is
     *    never sent to a ground station by design; `rate_limited=true` means it was, and the bound
     *    suppressed it. The two are entirely different conversations and the record must not
     *    conflate them.
     *
     * [text] is the exact sentence that went (or would have gone) on the wire, so the log and the
     * operator's screen can be read against each other verbatim.
     */
    class Warn(
        override val monoNanos: Long,
        /**
         * Which DJI subsystem said it — `WarnSource.label`: `health`, `wind`, and whatever comes
         * next. **The field that made this entry general**, and the one a reader filters on.
         */
        val source: String,
        /** `appeared` | `changed` | `cleared`, lowercase — `WarnChange.name` lowercased. */
        val change: String,
        val code: String,
        /**
         * **DJI's own state word, verbatim** — `WARNING`, `LEVEL_2`. Kept next to [level], which is
         * our translation of it onto the one severity ladder, because they answer different
         * questions: `state` is what DJI said and what a DJI forum post will call it; `level` is
         * what we decided it costs. A record with only one of them cannot be audited against the
         * other.
         */
        val state: String,
        val level: String,
        val previousLevel: String?,
        val title: String?,
        val description: String?,
        val componentId: Int?,
        val sensorIndex: Int?,
        /**
         * The number that belongs with this warning, formatted with its unit — `14.2 m/s` — or
         * absent when the source has none. landing14's lesson: a level without its measurement is
         * the report that failed then.
         */
        val measurement: String?,
        val severity: String,
        val forwarded: Boolean,
        val rateLimited: Boolean,
        val text: String,
    ) : LogEntry() {
        override val kind: String get() = KIND_DJI_WARN

        /** Discrete, rare, and the reason a session is being read at all. Never batched. */
        override val urgent: Boolean get() = true

        override fun writeBody(o: JsonObject) {
            o.put("src", source)
            o.put("chg", change)
            o.put("code", code)
            o.put("state", state)
            o.put("lvl", level)
            o.put("prev", previousLevel)
            o.put("title", title)
            o.put("desc", description)
            o.put("comp", componentId)
            o.put("sensor", sensorIndex)
            o.put("meas", measurement)
            o.put("sev", severity)
            o.put("fwd", forwarded)
            o.put("rate_limited", rateLimited)
            o.put("msg", text)
        }
    }

    /**
     * **Where the camera was pointing** — the one Zenoh telemetry channel a flight record could
     * not reproduce at all until this entry existed (`replay/ReplayCoverage.GIMBAL`).
     *
     * ## Why one entry rather than three `dji_field` lines
     *
     * `ReplayCoverage` offered both. Three fields would be smaller to add and worse to read, for
     * three reasons:
     *
     *  - The `gimbal` channel on the Zenoh bus is one `geometry_msgs.Vector3` carrying all three
     *    axes together. Three independent on-change lines would have to be re-joined by a replayer
     *    with a last-seen merge, and a merge across lines is exactly where "the roll we replayed
     *    was from four seconds ago" gets in.
     *  - [Field] is `urgent`, i.e. an `fsync` per line. That is right for a mode change and wrong
     *    for something that can move at 5 Hz. This entry is **not** urgent.
     *  - Three deadbanded fields that move together are three independent chances to emit; one
     *    entry with one deadband emits once.
     *
     * ## `age` — the same discipline [DjiState] uses, for the same trap
     *
     * `Gimbal.KeyGimbalAttitude` is **change-driven**: it goes silent when the camera is still.
     * This project has been caught by that shape three times, and a gimbal trace is the worst case
     * of it, because "the camera held a steady angle through the orbit" and "the attitude feed
     * died at the top of the climb" produce an identical run of identical numbers. [ageMs] is
     * milliseconds since DJI last delivered the reading, so the two are separable **on the same
     * line as the value they describe**. Absent means never delivered; the format's rule
     * everywhere is that absent is not zero.
     *
     * Pitch is the only axis this airframe means anything by — the Mini 4 Pro's gimbal has no
     * usable yaw or roll — but roll and yaw are carried *as reported* rather than written to zero,
     * for the reason `ZenohTelemetryEncoder.gimbalAttitudeOrNull` gives: 0° is a real angle, and a
     * future airframe with a real yaw axis must not be silently flattened.
     *
     * Degrees throughout, matching both `gimbal_cmd` and `GimbalReading`.
     */
    class Gimbal(
        override val monoNanos: Long,
        val pitchDeg: Double?,
        val rollDeg: Double?,
        val yawDeg: Double?,
        /** Milliseconds since DJI last delivered the attitude, or null if it never has. */
        val ageMs: Long?,
    ) : LogEntry() {
        override val kind: String get() = KIND_GIMBAL

        override fun writeBody(o: JsonObject) {
            // 2 dp ≈ 0.01°, two orders finer than the 0.5° deadband that decides whether this
            // line exists at all, and finer than anything DJI reports.
            o.put("p", pitchDeg, 2)
            o.put("r", rollDeg, 2)
            o.put("y", yawDeg, 2)
            o.put("age", ageMs)
        }
    }

    /**
     * **One encoded video frame's index line** — where its bytes are, and when they arrived.
     *
     * The bytes themselves are **not** here. 5 Mbit/s of H.264 is two orders of magnitude larger
     * than everything else this record holds, so the frames go to a sidecar beside the JSONL and
     * this line says which one and where — `docs/apriltag-landing-recording.md` §2.
     *
     * ## Why an index and not a container
     *
     * [monoNanos] is stamped **first thing in the stream callback**, from the same
     * `SystemClock.elapsedRealtimeNanos()` every other entry uses. That single sentence is the whole
     * timestamp story: the frame's time, the gimbal's time and the altitude's time are one clock by
     * construction, with no join for an offline tool to get wrong. An MP4 would have put the frame
     * times on a second clock and lost the lot to a missing `moov` atom if the process died.
     *
     * A line whose [part] has since been rotated away is **not an error**. Knowing a frame existed
     * and when it arrived is evidence even when its bytes are gone, and that is exactly the
     * difference between an index and a container.
     *
     * **This entry does not prove the frame was decodable.** It proves it arrived, how big it was,
     * and where it was written. What is in those bytes is the decoder's business.
     */
    class Frame(
        override val monoNanos: Long,
        /** Ordinal within the session, from 1. Monotonic — a gap is a drop, and a drop is evidence. */
        val n: Long,
        /** Which sidecar part holds the bytes. Rotation makes this necessary rather than decorative. */
        val part: Int,
        /** Byte offset into that part. */
        val offset: Long,
        /** Bytes written for this frame, including any re-injected parameter sets. */
        val length: Int,
        /** True only on a keyframe; omitted otherwise, per the format's null rule. */
        val keyframe: Boolean,
        /**
         * Parameter sets (SPS/PPS) were re-injected ahead of this frame, so it is a seek point.
         * `RtpVideoSink` does the same for a late-joining receiver, and an offline tool seeking to
         * an arbitrary keyframe *is* a late joiner — without it a record is only decodable from its
         * first frame, which defeats the point of having an index at all.
         */
        val parameterSets: Boolean,
        /**
         * DJI's own `StreamInfo.presentationTimeMs`, verbatim, whatever clock it turns out to be
         * on. Recorded rather than interpreted: it is read today and used nowhere, and it may be
         * the cheapest answer there is to the glass-to-phone latency question
         * (`docs/apriltag-landing-recording.md` §3.2). Null when DJI does not supply it.
         */
        val presentationMs: Long?,
        /**
         * Frame dimensions and codec, **only when they change**, plus once at the start. A
         * resolution change invalidates the camera intrinsics every downstream pose solution rests
         * on, so it has to be a discrete timestamped line rather than something a reader infers by
         * diffing two frames it happened to look at.
         */
        val width: Int?,
        val height: Int?,
        val mime: String?,
    ) : LogEntry() {
        override val kind: String get() = KIND_FRAME

        override fun writeBody(o: JsonObject) {
            o.put("n", n)
            o.put("part", part)
            o.put("off", offset)
            o.put("len", length)
            if (keyframe) o.put("key", true)
            if (parameterSets) o.put("psi", true)
            o.put("pts", presentationMs)
            o.put("w", width)
            o.put("h", height)
            o.put("mime", mime)
        }
    }

    /**
     * **One AprilTag the on-board detector saw**, in pixels, in the camera's frame, and — when it
     * could be worked out — in the world.
     *
     * ## Why the record is the primary evidence and not a copy of it
     *
     * A tag sighting exists for about a tenth of a second. Nothing on the aircraft keeps it, the
     * Zenoh channel that will eventually carry it does not exist yet, and the flight recorder is
     * running on every flight this project makes. So this line **is** the measurement: the detection
     * rate against height, the false-id rate, whether the latch fired at the right moment and
     * whether the position it latched was anywhere near the pad are all questions answered from
     * these lines and nothing else. That is why it does not depend on a transport being up, and why
     * it is written before anything else is told.
     *
     * ## Why every uncertainty field is on the line rather than in a document
     *
     * [metric] and [bearingAssumed] travel with the number they qualify. A reader six months from
     * now, joining this against altitude to work out whether a landing would have worked, must not
     * have to know that on this date the focal length was a two-flight fit and the camera-to-body
     * rotation had never been measured. `docs/apriltag-landing-recording.md` §6.2 has the argument;
     * these two booleans are what make the argument travel with the data.
     *
     * ## Volume
     *
     * Capped by the detector's 10 Hz ceiling and by there being at most one line per *detected*
     * frame — so at worst 10 lines a second, and only while armed. About 150 B a line, so 1.5 kB/s
     * during an approach against the video sidecar's 649 kB/s. It is not a stream; it is the reason
     * the stream was recorded.
     *
     * **Not urgent.** It can arrive at 10 Hz, and an `fsync` per line on a 10 Hz stream is exactly
     * what [Gimbal] declines for the same reason.
     */
    class Tag(
        override val monoNanos: Long,
        val tagId: Int,
        /** Tag centre in the frame, pixels, x right and y **down**. */
        val centreX: Double,
        val centreY: Double,
        /** The longest of the tag's four edges, pixels. The trust proxy and the rate curve's x-axis. */
        val pixelSize: Double,
        /** Frame geometry, so a bearing can be re-derived under a real calibration. */
        val width: Int,
        val height: Int,
        /** Bit errors corrected to reach [tagId]. The per-detection half of the `maxhamming` question. */
        val hamming: Int,
        /** The detector's own confidence. Recorded, never thresholded. */
        val decisionMargin: Double,
        /** Camera frame, metres: x right, y down, z along the optical axis. */
        val x: Double,
        val y: Double,
        val z: Double,
        /** **False while the intrinsics are assumed rather than calibrated.** Today: always false. */
        val metric: Boolean,
        /** Metres north/east of the takeoff datum, or null when the fix could not be made. */
        val northM: Double?,
        val eastM: Double?,
        /** The height the fix was made from. Null exactly when [northM] is. */
        val fromHeightM: Double?,
        /** **True while the camera-to-body rotation is assumed.** See `vision/TagWorld`. */
        val bearingAssumed: Boolean,
        /** True on the one line where this tag became the flight's latched tag. */
        val latched: Boolean,
        /**
         * **True when the fix's camera pitch was the *reported* gimbal attitude** rather than an
         * angle this bridge commanded — `vision/TagFix.pitchReported`, travelling with the number
         * it qualifies exactly as [bearingAssumed] does, and additive exactly as the corners are:
         * an older reader sees a valid line, an older *line* reads as "commanded", which is what
         * every line before 2026-07-28 was.
         */
        val pitchReported: Boolean = false,
        /**
         * The four corners in pixels, since 2026-07-28, all-or-nothing: either all eight are
         * present or none is. **Additive** — a reader that predates them sees a valid line, and
         * a line that predates them reads as "corners not measured", which is exactly true.
         * Valuable independently of the solve: `tools/tagcorners` currently re-detects these
         * offline from video, at the cost of pulling 67 MB sidecars.
         */
        val c0x: Double? = null,
        val c0y: Double? = null,
        val c1x: Double? = null,
        val c1y: Double? = null,
        val c2x: Double? = null,
        val c2y: Double? = null,
        val c3x: Double? = null,
        val c3y: Double? = null,
        /**
         * apriltag's pose solve, since 2026-07-28, **raw and ungated** — see
         * `vision/TagPoseSolve` for frames and meaning, and `vision/TagPose.trusted` for the
         * gates that decide belief at publication. Recorded ungated so the gates stay
         * measurable from flight data. All-or-nothing with [e1]; [e2] may legitimately be
         * `+Infinity` (no second PnP minimum — unambiguous), which the format writes as its
         * documented `"Infinity"` string.
         */
        val qx: Double? = null,
        val qy: Double? = null,
        val qz: Double? = null,
        val qw: Double? = null,
        val tx: Double? = null,
        val ty: Double? = null,
        val tz: Double? = null,
        val e1: Double? = null,
        val e2: Double? = null,
        /** The tag size the solve scaled by, metres. Present exactly when the solve is. */
        val tagM: Double? = null,
        /**
         * **True when the fix on this line is metric** — its lateral built from a trusted
         * solved pose (`vision/TagFix.metric`), not from a pixel ray scaled by the barometer.
         * Additive like [pitchReported]: an older reader sees a valid line, an older *line*
         * reads as a bearing fix, which every line before this field was. Deliberately a
         * separate key from [metric], which describes the *sighting's* camera-frame pose and
         * whose always-written false is a published convention this field must not disturb.
         * The next flight's record should show this flipping true below ~1.8 m (the 60 px
         * gate on the shipped 75 mm marker) and false above — that switch is the evidence
         * the metric path flew.
         */
        val fixMetric: Boolean = false,
        /**
         * The solved range along the optical axis, metres — `vision/TagFix.rangeM`. Present
         * exactly when [fixMetric]; the one range on this line with no barometer in it, which
         * is what makes the `range_m`-versus-`from_h` disagreement in a landing's last metre
         * a measurement rather than a mystery (landing06: up to 0.46 m).
         */
        val rangeM: Double? = null,
        /**
         * **Which rung of the range ladder scaled this fix** — `vision/RangeSource`'s name,
         * lowercase: `solve`, `size`, or `baro`. Present exactly when the fix is; additive like
         * [pitchReported], and added 2026-07-29 under the record-first rule when the fix reached
         * the Zenoh bus (`tag_fix`): the bus carries the rung as provenance, the bus may carry
         * nothing the record cannot reproduce, and until this field the record distinguished
         * only solve ([fixMetric]) from not-solve — a `size`-scaled and a `baro`-scaled bearing
         * fix, which landing07 measured disagreeing by ~1.2 m, wrote identical lines. An older
         * line reads as "rung unrecorded", which is exactly true, and the bus omits the token
         * rather than guessing.
         */
        val rangeSource: String? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_TAG

        override fun writeBody(o: JsonObject) {
            o.put("id", tagId)
            // 1 dp on pixels: the detector reports sub-pixel corners and the quad fit is genuinely
            // that precise, but a tenth of a pixel is already below anything the rate curve bins by.
            o.put("cx", centreX, 1)
            o.put("cy", centreY, 1)
            o.put("px", pixelSize, 1)
            o.put("w", width)
            o.put("h", height)
            o.put("ham", hamming)
            o.put("margin", decisionMargin, 1)
            // 3 dp ≈ 1 mm, well inside a pose whose focal length is a 1.2 % fit.
            o.put("x", x, 3)
            o.put("y", y, 3)
            o.put("z", z, 3)
            // Written only when true, per the format's rule that absent is the common case. `metric`
            // is the exception and is always written: a reader must never have to decide whether an
            // absent `metric` meant false or meant an older writer that did not know about it.
            o.put("metric", metric)
            o.put("n", northM, 3)
            o.put("e", eastM, 3)
            o.put("from_h", fromHeightM, 2)
            if (bearingAssumed) o.put("bearing_assumed", true)
            if (latched) o.put("latched", true)
            if (pitchReported) o.put("pitch_reported", true)
            // 2 dp on corners, not the 1 dp of cx/cy: corners feed offline geometry (homography,
            // pose re-derivation) where a hundredth of a pixel is cheap insurance, and eight
            // values per line at one extra digit is bytes we can afford.
            o.put("c0x", c0x, 2)
            o.put("c0y", c0y, 2)
            o.put("c1x", c1x, 2)
            o.put("c1y", c1y, 2)
            o.put("c2x", c2x, 2)
            o.put("c2y", c2y, 2)
            o.put("c3x", c3x, 2)
            o.put("c3y", c3y, 2)
            // 6 dp on the quaternion — ~a microradian, far below the solve's own error, and the
            // rounded value is what both offline reproducers encode, so the two cannot drift.
            o.put("qx", qx, 6)
            o.put("qy", qy, 6)
            o.put("qz", qz, 6)
            o.put("qw", qw, 6)
            o.put("tx", tx, 3)
            o.put("ty", ty, 3)
            o.put("tz", tz, 3)
            // 12 dp on the object-space errors: they are tiny (median ~1e-5 on the measured
            // flight) and the *ratio* e1/e2 is a published gate, so fixed 3-dp rounding would
            // destroy exactly the quantity the record exists to preserve.
            o.put("e1", e1, 12)
            o.put("e2", e2, 12)
            o.put("tag_m", tagM, 3)
            // Written only when true / present, the format's rule for the additive fields.
            if (fixMetric) o.put("fix_metric", true)
            o.put("range_m", rangeM, 3)
            o.put("range_src", rangeSource)
        }
    }

    /**
     * **One discrete thing we asked the aircraft to do, or DJI's answer to one** — see [DjiCalls]
     * for the argument and [Tap.aircraftOut] for the contract.
     *
     * Two lines per action, joined by [sequence]: the ask, written before the SDK is touched, and
     * the answer, written whenever DJI gets round to it — or a [DjiPhase.NONE] when DJI never
     * does, which is measured behaviour on this airframe rather than a defensive possibility.
     *
     * Deliberately **not** merged into one line at answer time. A single line would be smaller and
     * would lose the two facts that matter most: that the ask happened at all when the process
     * dies before the answer, and the ordering of an ask against the `dji_field` and `stick_cmd`
     * lines around it. `sequence` is cheap; a lost ask is not.
     *
     * [urgent] is per-entry rather than per-kind because this kind spans two cadences. A takeoff
     * is one line a session and belongs on the flash immediately; a gimbal aim during an orbit can
     * arrive at 5 Hz and must not put an `fsync` on that stream. **A refusal is urgent either
     * way** — `DjiCalls.urgentFor`.
     */
    class DjiCall(
        override val monoNanos: Long,
        /** Joins an ask to its answer. Monotonic within a session, from `DjiCalls`. */
        val sequence: Long,
        /** One of [DjiOp]. */
        val op: String,
        /** One of [DjiPhase]. */
        val phase: String,
        /** A complete pre-rendered JSON object describing what was asked. Ask lines only. */
        val argsJson: String? = null,
        /** DJI's `IDJIError` name verbatim, or a synchronous call's throwable message. */
        val error: String? = null,
        /** Milliseconds from the ask to this answer. Answer lines only. */
        val elapsedMs: Long? = null,
        /** True when this `none` was forced by the outstanding-call cap rather than by the timeout. */
        val overflow: Boolean = false,
        override val urgent: Boolean = true,
    ) : LogEntry() {
        override val kind: String get() = KIND_DJI_CALL

        override fun writeBody(o: JsonObject) {
            o.put("seq", sequence)
            o.put("op", op)
            o.put("phase", phase)
            o.putRaw("args", argsJson)
            o.put("err", error)
            o.put("ms", elapsedMs)
            // Omitted when false — the common case, and the format's rule is that absent is the
            // ordinary state of affairs.
            o.put("overflow", if (overflow) true else null)
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Control — the entries the "it flew the wrong way" question turns on
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One virtual-stick command, carrying **four things that must never be
     * separated**:
     *
     *  - [source] — which inbound MAVLink message we believe asked for it,
     *  - [setpoint] — what our controller decided, in our own frame and units,
     *  - [axes] — the exact numbers handed to the DJI SDK,
     *  - [modes] — the control modes in force **for this command**.
     *
     * The last is the one that is easy to get wrong and impossible to recover
     * afterwards. A DJI stick value is meaningless without its control mode: with
     * `RollPitchControlMode.VELOCITY` a roll of `2.0` is 2 m/s sideways, with
     * `ANGLE` it is a 2° bank, and with `VerticalControlMode.POSITION` a throttle
     * of `2.0` is an altitude target, not a climb rate. Logging modes once at
     * configuration time is not enough — if anything ever re-sets them (a
     * reconnect, a second code path, an SDK default after authority takeback) the
     * log would still claim the mode we configured hours earlier. So the modes ride
     * on every command.
     */
    class StickCmd(
        override val monoNanos: Long,
        val sequence: Long,
        val setpoint: Setpoint?,
        val axes: StickAxes,
        val modes: StickModes,
        val source: CommandSource? = null,
        val range: StickRange? = null,
        /** Which SDK surface was used — see [StickPath]. */
        val path: String,
        /** Did the SDK accept it? `false` with [error] set is a refusal. */
        val accepted: Boolean? = null,
        val error: String? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_STICK_CMD

        override fun writeBody(o: JsonObject) {
            o.put("seq", sequence)
            source?.let { s -> o.obj("src") { it.put("name", s.messageName); it.put("seq", s.sequence); it.put("t", s.monoNanos) } }
            setpoint?.let { sp ->
                o.obj("sp") { w ->
                    w.put("frame", sp.frame)
                    // The velocity and yaw-rate precisions are named constants because they
                    // are contract, not merely disk cost — see Setpoint.VELOCITY_DECIMALS.
                    w.put("vn", sp.north, Setpoint.VELOCITY_DECIMALS)
                    w.put("ve", sp.east, Setpoint.VELOCITY_DECIMALS)
                    w.put("vd", sp.down, Setpoint.VELOCITY_DECIMALS)
                    w.put("yaw", sp.yawDeg, 2)
                    w.put("yawrate", sp.yawRateDegPerS, Setpoint.YAW_RATE_DECIMALS)
                    w.put("alt", sp.altitudeM, Setpoint.VELOCITY_DECIMALS)
                    w.put("units", sp.units)
                }
            }
            o.obj("dji") { w ->
                w.put("pitch", axes.pitch, 4)
                w.put("roll", axes.roll, 4)
                w.put("yaw", axes.yaw, 4)
                w.put("throttle", axes.verticalThrottle, 4)
            }
            o.obj("modes") { w ->
                w.put("rp", modes.rollPitch)
                w.put("yaw", modes.yaw)
                w.put("vert", modes.vertical)
                w.put("coord", modes.coordinateSystem)
                w.put("adv", modes.advanced)
                w.put("code", modes.code())
            }
            range?.let { r ->
                o.obj("range") { w ->
                    w.put("rp_max", r.rollPitchMax, 3)
                    w.put("vert_max", r.verticalMax, 3)
                    w.put("yaw_max", r.yawMax, 3)
                }
            }
            o.put("path", path)
            o.put("accepted", accepted)
            o.put("err", error)
        }
    }

    /**
     * Where the human's own sticks were. Answers "were the RC sticks fighting us"
     * without inference, and it is the only thing that can: DJI mixes RC input with
     * virtual-stick input, so an aircraft moving on an axis we never commanded is
     * an entirely normal outcome of a nudged transmitter.
     *
     * Raw values are the SDK's `[-660, 660]`; the normalised `-1..+1` copy is there
     * so the number is comparable with a stick command without arithmetic.
     */
    class RcStick(
        override val monoNanos: Long,
        val leftHorizontal: Int?,
        val leftVertical: Int?,
        val rightHorizontal: Int?,
        val rightVertical: Int?,
    ) : LogEntry() {
        override val kind: String get() = KIND_RC_STICK

        override fun writeBody(o: JsonObject) {
            o.put("lh", leftHorizontal)
            o.put("lv", leftVertical)
            o.put("rh", rightHorizontal)
            o.put("rv", rightVertical)
            o.obj("n") { w ->
                w.put("lh", norm(leftHorizontal), 4)
                w.put("lv", norm(leftVertical), 4)
                w.put("rh", norm(rightHorizontal), 4)
                w.put("rv", norm(rightVertical), 4)
            }
        }

        companion object {
            /** Full deflection per `IStick`: `[-660, 660]`. */
            const val FULL_DEFLECTION = 660.0
            fun norm(raw: Int?): Double? = raw?.let { it / FULL_DEFLECTION }
        }
    }

    /**
     * Whether virtual stick was actually engaged, and who held flight-control
     * authority. This is what distinguishes "our command was wrong" from "our
     * command was never in force".
     *
     * [changeReason] is DJI's own explanation of an authority change and is the
     * single highest-value field in the whole log for case 7: `NEAR_BOUNDARY` is
     * the geofence takeback (virtual stick stops working within ~30 m of a limit —
     * djidoc: IVirtualStickManager), `RC_NOT_P_MODE` / `RC_SWITCH` /
     * `RC_PAUSE_STOP` / `RC_ONE_KEY_GO_HOME` are the pilot taking over, and
     * `BATTERY_LOW_GO_HOME` / `BATTERY_SUPER_LOW_LANDING` are the aircraft doing so.
     */
    class VsState(
        override val monoNanos: Long,
        val enabled: Boolean?,
        val advanced: Boolean?,
        val authority: String?,
        val changeReason: String? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_VS_STATE
        override val urgent: Boolean get() = true

        override fun writeBody(o: JsonObject) {
            o.put("on", enabled)
            o.put("adv", advanced)
            o.put("auth", authority)
            o.put("reason", changeReason)
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Narrative and recorder health
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * A discrete thing that happened, in words: virtual-stick engage/disengage and
     * its result, a mode change, a takeoff or land command, an SDK error, recorder
     * start and stop. [Field] records the raw change; this records the story, so a
     * human reading the log top to bottom gets a timeline without a tool.
     */
    class Event(
        override val monoNanos: Long,
        val code: String,
        val severity: String = SEV_INFO,
        val message: String? = null,
        val dataJson: String? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_EVENT
        override val urgent: Boolean get() = true

        override fun writeBody(o: JsonObject) {
            o.put("code", code)
            o.put("sev", severity)
            o.put("msg", message)
            o.putRaw("d", dataJson)
        }
    }

    /**
     * Entries the recorder lost, **timestamped where they were lost**.
     *
     * A counter in the header would say "12 lost" and leave open the one question
     * that matters, which is whether they were lost during the interesting second.
     * This says "12 lost between t=83.412 and t=83.605", so `tools/flightlog` can
     * mark the gap in the middle of a diagnosis table rather than reporting a
     * clean-looking sequence that is quietly missing its middle.
     *
     * Written directly by the writer thread and never enqueued, because a drop
     * record that can itself be dropped is worthless.
     */
    class Drop(
        override val monoNanos: Long,
        val count: Long,
        val firstMonoNanos: Long,
        val lastMonoNanos: Long,
        val total: Long,
        val kindCountsJson: String? = null,
    ) : LogEntry() {
        override val kind: String get() = KIND_DROP
        override val urgent: Boolean get() = true

        override fun writeBody(o: JsonObject) {
            o.put("n", count)
            o.put("from", firstMonoNanos)
            o.put("to", lastMonoNanos)
            o.put("total", total)
            o.putRaw("kinds", kindCountsJson)
        }
    }

    /** Periodic recorder self-report, so the log states its own health. */
    class Stats(
        override val monoNanos: Long,
        val written: Long,
        val dropped: Long,
        val queued: Int,
        val peakQueued: Int,
        val bytes: Long,
        val flushes: Long,
        val syncs: Long,
        val rotations: Int,
        val part: Int,
    ) : LogEntry() {
        override val kind: String get() = KIND_STATS

        override fun writeBody(o: JsonObject) {
            o.put("written", written)
            o.put("dropped", dropped)
            o.put("queued", queued)
            o.put("peak_queued", peakQueued)
            o.put("bytes", bytes)
            o.put("flushes", flushes)
            o.put("syncs", syncs)
            o.put("rotations", rotations)
            o.put("part", part)
        }
    }

    companion object {
        const val KIND_HEADER = "header"
        const val KIND_MAV_IN = "mav_in"
        const val KIND_MAV_OUT = "mav_out"
        const val KIND_DJI_STATE = "dji_state"
        const val KIND_DJI_FIELD = "dji_field"

        /** [Warn] — one DJI warning changing state, from any source. `Channel.AIRCRAFT_WARN`. */
        const val KIND_DJI_WARN = "dji_warn"

        /**
         * **Retired 2026-07-30, readable forever.** [Warn]'s kind before the warning path became
         * one owner for every source; records up to and including landing17 carry it, and every
         * reader (`replay/FlightRecordReader`, `tools/memexport`, `tools/flightlog`) still accepts
         * it. **Nothing writes it.** Kept as a constant rather than a string in three tools, and
         * kept at all because deleting it would silently stop old sessions parsing.
         */
        const val KIND_DJI_HEALTH = "dji_health"

        /** [Gimbal] — where the camera is pointing. `Channel.AIRCRAFT_GIMBAL`. */
        const val KIND_GIMBAL = "gimbal"

        /** [DjiCall] — one ask to the aircraft, or DJI's answer. `Channel.AIRCRAFT_ACTION`. */
        const val KIND_DJI_CALL = "dji_call"

        /** [Frame] — where one encoded video frame's bytes are. `Channel.AIRCRAFT_VIDEO_FRAME`. */
        const val KIND_FRAME = "frame"

        /** [Tag] — one AprilTag the on-board detector saw. `Channel.AIRCRAFT_TAG`. */
        const val KIND_TAG = "tag"

        const val KIND_STICK_CMD = "stick_cmd"
        const val KIND_RC_STICK = "rc_stick"
        const val KIND_VS_STATE = "vs_state"
        const val KIND_EVENT = "event"
        const val KIND_DROP = "drop"
        const val KIND_STATS = "stats"

        const val SEV_INFO = "info"
        const val SEV_WARN = "warn"
        const val SEV_ERROR = "error"
    }
}

/** Which inbound MAVLink message our controller believes it is obeying. */
data class CommandSource(
    val messageName: String,
    val sequence: Int?,
    /** The `monoNanos` of the `mav_in` entry, so the two lines join exactly. */
    val monoNanos: Long?,
)

/**
 * What the guided controller decided, expressed in **our** frame and units rather
 * than DJI's — so the log can show our intent next to DJI's numbers and let the
 * mapping between them be checked instead of assumed.
 *
 * `null` for an axis means "this setpoint says nothing about that axis", which is
 * different from zero.
 */
data class Setpoint(
    /** e.g. `NED_VELOCITY`, `BODY_VELOCITY`, `NED_POSITION` — see [SetpointFrame]. */
    val frame: String,
    val north: Double? = null,
    val east: Double? = null,
    /** Positive **down**, matching MAVLink and DJI's NED velocity. */
    val down: Double? = null,
    val yawDeg: Double? = null,
    val yawRateDegPerS: Double? = null,
    val altitudeM: Double? = null,
    val units: String = "m/s;deg",
) {
    companion object {
        /**
         * The record's precision for a setpoint velocity, in decimal digits — what
         * [LogEntry.StickCmd.writeBody] hands `Json.num` for `vn`/`ve`/`vd`.
         *
         * A named single owner rather than a literal in the writer, because since 2026-07-29
         * the precision is part of a *published contract* and not only a disk-cost choice:
         * the Zenoh `setpoint` channel's rule is that the wire may carry nothing the record
         * cannot reproduce, so `ZenohTelemetryEncoder.setpointOrNull` quantises through
         * `Json.roundTo` at exactly this precision before encoding. Two literals drifting
         * apart would make the live bus and a record-derived mem2 store disagree in the last
         * digit — silently, on every message.
         */
        const val VELOCITY_DECIMALS = 3

        /** The record's precision for the yaw-rate half, deg/s — same ownership argument. */
        const val YAW_RATE_DECIMALS = 2
    }
}

/** Frame names a [Setpoint] may declare. Strings on the wire, so a new one is additive. */
object SetpointFrame {
    /** North/east/down velocity in m/s, earth frame. */
    const val NED_VELOCITY = "NED_VELOCITY"
    /** Forward/right/down velocity in m/s, body frame (nose-relative). */
    const val BODY_VELOCITY = "BODY_VELOCITY"
    /** North/east offsets in metres plus an altitude. */
    const val NED_POSITION = "NED_POSITION"
}

/**
 * The four numbers handed to DJI, named as DJI names them.
 *
 * Note DJI's naming trap, which is exactly the trap the scenario describes:
 * `roll` moves the aircraft **left/right** and `pitch` moves it
 * **forward/backward**, and in `VELOCITY` mode these are velocities, not angles.
 * A controller that thinks in north/east must decide which of the two carries
 * which — and get the coordinate system right, because
 * `FlightCoordinateSystem.GROUND` makes roll/pitch earth-referenced while `BODY`
 * makes them nose-referenced. That decision is what [StickCmd] exists to record.
 */
data class StickAxes(
    val pitch: Double?,
    val roll: Double?,
    val yaw: Double?,
    val verticalThrottle: Double?,
)

/**
 * The control modes in force for one command. Names match the DJI enums
 * (`RollPitchControlMode`, `YawControlMode`, `VerticalControlMode`,
 * `FlightCoordinateSystem`) so a log line can be read against DJI's own docs.
 */
data class StickModes(
    /** `ANGLE` | `VELOCITY` | `POSITION` | `UNKNOWN` */
    val rollPitch: String?,
    /** `ANGLE` | `ANGULAR_VELOCITY` | `UNKNOWN` */
    val yaw: String?,
    /** `VELOCITY` | `POSITION` | `UNKNOWN` */
    val vertical: String?,
    /** `GROUND` | `BODY` | `UNKNOWN` */
    val coordinateSystem: String?,
    /**
     * Advanced mode changes what the numbers mean entirely: without it only
     * `IStick` positions in `[-660, 660]` are available and the control modes do
     * not apply at all.
     */
    val advanced: Boolean?,
) {
    /**
     * A single plottable integer, so the modes survive the trip through
     * `NAMED_VALUE_FLOAT` into QGC's `.tlog` where only floats fit.
     *
     * `rp*1000 + yaw*100 + vert*10 + coord`, each digit the DJI enum's own
     * integer value (recovered by javap from each enum's `static {}` block),
     * with `9` standing in for unknown or absent:
     *
     * | digit | 0 | 1 | 2 |
     * |---|---|---|---|
     * | rp   | ANGLE | VELOCITY | POSITION |
     * | yaw  | ANGLE | ANGULAR_VELOCITY | — |
     * | vert | VELOCITY | POSITION | — |
     * | coord| GROUND | BODY | — |
     */
    fun code(): Int =
        digit(rollPitch, RP_ORDER) * 1000 +
            digit(yaw, YAW_ORDER) * 100 +
            digit(vertical, VERT_ORDER) * 10 +
            digit(coordinateSystem, COORD_ORDER)

    private fun digit(v: String?, order: List<String>): Int {
        val i = order.indexOf(v)
        return if (i < 0) 9 else i
    }

    companion object {
        val RP_ORDER = listOf("ANGLE", "VELOCITY", "POSITION")
        val YAW_ORDER = listOf("ANGLE", "ANGULAR_VELOCITY")
        val VERT_ORDER = listOf("VELOCITY", "POSITION")
        val COORD_ORDER = listOf("GROUND", "BODY")

        /** Modes are unknown — e.g. non-advanced mode, where they do not apply. */
        val UNKNOWN = StickModes(null, null, null, null, null)
    }
}

/**
 * The stick ranges in force, if `VirtualStickRange` was read. A command of `10.0`
 * means nothing without knowing whether the maximum is 10 or 15 — a saturated
 * command and a satisfied one look identical in the number alone.
 */
data class StickRange(
    val rollPitchMax: Double?,
    val verticalMax: Double?,
    val yawMax: Double?,
)

/** Which SDK surface a [StickCmd] went through. */
object StickPath {
    /** `KeyVirtualStickAdvancedParam` / advanced-mode param struct: real units. */
    const val ADVANCED_PARAM = "advanced_param"
    /** `IVirtualStickManager.getLeftStick()/getRightStick()`: `[-660, 660]` positions. */
    const val STICK_POSITION = "stick_position"
}

/** Event codes, so `tools/flightlog` can key on them rather than on prose. */
object EventCode {
    const val RECORDER_START = "recorder_start"
    const val RECORDER_STOP = "recorder_stop"
    const val ROTATE = "rotate"

    /**
     * The session's video budget is spent and frames have stopped being written.
     *
     * **Said out loud, at warning severity, because silence here is indistinguishable from an
     * aircraft that stopped delivering video** — and those two want opposite diagnoses. See
     * `VideoSidecar`.
     */
    const val VIDEO_BUDGET_SPENT = "video_budget_spent"

    /**
     * The operator turned video recording on or off during a session.
     *
     * In the record because a gap in the `frame` index otherwise has two explanations — the
     * operator's hand, or the aircraft going quiet — and they want opposite responses.
     */
    const val VIDEO_RECORDING = "video_recording"

    /**
     * **The first frame the aircraft actually delivered this session.**
     *
     * `video_phase` reaching `SERVING` says only that MSDK was asked to start. This says something
     * arrived, and is what separates "the camera never delivered" from "it delivered and nobody
     * recorded it" — the two explanations for an empty frame index.
     */
    const val VIDEO_FIRST_FRAME = "video_first_frame"

    /** Frames were arriving and have stopped for `VideoSidecar.STALL_MS`. */
    const val VIDEO_STALLED = "video_stalled"

    /** Frames are arriving again after a stall. */
    const val VIDEO_RESUMED = "video_resumed"

    // ── the Zenoh bus ────────────────────────────────────────────────────────
    //
    // The second transport's four lines. They are in the flight record rather than only in
    // logcat for the reason every other transport event is: logcat is not on the phone after the
    // flight, and "the bus was silent" and "the bus was never up" are the two explanations for a
    // subscriber that saw nothing.

    /** The publisher was started: where it is dialling, under what prefix, at what rate. */
    const val ZENOH_START = "zenoh_start"

    /**
     * The session's phase moved — connecting, publishing, or back to connecting after a failure.
     *
     * `SEV_WARN` for everything except `PUBLISHING`, and deliberately: a publisher that cannot
     * reach the router while the operator asked for one is somebody staring at a screen with
     * nothing on it, which is the same severity a video failure gets and for the same reason.
     */
    const val ZENOH_PHASE = "zenoh_phase"

    /**
     * The publisher was stopped, with the session's whole tally: published, dropped, discarded,
     * failures, opens and the peak queue depth.
     *
     * **`dropped` and `discarded` are separate on purpose.** Dropped means the queue was full —
     * the bus could not keep up with us. Discarded means there was no session — there was no bus.
     * One is a capacity problem and one is a network problem, and a single "lost" number would
     * send a post-mortem to the wrong one.
     */
    const val ZENOH_STOP = "zenoh_stop"

    /** A sampler iteration threw. Loud, and does not stop the sampler. */
    const val ZENOH_ERROR = "zenoh_error"

    // ── a published replay ───────────────────────────────────────────────────
    //
    // **The record is never written from a replayed state** — `Recorder.sample` reads the live
    // aircraft and does not consult `telemetry/StateSource` at all. These two lines are what
    // stands in for that absence: a window in which this phone described somebody else's
    // afternoon to a ground station and a bus, marked in the one file a post-mortem will have.
    //
    // Without them a published replay leaves a stretch of ordinary quiet `dji_state` lines and
    // no explanation, and the reader's honest conclusion — "nothing happened" — is wrong in the
    // way that matters: something was happening, on two networks, and it was not this aircraft.
    // `replay/ReplayPublication` carries the whole argument for recording the window rather than
    // the states.

    /** A recording was switched onto the MAVLink link, the Zenoh bus, or both. */
    const val REPLAY_PUBLISH_START = "replay_publish_start"

    /** …and is no longer being published, whether switched off or closed. */
    const val REPLAY_PUBLISH_STOP = "replay_publish_stop"

    const val VS_ENABLE_REQUEST = "vs_enable_request"
    const val VS_ENABLE_RESULT = "vs_enable_result"
    const val VS_DISABLE_REQUEST = "vs_disable_request"
    const val VS_DISABLE_RESULT = "vs_disable_result"
    const val VS_AUTHORITY_CHANGE = "vs_authority_change"
    const val MODE_CHANGE = "mode_change"
    const val TAKEOFF = "takeoff"
    const val LAND = "land"
    const val GO_HOME = "go_home"
    const val SDK_ERROR = "sdk_error"

    /**
     * DJI key subscriptions are live. Pairs with the `sdk_error` written when the
     * recorder starts before MSDK registration — which is every session, since
     * registration takes ~1 s. Without this the warning is never retracted and a
     * reader concludes the DJI half of the log is missing when it is present.
     */
    const val SDK_ATTACHED = "sdk_attached"

    /**
     * A continuously-delivered DJI key has gone quiet for longer than its own
     * limit — the reading below it in the log is a cached one of unknown validity.
     * Carries `d = {"sig": …, "age_ms": …, "limit_ms": …}` so a reader keys on the
     * data rather than on the prose.
     *
     * **Not an error by itself.** `KeyAircraftVelocity` is change-only on this
     * airframe, so a stationary aircraft produces this legitimately; what it says
     * is "you cannot tell a zero from a dead feed here", which is precisely the
     * thing a post-flight reader must know before believing a flat trace.
     */
    const val SIGNAL_STALE = "signal_stale"

    /** The same key is being delivered again. Pairs with [SIGNAL_STALE]. */
    const val SIGNAL_FRESH = "signal_fresh"

    const val SETPOINT_STALE = "setpoint_stale"
    const val LINK_UP = "link_up"
    const val LINK_DOWN = "link_down"

    /**
     * M3 Stage A's engagement lifecycle, from `guided/GuidedStickEngine`. `guided_engaged` is
     * written only when DJI's own `VirtualStickState` confirmed enabled + advanced + MSDK —
     * never on our request (that is `vs_enable_request`) — and carries the armed link-loss
     * policy's name, so a log always says which Q4 behaviour was in force.
     * `guided_released` carries the disengage reason (released / sticks / interlock /
     * authority / link-lost / idle / timeout / stopped) plus DJI's own word when there is one.
     */
    const val GUIDED_ENGAGED = "guided_engaged"
    const val GUIDED_RELEASED = "guided_released"

    /**
     * M3 Stage B's reposition lifecycle. `goto_accepted` is written when a `DO_REPOSITION`
     * passed every gate and the target was actually taken (the same instant the wire ack says
     * `ACCEPTED`), with the target and its distance in the message. `goto_denied` carries the
     * refusal reason the operator's modal names. `goto_arrived` is written only when both
     * arrival conjuncts held for the required consecutive ticks. `goto_ended` is a manoeuvre
     * ending *without* arrival while the engagement survives — paused, cancelled by GCS
     * sticks, or replaced by a newer target; engagement-ending exits stay `guided_released`.
     */
    const val GOTO_ACCEPTED = "goto_accepted"
    const val GOTO_DENIED = "goto_denied"
    const val GOTO_ARRIVED = "goto_arrived"
    const val GOTO_ENDED = "goto_ended"

    /**
     * The takeoff's **second phase**, which is a manoeuvre M2.5 starts and M3 flies
     * (`docs/m4-mission-execution.md` §3.6).
     *
     * `takeoff_climb_armed` is written when an accepted `MAV_CMD_NAV_TAKEOFF` left a climb
     * waiting behind DJI's own hop, with the height that will actually be flown to and whether
     * the ceiling capped it. `takeoff_climb_ended` is written for every one of the three ways it
     * stops being pending — `fired` (DJI reported the takeoff finished and the climb was handed
     * to the reposition path, which then writes its own `goto_accepted` or `goto_denied`),
     * `expired`, or a cancellation naming the abort rung. Exactly one `*_ended` follows every
     * `*_armed`, which is what makes "did an armed climb ever survive an abort?" answerable from
     * a flight log rather than from the source.
     */
    const val TAKEOFF_CLIMB_ARMED = "takeoff_climb_armed"
    const val TAKEOFF_CLIMB_ENDED = "takeoff_climb_ended"

    /**
     * **The phone's own Take off button** — the ask and its verdict on one line, written by
     * `Bridge.takeoffFromPhone`. It exists because this is the one takeoff origin with no
     * `mav_in` witness: a QGC takeoff leaves its `COMMAND_LONG` 22 in the record, while a phone
     * press would otherwise appear only as a `dji_call op=takeoff` from nowhere. The message
     * carries the requested height and the dispatcher's verdict, so a refused press is as
     * readable as an accepted one; everything downstream (`dji_call` pair, `takeoff_climb_*`,
     * the `gimbal_rotate` pair at handback) is recorded by the existing seams.
     */
    const val PHONE_TAKEOFF = "phone_takeoff"

    /**
     * M3 Stage C — one orbit's life, on the same terms as the `goto_*` family above.
     *
     * `orbit_accepted` is written only when every gate passed and the circle was actually taken,
     * with the centre, radius, signed direction and the required sweep in the message.
     * `orbit_denied` carries the refusal reason the operator's modal names. `orbit_circling` marks
     * the join leg arriving and the tangential ramp beginning — the transition the swept-angle
     * counter starts from. `orbit_ended` is a circle ending while the engagement survives:
     * completed, timed out, paused, cancelled by sticks, or replaced.
     */
    const val ORBIT_ACCEPTED = "orbit_accepted"
    const val ORBIT_DENIED = "orbit_denied"
    const val ORBIT_CIRCLING = "orbit_circling"
    const val ORBIT_ENDED = "orbit_ended"

    /**
     * M3 Stage D — one tag-tracked descent's life, on the same terms as the `goto_*` and
     * `orbit_*` families, plus one code they do not need.
     *
     * `tag_descent_armed` is written only when every arm gate passed and the descent was
     * actually taken, with the latched tag id, the arming height and the fix's age in the
     * message. `tag_descent_denied` carries the refusal reason the operator's sentence names.
     * `tag_descent_phase` is written on **every transition** of the staleness ladder and on the
     * terminal latch — tracking / holding / climbing / terminal, with the fix age at the
     * transition — because a post-flight analysis must be able to replay the whole engagement:
     * which rung the ladder was on, when, and why, comes from these lines and the `stick_cmd`
     * stream (`src = TAG_DESCENT`) between them. `tag_descent_ended` is written for every way a
     * descent stops being armed — completed-and-released, disarmed, cancelled by sticks, latch
     * lost, camera off nadir, tag gone past the abort bound, replaced by another manoeuvre, or
     * any abort rung — naming which. Exactly one `*_ended` follows every `*_armed`, the
     * `takeoff_climb_*` discipline, so "did a descent ever survive an abort?" is answerable
     * from a flight log rather than from the source.
     */
    const val TAG_DESCENT_ARMED = "tag_descent_armed"
    const val TAG_DESCENT_DENIED = "tag_descent_denied"
    const val TAG_DESCENT_PHASE = "tag_descent_phase"
    const val TAG_DESCENT_ENDED = "tag_descent_ended"

    /**
     * **A plan's `NAV_LAND` with "Precision Land" set — the mission's own half of a tag landing**,
     * written so that the whole sequence can be read from the record alone and so that the join to the
     * descent is explicit rather than inferred from two families happening to be adjacent.
     *
     * `land_tag_begun` — the gates passed at the moment the cursor reached the item, carrying every
     * number they were judged on: the recorded takeoff point, how far the aircraft and the drawn item
     * were from it, the current and the plan's takeoff heights, and the `param2` mode (so
     * Opportunistic and Required are distinguishable on the record even while they behave identically
     * — the difference is named here and nowhere else). One per sequence.
     *
     * `land_tag_phase` — every transition of `PrecisionLand.Phase` plus the arm: `transit`, `aiming`,
     * `lowering` (with the frozen arm height and the altitude it was clamped from — the never-climb
     * decision, on the record), `armed`. Also the ROI clear, when the sequence had one to clear.
     *
     * `land_tag_refused` — every way the sequence declines, with the reason word the operator's
     * `STATUSTEXT` carried: a failed gate, a camera that never reached believed nadir, or an arm the
     * descent's own door refused (whose own `tag_descent_denied` line sits beside this one and names
     * which gate). Exactly one of `land_tag_refused` or a `land_tag_phase armed` follows every
     * `land_tag_begun`, the `takeoff_climb_*` discipline, so "did a mission's landing ever silently
     * not happen?" is answerable from a flight log rather than from the source.
     */
    const val LAND_TAG_BEGUN = "land_tag_begun"
    const val LAND_TAG_PHASE = "land_tag_phase"
    const val LAND_TAG_REFUSED = "land_tag_refused"

    /**
     * **Stage C — the full autoland's own codes**, written so the measurement flight the stage
     * doubles as can be read from the record alone: when did DJI raise its confirmation
     * question during a virtual-stick descent (if ever), what did the bridge do about it and
     * why, did DJI force the camera, did the −90° re-command stick, and did the FC brake the
     * descent. The engagement lifecycle itself stays on the `tag_descent_*` family above —
     * `tag_descent_phase: landing …` is the commit, `tag_descent_ended: touchdown` the wheels.
     *
     * `landing_confirm_needed` — **both edges** of `FC.KeyIsLandingConfirmationNeeded`, message
     * `true`/`false`/`null`, written on every transition while the bridge is listening.
     * Whether this key fires at all during a sustained virtual-stick descent is a primary open
     * unknown (`landingdata.md` §2.2: it never appeared in four RC landings, but those
     * subscriptions were lazy) and an absence must be a measured absence.
     *
     * `landing_confirm` — what the bridge did about a `true`: `sent` (the one guarded
     * auto-confirm went to DJI), `refused: <reason>` (the guard said no — stale fix, fix
     * outside the cone, interlock off — and the decision was left to the operator), or
     * `failed: <DJI error verbatim>`. At most one `sent` per landing episode, the
     * `MsdkFlightActions` single-confirm discipline.
     *
     * `gimbal_watchdog` — every nadir re-command the landing's gimbal contingency asked for
     * (attempt number and the observed pitch that tripped it), plus its give-up line. The
     * `dji_call op=gimbal_rotate` pair beside it carries DJI's answer.
     *
     * `landing_stall` — the altitude stopped falling under a sustained down command
     * (`TagDescentGuidance.LAND_STALL_MS`). Originally pure measurement; **promoted to the
     * commit trigger by landing04** (the FC floors a virtual-stick descent at ~1.4 m and holds
     * indefinitely), so this line now marks the fact the commit decision stood on. Still never
     * a behaviour change to the descent command itself.
     *
     * `landing_commit` — the one-shot decision to hand the landing to DJI
     * (`KeyStartAutoLanding` through the `DjiLanding` seam): the height it fired at, the fix
     * age that vouched for it, and whether the trigger was the FC floor or the terminal hold.
     * The `dji_call op=land` pair beside it carries DJI's answer. Exactly one per engagement,
     * structurally (the committed phase has no exit inside the law).
     *
     * `landing_stop` — rule 1's (and the operator withdrawals') `KeyStopAutoLanding` verdict:
     * `asked (<cause>)` or `not asked: <reason>`. Whether DJI honours the stop is one of the
     * facts the next flight measures; the `dji_call op=stop_landing` pair is the other half.
     */
    const val LANDING_CONFIRM_NEEDED = "landing_confirm_needed"
    const val LANDING_CONFIRM = "landing_confirm"
    const val GIMBAL_WATCHDOG = "gimbal_watchdog"
    const val LANDING_STALL = "landing_stall"
    const val LANDING_COMMIT = "landing_commit"
    const val LANDING_STOP = "landing_stop"

    /**
     * **Historical: written by the retired OWN_LANDING experiment (landing11/landing12); no
     * current writer.** The experiment measured whether the FC's virtual-stick descent floor
     * could be lifted by disabling downward obstacle avoidance, and its records answered:
     * the per-direction DOWNWARD switch is UNSUPPORTED on the Mini 4 Pro (even the read
     * refuses — landing11 t=55.42) and with `ObstacleAvoidanceType=CLOSE` the floor persists
     * at 0.5 m — the un-disableable infrared ToF owns the last half-metre (landing12 t=66.2).
     * Question closed; the code was retired 2026-07-30 per Ivan (the verdict lives in
     * `docs/msdk/actions.md` §7 and CLAUDE.md's flight-measured facts).
     *
     * The constants stay because the record vocabulary is append-only by convention:
     * landing11/landing12's JSONL lines carry these codes, `tools/blackbox` reads them, and a
     * deleted name would turn a measured flight into an unreadable one.
     *
     * `own_landing_commit` — the commit fired with the experiment armed (`landing_commit`'s
     * message shape, destination our own descent). `perception_snapshot` — the captured
     * pre-arm values (`downward_oa=… oa_type=…`). `perception_arm` — the arm decisions
     * (`armed: B_TYPE_CLOSE`, refusals verbatim). `perception_restore` — the restore trail
     * (`begin`/`clean`/`ok`/`GAVE UP`). The DJI answers ride the `dji_call op=perception_*`
     * pairs beside these lines in those records.
     */
    const val OWN_LANDING_COMMIT = "own_landing_commit"
    const val PERCEPTION_SNAPSHOT = "perception_snapshot"
    const val PERCEPTION_ARM = "perception_arm"
    const val PERCEPTION_RESTORE = "perception_restore"

    /**
     * **Which instrument the tag descent's height law is flying on** — the range ladder's
     * verdict (`TagDescentGuidance.descentHeight`: trusted solve ≻ size-implied range ≻
     * barometer), written on every switch of source with both instruments' numbers, plus once
     * at the first tick so the record always says what the descent started on. Landing07's
     * line (`datasets/landing07/20260729-095413.001.jsonl`): the descent that missed by 46 cm
     * flew a barometer that had drifted ~1.2 m, and nothing in that record says so directly —
     * a reader had to reconstruct it from pixel sizes. Message shapes: `solve range=0.59
     * baro=0.80`, `size range=1.93 baro=0.70`, `baro height=5.00 tag=none`, `none` (warn —
     * no instrument could vouch).
     */
    const val HEIGHT_SOURCE = "height_source"

    /**
     * **The tag-derived range and the barometer disagree beyond the measured healthy bracket**
     * (`TagDescentGuidance.RANGE_DIVERGENCE_FACTOR`, ×1.8 — landing07 measured healthy 1.36
     * and the baro-lying failure at 2.76). **A measurement, never a gate**: the range ladder
     * has already chosen the tag (*"we should land if we see the tag"* — Ivan, 2026-07-29),
     * so this line exists purely so a post-flight read shows the barometer's lies against the
     * instrument the descent actually flew. One line per divergence episode, both numbers and
     * the ratio in the message, warn severity.
     */
    const val RANGE_BARO_DIVERGENCE = "range_baro_divergence"

    /**
     * DJI's firmware recentred the gimbal — the measured landing behaviour (`landingdata.md`:
     * on entering a landing mode the gimbal slews from wherever it is to level in ~0.35 s,
     * before the mode change itself is even delivered). Emitted by `record/GimbalRecenter`
     * when the reported pitch moves more than its threshold within its window while the
     * flight mode is a landing mode — decision-free, record-side only, so every future landing
     * documents the slew as an event instead of leaving it to be excavated from pitch samples.
     */
    const val GIMBAL_RECENTERED = "gimbal_recentered_by_dji"

    /**
     * The region of interest — where the camera is being pointed, which is **not** a manoeuvre and
     * therefore not a `*_accepted`/`*_ended` pair about flight.
     *
     * `roi_accepted` is written when a `DO_SET_ROI_LOCATION` (or the legacy `DO_SET_ROI`) passed its
     * gates and the target was taken, with the lat/lon in the message — never the commanded `z`,
     * which is discarded on purpose (`docs/m4-mission-execution.md` §9.1) and whose absence from
     * this record is the point. `roi_denied` carries the refusal reason the operator's modal names.
     * `roi_cleared` is a `DO_SET_ROI_NONE`, an abort suspending the tracking, or the target being
     * dropped with the session — the message says which.
     *
     * **Two doors write these lines, and `seq=N` is how a reader tells them apart** (2026-07-30). A
     * live command carries no `seq`; a **plan's** `DO_SET_ROI_LOCATION` / `DO_SET_ROI_NONE` item carries
     * the wire sequence number of the navigable item the cursor was moving to when it acted, so
     * `roi_accepted … seq=7` followed later by `roi_cleared commanded seq=9` reconstructs the whole of
     * `big1.plan`'s aiming (item 6 sets, item 8 clears) against `mission_current` without guessing. A
     * plan that ends with its ROI still set writes `roi_cleared plan ended with the ROI set seq=N` at
     * warn severity — the one case where the camera was left pointing at something the plan never
     * cancelled.
     *
     * The `z` note above still holds for the Fly-view door and is now half-untrue for the other one:
     * since 2026-07-30 a `MAV_FRAME_GLOBAL_RELATIVE_ALT` ROI's height **is** in our own datum and is
     * used, so `relAlt=` carries a number for those and `relAlt=none` for the discarded AMSL case. The
     * absence is still the point; which of the two happened is now readable rather than assumed.
     */
    const val ROI_ACCEPTED = "roi_accepted"
    const val ROI_DENIED = "roi_denied"
    const val ROI_CLEARED = "roi_cleared"

    /**
     * The WiFi lifecycle of the bound MAVLink socket — wifi-fix.md gotcha #2.
     * `wifi_lost`: the bound network went away, telemetry is dark. `wifi_rebound`:
     * WiFi returned as a new Network and the link swapped its socket onto it —
     * link maintenance, not an aircraft action, so it happens automatically.
     * `wifi_rebind_failed`: the swap failed; the socket stays on the dead network
     * and only a bridge restart recovers, which the operator must do.
     */
    const val WIFI_LOST = "wifi_lost"
    const val WIFI_REBOUND = "wifi_rebound"
    const val WIFI_REBIND_FAILED = "wifi_rebind_failed"

    /** Bridge start refused: no WiFi network to bind the socket to. */
    const val WIFI_REFUSED = "wifi_refused"
}
