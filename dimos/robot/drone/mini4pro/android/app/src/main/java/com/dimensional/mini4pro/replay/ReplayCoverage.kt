package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.zenoh.ZenohChannel

/**
 * **The coverage report**: exactly what `ZenohTelemetryEncoder` needs that a
 * `mini4pro-flightlog-1` record does not carry, which message each gap blocks, and whether
 * the thing is absent from the schema or merely not recorded often enough.
 *
 * This is the specification for extending the recorder, and it is what decides whether
 * *"record all the messages coming from a drone so that we can replay them into Zenoh so
 * that we can test everything offline"* is actually true or only nearly true. The answer,
 * measured against `android/app/src/test/resources/replay/orbit-real-air.jsonl` (a genuine 371 s flight with a
 * 24.7 m orbit, 2026-07-27):
 *
 * > **Seven of the eight telemetry channels replay completely and faithfully. One —
 * > `gimbal` — cannot be replayed at all, because no line of the record says where the
 * > camera was pointing.**
 *
 * Nothing else on the Zenoh telemetry side is missing. Every input the other seven channels
 * read is present, including the per-signal staleness that decides whether they publish at
 * all, which is the part that is easy to get wrong and was already right.
 *
 * ## The gaps, exactly
 *
 * **Amended 2026-07-27**: the gimbal's *recorder* half is closed — a record taken today carries a
 * `gimbal` line with all three axes and the delivery age. `FlightReplay` does not yet read it, so
 * the channel is still listed as blocked; see [GIMBAL] for exactly which half moved.
 *
 * | what | message it blocks | absent, or under-recorded? |
 * |---|---|---|
 * | gimbal pitch / roll / yaw | `gimbal` (`geometry_msgs.Vector3`) — **the whole channel** | **No longer absent.** Recorded since 2026-07-27 as its own `gimbal` entry; what is missing now is the reader, and any record older than that date |
 * | operator sentences | `status` (`std_msgs.String`) — **the whole channel** | **Under-recorded in form, not in substance.** Every sentence is in the record; which lines *were* announcements is not derivable yet. Live-only — see [STATUS_SENTENCES] |
 * | `goHomeHeightM` | nothing on Zenoh; `RTL_RETURN_ALT` on the MAVLink wire | **Absent entirely.** `Signal.GO_HOME_HEIGHT` exists and `handshake/Parameters` reads it, but no recorder line emits it |
 * | `TAKEOFF_ALTITUDE`'s **age** | nothing on Zenoh — see [TAKEOFF_ALTITUDE_AGE] | **Under-recorded.** The value is present; only its delivery age is not recoverable |
 * | event-driven signals' **ages** | nothing — see [EVENT_DRIVEN_AGES] | **Under-recorded, and harmless.** No consumer reads them |
 *
 * ## What is *not* a gap, and is worth saying out loud
 *
 * - **The four continuous ages are recorded on every sample.** `age.pos`, `age.relalt`,
 *   `age.att`, `age.vel` appear on all 1855 `dji_state` lines of the reference flight. They
 *   are the four `Signal`s with a `staleAfterMs`, and therefore the only four whose age the
 *   encoder ever reads. This is the single most important thing the recorder already gets
 *   right: without them a replay would publish `odom` continuously where the real flight
 *   published it 40 % of the time.
 * - **`isHomeLocationSet` is recorded** even though it is not in `record/StateDelta.F` —
 *   `record/Recorder.subscribeDji` subscribes to `KeyIsHomeLocationSet` directly. So the
 *   home-point sentinel case (`telemetry/Geo`) is replayable.
 * - **Per-cell voltages survive as a list**, `"3999,3999"`, never averaged, so
 *   `BatteryState.cell_voltage` replays with its cell imbalance intact.
 * - **`NaN` and `Infinity` survive the round trip**, written by `record/Json` as the three
 *   quoted spellings and read back by [RecordJson.number].
 *
 * ## What the record carries that the *inbound* half will need, and does
 *
 * `mav_in`/`mav_out` are in the record with both decoded fields and raw hex, so the command
 * half of the bus — `goal`, `cmd_vel`, `command` — can be replayed against the MAVLink
 * equivalents when that transport exists. That is out of scope here and is noted so the
 * absence is not mistaken for a gap.
 *
 * ## Reading this against a session of your own
 *
 * `tools/zenohreplay <session>.jsonl` prints the same report for any record, plus what that
 * particular flight would have published and withheld. `tools/zenohreplay --coverage` prints
 * this list without a record.
 */
object ReplayCoverage {

    /**
     * One entry of the coverage table, in a form a tool can print and a test can assert on.
     *
     * @param blocks the Zenoh channel this gap makes impossible, or null when it blocks
     *   nothing on the Zenoh side.
     * @param absent true when no line of the record carries the thing at all; false when it
     *   is present but not often enough, or not in a form that answers the question asked.
     */
    data class Gap(
        val what: String,
        val blocks: ZenohChannel?,
        val absent: Boolean,
        val detail: String,
    )

    /**
     * The gimbal — **the recorder half is closed as of 2026-07-27; the reader half is not.**
     *
     * When this file was written the reading was absent from the record entirely. It no longer
     * is: `record/LogEntry.Gimbal` is a `gimbal` line carrying all three axes and the delivery
     * age, written by `record/Recorder.sampleGimbal` on change past a 0.5° deadband with a 1 Hz
     * heartbeat, fed by a supplier `Bridge` installs alongside the gimbal's DJI half. So a record
     * taken today **does** say where the camera was pointing.
     *
     * [blocks] is still [ZenohChannel.GIMBAL] because this object reports what a *replay*
     * can reproduce, and `FlightReplay` does not yet read the new line. That is the remaining
     * half and it is a reader change, not a recorder one. Records taken before 2026-07-27 — the
     * reference flight in `src/test/resources/replay/` among them — carry no `gimbal` line at all
     * and remain unreplayable on this channel whatever the reader learns to do.
     */
    val GIMBAL = Gap(
        what = "gimbal attitude (pitch/roll/yaw, degrees)",
        blocks = ZenohChannel.GIMBAL,
        absent = false,
        detail = "Recorder half CLOSED 2026-07-27, reader half open. The record now carries a " +
            "`gimbal` line (`record/LogEntry.Gimbal`) with pitch, roll, yaw and the delivery " +
            "age, on change past a 0.5° deadband with a 1 Hz heartbeat — the age being what " +
            "separates a camera holding steady from a feed that died, which is the trap this " +
            "channel is worst for. `ZenohTelemetryEncoder.gimbalAttitudeOrNull` still takes " +
            "loose angles and still imports nothing from `gimbal/`; the recorder's " +
            "`GimbalSample` is its own type for the same reason. What remains is `FlightReplay` " +
            "learning to read the line, plus the fact that records taken before this date do " +
            "not contain it.",
    )

    val GO_HOME_HEIGHT = Gap(
        what = "AircraftState.goHomeHeightM",
        blocks = null,
        absent = true,
        detail = "Absent entirely — `record/StateDelta.F` has no entry for it and " +
            "`record/Recorder` does not subscribe to `KeyGoHomeHeight`. It is the only " +
            "`AircraftState` field no line of the record carries. Nothing on the Zenoh side " +
            "reads it, so Zenoh replay is complete without it; `RTL_RETURN_ALT` on the MAVLink " +
            "wire is not, and a MAVLink parameter replay would report the wrong RTH altitude.",
    )

    /**
     * The one place a reconstructed age is a genuine approximation rather than a formality.
     *
     * `TAKEOFF_ALTITUDE` is the only signal that both carries a `Signal.staleAfterMs` (2 s)
     * and is written on change with a deadband (`StateDelta.Deadbands.takeoffAltitudeM`,
     * 0.25 m). So the record's stamps say when the *value moved a quarter of a metre*, not
     * when DJI last delivered it — four lines in 371 seconds against a key measured at ~10 Hz
     * on the ground probe. A replay that reconstructed an age from those stamps would call
     * the signal stale for essentially the whole flight.
     *
     * It costs nothing today: `ZenohTelemetryEncoder` never gates on `TAKEOFF_ALTITUDE`
     * (`OdomDatum` takes the altitude as metadata and publishes `NaN` when it is absent), and
     * `TelemetryEncoder` reads no `Signal` at all. It would cost something the moment a
     * consumer starts checking it, which is why it is on this list rather than dismissed.
     */
    val TAKEOFF_ALTITUDE_AGE = Gap(
        what = "Signal.TAKEOFF_ALTITUDE delivery age",
        blocks = null,
        absent = false,
        detail = "Under-recorded. The value is present but deadbanded at 0.25 m, so its stamps " +
            "are change times, not delivery times, and the reconstructed age is an upper bound. " +
            "No current consumer reads it. Fixed by adding `age.takeoffalt` to the `dji_state` " +
            "entry beside the four that are already there — ~10 bytes a line.",
    )

    /**
     * Every event-driven signal's age, which is under-recorded and does not matter.
     *
     * `SampleAges.isFresh` reduces to "have we ever heard this key" when `staleAfterMs` is
     * null, and that is the whole of what any consumer asks of a battery, a flight mode or a
     * home point. So reconstructing those ages from change stamps — which is what
     * [FlightReplay] does — is exact for the only question anyone asks, and wrong only for a
     * number nobody reads.
     */
    val EVENT_DRIVEN_AGES = Gap(
        what = "delivery ages of the event-driven signals (battery, mode, home, flags)",
        blocks = null,
        absent = false,
        detail = "Under-recorded and harmless. `Signal.staleAfterMs` is null for all of them, " +
            "so `isFresh` asks only whether the key has ever been heard, which the record " +
            "answers exactly. The reconstructed age is time-since-change; no consumer reads it.",
    )

    /**
     * **`status` — the operator's sentences, which a record holds and a replay cannot rebuild.**
     *
     * Added 2026-07-27 with the live publisher, because until then this object had no reason to
     * mention a channel nothing published. `docs/zenoh-replay-contract.md` §7 already listed the
     * row — *"the record carries the underlying events, but `zenohreplay` does not yet reconstruct
     * them"* — and this is that row, in the place a tool prints from.
     *
     * The gap is genuinely a reader gap and not a recorder one, exactly like [GIMBAL]'s. Every
     * sentence an operator was ever shown is in the record as an `event` or as the `STATUSTEXT`
     * inside a `mav_out` line, because `command/Announcer` fans out to both. What is missing is
     * the derivation: which of those lines were announcements, in what order, at what severity.
     *
     * It is [absent] `false` for that reason, and it blocks the channel because a replayed bus
     * that carried a *partial* reconstruction of the refusals would be worse than one that
     * carries none — a subscriber cannot tell a quiet flight from a lossy transcript.
     */
    val STATUS_SENTENCES = Gap(
        what = "operator sentences (`status`, std_msgs.String)",
        blocks = ZenohChannel.STATUS,
        absent = false,
        detail = "Reader half open. The sentences are in the record — `command/Announcer` fans " +
            "every one out to the MAVLink STATUSTEXT the recorder taps, and the guided and " +
            "command layers write their own `event` lines beside them — but `FlightReplay` does " +
            "not reconstruct which lines were announcements, at what severity, in what order. " +
            "The live publisher attaches an `Announcer.Sink` and publishes them directly, so " +
            "this channel is live-only until that derivation exists.",
    )

    /**
     * **`tf` — the tree, which a replay could half-build and therefore must not.**
     *
     * The `world` → `base_link` edge is entirely derivable from a record: it is `pose`'s position
     * and `pose`'s attitude under `pose`'s rule. The `base_link` → `camera` edge is not, from the
     * records this project holds — `ReplaySample` carries no gimbal at all, and every recording
     * taken before `RecordedGimbalPort` (`0c7bf51`) carries no commanded angle either.
     *
     * So a replayed `tf` would be a tree with the camera silently missing from it, and that is
     * strictly worse than no tree: a consumer resolving `drone/world` → `drone/camera_optical`
     * would get *"no such path"* from a flight where the camera was in fact pointed and known,
     * with nothing distinguishing that from a flight where it was not. [STATUS_SENTENCES] blocks
     * its channel on exactly this argument — a partial reconstruction a subscriber cannot tell
     * from a complete one.
     *
     * `tools/memexport` does build the tree, from the same records, because it reads the raw JSONL
     * rather than [ReplaySample] and can therefore reach the `gimbal` lines and the `dji_call`
     * asks. Closing this gap is a `FlightReplay` change — carry the gimbal onto the sample — and
     * then all three sinks gain the tree at once, which is the shape `docs/mem2-converter.md` §6.2
     * argues for and the reason this is recorded rather than quietly worked around.
     */
    val TF_TREE = Gap(
        what = "the frame tree (`tf`, tf2_msgs.TFMessage)",
        blocks = ZenohChannel.TF,
        absent = false,
        detail = "Reader half open. `LogEntry.Gimbal` and the `dji_call` gimbal asks are both in " +
            "the record — `tools/memexport` builds the tree from them — but `ReplaySample` " +
            "carries no gimbal, so a replayed tree would omit the camera edge with no way for a " +
            "subscriber to tell that from a flight where the camera was never aimed. Fixed by " +
            "carrying the gimbal onto `ReplaySample`; all three sinks gain the tree together.",
    )

    /**
     * **`camera_info` — the intrinsics, which need the stream's stated geometry.**
     *
     * The record has it: `LogEntry.Frame` writes `w`/`h`/`mime` on change plus once at the start,
     * precisely because a resolution change invalidates the intrinsics. `ReplaySample` does not —
     * it is a fold of `dji_state`, and a `frame` line is not one. The same one-line fix as
     * [TF_TREE], on the same object, and it is [absent] `false` for the same reason.
     */
    val CAMERA_GEOMETRY = Gap(
        what = "the video stream's stated geometry (`camera_info`, sensor_msgs.CameraInfo)",
        blocks = ZenohChannel.CAMERA_INFO,
        absent = false,
        detail = "Reader half open. `LogEntry.Frame` states `w`/`h` on change plus once at the " +
            "start, so a record carries the resolution the intrinsics scale with — but " +
            "`ReplaySample` folds `dji_state` only and never sees a `frame` line. " +
            "`tools/memexport` reads the JSONL directly and does produce this stream.",
    )

    /**
     * **`video` — and this one is not a gap in the reader at all.**
     *
     * The pixels are not in the JSONL and were never meant to be: they are in the `.h264` sidecar
     * beside it, indexed by one `frame` line per access unit, because 5.85 Mbit/s of H.264 is two
     * orders of magnitude larger than everything else the record holds. `tools/videoexport` and
     * `tools/memexport` read the sidecar and reproduce the channel byte for byte.
     *
     * What cannot is `replay/`, which is handed a parsed JSONL and no file handle — so this is a
     * scope boundary rather than a missing derivation, and it is [absent] `true` because from
     * this package's point of view the bytes genuinely are not there.
     */
    val VIDEO_BYTES = Gap(
        what = "the encoded video (`video`, foxglove_msgs.CompressedVideo)",
        blocks = ZenohChannel.VIDEO,
        absent = true,
        detail = "Out of scope rather than missing. The access units live in the `.h264` sidecar " +
            "beside the JSONL, one `frame` index line each; `tools/videoexport` and " +
            "`tools/memexport` reproduce them byte for byte. `replay/` is handed a parsed record " +
            "and no sidecar, so it cannot and should not.",
    )

    /**
     * The tag sightings — **the recorder half was closed on 2026-07-28; the reader half is not.**
     *
     * Exactly [GIMBAL]'s shape and exactly its lesson. `record/LogEntry.Tag` carries everything
     * `vision_msgs.Detection3D` needs — the id, the camera-frame metres, the frame geometry, the
     * hamming distance, the decision margin and `metric` — on one line per detected frame, stamped
     * with the frame's arrival. What `replay/` does not have is a reader for it: `FlightReplay`
     * folds a record into `ReplaySample`, which is an aircraft state, and a sighting is not one.
     *
     * So this is a reader gap and not a recorder gap, [absent] false, and the channel is blocked
     * until somebody writes the fold. Doing it half-way would be worse: `detections` on a replay
     * that silently emitted nothing would be indistinguishable from a flight where the detector
     * never armed, which is the identical confusion `status` and `tf` are blocked to avoid.
     */
    val TAG_SIGHTINGS = Gap(
        what = "the tag sightings (`detections`, vision_msgs.Detection3DArray)",
        blocks = ZenohChannel.DETECTIONS,
        absent = false,
        detail = "Recorded since 2026-07-28 as `LogEntry.Tag`, one line per detected frame, with " +
            "the camera-frame metres and the `metric` flag the message needs. `replay/` folds a " +
            "record into an aircraft state and a sighting is not one, so the reader does not " +
            "exist yet. A record older than that date carries no sighting at all.",
    )

    /**
     * The tag's world-frame fix — [TAG_SIGHTINGS]'s shape exactly, on the same `tag` lines.
     *
     * The recorder half has carried the fix since the line existed (`n`/`e`, `fix_metric`,
     * `range_src` since 2026-07-29, `pitch_reported`), and the same reader gap blocks it: a fix
     * is not an aircraft state and `FlightReplay` has no fold for one. `tools/memexport` and
     * `tools/kotlinframes` both read the lines directly, which is why the channel is byte-checked
     * offline while remaining unreplayable through `replay/`.
     */
    val TAG_FIX = Gap(
        what = "the tag's world-frame fix (`tag_fix`, vision_msgs.Detection3DArray)",
        blocks = ZenohChannel.TAG_FIX,
        absent = false,
        detail = "The `tag` line has carried the fix half (`n`/`e` and its provenance) since the " +
            "line existed; the reader gap is TAG_SIGHTINGS's — a fix is not an aircraft state, " +
            "and `replay/` folds a record into aircraft states.",
    )

    /**
     * The commanded velocity — recorded exhaustively since the first `stick_cmd` line, and
     * blocked from `replay/` by the same shape of reader gap: a command is not an aircraft
     * state. The offline consumers (`tools/memexport`, `tools/kotlinframes`) read the lines
     * directly; the in-app replay has no reason to re-command anything.
     */
    val STICK_COMMANDS = Gap(
        what = "the commanded velocity (`setpoint`, geometry_msgs.TwistStamped)",
        blocks = ZenohChannel.SETPOINT,
        absent = false,
        detail = "`stick_cmd` lines carry the setpoint, its frame and the SDK verdict on every " +
            "send. A command is not an aircraft state, so `FlightReplay` has no fold for it; " +
            "the offline converters read the lines directly and are byte-checked.",
    )

    /**
     * The wind reading — recorded since the `windSpeedDmS` listener existed, and blocked from
     * `replay/` by [STICK_COMMANDS]'s shape of reader gap, **with one deliberate twist**: wind
     * is kept out of `AircraftState` *on purpose* (`ZenohChannel.WIND`), because the key is
     * change-driven and any per-sample carrier would let a sampler republish a held reading
     * as a fresh measurement. So this reader gap is not an omission awaiting a fold — closing
     * it by adding wind to the state would break the channel's cadence contract.
     */
    val WIND_READINGS = Gap(
        what = "DJI's wind estimate (`wind`, std_msgs.Float32)",
        blocks = ZenohChannel.WIND,
        absent = false,
        detail = "`dji_field` windSpeedDmS lines carry every delivery, on change, in DJI's own " +
            "dm/s. A key delivery is not an aircraft state — deliberately, see " +
            "`ZenohChannel.WIND` — so `FlightReplay` has no fold for it; the offline " +
            "converters (`tools/memexport`, `tools/kotlinframes`) read the lines directly " +
            "and are byte-checked.",
    )

    /**
     * DJI's warnings — recorded exhaustively as `dji_warn` (and as `dji_health` up to landing17),
     * and blocked from `replay/` by [STICK_COMMANDS]'s shape of reader gap: **a warning is not an
     * aircraft state.**
     *
     * The record half is not merely present, it is the *decided* event — the severity, the
     * sentence and the diagnostic level a live subscriber saw, written by the one owner
     * (`warn/WarningBus`) that fed both. So an offline converter reproduces the channel from the
     * record without re-deciding anything, which is exactly the property `tools/memexport` relies
     * on and the one a fold into `AircraftState` would put at risk.
     */
    val WARNINGS = Gap(
        what = "DJI's warnings (`warnings`, diagnostic_msgs.DiagnosticArray)",
        blocks = ZenohChannel.WARNINGS,
        absent = false,
        detail = "`dji_warn` lines carry every warning change, already decided: DJI's own state " +
            "word, our level, the severity, the sentence and whether the operator was told. A " +
            "warning is not an aircraft state, so `FlightReplay` has no fold for it; the " +
            "offline converters read the lines directly.",
    )

    /** Everything above, in report order. */
    val GAPS: List<Gap> = listOf(
        GIMBAL, STATUS_SENTENCES, TF_TREE, CAMERA_GEOMETRY, VIDEO_BYTES, TAG_SIGHTINGS,
        TAG_FIX, STICK_COMMANDS, WIND_READINGS, WARNINGS,
        GO_HOME_HEIGHT, TAKEOFF_ALTITUDE_AGE, EVENT_DRIVEN_AGES,
    )

    /** The gaps that make a Zenoh channel impossible — the ones that decide the headline. */
    val BLOCKING: List<Gap> = GAPS.filter { it.blocks != null }

    /**
     * The channels a record can reproduce in full. Everything in `docs/zenoh-topics.md`'s
     * telemetry table that this encoder produces, minus [BLOCKING].
     */
    val REPLAYABLE_CHANNELS: List<ZenohChannel> =
        ZenohChannel.entries.filter { ch -> BLOCKING.none { it.blocks == ch } }
}
