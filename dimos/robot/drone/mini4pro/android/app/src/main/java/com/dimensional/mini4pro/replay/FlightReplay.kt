package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import com.dimensional.mini4pro.vision.TagSighting

/**
 * Turns a recorded flight back into the `AircraftState` snapshots it was made of — **the
 * replay harness**, and the thing that makes a real flight into an offline test fixture.
 *
 * Pure Kotlin: no Android, no DJI, no Zenoh, no clock reads. It takes a [FlightRecord] and
 * emits `(timestamp, AircraftState)`, which is exactly the input `ZenohTelemetryEncoder`
 * and `TelemetryEncoder` take. Everything downstream of `StateCache` can therefore be run,
 * on the JVM, against 371 seconds of a genuine orbit rather than against a synthetic state
 * somebody typed.
 *
 * ## 1. Two streams, one picture
 *
 * The recorder splits `AircraftState` by cadence, not by importance (`record/StateDelta`):
 *
 * - **`dji_state`**, at the recorder's rate, carries the fast half — position, altitude,
 *   attitude, velocity — in DJI-native units.
 * - **`dji_field`**, on change only, carries the slow half: battery, satellites, GPS level,
 *   flight mode, home, the readiness flags.
 *
 * Replay is the inverse: fold every `dji_field` forward into a running picture, and merge
 * that picture with each `dji_state` sample. [FIELD_MAP] is the whole mapping, one entry per
 * `AircraftState` field the record carries, and [UNRECORDED] names the ones it does not.
 *
 * ### Ordering, which is not the file's order
 *
 * `StateDelta.entriesFor` derives the `dji_field` lines *from* the state it was handed, and
 * writes them **after** the `dji_state` line, at the same `t`. So a field change stamped at
 * the same instant as a sample was already true *in* that sample, and applying it in file
 * order would put every slow value one sample late. This applies fields with
 * `t <= sample.t` before emitting, which restores the state the recorder actually had.
 *
 * ## 2. Staleness is replayed, not recomputed — the point of the whole exercise
 *
 * `ZenohTelemetryEncoder` **withholds** a message when its input is stale, so a replay that
 * did not reproduce staleness would be exercising a different program than the one that
 * flew. The four continuous signals have their ages recorded on every `dji_state` line and
 * are replayed **verbatim** — not derived from how long ago the value changed, which is a
 * different question with a different answer (`SampleAges`: delivered, not changed).
 *
 * That distinction is load-bearing on this data. In the reference flight, `ALTITUDE`'s age
 * ramps linearly from ~100 ms to ~5.7 s and resets — `KeyAltitude` arrives in bursts every
 * five or six seconds — while the *value* sits at 29.3 m and does not move at all. Derived
 * from value changes, altitude would look fresh for the entire orbit; from the recorded
 * ages it is stale in 688 of 1155 in-flight samples, and the encoder withholds `odom`,
 * `pose` and `gps_location` for every one of them. The second number is the true one.
 *
 * For the **event-driven** signals — everything with a null `Signal.staleAfterMs` — an age
 * is reconstructed as *time since the last recorded change*. That is honest for them
 * precisely because `SampleAges.isFresh` reduces to "have we ever heard it" when there is no
 * limit, so only presence matters and the number itself is never read. The one signal where
 * this is a genuine approximation is `TAKEOFF_ALTITUDE`, which has a 2 s limit *and* is
 * deadbanded at 0.25 m by the recorder; see [ReplayCoverage.TAKEOFF_ALTITUDE_AGE].
 *
 * ## 3. What it does not invent
 *
 * A field the record does not carry stays null, and a signal never recorded stays absent
 * from [SampleAges] — which is what `AircraftState` means by null and what makes the
 * replayed encoder output honest. Nothing here defaults, clamps or interpolates.
 */
object FlightReplay {

    /**
     * One recorded slow field, and where it lands in [AircraftState].
     *
     * @param signal the [Signal] whose delivery this field is evidence of, so a folded value
     *   also marks its signal as heard-from. Two fields may share a signal (`homeLatitude`
     *   and `homeLongitude` are one `KeyHomeLocation` delivery).
     */
    class FieldBinding(
        /** The name on the wire, as `record/StateDelta.F` or `record/Recorder` spells it. */
        val name: String,
        /** The [AircraftState] property this lands in. Usually the same word; `isHomeLocationSet` is not. */
        val stateField: String,
        val signal: Signal,
        val apply: (Fold, String?) -> Unit,
    )

    /** The mutable picture a fold builds. Public so a caller can inspect it mid-replay. */
    class Fold {
        var fcConnected: Boolean = false
        var takeoffAltitudeAmsl: Double? = null
        var satelliteCount: Int? = null
        var gpsSignalLevel: Int? = null
        var homeLatitude: Double? = null
        var homeLongitude: Double? = null
        var homeLocationSet: Boolean? = null
        var isFlying: Boolean? = null
        var motorsOn: Boolean? = null
        var flightMode: String? = null
        var notAllowMotorStart: Boolean? = null
        var imuWarmingUp: Boolean? = null
        var inFailsafe: Boolean? = null
        var batteryPercent: Int? = null
        var voltageMv: Int? = null
        var currentMa: Int? = null
        var cellCount: Int? = null
        var cellVoltagesMv: List<Int>? = null
        var batteryTempC: Double? = null

        /** `t` of the last recorded change per signal — the basis of a reconstructed age. */
        val changedAtSeconds = HashMap<Signal, Double>()

        /** Field names seen in the record that no binding claims. Reported, never guessed at. */
        val unmappedFields = LinkedHashMap<String, Int>()
    }

    /**
     * Every `AircraftState` field the record carries, and nothing else.
     *
     * The names are the recorder's own: `record/StateDelta.F` for the ones derived from
     * `AircraftState`, and `record/Recorder.subscribeDji` for `isHomeLocationSet`, which the
     * recorder subscribes to directly rather than through the state cache. If a name here
     * stops matching a name there the replay silently loses a field, so
     * `FlightReplayTest` asserts the two lists against each other rather than trusting this
     * comment.
     */
    val FIELD_MAP: List<FieldBinding> = listOf(
        FieldBinding("fcConnected", "fcConnected", Signal.FC_CONNECTION) { f, v -> f.fcConnected = bool(v) ?: false },
        FieldBinding("takeoffAltitudeAmsl", "takeoffAltitudeAmsl", Signal.TAKEOFF_ALTITUDE) { f, v -> f.takeoffAltitudeAmsl = dbl(v) },
        FieldBinding("satelliteCount", "satelliteCount", Signal.SATELLITES) { f, v -> f.satelliteCount = int(v) },
        FieldBinding("gpsSignalLevel", "gpsSignalLevel", Signal.GPS_LEVEL) { f, v -> f.gpsSignalLevel = int(v) },
        FieldBinding("homeLatitude", "homeLatitude", Signal.HOME) { f, v -> f.homeLatitude = dbl(v) },
        FieldBinding("homeLongitude", "homeLongitude", Signal.HOME) { f, v -> f.homeLongitude = dbl(v) },
        FieldBinding("isHomeLocationSet", "homeLocationSet", Signal.HOME_SET) { f, v -> f.homeLocationSet = bool(v) },
        FieldBinding("isFlying", "isFlying", Signal.FLYING) { f, v -> f.isFlying = bool(v) },
        FieldBinding("motorsOn", "motorsOn", Signal.MOTORS) { f, v -> f.motorsOn = bool(v) },
        FieldBinding("flightMode", "flightMode", Signal.FLIGHT_MODE) { f, v -> f.flightMode = v },
        FieldBinding("notAllowMotorStart", "notAllowMotorStart", Signal.MOTOR_START_VETO) { f, v -> f.notAllowMotorStart = bool(v) },
        FieldBinding("imuWarmingUp", "imuWarmingUp", Signal.IMU_WARMUP) { f, v -> f.imuWarmingUp = bool(v) },
        FieldBinding("inFailsafe", "inFailsafe", Signal.FAILSAFE) { f, v -> f.inFailsafe = bool(v) },
        FieldBinding("batteryPercent", "batteryPercent", Signal.BATTERY_PERCENT) { f, v -> f.batteryPercent = int(v) },
        FieldBinding("voltageMv", "voltageMv", Signal.BATTERY_VOLTAGE) { f, v -> f.voltageMv = int(v) },
        FieldBinding("currentMa", "currentMa", Signal.BATTERY_CURRENT) { f, v -> f.currentMa = int(v) },
        FieldBinding("cellCount", "cellCount", Signal.CELL_COUNT) { f, v -> f.cellCount = int(v) },
        FieldBinding("cellVoltagesMv", "cellVoltagesMv", Signal.CELL_VOLTAGES) { f, v -> f.cellVoltagesMv = intList(v) },
        FieldBinding("batteryTempC", "batteryTempC", Signal.BATTERY_TEMP) { f, v -> f.batteryTempC = dbl(v) },
    )

    private val BY_NAME: Map<String, FieldBinding> = FIELD_MAP.associateBy { it.name }

    /**
     * `AircraftState` fields **no line of the record carries** — the replay's blind spots,
     * stated here so a caller can see them rather than discover them as a null.
     *
     * `goHomeHeightM` is the only one: nothing in `record/StateDelta.F` emits it and
     * `record/Recorder` does not subscribe to `KeyGoHomeHeight`. It feeds `RTL_RETURN_ALT`
     * on the MAVLink wire and nothing at all on the Zenoh side, so Zenoh replay is complete
     * without it — see [ReplayCoverage].
     */
    val UNRECORDED: List<String> = listOf("goHomeHeightM")

    /**
     * The replayed samples, one per `dji_state` line, in time order.
     *
     * @param eventDrivenAges when true (the default) a folded field also marks its signal as
     *   delivered, with an age measured from the last recorded change. False leaves
     *   [SampleAges] carrying only the four recorded ages, which is what you want to prove a
     *   consumer does not secretly depend on an event-driven signal's age.
     */
    fun samples(record: FlightRecord, eventDrivenAges: Boolean = true): List<ReplaySample> {
        val fold = Fold()
        val fields = record.fields
        val out = ArrayList<ReplaySample>(record.states.size)
        var next = 0
        for (entry in record.entries) {
            if (entry !is RecordEntry.DjiState) continue
            // Fields stamped at or before this sample were already true in it — see §1.
            while (next < fields.size && fields[next].tSeconds <= entry.tSeconds) {
                applyField(fold, fields[next])
                next++
            }
            out.add(sampleAt(record.header, fold, entry, eventDrivenAges))
        }
        return out
    }

    /** Folds one `dji_field` line into [fold], counting it as unmapped when nothing claims it. */
    fun applyField(fold: Fold, field: RecordEntry.Field) {
        val binding = BY_NAME[field.name]
        if (binding == null) {
            fold.unmappedFields[field.name] = (fold.unmappedFields[field.name] ?: 0) + 1
            return
        }
        binding.apply(fold, field.value)
        fold.changedAtSeconds[binding.signal] = field.tSeconds
    }

    /** The state and stamps for one `dji_state` line against the current [fold]. */
    fun sampleAt(
        header: RecordHeader?,
        fold: Fold,
        s: RecordEntry.DjiState,
        eventDrivenAges: Boolean = true,
    ): ReplaySample {
        val ages = HashMap<Signal, Long>(24)
        // The four recorded ages, verbatim. Absent means never delivered — §2.
        s.positionAgeMs?.let { ages[Signal.POSITION] = it }
        s.altitudeAgeMs?.let { ages[Signal.ALTITUDE] = it }
        s.attitudeAgeMs?.let { ages[Signal.ATTITUDE] = it }
        s.velocityAgeMs?.let { ages[Signal.VELOCITY] = it }
        if (eventDrivenAges) {
            for ((signal, at) in fold.changedAtSeconds) {
                // Never overwrite a recorded age with a reconstructed one.
                if (signal in ages) continue
                ages[signal] = Math.round((s.tSeconds - at) * 1000.0).coerceAtLeast(0L)
            }
        }
        val state = AircraftState(
            fcConnected = fold.fcConnected,
            latitude = s.latitude,
            longitude = s.longitude,
            relativeAltitude = s.relativeAltitudeM,
            takeoffAltitudeAmsl = fold.takeoffAltitudeAmsl,
            rollDeg = s.rollDeg,
            pitchDeg = s.pitchDeg,
            yawDeg = s.yawDeg,
            velocityNorth = s.velocityNorth,
            velocityEast = s.velocityEast,
            velocityDown = s.velocityDown,
            satelliteCount = fold.satelliteCount,
            gpsSignalLevel = fold.gpsSignalLevel,
            homeLatitude = fold.homeLatitude,
            homeLongitude = fold.homeLongitude,
            homeLocationSet = fold.homeLocationSet,
            isFlying = fold.isFlying,
            motorsOn = fold.motorsOn,
            flightMode = fold.flightMode,
            notAllowMotorStart = fold.notAllowMotorStart,
            imuWarmingUp = fold.imuWarmingUp,
            inFailsafe = fold.inFailsafe,
            batteryPercent = fold.batteryPercent,
            voltageMv = fold.voltageMv,
            currentMa = fold.currentMa,
            cellCount = fold.cellCount,
            cellVoltagesMv = fold.cellVoltagesMv,
            batteryTempC = fold.batteryTempC,
            ages = SampleAges.of(ages),
        )
        return ReplaySample(
            tSeconds = s.tSeconds,
            monoNanos = header?.monoNanosAt(s.tSeconds),
            unixMillis = header?.unixMillisAt(s.tSeconds),
            state = state,
        )
    }

    // ── value parsing ─────────────────────────────────────────────────────────
    //
    // The recorder renders every slow value with `toString()` or `Json.num`, so these are
    // the exact inverses. A value that will not parse becomes null rather than a default:
    // "we cannot read this" and "it was 0" are the two things this codebase refuses to
    // confuse.

    fun bool(v: String?): Boolean? = when (v) {
        "true" -> true
        "false" -> false
        else -> null
    }

    fun int(v: String?): Int? = v?.trim()?.toIntOrNull() ?: v?.trim()?.toDoubleOrNull()?.let {
        if (it.isFinite()) Math.round(it).toInt() else null
    }

    fun dbl(v: String?): Double? = v?.trim()?.toDoubleOrNull()

    /** `"3999,3999"` — `StateDelta.emitCells` joins the raw per-cell list with commas. */
    fun intList(v: String?): List<Int>? {
        if (v == null) return null
        if (v.isBlank()) return emptyList()
        val parts = v.split(',')
        val out = ArrayList<Int>(parts.size)
        for (p in parts) out.add(p.trim().toIntOrNull() ?: return null)
        return out
    }

    /**
     * **The tag sightings a record carries, back as `Sighting`s** — one per `tag` line.
     *
     * The record already holds everything a sighting is: `TagTap` writes the id, the camera-frame
     * metres, the pixel centre, the frame geometry, the hamming distance, the decision margin and
     * `metric`, stamped at the frame's arrival. This is the inverse, and it is deliberately the
     * same reconstruction `tools/kotlinframes` does on the desktop — that one exists to
     * byte-compare a conversion against the shipping encoder, this one to feed the shipping
     * encoder from a file, and a second field mapping that disagreed with the first would be a
     * bug neither could see.
     *
     * ## Tolerant of fields that are not there, and of fields that are not there **yet**
     *
     * Every value is read by key through [RecordJson], which answers null for an absent one, and
     * every absent one falls back to the default [TagSighting.Sighting] itself declares. So a
     * record written before a field existed replays as a sighting that never had it — which is
     * true — and a record written *after* one is added replays unchanged until this function is
     * taught to read it. That matters right now: corner and solved-pose fields are being added to
     * `tag` lines by other work, and neither direction of that skew may break a replay.
     *
     * A line with **no `id` is skipped** rather than defaulted. Tag 0 is a real tag, so there is
     * no value that could stand in for "this line did not say", and a sighting of tag 0 that was
     * never seen is precisely the false detection `TagLatch` exists to keep out.
     *
     * ## What it deliberately does not reconstruct
     *
     * The `TagFix` — the world position — and the latch flag. Both are on the line and neither is
     * carried, because the only consumer is `ZenohBus.publishDetection`, which takes a sighting
     * and nothing else: a `Detection3D` is stamped `drone/camera_optical` and a fix is a place in
     * `drone/world`, so a fix belongs on a `tf`-relative channel that does not exist. A value
     * nothing reads is a second mapping to keep honest for no consumer.
     */
    fun sightings(record: FlightRecord): List<TimedSighting> {
        val out = ArrayList<TimedSighting>()
        for (e in record.entries) {
            if (e !is RecordEntry.Other || e.kind != LogEntry.KIND_TAG) continue
            val raw = e.raw
            val id = RecordJson.long(raw["id"])?.toInt() ?: continue
            out.add(
                TimedSighting(
                    tSeconds = e.tSeconds,
                    sighting = TagSighting.Sighting(
                        tagId = id,
                        x = RecordJson.number(raw["x"]) ?: 0.0,
                        y = RecordJson.number(raw["y"]) ?: 0.0,
                        z = RecordJson.number(raw["z"]) ?: 0.0,
                        // The frame's arrival on the **record's own** clock, in nanoseconds. An
                        // absolute monotonic value from the recording phone means nothing on this
                        // one, so `t` is used directly; the only reader of the difference is
                        // `ageMillisAt`, and the replay controller supplies a "now" on this scale.
                        atNanos = Math.round(e.tSeconds * 1e9),
                        pixelSize = RecordJson.number(raw["px"]) ?: 0.0,
                        metric = raw["metric"] == true,
                        hamming = RecordJson.long(raw["ham"])?.toInt() ?: 0,
                        decisionMargin = RecordJson.number(raw["margin"]) ?: 0.0,
                        centreX = RecordJson.number(raw["cx"]) ?: 0.0,
                        centreY = RecordJson.number(raw["cy"]) ?: 0.0,
                        imageWidth = RecordJson.long(raw["w"])?.toInt() ?: 0,
                        imageHeight = RecordJson.long(raw["h"])?.toInt() ?: 0,
                    ),
                )
            )
        }
        return out
    }
}

/**
 * One recorded sighting and the record's own `t` for it.
 *
 * Separate from [ReplaySample] and not folded into it, because the two streams have genuinely
 * different rates and different absences: states arrive at the recorder's 5 Hz for the whole
 * session, sightings at up to 10 Hz for the seconds a tag was in frame and never otherwise.
 * Merging them would mean either a state per sighting — inventing samples — or a nullable
 * sighting on every state, which is a column that is empty for 95 % of every flight.
 */
data class TimedSighting(val tSeconds: Double, val sighting: TagSighting.Sighting)

/**
 * One replayed instant: an `AircraftState` and the three clocks it can be stamped with.
 *
 * [unixMillis] is the one a `LcmTime` must come from — LCM headers carry seconds since the
 * epoch, and a replay stamped with [tSeconds] alone would publish a fleet of messages dated
 * 1970. It is null when the record had no header, which is a real case for a session file
 * that was split or truncated, and null is the honest answer rather than 0.
 */
data class ReplaySample(
    /** Seconds since the session started, as the record stamps it. */
    val tSeconds: Double,
    /** `SystemClock.elapsedRealtimeNanos` on the recording phone, or null with no header. */
    val monoNanos: Long?,
    /** Unix milliseconds, or null with no header. */
    val unixMillis: Long?,
    val state: AircraftState,
)
