package com.dimensional.mini4pro.telemetry

/**
 * One independently-delivered reading from the aircraft — **the unit of freshness**.
 *
 * There is one constant here per `KeyManager` key that `StateCache` subscribes to,
 * not one per [AircraftState] field, because a DJI key is what actually arrives:
 * `KeyAircraftVelocity` delivers north, east and down together or not at all, so
 * asking "how old is `velocityEast`" separately from `velocityNorth` would be
 * inventing a distinction the SDK does not make.
 *
 * ## Freshness limits
 *
 * [staleAfterMs] is **null for most signals, and that is the honest answer.** DJI
 * delivers a key on its own schedule and several of them are event-driven: the
 * flight mode, the home point, the failsafe flag and the motor-start veto arrive
 * when they change and then stay silent for the rest of the flight. A large age on
 * one of those means "nothing has happened", not "the feed died", so attaching a
 * limit would manufacture a warning out of normal operation — the surest way to
 * teach an operator to ignore warnings, which this project has already reasoned
 * itself out of twice (see `TelemetryEncoder.prearmHealthy` and
 * `HandshakeResponder`'s mode-refusal de-duplication).
 *
 * A limit is set only where the probe measured a **continuous** delivery cadence
 * (`docs/measurements/2026-07-25-ground-probe.md`, 2595 callbacks over ~35 s):
 *
 * | key | samples in ~35 s | rate | limit here |
 * |---|---|---|---|
 * | `KeyAircraftLocation3D` | 1757 | ~50 Hz | 1000 ms |
 * | `KeyTakeoffLocationAltitude` | ~350 | ~10 Hz | 2000 ms |
 * | `KeyAircraftAttitude` | 67 | ~1.9 Hz | 2000 ms |
 * | `KeyAircraftVelocity` | **1** | change-only | 2000 ms, **see below** |
 *
 * Every limit is at least ~4× the measured period, so ordinary jitter never trips
 * it. `KeyAltitude` was not counted separately in the probe; it is given
 * `KeyAircraftLocation3D`'s limit because it is the same barometric/GNSS solution
 * arriving through a second key.
 *
 * ## MEASURED 2026-07-26: attitude is change-driven too, and this table understates it
 *
 * `docs/measurements/2026-07-26-attitude-and-staleness.md`. Against a real aircraft, motors off,
 * sitting still for 25 s then hand-tilted:
 *
 * | signal | median age | max age |
 * |---|---|---|
 * | `POSITION` | 81 ms | 1.8 s |
 * | `ATTITUDE` | 945 ms | **15.3 s** |
 * | `VELOCITY` | 4.3 s | **22.3 s** |
 * | `TAKEOFF_ALTITUDE` | 25.2 s | **50.3 s** |
 *
 * `ATTITUDE` and `VELOCITY` went fresh **within 600 ms of the aircraft being picked up** and stale
 * again once it was set down. So DJI publishes them on change, not on a schedule — and the ~1.9 Hz
 * figure in the table above is an artefact of a probe that was *handling* the aircraft while
 * measuring it.
 *
 * The practical effect: the 2 s limit on `ATTITUDE` fired four times in 53 s on a motionless
 * aircraft, and `VELOCITY`'s three times. On the ground those are false positives by construction.
 * They are left in place anyway, for two reasons. **In flight they are probably right** — an
 * airborne multirotor's attitude never stops changing, so silence there really would mean a dead
 * feed, which is exactly what M3 needs to know. And **nothing acts on them**: they reach the flight
 * log and stop there, so a false positive costs a log line rather than an operator warning. Whether
 * these keys deliver at rate in flight is the open item that decides it, now covering attitude as
 * well as velocity.
 *
 * `POSITION`'s real rate is ~12 Hz, so its 1 s limit is ~12× the period rather than the ~50×
 * implied by the probe's sample counts. Still comfortable, and now grounded in a direct measurement.
 *
 * ## The velocity caveat, which is the whole reason this exists
 *
 * `KeyAircraftVelocity` fired **once in 35 seconds** on a stationary aircraft. On
 * this airframe, on the ground, `VELOCITY` will therefore read stale essentially
 * always — and that is **correct information, not a false alarm**: we genuinely
 * cannot tell "the aircraft is not moving" from "the velocity feed stopped". The
 * two are indistinguishable from the app side, which is exactly why a guided
 * controller must treat an aged velocity as *no feedback at all* rather than as a
 * measured zero.
 *
 * Whether the key delivers continuously **in flight** is unmeasured and is an open
 * item on the moving-aircraft session. If it turns out to deliver at rate while
 * airborne, this limit becomes a genuine liveness check; if it stays change-only,
 * the limit stays a statement about what we do not know.
 */
enum class Signal(
    /**
     * How long after the last *delivery* this reading stops being usable as a live
     * measurement, or **null when age carries no freshness meaning** because DJI
     * only sends the key on change. See the class doc.
     */
    val staleAfterMs: Long? = null,
) {
    /** `FlightControllerKey.KeyConnection` — event-driven. */
    FC_CONNECTION,

    /** `KeyAircraftLocation3D`: latitude + longitude (+ its unused altitude). */
    POSITION(1_000),

    /** `KeyAltitude`: metres relative to the takeoff point. */
    ALTITUDE(1_000),

    /** `KeyTakeoffLocationAltitude`: the AMSL datum. */
    TAKEOFF_ALTITUDE(2_000),

    /** `KeyAircraftAttitude`: roll + pitch + yaw, delivered together. */
    ATTITUDE(2_000),

    /** `KeyAircraftVelocity`: north + east + down, delivered together. */
    VELOCITY(2_000),

    /** `KeyGPSSatelliteCount`. */
    SATELLITES,

    /** `KeyGPSSignalLevel`. */
    GPS_LEVEL,

    /** `KeyHomeLocation` — event-driven; set once and then silent. */
    HOME,

    /**
     * `KeyIsHomeLocationSet` — DJI's own answer to "is there a home point?".
     *
     * Event-driven and therefore **no limit**, for exactly the reason in the class
     * doc: measured 2026-07-26, it delivered once at connect and then again only
     * when home was actually recorded (`false` at t=1036.5, `true` at t=1091.2 in
     * `tmp/session-logs/20260725-191604.001.jsonl`). An hour of silence on this key
     * means the home point has not changed, not that the feed died.
     */
    HOME_SET,

    /** `KeyIsFlying`. */
    FLYING,

    /** `KeyAreMotorsOn`. */
    MOTORS,

    /** `KeyFCFlightMode`. */
    FLIGHT_MODE,

    /** `KeyNotAllowMotorStart` — DJI's own arming veto. */
    MOTOR_START_VETO,

    /** `KeyIsIMUWarmingUp`. */
    IMU_WARMUP,

    /** `KeyIsFailSafe`. */
    FAILSAFE,

    /**
     * `FlightControllerKey.KeyGoHomeHeight` — the aircraft's configured
     * return-to-home altitude, in metres above the takeoff point.
     *
     * **No limit, and this is the clearest case in the enum for why.** It is a
     * *setting*, not a measurement: it changes when somebody changes it in DJI Fly
     * and is otherwise silent for an entire flight. `canGet/canSet/canListen` are
     * all true and `setIsEvent` is false in the key's static initialiser
     * (bytecode, MSDK 5.18.0 — see `handshake/Parameters.kt`), so `getOnce`
     * delivers it at subscribe and then nothing arrives until an operator moves
     * the slider. An hour of silence here means "the RTH altitude has not been
     * changed", which is the normal case; a staleness limit would turn that into
     * a warning on every flight.
     *
     * It also has a consumer that would be actively harmed by one: this is the
     * only signal that feeds a **MAVLink parameter** (`RTL_RETURN_ALT`), and a
     * parameter has no freshness channel on the wire at all. The value we publish
     * is the last one DJI stated, which stays true until DJI states another.
     */
    GO_HOME_HEIGHT,

    /** `BatteryKey.KeyChargeRemainingInPercent`. */
    BATTERY_PERCENT,

    /** `BatteryKey.KeyVoltage` — whole-pack millivolts. */
    BATTERY_VOLTAGE,

    /** `BatteryKey.KeyCurrent` — milliamps, negative while discharging. */
    BATTERY_CURRENT,

    /** `BatteryKey.KeyNumberOfCells`. */
    CELL_COUNT,

    /** `BatteryKey.KeyCellVoltages`. */
    CELL_VOLTAGES,

    /** `BatteryKey.KeyBatteryTemperature`. */
    BATTERY_TEMP,
    ;

    companion object {
        /** The signals whose delivery is continuous enough for an age to mean something. */
        val CONTINUOUS: List<Signal> = entries.filter { it.staleAfterMs != null }
    }
}

/**
 * How long ago each [Signal] was last **delivered**, as of the instant an
 * [AircraftState] was taken. Milliseconds; absent means *never delivered*.
 *
 * ## Delivered, not changed
 *
 * This is the distinction the whole feature turns on. A key that DJI re-delivers
 * with an identical value is **fresh but unchanged**; a key that stops being
 * delivered is **stale**. `StateCache` therefore stamps on every callback,
 * including one that carries the same number as the last, and including one that
 * carries `null` — a `null` from a `KeyManager` listener is DJI's documented
 * *component-gone* signal, which is a *fresh statement* that the component is
 * gone. So a signal can legitimately be fresh with a null value, and that pair
 * means something quite specific: we have just been told there is no reading.
 *
 * Implementing this by watching for value changes instead would report a
 * genuinely motionless aircraft and a dead velocity feed identically — the exact
 * bug this exists to expose.
 *
 * ## Why ages and not timestamps
 *
 * An [AircraftState] is a snapshot, and a snapshot that carried absolute stamps
 * would force every reader to also know *when* the snapshot was taken and on which
 * clock. Ages are self-contained, keep [AircraftState] free of clock reads (it
 * stays a pure value object with no Android or DJI imports), and make a unit test a
 * matter of writing down a number rather than of controlling time.
 */
class SampleAges private constructor(private val ageMs: Map<Signal, Long>) {

    /** Milliseconds since [signal] was last delivered, or null if it never was. */
    operator fun get(signal: Signal): Long? = ageMs[signal]

    /**
     * True when [signal] arrived within [limitMs]. A signal that has **never**
     * arrived is not fresh — an absence is not a young reading.
     */
    fun isFresh(signal: Signal, limitMs: Long): Boolean {
        val age = ageMs[signal] ?: return false
        return age <= limitMs
    }

    /** [isFresh] against the signal's own [Signal.staleAfterMs]. */
    fun isFresh(signal: Signal): Boolean {
        val limit = signal.staleAfterMs ?: return ageMs.containsKey(signal)
        return isFresh(signal, limit)
    }

    fun isStale(signal: Signal, limitMs: Long): Boolean = !isFresh(signal, limitMs)

    /**
     * Whether [signal] has gone quiet for longer than its own limit.
     *
     * For a signal with no limit — an event-driven key — this answers only "have we
     * ever heard it", because for those a long silence is normal operation and not
     * a fault. See [Signal] for why that is deliberate rather than a gap.
     */
    fun isStale(signal: Signal): Boolean = !isFresh(signal)

    /**
     * Every continuous signal that has gone quiet past its own limit, in [Signal]
     * declaration order. Event-driven signals are never listed — see [Signal].
     */
    fun staleSignals(): List<Signal> = Signal.CONTINUOUS.filter { isStale(it) }

    /** Ages as a plain map, for a recorder or a test. Never contains nulls. */
    fun asMap(): Map<Signal, Long> = ageMs

    override fun equals(other: Any?): Boolean =
        this === other || (other is SampleAges && ageMs == other.ageMs)

    override fun hashCode(): Int = ageMs.hashCode()

    override fun toString(): String =
        if (ageMs.isEmpty()) "SampleAges(none)"
        else "SampleAges(" + Signal.entries.mapNotNull { s -> ageMs[s]?.let { "$s=${it}ms" } }
            .joinToString(", ") + ")"

    companion object {
        /**
         * Nothing has ever been delivered. This is the default on a bare
         * [AircraftState], which is exactly what `Bridge` and `Recorder` use before
         * MSDK registration completes — so "we have never heard from the aircraft"
         * is representable and is not confused with "we heard from it just now".
         */
        val NONE = SampleAges(emptyMap())

        fun of(vararg ages: Pair<Signal, Long>): SampleAges =
            if (ages.isEmpty()) NONE else SampleAges(mapOf(*ages))

        fun of(ages: Map<Signal, Long>): SampleAges =
            if (ages.isEmpty()) NONE else SampleAges(HashMap(ages))

        /**
         * Ages at [nowMs] from the monotonic instants each signal was last
         * delivered. A signal absent from [deliveredAtMs] has never been delivered
         * and stays absent here.
         *
         * Both arguments must come from the **same monotonic** clock
         * (`SystemClock.elapsedRealtime`, not `System.currentTimeMillis`): a
         * wall-clock step mid-flight would otherwise turn a fresh reading into an
         * hour-old one, or worse, into a negative age. Ages are clamped at 0 so a
         * clock that misbehaves anyway cannot produce a nonsensical number.
         */
        fun since(nowMs: Long, deliveredAtMs: Map<Signal, Long>): SampleAges {
            if (deliveredAtMs.isEmpty()) return NONE
            val out = HashMap<Signal, Long>(deliveredAtMs.size * 2)
            for ((signal, at) in deliveredAtMs) out[signal] = (nowMs - at).coerceAtLeast(0)
            return SampleAges(out)
        }
    }
}
