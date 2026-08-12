package com.dimensional.mini4pro.telemetry

/**
 * The aircraft state in plain Kotlin — no DJI types anywhere.
 *
 * This is a deliberate seam. The MSDK is not on the unit-test runtime classpath,
 * so anything that touches `dji.*` cannot be unit-tested. Keeping this model free
 * of DJI types means the arithmetic that turns it into MAVLink (degE7, mm, cm/s,
 * centiamps, sign inversions) is testable on the JVM, which is where a units bug
 * gets caught before it reaches an aircraft.
 *
 * `StateCache` adapts DJI values into this; [TelemetryEncoder] turns it into
 * MAVLink messages.
 *
 * **Every field is nullable and null means "no valid reading", never zero.** A
 * `null` from a KeyManager listener is DJI's component-gone signal, and a zero
 * that is really "unknown" is how a bridge ends up telling a GCS the aircraft is
 * at latitude 0, longitude 0.
 *
 * **That contract is this model's job to uphold, not DJI's to honour.** DJI does
 * not always report absence as null: with no home point recorded,
 * `KeyHomeLocation` returns a populated coordinate whose latitude and longitude
 * are both `4.583662361046586E7` (measured 2026-07-26 — 220 fabricated
 * `HOME_POSITION` messages, `docs/measurements/2026-07-26-home-position-sentinel.md`).
 * `StateCache` validates on the way in so that a null here really does mean
 * unknown; see [Geo] for the rule.
 *
 * Units here are the *DJI-native* ones, unconverted, as measured on device
 * 2026-07-25 (see docs/measurements/2026-07-25-ground-probe.md). Conversion is
 * the encoder's job, in one place, under test.
 *
 * ## A value and its age are two different facts
 *
 * Every DJI key updates on its own schedule and some of them barely update at all
 * — `KeyAircraftVelocity` fired **once in 35 s** in the ground probe. The fields
 * above are a cache, so a reader at 5 Hz is often looking at a value that arrived
 * long ago, and a flat velocity of zero could equally mean "stationary" or "the
 * feed died". [ages] carries how long ago each reading was last *delivered*, so
 * that question is answerable rather than guessed at.
 *
 * It is deliberately a **separate, defaulted field rather than a wrapper around
 * every value**: `s.velocityNorth` still reads as a number, and only the code that
 * cares about freshness — the flight recorder today, `GuidedController` in M3 —
 * pays any attention. See [SampleAges] for the delivered-vs-changed distinction
 * and `docs/decisions/2026-07-25-per-field-staleness.md` for why this changes
 * nothing on the wire.
 */
data class AircraftState(
    /** True once the flight controller is up — i.e. the aircraft itself is powered. */
    val fcConnected: Boolean = false,

    // ── position ──
    /** Degrees. */
    val latitude: Double? = null,
    /** Degrees. */
    val longitude: Double? = null,
    /** Metres, **relative to the takeoff point** (reads 0 on the ground). */
    val relativeAltitude: Double? = null,
    /**
     * `FlightControllerKey.KeyTakeoffLocationAltitude`, metres — the datum the
     * whole bridge's AMSL is built on, and **it is pressure altitude, not AMSL.**
     *
     * Measured 2026-07-26 (`docs/measurements/2026-07-26-amsl-datum.md`): across
     * eight sessions at one stationary site it read 103.2 m on 2026-07-25 and
     * 59.2 m on 2026-07-26 — a 44 m swing — and subtracting ICAO-standard pressure
     * altitude computed from independent reanalysis surface pressure leaves a
     * **constant +12.5 ± 2.4 m**. So the reading is a real barometer, correctly
     * measuring the wrong quantity: it is referenced to 1013.25 hPa rather than to
     * the local QNH, and its error against true elevation is
     * `(1013.25 - QNH) x ~8.3 m`, which was +14 m one day and −28 m the next.
     *
     * The name is kept because it is DJI's, and no constant is applied because
     * there is no constant to apply. What the value is safe for is a **round
     * trip**: report `datum + relativeAltitude` as AMSL, and convert a commanded
     * AMSL back with the *same* datum, and the offset cancels exactly. What it is
     * not safe for is comparison with anything that did not come from this
     * barometer — a survey, a terrain database, or a plan built on another day.
     * Barometric drift is ~0.7 m within a session and up to 2.3 m over twelve
     * minutes while stationary, so even the round trip wants a recent datum.
     */
    val takeoffAltitudeAmsl: Double? = null,

    // ── attitude ──
    /** Degrees. Sign convention UNVERIFIED — needs a tilt test. */
    val rollDeg: Double? = null,
    /** Degrees. Sign convention UNVERIFIED. */
    val pitchDeg: Double? = null,
    /** Degrees, signed [-180, 180], 0 = north. */
    val yawDeg: Double? = null,

    // ── velocity, NED (north/east/down), metres per second ──
    val velocityNorth: Double? = null,
    val velocityEast: Double? = null,
    /** Positive **down**. MAVLink climb rate is positive up — negate. */
    val velocityDown: Double? = null,

    // ── gps ──
    val satelliteCount: Int? = null,
    /** 0..5 as measured (`GPSSignalLevel.LEVEL_0`..`LEVEL_5`); null when NONE/UNKNOWN. */
    val gpsSignalLevel: Int? = null,

    // ── home ──
    val homeLatitude: Double? = null,
    val homeLongitude: Double? = null,

    /**
     * `FlightControllerKey.KeyIsHomeLocationSet` — DJI's own answer to whether a
     * home point exists, and the authority on the question.
     *
     * It exists here because **`homeLatitude`/`homeLongitude` cannot answer it by
     * themselves**. Before a home point is recorded, `KeyHomeLocation` returns a
     * populated coordinate rather than null: measured 2026-07-26, both fields read
     * `4.583662361046586E7` while this key read `false`, and the two flipped
     * together the moment a real home was recorded
     * (`docs/measurements/2026-07-26-home-position-sentinel.md`).
     *
     * `false` means DJI is telling us there is no home. `null` means the key has
     * never been delivered — which is *not* evidence either way, so it must not be
     * read as "no home"; the coordinate validation in [Geo] is what covers that
     * case. See `TelemetryEncoder.homePosition` for the precedence between them.
     */
    val homeLocationSet: Boolean? = null,

    // ── flight state ──
    val isFlying: Boolean? = null,
    val motorsOn: Boolean? = null,
    /** `FCFlightMode.name`, e.g. "APAS". Kept as a string to avoid a DJI enum here. */
    val flightMode: String? = null,

    // ── prearm / readiness ──
    /**
     * DJI's own "motors are not allowed to start" flag
     * (`FlightControllerKey.KeyNotAllowMotorStart`).
     *
     * **This is necessary but not sufficient for flight readiness.** DJI's
     * support has stated it cannot always explain `KeyStartTakeoff` failing with
     * `-7` (MSDK issue #783), so the aircraft can refuse takeoff for reasons not
     * exposed by any key. Treat `false` as "no known blocker", never as a
     * guarantee, and treat `null` as not-ready rather than ready.
     */
    val notAllowMotorStart: Boolean? = null,
    val imuWarmingUp: Boolean? = null,
    val inFailsafe: Boolean? = null,

    // ── aircraft settings ──
    /**
     * `FlightControllerKey.KeyGoHomeHeight` — the return-to-home altitude the
     * aircraft is configured to climb to, in **metres relative to the takeoff
     * point** (DJI: *"relative altitude when returning home … related to the
     * altitude when taking off"*).
     *
     * A *setting*, not a measurement: it is the first entry here that describes
     * how the aircraft is configured rather than what it is doing, and it is the
     * only one read by the parameter layer (`RTL_RETURN_ALT`). See
     * [Signal.GO_HOME_HEIGHT] for why it carries no staleness limit and
     * `handshake/Parameters.AIRCRAFT_PARAMETERS` for the two behavioural caveats
     * that ride with it.
     */
    val goHomeHeightM: Int? = null,

    // ── battery ──
    val batteryPercent: Int? = null,
    /** Whole-pack millivolts. */
    val voltageMv: Int? = null,
    /** Milliamps, **negative while discharging** (DJI convention). */
    val currentMa: Int? = null,
    val cellCount: Int? = null,
    /**
     * Per-cell millivolts, in cell order, straight from `BatteryKey.KeyCellVoltages`.
     * Measured `[4186, 4183]` on the 2S pack — real per-cell data, so
     * `BATTERY_STATUS.voltages` should carry these rather than an even split of
     * the pack voltage. An even split would hide cell imbalance, which is exactly
     * the condition the per-cell field exists to expose.
     */
    val cellVoltagesMv: List<Int>? = null,
    /** Degrees Celsius. */
    val batteryTempC: Double? = null,

    // ── freshness ──
    /**
     * How long ago each [Signal] was last delivered by DJI, at the instant this
     * snapshot was taken. Defaults to [SampleAges.NONE] — "we have never heard
     * from the aircraft" — which is the truth for a bare `AircraftState()`.
     *
     * Prefer the [ageMs] / [isFresh] / [isStale] accessors below at a read site.
     */
    val ages: SampleAges = SampleAges.NONE,
) {

    /** Milliseconds since [signal] was last delivered, or null if it never was. */
    fun ageMs(signal: Signal): Long? = ages[signal]

    /** True when [signal] arrived within its own [Signal.staleAfterMs]. */
    fun isFresh(signal: Signal): Boolean = ages.isFresh(signal)

    /** True when [signal] arrived within [limitMs]. Never delivered is never fresh. */
    fun isFresh(signal: Signal, limitMs: Long): Boolean = ages.isFresh(signal, limitMs)

    /**
     * True when [signal] has gone quiet past its own limit, or was never delivered.
     *
     * **For M3:** an aged velocity is not a measured zero, it is the absence of
     * feedback, and a controller must treat it as such. See [SampleAges].
     */
    fun isStale(signal: Signal): Boolean = ages.isStale(signal)

    /** True when [signal] has gone quiet for longer than [limitMs]. */
    fun isStale(signal: Signal, limitMs: Long): Boolean = ages.isStale(signal, limitMs)
}
