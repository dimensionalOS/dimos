package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.record.StickAxes
import kotlin.math.ceil
import kotlin.math.exp
import kotlin.math.ln

/**
 * The Q1 envelope — hard-coded, enforced in this package, not configurable from MAVLink.
 *
 * `docs/decisions/2026-07-26-m3-guided-control.md` Q1, closed: single interlock, and this
 * envelope is now **the only thing bounding guided authority** — treat these numbers as the
 * safety argument, not as tuning. For a stick stream the speed limits are the *scale*: full
 * deflection maps to the envelope maximum and can never exceed it, because the mapping is a
 * multiplication by the limit rather than a clamp applied after a larger gain.
 *
 * A writable parameter that raises any of these is exactly the inbound surface
 * `CommandInterlock` exists to forbid (`CommandDispatcher.kt` KDoc); different numbers change
 * here and ship with a build.
 */
object GuidedEnvelope {

    /** Q1: full horizontal deflection commands this many m/s. The flight of 2026-07-26 reached 8.1 m/s under a human. */
    const val HORIZONTAL_MAX_MS = 3.0

    /** Q1: full vertical deflection commands this many m/s. */
    const val VERTICAL_MAX_MS = 1.5

    /**
     * Q1: no climb is commanded at or above this height over our own takeoff datum.
     *
     * **Raised from 30 m to 100 m on 2026-07-27**, by Ivan, after hitting the old value four
     * times in one bench session and once in real air — he reached for 50 m without thinking
     * about it, which is the honest signal about what the number should have been.
     *
     * 30 m was chosen in `docs/decisions/2026-07-26-m3-guided-control.md` Q1 when **nothing had
     * ever flown**: a deliberately conservative bound on how far a runaway could climb before
     * someone stopped it. Since then the envelope, the abort ladder, the arrival test and the
     * circling law have all been exercised on a real aircraft, and every abort gesture has fired
     * in flight. The evidence the number was waiting for exists.
     *
     * It remains a **refusal** threshold in the mission layer and a **cap** on a live command
     * (`docs/m3-stage-b.md` JC-2's split), and it is still well inside the 120 m that European
     * rules allow above ground — the aircraft's legal ceiling is not this constant's job, and
     * this constant must not be read as permission.
     *
     * **Raising it forced [MANOEUVRE_TIMEOUT_MS] up with it** — see that constant. A climb to
     * 100 m at [VERTICAL_MAX_MS] takes 67 s, which the old 60 s timeout would have aborted at
     * about 90 m, every time. Two constants that had never interacted at 30 m interact at 100.
     */
    const val CEILING_M = 100.0

    /**
     * Q1: the farthest a single reposition may reach, metres horizontally from where the
     * aircraft was when the command arrived. Beyond it the command is refused with
     * `MAV_RESULT_DENIED` and a sentence naming the limit — refused, never silently clamped,
     * because a clamped command is one the operator believes was obeyed.
     *
     * **Raised from 100 m to 2 km on 2026-07-30, by Ivan** — *"2 km is our new limit"* —
     * `docs/decisions/2026-07-30-two-kilometre-envelope.md`, which supersedes M4-3's *"line of
     * sight at this site"* reasoning for this number and for [MissionGuidance.MAX_HOME_DIST_M].
     * The 100 m was M3 Q1's, set when nothing had flown; what forced the decision was his own
     * `big1.plan` being refused at the desk (`Leg 5 is 140m: 100m max`, then `item 4 too far from
     * home` at 145.6 m against the 144 m the home bound enforced) for a survey box that is
     * ordinary flying.
     *
     * **Raising it forced the manoeuvre deadline to change shape, not just value** — see
     * [manoeuvreDeadlineMs]. At [HORIZONTAL_MAX_MS] a 2 km leg is ~667 s of travel, and the flat
     * 150 s would have aborted it mid-leg **every time**, which is strictly worse than the honest
     * desk refusal it replaced. The same trap as `CEILING_M` -> `MANOEUVRE_TIMEOUT_MS` in
     * 2026-07-27, one order of magnitude larger.
     *
     * **What this constant is not.** It is not a claim that 2 km is safe, legal or within battery:
     * at 3 m/s it is ~11 minutes one way, nothing in this repo models battery against distance,
     * and visual line of sight at ~2 km is a regulatory question the pilot owns and this bridge
     * cannot check. The decision doc states each of those in full. What the constant does is stop
     * refusing what Ivan decided to allow.
     */
    const val MAX_REPOSITION_DISTANCE_M = 2000.0

    /**
     * Q1's manoeuvre deadline in its original flat form: **the deadline for a manoeuvre with no
     * commanded distance, and the floor under every distance-derived one** ([manoeuvreDeadlineMs]).
     *
     * A manoeuvre — for a stick stream, a deflection held continuously outside the neutral band —
     * that lasts this long is treated as a stuck axis and aborted. Stage A's own design note names
     * *"a stuck axis is a continuous command with no reply channel"* as the failure this bounds
     * (`docs/m3-guided-control.md` §1.3), and a human flying the Stage A diagnostic crosses neutral
     * far more often than once a minute.
     *
     * **Raised from 60 s to 150 s on 2026-07-27** because [CEILING_M] moved: the longest manoeuvre
     * the envelope then permitted was a full-range climb, 67 s, which the old 60 s would have
     * aborted at about 90 m on every attempt.
     *
     * **Kept at 150 s on 2026-07-30, and demoted to a floor**, when [MAX_REPOSITION_DISTANCE_M]
     * became 2 km. A flat number cannot serve both a 5 m nudge and a 2 km transit: raising it to
     * cover 2 km would leave a stuck stick unbounded for twenty minutes, and leaving it at 150 s
     * would abort every long leg. So the *shape* changed and this number stayed exactly where the
     * 2026-07-27 argument put it — which is why nothing short regresses.
     *
     * Who reads it flat, and why each is right to:
     *
     * | site | why flat |
     * |---|---|
     * | the stuck-axis rung (a held GCS deflection) | a stick stream commands a *velocity*; there is no distance and no destination, so there is nothing to derive from. This is the constant's original meaning. |
     * | the tag descent's never-finished bound | its distances are metres, not kilometres: 7 m at `TagDescentGuidance.V_DESCENT_MAX_MS` is ~18 s of vertical plus the ladder's excursions, so the floor dominates any derivation and deriving one would only add a second speed model. |
     * | the shadow descent segment's mirror of it | must stay byte-identical to the live rung it mirrors, or the recording stops describing the flight. |
     */
    const val MANOEUVRE_TIMEOUT_MS = 150_000L

    /**
     * How much longer than the travel time a manoeuvre's deadline is — the whole margin in
     * [manoeuvreDeadlineMs], one number, stated rather than folded into a formula.
     *
     * **2.0, and the factor is the headwind allowance.** A deadline is a *ratio* problem, not an
     * additive one: a headwind does not cost a fixed number of seconds, it scales the ground speed,
     * so a 1.5 m/s headwind against the 3.0 m/s [HORIZONTAL_MAX_MS] doubles the time whether the
     * leg is 50 m or 2 km. Doubling is therefore the minimum honest allowance, not a generous one —
     * and no finite factor covers a 3 m/s headwind, at which the ground speed is zero and the
     * manoeuvre genuinely cannot finish. (Measured for scale: landing14 flew in 9.1 m/s of wind at
     * 10 m against a rated ~10.7 m/s resistance, so a 1.5 m/s headwind is an ordinary day here.)
     *
     * It also preserves the 2026-07-27 argument word for word — *"a manoeuvre that has not finished
     * in more than double its own worst case is not going to"* — now applied per manoeuvre instead
     * of once against the envelope's largest.
     */
    const val MANOEUVRE_MARGIN = 2.0

    /**
     * Quadrature intervals used by [travelSeconds]. Even, for Simpson's rule.
     *
     * 1024 over at most `ln(2000/2)` = 6.9 of log-distance is `h ~ 0.0067`, which pins the integral
     * to better than 0.01 % of the closed-form solution across the whole envelope — measured
     * against that closed form in `StickMappingTest`, which carries the algebra as an independent
     * witness rather than as a second implementation. The cost is ~1000 `min`/`sqrt` evaluations
     * **once per accepted manoeuvre**, not per tick.
     */
    private const val DEADLINE_QUADRATURE_STEPS = 1024

    /**
     * How long the guidance law itself needs to bring [distanceM] of error inside [acceptM] on one
     * axis under [axisCapMs] — seconds, **integrated from [RepositionGuidance.clampedSpeed] rather
     * than from a second speed model**.
     *
     * This is the one place in the project that answers "how long will this take?", and it answers
     * it by asking the law that will actually fly it:
     *
     * ```
     * t = INTEGRAL de / v(e)   from acceptM to distanceM,  v = clampedSpeed(e, axisCapMs)
     * ```
     *
     * **Why an integral and not `distance / cap`.** The law is not a constant speed: it is
     * `min(k_p*e, cap, sqrt(2*a_max*e))`, so the last metres are flown on the braking curve and
     * then on the proportional term, and a division would understate a short move badly (a 10 m
     * goto is 3.7 s, not 3.3 s) while being indistinguishable on a long one. Deriving it from
     * `clampedSpeed` means `KP_PER_S`, `A_MAX_MS2` and the axis caps have exactly one consumer for
     * this purpose: change the braking curve and every deadline in the bridge moves with it, with
     * no comment to update.
     *
     * **Why the lower limit is [acceptM] and not zero.** `v -> k_p*e` as `e -> 0`, so the integral
     * to zero diverges logarithmically — the law asymptotes onto its target and never formally
     * arrives. The manoeuvre ends at the arrival test's radius ([RepositionGuidance.R_ACCEPT_M]
     * horizontally, [RepositionGuidance.VERTICAL_ACCEPT_M] vertically), so that is the honest lower
     * limit and it is passed in rather than assumed.
     *
     * **How.** Simpson's rule in `u = ln e`, where the integrand `e/v(e)` is bounded (it tends to
     * `1/k_p`) and smooth apart from the law's two regime kinks — a substitution rather than a
     * refinement, so three decades of distance cost the same 1024 intervals as one.
     *
     * Fail-closed on nonsense: a non-finite or non-positive input returns **0.0**, which makes the
     * caller's deadline the flat [MANOEUVRE_TIMEOUT_MS] floor. That is the safe direction — a
     * garbage distance must not buy an unbounded manoeuvre.
     */
    fun travelSeconds(distanceM: Double, axisCapMs: Double, acceptM: Double): Double {
        if (!distanceM.isFinite() || !axisCapMs.isFinite() || !acceptM.isFinite()) return 0.0
        if (axisCapMs <= 0.0 || acceptM <= 0.0) return 0.0
        if (distanceM <= acceptM) return 0.0

        val lo = ln(acceptM)
        val hi = ln(distanceM)
        val h = (hi - lo) / DEADLINE_QUADRATURE_STEPS
        var sum = 0.0
        for (i in 0..DEADLINE_QUADRATURE_STEPS) {
            val e = exp(lo + i * h)
            val v = RepositionGuidance.clampedSpeed(e, axisCapMs)
            // Unreachable for e > 0 with any positive gain, cap and a_max; if the law ever did
            // command zero at a non-zero error the manoeuvre could not finish, and saying so with
            // an infinite travel time is more honest than dividing by zero.
            if (!v.isFinite() || v <= 0.0) return Double.POSITIVE_INFINITY
            val weight = when {
                i == 0 || i == DEADLINE_QUADRATURE_STEPS -> 1.0
                i % 2 == 1 -> 4.0
                else -> 2.0
            }
            sum += weight * e / v
        }
        return sum * h / 3.0
    }

    /**
     * **The single owner of "how long may this manoeuvre take".** Milliseconds, from the distance
     * the manoeuvre was actually commanded to cover.
     *
     * ```
     * deadline = max(MANOEUVRE_TIMEOUT_MS,
     *                MANOEUVRE_MARGIN * max(travelSeconds(horizontal), travelSeconds(vertical)))
     * ```
     *
     * Three deliberate choices, each of which was a bug in an earlier shape of this code:
     *
     *  - **`max` of the two axes, not their sum.** The horizontal and vertical laws run
     *    *concurrently* under their own caps ([RepositionGuidance.velocity]), so a goto that
     *    translates 2 km while climbing 40 m takes as long as its slower axis, not both. This is
     *    also what retires the hand-computed "a full-range climb is 67 s" of 2026-07-27: the
     *    vertical axis now states its own worst case, from the same law, in code.
     *  - **[MANOEUVRE_TIMEOUT_MS] as a floor**, so no short manoeuvre gets a *tighter* deadline
     *    than it had before 2026-07-30. Everything the old envelope permitted — a 100 m goto (34 s
     *    of travel), a ground-to-ceiling climb (67 s) — derives to less than 150 s and therefore
     *    keeps exactly the bound it has always had. Nothing regresses; only long legs move.
     *  - **A deadline, never a clamp.** Exceeding it ends the manoeuvre and says `timeout`; it
     *    never quietly shortens the leg.
     *
     * ## What this deadline stops being, at 2 km — stated because it matters more than the formula
     *
     * At 100 m the deadline was a real safety bound: 150 s is inside every battery and inside an
     * operator's attention, so "stuck" and "too slow" were the same event. At 2 km it derives to
     * **~1334 s (22 minutes)**, which is longer than a Mini 4 Pro flies. So for a long leg this
     * bound no longer protects the flight — **the battery does, and DJI's own RTH failsafe is the
     * only backstop**, because nothing in this repo models battery against distance and no code
     * here can. What the deadline still does honestly is what it was originally for: it ends a
     * manoeuvre that has *stopped making progress*, and it does so without the false abort a flat
     * 150 s would have produced on every long leg.
     *
     * @param horizontalM commanded horizontal distance, metres. 0.0 for a manoeuvre with no lateral
     *   component; non-finite or negative is treated as "no distance" and yields the floor.
     * @param verticalM commanded height change, metres, unsigned — same treatment. **Not zero when
     *   unknown**: a caller that cannot measure the height change must pass the largest the
     *   envelope permits ([CEILING_M]), because a substituted zero would shorten the deadline of
     *   the very manoeuvre nobody could measure.
     */
    fun manoeuvreDeadlineMs(horizontalM: Double, verticalM: Double): Long {
        val horizontal = travelSeconds(horizontalM, HORIZONTAL_MAX_MS, RepositionGuidance.R_ACCEPT_M)
        val vertical = travelSeconds(verticalM, VERTICAL_MAX_MS, RepositionGuidance.VERTICAL_ACCEPT_M)
        val travel = if (horizontal >= vertical) horizontal else vertical
        if (!travel.isFinite()) return Long.MAX_VALUE
        val derived = ceil(travel * MANOEUVRE_MARGIN * 1_000.0)
        if (derived <= MANOEUVRE_TIMEOUT_MS.toDouble()) return MANOEUVRE_TIMEOUT_MS
        return derived.toLong()
    }

    /** Q1: virtual stick is released after this long with every axis neutral. */
    const val IDLE_DISENGAGE_MS = 300_000L

    /**
     * Full yaw deflection commands this many deg/s. **Not in the Q1 table — a judgement call
     * recorded in `docs/m3-stage-a.md`.** Q1 named no yaw limit; DJI's own documented range is
     * ±100 (units inferred deg/s, `docs/m3-guided-control.md` §1.4).
     *
     * **Raised from 30 to 90 on 2026-07-27, on Ivan's call after watching an ROI reacquire.**
     * The original 30 was chosen as a deliberately timid first number — enough to prove the sign
     * and the units, and well under what a pilot uses. Three flights later the sign, the units and
     * the loop are all flight-verified, and the timidity had become the binding constraint on every
     * pointing law we have: an ROI reacquisition measured on
     * `docs/measurements/2026-07-27-first-mission-flown.md` sat **pinned at the clamp for seven
     * seconds** swinging 155°, and heading-follows-course saturates at it through every mission
     * corner. 90 °/s turns the aircraft fully around in 4 s.
     *
     * This bounds **rotation, not translation**: no yaw rate moves the aircraft anywhere, so the
     * distance bounds, the ceiling and the stopping envelope are untouched by it. What it does
     * change is how fast the camera sweeps — worth watching once in the air, since a nose that
     * whips round is unpleasant to fly behind even when it is safe.
     *
     * **Why 90 and not 120.** DJI documents ±100 for this parameter and the units are *inferred*
     * deg/s rather than measured, so 100 is the edge of what is known rather than a tested limit.
     * Commanding past it buys one of two things, both bad: a silent clamp inside DJI, which makes
     * this constant a fiction and every envelope argument that rests on it untrue, or a rejected
     * setpoint. 90 keeps a tenth of the documented range in hand for the inference being wrong.
     */
    const val YAW_RATE_MAX_DEGS = 90.0

    /**
     * §3.1's stick watchdog: `MANUAL_CONTROL` older than this stops being a command. The ramp
     * to zero starts here. Note the measured worst inbound gap on this rig is **1.01 s**
     * (`docs/status.md`, 2026-07-26 09:29), so this window *will* fire during normal operation
     * on a bad second — deliberately: the response is a benign slow-to-zero that reverses the
     * moment input resumes, and 500 ms of unrefreshed command is the most this design is
     * willing to keep flying on. The number is the design doc's own (§1.3, §3.1).
     */
    const val INPUT_STALE_MS = 500L

    /** Q4's "~0.5 s" deceleration, also used for the stale-input ramp. */
    const val RAMP_TO_ZERO_MS = 500L

    /**
     * No `MANUAL_CONTROL` for this long is treated as the stream ending, not a gap. Chosen at
     * ~3× the measured worst inbound gap (1.01 s) so a bad wireless second never triggers a
     * terminal sequence, and near QGC's own 3.5 s connection-lost heuristic so both ends give
     * up on roughly the same schedule. Whether the *rest* of the link is also silent decides
     * released-vs-link-lost — see `GuidedStickEngine`.
     */
    const val LINK_LOST_MS = 3_000L
}

/**
 * One `MANUAL_CONTROL` (#69) frame's axes, exactly as they came off the wire. `Int` because the
 * message fields are int16; no interpretation has happened yet.
 */
data class GcsStickFrame(val x: Int, val y: Int, val z: Int, val r: Int)

/** A velocity setpoint in this project's own frame: NED m/s, down-positive, yaw-rate clockwise-positive. */
data class StickVelocities(
    val north: Double,
    val east: Double,
    /** Positive **down**, like every other vertical velocity in this bridge. */
    val down: Double,
    val yawRateDegPerS: Double,
) {
    fun scaled(factor: Double): StickVelocities =
        StickVelocities(north * factor, east * factor, down * factor, yawRateDegPerS * factor)

    companion object {
        val ZERO = StickVelocities(0.0, 0.0, 0.0, 0.0)
    }
}

/**
 * `MANUAL_CONTROL` axes → envelope-bounded NED velocities → DJI virtual-stick axes. Every
 * conversion and every sign in Stage A lives in this object, unit-tested in both directions.
 *
 * ## What QGC actually sends — read from source at the pinned checkout, `da14fad28`
 *
 * Both producers scale ±1 inputs by 1000 and pack `x = pitch, y = roll, z = thrust, r = yaw`
 * (`Vehicle.cc:2998-3027`).
 *
 *  - **On-screen sticks** (`VirtualJoystick.qml:29`, 25 Hz whenever the setting is on): x, y, r
 *    are ±1000 with 0 centre. The throttle pad is `yAxisPositiveRangeOnly` for a multicopter
 *    (`VirtualJoystick.qml:68`), so **z is 0..1000**; with the default
 *    `virtualJoystickAutoCenterThrottle = true` (`App.SettingsGroup.json:159`) a released stick
 *    reads **500**. With auto-centre off the stick *starts at the bottom* — z = 0 — and holds
 *    wherever it is released.
 *  - **Gamepad** (`Joystick.cc:1129-1136`): with the default `throttleModeCenterZero = false`
 *    (`Joystick.SettingsGroup.json:67`), `throttle = (throttle + 1) / 2` — again **z 0..1000
 *    with 500 at the physical centre**. If the operator enables centre-zero, PX4's plugin
 *    supports it and a copter gets `throttle = max(0, throttle)`: z is still 0..1000 but
 *    **neutral becomes 0** and descent cannot be commanded at all.
 *
 * So one wire value, `z = 0`, means "full descend" in the default regime and "centred" in the
 * opt-in one, and nothing in the message says which regime is in force. This is landmine #2 —
 * get it wrong and neutral throttle flies into the ground.
 *
 * ## The interpretation, and how the ambiguity is made safe
 *
 * This bridge interprets **exactly one convention — QGC's default: z in [0, 1000], 500 =
 * neutral** — and refuses everything it cannot read ([read] returns a reason instead of a
 * guess for z outside 0..1000, any axis outside ±1000, or MAVLink's INT16_MAX axis-invalid
 * sentinel). The opt-in centre-zero regime cannot be *detected*, so it is excluded at the
 * engagement gate instead: `GuidedStickEngine` engages only from a stream recently observed at
 * rest — all axes neutral, **including z within [NEUTRAL_BAND] of 500** — and a centre-zero
 * stream idles at z = 0, which never reads as neutral, so it can never engage. The failure mode
 * of the wrong regime is therefore a refusal with a sentence, not a descent.
 *
 * ## The DJI half, and the one dangerous sign
 *
 * DJI's commanded vertical axis is **up-positive** — `VirtualStickRange`,
 * `VERTICAL_CONTROL_MAX_VELOCITY`: *"A positive number means to control the aircraft to fly
 * upwards"* — which is the opposite of the down-positive `vd` this bridge uses everywhere else
 * (measured 2026-07-26: climbing → `vd` negative). [toDji] carries the flip. See it for the
 * units caveat too.
 */
object StickMapping {

    /** MAVLink's "this axis is invalid" sentinel for MANUAL_CONTROL axes. */
    const val AXIS_INVALID = 32767 // INT16_MAX

    /** Wire units per full deflection (`Vehicle.cc:2998`, `axesScaling = 1.0 * 1000.0`). */
    const val FULL_SCALE = 1000

    /** The z value QGC's default regimes send for a centred/released throttle. */
    const val Z_NEUTRAL = 500

    /**
     * How far off exact centre an axis may sit and still be "neutral". 40/1000 = 4%: the
     * on-screen sticks rest at exactly 0 (and 500), so this exists for gamepad centring slop,
     * which is a few percent on a worn stick. Well below [DELIBERATE], so drift can never
     * engage anything.
     */
    const val NEUTRAL_BAND = 40

    /**
     * How far an axis must move to count as a deliberate act — the engagement trigger.
     * 100/1000 = 10% of travel: far outside any resting drift, and small enough that the first
     * commanded velocity on engagement is ≤ 0.3 m/s.
     */
    const val DELIBERATE = 100

    /** Why a frame could not be read. The text reaching the operator is `GuidedStatusTexts.BAD_AXES`. */
    sealed interface Reading {
        data class Valid(val frame: GcsStickFrame) : Reading
        data class Unreadable(val reason: String) : Reading
    }

    /**
     * Accept a frame this object knows how to interpret, or name why not. **Refusal, never a
     * guess**: an axis at INT16_MAX, outside ±1000, or a z outside [0, 1000] is a convention
     * this bridge has not measured, and interpreting it anyway is how a neutral throttle
     * becomes a descent.
     */
    fun read(x: Int, y: Int, z: Int, r: Int): Reading {
        for (v in intArrayOf(x, y, z, r)) {
            if (v == AXIS_INVALID) return Reading.Unreadable("axis INT16_MAX (invalid)")
        }
        if (x !in -FULL_SCALE..FULL_SCALE) return Reading.Unreadable("x=$x outside ±$FULL_SCALE")
        if (y !in -FULL_SCALE..FULL_SCALE) return Reading.Unreadable("y=$y outside ±$FULL_SCALE")
        if (r !in -FULL_SCALE..FULL_SCALE) return Reading.Unreadable("r=$r outside ±$FULL_SCALE")
        if (z !in 0..FULL_SCALE) {
            // The −1000..1000 z convention (or anything else). Never interpreted: in that
            // convention z=0 is neutral, and reading it with our 500-centre rule would command
            // a half-scale descent on a centred stick.
            return Reading.Unreadable("z=$z outside 0..$FULL_SCALE")
        }
        return Reading.Valid(GcsStickFrame(x, y, z, r))
    }

    /** All four axes at rest — including z at its 500 centre. The engagement gate's predicate. */
    fun isNeutral(f: GcsStickFrame): Boolean =
        absWithin(f.x, NEUTRAL_BAND) && absWithin(f.y, NEUTRAL_BAND) &&
            absWithin(f.r, NEUTRAL_BAND) && abs(f.z - Z_NEUTRAL) <= NEUTRAL_BAND

    /** Any axis moved far enough to read as an operator's act. */
    fun isDeliberate(f: GcsStickFrame): Boolean =
        !absWithin(f.x, DELIBERATE) || !absWithin(f.y, DELIBERATE) ||
            !absWithin(f.r, DELIBERATE) || abs(f.z - Z_NEUTRAL) > DELIBERATE

    /**
     * A valid frame → NED velocities, scaled by the envelope.
     *
     * The mapping is a multiplication by the limit, so **full deflection is the envelope
     * maximum and no input can exceed it** — the Q1 property. Signs, each source-verified:
     *
     *  - `x` (+forward stick) → **north**, `y` (+right stick) → **east**. MANUAL_CONTROL's x/y
     *    are the pitch/roll sticks (`Vehicle.cc:3023-3027`); under DJI's `GROUND` coordinate
     *    system the design maps stick-forward to earth-north — the frame Stage B needs and the
     *    assumption `tools/flightlog --diagnose-axis` declares (see `KeyManagerVirtualStickPort`
     *    for what `GROUND` is and is not known to mean).
     *  - `z` → **down**, negated around the 500 centre: z = 1000 (full up-stick) is a climb,
     *    i.e. down = −[GuidedEnvelope.VERTICAL_MAX_MS].
     *  - `r` (+right/clockwise) → **positive yaw rate**, clockwise looking down. This follows
     *    the QGC/PX4 reading of the field (QGC's on-screen yaw stick right sends +1000, and a
     *    PX4 yaws clockwise on it), **not** the MAVLink prose ("counter-clockwise being 1000"),
     *    which describes a twisting joystick and contradicts the ecosystem we emulate. DJI's
     *    positive yaw is also clockwise looking down (`getLeftStick` prose), so the pass-through
     *    is sign-free — and the bench observes it before anything flies.
     */
    fun velocities(f: GcsStickFrame): StickVelocities = StickVelocities(
        north = f.x / FULL_SCALE.toDouble() * GuidedEnvelope.HORIZONTAL_MAX_MS,
        east = f.y / FULL_SCALE.toDouble() * GuidedEnvelope.HORIZONTAL_MAX_MS,
        down = -(f.z - Z_NEUTRAL) / Z_NEUTRAL.toDouble() * GuidedEnvelope.VERTICAL_MAX_MS,
        yawRateDegPerS = f.r / FULL_SCALE.toDouble() * GuidedEnvelope.YAW_RATE_MAX_DEGS,
    )

    /**
     * NED velocities → the four numbers handed to `sendVirtualStickAdvancedParam`.
     *
     * ## UNITS ARE INFERRED, NOT MEASURED — the Stage A bench/flight confirms them
     *
     * DJI states **no units anywhere** for virtual-stick setpoints. The ±23 / ±6 / ±100
     * envelope in `VirtualStickRange` only reads sensibly as m/s, m/s and deg/s
     * (`docs/m3-guided-control.md` §1.4), and this bridge is built on that inference — **in
     * this one function and nowhere else**. The measurement that confirms or refutes it is
     * session-plan item 8: command exactly +1.0 on one axis for 3 s and read
     * `KeyAircraftVelocity` back. Until that has run, no propeller turns on these numbers.
     *
     * ## THE MOST DANGEROUS LINE IN M3
     *
     * `verticalThrottle = -velocities.down`. DJI's commanded vertical axis is documented
     * **up-positive** (`VirtualStickRange`: *"A positive number means to control the aircraft
     * to fly upwards"*) — the opposite sign to the down-positive `vd` measured off this
     * aircraft and passed through everywhere else in the bridge. Remove or double this
     * negation and a commanded climb is a commanded descent. It is pinned by named tests in
     * both directions and stays on the bench checklist (item 10) because a documented sign and
     * a real one are different facts.
     *
     * Axis naming is DJI's trap, and the bench settled it (**measured 2026-07-26**, record
     * `20260726-204721.001`, `docs/measurements/2026-07-26-stage-a-first-engagement.md`):
     * under `GROUND` + `VELOCITY` the axes are earth-referenced velocities — GROUND was
     * proven earth-frame by identical response at yaw 0°, 45° and 116° — but **`pitch`
     * drives EAST and `roll` drives NORTH**, the exact swap of the pre-measurement
     * assumption (and of what `StickAxes`' names suggest). Commanded (N+3, E−3) flew
     * (N−3, E+3) until this swap; after it, magnitudes matched 3.0–3.2 m/s at every yaw.
     * The other three axes were measured correct the same session: vertical sign (climb
     * commanded, climb flown), m/s magnitudes, and yaw at 6.2°/s commanded → 6.7°/s flown,
     * 30°/s cap → 29.4°/s flown.
     */
    fun toDji(v: StickVelocities): StickAxes = StickAxes(
        pitch = v.east,
        roll = v.north,
        yaw = v.yawRateDegPerS,
        verticalThrottle = -v.down,
    )

    /**
     * The physical RC sticks, expressed in the **same envelope-scaled velocities**
     * [velocities] produces — for comparing the operator's hand against a controller's
     * command, and for exactly nothing else: no setpoint is ever built from this.
     *
     * It is [velocities] itself, fed the RC's `[-660, 660]` travel rescaled to the
     * `MANUAL_CONTROL` wire's ±1000 (and the throttle re-centred onto the 0..1000/500
     * convention), so there is **one mapping** and a comparison cannot lie by using a second
     * one. Mode-2 layout, this project's RC: right stick = pitch (forward) / roll (right),
     * left stick = throttle (up) / yaw.
     *
     * Two honesty notes for any consumer. The output's `north`/`east` are really the sticks'
     * **body-frame forward/right** — DJI's RC commands the airframe, not the world — so a
     * comparison against an earth-frame command must rotate by the aircraft's heading first.
     * And the *scale* is the Q1 envelope's: DJI's own full-stick response in GPS mode is
     * faster than 3 m/s, so magnitudes are comparable between the two arrows this feeds, not
     * against the aircraft's actual speed.
     */
    fun rcVelocities(rc: RcSticks): StickVelocities? {
        if (!rc.allPresent()) return null
        fun toWire(v: Int): Int =
            Math.round(v * FULL_SCALE / RcSticks.FULL_DEFLECTION).toInt().coerceIn(-FULL_SCALE, FULL_SCALE)
        return velocities(
            GcsStickFrame(
                x = toWire(rc.rightVertical!!),
                y = toWire(rc.rightHorizontal!!),
                z = (Z_NEUTRAL + Math.round(rc.leftVertical!! * Z_NEUTRAL / RcSticks.FULL_DEFLECTION).toInt())
                    .coerceIn(0, FULL_SCALE),
                r = toWire(rc.leftHorizontal!!),
            )
        )
    }

    private fun absWithin(v: Int, band: Int): Boolean = abs(v) <= band

    private fun abs(v: Int): Int = if (v < 0) -v else v
}
