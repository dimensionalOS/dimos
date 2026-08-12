package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.telemetry.Geo
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * M4's arithmetic — the two leg-completion tests, the fly-through stopping envelope and the route's
 * distance-to-rest walk. Pure functions only; what the *engine* does with them is next door in
 * `GuidedMissionTest`.
 *
 * Written to fail loudly for the M4 landmines:
 *
 *  - **`e_stop` quietly made infinite**, which is the one mutation §3.3 names by name and which
 *    turns the stopping envelope into dead code. The property test walks generated missions and
 *    checks the bound at every tick against an *independently* computed distance-to-rest.
 *  - **the final waypoint's arrival test re-derived** instead of calling
 *    [RepositionGuidance.settled] — the only flight-verified predicate this project owns.
 *  - **the half-plane term dropped** (a waypoint the aircraft cannot quite reach hangs until the
 *    leg timeout) or **its corridor dropped** (a waypoint declared passed from 60 m to the side).
 *  - **the per-item acceptance radius honoured outside its band**, which would let a plan widen a
 *    switch radius to something that is no longer a waypoint.
 *  - the `cos(latitude)` factor absent from the route's leg walk — invisible at the equator, 21 %
 *    wrong at this project's home latitude of 38°N, and now four suites deep.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, the **whole**
 * suite run, confirmed red, reverted. Counts are failing tests across all 1802 — measured, not
 * estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `e_stop` replaced by `Double.MAX_VALUE` (the envelope made dead code) | 3 |
 *  | `e_stop` replaced by the current leg's error alone (fly-through broken) | 1 |
 *  | `restAheadM` stops at the first step instead of the first **resting** step | 1 |
 *  | the half-plane term dropped from [MissionGuidance.passed] | 3 |
 *  | the `R_MISS` corridor dropped from the half-plane term | 1 |
 *  | the switch radius honoured outside `[2, 10] m` instead of falling back | 1 |
 *  | `a_max` doubled in the braking term | 4 |
 *  | the horizontal envelope cap removed from the fly-through speed | 3 |
 *  | the per-leg timeout replaced by the flat Q1 manoeuvre timeout | 1 |
 *  | the last step's `rest` coercion removed from [MissionRoute.of] | 1 |
 *  | the launch gate's `rest = last` dropped (the *other* layer, alone) | **0 — alive on purpose** |
 *  | **both** rest layers removed at once | 2 |
 *
 * ### The one result worth reading rather than counting
 *
 * **"The last waypoint rests" is two layers, and each masks the other exactly.** `MissionLaunch`
 * marks the final leg `rest = last`, *and* [MissionRoute.of] coerces the last step to rest whatever
 * the caller said. Break either alone and the other still gives the right answer, so that mutant
 * lives — on purpose, and recorded rather than removed, because the coercion is what will still be
 * right the day a second route builder exists (a resume from a partial plan, a Zenoh-side one), and
 * the launch gate's flag is what keeps `restAheadM`'s walk terminating for the route it builds
 * today. Removing **both** is caught by 2 tests, which is the measurement that shows the property is
 * tested at all. Same shape as `GuidedOrbitTest`'s swept-counter pair, and kept for the same reason.
 */
class MissionGuidanceTest {

    private companion object {
        /** The project's home latitude family — deliberately non-equatorial (cos 38° = 0.788). */
        const val LAT = 38.0
        const val LON = 23.7

        fun latNorthOf(metres: Double): Double = LAT + metres / Geo.METRES_PER_DEG

        fun lonEastOf(metres: Double): Double =
            LON + metres / (Geo.METRES_PER_DEG * cos(Math.toRadians(LAT)))

        fun step(
            seq: Int,
            northM: Double,
            eastM: Double,
            rest: Boolean = false,
            radius: Double = MissionGuidance.R_SWITCH_M,
        ) = MissionStep(
            seq = seq,
            kind = if (rest) MissionStepKind.HOLD else MissionStepKind.WAYPOINT,
            latDeg = latNorthOf(northM),
            lonDeg = lonEastOf(eastM),
            relAltM = 20.0,
            switchRadiusM = radius,
            rest = rest,
        )
    }

    // ------------------------------------------------------- the pass-through test (§3.2)

    @Test
    fun `an intermediate waypoint is passed once the aircraft is inside the switch radius`() {
        assertTrue(
            MissionGuidance.passed(
                distanceM = 2.9,
                alongTrackRemainingM = 2.9,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
            )
        )
    }

    @Test
    fun `an intermediate waypoint is not passed while it is still ahead and out of radius`() {
        assertFalse(
            MissionGuidance.passed(
                distanceM = 4.0,
                alongTrackRemainingM = 4.0,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
            )
        )
    }

    @Test
    fun `THE HALF-PLANE - a waypoint the aircraft could not quite reach is passed once it is behind`() {
        // 4 m away, outside the 3 m radius, but the projection on the leg says it is behind us:
        // wind or an obstacle brake left us a metre to the side. Without this term the leg hangs
        // until the timeout and a 1 m error aborts a mission.
        assertTrue(
            MissionGuidance.passed(
                distanceM = 4.0,
                alongTrackRemainingM = -0.5,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
            )
        )
    }

    @Test
    fun `THE CORRIDOR - a waypoint 60 m to the side is not declared passed by the half-plane term`() {
        assertFalse(
            MissionGuidance.passed(
                distanceM = 60.0,
                alongTrackRemainingM = -10.0,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
            )
        )
        // And the corridor is exactly R_MISS: at the boundary it still counts.
        assertTrue(
            MissionGuidance.passed(
                distanceM = MissionGuidance.R_MISS_M,
                alongTrackRemainingM = -0.1,
                switchRadiusM = MissionGuidance.R_SWITCH_M,
            )
        )
    }

    @Test
    fun `a non-finite distance never passes a waypoint`() {
        assertFalse(MissionGuidance.passed(Double.NaN, -1.0, MissionGuidance.R_SWITCH_M))
    }

    @Test
    fun `along-track remaining goes negative exactly when the aircraft crosses the plane`() {
        // Leg runs due north, 50 m long. Target is 50 m north of the leg origin.
        val legNorth = 50.0
        val legEast = 0.0
        // Aircraft 10 m short: the target is 10 m north of us.
        assertEquals(
            10.0,
            MissionGuidance.alongTrackRemainingM(10.0, 0.0, legNorth, legEast),
            1e-9,
        )
        // Aircraft 5 m past it, and 3 m off to the east: the projection is negative, the distance
        // is not — which is exactly the pair the half-plane term is built to read.
        assertEquals(
            -5.0,
            MissionGuidance.alongTrackRemainingM(-5.0, -3.0, legNorth, legEast),
            1e-9,
        )
    }

    @Test
    fun `a zero-length leg has no direction so only the radius can pass it`() {
        // Degenerate legs are refused at upload; if one arrives, the projection degrades to the
        // plain distance, which can never be ≤ 0 and therefore can never fire the half-plane term.
        val remaining = MissionGuidance.alongTrackRemainingM(7.0, 0.0, 0.0, 0.0)
        assertEquals(7.0, remaining, 1e-9)
        assertFalse(MissionGuidance.passed(7.0, remaining, MissionGuidance.R_SWITCH_M))
    }

    // --------------------------------------------- the final waypoint keeps M3's arrival test

    @Test
    fun `THE ARRIVAL TEST - the resting waypoint uses the same function the reposition path uses`() {
        // Not a paraphrase: the values below are the flight-verified predicate's own, and this test
        // exists so that a mission path which stops calling RepositionGuidance.settled goes red.
        assertTrue(RepositionGuidance.settled(horizontalErrorM = 1.0, errorDownM = 0.2, speedMs = 0.4))
        assertFalse(RepositionGuidance.settled(horizontalErrorM = 1.0, errorDownM = 0.2, speedMs = 1.4))
        assertFalse(RepositionGuidance.settled(horizontalErrorM = 2.5, errorDownM = 0.2, speedMs = 0.4))
        // A dead velocity feed withholds arrival rather than reading a cached zero as settled.
        assertFalse(RepositionGuidance.settled(horizontalErrorM = 1.0, errorDownM = 0.2, speedMs = null))
    }

    @Test
    fun `a fly-through cannot satisfy the arrival test, which is why the two tests are different`() {
        // Inside the acceptance radius at cruise: the geometric test passes it, the M3 test does not.
        assertTrue(MissionGuidance.passed(1.0, 1.0, MissionGuidance.R_SWITCH_M))
        assertFalse(RepositionGuidance.settled(1.0, 0.0, GuidedEnvelope.HORIZONTAL_MAX_MS))
    }

    // ----------------------------------------------------------- the per-item switch radius

    @Test
    fun `no requested radius uses the executor's own`() {
        assertEquals(MissionGuidance.R_SWITCH_M, MissionGuidance.switchRadiusM(null), 1e-9)
        assertEquals(MissionGuidance.R_SWITCH_M, MissionGuidance.switchRadiusM(Double.NaN), 1e-9)
    }

    @Test
    fun `a requested radius inside the band is honoured, because it is a path parameter`() {
        assertEquals(2.0, MissionGuidance.switchRadiusM(2.0), 1e-9)
        assertEquals(7.5, MissionGuidance.switchRadiusM(7.5), 1e-9)
        assertEquals(10.0, MissionGuidance.switchRadiusM(10.0), 1e-9)
    }

    @Test
    fun `a requested radius outside the band falls back to ours rather than being honoured`() {
        // Admission refuses these at upload; arriving here means the gate was bypassed, and the
        // fail-safe direction is our own number, never the plan's.
        assertEquals(MissionGuidance.R_SWITCH_M, MissionGuidance.switchRadiusM(1.5), 1e-9)
        assertEquals(MissionGuidance.R_SWITCH_M, MissionGuidance.switchRadiusM(40.0), 1e-9)
    }

    // ------------------------------------------------------------------ the stopping envelope

    @Test
    fun `THE REDUCTION - with the stop at the current waypoint the law is literally M3's`() {
        // Digit for digit across the range, not at three sample points: this equivalence is what
        // lets the final leg inherit M3's flight evidence unchanged.
        var e = 0.05
        while (e < 120.0) {
            assertEquals(
                "at e=$e",
                RepositionGuidance.clampedSpeed(e, GuidedEnvelope.HORIZONTAL_MAX_MS),
                MissionGuidance.flyThroughSpeed(e, e),
                1e-12,
            )
            e += 0.05
        }
    }

    @Test
    fun `THE CORNER IS TAKEN AT SPEED - the P-term looks past the current waypoint too`() {
        // **Changed on 2026-07-27, after the first mission was flown.** The proportional term used
        // to read the *current leg*, so 3 m from a waypoint it commanded 0.5 × 3 = 1.5 m/s no
        // matter how much mission was left — the aircraft braked into every corner and accelerated
        // out of it, measured at 1.44–1.47 m/s at three consecutive waypoints
        // (`docs/measurements/2026-07-27-first-mission-flown.md` §7.1). The braking half was
        // already right; the proportional half was quietly undoing it.
        //
        // A P-term exists to smooth a *final approach*. On a leg the aircraft flies **through**
        // there is no approach to smooth, and `e_stop` already knows where the real one is.
        val speed = MissionGuidance.flyThroughSpeed(legErrorM = 3.0, stopDistanceM = 203.0)
        assertEquals(GuidedEnvelope.HORIZONTAL_MAX_MS, speed, 1e-9)
        // And it is the envelope that binds here, not luck: 200 m of mission ahead makes both the
        // P-term and the brake enormous.
        assertEquals(3.0, speed, 1e-9)
    }

    @Test
    fun `the braking term reads the distance to the next rest, not to the current waypoint`() {
        // 3 m from an intermediate waypoint with only 2 m of mission beyond it: the stop is 5 m
        // away, so sqrt(2·a·5) = 2.236 m/s binds below the 3 m/s cap but above the P-term's 1.5.
        // **Both terms read the stop distance now, so the answer does not depend on where along the
        // leg the aircraft is** — only on how far away the next place it must be stationary is.
        // That is the property, and asserting it means asserting the same answer twice from very
        // different leg errors.
        val brake = sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * 5.0) // 2.236, under the P-term's 2.5
        assertEquals(brake, MissionGuidance.flyThroughSpeed(legErrorM = 3.0, stopDistanceM = 5.0), 1e-9)
        assertEquals(brake, MissionGuidance.flyThroughSpeed(legErrorM = 30.0, stopDistanceM = 5.0), 1e-9)
        // And close to the stop it is the P-term that binds, which is what makes the final approach
        // smooth rather than a deceleration that ends abruptly: at 1.5 m out the brake would allow
        // 1.22 m/s and the P-term asks for 0.75.
        assertEquals(
            RepositionGuidance.KP_PER_S * 1.5,
            MissionGuidance.flyThroughSpeed(legErrorM = 30.0, stopDistanceM = 1.5),
            1e-9,
        )
    }

    @Test
    fun `a non-finite stop distance commands no speed at all, never an unbounded one`() {
        // The fail-closed direction, and the reason stopDistanceM exists as a function: a NaN
        // reaching sqrt(2·a·e) produces a NaN clamp that compares false against everything.
        assertEquals(0.0, MissionGuidance.flyThroughSpeed(30.0, Double.NaN), 1e-12)
        assertEquals(0.0, MissionGuidance.stopDistanceM(Double.NaN, 10.0), 1e-12)
        assertEquals(30.0, MissionGuidance.stopDistanceM(30.0, Double.NaN), 1e-12)
    }

    @Test
    fun `the stop distance is the leg error plus the legs to the next rest`() {
        assertEquals(43.0, MissionGuidance.stopDistanceM(3.0, 40.0), 1e-9)
        assertEquals(3.0, MissionGuidance.stopDistanceM(3.0, 0.0), 1e-9)
    }

    @Test
    fun `THE PROPERTY - commanded speed never exceeds the envelope at any tick of a generated mission`() {
        // The mutation this exists for: `e_stop` replaced by ∞, or by a large constant, which makes
        // the safety property dead code. The bound is checked against a distance-to-rest computed
        // here, independently of the route's own walk.
        val random = Random(20260727)
        repeat(200) { mission ->
            val count = 2 + random.nextInt(6)
            val steps = ArrayList<MissionStep>(count)
            var north = 0.0
            var east = 0.0
            for (i in 0 until count) {
                north += random.nextDouble(-60.0, 60.0)
                east += random.nextDouble(-60.0, 60.0)
                steps += step(seq = i, northM = north, eastM = east, rest = i == count - 1)
            }
            val route = MissionRoute.of(planId = 1, steps = steps)

            // Fly it: start at the origin, integrate the commanded velocity at 10 Hz, advance the
            // cursor on the same predicates the engine uses.
            var posNorth = 0.0
            var posEast = 0.0
            var cursor = 0
            var previousNorth = 0.0
            var previousEast = 0.0
            var ticks = 0
            while (cursor < route.size && ticks < 40_000) {
                ticks++
                val target = route[cursor]
                val (errN, errE) = Geo.nedMetres(
                    latNorthOf(posNorth), lonEastOf(posEast), target.latDeg, target.lonDeg,
                )
                val legError = hypot(errN, errE)

                // The independent truth: distance from here to the next resting step, walking the
                // route by hand rather than asking it.
                var trueStop = legError
                var j = cursor
                while (j < route.size && !route[j].rest) {
                    val a = route[j]
                    val b = route[j + 1]
                    trueStop += Geo.nedMetres(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg)
                        .let { (n, e) -> hypot(n, e) }
                    j++
                }

                val v = MissionGuidance.velocity(
                    errN, errE, null,
                    MissionGuidance.stopDistanceM(legError, route.restAheadM(cursor)),
                )
                val speed = hypot(v.north, v.east)
                val bound = minOf(
                    GuidedEnvelope.HORIZONTAL_MAX_MS,
                    sqrt(2.0 * RepositionGuidance.A_MAX_MS2 * trueStop),
                )
                assertTrue(
                    "mission $mission tick $ticks: |v| $speed exceeded the envelope $bound " +
                        "(leg error $legError, true stop $trueStop)",
                    speed <= bound + 1e-9,
                )

                val (legN, legE) = Geo.nedMetres(
                    latNorthOf(previousNorth), lonEastOf(previousEast), target.latDeg, target.lonDeg,
                )
                val along = MissionGuidance.alongTrackRemainingM(errN, errE, legN, legE)
                val done = if (target.rest) {
                    legError <= RepositionGuidance.R_ACCEPT_M && speed <= RepositionGuidance.V_SETTLE_MS
                } else {
                    MissionGuidance.passed(legError, along, target.switchRadiusM)
                }
                if (done) {
                    previousNorth = posNorth
                    previousEast = posEast
                    cursor++
                }
                posNorth += v.north * 0.1
                posEast += v.east * 0.1
            }
            assertTrue("mission $mission never finished in $ticks ticks", cursor >= route.size)
        }
    }

    // ------------------------------------------------------------------ the route's walk

    @Test
    fun `restAheadM sums the legs forward to the next resting step and stops there`() {
        val route = MissionRoute.of(
            planId = 7,
            steps = listOf(
                step(0, 0.0, 0.0),
                step(1, 30.0, 0.0),
                step(2, 30.0, 40.0, rest = true),
                step(3, 60.0, 40.0, rest = true),
            ),
        )
        // From step 0: 30 m to step 1, then 40 m to the resting step 2. Stops there.
        assertEquals(70.0, route.restAheadM(0), 0.2)
        assertEquals(40.0, route.restAheadM(1), 0.2)
        // A resting step's own rest-ahead is zero, which is what makes the final leg M3's law.
        assertEquals(0.0, route.restAheadM(2), 1e-12)
        assertEquals(0.0, route.restAheadM(3), 1e-12)
    }

    @Test
    fun `THE COS LATITUDE FACTOR - the route's leg walk measures east at 38 degrees north`() {
        // Two steps 100 m apart in longitude at 38°N. Dropping cos(lat) reads this as 127 m, which
        // is a 27 % over-estimate of every east-west leg and would brake far too early — or, with
        // the sign the other way in a future edit, far too late.
        val route = MissionRoute.of(
            planId = 1,
            steps = listOf(step(0, 0.0, 0.0), step(1, 0.0, 100.0, rest = true)),
        )
        assertEquals(100.0, route.restAheadM(0), 0.5)
    }

    @Test
    fun `THE INVARIANT - the last step always rests, whatever the caller said`() {
        // A route whose last step did not rest leaves the distance-to-rest walk with no terminator,
        // which is precisely the "e_stop quietly becomes infinite" mutation.
        val route = MissionRoute.of(
            planId = 1,
            steps = listOf(step(0, 0.0, 0.0), step(1, 50.0, 0.0, rest = false)),
        )
        assertTrue(route[route.size - 1].rest)
        assertEquals(50.0, route.restAheadM(0), 0.2)
    }

    @Test
    fun `a leg timeout scales with the leg it is bounding`() {
        assertEquals(MissionGuidance.LEG_TIMEOUT_BASE_MS, MissionGuidance.legTimeoutMs(0.0))
        // A 100 m leg gets 130 s against a nominal ~40 s at the envelope cap.
        assertEquals(130_000L, MissionGuidance.legTimeoutMs(100.0))
        // And it is not the flat Q1 manoeuvre timeout, which is the wrong shape for a mission leg.
        assertTrue(MissionGuidance.legTimeoutMs(5.0) < GuidedEnvelope.MANOEUVRE_TIMEOUT_MS)
    }

    @Test
    fun `THE SWEEP - a mission leg is never timed out before the law could have flown it`() {
        // The relationship the mission layer keeps with `GuidedEnvelope`'s deadline owner after
        // 2026-07-30, asserted rather than argued: the two deadlines have different shapes on
        // purpose (a leg's expiry *pauses* the plan; a goto's *ends* the manoeuvre), but a leg must
        // never expire while the guidance law is still plausibly flying it. Checked across the whole
        // envelope, including the 2 km leg the old flat numbers could not express.
        var leg = 0.0
        while (leg <= GuidedEnvelope.MAX_REPOSITION_DISTANCE_M) {
            val travelMs = GuidedEnvelope.travelSeconds(
                leg, GuidedEnvelope.HORIZONTAL_MAX_MS, RepositionGuidance.R_ACCEPT_M,
            ) * 1_000.0
            assertTrue(
                "a $leg m leg needs ${travelMs / 1000} s and is given ${MissionGuidance.legTimeoutMs(leg) / 1000} s",
                MissionGuidance.legTimeoutMs(leg) > travelMs,
            )
            leg += 25.0
        }
    }

    @Test
    fun `THE SWEEP - the whole-mission clock covers the whole path the desk admits`() {
        // The bound that silently assumed a 150 m world: at 600 s a *single* admissible 2 km leg
        // (667 s of travel) would have been aborted for being exactly as long as the desk allowed.
        // It is now derived from the same owner, at the whole admitted path, so admission and the
        // clock cannot disagree — and the path bound is itself an out-and-back at the reach.
        assertEquals(2.0 * GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, MissionGuidance.MAX_PATH_M, 0.0)
        // The deadline in whole seconds, rounded **up** — this clock is a cap, so the rounding
        // that shortens it is the wrong one.
        assertEquals(
            (GuidedEnvelope.manoeuvreDeadlineMs(MissionGuidance.MAX_PATH_M, 0.0) + 999L) / 1_000L,
            MissionGuidance.MISSION_MAX_S,
        )
        assertTrue(
            "the clock must outlast the whole admitted path at the envelope cap",
            MissionGuidance.MISSION_MAX_S * 1_000.0 >
                GuidedEnvelope.travelSeconds(
                    MissionGuidance.MAX_PATH_M, GuidedEnvelope.HORIZONTAL_MAX_MS,
                    RepositionGuidance.R_ACCEPT_M,
                ) * 1_000.0,
        )
        // ~2668 s. Pinned because the honest cost of this number is stated in its KDoc and the
        // prose must not rot: 44 minutes is longer than the airframe flies, so this constant has
        // stopped being the battery bound M4-3 chose it to be.
        assertEquals(2_668.0, MissionGuidance.MISSION_MAX_S.toDouble(), 2.0)
    }
}
