package com.dimensional.mini4pro.guided

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Every conversion and every sign between a `MANUAL_CONTROL` frame and a DJI virtual-stick
 * param — landmines 1 (DJI's commanded vertical is up-positive), 2 (QGC's z convention), and
 * the Q1 rule that full deflection is the envelope maximum, never beyond.
 *
 * Mutation-checked 2026-07-26 — each breakage made deliberately, failing tests counted across
 * both guided suites (this one and `GuidedStickEngineTest`, which drives the same mapping end
 * to end), code reverted:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | vertical sign flip: `toDji` uses `verticalThrottle = v.down` (no negation) | 6 |
 *  | z centre dropped: `velocities` reads z as 0-centred (`-f.z / 500.0 * MAX`) | 12 |
 *  | axes swapped: `toDji` puts north on `pitch`, east on `roll` (the pre-measurement mapping — reverted by the 2026-07-26 bench, which measured pitch→EAST, roll→NORTH) | 6 |
 *  | envelope removed: horizontal mapping divides by 1000 only (1 m/s full scale) | 9 |
 *  | envelope exceeded: horizontal scale doubled | 10 |
 *  | z range check removed from `read` (negative z interpreted) | 3 |
 *  | INT16_MAX sentinel check removed | **0 — equivalent mutant, see below** |
 *  | neutral z band checked around 0 instead of 500 | 41 |
 *  | `isDeliberate` z test dropped | 2 |
 *  | yaw sign flip (`yawRateDegPerS = -r * …`) | 2 |
 *
 * The INT16_MAX row is an **equivalent mutant, verified rather than a hole**: with the
 * sentinel branch removed, 32767 still fails the ±1000 range checks on every axis, so the
 * frame is refused either way and only the logged reason string changes. The branch stays in
 * the code because MAVLink defines INT16_MAX as *axis invalid* — a different fact from
 * out-of-range, worth naming in the log — the same reasoning `CommandDispatcher` records for
 * its own equivalent mutant.
 *
 * ## The 2 km envelope and its distance-derived deadline (2026-07-30) — measured kill counts
 *
 * Ivan's *"2 km is our new limit"* (`docs/decisions/2026-07-30-two-kilometre-envelope.md`), which
 * moved [GuidedEnvelope.MAX_REPOSITION_DISTANCE_M] and [MissionGuidance.MAX_HOME_DIST_M] to 2000 m
 * and turned the flat manoeuvre timeout into [GuidedEnvelope.manoeuvreDeadlineMs] with the flat
 * constant as its floor. Same protocol: one breakage at a time against the **2633-test** tree,
 * whole suite per mutant, `test-results` deleted first, confirmed red, reverted. **No survivors.**
 * The mission-layer rows are the same campaign and are summarised again in `MissionBig1PlanTest`,
 * which is this decision's acceptance test.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | leg bound dropped (any distance admitted between two items) | 3 |
 *  | home bound dropped at Start (`MissionLaunch`'s distance check) | 3 |
 *  | **the distance-derived deadline reverted to the flat 150 s** | 4 |
 *  | the deadline's short-move floor dropped (derived value used raw) | 5 |
 *  | `legTooLong` stops naming the limit ("Leg 5 is too long") | 4 |
 *  | `REASON_TOO_FAR` stops naming the limit ("target too far") | 2 |
 *  | the reach bound reverted to 100 m — **the spot-check mutant** | 11 |
 *
 * Two things worth keeping from the campaign. The **flat-deadline** mutant is killed by
 * `GuidedRepositionTest`'s long-leg test *and* by the whole-mission clock, which is the coupling
 * working: the mission cap is derived from the same owner, so flattening the owner breaks the layer
 * above it too. The **floor** mutant kills a test that has nothing to do with distance
 * (`GuidedRoiTest`'s holding-station-with-an-ROI), because without the floor a short goto's
 * deadline collapses to seconds — which is exactly the regression the floor exists to prevent, found
 * by a test nobody wrote for it.
 *
 * One first-pass run of the leg-bound mutant also reported `TagRecogniserTest` failing; it did not
 * recur on the re-measured campaign and is **not** counted above — an unrelated flake, recorded
 * here rather than quietly dropped.
 */
class StickMappingTest {

    // ------------------------------------------------------------------ read()

    @Test
    fun `a neutral QGC frame reads valid`() {
        val r = StickMapping.read(0, 0, 500, 0)
        assertTrue(r is StickMapping.Reading.Valid)
    }

    @Test
    fun `INT16_MAX on any axis is refused - MAVLink's axis-invalid sentinel`() {
        for (frame in listOf(
            StickMapping.read(32767, 0, 500, 0),
            StickMapping.read(0, 32767, 500, 0),
            StickMapping.read(0, 0, 32767, 0),
            StickMapping.read(0, 0, 500, 32767),
        )) {
            assertTrue("expected refusal, got $frame", frame is StickMapping.Reading.Unreadable)
        }
    }

    @Test
    fun `negative z is refused, never read as a descent`() {
        // The −1000..1000 z convention: its neutral (0) would read as a half-scale descent
        // under our 500-centre rule, so nothing below 0 may ever be interpreted.
        val r = StickMapping.read(0, 0, -500, 0)
        assertTrue(r is StickMapping.Reading.Unreadable)
        assertTrue((r as StickMapping.Reading.Unreadable).reason.contains("z="))
    }

    @Test
    fun `z above 1000 is refused`() {
        assertTrue(StickMapping.read(0, 0, 1500, 0) is StickMapping.Reading.Unreadable)
    }

    @Test
    fun `x y r outside plus-minus 1000 are refused`() {
        assertTrue(StickMapping.read(1001, 0, 500, 0) is StickMapping.Reading.Unreadable)
        assertTrue(StickMapping.read(0, -1001, 500, 0) is StickMapping.Reading.Unreadable)
        assertTrue(StickMapping.read(0, 0, 500, 2000) is StickMapping.Reading.Unreadable)
    }

    @Test
    fun `full-scale extremes are valid`() {
        assertTrue(StickMapping.read(1000, -1000, 0, 1000) is StickMapping.Reading.Valid)
        assertTrue(StickMapping.read(-1000, 1000, 1000, -1000) is StickMapping.Reading.Valid)
    }

    // -------------------------------------------------------------- neutrality

    @Test
    fun `neutral means all axes at rest including z at its 500 centre`() {
        assertTrue(StickMapping.isNeutral(GcsStickFrame(0, 0, 500, 0)))
        assertTrue(StickMapping.isNeutral(GcsStickFrame(30, -30, 530, 30)))
    }

    @Test
    fun `z at 0 is NOT neutral - the centre-zero regime must never look at rest`() {
        // This is the whole guard on QGC's opt-in centre-zero throttle: its idle frame is
        // z = 0, which must read as a deflection, so the engagement gate never opens on it.
        assertFalse(StickMapping.isNeutral(GcsStickFrame(0, 0, 0, 0)))
    }

    @Test
    fun `deliberate requires a tenth of travel, drift does not qualify`() {
        assertFalse(StickMapping.isDeliberate(GcsStickFrame(90, 0, 500, 0)))
        assertTrue(StickMapping.isDeliberate(GcsStickFrame(101, 0, 500, 0)))
        assertTrue(StickMapping.isDeliberate(GcsStickFrame(0, -101, 500, 0)))
        assertTrue(StickMapping.isDeliberate(GcsStickFrame(0, 0, 500, 101)))
        // z deviates from the 500 centre, both directions.
        assertTrue(StickMapping.isDeliberate(GcsStickFrame(0, 0, 601, 0)))
        assertTrue(StickMapping.isDeliberate(GcsStickFrame(0, 0, 399, 0)))
    }

    // ------------------------------------------------------------- velocities()

    @Test
    fun `neutral frame maps to zero velocity on every axis`() {
        val v = StickMapping.velocities(GcsStickFrame(0, 0, 500, 0))
        assertEquals(0.0, v.north, 1e-9)
        assertEquals(0.0, v.east, 1e-9)
        assertEquals(0.0, v.down, 1e-9)
        assertEquals(0.0, v.yawRateDegPerS, 1e-9)
    }

    @Test
    fun `full forward stick is exactly the 3 m per s envelope north - never beyond`() {
        val v = StickMapping.velocities(GcsStickFrame(1000, 0, 500, 0))
        assertEquals(GuidedEnvelope.HORIZONTAL_MAX_MS, v.north, 1e-9)
        assertEquals(3.0, v.north, 1e-9) // the Q1 number itself, so an envelope edit is loud
        assertEquals(0.0, v.east, 1e-9)
        assertEquals(0.0, v.down, 1e-9)
    }

    @Test
    fun `full right stick is exactly the envelope east`() {
        val v = StickMapping.velocities(GcsStickFrame(0, 1000, 500, 0))
        assertEquals(3.0, v.east, 1e-9)
        assertEquals(0.0, v.north, 1e-9)
    }

    @Test
    fun `back and left sticks are negative north and east`() {
        val v = StickMapping.velocities(GcsStickFrame(-1000, -500, 500, 0))
        assertEquals(-3.0, v.north, 1e-9)
        assertEquals(-1.5, v.east, 1e-9)
    }

    @Test
    fun `full up throttle z=1000 is a climb - down is MINUS the vertical envelope`() {
        // NED: climbing is negative down. z above the 500 centre is up-stick.
        val v = StickMapping.velocities(GcsStickFrame(0, 0, 1000, 0))
        assertEquals(-GuidedEnvelope.VERTICAL_MAX_MS, v.down, 1e-9)
        assertEquals(-1.5, v.down, 1e-9)
    }

    @Test
    fun `full down throttle z=0 is a descent - down is PLUS the vertical envelope`() {
        val v = StickMapping.velocities(GcsStickFrame(0, 0, 0, 0))
        assertEquals(+1.5, v.down, 1e-9)
    }

    @Test
    fun `half deflections scale linearly`() {
        val v = StickMapping.velocities(GcsStickFrame(500, -500, 750, 500))
        assertEquals(1.5, v.north, 1e-9)
        assertEquals(-1.5, v.east, 1e-9)
        assertEquals(-0.75, v.down, 1e-9)
        // Derived, not a literal: half deflection is half the envelope, whatever the envelope is.
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS / 2.0, v.yawRateDegPerS, 1e-9)
    }

    @Test
    fun `yaw stick right is a positive - clockwise - yaw rate at the envelope maximum`() {
        // QGC/PX4 reading of MANUAL_CONTROL r: +1000 = stick right = clockwise looking down,
        // which is also DJI's positive yaw (getLeftStick prose). Not the MAVLink field prose.
        val v = StickMapping.velocities(GcsStickFrame(0, 0, 500, 1000))
        assertEquals(GuidedEnvelope.YAW_RATE_MAX_DEGS, v.yawRateDegPerS, 1e-9)
        val left = StickMapping.velocities(GcsStickFrame(0, 0, 500, -1000))
        assertEquals(-GuidedEnvelope.YAW_RATE_MAX_DEGS, left.yawRateDegPerS, 1e-9)
    }

    // ------------------------------------------------------------------ toDji()

    @Test
    fun `a commanded climb becomes a POSITIVE verticalThrottle - DJI's axis is up-positive`() {
        // THE most dangerous sign in M3. down = -1.5 (climb, NED) must reach DJI as +1.5,
        // because VirtualStickRange documents positive = "fly upwards".
        val axes = StickMapping.toDji(StickVelocities(0.0, 0.0, down = -1.5, yawRateDegPerS = 0.0))
        assertEquals(+1.5, axes.verticalThrottle!!, 1e-9)
    }

    @Test
    fun `a commanded descent becomes a NEGATIVE verticalThrottle`() {
        val axes = StickMapping.toDji(StickVelocities(0.0, 0.0, down = +1.5, yawRateDegPerS = 0.0))
        assertEquals(-1.5, axes.verticalThrottle!!, 1e-9)
    }

    @Test
    fun `north rides DJI roll and east rides DJI pitch - GROUND frame, MEASURED 2026-07-26`() {
        // DJI's naming trap, settled on hardware (record 20260726-204721.001, first Stage A
        // engagement): under GROUND + VELOCITY, `pitch` drives EAST and `roll` drives NORTH —
        // the exact swap of the pre-measurement assumption. Commanded (N+3, E−3) flew
        // (N−3, E+3) at yaw 0°, 45° and 116° until the swap; see
        // docs/measurements/2026-07-26-stage-a-first-engagement.md.
        val axes = StickMapping.toDji(StickVelocities(north = 2.0, east = -1.0, down = 0.0, yawRateDegPerS = 5.0))
        assertEquals(-1.0, axes.pitch!!, 1e-9)
        assertEquals(2.0, axes.roll!!, 1e-9)
        assertEquals(5.0, axes.yaw!!, 1e-9)
        assertEquals(0.0, axes.verticalThrottle!!, 1e-9)
    }

    @Test
    fun `wire frame to DJI axes end to end - full up-stick climbs, full down-stick descends`() {
        // Both directions through the whole chain, because the two halves' signs could cancel
        // a double error in a single-direction test.
        val climb = StickMapping.toDji(StickMapping.velocities(GcsStickFrame(0, 0, 1000, 0)))
        assertTrue("z=1000 must reach DJI as a positive (up) throttle", climb.verticalThrottle!! > 0)
        assertEquals(+1.5, climb.verticalThrottle!!, 1e-9)

        val descend = StickMapping.toDji(StickMapping.velocities(GcsStickFrame(0, 0, 0, 0)))
        assertTrue("z=0 must reach DJI as a negative (down) throttle", descend.verticalThrottle!! < 0)
        assertEquals(-1.5, descend.verticalThrottle!!, 1e-9)
    }

    @Test
    fun `no input can exceed the envelope on any axis`() {
        // The mapping is a multiplication by the limit — but pin the property itself, not the
        // mechanism, across the extreme corners of the valid input space.
        for (x in intArrayOf(-1000, 1000)) for (y in intArrayOf(-1000, 1000)) {
            for (z in intArrayOf(0, 1000)) for (r in intArrayOf(-1000, 1000)) {
                val v = StickMapping.velocities(GcsStickFrame(x, y, z, r))
                assertTrue(kotlin.math.abs(v.north) <= GuidedEnvelope.HORIZONTAL_MAX_MS + 1e-9)
                assertTrue(kotlin.math.abs(v.east) <= GuidedEnvelope.HORIZONTAL_MAX_MS + 1e-9)
                assertTrue(kotlin.math.abs(v.down) <= GuidedEnvelope.VERTICAL_MAX_MS + 1e-9)
                assertTrue(kotlin.math.abs(v.yawRateDegPerS) <= GuidedEnvelope.YAW_RATE_MAX_DEGS + 1e-9)
            }
        }
    }

    @Test
    fun `scaled shrinks every axis together - the ramp's primitive`() {
        val v = StickVelocities(3.0, -1.5, 1.5, 30.0).scaled(0.5)
        assertEquals(1.5, v.north, 1e-9)
        assertEquals(-0.75, v.east, 1e-9)
        assertEquals(0.75, v.down, 1e-9)
        // A literal on purpose here: `scaled` is given a literal 30 °/s, so this is arithmetic on
        // the input rather than anything to do with the envelope.
        assertEquals(15.0, v.yawRateDegPerS, 1e-9)
    }

    @Test
    fun `the envelope constants are the Q1 decisions, as amended by Ivan`() {
        // Changing the safety argument must fail a named test, not slip through as tuning.
        assertEquals(3.0, GuidedEnvelope.HORIZONTAL_MAX_MS, 0.0)
        assertEquals(1.5, GuidedEnvelope.VERTICAL_MAX_MS, 0.0)
        assertEquals(100.0, GuidedEnvelope.CEILING_M, 0.0)
        // 2026-07-30, Ivan: "2 km is our new limit". Was 100 m (M3 Q1).
        assertEquals(2000.0, GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, 0.0)
        // Unchanged by that decision, and now the *floor* under a derived deadline rather than the
        // whole of it — see the coupling tests below.
        assertEquals(150_000L, GuidedEnvelope.MANOEUVRE_TIMEOUT_MS)
        assertEquals(2.0, GuidedEnvelope.MANOEUVRE_MARGIN, 0.0)
        assertEquals(300_000L, GuidedEnvelope.IDLE_DISENGAGE_MS)
    }

    // ------------------------------------------------------ the manoeuvre deadline, and the coupling

    @Test
    fun `travelSeconds integrates the guidance law itself - checked against the closed form`() {
        // The production code integrates `RepositionGuidance.clampedSpeed` numerically so that
        // there is exactly one speed model in the bridge. This test carries the algebra as an
        // **independent witness**: with k_p = 0.5, a_max = 0.5 and cap = 3.0 the law's regimes are
        // e < 4 (proportional), 4..9 (braking curve), e > 9 (envelope cap), so
        //
        //     t(D) = 2·ln(4/2) + 2·(√9 − √4) + (D − 9)/3     for D ≥ 9, down to R_ACCEPT_M = 2
        //
        // If the quadrature and the algebra disagree, one of them is wrong and this says so.
        val kink = 2.0 * RepositionGuidance.A_MAX_MS2 / (RepositionGuidance.KP_PER_S * RepositionGuidance.KP_PER_S)
        val capAt = GuidedEnvelope.HORIZONTAL_MAX_MS * GuidedEnvelope.HORIZONTAL_MAX_MS /
            (2.0 * RepositionGuidance.A_MAX_MS2)
        assertEquals(4.0, kink, 1e-12)
        assertEquals(9.0, capAt, 1e-12)
        fun closedForm(distanceM: Double): Double =
            2.0 * kotlin.math.ln(kink / RepositionGuidance.R_ACCEPT_M) +
                2.0 * (kotlin.math.sqrt(capAt) - kotlin.math.sqrt(kink)) +
                (distanceM - capAt) / GuidedEnvelope.HORIZONTAL_MAX_MS

        for (distance in listOf(10.0, 50.0, 100.0, 450.0, 1000.0, 2000.0)) {
            val quadrature = GuidedEnvelope.travelSeconds(
                distance, GuidedEnvelope.HORIZONTAL_MAX_MS, RepositionGuidance.R_ACCEPT_M,
            )
            assertEquals(
                "the deadline's integral must agree with the law's algebra at $distance m",
                closedForm(distance), quadrature, closedForm(distance) * 1e-4,
            )
        }

        // The two numbers the KDocs quote, pinned so the prose cannot rot: the full reach is
        // ~667 s of travel and a ground-to-ceiling climb is ~67 s — the latter being the 2026-07-27
        // hand-computed figure, now reproduced by the law rather than asserted about it.
        assertEquals(
            667.0,
            GuidedEnvelope.travelSeconds(
                GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, GuidedEnvelope.HORIZONTAL_MAX_MS,
                RepositionGuidance.R_ACCEPT_M,
            ),
            1.0,
        )
        assertEquals(
            67.0,
            GuidedEnvelope.travelSeconds(
                GuidedEnvelope.CEILING_M, GuidedEnvelope.VERTICAL_MAX_MS,
                RepositionGuidance.VERTICAL_ACCEPT_M,
            ),
            1.0,
        )

        // Nothing to travel, and garbage, are both zero — the fail-closed direction, because the
        // caller turns zero travel into the flat floor rather than into no bound at all.
        assertEquals(0.0, GuidedEnvelope.travelSeconds(0.0, 3.0, 2.0), 0.0)
        assertEquals(0.0, GuidedEnvelope.travelSeconds(1.5, 3.0, 2.0), 0.0)
        assertEquals(0.0, GuidedEnvelope.travelSeconds(Double.NaN, 3.0, 2.0), 0.0)
        assertEquals(0.0, GuidedEnvelope.travelSeconds(500.0, Double.NaN, 2.0), 0.0)
        assertEquals(0.0, GuidedEnvelope.travelSeconds(500.0, 3.0, 0.0), 0.0)
    }

    @Test
    fun `THE COUPLING - the deadline covers every manoeuvre the envelope permits, at every distance`() {
        // The successor to the 2026-07-27 relation test, which compared the flat timeout against
        // the single longest manoeuvre the envelope then allowed. At a 2 km reach one number cannot
        // do that job, so the property is now asserted **at every distance**: whatever the envelope
        // admits, the deadline covers it with the stated margin. A mutant that flattens, shortens or
        // rescales the derivation fails here.
        var distance = 0.0
        while (distance <= GuidedEnvelope.MAX_REPOSITION_DISTANCE_M) {
            val travelMs = GuidedEnvelope.travelSeconds(
                distance, GuidedEnvelope.HORIZONTAL_MAX_MS, RepositionGuidance.R_ACCEPT_M,
            ) * 1_000.0
            val deadline = GuidedEnvelope.manoeuvreDeadlineMs(distance, 0.0)
            assertTrue(
                "a $distance m leg needs ${travelMs / 1000} s and was given ${deadline / 1000} s",
                deadline >= travelMs * GuidedEnvelope.MANOEUVRE_MARGIN,
            )
            assertTrue(
                "no manoeuvre may get a tighter deadline than the flat Q1 floor ($distance m)",
                deadline >= GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
            )
            distance += 25.0
        }
        // The vertical axis states its own worst case from the same law: a ground-to-ceiling climb
        // is inside the floor, which is why 2026-07-27's 150 s survived this change untouched.
        assertEquals(
            GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
            GuidedEnvelope.manoeuvreDeadlineMs(0.0, GuidedEnvelope.CEILING_M),
        )
        // The axes run concurrently, so the deadline is the slower of the two and never their sum.
        assertEquals(
            GuidedEnvelope.manoeuvreDeadlineMs(GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, 0.0),
            GuidedEnvelope.manoeuvreDeadlineMs(
                GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, GuidedEnvelope.CEILING_M,
            ),
        )
    }

    @Test
    fun `THE COUPLING - what the flat 150 s would have done to a 2 km leg, measured`() {
        // The trap this change exists to avoid, stated as arithmetic rather than as a worry: the
        // flat deadline buys 150 s, which at the envelope cap is 450 m of travel — so **every leg
        // longer than that would have been aborted mid-flight, every time**, once the reach became
        // 2 km. That is strictly worse than the honest desk refusal it replaced.
        val flatReachM = GuidedEnvelope.MANOEUVRE_TIMEOUT_MS / 1000.0 * GuidedEnvelope.HORIZONTAL_MAX_MS
        assertEquals(450.0, flatReachM, 0.0)
        assertTrue(
            "a 450 m leg cannot be flown inside the flat deadline",
            GuidedEnvelope.travelSeconds(
                flatReachM, GuidedEnvelope.HORIZONTAL_MAX_MS, RepositionGuidance.R_ACCEPT_M,
            ) * 1_000.0 > GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
        )
        // And the derived deadline for the full reach, pinned: ~1334 s, i.e. 22 minutes — longer
        // than the airframe flies, which is why the KDoc says out loud that at this distance the
        // deadline has stopped being the battery's guard and DJI's RTH is the backstop.
        assertEquals(
            1_334_100.0,
            GuidedEnvelope.manoeuvreDeadlineMs(GuidedEnvelope.MAX_REPOSITION_DISTANCE_M, 0.0).toDouble(),
            100.0,
        )
        // The floor stops binding at ~224 m: below that a manoeuvre keeps exactly the 150 s it had
        // before 2026-07-30, which is what "nothing short regresses" means in numbers.
        assertEquals(
            GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
            GuidedEnvelope.manoeuvreDeadlineMs(220.0, 0.0),
        )
        assertTrue(
            GuidedEnvelope.manoeuvreDeadlineMs(230.0, 0.0) > GuidedEnvelope.MANOEUVRE_TIMEOUT_MS,
        )
    }
}
