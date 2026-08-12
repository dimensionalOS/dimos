package com.dimensional.mini4pro.guided

import com.dimensional.mini4pro.vision.RangeSource
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.hypot

/**
 * M3 Stage D's arithmetic and state machine — [TagDescentGuidance] and [TagDescent] — pinned the
 * way `OrbitGuidanceTest` pins the circle's: pure functions and a hand-cranked machine, no
 * engine, no clock, no aircraft. What the *engine* does with these is next door in
 * `GuidedTagDescentTest`.
 *
 * Written to fail loudly for the Stage D landmines:
 *
 *  - descent commanded **outside the alignment cone** — altitude bought with misalignment, on a
 *    stage whose whole point is that later stages land on this tag
 *  - descent commanded **on a stale fix** — the ladder's first rung dropped
 *  - the reacquisition climb **unbounded** — a lost tag turning into a climb-away
 *  - the terminal hold **descending below the target** — Stage C flown by accident
 *  - a fly-through declaring the stage complete
 *  - the terminal phase exiting back into a descent — a resume nobody armed
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-28, one breakage at a time, applied to the shipped source, the whole
 * suite run (`:app:testDebugUnitTest`, 2296 tests, test-results deleted first), confirmed red
 * and reverted. Counts are failing tests across the whole suite — **measured, not estimated**,
 * and re-measured against the final tree after shadow mode landed (which is why the ladder and
 * GONE rows grew: the shadow suite kills them too). The engine-side mutations for the same
 * properties are in `GuidedTagDescentTest`'s table.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | cone check dropped (descend at any lateral error) | 2 |
 *  | cone slope inverted (widens toward the ground) | 3 |
 *  | T_HOLD rung dropped (descend on a stale fix until T_CLIMB) | 7 |
 *  | reacquisition climb unbounded (no ARM_CEILING stop) | 3 |
 *  | descentRate can go past the target (no floor at TARGET_HEIGHT) | 3 |
 *  | terminal latch needs 1 tick instead of TERMINAL_TICKS | 3 |
 *  | TERMINAL exits back to TRACKING (the early terminal branch dropped) | 3 |
 *  | GONE rung dropped (the machine never hands back) | 4 |
 *
 * (An earlier pass of this campaign, before shadow mode, saw two unrelated
 * `TagRecogniserTest` thread-timing flakes in runs that touch only this file; the recorded
 * final runs contain none, and the unmutated suite was green before and after every run.)
 *
 * **Stage C (the DJI_LANDING commit, 2026-07-28 — the post-landing04 pivot)** was
 * mutation-checked by the same protocol against the 2424-test suite. The law-side rows,
 * measured (the full Stage C table, the engine's half and the two-survivors story are in
 * `GuidedAutolandTest`'s KDoc):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | floor commit: `floorStalled` conjunct dropped | 8 |
 *  | floor commit: fresh conjunct dropped (stale rungs commit) | 1 |
 *  | floor commit: cone conjunct dropped | 1 |
 *  | both commits ignore `fullAutoland` | 7 |
 *  | floor commit: ceiling conjunct dropped | 1 |
 *  | `COMMIT_CEILING_M` broken (2.5 → 25 m) | 1 |
 *  | DJI_LANDING re-emits its entry edge every tick | 2 |
 *  | DJI_LANDING still descends (the zero setpoint broken) | 4 |
 *
 * **The odometric blind final and the velocity gate (2026-07-29, landing06's features)** —
 * same protocol against the 2500-test tree, no survivors; the law-side rows (the engine's
 * half and the full table are in `GuidedAutolandTest`'s KDoc):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | committed phase commands vertical (0.0 → 0.1) | 5 |
 *  | committed phase commands yaw (0.0 → 1.0) | 4 |
 *  | `landingLateral` cap dropped (→ the 1.0 tracking cap) | 2 |
 *  | speed conjunct dropped at the TERMINAL entry | 2 |
 *  | speed conjunct dropped at the floor entry | 3 |
 *  | `LAND_COMMIT_SPEED_MS` broken (0.05 → 5) | 4 |
 *  | quantisation semantics inverted (0.05 → 0.1: one quantum passes) | 4 |
 *  | `LAND_TARGET_WINDOW_MS` stretched (0.5 s → 10 s) | 3 |
 *  | `landingTarget` fewer-than-2-samples fallback broken | 1 |
 *
 * **The range ladder (2026-07-29, landing07's features)** — same protocol against the
 * 2511-test tree; the full table, the engine's half and the one-survivor story are in
 * `GuidedAutolandTest`'s KDoc, the fix-scale rows in `TagWorldTest`'s. The law-side rows:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `descentHeight` precedence flipped (baro outranks a fresh tag range) | 6 |
 *  | `descentHeight` freshness dropped (a stale tag range flown) | 2 |
 *  | `range_baro_divergence` record dropped (engine-side) | 2 |
 *
 * **The approach (2026-07-29 — arm above the band, fly down into it; landing13 t=41.8 is the
 * measured friction it absorbs)** — same protocol against the 2623-test tree, one mutant at a
 * time, whole suite per run, test-results deleted first, every count confirmed red and
 * reverted. Law-side and engine-side rows are one campaign because the feature is one seam;
 * no survivors:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | approach ceiling dropped in the gate (arm at ANY height, no upper bound) | 2 |
 *  | the gate grants APPROACH below the band (`approach = true` for every arm) | 4 |
 *  | the machine's birth phase ignores the flag (every machine starts APPROACH) | 8 |
 *  | approach descends at any staleness (the FRESH/HOLD conjunct dropped) | 2 |
 *  | band-entry hysteresis dropped (entry at `ARM_CEILING_M` itself) | 1 |
 *  | band entry never fires (the handoff broken outright) | 4 |
 *  | above-band arm skips the RC gate (representative skipped-gate shape) | 1 |
 *
 * The skipped-gate row is one representative, argued: every non-ceiling gate runs
 * unconditionally in the single linear `descentGateLocked`, so skipping any of them for an
 * above-band arm requires manufacturing exactly the height-conditional the mutant did — and
 * the successor test exercises nadir and fix staleness at 10 m beside the RC rung it pinned
 * red. (The recorded runs contain no unrelated failures; two separate `TagRecogniserTest`
 * thread-timing flakes appeared in *unmutated* full runs during the campaign — the known
 * flake shape noted above — and both re-ran green with no mutation applied.)
 */
class TagDescentGuidanceTest {

    // ------------------------------------------------------------------ the ladder

    @Test
    fun `the ladder's rungs sit exactly at the documented bounds, inclusive on the fresh side`() {
        assertEquals(TagDescentGuidance.Rung.FRESH, TagDescentGuidance.rung(0))
        assertEquals(TagDescentGuidance.Rung.FRESH, TagDescentGuidance.rung(TagDescentGuidance.T_HOLD_MS))
        assertEquals(TagDescentGuidance.Rung.HOLD, TagDescentGuidance.rung(TagDescentGuidance.T_HOLD_MS + 1))
        assertEquals(TagDescentGuidance.Rung.HOLD, TagDescentGuidance.rung(TagDescentGuidance.T_CLIMB_MS))
        assertEquals(TagDescentGuidance.Rung.CLIMB, TagDescentGuidance.rung(TagDescentGuidance.T_CLIMB_MS + 1))
        assertEquals(TagDescentGuidance.Rung.CLIMB, TagDescentGuidance.rung(TagDescentGuidance.T_ABORT_MS))
        assertEquals(TagDescentGuidance.Rung.GONE, TagDescentGuidance.rung(TagDescentGuidance.T_ABORT_MS + 1))
    }

    @Test
    fun `the ladder is ordered - hold before climb before gone`() {
        assertTrue(TagDescentGuidance.T_HOLD_MS < TagDescentGuidance.T_CLIMB_MS)
        assertTrue(TagDescentGuidance.T_CLIMB_MS < TagDescentGuidance.T_ABORT_MS)
    }

    @Test
    fun `the arm freshness bound and the hold rung are the same claim`() {
        assertEquals(TagDescentGuidance.ARM_FRESH_MS, TagDescentGuidance.T_HOLD_MS)
    }

    // ------------------------------------------------------------------ the cone

    @Test
    fun `the cone tightens with height and never below its floor`() {
        // 8.5 deg of half-angle: 1.05 m at the 7 m ceiling.
        assertEquals(1.05, TagDescentGuidance.coneRadiusM(7.0), 1e-9)
        assertEquals(0.75, TagDescentGuidance.coneRadiusM(5.0), 1e-9)
        assertEquals(0.30, TagDescentGuidance.coneRadiusM(2.0), 1e-9)
        // Below ~1.33 m the floor binds — the sensor pair cannot resolve tighter.
        assertEquals(TagDescentGuidance.CONE_FLOOR_M, TagDescentGuidance.coneRadiusM(1.0), 1e-9)
        assertEquals(TagDescentGuidance.CONE_FLOOR_M, TagDescentGuidance.coneRadiusM(0.6), 1e-9)
        // Monotone non-increasing on the way down, across the whole descent band.
        var previous = Double.MAX_VALUE
        var h = TagDescentGuidance.ARM_CEILING_M
        while (h > 0.0) {
            val cone = TagDescentGuidance.coneRadiusM(h)
            assertTrue("cone grew on the way down at h=$h", cone <= previous)
            previous = cone
            h -= 0.1
        }
    }

    @Test
    fun `the cone dominates the measured nadir-point residual`() {
        // 2.99 deg of unseparated principal-point/gimbal error (2026-07-28). A cone comparable
        // to it would gate descent on calibration noise; the slope must sit well above it.
        val residualTan = kotlin.math.tan(Math.toRadians(2.99))
        assertTrue(TagDescentGuidance.CONE_TAN > 2.0 * residualTan)
    }

    @Test
    fun `garbage heights fall to the floor rather than to a huge cone`() {
        assertEquals(TagDescentGuidance.CONE_FLOOR_M, TagDescentGuidance.coneRadiusM(Double.NaN), 1e-9)
        assertEquals(TagDescentGuidance.CONE_FLOOR_M, TagDescentGuidance.coneRadiusM(-3.0), 1e-9)
    }

    // ------------------------------------------------------------------ the laws

    @Test
    fun `lateral keeps the direction and clamps the magnitude at V_LATERAL_MAX`() {
        val (n, e) = TagDescentGuidance.lateral(30.0, 40.0)
        // Direction preserved: 3-4-5 triangle.
        assertEquals(0.6, n / hypot(n, e), 1e-9)
        assertEquals(0.8, e / hypot(n, e), 1e-9)
        assertEquals(TagDescentGuidance.V_LATERAL_MAX_MS, hypot(n, e), 1e-9)
    }

    @Test
    fun `lateral is the M3 clamped-speed law - proportional close in`() {
        val (n, _) = TagDescentGuidance.lateral(0.4, 0.0)
        // kp * 0.4 = 0.2, under both the cap and the braking curve sqrt(2*0.5*0.4)=0.63.
        assertEquals(RepositionGuidance.KP_PER_S * 0.4, n, 1e-9)
    }

    @Test
    fun `garbage lateral errors hover`() {
        assertEquals(Pair(0.0, 0.0), TagDescentGuidance.lateral(Double.NaN, 1.0))
        assertEquals(Pair(0.0, 0.0), TagDescentGuidance.lateral(1.0, Double.POSITIVE_INFINITY))
        assertEquals(Pair(0.0, 0.0), TagDescentGuidance.lateral(0.0, 0.0))
    }

    @Test
    fun `descentRate is capped, brakes into the target, and is zero at and below it`() {
        // High up: the cap binds.
        assertEquals(TagDescentGuidance.V_DESCENT_MAX_MS, TagDescentGuidance.descentRate(7.0), 1e-9)
        // Close in: kp * remaining error — the braking shape, not a step to zero.
        assertEquals(RepositionGuidance.KP_PER_S * 0.1, TagDescentGuidance.descentRate(0.7), 1e-9)
        // At the target and below it: never a descent. This is the terminal-hold property at
        // the law level — "does not descend below target" is arithmetic before it is a phase.
        assertEquals(0.0, TagDescentGuidance.descentRate(TagDescentGuidance.TARGET_HEIGHT_M), 1e-9)
        assertEquals(0.0, TagDescentGuidance.descentRate(0.3), 1e-9)
        assertEquals(0.0, TagDescentGuidance.descentRate(Double.NaN), 1e-9)
        // Never negative anywhere: this function cannot climb.
        var h = 0.0
        while (h < 10.0) {
            assertTrue(TagDescentGuidance.descentRate(h) >= 0.0)
            h += 0.05
        }
    }

    @Test
    fun `the reacquisition climb is slow, upward, and stops at the arm ceiling`() {
        assertEquals(-TagDescentGuidance.V_REACQUIRE_MS, TagDescentGuidance.reacquireRate(3.0), 1e-9)
        // At and above the ceiling: hold. Above the band a climb buys nothing — the measured
        // rate above the cliff is not a sensor — and an unbounded climb is a fly-away.
        assertEquals(0.0, TagDescentGuidance.reacquireRate(TagDescentGuidance.ARM_CEILING_M), 1e-9)
        assertEquals(0.0, TagDescentGuidance.reacquireRate(20.0), 1e-9)
        assertEquals(0.0, TagDescentGuidance.reacquireRate(Double.NaN), 1e-9)
    }

    @Test
    fun `terminalNow needs both conjuncts - the height band and the innermost cone`() {
        val target = TagDescentGuidance.TARGET_HEIGHT_M
        assertTrue(TagDescentGuidance.terminalNow(target, 0.0))
        // Just inside the band on both sides (not the exact edge — 0.6 + 0.15 is not exactly
        // representable in binary and the sensor is 0.1 m-quantised anyway).
        assertTrue(TagDescentGuidance.terminalNow(target + 0.14, 0.19))
        assertTrue(TagDescentGuidance.terminalNow(target - 0.14, 0.19))
        // Height out of band.
        assertFalse(TagDescentGuidance.terminalNow(target + 0.16, 0.0))
        // Lateral outside the innermost cone — which is the floor, at the target height.
        assertFalse(TagDescentGuidance.terminalNow(target, TagDescentGuidance.CONE_FLOOR_M + 0.01))
        assertFalse(TagDescentGuidance.terminalNow(Double.NaN, 0.0))
        assertFalse(TagDescentGuidance.terminalNow(target, Double.NaN))
    }

    // ------------------------------------------------------------------ the machine

    private fun fly(step: TagDescent.Step): TagDescent.Step.Fly = step as TagDescent.Step.Fly

    @Test
    fun `fresh and inside the cone - lateral plus descent, no yaw ever`() {
        val m = TagDescent()
        val step = fly(m.step(heightM = 5.0, errorNorthM = 0.3, errorEastM = 0.0, fixAgeMs = 100))
        assertEquals(TagDescentPhase.TRACKING, m.phase)
        assertNull(step.entered)
        assertTrue(step.velocities.north > 0.0)
        assertEquals(TagDescentGuidance.V_DESCENT_MAX_MS, step.velocities.down, 1e-9)
        assertEquals(0.0, step.velocities.yawRateDegPerS, 1e-9)
    }

    @Test
    fun `outside the cone the aircraft holds altitude and keeps centring - same phase`() {
        val m = TagDescent()
        // 5 m up, cone is 0.75 m; 2 m of error is genuinely off-centre.
        val step = fly(m.step(5.0, 2.0, 0.0, fixAgeMs = 100))
        assertEquals(TagDescentPhase.TRACKING, m.phase)
        assertEquals("altitude bought with misalignment", 0.0, step.velocities.down, 1e-9)
        assertTrue(step.velocities.north > 0.0)
    }

    @Test
    fun `exactly on the cone boundary still descends - the gate is on exceeding it`() {
        val m = TagDescent()
        val step = fly(m.step(5.0, TagDescentGuidance.coneRadiusM(5.0), 0.0, fixAgeMs = 0))
        assertTrue(step.velocities.down > 0.0)
    }

    @Test
    fun `a stale fix stops the descent and announces the phase once`() {
        val m = TagDescent()
        fly(m.step(5.0, 0.0, 0.0, fixAgeMs = 100))
        val held = fly(m.step(5.0, 0.0, 0.0, fixAgeMs = TagDescentGuidance.T_HOLD_MS + 1))
        assertEquals(TagDescentPhase.HOLDING, m.phase)
        assertEquals(TagDescentPhase.HOLDING, held.entered)
        assertEquals(0.0, held.velocities.down, 1e-9)
        // Second stale tick: same phase, no repeated edge.
        assertNull(fly(m.step(5.0, 0.0, 0.0, fixAgeMs = 600)).entered)
    }

    @Test
    fun `past T_CLIMB the machine climbs - slowly, and never above the arm ceiling`() {
        val m = TagDescent()
        val climb = fly(m.step(4.0, 0.0, 0.0, fixAgeMs = TagDescentGuidance.T_CLIMB_MS + 1))
        assertEquals(TagDescentPhase.CLIMBING, climb.entered)
        assertEquals(-TagDescentGuidance.V_REACQUIRE_MS, climb.velocities.down, 1e-9)
        // At the ceiling the climb stops and the aircraft holds, still centring.
        val atCeiling = fly(m.step(TagDescentGuidance.ARM_CEILING_M, 1.0, 0.0, fixAgeMs = 3_000))
        assertEquals(0.0, atCeiling.velocities.down, 1e-9)
        assertTrue(atCeiling.velocities.north > 0.0)
    }

    @Test
    fun `climbing blind holds instead - the ceiling cannot be enforced without a height`() {
        val m = TagDescent()
        val step = fly(m.step(heightM = null, errorNorthM = 0.0, errorEastM = 0.0, fixAgeMs = 3_000))
        assertEquals(TagDescentPhase.CLIMBING, m.phase)
        assertEquals(0.0, step.velocities.down, 1e-9)
    }

    @Test
    fun `past T_ABORT the machine hands back, from any phase including terminal`() {
        val m = TagDescent()
        assertTrue(m.step(5.0, 0.0, 0.0, TagDescentGuidance.T_ABORT_MS + 1) is TagDescent.Step.HandBack)

        val terminal = reachTerminal()
        assertTrue(
            terminal.step(0.6, 0.0, 0.0, TagDescentGuidance.T_ABORT_MS + 1) is TagDescent.Step.HandBack,
        )
    }

    @Test
    fun `the tag coming back is an announced transition back to tracking`() {
        val m = TagDescent()
        fly(m.step(5.0, 0.0, 0.0, fixAgeMs = 600))
        assertEquals(TagDescentPhase.HOLDING, m.phase)
        val back = fly(m.step(5.0, 0.0, 0.0, fixAgeMs = 50))
        assertEquals(TagDescentPhase.TRACKING, back.entered)
        assertTrue(back.velocities.down > 0.0)
    }

    @Test
    fun `unknown altitude - lateral centring continues, nothing vertical, no terminal`() {
        val m = TagDescent()
        repeat(20) {
            val step = fly(m.step(heightM = null, errorNorthM = 0.05, errorEastM = 0.0, fixAgeMs = 100))
            assertEquals(0.0, step.velocities.down, 1e-9)
            assertTrue(step.velocities.north > 0.0)
        }
        assertEquals("terminal declared blind", TagDescentPhase.TRACKING, m.phase)
    }

    // ------------------------------------------------------------------ the approach

    @Test
    fun `a machine armed above the band starts in APPROACH - and the default never does`() {
        assertEquals(TagDescentPhase.APPROACH, TagDescent(approach = true).phase)
        // The below-band arm is byte-identical to yesterday's machine: no path into APPROACH.
        assertEquals(TagDescentPhase.TRACKING, TagDescent().phase)
    }

    @Test
    fun `the approach constants - the ceiling at the decode reach, the entry inside the band`() {
        // 12 m: the measured decode-reach edge (nothing has ever decoded above the 9-10 m
        // band) plus the measured session baro wander, and above the 10 m takeoff hover the
        // feature exists for (landing13 t=41.8) — the constant's KDoc carries the table.
        assertEquals(12.0, TagDescentGuidance.APPROACH_CEILING_M, 1e-9)
        assertTrue(
            "the ceiling must admit the 10 m takeoff default",
            TagDescentGuidance.APPROACH_CEILING_M >
                com.dimensional.mini4pro.command.CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M,
        )
        assertTrue(TagDescentGuidance.APPROACH_CEILING_M > TagDescentGuidance.ARM_CEILING_M)
        // The entry sits one measurement bin (0.5 m — the resolution the detection cliff was
        // located at) inside the band: five baro quanta, unchatterable.
        assertEquals(
            TagDescentGuidance.ARM_CEILING_M - TagDescentGuidance.APPROACH_ENTRY_MARGIN_M,
            TagDescentGuidance.APPROACH_BAND_ENTRY_M,
            1e-9,
        )
        assertEquals(6.5, TagDescentGuidance.APPROACH_BAND_ENTRY_M, 1e-9)
        assertTrue(TagDescentGuidance.APPROACH_ENTRY_MARGIN_M >= 0.5)
    }

    @Test
    fun `the approach descends in-cone on a fresh fix at approach heights - no yaw`() {
        val m = TagDescent(approach = true)
        val step = fly(m.step(heightM = 10.0, errorNorthM = 0.3, errorEastM = 0.0, fixAgeMs = 100))
        assertEquals(TagDescentPhase.APPROACH, m.phase)
        assertNull(step.entered)
        assertTrue(step.velocities.north > 0.0)
        assertEquals(TagDescentGuidance.V_DESCENT_MAX_MS, step.velocities.down, 1e-9)
        assertEquals(0.0, step.velocities.yawRateDegPerS, 1e-9)
    }

    @Test
    fun `outside the cone the approach holds altitude and keeps centring`() {
        val m = TagDescent(approach = true)
        // At 10 m the cone is 1.5 m; 2.5 m of error is genuinely off-centre even up here.
        val step = fly(m.step(10.0, 2.5, 0.0, fixAgeMs = 100))
        assertEquals("altitude bought with misalignment", 0.0, step.velocities.down, 1e-9)
        assertTrue(step.velocities.north > 0.0)
        assertEquals(TagDescentPhase.APPROACH, m.phase)
    }

    @Test
    fun `the approach descends through short staleness and holds past the climb rung`() {
        val m = TagDescent(approach = true)
        // The HOLD rung — the sparse weather's expected gap shape (measured mean gaps of
        // ~3.7-7.7 s at 8-10 m): the approach keeps descending, blind-bounded to
        // T_CLIMB * V_DESCENT_MAX = 0.8 m, under the cone at these heights.
        val hold = fly(m.step(9.0, 0.3, 0.0, fixAgeMs = TagDescentGuidance.T_HOLD_MS + 1))
        assertTrue("the approach must ride through short decode gaps", hold.velocities.down > 0.0)
        // Inclusive on the descending side, like every rung boundary in the ladder.
        assertTrue(fly(m.step(9.0, 0.3, 0.0, TagDescentGuidance.T_CLIMB_MS)).velocities.down > 0.0)
        // Past T_CLIMB: hold — never descend further blind, and never climb (there is
        // nothing above the band a climb can buy). Centring continues on the last fix.
        val stale = fly(m.step(9.0, 0.3, 0.0, fixAgeMs = TagDescentGuidance.T_CLIMB_MS + 1))
        assertEquals(0.0, stale.velocities.down, 1e-9)
        assertTrue(stale.velocities.north > 0.0)
        assertEquals(TagDescentPhase.APPROACH, m.phase)
        // The tag coming back resumes the descent, same phase — no edge chatter.
        val resumed = fly(m.step(9.0, 0.3, 0.0, fixAgeMs = 100))
        assertTrue(resumed.velocities.down > 0.0)
        assertNull(resumed.entered)
    }

    @Test
    fun `the approach hands back past T_ABORT - the GONE bound is unchanged up here`() {
        val m = TagDescent(approach = true)
        assertTrue(
            m.step(10.0, 0.0, 0.0, TagDescentGuidance.T_ABORT_MS + 1) is TagDescent.Step.HandBack,
        )
    }

    @Test
    fun `unknown height during the approach - centring continues, nothing vertical, no entry`() {
        val m = TagDescent(approach = true)
        repeat(10) {
            val step = fly(m.step(heightM = null, errorNorthM = 0.2, errorEastM = 0.0, fixAgeMs = 100))
            assertEquals("a blind band entry cannot be evaluated", 0.0, step.velocities.down, 1e-9)
            assertTrue(step.velocities.north > 0.0)
        }
        assertEquals(TagDescentPhase.APPROACH, m.phase)
    }

    @Test
    fun `band entry - above the hysteresis edge approach, at it a one-way announced handoff`() {
        val m = TagDescent(approach = true)
        // One baro quantum above the entry: still the approach — the margin is what keeps the
        // quantised change-driven altitude from flickering the regime.
        fly(m.step(TagDescentGuidance.APPROACH_BAND_ENTRY_M + 0.1, 0.0, 0.0, 100))
        assertEquals(TagDescentPhase.APPROACH, m.phase)
        // At the entry: the handoff fires, announced, and the same tick already flies the
        // tracking law — the seamless half of the design.
        val entry = fly(m.step(TagDescentGuidance.APPROACH_BAND_ENTRY_M, 0.0, 0.0, 100))
        assertEquals(TagDescentPhase.TRACKING, entry.entered)
        assertEquals(TagDescentPhase.TRACKING, m.phase)
        assertTrue(entry.velocities.down > 0.0)
        // One-way: a gust lifting the aircraft back above the entry leaves it in the ladder —
        // there is no path back into APPROACH.
        fly(m.step(7.5, 0.0, 0.0, 100))
        assertEquals(TagDescentPhase.TRACKING, m.phase)
    }

    @Test
    fun `a stale band entry lands on its honest rung - HOLDING, announced as such`() {
        val m = TagDescent(approach = true)
        val entry = fly(m.step(6.4, 0.0, 0.0, fixAgeMs = 600))
        assertEquals(TagDescentPhase.HOLDING, entry.entered)
        assertEquals(TagDescentPhase.HOLDING, m.phase)
        assertEquals(0.0, entry.velocities.down, 1e-9)
    }

    @Test
    fun `after the handoff the machine is step-identical to one armed below the band`() {
        val approached = TagDescent(approach = true)
        fly(approached.step(8.0, 0.4, 0.0, 100))
        fly(approached.step(7.2, 0.4, 0.0, 700))
        fly(approached.step(TagDescentGuidance.APPROACH_BAND_ENTRY_M, 0.4, 0.0, 100))
        check(approached.phase == TagDescentPhase.TRACKING)
        val plain = TagDescent()
        // One identical input tape into both machines: every step and every phase must match
        // — "byte-identical to the current behaviour from the band", pinned.
        data class Tape(val h: Double?, val n: Double, val e: Double, val age: Long)
        val tape = listOf(
            Tape(6.0, 0.3, 0.1, 100), Tape(5.0, 0.1, 0.0, 100), Tape(5.0, 0.1, 0.0, 600),
            Tape(4.0, 0.0, 0.0, 2_500), Tape(null, 0.2, 0.0, 100), Tape(3.0, 0.05, 0.0, 100),
        )
        for (t in tape) {
            assertEquals(plain.step(t.h, t.n, t.e, t.age), approached.step(t.h, t.n, t.e, t.age))
            assertEquals(plain.phase, approached.phase)
        }
    }

    @Test
    fun `no commit can fire during the approach - a floor verdict up there changes nothing`() {
        // An FC floor reading at approach heights is an obstacle, not the pad (the commit
        // ceiling's argument, 4x over) — and structurally the approach branch owns the tick,
        // so the commit conjuncts are unreachable however the flags read.
        val m = TagDescent(fullAutoland = true, approach = true)
        val step = fly(m.step(10.0, 0.0, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.0))
        assertNull(step.entered)
        assertEquals(TagDescentPhase.APPROACH, m.phase)
    }

    // ------------------------------------------------------------------ terminal

    private fun reachTerminal(): TagDescent {
        val m = TagDescent()
        repeat(TagDescentGuidance.TERMINAL_TICKS) {
            m.step(TagDescentGuidance.TARGET_HEIGHT_M, 0.05, 0.0, fixAgeMs = 100)
        }
        check(m.phase == TagDescentPhase.TERMINAL)
        return m
    }

    @Test
    fun `terminal needs TERMINAL_TICKS consecutive ticks and announces once`() {
        val m = TagDescent()
        repeat(TagDescentGuidance.TERMINAL_TICKS - 1) {
            assertNull(fly(m.step(0.6, 0.05, 0.0, 100)).entered)
            assertEquals(TagDescentPhase.TRACKING, m.phase)
        }
        val done = fly(m.step(0.6, 0.05, 0.0, 100))
        assertEquals(TagDescentPhase.TERMINAL, done.entered)
        assertEquals(TagDescentPhase.TERMINAL, m.phase)
        assertEquals(0.0, done.velocities.down, 1e-9)
    }

    @Test
    fun `a fly-through cannot complete - the counter resets on any failing tick`() {
        val m = TagDescent()
        repeat(TagDescentGuidance.TERMINAL_TICKS - 1) { m.step(0.6, 0.05, 0.0, 100) }
        // One tick outside the innermost cone, then back in: the count starts over.
        m.step(0.6, 0.5, 0.0, 100)
        repeat(TagDescentGuidance.TERMINAL_TICKS - 1) {
            m.step(0.6, 0.05, 0.0, 100)
            assertEquals(TagDescentPhase.TRACKING, m.phase)
        }
    }

    @Test
    fun `terminal holds - it never descends below the target and never resumes the descent`() {
        val m = reachTerminal()
        // At the target: zero vertical.
        assertEquals(0.0, fly(m.step(0.6, 0.0, 0.0, 100)).velocities.down, 1e-9)
        // Sunk below the target: a gentle climb back, never a descent.
        val below = fly(m.step(0.45, 0.0, 0.0, 100))
        assertTrue("below the target the hold must not descend", below.velocities.down < 0.0)
        assertTrue(abs(below.velocities.down) <= TagDescentGuidance.V_REACQUIRE_MS + 1e-9)
        // Blown above the target: re-descends toward it, but only fresh and inside the cone…
        assertTrue(fly(m.step(1.2, 0.0, 0.0, 100)).velocities.down > 0.0)
        // …and never on a stale fix, and never outside the cone.
        assertEquals(0.0, fly(m.step(1.2, 0.0, 0.0, 600)).velocities.down, 1e-9)
        assertEquals(0.0, fly(m.step(1.2, 1.0, 0.0, 100)).velocities.down, 1e-9)
        // And through all of that it stays TERMINAL: there is no way back into the descent.
        assertEquals(TagDescentPhase.TERMINAL, m.phase)
    }

    @Test
    fun `terminal keeps centring - the lateral law does not stop at the ending`() {
        val m = reachTerminal()
        val step = fly(m.step(0.6, 0.3, 0.0, 100))
        assertTrue(step.velocities.north > 0.0)
        assertEquals(0.0, step.velocities.yawRateDegPerS, 1e-9)
    }

    // ------------------------------------------------------------------ landing (Stage C)

    private fun reachTerminal(autoland: Boolean): TagDescent {
        val m = TagDescent(fullAutoland = autoland)
        repeat(TagDescentGuidance.TERMINAL_TICKS) {
            m.step(TagDescentGuidance.TARGET_HEIGHT_M, 0.05, 0.0, fixAgeMs = 100)
        }
        check(m.phase == TagDescentPhase.TERMINAL)
        return m
    }

    private fun reachCommitted(): TagDescent {
        val m = reachTerminal(autoland = true)
        val step = fly(
            m.step(TagDescentGuidance.TARGET_HEIGHT_M, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = 0.0),
        )
        check(step.entered == TagDescentPhase.DJI_LANDING)
        return m
    }

    @Test
    fun `a plain machine's terminal never lands - the default is yesterday's stage B`() {
        val m = reachTerminal(autoland = false)
        repeat(50) {
            val step = fly(
                m.step(TagDescentGuidance.TARGET_HEIGHT_M, 0.0, 0.0, fixAgeMs = 100, lateralSpeedM = 0.0),
            )
            assertEquals(TagDescentPhase.TERMINAL, m.phase)
            assertEquals(0.0, step.velocities.down, 1e-9)
        }
    }

    @Test
    fun `a plain machine never commits at the floor either - the option is the gate`() {
        val m = TagDescent(fullAutoland = false)
        repeat(50) {
            // Every other conjunct deliberately satisfied — speed included — so the option is
            // the one thing this tick lacks.
            m.step(1.4, 0.05, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.0)
            assertEquals(TagDescentPhase.TRACKING, m.phase)
        }
    }

    @Test
    fun `terminal commits only on a fresh in-cone slow fix at a known height`() {
        // Stale fix: the hold continues — committing on a memory is the failure this pins.
        val stale = reachTerminal(autoland = true)
        stale.step(0.6, 0.05, 0.0, fixAgeMs = TagDescentGuidance.LAND_FRESH_MS + 1, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TERMINAL, stale.phase)

        // Fresh but outside the innermost cone: not over the tag, no commitment.
        val offCentre = reachTerminal(autoland = true)
        offCentre.step(0.6, TagDescentGuidance.CONE_FLOOR_M + 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TERMINAL, offCentre.phase)

        // Unknown height: the cone cannot be evaluated, so neither can "over the tag".
        val blindHeight = reachTerminal(autoland = true)
        blindHeight.step(null, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TERMINAL, blindHeight.phase)

        // Moving: one quantum of reading (0.1 — true speed maybe 0.15 m/s) keeps holding;
        // landing06 measured ~20 cm of touchdown miss per 12 cm/s carried across the commit.
        val moving = reachTerminal(autoland = true)
        moving.step(0.6, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = 0.1)
        assertEquals(TagDescentPhase.TERMINAL, moving.phase)

        // Unknown speed: a feed that cannot vouch for slow cannot commit — null defaults closed.
        val blindSpeed = reachTerminal(autoland = true)
        blindSpeed.step(0.6, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = null)
        assertEquals(TagDescentPhase.TERMINAL, blindSpeed.phase)

        // All of them recover: the next fresh centred slow tick commits, announced exactly
        // once, and the commit's own output is already the neutral stick — DJI flies from here.
        val commit = fly(blindHeight.step(0.6, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = 0.0))
        assertEquals(TagDescentPhase.DJI_LANDING, blindHeight.phase)
        assertEquals(TagDescentPhase.DJI_LANDING, commit.entered)
        assertEquals(StickVelocities.ZERO, commit.velocities)
        assertNull(fly(blindHeight.step(0.6, 0.05, 0.0, fixAgeMs = 100)).entered)
    }

    @Test
    fun `the speed gate sits below the feed's quantum - only a 0,0 reading can pass`() {
        // The velocity feed is 0.1 m/s-quantised. A bound at or above the quantum would pass
        // a 0.1 reading whose truth may be 0.15 m/s — more momentum than landing06's whole
        // budget. Below it, the only passing reading is 0.0, which means true ≤ 0.05.
        assertTrue(TagDescentGuidance.LAND_COMMIT_SPEED_MS < 0.1)
        assertEquals(0.05, TagDescentGuidance.LAND_COMMIT_SPEED_MS, 0.0)

        // The boundary is inclusive on the slow side, like every other gate here: the gate
        // fires on exceeding the bound.
        val atBound = reachTerminal(autoland = true)
        atBound.step(0.6, 0.05, 0.0, fixAgeMs = 100, lateralSpeedM = TagDescentGuidance.LAND_COMMIT_SPEED_MS)
        assertEquals(TagDescentPhase.DJI_LANDING, atBound.phase)
        val overBound = reachTerminal(autoland = true)
        overBound.step(0.6, 0.05, 0.0, 100, lateralSpeedM = TagDescentGuidance.LAND_COMMIT_SPEED_MS + 0.01)
        assertEquals(TagDescentPhase.TERMINAL, overBound.phase)
    }

    @Test
    fun `the floor commits from tracking - and only with every conjunct on the same tick`() {
        fun tracking() = TagDescent(fullAutoland = true).also {
            it.step(1.4, 0.05, 0.0, fixAgeMs = 100)
            check(it.phase == TagDescentPhase.TRACKING)
        }

        // No floor verdict: an in-cone fresh descent at 1.4 m keeps descending — the trigger
        // is the FC's measured refusal, never height alone.
        val noFloor = tracking()
        noFloor.step(1.4, 0.05, 0.0, fixAgeMs = 100, floorStalled = false, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TRACKING, noFloor.phase)

        // Floor but the fix is stale: an FC floor over ground nobody can vouch for holds.
        val staleFix = tracking()
        staleFix.step(
            1.4, 0.05, 0.0, fixAgeMs = TagDescentGuidance.T_HOLD_MS + 1,
            floorStalled = true, lateralSpeedM = 0.0,
        )
        assertEquals(TagDescentPhase.HOLDING, staleFix.phase)

        // Floor but off the tag: outside the cone at 1.4 m (cone = 0.21 m).
        val offCone = tracking()
        offCone.step(1.4, 0.5, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TRACKING, offCone.phase)

        // Floor but blind height: nothing to evaluate the cone or the ceiling against.
        val noHeight = tracking()
        noHeight.step(null, 0.05, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.0)
        assertEquals(TagDescentPhase.TRACKING, noHeight.phase)

        // Floor but moving: a quantum of lateral reading (true speed maybe 0.15 m/s) keeps
        // descending — momentum carried across a commit is touchdown miss (landing06), and
        // this entry point has the same speed conjunct as TERMINAL's, same tick discipline.
        val moving = tracking()
        moving.step(1.4, 0.05, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.1)
        assertEquals(TagDescentPhase.TRACKING, moving.phase)

        // Floor but the speed is unknown: null defaults the commit closed.
        val blindSpeed = tracking()
        blindSpeed.step(1.4, 0.05, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = null)
        assertEquals(TagDescentPhase.TRACKING, blindSpeed.phase)

        // Every conjunct: committed, entered edge once, output already neutral.
        val commit = tracking()
        val step = fly(commit.step(1.4, 0.05, 0.0, fixAgeMs = 100, floorStalled = true, lateralSpeedM = 0.0))
        assertEquals(TagDescentPhase.DJI_LANDING, commit.phase)
        assertEquals(TagDescentPhase.DJI_LANDING, step.entered)
        assertEquals(StickVelocities.ZERO, step.velocities)
    }

    @Test
    fun `the commit ceiling - a floor met above 2,5 m is an obstacle, not the ground`() {
        // At the ceiling: commit (the gate fires on exceeding it).
        val atCeiling = TagDescent(fullAutoland = true)
        atCeiling.step(
            TagDescentGuidance.COMMIT_CEILING_M, 0.05, 0.0, 100,
            floorStalled = true, lateralSpeedM = 0.0,
        )
        assertEquals(TagDescentPhase.DJI_LANDING, atCeiling.phase)

        // Above it: never — a stalled descent at 3 m is the obstacle sensor seeing something
        // that is not the pad, and StartAutoLanding onto it is the command this refuses.
        val above = TagDescent(fullAutoland = true)
        repeat(20) {
            above.step(
                TagDescentGuidance.COMMIT_CEILING_M + 0.1, 0.05, 0.0, 100,
                floorStalled = true, lateralSpeedM = 0.0,
            )
            assertEquals(TagDescentPhase.TRACKING, above.phase)
        }
    }

    @Test
    fun `committed - lateral only, exits nothing, hands back nothing`() {
        // The successor of the frozen design's "flies nothing" pin (its deliberate
        // replacement, not a deletion): the committed phase now flies the odometric lateral
        // on whatever error the engine vouched for — and still exits nothing, whatever the
        // fix age, because tag loss is the landing's shape and the target is a world
        // constant. Vertical and yaw stay exactly zero; the neutral-when-zero-error case is
        // pinned separately below.
        val m = reachCommitted()
        val heights = listOf(null, -0.1, 0.3, 0.6, 5.0)
        val ages = listOf(0L, 500L, TagDescentGuidance.T_ABORT_MS + 1, 600_000L)
        for (h in heights) for (age in ages) {
            val step = m.step(h, 2.0, 1.0, age)
            assertTrue("age $age must not hand back - tag loss is the landing's shape", step is TagDescent.Step.Fly)
            val v = (step as TagDescent.Step.Fly).velocities
            assertEquals("DJI owns the descent", 0.0, v.down, 0.0)
            assertEquals("no yaw, ever", 0.0, v.yawRateDegPerS, 0.0)
            // The lateral is exactly the landing law on the given error — capped, toward it.
            val (n, e) = TagDescentGuidance.landingLateral(2.0, 1.0)
            assertEquals(n, v.north, 1e-12)
            assertEquals(e, v.east, 1e-12)
            assertNull(step.entered)
            assertEquals(TagDescentPhase.DJI_LANDING, m.phase)
        }
    }

    @Test
    fun `committed with a zero error - the dead-stick shape is byte-for-byte the old neutral`() {
        // The engine passes exactly (0, 0) when the position feed goes stale or no believed
        // target exists: the law must turn that into the frozen design's neutral stick, so
        // "stale position -> zeros, not a random walk" is arithmetic here before it is an
        // engine gate.
        val m = reachCommitted()
        for (age in listOf(0L, 500L, 600_000L)) {
            val step = fly(m.step(null, 0.0, 0.0, age))
            assertEquals(StickVelocities.ZERO, step.velocities)
        }
    }

    @Test
    fun `the commit ceiling sits between the measured floor and the detection band`() {
        // 1.8× the one measured FC floor (1.4 m, landing04), inside the 100 % per-frame
        // detection band, and low enough that DJI's own blind final from it stays ~8 s.
        assertTrue(TagDescentGuidance.COMMIT_CEILING_M > 1.4)
        assertTrue(TagDescentGuidance.COMMIT_CEILING_M < TagDescentGuidance.ARM_CEILING_M)
        assertEquals(2.5, TagDescentGuidance.COMMIT_CEILING_M, 0.0)
    }

    @Test
    fun `the landing freshness bound is the hold bound - one claim, made once`() {
        assertEquals(TagDescentGuidance.T_HOLD_MS, TagDescentGuidance.LAND_FRESH_MS)
    }

    // ------------------------------------------------- the blind-final steering (Stage C)

    @Test
    fun `landingLateral is the tracking law under the landing's own cap`() {
        // Direction kept, magnitude capped at V_LAND_LATERAL_MAX — a fifth of tracking's.
        val (n, e) = TagDescentGuidance.landingLateral(30.0, 40.0)
        assertEquals(0.6, n / hypot(n, e), 1e-9)
        assertEquals(0.8, e / hypot(n, e), 1e-9)
        assertEquals(TagDescentGuidance.V_LAND_LATERAL_MAX_MS, hypot(n, e), 1e-9)
        // Close in it is the same clamped-speed arithmetic as everything else in M3: at the
        // measured worst-case blind-final error (~0.3 m) the command is kp·e = 0.15 m/s —
        // under the cap, so the budget's own commands ride the plain braking curve.
        val (closeN, _) = TagDescentGuidance.landingLateral(0.3, 0.0)
        assertEquals(RepositionGuidance.KP_PER_S * 0.3, closeN, 1e-9)
        // Garbage in, dead stick out.
        assertEquals(Pair(0.0, 0.0), TagDescentGuidance.landingLateral(Double.NaN, 1.0))
        assertEquals(Pair(0.0, 0.0), TagDescentGuidance.landingLateral(0.0, 0.0))
    }

    @Test
    fun `the landing cap out-pulls the measured momentum and bounds the wrong-world runaway`() {
        assertEquals(0.2, TagDescentGuidance.V_LAND_LATERAL_MAX_MS, 0.0)
        // Above the measured 12 cm/s handover momentum it must correct (landing06)…
        assertTrue(TagDescentGuidance.V_LAND_LATERAL_MAX_MS > 0.12)
        // …and well under the tracking cap: a wrong fix at ankle height over the measured
        // 3.2 s blind final is bounded to 0.64 m, not 3.2 m.
        assertTrue(TagDescentGuidance.V_LAND_LATERAL_MAX_MS <= TagDescentGuidance.V_LATERAL_MAX_MS / 5.0)
    }

    @Test
    fun `landingTarget - the window median, anchored to the samples' own clock`() {
        fun s(atMs: Long, n: Double, e: Double) = TagDescentGuidance.FixSample(atMs, n, e)

        // Empty: no believed samples is no target — null, never a guess.
        assertNull(TagDescentGuidance.landingTarget(emptyList()))

        // One sample: the single last fix, unaveraged — the pre-averaging design, and fine.
        assertEquals(Pair(0.10, 0.02), TagDescentGuidance.landingTarget(listOf(s(1_000, 0.10, 0.02))))

        // One sample in the window plus older ones outside it: still the newest, unaveraged —
        // a lonely fresh fix must not be averaged with another height's biased ones.
        assertEquals(
            Pair(0.10, 0.02),
            TagDescentGuidance.landingTarget(
                listOf(s(1_000, 0.30, 0.30), s(1_200, 0.30, 0.30), s(10_000, 0.10, 0.02)),
            ),
        )

        // The median is per axis, over exactly the window behind the newest sample.
        val window = listOf(
            s(10_000, 0.10, 0.00), s(10_100, 0.14, 0.02), s(10_200, 0.12, 0.04),
        )
        val odd = TagDescentGuidance.landingTarget(window)!!
        assertEquals(0.12, odd.first, 1e-9)
        assertEquals(0.02, odd.second, 1e-9)

        // Even count: the middle pair's mean, the standard median.
        val evenTarget = TagDescentGuidance.landingTarget(
            listOf(s(10_000, 0.10, 0.00), s(10_100, 0.14, 0.02)),
        )!!
        assertEquals(0.12, evenTarget.first, 1e-9)
        assertEquals(0.01, evenTarget.second, 1e-9)

        // Robustness: one straggler inside the window cannot drag the target the way a mean
        // would (the id gate already kills false decodes; the median bounds whatever remains).
        val straggler = listOf(
            s(10_000, 0.10, 0.00), s(10_100, 0.11, 0.01), s(10_200, 0.12, 0.02),
            s(10_300, 0.11, 0.01), s(10_400, 5.00, 5.00),
        )
        val robust = TagDescentGuidance.landingTarget(straggler)!!
        assertEquals(0.11, robust.first, 1e-9)
        assertEquals(0.01, robust.second, 1e-9)
    }

    @Test
    fun `the window is short enough to exclude the measured height-correlated walk`() {
        // Landing06's fix reconstructions: 12 cm of east spread over 8 s that is NOT noise —
        // a height-correlated systematic walk (~9 cm of migration across 2.5 -> 1.0 m of
        // descent). This plants that walk: 8 s of samples whose east migrates linearly to
        // 0.12, with the last half-second clustered at the final (lowest, least biased)
        // height. The target must be the recent cluster's value, not the full-history median
        // — which is exactly what a stretched window would produce.
        fun s(atMs: Long, n: Double, e: Double) = TagDescentGuidance.FixSample(atMs, n, e)
        val walk = ArrayList<TagDescentGuidance.FixSample>()
        var t = 0L
        while (t <= 8_000L) {
            walk += s(t, 0.0, 0.12 * t / 8_000.0)
            t += 100L
        }
        val target = TagDescentGuidance.landingTarget(walk)!!
        // The 500 ms window holds the 7.5–8.0 s samples: east 0.1125..0.12, median ≈ 0.116.
        // A 10 s window would read the whole walk's median ≈ 0.06 — half the height away.
        assertEquals(0.11625, target.second, 0.005)
        assertTrue("a stretched window averaged in another height's bias", target.second > 0.10)
        assertEquals(TagDescentGuidance.LAND_TARGET_WINDOW_MS, 500L)
    }

    // ------------------------------------------- the range ladder's height (landing07)

    @Test
    fun `the height ladder - a fresh tag-derived range outranks the barometer`() {
        // Landing07's landing B, in numbers: baro 0.7 (lying by ~1.2 m), size range 1.93
        // (right). The law must fly the tag's number, and say which instrument it was.
        val size = TagDescentGuidance.descentHeight(0.7, 1.93, RangeSource.SIZE, fixAgeMs = 100)!!
        assertEquals(1.93, size.heightM, 0.0)
        assertEquals(RangeSource.SIZE, size.source)

        // Landing A's commit: solve 0.59 against baro 0.8 — the near-ground baro bias, not
        // flown on either.
        val solve = TagDescentGuidance.descentHeight(0.8, 0.59, RangeSource.SOLVE, fixAgeMs = 100)!!
        assertEquals(0.59, solve.heightM, 0.0)
        assertEquals(RangeSource.SOLVE, solve.source)
    }

    @Test
    fun `the height ladder - baro is the fallback, and only for what it can vouch`() {
        // A stale tag range is a height from another place: baro.
        val stale = TagDescentGuidance.descentHeight(
            5.0, 1.93, RangeSource.SIZE, fixAgeMs = TagDescentGuidance.LAND_FRESH_MS + 1,
        )!!
        assertEquals(RangeSource.BARO, stale.source)
        assertEquals(5.0, stale.heightM, 0.0)
        // The freshness boundary is inclusive, like every gate here.
        val atBound = TagDescentGuidance.descentHeight(
            5.0, 1.93, RangeSource.SIZE, fixAgeMs = TagDescentGuidance.LAND_FRESH_MS,
        )!!
        assertEquals(RangeSource.SIZE, atBound.source)

        // A fix that itself rested on baro offers no tag range — current baro, not the fix's.
        val baroFix = TagDescentGuidance.descentHeight(5.0, null, RangeSource.BARO, fixAgeMs = 100)!!
        assertEquals(RangeSource.BARO, baroFix.source)

        // Garbage tag ranges cannot win the ladder.
        assertEquals(
            RangeSource.BARO,
            TagDescentGuidance.descentHeight(5.0, Double.NaN, RangeSource.SIZE, 100)!!.source,
        )
        assertEquals(
            RangeSource.BARO,
            TagDescentGuidance.descentHeight(5.0, -1.0, RangeSource.SOLVE, 100)!!.source,
        )

        // No instrument at all: null — unknown is never zero.
        assertNull(TagDescentGuidance.descentHeight(null, null, null, 100))
        assertNull(TagDescentGuidance.descentHeight(Double.NaN, null, null, 100))

        // And a fresh tag range survives a dead barometer: the tag is the instrument.
        val noBaro = TagDescentGuidance.descentHeight(null, 1.93, RangeSource.SIZE, 100)!!
        assertEquals(1.93, noBaro.heightM, 0.0)
    }

    @Test
    fun `the divergence check - a measurement between landing07's two brackets, never a gate`() {
        assertEquals(1.8, TagDescentGuidance.RANGE_DIVERGENCE_FACTOR, 0.0)

        // Landing A's healthy pair (ratio 1.36): no line — a threshold under it would flag
        // every honest landing ever flown.
        assertNull(TagDescentGuidance.rangeDivergence(0.59, 0.8))

        // Landing B's failure pair (ratio 2.76): the line, with both numbers in it.
        val detail = TagDescentGuidance.rangeDivergence(1.93, 0.7)
        assertNotNull(detail)
        assertTrue(detail!!.contains("tag=1.93"))
        assertTrue(detail.contains("baro=0.70"))

        // Symmetric: an instrument lying in either direction is worth a line.
        assertNotNull(TagDescentGuidance.rangeDivergence(0.7, 1.93))

        // The boundary is inclusive on the agreeing side.
        assertNull(TagDescentGuidance.rangeDivergence(1.8, 1.0))
        assertNotNull(TagDescentGuidance.rangeDivergence(1.81, 1.0))

        // An absent instrument is an absence, not a divergence — no warn spam for the whole
        // baro-free descent the ladder makes possible.
        assertNull(TagDescentGuidance.rangeDivergence(null, 0.7))
        assertNull(TagDescentGuidance.rangeDivergence(1.93, null))
        assertNull(TagDescentGuidance.rangeDivergence(1.93, 0.0))
    }

    // ------------------------------------------------------------------ envelopes

    @Test
    fun `no step the machine can produce exceeds its own caps, over a grid of inputs`() {
        val m = TagDescent()
        val heights = listOf(null, 0.2, 0.6, 1.0, 3.0, 7.0, 12.0)
        val errors = listOf(0.0, 0.1, 0.5, 2.0, 30.0)
        val ages = listOf(0L, 400L, 401L, 2_001L, 9_999L)
        for (h in heights) for (eN in errors) for (eE in errors) for (age in ages) {
            val step = m.step(h, eN, eE, age)
            if (step !is TagDescent.Step.Fly) continue
            val v = step.velocities
            assertTrue(hypot(v.north, v.east) <= TagDescentGuidance.V_LATERAL_MAX_MS + 1e-9)
            assertTrue(v.down <= TagDescentGuidance.V_DESCENT_MAX_MS + 1e-9)
            assertTrue(-v.down <= TagDescentGuidance.V_REACQUIRE_MS + 1e-9)
            assertEquals(0.0, v.yawRateDegPerS, 1e-9)
        }
    }

    @Test
    fun `no committed step ever commands vertical or yaw, and the lateral never exceeds its cap`() {
        // The successor of "no committed step is anything but zero": the property the pivot
        // keeps is that fighting DJI's vertical is structurally impossible — down and yaw are
        // the literal constants 0.0 for every input the machine can be handed — and the
        // lateral, however wrong the error, is bounded by the landing's own cap, a fifth of
        // the tracking cap (the budget is in V_LAND_LATERAL_MAX_MS's KDoc).
        val m = reachCommitted()
        val heights = listOf(null, -0.1, 0.2, 0.6, 3.0)
        val errors = listOf(0.0, 0.1, 2.0, 30.0, Double.NaN)
        val ages = listOf(0L, 401L, 2_001L, 60_000L)
        for (h in heights) for (eN in errors) for (eE in errors) for (age in ages) {
            val step = m.step(h, eN, eE, age)
            assertTrue(step is TagDescent.Step.Fly)
            val v = (step as TagDescent.Step.Fly).velocities
            assertEquals(0.0, v.down, 0.0)
            assertEquals(0.0, v.yawRateDegPerS, 0.0)
            assertTrue(hypot(v.north, v.east) <= TagDescentGuidance.V_LAND_LATERAL_MAX_MS + 1e-9)
        }
    }

    @Test
    fun `the descent's caps sit inside the Q1 envelope with room to spare`() {
        assertTrue(TagDescentGuidance.V_LATERAL_MAX_MS < GuidedEnvelope.HORIZONTAL_MAX_MS)
        assertTrue(TagDescentGuidance.V_LAND_LATERAL_MAX_MS < GuidedEnvelope.HORIZONTAL_MAX_MS)
        assertTrue(TagDescentGuidance.V_DESCENT_MAX_MS < GuidedEnvelope.VERTICAL_MAX_MS)
        assertTrue(TagDescentGuidance.V_REACQUIRE_MS < GuidedEnvelope.VERTICAL_MAX_MS)
    }

    @Test
    fun `the nadir tolerance is TagWorld's own - one constant, not a second number`() {
        assertEquals(
            com.dimensional.mini4pro.vision.TagWorld.NADIR_TOLERANCE_DEG,
            TagDescentGuidance.NADIR_TOLERANCE_DEG,
            0.0,
        )
    }

    @Test
    fun `the arm ceiling is the measured 7 m band, below the CPU rule's 8 m`() {
        assertEquals(7.0, TagDescentGuidance.ARM_CEILING_M, 0.0)
        assertTrue(
            TagDescentGuidance.ARM_CEILING_M <
                com.dimensional.mini4pro.vision.TagArming.ACQUIRE_CEILING_M,
        )
    }
}
