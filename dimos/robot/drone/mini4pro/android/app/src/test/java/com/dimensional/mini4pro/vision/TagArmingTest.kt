package com.dimensional.mini4pro.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Ivan's rule, row by row.**
 *
 * > Run the detector during takeoff. If the tag is recognised then, latch it for the flight — and
 * > the recogniser auto-enables for the return and descent.
 *
 * The rule decides whether the phone spends **0.06 cores or 0.68** (measured on the aircraft,
 * 2026-07-28: 0.49 attached-and-idle and 1.11 running, both over a 0.43 floor), on a device clamped
 * to a controller in the sun, on an airframe whose characteristic failure is an overheat go-home.
 * That is why it is a rule with tests rather than a boolean somebody flips.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | a new flight is never reported | **20** |
 * | takeoff does not open the acquisition window | **19** |
 * | AUTO arms on descent with no latch — geometry instead of evidence | 5 |
 * | the acquisition window never closes | 3 |
 * | OFF still arms | 3 |
 * | ON does not arm on the ground | 2 |
 * | the ceiling reverts to the baseline's 3 m | 2 |
 * | unknown altitude counts as inside the band | 1 |
 *
 * The two large counts are not a better-tested property, they are a *shared* one: the acquisition
 * window and the new-flight edge are read by almost every row here and by `TagRecogniserTest`, so
 * breaking either shows up everywhere. The single-failure row at the bottom is the one that is
 * pinned by exactly one test, and it is the standing "unknown is not zero" rule.
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 *
 * ## Mutations killed, measured 2026-07-30 (the landing16 campaign)
 *
 * Whole suite per mutant, **2644 tests**, `test-results` deleted first, confirmed red, reverted.
 * **No survivors.** The engine-side rows of the same campaign — the flag's owner, and the sequence
 * that sets it — live in `guided/GuidedPrecisionLandTest` and `guided/GuidedMissionTest`.
 *
 * | mutation | failures |
 * |---|---|
 * | the `latched` gate dropped wholesale (the CPU rule deleted) | **5** |
 * | the detector ceiling reverts to the reliable band's 8.0 m | 4 |
 * | a commanded tag landing ignored — landing16's own failure | 3 |
 * | the operator's OFF override loses to a commanded landing | 1 |
 *
 * ### What the numbers say
 *
 * **The CPU rule is the best-defended thing in this file (5), and that is the point of the whole
 * change.** The new rung widens *one* case — a landing this bridge has committed to — and the row
 * above proves it did not quietly become "always on": with no landing commanded, no latch is still
 * no detector, at every height, descending, returning or landing.
 *
 * **The commanded-landing rung scores 3** rather than the double figures the acquisition window
 * scores, and that is honest rather than thin: it is a *new* branch with no shared state, so only
 * the rows written for it can see it. The failure it prevents is not subtle —
 * `datasets/landing16/20260730-161329.001.jsonl` has **zero `tag` lines in 241 s** against 617 on
 * the same morning's manual flight (`20260730-101110`), and the sequence that could not arm was
 * hovering over the pad with the camera pointing straight down at it.
 *
 * **The OFF row scores 1, and it is the row to re-run first after any edit to this file.** The
 * override is read in the outer `when(mode)`, above every rung of the AUTO ladder, and a rung added
 * *above* that `when` — which is the natural way to write "this always arms" — would take the
 * detector out of the operator's hands on the exact flight where the phone is hot.
 */
class TagArmingTest {

    private val s = 1_000_000_000L

    private fun view(
        flying: Boolean = true,
        alt: Double? = 2.0,
        returning: Boolean = false,
        landing: Boolean = false,
        landingOnTag: Boolean = false,
    ) = FlightView(flying, alt, returning, landing, landingOnTag)

    private fun auto(
        state: TagArmingState,
        view: FlightView,
        latched: Boolean = false,
        now: Long = 0,
    ) = TagArming.step(TagArm.AUTO, state, view, latched, now)

    // ────────────────────────────────────────────────────────── on the ground

    @Test
    fun onTheGroundAutoDoesNotArm() {
        val d = auto(TagArmingState(), view(flying = false, alt = 0.0))
        assertFalse(d.armed)
        assertEquals("on the ground", d.why)
        assertNull("a grounded aircraft holds no flight", d.state.flightStartedNanos)
    }

    /** Landing resets the state, so the next takeoff gets a fresh acquisition window. */
    @Test
    fun landingClearsTheFlightState() {
        val flying = auto(TagArmingState(), view(), now = 10 * s).state
        val landed = auto(flying, view(flying = false, alt = 0.0), now = 20 * s).state
        assertEquals(TagArmingState(), landed)
    }

    // ─────────────────────────────────────────────────── the acquisition window

    @Test
    fun takingOffOpensTheWindowAndArms() {
        val d = auto(TagArmingState(), view(alt = 0.5), now = 5 * s)
        assertTrue(d.armed)
        assertEquals("acquiring on takeoff", d.why)
        assertEquals(5 * s, d.state.flightStartedNanos)
        assertTrue("a takeoff is a new flight", d.newFlight)
    }

    /** `newFlight` is an edge, once. A latch reset on every tick would erase the latch. */
    @Test
    fun newFlightIsAnEdgeAndNotALevel() {
        val first = auto(TagArmingState(), view(), now = s)
        assertTrue(first.newFlight)
        val second = auto(first.state, view(), now = 2 * s)
        assertFalse(second.newFlight)
    }

    /** It arms with no latch at all — the whole point is that this is the acquisition opportunity. */
    @Test
    fun theWindowArmsWithoutAnyEvidence() {
        val d = auto(TagArmingState(flightStartedNanos = 0), view(alt = 1.5), latched = false, now = 10 * s)
        assertTrue(d.armed)
    }

    /** Climbing out of the band closes the window, whatever the clock says. */
    @Test
    fun climbingAboveTheCeilingClosesTheWindow() {
        val open = TagArmingState(flightStartedNanos = 0)
        val climbed = auto(open, view(alt = 11.0), now = 10 * s)
        assertTrue("the state must remember it left the band", climbed.state.leftAcquisitionBand)
        assertFalse(climbed.armed)
        assertEquals("no tag seen this flight", climbed.why)
        // And coming back down does not reopen it — the window is a takeoff window, not an altitude
        // band. Below the ceiling with nothing latched is still not a reason to spend CPU.
        val back = auto(climbed.state, view(alt = 2.0), now = 20 * s)
        assertFalse(back.armed)
    }

    /**
     * **The ceiling is the measured decode *reach*, not the reliable band and not the baseline's
     * 3 m.** The 3 m was OpenCV's number; the 8.0 m that replaced it was the 100 %-rate cliff plus
     * a metre, which answers `TagDescentGuidance.ARM_CEILING_M`'s question rather than this one.
     *
     * The number that matters is 10 m: 1.3 % per-frame decode at 9–10 m and **0 % in all 60 frames
     * of the 10–11 m band** (`docs/measurements/2026-07-27-apriltag-c-vs-opencv.md` §1), which is
     * the same measurement `TagDescentGuidance.APPROACH_CEILING_M` is built on — the two constants
     * now differ by the barometer's measured wander and by nothing else.
     */
    @Test
    fun theCeilingIsTheMeasuredDecodeReach() {
        assertEquals(10.0, TagArming.ACQUIRE_CEILING_M, 0.0)
        val open = TagArmingState(flightStartedNanos = 0)
        // 7.0 m is the highest band with 100 % detection on the measured flight; it must stay inside.
        assertFalse(auto(open, view(alt = 7.0), now = s).state.leftAcquisitionBand)
        // 8.8 m is where landing16's precision landing actually arrived and asked to arm
        // (`datasets/landing16/20260730-161329.001.jsonl`, t=188.8) — inside
        // `PrecisionLand.LAND_TAG_ARM_HEIGHT_M` ± `VERTICAL_ACCEPT_M`, and above the old 8.0 m
        // ceiling. A detector that is off at the heights arms are taken at cannot serve them.
        assertFalse(
            "the precision-land arm window (7-9 m) must be inside the band",
            auto(open, view(alt = 8.8), now = s).state.leftAcquisitionBand,
        )
        assertTrue(auto(open, view(alt = 10.5), now = s).state.leftAcquisitionBand)
    }

    /**
     * **The two ceilings rest on one measurement.** `APPROACH_CEILING_M` is this constant plus the
     * measured ~2 m of within-session barometric wander, so an arm is permitted a little above the
     * heights that decode, and the detector is not run where nothing decodes. The relationship is
     * asserted rather than left to two KDocs that could drift apart — the silent contradiction
     * between them (arm to 12 m, look only to 8 m) is what this row exists to prevent recurring.
     */
    @Test
    fun theDetectorCeilingSitsUnderTheArmCeilingByTheBaroMargin() {
        assertTrue(
            "the detector must not be run above the heights an arm is even permitted at",
            TagArming.ACQUIRE_CEILING_M <
                com.dimensional.mini4pro.guided.TagDescentGuidance.APPROACH_CEILING_M,
        )
        assertEquals(
            "the gap is the barometer's measured wander, ~2 m",
            2.0,
            com.dimensional.mini4pro.guided.TagDescentGuidance.APPROACH_CEILING_M -
                TagArming.ACQUIRE_CEILING_M,
            1e-9,
        )
    }

    /** The window also closes on time, for the flight that never climbs. */
    @Test
    fun theWindowExpiresOnTime() {
        val open = TagArmingState(flightStartedNanos = 0)
        assertTrue(auto(open, view(alt = 1.0), now = TagArming.ACQUIRE_WINDOW_NANOS).armed)
        val expired = auto(open, view(alt = 1.0), now = TagArming.ACQUIRE_WINDOW_NANOS + 1)
        assertFalse(expired.armed)
        assertEquals("no tag seen this flight", expired.why)
    }

    // ──────────────────────────────────────────── evidence, not geometry

    /**
     * **The rule's whole point.** Low, descending, over the pad — and nothing was seen on the way
     * up, so nothing is spent on the way down.
     */
    @Test
    fun withoutALatchADescentDoesNotArm() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        for (v in listOf(view(alt = 1.0), view(alt = 3.0), view(returning = true), view(landing = true))) {
            val d = auto(cruised, v, latched = false, now = 600 * s)
            assertFalse("armed on $v with no evidence", d.armed)
            assertEquals("no tag seen this flight", d.why)
        }
    }

    @Test
    fun withALatchDescendingIntoTheBandArms() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val d = auto(cruised, view(alt = 5.0), latched = true, now = 600 * s)
        assertTrue(d.armed)
        assertEquals("below 10 m, tag latched", d.why)
    }

    // ─────────────────────────────────────── a commanded tag landing, landing16's rung

    /**
     * **The landing16 row.** A tag landing this bridge is flying arms the detector with **no latch
     * at all** and with the acquisition band long closed — the exact state landing16 was in when it
     * refused itself (`datasets/landing16/20260730-161329.001.jsonl`: takeoff at t=54.2 with the
     * camera never commanded to nadir, the one-way band exit at t=62.0, `tag_descent_denied
     * NO_TAG_LATCHED` at t=188.8 over a pad the camera was pointing straight at, and **zero `tag`
     * lines in 241 s**).
     *
     * The reason string names the new cause, because the record has to say why 0.68 cores are
     * running.
     */
    @Test
    fun aCommandedTagLandingArmsWithNoLatchAndTheBandClosed() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val d = auto(cruised, view(alt = 8.8, landingOnTag = true), latched = false, now = 600 * s)
        assertTrue("a commanded tag landing must arm on no evidence", d.armed)
        assertEquals("landing on the tag", d.why)
    }

    /**
     * **Regardless of altitude, exactly as return and landing are** — the precision-`NAV_LAND`
     * sequence sets it when it aims the camera at the *item's* height (15.5 m in landing16), and
     * the whole point is that the detector is already looking on the way down rather than starting
     * when it arrives.
     */
    @Test
    fun aCommandedTagLandingArmsFromAnyHeight() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        for (alt in listOf(15.5, 11.0, 8.8, 5.0)) {
            val d = auto(cruised, view(alt = alt, landingOnTag = true), latched = false, now = 600 * s)
            assertTrue("not armed at $alt m during a commanded tag landing", d.armed)
            assertEquals("landing on the tag", d.why)
        }
    }

    /**
     * **On the ground it still refuses.** The rung sits under the `flying` conjunct deliberately:
     * a landing run that outlived its flight is a bug in whoever owns the flag, not a reason to
     * burn CPU here.
     */
    @Test
    fun aCommandedTagLandingDoesNotArmOnTheGround() {
        val d = auto(TagArmingState(), view(flying = false, alt = 0.0, landingOnTag = true))
        assertFalse(d.armed)
        assertEquals("on the ground", d.why)
    }

    /**
     * **The CPU rule survives whole for ordinary cruise.** With no landing commanded, no latch is
     * still no detector — at every height, descending, returning or landing. This is the row that
     * says the new rung widened one case rather than deleting the rule.
     */
    @Test
    fun withoutACommandedLandingTheLatchRuleIsUntouched() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val views = listOf(
            view(alt = 1.0), view(alt = 3.0), view(alt = 9.5), view(alt = 40.0),
            view(returning = true), view(landing = true), view(alt = null),
        )
        for (v in views) {
            val d = auto(cruised, v, latched = false, now = 600 * s)
            assertFalse("armed on $v with no evidence and no commanded landing", d.armed)
        }
    }

    /**
     * **Unknown altitude is still not low, even mid-landing** — but for the opposite reason to the
     * latched case, and it is worth being explicit about which rule is doing the work. The landing
     * rung arms on a *commanded* fact and never consults the altitude at all, so a silent altitude
     * feed cannot change its answer; what must stay true is that the `inBand` fallback below it
     * still refuses an unknown height, which the latched row pins.
     */
    @Test
    fun aCommandedTagLandingDoesNotNeedAnAltitudeAndDoesNotInventOne() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val d = auto(cruised, view(alt = null, landingOnTag = true), latched = true, now = 600 * s)
        assertTrue(d.armed)
        assertEquals("landing on the tag", d.why)
        // Without the commanded landing the same unknown altitude refuses, unchanged.
        val without = auto(cruised, view(alt = null), latched = true, now = 600 * s)
        assertFalse(without.armed)
        assertEquals("above the band", without.why)
    }

    /**
     * Return and landing arm it **regardless of altitude**, so the detector is already looking by
     * the time the aircraft is low enough to see anything rather than starting when it gets there.
     */
    @Test
    fun withALatchAReturnArmsFromAnyHeight() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val returning = auto(cruised, view(alt = 40.0, returning = true), latched = true, now = 600 * s)
        assertTrue(returning.armed)
        assertEquals("returning, tag latched", returning.why)
        val landing = auto(cruised, view(alt = 40.0, landing = true), latched = true, now = 600 * s)
        assertTrue(landing.armed)
        assertEquals("landing, tag latched", landing.why)
    }

    @Test
    fun withALatchButHighAndNotComingBackItStaysOff() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val d = auto(cruised, view(alt = 30.0), latched = true, now = 600 * s)
        assertFalse(d.armed)
        assertEquals("above the band", d.why)
    }

    /**
     * **Unknown altitude is not low.** The project's standing rule, applied here: a detector that
     * armed whenever telemetry went quiet would run for the whole of any flight with a bad altitude
     * feed, which is the flight least able to afford it.
     */
    @Test
    fun unknownAltitudeIsNotTreatedAsInTheBand() {
        val cruised = TagArmingState(flightStartedNanos = 0, leftAcquisitionBand = true)
        val d = auto(cruised, view(alt = null), latched = true, now = 600 * s)
        assertFalse(d.armed)
        assertEquals("above the band", d.why)
    }

    /** Unknown altitude also cannot close the acquisition window, for the same reason. */
    @Test
    fun unknownAltitudeDoesNotCloseTheWindow() {
        val open = TagArmingState(flightStartedNanos = 0)
        val d = auto(open, view(alt = null), now = 10 * s)
        assertFalse(d.state.leftAcquisitionBand)
        assertTrue(d.armed)
    }

    // ───────────────────────────────────────────────────────────── overrides

    /**
     * **The operator's OFF beats everything, including a commanded landing.** The override is read
     * in the outer `when(mode)`, above every rung of the AUTO ladder, and that ordering is the
     * whole content of this row: OFF exists because a phone clamped to a controller in the sun can
     * overheat an airframe whose characteristic failure is a go-home, and a hand reaching for that
     * switch must not be outranked by anything this file decides — least of all by the rung that
     * was added to make the detector run *more*.
     */
    @Test
    fun offNeverArmsWhateverTheAircraftIsDoing() {
        val states = listOf(TagArmingState(), TagArmingState(flightStartedNanos = 0))
        val views = listOf(
            view(), view(flying = false), view(landing = true), view(alt = 0.5),
            view(landingOnTag = true), view(alt = 8.8, landingOnTag = true),
            view(alt = 0.5, landingOnTag = true),
        )
        for (st in states) for (v in views) {
            val d = TagArming.step(TagArm.OFF, st, v, latched = true, nowNanos = 5 * s)
            assertFalse("armed under OFF for $v", d.armed)
            assertEquals("off (operator)", d.why)
        }
    }

    @Test
    fun onArmsEvenOnTheGroundWhichIsWhatABenchRunNeeds() {
        val d = TagArming.step(TagArm.ON, TagArmingState(), view(flying = false, alt = 0.0), false, 0)
        assertTrue(d.armed)
        assertEquals("on (operator)", d.why)
    }

    /**
     * **The overrides do not freeze the state machine.** Held OFF through a takeoff and a climb,
     * then released, the rule must be where the flight is — not where it was when the switch moved.
     */
    @Test
    fun anOverrideStillStepsTheUnderlyingState() {
        var st = TagArmingState()
        // Takeoff, held off.
        val off1 = TagArming.step(TagArm.OFF, st, view(alt = 0.5), false, 0)
        assertTrue("a takeoff is a new flight even under OFF", off1.newFlight)
        st = off1.state
        // Climb out, still off.
        st = TagArming.step(TagArm.OFF, st, view(alt = 20.0), false, 10 * s).state
        assertTrue(st.leftAcquisitionBand)
        // Released at altitude with nothing latched: the window is gone, so it stays off.
        val released = auto(st, view(alt = 20.0), latched = false, now = 20 * s)
        assertFalse(released.armed)
        assertEquals("no tag seen this flight", released.why)
    }

    @Test
    fun theWindowIsTwoMinutes() {
        // Long enough for a bench hover, short enough that it cannot become "always on": about a
        // quarter of this airframe's battery.
        assertEquals(120L, TagArming.ACQUIRE_WINDOW_NANOS / s)
    }
}
