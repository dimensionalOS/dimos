package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.guided.GuidedEnvelope
import com.dimensional.mini4pro.guided.MissionGuidance
import com.dimensional.mini4pro.guided.MissionStepKind
import com.dimensional.mini4pro.guided.ResumeBlock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * §7.2's launch admission, and the translation from a stored plan into a route.
 *
 * Pure, with no fakes at all: [LaunchInputs] is a data class and [MissionLaunch.evaluate] has no
 * clock and no aircraft, which is the whole reason those two types are shaped the way they are.
 *
 * The rule under test, stated once: **a mission that violates a bound is refused before it flies,
 * not aborted halfway** — a halfway abort leaves an aircraft somewhere nobody planned, which is the
 * worst of both outcomes. And a failed check changes **nothing**: no engage, no setpoint, no mode
 * claim, and one sentence naming the **first** failure, because a list of five reasons is a list
 * nobody reads.
 *
 * Written to fail loudly for:
 *
 *  - **any check removed**, which is the mutation family §10.1 names for this area.
 *  - the `DO_SET_HOME` block removed — the silent vertical error M4-14 exists to prevent.
 *  - the battery floor removed.
 *  - the corner allowance dropped from the distance-from-home bound, so a plan cleared at the desk
 *    could be violated by the corner-cutting the design itself chose.
 *  - the rejoin gate removed, or a rejoin flown as a pass-through rather than as a resting leg.
 *  - an item this executor cannot fly being **silently skipped** rather than refused.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one at a time, applied to the shipped source, the **whole** suite
 * run, confirmed red, reverted. Counts are failing tests across all 1802 — measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the interlock check removed | 2 |
 *  | the battery floor removed | 1 |
 *  | the `DO_SET_HOME` block removed | 2 |
 *  | the home-distance bound removed | 2 |
 *  | the corner allowance dropped from that bound | 1 |
 *  | the ceiling re-check removed at Start | 1 |
 *  | the rejoin bound removed | 1 |
 *  | the plan-id check dropped from a resume | 1 |
 *  | a rejoin reported to the engine as an ordinary start | 2 |
 *  | the aircraft-state-matches-first-item check removed | 1 |
 *  | an unflyable item skipped instead of refused | 2 |
 *  | the last waypoint's `rest` flag dropped here alone | **0 — alive on purpose** |
 *
 * The last row is one half of a two-layer property: `MissionRoute.of` coerces the final step to rest
 * whatever this gate says, so each masks the other exactly. `MissionGuidanceTest`'s table carries the
 * combined measurement (2) and the argument for keeping both.
 */
class MissionLaunchTest {

    private companion object {
        const val LAT = MissionFixtures.LAT
        const val LON = MissionFixtures.LON

        fun latNorthOf(metres: Double): Double = LAT + metres / MissionGeo.METRES_PER_DEG

        /** The store, with one committed plan. A real store, because commit is where legs resolve. */
        fun planOf(items: List<StoredItem>): MissionPlan = MissionStore().commit(
            items = items,
            homeAtUpload = GeoPoint(LAT, LON),
            amslDatumAtUpload = 100.0,
            uploadedAtMs = 0L,
        )

        /** A flying aircraft over home, everything fresh, with a healthy battery and a plan loaded. */
        fun inputs(
            plan: MissionPlan? = planOf(listOf(MissionFixtures.waypoint(0, 20.0), MissionFixtures.waypoint(1, 40.0))),
            interlockOn: Boolean = true,
            resuming: Boolean = false,
            cursorSeq: Int? = null,
            planIdAtPause: Int? = null,
            resumeBlock: ResumeBlock? = null,
            fix: Pair<Double, Double>? = LAT to LON,
            positionFresh: Boolean = true,
            linkAlive: Boolean = true,
            relativeAltitudeM: Double? = 20.0,
            amslDatumM: Double? = 100.0,
            homeSet: Boolean? = true,
            homeLatDeg: Double? = LAT,
            homeLonDeg: Double? = LON,
            batteryPercent: Int? = 80,
            isFlying: Boolean? = true,
            homeMoved: Boolean = false,
            takeoffAvailable: Boolean = true,
        ) = LaunchInputs(
            interlockOn = interlockOn,
            plan = plan,
            resuming = resuming,
            cursorSeq = cursorSeq,
            planIdAtPause = planIdAtPause,
            resumeBlock = resumeBlock,
            fix = fix,
            positionFresh = positionFresh,
            linkAlive = linkAlive,
            relativeAltitudeM = relativeAltitudeM,
            amslDatumM = amslDatumM,
            homeSet = homeSet,
            homeLatDeg = homeLatDeg,
            homeLonDeg = homeLonDeg,
            batteryPercent = batteryPercent,
            isFlying = isFlying,
            homeMovedThisSession = homeMoved,
            takeoffAvailable = takeoffAvailable,
        )

        fun refusal(verdict: LaunchVerdict): String =
            (verdict as? LaunchVerdict.Refused)?.reason ?: "CLEARED"
    }

    // ------------------------------------------------------------------ the happy path

    @Test
    fun `a plain two-waypoint plan over home clears, and only the last waypoint rests`() {
        val verdict = MissionLaunch.evaluate(inputs())
        val cleared = verdict as LaunchVerdict.Cleared
        assertEquals(2, cleared.route.size)
        assertEquals(0, cleared.startIndex)
        assertFalse(cleared.rejoining)
        // Ivan's fourth answer, in the shape the engine reads it: fly through, rest at the last.
        assertFalse("the first waypoint must be flown through", cleared.route[0].rest)
        assertTrue("the last waypoint must be rested at", cleared.route[1].rest)
    }

    // ------------------------------------------------------------------ the checks, in order

    @Test
    fun `the interlock is checked first, because without it nothing engages`() {
        assertEquals(
            MissionLaunch.REASON_INTERLOCK,
            refusal(MissionLaunch.evaluate(inputs(interlockOn = false, batteryPercent = 1, fix = null))),
        )
    }

    @Test
    fun `no committed plan is refused`() {
        assertEquals(MissionLaunch.REASON_NO_PLAN, refusal(MissionLaunch.evaluate(inputs(plan = null))))
    }

    @Test
    fun `a stale or absent position fix is refused - nothing is flown from a cached one`() {
        assertEquals(MissionLaunch.REASON_NO_FIX, refusal(MissionLaunch.evaluate(inputs(fix = null))))
        assertEquals(
            MissionLaunch.REASON_NO_FIX,
            refusal(MissionLaunch.evaluate(inputs(positionFresh = false))),
        )
    }

    @Test
    fun `A PARKED AIRCRAFT'S POSITION IS NOT A LOST ONE - a takeoff plan starts from the ground`() {
        // **The bench failure of 2026-07-27, and the reason this rule is split.** DJI's position key
        // is change-driven: a parked aircraft does not move, so nothing is published, so `POSITION`
        // reads stale within a second of being set down and stays that way. Measured: 38 consecutive
        // `GLOBAL_POSITION_INT` messages while parked, every one a good coordinate, one distinct
        // value. With a flat freshness rule a plan that begins with a takeoff can never start —
        // this check wants a moving aircraft and the takeoff check wants a stationary one — and QGC
        // answered `no position fix` with a full-strength fix on its own screen.
        val plan = planOf(MissionFixtures.takeoffAndHoldPlan())
        val verdict = MissionLaunch.evaluate(
            inputs(plan = plan, isFlying = false, positionFresh = false, linkAlive = true)
        )
        assertTrue("a takeoff plan could not start from the ground: ${refusal(verdict)}", verdict is LaunchVerdict.Cleared)
    }

    @Test
    fun `THE RELAXATION IS THE GROUND'S ALONE - a stale fix in the air is still refused`() {
        // The half that must not move. In the air a quiet position feed is a controller flying
        // blind, and no amount of link health makes it otherwise.
        assertEquals(
            MissionLaunch.REASON_NO_FIX,
            refusal(MissionLaunch.evaluate(inputs(isFlying = true, positionFresh = false, linkAlive = true))),
        )
        // Nor does it apply when DJI has not said where the aircraft is. A null is not a "false".
        assertEquals(
            MissionLaunch.REASON_NO_FIX,
            refusal(MissionLaunch.evaluate(inputs(isFlying = null, positionFresh = false, linkAlive = true))),
        )
    }

    @Test
    fun `on the ground a dead link is still refused - silence needs someone to be there`() {
        // What replaces freshness on the ground is *liveness*, and it has to be a signal that does
        // not go quiet along with the data. A parked aircraft with the link gone looks exactly like
        // a parked aircraft with the link up, on every telemetry key this airframe has.
        val plan = planOf(MissionFixtures.takeoffAndHoldPlan())
        assertEquals(
            MissionLaunch.REASON_NO_FIX,
            refusal(MissionLaunch.evaluate(
                inputs(plan = plan, isFlying = false, positionFresh = false, linkAlive = false)
            )),
        )
    }

    @Test
    fun `no published AMSL datum is refused, verbatim as Stage B refuses it`() {
        assertEquals(
            MissionLaunch.REASON_NO_DATUM,
            refusal(MissionLaunch.evaluate(inputs(amslDatumM = null))),
        )
        // A wholly relative-alt plan still needs `relativeAltitude` to be live: it is the number the
        // ceiling is enforced against, and enforcing a ceiling blind is not enforcing it.
        assertEquals(
            MissionLaunch.REASON_NO_DATUM,
            refusal(MissionLaunch.evaluate(inputs(relativeAltitudeM = null))),
        )
    }

    @Test
    fun `M4-14 - a DO_SET_HOME anywhere in the session blocks mission start`() {
        // QGC's plan altitudes are relative to **home**; our relativeAltitude is relative to the
        // **takeoff point**. The same number normally, different after a DO_SET_HOME — silently, in
        // the vertical axis, in the direction that ends in the ground.
        assertEquals(
            MissionLaunch.REASON_HOME_MOVED,
            refusal(MissionLaunch.evaluate(inputs(homeMoved = true))),
        )
    }

    @Test
    fun `home must be known, and a filler coordinate is not home`() {
        assertEquals(
            MissionLaunch.REASON_HOME_UNKNOWN,
            refusal(MissionLaunch.evaluate(inputs(homeSet = false))),
        )
        assertEquals(
            MissionLaunch.REASON_HOME_UNKNOWN,
            refusal(MissionLaunch.evaluate(inputs(homeLatDeg = null))),
        )
        // DJI's 4.58e7 placeholder, which `Geo.coordinateOrNull` already refuses everywhere else.
        assertEquals(
            MissionLaunch.REASON_HOME_UNKNOWN,
            refusal(MissionLaunch.evaluate(inputs(homeLatDeg = 4.58e7, homeLonDeg = 4.58e7))),
        )
    }

    @Test
    fun `the battery floor refuses a start rather than intervening in a flight`() {
        assertEquals(
            MissionLaunch.REASON_BATTERY,
            refusal(MissionLaunch.evaluate(inputs(batteryPercent = MissionGuidance.BATTERY_START_MIN_PCT - 1))),
        )
        // An unknown battery is refused too: the coarse signal is safe *because* it refuses.
        assertEquals(
            MissionLaunch.REASON_BATTERY,
            refusal(MissionLaunch.evaluate(inputs(batteryPercent = null))),
        )
        assertTrue(
            MissionLaunch.evaluate(inputs(batteryPercent = MissionGuidance.BATTERY_START_MIN_PCT))
                is LaunchVerdict.Cleared
        )
    }

    @Test
    fun `the aircraft's own state must match what the plan starts with`() {
        // A plan that begins with a waypoint needs an aircraft that is already flying.
        assertEquals(
            MissionLaunch.REASON_ON_THE_GROUND,
            refusal(MissionLaunch.evaluate(inputs(isFlying = false))),
        )
        assertEquals(
            MissionLaunch.REASON_ON_THE_GROUND,
            refusal(MissionLaunch.evaluate(inputs(isFlying = null))),
        )
        // ...and a plan that begins with a takeoff needs one that is not.
        val withTakeoff = planOf(MissionFixtures.takeoffAndHoldPlan())
        assertEquals(
            MissionLaunch.REASON_ALREADY_FLYING,
            refusal(MissionLaunch.evaluate(inputs(plan = withTakeoff, isFlying = true))),
        )
        assertTrue(
            MissionLaunch.evaluate(inputs(plan = withTakeoff, isFlying = false)) is LaunchVerdict.Cleared
        )
    }

    @Test
    fun `a plan beginning with a takeoff is refused when nothing can start one`() {
        assertEquals(
            MissionLaunch.REASON_NO_TAKEOFF,
            refusal(
                MissionLaunch.evaluate(
                    inputs(
                        plan = planOf(MissionFixtures.takeoffAndHoldPlan()),
                        isFlying = false,
                        takeoffAvailable = false,
                    )
                )
            ),
        )
    }

    // ------------------------------------------------------- the distance and height bounds

    @Test
    fun `an item beyond the home bound is refused, naming the item`() {
        // 2 km since 2026-07-30 (Ivan), so the fixture is derived from the bound rather than from
        // the 200 m that used to be past it.
        val far = planOf(
            listOf(
                MissionFixtures.waypoint(0, 20.0),
                MissionFixtures.waypoint(1, MissionGuidance.MAX_HOME_DIST_M + 50.0),
            )
        )
        assertEquals(
            MissionLaunch.reasonTooFar(1),
            refusal(MissionLaunch.evaluate(inputs(plan = far))),
        )
    }

    @Test
    fun `THE CORNER ALLOWANCE - the home bound is tightened by what the corner-cutting will add`() {
        // A plan admitted at the desk must not be violated by the corner-cutting §3.3 chose. The
        // enforced bound is MAX_HOME_DIST_M minus the allowance, so a waypoint between the two is
        // refused even though it is inside the raw number.
        val between = MissionGuidance.MAX_HOME_DIST_M - MissionGuidance.CORNER_ALLOWANCE_M / 2.0
        val plan = planOf(
            listOf(MissionFixtures.waypoint(0, 10.0), MissionFixtures.waypoint(1, between))
        )
        assertEquals(MissionLaunch.reasonTooFar(1), refusal(MissionLaunch.evaluate(inputs(plan = plan))))
        // And just inside the tightened bound, it clears.
        val inside = MissionGuidance.MAX_HOME_DIST_M - MissionGuidance.CORNER_ALLOWANCE_M - 1.0
        val ok = planOf(listOf(MissionFixtures.waypoint(0, 10.0), MissionFixtures.waypoint(1, inside)))
        assertTrue(MissionLaunch.evaluate(inputs(plan = ok)) is LaunchVerdict.Cleared)
    }

    @Test
    fun `the ceiling is re-checked at Start, because the datum drifts`() {
        // Admission checked this at upload against the datum of the day; the barometric datum moves
        // 2.3 m in twelve minutes and 41.5 m between sessions, so a plan admitted twenty minutes ago
        // was admitted against a different number.
        val high = planOf(
            listOf(
                MissionFixtures.waypoint(0, 10.0),
                MissionFixtures.waypoint(1, 20.0, z = (GuidedEnvelope.CEILING_M + 5.0).toFloat()),
            )
        )
        assertEquals(MissionLaunch.reasonTooHigh(1), refusal(MissionLaunch.evaluate(inputs(plan = high))))
    }

    // ------------------------------------------------------------------ what is not flown

    @Test
    fun `an orbit item is refused at Start, naming the item, never silently skipped`() {
        // `MissionAdmission` accepts DO_ORBIT into the store; this executor does not sequence one,
        // because an orbit's completion is neither of §3.1's two tests. Refused out loud rather than
        // dropped — a skipped item is a plan the operator did not author.
        val plan = planOf(
            listOf(
                MissionFixtures.waypoint(0, 10.0),
                MissionFixtures.item(
                    1, MissionCommands.DO_ORBIT,
                    param1 = 20f, param2 = Float.NaN, param3 = 5f, param4 = Float.NaN,
                    x = MissionFixtures.northOf(20.0), y = MissionFixtures.latE7(LON),
                ),
                MissionFixtures.waypoint(2, 30.0),
            )
        )
        assertEquals(MissionLaunch.reasonNotFlyable(1), refusal(MissionLaunch.evaluate(inputs(plan = plan))))
    }

    @Test
    fun `a timed loiter or a delay is refused, because a cursor may not advance on a clock`() {
        val loiter = planOf(
            listOf(
                MissionFixtures.waypoint(0, 10.0),
                MissionFixtures.item(
                    1, MissionCommands.NAV_LOITER_TIME, param1 = 30f,
                    x = MissionFixtures.northOf(20.0), y = MissionFixtures.latE7(LON),
                ),
            )
        )
        assertEquals(MissionLaunch.reasonNotFlyable(1), refusal(MissionLaunch.evaluate(inputs(plan = loiter))))

        val delay = planOf(
            listOf(
                MissionFixtures.waypoint(0, 10.0),
                MissionFixtures.item(1, MissionCommands.NAV_DELAY, param1 = 5f),
            )
        )
        assertEquals(MissionLaunch.reasonNotFlyable(1), refusal(MissionLaunch.evaluate(inputs(plan = delay))))
    }

    // ------------------------------------------------------------------ land and RTL hold

    @Test
    fun `M4-5 - a land item that does not ask for precision becomes a hold commanding no height`() {
        // `takeoffAndHoldPlan`'s land item carries QGC's default `param2 = 0`, which is the whole
        // forward-compatibility property: a plan that does not ask to land does not land.
        val plan = planOf(MissionFixtures.takeoffAndHoldPlan())
        val cleared = MissionLaunch.evaluate(inputs(plan = plan, isFlying = false)) as LaunchVerdict.Cleared
        val landing = cleared.route[cleared.route.size - 1]
        assertEquals(MissionStepKind.HOLD, landing.kind)
        assertTrue(landing.rest)
        // Null, deliberately: the aircraft holds whatever altitude it arrives with. A height here
        // would be this layer quietly promising a descent, which is the capability M4-5 removed.
        assertEquals(null, landing.relAltM)
    }

    @Test
    fun `a precision land item becomes a PRECISION_LAND step that keeps its own altitude`() {
        // big1.plan's item 7: `param2 = 2`, frame 3, 15 m, drawn 0.3 m from the planned home. The step
        // keeps the drawn coordinate (the site cross-check's subject, never a target — the flown XY is
        // the recorded takeoff point, resolved when the item begins) and it keeps the altitude, which
        // for this step is an ordinary waypoint altitude.
        for (mode in listOf(PrecisionLandMode.OPPORTUNISTIC, PrecisionLandMode.REQUIRED)) {
            val plan = planOf(
                listOf(
                    MissionFixtures.item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 10f),
                    MissionFixtures.waypoint(1, 20.0, z = 60f),
                    MissionFixtures.item(
                        2, MissionCommands.NAV_LAND,
                        param2 = mode.toFloat(),
                        x = MissionFixtures.northOf(0.3), y = MissionFixtures.latE7(LON), z = 15f,
                    ),
                )
            )
            val cleared = MissionLaunch.evaluate(
                inputs(plan = plan, isFlying = false)
            ) as LaunchVerdict.Cleared
            val landing = cleared.route[cleared.route.size - 1]
            assertEquals("mode $mode", MissionStepKind.PRECISION_LAND, landing.kind)
            assertTrue(landing.rest)
            assertEquals(15.0, landing.relAltM!!, 0.0)
            // 1e-7 degrees ≈ 1 cm: MAVLink's integer coordinate is exactly that quantised.
            assertEquals(latNorthOf(0.3), landing.latDeg, 1e-7)
            assertEquals(mode, landing.precisionLandMode)
            // And the route knows the height the plan cleared — the number the sequence's own
            // do-not-start-from-below-this gate is measured against.
            assertEquals(10.0, cleared.route.takeoffRelAltM!!, 0.0)
        }
    }

    @Test
    fun `a precision land item with no usable height is refused at the desk, not flown blind`() {
        // The one case that must not become a null discovered above the pad: a relative-frame item is
        // always given a z by QGC, so this is the frame-mismatch/NaN corner, and the honest answer is a
        // sentence the operator reads before the aircraft moves.
        val plan = planOf(
            listOf(
                MissionFixtures.waypoint(0, 20.0),
                MissionFixtures.item(
                    1, MissionCommands.NAV_LAND, param2 = 2f,
                    x = 0, y = 0, z = Float.NaN,
                ),
            )
        )
        assertEquals(MissionLaunch.reasonNotFlyable(1), refusal(MissionLaunch.evaluate(inputs(plan = plan))))
    }

    @Test
    fun `M4-7 - an RTL becomes a hold at home, never DJI's own return-to-home`() {
        val plan = planOf(
            listOf(
                MissionFixtures.waypoint(0, 20.0),
                MissionFixtures.item(1, MissionCommands.NAV_RETURN_TO_LAUNCH, x = 0, y = 0, z = 0f),
            )
        )
        val cleared = MissionLaunch.evaluate(inputs(plan = plan)) as LaunchVerdict.Cleared
        val rtl = cleared.route[1]
        assertEquals(MissionStepKind.HOLD, rtl.kind)
        assertTrue(rtl.rest)
        assertEquals(null, rtl.relAltM)
        // Resolved to home here, which is the one place this bridge decides what "return to launch"
        // means: fly to home and hold.
        assertEquals(LAT, rtl.latDeg, 1e-9)
        assertEquals(LON, rtl.lonDeg, 1e-9)
    }

    @Test
    fun `a land item that is not last is refused rather than flown early`() {
        val plan = planOf(
            listOf(
                MissionFixtures.item(0, MissionCommands.NAV_LAND, x = 0, y = 0, z = 0f),
                MissionFixtures.waypoint(1, 20.0),
            )
        )
        assertEquals(MissionLaunch.reasonNotFlyable(0), refusal(MissionLaunch.evaluate(inputs(plan = plan))))
    }

    // ------------------------------------------------------------------ the resume

    @Test
    fun `THE REJOIN IS A RESTING LEG - a resume flies to the cursor and comes to rest there`() {
        // §6.3: a resume is not "continue", it is "fly a new leg to the cursor, then continue", and
        // that leg was never drawn on anybody's map. It gets the conservative completion test.
        val verdict = MissionLaunch.evaluate(
            inputs(resuming = true, cursorSeq = 1, planIdAtPause = 1)
        )
        val cleared = verdict as LaunchVerdict.Cleared
        assertTrue(cleared.rejoining)
        assertEquals(1, cleared.startIndex)
    }

    @Test
    fun `THE REJOIN GATE - a resume beyond REJOIN_MAX_M is refused`() {
        // Someone pressing Continue is picturing the aircraft roughly where it stopped. The cursor
        // is 40 m north of home; put the aircraft 100 m south of it and the rejoin leg is 140 m.
        val verdict = MissionLaunch.evaluate(
            inputs(
                resuming = true, cursorSeq = 1, planIdAtPause = 1,
                fix = latNorthOf(-100.0) to LON,
            )
        )
        assertEquals(MissionLaunch.REASON_TOO_FAR_TO_REJOIN, refusal(verdict))
        // Just inside it, and the resume clears.
        val near = MissionLaunch.evaluate(
            inputs(
                resuming = true, cursorSeq = 1, planIdAtPause = 1,
                fix = latNorthOf(40.0 - MissionGuidance.REJOIN_MAX_M + 1.0) to LON,
            )
        )
        assertTrue(near is LaunchVerdict.Cleared)
    }

    @Test
    fun `the rejoin bound is stricter than a fresh goto's, deliberately`() {
        assertTrue(MissionGuidance.REJOIN_MAX_M < GuidedEnvelope.MAX_REPOSITION_DISTANCE_M)
    }

    @Test
    fun `a resume across a plan change is refused, because the cursor indexed a plan that is gone`() {
        assertEquals(
            MissionLaunch.REASON_PLAN_CHANGED,
            refusal(MissionLaunch.evaluate(inputs(resuming = true, cursorSeq = 1, planIdAtPause = 99))),
        )
    }

    @Test
    fun `a resume to a cursor no longer in the plan is refused rather than clamped`() {
        assertEquals(
            MissionLaunch.REASON_PLAN_CHANGED,
            refusal(MissionLaunch.evaluate(inputs(resuming = true, cursorSeq = 7, planIdAtPause = 1))),
        )
    }

    @Test
    fun `a standing resume block refuses the resume, naming it`() {
        for (block in ResumeBlock.values()) {
            assertEquals(
                "block $block",
                block.reason,
                refusal(
                    MissionLaunch.evaluate(
                        inputs(resuming = true, cursorSeq = 1, planIdAtPause = 1, resumeBlock = block)
                    )
                ),
            )
        }
    }

    @Test
    fun `a block only refuses a resume, not a fresh start from LOADED`() {
        // The block belongs to the paused *run*. A plan started again from item 0 is a new flight
        // through the whole launch check, and the block that ended the last run is not a fact about
        // this one.
        assertTrue(
            MissionLaunch.evaluate(inputs(resumeBlock = ResumeBlock.LEG_TIMEOUT)) is LaunchVerdict.Cleared
        )
    }

    // ------------------------------------------------------------- the sticky ROI, carried

    @Test
    fun `THE ROI - a plan's DO_SET_ROI_LOCATION reaches the route, sticky until its DO_SET_ROI_NONE`() {
        // big1's shape: an ROI item between two waypoints and a clear after the second. The store's
        // sticky walk decides *which* leg is under it (the same walk that resolves `DO_CHANGE_SPEED`);
        // this asserts `routeOf` carries that answer instead of dropping it, which is the whole of the
        // 2026-07-30 fix. `GuidedMissionRoiTest` has the engine's half.
        // The item's own 1e7 integer, read back as degrees — the e7 round trip must be exact, because
        // the plan path re-encodes the store's decimal coordinate to reach `RoiCommand`.
        val roiLat = MissionFixtures.northOf(50.0) / MissionGeo.DEGREES_E7
        val plan = planOf(
            listOf(
                MissionFixtures.waypoint(0, 20.0),
                MissionFixtures.item(
                    1, MissionCommands.DO_SET_ROI_LOCATION,
                    x = MissionFixtures.northOf(50.0), y = MissionFixtures.eastOf(30.0), z = 12f,
                ),
                MissionFixtures.waypoint(2, 40.0),
                MissionFixtures.waypoint(3, 60.0),
                MissionFixtures.item(4, MissionCommands.DO_SET_ROI_NONE, frame = MissionFrames.MISSION),
                MissionFixtures.waypoint(5, 80.0),
            )
        )
        val route = (MissionLaunch.evaluate(inputs(plan)) as LaunchVerdict.Cleared).route

        // Four steps — the two ROI items occupy sequence numbers and are not places — and the ROI is on
        // the two legs between its own item and the clear. Not on the first: applying it there would be
        // a leg early, which is the off-by-one this shape exists to prevent.
        assertEquals(listOf(0, 2, 3, 5), route.steps.map { it.seq })
        assertEquals(listOf(2, 3), route.steps.filter { it.roi != null }.map { it.seq })

        // The item's own numbers, and its `z`: frame 3 means metres above *our* takeoff datum, so it is
        // usable and is kept. `RoiCommand.relativeAltMOrNull` is the single owner of that decision and
        // this asserts the plan path reaches it rather than re-deciding.
        val roi = route.steps.first { it.roi != null }.roi!!
        assertEquals(roiLat, roi.targetDegrees()!!.first, 1e-9)
        assertEquals(12.0, roi.relativeAltMOrNull()!!, 0.0)
    }

    @Test
    fun `a plan with no ROI items builds a route that names none - the pre-2026-07-30 plans, unchanged`() {
        val route = (MissionLaunch.evaluate(inputs()) as LaunchVerdict.Cleared).route
        assertTrue(route.steps.all { it.roi == null })
    }

    @Test
    fun `every refusal reason fits the sentence it has to ride in`() {
        // 50 bytes total for "Mission refused: <reason>", so a reason over ~33 bytes starts eating
        // its own framing. `preferring` keeps the reason and drops the frame, but a reason that does
        // not fit at all is a reason nobody can act on.
        val reasons = listOf(
            MissionLaunch.REASON_INTERLOCK, MissionLaunch.REASON_NO_PLAN, MissionLaunch.REASON_NO_FIX,
            MissionLaunch.REASON_NO_DATUM, MissionLaunch.REASON_HOME_UNKNOWN,
            MissionLaunch.REASON_HOME_MOVED, MissionLaunch.REASON_BATTERY,
            MissionLaunch.REASON_ON_THE_GROUND, MissionLaunch.REASON_ALREADY_FLYING,
            MissionLaunch.REASON_NO_TAKEOFF, MissionLaunch.REASON_PLAN_CHANGED,
            MissionLaunch.REASON_TOO_FAR_TO_REJOIN, MissionLaunch.REASON_NOTHING_TO_FLY,
            MissionLaunch.reasonTooFar(19), MissionLaunch.reasonNotFlyable(19),
            MissionLaunch.reasonTooHigh(19),
        )
        for (reason in reasons) {
            assertTrue("reason too long: $reason", reason.toByteArray(Charsets.UTF_8).size <= 33)
        }
    }
}
