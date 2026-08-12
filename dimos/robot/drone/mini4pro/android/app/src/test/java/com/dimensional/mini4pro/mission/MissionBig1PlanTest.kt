package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.guided.MissionGuidance
import com.dimensional.mini4pro.guided.MissionStep
import com.dimensional.mini4pro.guided.MissionStepKind
import com.dimensional.mini4pro.guided.PrecisionLand
import com.dimensional.mini4pro.guided.RepositionGuidance
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Ivan's actual plan, transcribed off the wire it will arrive on, and what this bridge does with
 * it** — `/home/lesh/Documents/QGroundControl Daily/Missions/big1.plan`, sha256 `2eaa6cfd4855e43b…`,
 * 5 871 bytes, ten items, read 2026-07-30.
 *
 * This suite exists because Ivan asked for it in exactly these terms — *"should we use an actual
 * big1 plan for tests for this, see which output they generate"* — and it answers a question the
 * per-unit tests next door deliberately do not: **what happens when he presses Start on the plan he
 * actually flies?** Everything else in the mission suites tests one rule against a fixture shaped for
 * it; this tests the whole projection against one real plan, and its assertions are the *output*, not
 * a restatement of the rules.
 *
 * ## Transcribed, not parsed, and that is the honest choice
 *
 * The items below are the file's `mission.items[*].params` verbatim, converted the way QGC's
 * `PlanManager` converts them for the wire: `params[4]`/`params[5]` become 1e7-degree `x`/`y`,
 * `params[6]` becomes `z`, `params[0..3]` become `param1..4`, `null` becomes `NaN`. There is
 * deliberately **no `.plan` parser** in this project — QGC uploads over MAVLink and this bridge has
 * never read a plan file — and writing a JSON reader for a test would be testing a parser nobody
 * ships. What is asserted first, therefore, is that the transcription *is admissible*, which is the
 * same gate the real upload passes through.
 *
 * If Ivan edits the plan, this suite goes stale rather than wrong: the numbers below still describe a
 * plan of that shape. The sha256 is here so a reader can tell which version they are looking at.
 *
 * ## The verdict flipped on 2026-07-30, and that is what this suite is for
 *
 * Until that day the first two tests below asserted that this plan is **refused** — `Leg 5 is 140m:
 * 100m max` at the desk, then `item 4 too far from home` at Start. Ivan read those refusals, and
 * instead of editing the box he moved the envelope: *"2 km is our new limit"*
 * (`docs/decisions/2026-07-30-two-kilometre-envelope.md`). They now assert it is **admitted and
 * cleared**, with a refusal at the *new* bound kept beside them so the expansion cannot be mistaken
 * for a removal. **This suite is that decision's acceptance test**, which is why the change of
 * verdict is written out in the tests rather than tidied away.
 *
 * Mutation rows from that campaign, measured on the 2633-test tree, whole suite per mutant, no
 * survivors (the full table, including the envelope-side rows, is in `StickMappingTest`):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | leg bound dropped (any distance admitted between two items) | 3 |
 *  | home bound dropped at Start | 3 |
 *  | `legTooLong` stops naming the limit | 4 |
 *  | the reach bound reverted to 100 m | 11 |
 *
 * ## What it pins, and one thing it exposes
 *
 * The last of these was a **gap, deliberately pinned rather than quietly fixed**: item 6 is a
 * `DO_SET_ROI_LOCATION` and item 8 a `DO_SET_ROI_NONE`, and until 2026-07-30 **neither reached the
 * flying half at all** — `MissionStore.resolve` carried the ROI into [ResolvedLeg.roi] and
 * `MissionLaunch.routeOf` dropped it, because a `MissionStep` had no ROI field. Ivan closed it the same
 * day (*"fix mission execution to respect plan ROI"*), so the last test now asserts the opposite fact
 * from the same plan: the ROI reaches the route, on the one leg his sequencing sentence names, with its
 * frame-3 height intact. The engine's half of that feature is `GuidedMissionRoiTest`.
 */
class MissionBig1PlanTest {

    private companion object {

        /** `params[4]`/`params[5]` as MAVLink's 1e7 degrees, QGC's own conversion. */
        fun e7(degrees: Double): Int = Math.round(degrees * MissionGeo.DEGREES_E7).toInt()

        fun item(
            seq: Int, command: Int, frame: Int,
            p1: Float = 0f, p2: Float = 0f, p3: Float = 0f, p4: Float = Float.NaN,
            lat: Double = 0.0, lon: Double = 0.0, z: Float = 0f,
        ) = StoredItem(
            seq = seq, frame = frame, command = command, current = 0, autocontinue = 1,
            param1 = p1, param2 = p2, param3 = p3, param4 = p4,
            x = if (lat == 0.0) 0 else e7(lat), y = if (lon == 0.0) 0 else e7(lon), z = z,
            missionType = MissionTypes.MISSION,
        )

        /** `plannedHomePosition` — 37.99389107 / 23.72532118 at 78 m. Provenance only. */
        const val PLANNED_HOME_LAT = 37.99389107
        const val PLANNED_HOME_LON = 23.72532118

        /** The Land item's own drawn coordinate, item 9. */
        const val LAND_LAT = 37.99388856209923
        const val LAND_LON = 23.725319508042617

        /** big1.plan, item for item. */
        fun big1(): List<StoredItem> = listOf(
            // 0 — NAV_TAKEOFF, 10 m: the height every later gate calls "the height the plan cleared".
            item(0, MissionCommands.NAV_TAKEOFF, 3, lat = 37.99389107, lon = 23.72532118, z = 10f),
            // 1..5 — the box: 30 m, then four corners at 60 m.
            item(1, MissionCommands.NAV_WAYPOINT, 3, lat = 37.99389635, lon = 23.72522931, z = 30f),
            item(2, MissionCommands.NAV_WAYPOINT, 3, lat = 37.99351006, lon = 23.72519646, z = 60f),
            item(3, MissionCommands.NAV_WAYPOINT, 3, lat = 37.99277128, lon = 23.72514214, z = 60f),
            item(4, MissionCommands.NAV_WAYPOINT, 3, lat = 37.99273535, lon = 23.72610237, z = 60f),
            item(5, MissionCommands.NAV_WAYPOINT, 3, lat = 37.99399306, lon = 23.72619089, z = 60f),
            // 6 — DO_SET_ROI_LOCATION, **frame 3**, z = 0: a height in *our* datum, param4 = 0.
            item(6, MissionCommands.DO_SET_ROI_LOCATION, 3, p4 = 0f,
                lat = 37.99384298942512, lon = 23.72577905336516, z = 0f),
            // 7 — the last waypoint, 60 m, ~8 m from the pad.
            item(7, MissionCommands.NAV_WAYPOINT, 3, lat = 37.993916961081936, lon = 23.72524138553902, z = 60f),
            // 8 — DO_SET_ROI_NONE, frame 2 (MAV_FRAME_MISSION), every param zero.
            item(8, MissionCommands.DO_SET_ROI_NONE, 2, p4 = 0f),
            // 9 — NAV_LAND, **param2 = 2 (Required)**, 15 m, drawn 0.3 m from the planned home.
            item(9, MissionCommands.NAV_LAND, 3, p2 = 2f, lat = LAND_LAT, lon = LAND_LON, z = 15f),
        )

        fun planOf(items: List<StoredItem>): MissionPlan = MissionStore().commit(
            items = items,
            homeAtUpload = GeoPoint(PLANNED_HOME_LAT, PLANNED_HOME_LON),
            amslDatumAtUpload = 100.0,
            uploadedAtMs = 0L,
        )

        /** Airborne over the pad at 60 m, everything fresh — the state at item 7's completion. */
        fun inputs(plan: MissionPlan, isFlying: Boolean? = false) = LaunchInputs(
            interlockOn = true,
            plan = plan,
            resuming = false,
            cursorSeq = null,
            planIdAtPause = null,
            resumeBlock = null,
            fix = PLANNED_HOME_LAT to PLANNED_HOME_LON,
            positionFresh = true,
            linkAlive = true,
            relativeAltitudeM = 0.0,
            amslDatumM = 100.0,
            homeSet = true,
            homeLatDeg = PLANNED_HOME_LAT,
            homeLonDeg = PLANNED_HOME_LON,
            batteryPercent = 80,
            isFlying = isFlying,
            homeMovedThisSession = false,
            takeoffAvailable = true,
        )
    }

    // ------------------------------------------------------------------ the door

    @Test
    fun `THE VERDICT - big1 as authored is ADMITTED at the desk, on Ivan's 2 km envelope`() {
        // **The acceptance criterion for the 2026-07-30 envelope expansion.** This test asserted the
        // opposite until that day, and the change of verdict is the whole point of the work:
        //
        //   before: `Leg 5 is 140m: 100m max`, then `item 4 too far from home` (145.6 m against 144)
        //   after:  admitted, unchanged, exactly as authored.
        //
        // Leg 5 - item 4 (37.99273535, 23.72610237) to item 5 (37.99399306, 23.72619089) - is 140 m,
        // and `MissionAdmission.MAX_LEG_M` = `GuidedEnvelope.MAX_REPOSITION_DISTANCE_M` is now 2000 m.
        // Nothing about the plan moved; the bound did, because its owner decided it should
        // (`docs/decisions/2026-07-30-two-kilometre-envelope.md`).
        assertNull("big1 as authored must now be admitted", MissionAdmission.admissible(big1()))

        val leg5 = RepositionGuidance.horizontalMetres(
            37.99273535, 23.72610237, 37.99399306, 23.72619089,
        )
        assertEquals(140.0, leg5, 2.0)
        assertTrue("leg 5 is inside the new bound with room", leg5 < MissionAdmission.MAX_LEG_M)
        // And the whole box is inside the path bound, which also had to move: 449 m of drawn path
        // against the 4 km an out-and-back at the new reach allows.
        assertTrue(MissionAdmission.MAX_TOTAL_M >= 4_000.0)
    }

    @Test
    fun `THE VERDICT - and Start clears it too, item 4 included`() {
        // The second door, which used to refuse independently of the first: item 4 is 145.6 m from
        // the planned home, against the 144 m that `MAX_HOME_DIST_M` (150) minus the corner
        // allowance (6) enforced until 2026-07-30. The bound is now 2000 − 6 = 1994 m of drawn
        // distance, and it is still checked at **Start** rather than at upload, because the
        // barometric datum and home both drift between the two.
        val item4FromHome = RepositionGuidance.horizontalMetres(
            PLANNED_HOME_LAT, PLANNED_HOME_LON, 37.99273535, 23.72610237,
        )
        assertEquals(146.0, item4FromHome, 2.0)
        assertTrue(
            "item 4 is inside the enforced home bound",
            item4FromHome < MissionGuidance.MAX_HOME_DIST_M - MissionGuidance.CORNER_ALLOWANCE_M,
        )
        assertTrue(MissionLaunch.evaluate(inputs(planOf(big1()))) is LaunchVerdict.Cleared)
    }

    @Test
    fun `THE BOUND IS STILL THERE - big1 with one corner pushed past 2 km is refused, by name`() {
        // The other half of an expansion: the envelope moved, it did not disappear. Item 4 pushed
        // ~2.3 km south of the box makes leg 4 (item 3 → item 4) too long, and the operator gets the
        // same legible sentence they got at 100 m — with the new number in it.
        val far = big1().map {
            if (it.seq == 4) it.copy(x = e7(37.99273535 - 2_300.0 / MissionGeo.METRES_PER_DEG)) else it
        }
        val refusal = MissionAdmission.admissible(far)
        assertNotNull("a leg past the new bound must still be refused", refusal)
        assertEquals(4, refusal!!.seq)
        assertTrue(
            "the sentence must name the limit, got '${refusal.reason}'",
            refusal.reason.contains("${MissionAdmission.MAX_LEG_M.toInt()}m max"),
        )
        // …and Start's home bound refuses the same plan independently, as it always did — two
        // doors, one for the drawn leg and one for the distance from home.
        assertEquals(
            MissionLaunch.reasonTooFar(4),
            (MissionLaunch.evaluate(inputs(planOf(far))) as LaunchVerdict.Refused).reason,
        )
    }

    @Test
    fun `big1's landing items are admissible - nothing about the plan's new field is refused`() {
        // item 6's frame 3, item 8's frame 2 (`MAV_FRAME_MISSION`), and the Land item's `param2 = 2`
        // and 15 m all pass the desk untouched. Kept on the pulled-in variant as well as the
        // authored plan, because the rest of this suite flies that variant and its admissibility is
        // what makes those tests meaningful.
        assertNull(MissionAdmission.admissible(big1()))
    }

    // ------------------------------------------------------------------ the projection

    @Test
    fun `big1 resolves to eight legs, and the Land item keeps its mode and its height`() {
        val plan = planOf(big1())

        // Ten items, eight legs: the two ROI items occupy sequence numbers and are not places.
        assertEquals(10, plan.items.size)
        assertEquals(listOf(0, 1, 2, 3, 4, 5, 7, 9), plan.legs.map { it.seq })
        assertEquals(
            listOf(
                LegKind.TAKEOFF, LegKind.WAYPOINT, LegKind.WAYPOINT, LegKind.WAYPOINT,
                LegKind.WAYPOINT, LegKind.WAYPOINT, LegKind.WAYPOINT, LegKind.LAND,
            ),
            plan.legs.map { it.kind },
        )

        // The Land item, which is what this whole feature turns on.
        val land = plan.legAt(9)!!
        assertEquals(PrecisionLandMode.REQUIRED, land.precisionLandMode)
        assertEquals("the item's altitude is a height we fly to now", 15.0, land.relativeAltM!!, 0.0)
        assertEquals(LAND_LAT, land.target!!.latDeg, 1e-7)

        // The plan's takeoff height — the number the sequence's low gate is measured against.
        assertEquals(10.0, plan.legAt(0)!!.relativeAltM!!, 0.0)
    }

    @Test
    fun `the drawn Land item is 0,3 m from the planned home - a cross-check that passes with room`() {
        // The site cross-check compares the *recorded* takeoff point with this coordinate. In the
        // field the recorded point differs from the planned one by DJI's own GPS wander (~1 m,
        // landing09) plus wherever the aircraft actually stood — Ivan: "can be 5 m away for example".
        // The drawn-vs-planned distance is the part that is knowable at the desk, and it is 0.3 m.
        val drawn = RepositionGuidance.horizontalMetres(
            PLANNED_HOME_LAT, PLANNED_HOME_LON, LAND_LAT, LAND_LON,
        )
        assertEquals(0.3, drawn, 0.1)
        assertTrue(drawn < PrecisionLand.LAND_TAG_RADIUS_M)
        // The last waypoint before the landing is ~8 m from the pad, so the transit both translates
        // and descends: 60 m -> 15 m over ~8 m of ground.
        val fromLastWaypoint = RepositionGuidance.horizontalMetres(
            37.993916961081936, 23.72524138553902, PLANNED_HOME_LAT, PLANNED_HOME_LON,
        )
        assertEquals(8.0, fromLastWaypoint, 2.0)
        assertTrue(fromLastWaypoint < PrecisionLand.LAND_TAG_RADIUS_M)
    }

    // ------------------------------------------------------------------ the route Start builds

    @Test
    fun `THE OUTPUT - Start on big1 builds a route whose last step is the tag landing at 15 m`() {
        val cleared = MissionLaunch.evaluate(inputs(planOf(big1()))) as LaunchVerdict.Cleared
        val route = cleared.route

        // Eight steps, in the plan's own order, and the shape of each one.
        assertEquals(8, route.size)
        assertEquals(
            listOf(
                MissionStepKind.TAKEOFF, MissionStepKind.WAYPOINT, MissionStepKind.WAYPOINT,
                MissionStepKind.WAYPOINT, MissionStepKind.WAYPOINT, MissionStepKind.WAYPOINT,
                MissionStepKind.WAYPOINT, MissionStepKind.PRECISION_LAND,
            ),
            route.steps.map { it.kind },
        )
        assertEquals(
            listOf(10.0, 30.0, 60.0, 60.0, 60.0, 60.0, 60.0, 15.0),
            route.steps.map { it.relAltM },
        )
        // Only the first and last rest: the six in between are flown through (Ivan's fourth answer).
        assertEquals(
            listOf(true, false, false, false, false, false, false, true),
            route.steps.map { it.rest },
        )

        val landing = route[route.size - 1]
        assertEquals(9, landing.seq)
        assertEquals(PrecisionLandMode.REQUIRED, landing.precisionLandMode)
        assertEquals(15.0, landing.relAltM!!, 0.0)
        assertEquals(10.0, route.takeoffRelAltM!!, 0.0)
        assertEquals(0, cleared.startIndex)
        assertTrue(!cleared.rejoining)
    }

    @Test
    fun `THE OUTPUT - what the sequence will then do, from the plan's own numbers`() {
        // The gate as it will be evaluated in the air at item 9's begin: aircraft over the pad at
        // 60 m, DJI's recorded home 1 m from the planned one (landing09's measured session wander).
        val recordedTakeoff = PLANNED_HOME_LAT + 1.0 / RepositionGuidance.METRES_PER_DEG to PLANNED_HOME_LON
        val decision = PrecisionLand.gate(
            aircraft = 37.993916961081936 to 23.72524138553902,  // item 7, where the cursor arrives
            takeoff = recordedTakeoff,
            itemLatDeg = LAND_LAT,
            itemLonDeg = LAND_LON,
            itemRelAltM = 15.0,
            currentRelAltM = 60.0,
            planTakeoffRelAltM = 10.0,
        ) as PrecisionLand.Decision.Clear

        // The transit's target: the RECORDED takeoff point (not the drawn item, 1.1 m away here) at
        // the item's own 15 m — so the leg descends 45 m while translating ~8 m.
        assertEquals(recordedTakeoff.first, decision.takeoffLatDeg, 1e-12)
        assertEquals(15.0, decision.transitRelAltM, 0.0)
        // Then the lowering: min(8, 15) = 8 m above the takeoff datum, which is above the 7 m tag band
        // and under the 12 m decode ceiling — so the descent arms into its APPROACH segment.
        assertEquals(8.0, PrecisionLand.armHeightTargetM(decision.transitRelAltM), 0.0)
    }

    @Test
    fun `flipping big1's param2 to Disabled turns the same plan back into a hold`() {
        // The forward-compatibility property, on the real plan: one field, and this becomes the
        // mission every plan authored before 2026-07-30 flies — arrive at the item and hover.
        val disabled = big1().map {
            if (it.command == MissionCommands.NAV_LAND) it.copy(param2 = 0f) else it
        }
        val cleared = MissionLaunch.evaluate(inputs(planOf(disabled))) as LaunchVerdict.Cleared
        val landing = cleared.route[cleared.route.size - 1]
        assertEquals(MissionStepKind.HOLD, landing.kind)
        assertNull("a Disabled land item commands no height", landing.relAltM)
        assertEquals(0, landing.precisionLandMode)
    }

    // ------------------------------------------------------------- the ROI, no longer a gap

    @Test
    fun `THE OUTPUT - big1's ROI items reach the route, on exactly the leg Ivan expects`() {
        val plan = planOf(big1())

        // The store does its half: the ROI is sticky from item 6 to item 7, cleared by item 8, and its
        // height is carried because frame 3's z is metres above *our* takeoff datum.
        val underRoi = plan.legAt(7)!!
        assertNotNull("item 7 flies under the ROI set at item 6", underRoi.roi)
        assertEquals(37.99384298942512, underRoi.roi!!.target.latDeg, 1e-7)
        assertEquals("frame 3's z is our own datum, and is kept", 0.0, underRoi.roi!!.relativeAltM!!, 0.0)
        assertNull("item 8's DO_SET_ROI_NONE clears it", plan.legAt(9)!!.roi)

        // **And since 2026-07-30 the route carries it** (Ivan: *"fix mission execution to respect plan
        // ROI"*). This test used to assert the opposite — that `MissionStep` had no ROI field and a
        // plan's ROI items pointed nothing — and the assertion that replaces it is the same fact from
        // the other side: exactly one step of this route names an ROI, and it is the step for item 7.
        val cleared = MissionLaunch.evaluate(inputs(plan)) as LaunchVerdict.Cleared
        val route = cleared.route
        assertEquals(8, route.size)
        assertEquals(
            "only item 7's leg flies under the ROI — items 0-5 precede it, item 9 follows the clear",
            listOf(7),
            route.steps.filter { it.roi != null }.map { it.seq },
        )

        // Ivan's sentence, in his own numbering: *"after wp6 it looks at roi, at wp8 it should stop"*.
        // QGC displays the wire `seq` plus one (it deletes the planned-home marker and renumbers down
        // for PX4), so his wp6 is item 5 and his wp8 is item 7 — the ROI is in force for the leg that
        // begins when item 5 is reached and is gone once item 7 is.
        val roiStep = route.steps.single { it.roi != null }
        assertEquals("Ivan's wp8, on the wire", 7, roiStep.seq)
        assertEquals(5, route.steps[route.steps.indexOf(roiStep) - 1].seq)

        // The item's own numbers reach the pointing solution unrounded, and its `z` with them: frame 3
        // means metres above *our* takeoff datum, which is directly usable and is not discarded.
        val roi = roiStep.roi!!
        val (lat, lon) = roi.targetDegrees()!!
        assertEquals(37.99384298942512, lat, 1e-7)
        assertEquals(23.72577905336516, lon, 1e-7)
        assertEquals(0.0, roi.relativeAltMOrNull()!!, 0.0)
    }
}
