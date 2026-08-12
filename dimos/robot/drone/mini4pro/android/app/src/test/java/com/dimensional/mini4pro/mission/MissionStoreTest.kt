package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.mission.MissionFixtures.item
import com.dimensional.mini4pro.mission.MissionFixtures.latE7
import com.dimensional.mini4pro.mission.MissionFixtures.northOf
import com.dimensional.mini4pro.mission.MissionFixtures.waypoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * M4 transport — the store and its projection: verbatim storage, the atomic swap, `planId`, and
 * the sticky ROI/speed resolution of `docs/m4-mission-transport.md` §3.3.
 *
 * Written to fail loudly for the store's landmines:
 *
 *  - a `StoredItem` "helpfully" normalised on the way in, so the read-back paraphrases the plan
 *    an operator authored
 *  - sticky state resolved from the *end* of the plan rather than as it stood at each leg — the
 *    error that is correct in review and wrong at item 14
 *  - `DO_SET_ROI_NONE` leaving the previous ROI standing, so a plan that says "stop looking at
 *    that" keeps looking at it
 *  - a `planId` that does not move on a clear, which would let a paused cursor survive into a
 *    plan that no longer exists
 *  - `x`/`y` read from a `MAV_FRAME_MISSION` item, where they are a NaN-to-int32 cast
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted across the four
 * mission suites (this one, `MissionAdmissionTest`, `MissionTransactionTest`,
 * `MissionProgressTest`), code reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | sticky speed resolved from the last `DO_CHANGE_SPEED` in the plan rather than the one in force | 1 |
 *  | `DO_SET_ROI_NONE` leaves the previous ROI standing | 1 |
 *  | `planId` not bumped on clear | 1 |
 *  | `commit` keeps the caller's list instead of copying it | 1 |
 *  | contiguity `require` removed from `commit` | 1 |
 *  | `MissionGeo.pointOrNull` ignores the frame (reads x/y under `MAV_FRAME_MISSION`) | 1 |
 *  | `MissionGeo.pointOrNull` skips `Geo.coordinateOrNull` (DJI's filler accepted) | 13 |
 *  | `cos(latitude)` dropped from `distanceM` | 1 |
 *  | `NAV_LAND`'s terminal hold commands the item's altitude (a descent promised) | 1 |
 *
 * The 13 is `Geo.coordinateOrNull`, and it is the whole suite refusing to run past a coordinate
 * nobody checked — the same range-and-filler rule that caught DJI's 4.58e7 placeholder before it
 * reached the wire as `latitude = 2147483647`.
 *
 * **The frame mutant scored 0 on the first pass**, and the reason is worth keeping. The test then
 * used `x = Int.MAX_VALUE`, which as 1e7 degrees is a latitude of 214.7 — so the *range* check
 * killed it and the frame check was never load-bearing. Defence in depth masking the primary
 * guard, exactly as `GuidedRepositionTest`'s target-survives-abort mutant did. The observable
 * difference is QGC's own encoding for `MAV_FRAME_MISSION`: it writes `param5` **unscaled**
 * (`PlanManager.cc:547-548`), so a target at 38 °N arrives as `x = 38`, which read as 1e7 degrees
 * is 3.8e-6 — perfectly in range, distinct from its longitude, and a few centimetres from null
 * island. Only the frame check catches that one, and the test now uses it.
 */
class MissionStoreTest {

    private val store = MissionStore()

    // ------------------------------------------------------------------ verbatim, and atomic

    @Test
    fun `an item is stored exactly as it came off the wire`() {
        // Deliberately hostile values: a hold time, an acceptance radius, a NaN yaw and a
        // three-decimal altitude. Every one of them must come back bit-identical, because
        // MISSION_REQUEST_INT is answered from this list and a paraphrase of the operator's plan
        // is what they would then see after a reconnect.
        val original = item(
            seq = 0,
            command = MissionCommands.NAV_WAYPOINT,
            param1 = 3.5f, param2 = 4f, param3 = 0f, param4 = Float.NaN,
            x = northOf(12.0), y = latE7(MissionFixtures.LON), z = 12.345f,
            current = 1, autocontinue = 0,
        )
        store.commit(listOf(original), null, null, 0L)

        val readBack = store.items.single()
        assertEquals(original, readBack)
        assertEquals(12.345f, readBack.z, 0f)
        assertEquals(1, readBack.current)
        assertEquals(0, readBack.autocontinue)
        assertTrue("a NaN yaw must survive the store", readBack.param4.isNaN())
    }

    @Test
    fun `a commit replaces the snapshot rather than mutating the old one`() {
        // The caller's list is the transaction's pending buffer. The snapshot must be a copy of
        // it, not a window onto it: the executor may be holding `first` on its own 10 Hz thread,
        // and a plan that grows underneath it is a plan with a discontinuity in it — which the
        // aircraft would fly.
        val callersList = mutableListOf(waypoint(0, 10.0))
        store.commit(callersList, null, null, 0L)
        val first = store.plan()!!
        val firstItems = first.items

        callersList.add(waypoint(1, 20.0))
        assertEquals("the snapshot must not follow the caller's list", 1, first.itemCount)
        assertSame(firstItems, first.items)

        store.commit(listOf(waypoint(0, 10.0), waypoint(1, 20.0)), null, null, 1L)
        val second = store.plan()!!
        assertEquals("the old snapshot still describes the plan it was handed", 1, first.itemCount)
        assertEquals(2, second.itemCount)
        assertTrue("a new upload must yield a new object", first !== second)
    }

    @Test
    fun `planId moves on every commit and on every clear`() {
        assertEquals("nothing committed must never name a plan", 0, store.planId)

        store.commit(listOf(waypoint(0, 10.0)), null, null, 0L)
        val afterFirst = store.planId
        assertTrue(afterFirst > 0)

        store.commit(listOf(waypoint(0, 10.0)), null, null, 1L)
        val afterSecond = store.planId
        assertTrue("a re-upload of an identical plan is still a new plan", afterSecond > afterFirst)

        // The clear is the one that matters: a paused cursor is only meaningful against the plan
        // it was paused in, and a clear that left planId alone would let one survive into
        // nothing.
        store.clear()
        assertTrue(store.planId > afterSecond)
        assertNull(store.plan())
        assertEquals(0, store.count)
    }

    @Test
    fun `provenance is recorded and is never used as a datum`() {
        val home = GeoPoint(MissionFixtures.LAT, MissionFixtures.LON)
        val plan = store.commit(listOf(waypoint(0, 10.0, z = 15f)), home, 103.2, 42L)

        assertEquals(home, plan.homeAtUpload)
        assertEquals(103.2, plan.amslDatumAtUpload!!, 0.0)
        assertEquals(42L, plan.uploadedAtMs)
        // The altitude is untouched by the datum. Transport records the fact and refuses to
        // rewrite a plan an operator authored; whether a moved home blocks a start is the
        // execution half's decision (§3.2, Q4).
        assertEquals(15.0, plan.legs.single().relativeAltM!!, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `a plan with a gap cannot be committed`() {
        // The executor's cursor is an index. A gap would make "advance" undefined, so this is a
        // structural invariant rather than a validation rule.
        store.commit(listOf(waypoint(0, 10.0), waypoint(2, 20.0)), null, null, 0L)
    }

    // ---------------------------------------------------------------------- sticky resolution

    @Test
    fun `speed is the DO_CHANGE_SPEED in force at each leg, not the last one in the plan`() {
        val items = listOf(
            waypoint(0, 10.0),
            item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                param1 = 1f, param2 = 2f),
            waypoint(2, 20.0),
            item(3, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                param1 = 1f, param2 = 3f),
            waypoint(4, 30.0),
        )
        val legs = store.commit(items, null, null, 0L).legs

        assertEquals(3, legs.size)
        // Before any DO_CHANGE_SPEED there is no limit and the executor uses its own.
        assertNull(legs[0].speedLimitMps)
        assertEquals(2.0, legs[1].speedLimitMps!!, 0.0)
        assertEquals(3.0, legs[2].speedLimitMps!!, 0.0)
    }

    @Test
    fun `ROI is sticky until another ROI, and DO_SET_ROI_NONE clears it`() {
        val roiX = northOf(50.0)
        val items = listOf(
            waypoint(0, 10.0),
            item(1, MissionCommands.DO_SET_ROI_LOCATION, x = roiX, y = latE7(MissionFixtures.LON), z = 3f),
            waypoint(2, 20.0),
            waypoint(3, 30.0),
            item(4, MissionCommands.DO_SET_ROI_NONE, frame = MissionFrames.MISSION),
            waypoint(5, 40.0),
        )
        val legs = store.commit(items, null, null, 0L).legs

        assertEquals(listOf(0, 2, 3, 5), legs.map { it.seq })
        assertNull("nothing is being looked at before the first ROI", legs[0].roi)
        // Sticky across every following leg…
        assertNotNull(legs[1].roi)
        assertNotNull(legs[2].roi)
        assertEquals(roiX / MissionGeo.DEGREES_E7, legs[1].roi!!.target.latDeg, 1e-9)
        // …and a plan item in GLOBAL_RELATIVE_ALT carries a height in *our* frame, so it is kept.
        assertEquals(3.0, legs[1].roi!!.relativeAltM!!, 0.0)
        // "Point at nothing" is a state, not the absence of a command.
        assertNull(legs[3].roi)
    }

    @Test
    fun `the legacy DO_SET_ROI folds into the same target`() {
        // 201 is not in QGC's palette and its shape is source-only; accepted for imported plans.
        // As a MISSION_ITEM_INT its param5/6/7 *are* x/y/z, so one code path serves both.
        val items = listOf(
            item(0, MissionCommands.DO_SET_ROI, x = northOf(25.0), y = latE7(MissionFixtures.LON), z = 2f),
            waypoint(1, 10.0),
        )
        val legs = store.commit(items, null, null, 0L).legs
        assertEquals(1, legs.size)
        assertNotNull(legs[0].roi)
        assertEquals(2.0, legs[0].roi!!.relativeAltM!!, 0.0)
    }

    @Test
    fun `legs are the navigable subset and each carries its own wire seq`() {
        val items = listOf(
            item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
            item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION, param1 = 1f, param2 = 2f),
            waypoint(2, 20.0),
            item(3, MissionCommands.NAV_LAND, x = 0, y = 0, z = 0f),
        )
        val plan = store.commit(items, null, null, 0L)

        // Three legs for four items: the index into `legs` is deliberately NOT the seq, which is
        // why every leg carries one.
        assertEquals(listOf(0, 2, 3), plan.legs.map { it.seq })
        assertEquals(
            listOf(LegKind.TAKEOFF, LegKind.WAYPOINT, LegKind.LAND),
            plan.legs.map { it.kind },
        )
        assertEquals(LegKind.WAYPOINT, plan.legAt(2)!!.kind)
        assertNull("a sticky item is not a leg", plan.legAt(1))
    }

    // ---------------------------------------------------------------------- the land item's param2

    @Test
    fun `PARAM2 == 0 - a land item that does not ask for precision keeps M4-5's hold, height discarded`() {
        // QGC's default, and every plan authored before 2026-07-30 carries it. The height must stay
        // null: returning it would be the projection quietly promising a descent, which is the
        // capability M4-5 removed and which this field is now the *only* way to ask for.
        val leg = store.commit(
            listOf(item(0, MissionCommands.NAV_LAND, x = 0, y = 0, param2 = 0f, z = 15f)),
            null, null, 0L,
        ).legs.single()
        assertEquals(LegKind.LAND, leg.kind)
        assertEquals(0, leg.precisionLandMode)
        assertNull("a Disabled land item commands no height", leg.relativeAltM)
    }

    @Test
    fun `PARAM2 of 1 or 2 - a precision land item carries its mode AND its altitude`() {
        // big1.plan's own item 7: frame 3, z = 15, param2 = 2 (Required). For the tag-landing sequence
        // the altitude is an ordinary waypoint altitude that the aircraft flies to, so it must survive
        // the projection.
        for (mode in listOf(PrecisionLandMode.OPPORTUNISTIC, PrecisionLandMode.REQUIRED)) {
            val leg = store.commit(
                listOf(item(0, MissionCommands.NAV_LAND, x = 0, y = 0, param2 = mode.toFloat(), z = 15f)),
                null, null, 0L,
            ).legs.single()
            assertEquals("mode $mode", mode, leg.precisionLandMode)
            assertEquals("mode $mode", 15.0, leg.relativeAltM!!, 0.0)
        }
    }

    @Test
    fun `an unrecognised param2 reads as Disabled - the fail-closed answer to shall I land`() {
        // 3, −1 and NaN are not modes this bridge knows. A plan asking for something we cannot vouch
        // for is flown as the safe behaviour (a hold) rather than refused at upload, which would make
        // it unflyable even as one; and it is emphatically not flown as a landing.
        for (param2 in listOf(3f, -1f, Float.NaN, 99f)) {
            val leg = store.commit(
                listOf(item(0, MissionCommands.NAV_LAND, x = 0, y = 0, param2 = param2, z = 15f)),
                null, null, 0L,
            ).legs.single()
            assertEquals("param2 $param2", PrecisionLandMode.DISABLED, leg.precisionLandMode)
            assertNull("param2 $param2", leg.relativeAltM)
        }
        // A float that has been round-tripped through QGC's two-decimal editor is still its integer.
        val rounded = store.commit(
            listOf(item(0, MissionCommands.NAV_LAND, x = 0, y = 0, param2 = 1.9999998f, z = 15f)),
            null, null, 0L,
        ).legs.single()
        assertEquals(PrecisionLandMode.REQUIRED, rounded.precisionLandMode)
    }

    @Test
    fun `only a land item has an opinion about precision landing`() {
        val legs = store.commit(
            listOf(
                item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
                // A waypoint's param2 is its acceptance radius — 4 m here — and reading it as a
                // precision-land mode would be the same field meaning two things.
                item(1, MissionCommands.NAV_WAYPOINT, param2 = 4f, x = northOf(20.0), z = 10f),
                item(2, MissionCommands.NAV_RETURN_TO_LAUNCH, x = 0, y = 0, z = 0f),
            ),
            null, null, 0L,
        ).legs
        assertTrue(legs.all { it.precisionLandMode == null })
        assertEquals(4.0, legs[1].acceptRadiusM!!, 0.0)
    }

    @Test
    fun `a loiter item resolves to a hover, never to a circle`() {
        // JC-8. On a multirotor `param3`'s radius is a fixed-wing concept and QGC's own Plan view
        // draws these as a point. Flying a circle where the plan says hover is a substitution in
        // the direction of *more* motion, which is what EMERGENCY_STOP_TEXT's reasoning forbids.
        val items = listOf(
            item(0, MissionCommands.NAV_LOITER_TIME, param1 = 15f, z = 10f),
            item(1, MissionCommands.NAV_LOITER_UNLIM, z = 10f),
        )
        val legs = store.commit(items, null, null, 0L).legs

        assertEquals(LegKind.HOVER, legs[0].kind)
        assertEquals(15.0, legs[0].holdSeconds, 0.0)
        assertNull("a hover has no orbit spec", legs[0].orbit)
        assertEquals(LegKind.HOVER, legs[1].kind)
        assertTrue("hover-until-told-otherwise is terminal", legs[1].holdSeconds.isInfinite())
    }

    @Test
    fun `an orbit keeps its signed radius and its yaw behaviour as an integer`() {
        val items = listOf(
            item(
                0, MissionCommands.DO_ORBIT,
                // Negative radius: QGC encodes counter-clockwise as the sign of param1.
                param1 = -30f,
                // NaN velocity and NaN turns are QGC's "vehicle default" for both.
                param2 = Float.NaN, param3 = 5f, param4 = Float.NaN,
                z = 20f,
            )
        )
        val orbit = store.commit(items, null, null, 0L).legs.single().orbit!!

        assertEquals(-30.0, orbit.radiusM, 0.0)
        assertNull(orbit.velocityMps)
        assertNull(orbit.turns)
        // 5 is ORBIT_YAW_BEHAVIOUR_UNCHANGED, which exists in QGC's dialect and **not** in
        // io.dronefleet.mavlink 1.1.11's. Carried as an integer for exactly that reason.
        assertEquals(OrbitYawBehaviour.UNCHANGED, orbit.yawBehaviour)
    }

    @Test
    fun `x and y are never read from a MAV_FRAME_MISSION item`() {
        // §4.2's trap: QGC does not scale x/y by 1e7 for this frame, and survey-generated items
        // carry param5/6/7 = NaN, so x/y are whatever a double NaN → int32 cast produced. Here
        // that is Int.MAX_VALUE, which as 1e7 degrees is a latitude of 214.7 — not a place.
        // DO_SET_ROI_LOCATION is coordinate-bearing, so only the *frame* stands between the
        // projection and a garbage place. (Admission refuses this shape at the door too; this
        // pins the second layer, because the resolver is what an executor would consume.)
        //
        // The values are QGC's own for this frame: it writes `item->param5()` **unscaled**
        // (`PlanManager.cc:547-548`), so a target at 38 °N, 23.7 °E arrives as x = 38, y = 23.
        // Read as 1e7 degrees those are 3.8e-6 and 2.3e-6 — both perfectly in range, both
        // distinct, and both a few centimetres from null island off the coast of Ghana. The range
        // check cannot catch this one; only the frame check can.
        val unscaled = item(
            0, MissionCommands.DO_SET_ROI_LOCATION, frame = MissionFrames.MISSION,
            x = 38, y = 23, z = 2f,
        )
        assertNull(MissionGeo.pointOrNull(unscaled))
        // …and it must not sneak in through the sticky resolution either.
        assertNull(MissionStore.resolve(listOf(unscaled, waypoint(1, 10.0))).single().roi)

        // The other shape of the same trap: survey items carry param5/6/7 = NaN, so x/y are
        // whatever a double NaN → int32 cast produced.
        assertNull(
            MissionGeo.pointOrNull(
                item(0, MissionCommands.DO_SET_ROI_LOCATION, frame = MissionFrames.MISSION,
                    x = Int.MAX_VALUE, y = Int.MIN_VALUE)
            )
        )
    }

    @Test
    fun `DJI's one-number-in-both-fields filler is not a coordinate`() {
        // The 4.583662361046586E7 placeholder that reached the wire as latitude = 2147483647
        // before `Geo` existed. The mission layer must use the same rule, not a second one.
        val filler = latE7(4.583662361046586E7 / 1e7)
        val item = item(0, MissionCommands.NAV_WAYPOINT, x = filler, y = filler)
        assertNull(MissionGeo.pointOrNull(item))
    }

    @Test
    fun `distance carries the cos latitude term`() {
        val origin = GeoPoint(MissionFixtures.LAT, MissionFixtures.LON)
        val east = GeoPoint(
            MissionFixtures.LAT,
            MissionFixtures.LON + 100.0 /
                (MissionGeo.METRES_PER_DEG * kotlin.math.cos(Math.toRadians(MissionFixtures.LAT))),
        )
        // Without cos(38°) = 0.788 this reads 127 m rather than 100 — a 27% error, invisible at
        // the equator and decisive at the leg bound.
        assertEquals(100.0, MissionGeo.distanceM(origin, east), 0.5)

        val north = GeoPoint(MissionFixtures.LAT + 100.0 / MissionGeo.METRES_PER_DEG, MissionFixtures.LON)
        assertEquals(100.0, MissionGeo.distanceM(origin, north), 0.5)
    }
}
