package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.command.StatusTexts
import com.dimensional.mini4pro.mission.MissionFixtures.LON
import com.dimensional.mini4pro.mission.MissionFixtures.takeoffAndHoldPlan
import com.dimensional.mini4pro.mission.MissionFixtures.item
import com.dimensional.mini4pro.mission.MissionFixtures.latE7
import com.dimensional.mini4pro.mission.MissionFixtures.northOf
import com.dimensional.mini4pro.mission.MissionFixtures.waypoint
import io.dronefleet.mavlink.common.MavMissionResult
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * M4 transport — **static admission**: `docs/m4-mission-transport.md` §4.2/§4.3 and the execution
 * design's §7.1. Every branch of the pure function that decides whether a list of items is a plan
 * this bridge is willing to hold.
 *
 * Written to fail loudly for the admission landmines:
 *
 *  - **an absolute altitude accepted** — the finding that separates M4 from M3. Stage B's
 *    `DO_REPOSITION` AMSL is composed from *our own* published datum and cancels exactly; a plan
 *    item's is a number an operator typed or a `.plan` file remembered, in a datum ours moved
 *    41.5 m away from between two sessions
 *  - an allow-list that is really a deny-list, so a command QGC gains next release flies
 *  - a bound that clamps instead of refusing — the command an operator believes was obeyed
 *  - a refusal with the wrong `MAV_MISSION_RESULT`, so QGC's own dialog says the wrong thing
 *  - a refusal sentence over `STATUSTEXT`'s 50-byte field, which is cut silently on the wire
 *  - `x`/`y` read from a `MAV_FRAME_MISSION` item
 *
 * Mutation-checked 2026-07-27, one breakage at a time, failing tests counted across the four
 * mission suites, code reverted after each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | absolute frames accepted (`MissionFrames.RELATIVE` includes GLOBAL/GLOBAL_INT) | 2 |
 *  | terrain frames accepted | 1 |
 *  | `MAV_FRAME_MISSION` accepted for coordinate-bearing items too | 1 — killed on the *result code*, see below |
 *  | **both** `MAV_FRAME_MISSION` guards removed (here and in `MissionGeo.pointOrNull`) | 2 |
 *  | allow-list inverted to a deny-list (unknown commands accepted) | 3 |
 *  | ceiling check removed (any altitude accepted) | 2 |
 *  | `z <= 0` accepted for a waypoint | 1 |
 *  | the coordinate check skipped entirely | 1 |
 *  | speed envelope check removed | 2 |
 *  | airspeed request accepted as a ground speed | 1 |
 *  | acceptance-radius band removed | 1 |
 *  | `param3` pass-radius check removed | 1 |
 *  | `param4` yaw check removed | 1 |
 *  | `NAV_TAKEOFF` `param4` yaw check removed | 1 |
 *  | the loiter `param4` yaw check removed | 1 |
 *  | the loiter `param3` radius check removed (JC-8's circle, accepted) | 1 |
 *  | takeoff-at-seq-0 rule removed | 1 |
 *  | land-must-be-last rule removed | 1 |
 *  | `NAV_LOITER_UNLIM`-must-be-last rule removed | 1 |
 *  | uniqueness check removed (two takeoffs accepted) | 1 — see below |
 *  | item-count bound removed | 2 |
 *  | per-leg distance bound removed | 2 |
 *  | total-path bound removed | 1 |
 *  | zero-length leg accepted | 1 |
 *  | at-least-one-navigable-item check removed | 1 |
 *  | gimbal roll check removed | 1 |
 *  | gimbal-manager yaw checks removed | 1 |
 *  | orbit radius band removed | 1 |
 *  | orbit radius compared signed rather than by magnitude | 1 |
 *  | orbit yaw behaviours 0 and 3 silently downgraded rather than refused | 1 |
 *  | `admissible` runs only the whole-plan half (per-item checks skipped) | 24 — **and read the note** |
 *  | **the shipping path** skips per-item admission (`MissionTransaction`'s `itemRefusal` call) | 1 |
 *
 * **The 24 does not mean what it looks like, and the correction is the point.** `admissible` has
 * **no production caller**: an upload is refused item by item, as each item arrives, by
 * `MissionTransaction` calling `MissionAdmission.itemRefusal` directly (JC-2 — it is what makes
 * QGC name the index, the command and the offending value). `admissible` is the composed form the
 * execution design asked for, it will be what the executor calls at Start, and today it is what
 * *this suite* calls — `refusalFor` is one line and it is `admissible`. So neutering it fails
 * every test in this file and **not one** in `MissionTransactionTest`, which is exactly what the
 * measurement shows.
 *
 * The number that describes the shipping path is the row beneath it: **1**. That is one test —
 * `a refused item ends the transaction and leaves the previous plan alone` — standing between an
 * inadmissible plan and the store, on the path an upload actually takes. Re-measured 2026-07-27
 * after the review pointed out that the 22 (now 24, with two tests added since) was being read as
 * 22 tests defending code that ships.
 *
 * **The `MAV_FRAME_MISSION` row is honest about how it dies.** Removing this file's frame guard
 * alone still leaves the item refused — by `coordinateRefusal` downstream, which reads
 * `MissionGeo.pointOrNull` and finds its *own* frame guard there. So the test dies on the result
 * code (`expected UNSUPPORTED_FRAME but was INVALID_PARAM5_X`) rather than on the item being
 * flown. Remove **both** guards and the answer changes to *"expected a refusal"* — the plan is
 * admitted, with a coordinate a few centimetres off the coast of Ghana.
 *
 * That second measurement is the one the fixture values buy, and it is why they were corrected on
 * 2026-07-27 from `Int.MAX_VALUE`/`Int.MIN_VALUE` to QGC's own unscaled `x = 38, y = 23`. With the
 * old values, both guards removed still produced `INVALID_PARAM5_X`, because the *range* check
 * caught the garbage: the test passed, and the property it is named for was undefended. Measured
 * both ways, one after the other.
 *
 * **The uniqueness mutant scored 0 on the first pass**, and the reason is a finding rather than a
 * test gap: the check is *unreachable* through `admissible`. Two takeoffs cannot both be at seq 0
 * and two landings cannot both be last, so the positional rules refuse every list uniqueness
 * would. It is kept because the positional rules are the kind of thing that gets relaxed, and
 * `the whole-plan half refuses a duplicate phase on its own` pins it where it can be reached —
 * `planRefusal` directly, which is a public entry point in its own right.
 */
class MissionAdmissionTest {

    private fun refusalFor(items: List<StoredItem>): MissionRefusal? =
        MissionAdmission.admissible(items)

    private fun assertAccepted(items: List<StoredItem>) {
        val refusal = refusalFor(items)
        assertNull("expected acceptance, got ${refusal?.result} — ${refusal?.reason}", refusal)
    }

    private fun assertRefused(
        items: List<StoredItem>,
        result: MavMissionResult,
        seq: Int?,
    ): MissionRefusal {
        val refusal = refusalFor(items)
        assertNotNull("expected a refusal", refusal)
        assertEquals(result, refusal!!.result)
        assertEquals(seq, refusal.seq)
        // Every sentence goes through StatusTexts.clamp; a sentence over the field width is cut
        // silently on the wire, and the operator then searches for a string that does not exist.
        assertTrue(
            "refusal '${refusal.reason}' is ${refusal.reason.toByteArray().size} bytes",
            refusal.reason.toByteArray(Charsets.UTF_8).size <= StatusTexts.MAX_BYTES,
        )
        return refusal
    }

    // ------------------------------------------------------------------------- the happy path

    @Test
    fun `a takeoff-waypoint-hold plan is admissible`() {
        // Takeoff is ours; landing is not (M4-5). The terminal NAV_LAND is a *hold* — fly there,
        // come to rest, and wait for a human on the RC. The smallest plan that exercises the
        // structural rules.
        assertAccepted(takeoffAndHoldPlan())
    }

    @Test
    fun `a terminal item commands no altitude, because it does not descend`() {
        // The projection is where M4-5 has to be visible: the raw z stays in the store verbatim
        // for the read-back, and the *commanded* altitude is null — the aircraft holds whatever
        // it arrives with. A relativeAltM here would be the projection promising a descent.
        val plan = MissionStore.resolve(takeoffAndHoldPlan())
        assertEquals(LegKind.LAND, plan.last().kind)
        assertNull("a land item must not command a height", plan.last().relativeAltM)

        val rtl = MissionStore.resolve(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_RETURN_TO_LAUNCH, frame = MissionFrames.MISSION))
        )
        assertEquals(LegKind.RTL, rtl.last().kind)
        assertNull("return-to-launch is a hover at home, not DJI's RTH", rtl.last().relativeAltM)
    }

    @Test
    fun `the full accepted vocabulary is admissible together`() {
        assertAccepted(
            listOf(
                item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
                item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 1f, param2 = 2.5f),
                item(2, MissionCommands.DO_SET_ROI_LOCATION, x = northOf(30.0), y = latE7(LON), z = 2f),
                waypoint(3, 20.0),
                item(4, MissionCommands.NAV_LOITER_TIME, param1 = 10f, x = northOf(25.0), y = latE7(LON)),
                item(5, MissionCommands.NAV_DELAY, frame = MissionFrames.MISSION, param1 = 5f),
                item(6, MissionCommands.DO_MOUNT_CONTROL, frame = MissionFrames.MISSION,
                    param1 = -45f, param2 = 0f, param3 = 0f, z = 2f),
                item(7, MissionCommands.DO_SET_ROI_NONE, frame = MissionFrames.MISSION),
                item(8, MissionCommands.DO_ORBIT, param1 = 20f, param2 = Float.NaN,
                    param3 = 5f, param4 = Float.NaN, x = northOf(35.0), y = latE7(LON), z = 15f),
                item(9, MissionCommands.NAV_RETURN_TO_LAUNCH, frame = MissionFrames.MISSION),
            )
        )
    }

    // -------------------------------------------------------------------------------- frames

    @Test
    fun `an absolute altitude is refused, not converted`() {
        // THE landmine. QGC's Plan view lets an operator pick AMSL per item and plan-wide, and a
        // plan built that way must not upload: our "AMSL" is DJI's pressure altitude on the
        // 1013.25 hPa reference, measured 14 m high one day and 28 m low the next. There is no
        // arithmetic that recovers the operator's datum from ours.
        listOf(MissionFrames.GLOBAL, MissionFrames.GLOBAL_INT).forEach { frame ->
            val refusal = assertRefused(
                listOf(item(0, MissionCommands.NAV_WAYPOINT, frame = frame, x = northOf(10.0), y = latE7(LON))),
                MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME,
                seq = 0,
            )
            // The sentence must say what to do, because this refusal will surprise an operator
            // who has a perfectly good plan in front of them.
            assertEquals(MissionStatusTexts.ABSOLUTE_ALTITUDE, refusal.reason)
        }
    }

    @Test
    fun `both relative frames are accepted and mean the same thing`() {
        // QGC normalises 6 → 3 on read (`PlanManager.cc:417-421`) but may upload either.
        listOf(MissionFrames.GLOBAL_RELATIVE_ALT, MissionFrames.GLOBAL_RELATIVE_ALT_INT)
            .forEach { frame ->
                assertAccepted(
                    listOf(item(0, MissionCommands.NAV_WAYPOINT, frame = frame,
                        x = northOf(10.0), y = latE7(LON)))
                )
            }
    }

    @Test
    fun `terrain frames are refused`() {
        // We claim no MAV_PROTOCOL_CAPABILITY_TERRAIN and QGC hides terrain frames from a PX4
        // vehicle anyway (`VehicleSupports::terrainFrame()` is literally `!px4Firmware()`), so
        // this should never arrive. Refusing costs nothing and catches an imported plan.
        listOf(MissionFrames.GLOBAL_TERRAIN_ALT, MissionFrames.GLOBAL_TERRAIN_ALT_INT)
            .forEach { frame ->
                assertRefused(
                    listOf(item(0, MissionCommands.NAV_WAYPOINT, frame = frame,
                        x = northOf(10.0), y = latE7(LON))),
                    MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME,
                    seq = 0,
                )
            }
    }

    @Test
    fun `MAV_FRAME_MISSION is accepted for a non-coordinate item and refused for a coordinate one`() {
        // A DO_SET_ROI_NONE has no place, so the frame is harmless…
        assertAccepted(
            listOf(
                item(0, MissionCommands.DO_SET_ROI_NONE, frame = MissionFrames.MISSION),
                waypoint(1, 10.0),
            )
        )
        // …but a waypoint under it would have its x/y read unscaled.
        //
        // **The values matter and this test used to have the wrong ones.** With
        // `Int.MAX_VALUE`/`Int.MIN_VALUE` the mutant that accepts `MAV_FRAME_MISSION` for a
        // coordinate item still died — but on `INVALID_PARAM5_X` from the *coordinate* check
        // downstream, not on the frame guard this test is named after. It was passing for the
        // wrong reason, and the guard it claims to defend was undefended.
        //
        // These are QGC's own values for this frame: it writes `item->param5()` **unscaled**
        // (`PlanManager.cc:547-548`), so a target at 38 °N, 23.7 °E arrives as `x = 38, y = 23`.
        // Read as 1e7 degrees those are 3.8e-6 and 2.3e-6 — in range, distinct, and a few
        // centimetres from null island off the coast of Ghana. No range check can catch that; only
        // the frame check can. Its twin in `MissionStoreTest` pins the same shape one layer down.
        assertRefused(
            listOf(item(0, MissionCommands.NAV_WAYPOINT, frame = MissionFrames.MISSION,
                x = 38, y = 23)),
            MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME,
            seq = 0,
        )
    }

    // ---------------------------------------------------------------------------- vocabulary

    @Test
    fun `camera items are refused with the reason an operator can act on`() {
        // The refusal that will actually bite: Survey, Corridor Scan and Structure Scan all
        // expand into plain waypoints **plus** camera items, so the most common real plan an
        // operator builds is waypoints we can fly and shutter commands we cannot honour.
        // Accepting one would be a survey that flies the pattern and brings back nothing,
        // discovered on the ground afterwards.
        val cameraCommands = listOf(
            MissionCommands.SET_CAMERA_MODE,
            MissionCommands.IMAGE_START_CAPTURE,
            MissionCommands.IMAGE_STOP_CAPTURE,
            MissionCommands.VIDEO_START_CAPTURE,
            MissionCommands.VIDEO_STOP_CAPTURE,
            MissionCommands.DO_DIGICAM_CONTROL,
            MissionCommands.DO_SET_CAM_TRIGG_DIST,
        )
        cameraCommands.forEach { command ->
            val refusal = assertRefused(
                listOf(waypoint(0, 10.0), item(1, command, frame = MissionFrames.MISSION)),
                MavMissionResult.MAV_MISSION_UNSUPPORTED,
                seq = 1,
            )
            assertEquals("Item 1 refused: no camera control", refusal.reason)
        }
    }

    @Test
    fun `the whole refused vocabulary is UNSUPPORTED rather than DENIED`() {
        // JC-4. QGC renders them as "Command is not supported" versus "Not accepting any mission
        // commands" (`:755`, `:788`). The first is true of a command we will never fly; the
        // second is true only while we are busy. Same distinction HandshakeResponder already
        // draws on the command surface.
        val refused = listOf(
            MissionCommands.DO_JUMP,
            MissionCommands.CONDITION_YAW,
            MissionCommands.CONDITION_GATE,
            MissionCommands.DO_SET_HOME,
            MissionCommands.DO_LAND_START,
            MissionCommands.DO_MOUNT_CONFIGURE,
            MissionCommands.DO_SET_ROI_WPNEXT_OFFSET,
            MissionCommands.NAV_LOITER_TURNS,
            MissionCommands.NAV_LOITER_TO_ALT,
            MissionCommands.DO_SET_SERVO,
            MissionCommands.DO_SET_ACTUATOR,
            MissionCommands.DO_GRIPPER,
        )
        refused.forEach { command ->
            assertRefused(
                listOf(waypoint(0, 10.0), item(1, command, frame = MissionFrames.MISSION)),
                MavMissionResult.MAV_MISSION_UNSUPPORTED,
                seq = 1,
            )
        }
    }

    @Test
    fun `a command nobody has heard of is refused by the allow-list, not accepted by an else`() {
        // The property that matters more than any individual row: an allow-list is the only list
        // that fails safe when QGC gains a feature.
        val invented = 31337
        val refusal = assertRefused(
            listOf(waypoint(0, 10.0), item(1, invented, frame = MissionFrames.MISSION)),
            MavMissionResult.MAV_MISSION_UNSUPPORTED,
            seq = 1,
        )
        assertEquals("Item 1 refused: ${MissionAdmission.UNKNOWN_COMMAND_REASON}", refusal.reason)
    }

    @Test
    fun `a fence or rally item is refused rather than stored`() {
        listOf(MissionTypes.FENCE, MissionTypes.RALLY).forEach { type ->
            assertRefused(
                listOf(item(0, MissionCommands.NAV_WAYPOINT, missionType = type,
                    x = northOf(10.0), y = latE7(LON))),
                MavMissionResult.MAV_MISSION_UNSUPPORTED,
                seq = 0,
            )
        }
    }

    // ----------------------------------------------------------------------------- structure

    @Test
    fun `takeoff is admissible only at seq 0`() {
        assertAccepted(listOf(item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f)))
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f)),
            MavMissionResult.MAV_MISSION_INVALID_SEQUENCE,
            seq = 1,
        )
    }

    @Test
    fun `land and return are admissible only as the last item`() {
        // NAV_LAND carries a coordinate, so it must be in a relative frame; RTL carries none by
        // MAVLink's definition and QGC sends it under MAV_FRAME_MISSION.
        val terminal = listOf(
            item(1, MissionCommands.NAV_LAND, x = 0, y = 0, z = 0f),
            item(1, MissionCommands.NAV_RETURN_TO_LAUNCH, frame = MissionFrames.MISSION),
        )
        terminal.forEach { last ->
            assertAccepted(listOf(waypoint(0, 10.0), last))
            assertRefused(
                listOf(waypoint(0, 10.0), last, waypoint(2, 20.0)),
                MavMissionResult.MAV_MISSION_INVALID_SEQUENCE,
                seq = 1,
            )
        }
    }

    @Test
    fun `hover-until-told-otherwise is terminal by construction`() {
        // An item after NAV_LOITER_UNLIM is unreachable, and an unreachable item in a plan is a
        // misunderstanding worth surfacing at the desk rather than in the air.
        assertAccepted(listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LOITER_UNLIM, z = 10f)))
        assertRefused(
            listOf(item(0, MissionCommands.NAV_LOITER_UNLIM, z = 10f), waypoint(1, 10.0)),
            MavMissionResult.MAV_MISSION_INVALID_SEQUENCE,
            seq = 0,
        )
    }

    @Test
    fun `two takeoffs or two landings are not a plan`() {
        // Through the whole function the positional rules get there first, and that is fine —
        // same code, same seq, same sentence family.
        assertRefused(
            listOf(
                item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
                item(1, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
            ),
            MavMissionResult.MAV_MISSION_INVALID_SEQUENCE,
            seq = 1,
        )
    }

    @Test
    fun `the whole-plan half refuses a duplicate phase on its own`() {
        // **The uniqueness check is unreachable through `admissible` and is kept anyway**, so it
        // has to be pinned where it can be reached: [MissionAdmission.planRefusal] alone.
        //
        // The reason it is unreachable is worth writing down rather than discovering later. Two
        // takeoffs cannot both be at seq 0, and two landings cannot both be last, so the
        // positional rules in `itemRefusal` already refuse every list the uniqueness rule would —
        // which is why the mutation that deletes it scored **0** through the front door. It stays
        // because the positional rules are the kind of thing that gets relaxed (a takeoff after a
        // land was floated once), and the day one of them goes this is the layer that catches the
        // plan with two takeoff phases in it.
        listOf(
            MissionCommands.NAV_TAKEOFF to "one takeoff only",
            MissionCommands.NAV_LAND to "one landing only",
            MissionCommands.NAV_RETURN_TO_LAUNCH to "one return only",
        ).forEach { (command, reason) ->
            val refusal = MissionAdmission.planRefusal(
                listOf(
                    item(0, command, frame = MissionFrames.MISSION, x = 0, y = 0, z = 5f),
                    waypoint(1, 10.0),
                    item(2, command, frame = MissionFrames.MISSION, x = 0, y = 0, z = 5f),
                )
            )
            assertNotNull("two $command items must not be a plan", refusal)
            assertEquals(MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, refusal!!.result)
            assertEquals(2, refusal.seq)
            assertEquals("Item 2 refused: $reason", refusal.reason)
        }
    }

    @Test
    fun `a plan of nothing but DO items has no legs`() {
        val refusal = assertRefused(
            listOf(
                item(0, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 1f, param2 = 2f),
                item(1, MissionCommands.DO_SET_ROI_NONE, frame = MissionFrames.MISSION),
            ),
            MavMissionResult.MAV_MISSION_ERROR,
            seq = null,
        )
        // A whole-plan failure has no natural index and QGC will blame the *last* item we
        // requested, so this sentence must carry the reason without naming an item.
        assertEquals(MissionStatusTexts.NO_NAVIGABLE_ITEM, refusal.reason)
        assertTrue("a whole-plan sentence must not name an item", !refusal.reason.startsWith("Item"))
    }

    // -------------------------------------------------------------------------------- bounds

    @Test
    fun `the item count is checked before a single item is requested`() {
        // The one refusal that does not wait for an item, because the count is the only failure
        // with no index to report — a 500-item survey is refused in one message instead of after
        // 500 round trips.
        assertNull(MissionAdmission.countRefusal(MissionAdmission.MAX_ITEMS))
        val refusal = MissionAdmission.countRefusal(MissionAdmission.MAX_ITEMS + 1)!!
        assertEquals(MavMissionResult.MAV_MISSION_NO_SPACE, refusal.result)
        assertNull(refusal.seq)
        assertEquals(
            "Plan too long: ${MissionAdmission.MAX_ITEMS + 1} items, ${MissionAdmission.MAX_ITEMS} max",
            refusal.reason,
        )
    }

    @Test
    fun `an altitude above the ceiling is refused, never capped`() {
        // M3 Q1's rule: an upload is a re-doable transaction, so there is no cost to saying no —
        // and a clamped altitude is one the operator believes was obeyed.
        assertAccepted(listOf(waypoint(0, 10.0, z = MissionAdmission.CEILING_M.toFloat())))
        val refusal = assertRefused(
            listOf(waypoint(0, 10.0, z = (MissionAdmission.CEILING_M + 0.1).toFloat())),
            MavMissionResult.MAV_MISSION_INVALID_PARAM7,
            seq = 0,
        )
        assertTrue("the sentence must name the limit", refusal.reason.contains("100m max"))
    }

    @Test
    fun `a waypoint at or below home is refused and a landing at zero is not`() {
        assertRefused(
            listOf(waypoint(0, 10.0, z = 0f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM7,
            seq = 0,
        )
        // Landing at zero above home is the whole instruction, so it is the one item allowed it.
        assertAccepted(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LAND, x = 0, y = 0, z = 0f))
        )
    }

    @Test
    fun `a single leg beyond the reposition cap is refused`() {
        assertAccepted(listOf(waypoint(0, 0.0), waypoint(1, MissionAdmission.MAX_LEG_M - 1.0)))
        val refusal = assertRefused(
            listOf(waypoint(0, 0.0), waypoint(1, MissionAdmission.MAX_LEG_M + 20.0)),
            MavMissionResult.MAV_MISSION_ERROR,
            seq = 1,
        )
        // The sentence names the limit in force — 2 km since 2026-07-30, and derived from the
        // constant so that this asserts *"the refusal names its bound"* rather than *"the bound is
        // 100"*. A refusal string that stops quoting the number fails here.
        assertTrue(
            "the sentence must name the limit, got '${refusal.reason}'",
            refusal.reason.contains("${MissionAdmission.MAX_LEG_M.toInt()}m max"),
        )
    }

    @Test
    fun `the whole path is bounded too`() {
        // Six 700 m legs: every one inside the 2 km leg bound, 4200 m in total, which is not — the
        // total is an out-and-back at the reach (4 km) since 2026-07-30, because a single 2 km leg
        // would otherwise have been admitted by one bound and refused by the other.
        val items = (0..6).map { waypoint(it, it * 700.0) }
        val refusal = assertRefused(items, MavMissionResult.MAV_MISSION_ERROR, seq = null)
        assertTrue(
            "the sentence must name the limit, got '${refusal.reason}'",
            refusal.reason.contains("${MissionAdmission.MAX_TOTAL_M.toInt()}m max"),
        )
        // Each leg on its own is admissible: it really is the *total* that refuses this plan.
        assertAccepted(listOf(waypoint(0, 0.0), waypoint(1, 700.0)))
    }

    @Test
    fun `a zero-length leg is refused because it has no direction`() {
        // The execution half's pass-through test is defined against the unit vector along the
        // leg, and two identical points do not have one.
        assertRefused(
            listOf(waypoint(0, 10.0), waypoint(1, 10.0)),
            MavMissionResult.MAV_MISSION_ERROR,
            seq = 1,
        )
    }

    // ---------------------------------------------------------------------------- parameters

    @Test
    fun `the acceptance radius is 0, NaN, or inside the band`() {
        assertAccepted(listOf(waypoint(0, 10.0).copy(param2 = 0f)))
        assertAccepted(listOf(waypoint(0, 10.0).copy(param2 = Float.NaN)))
        assertAccepted(listOf(waypoint(0, 10.0).copy(param2 = MissionAdmission.MIN_ACCEPT_RADIUS_M.toFloat())))
        assertAccepted(listOf(waypoint(0, 10.0).copy(param2 = MissionAdmission.MAX_ACCEPT_RADIUS_M.toFloat())))

        assertRefused(
            listOf(waypoint(0, 10.0).copy(param2 = 1f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 0,
        )
        assertRefused(
            listOf(waypoint(0, 10.0).copy(param2 = 25f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 0,
        )
    }

    @Test
    fun `a waypoint pass radius and a finite yaw are both refused`() {
        // Refused rather than ignored, for the same reason M3 refuses a finite DO_REPOSITION
        // param4: the engine pins yaw rate to zero and does not fly a pass radius, so accepting
        // either would be a plan the operator believes will be flown as authored.
        assertRefused(
            listOf(waypoint(0, 10.0).copy(param3 = 12f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM3, seq = 0,
        )
        assertRefused(
            listOf(waypoint(0, 10.0).copy(param4 = 1.57f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM4, seq = 0,
        )
    }

    @Test
    fun `a takeoff or a hover carrying a yaw is refused exactly as a waypoint is`() {
        // Settled 2026-07-27. Until then only NAV_WAYPOINT refused a commanded yaw, so the very
        // same field on a takeoff or a loiter uploaded silently — and was then flown as neither,
        // because the guided engine pins yaw rate to zero. Two doors into one engine cannot answer
        // differently about the same field, and the one that says nothing is the dangerous one.
        assertAccepted(listOf(item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f)))
        val takeoffYaw = assertRefused(
            listOf(item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f, param4 = 1.57f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM4, seq = 0,
        )
        // The same sentence as the waypoint's, verbatim: it is the same refusal.
        assertEquals("Item 0 refused: yaw is pinned to zero", takeoffYaw.reason)

        assertRefused(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.NAV_LOITER_TIME, param1 = 10f, z = 10f,
                    x = northOf(20.0), y = latE7(LON), param4 = 1.57f),
            ),
            MavMissionResult.MAV_MISSION_INVALID_PARAM4, seq = 1,
        )
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LOITER_UNLIM, z = 10f,
                x = northOf(20.0), y = latE7(LON), param4 = 1.57f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM4, seq = 1,
        )
    }

    @Test
    fun `a loiter radius is refused rather than flown as a circle`() {
        // JC-8 at the door instead of only in the projection: both loiter commands map to
        // LegKind.HOVER and are flown as a *point*, so accepting a radius of 40 m would be
        // accepting a circle and flying a dot — the operator's plan saying one thing and the
        // aircraft doing another, in the direction of more motion.
        assertRefused(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.NAV_LOITER_TIME, param1 = 10f, param3 = 40f, z = 10f,
                    x = northOf(20.0), y = latE7(LON)),
            ),
            MavMissionResult.MAV_MISSION_INVALID_PARAM3, seq = 1,
        )
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LOITER_UNLIM, param3 = 40f,
                z = 10f, x = northOf(20.0), y = latE7(LON))),
            MavMissionResult.MAV_MISSION_INVALID_PARAM3, seq = 1,
        )
        // Zero and NaN are "no radius asked for" and stay admissible — QGC leaves the field alone
        // for a multirotor, and refusing its own default would refuse every plan it writes.
        assertAccepted(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LOITER_UNLIM, param3 = 0f,
                z = 10f, x = northOf(20.0), y = latE7(LON)))
        )
        assertAccepted(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.NAV_LOITER_UNLIM, param3 = Float.NaN,
                z = 10f, x = northOf(20.0), y = latE7(LON)))
        )
    }

    @Test
    fun `a speed above the envelope is refused and names the limit`() {
        val refusal = assertRefused(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 1f, param2 = 8f),
            ),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 1,
        )
        assertEquals("Item 1 speed 8.0 refused: 3.0 max", refusal.reason)
    }

    @Test
    fun `an airspeed request is refused rather than read as a ground speed`() {
        assertRefused(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 0f, param2 = 2f),
            ),
            MavMissionResult.MAV_MISSION_INVALID_PARAM1, seq = 1,
        )
    }

    @Test
    fun `a negative speed is MAVLink's no-change and a zero one is a stop in disguise`() {
        assertAccepted(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 1f, param2 = -1f),
            )
        )
        assertRefused(
            listOf(
                waypoint(0, 10.0),
                item(1, MissionCommands.DO_CHANGE_SPEED, frame = MissionFrames.MISSION,
                    param1 = 1f, param2 = 0f),
            ),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 1,
        )
    }

    @Test
    fun `gimbal roll and yaw are refused, because the airframe refuses them`() {
        // DJI #527: yaw and roll are refused *by the aircraft*. A plan whose camera move will not
        // happen must not upload silently.
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_MOUNT_CONTROL,
                frame = MissionFrames.MISSION, param1 = -30f, param2 = 5f, z = 2f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 1,
        )
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_MOUNT_CONTROL,
                frame = MissionFrames.MISSION, param1 = -30f, param3 = 90f, z = 2f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM3, seq = 1,
        )
        // And the mount mode we do not offer.
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_MOUNT_CONTROL,
                frame = MissionFrames.MISSION, param1 = -30f, z = 0f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM7, seq = 1,
        )
    }

    @Test
    fun `the gimbal manager form refuses yaw on its own field layout`() {
        // Not "as 205": here param2 is the yaw *angle* and param4 is the yaw *rate*, so the two
        // fields that must be absent are 2 and 4 rather than 2 and 3.
        assertAccepted(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_GIMBAL_MANAGER_PITCHYAW,
                frame = MissionFrames.MISSION, param1 = -45f, param2 = 0f, param3 = 0f, param4 = 0f))
        )
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_GIMBAL_MANAGER_PITCHYAW,
                frame = MissionFrames.MISSION, param1 = -45f, param2 = 90f, param4 = 0f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM2, seq = 1,
        )
        assertRefused(
            listOf(waypoint(0, 10.0), item(1, MissionCommands.DO_GIMBAL_MANAGER_PITCHYAW,
                frame = MissionFrames.MISSION, param1 = -45f, param2 = 0f, param4 = 30f)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM4, seq = 1,
        )
    }

    @Test
    fun `an orbit that would need yaw authority is refused, never downgraded`() {
        // A downgrade is a command the operator believes was obeyed. QGC's orbit UI implies
        // nose-to-centre, so silently flying it heading-locked would be exactly that.
        listOf(
            OrbitYawBehaviour.HOLD_FRONT_TO_CIRCLE_CENTER,
            OrbitYawBehaviour.HOLD_FRONT_TANGENT_TO_CIRCLE,
        ).forEach { behaviour ->
            assertRefused(
                listOf(item(0, MissionCommands.DO_ORBIT, param1 = 20f, param2 = Float.NaN,
                    param3 = behaviour.toFloat(), param4 = Float.NaN,
                    x = northOf(10.0), y = latE7(LON), z = 15f)),
                MavMissionResult.MAV_MISSION_INVALID_PARAM3, seq = 0,
            )
        }
        // The four we can honour with yaw pinned, including QGC's own default of 5 — a value
        // absent from our dialect, which is why it is compared as an integer.
        listOf(
            OrbitYawBehaviour.HOLD_INITIAL_HEADING,
            OrbitYawBehaviour.UNCONTROLLED,
            OrbitYawBehaviour.RC_CONTROLLED,
            OrbitYawBehaviour.UNCHANGED,
        ).forEach { behaviour ->
            assertAccepted(
                listOf(item(0, MissionCommands.DO_ORBIT, param1 = 20f, param2 = Float.NaN,
                    param3 = behaviour.toFloat(), param4 = Float.NaN,
                    x = northOf(10.0), y = latE7(LON), z = 15f))
            )
        }
    }

    @Test
    fun `an orbit radius outside the 5 to 50 metre band is refused`() {
        // M4-3's band. Below 5 m the radial error is inside GPS noise and the required yaw rate
        // approaches the 30 deg/s cap; above 50 m the circle stops fitting inside the 150 m home
        // distance the execution half checks at Start.
        fun orbit(radius: Float) = listOf(
            item(0, MissionCommands.DO_ORBIT, param1 = radius, param2 = Float.NaN,
                param3 = 5f, param4 = Float.NaN, x = northOf(10.0), y = latE7(LON), z = 15f)
        )
        // The **magnitude** is what is bounded: QGC encodes counter-clockwise as a negative
        // param1, so a signed comparison would accept an arbitrarily large anticlockwise circle
        // and refuse every clockwise one.
        assertAccepted(orbit(-30f))
        assertAccepted(orbit(MissionAdmission.ORBIT_MIN_RADIUS_M.toFloat()))
        assertAccepted(orbit(MissionAdmission.ORBIT_MAX_RADIUS_M.toFloat()))
        assertRefused(orbit(3f), MavMissionResult.MAV_MISSION_INVALID_PARAM1, seq = 0)
        assertRefused(orbit(-60f), MavMissionResult.MAV_MISSION_INVALID_PARAM1, seq = 0)
        assertRefused(orbit(0f), MavMissionResult.MAV_MISSION_INVALID_PARAM1, seq = 0)
    }

    @Test
    fun `a coordinate that is not a place is refused, except for takeoff and land here`() {
        assertRefused(
            listOf(item(0, MissionCommands.NAV_WAYPOINT, x = 0, y = 0)),
            MavMissionResult.MAV_MISSION_INVALID_PARAM5_X, seq = 0,
        )
        // QGC's "here": take off from where you are, land where you are. Rejected by `Geo` as the
        // canonical unknown-encoded-as-zero, which is right for a position report and wrong for
        // these two items.
        assertAccepted(listOf(item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f)))
    }
}
