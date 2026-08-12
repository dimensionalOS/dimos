package com.dimensional.mini4pro.mission

import com.dimensional.mini4pro.guided.GuidedEnvelope
import com.dimensional.mini4pro.guided.MissionGuidance
import io.dronefleet.mavlink.common.MavMissionResult

/**
 * A refusal: the most specific `MAV_MISSION_RESULT` that is true, and the sentence that says why.
 *
 * Both channels are mandatory and the pairing is the design (§4.4). QGC turns the result code
 * into its own sentence and appends the item and its friendly command name — and for
 * `UNSUPPORTED_FRAME` it appends the frame number, and for `INVALID_PARAM<n>` it appends *the
 * value*. So `INVALID_PARAM3` on item 5 produces *"Param 3 invalid value. Item #5 Command:
 * Waypoint Value: 12"* with no work from us. What it never says is **why**, which is [reason].
 *
 * [seq] is the offending item's wire sequence number, or null for a whole-plan failure — which
 * has no natural index, and whose sentence must therefore not name one.
 */
data class MissionRefusal(
    val result: MavMissionResult,
    val reason: String,
    val seq: Int?,
)

/**
 * **Static admission: is this list of items a plan this bridge is willing to hold?**
 *
 * Pure functions over the item list. No clock, no aircraft state, no DJI, no I/O — the same inputs
 * always give the same answer, so every branch is a unit test.
 *
 * **Which of them the shipping path calls, precisely.** [MissionTransaction] calls [itemRefusal]
 * on each item as it arrives and [planRefusal] once when the last one has, both before
 * `MISSION_ACK(MAV_MISSION_ACCEPTED)` goes out — that is what makes "refused before it ever flies"
 * structural rather than a habit (`docs/m4-mission-execution.md` §2.1 rule 3). It does **not**
 * call [admissible]; see that function for what it is and is not. The distinction was worth
 * nothing until a mutation table read as though it were the other way round.
 *
 * The *launch* half of admission — interlock, fix freshness, home, battery, aircraft state — is
 * the execution half's and is deliberately absent here: those are facts about the world at the
 * instant of Start, not facts about a list.
 *
 * ## Refusal is at the item, not at the count (JC-2)
 *
 * [MissionTransaction] requests **every** item and validates as they arrive, refusing at the
 * first failure, rather than rejecting an upload the moment it can. It costs up to N round trips;
 * it buys QGC naming the item, the command and the offending value in its own dialog, for free,
 * and the index is the only thing that lets an operator fix the plan. The one exception is the
 * item *count*, which has no index and is checked immediately — a 500-item survey is refused in
 * one message instead of after 500 round trips.
 *
 * ## The frame rule, and why absolute altitudes are refused rather than converted
 *
 * This is the finding that separates M4 from M3 and it has to be stated loudly.
 *
 * Stage B's `DO_REPOSITION` carries an AMSL that **QGC composed from our own last published
 * AMSL** (`PX4FirmwarePlugin.cc:317-338`), so the datum enters QGC's sum with one sign and leaves
 * ours with the other and cancels exactly (`docs/m3-stage-b.md` §3). That is what makes a
 * pressure altitude safe to subtract.
 *
 * **A plan item's absolute altitude has no such provenance.** `AltitudeFrameAbsolute` is a number
 * an operator typed, or a terrain-database lookup, or a value saved in a `.plan` file last month
 * (`SimpleMissionItem.cc:733-742`). It is in a real-world datum; ours is pressure altitude on the
 * 1013.25 hPa reference, off by `(1013.25 − QNH) × 8.3 m` — measured at 14 m high one day and
 * 28 m low the next, a 41.5 m swing (`docs/measurements/2026-07-26-amsl-datum.md`). That document
 * ends in a requirement and this is it applied: **an AMSL from any other source is in a different
 * datum and must be refused, not converted.**
 *
 * | frame | id | verdict |
 * |---|---|---|
 * | `MAV_FRAME_GLOBAL_RELATIVE_ALT` | 3 | accept — `z` is metres above home, no datum to get wrong |
 * | `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` | 6 | accept, identical meaning |
 * | `MAV_FRAME_GLOBAL` | 0 | **refuse**, `MAV_MISSION_UNSUPPORTED_FRAME` — foreign datum |
 * | `MAV_FRAME_GLOBAL_INT` | 5 | **refuse**, same |
 * | `MAV_FRAME_GLOBAL_TERRAIN_ALT` | 10 / 11 | **refuse** — we claim no `TERRAIN` capability, and QGC hides terrain frames from a PX4 vehicle anyway (`VehicleSupports.cc:43-46`), so it should never arrive; refusing costs nothing and catches an imported plan |
 * | `MAV_FRAME_MISSION` | 2 | accept **for non-coordinate items only** — QGC does not scale `x`/`y` by 1e7 for this frame (`PlanManager.cc:547-548`) and survey items carry `param5/6/7 = NaN`, so `x`/`y` are whatever a `double NaN → int32` cast produced |
 * | anything else | | **refuse** |
 *
 * The refusal must therefore say what to do, which is what
 * [MissionStatusTexts.ABSOLUTE_ALTITUDE] is for. It is a real cost — QGC lets an operator author
 * a plan in AMSL and that plan will not upload — and it is reversible only by supplying a way to
 * establish the site's true elevation, which
 * `docs/measurements/2026-07-26-px4-reverify.md` finding 1 says nobody has. Transport Q3.
 *
 * ## Bounds, and where each number came from
 *
 * Every envelope number is **imported from [GuidedEnvelope]** rather than restated, so a plan
 * cannot be admitted against a ceiling the guided engine does not enforce. The two that are not
 * M3's are [MAX_ITEMS] and [MAX_TOTAL_M], both from the execution design's §7.1.
 */
object MissionAdmission {

    /**
     * The most items this bridge will hold.
     *
     * The transport design proposed 100 (`MAV_MISSION_NO_SPACE`'s natural reading) and the
     * execution design proposed **20** on supervisability grounds — *"a Mini 4 Pro flight is
     * minutes long; a plan the operator cannot hold in their head is not a plan they can
     * supervise"*. The transport design settles it explicitly: **the stricter number wins**, and
     * it is Ivan's Q3 to relax.
     */
    const val MAX_ITEMS = 20

    /** Q1's ceiling, above our own takeoff datum. Refused, never capped — an upload is redoable. */
    const val CEILING_M = GuidedEnvelope.CEILING_M

    /**
     * Q1's single-reposition cap, applied per leg: the distance beyond which committed
     * straight-line motion stops being supervisable.
     *
     * **Which legs it bounds, precisely, because "every leg" was never true.** [geometryRefusal]
     * measures leg *n* from the coordinate of item *n−1*, so the plan's **first** leg — from where
     * the aircraft actually is to the first waypoint — has no predecessor in the list and was
     * bounded by nothing at all. A plan of one waypoint 3 km away passed every check here.
     *
     * As of 2026-07-27 it is bounded in two places, and neither is a substitute for the other:
     *
     *  1. **At Start, always,** by the execution half, which requires every item within its own
     *     home-distance bound before a mission may begin. At Start home is known, because the
     *     aircraft is standing on it. That is the guarantee.
     *  2. **At upload, when we can** — [firstLegRefusal], run only when home is known at upload
     *     time. It is a courtesy, not a guarantee: it moves the refusal to the desk, where the
     *     operator fixes the plan in seconds, instead of to Start with propellers turning. When
     *     home is unknown at upload the check does not run, and that is expected rather than a
     *     gap.
     *
     * **2 km since 2026-07-30** — the constant follows [GuidedEnvelope.MAX_REPOSITION_DISTANCE_M]
     * by import, as it always has, so Ivan's decision reached the desk without a second edit. The
     * sentence an operator gets still names the number (`Leg 5 is 2400m: 2000m max`), which is the
     * half of this that must never be lost: `big1.plan`'s refusal was legible, and that is why it
     * became a decision instead of a mystery.
     */
    const val MAX_LEG_M = GuidedEnvelope.MAX_REPOSITION_DISTANCE_M

    /**
     * The whole-path bound — **[MissionGuidance.MAX_PATH_M], an out-and-back at the envelope's
     * reach**, 4 km since 2026-07-30.
     *
     * It was Ivan's flat 500 m (M4-3, *"~2.8 min of flying at cruise; comfortably inside a battery
     * and inside attention"*), which was coherent with a 100 m reach and is not with a 2 km one: a
     * single admissible 2 km leg is four times the old total, so the leg bound would have admitted
     * what this refused. Now derived in the guided layer so the two cannot drift apart, and see that
     * constant for why "out and back" is the shape rather than a larger free number.
     *
     * Still a **refusal** threshold, and still the only thing standing between 20 items at 2 km each
     * and 40 km of drawn path.
     */
    const val MAX_TOTAL_M = MissionGuidance.MAX_PATH_M

    /** Q1's horizontal envelope. A `DO_CHANGE_SPEED` above it is refused, never clamped. */
    const val MAX_SPEED_MPS = GuidedEnvelope.HORIZONTAL_MAX_MS

    /** `NAV_WAYPOINT.param2` band. Outside it — and non-zero — the request is refused. */
    const val MIN_ACCEPT_RADIUS_M = 2.0
    const val MAX_ACCEPT_RADIUS_M = 10.0

    /**
     * `DO_ORBIT.param1`'s magnitude band, settled as M4-3.
     *
     * Below 5 m *"the radial error is inside GPS noise and the required yaw rate approaches the
     * 30 °/s cap"*. Both are refusal thresholds, so being wrong means refusing an orbit that would
     * have been fine — visible, annoying and safe.
     *
     * **50 m survives the 2026-07-30 envelope expansion on a different argument than the one it was
     * given.** M4-3's reason was *"the whole circle must fit inside the 150 m home distance"*, and
     * that reason is gone — 2 km of home distance would fit a 900 m circle. The reason it stays is
     * the circling law's own clock: one turn at radius R costs `2*pi*R / 3 m/s`, so R = 50 m is
     * 105 s against [com.dimensional.mini4pro.guided.OrbitGuidance.ORBIT_MAX_S]'s 180 s, and
     * R = 100 m would be 210 s — **an orbit that can never complete a single turn before its own
     * time cap ends it**. Widening the radius is therefore a decision about `ORBIT_MAX_S`, not about
     * reach, and nobody has taken it.
     *
     * The orbit's own 180 s cap (`ORBIT_MAX_S`) is deliberately **not** here: an orbit has no
     * arrival and something has to end it, but that something is a clock during flight rather
     * than a fact about a list, so it belongs to the execution half's launch and timeout gates.
     */
    const val ORBIT_MIN_RADIUS_M = 5.0
    const val ORBIT_MAX_RADIUS_M = 50.0

    /** `DO_MOUNT_CONTROL.param7`: `MAV_MOUNT_MODE_MAVLINK_TARGETING`. The only mode we offer. */
    const val MOUNT_MODE_MAVLINK_TARGETING = 2

    /**
     * Everything this bridge will accept in a plan, with the reason word each refusal carries.
     * An **allow-list**, because an allow-list is the only list that fails safe when QGC gains a
     * feature: a command nobody has thought about is refused by [UNKNOWN_COMMAND_REASON] rather
     * than flown by an `else` branch that happened to be permissive.
     */
    val ACCEPTED_COMMANDS: Set<Int> = setOf(
        MissionCommands.NAV_WAYPOINT,
        MissionCommands.NAV_TAKEOFF,
        MissionCommands.NAV_LAND,
        MissionCommands.NAV_RETURN_TO_LAUNCH,
        MissionCommands.NAV_LOITER_TIME,
        MissionCommands.NAV_LOITER_UNLIM,
        MissionCommands.NAV_DELAY,
        MissionCommands.DO_CHANGE_SPEED,
        MissionCommands.DO_SET_ROI_LOCATION,
        MissionCommands.DO_SET_ROI_NONE,
        MissionCommands.DO_SET_ROI,
        MissionCommands.DO_MOUNT_CONTROL,
        MissionCommands.DO_GIMBAL_MANAGER_PITCHYAW,
        MissionCommands.DO_ORBIT,
    )

    /** What the operator reads for a command we will never fly. Keep each under ~20 bytes. */
    private val REFUSAL_REASONS: Map<Int, String> = mapOf(
        MissionCommands.SET_CAMERA_MODE to "no camera control",
        MissionCommands.IMAGE_START_CAPTURE to "no camera control",
        MissionCommands.IMAGE_STOP_CAPTURE to "no camera control",
        MissionCommands.VIDEO_START_CAPTURE to "no camera control",
        MissionCommands.VIDEO_STOP_CAPTURE to "no camera control",
        MissionCommands.DO_DIGICAM_CONTROL to "no camera control",
        MissionCommands.DO_SET_CAM_TRIGG_DIST to "no camera control",
        MissionCommands.DO_JUMP to "no jumps in a plan",
        MissionCommands.CONDITION_YAW to "yaw is pinned to zero",
        MissionCommands.CONDITION_GATE to "no condition gates",
        MissionCommands.DO_SET_HOME to "home cannot move midplan",
        MissionCommands.DO_LAND_START to "no land-start marker",
        MissionCommands.DO_SET_SERVO to "no such hardware",
        MissionCommands.DO_SET_ACTUATOR to "no such hardware",
        MissionCommands.DO_GRIPPER to "no such hardware",
        MissionCommands.DO_MOUNT_CONFIGURE to "mount mode not offered",
        MissionCommands.DO_SET_ROI_WPNEXT_OFFSET to "needs yaw authority",
        MissionCommands.NAV_LOITER_TURNS to "loiter turns not flown",
        MissionCommands.NAV_LOITER_TO_ALT to "loiter-to-alt not flown",
    )

    const val UNKNOWN_COMMAND_REASON = "command not supported"

    /**
     * The count check, run the instant `MISSION_COUNT` arrives — before a single item is
     * requested. The only check that does not wait for the item, because the count is the only
     * failure with no index to report.
     */
    fun countRefusal(count: Int): MissionRefusal? {
        if (count > MAX_ITEMS) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_NO_SPACE,
                MissionStatusTexts.planTooLong(count, MAX_ITEMS),
                seq = null,
            )
        }
        return null
    }

    /**
     * Everything knowable from one item alone, in order of specificity. Null means "nothing
     * wrong with this item"; it says nothing about the plan.
     *
     * @param index the item's position in the plan, which for a well-formed upload equals
     *   `item.seq`. Passed separately so a caller cannot be fooled by a `seq` field that
     *   disagrees with where the item actually arrived.
     * @param total the plan's item count, needed for the "last item only" structural rules.
     */
    fun itemRefusal(item: StoredItem, index: Int, total: Int): MissionRefusal? {
        val seq = index

        // 1. Mission type. Fence and rally have no capability bit and QGC skips them; anything
        //    else arriving is a foreign ground station and is refused rather than stored.
        if (item.missionType != MissionTypes.MISSION) {
            return refuse(seq, MavMissionResult.MAV_MISSION_UNSUPPORTED, "only plain missions")
        }

        // 2. Vocabulary, before anything else about the item is read: a command we will not fly
        //    makes every parameter question moot, and "not supported" is the true statement about
        //    it (JC-4 — DENIED would mean "not right now", which is a different claim).
        if (item.command !in ACCEPTED_COMMANDS) {
            val reason = REFUSAL_REASONS[item.command] ?: UNKNOWN_COMMAND_REASON
            return refuse(seq, MavMissionResult.MAV_MISSION_UNSUPPORTED, reason)
        }

        // 3. Frame. See the class KDoc for the datum argument; it is the reason this refuses
        //    rather than converts.
        frameRefusal(item, seq)?.let { return it }

        // 4. Structure: the items that have a phase rather than only a position.
        structureRefusal(item, seq, total)?.let { return it }

        // 5. The coordinate itself, through the bridge's one definition of a repeatable
        //    coordinate — the same range and filler checks that caught DJI's 4.58e7 placeholder.
        coordinateRefusal(item, seq)?.let { return it }

        // 6. Altitude, against Q1's ceiling.
        altitudeRefusal(item, seq)?.let { return it }

        // 7. Per-command parameters, each with the INVALID_PARAM<n> QGC will decorate with the
        //    offending value.
        return parameterRefusal(item, seq)
    }

    /**
     * Everything that is only knowable once the whole plan is present: the count, the structural
     * uniqueness rules, and the geometry.
     *
     * Whole-plan failures have no natural index and QGC will blame the *last* item we requested,
     * which is slightly misleading — so each sentence here names the real reason and, where it
     * genuinely has no item to blame, names no item at all.
     */
    fun planRefusal(items: List<StoredItem>): MissionRefusal? {
        countRefusal(items.size)?.let { return it }

        // A plan of nothing but DO_* items has no legs. MAV_MISSION_ERROR because no more
        // specific code is true: nothing is unsupported and no parameter is invalid.
        val legs = items.filter { it.command in MissionCommands.NAVIGABLE }
        if (legs.isEmpty()) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_ERROR, MissionStatusTexts.NO_NAVIGABLE_ITEM, seq = null,
            )
        }

        uniquenessRefusal(items)?.let { return it }
        return geometryRefusal(items)
    }

    /**
     * **The first leg, measured from home — a courtesy, and only when home is known.**
     *
     * The leg from where the aircraft is to the plan's first waypoint is the one leg
     * [geometryRefusal] cannot see, because it has no predecessor *in the list* — see [MAX_LEG_M]
     * for the whole shape of the rule. Home is not a fact about a list, so this cannot live in
     * [admissible] and does not: it takes [home] explicitly and [MissionTransaction] passes the
     * same value it records as the plan's provenance.
     *
     * **Null [home] means the check does not run, and that is expected.** The guarantee is the
     * execution half's at Start, where home is known because the aircraft is standing on it. This
     * only moves a refusal that would happen there to the desk, where it costs the operator
     * seconds instead of a walk back to the aircraft.
     *
     * A first leg of *zero* length is deliberately **not** refused, unlike a zero-length leg
     * between two items: a plan whose first waypoint sits on home is a takeoff followed by a
     * hover, which is a real instruction, and home at upload is a position report rather than the
     * surveyed truth.
     */
    fun firstLegRefusal(items: List<StoredItem>, home: GeoPoint?): MissionRefusal? {
        if (home == null) return null
        val first = items
            .filter { it.command in MissionCommands.NAVIGABLE }
            .firstNotNullOfOrNull { item -> MissionGeo.pointOrNull(item)?.let { item.seq to it } }
            ?: return null
        val (seq, point) = first
        val leg = MissionGeo.distanceM(home, point)
        if (leg > MAX_LEG_M) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_ERROR,
                MissionStatusTexts.legTooLong(seq, leg, MAX_LEG_M),
                seq,
            )
        }
        return null
    }

    /**
     * **The composed admission call — and it has no production caller today.**
     *
     * Static bounds only, over a whole list, in the order the transaction applies them: each item
     * on its own, then the plan as a whole. It is the shape the execution design asks for (`§2.1`
     * rule 3, `§7.1`) — *"is this list a plan we would hold?"* answered in one call — and it will
     * be what the execution half asks at Start, where it has a committed list and no transaction
     * to walk.
     *
     * What it is **not** is the path an upload takes. [MissionTransaction] refuses at the item, as
     * the item arrives (JC-2), because that is what makes QGC name the index, the command and the
     * offending value in its own dialog; a composed call at the end would refuse the same plans
     * and tell the operator less. It therefore calls [itemRefusal] and [planRefusal] directly and
     * never this.
     *
     * The two cannot disagree — this is `itemRefusal` in a loop followed by `planRefusal`, the
     * same functions in the same order — which is why keeping it is cheap. Stated this plainly
     * because it was not: the suite's largest mutation count was measured against this function
     * and read for a while as though 24 tests were defending the shipping path. They defend this
     * one. The shipping path's own equivalent measures 1. See `MissionAdmissionTest`'s table.
     */
    fun admissible(items: List<StoredItem>): MissionRefusal? {
        items.forEachIndexed { index, item ->
            itemRefusal(item, index, items.size)?.let { return it }
        }
        return planRefusal(items)
    }

    // ------------------------------------------------------------------ the individual rules

    private fun frameRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val frame = item.frame
        if (frame in MissionFrames.RELATIVE) return null

        // MAV_FRAME_MISSION is legal, but only for an item whose x/y are not a coordinate.
        // Reading them would read a NaN-to-int32 cast; see MissionCommands.COORDINATE_BEARING.
        if (frame == MissionFrames.MISSION && item.command !in MissionCommands.COORDINATE_BEARING) {
            return null
        }

        val absolute = frame == MissionFrames.GLOBAL || frame == MissionFrames.GLOBAL_INT
        return MissionRefusal(
            MavMissionResult.MAV_MISSION_UNSUPPORTED_FRAME,
            if (absolute) MissionStatusTexts.ABSOLUTE_ALTITUDE
            else MissionStatusTexts.frameRefused(seq, frame),
            seq,
        )
    }

    /**
     * The three items whose position in the list is part of their meaning.
     *
     * `NAV_TAKEOFF` at seq 0 only — a takeoff in the middle of a plan is a plan whose author
     * believed something about the aircraft that is not true. `NAV_LAND` and
     * `NAV_RETURN_TO_LAUNCH` as the last item only, and `NAV_LOITER_UNLIM` likewise because it is
     * terminal by construction: an item after "hover until told otherwise" is unreachable, and an
     * unreachable item in a plan is a misunderstanding worth surfacing at the desk.
     *
     * All three terminal items are now the **same** behaviour — a hold — since M4-5 and M4-7
     * turned land and return into hovers (see [LegKind]). The structural rule survives that
     * change unaltered, which is the point of stating it positionally rather than by outcome.
     *
     * `MAV_MISSION_INVALID_SEQUENCE` is the right code here and it is safe *in this direction*:
     * §1.2 finding 6 is that QGC only tolerates that code from **ArduPilot**, so under our PX4
     * identity it is a hard failure — which is exactly what we mean. What it must never be used
     * for is a "please resend" nudge mid-transaction.
     */
    private fun structureRefusal(item: StoredItem, seq: Int, total: Int): MissionRefusal? {
        val last = total - 1
        return when (item.command) {
            MissionCommands.NAV_TAKEOFF ->
                if (seq != 0) refuse(seq, MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, "takeoff must be first")
                else null

            MissionCommands.NAV_LAND ->
                if (seq != last) refuse(seq, MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, "land must be last")
                else null

            MissionCommands.NAV_RETURN_TO_LAUNCH ->
                if (seq != last) refuse(seq, MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, "return must be last")
                else null

            MissionCommands.NAV_LOITER_UNLIM ->
                if (seq != last) refuse(seq, MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, "hover must be last")
                else null

            else -> null
        }
    }

    /**
     * A coordinate-bearing item must carry a coordinate this bridge is willing to repeat.
     *
     * The two exceptions are `NAV_TAKEOFF` and `NAV_LAND` with `x == y == 0`, which is QGC's
     * "here" — take off from where you are, land where you are. That pair is rejected by
     * [com.dimensional.mini4pro.telemetry.Geo] as the canonical unknown-encoded-as-zero, which is
     * correct for a *position report* and wrong for these two items, where it is a real and
     * common instruction. Every other coordinate item must resolve.
     */
    private fun coordinateRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        if (item.command !in MissionCommands.COORDINATE_BEARING) return null
        if (MissionGeo.pointOrNull(item) != null) return null
        val heresOk = item.command == MissionCommands.NAV_TAKEOFF ||
            item.command == MissionCommands.NAV_LAND
        if (heresOk && item.x == 0 && item.y == 0) return null
        return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM5_X, "not a coordinate")
    }

    /**
     * `0 < z ≤ CEILING_M`, in metres above home, for every item that commands a height.
     *
     * `NAV_LAND` is the one item allowed `z == 0`, and since M4-5 that is for a different reason
     * than it was: the altitude is **not commanded at all** (we hold, we do not descend), so
     * QGC's customary zero must not be refused. The band is still applied, because a `z` of 300
     * in a *relative* frame is the signature of an AMSL number that slipped through, and that is
     * a real signal even about a field we never fly to.
     *
     * `NAV_RETURN_TO_LAUNCH` and `NAV_DELAY` have no altitude by MAVLink's definition and are not
     * checked. An ROI's `z` is a *target* height rather than a commanded one, so it is required
     * to be finite and otherwise left alone — a target below home is a valley, not an error.
     */
    private fun altitudeRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val z = item.z.toDouble()
        when (item.command) {
            MissionCommands.NAV_WAYPOINT, MissionCommands.NAV_TAKEOFF,
            MissionCommands.NAV_LOITER_TIME, MissionCommands.NAV_LOITER_UNLIM,
            MissionCommands.DO_ORBIT,
            -> {
                if (!z.isFinite() || z <= 0.0 || z > CEILING_M) {
                    return MissionRefusal(
                        MavMissionResult.MAV_MISSION_INVALID_PARAM7,
                        MissionStatusTexts.altitudeRefused(seq, z, CEILING_M),
                        seq,
                    )
                }
            }

            MissionCommands.NAV_LAND -> {
                if (!z.isFinite() || z < 0.0 || z > CEILING_M) {
                    return MissionRefusal(
                        MavMissionResult.MAV_MISSION_INVALID_PARAM7,
                        MissionStatusTexts.altitudeRefused(seq, z, CEILING_M),
                        seq,
                    )
                }
            }

            MissionCommands.DO_SET_ROI_LOCATION, MissionCommands.DO_SET_ROI -> {
                if (!z.isFinite()) {
                    return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM7, "ROI height unusable")
                }
            }

            else -> Unit
        }
        return null
    }

    private fun parameterRefusal(item: StoredItem, seq: Int): MissionRefusal? = when (item.command) {
        MissionCommands.NAV_WAYPOINT -> waypointRefusal(item, seq)
        MissionCommands.NAV_TAKEOFF -> takeoffRefusal(item, seq)
        MissionCommands.NAV_LOITER_TIME, MissionCommands.NAV_LOITER_UNLIM ->
            loiterRefusal(item, seq)
        MissionCommands.NAV_DELAY -> delayRefusal(item, seq)
        MissionCommands.DO_CHANGE_SPEED -> speedRefusal(item, seq)
        MissionCommands.DO_ORBIT -> orbitRefusal(item, seq)
        MissionCommands.DO_MOUNT_CONTROL -> mountRefusal(item, seq)
        MissionCommands.DO_GIMBAL_MANAGER_PITCHYAW -> gimbalManagerRefusal(item, seq)
        else -> null
    }

    /**
     * `param1` hold ≥ 0; `param2` acceptance radius 0/NaN or in `[2, 10]` m; `param3` pass radius
     * must be 0 or NaN; `param4` yaw must be NaN.
     *
     * `param3` and `param4` are refused rather than ignored for the same reason M3 refuses a
     * finite `DO_REPOSITION.param4`: the engine pins yaw rate to zero (`docs/m3-stage-b.md` §4)
     * and does not fly a pass radius, so honouring neither while accepting both would be a plan
     * the operator believes will be flown as authored.
     */
    private fun waypointRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val hold = item.param1.toDouble()
        if (!hold.isFinite() || hold < 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM1, "hold time invalid")
        }
        val radius = item.param2.toDouble()
        val radiusUnset = !radius.isFinite() || radius == 0.0
        if (!radiusUnset && (radius < MIN_ACCEPT_RADIUS_M || radius > MAX_ACCEPT_RADIUS_M)) {
            return refuse(
                seq, MavMissionResult.MAV_MISSION_INVALID_PARAM2,
                "accept radius ${MIN_ACCEPT_RADIUS_M.toInt()}-${MAX_ACCEPT_RADIUS_M.toInt()}m",
            )
        }
        val pass = item.param3.toDouble()
        if (pass.isFinite() && pass != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM3, "no pass radius")
        }
        val yaw = item.param4.toDouble()
        if (yaw.isFinite()) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM4, "yaw is pinned to zero")
        }
        return null
    }

    /**
     * `NAV_TAKEOFF`: `param4` yaw must be **absent**.
     *
     * Settled 2026-07-27 by the review, and the gap it closed is the one this project names most
     * often: a takeoff carrying a commanded yaw uploaded silently and was flown as neither — not
     * with the yaw, because the guided engine pins yaw rate to zero (`docs/m3-stage-b.md` §4), and
     * not as a plain takeoff, because that is not what the operator authored. That is the
     * substitution `EMERGENCY_STOP_TEXT`'s reasoning forbids, and a `NAV_WAYPOINT` in the very
     * same plan already refused it. Two doors into the same engine cannot answer differently about
     * the same field.
     *
     * `param1` is fixed-wing minimum pitch and `param2`/`param3` are empty in MAVLink's own
     * definition, so nothing here reads them: they are fields with no meaning on this airframe
     * rather than instructions we are declining to follow.
     */
    private fun takeoffRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val yaw = item.param4.toDouble()
        if (yaw.isFinite()) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM4, "yaw is pinned to zero")
        }
        return null
    }

    /**
     * `NAV_LOITER_TIME` and `NAV_LOITER_UNLIM`, which are both flown as **hovers** (JC-8).
     *
     * `param4` yaw on the same grounds as [takeoffRefusal] and [waypointRefusal]. `param3` is the
     * loiter **radius**, and refusing a non-zero one is JC-8 applied at the door instead of only
     * in the projection: `MissionStore.kindOf` maps these to [LegKind.HOVER] and flies a point, so
     * accepting a radius of 40 would be accepting a circle and flying a dot — *"the operator's
     * plan says one thing and the aircraft does another, in the direction of more motion"*. Both
     * were unchecked until the review of 2026-07-27.
     *
     * `param1` is the hold time and is checked for `NAV_LOITER_TIME` only; for `NAV_LOITER_UNLIM`
     * MAVLink defines it as empty, and the hold is infinite by construction.
     */
    private fun loiterRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        if (item.command == MissionCommands.NAV_LOITER_TIME) {
            val seconds = item.param1.toDouble()
            if (!seconds.isFinite() || seconds < 0.0) {
                return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM1, "hover time invalid")
            }
        }
        val radius = item.param3.toDouble()
        if (radius.isFinite() && radius != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM3, "a hover has no radius")
        }
        val yaw = item.param4.toDouble()
        if (yaw.isFinite()) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM4, "yaw is pinned to zero")
        }
        return null
    }

    private fun delayRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val seconds = item.param1.toDouble()
        if (!seconds.isFinite() || seconds < 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM1, "delay invalid")
        }
        return null
    }

    /**
     * `DO_CHANGE_SPEED`: **we have no airspeed**, so `param1 = MAV_SPEED_TYPE_AIRSPEED` is
     * refused rather than quietly read as a ground speed. `param2` above the envelope is refused
     * and **never clamped** — M3 Q1's rule, and the sentence names the limit so the operator can
     * fix the plan rather than guess at it. A negative `param2` is MAVLink's "no change" and is
     * accepted as a no-op; zero is not, because a zero ground speed is a stop dressed as a speed.
     */
    private fun speedRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val type = item.param1.toDouble()
        if (type.isFinite() && type.toInt() == SpeedTypes.AIRSPEED) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM1, "no airspeed sensor")
        }
        val speed = item.param2.toDouble()
        if (!speed.isFinite() || speed < 0.0) return null
        if (speed == 0.0 || speed > MAX_SPEED_MPS) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_INVALID_PARAM2,
                MissionStatusTexts.speedRefused(seq, speed, MAX_SPEED_MPS),
                seq,
            )
        }
        return null
    }

    /**
     * `DO_ORBIT`: radius is **signed** (QGC encodes direction as the sign), so the magnitude is
     * what is bounded. `param2` NaN means "vehicle default"; a finite value above the envelope is
     * refused. `param3` is an `ORBIT_YAW_BEHAVIOUR` **integer** — see [OrbitYawBehaviour] for why
     * it must never be an enum lookup.
     *
     * The two nose-pointing behaviours (`HOLD_FRONT_TO_CIRCLE_CENTER`, `HOLD_FRONT_TANGENT`) are
     * refused rather than silently downgraded to "hold heading", because a downgrade is a command
     * the operator believes was obeyed. That is the conflict between the two M4 designs and it is
     * Ivan's to settle (transport §5.4 / execution §8.5); until it is, they refuse.
     */
    private fun orbitRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val radius = kotlin.math.abs(item.param1.toDouble())
        if (!radius.isFinite() || radius < ORBIT_MIN_RADIUS_M || radius > ORBIT_MAX_RADIUS_M) {
            return refuse(
                seq, MavMissionResult.MAV_MISSION_INVALID_PARAM1,
                "orbit r ${ORBIT_MIN_RADIUS_M.toInt()}-${ORBIT_MAX_RADIUS_M.toInt()}m",
            )
        }
        val velocity = item.param2.toDouble()
        if (velocity.isFinite() && (velocity <= 0.0 || velocity > MAX_SPEED_MPS)) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_INVALID_PARAM2,
                MissionStatusTexts.speedRefused(seq, velocity, MAX_SPEED_MPS),
                seq,
            )
        }
        val yawBehaviour = item.param3.toDouble()
        val behaviour = if (yawBehaviour.isFinite()) yawBehaviour.toInt() else OrbitYawBehaviour.UNCHANGED
        val honourable = behaviour == OrbitYawBehaviour.HOLD_INITIAL_HEADING ||
            behaviour == OrbitYawBehaviour.UNCONTROLLED ||
            behaviour == OrbitYawBehaviour.RC_CONTROLLED ||
            behaviour == OrbitYawBehaviour.UNCHANGED
        if (!honourable) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM3, "orbit yaw not flown")
        }
        return null
    }

    /**
     * `DO_MOUNT_CONTROL`: pitch only. **The airframe has no gimbal yaw** — DJI refuses it with
     * error #527 (`docs/gimbal.md`, "The #527 problem") — and it has no gimbal roll either, so a
     * non-zero `param2`/`param3` is a plan whose camera move will not happen.
     */
    private fun mountRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val roll = item.param2.toDouble()
        if (roll.isFinite() && roll != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM2, "gimbal roll refused by DJI")
        }
        val yaw = item.param3.toDouble()
        if (yaw.isFinite() && yaw != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM3, "gimbal yaw refused by DJI")
        }
        val mode = item.z.toDouble()
        if (!mode.isFinite() || mode.toInt() != MOUNT_MODE_MAVLINK_TARGETING) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM7, "mount mode not offered")
        }
        return null
    }

    /**
     * `DO_GIMBAL_MANAGER_PITCHYAW`: pitch only, on the same #527 grounds as
     * [mountRefusal] — but **the field layout is different** and the transport design's "as 205"
     * shorthand cannot be taken literally. Here `param1` is pitch angle, `param2` is **yaw
     * angle**, `param3` is pitch rate and `param4` is **yaw rate**, so the two fields that must
     * be absent are 2 and 4 rather than 2 and 3.
     */
    private fun gimbalManagerRefusal(item: StoredItem, seq: Int): MissionRefusal? {
        val yawAngle = item.param2.toDouble()
        if (yawAngle.isFinite() && yawAngle != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM2, "gimbal yaw refused by DJI")
        }
        val yawRate = item.param4.toDouble()
        if (yawRate.isFinite() && yawRate != 0.0) {
            return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_PARAM4, "gimbal yaw refused by DJI")
        }
        return null
    }

    /** Takeoff and land are phases, not positions: two of either is not a plan. */
    private fun uniquenessRefusal(items: List<StoredItem>): MissionRefusal? {
        val duplicate = listOf(
            MissionCommands.NAV_TAKEOFF to "one takeoff only",
            MissionCommands.NAV_LAND to "one landing only",
            MissionCommands.NAV_RETURN_TO_LAUNCH to "one return only",
        ).firstOrNull { (command, _) -> items.count { it.command == command } > 1 }
            ?: return null
        val seq = items.last { it.command == duplicate.first }.seq
        return refuse(seq, MavMissionResult.MAV_MISSION_INVALID_SEQUENCE, duplicate.second)
    }

    /**
     * Leg lengths and the whole path's extent, over the navigable items that carry a coordinate.
     *
     * A zero-length leg is refused because it has **no direction**, and the execution half's
     * pass-through test needs one: its half-plane term is defined against the unit vector along
     * the leg, which does not exist for two identical points.
     */
    private fun geometryRefusal(items: List<StoredItem>): MissionRefusal? {
        val points = items
            .filter { it.command in MissionCommands.NAVIGABLE }
            .mapNotNull { item -> MissionGeo.pointOrNull(item)?.let { item.seq to it } }

        var total = 0.0
        for (index in 1 until points.size) {
            val (seq, here) = points[index]
            val previous = points[index - 1].second
            val leg = MissionGeo.distanceM(previous, here)
            if (leg == 0.0) {
                return refuse(seq, MavMissionResult.MAV_MISSION_ERROR, "zero-length leg")
            }
            if (leg > MAX_LEG_M) {
                return MissionRefusal(
                    MavMissionResult.MAV_MISSION_ERROR,
                    MissionStatusTexts.legTooLong(seq, leg, MAX_LEG_M),
                    seq,
                )
            }
            total += leg
        }
        if (total > MAX_TOTAL_M) {
            return MissionRefusal(
                MavMissionResult.MAV_MISSION_ERROR,
                MissionStatusTexts.planTooFar(total, MAX_TOTAL_M),
                seq = null,
            )
        }
        return null
    }

    private fun refuse(seq: Int, result: MavMissionResult, reason: String) =
        MissionRefusal(result, MissionStatusTexts.itemRefused(seq, reason), seq)
}
