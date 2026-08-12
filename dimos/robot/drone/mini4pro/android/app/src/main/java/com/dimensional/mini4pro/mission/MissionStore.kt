package com.dimensional.mini4pro.mission

import java.util.concurrent.atomic.AtomicReference

/**
 * What kind of thing a [ResolvedLeg] is. One value per navigable command we accept, so a
 * `when` over it breaks loudly the day the vocabulary grows.
 *
 * **[RTL] does not descend, and [LAND] descends only when the plan asks it to.**
 * `docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-5 half-reversed the ground-to-ground
 * answer: takeoff stays ours, landing does not. Ivan's words were *"just hover for now, I'll land
 * manually"*. `NAV_RETURN_TO_LAUNCH` (M4-7) still means exactly that — **fly home and hold**, a
 * hover at home, never DJI's own RTH, which has its own altitude, its own path and its own descent
 * and which we do not sequence.
 *
 * **[LAND] is now two behaviours, chosen by the item's own `param2`** ([PrecisionLandMode], 2026-07-30):
 *
 *  - `param2 = 0` (QGC's "Precision Land: Disabled", and its default) — **the M4-5 hold, verbatim**:
 *    fly to that point, come to rest, hold, in the air. The altitude it names is not a height we fly
 *    to; see [MissionStore.relativeAltOf].
 *  - `param2 >= 1` (Opportunistic / Required) — **the tag landing**: the item's altitude becomes an
 *    ordinary waypoint altitude, the leg goes to the *recorded takeoff point*, and the existing tag
 *    descent lands the aircraft through the arm door the phone's own arm uses
 *    (`guided/PrecisionLand`). Autonomous descent is still not something this executor implements —
 *    it is something it *calls*, through gates it does not weaken.
 *
 * What has not changed is why the distinction is drawn at all: autonomous descent is the one
 * capability where a bug cannot be undone by climbing. What changed is that this project now has a
 * measured descent-and-landing machine with its own arm gates, its own ladder and its own flight
 * record — so the honest default is a hold and the honest opt-in is a field the operator sets on the
 * item itself. The MAVLink command names are kept because they are what QGC sends.
 */
enum class LegKind { WAYPOINT, TAKEOFF, LAND, RTL, HOVER, ORBIT, DELAY }

/**
 * The place the camera is being pointed at, as resolved from the sticky `DO_SET_ROI*` items.
 *
 * [relativeAltM] is null for the Fly-view door, where QGC's `z` comes from **its own terrain
 * database** and is therefore in a datum we cannot check (JC-9 — the number is discarded, not
 * trusted, and the assumption is announced). A plan item in `MAV_FRAME_GLOBAL_RELATIVE_ALT`
 * carries a height in *our* frame, and there it is kept.
 */
data class RoiTarget(val target: GeoPoint, val relativeAltM: Double?)

/**
 * A `DO_ORBIT` (34), validated and projected.
 *
 * @param radiusM **signed** — QGC encodes direction as the sign of `param1`
 *   (`orbitMapCircle.radius() * (clockwiseRotation ? 1 : -1)`, `GuidedActionsController.qml:641`).
 *   Negative is counter-clockwise. Not measured; §9.9.
 * @param velocityMps null when QGC sent NaN, which means "the vehicle's own default".
 * @param yawBehaviour an `ORBIT_YAW_BEHAVIOUR` **integer**, never an enum — see
 *   [OrbitYawBehaviour]. QGC sends 5, a value absent from our dialect.
 * @param turns null when QGC sent NaN, which means "until stopped".
 */
data class OrbitSpec(
    val radiusM: Double,
    val velocityMps: Double?,
    val yawBehaviour: Int,
    val turns: Double?,
)

/**
 * One navigable item with every sticky value already resolved — the projection the execution
 * half actually consumes.
 *
 * Two properties this type is shaped to buy, both pinned by tests:
 *
 *  1. **The sticky resolution has exactly one implementation.** `DO_CHANGE_SPEED` and
 *     `DO_SET_ROI_*` set state that applies to every following leg until changed. Making the
 *     executor re-scan backwards from each waypoint is the kind of thing that is correct in
 *     review and wrong at item 14, so it is done once, here, at commit.
 *  2. **It cannot express an absolute altitude.** There is no `amslM` field, because §4.2
 *     refused every absolute frame at the door. The type carries the rule.
 *
 * [seq] is the item's **wire** sequence number (§4.1: QGC deletes the planned-home marker and
 * renumbers down by one for PX4, and adds the 1 back for display). Note this list is the
 * *navigable subset* of the plan, so the index into [MissionPlan.legs] is **not** [seq] — see
 * that property's KDoc for why the design document's shorthand cannot be taken literally.
 */
data class ResolvedLeg(
    val seq: Int,
    val kind: LegKind,
    /** Absolute lat/lon, or null for an item that has no place of its own (`RTL`, `DELAY`). */
    val target: GeoPoint?,
    /** Metres above home-at-upload. **Always** relative, by construction (§4.2). */
    val relativeAltM: Double?,
    val holdSeconds: Double,
    /** `param2` when the operator asked for one; null means "use the executor's own". */
    val acceptRadiusM: Double?,
    /** Sticky: the `DO_CHANGE_SPEED` in force at this leg, or null for the executor's own. */
    val speedLimitMps: Double?,
    /** Sticky: the ROI in force at this leg; null after a `DO_SET_ROI_NONE` or before any ROI. */
    val roi: RoiTarget?,
    /** Present only for [LegKind.ORBIT]. */
    val orbit: OrbitSpec?,
    /**
     * `NAV_LAND.param2` as a [PrecisionLandMode], and **null for every other item** — a leg that is
     * not a landing has no opinion about precision landing, and a 0 there would be an opinion.
     *
     * Parsed rather than passed through: the raw float is still in [StoredItem.param2] for the
     * read-back, and this is the projection's answer to *"does this plan ask us to land on the
     * tag?"*. One reader ([MissionLaunch.routeOf]), one meaning.
     */
    val precisionLandMode: Int? = null,
)

/**
 * An immutable snapshot of a committed plan. A new upload yields a new object; nothing here is
 * ever mutated in place, which is what makes the executor's 10 Hz read safe against the
 * receive thread's commit.
 */
interface MissionPlan {
    /**
     * Monotonic, bumped on **every** commit and every clear. The execution design calls this
     * `generation`; the names are reconciled in the transport design's §8 note. A paused cursor
     * is only meaningful against the plan it was paused in, so a resume across a change of this
     * number is refused by the execution half.
     */
    val planId: Int

    /** Every item, verbatim, index == wire `seq`, contiguous from 0. */
    val items: List<StoredItem>

    /**
     * The navigable items with sticky state resolved, in `seq` order.
     *
     * **Not indexed by `seq`,** despite the design document's shorthand in §8. It cannot be: a
     * plan contains `DO_CHANGE_SPEED` and `DO_SET_ROI_*` items which occupy sequence numbers and
     * are not legs, and [LegKind] has no value that could honestly describe them. Each leg
     * carries its own [ResolvedLeg.seq]; [legAt] is the lookup.
     */
    val legs: List<ResolvedLeg>

    /**
     * Where DJI said home was at the instant of commit, or null if home was unknown then.
     * **Provenance, never a datum.** The execution half uses it to decide whether a home that
     * has moved invalidates the plan's altitudes (§3.2, Q4); transport only records it.
     */
    val homeAtUpload: GeoPoint?

    /** `TelemetryEncoder.amslMetres` at commit. Provenance, on the same terms as [homeAtUpload]. */
    val amslDatumAtUpload: Double?

    val uploadedAtMs: Long

    val itemCount: Int get() = items.size

    /** The resolved leg for a wire sequence number, or null when that item is not navigable. */
    fun legAt(seq: Int): ResolvedLeg? = legs.firstOrNull { it.seq == seq }
}

/** What the execution half reads. One method, because a snapshot is the whole contract. */
interface MissionSource {
    /** Null when no plan is committed. Immutable; a new upload yields a new object. */
    fun plan(): MissionPlan?
}

private class CommittedPlan(
    override val planId: Int,
    override val items: List<StoredItem>,
    override val legs: List<ResolvedLeg>,
    override val homeAtUpload: GeoPoint?,
    override val amslDatumAtUpload: Double?,
    override val uploadedAtMs: Long,
) : MissionPlan

/**
 * The truth about what plan this bridge holds. **The bridge is the mission; the aircraft never
 * holds our plan** (`docs/m4-mission-transport.md` §0), so a read-back is a read of our own
 * store and is exact rather than a re-derivation.
 *
 * ## Atomicity is a property here, not a policy
 *
 * The whole snapshot is swapped in one [AtomicReference.set]. There is no instant at which a
 * reader can see half a plan, because there is no operation that writes half of one — an upload
 * accumulates into [MissionTransaction]'s own pending buffer and reaches this object only as a
 * finished list. The execution half may be reading on its own 10 Hz thread, and a half-written
 * plan is a plan with a discontinuity in it that an aircraft would fly. "Refuse to start" is a
 * policy; "cannot be in that state" is a property, and this layer can afford the property.
 *
 * ## Never persisted, and that is load-bearing (JC-5)
 *
 * This class touches no file, no `SharedPreferences`, no `Context` — exactly as
 * [com.dimensional.mini4pro.command.CommandInterlock] holds no storage. A restored plan is a
 * claim about a session that has ended: a process restart drops virtual-stick authority, so the
 * aircraft has already handed back, and the plan could not be flown without re-validation
 * anyway. Persisting would buy one thing — QGC reading back a plan we are not flying and cannot
 * start — which is the shape of every bug the honesty boundaries exist to prevent. A fresh
 * process answers `MISSION_COUNT 0`, which is true.
 *
 * It **does** outlive a link, though, and that is deliberate: QGC re-reads the plan on every
 * connect (§1.4), so a store that died with the socket would show an empty Plan view for an
 * aircraft that is flying one. Owned at service scope, like the interlock.
 *
 * Thread safety: committed and read from the `mavlink-rx` thread, read from the executor's
 * 10 Hz thread and from `Bridge.tick`. One atomic reference is the whole mechanism.
 */
class MissionStore(
    /** Trace hook; keeps `android.util.Log` out of a JVM-testable layer. */
    private val log: (String) -> Unit = {},
    /**
     * **The change notification the execution half requires** (`docs/m4-mission-execution.md` §2.1
     * rule 5): *"the executor must be told, **synchronously**, when the stored mission changes, so a
     * `PAUSED` cursor can be dropped on the same edge rather than discovered later."*
     *
     * Called with the new plan after a [commit] and with null after a [clear], on whichever thread
     * did the writing — the `mavlink-rx` thread in practice. Placed on the *store* rather than on
     * `MissionTransaction` deliberately: every write goes through here, so there is no second door
     * through which a plan can change unannounced.
     *
     * Must not block. The caller is holding a protocol whose retry window is 250 ms.
     */
    private val onChanged: (MissionPlan?) -> Unit = {},
) : MissionSource {

    private val current = AtomicReference<MissionPlan?>(null)

    /**
     * Bumped on every commit **and every clear**, so "the plan changed" is one comparison for the
     * execution half. Starts at 0; the first commit yields 1, and 0 therefore never names a plan.
     */
    private val generation = java.util.concurrent.atomic.AtomicInteger(0)

    override fun plan(): MissionPlan? = current.get()

    /** The committed items, or an empty list. What `MISSION_COUNT` and the read-back serve. */
    val items: List<StoredItem> get() = current.get()?.items ?: emptyList()

    val count: Int get() = items.size

    val isEmpty: Boolean get() = count == 0

    /** The current plan id, or 0 when nothing is committed. */
    val planId: Int get() = generation.get()

    /**
     * Replaces the plan, atomically. The caller has already validated [items] — this method
     * enforces only the structural invariant the executor's cursor depends on, because a gap
     * would make "advance" undefined.
     */
    fun commit(
        items: List<StoredItem>,
        homeAtUpload: GeoPoint?,
        amslDatumAtUpload: Double?,
        uploadedAtMs: Long,
    ): MissionPlan {
        require(items.mapIndexed { index, item -> item.seq == index }.all { it }) {
            "mission items must be contiguous from seq 0; got ${items.map { it.seq }}"
        }
        val plan = CommittedPlan(
            planId = generation.incrementAndGet(),
            items = items.toList(),
            legs = resolve(items),
            homeAtUpload = homeAtUpload,
            amslDatumAtUpload = amslDatumAtUpload,
            uploadedAtMs = uploadedAtMs,
        )
        current.set(plan)
        log("mission committed: ${items.size} items, ${plan.legs.size} legs, planId ${plan.planId}")
        // After the swap, never before: a listener that reads back must see the plan it was told
        // about, and the executor's paused-cursor drop happens on this edge by construction.
        onChanged(plan)
        return plan
    }

    /** Drops the plan and bumps [planId]. What `MISSION_CLEAR_ALL` does. */
    fun clear() {
        generation.incrementAndGet()
        current.set(null)
        log("mission cleared, planId ${generation.get()}")
        onChanged(null)
    }

    companion object {

        /**
         * The sticky resolution, and the only implementation of it in the codebase.
         *
         * Walks the plan once in `seq` order carrying two pieces of state — the speed limit in
         * force and the ROI in force — and emits one [ResolvedLeg] per navigable item with the
         * values *as they stand at that item*. An item that sets state produces no leg.
         *
         * `DO_SET_ROI_NONE` clears the ROI rather than leaving the previous one standing, which
         * is the one asymmetry worth naming: "point at nothing" is a state, not the absence of a
         * command.
         */
        fun resolve(items: List<StoredItem>): List<ResolvedLeg> {
            var speed: Double? = null
            var roi: RoiTarget? = null
            val legs = ArrayList<ResolvedLeg>(items.size)

            for (item in items) {
                when (item.command) {
                    MissionCommands.DO_CHANGE_SPEED -> {
                        // param2 < 0 or NaN is MAVLink's "no change"; admission has already
                        // refused a positive value outside the envelope and an airspeed request.
                        val requested = item.param2.toDouble()
                        if (requested.isFinite() && requested > 0.0) speed = requested
                    }

                    MissionCommands.DO_SET_ROI_LOCATION, MissionCommands.DO_SET_ROI -> {
                        val point = MissionGeo.pointOrNull(item)
                        if (point != null) {
                            roi = RoiTarget(
                                target = point,
                                relativeAltM = item.z.toDouble().takeIf { it.isFinite() },
                            )
                        }
                    }

                    MissionCommands.DO_SET_ROI_NONE -> roi = null

                    else -> Unit
                }

                val kind = kindOf(item.command) ?: continue
                legs.add(
                    ResolvedLeg(
                        seq = item.seq,
                        kind = kind,
                        target = MissionGeo.pointOrNull(item),
                        relativeAltM = relativeAltOf(item),
                        holdSeconds = holdOf(item),
                        acceptRadiusM = acceptRadiusOf(item),
                        speedLimitMps = speed,
                        roi = roi,
                        orbit = orbitOf(item),
                        precisionLandMode = precisionLandOf(item),
                    )
                )
            }
            return legs
        }

        private fun kindOf(command: Int): LegKind? = when (command) {
            MissionCommands.NAV_WAYPOINT -> LegKind.WAYPOINT
            MissionCommands.NAV_TAKEOFF -> LegKind.TAKEOFF
            // Neither of these descends — see [LegKind]. They are terminal *holds*.
            MissionCommands.NAV_LAND -> LegKind.LAND
            MissionCommands.NAV_RETURN_TO_LAUNCH -> LegKind.RTL
            // JC-8: loiter items are flown as **hovers**, never as circles. On a multirotor
            // `param3`'s radius is a fixed-wing concept and QGC's own Plan view draws these as a
            // point. Flying a circle where the plan says hover is the substitution
            // `EMERGENCY_STOP_TEXT`'s reasoning forbids — the operator's plan says one thing and
            // the aircraft does another, in the direction of *more* motion.
            MissionCommands.NAV_LOITER_TIME, MissionCommands.NAV_LOITER_UNLIM -> LegKind.HOVER
            MissionCommands.NAV_DELAY -> LegKind.DELAY
            MissionCommands.DO_ORBIT -> LegKind.ORBIT
            else -> null
        }

        /**
         * `NAV_LOITER_UNLIM` is terminal by construction, so its hold is infinite rather than
         * zero. Everything else reads the field MAVLink defines as a hold time.
         */
        private fun holdOf(item: StoredItem): Double = when (item.command) {
            MissionCommands.NAV_WAYPOINT, MissionCommands.NAV_LOITER_TIME,
            MissionCommands.NAV_DELAY,
            -> item.param1.toDouble().takeIf { it.isFinite() && it > 0.0 } ?: 0.0

            MissionCommands.NAV_LOITER_UNLIM -> Double.POSITIVE_INFINITY
            else -> 0.0
        }

        /**
         * Null for an item whose `z` is not a height **this bridge commands**.
         *
         * `RTL` and `DELAY` have no altitude of their own by MAVLink's definition, and a
         * `MAV_FRAME_MISSION` item's `z` is not an altitude at all.
         *
         * **`NAV_LAND` is the interesting one, and since 2026-07-30 it splits on its own `param2`:**
         *
         *  - **precision landing Disabled** (`param2 = 0`, QGC's default) — null, exactly as M4-5
         *    left it. The item does not descend, so the height it names is not a height we fly to;
         *    the aircraft holds whatever altitude it arrives with. Returning it here would be the
         *    projection quietly promising a descent, and every plan authored before this feature
         *    carries that zero.
         *  - **precision landing Opportunistic or Required** — the height **is** returned, because
         *    for the tag-landing sequence the item's altitude is an ordinary waypoint altitude that
         *    the aircraft flies to (Ivan, 2026-07-30; `guided/PrecisionLand`, and
         *    [MissionLaunch.routeOf] is where it becomes a step). `MissionAdmission` has already
         *    checked it against the M3 ceiling, on the same band as every other height in a plan.
         *
         * The raw `z` is in the store either way, verbatim, because the read-back must return what
         * the operator authored.
         */
        private fun relativeAltOf(item: StoredItem): Double? {
            if (item.frame !in MissionFrames.RELATIVE) return null
            if (item.command == MissionCommands.NAV_RETURN_TO_LAUNCH) return null
            if (item.command == MissionCommands.NAV_DELAY) return null
            if (item.command == MissionCommands.NAV_LAND &&
                !PrecisionLandMode.landsOnTag(PrecisionLandMode.of(item.param2))
            ) {
                return null
            }
            return item.z.toDouble().takeIf { it.isFinite() }
        }

        /**
         * `NAV_LAND.param2` as a [PrecisionLandMode], and null for everything else — the parse
         * [ResolvedLeg.precisionLandMode] carries and the *only* place this field is interpreted.
         */
        private fun precisionLandOf(item: StoredItem): Int? {
            if (item.command != MissionCommands.NAV_LAND) return null
            return PrecisionLandMode.of(item.param2)
        }

        /** `NAV_WAYPOINT.param2`, when the operator asked for one. 0 and NaN mean "yours". */
        private fun acceptRadiusOf(item: StoredItem): Double? {
            if (item.command != MissionCommands.NAV_WAYPOINT) return null
            val radius = item.param2.toDouble()
            return radius.takeIf { it.isFinite() && it > 0.0 }
        }

        private fun orbitOf(item: StoredItem): OrbitSpec? {
            if (item.command != MissionCommands.DO_ORBIT) return null
            return OrbitSpec(
                radiusM = item.param1.toDouble(),
                velocityMps = item.param2.toDouble().takeIf { it.isFinite() },
                // Integer, never an enum lookup: QGC sends 5, which our dialect does not have.
                yawBehaviour = item.param3.toDouble()
                    .takeIf { it.isFinite() }?.toInt() ?: OrbitYawBehaviour.UNCHANGED,
                turns = item.param4.toDouble().takeIf { it.isFinite() },
            )
        }
    }
}
