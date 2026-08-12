package com.dimensional.mini4pro.mission

/**
 * The wire vocabulary of a mission item, and the one item type this bridge stores.
 *
 * Pure Kotlin: no DJI, no Android, no `io.dronefleet.mavlink` types. The MAVLink enums are
 * spelled as **integers** here for the reason `HandshakeResponder` already spells command ids as
 * integers — a dialect that predates a constant still compiles, and a constant that acquires a
 * different value later cannot silently change what we accept. `docs/m4-mission-transport.md`
 * §5.2 has the worked example: `ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5` exists in QGroundControl's
 * dialect and **not** in `io.dronefleet.mavlink` 1.1.11's, so an enum lookup for it would either
 * fail or resolve to whatever 5 comes to mean.
 */

/** `MAV_CMD` ids that can appear in a plan. Integers, never enum entries — see the file KDoc. */
object MissionCommands {
    const val NAV_WAYPOINT = 16
    const val NAV_LOITER_UNLIM = 17
    const val NAV_LOITER_TURNS = 18
    const val NAV_LOITER_TIME = 19
    const val NAV_RETURN_TO_LAUNCH = 20
    const val NAV_LAND = 21
    const val NAV_TAKEOFF = 22
    const val NAV_LOITER_TO_ALT = 31
    const val DO_ORBIT = 34
    const val NAV_DELAY = 93
    const val CONDITION_YAW = 115
    const val DO_JUMP = 177
    const val DO_CHANGE_SPEED = 178
    const val DO_SET_HOME = 179
    const val DO_SET_SERVO = 183
    const val DO_SET_ACTUATOR = 187
    const val DO_LAND_START = 189
    const val DO_SET_ROI = 201
    const val DO_DIGICAM_CONTROL = 203
    const val DO_MOUNT_CONFIGURE = 204
    const val DO_MOUNT_CONTROL = 205
    const val DO_SET_CAM_TRIGG_DIST = 206
    const val DO_GRIPPER = 211

    /**
     * `MAV_CMD_DO_SET_MISSION_CURRENT`. Accepted as a *command* (§1.6) and routed to the
     * executor, which refuses it — the point being that a `COMMAND_ACK` is a channel through
     * which a refusal can be heard, and QGC caches our first answer for the lifetime of the
     * vehicle instance. Answering `UNSUPPORTED` once would make every later press fall back to
     * the deprecated `MISSION_SET_CURRENT` message (#41), which has no reply at all.
     */
    const val DO_SET_MISSION_CURRENT = 224

    const val DO_SET_ROI_LOCATION = 195
    const val DO_SET_ROI_WPNEXT_OFFSET = 196
    const val DO_SET_ROI_NONE = 197
    const val CONDITION_GATE = 4501
    const val DO_GIMBAL_MANAGER_PITCHYAW = 1000
    const val SET_CAMERA_MODE = 530
    const val IMAGE_START_CAPTURE = 2000
    const val IMAGE_STOP_CAPTURE = 2001
    const val VIDEO_START_CAPTURE = 2500
    const val VIDEO_STOP_CAPTURE = 2501

    /**
     * The commands that put the aircraft somewhere — the ones a [ResolvedLeg] is produced for.
     * Everything else in an accepted plan is sticky state or a pure sequencer wait.
     */
    val NAVIGABLE: Set<Int> = setOf(
        NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND, NAV_RETURN_TO_LAUNCH,
        NAV_LOITER_TIME, NAV_LOITER_UNLIM, NAV_DELAY, DO_ORBIT,
    )

    /**
     * The commands whose `x`/`y`/`z` are a real coordinate. Everything else must never have its
     * `x`/`y` read: §4.2's trap is that QGC does **not** scale `x`/`y` by 1e7 under
     * `MAV_FRAME_MISSION`, and survey-generated items carry `param5/6/7 = NaN`, so `x`/`y` are
     * whatever a `double NaN → int32` cast produced.
     */
    val COORDINATE_BEARING: Set<Int> = setOf(
        NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND, NAV_LOITER_TIME, NAV_LOITER_UNLIM,
        DO_ORBIT, DO_SET_ROI_LOCATION, DO_SET_ROI,
    )
}

/** `MAV_FRAME` ids. See [MissionAdmission] for which are accepted and the datum argument. */
object MissionFrames {
    const val GLOBAL = 0
    const val MISSION = 2
    const val GLOBAL_RELATIVE_ALT = 3
    const val GLOBAL_INT = 5
    const val GLOBAL_RELATIVE_ALT_INT = 6
    const val GLOBAL_TERRAIN_ALT = 10
    const val GLOBAL_TERRAIN_ALT_INT = 11

    /** `z` is metres above home, with no datum to get wrong. The only two we accept. */
    val RELATIVE: Set<Int> = setOf(GLOBAL_RELATIVE_ALT, GLOBAL_RELATIVE_ALT_INT)
}

/** `MAV_MISSION_TYPE`. Only [MISSION] is ever stored; see `docs/m4-mission-execution.md` §2.1. */
object MissionTypes {
    const val MISSION = 0
    const val FENCE = 1
    const val RALLY = 2
}

/**
 * `ORBIT_YAW_BEHAVIOUR`, as integers **on purpose**.
 *
 * `ref/mavlink/definition-xml/common.xml:811-827` — the XML `io.dronefleet.mavlink` 1.1.11 was
 * generated from — stops at 4. **`UNCHANGED = 5` is absent from our dialect** and exists only in
 * QGroundControl's newer definitions (`_deps/mavlink-build/.../common.h:515`), which is precisely
 * the value QGC's Orbit button sends (measured, `HandshakeResponder.kt:140`). An enum lookup for
 * it would fail today or, worse, resolve to whatever 5 acquires when a dialect is regenerated.
 * So: named integer constants, and never `EnumValue`.
 */
object OrbitYawBehaviour {
    const val HOLD_FRONT_TO_CIRCLE_CENTER = 0
    const val HOLD_INITIAL_HEADING = 1
    const val UNCONTROLLED = 2
    const val HOLD_FRONT_TANGENT_TO_CIRCLE = 3
    const val RC_CONTROLLED = 4

    /** **Not in our MAVLink dialect.** QGC's default and the only value ever measured. */
    const val UNCHANGED = 5
}

/**
 * `PRECISION_LAND_MODE` — **`MAV_CMD_NAV_LAND.param2`**, and the field that decides whether a
 * plan's Land item is yesterday's terminal hold or the tag landing.
 *
 * Integers, for this file's standing reason, and the values are MAVLink's own
 * (`ref/mavlink/definition-xml/common.xml:4036-4047`, enum `PRECISION_LAND_MODE`, referenced from
 * `MAV_CMD_NAV_LAND`'s `param index="2" label="Land Mode"`). QGC exposes them on the Land item's
 * own editor as **"Precision Land": Disabled / Opportunistic / Required**, default 0
 * (`ref/qgroundcontrol/src/MissionManager/MavCmdInfoCommon.json:203-209` — `enumStrings`
 * `"Disabled,Opportunistic,Required"`, `enumValues` `"0,1,2"`), so this is a field an operator can
 * already set on an existing plan with no new dialect, no custom command and nothing advertised.
 *
 * ## What each value means *here*
 *
 *  - **[DISABLED] (0) — today's documented behaviour, unchanged.** Fly to the item, come to rest,
 *    hold, in the air (M4-5's *"just hover for now, I'll land manually"*). This is the value QGC
 *    writes by default, which is what makes every plan authored before this feature still mean
 *    exactly what it meant: **a plan that does not ask to land does not land.** That property is
 *    forward-compatibility in the only direction that matters and it has its own mutation row.
 *  - **[REQUIRED] (2) — the tag landing**: gates, the leg to the recorded takeoff point, the
 *    lowering to the arm height, and the existing tag descent armed for full autoland.
 *  - **[OPPORTUNISTIC] (1) — implemented identically to [REQUIRED] for now**, and the difference is
 *    named on the flight record rather than in behaviour. MAVLink's own definition of 1 is *"use
 *    precision landing if beacon detected …, otherwise land normally"*, and **"otherwise land
 *    normally" is a landing nobody has decided to build**: this bridge's only non-tag landing is
 *    DJI's own, `landingdata.md` §4 requires a deliberate decision before any of it runs, and Ivan
 *    has not taken that one. Inventing it here would be exactly the substitution
 *    `EMERGENCY_STOP_TEXT`'s reasoning forbids. So 1 behaves as 2 — refusing by name rather than
 *    falling back — until there is an answer to fall back *to*.
 */
object PrecisionLandMode {

    /** Normal (non-precision) landing — for this bridge, the terminal hold M4-5 chose. */
    const val DISABLED = 0

    /** *"Use precision landing if beacon detected"*. Implemented as [REQUIRED]; see the object KDoc. */
    const val OPPORTUNISTIC = 1

    /** *"Use precision landing, searching for beacon"*. The tag landing. */
    const val REQUIRED = 2

    /**
     * The mode a stored `NAV_LAND` asks for, as an integer, or [DISABLED] when its `param2` is not
     * a mode we recognise.
     *
     * **Non-finite, negative and unknown values all read as [DISABLED]**, deliberately: an
     * unrecognised number in this field is a plan whose author asked for something this bridge
     * cannot vouch for, and the fail-closed answer to *"shall I land autonomously?"* is no. Note the
     * asymmetry with `MissionAdmission`'s usual habit of refusing an out-of-band parameter at the
     * desk — refusing the *upload* would make a plan carrying a mode we do not know unflyable even
     * as a hold, which is worse for the operator and no safer, because the hold is the safe
     * behaviour anyway. Rounded rather than truncated: QGC writes this field as a float with two
     * decimal places, so `1.9999998` is a 2 that survived a round trip.
     */
    fun of(param2: Float): Int {
        val value = param2.toDouble()
        if (!value.isFinite()) return DISABLED
        return when (Math.round(value).toInt()) {
            OPPORTUNISTIC -> OPPORTUNISTIC
            REQUIRED -> REQUIRED
            else -> DISABLED
        }
    }

    /** True when [mode] asks this bridge to land on the tag — the one place `>= 1` is spelled. */
    fun landsOnTag(mode: Int): Boolean = mode == OPPORTUNISTIC || mode == REQUIRED
}

/** `MAV_SPEED_TYPE`, for `DO_CHANGE_SPEED.param1`. */
object SpeedTypes {
    const val AIRSPEED = 0
    const val GROUNDSPEED = 1
}

/**
 * A geodetic point in degrees. Only ever constructed through [MissionGeo.pointOrNull], so an
 * instance is a coordinate this bridge has already agreed to repeat.
 */
data class GeoPoint(val latDeg: Double, val lonDeg: Double)

/**
 * One `MISSION_ITEM_INT` **exactly as it came off the wire**: same field names, same units, same
 * types, nothing parsed, nothing normalised, nothing converted.
 *
 * That is the whole point, and it is the read-back that forces it. `MISSION_REQUEST_INT(seq)`
 * must be answered with the numbers QGC gave us, or the plan an operator sees after a reconnect
 * is a paraphrase of the plan they authored. Every interpretation happens on the way *out* of the
 * store, into [ResolvedLeg], and that projection is never what goes on the wire.
 *
 * Same discipline as `AircraftState` carrying DJI-native units (`docs/architecture.md`): store the
 * measurement, convert at the edge.
 */
data class StoredItem(
    val seq: Int,
    val frame: Int,
    val command: Int,
    val current: Int,
    val autocontinue: Int,
    val param1: Float,
    val param2: Float,
    val param3: Float,
    val param4: Float,
    /** Latitude in 1e7 degrees — **unless [frame] is `MAV_FRAME_MISSION`**, where it is not a
     *  coordinate at all. See [MissionCommands.COORDINATE_BEARING]. */
    val x: Int,
    /** Longitude in 1e7 degrees, with the same caveat as [x]. */
    val y: Int,
    /** Altitude in whatever [frame] says. Accepted plans only ever carry relative metres. */
    val z: Float,
    val missionType: Int,
)

/** Coordinate arithmetic and validity for the mission layer. */
object MissionGeo {

    /** 1e7-degree scaling, MAVLink's integer coordinate convention. */
    const val DEGREES_E7 = 1e7

    /**
     * Metres per degree of latitude — **an alias for the guidance constant, unified 2026-07-27.**
     *
     * It was `111_320.0` until Ivan ruled *"let's just put the most reasonable constant
     * everywhere"*, and the history is worth keeping because it is how the discrepancy survived.
     * An earlier KDoc here claimed this was "the same spherical constant
     * `RepositionGuidance.METRES_PER_DEG` uses". **It was not.** This layer used 111 320.0 (the
     * round WGS-84 equatorial figure) while the flight law used 111 194.93 (`2πR/360` at the mean
     * radius 6 371 000 m) — a divergence describing itself as a duplicate, which is exactly how a
     * 0.11 % difference in a safety gate goes unnoticed.
     *
     * **Why 111 194.93 is the one to keep**, rather than the rounder number:
     *
     *  - It is the **mean-radius** value, so it sits between the two quantities a single constant
     *    has to serve: a degree of latitude at 38°N is ≈ 110 996 m, and the coefficient for
     *    longitude is ≈ 111 320 m. A flat-earth approximation that uses one number for both is
     *    better served by the value between them than by either endpoint.
     *  - It is the number that has **flown**. Every goto and every orbit this project has
     *    performed used it, and the arrival errors validate it empirically — 0.09 m, 0.01 m,
     *    0.68 m, 0.74 m in real air, and a real-air orbit that held its radius to 9 cm of a 24.7 m
     *    target. Whatever the theory, this constant demonstrably lands the aircraft where the
     *    operator clicked.
     *  - Consistency is worth more than the difference. This ruler decides whether an upload is
     *    **refused**; the flight law decides where the aircraft actually goes. Having the gate
     *    measure in different units from the flight is a worse property than 11 cm of anything.
     *
     * **What the change does**, stated plainly rather than buried: the constant gets smaller, so a
     * given leg now measures 0.11 % *shorter*, so the `MAX_LEG_M` gate is **very slightly more
     * permissive** — a leg that computed as 100.05 m and was refused now computes as 99.94 m and
     * is accepted. Eleven centimetres on a hundred metres. Accepted deliberately.
     *
     * Not unified with it, and still different on purpose: [distanceM] evaluates the scale at the
     * **mean** of the two latitudes rather than at the start, which is the better choice for a
     * distance *between two points* and is not what the guidance law wants for a bearing *from*
     * one. And this layer still has no antimeridian wrap — reachable only by a plan crossing 180°,
     * and flagged rather than fixed.
     */
    const val METRES_PER_DEG = com.dimensional.mini4pro.telemetry.Geo.METRES_PER_DEG

    /**
     * The item's coordinate, or null when it does not carry one or the numbers are not a place.
     *
     * Delegates the *range* rule to [com.dimensional.mini4pro.telemetry.Geo] so the bridge has
     * exactly one definition of "a coordinate we are willing to repeat", including the
     * lat == lon filler check that caught DJI's 4.58e7 placeholder.
     */
    fun pointOrNull(item: StoredItem): GeoPoint? {
        if (item.command !in MissionCommands.COORDINATE_BEARING) return null
        if (item.frame !in MissionFrames.RELATIVE) return null
        val lat = item.x / DEGREES_E7
        val lon = item.y / DEGREES_E7
        val pair = com.dimensional.mini4pro.telemetry.Geo.coordinateOrNull(lat, lon) ?: return null
        return GeoPoint(pair.first, pair.second)
    }

    /**
     * Horizontal distance in metres, equirectangular with the cos(latitude) term that Stage B's
     * measurements made non-negotiable: dropping it is invisible at the equator and 21% wrong at
     * 38°N, which is this project's home latitude.
     *
     * That term is [com.dimensional.mini4pro.telemetry.Geo.longitudeScale] — **the one copy in
     * the bridge**, shared with the guidance law's `nedMetres`, its inverse, and the Zenoh
     * encoder's `enuMetres` (unified 2026-07-27). Only the term is shared: this function keeps
     * its own [METRES_PER_DEG] and evaluates the scale at the **mean** of the two latitudes
     * rather than the start latitude, because both choices are baked into an admission gate and
     * changing either moves which plans are refused. See [METRES_PER_DEG].
     */
    fun distanceM(from: GeoPoint, to: GeoPoint): Double {
        val dLat = (to.latDeg - from.latDeg) * METRES_PER_DEG
        val meanLatDeg = (to.latDeg + from.latDeg) / 2.0
        val dLon = (to.lonDeg - from.lonDeg) * METRES_PER_DEG *
            com.dimensional.mini4pro.telemetry.Geo.longitudeScale(meanLatDeg)
        return kotlin.math.hypot(dLat, dLon)
    }
}
