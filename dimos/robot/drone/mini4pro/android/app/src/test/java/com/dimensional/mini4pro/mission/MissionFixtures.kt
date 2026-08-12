package com.dimensional.mini4pro.mission

/**
 * Shared shapes for the three mission suites. Deliberately tiny: a builder that defaults every
 * field to something *admissible* would hide the fields a test is actually about, so the defaults
 * here are the ones a plain QGC waypoint really carries and every test names what it changes.
 *
 * The coordinates are the project's home family — 38 °N, 23.7 °E — chosen for the same reason
 * `GuidedRepositionTest` chose them: cos(38°) = 0.788, so a missing cos(latitude) term is a 21%
 * error rather than nothing.
 */
object MissionFixtures {

    const val LAT = 38.0
    const val LON = 23.7

    fun latE7(degrees: Double): Int = Math.round(degrees * MissionGeo.DEGREES_E7).toInt()

    /** A latitude [metres] north of [LAT], as 1e7 degrees. */
    fun northOf(metres: Double): Int = latE7(LAT + metres / MissionGeo.METRES_PER_DEG)

    /** A longitude [metres] east of [LON], as 1e7 degrees. */
    fun eastOf(metres: Double): Int = latE7(
        LON + metres / (MissionGeo.METRES_PER_DEG * kotlin.math.cos(Math.toRadians(LAT)))
    )

    fun item(
        seq: Int,
        command: Int,
        frame: Int = MissionFrames.GLOBAL_RELATIVE_ALT,
        param1: Float = 0f,
        param2: Float = 0f,
        param3: Float = 0f,
        // NaN, not 0: QGC's own plan items leave yaw unspecified, and a finite yaw is refused
        // because the engine pins yaw rate to zero.
        param4: Float = Float.NaN,
        x: Int = latE7(LAT),
        y: Int = latE7(LON),
        z: Float = 10f,
        current: Int = 0,
        autocontinue: Int = 1,
        missionType: Int = MissionTypes.MISSION,
    ) = StoredItem(
        seq = seq,
        frame = frame,
        command = command,
        current = current,
        autocontinue = autocontinue,
        param1 = param1,
        param2 = param2,
        param3 = param3,
        param4 = param4,
        x = x,
        y = y,
        z = z,
        missionType = missionType,
    )

    /** A plain waypoint [metres] north of the origin. */
    fun waypoint(seq: Int, metresNorth: Double, z: Float = 10f) =
        item(seq, MissionCommands.NAV_WAYPOINT, x = northOf(metresNorth), y = latE7(LON), z = z)

    /**
     * The smallest admissible plan: takeoff, one waypoint, and a terminal `NAV_LAND`.
     *
     * The last item does **not** descend — M4-5 turned it into "fly there, come to rest, and
     * hold". The name is MAVLink's; the behaviour is a hover, and the aircraft ends the mission
     * in the air waiting for a human on the RC.
     */
    fun takeoffAndHoldPlan(): List<StoredItem> = listOf(
        item(0, MissionCommands.NAV_TAKEOFF, x = 0, y = 0, z = 5f),
        waypoint(1, 20.0),
        item(2, MissionCommands.NAV_LAND, x = 0, y = 0, z = 0f),
    )
}
