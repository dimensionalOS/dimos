package com.dimensional.mini4pro.telemetry

import io.dronefleet.mavlink.minimal.MavModeFlag

/**
 * DJI `FCFlightMode` → PX4 `custom_mode`.
 *
 * We identify as `MAV_AUTOPILOT_PX4` (see [TelemetryEncoder.heartbeat]), which
 * means QGC loads `PX4FirmwarePlugin` and reads the flight mode from
 * `custom_mode` alone. This file is the whole translation, kept out of the
 * encoder because it is a *judgement table*, not arithmetic, and it is the part
 * of the bridge most likely to be wrong.
 *
 * ## The wire contract
 *
 * `custom_mode` is a packed union (`ref/qgroundcontrol/src/FirmwarePlugin/PX4/
 * px4_custom_mode.h:39-51`):
 *
 * ```
 * custom_mode = (main_mode << 16) | (sub_mode << 24)
 * ```
 *
 * The low 16 bits are `reserved` and are always 0 here.
 * `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` must be set in `base_mode` or
 * `PX4FirmwarePlugin::flightMode()` returns the bare string `"Unknown"` and never
 * looks at `custom_mode` at all — there is **no `base_mode` fallback** like the
 * generic plugin's (`PX4FirmwarePlugin.cc:133-142`). QGC then looks the exact
 * `custom_mode` value up in a table built in `PX4FirmwarePlugin.cc:50-73`; a
 * value not in that table renders as the literal `Unknown <base_mode>:<custom_mode>`.
 *
 * ## The rule this table is built on
 *
 * **Map a DJI mode only when QGC's display string is a true description of what
 * the aircraft is doing.** The operator reads a word, not a number, and acts on
 * it. That rule is why `FOLLOW_ME` is mapped (QGC says "Follow Me"; the aircraft
 * is following the operator) and why `TAP_FLY` is not (QGC would say "Hold"
 * while the aircraft flies across a field) — see [UNMAPPED] and the rejected
 * mappings recorded below.
 *
 * Anything not in [byDjiMode] encodes as [UNMAPPED] and is **never** given a
 * plausible-looking neighbour. A mode we cannot name is not a mode we may
 * rename.
 *
 * ## Provenance and confidence
 *
 * `FCFlightMode` is **undocumented** — it exists only in the jar (`javap
 * dji.sdk.keyvalue.value.flightcontroller.FCFlightMode`, MSDK 5.18.0, **79**
 * constants including `UNKNOWN`; `docs/msdk/flight-modes.md:27` says "~80" and a
 * count of 78 has been quoted elsewhere — the jar is the authority and
 * [Px4ModeTest] pins the list), not in DJI's API reference. So almost every row below is inferred
 * from the constant's *name*, and each group says so. Exactly one row has
 * hardware evidence behind it: `APAS` is what the real Mini 4 Pro reports parked
 * on the ground with motors off (`docs/measurements/2026-07-25-key-sweep.md:47-51`).
 * Note that `KeyFlightMode` — the public 30-value `FlightMode` enum that
 * `docs/msdk/flight-modes.md` §D recommends — reads `UNKNOWN` on this airframe,
 * which is why `AircraftState.flightMode` carries an `FCFlightMode.name` and why
 * this table is keyed on those strings.
 *
 * Prior art: RosettaDrone maps the same enum (its MSDK v4 `FlightMode` shares the
 * distinctive constants `ATTI_HOVER`, `HOVER`, `GPS_BLAKE`, `ATTI_LIMITED`,
 * `GPS_ATTI_WRISTBAND`, `CONFIRM_LANDING`, `MOTORS_JUST_STARTED`, so it is the
 * same underlying firmware enum) onto ArduCopter numbers in
 * `ref/rosettadrone/app/src/main/java/sq/rogue/rosettadrone/DroneModel.java:887-973`.
 * Its numbers are useless to us; its judgements are a cross-check, and where we
 * disagree the disagreement is recorded at the row.
 */
object Px4Mode {

    // ── PX4 main modes (px4_custom_mode.h:5-17) ───────────────────────────────
    // Transcribed complete, including the values we never send, so this file is
    // the local record of the enum and a reader can see at a glance what PX4 has
    // that we do not use. Note main mode 0 is absent — that is what makes it safe
    // as our [UNMAPPED] sentinel.
    const val MAIN_MANUAL = 1
    const val MAIN_ALTCTL = 2
    const val MAIN_POSCTL = 3
    const val MAIN_AUTO = 4
    const val MAIN_ACRO = 5
    const val MAIN_OFFBOARD = 6
    const val MAIN_STABILIZED = 7
    const val MAIN_RATTITUDE_DEPRECATED = 8
    const val MAIN_SIMPLE = 9 // "unused, but reserved for future use" — px4_custom_mode.h:14
    const val MAIN_TERMINATION = 10
    const val MAIN_ALTITUDE_CRUISE = 11

    // ── AUTO sub-modes (px4_custom_mode.h:19-31) ──────────────────────────────
    // 7 is a hole: PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05.
    const val SUB_AUTO_READY = 1
    const val SUB_AUTO_TAKEOFF = 2
    const val SUB_AUTO_LOITER = 3
    const val SUB_AUTO_MISSION = 4
    const val SUB_AUTO_RTL = 5
    const val SUB_AUTO_LAND = 6
    const val SUB_AUTO_FOLLOW_TARGET = 8
    const val SUB_AUTO_PRECLAND = 9
    const val SUB_AUTO_VTOL_TAKEOFF = 10
    const val SUB_AUTO_GUIDED_COURSE = 11

    // ── POSCTL sub-modes (px4_custom_mode.h:33-37) ────────────────────────────
    const val SUB_POSCTL_POSCTL = 0
    const val SUB_POSCTL_ORBIT = 1
    const val SUB_POSCTL_SLOW = 2

    /**
     * Packs a PX4 main/sub mode pair into `custom_mode`.
     *
     * The shifts are the union's byte layout, not an arbitrary encoding:
     * `{uint16 reserved; uint8 main_mode; uint8 sub_mode;}` little-endian
     * (`px4_custom_mode.h:39-51`). Get the two shifts the wrong way round and
     * every mode still *encodes*, it just names a different mode — which is
     * precisely the failure this file exists to avoid, so [Px4ModeTest] asserts
     * the packed constants against hand-computed literals rather than against
     * this function.
     */
    fun packed(main: Int, sub: Int = 0): Long =
        (main.toLong() shl 16) or (sub.toLong() shl 24)

    // ── the modes we can name ─────────────────────────────────────────────────
    // Named for QGC's display string in the comment, since that is what an
    // operator actually sees (PX4FirmwarePlugin.cc:50-73).

    /** "Manual" */
    val MANUAL = packed(MAIN_MANUAL)

    /** "Altitude" — attitude sticks, altitude held, position free. */
    val ALTCTL = packed(MAIN_ALTCTL)

    /** "Position" — sticks command velocity, aircraft holds position when centred. */
    val POSCTL = packed(MAIN_POSCTL, SUB_POSCTL_POSCTL)

    /** "Orbit" */
    val POSCTL_ORBIT = packed(MAIN_POSCTL, SUB_POSCTL_ORBIT)

    /** "Position Slow" — position control with a reduced speed envelope. */
    val POSCTL_SLOW = packed(MAIN_POSCTL, SUB_POSCTL_SLOW)

    /** "Offboard" — an off-vehicle computer is commanding the setpoints. */
    val OFFBOARD = packed(MAIN_OFFBOARD)

    /** "Takeoff" */
    val AUTO_TAKEOFF = packed(MAIN_AUTO, SUB_AUTO_TAKEOFF)

    /** "Land" */
    val AUTO_LAND = packed(MAIN_AUTO, SUB_AUTO_LAND)

    /** "Return" */
    val AUTO_RTL = packed(MAIN_AUTO, SUB_AUTO_RTL)

    /** "Mission" */
    val AUTO_MISSION = packed(MAIN_AUTO, SUB_AUTO_MISSION)

    /** "Follow Me" */
    val AUTO_FOLLOW_TARGET = packed(MAIN_AUTO, SUB_AUTO_FOLLOW_TARGET)

    /**
     * "Hold". **Deliberately unused by [byDjiMode]** — see the rejected mappings
     * at the bottom of this file. Defined so the reasoning has something to point
     * at and so a future author who decides otherwise does not have to re-derive
     * the number.
     */
    val AUTO_LOITER = packed(MAIN_AUTO, SUB_AUTO_LOITER)

    /**
     * The value we send when we have no honest PX4 name for what DJI reported —
     * an unmapped `FCFlightMode`, `FCFlightMode.UNKNOWN` itself, or no reading at
     * all. QGC renders it as `Unknown <base_mode>:0`
     * (`PX4FirmwarePlugin.cc:138`).
     *
     * **Why 0, and why not simply drop `CUSTOM_MODE_ENABLED`.**
     *
     * Both candidates put the word "Unknown" on the operator's toolbar, so
     * neither can mislead. The choice was made on the three secondary points:
     *
     * 1. **0 can never collide with a real PX4 mode.** `PX4_CUSTOM_MAIN_MODE`
     *    starts at `MANUAL = 1` (`px4_custom_mode.h:5-17`); main mode 0 is not a
     *    member and cannot become one, because a new PX4 mode takes a new number
     *    rather than the reserved-empty slot. So this sentinel stays unambiguous
     *    across QGC and PX4 versions in a way an invented number would not.
     * 2. **`CUSTOM_MODE_ENABLED` is a statement about our encoding, not about
     *    the GCS's lookup table.** The flag means "this autopilot reports its
     *    mode in `custom_mode`", and a real PX4 sets it unconditionally. Toggling
     *    it as our own translation coverage comes and goes would make the field's
     *    validity flicker for reasons no receiver can infer, and would contradict
     *    the identity we claim in the same message.
     * 3. It keeps one code path. `base_mode` is then a constant-shaped field and
     *    the only thing that varies with mode is `custom_mode`, which is what the
     *    tests can hold to.
     *
     * What is deliberately *not* done: encoding the DJI mode's ordinal, or any
     * other private numbering, into the reserved low 16 bits. That would put a
     * number on the wire that looks like a PX4 mode and means something only we
     * know — the same objection that kept `custom_mode` at 0 under the previous
     * `MAV_AUTOPILOT_GENERIC` identity.
     *
     * The cost, stated plainly: an unmapped DJI mode and a dead telemetry key are
     * indistinguishable on the toolbar. The DJI mode name is in the flight
     * recorder either way, which is where that distinction is actually made.
     */
    const val UNMAPPED = 0L

    /**
     * The DJI flight modes that mean **the aircraft is in a landing sequence** — the same five
     * names this file's table maps to PX4 "Land", plus `FORCE_LANDING` (a member of DJI's
     * *public* `FlightMode` enum this airframe has not been observed to report; included
     * because a forced landing that ever did appear is still a landing). One set, one meaning,
     * two consumers: the guided engine's mode-seizure exemption during a commanded landing
     * (`GuidedStickEngine`), and the flight recorder's gimbal-recenter detector
     * (`record/GimbalRecenter`) — both need "is this a landing mode?" and a second private
     * list in either would drift.
     *
     * The one measured member on this airframe is `CONFIRM_LANDING`: all four landing01/02
     * landings went straight `GPS_ATTI → CONFIRM_LANDING` with no `AUTO_LANDING` phase first
     * (`landingdata.md` §2.2).
     */
    val DJI_LANDING_MODES: Set<String> = setOf(
        "AUTO_LANDING", "ATTI_LANDING", "CONFIRM_LANDING", "BASE_LANDING", "BACKUP_LANDING",
        "FORCE_LANDING",
    )

    /** Whether [djiMode] (an `FCFlightMode.name`, or null) is one of [DJI_LANDING_MODES]. */
    fun isDjiLandingMode(djiMode: String?): Boolean = djiMode in DJI_LANDING_MODES

    /**
     * The table. Keys are `FCFlightMode.name` exactly as `StateCache` reports it
     * (`StateCache.kt:86`, `s.flightMode?.name`).
     *
     * Every one of the 79 `FCFlightMode` constants is accounted for: either it is
     * a key here, or it appears in the rejected/unmapped notes below with a
     * reason. [Px4ModeTest] holds that invariant against the enum list captured
     * from the jar.
     */
    val byDjiMode: Map<String, Long> = buildMap {

        // ── Return to home → "Return" ─────────────────────────────────────────
        // The strongest equivalence in the table: the aircraft is autonomously
        // flying to its recorded home point and will land there. That is exactly
        // PX4 AUTO.RTL. RosettaDrone agrees (GO_HOME → ArduCopter RTL,
        // DroneModel.java:910).
        // BACKUP_GO_HOME is name-inferred: a redundant/secondary return path is
        // still a return, and nothing else in the enum reads that way.
        put("GO_HOME", AUTO_RTL)
        put("BACKUP_GO_HOME", AUTO_RTL)

        // ── Landing → "Land" ──────────────────────────────────────────────────
        // Every member is an autonomous descent to touchdown, which is what PX4
        // AUTO.LAND means. The distinctions DJI draws between them are about how
        // the descent is stabilised or who asked for it, and PX4 has no field for
        // that; none of them changes the sentence "this aircraft is landing".
        //
        // - AUTO_LANDING: the ordinary auto-land. RosettaDrone agrees
        //   (DroneModel.java:901).
        // - ATTI_LANDING: landing without a position solution. Still a landing;
        //   the loss of GPS is reported through GPS_RAW_INT, not here.
        // - CONFIRM_LANDING: DJI's "confirm landing?" state, hovering low over
        //   ground it is unsure of, waiting for the pilot. Mapped **with a known
        //   loss**: "Land" does not convey that the descent is paused pending
        //   confirmation. Kept because the aircraft is in a landing sequence and
        //   at low altitude, which is the part the operator must act on; the
        //   alternative ("Unknown" at 2 m over water) is worse.
        // - BASE_LANDING, BACKUP_LANDING: name-inferred, never observed. Almost
        //   certainly enterprise dock/redundancy features absent on a Mini 4 Pro.
        put("AUTO_LANDING", AUTO_LAND)
        put("ATTI_LANDING", AUTO_LAND)
        put("CONFIRM_LANDING", AUTO_LAND)
        put("BASE_LANDING", AUTO_LAND)
        put("BACKUP_LANDING", AUTO_LAND)

        // ── Takeoff → "Takeoff" ───────────────────────────────────────────────
        // The aircraft is climbing away from the ground under its own control.
        // AUTO_TAKE_OFF is the plain one (RosettaDrone agrees,
        // DroneModel.java:896). ASSISTED_TAKE_OFF / TAKEOFF /
        // QUICKTAKEOFF_ASSIST / PALM_LAUNCH are name-inferred: each name states
        // a takeoff, and no other reading of "launch" or "takeoff" is available.
        // None has been observed on this airframe.
        put("AUTO_TAKE_OFF", AUTO_TAKEOFF)
        put("ASSISTED_TAKE_OFF", AUTO_TAKEOFF)
        put("TAKEOFF", AUTO_TAKEOFF)
        put("QUICKTAKEOFF_ASSIST", AUTO_TAKEOFF)
        put("PALM_LAUNCH", AUTO_TAKEOFF)

        // ── Waypoint navigation → "Mission" ───────────────────────────────────
        // NAVI_GO / NAVI_GO_NEW are DJI's waypoint-route execution states ("navi"
        // = navigation; the aircraft is flying a stored route). PX4 AUTO.MISSION
        // is the same behaviour. RosettaDrone maps the equivalent v4 constant
        // (GPS_WAYPOINT) to ArduCopter AUTO (DroneModel.java:905).
        //
        // Caveat for M4 (missions): "Mission" invites the operator to read this
        // as *the mission they uploaded in QGC's Plan view*. Until MAVLink
        // missions are actually wired to WPML, a route started from the DJI Fly
        // app would display the same way. The sentence "this aircraft is flying a
        // waypoint route" stays true either way, which is why the row stands.
        put("NAVI_GO", AUTO_MISSION)
        put("NAVI_GO_NEW", AUTO_MISSION)

        // ── Virtual stick → "Offboard" ────────────────────────────────────────
        // The one row that matters for M3, and the one whose evidence is
        // weakest — `docs/measurements/2026-07-25-ground-probe.md:99` lists
        // "whether virtual stick surfaces as a distinct FCFlightMode" as an
        // **open hardware question that blocks the M3 mode mapping**. It has not
        // been measured. If it turns out virtual stick leaves the mode at
        // GPS_ATTI/APAS, this row is simply never reached and the GUIDED claim
        // must instead come from our own GuidedController state.
        //
        // The claim itself: PX4 OFFBOARD means "setpoints are arriving from a
        // computer that is not the flight controller", which is exactly what DJI
        // virtual stick is. So *if* the aircraft reports JOYSTICK, "Offboard" is
        // the true description.
        //
        // RosettaDrone deliberately left custom_mode unset for JOYSTICK, noting
        // "JOYSTICK is also used in Auto Mode" (DroneModel.java:915-917). We read
        // that as self-referential rather than as a fact about DJI: RosettaDrone
        // synthesises its own waypoint following *through* virtual stick, so of
        // course the aircraft reported JOYSTICK during its auto mode. The lesson
        // that does transfer: once we synthesise missions the same way (there is
        // no FlyTo on this airframe — docs/mini4pro-constraints.md:10), the
        // aircraft will report JOYSTICK while flying a mission, and the override
        // to AUTO_MISSION must then come from our controller, not from this table.
        put("JOYSTICK", OFFBOARD)

        // ── Normal GPS flight → "Position" ────────────────────────────────────
        // Sticks command velocity; release them and the aircraft holds position.
        // That is PX4 POSCTL, and it is what a Mini 4 Pro does in P mode.
        //
        // - APAS: **the only measured row.** Reported parked on the ground,
        //   motors off, never flown (docs/measurements/2026-07-25-key-sweep.md:47-51).
        //   APAS is position mode with obstacle sensing active; the sensing has no
        //   PX4 representation and is not a change of control regime.
        // - GPS_ATTI: the plain P mode this airframe's firmware is built around.
        // - GPS_BRAKE: the braking transient after the sticks are centred. Still
        //   position mode, still pilot-authoritative — this is *not* AUTO.LOITER,
        //   because the pilot has not handed over anything.
        // - GPS_HOMELOCK, GPS_CL (course lock): position control flown in a fixed
        //   stick frame rather than the aircraft's. PX4 reserves a SIMPLE main
        //   mode (px4_custom_mode.h:14) for exactly this idea, but it is marked
        //   "unused, but reserved for future use" and QGC renders it as the
        //   uninformative "Simple"; "Position" is the true statement about the
        //   control regime, so it wins.
        // - GPS_ATTI_WRISTBAND: GPS_ATTI flown from a wristband controller. The
        //   name is compositional, so the regime is unambiguous even though the
        //   feature is not on this airframe.
        // - GPS_SPORT: **decided against MANUAL.** DJI Sport raises the speed
        //   envelope and disables obstacle avoidance, but position hold stays on:
        //   centre the sticks and the aircraft stops. PX4 MANUAL on a multirotor
        //   means no position *or* altitude hold, so reporting MANUAL would tell
        //   the operator the aircraft will drift if they let go — false, and
        //   false in the dangerous direction. The cost of POSCTL is that Sport
        //   and Normal look identical on the toolbar; PX4 has no "position, fast"
        //   mode to carry the difference, and inventing one is not available.
        put("APAS", POSCTL)
        put("GPS_ATTI", POSCTL)
        put("GPS_BRAKE", POSCTL)
        put("GPS_HOMELOCK", POSCTL)
        put("GPS_CL", POSCTL)
        put("GPS_ATTI_WRISTBAND", POSCTL)
        put("GPS_SPORT", POSCTL)

        // ── Speed-limited GPS flight → "Position Slow" ────────────────────────
        // Position control with a reduced speed envelope: full position hold, but
        // a lower speed limit and gentler stick response. DJI Tripod (TRIPOD_GPS)
        // and Beginner (GPS_NOVICE) are exactly that, and so is PX4's POSCTL_SLOW,
        // so the two say the same thing about the aircraft.
        // `FC.KeyIsTripodModeExpSupport` reads true on this airframe
        // (docs/measurements/2026-07-25-key-sweep.md:34), so TRIPOD_GPS is a mode
        // it really has; GPS_NOVICE is name-inferred.
        //
        // POSCTL_SLOW is preferred over plain POSCTL because both are true and this
        // one is not coarse: it keeps the answer to the in-flight question
        // *will it hold if I let go of the sticks* — yes — and adds the speed limit
        // that "Position" drops. QGC renders `33_751_040` as "Position Slow"
        // (`slowFlightModeName`, PX4FirmwarePlugin.cc:44 and :60), measured against
        // the binary in docs/measurements/2026-07-26-qgc-master-mode-sweep.md.
        //
        // These two rows were downgraded to plain POSCTL from 2026-07-25 to
        // 2026-07-26: QGC 5.0.8 had no `PX4_CUSTOM_SUB_MODE_POSCTL_SLOW` at all and
        // printed the bare "Unknown 81:33751040", so the coarse word beat no word.
        // **The rule, if that situation ever returns:** the deciding fact is
        // whether the *installed* QGC lists POSCTL_SLOW in
        // `_setModeEnumToModeStringMapping`. If a target QGC does not, downgrade
        // these two rows to [POSCTL] again — "Position" is coarse but true, and its
        // error runs slow, which is the safe direction. While the installed QGC
        // does list it, the downgrade is pure information loss and must not stand.
        put("TRIPOD_GPS", POSCTL_SLOW)
        put("GPS_NOVICE", POSCTL_SLOW)

        // ── Point-of-interest circling → "Orbit" ──────────────────────────────
        // "Hotpoint" is DJI's own long-standing name for orbiting a point of
        // interest (v4 shipped a `HotpointMission` class), and PX4's POSCTL_ORBIT
        // is circling a commanded centre. Same behaviour, same display word.
        // VISION_POI is deliberately *not* here: a vision-selected point of
        // interest may be tracked rather than circled, and we cannot tell which.
        put("GPS_HOTPOINT", POSCTL_ORBIT)

        // ── Attitude modes → "Altitude" ───────────────────────────────────────
        // DJI ATTI: no position hold, barometric altitude hold, sticks command
        // attitude. PX4 ALTCTL is defined the same way, and PX4 STABILIZED is
        // **not** the match — STABILIZED has manual throttle and no altitude
        // hold, which would understate what the aircraft is doing for the pilot.
        //
        // ATTI_HOVER is ATTI holding as best it can (i.e. drifting) — "Altitude"
        // says exactly that. ATTI_CL is ATTI in a course-locked stick frame,
        // ATTI_LIMITED is ATTI inside an envelope limit; both are name-inferred
        // and neither changes the altitude/position story.
        //
        // **RosettaDrone disagrees and is wrong here.** It maps ATTI to
        // ArduCopter LOITER (DroneModel.java:892), and LOITER means GPS position
        // hold — the one thing ATTI does not have. `docs/msdk/flight-modes.md:71`
        // already caught this; recording it again because it is the kind of error
        // that gets copied back in.
        put("ATTI", ALTCTL)
        put("ATTI_HOVER", ALTCTL)
        put("ATTI_CL", ALTCTL)
        put("ATTI_LIMITED", ALTCTL)

        // ── Manual → "Manual" ─────────────────────────────────────────────────
        // Name identity, and the meanings agree: direct attitude command with no
        // altitude or position assistance. RosettaDrone agrees (MANUAL →
        // ArduCopter STABILIZE, DroneModel.java:887 — the closest ArduCopter has).
        // Whether a Mini 4 Pro will ever report it is another matter; DJI exposes
        // no acro/manual mode to consumers.
        put("MANUAL", MANUAL)

        // ── Follow the operator → "Follow Me" ─────────────────────────────────
        // DJI Follow Me flies to the controller's own GPS position; PX4
        // AUTO.FOLLOW_TARGET flies to the target position the ground station
        // reports. Same target, same behaviour, and "Follow Me" is a true
        // sentence about both.
        //
        // ACTIVE_TRACK and ACTIVE_TRACK_COURSE_LOCK are deliberately excluded:
        // they follow a subject picked out of the camera image, which may not be
        // the operator at all, and "Follow Me" would tell the operator the
        // aircraft is coming to *them*. RosettaDrone maps ACTIVE_TRACK to
        // ArduCopter GUIDED (DroneModel.java:930), which does not carry that
        // claim — under ArduPilot it had a neutral word available and we do not.
        //
        // Displays as "Follow Me": the string is wired to `AUTO_FOLLOW_TARGET` in
        // `_setModeEnumToModeStringMapping` (PX4FirmwarePlugin.cc:41 and :66), and
        // the value we send was measured printing that word
        // (docs/measurements/2026-07-26-qgc-master-mode-sweep.md).
        //
        // It was undisplayable on QGC 5.0.8, which defined `AUTO_FOLLOW_TARGET`
        // with exactly our number and declared `followMeFlightModeName` one screen
        // earlier, then never connected them — so the row rendered as
        // "Unknown 85:134479872". The row stood through that unchanged, because
        // unlike TRIPOD_GPS there is no truthful coarser word to fall back to, and
        // because no Mini 4 Pro reaches it anyway: the airframe has FocusTrack
        // (subject-following), not the GPS-follows-the-controller feature this
        // names. Should an older QGC become the target again, it stays as it is.
        put("FOLLOW_ME", AUTO_FOLLOW_TARGET)
    }

    /**
     * The DJI mode's `custom_mode`, or [UNMAPPED] for a mode we cannot honestly
     * name — including `null` (no reading) and `"UNKNOWN"` (DJI's own no-reading,
     * which is what `KeyFlightMode` returns on this airframe and what
     * `KeyFCFlightMode` would return if the firmware lost track).
     */
    fun customMode(djiFlightMode: String?): Long =
        djiFlightMode?.let { byDjiMode[it] } ?: UNMAPPED

    /**
     * The `base_mode` description flags implied by a `custom_mode`, on top of the
     * unconditional ones in [TelemetryEncoder.baseModeFlags].
     *
     * Derived from `custom_mode` rather than from a second lookup on the DJI mode
     * so the two halves of the heartbeat cannot disagree with each other. Real
     * PX4 populates these flags too, so setting them is part of being a
     * convincing PX4 — but note QGC itself reads none of them for display under
     * this identity (`PX4FirmwarePlugin::flightMode()` branches on
     * `CUSTOM_MODE_ENABLED` and then on `custom_mode` alone,
     * `PX4FirmwarePlugin.cc:133-142`). They are for other consumers and for the
     * log.
     *
     * - `AUTO_ENABLED` ("system finds its own goal positions") for the AUTO main
     *   mode: takeoff, land, RTL, mission, follow.
     * - `GUIDED_ENABLED` for OFFBOARD, where the goal positions come from us, and
     *   for POSCTL_ORBIT, where the circle was designated by an operator rather
     *   than flown by hand.
     * - nothing extra for MANUAL / ALTCTL / POSCTL / POSCTL_SLOW: those are the
     *   pilot flying the aircraft, already covered by `MANUAL_INPUT_ENABLED` and
     *   `STABILIZE_ENABLED`.
     * - nothing extra for [UNMAPPED]: an unknown mode claims the least
     *   capability, never autonomy we cannot demonstrate.
     */
    fun extraBaseModeFlags(customMode: Long): List<MavModeFlag> = when {
        customMode == UNMAPPED -> emptyList()
        mainMode(customMode) == MAIN_AUTO -> listOf(MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED)
        mainMode(customMode) == MAIN_OFFBOARD -> listOf(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
        customMode == POSCTL_ORBIT -> listOf(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)
        else -> emptyList()
    }

    /** The `main_mode` byte of a packed `custom_mode`. */
    fun mainMode(customMode: Long): Int = ((customMode shr 16) and 0xFF).toInt()

    /** The `sub_mode` byte of a packed `custom_mode`. */
    fun subMode(customMode: Long): Int = ((customMode shr 24) and 0xFF).toInt()

    /**
     * `FCFlightMode` modes that are **deliberately** not in [byDjiMode], with the
     * reason. Present as data so [Px4ModeTest] can assert that the two lists
     * together cover all 79 constants in the jar — i.e. that no mode was left out
     * by accident rather than by decision.
     *
     * The names are the enum's, from
     * `javap dji.sdk.keyvalue.value.flightcontroller.FCFlightMode` (MSDK 5.18.0).
     */
    val deliberatelyUnmapped: Map<String, String> = mapOf(

        // --- flying to a designated point: rejected on the display-string rule ---
        // PX4 has no "flying to a point" mode. Its own goto is DO_REPOSITION,
        // which puts PX4 into AUTO.LOITER — so a real PX4 does display "Hold"
        // while translating (PX4FirmwarePlugin.cc:614, gotoFlightMode() is
        // AUTO_LOITER). We still refuse the mapping: "Hold" reads to an operator
        // as a stationary aircraft, and these are DJI-initiated moves the QGC
        // operator did not command, so there is no pending action to explain the
        // word. Reporting nothing costs information; reporting "Hold" costs
        // correctness of belief, and this project prefers the former (PLAN.md,
        // "Honesty boundaries"). One line each to reverse if a future author
        // weighs it the other way.
        "CLICK_GO" to "flying to a tapped point; PX4's only word for it is \"Hold\"",
        "TAP_FLY" to "as CLICK_GO. RosettaDrone mapped it to ArduCopter GUIDED, which PX4 has no equivalent of",
        "GO_TARGET_POINT" to "name suggests a goto; undocumented, and same \"Hold\" problem",
        "FLYTO_LIVE_TARGET" to "\"live target\" may be a moving subject; cannot tell goto from tracking",

        // --- vision tracking: would claim "Follow Me" about a subject that is not the operator ---
        "ACTIVE_TRACK" to "follows a camera-selected subject, not the operator; \"Follow Me\" would lie about who",
        "ACTIVE_TRACK_COURSE_LOCK" to "as ACTIVE_TRACK",
        "VISION_POI" to "vision point-of-interest; cannot tell circling (Orbit) from tracking",

        // --- canned cinematic routines: would claim "Mission" ---
        // The aircraft is autonomous, but PX4's only autonomous-flight word is
        // AUTO.MISSION, which means *the mission the operator uploaded*. With M4
        // about to make that literal, labelling a QuickShot "Mission" would point
        // the operator at a Plan-view route that is not running.
        "CINEMATIC" to "canned routine; \"Mission\" would name the operator's uploaded mission",
        "DRAW" to "canned routine",
        "PANO" to "canned routine",
        "QUICK_MOVIE" to "canned routine",
        "TIME_LAPSE" to "canned routine",
        "MASTER_SHOT" to "canned routine",
        "DANCING" to "canned routine",
        "JUMPING" to "canned routine",
        "FIREFLY" to "canned routine",
        "FIREFLY_EXIT" to "canned routine",
        "GESTURE_CONTROL" to "gesture-initiated capture; regime unclear",
        "NOE" to "nap-of-the-earth terrain following; PX4 has no terrain-follow mode",
        "FPV" to "on DJI a camera/feed mode rather than a flight regime; unclear",

        // --- degraded or fault states: must not read as a normal mode ---
        // PX4's TERMINATION is flight termination (outputs cut / chute). Nothing
        // DJI reports means that, and reaching for it because it is the only
        // "something is wrong" slot would be a far worse error than saying
        // nothing.
        "FAULT_TOLERANT" to "degraded control after a failure; naming it any normal mode understates it, and TERMINATION overstates it",
        "ROLLOVER_RESCUE" to "motor recovery after a ground rollover; not a flight mode",

        // --- ground / transitional states ---
        // MOTOR_START is the CSC spin-up. AUTO_READY ("Ready") was considered and
        // rejected: its main mode is AUTO, which would claim autonomy for the
        // pilot holding a stick combination, and armed/landed state already tell
        // QGC the ground truth through the heartbeat and EXTENDED_SYS_STATE.
        "MOTOR_START" to "ground spin-up; AUTO_READY would claim AUTO main mode for a pilot action",
        "PRE_MANUAL" to "transitional; unclear whether manual authority is already in effect",
        "HOVER" to "no prefix, so we cannot tell GPS hold (Position) from attitude drift (Altitude) — the one distinction the operator needs",

        // --- not this airframe, or no readable meaning ---
        "FARMING" to "agricultural aircraft",
        "FARM_WORK" to "agricultural aircraft",
        "FIXED_WING" to "not a multirotor mode",
        "ADSB_ACTION" to "ADS-B avoidance manoeuvre; regime unclear",
        "NAVI_SUBMODE_TA" to "undocumented waypoint sub-mode",
        "FLASHLIGHT" to "undocumented",
        "FLASHLIGHT_SPORT" to "undocumented",
        "FLASHLIGHT_ATTI" to "undocumented",
        "SPOTLIGHT_NORMAL" to "undocumented",
        "SPOTLIGHT_TRIPOD" to "undocumented",
        "SPOTLIGHT_SPORT" to "undocumented",
        "TRANSPORT_DIVERT" to "delivery aircraft",
        "TRANSPORT_HOIST_ASSIST" to "delivery aircraft",
        "COMMANDER_MODE" to "undocumented",
        "VISUAL_EXPLORATION" to "undocumented",
        "AUTO_EXPLORE" to "undocumented",
        "SDR_QUALITY_DETECT" to "link diagnostic, not a flight regime",
        "DEPARTURE_WAYLINE_TEST" to "route self-test; enterprise",
        "CALI_POWER_MODEL" to "calibration routine",
        "CABLE_FOLLOW" to "powerline inspection; enterprise",
        "CABLE_INSPECTION" to "powerline inspection; enterprise",
        "LOCK_YAW" to "a yaw constraint on top of some other mode, not a mode",

        // --- DJI's own no-reading ---
        "UNKNOWN" to "DJI does not know either; must not be given a name",
    )
}
