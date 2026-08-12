package com.dimensional.mini4pro.telemetry

import io.dronefleet.mavlink.common.Attitude
import io.dronefleet.mavlink.common.BatteryStatus
import io.dronefleet.mavlink.common.ExtendedSysState
import io.dronefleet.mavlink.common.GlobalPositionInt
import io.dronefleet.mavlink.common.GpsFixType
import io.dronefleet.mavlink.common.GpsGlobalOrigin
import io.dronefleet.mavlink.common.GpsRawInt
import io.dronefleet.mavlink.common.HomePosition
import io.dronefleet.mavlink.common.MavBatteryChargeState
import io.dronefleet.mavlink.common.MavBatteryFunction
import io.dronefleet.mavlink.common.MavBatteryType
import io.dronefleet.mavlink.common.MavLandedState
import io.dronefleet.mavlink.common.MavSysStatusSensor
import io.dronefleet.mavlink.common.MavVtolState
import io.dronefleet.mavlink.common.SysStatus
import io.dronefleet.mavlink.common.VfrHud
import io.dronefleet.mavlink.minimal.MavAutopilot
import io.dronefleet.mavlink.minimal.MavModeFlag
import io.dronefleet.mavlink.minimal.MavState
import io.dronefleet.mavlink.minimal.MavType
import io.dronefleet.mavlink.minimal.Heartbeat
import io.dronefleet.mavlink.util.EnumValue
import java.math.BigInteger
import kotlin.math.floor
import kotlin.math.hypot

/**
 * Turns an [AircraftState] into MAVLink messages. Pure arithmetic: no Android
 * imports, no DJI imports, no clock reads — the caller passes `timeBootMs`, so
 * every message this produces is a deterministic function of its inputs and can
 * be asserted field by field in a JVM unit test.
 *
 * Unit conversions live here and nowhere else. The ones that have already cost
 * time, all confirmed against real readings in
 * `docs/measurements/2026-07-25-ground-probe.md`:
 *
 * - **AMSL = `takeoffAltitudeAmsl + relativeAltitude`.** `relativeAltitude` alone
 *   is takeoff-relative and reads 0 on the ground. If either half is missing,
 *   AMSL is *unknown* — see [ALT_UNKNOWN_MM].
 *   **The sum is only as absolute as its datum, and the datum is not absolute.**
 *   `takeoffAltitudeAmsl` was measured on 2026-07-26 to be *pressure altitude* on
 *   the ISA 1013.25 hPa reference, not a QNH-corrected elevation: it moved 44 m at
 *   a stationary site over two days, tracking the weather to ±2.4 m
 *   (`docs/measurements/2026-07-26-amsl-datum.md`). Every "AMSL" this file emits
 *   inherits that offset, which was +14 m one day and −28 m the next. It is
 *   deliberately **not** corrected here — see that document for why a constant
 *   would be a worse lie than the offset.
 * - `GLOBAL_POSITION_INT.alt` is **mm**; `VFR_HUD.alt` is **metres**.
 * - `GLOBAL_POSITION_INT.hdg` is **centi-degrees** 0..35999;
 *   `VFR_HUD.heading` is **integer degrees** 0..359.
 * - `ATTITUDE.yaw` takes the signed yaw straight through (both are [-pi,+pi] /
 *   [-180,180] with 0 = north); only the two heading fields wrap to 0..360.
 * - DJI current is mA and **negative while discharging**; MAVLink is cA and
 *   **positive for draw**. So cA = -mA / 10.
 *
 * ## Unknown values
 *
 * `AircraftState` is null-per-field on purpose, and a null must never be encoded
 * as 0: that is how a bridge tells a GCS the aircraft is at lat 0 / lon 0, on the
 * ground, with a flat battery. Each field uses the sentinel its own MAVLink
 * definition documents (`UINT16_MAX`, `UINT8_MAX`, `INT16_MAX`, `-1`, …), quoted
 * at the point of use. Where a field has **no** documented sentinel the choice is
 * called out in a comment, because there the encoding is a judgement call:
 *
 * - int32 mm altitudes → [ALT_UNKNOWN_MM] (`INT32_MIN`): absurd on a GCS display,
 *   which is the point. 0 would read as a plausible sea-level altitude.
 * - float HUD scalars → `NaN`. QGC explicitly tests these for NaN
 *   (`VehicleFactGroup.cc:215-217`), so NaN is the value it understands as
 *   "no data".
 * - `ATTITUDE.roll/pitch/yaw` → `NaN`. A missing attitude encoded as 0 is a
 *   confidently level aircraft; QGC will instead render non-numeric, which is
 *   the honest outcome.
 * - lat/lon → 0/0, *the one place a zero is right*: QGC discards a
 *   `GLOBAL_POSITION_INT` whose lat and lon are both 0 (`Vehicle.cc:875`) and
 *   ignores `GPS_RAW_INT` positions below a 3D fix (`Vehicle.cc:844`), so the
 *   0/0 pair plus a `NO_GPS` fix type is the "do not plot me" encoding. A
 *   partially valid pair is never emitted — if either coordinate is missing or
 *   fails [Geo]'s validity rule, both go to 0.
 * - velocities → 0 cm/s. No sentinel exists for the int16 velocity fields. 0 is
 *   bounded and indistinguishable from a stationary aircraft, which is the least
 *   dangerous reading of the three (unlike position or altitude, it cannot put a
 *   symbol somewhere the aircraft is not).
 * - `SYS_STATUS`'s `MAV_SYS_STATUS_PREARM_CHECK` — the flag behind QGC's green
 *   "Ready" — is healthy only on positive evidence, and any unknown reads as not
 *   ready. It is the most consequential judgement in this file; the reasoning,
 *   including why a "Ready" here still does not promise the aircraft will fly,
 *   is in [prearmHealthy].
 * - `HOME_POSITION` / `GPS_GLOBAL_ORIGIN` are **not emitted at all** when home is
 *   unknown ([homePosition] and [gpsGlobalOrigin] return null) — and "unknown"
 *   includes DJI saying so via `KeyIsHomeLocationSet`, and DJI handing us a
 *   non-null coordinate that is not one, which is what it does before a home
 *   point exists. Both gates are in [homeCoordinate]. QGC plots the
 *   home symbol from these and does altitude arithmetic against them, so there is
 *   no safe placeholder — silence is the encoding. Under the PX4 identity this
 *   matters *more* than it did: `Vehicle::setGuidedModeROI` takes the
 *   `px4Firmware()` branch into `TerrainQueryCoordinator::roiWithTerrain`
 *   (`Vehicle.cc:1983-1986`), whose failure path uses
 *   `homePosition().altitude()` as the ROI's AMSL altitude
 *   (`TerrainQueryCoordinator.cc:123`). A fabricated home would become a
 *   fabricated commanded altitude.
 *
 * ## Stale values — this encoder ignores [AircraftState.ages], deliberately
 *
 * `AircraftState` now carries a per-signal age ([SampleAges]), because DJI updates
 * each key on its own schedule and `KeyAircraftVelocity` fired **once in 35 s** on
 * the ground probe — so a 5 Hz emitter re-sends cached values and nothing here can
 * tell a stationary aircraft from a dead feed. **Nothing in this file reads those
 * ages, and the bytes on the wire are unchanged by them.** That is a decision, not
 * an oversight; the full argument is in
 * `docs/decisions/2026-07-25-per-field-staleness.md` and the short form is:
 *
 * - **The [globalPositionIntOrNull] precedent does not extend to age.** That
 *   suppression exists because an unknown position has *no encodable form*: lat/lon
 *   have no sentinel, and 0/0 is a real place QGC plots confidently. Suppression is
 *   the only honest encoding of "we have nothing". A stale reading is a different
 *   thing entirely — it is a **real measurement that may still be correct**, and it
 *   is the best estimate we have. Suppressing it replaces a possibly-old truth with
 *   a definite absence.
 * - **Age is not a reliable staleness signal on this airframe yet.** The velocity
 *   key is change-only, so a large age is the *expected steady state* of a hovering
 *   or parked aircraft. Suppressing on it would blank velocity precisely when the
 *   aircraft is holding still, and would teach the operator that gaps mean nothing.
 *   Whether the key delivers at rate in flight is unmeasured — it is an open item
 *   on the moving-aircraft session.
 * - **Suppression buys nothing from QGC anyway.** QGC holds the last value of a
 *   fact and keeps drawing it; dropping `ATTITUDE` freezes the HUD at the same
 *   number rather than blanking it, while also breaking the continuous stream a GCS
 *   expects. A `STATUSTEXT` warning was considered and rejected for the same reason
 *   `HandshakeResponder` de-duplicates its mode refusals: a warning that fires
 *   whenever the aircraft is parked is a warning nobody reads.
 *
 * So staleness is made **observable** — to the flight recorder now, and to
 * `GuidedController` in M3, which must treat an aged velocity as *no feedback*
 * rather than as a measured zero — without changing what a ground station sees.
 * Changing what we emit to QGC is a bigger step than making staleness visible, and
 * it should be taken on measured evidence about in-flight delivery rates, not on
 * a ground probe. [periodicMessages] being a function of values alone is pinned by
 * a test, so reversing this is a deliberate act rather than a drift.
 */
object TelemetryEncoder {

    // ── MAVLink's documented sentinels, named once ────────────────────────────
    /** `UINT16_MAX`, the documented "unknown" for eph/epv/vel/cog/hdg/voltage. */
    const val UINT16_MAX = 65535

    /** `UINT8_MAX`, documented "unknown" for `GPS_RAW_INT.satellites_visible`. */
    const val UINT8_MAX = 255

    /** `INT16_MAX`, documented "unknown" for `BATTERY_STATUS.temperature`. */
    const val INT16_MAX = 32767

    /**
     * Our sentinel for an unknown int32 millimetre altitude
     * (`GLOBAL_POSITION_INT.alt`/`.relative_alt`, `GPS_RAW_INT.alt`). MAVLink
     * documents none. `INT32_MIN` displays as roughly -2 147 483 m, which no
     * operator can mistake for an altitude; 0 would silently read as sea level.
     * [mm] never returns this value for a real measurement — it saturates at
     * `INT32_MIN + 1` — so the sentinel stays unambiguous.
     */
    const val ALT_UNKNOWN_MM = Int.MIN_VALUE

    // ── whole-tick convenience ────────────────────────────────────────────────

    /**
     * The messages due on every telemetry tick, in a sensible send order.
     * `HOME_POSITION`/`GPS_GLOBAL_ORIGIN` are deliberately absent: they are
     * event messages, see [eventMessages].
     */
    fun periodicMessages(s: AircraftState, timeBootMs: Long): List<Any> = listOf(
        heartbeat(s),
        sysStatus(s),
        gpsRawInt(s, timeBootMs),
        globalPositionInt(s, timeBootMs),
        attitude(s, timeBootMs),
        vfrHud(s),
        batteryStatus(s),
        // QGC requests this one by interval; landed_state should change promptly
        // on takeoff and touchdown, so it rides the same tick rather than being
        // polled. Two bytes of payload.
        extendedSysState(s),
    )

    /**
     * The event messages — send on home-set and on change, not at rate. Empty
     * while home is unknown, which is the whole point (see the class doc).
     */
    fun eventMessages(s: AircraftState): List<Any> =
        listOfNotNull(homePosition(s), gpsGlobalOrigin(s))

    // ── HEARTBEAT ─────────────────────────────────────────────────────────────

    /**
     * `HEARTBEAT`.
     *
     * **We identify as `MAV_AUTOPILOT_PX4.`** This is the third identity this
     * bridge has worn, and the reasoning for each move matters more than the
     * answer:
     *
     * - **ArduPilot — tried on the real QGC and reversed.** Claiming APM makes
     *   QGC load its ArduPilot plugin, which demands ~30 per-flight-controller
     *   *calibration* parameters (`COMPASS_OFS_*`, `INS_ACCOFFS_*`, `RCMAP_*`,
     *   `RC0_MIN/MAX/TRIM`, `FLTMODE1..6`, `ARMING_CHECK`) and pops "Your vehicle
     *   requires setup prior to flight". DJI exposes none of them because it
     *   calibrates internally, and **fabricating calibration values in software
     *   that flies an aircraft is not an option.** ArduPilot stays rejected on
     *   exactly that ground and no other.
     * - **GENERIC — honest, and blind.** `FirmwarePlugin::isCapable()` returns
     *   false for everything on QGC's base class (`FirmwarePlugin.h:127`), so a
     *   generic vehicle has no Takeoff / Land / RTL / Pause / Go-to / Orbit / ROI
     *   buttons at all and an empty mode dropdown. We could implement M2 and M3
     *   perfectly and QGC would offer nothing to press.
     * - **PX4 — the control UI, still fabricating nothing.**
     *   `PX4FirmwarePlugin::isCapable()` (`PX4FirmwarePlugin.cc:167-181`) grants
     *   `SetFlightMode | PauseVehicle | GuidedMode`, plus `ROI | ChangeHeading |
     *   Takeoff | GuidedTakeoff | Orbit` for a multirotor — and it reads
     *   **nothing** from the parameter set; it is a hard-coded list keyed only on
     *   vehicle type. The PX4 parameters we cannot supply (`SYS_AUTOSTART`,
     *   `CAL_*_ID`, `RC_MAP_*`) feed `setupComplete()`, which only colours setup
     *   pages red. So PX4 buys the operator interface at the price of a nagging
     *   config screen, not at the price of an invented measurement.
     *
     * The bill PX4 does present is a real one and it is paid in [Px4Mode]:
     * `PX4FirmwarePlugin::flightMode()` (`PX4FirmwarePlugin.cc:133-142`) has **no
     * `base_mode` fallback** — with `CUSTOM_MODE_ENABLED` clear it returns the
     * bare string `"Unknown"` and never looks further. So a DJI→PX4 `custom_mode`
     * mapping stopped being optional the moment we claimed this identity. See
     * [Px4Mode] for the table and for what happens to modes we cannot name.
     *
     * Note for whoever writes M2: RTL, Land and Takeoff are
     * `_setFlightModeAndValidate()` — QGC sends the mode and then *watches
     * `flightMode()` change to confirm* (`PX4FirmwarePlugin.cc:286-295`,
     * `:556-568`). A button therefore only completes if the DJI mode we read back
     * maps to the PX4 mode QGC asked for. Nothing here can fake that, and nothing
     * here should.
     */
    fun heartbeat(s: AircraftState, missionClaim: Long? = null): Heartbeat {
        // The PX4 packed main/sub encoding, or Px4Mode.UNMAPPED (0) when we have
        // no honest name for what DJI reported. Never a plausible-looking
        // substitute — see Px4Mode.UNMAPPED for why 0 and why the
        // CUSTOM_MODE_ENABLED bit stays set regardless.
        //
        // **The one override, and it is a report rather than an echo** (M4-1). `missionClaim` is
        // non-null only while all three of the executor's observed conjuncts hold — a plan is
        // committed, the executor is RUNNING, and DJI itself reports we hold virtual-stick
        // authority with a setpoint recently out. Under those conditions `AUTO.MISSION` is a true
        // description of what the aircraft is doing in PX4's own vocabulary, and without it QGC's
        // Start and Continue buttons cannot work at all: `PX4FirmwarePlugin::startMission` sends an
        // unacknowledged `SET_MODE` and then polls *this field* three times over 1.3 s.
        //
        // It is never set because a `SET_MODE` arrived — that ordering is the whole difference
        // between this and the echo `PLAN.md` forbids — and it is never latched: any conjunct
        // dropping reverts this to whatever DJI says, on the same tick.
        val custom = missionClaim ?: Px4Mode.customMode(s.flightMode)
        return Heartbeat.builder()
            .type(MavType.MAV_TYPE_QUADROTOR)
            .autopilot(MavAutopilot.MAV_AUTOPILOT_PX4)
            .baseMode(EnumValue.create<MavModeFlag>(*baseModeFlags(s, custom).toTypedArray()))
            .customMode(custom)
            .systemStatus(systemStatus(s))
            .mavlinkVersion(3)
            .build()
    }

    private fun systemStatus(s: AircraftState): MavState = when {
        // No flight controller means the aircraft is off (the RC alone cannot
        // report one). BOOT rather than STANDBY so QGC does not show a ready
        // aircraft that is not powered.
        !s.fcConnected -> MavState.MAV_STATE_BOOT
        s.motorsOn == true || s.isFlying == true -> MavState.MAV_STATE_ACTIVE
        // No MAV_STATE_CRITICAL branch on purpose: it wants KeyIsFailSafe /
        // KeyIsSeriousLowBatteryWarning, which AircraftState does not carry yet.
        // Inferring "critical" from the battery percentage would be inventing a
        // failsafe the aircraft never declared.
        else -> MavState.MAV_STATE_STANDBY
    }

    // ── EXTENDED_SYS_STATE ────────────────────────────────────────────────────

    /**
     * `EXTENDED_SYS_STATE` (245). QGC asks for this explicitly and repeatedly
     * (`MAV_CMD_SET_MESSAGE_INTERVAL p1=245`, observed live) and drives its
     * on-ground / in-air / landing indication from `landed_state`
     * (`Vehicle.cc:1037-1052`).
     *
     * The message id is [MESSAGE_ID_EXTENDED_SYS_STATE] if it needs registering
     * with the handshake layer's `registerMessageProvider`. It must be emitted
     * from exactly one place — this encoder is that place.
     */
    fun extendedSysState(s: AircraftState): ExtendedSysState = ExtendedSysState.builder()
        // "Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL
        // configuration" — a quadrotor never is.
        .vtolState(MavVtolState.MAV_VTOL_STATE_UNDEFINED)
        .landedState(landedState(s))
        .build()

    /** Message id of `EXTENDED_SYS_STATE`, for on-demand `MAV_CMD_REQUEST_MESSAGE`. */
    const val MESSAGE_ID_EXTENDED_SYS_STATE = 245

    /**
     * `landed_state` from `KeyIsFlying` / `KeyAreMotorsOn`, never from the flight
     * mode alone: on the ground with motors off the Mini 4 Pro still reports
     * `APAS` (measured 2026-07-25), so `FCFlightMode` cannot tell us whether it
     * is airborne.
     *
     * - flying → `LANDING` or `TAKEOFF` when the flight mode says so, else
     *   `IN_AIR`. Both refinements are gated on `isFlying == true` because QGC
     *   reads `LANDING`/`TAKEOFF` as *flying* (`Vehicle.cc:1042-1050`); claiming
     *   either off a stale mode string after touchdown would override an honest
     *   "on the ground".
     *
     *   Which modes count is taken from [Px4Mode] rather than from a second list
     *   here, so this and the heartbeat cannot disagree about what a landing is.
     *   That widened the set on the PX4 switch: `ATTI_LANDING`,
     *   `CONFIRM_LANDING`, `BASE_LANDING` and `BACKUP_LANDING` are real
     *   `FCFlightMode` landings that used to fall through to `IN_AIR`, and
     *   `ASSISTED_TAKE_OFF` / `TAKEOFF` / `QUICKTAKEOFF_ASSIST` / `PALM_LAUNCH`
     *   likewise for takeoff. It also deleted a dead branch: the old list tested
     *   for `"FORCE_LANDING"`, which is a member of DJI's *public* `FlightMode`
     *   enum and **not** of `FCFlightMode` — the enum `AircraftState.flightMode`
     *   actually carries — so it could never have matched.
     *
     *   `GO_HOME` is deliberately not a landing even though RTH ends in one: the
     *   match is on the exact PX4 target (`AUTO_LAND` / `AUTO_TAKEOFF`), and RTH
     *   maps to `AUTO_RTL`. An aircraft crossing a field homeward is not landing.
     * - not flying → `ON_GROUND`.
     * - `isFlying` unknown but motors confirmed **off** → `ON_GROUND`. A
     *   quadrotor with stopped motors is not flying; this is an inference from a
     *   known reading, not a guess at a missing one.
     * - anything else, including both readings absent → `UNDEFINED`, which is
     *   what the spec documents for "landed state is unknown" and which QGC
     *   ignores rather than acting on (`Vehicle.cc:1051`). Never `ON_GROUND` on
     *   no data: that is the reading that makes a GCS believe an airborne
     *   aircraft has landed.
     */
    fun landedState(s: AircraftState): MavLandedState = when {
        s.isFlying == true -> when (Px4Mode.customMode(s.flightMode)) {
            Px4Mode.AUTO_LAND -> MavLandedState.MAV_LANDED_STATE_LANDING
            Px4Mode.AUTO_TAKEOFF -> MavLandedState.MAV_LANDED_STATE_TAKEOFF
            else -> MavLandedState.MAV_LANDED_STATE_IN_AIR
        }
        s.isFlying == false -> MavLandedState.MAV_LANDED_STATE_ON_GROUND
        s.motorsOn == false -> MavLandedState.MAV_LANDED_STATE_ON_GROUND
        else -> MavLandedState.MAV_LANDED_STATE_UNDEFINED
    }

    /**
     * `base_mode`.
     *
     * ## `CUSTOM_MODE_ENABLED` is now unconditional, and that is the change
     *
     * Under the previous `MAV_AUTOPILOT_GENERIC` identity this bit was
     * deliberately **clear**, because QGC's base `FirmwarePlugin::flightMode()`
     * (`FirmwarePlugin.cc:47-81`) treats the branches as mutually exclusive:
     * claiming a custom mode replaced a readable "Manual Stabilize Guided"
     * summary with an unreadable `Custom:0x0`, since the only thing that can fill
     * `_modeEnumToString` for a generic vehicle is `AVAILABLE_MODES` (435), which
     * we cannot send (`StandardModes.cc:84`).
     *
     * **None of that survives the PX4 switch.** `PX4FirmwarePlugin::flightMode()`
     * (`PX4FirmwarePlugin.cc:133-142`) is:
     *
     * ```
     * if (base_mode & CUSTOM_MODE_ENABLED)
     *     return _modeEnumToString.value(custom_mode, "Unknown <base>:<custom>");
     * return "Unknown";
     * ```
     *
     * There is no flag-summary branch. Leaving the bit clear now buys the
     * operator the single word "Unknown" in every mode the aircraft can be in.
     * The bit is therefore always set, and the mode is carried by `custom_mode`
     * from [Px4Mode] — including for modes we cannot name, which is argued in
     * [Px4Mode.UNMAPPED].
     *
     * The `AVAILABLE_MODES` (435) work this used to be waiting on is **no longer
     * needed for the mode name**: PX4's plugin brings its own table. That removes
     * the dronefleet-1.1.11 problem (the message postdates the library) with it.
     *
     * ## The descriptive flags
     *
     * QGC reads none of these under PX4 — its display comes entirely from
     * `custom_mode`, and outside that function QGC reads only `SAFETY_ARMED`,
     * `HIL_ENABLED` and `CUSTOM_MODE_ENABLED` from `base_mode`. They are kept
     * because a real PX4 populates them, because other GCSs and the flight
     * recorder read them, and because each is a true statement:
     *
     * - `MANUAL_INPUT_ENABLED` ("remote control input is enabled") — always. The
     *   RC-N2 is physically cabled to the phone for the link to exist at all, and
     *   it can always take authority back: DJI documents the RC reclaiming
     *   control from virtual stick near a geofence.
     * - `STABILIZE_ENABLED` ("system stabilizes electronically its attitude (and
     *   optionally position)") — always. A Mini 4 Pro cannot fly unstabilized;
     *   DJI exposes no acro/rate mode. An unconditional true statement, not a
     *   discriminator.
     * - `CUSTOM_MODE_ENABLED` — always, as argued above.
     * - `SAFETY_ARMED` — from `motorsOn`.
     * - `GUIDED_ENABLED` / `AUTO_ENABLED` — derived from the mapped
     *   `custom_mode`, in [Px4Mode.extraBaseModeFlags], so they cannot contradict
     *   it. That replaced a second hand-written mode list that had drifted: it
     *   keyed on `"VIRTUAL_STICK"`, `"WAYPOINT"`, `"FORCE_LANDING"`, `"POI"`,
     *   `"SMART_FLIGHT"` and friends, which are members of DJI's *public*
     *   `FlightMode` enum and **not** of `FCFlightMode` — the enum
     *   `AircraftState.flightMode` actually carries — so those branches were
     *   unreachable.
     * - `TEST_ENABLED`, `HIL_ENABLED` — never.
     *
     * Note that **APAS is the steady-state ground reading** — measured 2026-07-25
     * parked, motors off, never flown — so QGC shows "Position" for an aircraft
     * sitting on the table. DJI has no idle/standby flight mode, which is why
     * `FCFlightMode` must not be used to infer "on the ground"; that is
     * [landedState]'s job, from `isFlying`/`motorsOn`.
     */
    fun baseModeFlags(
        s: AircraftState,
        /**
         * The `custom_mode` actually going out, so the derived flags cannot contradict it. Defaults
         * to the mapping from DJI's own reading, which is what it was before M4 gave the heartbeat
         * one conditional override.
         */
        customMode: Long = Px4Mode.customMode(s.flightMode),
    ): List<MavModeFlag> {
        val flags = ArrayList<MavModeFlag>(6)
        flags += MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
        flags += MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED
        flags += MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        // Armed follows motors, not isFlying: the Mini 4 Pro can spin motors on
        // the ground. Under PX4, QGC always decodes armed from this bit
        // (`Vehicle.cc:1285-1298`: the MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS
        // preference is guarded by `apmFirmware()`, which is false for us).
        if (s.motorsOn == true) flags += MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED
        flags += Px4Mode.extraBaseModeFlags(customMode)
        return flags
    }

    // ── SYS_STATUS ────────────────────────────────────────────────────────────

    /**
     * Sensors we claim to have, in `present` and `enabled`.
     *
     * **`MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS` is missing on purpose — do not
     * "complete" this list.** The direct reason is simply that we have no
     * motor-output status to report: nothing in the MSDK exposes per-ESC health.
     *
     * It used to carry a second, sharper hazard: on ArduPilot firmware QGC
     * prefers that bit over the heartbeat when deciding whether the vehicle is
     * armed (`Vehicle.cc:1285-1298`), so advertising it silently broke arm
     * detection. That branch is guarded by `apmFirmware()`, which is false under
     * both GENERIC and PX4 — verified again on the PX4 switch — so QGC always
     * takes armed from the heartbeat. The hazard is dormant, not gone: **it
     * returns the moment anyone reinstates the ArduPilot identity in
     * [heartbeat].**
     *
     * `MAV_SYS_STATUS_PREARM_CHECK` *is* here, deliberately — see
     * [prearmHealthy] for the whole argument, which is the most consequential
     * judgement call in this file.
     */
    private val sensorsPresent = sensors(
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_GYRO,
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_MAG,
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
        MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS,
        MavSysStatusSensor.MAV_SYS_STATUS_PREARM_CHECK,
    )

    fun sysStatus(s: AircraftState): SysStatus {
        // Health is built up from positive evidence only, never masked down from
        // "everything healthy".
        val healthy = ArrayList<MavSysStatusSensor>(6)
        if (s.fcConnected) {
            // Nothing at all is healthy on an aircraft that is not there. With
            // the FC up, the IMU/compass/barometer have no per-sensor fault key
            // in AircraftState, so they are claimed healthy — a compass-error
            // key exists in the MSDK and belongs here when it is wired through.
            healthy += MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_GYRO
            healthy += MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_ACCEL
            healthy += MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_3D_MAG
            healthy += MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            // GPS unhealthy while there is no usable fix. Level 0 is a real level
            // meaning "worst", null means "no information" — neither is a fix
            // worth claiming.
            if ((s.gpsSignalLevel ?: 0) >= 1) {
                healthy += MavSysStatusSensor.MAV_SYS_STATUS_SENSOR_GPS
            }
            if (prearmHealthy(s)) healthy += MavSysStatusSensor.MAV_SYS_STATUS_PREARM_CHECK
        }
        val health = sensors(*healthy.toTypedArray())
        return SysStatus.builder()
            .onboardControlSensorsPresent(sensorsPresent)
            .onboardControlSensorsEnabled(sensorsPresent)
            .onboardControlSensorsHealth(health)
            // "Battery voltage, UINT16_MAX: Voltage not sent by autopilot"
            .voltageBattery(s.voltageMv ?: UINT16_MAX)
            // "Battery current, -1: Current not sent by autopilot"
            .currentBattery(centiAmps(s.currentMa) ?: -1)
            // "Battery energy remaining, -1: … not sent by autopilot"
            .batteryRemaining(s.batteryPercent ?: -1)
            // No MSDK source for any of these. 0 is the documented "nothing to
            // report" for load/error counters, not a fabricated measurement.
            // drop_rate_comm would need AirLinkKey.KeySignalQuality, which
            // AircraftState does not carry.
            .load(0)
            .dropRateComm(0)
            .errorsComm(0)
            .errorsCount1(0)
            .errorsCount2(0)
            .errorsCount3(0)
            .errorsCount4(0)
            .build()
    }

    /**
     * Whether to advertise `MAV_SYS_STATUS_PREARM_CHECK` as **healthy**, i.e.
     * whether QGC shows a green "Ready".
     *
     * ## What QGC does with it
     *
     * `SYS_STATUS.onboard_control_sensors_enabled & PREARM_CHECK` latches
     * `readyToFlyAvailable`; the same bit in `_health` becomes `readyToFly`
     * (`Vehicle.cc:1082-1091`). Both are **display only** — no arm, takeoff or
     * mission command in QGC is gated on them (`readyToFly` has exactly one
     * consumer, `MainStatusIndicator.qml:86`). Setting the bit changes an
     * indicator, never an interlock, which is what makes this defensible at all.
     *
     * Because `present` is also set, QGC lists this by name ("Pre-Arm Check")
     * with its health in the sensor-status dropdown
     * (`SysStatusSensorInfo.cc:26` only walks bits that are *present*), so the
     * operator gets a named item rather than an unexplained colour.
     *
     * `MainStatusIndicator.qml:73-102` consults three sources, in order:
     * 1. `healthAndArmingCheckReport.supported` — PX4's events-protocol arming
     *    report. It becomes true only when an `EVENT` message actually delivers
     *    check results (`HealthAndArmingCheckReport.cc:42`), and we send none, so
     *    it stays **false** for us. Claiming PX4 does not by itself claim this.
     * 2. `readyToFlyAvailable` / `readyToFly` — our bit.
     * 3. `allSensorsHealthy && autopilotPlugin.setupComplete`.
     *
     * ## Why advertise it, given we cannot fully know
     *
     * The case against is real and must not be lost: **DJI's support has stated
     * it cannot always explain `KeyStartTakeoff` returning `-7`** (MSDK issue
     * #783). The aircraft can refuse to fly for reasons no key exposes, so this
     * signal is **necessary but not sufficient**: a green "Ready" can still be
     * followed by a takeoff that fails. What this bit reports is "no blocker that
     * we can see", not "this aircraft will fly".
     *
     * **The argument for reporting it has now inverted twice, and under PX4 it is
     * the strongest it has been.** Re-derived for this identity:
     *
     * - Under **ArduPilot**, path 3 was permanently *false*:
     *   `APMRadioComponent::setupComplete()` wants `RCMAP_*` and
     *   `RC#_MIN/MAX/TRIM` (`APMRadioComponent.cc:17-45`), which we can never
     *   supply. The bit was merely better than a stuck wrong answer.
     * - Under **GENERIC**, path 3 became permanently *true* and therefore
     *   dangerous: the only component was a `JoystickComponent`
     *   (`GenericAutoPilotPlugin.cc:10-17`) whose `setupComplete()` is
     *   `!activeJoystick() || calibrated` (`JoystickComponent.cc:31`) — true
     *   whenever no gamepad is attached. QGC would have shown a confident green
     *   built entirely out of our own sensor-presence claims. The bit was the
     *   only available brake.
     * - Under **PX4**, path 3 is permanently *false* again, for the same
     *   structural reason as ArduPilot but through different parameters.
     *   `PX4AutoPilotPlugin::vehicleComponents()` builds Airframe, Sensors,
     *   Radio, FlightModes, Power, Motor/Actuator, Safety and Tuning components
     *   (`PX4AutoPilotPlugin.cc:53-120`), and `AutoPilotPlugin::setupComplete` is
     *   the AND of all of them (`AutoPilotPlugin.cc:30-33`). Three can never pass
     *   against our four `BRG_*` parameters:
     *   `AirframeComponent::setupComplete()` needs `SYS_AUTOSTART != 0`
     *   (`AirframeComponent.cc:32-35`), `SensorsComponent::setupComplete()` needs
     *   every `CAL_*_ID` non-zero (`SensorsComponent.cc:41-48`), and
     *   `PX4RadioComponent::setupComplete()` needs `RC_MAP_ROLL/PITCH/YAW/THROTTLE`
     *   non-zero (`PX4RadioComponent.cc:31-44`). Supplying those means inventing
     *   calibration identifiers, which is the thing that killed ArduPilot.
     *
     * So under PX4 this bit is **the only way an operator can ever see a green
     * "Ready" for this aircraft — and equally the only thing that can honestly
     * withhold it.** Without it, QGC falls to path 3 and shows a permanent yellow
     * "Not Ready" that means nothing, in every state, forever; that is precisely
     * the indicator an operator learns to ignore.
     *
     * So: report it, from positive evidence only, and let it fail toward caution.
     *
     * Two alternatives were weighed and rejected:
     * - *Do not set the bit at all* — under PX4 this yields a permanent, constant
     *   "Not Ready". Not unsafe in the GENERIC sense (it never falsely goes
     *   green), but it destroys the indicator's information content, which is the
     *   same failure by a slower route.
     * - *Set `enabled` but keep `health` permanently false* — identical outcome
     *   to the above, with a claim ("this vehicle reports prearm status")
     *   attached to a constant that reports nothing.
     *
     * ## The rule
     *
     * Healthy **only** on positive evidence, every condition a `false`/known
     * value rather than an absence:
     * - flight controller connected (the aircraft is actually there), and
     * - `notAllowMotorStart == false` — DJI's own arming veto, and
     * - `imuWarmingUp == false`, and
     * - `inFailsafe == false`, and
     * - a 3D GPS fix by [fixType]'s standard.
     *
     * **Any `null` means not healthy.** An unknown blocker is a blocker; the
     * whole point is that "we have not heard" must never render as "Ready".
     *
     * The GPS condition is *stricter than DJI itself*, which will happily fly in
     * ATTI mode with no fix. That is intentional: this bridge exists so a GCS can
     * command position — goto, missions, RTH — and none of that can be honoured
     * without a position solution. A hand-flown indoor hover will therefore read
     * "Not Ready" while being perfectly flyable on the sticks; that is the
     * cautious direction, and the sensor dropdown names GPS as the reason.
     *
     * The one relaxation is the spec's own: PREARM_CHECK is documented "**Always
     * healthy when armed**" (`MavSysStatusSensor.java:180`). Motors running is
     * both what the spec means by armed and the strongest possible evidence that
     * DJI permitted the start, and without this the in-flight
     * `allSensorsHealthy` would drop to false and raise a sensor alarm that means
     * nothing — the same "teach them to ignore it" failure, in the air.
     *
     * **Do not "improve" this into an unconditional `true`.** If it ever needs to
     * be less conservative, the honest fix is to wire more real blockers
     * (compass error, calibration state, geofence/no-fly status, RC link) into
     * `AircraftState` and add them here, not to drop the conditions.
     */
    fun prearmHealthy(s: AircraftState): Boolean {
        if (!s.fcConnected) return false
        // "Always healthy when armed" — prearm checks no longer apply once the
        // motors are turning, and DJI having started them is itself the evidence.
        if (s.motorsOn == true || s.isFlying == true) return true
        return s.notAllowMotorStart == false &&
            s.imuWarmingUp == false &&
            s.inFailsafe == false &&
            fixType(s) == GpsFixType.GPS_FIX_TYPE_3D_FIX
    }

    // ── GPS_RAW_INT ───────────────────────────────────────────────────────────

    fun gpsRawInt(s: AircraftState, timeBootMs: Long): GpsRawInt {
        val pos = position(s)
        val speed = groundSpeedMs(s)
        return GpsRawInt.builder()
            // "Timestamp (UNIX Epoch time or time since system boot). The
            // receiving end can infer timestamp format … by checking for the
            // magnitude of the number." Time-since-boot keeps this function
            // clock-free and therefore testable.
            .timeUsec(BigInteger.valueOf(timeBootMs * 1000L))
            .fixType(fixType(s))
            // 0/0 is tolerable *here* and only here: this message carries fixType,
            // which is NO_GPS whenever the position is unknown, so the zero cannot be
            // mistaken for a fix. GLOBAL_POSITION_INT has no such disambiguator and
            // is suppressed instead.
            .lat(pos?.first ?: 0)
            .lon(pos?.second ?: 0)
            .alt(amslMm(s))
            // "GPS HDOP/VDOP … If unknown, set to: UINT16_MAX". We have no DOP
            // at all, so this is always unknown — 0 would claim perfect
            // precision.
            .eph(UINT16_MAX)
            .epv(UINT16_MAX)
            // "GPS ground speed. If unknown, set to: UINT16_MAX"
            .vel(speed?.let { cm(it) } ?: UINT16_MAX)
            .cog(courseOverGroundCdeg(s))
            // "Number of satellites visible. If unknown, set to UINT8_MAX"
            .satellitesVisible(s.satelliteCount ?: UINT8_MAX)
            // Extension fields: alt_ellipsoid and the *_acc accuracies have no
            // source and stay 0; yaw is 0, which the spec defines as "this GPS
            // does not provide yaw".
            .build()
    }

    /**
     * `GPSSignalLevel` (0..5 as measured) + satellite count → `GPS_FIX_TYPE_*`.
     *
     * Both directions of error hurt, so this is deliberately narrow:
     * **QGC discards any `GPS_RAW_INT` position below `GPS_FIX_TYPE_3D_FIX`**
     * (`Vehicle.cc:844`), so under-reporting hides the aircraft from the
     * operator; over-reporting hides a genuinely bad fix behind a confident
     * symbol on the map.
     *
     * The mapping, and why:
     * - `null` (LEVEL_NONE/UNKNOWN, or the key never fired) → `NO_GPS`. Not
     *   `NO_FIX`: we do not even know the receiver is reporting.
     * - level 0, 1 → `NO_FIX`. DJI's own UI treats these as unflyable.
     * - level 2 → `2D_FIX`. Marginal; enough to have a position, not enough to
     *   assert altitude, and QGC will not plot it. That is the correct outcome.
     * - level >= 3 → `3D_FIX`, *provided* at least [MIN_SATS_FOR_3D] satellites
     *   are visible. Levels 3-5 are DJI's "good enough to fly" band. The
     *   satellite floor is the guard: a 3D solution needs four satellites at
     *   minimum, six is a normal working figure, and `KeyGPSSignalLevel` updates
     *   more slowly than the satellite count, so a stale level with collapsing
     *   satellites is exactly the case where we must not keep claiming a fix.
     * - level >= 3 with an *unknown* satellite count → `3D_FIX`. Downgrading on
     *   a missing cross-check would blank the aircraft on the map over a
     *   telemetry gap, which is the worse failure.
     *
     * We never claim DGPS/RTK: nothing in the MSDK reports SBAS or RTK status,
     * and QGC would show a precision the airframe does not have.
     */
    fun fixType(s: AircraftState): GpsFixType {
        val level = s.gpsSignalLevel ?: return GpsFixType.GPS_FIX_TYPE_NO_GPS
        val sats = s.satelliteCount
        return when {
            level <= 1 -> GpsFixType.GPS_FIX_TYPE_NO_FIX
            level == 2 -> GpsFixType.GPS_FIX_TYPE_2D_FIX
            sats != null && sats < MIN_SATS_FOR_3D -> GpsFixType.GPS_FIX_TYPE_2D_FIX
            else -> GpsFixType.GPS_FIX_TYPE_3D_FIX
        }
    }

    /** Satellites required before a DJI signal level of 3+ is called a 3D fix. */
    const val MIN_SATS_FOR_3D = 6

    // ── GLOBAL_POSITION_INT ───────────────────────────────────────────────────

    /**
     * `GLOBAL_POSITION_INT`, or **null when the position is unknown**.
     *
     * `GLOBAL_POSITION_INT` has no sentinel for lat/lon, so an unknown position can
     * only be encoded as 0/0 — which is a *valid* coordinate in the Gulf of Guinea
     * and plots as a confident fix. `mavverify`'s `pos.nonzero` check caught us doing
     * exactly that on 2026-07-25, on every frame, whenever the aircraft was off.
     *
     * So we do what we already do for `HOME_POSITION` and `GPS_GLOBAL_ORIGIN`: say
     * nothing. A GCS showing no position is correct; one showing the wrong position
     * is the failure this layer exists to prevent.
     *
     * **The cost, and it is not free:** `relative_alt` rides in this message and is
     * QGC's `altitudeRelative` fact, so suppressing the message also suppresses
     * relative altitude. Altitude is *not* lost — `VFR_HUD.alt` carries AMSL at the
     * same 5 Hz and is emitted independently — but a GPS dropout in flight will take
     * the relative-altitude readout with it. That is the deliberate trade: a missing
     * number the operator can see is missing, over a wrong one they cannot.
     */
    fun globalPositionIntOrNull(s: AircraftState, timeBootMs: Long): GlobalPositionInt? =
        if (position(s) == null) null else globalPositionInt(s, timeBootMs)

    fun globalPositionInt(s: AircraftState, timeBootMs: Long): GlobalPositionInt {
        val pos = position(s)
        return GlobalPositionInt.builder()
            .timeBootMs(timeBootMs)
            .lat(pos?.first ?: 0)
            .lon(pos?.second ?: 0)
            // mm AMSL — note the unit difference against VFR_HUD.alt (metres).
            //
            // Unlike GPS_RAW_INT, a 0/0 here would be read as a real fix — which is
            // why this message is suppressed entirely when the position is unknown.
            // Callers must use globalPositionIntOrNull; this builder assumes a
            // position exists.
            .alt(amslMm(s))
            // mm above the takeoff point. This is QGC's `altitudeRelative` fact
            // (`VehicleFactGroup.cc:156`), read straight from this field for any
            // firmware, and the HUD's default altitude — so it matters more than
            // alt.
            .relativeAlt(s.relativeAltitude.finite()?.let { mm(it) } ?: ALT_UNKNOWN_MM)
            // NED straight through: vx north+, vy east+, vz down+. Same frame as
            // MSDK's Velocity3D, so no axis remap — only m/s → cm/s.
            .vx(cmOrZero(s.velocityNorth))
            .vy(cmOrZero(s.velocityEast))
            .vz(cmOrZero(s.velocityDown))
            // centi-degrees 0..35999; UINT16_MAX when unknown.
            .hdg(headingCdeg(s.yawDeg))
            .build()
    }

    // ── ATTITUDE ──────────────────────────────────────────────────────────────

    fun attitude(s: AircraftState, timeBootMs: Long): Attitude = Attitude.builder()
        .timeBootMs(timeBootMs)
        // Radians. Roll/pitch sign conventions are still UNVERIFIED against
        // MAVLink's Z-down/X-front frame (needs a tilt test) — this passes DJI's
        // degrees through unchanged, which is the assumption to re-check first if
        // QGC's HUD tilts the wrong way.
        .roll(radians(s.rollDeg))
        .pitch(radians(s.pitchDeg))
        // No wrapping: DJI's [-180,180] with 0 = north maps 1:1 onto MAVLink's
        // (-pi..+pi). Only GLOBAL_POSITION_INT.hdg and VFR_HUD.heading wrap.
        .yaw(radians(s.yawDeg))
        // No angular-rate source. Differentiating attitude to fake them would
        // feed QGC noise it draws as real.
        .rollspeed(0f)
        .pitchspeed(0f)
        .yawspeed(0f)
        .build()

    // ── VFR_HUD ───────────────────────────────────────────────────────────────

    fun vfrHud(s: AircraftState): VfrHud {
        val speed = groundSpeedMs(s)
        return VfrHud.builder()
            // The Mini 4 Pro has no airspeed sensor. Ground speed reported as
            // airspeed is a known lie, kept because QGC's HUD expects a number;
            // it is the same value as groundspeed, never an independent estimate.
            .airspeed(speed?.toFloat() ?: Float.NaN)
            .groundspeed(speed?.toFloat() ?: Float.NaN)
            // Integer degrees 0..359 here — NOT centi-degrees like
            // GLOBAL_POSITION_INT.hdg. Easiest field in the set to get wrong.
            .heading(headingDeg(s.yawDeg))
            // No collective/throttle key exists. 0 until GuidedController owns
            // the sticks and can report its own normalised verticalThrottle.
            .throttle(0)
            // **Metres**, and a float — not the millimetres of
            // GLOBAL_POSITION_INT.alt.
            .alt(amslMetres(s)?.toFloat() ?: Float.NaN)
            // MAVLink climb is positive up; NED z is positive down. Negate.
            .climb(s.velocityDown.finite()?.let { (-it).toFloat() } ?: Float.NaN)
            .build()
    }

    // ── BATTERY_STATUS ────────────────────────────────────────────────────────

    fun batteryStatus(s: AircraftState): BatteryStatus = BatteryStatus.builder()
        .id(0)
        .batteryFunction(MavBatteryFunction.MAV_BATTERY_FUNCTION_ALL)
        .type(MavBatteryType.MAV_BATTERY_TYPE_LIPO)
        // "Temperature of the battery. INT16_MAX for unknown temperature." cdegC.
        .temperature(s.batteryTempC.finite()?.let { round(it * 100.0) } ?: INT16_MAX)
        .voltages(cellVoltages(s))
        // cA, positive for draw. "-1: autopilot does not measure the current".
        .currentBattery(centiAmps(s.currentMa) ?: -1)
        // "Consumed charge, -1: autopilot does not provide consumption estimate".
        // Would need KeyFullChargeCapacity - KeyChargeRemaining, neither of which
        // AircraftState carries.
        .currentConsumed(-1)
        // "Consumed energy, -1: … does not provide energy consumption estimate"
        .energyConsumed(-1)
        // "Values: [0-100], -1: autopilot does not estimate the remaining"
        .batteryRemaining(s.batteryPercent ?: -1)
        // "0: autopilot does not provide remaining battery time estimate".
        // KeyTimeNeededToGoHome is a different quantity — do not substitute it.
        .timeRemaining(0)
        .chargeState(chargeState(s.batteryPercent))
        // voltages_ext (cells 11-14) left unset: there its 0 means "not
        // supported", the opposite of the voltages field's UINT16_MAX.
        .build()

    /**
     * Per-cell millivolts, ten entries.
     *
     * "Cells in this field above the valid cell count for this battery should
     * have the `UINT16_MAX` value. If individual cell voltages are unknown … the
     * overall battery voltage should be filled in cell 0, with all others set to
     * `UINT16_MAX`."
     *
     * Preference order:
     *
     * 1. **`cellVoltagesMv`, verbatim and in order** — real per-cell readings
     *    (measured `[4186, 4183]` on the 2S pack). These are passed through
     *    untouched precisely so **cell imbalance stays visible**: a spread
     *    between cells is a flight-safety signal, and any averaging erases it.
     *    The array's own length wins over [AircraftState.cellCount] if the two
     *    disagree — the readings are the measurement, `cellCount` is metadata,
     *    and neither padding a short array nor truncating a long one can be
     *    right. A mismatch is worth logging elsewhere; here it is data.
     * 2. Pack voltage split evenly across `cellCount` — an **estimate**, see
     *    [evenSplit].
     * 3. Pack voltage in cell 0, the spec's own fallback, when there is no cell
     *    count to split by.
     * 4. All `UINT16_MAX` when even the pack voltage is unknown.
     */
    private fun cellVoltages(s: AircraftState): List<Int> {
        val out = MutableList(10) { UINT16_MAX }
        val measured = s.cellVoltagesMv?.takeIf { it.isNotEmpty() }
        if (measured != null) {
            // Cells 1-10 live in this field; 11-14 would belong in voltages_ext,
            // which the Mini 4 Pro's 2S pack will never need.
            for ((i, mv) in measured.withIndex()) {
                if (i >= 10) break
                out[i] = mv.coerceIn(0, UINT16_MAX - 1)
            }
            return out
        }
        val pack = s.voltageMv ?: return out
        val cells = s.cellCount?.takeIf { it in 1..10 }
        if (cells == null) {
            out[0] = pack.coerceAtMost(UINT16_MAX - 1)
            return out
        }
        return evenSplit(pack, cells, out)
    }

    /**
     * Fallback only: pack millivolts divided evenly, remainder on the leading
     * cells so the entries still **sum to the pack voltage** (8371 mV on 2S gives
     * 4186 + 4185, against a real reading of 4186 + 4183).
     *
     * **These are an estimate, not measurements — by construction they cannot
     * show cell imbalance.** Kept rather than emitting all-`UINT16_MAX` because
     * the pack voltage is a genuine reading and a GCS that sums the cells to
     * derive pack voltage (or draws a per-cell bar) then gets a correct total
     * instead of nothing. The trade is that an even split *looks* like a
     * perfectly balanced pack. `SYS_STATUS.voltage_battery` carries the same pack
     * voltage unsplit, so nothing is lost by preferring this over silence, and a
     * reader who wants only measured per-cell data has `KeyCellVoltages`
     * populated in the normal case anyway.
     */
    private fun evenSplit(packMv: Int, cells: Int, out: MutableList<Int>): List<Int> {
        val base = packMv / cells
        val remainder = packMv % cells
        for (i in 0 until cells) out[i] = base + if (i < remainder) 1 else 0
        return out
    }

    /**
     * DJI gives no explicit charge state, so this is derived from the percentage
     * using the aircraft's own thresholds: 10% is DJI's documented default for
     * `KeySeriousLowBatteryWarningThreshold`, and 15% is the bottom of the
     * settable `KeyLowBatteryWarningThreshold` range [15,50] — i.e. the most
     * conservative "low" the aircraft can be configured with. Neither threshold
     * key is in `AircraftState`, so these are fixed placeholders. Unknown
     * percentage maps to `UNDEFINED`, whose documented meaning is "reporting not
     * supported" — never `OK`.
     */
    private fun chargeState(percent: Int?): MavBatteryChargeState = when {
        percent == null -> MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_UNDEFINED
        percent <= 10 -> MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_CRITICAL
        percent <= 15 -> MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_LOW
        else -> MavBatteryChargeState.MAV_BATTERY_CHARGE_STATE_OK
    }

    // ── HOME_POSITION / GPS_GLOBAL_ORIGIN ─────────────────────────────────────

    /**
     * Null until home *and* its AMSL datum are both known — see the class doc.
     * A home position is what QGC draws the home symbol from and what
     * altitude-above-home arithmetic is measured against, so a placeholder here
     * is worse than no message.
     */
    fun homePosition(s: AircraftState): HomePosition? {
        val (lat, lon) = homeCoordinate(s) ?: return null
        val altM = s.takeoffAltitudeAmsl.finite() ?: return null
        return HomePosition.builder()
            .latitude(degE7(lat))
            .longitude(degE7(lon))
            // mm AMSL. Barometric and drifting ~0.7 m while stationary, so this
            // is a datum, not a survey.
            .altitude(mm(altM))
            // Home is the local-frame origin, so its NED offset is zero, and the
            // ground has no measured slope to report.
            .x(0f)
            .y(0f)
            .z(0f)
            .q(listOf(1f, 0f, 0f, 0f))
            .approachX(0f)
            .approachY(0f)
            .approachZ(0f)
            .timeUsec(BigInteger.ZERO)
            .build()
    }

    /**
     * Same source and the same all-or-nothing rule as [homePosition]: DJI's
     * takeoff point is the only origin we have, and there is no separate EKF
     * origin to report.
     */
    fun gpsGlobalOrigin(s: AircraftState): GpsGlobalOrigin? {
        val (lat, lon) = homeCoordinate(s) ?: return null
        val altM = s.takeoffAltitudeAmsl.finite() ?: return null
        return GpsGlobalOrigin.builder()
            .latitude(degE7(lat))
            .longitude(degE7(lon))
            .altitude(mm(altM))
            .timeUsec(BigInteger.ZERO)
            .build()
    }

    // ── conversions ───────────────────────────────────────────────────────────

    /**
     * Metres AMSL, or null when unknown.
     *
     * **`relativeAltitude` is takeoff-relative** — measured 0 on the ground with
     * the site at ~103 m — so AMSL is only meaningful as
     * `takeoffAltitudeAmsl + relativeAltitude`. Either half missing means the sum
     * is unknown; adding a guessed datum to a relative altitude and calling it
     * AMSL is the mistake RosettaDrone shipped.
     */
    fun amslMetres(s: AircraftState): Double? {
        val takeoff = s.takeoffAltitudeAmsl.finite() ?: return null
        val relative = s.relativeAltitude.finite() ?: return null
        return takeoff + relative
    }

    private fun amslMm(s: AircraftState): Int =
        amslMetres(s)?.let { mm(it) } ?: ALT_UNKNOWN_MM

    /**
     * lat/lon in degE7, or null if the pair is not a coordinate we will repeat —
     * see [Geo]. All-or-nothing: a valid latitude beside a zeroed longitude is a
     * position QGC *would* plot, in the Gulf of Guinea.
     *
     * `StateCache` applies the same [Geo] rule at the seam where DJI values are
     * unwrapped, so in the live bridge these two coordinates are already known
     * good. Re-checking here is deliberate: this function is the last thing
     * between a `Double` and a number on a ground station's map, and it is also
     * where a unit test can prove that an out-of-range coordinate produces no
     * message. Nothing downstream re-validates.
     */
    private fun position(s: AircraftState): Pair<Int, Int>? {
        val (lat, lon) = Geo.coordinateOrNull(s.latitude, s.longitude) ?: return null
        return degE7(lat) to degE7(lon)
    }

    /**
     * Home lat/lon in degrees, or null when we do not have a home point.
     *
     * **Two independent gates, and neither is redundant.** They answer different
     * questions and they fail in different situations, so deleting either one
     * re-opens a hole:
     *
     * 1. **[AircraftState.homeLocationSet] == false is decisive.** That is DJI's
     *    own `KeyIsHomeLocationSet` saying there is no home point, and it outranks
     *    anything the coordinates look like — a plausible-looking pair beside a
     *    `false` here is a stale or invented one, not a home. Note the asymmetry:
     *    only `false` blocks. `null` means the key has never been delivered, which
     *    is not evidence of anything, and `true` is not a licence to skip gate 2.
     * 2. **[Geo]'s coordinate validation always runs.** It is what catches DJI
     *    handing us garbage in any situation this key does not cover — including
     *    the one where `KeyIsHomeLocationSet` has not arrived yet, which on
     *    2026-07-26 was the first ~2.8 s of every session, and any future firmware
     *    that fills the coordinate in without setting the flag.
     */
    fun homeCoordinate(s: AircraftState): Pair<Double, Double>? {
        if (s.homeLocationSet == false) return null
        return Geo.coordinateOrNull(s.homeLatitude, s.homeLongitude)
    }

    private fun groundSpeedMs(s: AircraftState): Double? {
        val n = s.velocityNorth.finite() ?: return null
        val e = s.velocityEast.finite() ?: return null
        return hypot(n, e)
    }

    /**
     * Course over ground in centi-degrees — the direction of *travel*, not the
     * heading. Below [COG_MIN_SPEED_MS] the direction is numerically meaningless,
     * and the spec's own "If unknown, set to: UINT16_MAX" is the honest answer.
     */
    private fun courseOverGroundCdeg(s: AircraftState): Int {
        val n = s.velocityNorth.finite() ?: return UINT16_MAX
        val e = s.velocityEast.finite() ?: return UINT16_MAX
        if (hypot(n, e) < COG_MIN_SPEED_MS) return UINT16_MAX
        return cdeg(Math.toDegrees(Math.atan2(e, n)))
    }

    /** Ground speed below which course over ground is noise. */
    const val COG_MIN_SPEED_MS = 0.5

    /** Wraps a signed heading into [0,360). */
    private fun wrap360(deg: Double): Double {
        val w = deg % 360.0
        return if (w < 0.0) w + 360.0 else w
    }

    /** Centi-degrees in 0..35999, as `GLOBAL_POSITION_INT.hdg` wants. */
    private fun cdeg(deg: Double): Int {
        // The modulo guards the boundary: 359.999 deg rounds to 36000 cdeg,
        // which is out of the documented 0..35999 range.
        val v = round(wrap360(deg) * 100.0) % 36000
        return if (v < 0) v + 36000 else v
    }

    private fun headingCdeg(yawDeg: Double?): Int =
        yawDeg.finite()?.let { cdeg(it) } ?: UINT16_MAX

    /**
     * Integer degrees in 0..359 for `VFR_HUD.heading`. Truncated, not rounded:
     * rounding 359.6 would produce 360, which the field's "0-360, 0=north" does
     * not allow. QGC truncates the same way for its own heading fact
     * (`VehicleFactGroup.cc:124`).
     */
    private fun headingDeg(yawDeg: Double?): Int {
        val deg = yawDeg.finite() ?: return UINT16_MAX
        return floor(wrap360(deg)).toInt() % 360
    }

    private fun radians(deg: Double?): Float =
        deg.finite()?.let { Math.toRadians(it).toFloat() } ?: Float.NaN

    /** DJI mA (negative = discharging) → MAVLink cA (positive = draw). */
    private fun centiAmps(currentMa: Int?): Int? {
        if (currentMa == null) return null
        // Rounded rather than truncated: -905 mA is 90.5 cA, and 91 is nearer
        // the truth than 90. A genuine 10 mA charge current would encode as -1,
        // colliding with the "not measured" sentinel — harmless, and not worth
        // distorting the scale to avoid.
        return round(-currentMa / 10.0)
    }

    private fun degE7(deg: Double): Int = round(deg * 1e7)

    /** Metres → millimetres, saturating short of [ALT_UNKNOWN_MM]. */
    private fun mm(metres: Double): Int {
        val v = Math.round(metres * 1000.0)
        return when {
            v > Int.MAX_VALUE.toLong() -> Int.MAX_VALUE
            // Never collide with the unknown-altitude sentinel.
            v <= Int.MIN_VALUE.toLong() -> Int.MIN_VALUE + 1
            else -> v.toInt()
        }
    }

    /** m/s → cm/s. */
    private fun cm(metresPerSecond: Double): Int = round(metresPerSecond * 100.0)

    /**
     * m/s → cm/s, with 0 for unknown. MAVLink defines no sentinel for the int16
     * velocity fields; 0 is bounded and reads as "stationary", which cannot draw
     * the aircraft anywhere it is not.
     */
    private fun cmOrZero(metresPerSecond: Double?): Int =
        metresPerSecond.finite()?.let { cm(it) } ?: 0

    private fun round(v: Double): Int {
        val r = Math.round(v)
        return when {
            r > Int.MAX_VALUE.toLong() -> Int.MAX_VALUE
            r < Int.MIN_VALUE.toLong() -> Int.MIN_VALUE
            else -> r.toInt()
        }
    }

    /** NaN/infinite readings are as absent as nulls; never encode them as data. */
    private fun Double?.finite(): Double? = this?.takeIf { it.isFinite() }

    private fun sensors(vararg flags: MavSysStatusSensor): EnumValue<MavSysStatusSensor> =
        EnumValue.create(*flags)
}
