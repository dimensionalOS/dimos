package com.dimensional.mini4pro.handshake

import com.dimensional.mini4pro.telemetry.AircraftState
import io.dronefleet.mavlink.common.MavParamType

/**
 * One MAVLink parameter as we present it to the GCS.
 *
 * [value] is the parameter's *numeric* value, held as a float for uniformity — `1f` in an
 * `INT32` parameter means the integer 1. It is **not** the bit pattern that goes on the wire;
 * [ParamCodec] does that conversion at the boundary.
 *
 * @param writable whether this parameter is *conceptually* settable. A write is only ever
 *   applied if a writer has also been registered with the responder (see
 *   [HandshakeResponder.registerParameterWriter]) and that writer accepts the value — so the
 *   default build rejects every write rather than pretending one took effect.
 * @param doc why this parameter exists. Kept in-band so the reason cannot drift away from
 *   the definition.
 */
data class Parameter(
    val name: String,
    val type: MavParamType,
    val value: Float,
    val writable: Boolean = false,
    val doc: String,
) {
    /** The `param_value` float field as it must appear on the wire. See [ParamCodec]. */
    fun wireValue(): Float = ParamCodec.encode(type, value)
}

/**
 * Byte-wise encoding of parameter values into the `param_value` float field.
 *
 * <https://mavlink.io/en/services/parameter.html#parameter-encoding>
 *
 * MAVLink has two mutually exclusive conventions for stuffing a non-float parameter into a
 * float field, and which one applies is a property of the *vehicle*, declared through
 * `MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE` / `_C_CAST`:
 *
 *  - **bytewise** (the MAVLink default, what we do): the value's bytes are reinterpreted as a
 *    float. Integer 1 travels as the float whose bit pattern is `0x00000001`.
 *  - **C cast**: the value is converted to a float numerically, so integer 1 travels as `1.0f`.
 *    This is ArduPilot's convention.
 *
 * QGroundControl implements bytewise natively: it copies `param_value` into
 * `mavlink_param_union_t.param_float` and then reads the union member for the declared type
 * (`ParameterManager::_mavlinkParamUnionToVariant`), i.e. a bit reinterpretation. `PARAM_SET`
 * goes out the same way (`_fillMavlinkParamUnion`). The only reason the C-cast convention ever
 * worked was `APMFirmwarePlugin::_handleIncomingParamValue` /
 * `_handleOutgoingParamSetThreadSafe`, which translated in both directions for vehicles
 * claiming `MAV_AUTOPILOT_ARDUPILOTMEGA`.
 *
 * We identify as `MAV_AUTOPILOT_PX4`, and PX4 reintroduces no such translation:
 * `PX4FirmwarePlugin::adjustIncomingMavlinkMessage` switches on `AUTOPILOT_VERSION` and nothing
 * else (`PX4FirmwarePlugin.cc:640-654`), and PX4 does not override
 * `adjustOutgoingMavlinkMessageThreadSafe` at all — `FirmwarePlugin.h:289` is an empty base and
 * `APMFirmwarePlugin.h:51` is its only override in the tree. So bytewise remains the only
 * correct choice. Getting this backwards is silent: QGC would render our `BRG_TLM_HZ = 1` as
 * `1.4e-45` (bytewise sent, float read) or `1065353216` (C-cast sent, bytewise read).
 *
 * **Confirmed visually on 2026-07-25**, which had never been done before: with a PX4-identifying
 * vehicle sending these four `INT32` parameters bytewise, QGroundControl 5.0.8's parameter editor
 * displays them as plain `0` and `1`. The bug found while switching off ArduPilot is fixed in
 * fact, not only in test.
 *
 * 8- and 16-bit types occupy the low bytes; the remaining bytes are not meaningful, so we zero
 * them and mask on the way back in.
 */
object ParamCodec {

    /** Numeric [value] → the float field on the wire. */
    fun encode(type: MavParamType, value: Float): Float = when (type) {
        MavParamType.MAV_PARAM_TYPE_REAL32 -> value
        MavParamType.MAV_PARAM_TYPE_INT32, MavParamType.MAV_PARAM_TYPE_UINT32 ->
            Float.fromBits(value.toLong().toInt())
        MavParamType.MAV_PARAM_TYPE_INT16, MavParamType.MAV_PARAM_TYPE_UINT16 ->
            Float.fromBits(value.toInt() and 0xFFFF)
        MavParamType.MAV_PARAM_TYPE_INT8, MavParamType.MAV_PARAM_TYPE_UINT8 ->
            Float.fromBits(value.toInt() and 0xFF)
        else -> throw IllegalArgumentException("$type cannot be carried in PARAM_VALUE.param_value")
    }

    /** The float field off the wire → numeric value. */
    fun decode(type: MavParamType, wire: Float): Float {
        val bits = wire.toRawBits()
        return when (type) {
            MavParamType.MAV_PARAM_TYPE_REAL32 -> wire
            MavParamType.MAV_PARAM_TYPE_INT32 -> bits.toFloat()
            MavParamType.MAV_PARAM_TYPE_UINT32 -> (bits.toLong() and 0xFFFFFFFFL).toFloat()
            MavParamType.MAV_PARAM_TYPE_INT16 -> (bits and 0xFFFF).toShort().toFloat()
            MavParamType.MAV_PARAM_TYPE_UINT16 -> (bits and 0xFFFF).toFloat()
            MavParamType.MAV_PARAM_TYPE_INT8 -> (bits and 0xFF).toByte().toFloat()
            MavParamType.MAV_PARAM_TYPE_UINT8 -> (bits and 0xFF).toFloat()
            else -> throw IllegalArgumentException("$type cannot be carried in PARAM_VALUE.param_value")
        }
    }

    /** False for the 64-bit types, which do not fit the parameter protocol's float field. */
    fun isCarryable(type: MavParamType): Boolean = when (type) {
        MavParamType.MAV_PARAM_TYPE_INT64,
        MavParamType.MAV_PARAM_TYPE_UINT64,
        MavParamType.MAV_PARAM_TYPE_REAL64 -> false
        else -> true
    }
}

/**
 * A parameter whose value is a fact about the **aircraft**, read from `KeyManager`, rather than
 * a fact about this bridge.
 *
 * The distinction is the whole reason this type exists separately from [Parameter]. A `BRG_*`
 * value is something we know by construction and can state at any time; one of these is only
 * publishable **once DJI has actually said it**. [read] returning null means DJI has not, and
 * the parameter is then left out of the table entirely rather than being given a plausible
 * default — see [ParameterStore.forAircraft].
 *
 * [read] is called again on every read of the parameter, so an operator who changes the setting
 * in DJI Fly mid-session sees the new value rather than the one that happened to be current at
 * connect. If a later read comes back null the previously delivered value is kept: these are
 * *settings*, and DJI going quiet about a setting does not change what the aircraft is
 * configured to do.
 *
 * @param source the `KeyManager` key this comes from. Named so the log line for an omitted
 *   parameter says which key never arrived, which is the only thing that would explain it.
 */
class AircraftParameter(
    val name: String,
    val type: MavParamType,
    val source: String,
    val doc: String,
    val read: (AircraftState) -> Float?,
)

/**
 * The parameter table QGroundControl downloads during its connect sequence.
 *
 * QGC's `ParameterManager` needs the set to be non-empty and self-consistent: every
 * `PARAM_VALUE` carries the same `param_count`, indices are 0-based and dense, and each index
 * appears exactly once. It builds its wait list from the first `param_count` it sees and only
 * declares parameters ready when every index has arrived, so a count/index disagreement makes
 * it re-request forever (`ParameterManager.cc:_handleParamValue`, `_checkInitialLoadComplete`).
 *
 * Naming: facts about the bridge live in a `BRG_` namespace, and a real flight-controller name
 * is published **only when the aircraft itself supplies the value and the two names mean the
 * same thing**. We are not a flight controller: there is no `RCMAP_ROLL`, no `COMPASS_OFS_X`,
 * no `INS_ACCOFFS_Y` on a DJI aircraft that we could read, and a GCS that finds those names
 * expects to be able to calibrate against them. Borrowing a name also re-attaches that
 * firmware's semantics in any GCS that recognises it, which is exactly the trap the autopilot
 * masquerade turned out to be — so a borrowed name is a claim about *meaning* as well as about
 * value, and both halves have to hold. [AIRCRAFT_PARAMETERS] carries the argument for each one
 * that passed; [FORBIDDEN_PARAMETERS] lists the names that must never appear even by accident,
 * and [PX4_MISSING_PARAMETERS] records the ones QGC asks for that did not pass and why.
 *
 * ## The set is fixed when the store is built, and never grows
 *
 * `PARAM_VALUE.param_count` must be the same in every message and the indices must be dense:
 * QGC builds its wait list from the first count it sees and `_checkInitialLoadComplete` removes
 * entries **by index**. So an aircraft-derived parameter whose key DJI has not delivered by the
 * time the store is built is **omitted for the whole link session** and logged — never a
 * placeholder value, and never appended later. Adding a parameter mid-session would renumber
 * nothing but would raise the count, and every already-downloaded index would then be one short
 * of the total forever.
 *
 * The cost of that rule is a session in which `RTL_RETURN_ALT` is simply absent, which is the
 * same thing QGC already copes with for the other 29 names it asks for. The cost of the
 * alternative is a number in front of an operator that no aircraft ever stated.
 */
class ParameterStore(
    entries: List<Parameter>,
    /**
     * name → how to re-read that parameter from the aircraft, for the subset of [entries] that
     * came from [AIRCRAFT_PARAMETERS]. Every key must name a declared parameter; a reader for a
     * parameter that is not in the table would be a value nobody could ever see.
     */
    private val readers: Map<String, (AircraftState) -> Float?> = emptyMap(),
    /** The live aircraft state. Null in the bridge-only build, where nothing needs it. */
    private val state: (() -> AircraftState)? = null,
) {

    companion object {
        /** `PARAM_VALUE.param_id` / `PARAM_SET.param_id` is 16 bytes, not NUL-terminated. */
        const val MAX_NAME_LENGTH = 16

        /**
         * **Measured, 2026-07-25, QGroundControl 5.0.8.** The exact parameters QGC reported
         * missing for a `MAV_AUTOPILOT_PX4` / `MAV_TYPE_QUADROTOR` vehicle publishing only the
         * four `BRG_*` values below, in the order the dialog listed them:
         *
         * > Parameters are missing from firmware. You may be running a version of firmware which
         * > is not fully supported or your firmware has a bug in it. Missing params: …
         *
         * Read from a screenshot of the dialog. QGC's `showAppMessage` writes the text to the log
         * only when QGC is running its own unit tests (`QGCApplication.cc:431-446`), and the C++
         * `ParameterManager::getParameter` → `reportMissingParameter` path (`:792-803`,
         * `QGCApplication.cc:368-376`) logs nothing at all, so the dialog is the only place this
         * list exists.
         *
         * **How much of it you get depends on which view is open, and that is worth knowing
         * before judging the identity.** QGC accumulates a name each time
         * `ParameterManager::getParameter` or `FactPanelController::getParameterFact` is called
         * for a name we do not publish; `parameterExists()` is silent. On a bare connect the only
         * C++ caller is `AutoPilotPlugin::_recalcSetupComplete`, which **breaks at the first
         * incomplete component** (`AutoPilotPlugin.cc:30-32`) — and `AirframeComponent` sorts
         * first, so it stops at `SYS_AUTOSTART` (`AirframeComponent.cc:32-35`). The other 29
         * names come from the Setup view's summary pages, which bind every component at once
         * (`VehicleSummary.qml:86-144`) using `getParameterFact`'s `reportMissing = true` default
         * (`FactPanelController.h:25`). Ivan confirmed on the day: the large list appears once
         * the parameter/setup page is opened, and he was unsure whether a smaller one had already
         * appeared on the Fly view. So treat this list as "everything QGC will ever ask for",
         * and [PX4_MISSING_ON_BARE_CONNECT] as what a connect alone costs.
         *
         * This dialog comes paired with *"One or more vehicle components require setup prior to
         * flight."* (measured wording; `AutoPilotPlugin.cc:59-61` in QGC master phrases it
         * differently), which is raised whenever setup is incomplete. `GenericAutoPilotPlugin`
         * showed neither, because its only component is a `JoystickComponent` that reports
         * complete with no gamepad attached.
         *
         * Together with the two dialogs caused by [AutopilotIdentity]'s `flightSwVersion = 0`,
         * **a PX4 connect raises four modals**. That is the real, measured price of the identity —
         * not a blocked capability, but four things to dismiss. Weigh it against the Fly-view
         * controls it buys, and do not pay it down by inventing parameters.
         *
         * **Read this before deciding PX4 was free.** These are the same *class* of data that got
         * the ArduPilot identity reversed: sensor calibration ids and RC channel mappings. Two
         * things make it a different bargain, and both were checked rather than assumed:
         *
         *  1. **Nothing is gated.** `PX4FirmwarePlugin::isCapable` (`PX4FirmwarePlugin.cc:167-181`)
         *     is driven purely by `MAV_TYPE` and reads no parameter, and
         *     `GuidedActionsController.qml` never consults `autopilotPlugin.setupComplete` at all.
         *     Missing parameters change a toolbar label's colour, a red dot on the summary page,
         *     `ConfigButton.qml:5`'s icon colour, and — via `PX4AutoPilotPlugin::prerequisiteSetup`
         *     (`:171-207`) — whether the *configuration pages* are reachable. No Fly-view button,
         *     no arming path and no part of the connect sequence depends on them. Under ArduPilot,
         *     `APMRadioComponent::setupComplete` genuinely required `RCMAP_*` > 0.
         *  2. **Publishing some would not help.** The connect-time check is a cascade: satisfying
         *     `SYS_AUTOSTART` advances it to `COM_RC_IN_MODE` and `RC_MAP_ROLL`, then to
         *     `CAL_GYRO0_ID`, then `CAL_ACC0_ID`, then `CAL_MAG0_ID`. Silencing it needs the whole
         *     calibration set to be present *and non-zero* — i.e. exactly the fabrication we
         *     refused. There is no partial credit, so there is no slippery slope to start down.
         *
         * Nothing here may be published. [PX4_CALIBRATION_PARAMETERS] enforces that for the subset
         * whose value we could never know truthfully.
         */
        val PX4_MISSING_PARAMETERS: List<String> = listOf(
            "SYS_AUTOSTART", "CAL_GYRO0_ID", "COM_RC_IN_MODE", "RC_MAP_ROLL", "SYS_AUTOCONFIG",
            "MAV_SYS_ID", "CAL_ACC0_ID", "CAL_MAG2_ID", "CAL_MAG1_ID", "CAL_MAG0_ID",
            "RC_MAP_AUX2", "RC_MAP_AUX1", "RC_MAP_FLAPS", "RC_MAP_THROTTLE", "RC_MAP_YAW",
            "RC_MAP_PITCH", "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3", "COM_FLTMODE4",
            "COM_FLTMODE5", "COM_FLTMODE6", "BAT1_SOURCE", "RTL_LAND_DELAY", "RTL_DESCEND_ALT",
            "RTL_RETURN_ALT", "NAV_DLL_ACT", "COM_RC_LOSS_T", "NAV_RCL_ACT", "COM_LOW_BAT_ACT",
        )

        /**
         * The one name QGC reports on a bare connect, before any setup page is opened —
         * `AutoPilotPlugin.cc:30-32` breaks at the first incomplete component and
         * `AirframeComponent` sorts first. Source-derived, not separately measured.
         */
        val PX4_MISSING_ON_BARE_CONNECT = "SYS_AUTOSTART"

        /**
         * The subset of [PX4_MISSING_PARAMETERS] that can **never** be published truthfully,
         * whatever we later learn from the DJI SDK. Every one of these asserts a fact about
         * flight-controller hardware that does not exist on this aircraft:
         *
         *  - `CAL_*_ID` — the device id of a calibrated gyro/accel/magnetometer. A non-zero
         *    value is a claim that *that sensor was calibrated through this interface*. DJI
         *    exposes no such thing, and QGC's sensor page acts on these.
         *  - `RC_MAP_*` — which RC channel drives which axis. There is no RC channel mapping to
         *    read: the RC-N2 talks to the aircraft, not to us, and the radio calibration page
         *    would write back into a mapping that controls nothing.
         *  - `SYS_AUTOSTART` / `SYS_AUTOCONFIG` — the PX4 airframe id. Choosing one tells QGC
         *    the aircraft is a specific PX4 airframe with that airframe's mixer and limits.
         *  - `COM_FLTMODE1..6` / `COM_RC_IN_MODE` — the switch-to-flight-mode assignment. DJI's
         *    mode switch is not ours to describe, and a mapping here changes what QGC believes
         *    a stick flick will do.
         *
         * Silencing a dialog is never a reason to publish one of these.
         *
         * Deliberately **not** here, and therefore merely absent rather than banned: the rest of
         * [PX4_MISSING_PARAMETERS]. `RTL_RETURN_ALT`, `RTL_DESCEND_ALT`, `RTL_LAND_DELAY`,
         * `BAT1_SOURCE`, `NAV_RCL_ACT`, `NAV_DLL_ACT`, `COM_RC_LOSS_T`, `COM_LOW_BAT_ACT` and
         * `MAV_SYS_ID` all describe behaviour a DJI aircraft genuinely has and that `KeyManager`
         * may one day report — RTH altitude most obviously. Publishing one of those is legitimate
         * *once it is read from the aircraft*, and only then. It stays wrong to seed them with
         * plausible defaults.
         *
         * **`RTL_RETURN_ALT` has since made that trip** and now lives in [AIRCRAFT_PARAMETERS],
         * read from `FlightControllerKey.KeyGoHomeHeight`. The others were surveyed at the same
         * time and each was rejected on *meaning*, not on availability — see
         * [AIRCRAFT_PARAMETERS] for the mismatch that killed each one. That survey is the
         * standard: "DJI has a key with a similar name" is not the test.
         */
        val PX4_CALIBRATION_PARAMETERS: Set<String> = setOf(
            "SYS_AUTOSTART", "SYS_AUTOCONFIG",
            "CAL_GYRO0_ID", "CAL_ACC0_ID", "CAL_MAG0_ID", "CAL_MAG1_ID", "CAL_MAG2_ID",
            "RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_YAW", "RC_MAP_THROTTLE",
            "RC_MAP_FLAPS", "RC_MAP_AUX1", "RC_MAP_AUX2",
            "COM_RC_IN_MODE",
            "COM_FLTMODE1", "COM_FLTMODE2", "COM_FLTMODE3",
            "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6",
        )

        /**
         * Parameter names whose mere *presence* changes how a GCS interprets the vehicle.
         * Never publish one of these, whatever value we could put in it.
         *
         * `_HASH_CHECK` — **PX4's parameter-cache handshake, and the most dangerous name on
         * this list now that we identify as `MAV_AUTOPILOT_PX4`.** QGC intercepts any
         * `PARAM_VALUE` with this name before its normal bookkeeping
         * (`ParameterManager.cc:145-152`) and treats the value as a CRC32 over the vehicle's
         * whole parameter set. Two independent failures follow from publishing it:
         *
         *  1. **The download never completes.** That branch `return`s before the wait-list
         *     bookkeeping at `:196`, so the index we assigned it is never removed from
         *     `_waitingReadParamIndexMap` and `_checkInitialLoadComplete` never fires — QGC
         *     re-requests forever.
         *  2. **QGC may display another vehicle's parameters as ours.** On receiving a hash QGC
         *     calls `_tryCacheHashLoad` (`:1078-1200`), which CRC32s the on-disk cache file
         *     `ParamCache/<vehicleId>_<componentId>.v2` — for us `1_1.v2` — and, on a match,
         *     loads every parameter *from that file* and tells us to stop sending
         *     (`PARAM_SET _HASH_CHECK`, `:1141-1157`). The cache is keyed only on sysid/compid,
         *     so any real PX4 vehicle previously connected as system 1 component 1 shares the
         *     file. A hash we invented has no business colliding with it, and if it ever did the
         *     operator would be shown a parameter set the aircraft does not have.
         *
         * The correct behaviour is silence, which is what [HandshakeResponder] already does with
         * a `PARAM_REQUEST_READ` for a name it does not know. Measured cost of that silence:
         * QGC master asks for `_HASH_CHECK` explicitly (`ParameterManager.cc:658-666`) and falls
         * back to `PARAM_REQUEST_LIST` after `kHashCheckTimeoutMs` = 1000 ms
         * (`ParameterManager.h:113`, `:1362-1372`). QGC 5.0.8 — the build actually on the
         * laptop — does not ask at all; measured, it goes straight to `PARAM_REQUEST_LIST`.
         * Either way one second is the worst case and nothing is fabricated.
         *
         * `ARMING_REQUIRE` — the armed-state landmine. QGC reads it in
         * `Vehicle::_apmArmingNotRequired()`: a value of 0 means "always armed, but the motors
         * may be unpowered", at which point QGC stops trusting the heartbeat for armed state
         * and derives it from `SYS_STATUS`'s `MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS` bit
         * (`Vehicle.cc:1119`, `Vehicle.cc:1291`).
         *
         * Both of those call sites are guarded by `apmFirmware()`, so under `MAV_AUTOPILOT_PX4`
         * the parameter is inert — the heartbeat always drives armed state (`Vehicle.cc:1296`).
         * It stays on this list for two reasons: it costs nothing, and anyone who re-enables an
         * ArduPilot masquerade would otherwise silently break arm detection in software that
         * flies an aircraft.
         *
         * Not to be confused with `ARMING_CHECK`, which appeared in QGC's missing-parameter
         * complaint while we were masquerading. That one is ArduPilot's pre-arm check bitmask
         * and is only read by APM's own setup QML (`APMFlightSafetyComponent*.qml`, with
         * `reportMissing = false`). It has nothing to do with armed-state detection, and nothing
         * asks for it under PX4.
         *
         * [PX4_CALIBRATION_PARAMETERS] is folded in wholesale — see its own note for why
         * QGC asking for those names is not a reason to publish them.
         *
         * Not on this list, and worth saying why: PX4 names QGC *reads* if present but treats
         * absence as "feature off" are safe to omit and unsafe to invent — `MPC_XY_VEL_MAX`
         * (speed limits, `PX4FirmwarePlugin.cc:340-352`), `FW_AIRSPD_MIN`/`MAX`,
         * `GF_MAX_HOR_DIST` (the geofence circle, `GeoFenceController.cc:481`),
         * `COM_DISARM_LAND` (auto-disarm display). Omitting each simply hides a control we
         * cannot honour; publishing a guess would put a wrong number in front of an operator.
         *
         * **`GF_MAX_HOR_DIST` is the near miss, and the reason it is still absent is worth
         * keeping.** DJI does state it: `KeyDistanceLimit` (`DJIKeyInfo<Integer>`, metres,
         * `canGet`/`canSet`/`canListen` all true), gated by `KeyDistanceLimitEnabled`, and DJI's
         * own wording — *"the distance between aircraft and home point"* — is the **same datum**
         * PX4 uses, a closer match than `RTL_RETURN_ALT` manages. PX4's `0 = disabled` maps onto
         * the switch being off. What stops it is what QGC does with the number, read on
         * 2026-07-26: `GeoFenceMapVisuals.qml:115-130` draws a `MapCircle` of that radius, and
         * `FlyViewMap.qml:323-329` puts it on the **live flight map** centred on the reported
         * home point, not only on the Plan map. Combined with the pull-only limitation described
         * in [AIRCRAFT_PARAMETERS], an operator who changes the limit in DJI Fly mid-session is
         * then navigating by a circle QGC will not re-read. A stale number in a config page and
         * a stale ring on the map an aircraft is being flown against are not the same risk, so
         * this one gets its own decision rather than riding in on `RTL_RETURN_ALT`'s.
         */
        val FORBIDDEN_PARAMETERS: Set<String> =
            setOf("ARMING_REQUIRE", "_HASH_CHECK") + PX4_CALIBRATION_PARAMETERS

        /**
         * The initial set: only facts the bridge itself owns and can state truthfully without
         * the DJI SDK. Aircraft-side settings (flight ceiling, RTH altitude, battery
         * thresholds, …) are deliberately absent until they are actually read from
         * `KeyManager` — a parameter value QGC displays must not be invented.
         */
        val DEFAULT_PARAMETERS: List<Parameter> = listOf(
            Parameter(
                name = "BRG_PROTO_VER",
                type = MavParamType.MAV_PARAM_TYPE_INT32,
                value = 1f,
                doc = "Revision of the MAVLink surface this bridge presents (handshake, " +
                    "parameter set, command coverage). Bumped whenever that surface changes, " +
                    "so a GCS log identifies exactly what it was talking to.",
            ),
            Parameter(
                name = "BRG_MAV_SYSID",
                type = MavParamType.MAV_PARAM_TYPE_INT32,
                value = 1f,
                doc = "MAVLink system id the bridge transmits as. Read-only: it is fixed when " +
                    "the UDP link opens (MavlinkLink.SYSTEM_ID).",
            ),
            Parameter(
                name = "BRG_TLM_HZ",
                type = MavParamType.MAV_PARAM_TYPE_INT32,
                value = 1f,
                doc = "Rate in Hz at which the bridge emits its fixed-rate telemetry and " +
                    "heartbeat. Read-only, and note it is the *only* rate we have: this build " +
                    "does no per-message rate control, so SET_MESSAGE_INTERVAL is answered " +
                    "MAV_RESULT_UNSUPPORTED.",
            ),
            Parameter(
                name = "BRG_GUIDED_OK",
                type = MavParamType.MAV_PARAM_TYPE_INT32,
                value = 0f,
                doc = "1 when the bridge can act on guided setpoints (M3 virtual-stick " +
                    "control), 0 while it cannot. 0 today: every guided command is answered " +
                    "MAV_RESULT_UNSUPPORTED. Exists so an operator can see the honesty " +
                    "boundary from the parameter editor instead of discovering it in flight.",
            ),
        )

        /**
         * PX4 names we publish **because the aircraft supplies the value**, which is the only
         * thing that ever makes a borrowed name legitimate (see [PX4_CALIBRATION_PARAMETERS]'s
         * closing note, which anticipated exactly this).
         *
         * Two tests have to pass before a name gets in here, and the second is the one that
         * disqualifies most candidates:
         *
         *  1. **DJI states the value.** A `KeyManager` key, verified in the jar rather than in
         *     the docs — this project's convention, and the docs have been wrong (see
         *     `SimulatorManager.isSimulatorEnabled()`, stubbed to `return false`).
         *  2. **The two names mean the same thing.** Not "describe the same area of behaviour" —
         *     mean the same thing, closely enough that a number moved from one to the other is
         *     still true. This is where `BAT1_SOURCE` (PX4: which *hardware* measures pack
         *     voltage), `RTL_DESCEND_ALT` (PX4: the descend target; DJI's nearest key is an
         *     operator-confirmation height, a different mechanism) and the failsafe action enums
         *     `NAV_RCL_ACT`/`NAV_DLL_ACT`/`COM_LOW_BAT_ACT` all fail. The action enums are the
         *     dangerous ones: their encodings differ, so a wrong value tells the operator the
         *     aircraft will Land when it will actually Return.
         *
         * `BAT1_N_CELLS` passes both and is still absent: QGC never asks for it and it buys
         * nothing, so it is cost without benefit rather than a judgement about honesty.
         *
         * ### What QGC does with these
         *
         * A published name is *removed* from the "Parameters are missing from firmware" list at
         * every site that asks for it with `reportMissing = true`, and any QML bound to it
         * becomes live. That is a UI change, and each entry below has to be worth it.
         *
         * ### The limitation that applies to every entry here, and has no fix in this build
         *
         * MAVLink parameters are **pull, not push**. QGC downloads the table once at connect and
         * caches each value in a `Fact`; nothing re-reads it. Our side is live — a
         * `PARAM_REQUEST_READ`, or the parameter editor's Refresh, gets whatever DJI last said —
         * but if an operator changes the setting in DJI Fly *while QGC is connected*, QGC keeps
         * showing the connect-time number until something makes it ask again.
         *
         * **This is a new hazard, not one PX4 has.** On a real PX4 vehicle QGC is the only thing
         * that changes parameters, so its cache is authoritative by construction. Here a second
         * app on a second screen can move the value behind it. The honest fix is an unsolicited
         * `PARAM_VALUE` pushed when the underlying key changes — MAVLink permits it and QGC's
         * `_handleParamValue` would apply it — and it is deliberately **not** in this pass: it
         * adds an outbound path that has never been measured against a live parameter download.
         * Until it exists, treat a value here as "what the aircraft said when the link opened".
         */
        val AIRCRAFT_PARAMETERS: List<AircraftParameter> = listOf(
            AircraftParameter(
                name = "RTL_RETURN_ALT",
                // PX4's own metadata says Float, units m, min 0
                // (`PX4ParameterFactMetaData.json`). DJI's key is an Integer number of metres,
                // so every value we can produce is exactly representable in a float.
                type = MavParamType.MAV_PARAM_TYPE_REAL32,
                source = "FlightControllerKey.KeyGoHomeHeight",
                doc = "The aircraft's configured return-to-home altitude, in metres, read from " +
                    "FlightControllerKey.KeyGoHomeHeight. Not a bridge setting and not " +
                    "writable from here: this is DJI's number, reported.",
                read = { it.goHomeHeightM?.toFloat() },
            ),
        )

        /** The stock table: bridge facts only, and no aircraft required. */
        fun default(): ParameterStore = ParameterStore(DEFAULT_PARAMETERS)

        /**
         * [DEFAULT_PARAMETERS] plus every [AIRCRAFT_PARAMETERS] entry whose key DJI has
         * **already delivered**, resolved once, here, and then fixed for the life of the store.
         *
         * Call this when a link opens, not when the process starts: before MSDK registration
         * `state()` is a bare [AircraftState] and every aircraft parameter would be omitted.
         *
         * A key that has not arrived yet produces a log line and nothing else. It is not given
         * a default, not published as 0, and not added later when it does arrive — see the class
         * doc for why the table cannot grow mid-session. The operator's experience of that is
         * one absent parameter and QGC's existing missing-parameter dialog, which is the same
         * thing they see today.
         */
        fun forAircraft(
            state: () -> AircraftState,
            log: (String) -> Unit = {},
        ): ParameterStore {
            val atBuildTime = state()
            val entries = ArrayList<Parameter>(DEFAULT_PARAMETERS)
            val readers = HashMap<String, (AircraftState) -> Float?>()
            for (candidate in AIRCRAFT_PARAMETERS) {
                val value = candidate.read(atBuildTime)
                if (value == null) {
                    log(
                        "parameter '${candidate.name}' omitted for this link session: " +
                            "${candidate.source} has not been delivered. It will not appear " +
                            "until the next link, because param_count cannot change."
                    )
                    continue
                }
                entries.add(
                    Parameter(
                        name = candidate.name,
                        type = candidate.type,
                        value = value,
                        writable = false,
                        doc = candidate.doc,
                    )
                )
                readers[candidate.name] = candidate.read
                log("parameter '${candidate.name}' published from ${candidate.source} = $value")
            }
            return ParameterStore(entries, readers, state)
        }
    }

    private val declared: List<Parameter> = entries.toList()
    private val indexByName: Map<String, Int>
    private val values: FloatArray
    private val lock = Any()

    init {
        require(declared.isNotEmpty()) {
            "QGC's parameter download never completes on an empty set; publish at least one parameter"
        }
        declared.forEach { p ->
            require(p.name.isNotEmpty() && p.name.length <= MAX_NAME_LENGTH) {
                "parameter name '${p.name}' must be 1..$MAX_NAME_LENGTH characters"
            }
            require(p.name !in FORBIDDEN_PARAMETERS) {
                "'${p.name}' must not be published: see ParameterStore.FORBIDDEN_PARAMETERS"
            }
            require(ParamCodec.isCarryable(p.type)) {
                "'${p.name}': ${p.type} does not fit PARAM_VALUE's 32-bit param_value field"
            }
        }
        val byName = HashMap<String, Int>(declared.size)
        declared.forEachIndexed { index, p ->
            require(byName.put(p.name, index) == null) { "duplicate parameter '${p.name}'" }
        }
        indexByName = byName
        values = FloatArray(declared.size) { declared[it].value }
        readers.keys.forEach { name ->
            require(name in byName) { "reader for '$name', which is not a declared parameter" }
        }
        require(readers.isEmpty() || state != null) {
            "aircraft-derived parameters need a state supplier to re-read them"
        }
    }

    /** `PARAM_VALUE.param_count`. */
    val count: Int get() = declared.size

    /** Declaration order is the wire index order; QGC's wait list is index based. */
    fun at(index: Int): Parameter? =
        if (index in declared.indices) withCurrentValue(index) else null

    fun byName(name: String): Parameter? = indexByName[name]?.let { withCurrentValue(it) }

    fun indexOf(name: String): Int = indexByName[name] ?: -1

    /** Every parameter with its current value, in wire index order. */
    fun snapshot(): List<Parameter> = declared.indices.map { withCurrentValue(it) }

    /**
     * Stores a new value for a known, writable parameter.
     *
     * Returns false for unknown or read-only parameters, in which case the caller must not
     * report success to the GCS.
     */
    fun store(name: String, value: Float): Boolean {
        val index = indexByName[name] ?: return false
        if (!declared[index].writable) return false
        synchronized(lock) { values[index] = value }
        return true
    }

    private fun withCurrentValue(index: Int): Parameter {
        val parameter = declared[index]
        refreshFromAircraft(index, parameter.name)
        val current = synchronized(lock) { values[index] }
        return parameter.copy(value = current)
    }

    /**
     * Re-reads an aircraft-derived parameter, so a setting an operator changed in DJI Fly is
     * what the next `PARAM_REQUEST_READ` returns rather than the value that happened to be
     * current when the link opened.
     *
     * **A null reading keeps the last one.** These are settings, and DJI going quiet about a
     * setting — a component-gone `null`, an unregistered SDK — is not a statement that the
     * aircraft's return altitude changed. The last thing DJI said remains the last thing DJI
     * said. The alternative, publishing 0, would tell an operator the aircraft returns at ground
     * level; there is no third option, because the parameter is in the table for this session
     * and `param_count` cannot shrink.
     *
     * Does nothing for a `BRG_*` parameter, which has no reader.
     */
    private fun refreshFromAircraft(index: Int, name: String) {
        val reader = readers[name] ?: return
        val supplier = state ?: return
        val fresh = reader(supplier()) ?: return
        synchronized(lock) { values[index] = fresh }
    }
}
