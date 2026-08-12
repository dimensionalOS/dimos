package com.dimensional.mini4pro.handshake

import io.dronefleet.mavlink.common.AutopilotVersion
import io.dronefleet.mavlink.common.MavProtocolCapability
import io.dronefleet.mavlink.util.EnumValue

/**
 * What we tell the GCS about ourselves in `AUTOPILOT_VERSION`.
 *
 * QGC consumes this in `InitialConnectStateMachine::_handleAutopilotVersionSuccess`: the
 * capability bitmask goes straight into `Vehicle::_setCapabilities`, and several managers are
 * gated on individual bits. So each bit is a promise, and a bit we cannot keep produces UI
 * that fails when used (or, worse, a protocol QGC tries first and we cannot answer).
 *
 * We identify as `MAV_AUTOPILOT_PX4`, so QGC serves us with `PX4FirmwarePlugin` /
 * `PX4AutoPilotPlugin`. The reason is the Fly view: `FirmwarePlugin::isCapable` is false for
 * everything on the base class (`FirmwarePlugin.h:127`), so a generic vehicle gets no
 * Takeoff/Land/RTL/Pause/Go-to/Orbit/ROI buttons at all, while `PX4FirmwarePlugin::isCapable`
 * (`PX4FirmwarePlugin.cc:167`) grants them from the vehicle *type* alone and reads nothing from
 * the parameter set. Missing PX4 setup parameters gate no capability — they colour config
 * buttons red (`ConfigButton.qml:5`) and block navigation to the *setup pages*
 * (`PX4AutoPilotPlugin::prerequisiteSetup`), nothing that flies. ArduPilot stays rejected
 * because `APMRadioComponent::setupComplete` genuinely demands invented calibration data.
 *
 * **This is not the same as PX4 being free.** A connect raises four modal dialogs, two from
 * `flightSwVersion` below and two from the parameter set; QGC names 30 PX4 parameters it wants.
 * All of it is noise rather than blockage, but it is measured noise — see
 * [ParameterStore.PX4_MISSING_PARAMETERS] before concluding this bargain was costless.
 *
 * **PX4 changes what silence means, and makes answering load-bearing.**
 * `_handleAutopilotVersionFailure` (`InitialConnectStateMachine.cc:377-385`) assumes
 * `MAVLINK2` for a generic vehicle but `MAVLINK2 | MISSION_INT | COMMAND_INT | MISSION_FENCE |
 * MISSION_RALLY` for PX4 or APM. Under GENERIC, failing to answer merely left QGC uninformed.
 * Under PX4 it would leave QGC believing we support **geofence and rally points**, so
 * `GeoFenceManager::supported()` / `RallyPointManager::supported()` become true and QGC runs
 * those connect states against a vehicle that has neither. Answering is now the difference
 * between QGC skipping them and QGC waiting on them.
 */
class AutopilotIdentity(
    /**
     * `flight_custom_version`, 8 bytes, **all zero on purpose**.
     *
     * This is one of the two fields PX4 identity reinterprets. QGC reads it two different ways
     * (`InitialConnectStateMachine.cc:345-363`):
     *
     *  - non-PX4: 8 ASCII characters, shown verbatim as the git hash. That is why this used to
     *    carry the build tag `"mini4pro"`.
     *  - **PX4: raw binary, byte-reversed, printed as 16 hex digits**, and additionally
     *    `setFirmwareCustomVersion(buf[2], buf[1], buf[0])`.
     *
     * So under PX4 the old ASCII tag would be displayed as the git hash `6f7270346e696d00`
     * and the custom version `110.105.109` — digits that look like a real firmware revision and
     * are pure accident. All-zero renders as `0000000000000000` / `0.0.0`, which reads as
     * "unset" rather than as a fabricated revision. The build tag now belongs in
     * `BRG_PROTO_VER`, which no dialect reinterprets.
     */
    private val flightCustomVersion: ByteArray = ByteArray(8),
    /**
     * Left at 0 = unknown, on purpose. QGC only calls `setFirmwareVersion()` when this is
     * non-zero, so 0 means "no flight-stack version claimed" — which is the truth for a bridge
     * that is not a flight stack.
     *
     * **Known cost under PX4. Measured, and an open decision — see the report.**
     * Zero costs us *two* of the four modal dialogs QGC 5.0.8 raises on connect, both observed
     * on 2026-07-25:
     *
     *  1. On the Fly view, straight off `AUTOPILOT_VERSION`:
     *     *"QGroundControl supports PX4 Pro firmware Version 1.4.1 and above. You are using a
     *     version prior to that which will lead to unpredictable results. Please upgrade your
     *     firmware."* — `PX4FirmwarePlugin::_handleAutopilotVersion` (`:656-695`) treats
     *     `flight_sw_version == 0` as older than 1.4.1. Once per vehicle instance.
     *  2. *"Vehicle is not running latest stable firmware! Running **-1.-1.-1**, latest stable
     *     is 1.14.4."* — `FirmwarePlugin::_versionFileDownloadFinished` (`FirmwarePlugin.cc:362`).
     *     Because `setFirmwareVersion()` is never called, the version facts stay at
     *     `versionNotSetValue = -1` (`VehicleTypes.h:31`) and QGC prints them literally. Note this
     *     one costs a **network fetch of PX4's release file on every connect**.
     *
     * The other two dialogs are the parameter set's doing, not this field's — see
     * [ParameterStore.PX4_MISSING_PARAMETERS].
     *
     * The only value that silences both is one encoding >= 1.4.1, which is a specific, checkable
     * claim to be a PX4 build we are not, and it would additionally select PX4's
     * version-specific parameter metadata remap. Between "two dialogs, one of which displays a
     * nonsense version" and "a fabricated flight-stack version", 0 is kept: it is ugly and true,
     * and `-1.-1.-1` at least reads as *unset* rather than as a real build. Reverse this only as
     * a deliberate decision, not to quiet a dialog.
     */
    private val flightSwVersion: Long = 0,
) {

    companion object {
        /** `MAVLINK_MSG_ID_AUTOPILOT_VERSION`, the id QGC puts in `MAV_CMD_REQUEST_MESSAGE.param1`. */
        const val MESSAGE_ID_AUTOPILOT_VERSION = 148

        /**
         * The capabilities we can actually honour.
         *
         * - `MAVLINK2` — true: [com.dimensional.mini4pro.mavlink.MavlinkLink] sends with
         *   `send2()`.
         * - `PARAM_ENCODE_BYTEWISE` — true, and still correct under PX4. MAVLink requires one
         *   of the two encoding flags whenever the parameter protocol is supported, and
         *   bytewise is what we send: see [ParamCodec]. The C-cast convention we claimed while
         *   masquerading as ArduPilot was only ever right *because*
         *   `APMFirmwarePlugin::_handleIncomingParamValue` /
         *   `_handleOutgoingParamSetThreadSafe` translated it. PX4 reintroduces no such
         *   translation: `PX4FirmwarePlugin::adjustIncomingMavlinkMessage`
         *   (`PX4FirmwarePlugin.cc:640-654`) switches on `AUTOPILOT_VERSION` only, and PX4 does
         *   not override `adjustOutgoingMavlinkMessageThreadSafe` at all (`FirmwarePlugin.h:289`
         *   is a no-op; `APMFirmwarePlugin.h:51` is the sole override). So QGC reads
         *   `param_value` through `mavlink_param_union_t` directly, which is bytewise.
         * - `COMMAND_INT` — true at the protocol level: [HandshakeResponder] decodes
         *   `COMMAND_INT` and acknowledges it exactly like `COMMAND_LONG`. Under PX4 this bit
         *   became visible on the wire: `PX4FirmwarePlugin::guidedModeGotoLocation`
         *   (`PX4FirmwarePlugin.cc:395`), `Vehicle::guidedModeOrbit` (`Vehicle.cc:1947`),
         *   `Vehicle::stopGuidedModeROI` (`Vehicle.cc:2007`) and
         *   `TerrainQueryCoordinator::sendROICommand` (`TerrainQueryCoordinator.cc:135`) all
         *   branch on it and send `COMMAND_INT` with a real `MAV_FRAME` and 1e7 lat/lon rather
         *   than degrading the coordinate to a float. It says nothing about which commands are
         *   implemented; unimplemented ones are answered `MAV_RESULT_UNSUPPORTED` in either
         *   form.
         * - `MISSION_INT` — **turned on by M4** (`docs/m4-mission-transport.md` §4.8). We hold a
         *   real mission store, we take `MISSION_ITEM_INT` uploads and we serve them back
         *   verbatim; see `mission/MissionTransaction`. The bit is believed **inert** on QGC's
         *   side — `_handleMissionRequest` packs `mission_item_int` unconditionally
         *   (`PlanManager.cc:532`) and does not consult it — which is precisely why it needs a
         *   measurement (§9.2) before it is called free. "Believed inert" is how `MISSION_FENCE`
         *   nearly cost us a geofence we do not have.
         *
         * Deliberately omitted — and under PX4 the omissions are what stop QGC assuming
         * otherwise, because [these are exactly the bits `_handleAutopilotVersionFailure`
         * would invent for us]:
         * - `MISSION_FENCE` / `MISSION_RALLY` — no geofence or rally support. Omitting them
         *   makes QGC *skip* its GeoFence and RallyPoints connect states outright
         *   (`GeoFenceManager::supported()`, `RallyPointManager::supported()`), which are
         *   capability-gated rather than firmware-gated.
         * - `MISSION_FLOAT` — we take only `MISSION_ITEM_INT`. QGC never sends the float form to
         *   a PX4 vehicle anyway (`writeArduPilotGuidedMissionItem` is APM-only,
         *   `MissionManager.cc:25`), and an inbound `MISSION_ITEM` is refused rather than
         *   converted: a float latitude is ~1 m of precision lost, which is the same defect
         *   `DO_SET_HOME` has and it was measured there.
         * - `FTP` — we do not implement MAVLink FTP. Under PX4 nothing on the connect path
         *   tries it either: `ParameterManager`'s FTP parameter download is gated on
         *   `apmFirmware()` (`ParameterManager.cc:42`), and component-information metadata only
         *   goes over FTP when the URI *we publish in `COMPONENT_INFORMATION`* uses the `mftp`
         *   scheme (`RequestMetaDataTypeStateMachine.cc:504`, `FTPManager.h:74`) — we answer
         *   that request `UNSUPPORTED`, so no URI exists. Note neither path consults this bit.
         *   The only code in the tree that reads it is `OnboardLogController.cc:500`, where
         *   claiming it would switch QGC's log browser from `LOG_REQUEST_LIST` messages to an
         *   FTP directory listing we cannot serve. See [MavlinkFtp].
         * - `SET_ATTITUDE_TARGET` / `SET_POSITION_TARGET_LOCAL_NED` /
         *   `SET_POSITION_TARGET_GLOBAL_INT` — guided setpoints are M3. Add the ones we
         *   implement when virtual-stick guided control works, and flip `BRG_GUIDED_OK` with
         *   them. Note these bits are *not* what gates QGC's guided buttons — under PX4 those
         *   are already on, which is the whole point of the identity and the reason
         *   [HandshakeResponder] now has a live inbound command surface.
         * - `TERRAIN`, `FLIGHT_TERMINATION`, `COMPASS_CALIBRATION`, `SET_ACTUATOR_TARGET` —
         *   not implemented, and each drives QGC UI that would fail.
         */
        val CAPABILITIES: List<MavProtocolCapability> = listOf(
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_COMMAND_INT,
            MavProtocolCapability.MAV_PROTOCOL_CAPABILITY_MISSION_INT,
        )
    }

    /** The numeric bitmask we put on the wire; handy for assertions and logging. */
    val capabilityBits: Int = CAPABILITIES.fold(0) { bits, c -> bits or EnumValue.of(c).value() }

    fun autopilotVersion(): AutopilotVersion = AutopilotVersion.builder()
        .capabilities(EnumValue.create<MavProtocolCapability>(capabilityBits))
        .flightSwVersion(flightSwVersion)
        .middlewareSwVersion(0)
        .osSwVersion(0)
        .boardVersion(0)
        .flightCustomVersion(flightCustomVersion.copyOf())
        // vendorId / productId / uid stay 0: we have no board ids, and QGC treats 0 as
        // unknown rather than displaying a wrong board.
        .vendorId(0)
        .productId(0)
        .build()
}
