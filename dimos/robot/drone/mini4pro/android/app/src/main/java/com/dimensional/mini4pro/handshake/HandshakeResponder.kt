package com.dimensional.mini4pro.handshake

import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.CommandInt
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.FileTransferProtocol
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import io.dronefleet.mavlink.common.ParamRequestList
import io.dronefleet.mavlink.common.ParamRequestRead
import io.dronefleet.mavlink.common.ParamSet
import io.dronefleet.mavlink.common.ParamValue
import io.dronefleet.mavlink.common.RequestDataStream
import io.dronefleet.mavlink.common.SetMode
import io.dronefleet.mavlink.util.EnumValue
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArrayList

/**
 * Answers the protocol questions a GCS asks before it will consider the vehicle connected.
 *
 * This layer is pure MAVLink: no DJI SDK, no Android, no flight action of any kind. It answers
 * questions about *us* (version, capabilities, parameters) and it guarantees that
 * every command gets exactly one `COMMAND_ACK`. Anything it cannot itself perform is answered
 * `MAV_RESULT_UNSUPPORTED` — never `MAV_RESULT_ACCEPTED`, and never silence. Telling a GCS or an
 * operator that a command succeeded when nothing happened is the one failure mode that can hurt
 * someone, so real behaviour is opted *in* through the register* hooks below.
 *
 * ## What QGroundControl actually requires
 *
 * We identify as `MAV_AUTOPILOT_PX4`. The route here was APM → GENERIC → PX4: ArduPilot was
 * reversed because it demanded ~30 flight-controller calibration parameters DJI never exposes,
 * and GENERIC was left because `FirmwarePlugin::isCapable` is false for everything on the base
 * class, so QGC showed no flight controls at all. `PX4FirmwarePlugin::isCapable`
 * (`PX4FirmwarePlugin.cc:167-181`) grants Takeoff/Land/RTL/Pause/Guided/Orbit/ROI from the
 * vehicle *type* and reads nothing from the parameter set — so PX4 buys the control UI without
 * fabricating anything. See [AutopilotIdentity] for the capability consequences.
 *
 * The price is a **live inbound command surface** that did not exist under GENERIC. See
 * "The PX4 button surface" below; it is the reason this class exists.
 *
 * `InitialConnectStateMachine` (QGC @ `da14fad28`) walks: AutopilotVersion → StandardModes →
 * CompInfo → Parameters → Mission → GeoFence → RallyPoints → complete. Every step has a timeout
 * that advances to the next one, and the mission/fence/rally steps are additionally skippable —
 * so **no step blocks the sequence forever**.
 *
 * | step | if we answer nothing (PX4) | what answering buys |
 * |---|---|---|
 * | AUTOPILOT_VERSION | ~10 s, then QGC **invents** `MAVLINK2 \| MISSION_INT \| COMMAND_INT \| MISSION_FENCE \| MISSION_RALLY` for us (`InitialConnectStateMachine.cc:377-385`) — under GENERIC it assumed `MAVLINK2` alone | our real bits, the parameter-encoding declaration, and QGC *skipping* GeoFence/RallyPoints instead of asking for them |
 * | StandardModes (`AVAILABLE_MODES`) | request fails, `StandardModes::gotMessage` still emits `requestCompleted` | nothing today; answer `UNSUPPORTED`. See the mode-name note below |
 * | CompInfo (`COMPONENT_INFORMATION*`) | six `MAV_CMD_REQUEST_MESSAGE` in ~5 s (3× id 397, 3× id 395), then done. Not PX4-specific | nothing; answer `UNSUPPORTED` |
 * | **Parameters** | 4 retries, then most of QGC's UI stays dead. **Better than under GENERIC:** the "did not respond to request for parameters" dialog is suppressed only for `genericFirmware()` (`ParameterManager.cc:1391`), so under PX4 a parameter failure is visible again instead of silent | the parameter UI, and `autopilotPlugin.setupComplete` |
 * | Mission / GeoFence / RallyPoints | 15–30 s timeouts each; fence and rally are skipped outright because their capability bits are absent — capability-gated, not firmware-gated | seconds, and no spurious retries |
 *
 * Findings worth keeping in mind:
 *  - The requests are driven by `MAV_CMD_REQUEST_MESSAGE`, and QGC's `RequestMessageCoordinator`
 *    needs **both** a `COMMAND_ACK` *and* the message. An `UNSUPPORTED` ack fails the request
 *    immediately; an `ACCEPTED` ack with no message makes it wait ~1 s per inbound message tick.
 *    So `UNSUPPORTED` is both the honest and the fast answer.
 *  - Parameter values are still **bytewise** encoded: PX4 reintroduces no translation in either
 *    direction. See [ParamCodec] and [AutopilotIdentity.CAPABILITIES] for the proof.
 *  - **`_HASH_CHECK` is PX4's parameter-cache handshake, and we must not play.** See
 *    [ParameterStore.FORBIDDEN_PARAMETERS].
 *  - The FTP parameter download stays off: `ParameterManager`'s `_tryftp` is
 *    `vehicle->apmFirmware()` (`ParameterManager.cc:42`), so [MavlinkFtp] is still off QGC's
 *    critical path. Do not claim `MAV_PROTOCOL_CAPABILITY_FTP` to "help".
 *  - QGC sends **no `REQUEST_DATA_STREAM` and no `SET_MESSAGE_INTERVAL`** to a PX4 vehicle
 *    either: both came from `APMFirmwarePlugin::initializeStreamRates`, which PX4 does not
 *    override. We still answer both because other GCSs use them, but nothing we emit will be
 *    solicited — anything QGC needs must be streamed unprompted.
 *  - `MAV_CMD_REQUEST_MESSAGE` carries a **second argument**: measured against QGC 5.0.8, the
 *    `AVAILABLE_MODES` request arrives as `param1 = 435, param2 = 1` — the mode index. Any
 *    future provider for 435 must be `param2`-aware; [provide] deliberately ignores it today
 *    because every id we serve is unparameterised.
 *  - Answering `UNSUPPORTED` does not stop QGC re-asking, so a trickle of acks after connect is
 *    normal rather than a bug. Measured against QGC 5.0.8 with everything answered
 *    `UNSUPPORTED`: `MAV_CMD_REQUEST_MESSAGE` for `GIMBAL_MANAGER_INFORMATION` (280) six times
 *    at 1 Hz then stops; `CAMERA_INFORMATION` (259) paired with
 *    `MAV_CMD_REQUEST_CAMERA_INFORMATION` (521) on a doubling backoff at t ≈ 0, 2, 6, 14, 30 s.
 *    Both are the gimbal/camera managers rather than the connect sequence, and neither is
 *    PX4-specific.
 *  - Two more PX4 gates turn out to cost us nothing, checked rather than assumed. The MAVLink
 *    **events protocol** only reaches us through `MAVLinkEventManager`, whose one PX4-specific
 *    line drops `[cal]`-prefixed text carried inside an `EVENT` (#410) message
 *    (`MAVLinkEventManager.cc:199`); we never send `EVENT`, so the branch is unreachable and QGC
 *    sends nothing inbound for it. And `VehicleSupports::terrainFrame()` is literally
 *    `!px4Firmware()` (`VehicleSupports.cc:43-46`), whose only consumers are Plan-view QML —
 *    `AltFrameCombo.qml:36` drops the "terrain" altitude frame and `MissionDefaultsEditor.qml:35`
 *    / `TransectStyleComplexItemTerrainFollow.qml:23` hide terrain following. It removes UI for a
 *    capability we do not claim (`MAV_PROTOCOL_CAPABILITY_TERRAIN` is absent) and changes nothing
 *    on the wire.
 *
 * "Not Ready" in QGC's toolbar is not this layer's business. `MainStatusIndicator.qml` shows
 * "Ready" when `SYS_STATUS` advertises `MAV_SYS_STATUS_PREARM_CHECK` as enabled and healthy,
 * **or** when all sensors are healthy and `autopilotPlugin.setupComplete`. Under PX4,
 * `setupComplete` now depends on `PX4AutoPilotPlugin`'s component list rather than the single
 * `JoystickComponent` of `GenericAutoPilotPlugin`, so it is likely false — which makes the
 * `PREARM_CHECK` bit the only route to green, and leaves that honesty boundary exactly where it
 * was. Missing PX4 setup parameters colour config buttons red (`ConfigButton.qml:5`) and gate
 * nothing.
 *
 * ## The PX4 button surface
 *
 * Every one of these is unimplemented (M2/M3 are not started), so every one is answered
 * `MAV_RESULT_UNSUPPORTED` by the `else` branch of [handleCommand]. `HandshakeResponderTest`
 * enumerates them and asserts exactly that, because this list is the whole reason the identity
 * change is not free.
 *
 * The refusal was confirmed end to end on 2026-07-25: pressing Takeoff in QGC 5.0.8 against a
 * PX4-identifying vehicle that answers `UNSUPPORTED` produces the modal **"MAV_CMD_NAV_TAKEOFF
 * command not supported"** (`MavCommandQueue::showCommandAckError`, `:474-476`) and the vehicle
 * is *not* armed afterwards. That dialog is the honesty boundary made visible — it is what an
 * operator sees instead of a silent no-op, and it is the reason `UNSUPPORTED` beats silence.
 *
 * Rows marked **measured** were captured off the wire from QGC 5.0.8 on 2026-07-25 against a
 * PX4-identifying fake vehicle; the rest are read from QGC's source and have not been observed.
 *
 * | QGC action | on the wire | source |
 * |---|---|---|
 * | Takeoff | **measured:** `MAV_CMD_NAV_TAKEOFF` (22), `param1 = -1`, `param2/3 = 0`, `param4/5/6` NaN, `param7 = 106.2` for a vehicle at 103.2 m AMSL and a 3 m takeoff — i.e. **`param7` is AMSL, not height above ground** | `PX4FirmwarePlugin.cc:315-327` |
 * | (after a takeoff `ACCEPTED`) | `MAV_CMD_COMPONENT_ARM_DISARM` (400) `param1 = 1` — **our `UNSUPPORTED` is what stops QGC arming** | `PX4FirmwarePlugin.cc:296-311` |
 * | Arm / Disarm | `MAV_CMD_COMPONENT_ARM_DISARM` (400) `param1 = 1/0` | `Vehicle.cc:1437-1444` |
 * | Emergency Stop | **measured:** `MAV_CMD_COMPONENT_ARM_DISARM` (400) `param1 = 0, param2 = 21196` | `Vehicle.cc:2079-2087` |
 * | Pause | `MAV_CMD_DO_REPOSITION` (192) `param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE`, lat/lon/alt NaN | `PX4FirmwarePlugin.cc:271-283` |
 * | Go To location | **measured:** `COMMAND_INT` 192, `frame = 0` (`MAV_FRAME_GLOBAL`), `param1 = -1`, `param2 = 1` (`CHANGE_MODE`), `param4` NaN, `x = 379975294`, `y = 237272504` (1e7 deg), `z = 133.2` — **AMSL**, matching our reported 103.2 m ground + 30 m altitude | `PX4FirmwarePlugin.cc:384-421` |
 * | Change Altitude | `MAV_CMD_DO_REPOSITION` (192) twice — pause, then the new **AMSL** altitude | `PX4FirmwarePlugin.cc:470-500` |
 * | Change Heading | `MAV_CMD_DO_REPOSITION` (192) `param4` = radians | `PX4FirmwarePlugin.cc:536-554` |
 * | Change Speed | `MAV_CMD_DO_CHANGE_SPEED` (178) | `PX4FirmwarePlugin.cc:503-534` |
 * | Orbit | **measured:** `COMMAND_INT` 34, `frame = 0`, `param1 = 30` (radius m), `param2` NaN (default velocity), `param3 = 5` (`ORBIT_YAW_BEHAVIOUR_UNCHANGED`), `param4` NaN, 1e7 `x`/`y`, `z = 133.2` AMSL | `Vehicle.cc:1941-1970` |
 * | ROI | **measured:** `COMMAND_INT` 195, `frame = 0`, `param1-4` NaN, 1e7 `x`/`y`, `z = 0.0` — PX4 does a *terrain* lookup first and sends the result as AMSL, which resolved to 0 here. **Answered** by `guided/GuidedStickEngine.roi`; the `z` is discarded | `Vehicle.cc:1983`, `TerrainQueryCoordinator.cc:130-160` |
 * | ROI off | `MAV_CMD_DO_SET_ROI_NONE` (197). **Answered**, in every state | `Vehicle.cc:2003-2029` |
 *
 * **The ROI ack is a display claim as well as a reply.** `Vehicle::_handleCommandAck` flips QGC's
 * own `isROIEnabled` on our `ACCEPTED` and draws the ROI marker on the map, so accepting one we
 * cannot point at would put a marker on the operator's screen for a camera aimed elsewhere. That is
 * why `GuidedStickEngine.roi` refuses when there is no camera at all, rather than accepting
 * politely.
 * | Set Home (map) | **measured:** `COMMAND_LONG` 179 — *not* `COMMAND_INT`, so lat/lon arrive as **floats** in `param5`/`param6` (37.994545, 23.728001; ~1 m of precision lost) with `param7 = 78.0`, a terrain-resolved AMSL | `TerrainQueryCoordinator.cc:60-73` |
 * | Abort Landing | `MAV_CMD_DO_GO_AROUND` (191) | `Vehicle.cc:2054-2061` |
 * | PX4 log page | `MAV_CMD_LOGGING_START` (2510) / `STOP` (2511), all params 0 | `Vehicle.cc:2471-2479` |
 *
 * **`MAV_CMD_LOGGING_START` is user-initiated only.** `MAVLinkLogManager` wires itself up only
 * for `px4Firmware()` (`MAVLinkLogManager.cc:321`), but it sends nothing on connect: the auto
 * path needs the persisted `MAVLinkLogGroup/EnableAutoStart`, which defaults to **false**
 * (`MAVLinkLogManager.cc:300`). It is not retried (`MAV_CMD_LOGGING_START` is absent from
 * `MavCommandQueue::_shouldRetry`), and our `UNSUPPORTED` leaves `_loggingDenied` false, so the
 * button simply stays pressable. Nothing to do; `UNSUPPORTED` is the honest answer.
 *
 * ### RTL, Land, Hold, Mission and the mode dropdown are **not commands**
 *
 * This is the trap, and it is the opposite of ArduPilot. `PX4FirmwarePlugin::guidedModeRTL`,
 * `guidedModeLand`, `setGuidedMode` and `startMission` are all `_setFlightModeAndValidate()`
 * (`PX4FirmwarePlugin.cc:285-293`, `:580-587`, `FirmwarePlugin.cc:246-274`), and PX4 does
 * **not** override `MAV_CMD_DO_SET_MODE_is_supported()` — it is false on the base class
 * (`FirmwarePlugin.h:144`) and true only for APM (`APMFirmwarePlugin.h:39`). So `Vehicle::
 * setFlightMode` (`Vehicle.cc:1494-1510`) takes the **`SET_MODE` message (#11)** branch:
 * `base_mode` = our last heartbeat's base mode with `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` forced
 * on, `custom_mode` = the PX4 mode.
 *
 * `SET_MODE` has no acknowledgement. QGC judges success purely by watching our heartbeat's mode
 * string change, retries three times, and blocks its own UI thread for up to ~3.9 s before
 * showing "Unable to ... : Vehicle not changing to <mode> flight mode."
 *
 * Measured against QGC 5.0.8, with `base_mode` = 1 while disarmed and **129** while armed
 * (`MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`), confirming QGC ORs our own
 * heartbeat's base mode back at us:
 *
 * | QGC action | `custom_mode` | PX4 meaning | sends |
 * |---|---|---|---|
 * | **Return** button | `0x05040000` (84148224) | main AUTO(4), sub RTL(5) | 3× ~1.34 s apart |
 * | **Land** button | `0x06040000` (100925440) | main AUTO(4), sub LAND(6) | 3× ~1.34 s apart |
 * | dropdown → Hold | `0x03040000` (50593792) | main AUTO(4), sub LOITER(3) | once |
 * | dropdown → Mission | `0x04040000` (67371008) | main AUTO(4), sub MISSION(4) | once |
 * | dropdown → Position | `0x00030000` (196608) | main POSCTL(3) | once |
 * | dropdown → Altitude | `0x00020000` (131072) | main ALTCTL(2) | once |
 * | dropdown → Acro | `0x00050000` (327680) | main ACRO(5) | once |
 * | dropdown → Stabilized | `0x00070000` (458752) | main STABILIZED(7) | once |
 *
 * The 3× vs 1× split is the difference between `_setFlightModeAndValidate` (guided buttons, which
 * retry and then complain) and `Vehicle::setFlightMode` called straight from the dropdown, which
 * sends once and never notices that nothing happened. **The dropdown therefore fails silently**
 * — the operator sees the mode revert with no message at all.
 *
 * Two consequences this class enforces:
 *  1. There is nothing to refuse. We cannot answer `UNSUPPORTED` to a message that asked no
 *     question, so [onMessage] **records and logs** the request and sends nothing. QGC's own
 *     failure dialog is the operator-visible refusal.
 *  2. **Never echo a requested mode back in the heartbeat.** Doing so is how this layer would
 *     tell an operator the aircraft is returning home while it hovers. The mode in our
 *     heartbeat must be derived from the aircraft, never from what a GCS asked for. See
 *     [requestedModes] — it is a record, not a setpoint.
 *
 *  - `AVAILABLE_MODES` (#435) is still worth revisiting: it is the only channel that gets the
 *    real DJI mode name to the operator. Under PX4 it is *also* the only thing that could make
 *    `PX4FirmwarePlugin::flightMode()` show anything but the fixed PX4 names — that function
 *    has no `base_mode` fallback and returns `"Unknown"` for a `custom_mode` outside its table
 *    (`PX4FirmwarePlugin.cc:355-363`).
 *
 * ## Wiring
 *
 * ```
 * val responder = HandshakeResponder(send = link::send)
 * // ... in MavlinkLink's onMessage callback:
 * responder.onMessage(message)
 * ```
 *
 * [send] is called from whatever thread delivers inbound messages (the `mavlink-rx` thread) and
 * must be safe to call from there — `MavlinkLink.send` is.
 */
class HandshakeResponder(
    private val send: (Any) -> Unit,
    /**
     * The parameter table QGC downloads. Extend it, never with a real flight-controller name we
     * do not honour, and never with `_HASH_CHECK` — see [ParameterStore.FORBIDDEN_PARAMETERS].
     *
     * Defaults to the bridge-only table so a responder constructed with no aircraft still
     * answers a parameter download. [rebindParameters] swaps in one built from live aircraft
     * state when a link opens.
     */
    parameters: ParameterStore = ParameterStore.default(),
    private val identity: AutopilotIdentity = AutopilotIdentity(),
    /** Our own ids, used only to decide whether a targeted message is for us. */
    private val systemId: Int = 1,
    private val componentId: Int = 1,
    /** Answer MAVLink FTP requests with a NAK. See [MavlinkFtp] for why this matters. */
    private val nakFileTransfers: Boolean = true,
    /** Optional trace hook; keeps `android.util.Log` out of a unit-testable layer. */
    private val log: (String) -> Unit = {},
    /**
     * Wall clock, injected so the [MODE_REFUSAL_REPEAT_MS] window is testable without
     * sleeping. Used for nothing else.
     */
    private val nowMs: () -> Long = { System.currentTimeMillis() },
) {

    companion object {
        /** `MAV_CMD` ids we handle here. Ids rather than enum entries so a dialect that predates
         *  a command still compiles and still answers correctly. */
        /**
         * Shown to the operator when a `SET_MODE` is refused. **50 bytes is the hard
         * `STATUSTEXT` field width** — this is 45, counted, and must stay under it or the
         * text is truncated on the wire.
         */
        const val MODE_REFUSAL_TEXT = "Mode change refused: bridge is telemetry-only"

        /** Retry-burst window; QGC's guided buttons resend the same mode ~1.34 s apart, 3×. */
        const val MODE_REFUSAL_REPEAT_MS = 5_000L

        /**
         * Shown to the operator when a `PARAM_SET` is refused because the parameter is
         * read-only, which today is **every** parameter we publish. The name is appended, so the
         * operator learns which of their edits did not take; see [paramRefusalText] for the
         * width argument.
         */
        const val PARAM_REFUSAL_READ_ONLY = "Param refused, read-only: "

        /**
         * Shown when a *writable* parameter's write was not applied — the writer declined it,
         * no writer is registered, or the GCS named a type we do not agree with.
         *
         * Unreachable in this build, because nothing is writable and no writer is registered.
         * It exists so "a refused write is never silent" is a property of the code rather than
         * of the current table: the moment anyone marks a parameter writable, the failure path
         * already tells the operator instead of quietly echoing the old number back.
         */
        const val PARAM_REFUSAL_NOT_APPLIED = "Param write not applied: "

        /**
         * The refusal sentence for [name]. Both prefixes are short enough that the longest
         * legal parameter name still fits `STATUSTEXT`'s 50-byte field:
         * `PARAM_REFUSAL_READ_ONLY` is 26 bytes and [ParameterStore.MAX_NAME_LENGTH] is 16, so
         * the worst case is 42. Pinned by a test rather than trusted.
         *
         * Unlike [MODE_REFUSAL_TEXT], which deliberately omits the mode it refused, this does
         * name its subject. The reason `MODE_REFUSAL_TEXT` stays anonymous is that the mode
         * would have to be decoded back out of a PX4 `custom_mode`, which is the encoder's
         * table and not this layer's. No such problem here: the name arrived in the `PARAM_SET`,
         * we already matched it against our own table, and its length is bounded by our own
         * declaration.
         */
        fun paramRefusalText(prefix: String, name: String): String = prefix + name

        /**
         * Retry-burst window for [paramRefusalText], and it is not decoration: QGC master
         * retransmits a `PARAM_SET` **3 times about 1 s apart** before giving up
         * (`ParameterManager.h:109-111`, `kParamSetRetryCount = 2` plus the initial attempt,
         * `kWaitForParamValueAckMs = 1000`). Without de-duplication one operator edit would
         * produce three identical popups.
         */
        const val PARAM_REFUSAL_REPEAT_MS = 5_000L

        const val MAV_CMD_GET_MESSAGE_INTERVAL = 510
        const val MAV_CMD_SET_MESSAGE_INTERVAL = 511
        const val MAV_CMD_REQUEST_MESSAGE = 512
        const val MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520

        /**
         * The command form of a mode change. **QGC never sends this to us** — PX4 leaves
         * `MAV_CMD_DO_SET_MODE_is_supported()` false (`FirmwarePlugin.h:144`) so QGC uses the
         * `SET_MODE` message instead; only ArduPilot takes the command branch
         * (`APMFirmwarePlugin.h:39`). Named here because other ground stations do send it, and
         * because it is the one mode-change form that *can* be refused with a `COMMAND_ACK`.
         */
        const val MAV_CMD_DO_SET_MODE = 176

        /** `MAV_COMP_ID_ALL`; QGC addresses `PARAM_REQUEST_LIST` to it. */
        private const val COMPONENT_ID_ALL = 0

        /** Sender ids assumed when a caller hands us a bare payload (QGC's defaults). */
        const val DEFAULT_GCS_SYSTEM_ID = 255
        const val DEFAULT_GCS_COMPONENT_ID = 190
    }

    /** A command as received, in either `COMMAND_LONG` or `COMMAND_INT` form. */
    class CommandRequest(
        val command: Int,
        val param1: Float = 0f,
        val param2: Float = 0f,
        val param3: Float = 0f,
        val param4: Float = 0f,
        /** `COMMAND_LONG` only; for `COMMAND_INT` the positional data is in [x]/[y]. */
        val param5: Float = 0f,
        val param6: Float = 0f,
        /** `COMMAND_LONG.param7`, or `COMMAND_INT.z` — the same field by MAVLink's definition. */
        val param7: Float = 0f,
        val isCommandInt: Boolean = false,
        /** `MAV_FRAME`, `COMMAND_INT` only. Needed to interpret [x]/[y] (1e7 degrees vs cm). */
        val frame: Int = 0,
        val x: Int = 0,
        val y: Int = 0,
        val senderSystemId: Int = DEFAULT_GCS_SYSTEM_ID,
        val senderComponentId: Int = DEFAULT_GCS_COMPONENT_ID,
    )

    /** Rates requested via `REQUEST_DATA_STREAM`, the legacy ArduPilot path. */
    class StreamRequest(val streamId: Int, val rateHz: Int, val start: Boolean)

    /**
     * A mode change asked for with the `SET_MODE` message — QGC's PX4 route for RTL, Land,
     * Hold, Mission and the mode dropdown.
     *
     * A record of what was *asked*, never of what the aircraft is doing. `SET_MODE` carries no
     * acknowledgement, so this is the only trace an operator or the flight recorder has that a
     * refusal happened at all.
     */
    class ModeRequest(
        /** `base_mode`, with `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (bit 0) forced on by QGC. */
        val baseMode: Int,
        /** PX4 `custom_mode`: main mode in bits 16-23, sub mode in bits 24-31. */
        val customMode: Long,
        val senderSystemId: Int,
        val senderComponentId: Int,
    )

    /** Last mode refusal announced, so a retry burst does not become three warnings. */
    @Volatile private var lastRefusedMode: Long = -1L

    @Volatile private var lastRefusalAtMs: Long = Long.MIN_VALUE

    /** The same, for parameter writes: QGC sends each `PARAM_SET` up to three times. */
    @Volatile private var lastRefusedParam: String? = null

    @Volatile private var lastParamRefusalAtMs: Long = Long.MIN_VALUE

    /**
     * Tells the operator, in words, that a mode change did not happen.
     *
     * `SET_MODE` carries no acknowledgement, so refusing it is invisible: our only "no" is
     * failing to change the heartbeat's mode. For the guided buttons that is eventually
     * legible — `_setFlightModeAndValidate` gives up after ~4 s and says "Vehicle not
     * changing to <mode> flight mode". **The mode dropdown says nothing at all**: it sends
     * once, never re-reads, and the selection simply springs back
     * (`PX4FirmwarePlugin::setFlightMode`, and no validate call on that path).
     *
     * A control that silently does nothing is the failure this layer exists to prevent —
     * the same reason unimplemented commands get `MAV_RESULT_UNSUPPORTED` rather than
     * `ACCEPTED`. `STATUSTEXT` is the only channel left, so we use it.
     *
     * De-duplicated because QGC's guided buttons send the same `SET_MODE` three times about
     * 1.34 s apart; three identical warnings for one click would train the operator to
     * ignore them. A *different* mode, or the same one after the window, is a new intent and
     * is announced again.
     */
    private fun announceModeRefusal(request: ModeRequest) {
        val now = nowMs()
        val repeated = request.customMode == lastRefusedMode &&
            now - lastRefusalAtMs < MODE_REFUSAL_REPEAT_MS
        if (repeated) return
        lastRefusedMode = request.customMode
        lastRefusalAtMs = now
        // Deliberately does not name the requested mode: STATUSTEXT is 50 bytes, and the
        // mode name would have to be translated back out of a PX4 custom_mode, which is the
        // encoder's table, not this layer's. What the operator needs is that nothing happened.
        //
        // ERROR, not WARNING, and this was measured rather than chosen. At WARNING an operator
        // pressed Return on a real QGC 5.0.8 and **saw nothing at all**: only EMERGENCY, ALERT,
        // CRITICAL and ERROR satisfy `StatusText::severityIsError()`
        // (`StatusTextHandler.cc:18-24`), and only those increment the error count that makes
        // QGroundControl surface a message (`:241`). A WARNING is filed silently in the message
        // list, which nobody opens after pressing a button that appeared to do nothing.
        //
        // That made the honest-refusal design a no-op in practice — this whole path exists so a
        // mode change that will not happen cannot fail silently, and it was failing silently.
        // The usual argument against escalating severity is alarm fatigue; it does not apply
        // here, because this fires only in response to an operator pressing a button, and is
        // de-duplicated over [MODE_REFUSAL_REPEAT_MS] so QGC's 3× retry burst produces one.
        //
        // Not CRITICAL: that is reserved for the emergency-stop reply, where a safety function
        // the operator expects does not exist at all. Note also that CRITICAL-or-worse text
        // beginning with "preflight" is swallowed as a PX4 prearm message
        // (`Vehicle.cc:3445`) — not a hazard for this text, but a trap for the next one.
        send(
            Statustext.builder()
                .severity(MavSeverity.MAV_SEVERITY_ERROR)
                .text(MODE_REFUSAL_TEXT)
                .build()
        )
    }

    /**
     * The table as it stands. See the constructor parameter, and [rebindParameters] for the one
     * moment it is allowed to change.
     */
    @Volatile
    var parameters: ParameterStore = parameters
        private set

    /**
     * Replaces the parameter table. **Call this only while no ground station can be mid-download
     * — in practice, before the link's socket is opened.**
     *
     * The reason is `param_count`. QGC sizes its wait list from the first count it sees and
     * `_checkInitialLoadComplete` removes entries by index; a table that changed size underneath
     * a download in progress would leave indices outstanding that no message will ever carry,
     * and QGC would re-request forever. Within one link session the table is therefore fixed,
     * which is also why [ParameterStore.forAircraft] omits an undelivered parameter for the
     * whole session instead of adding it late.
     *
     * This exists because the two facts a table needs arrive at different times: the bridge's
     * own values are known when the process starts, and the aircraft's are known only once the
     * MSDK has registered and `KeyManager` has delivered. A link opening is the first instant
     * both are true and no GCS is listening yet.
     */
    fun rebindParameters(store: ParameterStore) {
        parameters = store
        log("parameter table rebound: ${store.count} params — ${store.snapshot().joinToString { it.name }}")
    }

    private val messageProviders = ConcurrentHashMap<Int, () -> Any?>()
    private val commandHandlers = ConcurrentHashMap<Int, (CommandRequest) -> MavResult>()

    @Volatile
    private var modeHandler: ((ModeRequest) -> Boolean)? = null

    @Volatile
    private var parameterWriter: ((String, Float) -> Boolean)? = null

    @Volatile
    private var intervalSink: ((Int, Long) -> Boolean)? = null

    private val intervals = ConcurrentHashMap<Int, Long>()
    private val streams = ConcurrentHashMap<Int, StreamRequest>()
    private val modeRequests = CopyOnWriteArrayList<ModeRequest>()

    init {
        // AUTOPILOT_VERSION is the one message this layer can produce on its own.
        messageProviders[AutopilotIdentity.MESSAGE_ID_AUTOPILOT_VERSION] = { identity.autopilotVersion() }
    }

    /**
     * Lets another layer supply a message on demand, e.g. the telemetry encoder registering
     * `EXTENDED_SYS_STATE` (245) or `HOME_POSITION` (242). Once registered, a
     * `MAV_CMD_REQUEST_MESSAGE` for that id is answered `ACCEPTED` and the message is sent; a
     * provider returning null means "not available right now" and yields `UNSUPPORTED`.
     */
    fun registerMessageProvider(messageId: Int, provider: () -> Any?) {
        messageProviders[messageId] = provider
    }

    /**
     * Lets a command actually do something. The handler runs on the receiving thread, must
     * return quickly, and must return the result it truly achieved — `ACCEPTED` only if the
     * action was started, `TEMPORARILY_REJECTED`/`DENIED`/`FAILED` otherwise. Unregistered
     * commands stay `UNSUPPORTED`.
     *
     * **Do not return `MAV_RESULT_IN_PROGRESS`.** This layer sends exactly one `COMMAND_ACK` per
     * command and has no mechanism for a follow-up terminal ack, and QGC keeps an `IN_PROGRESS`
     * command pending forever (`MavCommandQueue.cc:497-506`, the only exception being PX4's
     * autotune). A pending entry then blocks QGC from re-sending that same command at all
     * (`MavCommandQueue::sendWorker`'s duplicate check), so the button dies silently until
     * reconnect. If a DJI action is genuinely long-running, answer `ACCEPTED` once it has
     * actually started and report progress through telemetry.
     */
    fun registerCommandHandler(command: Int, handler: (CommandRequest) -> MavResult) {
        commandHandlers[command] = handler
    }

    /**
     * Lets a `SET_MODE` (#11) do something — QGC's route for Return and Land, which arrive as
     * unacknowledged mode changes rather than as commands.
     *
     * The handler answers one question: **did you take responsibility for this request?**
     * Returning true means the request was acted on *and* the operator was told whatever there
     * is to tell, so this layer stays quiet. Returning false — and being absent, and throwing —
     * all mean the same thing and run the untouched pre-M2 path: record the request, log it, and
     * warn the operator with [MODE_REFUSAL_TEXT]. There is deliberately no second refusal
     * branch for a handler to fall through into; the refusal an operator sees when commands are
     * switched off is the same code, and therefore the same behaviour, as before M2 existed.
     *
     * What the handler is **not** allowed to do, and cannot do from here: change the flight mode
     * this bridge reports. See [requestedModes]. It is handed a record of what was asked, and
     * the heartbeat keeps coming from the aircraft.
     *
     * Runs on the receiving thread, so it must return quickly. The request has already been
     * added to [requestedModes] by the time it is called — the record is of what QGC asked for,
     * which is true regardless of what anyone does about it.
     */
    fun registerModeHandler(handler: (ModeRequest) -> Boolean) {
        modeHandler = handler
    }

    /**
     * Lets `PARAM_SET` take effect. Called only for parameters declared `writable`; returning
     * false rejects the write, and the GCS is told so by echoing the unchanged value.
     */
    fun registerParameterWriter(writer: (name: String, value: Float) -> Boolean) {
        parameterWriter = writer
    }

    /**
     * Lets `MAV_CMD_SET_MESSAGE_INTERVAL` be honoured. Without a sink we record the request (see
     * [requestedIntervals]) and answer `UNSUPPORTED`, because this layer does no rate control.
     * A sink returning true means the rate was accepted by whoever owns the emitters.
     */
    fun registerIntervalSink(sink: (messageId: Int, intervalUs: Long) -> Boolean) {
        intervalSink = sink
    }

    /** Intervals requested via `MAV_CMD_SET_MESSAGE_INTERVAL`: message id → interval in µs
     *  (0 = default rate, -1 = disable). Recorded whether or not we honour them. */
    val requestedIntervals: Map<Int, Long> get() = intervals

    /** Stream rates requested via `REQUEST_DATA_STREAM`: `MAV_DATA_STREAM` id → request. */
    val requestedStreams: Map<Int, StreamRequest> get() = streams

    /**
     * Mode changes asked for with `SET_MODE`, oldest first — QGC's PX4 route for RTL, Land,
     * Hold, Mission and the mode dropdown, and the only inbound flight request that carries no
     * acknowledgement.
     *
     * Read-only, and **not** an input to anything that flies. Whoever eventually implements M2
     * should consume this to *start a DJI action*, and must still derive the heartbeat's mode
     * from the aircraft. Feeding a requested mode back into the heartbeat is how a bridge tells
     * an operator the aircraft is returning home while it hovers, and `_setFlightModeAndValidate`
     * reads exactly that field to decide the mode change "worked".
     *
     * QGC retries three times per button press, so expect up to three entries per press.
     */
    val requestedModes: List<ModeRequest> get() = modeRequests

    /** Entry point for [io.dronefleet.mavlink.MavlinkConnection] traffic. Returns quickly. */
    fun onMessage(message: MavlinkMessage<*>) {
        val payload = message.payload ?: return
        onMessage(payload, message.originSystemId, message.originComponentId)
    }

    /** Payload-level entry point; also what the tests drive. */
    fun onMessage(
        payload: Any,
        senderSystemId: Int = DEFAULT_GCS_SYSTEM_ID,
        senderComponentId: Int = DEFAULT_GCS_COMPONENT_ID,
    ) {
        when (payload) {
            is CommandLong -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                handleCommand(
                    CommandRequest(
                        command = payload.command().value(),
                        param1 = payload.param1(),
                        param2 = payload.param2(),
                        param3 = payload.param3(),
                        param4 = payload.param4(),
                        param5 = payload.param5(),
                        param6 = payload.param6(),
                        param7 = payload.param7(),
                        senderSystemId = senderSystemId,
                        senderComponentId = senderComponentId,
                    )
                )
            }

            is CommandInt -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                handleCommand(
                    CommandRequest(
                        command = payload.command().value(),
                        param1 = payload.param1(),
                        param2 = payload.param2(),
                        param3 = payload.param3(),
                        param4 = payload.param4(),
                        param7 = payload.z(),
                        isCommandInt = true,
                        frame = payload.frame().value(),
                        x = payload.x(),
                        y = payload.y(),
                        senderSystemId = senderSystemId,
                        senderComponentId = senderComponentId,
                    )
                )
            }

            is ParamRequestList -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                sendAllParameters()
            }

            is ParamRequestRead -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                handleParamRequestRead(payload)
            }

            is ParamSet -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                handleParamSet(payload)
            }

            // The five mission branches that used to live here — MISSION_REQUEST_LIST answered
            // with a count of 0, MISSION_COUNT and MISSION_CLEAR_ALL refused UNSUPPORTED, and
            // both request forms answered INVALID_SEQUENCE — are **deleted**, not extended
            // (`docs/m4-mission-transport.md` JC-10). They existed to make an absent feature
            // legible; `mission/MissionTransaction` is that feature, `Bridge.onInbound` routes
            // the eight mission message types to it, and a second place mission messages are
            // handled would be the shape of a bug that only appears after a reconnect — because
            // this class runs *first*. The tests moved with them, to `MissionTransactionTest`.

            is RequestDataStream -> {
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                // The legacy ArduPilot rate path. There is nothing to acknowledge: it is a
                // message, not a command, and QGC has no handler for the DATA_STREAM reply —
                // it judges success purely by whether the data shows up. So record the ask and
                // let the emitter layer decide.
                //
                // Expect a lot of these: APMFirmwarePlugin::initializeStreamRates re-runs
                // whenever BATTERY_STATUS or HOME_POSITION go missing, which is exactly our
                // situation until telemetry emits them.
                streams[payload.reqStreamId()] = StreamRequest(
                    streamId = payload.reqStreamId(),
                    rateHz = payload.reqMessageRate(),
                    start = payload.startStop() == 1,
                )
            }

            is SetMode -> {
                // SET_MODE has no target_component field, so only the system id can be checked.
                if (payload.targetSystem() != 0 && payload.targetSystem() != systemId) return
                // QGC's PX4 route for RTL / Land / Hold / Mission / the mode dropdown. There is
                // deliberately no reply: SET_MODE is a message, not a command, so MAVLink gives
                // us nothing to refuse with — and answering a COMMAND_ACK to a message QGC never
                // registered would be discarded ("Ack not in list", MavCommandQueue.cc:489).
                //
                // What we must NOT do is act like it worked. QGC's _setFlightModeAndValidate
                // polls our heartbeat's flight mode for ~1.3 s, three times over, and treats a
                // matching mode as proof the vehicle complied (FirmwarePlugin.cc:246-274). Since
                // nothing here touches the heartbeat, QGC times out and tells the operator
                // "Vehicle not changing to <mode> flight mode" — which is the truth.
                val request = ModeRequest(
                    baseMode = payload.baseMode().value(),
                    customMode = payload.customMode(),
                    senderSystemId = senderSystemId,
                    senderComponentId = senderComponentId,
                )
                modeRequests.add(request)
                // Offer it to whoever registered for it (M2's command layer) before refusing.
                // A handler that declines, is absent, or throws leaves the refusal below
                // exactly as it was before there was a handler at all.
                val handler = modeHandler
                val taken = handler != null && guarded("mode handler", false) { handler(request) }
                if (taken) {
                    log(
                        "SET_MODE base=0x%02x custom=0x%08x — taken by the command layer"
                            .format(request.baseMode, request.customMode)
                    )
                    return
                }
                log(
                    "SET_MODE base=0x%02x custom=0x%08x — recorded, not performed"
                        .format(request.baseMode, request.customMode)
                )
                announceModeRefusal(request)
            }

            is FileTransferProtocol -> {
                if (!nakFileTransfers) return
                if (!addressedToUs(payload.targetSystem(), payload.targetComponent())) return
                val request = MavlinkFtp.parse(payload.payload()) ?: return
                if (request.opcode == MavlinkFtp.RSP_ACK || request.opcode == MavlinkFtp.RSP_NAK) return
                log("ftp opcode ${request.opcode} → NAK ${MavlinkFtp.errorCodeFor(request.opcode)}")
                send(MavlinkFtp.nak(request, senderSystemId, senderComponentId))
            }

            else -> Unit
        }
    }

    // ---------------------------------------------------------------- commands

    private fun handleCommand(request: CommandRequest) {
        var followUp: Any? = null

        val result = when (request.command) {
            MAV_CMD_REQUEST_MESSAGE -> {
                val provided = provide(request.param1.toInt())
                if (provided == null) {
                    MavResult.MAV_RESULT_UNSUPPORTED
                } else {
                    followUp = provided
                    MavResult.MAV_RESULT_ACCEPTED
                }
            }

            // The pre-MAV_CMD_REQUEST_MESSAGE way of asking the same question. QGC still
            // retries commands of this id, and other GCSs only know this form.
            MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES -> {
                val provided = provide(AutopilotIdentity.MESSAGE_ID_AUTOPILOT_VERSION)
                if (provided == null) {
                    MavResult.MAV_RESULT_UNSUPPORTED
                } else {
                    followUp = provided
                    MavResult.MAV_RESULT_ACCEPTED
                }
            }

            MAV_CMD_SET_MESSAGE_INTERVAL -> {
                val messageId = request.param1.toInt()
                val intervalUs = request.param2.toLong()
                intervals[messageId] = intervalUs
                val sink = intervalSink
                val accepted = sink != null && guarded("interval sink for $messageId", false) {
                    sink(messageId, intervalUs)
                }
                if (accepted) MavResult.MAV_RESULT_ACCEPTED else MavResult.MAV_RESULT_UNSUPPORTED
            }

            // Everything else is unimplemented until something registers a handler, and is
            // answered MAV_RESULT_UNSUPPORTED. Under PX4 this branch is no longer theoretical:
            // it is what answers every button in QGC's Fly view — takeoff, arm/disarm,
            // emergency stop, reposition/goto/pause/altitude/heading, change speed, orbit, ROI,
            // set home, abort landing, log streaming — plus MAV_CMD_GET_MESSAGE_INTERVAL and
            // the camera requests (MAV_CMD_REQUEST_CAMERA_INFORMATION and friends; we expose no
            // camera component, and when video lands the camera should answer for itself under
            // its own component id). See the class KDoc for the enumerated surface.
            //
            // UNSUPPORTED rather than DENIED or FAILED on purpose: QGC turns each into a
            // different operator-facing string ("<cmd> command not supported" vs "denied" vs
            // "failed", MavCommandQueue.cc:466-479), and "not supported" is the true one — the
            // bridge has no such capability at all, as opposed to having refused this attempt.
            else -> {
                val handler = commandHandlers[request.command]
                if (handler == null) {
                    MavResult.MAV_RESULT_UNSUPPORTED
                } else {
                    // A handler that throws did not do what it was asked, so say FAILED — and
                    // do not let it kill the receive thread.
                    guarded("handler for command ${request.command}", MavResult.MAV_RESULT_FAILED) {
                        handler(request)
                    }
                }
            }
        }

        if (result == MavResult.MAV_RESULT_UNSUPPORTED) {
            log("unsupported command ${request.command} (p1=${request.param1})")
        }

        send(
            CommandAck.builder()
                .command(EnumValue.create(MavCmd::class.java, request.command))
                .result(result)
                .progress(0)
                .resultParam2(0)
                .targetSystem(request.senderSystemId)
                .targetComponent(request.senderComponentId)
                .build()
        )

        // Ack first, then the payload. QGC's RequestMessageCoordinator accepts either order.
        followUp?.let { send(it) }
    }

    // -------------------------------------------------------------- parameters

    private fun sendAllParameters() {
        val all = parameters.snapshot()
        log("PARAM_REQUEST_LIST → ${all.size} params")
        all.forEachIndexed { index, parameter -> send(paramValue(parameter, index)) }
    }

    private fun handleParamRequestRead(request: ParamRequestRead) {
        val index = request.paramIndex()
        // paramIndex < 0 means "look me up by name" (MAVLink uses -1).
        val parameter = if (index >= 0) parameters.at(index) else parameters.byName(request.paramId())
        if (parameter == null) {
            // No reply. Inventing a zero-valued PARAM_VALUE for a parameter we do not have
            // would tell the GCS a setting exists and reads as 0.
            log("PARAM_REQUEST_READ for unknown param '${request.paramId()}' index=$index — ignored")
            return
        }
        send(paramValue(parameter, parameters.indexOf(parameter.name)))
    }

    private fun handleParamSet(request: ParamSet) {
        val name = request.paramId()
        val existing = parameters.byName(name)
        if (existing == null) {
            log("PARAM_SET for unknown param '$name' — ignored")
            return
        }

        // We own the definition, so we decode with our declared type. A GCS that names a
        // different type has misunderstood the parameter; decoding its bytes as our type would
        // produce a garbage value, so refuse the write instead.
        val typeMatches = request.paramType().value() == EnumValue.of(existing.type).value()
        val requested = if (typeMatches) ParamCodec.decode(existing.type, request.paramValue()) else 0f
        if (!typeMatches) {
            log("PARAM_SET '$name' type mismatch: got ${request.paramType().value()}, expected ${existing.type}")
        }

        val writer = parameterWriter
        val accepted = typeMatches && existing.writable && writer != null &&
            guarded("parameter writer for '$name'", false) { writer(name, requested) } &&
            parameters.store(name, requested)

        // Echo either way, and re-read rather than reusing `existing`: for an aircraft-derived
        // parameter this is what makes the revert *visible*, because the value that goes back
        // out is DJI's current one, not the one the GCS asked for.
        //
        // The echo alone is not enough, and measuring QGC master is what showed it. Its ack
        // predicate `checkForCorrectParamValue` (`ParameterManager.cc:315-349`) compares the
        // returned value against the requested one and *discards* a mismatch as "not for us",
        // so our unchanged echo does not complete the write; QGC retries twice more and after
        // ~3 s shows "Parameter write failed: param: <name> …". That is a real refusal, but it
        // is generic and late, and on QGC 5.0.8 it did not happen at all — that build matched
        // the ack on **name only**, so the echo satisfied the wait and the revert was silent.
        // MAVLink's own answer, `PARAM_ERROR`, is unusable: added 2026-04-13, in no release, and
        // absent from `io.dronefleet.mavlink` 1.1.11.
        //
        // So the echo stays (it is what puts the true value back in QGC's Fact) and a STATUSTEXT
        // is sent beside it, for the same measured reason mode refusals are at ERROR.
        val current = parameters.byName(name) ?: existing
        if (!accepted) {
            log("PARAM_SET '$name' rejected (writable=${existing.writable})")
            announceParamRefusal(name, readOnly = !existing.writable)
        }
        send(paramValue(current, parameters.indexOf(name)))
    }

    /**
     * Tells the operator that a parameter edit did not take, and why.
     *
     * **Read-only is not a formality here.** Every parameter we publish describes something
     * real: `RTL_RETURN_ALT` is the altitude this aircraft will climb to on Return. QGC's
     * parameter editor lets an operator type over any value it can read, and if a write were
     * accepted without being pushed to DJI, the operator would set a return altitude, believe
     * it, and watch the aircraft do something else. Writing aircraft configuration is separate
     * work with its own safety review; until it exists the refusal must be loud.
     *
     * `MAV_SEVERITY_ERROR` for the reason measured on the mode-refusal path: only
     * EMERGENCY/ALERT/CRITICAL/ERROR reach the operator (`StatusTextHandler.cc:18-30`, and the
     * popup at `:241-243`). A WARNING is filed in a list nobody opens.
     *
     * De-duplicated over [PARAM_REFUSAL_REPEAT_MS] because QGC sends each `PARAM_SET` three
     * times. A *different* parameter, or the same one after the window, is a new edit and is
     * announced again.
     */
    private fun announceParamRefusal(name: String, readOnly: Boolean) {
        val now = nowMs()
        val repeated = name == lastRefusedParam && now - lastParamRefusalAtMs < PARAM_REFUSAL_REPEAT_MS
        if (repeated) return
        lastRefusedParam = name
        lastParamRefusalAtMs = now
        val prefix = if (readOnly) PARAM_REFUSAL_READ_ONLY else PARAM_REFUSAL_NOT_APPLIED
        send(
            Statustext.builder()
                .severity(MavSeverity.MAV_SEVERITY_ERROR)
                .text(paramRefusalText(prefix, name))
                .build()
        )
    }

    private fun paramValue(parameter: Parameter, index: Int): ParamValue =
        ParamValue.builder()
            .paramId(parameter.name)
            // Bytewise encoding, which is what QGC reads natively; see ParamCodec.
            .paramValue(parameter.wireValue())
            .paramType(parameter.type)
            .paramCount(parameters.count)
            .paramIndex(index)
            .build()

    // ----------------------------------------------------------------- helpers

    /** Null when nothing can supply that message right now. */
    private fun provide(messageId: Int): Any? {
        val provider = messageProviders[messageId] ?: return null
        return guarded<Any?>("provider for message $messageId", null) { provider() }
    }

    /**
     * Runs a callback registered by another layer. Those callbacks reach into the DJI SDK, and
     * this code runs on the MAVLink receive thread: an exception escaping here would end the
     * thread and take the whole link down. A failed callback becomes [fallback] instead.
     */
    private inline fun <T> guarded(what: String, fallback: T, body: () -> T): T =
        try {
            body()
        } catch (e: Exception) {
            log("$what threw ${e.javaClass.simpleName}: ${e.message}")
            fallback
        }

    /**
     * A targeted message is ours if it names our system (or broadcasts with 0) and either our
     * component or `MAV_COMP_ID_ALL`. QGC addresses `PARAM_REQUEST_LIST` to `MAV_COMP_ID_ALL`
     * and commands to the autopilot component.
     */
    private fun addressedToUs(targetSystem: Int, targetComponent: Int): Boolean =
        (targetSystem == 0 || targetSystem == systemId) &&
            (targetComponent == COMPONENT_ID_ALL || targetComponent == componentId)
}
