package com.dimensional.mini4pro

import android.os.Looper
import android.os.SystemClock
import android.util.Log
import com.dimensional.mini4pro.command.CommandDispatcher
import com.dimensional.mini4pro.command.CommandInterlock
import com.dimensional.mini4pro.command.MsdkFlightActions
import com.dimensional.mini4pro.gimbal.GimbalManager
import com.dimensional.mini4pro.gimbal.KeyManagerGimbalPort
import com.dimensional.mini4pro.gimbal.MsdkGimbalAim
import com.dimensional.mini4pro.gimbal.RecordedGimbalPort
import com.dimensional.mini4pro.light.KeyManagerLightPort
import com.dimensional.mini4pro.light.LightControl
import com.dimensional.mini4pro.light.RecordedLightPort
import com.dimensional.mini4pro.health.DeviceHealthWatch
import com.dimensional.mini4pro.warn.WarnLevel
import com.dimensional.mini4pro.warn.WarnSource
import com.dimensional.mini4pro.warn.WarningBus
import com.dimensional.mini4pro.warn.WindWarnings
import com.dimensional.mini4pro.health.MsdkDeviceHealthPort
import com.dimensional.mini4pro.guided.ControlOrigin
import com.dimensional.mini4pro.guided.GuidedRecord
import com.dimensional.mini4pro.guided.GuidedSituation
import com.dimensional.mini4pro.guided.GuidedStickEngine
import com.dimensional.mini4pro.guided.KeyManagerVirtualStickPort
import com.dimensional.mini4pro.guided.ManoeuvreGimbal
import com.dimensional.mini4pro.guided.OrbitCommand
import com.dimensional.mini4pro.guided.RepositionCommand
import com.dimensional.mini4pro.guided.RoiCommand
import com.dimensional.mini4pro.record.CommandSource
import com.dimensional.mini4pro.record.Setpoint
import com.dimensional.mini4pro.record.StickAxes
import com.dimensional.mini4pro.record.StickModes
import com.dimensional.mini4pro.record.StickRange
import com.dimensional.mini4pro.record.StickPath
import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.handshake.ParameterStore
import com.dimensional.mini4pro.handshake.toMavResult
import com.dimensional.mini4pro.guided.MissionRoute
import com.dimensional.mini4pro.guided.MissionRunSink
import com.dimensional.mini4pro.mission.MissionCommands
import com.dimensional.mini4pro.mission.MissionExecutor
import com.dimensional.mini4pro.mission.MissionProgress
import com.dimensional.mini4pro.mission.MissionStore
import com.dimensional.mini4pro.mission.MissionTransaction
import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import com.dimensional.mini4pro.mavlink.LinkLocks
import com.dimensional.mini4pro.mavlink.MavlinkLink
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.mavlink.WifiBind
import com.dimensional.mini4pro.mavlink.WifiBindTracker
import com.dimensional.mini4pro.mavlink.WifiNetworkGate
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Recorder
import com.dimensional.mini4pro.record.GimbalSample
import com.dimensional.mini4pro.simulator.SimulatorControl
import com.dimensional.mini4pro.simulator.simulatorPort
import com.dimensional.mini4pro.simulator.SimulatorNotice
import com.dimensional.mini4pro.simulator.SimulatorPhase
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.HomeEventGate
import com.dimensional.mini4pro.telemetry.StateSource
import com.dimensional.mini4pro.telemetry.TelemetryEncoder
import com.dimensional.mini4pro.video.VideoEvents
import com.dimensional.mini4pro.video.VideoRequest
import com.dimensional.mini4pro.video.VideoStreamer
import com.dimensional.mini4pro.zenoh.ZenohBus
import com.dimensional.mini4pro.zenoh.ZenohSettings
import io.dronefleet.mavlink.MavlinkMessage
import io.dronefleet.mavlink.common.CommandInt
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.MavSeverity
import io.dronefleet.mavlink.common.Statustext
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/**
 * The MAVLink bridge: owns the link to the GCS, the fixed-rate telemetry loop,
 * and inbound routing.
 *
 * Identity is `MAV_AUTOPILOT_PX4`, encoded in [TelemetryEncoder]. All three
 * candidates were tried against the real QGC on 2026-07-25 — see the dialect
 * decision in PLAN.md. ArduPilot demands fabricated sensor calibration;
 * GENERIC is honest but leaves QGC with no control buttons at all; PX4 grants
 * the buttons while reading nothing from our parameter set.
 *
 * The consequence that lands here: QGC's PX4 buttons can send us commands we
 * cannot perform. They are refused with `MAV_RESULT_UNSUPPORTED` — an
 * `ACCEPTED` for a command that did nothing is the most dangerous reply this
 * layer can make. M2 opens exactly two of those buttons, Return and Land, and
 * only behind [commandInterlock]; see [commands].
 *
 * Every byte in and out of the socket is tapped into the flight recorder here,
 * which is the only place both directions are visible.
 */
object Bridge {

    private const val TAG = "Bridge"

    /**
     * Base tick. Individual messages are emitted on multiples of this, because
     * they have genuinely different natural rates — sending all of them at one
     * rate puts HEARTBEAT at 5 Hz, which is wrong and was caught by
     * `mavverify`'s rate check on 2026-07-25.
     */
    private const val BASE_PERIOD_MS = 200L

    // Divisors against BASE_PERIOD_MS. DJI's own keys update at roughly 10 Hz,
    // and the probe showed attitude arriving at ~2 Hz, so 5 Hz for the fast set
    // is already resending cached values — going faster would only inflate the
    // link, not the information.
    private const val EVERY_TICK = 1        // 5 Hz  — position, gps, attitude, hud
    private const val TICKS_2HZ = 2         // 2.5 Hz — sys status, extended sys state
    private const val TICKS_1HZ = 5         // 1 Hz  — heartbeat, battery

    /** MAVLink message id, for `MAV_CMD_REQUEST_MESSAGE`. */
    private const val MESSAGE_ID_HOME_POSITION = 242

    /**
     * How often the flight recorder's internals are mirrored to the GCS. 2 Hz is
     * the rate `GcsMirror` was costed at; it emits nothing at all unless virtual
     * stick is engaged, so this is free until M3.
     */
    private const val MIRROR_PERIOD_MS = 500L

    data class State(
        val running: Boolean = false,
        val target: String? = null,
        val sent: Long = 0,
        val received: Long = 0,
        val error: String? = null,
        /**
         * The WiFi-binding status line: bound/refused/lost/rebound, verbatim from
         * [WifiBind] and [WifiBindTracker]. Separate from [error] because "the
         * socket is on a dead network" must stay visible while send errors come
         * and go.
         */
        val wifi: String? = null,
    )

    private val _state = MutableStateFlow(State())
    val state: StateFlow<State> = _state.asStateFlow()

    private var link: MavlinkLink? = null
    private var timer: ScheduledExecutorService? = null

    /**
     * Sends [payload] from a thread that is allowed to touch the network.
     *
     * **DJI's `KeyManager` delivers its action callbacks on the Android main thread**, and Android
     * kills a process that sends UDP there (`NetworkOnMainThreadException`). Measured 2026-07-26,
     * twice, on the bench: DJI refused a takeoff asynchronously with `SYSTEM_ERROR`, the refusal
     * rode `reportAsyncDjiError` → `announce` → `MavlinkLink.send` on DJI's callback thread — the
     * main thread — and the app died announcing it. It had never fired before because no DJI
     * action had ever *failed asynchronously* until that session; every success path only logs.
     * The same wire ran under the gimbal's async errors and the landing auto-confirm, so the fix
     * belongs here, once, rather than at whichever call site crashed first.
     *
     * Off the main thread this is exactly the old `link?.send(payload)`, same thread, same
     * ordering. On the main thread the payload is handed to [timer] — the telemetry executor,
     * which exists precisely as long as the link does, and which already interleaves sends from
     * the tick. If the link is down (`timer` null or shut down, or torn down in the race between
     * the check and the execute), the payload is dropped with a log line: a STATUSTEXT with no
     * link has nowhere to go, and dropping it is what `link?.send` already did when `link` was
     * null.
     */
    private fun sendOffMain(payload: Any) {
        if (!Looper.getMainLooper().isCurrentThread) {
            link?.send(payload)
            return
        }
        val executor = timer
        if (executor == null || executor.isShutdown) {
            // **A stopped bridge has no link, and that is not a fault.** DJI keeps delivering
            // health whether or not a GCS is attached — which is *wanted*, because the recorder
            // still writes it and a compass fault discovered with the bridge down is worth having
            // in the record. What has nowhere to go is the STATUSTEXT, and dropping it is exactly
            // what `link?.send` did when `link` was null.
            //
            // Logged at debug rather than warn, on the reasoning `MissionTransitions.isMoot`
            // spells out: on 2026-07-27 a flickering compass produced a `W/` line every two
            // seconds with the bridge deliberately stopped, and a warning that fires on the
            // expected case teaches the reader to skip the line. A *link that vanished under a
            // live bridge* is a different claim and keeps its warning below.
            if (link == null) {
                Log.d(TAG, "no link: ${payload.javaClass.simpleName} not sent (bridge stopped)")
            } else {
                Log.w(TAG, "dropped ${payload.javaClass.simpleName} — link present but timer gone")
            }
            return
        }
        try {
            executor.execute { link?.send(payload) }
        } catch (e: java.util.concurrent.RejectedExecutionException) {
            // stop() shut the executor down between the check and the submit. Same answer as
            // above — and it must not throw on the main thread, which is the bug being fixed.
            Log.w(TAG, "dropped ${payload.javaClass.simpleName} — link stopped mid-send: $e")
        }
    }

    /**
     * Holds the WiFi radio and CPU up while the link is running. Optional so the
     * bridge still starts if nobody supplied a context — see [LinkLocks] for what
     * that costs in packet loss.
     */
    private var locks: LinkLocks? = null

    /**
     * Watches the WiFi network the socket is bound to, for the life of one link.
     * Loss is reported, never acted on toward the aircraft — but when WiFi
     * returns, the link's socket is rebuilt on the new Network automatically:
     * see [onWifiAvailable] for why that does not breach the no-autonomous-actions
     * rule.
     */
    private var wifiGate: WifiNetworkGate? = null
    private var wifiTracker: WifiBindTracker? = null

    /**
     * Answers QGC's connect sequence. One instance across reconnects: it holds
     * no link state, and the one thing that *is* per-session — the parameter
     * table, which must not change size while a GCS is downloading it — is
     * rebuilt in [start] before the socket opens.
     *
     * Note none of QGC's connect states hard-block — they all time out onward.
     * The point of answering is to state honest capabilities (its failure path
     * *assumes* APM features we lack) and to make the parameter editor work.
     */
    private val handshake = HandshakeResponder(
        send = ::sendOffMain,
        log = { msg -> Log.d(TAG, msg) },
    ).apply {
        // Turn MAV_CMD_REQUEST_MESSAGE for these from UNSUPPORTED into
        // ACCEPTED + the real message. The encoder owns their content.
        // **[outboundState], not [aircraftState].** These answer `MAV_CMD_REQUEST_MESSAGE`, so
        // they are the same telemetry [tick] sends and must describe the same thing it does —
        // otherwise a replay would be on the stream and a live (absent) aircraft in the reply to
        // a request for the very same message.
        registerMessageProvider(TelemetryEncoder.MESSAGE_ID_EXTENDED_SYS_STATE) {
            TelemetryEncoder.extendedSysState(outboundState())
        }
        registerMessageProvider(MESSAGE_ID_HOME_POSITION) {
            TelemetryEncoder.homePosition(outboundState())
        }
    }

    /**
     * The switch between "translates telemetry" and "can move an aircraft".
     *
     * Off at every process start and not persisted — see [CommandInterlock]. Owned here rather
     * than by the UI so it lives exactly as long as the process does, and so nothing in the
     * inbound MAVLink path can reach the only method that turns it on.
     */
    val commandInterlock = CommandInterlock(log = { msg -> Log.i(TAG, msg) })

    /**
     * **Heading follows course — the one control, and the one the interlock panel should carry.**
     *
     * When on (the default), a commanded translation turns the nose toward where it is going: a
     * plain `DO_REPOSITION` goto and every mission leg. When off, the aircraft flies with whatever
     * heading it has, which is the behaviour that has actually flown.
     *
     * `docs/decisions/2026-07-27-heading-follows-course.md` recommends exactly this shape — always
     * on, behind a switch on the existing on-screen interlock panel, reversible in flight — because
     * the current behaviour is the flight-verified one and a way back to it costs one boolean. A
     * yaw-mixing or setpoint-frame surprise in the air is then a tap rather than a new build.
     *
     * Deliberately **not** persisted and deliberately **not** reachable from MAVLink, on the same
     * two arguments [commandInterlock] rests on: it is a property of this session, and nothing on
     * the inbound path may change what the aircraft does with its own nose.
     *
     * Read fresh on every 10 Hz tick by [GuidedStickEngine], so flipping it takes effect on the
     * next setpoint rather than at the next engagement. **This is the seam the UI wires to.**
     */
    val headingFollowsCourse = java.util.concurrent.atomic.AtomicBoolean(true)

    /**
     * The one fan-out every operator-facing sentence from the command layer goes through.
     *
     * Holds exactly one sink today — the MAVLink one, wrapping the same [sendOffMain] the three
     * announcing classes used to be handed directly — so what an operator sees is unchanged to
     * the byte. What changes is that a second interface becomes a second `attach`, rather than a
     * second `send` lambda threaded through three constructors and remembered at every call site
     * (`docs/zenoh-dimos-transport.md` §3.1).
     *
     * Attached for the life of the process rather than per link, which is exactly what passing
     * `::sendOffMain` at construction already did: a sentence composed with no link running is
     * dropped by `sendOffMain` itself, with a log line, as it always was.
     */
    val announcer = Announcer(StatusTextSink(::sendOffMain))

    /**
     * The plan this bridge holds. **The bridge is the mission; the aircraft never holds our
     * plan** (`docs/m4-mission-transport.md` §0), so this object is the truth a read-back reads.
     *
     * Owned here, at service scope, and deliberately **not** torn down by [stop]: QGC re-reads
     * the plan on every connect (§1.4), and a store that died with the socket would show an empty
     * Plan view for an aircraft that is flying one. It is never persisted to disk (JC-5) — see
     * [MissionStore] for why a restored plan is a claim about a session that has ended.
     */
    val missionStore: MissionStore = MissionStore(
        log = { msg -> Log.i(TAG, "mission: $msg") },
        // §2.1 rule 5: the executor is told **synchronously**, on the write's own edge, so a paused
        // cursor is dropped there rather than discovered later. Referenced lazily — the executor is
        // constructed below and this runs only when a plan is written.
        onChanged = { plan ->
            if (plan != null) missionExecutor.onPlanCommitted() else missionExecutor.onPlanCleared()
        },
    )

    /**
     * **The mission executor** — the thing that takes an uploaded plan and flies it.
     *
     * Owned at service scope, beside the store and for the same reason: a lifecycle that died with
     * the socket would drop a cursor QGC is still drawing. The engine it commands is per-link and is
     * reached through a lambda, so a mission cannot outlive the link that could stop it.
     *
     * The [MissionExecutor.MissionEngine] adapter is the whole of what the executor may do to the
     * aircraft: start a route, and ask whether one is flying. It deliberately cannot reach `abort`
     * or `reposition` — the abort ladder is decided in exactly one place.
     */
    val missionExecutor: MissionExecutor = MissionExecutor(
        store = missionStore,
        engine = {
            guidedStick?.let { stick ->
                object : MissionExecutor.MissionEngine {
                    override fun missionStart(
                        route: MissionRoute,
                        startIndex: Int,
                        rejoining: Boolean,
                        sink: MissionRunSink,
                    ) = stick.missionStart(route, startIndex, rejoining, sink)

                    override fun missionFlying(): Boolean = stick.missionFlying()
                }
            }
        },
        aircraftState = ::aircraftState,
        interlockEnabled = { commandInterlock.enabled },
        reached = { seq -> missionProgress.reachedSink.onReached(seq) },
        announcer = announcer,
        log = { msg -> Log.i(TAG, "mission: $msg") },
    )

    /**
     * The mission protocol, both directions. One owner, one state machine (§2.1).
     *
     * Constructed once and reused across links, like [handshake] and for the same reason: it
     * holds protocol state that belongs to a *transaction*, not to a socket, and [start] calls
     * [MissionTransaction.reset] to drop any in-flight transaction and the per-session
     * announcement state without touching the store.
     *
     * The interlock gate is M2 §Q2's, verbatim: with commands off, an upload and a clear-all get
     * `MAV_MISSION_UNSUPPORTED` byte for byte as they did before M4 — and a **read** still tells
     * the truth, which is JC-1's one deliberate divergence.
     *
     * The provenance callbacks are the same two calls the rest of the bridge uses for home and
     * for the AMSL datum, so the numbers recorded against a plan are the numbers we published.
     * Neither is ever used as a datum by this layer; §3.2 explains why the execution half needs
     * them and why transport must not silently rewrite a plan's altitudes.
     */
    val missionTransaction: MissionTransaction = MissionTransaction(
        store = missionStore,
        send = ::sendOffMain,
        interlockEnabled = { commandInterlock.enabled },
        execution = { missionExecutor },
        // Deliberately *the same call* the encoder uses to fill HOME_POSITION rather than a
        // second derivation of the same idea, so a plan's recorded home is byte-for-byte the home
        // the ground station was shown — including DJI's `KeyIsHomeLocationSet` gate and the
        // 4.58e7 filler check behind it.
        homeAtUpload = {
            TelemetryEncoder.homeCoordinate(aircraftState())
                ?.let { com.dimensional.mini4pro.mission.GeoPoint(it.first, it.second) }
        },
        amslDatumAtUpload = { TelemetryEncoder.amslMetres(aircraftState()) },
        log = { msg -> Log.i(TAG, "mission: $msg") },
        nowMs = { SystemClock.elapsedRealtime() },
    )

    /**
     * `MISSION_CURRENT` and `MISSION_ITEM_REACHED`, on their own cadence — see [MissionProgress].
     * Withheld entirely while the store is empty (JC-7), which is every session until an upload.
     */
    val missionProgress: MissionProgress = MissionProgress(
        store = missionStore,
        execution = { missionExecutor },
        send = ::sendOffMain,
        log = { msg -> Log.i(TAG, "mission: $msg") },
    )

    /**
     * What this bridge believes about the MSDK aircraft simulator.
     *
     * Owned here for the same reason the interlock is — it must live exactly as long as the
     * process, and nothing in the inbound MAVLink path may reach the method that starts it — but
     * with one deliberate difference in lifecycle. The interlock is subscribed to nothing and
     * dies with the process; the simulator's state lives on the *aircraft*, so its `KeyManager`
     * subscription is tied to MSDK registration rather than to the link (`MainActivity` calls
     * [SimulatorControl.observe]) and `Bridge.stop` does **not** cancel it. A simulator can
     * outlive this link, this bridge, and this process, and the window in which we most need to
     * see one is before an operator has started a session.
     *
     * Every transition is mirrored into the flight recorder as well as logcat: "was this session
     * simulated?" is the first question a reader of a log must be able to answer, and it is not
     * recoverable from anything else in the file.
     */
    val simulator = SimulatorControl(
        // `simulatorPort(Recorder)`, not `KeyManagerSimulatorPort()` — the production class is
        // private to its file and the only factory takes a tap, so every start and stop request
        // and DJI's answer to it are on the record by construction.
        port = simulatorPort(Recorder),
        log = { msg ->
            Log.i(TAG, "simulator: $msg")
            recordSimulatorEvent(msg)
        },
        onChange = { _simulatorRevision.value++ },
        nowMs = { SystemClock.elapsedRealtime() },
    )

    /**
     * DJI's own health warnings — the DJI Fly messages — into QGroundControl, the flight record
     * and logcat. See `docs/device-health.md`.
     *
     * **Why this exists**: on 2026-07-26 the aircraft repeatedly force-landed and returned home on
     * the bench, and an hour went into theorising before someone opened DJI Fly and found an
     * overheat warning that had been showing the whole time. Every layer of this bridge was
     * working correctly and reporting a healthy aircraft. The aircraft was explaining itself
     * through a channel nobody had subscribed to.
     *
     * Owned here, and with the **simulator's** lifecycle rather than the link's: it is planted by
     * [tick]'s idempotent [com.dimensional.mini4pro.health.DeviceHealthWatch.ensure] once MSDK
     * registration lands, and [stop] deliberately does **not** remove it. The reasoning is the one
     * [simulator] gives — this is pure observation with no effect on the aircraft, the flight
     * recorder can outlive a link, and a warning standing at the moment the operator restarts the
     * bridge is exactly the one they most need to still be there. `flightActions`, `gimbalAim` and
     * `guidedStick` are all torn down on [stop] because each of them *can command an aircraft*;
     * this one cannot.
     *
     * `sendOffMain` is not optional here. DJI delivers health on the Android main thread, and a
     * UDP send from there kills the process — measured, twice; see [sendOffMain].
     */
    /**
     * **The single owner of what happens to a DJI warning** — every source in, all four surfaces
     * out. See `warn/WarningBus`; `CLAUDE.md` states the rule that nothing may bypass it.
     */
    val warnings = WarningBus(
        qgc = { text, severity -> sendOffMain(warnStatusText(text, severity)) },
        screen = { _ -> _warningRevision.value = _warningRevision.value + 1 },
        record = { event -> Recorder.warn(event) },
        bus = { event -> ZenohBus.publishWarning(event) },
        log = { event ->
            val w = event.warning
            val line = "warn ${w.source.label} ${event.change.name.lowercase()} ${w.code} " +
                "${w.state}/${w.level} fwd=${event.announce} — ${event.text}"
            if (event.recordSeverity == LogEntry.SEV_INFO) Log.i(TAG, line) else Log.w(TAG, line)
        },
        note = { msg -> Log.i(TAG, "warn: $msg") },
        nowMs = { SystemClock.elapsedRealtime() },
    )

    private val _warningRevision = MutableStateFlow(0L)

    /**
     * Bumped on every announced warning, purely so the status screen repaints.
     *
     * The phone screen is the surface Ivan actually holds — he flies looking at it as often as at
     * QGC — and it has no other reason to repaint when a warning arrives: `Msdk.state` and [state]
     * do not move because DJI said the wind is strong. Without this the sentence would reach QGC,
     * the record and the bus and stop one metre short of the pilot.
     */
    val warningRevision: StateFlow<Long> = _warningRevision.asStateFlow()

    val deviceHealth = DeviceHealthWatch(
        port = MsdkDeviceHealthPort(log = { msg -> Log.w(TAG, "health-port: $msg") }),
        warnings = warnings,
        note = { msg -> Log.i(TAG, "health: $msg") },
    )

    /**
     * A warning sentence as `STATUSTEXT`.
     *
     * The integer→enum step is written out rather than looked up, because `io.dronefleet.mavlink`'s
     * `MavSeverity` exposes **no accessor for its wire value** (javap, 2026-07-26: eight constants,
     * `values()`, `valueOf()`, nothing else). Any lookup would therefore have to go through
     * `ordinal`, which happens to be right for MAV_SEVERITY today and is exactly the kind of
     * coincidence that turns a CRITICAL into an INFO the day a dialect is regenerated.
     *
     * The `else` cannot be reached from [WarnLevel.mavSeverity]'s four values and is deliberately
     * WARNING rather than INFO: an unmappable severity is a bug in this bridge, and the safe
     * direction for a warning is louder.
     */
    private fun warnStatusText(text: String, severity: Int): Statustext {
        val level = when (severity) {
            WarnLevel.MAV_SEVERITY_CRITICAL -> MavSeverity.MAV_SEVERITY_CRITICAL
            WarnLevel.MAV_SEVERITY_ERROR -> MavSeverity.MAV_SEVERITY_ERROR
            WarnLevel.MAV_SEVERITY_WARNING -> MavSeverity.MAV_SEVERITY_WARNING
            WarnLevel.MAV_SEVERITY_INFO -> MavSeverity.MAV_SEVERITY_INFO
            else -> MavSeverity.MAV_SEVERITY_WARNING
        }
        return Statustext.builder().severity(level).text(text).build()
    }

    private val _simulatorRevision = MutableStateFlow(0L)

    /**
     * Bumped on every simulator phase change, purely so the status screen repaints.
     *
     * The screen's other two sources are `Msdk.state` and [state], and neither moves when a
     * simulator starts — [state] only ticks while a link is running, and the simulator is
     * deliberately independent of the link. Without this, an operator who starts a simulator with
     * the bridge stopped would go on reading "off" over a simulating aircraft, which is precisely
     * the display error this whole feature exists to prevent.
     */
    val simulatorRevision: StateFlow<Long> = _simulatorRevision.asStateFlow()

    /**
     * QGC's Takeoff, Return and Land, routed to whatever implements `FlightActions`.
     *
     * Registered with [handshake] unconditionally: with [commandInterlock] off it declines every
     * request and the pre-M2 refusal runs, so attaching it changes nothing until an operator
     * switches commands on. [CommandDispatcher.actions] is set to the DJI layer in [start] and
     * cleared in [stop] — see [flightActions] — and a request that somehow arrives with it null
     * is reported to the operator as a failure rather than absorbed.
     */
    val commands = CommandDispatcher(
        interlock = commandInterlock,
        announcer = announcer,
        // The AMSL datum a commanded takeoff altitude is inverted against, and it is deliberately
        // *the same call* that fills GLOBAL_POSITION_INT.alt in `tick` rather than a second
        // derivation of the same idea. QGC composes MAV_CMD_NAV_TAKEOFF's param7 as
        // "requested height + the AMSL we last published" (`PX4FirmwarePlugin.cc:317-338`), so
        // subtracting anything else leaves DJI's pressure-altitude offset uncancelled — 14 m one
        // day and 28 m the next (`docs/measurements/2026-07-26-amsl-datum.md`). Passing the
        // encoder's own function is what makes the round trip exact rather than approximately
        // right, and it is why this reads as one line instead of arithmetic.
        publishedAmslM = { TelemetryEncoder.amslMetres(aircraftState()) },
        // The takeoff's second phase: DJI's own hop takes no altitude, so the height the operator
        // asked for is flown afterwards, as an ordinary Change Altitude, by the engine that
        // already flies them. Read fresh rather than held — `guidedStick` is created by [start]
        // and cleared by [stop], and null is a legitimate answer that costs only the older
        // "DJI goes to 1.2m, not 3.0m" sentence. `command/PendingClimb` has the whole shape.
        climb = { guidedStick },
        // QGC's Start Mission and Continue Mission. The executor answers both, including the
        // refusals, so there is exactly one launch gate and it is the one with the checks in it.
        onMissionStart = { missionExecutor.start() },
        // The bottom auxiliary lamp, from QGC's Fly View actions (`qgc/MavlinkActions.json`).
        // Read fresh for the same reason `climb` is: created by [start], cleared by [stop], and a
        // null is a legitimate answer that costs only an UNSUPPORTED. Not behind the interlock,
        // on exactly the argument [gimbal] makes below — a lamp cannot move an aircraft, and the
        // interlock starts off every session, which is when an operator setting up after dark
        // wants it most.
        light = { light },
        log = { msg -> Log.d(TAG, msg) },
    ).also { it.attachTo(handshake) }

    /**
     * QGroundControl's camera controls, routed to whatever implements `GimbalAim`, plus the three
     * messages that make those controls appear at all.
     *
     * **Not behind [commandInterlock], and that is a decision with an argument** — see
     * [GimbalManager]'s KDoc and `docs/gimbal.md`. The short version: the interlock means "this
     * bridge can move an aircraft", a gimbal cannot move one, and gating the routine act of
     * aiming a camera behind the switch that arms Return and Land would leave that switch on for
     * most of every flight.
     *
     * Attached to [handshake] unconditionally, exactly as [commands] is, and equally inert until
     * there is something behind it: with no `GimbalAim` and no gimbal attitude from DJI,
     * [GimbalManager.tick] emits nothing and every command is refused with a reason. A bridge on
     * an aircraft whose gimbal never reports puts byte-for-byte the same traffic on the wire as
     * it did before this package existed.
     *
     * Its clock is [SystemClock.elapsedRealtime], monotonic, for the reason `MsdkFlightActions`
     * gives: a window measured on an aircraft must not be stretched or cut by an NTP step.
     */
    val gimbal = GimbalManager(
        announcer = announcer,
        log = { msg -> Log.d(TAG, "gimbal: $msg") },
        nowMs = { SystemClock.elapsedRealtime() },
        timeBootMs = { SystemClock.elapsedRealtime() },
    ).also { it.attachTo(handshake) }

    /**
     * The DJI half of M2, alive exactly as long as the link is. Kept as its concrete type
     * because [stop] must reach [MsdkFlightActions.stop] to cancel the landing-confirmation
     * subscription; [CommandDispatcher.actions] sees it only as `FlightActions`.
     *
     * Tied to the bridge lifecycle rather than to product connect/disconnect, and that is a
     * choice, not an accident: commands can only arrive over a link this object owns, so
     * "bridge running" is exactly the window in which a `FlightActions` can be asked anything.
     * Product presence is checked *per call* inside [KeyManagerActionPort.unavailableReason] —
     * strictly fresher than connect/disconnect callbacks (which `Bridge` never receives; the
     * `Msdk.state` collector lives in `MainActivity`, whose lifecycle is shorter than the
     * service that runs this bridge) — so a disconnect refuses the very next action with
     * `NO_PRODUCT` and a reconnect needs no re-wiring. The `KeyManager` subscription is made
     * lazily on the first `land()`, after that same check has passed, which is what keeps it
     * from being the silent pre-registration no-op `docs/architecture.md` warns about.
     */
    private var flightActions: MsdkFlightActions? = null

    /**
     * The DJI half of the gimbal, alive exactly as long as the link is — same lifecycle and same
     * reasoning as [flightActions], including the per-call product check inside
     * [KeyManagerGimbalPort]. Kept as its concrete type because [stop] must reach
     * [MsdkGimbalAim.stop] to cancel the `KeyManager` subscriptions; [GimbalManager.aim] sees it
     * only as `GimbalAim`.
     */
    private var gimbalAim: MsdkGimbalAim? = null

    /** The bottom auxiliary lamp, up with the link and gone with it. */
    private var light: LightControl? = null

    /**
     * The **commanded** camera pitch, remembered on the way to the wire — see
     * [com.dimensional.mini4pro.gimbal.CommandedGimbalPort]. The one thing `vision/` is allowed to
     * know about where the camera is pointing, because the reported angle goes silent exactly
     * during a nadir hold.
     */
    private var commandedGimbal: com.dimensional.mini4pro.gimbal.CommandedGimbalPort? = null

    /**
     * **The camera pitch this bridge believes — one owner, no local resolution.** A pass-through
     * to [GimbalManager.believedPitch], which is where the commanded/reported precedence lives
     * (`gimbal/PitchBelief`; commanded wins, reported is the fallback, null is never zero). The
     * detector's `CameraPose`, the guided engine's nadir gate and the Zenoh `tf` camera edge all
     * read exactly this, which is what keeps the fix ladder and the arm gate structurally unable
     * to disagree about where the camera points. Any second resolution appearing beside this is
     * the two-places-for-one-property failure and must be deleted, not tested around.
     */
    private fun believedCameraPitch(): com.dimensional.mini4pro.gimbal.PitchBelief? =
        gimbal.believedPitch()

    /**
     * **The on-board tag detector**, up with the link and gone with it, like everything else that
     * can reach the MSDK.
     *
     * Null when the native library is absent, which is a build-configuration failure and not a
     * detector result — [MainActivity] says "no on-board detector" rather than "tag not seen", and
     * an operator deciding whether to trust a descent needs those two apart.
     */
    @Volatile
    var tagRecogniser: com.dimensional.mini4pro.vision.TagRecogniser? = null
        private set

    /**
     * M3 Stage A: `MANUAL_CONTROL` → DJI virtual stick, behind [commandInterlock] — the same
     * single switch as Return and Land (M3 Q1). Alive exactly as long as the link is, like
     * [flightActions] and for the same reason: sticks can only arrive over a link this object
     * owns, and product presence is checked per call inside `KeyManagerVirtualStickPort`.
     *
     * Torn down **first** in [stop], before anything else DJI-side: it is the one component
     * that can be actively flying the aircraft when stop is pressed, and its teardown aborts
     * the engagement (zero setpoint → disable → announce) rather than merely unsubscribing.
     */
    private var guidedStick: GuidedStickEngine? = null

    // ── what the screen may ask of the engine ─────────────────────────────────
    //
    // Three narrow functions rather than a `val guided: GuidedStickEngine?`. Handing the UI the
    // engine itself would put every command entry point — `reposition`, `orbit`, `roi`,
    // `onInbound` — one dot away from a tap listener, and the situation view is a *consumer* of
    // flight state that must never become a second source of it. What the screen is allowed is:
    // read what is being flown, and withdraw it. There is deliberately nothing here that starts
    // anything (`docs/decisions/2026-07-27-m4-eighteen-answers.md` M4-2 puts Start in QGC).

    /**
     * A flat immutable copy of the manoeuvre in progress, or [GuidedSituation.IDLE] when the
     * bridge is stopped. Never null, so a caller has no reason to branch on the bridge's
     * lifecycle to draw a picture.
     */
    fun guidedSituation(): GuidedSituation = guidedStick?.situation() ?: GuidedSituation.IDLE

    /**
     * **Pause, from the phone.** The same `GuidedStickEngine.pause()` QGC's pause-shaped
     * `COMMAND_LONG` reaches — one implementation, one flight-record event, one `STATUSTEXT`.
     * False when there was nothing of ours to pause, which the caller shows rather than hides.
     */
    fun pauseManoeuvre(): Boolean =
        guidedStick?.pause() == com.dimensional.mini4pro.command.Verdict.ACCEPTED

    /**
     * **Arm the tag-tracked descent, from the phone** — M3 Stage D's one arm surface. The
     * decision is entirely `GuidedStickEngine.armTagDescent`'s (every gate, every named
     * refusal); this is routing, exactly as the `DO_REPOSITION` handler is routing. Null when
     * the bridge is stopped, which the caller shows rather than hides.
     *
     * The `enable()`-in-the-UI-layer rule does not apply here in reverse: arming a descent is
     * gated on the interlock *inside the engine*, so this method being callable from anywhere
     * changes nothing — with the interlock off it answers `UNSUPPORTED`, as every command does.
     *
     * [fullAutoland] is Stage C's explicit operator option (the screen's own toggle, **on by
     * default since 2026-07-30** — Ivan's ask, and only the switch's starting position moved;
     * the option is still taken per arm, behind the dialog that names where the flight ends):
     * the descent will not stop at the terminal hold but commit to a landing on the tag. An accepted full-autoland arm also plants `MsdkFlightActions`' confirmation
     * subscriptions ([MsdkFlightActions.armAutolandListening]) — that descent never calls
     * `land()`, so the lazy first-`land()` subscription would leave DJI's confirmation window
     * unobserved for exactly the flight that exists to measure it.
     */
    fun armTagDescent(fullAutoland: Boolean = false): com.dimensional.mini4pro.command.Verdict? {
        // Through the engine's phone door, which stamps ControlOrigin.PHONE — the label the
        // liveness watchdogs judge the engagement by (landing08: the same arm routed through
        // the origin-less spelling was refused LINK_DOWN six times on a QGC-less flight).
        val verdict = guidedStick?.armTagDescentFromPhone(fullAutoland = fullAutoland)
        if (fullAutoland && verdict == com.dimensional.mini4pro.command.Verdict.ACCEPTED) {
            flightActions?.armAutolandListening()
        }
        return verdict
    }

    /** Disarm the descent — a withdrawal, immediate, no dialog. False when nothing was armed. */
    fun disarmTagDescent(): Boolean =
        guidedStick?.disarmTagDescent() == com.dimensional.mini4pro.command.Verdict.ACCEPTED

    /**
     * **Take off, from the phone** — the screen's one takeoff surface, and routing only: the
     * decision is entirely `CommandDispatcher.takeoffFromPhone`'s, which is the same corridor
     * QGC's `MAV_CMD_NAV_TAKEOFF` travels (one range gate, one `perform`, one climb arming),
     * plus the phone sequence's camera-at-nadir flag that fires at DJI's handback. Height is
     * the dispatcher's own [CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M] — 10 m, Ivan's default.
     *
     * The ask rides the flight record here ([EventCode.PHONE_TAKEOFF]) because this is
     * the one takeoff origin with no inbound MAVLink line to witness it — verdict included, so
     * a refused press is as readable afterwards as an accepted one. Contained like every
     * recorder tap: an evidence problem must never become a command problem.
     *
     * The `enable()`-in-the-UI-layer rule does not apply in reverse, exactly as [armTagDescent]
     * argues: the interlock is re-checked inside the dispatcher, so this being callable from
     * anywhere changes nothing — with the interlock off it answers `UNSUPPORTED`, and the
     * screen names the interlock on it.
     */
    fun takeoffFromPhone(): com.dimensional.mini4pro.command.Verdict {
        val verdict = commands.takeoffFromPhone()
        try {
            Recorder.event(
                EventCode.PHONE_TAKEOFF,
                "relAlt=%.1f -> %s".format(CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M, verdict),
                if (verdict == com.dimensional.mini4pro.command.Verdict.ACCEPTED) {
                    LogEntry.SEV_INFO
                } else {
                    LogEntry.SEV_WARN
                },
            )
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder phone-takeoff event failed", e)
        }
        return verdict
    }

    /**
     * **Shadow mode** — the descent controller computing everything and actuating nothing,
     * for the validation flight where the operator hand-flies a tag landing while the
     * controller's would-be commands go to the flight record. Routing only; the semantics are
     * `GuidedStickEngine.setShadowDescent`'s.
     */
    fun setShadowDescent(on: Boolean): Boolean = guidedStick?.setShadowDescent(on) ?: false

    /** The comparison view's feed, or null when shadow mode is off or the bridge is stopped. */
    fun shadowComparison(): com.dimensional.mini4pro.guided.GuidedStickEngine.ShadowComparison? =
        guidedStick?.shadowComparison()

    /**
     * **Stop, from the phone** — the abort ladder's second rung, pulled deliberately.
     *
     * Disables the interlock *and* aborts, in that order, and both are idempotent. Disabling
     * alone would be enough — the engine's tick sees a dead interlock and aborts within 100 ms
     * — but a panic control should not have a tick of latency in it, and the explicit abort
     * costs nothing when the tick gets there first.
     *
     * That it disarms as well as stops is the decision rather than a side effect. A hand
     * reaching for STOP wants the aircraft to stop doing things, not to stop doing this
     * particular thing and remain available for the next command that arrives; and re-arming is
     * one deliberate switch away. `DisengageReason.INTERLOCK` is DJI-facing truth here — the
     * app switch really is what ended it.
     */
    fun stopManoeuvre() {
        commandInterlock.disable()
        guidedStick?.abort(GuidedStickEngine.DisengageReason.INTERLOCK, "stopped from the phone")
    }

    /**
     * The engine's recorder hooks, contained like every other recorder tap: an evidence
     * problem must never become a control problem, so throws are logged and swallowed here
     * rather than reaching the engine's tick.
     */
    private val guidedRecord = object : GuidedRecord {
        override fun stickCmd(
            setpoint: Setpoint?,
            axes: StickAxes,
            modes: StickModes,
            source: CommandSource?,
            range: StickRange?,
            accepted: Boolean?,
            error: String?,
        ) {
            try {
                Recorder.stickCmd(
                    setpoint = setpoint,
                    axes = axes,
                    modes = modes,
                    source = source,
                    range = range,
                    path = StickPath.ADVANCED_PARAM,
                    accepted = accepted,
                    error = error,
                )
            } catch (e: Throwable) {
                Log.w(TAG, "flight recorder stick_cmd failed", e)
            }
            // The record first, the bus second, unconditionally — `RecordedTagSink`'s ordering,
            // for its reason: the line the message must be reproducible from is on the queue
            // before the bus sees the send. `publishSetpoint` contains its own throws and
            // returns in nanoseconds when the bus is down; the `accepted = null` shadow lines
            // are refused inside it by name (Withheld.NOT_SENT), not filtered here.
            ZenohBus.publishSetpoint(setpoint, accepted)
        }

        override fun event(code: String, message: String?, warn: Boolean) {
            try {
                Recorder.event(code, message, if (warn) LogEntry.SEV_WARN else LogEntry.SEV_INFO)
            } catch (e: Throwable) {
                Log.w(TAG, "flight recorder guided event failed", e)
            }
        }
    }

    /** Remembers the home/origin we published, so those go out on change only. */
    private val homeEvents = HomeEventGate()

    /** Drives the per-message rate divisors. Only touched on the tx thread. */
    private var tickCount = 0

    /**
     * [context] is only used to hold a WiFi/CPU lock for the life of the link.
     * Without it Android parks the radio and the ground station's traffic is
     * silently dropped — [LinkLocks] has the measurements.
     *
     * @param video the camera passthrough, or null/disabled for telemetry only.
     *   See [startVideo] for why it lives on this call rather than beside it.
     */
    fun start(
        host: String,
        port: Int = MavlinkLink.DEFAULT_GCS_PORT,
        context: Context? = null,
        video: VideoRequest.Plan? = null,
        zenoh: ZenohSettings.Plan? = null,
    ) {
        if (_state.value.running) return

        // **The live source, installed once.** Two loops on two threads read it — this object's
        // tick and `ZenohBus`'s sampler — and before this line they each held their own copy of
        // the identical expression. It is installed here rather than at construction because
        // `Msdk` and `StateCache` are the Android/DJI half and `StateSource` is deliberately
        // neither; and it is never cleared, because it retains nothing and already answers a
        // stopped bridge with the same all-null state a caller would have built.
        StateSource.live = {
            if (Msdk.state.value.registered) StateCache.aircraftState() else AircraftState()
        }

        // **The wind source, installed once**, on the same argument: the recorder owns the tap and
        // the on-change dedup, `warn/WindWarnings` owns the mapping, and this line is the whole of
        // the wiring between them. Never cleared, for `deviceHealth`'s reason — a subscription
        // that only *observes* is allowed to outlive a link, and a warning standing when the
        // operator restarts the bridge is exactly the one they most need to still be there.
        Recorder.windWarningSink = { state, speedDmS ->
            warnings.deliver(WarnSource.WIND, WindWarnings.picture(state, speedDmS))
        }

        // Before the socket, so no datagram is ever sent to a parked radio.
        context?.let { locks = LinkLocks(it).apply { acquire() } }

        // wifi-fix.md gotcha #2: an unbound socket follows Android's *default*
        // network, and Android 16 moves that to cellular whenever WiFi fails the
        // router's validation — telemetry addressed to the LAN then exits via LTE
        // and dies. So the socket is bound to the WiFi Network, and with no WiFi
        // network the bridge refuses to start: the operator reads WHY there is no
        // telemetry instead of watching a link that looks up while sending into a
        // carrier network. The wait is milliseconds when WiFi is associated
        // (WifiNetworkGate.awaitNetwork), the full WAIT_MS only on the refusal path.
        var network: Network? = null
        if (context != null) {
            val tracker = WifiBindTracker()
            wifiTracker = tracker
            val gate = WifiNetworkGate(
                context,
                onWifiAvailable = ::onWifiAvailable,
                onWifiLost = { net ->
                    tracker.lost(net.toString())?.let { reportWifi(EventCode.WIFI_LOST, it) }
                },
            )
            wifiGate = gate
            gate.start()
            network = gate.awaitNetwork(WifiBind.WAIT_MS)
            if (network == null) {
                val why = WifiBind.refusalStatus(WifiBind.WAIT_MS)
                Log.e(TAG, "not starting: $why")
                recordWifiEvent(EventCode.WIFI_REFUSED, why)
                teardownWifiGate()
                locks?.release()
                locks = null
                _state.value = _state.value.copy(running = false, wifi = why)
                return
            }
        }

        val boundNet = network
        // `tap = Recorder` is not optional and could not be omitted: the link records both
        // directions itself, on the way past, and the constructor will not build one without a
        // tap. What used to be here — an `onSent` lambda installed after construction and a
        // `Recorder.mavIn` at the top of `onInbound` — were both conventions, and this project's
        // standing proof of what a convention is worth is the gimbal.
        val newLink = if (boundNet != null) {
            MavlinkLink(
                host, port, onMessage = ::onInbound, tap = Recorder,
                socketFactory = WifiBind.socketFactory { boundNet.bindSocket(it) },
            )
        } else {
            // No Context means no ConnectivityManager to ask. No app path gets
            // here — BridgeService always passes one — and the state line below
            // says UNBOUND so a non-Android caller cannot mistake this for bound.
            MavlinkLink(host, port, onMessage = ::onInbound, tap = Recorder)
        }
        // The parameter table is fixed for one link session, and this is the only instant at
        // which it can honestly be built: the MSDK has had a chance to register and deliver its
        // settings keys, and the socket below is not open yet, so no ground station can be
        // part-way through a download while param_count changes. Aircraft-derived parameters
        // whose key has not arrived are omitted for this session and logged — see
        // ParameterStore.forAircraft. Reconnecting is what gets them.
        handshake.rebindParameters(
            ParameterStore.forAircraft(
                state = ::aircraftState,
                log = { msg -> Log.i(TAG, "params: $msg") },
            )
        )

        try {
            newLink.start()
        } catch (e: Exception) {
            Log.e(TAG, "failed to open link", e)
            teardownWifiGate()
            // Do not leave the radio pinned awake for a link that never opened.
            locks?.release()
            locks = null
            _state.value = _state.value.copy(running = false, error = e.message)
            return
        }
        link = newLink
        // A new link means a ground station that has been told nothing.
        homeEvents.reset()
        // Drops any transaction left mid-flight by the previous link and re-arms JC-3's
        // once-per-session clear announcement. **It does not touch the store**: QGC re-reads the
        // plan on every connect, and a plan that died with the socket would show an empty Plan
        // view for an aircraft that is flying one (§3.2).
        missionTransaction.reset()
        missionProgress.reset()

        tickCount = 0
        timer = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "mavlink-tx").apply { isDaemon = true }
        }.also { exec ->
            exec.scheduleAtFixedRate(::tick, 0, BASE_PERIOD_MS, TimeUnit.MILLISECONDS)
            // Counters go to the UI at a human rate, not the telemetry rate.
            exec.scheduleAtFixedRate(::publishCounters, 500, 500, TimeUnit.MILLISECONDS)
            exec.scheduleAtFixedRate(
                ::mirrorTick, MIRROR_PERIOD_MS, MIRROR_PERIOD_MS, TimeUnit.MILLISECONDS,
            )
        }

        // The DJI half of M2 comes up with the link and not before: while stopped there is no
        // route by which a command could arrive, and no listener to leak. Note this only
        // *enables asking*: with the interlock off nothing reaches it, and with no product
        // connected every call refuses with NO_PRODUCT before any KeyManager touch.
        flightActions = MsdkFlightActions(
            // `actionPort(Recorder)`, not `KeyManagerActionPort()` — see `actionPort`. Every
            // takeoff, land, confirm-landing and return, and DJI's answer or its silence, is now
            // one correlated pair of `dji_call` lines rather than prose in an `event`.
            port = actionPort(Recorder),
            reportAsyncDjiError = commands::reportAsyncDjiError,
            announceLandingConfirmed = commands::reportLandingConfirmed,
            simulatedFlight = ::simulatorReportsRunning,
            // Stage C's guided auto-confirm scope: the same single interlock as every command,
            // read at the moment of the confirm; the engine's live LANDING facts, read fresh at
            // the moment of DJI's question; the confirm's acceptance noted back onto the
            // engagement's timeline; and the measurement lines onto the flight record.
            interlockEnabled = { commandInterlock.enabled },
            autolandClearance = { guidedStick?.autolandClearance() },
            onAutolandConfirmed = { guidedStick?.noteAutolandConfirmed() },
            recordEvent = { code, message, warn ->
                Recorder.event(code, message, if (warn) LogEntry.SEV_WARN else LogEntry.SEV_INFO)
            },
            log = { msg -> Log.i(TAG, msg) },
        ).also { commands.actions = it }

        // The gimbal's DJI half, on the same terms: it comes up with the link, it subscribes to
        // KeyManager only once MSDK registration has completed (checked inside MsdkGimbalAim, not
        // here), and until DJI reports a gimbal attitude nothing is advertised to the ground
        // station at all.
        // `RecordedGimbalPort` wraps the DJI port for the same reason `actionPort(Recorder)`
        // does above: the ask belongs on the record, and it belongs there on the way to the wire
        // rather than beside the decision. Until this was wired, `DjiOp.GIMBAL_ROTATE` had never
        // been written by anything — the camera had been aimed for weeks with no commanded angle
        // in any record, which is the gap `tools/memexport` hit building the camera frame.
        // `CommandedGimbalPort` outside `RecordedGimbalPort`, so the record still sees every ask
        // including ones the commanded-angle tracker declines to remember, and so the tracker only
        // ever learns about asks DJI actually accepted. Two decorators, one wire, no policy in
        // either — see each class's KDoc.
        val commanded = com.dimensional.mini4pro.gimbal.CommandedGimbalPort(
            RecordedGimbalPort(KeyManagerGimbalPort(), Recorder)
        ).also { it.nowNanos = { android.os.SystemClock.elapsedRealtimeNanos() } }
        commandedGimbal = commanded
        gimbalAim = MsdkGimbalAim(
            port = commanded,
            reportAsyncDjiError = gimbal::reportAsyncDjiError,
            log = { msg -> Log.i(TAG, "gimbal: $msg") },
        ).also { gimbal.aim = it }

        startTagRecogniser(context)

        // The bottom auxiliary lamp. Recorded on the way to the wire like every other DJI ask,
        // because the whole reason it exists is a question — does it help a camera see a tag after
        // dark? — and that is answered by comparing detections against when the lamp was actually
        // on, on the same clock as the frames.
        light = LightControl(
            port = RecordedLightPort(KeyManagerLightPort(), Recorder),
            log = { msg -> Log.i(TAG, "light: $msg") },
        )

        // **Closes the one Zenoh channel a flight record could not reproduce.** The gimbal had
        // been aiming a real camera for weeks with not one reading in any log
        // (`replay/ReplayCoverage.GIMBAL`), because `gimbal/` owns the reading behind its own seam
        // and nothing bridged it to the recorder.
        //
        // A supplier installed here rather than a push from `gimbal/`, for three reasons: that
        // package is being edited by someone else and this needs no change there at all;
        // `GimbalAim.reading()` is already the public, immutable, unit-testable snapshot the
        // gimbal chose to expose; and the deadband and the idle heartbeat belong on the
        // *recorder's* clock beside the identical logic for the RC sticks, not on the link's tick
        // (`Recorder.sampleGimbal`). `GimbalSample` is a `record/`-owned type, so the recorder
        // still imports nothing from `gimbal/`.
        //
        // Cleared in `stop()` with the aim it reads, so a stopped bridge records no camera.
        //
        // **Deliberately exempt from `believedCameraPitch()`**: the record's gimbal stream is a
        // measurement *of the gimbal* — raw reported samples, ages and all, the stream the
        // `GimbalRecenter` detector reads — not a claim about where the camera points. Filtering
        // it through the belief would erase exactly the commanded-versus-actual disagreements a
        // post-flight reader needs to see.
        Recorder.gimbalSource = {
            gimbalAim?.reading()?.let {
                GimbalSample(
                    pitchDeg = it.pitchDeg,
                    rollDeg = it.rollDeg,
                    yawDeg = it.yawDeg,
                    ageMs = it.attitudeAgeMs,
                )
            }
        }

        // **The `tf` tree's camera edge, on the same single believed-pitch owner as everything
        // else.** Installed here for `Recorder.gimbalSource`'s three reasons, and cleared with
        // it. This lambda used to resolve commanded-versus-reported itself; since 2026-07-28 the
        // choice is `GimbalManager.believedPitch` — the one implementation the arm gate and the
        // fix pipeline also read — and what remains here is only the *edge composition* per
        // source, which is genuinely this consumer's own (`docs/mem2-converter.md` §2.2):
        //
        //  - **COMMANDED**: roll zero and yaw from the **aircraft's** heading, because the
        //    Mini 4 Pro's gimbal has no usable independent roll or yaw axis and its yaw follows
        //    the nose. No age — a command has none. Note the commanded half is now the
        //    success-stamped `CommandedGimbalPort` record rather than the old ask-stamped
        //    dispatch memory: a rotate DJI refused no longer holds this edge, which is strictly
        //    more honest and the only behavioural change of the refit.
        //  - **REPORTED**: DJI's own roll/yaw/pitch with the delivery age carried, so a consumer
        //    auditing a held edge can see what it was held from.
        //
        // What is never done is substituting a zero for an angle nobody knows.
        ZenohBus.gimbalSource = {
            when (val belief = believedCameraPitch()) {
                null -> null
                else ->
                    if (!belief.reported) {
                        com.dimensional.mini4pro.zenoh.GimbalEarthAttitude.of(
                            rollDeg = 0.0,
                            pitchDeg = belief.pitchDeg,
                            yawDeg = aircraftState().yawDeg,
                            source = com.dimensional.mini4pro.zenoh.GimbalEarthAttitude.Source.COMMANDED,
                        )
                    } else {
                        gimbalAim?.reading()?.let {
                            com.dimensional.mini4pro.zenoh.GimbalEarthAttitude.of(
                                rollDeg = it.rollDeg,
                                pitchDeg = belief.pitchDeg,
                                yawDeg = it.yawDeg,
                                source = com.dimensional.mini4pro.zenoh.GimbalEarthAttitude.Source.REPORTED,
                                ageMs = it.attitudeAgeMs,
                            )
                        }
                    }
            }
        }

        // M3 Stage A comes up with the link on the same terms as flightActions: nothing here
        // *enables* anything — with the interlock off every deflection is refused with a
        // sentence, and engagement additionally requires DJI to confirm authority. The engine
        // owns its own 10 Hz thread (architecture.md: the guided controller is control, not
        // telemetry, and does not ride Bridge's 200 ms tick).
        guidedStick = GuidedStickEngine(
            port = KeyManagerVirtualStickPort(log = { msg -> Log.i(TAG, "guided-port: $msg") }),
            interlockEnabled = { commandInterlock.enabled },
            aircraftState = ::aircraftState,
            announcer = announcer,
            headingFollowsCourse = { headingFollowsCourse.get() },
            // Phase one of a mission's NAV_TAKEOFF, and the only way the guided engine can start a
            // motor. `MsdkFlightActions.takeoff` is the *same* call QGC's Takeoff button makes —
            // one implementation, one set of DJI-side refusals, one place the `KeyStartTakeoff`
            // flakiness (DJI #783) is handled — and DJI's own words reach the executor verbatim.
            missionTakeoff = com.dimensional.mini4pro.guided.MissionTakeoff { onFailure ->
                when (val outcome = flightActions?.takeoff()) {
                    null -> onFailure("NO_ACTIONS")
                    is com.dimensional.mini4pro.command.ActionOutcome.Requested -> Unit
                    is com.dimensional.mini4pro.command.ActionOutcome.Refused -> onFailure(outcome.djiError)
                    is com.dimensional.mini4pro.command.ActionOutcome.Unavailable -> onFailure(outcome.reason)
                }
            },
            // Stage C's commit seam: the guided engine's full autoland hands the landing to
            // DJI through the SAME MsdkFlightActions path QGC's Land button uses — one
            // implementation, one claim machinery (an accepted start becomes "ours to
            // confirm"), one set of refusals — and rule 1's withdrawal is the newly-wired
            // KeyStopAutoLanding. Null-actions answers by name rather than by silence.
            djiLanding = object : com.dimensional.mini4pro.guided.DjiLanding {
                override fun start(): String? =
                    when (val outcome = flightActions?.land()) {
                        null -> "NO_ACTIONS"
                        is com.dimensional.mini4pro.command.ActionOutcome.Requested -> null
                        is com.dimensional.mini4pro.command.ActionOutcome.Refused -> outcome.djiError
                        is com.dimensional.mini4pro.command.ActionOutcome.Unavailable -> outcome.reason
                    }

                override fun stop(): String? =
                    when (val outcome = flightActions?.cancelLanding()) {
                        null -> "NO_ACTIONS"
                        is com.dimensional.mini4pro.command.ActionOutcome.Requested -> null
                        is com.dimensional.mini4pro.command.ActionOutcome.Refused -> outcome.djiError
                        is com.dimensional.mini4pro.command.ActionOutcome.Unavailable -> outcome.reason
                    }
            },
            // M3 Stage C: the orbit holds the circle's centre in frame. Two thin calls into the
            // gimbal manager that already owns the DJI half — no attitude, no age, no feedback,
            // because `KeyGimbalAttitude` is change-driven and a steady orbit is exactly when it
            // goes silent (`guided/ManoeuvreGimbal` has the argument in full).
            manoeuvreGimbal = object : ManoeuvreGimbal {
                override fun pitchRangeDeg(): ClosedFloatingPointRange<Double>? =
                    gimbal.reportedPitchRange()

                override fun aimPitch(pitchDeg: Double) = gimbal.aimForManoeuvre(pitchDeg)
            },
            // M3 Stage D: the tag-tracked descent's sensor seam. Read fresh on every decision —
            // the recogniser starts and stops independently of the engine — and the age is
            // computed *here, at read time*, from the fix's own monotonic stamp, because a
            // sighting is already 60–160 ms old when it lands and the engine's staleness ladder
            // must see the true number (`vision/TagSighting.Sighting.ageMillisAt`).
            tagSense = {
                tagRecogniser?.let { r ->
                    val held = r.latched()
                    val fix = r.latestFix()
                    com.dimensional.mini4pro.guided.TagDescentSense(
                        latched = held != null,
                        latchedTagId = held?.tagId,
                        fixTagId = fix?.tagId,
                        fixNorthM = fix?.northM,
                        fixEastM = fix?.eastM,
                        fixAgeMs = fix?.let {
                            (android.os.SystemClock.elapsedRealtimeNanos() - it.atNanos) / 1_000_000
                        },
                        // The range ladder's facts, read off the fix that resolved them
                        // (TagFix.rangeSource / tagRangeM()) — the engine's height law flies
                        // the same number that scaled this fix's lateral, landing07's
                        // consistency requirement.
                        fixTagRangeM = fix?.tagRangeM(),
                        fixRangeSource = fix?.rangeSource,
                    )
                }
            },
            // **The same believed-pitch resolution the detector's own CameraPose reads**
            // (startTagRecogniser, below; `gimbal/PitchBelief` owns the precedence), so the
            // engine's nadir gate and `TagWorld.fix`'s nadir refusal judge the identical number
            // — commanded when this bridge has aimed the camera, the last-reported attitude when
            // it has not (the RC-wheel session), and null only when neither exists, which the
            // gate refuses by naming the camera rather than the tag.
            cameraPitchDeg = { believedCameraPitch()?.pitchDeg },
            // Stage C's landing gimbal watchdog reads the REPORTED pitch — the one consumer of
            // a gimbal measurement in the engine, admissible because it reads only the value
            // and never an age (the change-driven trap is about ages), and because the event
            // it watches for — DJI's own landing recenter — is a movement, which is exactly
            // what a change-driven key delivers (measured, landingdata.md §2.4).
            //
            // **Deliberately exempt from `believedCameraPitch()`**, and it must stay so: the
            // belief lets a commanded −90° outrank the reported angle, and DJI moving the
            // camera against our own commanded −90° is precisely the event this watchdog
            // exists to catch. A measurement of the gimbal, not a claim about pointing.
            gimbalReportedPitchDeg = { gimbalAim?.reading()?.pitchDeg },
            record = guidedRecord,
            log = { msg -> Log.i(TAG, "guided: $msg") },
            nowMs = { SystemClock.elapsedRealtime() },
        ).also { it.start() }

        // M3 Stage B: QGC's Go-to (COMMAND_INT 192) and its Pause form, routed to the same
        // engine so exactly one component ever holds virtual-stick authority. The handler runs
        // on the mavlink-rx thread and its return value is the COMMAND_ACK — the engine keeps
        // the ordering rule (ACCEPTED only when the target is actually taken). Registration
        // overwrites on every start; after stop() `guidedStick` is null and the reply reverts
        // to the pre-Stage-B UNSUPPORTED.
        handshake.registerCommandHandler(RepositionCommand.MAV_CMD_DO_REPOSITION) { req ->
            guidedStick?.reposition(
                RepositionCommand(
                    isCommandInt = req.isCommandInt,
                    frame = req.frame,
                    latE7 = req.x,
                    lonE7 = req.y,
                    zAmslM = req.param7,
                    groundSpeedMs = req.param1,
                    yawRad = req.param4,
                    param5 = req.param5,
                    param6 = req.param6,
                ),
                // Named rather than defaulted: this class is the MAVLink transport, and it is
                // the transport's job to say whose liveness its traffic is evidence of.
                ControlOrigin.MAVLINK,
            // The MAVLink edge of the engine's transport-neutral verdict. The `?:` is unchanged:
            // no engine means no Stage B, and the reply reverts to the pre-Stage-B UNSUPPORTED.
            )?.toMavResult() ?: MavResult.MAV_RESULT_UNSUPPORTED
        }

        // M3 Stage C: QGC's Orbit button (COMMAND_INT 34), routed to the same engine for the same
        // reason — exactly one component ever holds virtual-stick authority. Measured on the wire
        // 2026-07-27 and, until now, answered with a bare UNSUPPORTED and **no STATUSTEXT**: the
        // only refusal in this project that never told the operator why. Registering the handler
        // is what closes that, because every path out of `orbit()` except the interlock carries a
        // sentence.
        handshake.registerCommandHandler(OrbitCommand.MAV_CMD_DO_ORBIT) { req ->
            guidedStick?.orbit(
                OrbitCommand(
                    isCommandInt = req.isCommandInt,
                    frame = req.frame,
                    radiusM = req.param1,
                    velocityMs = req.param2,
                    yawBehaviour = req.param3,
                    turns = req.param4,
                    latE7 = req.x,
                    lonE7 = req.y,
                    zAmslM = req.param7,
                ),
                ControlOrigin.MAVLINK,
            )?.toMavResult() ?: MavResult.MAV_RESULT_UNSUPPORTED
        }

        // M4: the ROI — the operator clicks a point on QGC's map and the camera holds it. Three
        // command ids, one handler, one engine, for the reason the other two guided commands are
        // registered here: exactly one component may point the camera and exactly one may hold
        // virtual-stick authority, and an ROI needs the second only for its yaw half.
        //
        // QGC's ROI button is **already on the operator's screen** — `PX4FirmwarePlugin::isCapable`
        // grants `ROIModeCapability` from the vehicle type alone for any multirotor, so nothing has
        // to be advertised — and until now it was answered with a bare UNSUPPORTED. As with
        // `DO_ORBIT`, registering the handler is what turns a silent refusal into an answer.
        //
        // Note what is *not* here: an interlock check. Aiming a camera is not flying an aircraft,
        // and `GuidedStickEngine.roi` has the argument in full.
        for (roiCommand in listOf(
            RoiCommand.MAV_CMD_DO_SET_ROI_LOCATION,
            RoiCommand.MAV_CMD_DO_SET_ROI_NONE,
            RoiCommand.MAV_CMD_DO_SET_ROI,
        )) {
            handshake.registerCommandHandler(roiCommand) { req ->
                guidedStick?.roi(
                    RoiCommand(
                        command = roiCommand,
                        isCommandInt = req.isCommandInt,
                        frame = req.frame,
                        latE7 = req.x,
                        lonE7 = req.y,
                        param1 = req.param1,
                        param5 = req.param5,
                        param6 = req.param6,
                        // `COMMAND_INT.z` / `COMMAND_LONG.param7`. Whether it is a height we can read
                        // is the frame's business and `RoiCommand.relativeAltMOrNull`'s decision —
                        // QGC's Fly-view ROI sends an AMSL in frame 0 and it stays discarded; a
                        // relative-frame z is our own datum and is used.
                        z = req.param7,
                    ),
                    ControlOrigin.MAVLINK,
                )?.toMavResult() ?: MavResult.MAV_RESULT_UNSUPPORTED
            }
        }

        // §1.6: set-current has two forms, and **our first ack decides which one QGC speaks to us
        // for the lifetime of the vehicle instance**. `sendMavCommandWithLambdaFallback` caches
        // UNSUPPORTED and falls back forever to the reply-less `MISSION_SET_CURRENT` message
        // (`MavCommandQueue.cc:129-160`), so answering the *command* is what keeps a channel
        // through which a refusal can be heard — the same argument that makes `DO_REPOSITION`
        // better than `SET_MODE`. The executor does the refusing; today it is the stub, and it
        // answers DENIED rather than UNSUPPORTED precisely so the fallback is never cached.
        //
        // Registered here rather than at construction because `registerCommandHandler` is
        // `handshake/`'s hook and every other actuating registration is made on this line too.
        handshake.registerCommandHandler(MissionCommands.DO_SET_MISSION_CURRENT) { req ->
            missionTransaction.onSetCurrentCommand(req.param1.toInt())
        }

        _state.value = State(
            running = true,
            target = "$host:$port",
            wifi = boundNet?.let { wifiTracker?.bound(it.toString()) }
                ?: "UNBOUND — started without a Context; socket follows the default network",
        )
        Log.i(TAG, "bridge started → $host:$port (base ${BASE_PERIOD_MS}ms, wifi=${_state.value.wifi})")

        // Last, and only once the link is genuinely up. See startVideo.
        startVideo(video, boundNet)
        startZenoh(zenoh, boundNet, context)
    }

    /**
     * Brings the Zenoh bus up alongside the link.
     *
     * **Its failure does not refuse the bridge**, and that asymmetry against the WiFi gate above
     * is deliberate rather than inconsistent (`docs/zenoh-dimos-transport.md` §5). An unbound
     * socket is a *positive hazard* — telemetry leaving over the operator's mobile data — so the
     * bridge refuses to start without a WiFi network. An absent Zenoh session is simply a bridge
     * doing less: MAVLink still flies the aircraft, the recorder still records, and the only thing
     * missing is a subscriber's view. So this function cannot fail; [ZenohBus.start] returns
     * having opened nothing, and the session is the publisher thread's problem forever after.
     *
     * In Zenoh-only mode the WiFi gate still refuses, unchanged. Zenoh to a LAN peer over cellular
     * is the same bug as MAVLink to a LAN peer over cellular.
     *
     * **A transport cannot be enabled or disabled while the bridge runs**, which is why this is
     * here and not on a switch. Once there is an inbound half, enabling one mid-flight would
     * create a controller the per-origin watchdog has never seen, and disabling one could orphan
     * an active engagement. Stop and start is one tap, and it already disables the interlock —
     * which is exactly the property you want at the moment the command surface changes shape.
     */
    private fun startZenoh(plan: ZenohSettings.Plan?, network: Network?, context: Context?) {
        if (plan == null || !plan.enabled) {
            // Not a no-op, for the same reason `startVideo`'s branch is not: a previous session
            // may have left it running, and a bus that outlived the link it followed would be
            // publishing an aircraft this bridge has let go of.
            ZenohBus.stop()
            return
        }
        plan.warning?.let {
            Log.w(TAG, "zenoh: $it")
            recordZenohEvent(EventCode.ZENOH_PHASE, it, LogEntry.SEV_WARN)
        }
        // The WiFi interface's own IPv4, as zenoh's `#bind=`. **Not the equivalent of the socket
        // bind MAVLink gets** — zenoh-kotlin has no socket-factory hook at all, so this pins the
        // source address and cannot select Android's per-network route table. `ZenohConfig` states
        // the difference in full; it is written down rather than relied on.
        val bind = if (network != null && context != null) wifiIpv4(network, context) else null
        if (bind == null) {
            Log.w(TAG, "zenoh: no WiFi IPv4 to bind to — the session follows the default route")
        }
        Log.i(TAG, "zenoh: starting → ${plan.endpoint} as ${plan.prefix}")
        if (plan.video) {
            Log.w(TAG, "zenoh: video channel ON — this doubles the uplink; see docs/measurements")
            recordZenohEvent(
                EventCode.ZENOH_PHASE,
                "video channel on: ~5.9 Mbit/s of H.264 in addition to the RTP stream QGC " +
                    "already receives from the same bytes",
                LogEntry.SEV_WARN,
            )
        }
        ZenohBus.start(
            config = plan.config(bindAddress = bind),
            cfg = ZenohBus.Config(video = plan.video, detections = plan.detections),
            // The one fan-out every operator-facing sentence goes through. Handing it here is
            // what makes the `status` channel structural: a refusal composed anywhere in the
            // command layer reaches this bus without anybody remembering to wire it up.
            announcer = announcer,
        )
    }

    /**
     * The bound WiFi network's IPv4 address, or null.
     *
     * IPv4 specifically: the router endpoint is `tcp/10.55.1.50:7447`, and zenoh's `#bind=` must
     * name an address of the same family as the destination or the connect fails outright — which
     * would turn a best-effort improvement into a refusal to publish at all. A link-local or
     * loopback address is refused for the same reason.
     */
    private fun wifiIpv4(network: Network, context: Context): String? = try {
        val cm = context.getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager
        cm?.getLinkProperties(network)?.linkAddresses
            ?.map { it.address }
            ?.filterIsInstance<java.net.Inet4Address>()
            ?.firstOrNull { !it.isLoopbackAddress && !it.isLinkLocalAddress }
            ?.hostAddress
    } catch (e: Throwable) {
        Log.w(TAG, "zenoh: could not read the WiFi address", e)
        null
    }

    private fun recordZenohEvent(code: String, message: String, severity: String) {
        try {
            Recorder.event(code, message, severity)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder zenoh event failed", e)
        }
    }

    /**
     * Brings the camera passthrough up alongside the link, on the same network.
     *
     * **Why video is tied to the bridge at all, rather than being its own switch
     * on the status screen.** Three reasons, in order of weight.
     *
     * 1. *They share a hard precondition, and it is the interesting one.* The
     *    socket must be bound to the WiFi `android.net.Network` or Android 16
     *    routes it over cellular the moment WiFi fails the router's validation
     *    (`wifi-fix.md` gotcha #2, and [RtpVideoSink.udp]). This function is
     *    downstream of the one place in the app that has waited for that network
     *    and holds it. A separate video control would have to acquire it again,
     *    and the version of that which "mostly works" is the one that quietly
     *    sends the aircraft's camera out over the operator's mobile data.
     * 2. *It matches how everything else here behaves.* `flightActions`,
     *    `gimbalAim` and the parameter table all come up with the link and go down
     *    with it, for the same reason: the link is the session. Video following a
     *    fourth, independent lifecycle would be the odd one out.
     * 3. *The screen cannot afford another control.* The status strip already
     *    carries an interlock switch and three buttons, and vertical space is the
     *    scarce thing in landscape on the RC. The setting lives in the Settings
     *    dialog next to the GCS address, which is where it belongs anyway — it is
     *    set once and then only read.
     *
     * **The cost, stated plainly:** video cannot run without telemetry, including
     * on the path where [start] *refuses* over missing WiFi. That refusal is the
     * right answer for video too — an unbound RTP socket is worse than no RTP
     * socket — but it does mean "I turned video on and nothing happened" can be
     * caused by something that is not about video. That is exactly what
     * [VideoRequest.statusLine] exists to say out loud on the status screen.
     *
     * The counter-argument, for the record: QGC users reasonably expect video to
     * just work, and off-by-default means a first-time operator sees no picture.
     * Rejected because the default that produces silence is recoverable in one tap
     * from a screen that names the setting, whereas the default that streams
     * 5 Mbit/s of camera over LTE without being asked is recoverable only from a
     * phone bill.
     */
    private fun startVideo(plan: VideoRequest.Plan?, network: Network?) {
        if (plan == null || !plan.enabled) {
            // Not a no-op: a previous session may have left it running, and video
            // that outlived the link it followed would be exactly the surprise
            // this coupling exists to avoid.
            VideoStreamer.stop()
            return
        }
        // Both set before start(), because start() opens the socket synchronously
        // on the main looper and the binder is what keeps it off the carrier.
        VideoStreamer.socketBinder = network?.let { net -> { s -> net.bindSocket(s) } }
        VideoStreamer.eventSink = ::recordVideoEvent
        if (network == null) {
            Log.w(TAG, "video: no bound network — the RTP socket will follow the default route")
        }
        plan.warning?.let {
            Log.w(TAG, "video: $it")
            recordVideoEvent(VideoEvents.PHASE, it, error = false)
        }
        Log.i(TAG, "video: starting passthrough → ${plan.target} (${plan.source})")
        VideoStreamer.start(
            VideoStreamer.Config(
                mode = VideoStreamer.VideoMode.RTP,
                rtp = VideoStreamer.RtpTarget(host = plan.host, port = plan.port),
                // The flight record's video half, when it is on. **Read fresh on every frame**
                // rather than captured here, for the reason `commands`' `climb` is read fresh: the
                // recorder starts and stops independently of the video loop, and a streamer holding
                // a sidecar from a finished session would write frames into a file nobody is
                // indexing. The indirection costs one nullable read per frame, which is the same
                // price `liveSink` already pays to be able to swap the RTP socket underneath.
                rawSink = { data, offset, length, info ->
                    Recorder.videoSink()?.onEncodedFrame(data, offset, length, info)
                    // **The bus's video half, on the same terms and read equally fresh.** Null
                    // whenever the bus is down or the operator has not asked for video, which is
                    // the default; when it is not null the frame is copied onto its own queue and
                    // this thread returns. Nothing here can block on a router — see
                    // `zenoh/ZenohVideoPublisher`.
                    //
                    // The geometry goes across whether or not video is being published: the
                    // `camera_info` intrinsics are worth having when the pixels are not, and this
                    // callback is the only place in the app that learns the stream's resolution.
                    ZenohBus.noteVideoGeometry(info.width, info.height)
                    ZenohBus.videoSink()?.onEncodedFrame(data, offset, length, info)
                },
            ),
        )
    }

    /**
     * **The tag detector**, up with the link and gone with it, on the same terms as everything else
     * that can reach the MSDK.
     *
     * Nothing here *runs* the detector: it attaches to the frame stream and then asks
     * `vision/TagArming` every 250 ms whether it may work. On the ground with the interlock off, the
     * measured cost of being attached and disarmed is **0.06 cores** (0.49 against a 0.43 floor,
     * aircraft, 2026-07-28); armed at the shipped configuration it is 0.68 cores over that floor.
     * That gap is the whole reason the arming rule exists rather than the detector simply always
     * running.
     *
     * ## Why it starts with the bridge rather than with video recording
     *
     * Because the two answer different questions and one of them is not optional. Video recording is
     * an operator's deliberate act with a purpose. The detector is a *sensor* whose whole value is
     * that it was already looking when the aircraft took off — Ivan's rule is that the takeoff is
     * the acquisition opportunity, and an acquisition opportunity you have to remember to enable is
     * one that will be missed on the flight it mattered.
     *
     * It is also the reason the frame listener is attached rather than added and removed as arming
     * changes: `addFrameListener` is what keeps MSDK's decode alive for this app (see
     * `video/CameraStreamTap.start` on `setKeepAliveDecoding`), and churning subscriptions on a
     * stream a ground station is watching is not a trade worth 0.06 cores.
     */
    private fun startTagRecogniser(context: Context?) {
        if (!com.dimensional.mini4pro.vision.AprilTagDetector.available) {
            Log.w(TAG, "tag: libapriltagjni.so did not load — no on-board detector this session")
            return
        }
        // **The session's one camera model, resolved here and nowhere else.** A calibration
        // file in the app's files directory (the flight records' own home) is Ivan's chessboard
        // drop-in; no file is the assumed prior; a malformed file is refused by name, applied
        // not at all, and the session flies the prior — `CameraCalibration`'s KDoc holds the
        // gates and their measurements. Whichever way it resolves, the record says so, because
        // every tag line this session writes rests on it and a post-flight reader must not have
        // to guess which camera model a number came from.
        val loaded = context?.let {
            com.dimensional.mini4pro.vision.CameraCalibration.load(
                it.getExternalFilesDir(null) ?: it.filesDir
            )
        } ?: com.dimensional.mini4pro.vision.CameraCalibration.Loaded.Applied(
            com.dimensional.mini4pro.vision.CameraCalibration.ASSUMED
        )
        val calibration = when (loaded) {
            is com.dimensional.mini4pro.vision.CameraCalibration.Loaded.Applied -> {
                Recorder.event(
                    "camera_calibration",
                    (if (loaded.calibration.measured) "measured" else "assumed") +
                        ": ${loaded.calibration.source} — fx=${loaded.calibration.fxAt1920}" +
                        " fy=${loaded.calibration.fyAt1920} cx=${loaded.calibration.cxAt1920}" +
                        " cy=${loaded.calibration.cyAt1920}",
                )
                loaded.calibration
            }
            is com.dimensional.mini4pro.vision.CameraCalibration.Loaded.Refused -> {
                // Loud on the record and in the log: the operator who pushed the file believes
                // it applied, and a quiet fallback would fly the prior under a measured label.
                Log.w(TAG, "tag: ${loaded.why}")
                Recorder.event(
                    "camera_calibration",
                    "${loaded.why} — flying on the assumed prior",
                    LogEntry.SEV_WARN,
                )
                com.dimensional.mini4pro.vision.CameraCalibration.ASSUMED
            }
        }
        // One Config, so the tag size the recogniser cross-checks range with and the tag size
        // the detector's pose solve scales by cannot drift apart — they are the same marker.
        // The calibration rides the same object for the same reason, doubled: the JNI solve's
        // intrinsics and TagWorld's projection are the same camera or they are nonsense.
        val tagConfig = com.dimensional.mini4pro.vision.TagRecogniser.Config(
            calibration = calibration,
        )
        val recogniser = com.dimensional.mini4pro.vision.TagRecogniser(
            source = com.dimensional.mini4pro.vision.MsdkFrameSource(),
            // A factory, called on the worker thread: `AprilTagDetector` reuses one native image
            // across frames so a detect allocates nothing, which makes it single-threaded by
            // construction. One thread makes it and one thread uses it. `tagSizeM` turns the
            // in-detector pose solve on; what the solve is allowed to claim is gated downstream
            // (`TagPose.trusted`), not here.
            detectorFactory = {
                com.dimensional.mini4pro.vision.AprilTagDetector(
                    tagSizeM = tagConfig.tagSizeM,
                    calibration = tagConfig.calibration,
                )
            },
            config = tagConfig,
            flight = {
                val st = aircraftState()
                // Both read through `Px4Mode` rather than off a second list of DJI mode strings,
                // for the reason `TelemetryEncoder.landedState` gives: this and the heartbeat must
                // not be able to disagree about what a landing is. It also means the widened
                // `FCFlightMode` sets — `ATTI_LANDING`, `BASE_LANDING`, `BACKUP_GO_HOME` and the
                // rest — arm the detector without this file having to enumerate them.
                val mode = com.dimensional.mini4pro.telemetry.Px4Mode.customMode(st.flightMode)
                com.dimensional.mini4pro.vision.FlightView(
                    flying = st.isFlying == true,
                    relativeAltitudeM = st.relativeAltitude,
                    returning = mode == com.dimensional.mini4pro.telemetry.Px4Mode.AUTO_RTL,
                    landing = mode == com.dimensional.mini4pro.telemetry.Px4Mode.AUTO_LAND,
                    // **Our own tag landing, which DJI's mode cannot tell you about**: from the
                    // arm (or the precision-`NAV_LAND` sequence's nadir aim) to the Stage C
                    // commit the aircraft is in `JOYSTICK`, so both flags above are false for the
                    // whole of the descent that matters. Read fresh from the engine on every
                    // evaluation and never cached — `guidedStick` is created by [start] and the
                    // recogniser outlives no engagement; a held reference would be a landing flag
                    // from a bridge that had stopped. The engine derives it (never latches it), so
                    // this goes false with the landing run's every ending — see
                    // `GuidedStickEngine.landingOnTag`.
                    landingOnTag = guidedStick?.landingOnTag() == true,
                )
            },
            pose = {
                val st = aircraftState()
                val home = com.dimensional.mini4pro.telemetry.Geo
                    .coordinateOrNull(st.homeLatitude, st.homeLongitude)
                val here = com.dimensional.mini4pro.telemetry.Geo
                    .coordinateOrNull(st.latitude, st.longitude)
                val ned = if (home != null && here != null) {
                    com.dimensional.mini4pro.guided.RepositionGuidance
                        .nedMetres(home.first, home.second, here.first, here.second)
                } else {
                    null
                }
                // Read once so the angle and its provenance flag cannot come from two different
                // resolutions — a command landing between two reads would stamp a commanded
                // number as reported, or the reverse.
                val pitch = believedCameraPitch()
                com.dimensional.mini4pro.vision.CameraPose(
                    northM = ned?.first,
                    eastM = ned?.second,
                    relativeAltitudeM = st.relativeAltitude,
                    // `yawDeg` is documented as signed [−180, 180] with 0 = north, which is exactly
                    // the convention `TagWorld` wants. Roll and pitch carry "sign convention
                    // UNVERIFIED" in the same file and are not used here; yaw does not.
                    headingDeg = st.yawDeg,
                    // **Commanded first, reported as the fallback, provenance carried** —
                    // `gimbal/PitchBelief`, the same resolution the engine's nadir gate reads
                    // (`believedCameraPitch`), so the fix ladder and the arm gate judge the
                    // identical number. The commanded angle is exact and never stale; the
                    // reported one is `KeyGimbalAttitude`'s last delivery, believed by value and
                    // never by age (change-driven: silence means unchanged — the seven-times
                    // trap). The fallback is what makes an RC-wheel nadir hold produce fixes at
                    // all: the 2026-07-28 denial records are a camera genuinely at −90° whose
                    // every sighting died here because only the command was consulted. Null still
                    // means neither source has said anything, and `TagWorld.fix` refuses it
                    // rather than assuming.
                    cameraPitchDeg = pitch?.pitchDeg,
                    cameraPitchReported = pitch?.reported == true,
                )
            },
            // `RecordedTagSink` is not optional and could not be omitted: the recogniser is
            // constructed with it, so one whose sightings reach no record cannot be built. The
            // `downstream` lambda is the `vision_msgs.Detection3D` publisher, live since
            // 2026-07-28 and off unless the operator asked for it.
            //
            // **`ZenohBus` is asked on every sighting rather than captured once**, exactly as
            // `startVideo`'s `rawSink` reads `ZenohBus.videoSink()` per frame and for the same
            // reason: the bus starts and stops independently of the detector, and a lambda
            // holding a publisher from a finished session would offer into a queue nobody drains.
            // The switch, the encode and the containment all live on the far side, in
            // `ZenohBus.publishDetection`, so what appears here is the wiring and nothing else.
            //
            // The fix rides its own channel since 2026-07-29: `Detection3D` on `detections` is
            // stamped `drone/camera_optical` and the `TagFix` goes out as `tag_fix` in
            // `drone/world` — one truth per channel, composed at source by `TagWorld.fix`
            // rather than left to a consumer joining mismatched instants. Only the latch flag
            // is still dropped: it is a flight-control latch, not a bus fact.
            onSighting = com.dimensional.mini4pro.vision.RecordedTagSink(
                tap = Recorder,
                downstream = { sighting, fix, _ ->
                    ZenohBus.publishDetection(sighting)
                    ZenohBus.publishTagFix(sighting, fix)
                },
            ),
            nowNanos = { android.os.SystemClock.elapsedRealtimeNanos() },
            log = { msg -> Log.i(TAG, "tag: $msg") },
        )
        tagRecogniser = recogniser
        recogniser.start()?.let { Log.w(TAG, "tag: not started — $it") }
    }

    /**
     * Video's phase trail and failures, into the flight log.
     *
     * `SEV_ERROR` for failures rather than `SEV_WARN`: unlike a WiFi blip, every
     * one of these means the operator's ground station is showing a black
     * rectangle, and the whole reason this milestone existed is that such a
     * failure used to leave no trace anywhere.
     */
    private fun recordVideoEvent(code: String, message: String, error: Boolean) {
        try {
            Recorder.event(
                code, message,
                if (error) LogEntry.SEV_ERROR else LogEntry.SEV_INFO,
            )
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder video event failed", e)
        }
    }

    fun stop() {
        // First, and unconditionally. A stopped bridge with commands still armed is a
        // trap: the operator sees "stopped", believes nothing can reach the aircraft,
        // and a later start() would come up already able to command. The interlock is
        // meant to require a deliberate act per session, and letting it survive the
        // link would quietly weaken that to once per app launch.
        //
        // Only reachable through BridgeService.onDestroy, so no JVM test covers this
        // line; CommandInterlock.disable() itself is tested, and is idempotent.
        commandInterlock.disable()
        // Immediately after the interlock and before everything else DJI-side: this is the one
        // component that can be actively flying the aircraft at this instant. stop() aborts any
        // engagement — zero setpoint, disableVirtualStick, STATUSTEXT "stopped" — then cancels
        // its subscriptions and its tick thread.
        guidedStick?.stop()
        guidedStick = null
        // Nothing about a mission is persisted, and this is where that becomes true: a plan
        // resuming into a session whose operator never armed it is the failure the whole executor
        // design is built to prevent. The store outlives the link (QGC re-reads it on connect); the
        // *cursor* does not.
        missionExecutor.onBridgeStopped()
        // Then withdraw a simulator this process started, for the same reason and with the same
        // asymmetry: a stopped bridge with the aircraft still simulating is a trap, because every
        // warning the operator has been reading — the banner, the status line, the STATUSTEXT —
        // is produced by this app, and all three go quiet while the aircraft carries on.
        //
        // Only ours. A simulator this process did not start is left alone and stays visible in
        // the UI as FOREIGN: stopping it would be an unrequested act on an aircraft, which §Q4 of
        // `docs/decisions/2026-07-25-m2-command-safety.md` forbids. The operator can end one
        // deliberately from the screen.
        //
        // The DJI call is fire-and-forget by necessity — a `performAction` callback may never
        // arrive (measured, `docs/measurements/2026-07-26-m2-first-command.md`) — so this is a
        // request, not a guarantee, and `SimulatorControl` keeps reporting the simulator as
        // running until DJI says otherwise. The subscription is deliberately NOT cancelled here;
        // see `Bridge.simulator`.
        simulator.stopIfOurs()
        // Then the camera, before the socket and the locks go. It came up with the
        // link (see startVideo) and it goes down with it: a passthrough still
        // pushing 5 Mbit/s at a relay after the operator pressed "Stop bridge" is
        // the same class of trap as an interlock that survives the link — the
        // screen says stopped and the radio says otherwise. It also removes the
        // DJI receive-stream listener, which must not outlive the session that
        // justified it.
        VideoStreamer.stop()
        // Then the second transport, on the same terms and for the same reason: it came up with
        // the link and it goes down with it, and a bus still publishing an aircraft after the
        // operator pressed "Stop bridge" is a subscriber being told about a session nobody is
        // watching.
        //
        // **After `guidedStick?.stop()` above, deliberately.** That call aborts any engagement and
        // announces it, and the announcement is a `status` message — so the disengagement reaches
        // the bus before the sink comes off it. The other order would end every Zenoh-visible
        // flight with the aircraft's last known state being "engaged".
        //
        // Anything still queued is abandoned rather than flushed; `ZenohPublisher.stop` has the
        // argument, and the short version is that everything on this bus is also in the flight
        // log, so waiting out a `NEVER_DROP` publisher would put the wedge on this thread.
        ZenohBus.stop()
        // Detach before the listener teardown, so no confirmation callback can race a
        // dispatcher whose aircraft is being unplugged; then cancel the KeyManager
        // subscriptions the same way StateCache.stop does, so nothing DJI-side outlives the
        // link that justified it.
        commands.actions = null
        flightActions?.stop()
        flightActions = null
        // Same order and the same reason for the gimbal: detach first so no DJI callback can race
        // a manager whose aircraft is being unplugged, then cancel the KeyManager subscriptions,
        // then drop the coalesced setpoint and the record of who held gimbal control — a stale
        // primary controller surviving into the next link would lock the next operator out of
        // their own camera behind QGC's "Request Gimbal Control?" popup.
        // **Before** the gimbal goes, because the recogniser's pose supplier reads the commanded
        // angle through it and a detector still running against a torn-down camera would be
        // producing fixes from a pointing nobody is maintaining any more. It also removes the DJI
        // frame listener, which must not outlive the session that justified it — the same rule
        // `VideoStreamer.stop` follows for the receive-stream listener a few lines down.
        tagRecogniser?.stop()
        tagRecogniser = null
        gimbal.aim = null
        gimbalAim?.stop()
        gimbalAim = null
        commandedGimbal = null
        light?.stop()
        light = null
        gimbal.reset()
        // With the aim it reads. A supplier left behind would go on returning the last reading of
        // a gimbal that is no longer being listened to, and the recorder would keep stating a
        // camera angle for an aircraft this bridge has let go of.
        Recorder.gimbalSource = null
        // The same argument, on the bus: a supplier left behind would go on placing a camera edge
        // in the frame tree for an aircraft this bridge has let go of.
        ZenohBus.gimbalSource = null
        timer?.shutdownNow()
        timer = null
        link?.stop()
        link = null
        teardownWifiGate()
        // Last, so nothing is still transmitting when the radio is allowed to sleep.
        locks?.release()
        locks = null
        _state.value = _state.value.copy(running = false, wifi = null)
        Log.i(TAG, "bridge stopped")
    }

    /**
     * A WiFi network appeared (ConnectivityManager thread). If the bound network
     * was lost and the link is still up, swap the link's socket onto the new one.
     *
     * This is LINK maintenance, not an autonomous aircraft action: the Q4 rule
     * (`docs/decisions/2026-07-25-m2-command-safety.md`) forbids unrequested
     * *aircraft commands*, not keeping our own transport alive. The distinction
     * matters in the field — the phone is mounted on the RC with adb unreachable,
     * so a bridge that stays dark after a 2 s AP blip until someone gets
     * hands-on with the aircraft is an operational failure, not caution. Nothing
     * here touches DJI; only the UDP socket is rebuilt, and every rebind is
     * announced in the status line, logcat and the flight log.
     *
     * If the rebind fails, fall back to honest report-only: the loss stays
     * pending in the tracker (a later network event may still succeed), the
     * status names the failure, and the bridge keeps running — never crash it
     * over transport maintenance.
     */
    private fun onWifiAvailable(network: Network) {
        val tracker = wifiTracker ?: return
        val l = link
        val desc = network.toString()
        if (!tracker.shouldRebind(desc, linkRunning = l != null && l.isRunning)) return
        try {
            // rebind() re-checks running under its own lock; false means the link
            // stopped since the check above, and there is nothing to announce.
            if (l!!.rebind(WifiBind.socketFactory { network.bindSocket(it) })) {
                reportWifi(EventCode.WIFI_REBOUND, tracker.rebound(desc), LogEntry.SEV_INFO)
                // The video socket is bound to the same dead network and is just
                // as unable to transmit — netIds are never reused, so nothing
                // recovers by waiting. Recovering telemetry and leaving video dark
                // would be the worst of both: the operator sees a healthy link and
                // a black picture, with `rtpErrors` climbing on a screen they have
                // no reason to be looking at. No-op when video is off.
                VideoStreamer.socketBinder = { s -> network.bindSocket(s) }
                VideoStreamer.rebindTransport("WiFi rebound to $desc")
            }
        } catch (e: Exception) {
            reportWifi(EventCode.WIFI_REBIND_FAILED, tracker.rebindFailed(desc, e.message))
        }
    }

    /**
     * Surfaces a WiFi event to the operator (status line), logcat, and the flight
     * recorder.
     */
    private fun reportWifi(code: String, message: String, severity: String = LogEntry.SEV_WARN) {
        if (severity == LogEntry.SEV_WARN) Log.w(TAG, "wifi: $message") else Log.i(TAG, "wifi: $message")
        recordWifiEvent(code, message, severity)
        _state.value = _state.value.copy(wifi = message)
    }

    /**
     * Every simulator transition, into the flight log. `SEV_WARN` throughout, including the
     * stops: a session in which the simulator was on *at any point* is one whose telemetry cannot
     * be read as a measurement, and the log should make that impossible to miss on a skim.
     */
    private fun recordSimulatorEvent(message: String) {
        try {
            Recorder.event(SimulatorNotice.EVENT_PHASE, message, LogEntry.SEV_WARN)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder simulator event failed", e)
        }
    }

    private fun recordWifiEvent(code: String, message: String, severity: String = LogEntry.SEV_WARN) {
        // Contained like every recorder tap: an evidence problem must never
        // become a telemetry problem.
        try {
            Recorder.event(code, message, severity)
        } catch (e: Throwable) {
            Log.w(TAG, "flight recorder wifi event failed", e)
        }
    }

    private fun teardownWifiGate() {
        wifiGate?.stop()
        wifiGate = null
        wifiTracker = null
    }

    private fun publishCounters() {
        val l = link ?: return
        _state.value = _state.value.copy(
            sent = l.sent.get(),
            received = l.received.get(),
            error = l.lastError,
        )
    }

    /**
     * One telemetry tick. Snapshots the cache once and encodes from that single
     * snapshot, so every message in a tick describes the same instant rather than
     * a smear across the encode.
     */
    private fun tick() {
        val l = link ?: return
        // Cheap and idempotent: plants the guided RC feed once MSDK registration lands. The
        // bridge auto-start beats registration by ~1 s on fresh launches, so the engine's own
        // attach() is usually the silent pre-registration no-op (VirtualStickPort.ensureRcFeed).
        guidedStick?.ensureRcFeed()
        // Same cure, same trap, same 200 ms retry: a DJI listener planted before MSDK registration
        // completes silently never delivers, and the bridge comes up ~1 s before registration on a
        // fresh launch. Idempotent and cheap — a boolean read once it has taken. See
        // DeviceHealthWatch.ensure and `docs/device-health.md`.
        deviceHealth.ensure()
        // **The third instance of the same trap**, and it bites harder here than for either of the
        // two above. `MsdkFrameSource.start` refuses until MSDK is registered *and* an aircraft is
        // connected, and `Bridge.start` runs about a second before registration on a fresh launch —
        // so the one-shot attempt in `start()` almost always refuses, and a detector that gave up
        // there would be absent for exactly the flights it is meant to watch the takeoff of.
        // Idempotent and cheap: a started recogniser returns on a null check. The refusal sentence
        // is dropped here rather than logged, because logging it at 5 Hz until an aircraft appears
        // would bury the session; it is on the screen the whole time as `TagRecogniser.why`.
        tagRecogniser?.start()
        // The outbound seam: this is the aircraft, or a recording somebody switched onto the
        // link. See [outboundState] for which reads move and which deliberately do not.
        val state = outboundState()
        val timeBootMs = SystemClock.elapsedRealtime()
        // Wrap well short of overflow; the divisors' LCM makes the phase repeat
        // cleanly, so no message ever skips a beat at the wrap.
        val n = tickCount
        tickCount = (tickCount + 1) % 600

        try {
            // Fast set: what a GCS uses to draw the aircraft moving.
            if (n % EVERY_TICK == 0) {
                // Null when we have no position: 0/0 is a real place, and a GCS
                // plots it as a confident fix. See the encoder's KDoc.
                TelemetryEncoder.globalPositionIntOrNull(state, timeBootMs)?.let { l.send(it) }
                l.send(TelemetryEncoder.gpsRawInt(state, timeBootMs))
                l.send(TelemetryEncoder.attitude(state, timeBootMs))
                l.send(TelemetryEncoder.vfrHud(state))
            }
            // Status: changes slowly, but landed_state should react promptly.
            if (n % TICKS_2HZ == 0) {
                l.send(TelemetryEncoder.sysStatus(state))
                l.send(TelemetryEncoder.extendedSysState(state))
            }
            // MAVLink convention is a 1 Hz heartbeat; faster is simply wrong.
            if (n % TICKS_1HZ == 0) {
                // The executor's claim is re-earned here, on every heartbeat, from three
                // observations — never latched and never set because a `SET_MODE` arrived. Null in
                // every state but a genuinely-flying mission, and then the heartbeat carries DJI's
                // own reading exactly as it always has.
                l.send(TelemetryEncoder.heartbeat(state, missionExecutor.modeClaim()))
                l.send(TelemetryEncoder.batteryStatus(state))
            }
            emitEventMessagesIfChanged(l, state)
            emitSimulatorNoticeIfDue(l, timeBootMs)
            // Asked on every tick rather than on a rate divisor, because GimbalManager owns its
            // own two cadences (5 Hz attitude, 1 Hz status) — owning them there is what makes
            // them testable, and it returns an empty list whenever there is no gimbal to
            // advertise. It also flushes any coalesced aim setpoint, which is why it is called
            // even when nothing is due to be sent.
            for (m in gimbal.tick()) l.send(m)
            // Same shape as the gimbal: MissionProgress owns its own 1 Hz cadence and returns an
            // empty list whenever the store holds no plan (JC-7 — `seq` has no "none" value, and
            // sending 0 for a vehicle with no mission makes QGC highlight plan item 1).
            for (m in missionProgress.messages(timeBootMs)) l.send(m)
            // Our own upload/download timeouts. Cheap when nothing is in flight, and it must run
            // on a timer rather than on inbound traffic: the failure it exists for is QGC having
            // stopped talking to us.
            missionTransaction.tick()
        } catch (e: Exception) {
            // A malformed key value must not kill the telemetry thread — a dead
            // scheduler would look exactly like a disconnected aircraft.
            Log.w(TAG, "telemetry tick failed", e)
            _state.value = _state.value.copy(error = "encode: ${e.message}")
        }
    }

    /**
     * HOME_POSITION and GPS_GLOBAL_ORIGIN are event messages. Sending them at rate
     * is wasteful; never sending them means QGC has no home symbol.
     *
     * [HomeEventGate] owns the "has it changed?" decision — including staying
     * silent while home is unknown, and not counting barometric jitter in the AMSL
     * datum as a change. It returns an empty list when there is nothing new, and
     * only remembers what it hands back, so the send below is the whole protocol.
     */
    private fun emitEventMessagesIfChanged(l: MavlinkLink, state: AircraftState) {
        val events = homeEvents.messagesIfChanged(state)
        if (events.isEmpty()) return
        events.forEach { l.send(it) }
        Log.i(TAG, "published ${events.size} event message(s) for home ${homeEvents.lastPublished()}")
    }

    /**
     * Tells the ground station the aircraft it is watching is simulated.
     *
     * Asked on **every** tick rather than on a rate divisor, because [SimulatorControl.noticeIfDue]
     * owns the cadence itself and owning it there is what makes it testable: it holds the notice
     * to one per `NOTICE_PERIOD_MS` but releases it immediately on a phase change, so a simulator
     * coming up is announced within one 200 ms tick instead of within five seconds.
     *
     * A `STATUSTEXT` and nothing else — no heartbeat flag, no `system_status` override. The
     * argument for that choice, including why `MAV_MODE_FLAG_HIL_ENABLED` was rejected on both
     * truth and effectiveness grounds, is in [SimulatorNotice]'s class doc.
     */
    private fun emitSimulatorNoticeIfDue(l: MavlinkLink, timeBootMs: Long) {
        val text = simulator.noticeIfDue(timeBootMs) ?: return
        l.send(SimulatorNotice.statusText(text))
    }

    /**
     * Mirrors the recorder's own internals — commanded vs achieved velocity, stick
     * axes, virtual-stick authority — to the GCS, so a live session is diagnosable
     * from the ground rather than only after pulling files off the phone.
     *
     * `mirrorMessages` returns an empty list whenever the mirror is disabled or
     * virtual stick is not engaged, so this costs nothing until M3.
     */
    private fun mirrorTick() {
        val l = link ?: return
        try {
            val timeBootMs = SystemClock.elapsedRealtime()
            // The socket tap records these on the way out; no explicit log call.
            for (m in Recorder.mirrorMessages(timeBootMs)) l.send(m)
        } catch (e: Exception) {
            // Diagnostics must never take out telemetry — same rule as tick().
            Log.w(TAG, "mirror tick failed", e)
        }
    }

    /**
     * Whether **DJI reports a simulator running on this aircraft** — the takeoff precondition,
     * read fresh on every command (`MsdkFlightActions.REQUIRE_SIMULATOR`).
     *
     * An exhaustive `when` rather than a set membership test, so that adding a
     * [com.dimensional.mini4pro.simulator.SimulatorPhase] fails the build here instead of
     * defaulting a new state to "not simulated" — or, worse, to "simulated".
     *
     * The mapping is deliberately **identical to `SimulatorControl.noticeIfDue`'s**, which is what
     * decides whether QGroundControl is being told `SIMULATOR ACTIVE - telemetry is not real
     * flight`. That equivalence is the property worth having: takeoff is permitted exactly when
     * the ground station is being warned, so the gate and the warning cannot come apart.
     *
     *  - `ACTIVE` and `FOREIGN` — DJI has delivered `KeyIsSimulatorStarted = true`. Whether *we*
     *    started it is irrelevant to the only question being asked; a simulator started by DJI
     *    Assistant simulates just as thoroughly.
     *  - `STOPPING` — we asked it to stop and DJI still reports it running. Still simulated, and
     *    `docs/simulator.md` §4.3 already treats it that way on the wire.
     *  - `UNKNOWN` — DJI has not told us. **Not the same as off, and it fails closed anyway**:
     *    the question here is "may an aircraft leave the ground", so the absence of evidence
     *    must refuse.
     *  - `OFF` — DJI said `false`, the only state that licenses the word.
     *  - `STARTING` — we asked and DJI has not confirmed. Claims nothing, so it grants nothing;
     *    treating our own request as evidence would be the echo this project forbids everywhere.
     */
    private fun simulatorReportsRunning(): Boolean = when (simulator.phase) {
        SimulatorPhase.ACTIVE, SimulatorPhase.FOREIGN, SimulatorPhase.STOPPING -> true
        SimulatorPhase.UNKNOWN, SimulatorPhase.OFF, SimulatorPhase.STARTING -> false
    }

    /**
     * **The aircraft, always** — the read that cannot be replayed.
     *
     * Reads through to the live cache when the aircraft is up. Before
     * registration/connection there is nothing to read, and an all-null state is
     * the honest answer — the encoder turns that into sentinels.
     *
     * Everything that can *act* reads this: the guided engine, the mission executor and its
     * upload datum, the tag detector's arming and pose, the gimbal edge's yaw. A recording must
     * never reach any of them, and the way that is guaranteed is that this function has no
     * parameter and consults no feed — see [StateSource.liveState] for why the switchable read
     * is a separate function rather than a flag on this one.
     */
    private fun aircraftState(): AircraftState = StateSource.liveState()

    /**
     * **What we describe to somebody else** — the aircraft, or a recording an operator switched
     * onto the MAVLink link.
     *
     * The outbound half of the fan-out, and the only half that moves. Three call sites: [tick]'s
     * fast/status/heartbeat sets, and the two `MAV_CMD_REQUEST_MESSAGE` providers that must
     * agree with them. `ZenohBus` has the matching read for its own sink.
     *
     * Publishing is refused while an aircraft is connected (`replay/ReplayAdmission.mayPublish`),
     * so this and [aircraftState] never disagree about a *live* aircraft — when one is there,
     * both return it. They differ only in the state this feature exists for, and in that state
     * the command path is holding an aircraft that is not connected, which is exactly the
     * all-null snapshot it has always been given when there is nothing on the link.
     */
    private fun outboundState(): AircraftState = StateSource.read(StateSource.Sink.MAVLINK)

    /**
     * Routes one inbound message. **It is no longer this method's job to record it** —
     * `MavlinkLink` taps every message with its genuine wire bytes before handing it here, so the
     * `Recorder.mavIn(message)` that used to open this method is gone and cannot be forgotten by
     * whoever writes the next router.
     */
    private fun onInbound(message: MavlinkMessage<*>) {
        val payload = message.payload
        val detail = when (payload) {
            is CommandLong -> "CommandLong ${payload.command().entry()} p1=${payload.param1()}"
            is CommandInt -> "CommandInt ${payload.command().entry()}"
            else -> payload.javaClass.simpleName
        }
        Log.d(TAG, "rx $detail from sys ${message.originSystemId}")
        handshake.onMessage(message)
        // Then the gimbal, for the one inbound *message* (as opposed to command) it cares about:
        // GIMBAL_MANAGER_SET_ATTITUDE (282), QGC's joystick rate path, which HandshakeResponder
        // leaves in its `else -> Unit` branch. Routing it from here rather than adding a hook to
        // `handshake/` is deliberate: that package is being edited by someone else, and "routes
        // inbound" is this class's documented job. Order matters only in that the responder still
        // owns every acknowledged command; 282 is unacknowledged, so nothing is duplicated.
        gimbal.onInbound(payload, message.originSystemId, message.originComponentId)
        // Then the mission protocol — the eight mission message types, in both directions, owned
        // by exactly one state machine. `HandshakeResponder`'s five mission branches were deleted
        // rather than extended (JC-10): they exist above this line in the routing order, so
        // leaving them would have meant the dead answer winning. Routed from here for the same
        // reason the gimbal's 282 is: "routes inbound" is this class's documented job, and the
        // recorder tap at the top of this method has already put the genuine wire bytes in the
        // flight log — our replies go out through `sendOffMain` and are tapped on the socket.
        missionTransaction.onInbound(payload, message.originSystemId, message.originComponentId)
        // Last, the guided engine — and for *every* message, not only MANUAL_CONTROL (69):
        // any inbound traffic is evidence the GCS is alive, which is what lets the engine
        // tell "operator turned the sticks off" (released) from "the link died" (link-lost,
        // the Q4 policy). Routed from here like the gimbal's 282, and for the same reason:
        // MANUAL_CONTROL is an unacknowledged message, handshake/ stays untouched, and
        // "routes inbound" is this class's documented job.
        guidedStick?.onInbound(payload, message.sequence, ControlOrigin.MAVLINK)
    }
}
