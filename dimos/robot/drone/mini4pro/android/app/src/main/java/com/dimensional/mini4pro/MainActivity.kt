package com.dimensional.mini4pro

import android.content.Context
import android.os.Bundle
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.dimensional.mini4pro.guided.GuidedPhase
import com.dimensional.mini4pro.mavlink.MavlinkLink
import com.dimensional.mini4pro.record.EventCode
import com.dimensional.mini4pro.record.LogEntry
import com.dimensional.mini4pro.record.Recorder
import com.dimensional.mini4pro.replay.FlightLogLibrary
import com.dimensional.mini4pro.replay.ReplayAdmission
import com.dimensional.mini4pro.replay.ReplayPlayer
import com.dimensional.mini4pro.replay.ReplayPublication
import com.dimensional.mini4pro.telemetry.StateSource
import com.dimensional.mini4pro.simulator.SimulatorInterlock
import com.dimensional.mini4pro.simulator.SimulatorNotice
import com.dimensional.mini4pro.situation.FlownTrack
import com.dimensional.mini4pro.situation.Situation
import com.dimensional.mini4pro.situation.SituationReading
import com.dimensional.mini4pro.situation.SituationSource
import com.dimensional.mini4pro.telemetry.AircraftState
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.dimensional.mini4pro.simulator.SimulatorOutcome
import com.dimensional.mini4pro.simulator.SimulatorPhase
import com.dimensional.mini4pro.simulator.SimulatorRequest
import com.dimensional.mini4pro.video.VideoPhase
import com.dimensional.mini4pro.video.VideoRequest
import com.dimensional.mini4pro.video.VideoStatus
import com.dimensional.mini4pro.video.VideoStreamer
import com.dimensional.mini4pro.vision.TagArm
import com.dimensional.mini4pro.zenoh.ZenohBus
import com.dimensional.mini4pro.zenoh.ZenohChannel
import com.dimensional.mini4pro.zenoh.ZenohPublisher
import com.dimensional.mini4pro.zenoh.ZenohSettings
import com.dimensional.mini4pro.zenoh.ZenohTelemetryPump
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * Status screen and bridge control. QGroundControl is the cockpit — this is
 * deliberately just enough UI to point the link at a GCS and see it working.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var status: TextView
    private lateinit var settings: Button

    /**
     * The GCS address, held here rather than in a field on screen.
     *
     * It is set once a session and read constantly, so an `EditText` cost a
     * permanent row of the scarcest thing this layout has. It now lives in the
     * settings dialog and is displayed in the status strip — see the layout's
     * header comment. Prefs remain the source of truth; this is a cache so the
     * strip can render without touching disk on every frame.
     */
    private var gcsHost: String = ""
    private lateinit var toggle: Button
    private lateinit var commands: SwitchCompat
    private lateinit var banner: TextView
    private lateinit var commandRow: android.widget.LinearLayout
    private lateinit var simulator: Button

    /** The top-down picture, painted behind [status]. A pure consumer — see [SituationView]. */
    private lateinit var situationView: SituationView
    private lateinit var pauseManoeuvre: Button
    private lateinit var stopManoeuvre: Button
    private lateinit var replayButton: Button
    private lateinit var replayBar: android.widget.LinearLayout
    private lateinit var replayLabel: TextView
    private lateinit var replayPlay: Button
    private lateinit var replayClose: Button

    /**
     * The two publishing switches, built in code and added to the XML strip.
     *
     * Programmatic because they belong to the strip's *behaviour* rather than to its shape: the
     * strip is `GONE` unless a recording is open, and these two must be off every time it
     * appears. A layout attribute cannot express "off again whenever this becomes visible", and
     * the one thing neither of these may ever do is come back checked.
     */
    private lateinit var replayToMavlink: SwitchCompat
    private lateinit var replayToZenoh: SwitchCompat
    private lateinit var videoRecord: SwitchCompat
    private lateinit var tagDetect: SwitchCompat
    private lateinit var takeOff: Button
    private lateinit var descendTag: Button
    private lateinit var fullAutoland: androidx.appcompat.widget.SwitchCompat
    private lateinit var shadowCompare: ShadowCompareView
    private lateinit var vitals: TextView
    private lateinit var tagVitals: TextView

    /**
     * The on-board tag detector, or null when there is none.
     *
     * **Read fresh from [Bridge] rather than held**, exactly as `videoPlan` is recomputed rather
     * than cached: the recogniser comes up with the link and goes down with it, and a screen holding
     * a stopped session's detector would go on reporting a tag position for an aircraft this app has
     * let go of. Null says "nothing is looking", which the readout distinguishes from "something is
     * looking and cannot see it" — see [renderVitals].
     */
    private val tagSighting: com.dimensional.mini4pro.vision.TagSighting?
        get() = Bridge.tagRecogniser

    /**
     * The recording being replayed, or null when the screen is showing the aircraft.
     *
     * **Read only by [renderSituation] and the replay controls.** Nothing in the command path
     * can see this field, and nothing it produces has a method that could reach one: a replayed
     * sample becomes a `Situation` — flat value data with nothing to call — before it leaves
     * this function. `SituationHonestyTest` asserts that by reflection, and `ReplayAdmission`
     * is the behavioural half. See `replay/ReplayAdmission` for the whole argument.
     */
    private var replay: ReplayPlayer? = null

    /**
     * Whether the recording on screen is being fed to each downstream. **Both false always**
     * until an operator taps, and reset to false by [closeReplay] and by opening a new recording.
     *
     * Held here rather than read back off the switches so that the source of truth is a value
     * and not a view: `isChecked` is also written by us when we re-sync the strip, and a state
     * that can be set by a redraw is a state that can be set by a redraw *wrongly*. The switches
     * follow these; these follow nothing but a tap and an admission decision.
     */
    private var replayToMavlinkOn = false

    private var replayToZenohOn = false

    /**
     * Where the aircraft has been, accumulated from the same vetted fixes the symbol is drawn
     * from. Pure, bounded and testable — every rule about it lives in `situation/FlownTrack`.
     *
     * One track, not two: it clears itself on a change of `Situation.source`, so switching to a
     * recording and back can never draw one flight's path under another's. A *second* recording
     * is the one case that needs saying out loud, in [loadReplay].
     */
    private val flownTrack = FlownTrack()

    private val prefs by lazy { getSharedPreferences("mini4pro", Context.MODE_PRIVATE) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        status = findViewById(R.id.status)
        settings = findViewById(R.id.settings)
        hideSystemBars()
        settings.setOnClickListener { showSettings() }
        toggle = findViewById(R.id.toggle)
        commands = findViewById(R.id.commands)
        banner = findViewById(R.id.banner)
        commandRow = findViewById(R.id.commandRow)
        simulator = findViewById(R.id.simulator)
        situationView = findViewById(R.id.situation)
        pauseManoeuvre = findViewById(R.id.pauseManoeuvre)
        stopManoeuvre = findViewById(R.id.stopManoeuvre)
        replayButton = findViewById(R.id.replay)
        videoRecord = findViewById(R.id.videoRecord)
        tagDetect = findViewById(R.id.tagDetect)
        takeOff = findViewById(R.id.takeOff)
        descendTag = findViewById(R.id.descendTag)
        fullAutoland = findViewById(R.id.fullAutoland)
        shadowCompare = findViewById(R.id.shadowCompare)
        vitals = findViewById(R.id.vitals)
        tagVitals = findViewById(R.id.tagVitals)
        replayBar = findViewById(R.id.replayBar)
        replayLabel = findViewById(R.id.replayLabel)
        replayPlay = findViewById(R.id.replayPlay)
        replayClose = findViewById(R.id.replayClose)
        buildReplayPublishSwitches()
        wireWithdrawals()
        wireReplay()
        wireSimulator()
        // The banner is the switch's label now, so it has to carry the tap — text
        // beside a switch that does nothing when tapped reads as a broken switch.
        banner.setOnClickListener { commands.performClick() }
        wireCommandInterlock()
        wireVideoRecording()
        wireTagDetector()
        wireTakeoff()
        wireTagDescent()

        // Testable from adb:
        //   am start -n com.dimensional.mini4pro/.MainActivity \
        //     --es host 10.55.1.12 --ez autostart true
        val intentHost = intent?.getStringExtra(EXTRA_HOST)?.trim()?.takeIf { it.isNotEmpty() }
        // The default is the relay, not empty — which is what makes a bare launch on a phone
        // nobody has configured come up talking. `LaunchPolicy` reads emptiness as "nowhere to
        // send telemetry"; see `CockpitDefaults.GCS_HOST` for why that stopped being the right
        // starting state, and why `--ez autostart false` still outranks it.
        val savedHost = prefs.getString(KEY_HOST, CockpitDefaults.GCS_HOST)?.trim().orEmpty()
        gcsHost = intentHost ?: savedHost

        // Video extras are folded into the saved settings *before* anything can
        // start the bridge, so there is exactly one source of truth from here on
        // and a launch flag behaves the same as a tap in the dialog. `--ez video
        // false` really turns it off; the extra being absent leaves the setting
        // alone.
        //
        // This ordering is also the fix for the bug that made this milestone
        // necessary: `tools/session video` has been passing `--ez video true` for
        // weeks into an app that read no such extra, and nothing noticed, because
        // "video never appeared" and "video was never asked for" look identical
        // from outside.
        absorbVideoExtras()
        absorbZenohExtras()

        // Start the bridge by ourselves when we already know where the GCS is.
        //
        // This matters more than it looks: the phone's only USB port belongs to
        // the RC, and adb over WiFi has proved unreliable on this device. Anything
        // that requires an adb command to begin telemetry is therefore unusable in
        // the field. Plugging in the RC launches us via UsbAttachActivity, and we
        // must come up talking without help.
        //
        // But a caller that wants *only* the Activity must be able to say so, and
        // until 2026-07-27 it could not: a bare `am start` opened a real flight
        // session. `--ez autostart false` now means it. See [LaunchPolicy] for the
        // incident and the full table.
        val autostart =
            if (intent?.hasExtra(EXTRA_AUTOSTART) == true) {
                intent.getBooleanExtra(EXTRA_AUTOSTART, false)
            } else {
                null
            }

        if (intentHost != null) {
            prefs.edit().putString(KEY_HOST, intentHost).apply()
        }

        if (LaunchPolicy.shouldStartBridge(intentHost, savedHost, autostart)) {
            // Port is overridable so a validator can listen on its own port while
            // QGroundControl keeps 14550 — otherwise the two compete for the same
            // socket during an aircraft session.
            val port = if (intentHost != null) {
                intent.getIntExtra(EXTRA_PORT, MavlinkLink.DEFAULT_GCS_PORT)
                    // Remember the port too, so a later adb-free launch reuses it.
                    .also { prefs.edit().putInt(KEY_PORT, it).apply() }
            } else {
                prefs.getInt(KEY_PORT, MavlinkLink.DEFAULT_GCS_PORT)
            }
            startBridge(intentHost ?: savedHost, port)
        }
        probeRequested = intent?.getBooleanExtra(EXTRA_PROBE, false) == true
        sweepRequested = intent?.getBooleanExtra(EXTRA_SWEEP, false) == true

        toggle.setOnClickListener {
            if (Bridge.state.value.running) {
                stopBridge()
            } else {
                val target = gcsHost.trim()
                if (target.isEmpty()) {
                    // Send them to the one place that can fix it, rather than
                    // printing a complaint about a field no longer on screen.
                    showSettings()
                    return@setOnClickListener
                }
                prefs.edit().putString(KEY_HOST, target).apply()
                startBridge(target, prefs.getInt(KEY_PORT, MavlinkLink.DEFAULT_GCS_PORT))
            }
        }

        // Deliberately NOT inside repeatOnLifecycle: these are the things that must
        // happen when the MSDK finishes registering, and the app spends real
        // sessions in the background with DJI Fly in front. A UI-gated collector
        // meant a backgrounded app could miss `registered` and then never start
        // StateCache at all — telemetry would be all-sentinel forever, with nothing
        // in the log to say why. lifecycleScope still cancels this on destroy.
        lifecycleScope.launch {
            Msdk.state.collect { onMsdkState(it) }
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                // Three sources, not two: neither Msdk.state nor Bridge.state moves when a
                // simulator starts, and Bridge.state stops ticking entirely when the link is
                // down — see Bridge.simulatorRevision.
                // Four sources. VideoStreamer.state is its own because it is the
                // only thing that republishes the two counters an operator reads
                // this screen for — tapFrames and rtpPkts — and it does so at
                // 1 Hz whether or not anything else has moved. Without it the
                // numbers would only refresh when telemetry happened to tick.
                // Five sources since 2026-07-30: `Bridge.warningRevision` is the DJI warning
                // path's, for `simulatorRevision`'s reason exactly — nothing else on this screen
                // moves because DJI said the wind is strong, and a warning that repaints a minute
                // late is a warning the pilot acted after.
                combine(
                    Msdk.state, Bridge.state, Bridge.simulatorRevision, VideoStreamer.state,
                    Bridge.warningRevision,
                ) { msdk, bridge, _, video, _ ->
                    Triple(msdk, bridge, video)
                }.collect { (msdk, bridge, video) -> render(msdk, bridge, video) }
            }
        }

        // The picture's own cadence, and deliberately not the collector above.
        //
        // Two reasons. A replay advances on wall-clock time and nothing in `Bridge.state` moves
        // while it runs, so a state-driven repaint would show a frozen recording. And
        // `Bridge.state` stops emitting entirely when the link is down, which is exactly when an
        // operator is most likely to be looking at a recording.
        //
        // 200 ms — `Bridge`'s own tick, and a quarter of the fastest thing on screen. Drawing is
        // a consumer: this loop reads snapshots (`StateCache.aircraftState`,
        // `Bridge.guidedSituation`, `MissionStore.plan`) and blocks nothing. It cannot reach the
        // 10 Hz setpoint loop or a DJI callback, because it never calls into either.
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                while (true) {
                    renderSituation()
                    delay(SITUATION_TICK_MS)
                }
            }
        }
    }

    /**
     * Paints the situation view from whichever source is on screen, and nothing else.
     *
     * **The replay branch hands on a `Situation` and never an `AircraftState`.** That is the
     * whole structural safety argument in one line: past this point the drawing path is holding
     * coordinates with no methods on them, so there is no object for a replayed flight to be
     * routed into. See `replay/ReplayAdmission` and `SituationHonestyTest`.
     *
     * A replay is also given **no guided situation and no plan**, and that is honesty rather
     * than laziness: the engine's manoeuvre and the store's plan are facts about *now*, and
     * drawing today's loaded plan around a flight from last week would be the picture mixing two
     * afternoons together.
     */
    private fun renderSituation() {
        followSimulatorInterlock()
        val player = replay
        val situation: Situation
        // The time the *sample* belongs to, which the track thins and breaks by. For a replay
        // that is the recording's own clock and not the wall clock, so a flight played at four
        // times speed still draws the track the aircraft actually flew.
        val atMs: Long
        if (player != null) {
            player.onClock(android.os.SystemClock.elapsedRealtime())
            publishReplaySightings(player)
            val sample = player.current()
            situation = sample
                ?.let { SituationReading.read(SituationSource.REPLAY, it.state) }
                ?: Situation(SituationSource.REPLAY)
            atMs = ((sample?.tSeconds ?: 0.0) * 1000.0).toLong()
        } else {
            situation = SituationReading.read(
                source = SituationSource.LIVE,
                state = if (Msdk.state.value.registered) StateCache.aircraftState() else AircraftState(),
                guided = Bridge.guidedSituation(),
                plan = SituationReading.planMarkOf(Bridge.missionStore.plan()),
            )
            atMs = android.os.SystemClock.elapsedRealtime()
        }
        situationView.situation = situation.copy(
            track = flownTrack.accept(situation, atMs),
            // Across the whole picture, and the one indicator an operator sees without looking
            // for it. A drawn replay and a broadcast one are different states and the watermark
            // is the only place that says so at a glance.
            publishing = replayToMavlinkOn || replayToZenohOn,
        )
        renderReplayBar(player)
        renderWithdrawals(player)
        renderTakeoff(player)
        renderDescend(player)
        renderShadowCompare(player)
    }

    /**
     * The comparison instrument's feed, on the drawing tick. Visible **whenever shadow mode is
     * on** — not only during an armed segment, and that widening flew before it was designed:
     * on 2026-07-28 (flight 152922) Ivan enabled shadow above the detection band with the tag
     * out of view, no segment could arm, and the screen showed nothing at all — shadow-broken
     * and shadow-blocked were indistinguishable. Now the panel appears with the mode and shows
     * the current blocker text where the arrows will appear ("shadow: above 12m decode reach",
     * "shadow: TAG_NOT_IN_VIEW"), and the arrows take over the moment a segment arms. In live
     * mode the panel stays GONE — the pair it compares cannot coexist with a live descent, and
     * its absence is the truthful signal of which mode is running. The staleness blanking is
     * the engine's (`SHADOW_CMD_STALE_MS`).
     */
    private fun renderShadowCompare(player: ReplayPlayer?) {
        val cmp = if (player == null) Bridge.shadowComparison() else null
        shadowCompare.visibility = if (cmp != null) android.view.View.VISIBLE else android.view.View.GONE
        shadowCompare.comparison = cmp
    }

    /**
     * The one control in this app that can make an aircraft move, so it is the one
     * control that asks first.
     *
     * `docs/decisions/2026-07-25-m2-command-safety.md` §Q2. `CommandInterlock` is
     * off at every process start and stores nothing, so this switch is always off
     * when the app opens regardless of last session — that is the point, and it is
     * why the initial state is read from the interlock rather than assumed.
     *
     * `setOnClickListener`, not `setOnCheckedChangeListener`: a click is the
     * operator, whereas `isChecked = …` is us re-syncing the view. Using the click
     * means a programmatic update can never be mistaken for consent, which removes
     * the re-entrancy bug this kind of confirm-then-apply flow usually has.
     *
     * Turning it **on** is confirmed; turning it **off** is immediate. That
     * asymmetry is deliberate: disabling can only make things safer, and an
     * operator reaching for this switch mid-flight is not someone to put a dialog
     * in front of.
     */
    /**
     * Take the screen back from Android's system bars.
     *
     * In landscape the navigation bar sits on the **right edge**, over the only
     * part of this layout that holds controls — the interlock spent a day
     * half-hidden behind the home button before this. `BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE`
     * keeps them one swipe away rather than gone, which matters on a phone with
     * no other way out of the app.
     *
     * The buttons keep their end margin regardless: immersive mode is a request,
     * a swipe revokes it, and a control that is only reachable while a request
     * is being honoured is not reachable.
     */
    private fun hideSystemBars() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        // Android restores the bars whenever focus comes back — after a dialog,
        // after the screen unlocks, after DJI Fly has been in front. Without this
        // the app is immersive exactly once.
        if (hasFocus) hideSystemBars()
    }

    /**
     * Where the GCS address lives now.
     *
     * A dialog rather than a screen: there are two fields, they are set once a
     * session, and a separate activity would be one more thing to back out of on
     * a phone whose back gesture is behind an immersive-mode swipe.
     *
     * Changing the address does **not** restart a running bridge. Retargeting a
     * live telemetry link out from under an operator is the kind of helpfulness
     * that loses a session — the new address is saved and takes effect on the
     * next start, and the dialog says so.
     */
    private fun showSettings() {
        val port = prefs.getInt(KEY_PORT, MavlinkLink.DEFAULT_GCS_PORT)
        val hostField = EditText(this).apply {
            hint = "GCS address (e.g. 10.55.1.50)"
            setText(gcsHost)
            inputType = android.text.InputType.TYPE_CLASS_TEXT
            setSingleLine()
        }
        val portField = EditText(this).apply {
            hint = "port"
            setText(port.toString())
            inputType = android.text.InputType.TYPE_CLASS_NUMBER
            setSingleLine()
        }
        // Off by default and persisted. The argument for that default is at
        // `Bridge.startVideo`; the short version is that a camera stream is
        // bandwidth and battery nobody asked for, and that the failure mode of
        // "off" is one tap from a screen that names the setting, while the failure
        // mode of "on" is 5 Mbit/s of video leaving over LTE.
        val videoSwitch = SwitchCompat(this).apply {
            text = "Send camera video to the GCS"
            isChecked = prefs.getBoolean(VideoRequest.PREF_ENABLED, CockpitDefaults.GCS_VIDEO)
            setPadding(0, 24, 0, 0)
        }
        val videoHostField = EditText(this).apply {
            // Blank is the recommended value: the relay carries telemetry and
            // video in one process, so the two want the same address, and one
            // address is one thing to get wrong instead of two.
            hint = "video host (blank = same as the GCS address)"
            setText(prefs.getString(VideoRequest.PREF_HOST, "").orEmpty())
            inputType = android.text.InputType.TYPE_CLASS_TEXT
            setSingleLine()
        }
        // **The control-tuning rate.** 5 Hz matches `Bridge`'s emitter, so every `dji_state` has a
        // `mav_out` beside it and a session reads cleanly against the wire; 25 Hz is for the
        // question "what happened *between* two telemetry frames", which is the only rate at which
        // a ~200 ms command-to-motion delay can be measured at all. Off by default because it is
        // five times the entries for a question most sessions are not asking — but the cost is
        // small and now measured: 29.6 kB/s against the video's 739.
        val fastRecordSwitch = SwitchCompat(this).apply {
            text = "Record state at 25 Hz (control tuning)"
            isChecked = prefs.getBoolean(KEY_FAST_RECORD, CockpitDefaults.FAST_RECORD)
            setPadding(0, 24, 0, 0)
        }
        // **The second transport.** Off by default and persisted, on the same argument video's
        // default rests on and one more besides: Zenoh adds a JNI runtime and a second network
        // stack to a process that flies an aircraft, and `docs/zenoh-dimos-transport.md` §6.4
        // lists its thermal cost as *unmeasured* on an airframe whose characteristic failure is a
        // battery overheat. A default nobody chose is not the way to find that number out.
        //
        // It cannot be changed while the bridge runs, exactly like the address above, and for a
        // sharper reason: once this transport grows an inbound half, enabling one mid-flight would
        // create a controller the per-origin watchdog has never seen.
        val zenohSwitch = SwitchCompat(this).apply {
            text = "Publish telemetry to the Zenoh bus"
            isChecked = prefs.getBoolean(ZenohSettings.PREF_ENABLED, CockpitDefaults.ZENOH)
            setPadding(0, 24, 0, 0)
        }
        // **The bandwidth switch, and it is separate from the one above on purpose.** Telemetry on
        // this bus is about 0.12 Mbit/s; the video channel measured **5.85 Mbit/s at 43 fps**, and
        // those same bytes already leave the phone as RTP to QGroundControl — so turning this on
        // **doubles the uplink**, on a network `wifi-fix.md` has measured blackholing traffic in
        // one direction. An operator who wanted a subscriber to see where the aircraft is has not
        // thereby agreed to spend that. Ivan asked for the channel — *"it will make it much easier
        // to record"* — and this is the switch that makes the cost a choice rather than a
        // consequence.
        val zenohVideoSwitch = SwitchCompat(this).apply {
            text = "…and camera video (≈5.9 Mbit/s, doubles the uplink)"
            isChecked = prefs.getBoolean(ZenohSettings.PREF_VIDEO, CockpitDefaults.ZENOH_VIDEO)
            setPadding(0, 8, 0, 0)
        }
        // **The coarse-pose switch, and it is not about bandwidth.** 10 Hz of 585-byte messages is
        // 5.9 kB/s, a thousandth of the video. What makes it a choice is what is in them: a pose
        // built on a *fitted* focal length, an *assumed* principal point and no distortion model,
        // published onto a bus other people's software reads. `docs/tag-detector.md` §7 names that
        // as the one way this channel could mislead. Every message carries `metric=false` in its
        // `id` so the caveat cannot be separated from the number, and this switch is so that
        // putting it there is still somebody's decision.
        val zenohDetectionsSwitch = SwitchCompat(this).apply {
            text = "…and AprilTag detections (coarse pose, metric=false)"
            isChecked = prefs.getBoolean(ZenohSettings.PREF_DETECTIONS, CockpitDefaults.ZENOH_DETECTIONS)
            setPadding(0, 8, 0, 0)
        }
        val zenohHostField = EditText(this).apply {
            // Blank is the recommended value and the default is hyper1. Deliberately **not** the
            // GCS address: the bus is a router that DiMOS, a spy and a `dtop` attach to
            // independently, and it has nothing to do with where QGroundControl is.
            hint = "Zenoh router (blank = ${ZenohSettings.ROUTER_HOST})"
            setText(prefs.getString(ZenohSettings.PREF_HOST, "").orEmpty())
            inputType = android.text.InputType.TYPE_CLASS_TEXT
            setSingleLine()
        }
        val box = android.widget.LinearLayout(this).apply {
            orientation = android.widget.LinearLayout.VERTICAL
            setPadding(48, 24, 48, 24)
            addView(hostField)
            addView(portField)
            addView(videoSwitch)
            addView(videoHostField)
            addView(zenohSwitch)
            addView(zenohVideoSwitch)
            addView(zenohDetectionsSwitch)
            addView(zenohHostField)
            addView(fastRecordSwitch)
        }
        // **The custom view of an AlertDialog does not scroll.** Only the `setMessage` text does;
        // whatever `setView` is handed is measured once and clipped to what is left. Nine rows plus
        // the message fitted the old phone and did not fit the A26 (2026-07-30) — the Zenoh host
        // field and the 25 Hz switch were simply off the bottom, with nothing on screen to say so.
        // That is the silent-decline shape this project treats as a bug: a setting an operator
        // cannot see is a setting they cannot choose, and its value goes on being applied anyway.
        // Wrapping in a ScrollView makes the dialog fit any screen by construction rather than by
        // the count of settings staying below whatever this particular phone affords.
        val scroller = android.widget.ScrollView(this).apply { addView(box) }
        AlertDialog.Builder(this)
            .setTitle("Ground station")
            .setMessage(
                if (Bridge.state.value.running) {
                    "The bridge is running — a change here applies the next time it starts."
                } else {
                    "Where telemetry is sent. Point this at the relay, not the laptop.\n\n" +
                        "Video follows the bridge: it starts when the bridge starts and " +
                        "stops when it stops."
                },
            )
            .setView(scroller)
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Save") { _, _ ->
                gcsHost = hostField.text.toString().trim()
                val newPort = portField.text.toString().trim().toIntOrNull() ?: port
                prefs.edit()
                    .putString(KEY_HOST, gcsHost)
                    .putInt(KEY_PORT, newPort)
                    .putBoolean(VideoRequest.PREF_ENABLED, videoSwitch.isChecked)
                    .putString(VideoRequest.PREF_HOST, videoHostField.text.toString().trim())
                    .putBoolean(ZenohSettings.PREF_ENABLED, zenohSwitch.isChecked)
                    .putBoolean(ZenohSettings.PREF_VIDEO, zenohVideoSwitch.isChecked)
                    .putBoolean(ZenohSettings.PREF_DETECTIONS, zenohDetectionsSwitch.isChecked)
                    .putString(ZenohSettings.PREF_HOST, zenohHostField.text.toString().trim())
                    .putBoolean(KEY_FAST_RECORD, fastRecordSwitch.isChecked)
                    .apply()
                render(Msdk.state.value, Bridge.state.value)
            }
            .show()
    }

    /**
     * Folds `--ez video` / `--es videoHost` / `--ei videoPort` into the saved
     * settings, so the rest of the app has one source of truth.
     *
     * `hasExtra` before `getBooleanExtra`, because absent and `false` mean
     * genuinely different things here: absent leaves whatever the operator chose
     * last time, `false` is a deliberate "not this session". A flag that could
     * only ever turn something on is a flag nobody can use to rule video out as
     * the cause of a problem.
     */
    private fun absorbVideoExtras() {
        val i = intent ?: return
        val edit = prefs.edit()
        var touched = false
        if (i.hasExtra(VideoRequest.EXTRA_ENABLED)) {
            edit.putBoolean(
                VideoRequest.PREF_ENABLED,
                i.getBooleanExtra(VideoRequest.EXTRA_ENABLED, false),
            )
            touched = true
        }
        i.getStringExtra(VideoRequest.EXTRA_HOST)?.trim()?.let {
            edit.putString(VideoRequest.PREF_HOST, it)
            touched = true
        }
        i.getIntExtra(VideoRequest.EXTRA_PORT, 0).takeIf { it > 0 }?.let {
            edit.putInt(VideoRequest.PREF_PORT, it)
            touched = true
        }
        if (touched) edit.apply()
    }

    /**
     * The same fold for `--ez zenoh` / `--es zenohHost` / `--ei zenohPort` / `--es zenohPrefix`.
     *
     * Written out beside [absorbVideoExtras] rather than generalised with it, because the two
     * differ in the one place a shared helper would have to be parameterised anyway — video has a
     * fallback to the GCS address and this has not — and because a `--ez zenoh false` that
     * silently turned video off would be a spectacular way to lose an afternoon.
     */
    private fun absorbZenohExtras() {
        val i = intent ?: return
        val edit = prefs.edit()
        var touched = false
        if (i.hasExtra(ZenohSettings.EXTRA_ENABLED)) {
            edit.putBoolean(
                ZenohSettings.PREF_ENABLED,
                i.getBooleanExtra(ZenohSettings.EXTRA_ENABLED, false),
            )
            touched = true
        }
        // `--ez zenohVideo false` has to be able to turn it *off*, for the same reason
        // `--ez zenoh false` does: a flag that can only add bandwidth is a flag nobody can use to
        // rule the video channel out as the cause of a link problem.
        if (i.hasExtra(ZenohSettings.EXTRA_VIDEO)) {
            edit.putBoolean(
                ZenohSettings.PREF_VIDEO,
                i.getBooleanExtra(ZenohSettings.EXTRA_VIDEO, false),
            )
            touched = true
        }
        // `--ez zenohDetections`, both directions, for `--ez zenohVideo`'s reason.
        if (i.hasExtra(ZenohSettings.EXTRA_DETECTIONS)) {
            edit.putBoolean(
                ZenohSettings.PREF_DETECTIONS,
                i.getBooleanExtra(ZenohSettings.EXTRA_DETECTIONS, false),
            )
            touched = true
        }
        i.getStringExtra(ZenohSettings.EXTRA_HOST)?.trim()?.let {
            edit.putString(ZenohSettings.PREF_HOST, it)
            touched = true
        }
        i.getIntExtra(ZenohSettings.EXTRA_PORT, 0).takeIf { it > 0 }?.let {
            edit.putInt(ZenohSettings.PREF_PORT, it)
            touched = true
        }
        i.getStringExtra(ZenohSettings.EXTRA_PREFIX)?.trim()?.takeIf { it.isNotEmpty() }?.let {
            edit.putString(ZenohSettings.PREF_PREFIX, it)
            touched = true
        }
        if (touched) edit.apply()
    }

    /**
     * What video should do, from the settings and the current GCS address.
     *
     * Recomputed rather than cached because [gcsHost] is what it falls back to,
     * and that changes when the operator edits it — a video target that silently
     * kept following the *previous* telemetry address would be the second silent
     * failure this file is meant to be rid of.
     */
    private fun videoPlan(): VideoRequest.Plan = VideoRequest.resolve(
        savedEnabled = prefs.getBoolean(VideoRequest.PREF_ENABLED, CockpitDefaults.GCS_VIDEO),
        savedHost = prefs.getString(VideoRequest.PREF_HOST, null),
        savedPort = prefs.getInt(VideoRequest.PREF_PORT, 0).takeIf { it > 0 },
        gcsHost = gcsHost,
    )

    /**
     * The video-recording switch.
     *
     * Nothing here can move an aircraft, which is why it is a plain switch with no admission rule
     * behind it and lives at the opposite end of the screen from the interlock. What it costs is
     * **disk**: about 625 kB/s at the stream rate `docs/video.md` assumes, so the label carries the
     * running total rather than making the operator go and find it. A number on the switch is the
     * difference between "recording" and "recording, and you have four minutes left".
     */
    private fun wireVideoRecording() {
        videoRecord.isChecked = Recorder.videoEnabled
        videoRecord.setOnClickListener {
            val now = Recorder.setVideoEnabled(videoRecord.isChecked)
            // Put the switch back if the recorder declined — it cannot today, and a control that
            // silently disagrees with the thing it controls is the bug this line exists to prevent.
            videoRecord.isChecked = now
            updateVideoRecordLabel()
        }
        updateVideoRecordLabel()
    }

    /**
     * **The tag detector's switch**, following [wireVideoRecording] and beside it on the screen for
     * the same reason: neither can move an aircraft. What this one costs is CPU, and therefore heat,
     * on a phone clamped to a controller in the sun — 0.68 cores over the floor while running,
     * measured on the aircraft — on an airframe whose characteristic failure is an overheat.
     *
     * ## Three states on one control, and why not two
     *
     * AUTO is the design and is what an operator should almost never touch: acquire on takeoff,
     * re-arm for the descent only if something was acquired (`vision/TagArming`). But *"manual
     * overrides the latch in both directions"* is a requirement, not a nicety — force it on for a
     * bench experiment where the aircraft never flies, and force it off when the phone is hot — so
     * two states would not have covered it.
     *
     * Tap moves between AUTO and OFF, which are the two an operator will ever want in the field.
     * A **long press** forces it on, which is the bench case and not a field one. The label always
     * names the mode it is in — `auto`, `off`, `on` — and the long press answers with a `Toast` that
     * says which way it went, so the state is never something to infer from a switch position. The
     * gesture itself is documented here and in `docs/tag-detector.md` rather than on a 136 dp
     * control: it is for whoever is running a bench experiment, and they are reading something.
     *
     * The switch's checked state tracks "will it ever run", so forced-on and auto both read as on
     * and only the text separates them.
     *
     * The state is not persisted, deliberately, and it is the one place this file differs from
     * `KEY_FAST_RECORD`. A forced-off detector surviving into the next session is a sensor an
     * operator turned off once, for a reason that has passed, silently missing an acquisition on a
     * later flight — and the acquisition happens in the first seconds of a takeoff, before anyone
     * would think to look.
     */
    private fun wireTagDetector() {
        tagDetect.setOnClickListener {
            val r = Bridge.tagRecogniser
            if (r == null) {
                // A switch that cannot control anything must not look as though it did.
                tagDetect.isChecked = false
                updateTagDetectLabel()
                Toast.makeText(this, "No on-board detector in this build", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            r.mode = if (r.mode == TagArm.OFF) TagArm.AUTO else TagArm.OFF
            updateTagDetectLabel()
        }
        tagDetect.setOnLongClickListener {
            val r = Bridge.tagRecogniser ?: return@setOnLongClickListener true
            // Long-press is a toggle into and out of forced-on rather than a one-way door: a
            // control that can only be escaped by a different gesture from the one that entered it
            // is a control people get stuck in.
            r.mode = if (r.mode == TagArm.ON) TagArm.AUTO else TagArm.ON
            updateTagDetectLabel()
            Toast.makeText(
                this,
                if (r.mode == TagArm.ON) "Tag detector forced on" else "Tag detector back to auto",
                Toast.LENGTH_SHORT,
            ).show()
            true
        }
        updateTagDetectLabel()
    }

    /**
     * The switch's label: the mode, and — while it is running — what it is actually doing.
     *
     * The running counters are on the label rather than somewhere else for the reason
     * [updateVideoRecordLabel] puts a byte count there: "armed" and "armed, and it has looked at
     * 340 frames" are different amounts of confidence, and the difference is exactly what an
     * operator wants before trusting a descent.
     */
    private fun updateTagDetectLabel() {
        val r = Bridge.tagRecogniser
        if (r == null) {
            tagDetect.isChecked = false
            tagDetect.text = "Tags: none"
            return
        }
        val c = r.counters()
        tagDetect.isChecked = r.mode != TagArm.OFF
        tagDetect.text = when {
            r.mode == TagArm.OFF -> "Tags: off"
            !c.armed -> "Tags: %s (idle)".format(if (r.mode == TagArm.ON) "on" else "auto")
            c.detected == 0L -> "Tags: %s (no frames)".format(if (r.mode == TagArm.ON) "on" else "auto")
            else -> "Tags: %d/%d seen".format(c.hits, c.detected)
        }
    }

    /**
     * The three at-a-glance lines above the log: battery, where the aircraft is relative to where
     * it started, and what the camera can see.
     *
     * **Relative to home, not to the takeoff datum**, and the two are not the same thing. Home is
     * where DJI says home is — it moves if a `DO_SET_HOME` ever lands — while the altitude datum is
     * where *this bridge* saw the aircraft take off. The horizontal pair is measured from home
     * because that is what the operator is picturing when they look up; the vertical is the datum's
     * because that is the number every commanded altitude in this project is expressed against, and
     * showing a different one here would invite the two to be confused at exactly the wrong moment.
     *
     * Everything is `—` when it is unknown rather than 0. A zero that means "no data" is the single
     * most dangerous thing a status display can print.
     */
    private fun renderVitals() {
        val st = StateCache.aircraftState()
        val battery = buildString {
            append("battery ")
            append(st.batteryPercent?.let { "%3d%%".format(it) } ?: "  —")
            st.voltageMv?.let { append("  %.1fV".format(it / 1000.0)) }
            st.batteryTempC?.let { append("  %.0f°C".format(it)) }
        }
        val here = com.dimensional.mini4pro.telemetry.Geo
            .coordinateOrNull(st.latitude, st.longitude)
        val home = com.dimensional.mini4pro.telemetry.Geo
            .coordinateOrNull(st.homeLatitude, st.homeLongitude)
        val pos = if (here != null && home != null) {
            val (n, e) = com.dimensional.mini4pro.guided.RepositionGuidance
                .nedMetres(home.first, home.second, here.first, here.second)
            "pos  N%+6.1f E%+6.1f  up%s".format(
                n, e, st.relativeAltitude?.let { "%+6.1f m".format(it) } ?: "     — ",
            )
        } else {
            "pos  —"
        }
        vitals.text = "$battery   $pos"

        // The tag line. Null detector and null sighting say different things on purpose: one is
        // "nothing is looking", the other is "something is looking and cannot see it", and an
        // operator deciding whether to trust an automatic landing needs to tell those apart.
        val detector = tagSighting
        val seen = detector?.latest()
        val held = detector?.latched()
        tagVitals.text = when {
            detector == null -> "tag  — no on-board detector"
            // **The age is on the line, always.** A sighting reaching a 25 Hz loop is 60–160 ms old
            // and at 3 m/s that is half a metre; a readout that showed a position without saying how
            // old it was would be the exact false comfort `Sighting.ageMillisAt` exists to prevent.
            seen != null -> "tag  id %d  x%+5.2f y%+5.2f z%+5.2f m  %.0fpx  %dms%s".format(
                seen.tagId, seen.x, seen.y, seen.z, seen.pixelSize,
                seen.ageMillisAt(android.os.SystemClock.elapsedRealtimeNanos()),
                if (seen.metric) "" else "  (assumed intrinsics)",
            )
            // Latched but not currently visible is the normal state for most of a flight — the tag
            // is undetectable above 8 m — and it is worth more to an operator than "not seen",
            // because it is the thing that decides whether the descent will have a sensor at all.
            held != null -> held.fix?.let {
                "tag  id %d latched at N%+.1f E%+.1f (coarse), not in view".format(
                    held.tagId, it.northM, it.eastM,
                )
            } ?: "tag  id %d latched, position unknown, not in view".format(held.tagId)
            else -> "tag  not seen"
        }
        updateTagDetectLabel()
    }

    private fun updateVideoRecordLabel() {
        // **The switch follows the recorder, not the other way round**, and since 2026-07-30 it has
        // to: recording now starts ON with the session (`CockpitDefaults.RECORD_VIDEO`), and the
        // sidecar that decides it is built inside `Recorder.start` — long after `wireVideoRecording`
        // read `videoEnabled` off a recorder that had not started. Without this line the first
        // session of every launch would write video under a switch showing OFF, which is precisely
        // the disagreement the click handler two hundred lines up exists to prevent. Assigning from
        // the single owner on the drawing tick costs nothing and cannot drift.
        videoRecord.isChecked = Recorder.videoEnabled
        val counters = Recorder.videoCounters()
        videoRecord.text = when {
            counters == null -> "Record video"
            counters.budgetSpent -> "Video: budget spent"
            // **No frames have arrived at all**, which is a different problem from not recording
            // and the one that has actually bitten: the passthrough sometimes needs a bridge
            // restart before the aircraft delivers anything. Saying so on the switch means noticing
            // it before the flight rather than in the log afterwards.
            counters.seen == 0L -> "Record video (no feed)"
            counters.stalled -> "Video: feed stopped"
            !Recorder.videoEnabled -> "Record video (%d seen)".format(counters.seen)
            else -> "Video: %d fr / %d MB".format(counters.frames, counters.bytes / (1024 * 1024))
        }
    }

    /**
     * **The phone's Take off control** — the second thing on this screen that can put an
     * aircraft in the air (QGC's Takeoff button is the first), and it borrows the descend
     * button's grammar wholesale so nothing about it has to be learned separately:
     *
     *  - **Arming asks first**: the dialog says the whole sequence — DJI's own ~1.2 m hop, the
     *    bridge's climb to 10 m, the camera going to nadir at the handoff — and that moving the
     *    RC sticks cancels the climb completely.
     *  - **The interlock is named on screen**, before the dialog and again if the dispatcher
     *    refuses on it: Ivan's rule for this button verbatim — a takeoff with the interlock off
     *    must be refused by name, not by silence.
     *
     * The *decision* is entirely the dispatcher's (`CommandDispatcher.takeoffFromPhone` runs
     * the same corridor QGC's `MAV_CMD_NAV_TAKEOFF` travels); this listener is routing,
     * downstream of a human press — the `followSimulatorInterlock` placement rule, honoured in
     * the same direction. The interlock check here is a courtesy toast; the dispatcher checks
     * it again and answers `UNSUPPORTED` regardless, so a race cannot launch anything.
     */
    private fun wireTakeoff() {
        takeOff.setOnClickListener {
            if (replay != null) return@setOnClickListener
            if (!Bridge.commandInterlock.enabled) {
                Toast.makeText(this, "Take off refused - the command interlock is off", Toast.LENGTH_LONG).show()
                return@setOnClickListener
            }
            val height = com.dimensional.mini4pro.command.CommandDispatcher.PHONE_TAKEOFF_HEIGHT_M
            AlertDialog.Builder(this)
                .setTitle("Take off to ${height.toInt()} m?")
                .setMessage(
                    "The aircraft will start its motors and take off: DJI climbs to about " +
                        "1.2 m on its own, then this bridge climbs it to ${height.toInt()} m " +
                        "and points the camera straight down.\n\n" +
                        "Moving the RC sticks cancels the climb completely - DJI's own hop " +
                        "cannot be recalled once the motors start.",
                )
                .setNegativeButton("Cancel", null)
                .setPositiveButton("Take off") { _, _ ->
                    when (Bridge.takeoffFromPhone()) {
                        com.dimensional.mini4pro.command.Verdict.ACCEPTED -> Unit
                        // The dispatcher's interlock-off answer, named on screen — the race
                        // where the interlock dropped between the courtesy check and this press.
                        com.dimensional.mini4pro.command.Verdict.UNSUPPORTED -> Toast.makeText(
                            this, "Take off refused - the command interlock is off", Toast.LENGTH_LONG,
                        ).show()
                        // The dispatcher has already announced the named reason on the status
                        // channel; this just points the eye there.
                        else -> Toast.makeText(
                            this, "Refused - the status log names the reason", Toast.LENGTH_LONG,
                        ).show()
                    }
                }
                .show()
        }
    }

    /**
     * The takeoff control's visibility, on the 200 ms drawing tick — the descend button's
     * convention with this control's own sensor: the aircraft. Visible when there is a
     * connected aircraft that is not flying (null `isFlying` shows too — DJI declining to say
     * is not evidence either way, and every deeper gate refuses by name); never during a
     * replay. No armed face to render: once the press is confirmed the sequence's state lives
     * on the status channel and in the situation view, and the button's job is over.
     */
    private fun renderTakeoff(player: ReplayPlayer?) {
        val live = if (player == null && Msdk.state.value.registered) StateCache.aircraftState() else null
        val show = live != null && live.fcConnected && live.isFlying != true
        takeOff.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
    }

    /**
     * **M3 Stage D's arm control** — the one place a tag-tracked descent can be started, and it
     * follows the interlock's own grammar so nothing about it has to be learned separately:
     *
     *  - **It appears only when a tag is latched** ([renderDescend]): the operator's brief is
     *    that the option exists exactly when the sensor does.
     *  - **Arming asks first**, like enabling the interlock, because this is the second control
     *    on this screen that can make an aircraft move — and the dialog says the two things the
     *    operator must know: where it ends (holding at ~0.6 m; Stage C is not built) and that
     *    any RC stick movement kills it completely.
     *  - **Disarming is immediate**, like every withdrawal: the same button while armed, no
     *    dialog, because a person reaching to stop a descent is not someone to put a modal in
     *    front of.
     *
     * The *decision* is entirely the engine's (`GuidedStickEngine.armTagDescent` runs every
     * gate and names every refusal); this listener is routing, downstream of a human press —
     * the `followSimulatorInterlock` placement rule, honoured in the same direction. The
     * interlock check here is a courtesy toast; the engine checks it again and answers
     * `UNSUPPORTED` regardless, so a race cannot arm anything.
     */
    private fun wireTagDescent() {
        // Stage C's toggle: the label always says the state, because a switch whose caption is
        // its name alone can be misread across the cockpit. Read at arm time only.
        //
        // **Default ON since 2026-07-30** — Ivan: *"can we make this last stretch autoland the
        // default toggled on in the app?"*, after thirteen recorded descents and the first
        // mission that flew itself. The default lives in `activity_main.xml` (android:checked
        // with android:text agreeing, because this listener only fires on a *change* and a
        // first-run caption would otherwise lie about the state), which is the same place it
        // has always lived; nothing else moved. In particular the option is still **taken at
        // arm time**, behind the "Arm FULL AUTOLAND?" dialog that names where the flight ends,
        // and every gate the engine applies is untouched — a default is which way the switch
        // starts, never a decision taken on the operator's behalf.
        fullAutoland.setOnCheckedChangeListener { _, checked ->
            fullAutoland.text = if (checked) "Autoland: ON" else "Autoland: OFF"
        }
        // Long press: **shadow mode**, the validation gate — the controller computes
        // everything and actuates nothing, its would-be commands go to the flight record, and
        // the comparison instrument appears. A long press because it is the bench/validation
        // gesture, exactly as the tag switch's forced-on is; the toast is the state, said out
        // loud, because a mode that cannot move the aircraft still must never be ambiguous.
        descendTag.setOnLongClickListener {
            if (replay != null) return@setOnLongClickListener true
            val on = Bridge.shadowComparison() == null
            if (Bridge.setShadowDescent(on)) {
                Toast.makeText(
                    this,
                    if (on) "Shadow descent ON - the aircraft will not move" else "Shadow descent off",
                    Toast.LENGTH_LONG,
                ).show()
            }
            true
        }
        descendTag.setOnClickListener {
            if (replay != null) return@setOnClickListener
            if (Bridge.shadowComparison() != null) {
                // Shadow mode has its own off switch: a tap while shadowing turns it off,
                // because the button is showing the shadow face and a control must act on the
                // state it shows.
                Bridge.setShadowDescent(false)
                return@setOnClickListener
            }
            if (Bridge.guidedSituation().descent != null) {
                if (!Bridge.disarmTagDescent()) {
                    Toast.makeText(this, "Nothing to disarm", Toast.LENGTH_SHORT).show()
                }
                return@setOnClickListener
            }
            if (!Bridge.commandInterlock.enabled) {
                Toast.makeText(this, "Enable commands first - the interlock is off", Toast.LENGTH_LONG).show()
                return@setOnClickListener
            }
            // Stage C: the toggle is read at the moment of the press, and the dialog says
            // which flight the operator is about to arm — a descent that stops and holds, or
            // one that puts the aircraft on the ground.
            val autoland = fullAutoland.isChecked
            AlertDialog.Builder(this)
                .setTitle(if (autoland) "Arm FULL AUTOLAND?" else "Arm tag descent?")
                .setMessage(
                    if (autoland) {
                        "The aircraft will centre over the latched tag, descend to DJI's " +
                            "obstacle floor, and then hand the landing to DJI itself - " +
                            "LANDING ON THE TAG, blind for the last stretch when DJI turns " +
                            "the camera away.\n\n" +
                            "Moving the RC sticks cancels our part at any moment and asks " +
                            "DJI to stop its landing too - whether DJI honours that stop " +
                            "is not guaranteed."
                    } else {
                        "The aircraft will centre over the latched tag and descend, ending in a " +
                            "hold about 0.6 m above it. It does not land.\n\n" +
                            "Moving the RC sticks cancels it completely - re-arming takes a fresh " +
                            "press of this button."
                    },
                )
                .setNegativeButton("Cancel", null)
                .setPositiveButton(if (autoland) "Arm autoland" else "Arm") { _, _ ->
                    when (Bridge.armTagDescent(fullAutoland = autoland)) {
                        com.dimensional.mini4pro.command.Verdict.ACCEPTED -> Unit
                        null -> Toast.makeText(this, "Bridge is not running", Toast.LENGTH_SHORT).show()
                        // The engine has already announced the named reason on the status
                        // channel; this just points the eye there.
                        else -> Toast.makeText(
                            this, "Refused - the status log names the reason", Toast.LENGTH_LONG,
                        ).show()
                    }
                }
                .show()
        }
    }

    /**
     * The arm control's visibility and label, on the 200 ms drawing tick.
     *
     * Visible when a tag is latched **or** a descent is somehow still armed (so the disarm
     * face can never vanish while there is something to disarm), never during a replay. The
     * label is the state: the arm face, the disarm face, and the terminal face — "holding" —
     * which is the one the operator reads before taking the sticks back.
     */
    private fun renderDescend(player: ReplayPlayer?) {
        val descent = Bridge.guidedSituation().descent
        val latched = tagSighting?.latched()
        val shadowOn = Bridge.shadowComparison() != null
        val show = player == null && (latched != null || descent != null || shadowOn)
        descendTag.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
        // Stage C's toggle rides the arm control's visibility exactly, so the pair is always
        // read together — and stays interactable mid-descent only as a label; the option was
        // pinned into the run at arm time and the engine ignores later flips.
        fullAutoland.visibility = descendTag.visibility
        if (!show) return
        descendTag.text = when {
            // The shadow faces say SHADOW first: an operator must never wonder whether the
            // aircraft is about to move, and this label is the state the tap acts on.
            shadowOn && descent?.shadow == true && descent.terminal -> "SHADOW done (off)"
            shadowOn && descent?.shadow == true -> "SHADOW watching (off)"
            shadowOn -> "SHADOW gated (off)"
            descent == null -> "Descend on tag…"
            // The landing faces: visibly distinct from every Stage B face — DJI is flying —
            // and the blind/aligned distinction is the one fact the operator watching the
            // last metres needs; blind is the EXPECTED ending (DJI turns the camera away).
            descent.landing && descent.blind -> "DJI LANDING blind - DISARM"
            descent.landing -> "DJI LANDING - DISARM"
            descent.terminal -> "Descent holding - disarm"
            // The approach face: armed above the band, flying down into it. Distinct from
            // "Descending" because the operator who just armed at the 10 m hover is watching
            // a leg with different weather — sparse decodes, baro height — and the handoff
            // into the plain descending face is the visible band entry.
            descent.approach -> "Approaching band - DISARM"
            else -> "Descending - DISARM"
        }
    }

    /**
     * Wires the interlock switch, and — since 2026-07-30 — **arms it at startup**.
     *
     * The default and what it costs are argued in [CockpitDefaults.COMMAND_INTERLOCK]. What lives
     * here is the mechanism, and one property worth stating where the code is: `enable()` is called
     * from this file and nowhere else. Adding a startup caller does not weaken that — `onCreate`
     * runs before any socket exists, which makes it a colder path to `enable()` than the button
     * was, not a warmer one. `CommandInterlock`'s documented guarantee is about *inbound MAVLink*
     * being unable to arm itself, and no message can reach a process that has not finished starting.
     *
     * The replay gate is applied identically at startup and on the switch, because it is the one
     * refusal here that is about the aircraft rather than about the operator's attention: a loaded
     * replay must never arm a real airframe, and it says so by name.
     */
    private fun wireCommandInterlock() {
        // Startup arming. `armCommands` is the shared path, so the replay refusal and the switch
        // state cannot drift apart between the two callers.
        if (CockpitDefaults.COMMAND_INTERLOCK) armCommands()
        commands.isChecked = Bridge.commandInterlock.enabled
        commands.setOnClickListener {
            if (!commands.isChecked) {
                Bridge.commandInterlock.disable()
                return@setOnClickListener
            }
            // Revert first, so a refused arm cannot leave the switch showing "on" over an
            // interlock that is off. `armCommands` puts it back if it succeeds.
            commands.isChecked = false
            armCommands()
        }
    }

    /**
     * Turns the interlock on unless the replay gate refuses, and leaves the switch agreeing with
     * whatever happened.
     *
     * **The confirmation dialog that used to stand here is gone** (Ivan, 2026-07-30: *"We don't
     * need validation confirmation"*). It asked whether QGroundControl's Return and Land buttons
     * should reach DJI, and warned that there is no Emergency Stop — both still true, both still on
     * the record and in `CockpitDefaults`. What it had stopped being was informative: its answer
     * had been "Allow" every session since the interlock existed, and it was being answered on the
     * ground, minutes before the buttons it describes are pressed. The real motor cut is the stick
     * gesture on the physical RC and was never in this dialog's gift.
     *
     * The simulator branch went with it, since it existed only to *skip* this dialog.
     */
    private fun armCommands() {
        // The rule, not the greyed switch, is the authority. `performClick()` runs a listener
        // whether or not the view is enabled — and the banner beside this switch is wired to call
        // exactly that — so the check lives here as well as in the view state, and a
        // disabled-looking control cannot be talked into arming.
        val mayArm = ReplayAdmission.mayArm(replay != null)
        if (!mayArm.allowed) {
            commands.isChecked = false
            Toast.makeText(this, mayArm.reason, Toast.LENGTH_LONG).show()
            return
        }
        Bridge.commandInterlock.enable()
        commands.isChecked = true
    }

    /**
     * The last simulator phase this screen acted on — an **edge detector**, and the only thing
     * remembered about the simulator anywhere in the interlock story.
     *
     * Not a "we are in a simulator" flag: nothing reads it to decide whether something is
     * allowed. It exists purely so [followSimulatorInterlock] can tell a *change* from a
     * repetition, and every question about what may happen now is asked of the live phase.
     */
    private var lastSimulatorPhase: SimulatorPhase? = null

    /**
     * Applies `SimulatorInterlock`'s rule: a DJI-confirmed simulator arms the command path, and
     * losing it disarms.
     *
     * **The `enable()` call lives here, in the Activity, and that placement is load-bearing.**
     * `CommandInterlock`'s third documented property is that no inbound MAVLink message can turn
     * it on, and that property is structural only for as long as every caller of `enable()` sits
     * in the UI layer where a human pressed something. The *decision* is computed in
     * `SimulatorInterlock` — pure, testable, reachable from nowhere — and only ever applied from
     * here, downstream of an operator who pressed the simulator button. Moving this call into
     * `Bridge` or `SimulatorControl` would put `enable()` on a DJI callback path and turn a
     * structural guarantee into a comment.
     *
     * Called from the 200 ms drawing tick rather than only from the state collector, so the
     * disable arrives even if the collector is not running for whatever reason. It is an edge
     * detector, so calling it often costs nothing.
     */
    private fun followSimulatorInterlock() {
        val phase = Bridge.simulator.phase
        val effect = SimulatorInterlock.effectOf(lastSimulatorPhase, phase)
        lastSimulatorPhase = phase
        when (effect) {
            SimulatorInterlock.Effect.NONE -> return
            SimulatorInterlock.Effect.ENABLE -> Bridge.commandInterlock.enable()
            SimulatorInterlock.Effect.DISABLE -> {
                Bridge.commandInterlock.disable()
                // Said out loud. A command path that goes away silently is one the operator
                // discovers from a refused command, which is the worst moment to learn it.
                Toast.makeText(this, SimulatorInterlock.DROPPED, Toast.LENGTH_LONG).show()
            }
        }
        render(Msdk.state.value, Bridge.state.value)
    }

    /**
     * Pause and Stop: the two withdrawals, from the phone.
     *
     * **Neither asks anything.** Every other control on this screen that changes what an
     * aircraft may do puts a dialog in front of it; these two are the exception, and the
     * asymmetry is the same one the interlock already has — enabling is confirmed, disabling is
     * immediate. `docs/decisions/2026-07-26-m3-guided-control.md` Q3 makes withdrawals honoured
     * from any origin, and a person reaching for STOP is not someone to put a modal in front of.
     *
     * Both are idempotent and both are safe when nothing is happening, so the buttons never need
     * to be *right* — only present. They are greyed rather than hidden when there is nothing to
     * withdraw, because a control that appears is a control that has to be found first.
     */
    private fun wireWithdrawals() {
        pauseManoeuvre.setOnClickListener {
            if (replay != null) return@setOnClickListener
            // A refusal is shown rather than swallowed: "nothing to pause" is information.
            if (!Bridge.pauseManoeuvre()) {
                Toast.makeText(this, "Nothing to pause", Toast.LENGTH_SHORT).show()
            }
            renderSituation()
        }
        stopManoeuvre.setOnClickListener {
            if (replay != null) return@setOnClickListener
            Bridge.stopManoeuvre()
            // The interlock went off with it, so the switch and the banner must follow now
            // rather than at the next telemetry tick.
            render(Msdk.state.value, Bridge.state.value)
            renderSituation()
        }
    }

    /**
     * What the two withdrawal buttons offer, given what there is to withdraw.
     *
     * Disabled wholesale during a replay: there is no aircraft in that picture, and a live
     * withdrawal fired from a screen showing last week's flight is the confusion this feature is
     * arranged to prevent. STOP stays available whenever the interlock is on even with no
     * manoeuvre flying, because disarming is itself a withdrawal and the whole point of the
     * button is that it does not need the operator to work out which case they are in.
     */
    private fun renderWithdrawals(player: ReplayPlayer?) {
        val guided = Bridge.guidedSituation()
        val replaying = player != null
        pauseManoeuvre.isEnabled = !replaying && guided.hasManoeuvre
        stopManoeuvre.isEnabled = !replaying &&
            (guided.phase != GuidedPhase.IDLE || Bridge.commandInterlock.enabled)
    }

    /**
     * The recording picker, and the two rules that decide whether it may open at all.
     *
     * Loading happens on the IO dispatcher — a session file is megabytes, and
     * `FlightRecordReader` skips the MAVLink lines before parsing precisely because it is
     * megabytes. The picker itself is a plain list dialog: this is the *phone's own* log
     * directory, so there is no file browser to build and no permission to ask for.
     */
    private fun wireReplay() {
        replayButton.setOnClickListener { openReplayPicker() }
        replayPlay.setOnClickListener {
            replay?.toggle(android.os.SystemClock.elapsedRealtime())
            renderSituation()
        }
        // Closing consults nothing. Ending a picture can only make the screen more honest, so
        // nothing may refuse it — the same reasoning that makes a withdrawal unconditional.
        // Ending a *publication* is the same withdrawal one layer out, which is why closing
        // goes through the one function that ends both.
        replayClose.setOnClickListener {
            closeReplay()
            renderSituation()
            render(Msdk.state.value, Bridge.state.value)
        }
        replayToMavlink.setOnClickListener { onPublishSwitch(StateSource.Sink.MAVLINK) }
        replayToZenoh.setOnClickListener { onPublishSwitch(StateSource.Sink.ZENOH) }
    }

    /**
     * The two switches, built in code and put on the end of the strip the XML defines.
     *
     * `setOnClickListener`, never `setOnCheckedChangeListener` — the interlock's rule, for the
     * interlock's reason: a click is the operator and `isChecked = …` is us re-syncing the view,
     * and conflating them is how a redraw comes to look like consent.
     */
    private fun buildReplayPublishSwitches() {
        fun make(label: String) = SwitchCompat(this).apply {
            text = label
            isChecked = false
            textSize = 11f
            setTextColor(0xFFFFFFFF.toInt())
            setPadding(24, 0, 0, 0)
        }
        replayToMavlink = make("MAVLink out")
        replayToZenoh = make("Zenoh out")
        replayBar.addView(replayToMavlink)
        replayBar.addView(replayToZenoh)
    }

    /**
     * A publishing switch was tapped.
     *
     * **Turning on asks; turning off does not** — the interlock's asymmetry, and the same
     * argument: refusing to stop describing a recording to somebody could only ever make the
     * world less honest. `ReplayAdmission.mayPublish` owns the refusal and its sentence; this
     * function owns putting the switch back where it was when the answer is no, which matters
     * because the tap has already moved it.
     */
    private fun onPublishSwitch(sink: StateSource.Sink) {
        val view = if (sink == StateSource.Sink.MAVLINK) replayToMavlink else replayToZenoh
        val wanted = view.isChecked
        if (wanted) {
            val verdict = ReplayAdmission.mayPublish(
                // The **live** aircraft, never the replayed one: a recording worth publishing has
                // `fcConnected` true for most of its length, and reading that here would refuse
                // every publication there is. `ReplayAdmission.mayPublish` says so at the field.
                aircraftConnected = StateSource.liveState().fcConnected,
                interlockEnabled = Bridge.commandInterlock.enabled,
                manoeuvreEngaged = Bridge.guidedSituation().phase != GuidedPhase.IDLE,
            )
            if (!verdict.allowed) {
                view.isChecked = false
                Toast.makeText(this, verdict.reason, Toast.LENGTH_LONG).show()
                return
            }
        }
        when (sink) {
            StateSource.Sink.MAVLINK -> replayToMavlinkOn = wanted
            StateSource.Sink.ZENOH -> replayToZenohOn = wanted
        }
        applyReplayPublication(announceZenohChange = sink == StateSource.Sink.ZENOH)
        renderSituation()
        render(Msdk.state.value, Bridge.state.value)
    }

    /**
     * Installs, updates or removes the feed the outbound paths read, and says so where it has to
     * be said.
     *
     * The feed is a lambda over the player rather than a copy of anything: `Bridge.tick` and the
     * Zenoh sampler run on their own threads at their own rates and must see whatever the cursor
     * is on *now*, exactly as they see whatever `StateCache` holds now. That is what makes this
     * "the same way actual drone data is" rather than a second delivery mechanism.
     *
     * @param announceZenohChange true when the Zenoh switch itself moved, so the bus is told. The
     *   announcement is not sent for a MAVLink-only change: `status` is a Zenoh channel, and a
     *   sentence there about a stream its subscribers cannot see would be noise beside telemetry
     *   that is still the live aircraft's.
     */
    private fun applyReplayPublication(announceZenohChange: Boolean) {
        val player = replay
        if (player == null) {
            StateSource.install(null)
            return
        }
        val wasPublishing = StateSource.publishing
        StateSource.install(
            StateSource.Feed(
                name = player.name,
                toMavlink = replayToMavlinkOn,
                toZenoh = replayToZenohOn,
                read = { replay?.current()?.state },
            )
        )
        val nowPublishing = replayToMavlinkOn || replayToZenohOn
        if (announceZenohChange) {
            if (replayToZenohOn) {
                ZenohBus.announceReplay(
                    ReplayPublication.announcement(player.name, replayToMavlinkOn, true)
                )
            } else {
                ZenohBus.announceReplay(ReplayPublication.endedAnnouncement(player.name))
            }
        }
        if (nowPublishing != wasPublishing) recordReplayPublication(player.name, nowPublishing)
    }

    /**
     * Marks the window in the flight record. **The record is never written from a replayed
     * state** — `Recorder.sample` reads the live aircraft and consults no seam — so these two
     * lines are what stands in for that absence. `replay/ReplayPublication` argues both halves.
     */
    private fun recordReplayPublication(name: String, started: Boolean) {
        val sinks = ReplayPublication.sinkNames(replayToMavlinkOn, replayToZenohOn)
        try {
            Recorder.event(
                if (started) EventCode.REPLAY_PUBLISH_START else EventCode.REPLAY_PUBLISH_STOP,
                if (started) "replay $name published to $sinks — the states below are NOT this aircraft"
                else "replay $name is no longer published",
                LogEntry.SEV_WARN,
            )
        } catch (t: Throwable) {
            // An evidence problem must never become a UI problem. `Tap`'s standing rule.
        }
    }

    /**
     * Ends the recording **and** anything it was being published to, in that order.
     *
     * One function, so that closing can never leave a feed installed over a player nobody is
     * advancing — which would freeze one sample onto two networks for as long as the app ran.
     */
    private fun closeReplay() {
        val name = replay?.name
        val wasPublishing = replayToMavlinkOn || replayToZenohOn
        val wasZenoh = replayToZenohOn
        replayToMavlinkOn = false
        replayToZenohOn = false
        StateSource.install(null)
        replay = null
        replayToMavlink.isChecked = false
        replayToZenoh.isChecked = false
        if (wasZenoh && name != null) {
            ZenohBus.announceReplay(ReplayPublication.endedAnnouncement(name))
        }
        if (wasPublishing && name != null) recordReplayPublication(name, started = false)
    }

    /**
     * Hands the sightings the cursor has just crossed to the same publisher the live detector
     * feeds — `ZenohBus.publishDetection`, behind the same `detections` switch, through the same
     * gates and the same encoder.
     *
     * Only while Zenoh-out is on: detections have no MAVLink representation at all, so the
     * MAVLink switch has nothing to say about them.
     *
     * Drained rather than read, and `ReplayPlayer.takeSightings` explains why: a state is a level
     * and a sighting is an event, so re-publishing the one the cursor happens to be sitting past
     * would be a detection that never happened.
     */
    private fun publishReplaySightings(player: ReplayPlayer) {
        if (!replayToZenohOn) return
        for (s in player.takeSightings()) ZenohBus.publishDetection(s.sighting)
    }

    private fun openReplayPicker() {
        val verdict = ReplayAdmission.mayReplay(
            interlockEnabled = Bridge.commandInterlock.enabled,
            manoeuvreEngaged = Bridge.guidedSituation().phase != GuidedPhase.IDLE,
        )
        if (!verdict.allowed) {
            Toast.makeText(this, verdict.reason, Toast.LENGTH_LONG).show()
            return
        }
        val entries = FlightLogLibrary.list(applicationContext)
        if (entries.isEmpty()) {
            Toast.makeText(this, "No recordings on this phone yet", Toast.LENGTH_LONG).show()
            return
        }
        AlertDialog.Builder(this)
            .setTitle("Replay a recorded flight")
            .setItems(entries.map { it.label() }.toTypedArray()) { _, which ->
                loadReplay(entries[which])
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun loadReplay(entry: FlightLogLibrary.Entry) {
        lifecycleScope.launch {
            val player = try {
                withContext(Dispatchers.IO) { FlightLogLibrary.load(entry) }
            } catch (t: Throwable) {
                Toast.makeText(this@MainActivity, "Could not read ${entry.name}: $t", Toast.LENGTH_LONG).show()
                return@launch
            }
            if (player.isEmpty) {
                // A file with no `dji_state` lines is a real case — a session that recorded only
                // MAVLink — and saying so beats a blank picture of somewhere.
                Toast.makeText(this@MainActivity, "${entry.name} has no aircraft states to replay", Toast.LENGTH_LONG).show()
                return@launch
            }
            // Re-checked here and not only at the picker: the dialog and the file read both take
            // time, and an interlock could have been armed in between.
            val verdict = ReplayAdmission.mayReplay(
                interlockEnabled = Bridge.commandInterlock.enabled,
                manoeuvreEngaged = Bridge.guidedSituation().phase != GuidedPhase.IDLE,
            )
            if (!verdict.allowed) {
                Toast.makeText(this@MainActivity, verdict.reason, Toast.LENGTH_LONG).show()
                return@launch
            }
            // Ends whatever was open, including anything it was being published to. A new
            // recording must never inherit the last one's switches: the operator turned those on
            // for a flight they had chosen, and silently carrying them onto a different one is
            // the app publishing something nobody asked it to.
            closeReplay()
            replay = player
            // A new recording is a new flight. Both are `REPLAY`, so nothing about the source
            // could tell them apart, and last recording's path drawn under this one's would be
            // two afternoons in one picture.
            flownTrack.clear()
            player.play(android.os.SystemClock.elapsedRealtime())
            renderSituation()
            render(Msdk.state.value, Bridge.state.value)
        }
    }

    /**
     * The replay strip: gone entirely when nothing is replayed, so its presence is itself the
     * statement. One of three simultaneous indicators — the others are the watermark across the
     * whole picture ([SituationView]) and the command strip's wording and colour
     * ([renderBanner]).
     */
    private fun renderReplayBar(player: ReplayPlayer?) {
        if (player == null) {
            replayBar.visibility = android.view.View.GONE
            return
        }
        replayBar.visibility = android.view.View.VISIBLE
        // The values are the authority and the switches reflect them — the interlock's rule.
        replayToMavlink.isChecked = replayToMavlinkOn
        replayToZenoh.isChecked = replayToZenohOn
        replayPlay.text = if (player.playing) "Pause" else "Play"
        val at = player.positionSeconds.toInt()
        val of = player.durationSeconds.toInt()
        replayLabel.text = "REPLAY  ${player.name}  ${at}s / ${of}s"
    }

    /**
     * The simulator button. Starting asks **once**; stopping our own asks not at all.
     *
     * ## Why it used to ask twice, and why it does not any more
     *
     * There were two questions, deliberately separated: one about software, one about a
     * propeller at head height. The physical one existed because DJI describes
     * `SimulatorState.areMotorsOn` as *"whether the aircraft motor in the simulator has started
     * to spin"* and **whether the physical motors turn on a Mini 4 Pro was never verified** —
     * `docs/simulator.md` carried it as open item S2, treated as a requirement until measured.
     *
     * **It has now been measured. On the bench, 2026-07-27, the motors do not spin** (S2 in that
     * document, now recorded as observed rather than reasoned). So the dialog was asking a
     * question with a known answer, which is the kind of friction that trains an operator to tap
     * through dialogs — including the one that still matters. Propellers off remains the
     * standing bench practice and is still written into §3; it is no longer a modal, because a
     * modal is for a decision and this is no longer one.
     *
     * [confirmStart] is untouched and still asks: what the operator will be looking at
     * afterwards is fabricated, and that question has not become any less real.
     *
     * Stopping is asymmetric in the same way the interlock is, with one exception: ending a
     * simulator **this process did not start** is an act on an aircraft whose state we cannot
     * explain, so that one asks.
     */
    private fun wireSimulator() {
        simulator.setOnClickListener {
            when (Bridge.simulator.phase) {
                SimulatorPhase.ACTIVE -> report(Bridge.simulator.stop())
                SimulatorPhase.FOREIGN -> confirmStopForeign()
                SimulatorPhase.OFF, SimulatorPhase.UNKNOWN -> confirmStart()
                // The button is disabled in these phases; a queued click is dropped rather than
                // becoming a second request for something already in flight.
                SimulatorPhase.STARTING, SimulatorPhase.STOPPING -> Unit
            }
        }
    }

    /** What the operator will be looking at afterwards. */
    private fun confirmStart() {
        val request = simulatorRequest()
        AlertDialog.Builder(this)
            .setTitle("Start the simulator?")
            .setMessage(
                "QGroundControl will show position, altitude, attitude and velocity generated " +
                    "by the flight controller. None of it is a real flight, and no result from " +
                    "it proves anything about how the aircraft flies.\n\n" +
                    "Start position: ${request.latitude}, ${request.longitude}\n" +
                    "Satellites:     ${request.satelliteCount}\n\n" +
                    "Commands will be switched ON once DJI confirms the simulator, and switched " +
                    "OFF again the moment it stops.\n\n" +
                    "The simulator lives on the aircraft. Stopping the bridge stops it; killing " +
                    "this app does NOT — it keeps running until you stop it here or " +
                    "power-cycle the aircraft.",
            )
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Start simulator") { _, _ -> report(Bridge.simulator.start(request)) }
            .show()
    }

    private fun confirmStopForeign() {
        AlertDialog.Builder(this)
            .setTitle("Stop a simulator this app did not start?")
            .setMessage(
                "DJI reports a simulator running, and nothing in this session started it — a " +
                    "crashed run, DJI Assistant, or another app.\n\n" +
                    "Stopping it ends a simulated flight somebody else may be watching.",
            )
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Stop it") { _, _ -> report(Bridge.simulator.stop()) }
            .show()
    }

    /**
     * Where to put the simulated aircraft.
     *
     * The real aircraft's own position first, so a bench session outdoors simulates where it
     * actually is and DJI's flysafe database sees the truth. Falling back to the measured site
     * rather than to 0/0 — the seed feeds that same database, and a placeholder in a restricted
     * zone produces a simulator that refuses to fly for reasons nothing on this screen explains.
     * Both are overridable per launch, for bench work at an address that is neither.
     */
    private fun simulatorRequest(): SimulatorRequest {
        val live = if (Msdk.state.value.registered) StateCache.aircraftState() else null
        return SimulatorRequest(
            latitude = intent?.getStringExtra(EXTRA_SIM_LAT)?.toDoubleOrNull()
                ?: live?.latitude
                ?: SimulatorRequest.DEFAULT_LATITUDE,
            longitude = intent?.getStringExtra(EXTRA_SIM_LON)?.toDoubleOrNull()
                ?: live?.longitude
                ?: SimulatorRequest.DEFAULT_LONGITUDE,
            satelliteCount = intent?.getIntExtra(EXTRA_SIM_SATS, -1)?.takeIf { it >= 0 }
                ?: SimulatorRequest.DEFAULT_SATELLITES,
        )
    }

    /**
     * A refusal is shown, never swallowed. `SimulatorOutcome.Requested` says nothing on purpose:
     * the answer to "did it start?" is the banner changing, which comes from DJI.
     */
    private fun report(outcome: SimulatorOutcome?) {
        if (outcome is SimulatorOutcome.Refused) {
            Toast.makeText(this, "Simulator: ${outcome.reason}", Toast.LENGTH_LONG).show()
        }
    }

    /**
     * Starts the recorder and then the bridge, in that order, so the very first
     * datagram of a session — including QGC's connect exchange — is on the record.
     */
    private fun startBridge(host: String, port: Int = MavlinkLink.DEFAULT_GCS_PORT) {
        Recorder.start(
            applicationContext,
            Recorder.Config(
                // The intent extra still wins, so a scripted session can ask for any rate; the
                // saved switch is what an operator sets by hand.
                stateHz = intent?.getStringExtra(EXTRA_RECORD_HZ)?.toDoubleOrNull()
                    ?: if (prefs.getBoolean(KEY_FAST_RECORD, CockpitDefaults.FAST_RECORD)) 25.0 else 5.0,
                // Seeds the sidecar's switch; the operator can still flip it mid-session and that
                // takes effect on the next frame. See `CockpitDefaults.RECORD_VIDEO` for the disk
                // arithmetic this spends by default.
                video = CockpitDefaults.RECORD_VIDEO,
                note = intent?.getStringExtra(EXTRA_NOTE),
            ),
        )
        // Through the service, never Bridge.start directly. Started from an Activity
        // the bridge dies with the foreground: Android freezes cached processes, and
        // a session on 2026-07-25 lost 601 s of telemetry in one silence that way.
        // The service also owns Bridge.stop, so the link cannot outlive it.
        //
        // Video goes with it, resolved here where the GCS address it falls back to
        // is known. It is also why video runs in the service rather than the
        // activity: a camera stream owned by a foreground Activity would be frozen
        // by Android's app freezer the moment the operator switches to DJI Fly —
        // the identical failure that cost this project 601 s of telemetry on
        // 2026-07-25 and is the reason BridgeService exists at all.
        BridgeService.start(
            applicationContext, host, port,
            VideoRequest.resolve(
                savedEnabled = prefs.getBoolean(VideoRequest.PREF_ENABLED, CockpitDefaults.GCS_VIDEO),
                savedHost = prefs.getString(VideoRequest.PREF_HOST, null),
                savedPort = prefs.getInt(VideoRequest.PREF_PORT, 0).takeIf { it > 0 },
                gcsHost = host,
            ),
            zenohPlan(),
        )
    }

    /**
     * What the Zenoh transport should do, from the settings.
     *
     * **No `gcsHost` argument, and that is the difference from [videoPlan].** Video follows the
     * telemetry target because the relay carries `:14550` and `:5600` in one process, so they
     * have exactly one correct address between them. A Zenoh bus is a router several consumers
     * attach to independently and has nothing to do with where QGroundControl is; defaulting it
     * to the ground station would dial a laptop on a port nothing listens on.
     */
    private fun zenohPlan(): ZenohSettings.Plan = ZenohSettings.resolve(
        savedEnabled = prefs.getBoolean(ZenohSettings.PREF_ENABLED, CockpitDefaults.ZENOH),
        savedHost = prefs.getString(ZenohSettings.PREF_HOST, null),
        savedPort = prefs.getInt(ZenohSettings.PREF_PORT, 0).takeIf { it > 0 },
        savedPrefix = prefs.getString(ZenohSettings.PREF_PREFIX, null),
        savedVideo = prefs.getBoolean(ZenohSettings.PREF_VIDEO, CockpitDefaults.ZENOH_VIDEO),
        savedDetections = prefs.getBoolean(ZenohSettings.PREF_DETECTIONS, CockpitDefaults.ZENOH_DETECTIONS),
    )

    /** Bridge first, so nothing is still being sent when the log closes. */
    private fun stopBridge() {
        // Stopping the service calls Bridge.stop in its onDestroy.
        BridgeService.stop(applicationContext)
        Recorder.stop()
    }

    private var probeRequested = false
    private var probeStarted = false
    private var sweepRequested = false
    private var sweepStarted = false
    private var cacheStarted = false

    /**
     * Everything that must wait for MSDK registration. `KeyManager` is unusable
     * before it — subscribing earlier silently does nothing, no error and no
     * callback — so this is the single gate for all key work.
     */
    private fun onMsdkState(s: Msdk.State) {
        if (!cacheStarted && s.registered) {
            cacheStarted = true
            StateCache.start()
        }
        // Idempotent, and gated only on registration — not on productConnected, and not on the
        // bridge running. A simulator left over from a previous session is on the aircraft
        // before anybody presses anything here, so the subscription that would notice it has to
        // exist before anybody presses anything here too.
        if (s.registered) Bridge.simulator.observe()
        // Idempotent, and a no-op until the recorder is running, so it is safe to
        // call on every state emission — which is what makes the ordering between
        // "recorder started" and "MSDK registered" stop mattering.
        if (s.registered) Recorder.attachDjiSources()
        if (probeRequested && !probeStarted && s.registered) {
            probeStarted = true
            KeyProbe.run()
        }
        // The sweep reads the MSDK cache, so it needs the aircraft actually
        // connected or almost everything comes back null.
        if (sweepRequested && !sweepStarted && s.registered && s.productConnected) {
            sweepStarted = true
            KeySweep.run()
        }
    }

    /**
     * The one line readable at arm's length, in the order the operator cares about.
     *
     * Deliberately ranked by *severity*, not by layer: commands being live outranks
     * everything, because it is the only state on this screen that lets a ground
     * station move the aircraft, and it is the one an operator can forget. Below
     * that, the aircraft link outranks the MAVLink link — a bridge streaming
     * happily to QGC with no aircraft attached is the failure that has wasted the
     * most time on this project, and it looks entirely healthy in the text dump.
     *
     * Colour is a second channel carrying the same ranking, never the only one:
     * the words say it too, because these are read in sunlight on a phone strapped
     * to a controller, and because roughly one man in twelve cannot rely on the
     * red/green distinction at all.
     */
    private fun renderBanner(s: Msdk.State, b: Bridge.State) {
        // NOTE (simulator, 2026-07-26): `SimulatorNotice.banner()` exists, is tested, and is
        // *not* wired in here — this function is owned by the banner/landscape work and was left
        // alone deliberately. When it lands, the simulator branch belongs at the very top of this
        // `when`, above COMMANDS LIVE: not because a simulator is more dangerous than a live
        // command path (it is far less) but because it is the only state that makes every other
        // line on this screen false. "AIRCRAFT CONNECTED — telemetry only" is not wrong under the
        // simulator; it is the wrong sentence, because it invites the operator to believe the
        // numbers. Until then the loud channels are the button label, the `── simulator ──` block
        // in the status dump, and the repeating STATUSTEXT to QGC.
        // `productConnected` and `fcConnected` answer two different questions, and
        // conflating them has already cost a session. The first is the **RC** on the
        // end of the USB cable; the second is the **aircraft** talking to that RC.
        // An RC alone reports a connected product and streams stick positions, so
        // the app looks alive while every aircraft field is a sentinel — which is
        // exactly what happened on 2026-07-26 09:40, when a whole motion sequence
        // was recorded against an aircraft that was switched off, and the tell was
        // a satellite count of zero rather than anything on this screen.
        val fc = if (s.registered) StateCache.aircraftState().fcConnected else false
        val armed = Bridge.commandInterlock.enabled

        // Above every other reading, simulator included. A simulator makes the numbers on this
        // screen fabricated; a replay makes them *somebody else's afternoon*, and the picture
        // beside this strip is a moving aircraft. There is no state below this that could be
        // more important to say first.
        val (text, colour) = replayName()?.let {
            // `state_armed` when publishing, and that is deliberate rather than alarmist. That
            // colour means "this phone can change something outside itself", which until today
            // only the interlock could. A published replay changes what a ground station in
            // another field and a bus in another building believe, and an operator who has
            // learned one colour should not have to learn a second.
            ReplayPublication.banner(it, replayToMavlinkOn, replayToZenohOn) to getColor(
                if (replayToMavlinkOn || replayToZenohOn) R.color.state_armed
                else R.color.state_replay
            )
        } ?: SimulatorNotice.banner(Bridge.simulator.snapshot().phase)
            // Above COMMANDS LIVE, and not because a simulator is more dangerous —
            // it is far less. It ranks first because it is the only state that makes
            // every other sentence here false: "AIRCRAFT LIVE" is not *wrong* under
            // the simulator, it is the wrong thing to say, because it invites the
            // operator to believe fabricated numbers.
            ?: when {
                armed && fc -> "COMMANDS LIVE — QGC CAN MOVE THE AIRCRAFT" to R.color.state_armed
                // Armed with nothing to command is still armed: the aircraft coming
                // up makes it live with no further act, so this must not look calm.
                armed -> "COMMANDS LIVE — no aircraft yet" to R.color.state_armed
                fc && b.running -> "AIRCRAFT LIVE — telemetry only" to R.color.state_live
                fc -> "AIRCRAFT LIVE — bridge stopped" to R.color.state_partial
                s.productConnected && b.running -> "RC ONLY — aircraft off" to R.color.state_partial
                s.productConnected -> "RC ONLY — bridge stopped" to R.color.state_partial
                s.registered -> "NO RC" to R.color.state_idle
                else -> "MSDK NOT REGISTERED" to R.color.state_idle
            }.let { (t, res) -> t to getColor(res) }

        // The address rides in the strip because it lost its own row, and it earns
        // the space: pointing at the laptop instead of the relay is a mistake this
        // project has actually made, and it is invisible until nothing arrives.
        val where = if (b.running) b.target else gcsHost.ifEmpty { "unset" }
        banner.text = "$text   ·   → $where"
        // Colour goes on the row, not the label, so the interlock switch sits inside
        // the same coloured block: the control and the state it produces read as one
        // object rather than two things to correlate.
        commandRow.setBackgroundColor(colour)
        banner.setTextColor(0xFFFFFFFF.toInt())
    }

    private fun render(
        s: Msdk.State,
        b: Bridge.State,
        video: VideoStatus = VideoStreamer.state.value,
    ) {
        toggle.text = if (b.running) "Stop bridge" else "Start bridge"
        // The interlock is the authority; the switch only ever reflects it. Anything
        // that turns it off elsewhere — stopping the bridge does — must show here,
        // because a switch that reads "on" over a disabled interlock is the one
        // display error that would matter.
        commands.isChecked = Bridge.commandInterlock.enabled
        // Same rule for the video switch, for a smaller reason: the recorder is the authority and
        // the switch reflects it, so a session ending with recording on shows as off next time.
        videoRecord.isChecked = Recorder.videoEnabled
        updateVideoRecordLabel()
        renderVitals()
        // `ReplayAdmission`'s second rule, made visible: the switch is dead while a recording is
        // on screen. Rule one refuses to open a replay over a live command path; without this
        // one it would be a door you walk through and unlock from the inside.
        commands.isEnabled = ReplayAdmission.mayArm(replay != null).allowed
        renderBanner(s, b)
        // The aircraft, as distinct from the RC holding it — see renderBanner.
        val fcLive = if (s.registered) StateCache.aircraftState().fcConnected else false
        val sim = Bridge.simulator.snapshot()
        val (simLabel, simEnabled) = SimulatorNotice.buttonLabel(sim.phase)
        simulator.text = simLabel
        simulator.isEnabled = simEnabled
        status.text = buildString {
            // Above everything, and present only while it is true — unlike every other block
            // here, which is present always. A section that appears is a section that is read;
            // "what you are looking at is not the aircraft" is the one sentence on this screen
            // that must never be scrolled past.
            replay?.let { player ->
                appendLine("── REPLAY — THIS IS NOT THE AIRCRAFT ──")
                appendLine("file:       ${player.name}")
                appendLine(
                    "at:         %.1f s of %.1f s%s"
                        .format(player.positionSeconds, player.durationSeconds, if (player.playing) "" else "  (paused)"),
                )
                appendLine("samples:    ${player.samples.size}")
                appendLine("commands:   REFUSED while replaying")
                appendLine()
            }
            // First, above MSDK, and present in every phase including off. The text block is
            // where an operator goes to check, and a line that appears only when there is a
            // problem cannot be used to confirm there is not one.
            appendLine("── simulator ──")
            appendLine("state:      ${SimulatorNotice.statusLine(sim)}")
            SimulatorNotice.aircraftLine(sim.aircraft)?.let { appendLine("simulated:  $it") }
            appendLine()
            appendLine("── MSDK ──")
            appendLine("init:       ${s.initEvent ?: "…"}")
            appendLine("registered: ${if (s.registered) "yes" else "no"}")
            s.registerError?.let { appendLine("  error:    $it") }
            s.dbProgress?.let { (cur, total) -> appendLine("flysafe db: $cur / $total") }
            appendLine("rc/product: ${if (s.productConnected) "connected" else "not connected"}")
            appendLine("aircraft:   ${if (fcLive) "connected (FC talking)" else "NOT talking"}")
            appendLine("productId:  ${s.productId}")
            appendLine()
            appendLine("── MAVLink ──")
            appendLine("bridge:     ${if (b.running) "running → ${b.target}" else "stopped"}")
            // Bound / REFUSED / LOST / REBOUND, verbatim from Bridge. Present even
            // when the bridge is stopped, because "refused to start: no WiFi" is
            // exactly the line the operator needs then.
            b.wifi?.let { appendLine("wifi:       $it") }
            // The interlock, and — when the simulator is what is holding it — why. A command
            // path that is armed for a reason the operator did not choose must say the reason,
            // and one that is about to go away with the simulator must say that too.
            val armed = Bridge.commandInterlock.enabled
            val why = SimulatorInterlock.interlockLine(sim.phase, armed)
            appendLine("commands:   ${if (armed) "ALLOWED" else "refused"}${why?.let { " ($it)" } ?: ""}")
            appendLine("sent:       ${b.sent}")
            appendLine("received:   ${b.received}")
            b.error?.let { appendLine("error:      $it") }
            appendLine()
            // Below MAVLink and above the recorder: video depends on the link's
            // network and is subordinate to it, which is also the order in which
            // an operator should read a failure.
            //
            // The setting line is separate from the phase, always, because the two
            // fail for different reasons and one of them is invisible to the video
            // path itself. "ON, NOT RUNNING" is the honest reading when the bridge
            // was never started or refused for want of WiFi — states in which
            // VideoStreamer is correctly sitting in STOPPED with nothing to say.
            val plan = videoPlan()
            appendLine("── video ──")
            appendLine("setting:    ${VideoRequest.statusLine(plan, video.phase)}")
            plan.warning?.let { appendLine("WARNING:    $it") }
            // The detail only when there is something to be detailed about: twelve
            // lines of em-dashes for a feature nobody switched on would push the
            // recorder block off a screen an operator reads at arm's length.
            if (plan.enabled || video.phase != VideoPhase.STOPPED) append(video.describe())
            appendLine()
            // Below video and above the recorder. The reading order is the dependency order: the
            // bus needs the link's network, and its failures are read *after* WiFi's and before
            // the recorder's, because "no WiFi" explains a silent bus and a silent bus explains
            // nothing about the recorder.
            val zenoh = zenohPlan()
            val zphase = ZenohBus.phase()
            appendLine("── zenoh ──")
            appendLine("setting:    ${ZenohSettings.statusLine(zenoh, zphase)}")
            zenoh.warning?.let { appendLine("WARNING:    $it") }
            if (zenoh.enabled || zphase != ZenohPublisher.Phase.STOPPED) {
                ZenohBus.counters()?.let { c ->
                    appendLine("published:  ${c.published}")
                    // Both, always, and never summed. Dropped is a full queue — the bus could not
                    // keep up with us. Discarded is no session — there was no bus. One is a
                    // capacity problem and one is a network problem, and a single "lost" number
                    // would send an operator to look at the wrong one.
                    appendLine("dropped:    ${c.dropped}   (queue full)")
                    appendLine("discarded:  ${c.discarded}   (no session)")
                    appendLine("opens:      ${c.opens}   failures: ${c.failures}")
                    c.lastError?.let { appendLine("error:      $it") }
                }
                // **Where drone/world's origin is, and whether it is the takeoff point.** The one
                // line on this screen that changes what every published coordinate means: an
                // origin taken mid-flight measures height above wherever the bridge came up, not
                // above where the aircraft left the ground.
                appendLine(
                    "origin:     " + when (ZenohBus.datumOrigin()) {
                        ZenohTelemetryPump.DatumOrigin.NONE ->
                            "none yet — pose and odom withheld until the motors start"
                        ZenohTelemetryPump.DatumOrigin.MOTORS_ON -> "takeoff point"
                        ZenohTelemetryPump.DatumOrigin.MID_FLIGHT ->
                            "TAKEN IN FLIGHT — not the takeoff point"
                    },
                )
                // What the last sample actually did, per channel, with the encoder's own reason.
                // This is the line that separates "the bus is broken" from "DJI has not sent an
                // altitude for five seconds", which on the reference flight withheld `odom` for
                // 78 % of the samples and is not a fault at all.
                val reasons = ZenohBus.lastReasons()
                if (reasons.isNotEmpty()) {
                    appendLine(
                        "channels:   " + ZenohChannel.PUBLISHED.joinToString("  ") { ch ->
                            "${ch.channel}=${
                                reasons[ch]?.name?.lowercase() ?: "—"
                            }"
                        },
                    )
                }
                // **The video channel, counted separately because it is a separate publisher.**
                // `gops` is the number to read: `frames` counts everything that did not go out and
                // is dominated by frames deliberately skipped while waiting for a keyframe, while
                // a GOP is one time a subscriber's picture broke and had to wait ~4.4 s for the
                // next keyframe to come back.
                ZenohBus.videoCounters()?.let { v ->
                    appendLine(
                        "video:      %d frames / %.1f MB sent, %d seen"
                            .format(v.published, v.bytesPublished / 1e6, v.seen),
                    )
                    appendLine(
                        "video drop: ${v.gopsDropped} GOP(s), ${v.framesDropped} frame(s); " +
                            "${v.framesAwaitingKey} awaiting a keyframe" +
                            if (v.framesRefused > 0) ", ${v.framesRefused} refused" else "",
                    )
                }
            }
            appendLine()
            appendLine("── recorder ──")
            if (Recorder.isRunning) {
                appendLine("file:       ${Recorder.currentFile ?: "…"}")
                appendLine("entries:    ${Recorder.writtenCount}")
                // Dropped entries are shown even when zero: a silent recorder that
                // has been quietly discarding entries is the one failure that would
                // make a post-flight analysis wrong rather than merely incomplete.
                appendLine("dropped:    ${Recorder.dropCount}")
                appendLine("bytes:      ${Recorder.byteCount}")
            } else {
                appendLine("not recording")
            }
            appendLine()
            // **The surface Ivan actually holds.** He flies looking at the phone as often as at
            // QGC, and until 2026-07-30 a DJI warning reached QGC, the flight record, logcat and
            // the Zenoh bus — everything except the screen in his hand. landing17 is the flight
            // that made that a bug rather than an omission: four `LEVEL_2` wind warnings in a
            // 14.2 m/s wind, and the question *"is DJI giving us wind warnings or no?"* asked out
            // loud while the aircraft was answering it four times over.
            //
            // Last **and always**, including when there is nothing to say. A block that only
            // appears when something is wrong is a block whose absence means either "all clear" or
            // "the plumbing is broken", and this package exists because those two looked identical
            // for an hour once.
            appendLine("── DJI warnings ──")
            val standing = Bridge.warnings.snapshot().filter { it.level.forwardable() }
            if (standing.isEmpty()) {
                appendLine("none standing")
            } else {
                // DJI's own state word alongside its words and its number, because the operator's
                // next move may be to search for the state name.
                for (w in standing) {
                    val measured = w.measurement?.let { "  $it" } ?: ""
                    appendLine("${w.source.label}: ${w.name} [${w.state}]$measured")
                }
            }
            Bridge.warnings.lastText?.let { appendLine("last said:  $it") }
        }
    }

    /** The recording on screen, or null. The one place the banner asks about replay. */
    private fun replayName(): String? = replay?.name

    private companion object {
        /**
         * How often the situation picture is repainted, in milliseconds.
         *
         * `Bridge`'s own tick, and four times slower than the guided loop it draws the output
         * of. Nothing on this screen moves faster than the aircraft does, and a picture redrawn
         * faster than telemetry arrives would be spending battery to show the same frame twice.
         */
        const val SITUATION_TICK_MS = 200L

        const val KEY_HOST = "gcs_host"
        const val KEY_PORT = "gcs_port"
        const val EXTRA_HOST = "host"
        const val EXTRA_AUTOSTART = "autostart"
        const val EXTRA_PROBE = "probe"
        const val EXTRA_SWEEP = "sweep"
        const val EXTRA_PORT = "port"
        /** `--es recordHz 25` for a control-tuning session; 5 Hz otherwise. */
        const val EXTRA_RECORD_HZ = "recordHz"

        /** Whether state is sampled at 25 Hz rather than 5. Persisted; see the settings dialog. */
        const val KEY_FAST_RECORD = "fastRecord"
        /** `--es note "…"` — the site and intent, stored in the log header. */
        const val EXTRA_NOTE = "note"

        /** `--es simLat / --es simLon / --ei simSats` — where a bench simulator is seeded. */
        const val EXTRA_SIM_LAT = "simLat"
        const val EXTRA_SIM_LON = "simLon"
        const val EXTRA_SIM_SATS = "simSats"
    }
}
