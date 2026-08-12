package com.dimensional.mini4pro.sim

import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.mavlink.DatagramInputStream
import com.dimensional.mini4pro.mavlink.DatagramOutputStream
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.TelemetryEncoder
import io.dronefleet.mavlink.MavlinkConnection
import java.net.DatagramSocket
import java.net.InetSocketAddress
import kotlin.concurrent.thread

/**
 * A synthetic Mini 4 Pro, on the laptop, with no phone and no aircraft.
 *
 * Exists because [TelemetryEncoder] is deliberately free of DJI *and* Android
 * imports and `mavlink/DatagramStreams.kt` is pure `java.net`, so the exact bytes
 * the real bridge would put on the wire can be produced from a JVM `main()` and
 * pointed at a real QGroundControl. That is the only way to *see* what QGC prints
 * for a given `custom_mode` rather than reasoning about its source.
 *
 * **What this has confirmed, as of 2026-07-25 20:05:** all 11 distinct PX4 targets
 * in `Px4Mode`, swept 13 modes at 2.5 s apart against a real QGroundControl and
 * read off the toolbar — `docs/measurements/2026-07-25-qgc-mode-sweep.md`. Eleven
 * printed the intended word; two printed "Unknown", and chasing those down proved
 * they were **QGC 5.0.8's gaps rather than our encoding** (no `POSCTL_SLOW` in
 * that version's enum at all; `AUTO_FOLLOW_TARGET` present but never wired to its
 * own declared string). One mapping row changed as a result.
 *
 * The sweep is only valid with **nothing else on the wire** — unplug the phone and
 * stop the app first. An earlier attempt was discarded because a second heartbeat
 * appeared on sysid 1 partway through and the frames could not be attributed to us.
 *
 * What it still cannot tell you is *which* DJI mode the aircraft reports when. That
 * needs hardware, and only `GPS_ATTI` and `APAS` have ever been seen from it.
 *
 * **This is a diagnostic, not a test.** It is in the test source set because that
 * is where a JVM `main` can see the app's classes without shipping in the APK. It
 * has no assertions and JUnit never runs it (no `@Test`).
 *
 * Run it (from the repo root, inside `nix develop`):
 * ```
 * A=android/app/build; K=$A/tmp/kotlin-classes
 * M=~/.gradle/caches/modules-2/files-2.1/io.dronefleet.mavlink
 * java -cp "$K/debug:$K/debugUnitTest:$(find $M -name '*.jar' | paste -sd:):<kotlin-stdlib.jar>" \
 *   com.dimensional.mini4pro.sim.FakeVehicleKt
 * ```
 * then type DJI `FCFlightMode` names on stdin (`APAS`, `GO_HOME`, `FARMING`, …)
 * and watch QGC's toolbar. `q` quits. Argument 1 overrides the GCS host, 2 the
 * port (QGC binds 14550 exclusively, so use another port if `mavverify` is also
 * listening).
 *
 * It emits the real [TelemetryEncoder] output and answers the connect sequence
 * with the real [HandshakeResponder], so QGC connects properly rather than
 * sitting in "waiting for vehicle setup".
 */
fun main(args: Array<String>) {
    val host = args.getOrNull(0) ?: "127.0.0.1"
    val port = args.getOrNull(1)?.toInt() ?: 14550

    // The measured ground fixture, so the aircraft appears where it really was
    // and QGC has a plottable position: docs/measurements/2026-07-25-ground-probe.md.
    // Held in an AtomicReference because stdin, the tick thread and the rx thread
    // all touch it.
    val state = java.util.concurrent.atomic.AtomicReference(
        AircraftState(
            fcConnected = true,
            latitude = 37.9938612,
            longitude = 23.7253298,
            relativeAltitude = 0.0,
            takeoffAltitudeAmsl = 103.1696,
            rollDeg = -1.0,
            pitchDeg = 0.0,
            yawDeg = -121.1,
            velocityNorth = 0.0,
            velocityEast = 0.0,
            velocityDown = 0.0,
            satelliteCount = 14,
            gpsSignalLevel = 5,
            homeLatitude = 37.9938872,
            homeLongitude = 23.7253295,
            isFlying = false,
            motorsOn = false,
            flightMode = "APAS",
            notAllowMotorStart = false,
            imuWarmingUp = false,
            inFailsafe = false,
            batteryPercent = 98,
            voltageMv = 8371,
            currentMa = -905,
            cellCount = 2,
            cellVoltagesMv = listOf(4186, 4183),
            batteryTempC = 37.5,
        ),
    )

    val socket = DatagramSocket(0)
    var target: InetSocketAddress? = InetSocketAddress(host, port)
    val out = DatagramOutputStream(socket, { target })
    val input = DatagramInputStream(socket) { peer -> target = peer }
    val connection = MavlinkConnection.create(input, out)

    val send: (Any) -> Unit = { payload ->
        synchronized(connection) { connection.send2(1, 1, payload) }
    }
    val responder = HandshakeResponder(send = send, log = { println("[handshake] $it") })
    responder.registerMessageProvider(TelemetryEncoder.MESSAGE_ID_EXTENDED_SYS_STATE) {
        TelemetryEncoder.extendedSysState(state.get())
    }

    println("fake vehicle -> $host:$port (local ${socket.localPort})")
    println("type an FCFlightMode name to change mode, 'q' to quit")

    thread(isDaemon = true, name = "rx") {
        while (true) {
            val message = connection.next() ?: break
            responder.onMessage(message)
            // Mode requests are the interesting inbound traffic: QGC's RTL/Land
            // buttons are DO_SET_MODE followed by watching flightMode() change.
            responder.requestedModes.lastOrNull()?.let {
                println("[mode request] base=${it.baseMode} custom=${it.customMode}")
            }
        }
    }

    thread(isDaemon = true, name = "tick") {
        val start = System.currentTimeMillis()
        while (true) {
            val bootMs = System.currentTimeMillis() - start
            val s = state.get()
            for (m in TelemetryEncoder.periodicMessages(s, bootMs)) send(m)
            for (m in TelemetryEncoder.eventMessages(s)) send(m)
            Thread.sleep(200)
        }
    }

    while (true) {
        val line = readlnOrNull()?.trim() ?: break
        when {
            line.isEmpty() -> {}
            line == "q" -> break
            line == "fly" -> {
                state.set(state.get().copy(isFlying = true, motorsOn = true, relativeAltitude = 30.0))
                println("airborne")
            }
            line == "land" -> {
                state.set(state.get().copy(isFlying = false, motorsOn = false, relativeAltitude = 0.0))
                println("on ground")
            }
            else -> {
                val s = state.get().copy(flightMode = line)
                state.set(s)
                val hb = TelemetryEncoder.heartbeat(s)
                println("mode=$line custom_mode=${hb.customMode()} base_mode=${hb.baseMode().value()}")
            }
        }
    }
    socket.close()
}
