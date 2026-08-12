package com.dimensional.mini4pro.record

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

/**
 * **The test that makes bypassing the recorder impossible or loud.**
 *
 * Ivan's requirement is one sentence:
 *
 * > *"Recorder should be a general thing that all messages pass through. Structurally, we
 * > shouldn't be able to hook up something to ground control or Zenoh without it passing through a
 * > recording system."*
 *
 * A sentence is not a property. This file is what turns it into one, and it exists because the
 * project already had the proof that convention is not enough: **the gimbal aimed a real camera
 * for weeks and not one gimbal reading existed in any flight record**
 * (`replay/ReplayCoverage.GIMBAL`). Nobody decided that. It is simply what happens when recording
 * is something you remember to call.
 *
 * ## Two mechanisms, because one of them cannot cover the other's half
 *
 * **Impossible.** Where a type can carry the rule, it does, and these tests only pin it:
 * `MavlinkLink` takes a non-null `Tap` in its constructor and records both directions itself; the
 * production `ActionPort` and `SimulatorPort` classes are file-private and their only factories
 * *require* a `Tap`. There is no `KeyManagerActionPort()` to call. The compiler refuses.
 *
 * **Loud.** The remaining hole cannot be closed by a type, because the thing that goes wrong is an
 * *absence*: someone writes a **new** file that opens a socket or calls `KeyManager`, wires it up,
 * and never declares it anywhere. No signature is missing, so nothing fails to compile. So
 * [everyFileThatTouchesAWireIsDeclared] walks the actual source tree, finds every file that
 * touches a wire, and fails when one appears that [DECLARED] does not account for. That is the
 * test that fails when a new unrecorded path is added, and it is the one to run a mutation against
 * first.
 *
 * ## Why a source scan rather than reflection
 *
 * Because the classes that matter cannot be loaded. `MavlinkLink` imports `android.util.Log`,
 * every `KeyManager*Port` imports `dji.*`, and neither Android nor the MSDK is on the unit-test
 * runtime classpath (`docs/architecture.md`) — that is the whole reason these seams exist. A
 * reflective test would be unable to see exactly the classes it is supposed to police. The text of
 * the source is the only evidence available, and it is sufficient: what is being asserted is a
 * structural fact about how things are written, not a runtime behaviour.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time, applied to the shipped source, run,
 * confirmed red and reverted. Counts are failing tests across the four seam suites
 * (`RecordingSeamTest`, `DjiCallsTest`, `GimbalSampleTest`, `command/RecordedPortsTest`) —
 * **measured, not estimated**.
 *
 * The seam itself:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | a new source file that calls `KeyManager.getInstance()` and is not declared | 1 |
 *  | `MavlinkLink.tap` given a default (an omittable tap) | 1 |
 *  | `MavlinkLink` stops tapping inbound (`tap.gcsIn` removed) | 1 |
 *  | `MavlinkLink` stops tapping outbound (`tap.gcsOut` removed) | 1 |
 *  | `KeyManagerActionPort` made public again | 1 |
 *  | `actionPort` returns the raw port instead of the decorator | 1 |
 *  | `simulatorPort` returns the raw port instead of the decorator | 1 |
 *  | two channels made to share one entry kind | 2 |
 *  | `Bridge` records inbound MAVLink itself again (every message written twice) | 1 |
 *
 * The call record ([DjiCalls]):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `begin` stops recording the ask | 25 |
 *  | the answer is recorded but drops the correlating `seq` | 5 |
 *  | `sweep` made a no-op (a swallowed callback stays an absence) | 4 |
 *  | `urgentFor` returns `askWasUrgent` alone (a refusal is not fsynced) | 3 |
 *  | the outstanding-call cap removed (unbounded growth, no `overflow` line) | 1 |
 *  | `RecordedActionPort` swallows the caller's `onFailure` | 1 |
 *  | `RecordedActionPort` records the ask *after* the SDK call | 1 |
 *  | `RecordedSimulatorPort` drops the origin from the ask | 1 |
 *
 * The gimbal ([GimbalSample], and the deadband `Recorder.sampleGimbal` applies):
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | deadband applied to pitch only | 3 |
 *  | an axis appearing or disappearing treated as no move | 2 |
 *  | a gimbal stuck at `NaN` reports a move on every sample | 1 |
 *  | the deadband widened to 5° | 3 |
 *
 * **Containment, and the one honest zero.** *"A recorder fault must never reach the caller"* is
 * held four-deep — `begin`, `Handle.finish`, `emit`, and `sweep` each contain their own — so
 * removing any single layer leaves the property standing and kills nothing, which is the correct
 * result and is reported rather than hidden:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | containment removed from `emit` alone | 0 — absorbed by `begin` and `finish` |
 *  | containment removed from `Handle.finish` alone | 0 — absorbed by `emit` |
 *  | containment removed from `begin` alone | 2 |
 *  | **containment removed from all four layers** | **3** |
 *
 * Two mutations are deliberately absent because they are **compile-time** properties and a test
 * count would misrepresent them: giving `Channel.entryKind` an `else ->` branch, and constructing
 * a `MavlinkLink` or a production DJI port without a tap. Those do not fail a test; they fail the
 * build, which is the stronger outcome and the whole reason the seam is shaped this way.
 */
class RecordingSeamTest {

    // ─────────────────────────────────────────────────────────────────────────
    // the manifest
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One source file that talks to something outside this app, and how its traffic reaches the
     * flight record.
     *
     * @param path relative to `src/main/java`.
     * @param channel the [Channel] it carries, or null when the file crosses no boundary that
     *   produces traffic — a socket *factory*, a probe run by hand, a manager teardown.
     * @param why one line, so the manifest reads as an argument rather than an allow-list. This is
     *   the field that makes adding an entry a decision instead of a paste.
     */
    private data class Declared(val path: String, val channel: Channel?, val why: String)

    /**
     * **Every file in `src/main/java` that touches a socket or the MSDK**, and what records it.
     *
     * Adding a file that trips [MARKERS] and not adding it here fails
     * [everyFileThatTouchesAWireIsDeclared]. That is the point: the failure message asks the one
     * question the gimbal never got asked — *what puts this on the record?*
     */
    private val DECLARED = listOf(
        // ── ground station ──────────────────────────────────────────────────
        Declared(
            "com/dimensional/mini4pro/mavlink/MavlinkLink.kt", Channel.GCS_MAVLINK_OUT,
            "records every datagram itself, at the socket, through the non-null Tap it is " +
                "constructed with. Also carries GCS_MAVLINK_IN.",
        ),
        Declared(
            "com/dimensional/mini4pro/mavlink/DatagramStreams.kt", null,
            "the byte plumbing under MavlinkLink. Its outbound stream invokes the onDatagram " +
                "callback MavlinkLink wires to the tap; it holds no socket of its own and " +
                "reaches no wire MavlinkLink has not already tapped.",
        ),
        Declared(
            "com/dimensional/mini4pro/mavlink/WifiBind.kt", null,
            "a DatagramSocket *factory* handed to MavlinkLink. It builds and binds; it never " +
                "sends, so there is no traffic here to record.",
        ),

        // ── video ───────────────────────────────────────────────────────────
        Declared(
            "com/dimensional/mini4pro/video/RtpVideoSink.kt", Channel.GCS_VIDEO,
            "the RTP socket. Its *payload* is deliberately not recorded — 5 Mbit/s of H.264 " +
                "does not belong in a 32 MB flight record — and its lifecycle is, through " +
                "VideoStreamer.eventSink.",
        ),
        Declared(
            "com/dimensional/mini4pro/video/VideoStreamer.kt", Channel.GCS_VIDEO,
            "owns the passthrough's phase trail and failures and pushes them at eventSink, " +
                "which Bridge points at Recorder.event. KNOWN WEAKNESS: eventSink is a nullable " +
                "var, so this is still a convention rather than a seam — see the report.",
        ),
        Declared(
            "com/dimensional/mini4pro/video/CameraStreamTap.kt", null,
            "receives frames *from* the aircraft's camera into VideoStreamer. Inbound bulk " +
                "media, never recorded, for the same reason RtpVideoSink's payload is not.",
        ),

        // ── the camera, decoded ─────────────────────────────────────────────
        Declared(
            "com/dimensional/mini4pro/vision/MsdkFrameSource.kt", Channel.AIRCRAFT_TAG,
            "the decoded-frame tap (addFrameListener). The *frames* are bulk media and are never " +
                "recorded, exactly as CameraStreamTap's are not — what reaches the record is what " +
                "the detector makes of them, one LogEntry.Tag per detection, through the " +
                "RecordedTagSink the TagRecogniser is constructed with.",
        ),

        // ── aircraft, outbound ──────────────────────────────────────────────
        Declared(
            "com/dimensional/mini4pro/KeyManagerActionPort.kt", Channel.AIRCRAFT_ACTION,
            "the production class is private to this file; the only factory, actionPort(tap), " +
                "returns it wrapped in RecordedActionPort. An unrecorded one cannot be built.",
        ),
        Declared(
            "com/dimensional/mini4pro/simulator/KeyManagerSimulatorPort.kt", Channel.AIRCRAFT_ACTION,
            "same shape: private class, simulatorPort(tap) is the only factory, and it wraps.",
        ),
        Declared(
            "com/dimensional/mini4pro/gimbal/KeyManagerGimbalPort.kt", Channel.AIRCRAFT_ACTION,
            "wrapped by RecordedGimbalPort at the Bridge (2026-07-27). Until then this entry " +
                "read NOT YET WRAPPED and it was telling the truth: DjiOp.GIMBAL_ROTATE existed " +
                "and nothing ever wrote it, so no record carried a commanded camera angle.",
        ),
        Declared(
            "com/dimensional/mini4pro/light/KeyManagerLightPort.kt", Channel.AIRCRAFT_ACTION,
            "wrapped by RecordedLightPort at the Bridge. The lamp is recorded because the " +
                "reason it exists is a question — does it help a camera see a tag after dark? — " +
                "and that is answered only by comparing detections against when it was on.",
        ),
        Declared(
            "com/dimensional/mini4pro/guided/KeyManagerVirtualStickPort.kt", Channel.AIRCRAFT_STICK,
            "the 25 Hz setpoint stream is already recorded in full by LogEntry.StickCmd through " +
                "GuidedRecord, which is why it is deliberately NOT re-wrapped per tick. Its " +
                "three discrete asks (enable/disable/advanced-mode) are not yet on the call " +
                "record; `guided/` is owned by another agent — see the report.",
        ),

        // ── aircraft, inbound ───────────────────────────────────────────────
        Declared(
            "com/dimensional/mini4pro/StateCache.kt", Channel.AIRCRAFT_STATE,
            "the read side of the MSDK. Recorder samples it at its own rate into dji_state and " +
                "dji_field; it commands nothing.",
        ),
        Declared(
            "com/dimensional/mini4pro/record/Recorder.kt", Channel.AIRCRAFT_FIELD,
            "the recorder's own DJI subscriptions — the keys a post-mortem needs that " +
                "AircraftState does not carry. It is the tap; it cannot bypass itself.",
        ),

        // ── neither ─────────────────────────────────────────────────────────
        // `Bridge.kt` is deliberately absent: the wiring layer holds no wire of its own. It hands
        // `Recorder` to `MavlinkLink` and to the two port factories and touches nothing directly,
        // which is why it trips no marker. What it must *not* do is covered separately by
        // `noWiringLayerRecordsMavlinkTrafficItself`.
        Declared(
            "com/dimensional/mini4pro/KeyProbe.kt", null,
            "a hand-run diagnostic that reads keys and prints. Not on any flight path.",
        ),
        Declared(
            "com/dimensional/mini4pro/KeySweep.kt", null,
            "as KeyProbe: a hand-run read-only sweep of the key surface.",
        ),
    )

    /**
     * What counts as touching a wire.
     *
     * Kept blunt on purpose. A precise marker set would be a place to hide: the value of this list
     * is that it over-triggers, so a file that merely *looks* like a transport still has to be
     * declared and dismissed in writing. Every entry in [DECLARED] with a null channel is such a
     * dismissal.
     *
     * Matched against [code], not against the file: a KDoc that *documents* `KeyManager` — which
     * several seam interfaces do at length, correctly — is not a transport.
     */
    private val MARKERS = listOf(
        "KeyManager.getInstance()",
        "VirtualStickManager.getInstance()",
        "MediaDataCenter.getInstance()",
        "DatagramSocket",
        "MavlinkConnection.create",
    )

    // ─────────────────────────────────────────────────────────────────────────
    // the tests
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * **The one that fails when a new unrecorded path is added.**
     *
     * Walks `src/main/java`, collects every file containing one of [MARKERS], and compares that
     * set with [DECLARED]. A new transport is a red test with a message naming the file and asking
     * what records it; a *deleted* transport is equally red, so the manifest cannot rot into a
     * list of files that no longer exist.
     */
    @Test
    fun everyFileThatTouchesAWireIsDeclared() {
        val found = sourceFiles()
            .filter { (_, text) -> MARKERS.any { text.contains(it) } }
            .map { (path, _) -> path }
            .toSortedSet()
        val declared = DECLARED.map { it.path }.toSortedSet()

        val undeclared = found - declared
        assertTrue(
            "These source files touch a socket or the MSDK and are not declared in " +
                "RecordingSeamTest.DECLARED:\n" +
                undeclared.joinToString("\n") { "  $it" } +
                "\n\nA file that can reach a wire must say what puts its traffic on the flight " +
                "record. Add it to DECLARED with its Channel and one line of why — or with a " +
                "null Channel and one line of why it carries no traffic. This is the check that " +
                "the gimbal never had: it aimed a real camera for weeks and no record of it " +
                "existed anywhere.",
            undeclared.isEmpty(),
        )

        val stale = declared - found
        assertTrue(
            "DECLARED names files that no longer touch a wire (or no longer exist):\n" +
                stale.joinToString("\n") { "  $it" } +
                "\n\nRemove them, so the manifest stays an argument rather than an archive.",
            stale.isEmpty(),
        )
    }

    /** Every declaration carries a reason. An allow-list without reasons decays into a paste. */
    @Test
    fun everyDeclarationSaysWhy() {
        for (d in DECLARED) {
            assertTrue("${d.path} declares no reason", d.why.length > 30)
        }
        assertEquals(
            "DECLARED has a duplicate path",
            DECLARED.size, DECLARED.map { it.path }.toSet().size,
        )
    }

    /**
     * `MavlinkLink` cannot be built without a tap, and taps both directions itself.
     *
     * Pins the change from the old shape — a public `var onSent` that `Bridge` remembered to
     * install a recorder into, and a `Recorder.mavIn` at the top of `Bridge.onInbound` that
     * nothing obliged anyone to keep. Both were conventions. Recording now happens on the way
     * past, inside the class that owns the wire.
     */
    @Test
    fun theMavlinkLinkRecordsBothDirectionsItself() {
        val text = read("com/dimensional/mini4pro/mavlink/MavlinkLink.kt")

        assertTrue(
            "MavlinkLink must take a non-null, non-defaulted Tap: a link that records nothing " +
                "must not be constructible.",
            text.contains(Regex("""private val tap: Tap\s*,""")),
        )
        assertFalse(
            "MavlinkLink's tap must not be nullable — a null tap is a link that records nothing.",
            text.contains("val tap: Tap?"),
        )
        assertFalse(
            "MavlinkLink's tap must not have a default — a default is the bypass.",
            text.contains(Regex("""val tap: Tap\s*=""")),
        )
        assertTrue(
            "MavlinkLink must tap every outbound datagram at the socket.",
            text.contains("tap.gcsOut("),
        )
        assertTrue(
            "MavlinkLink must tap every inbound message before routing it.",
            text.contains("tap.gcsIn("),
        )
        assertFalse(
            "The `var onSent` hook is gone on purpose: an installable tap is an omittable tap.",
            text.contains("var onSent"),
        )
        // Inbound must be tapped *before* onMessage, so what the GCS asked for survives a router
        // that throws.
        assertTrue(
            "tap.gcsIn must run before onMessage",
            text.indexOf("tap.gcsIn(") < text.indexOf("onMessage(message)"),
        )
    }

    /**
     * The production DJI ports are unreachable without a tap.
     *
     * `private class` at file scope is what makes this structural rather than advisory: there is
     * no `KeyManagerActionPort()` for `Bridge` to call, so the *only* way to get one is the
     * factory, and the factory's signature requires a `Tap`.
     */
    @Test
    fun theProductionDjiPortsAreUnreachableWithoutATap() {
        val cases = listOf(
            Triple(
                "com/dimensional/mini4pro/KeyManagerActionPort.kt",
                "KeyManagerActionPort", "RecordedActionPort",
            ),
            Triple(
                "com/dimensional/mini4pro/simulator/KeyManagerSimulatorPort.kt",
                "KeyManagerSimulatorPort", "RecordedSimulatorPort",
            ),
        )
        for ((path, className, decorator) in cases) {
            val text = read(path)
            assertTrue(
                "$className must be private to its file, so nothing outside can construct an " +
                    "unrecorded port. The compiler is the enforcement; this only pins it.",
                text.contains("private class $className"),
            )
            // The factory must *construct the decorator around the raw port*, not merely mention
            // it. `text.contains("Recorded")` was the first version of this assertion and a
            // mutation walked straight past it: an import survives a factory that returns the raw
            // port, and a factory that returns the raw port is the whole bypass.
            assertTrue(
                "$path's factory must return $decorator wrapping $className(). Returning the raw " +
                    "port is exactly the bypass this seam exists to prevent, and it type-checks.",
                text.contains(Regex("""$decorator\(\s*$className\(\)\s*,\s*tap\s*,?\s*\)""")),
            )
        }

        // And nothing anywhere else constructs one, which would only be possible from inside the
        // same file but is worth stating so a later `internal` demotion is caught.
        val constructors = sourceFiles().filter { (path, text) ->
            !path.endsWith("KeyManagerActionPort.kt") &&
                !path.endsWith("KeyManagerSimulatorPort.kt") &&
                (text.contains("KeyManagerActionPort(") || text.contains("KeyManagerSimulatorPort("))
        }.map { it.first }
        assertEquals(
            "These files construct a raw DJI port directly: $constructors",
            emptyList<String>(), constructors,
        )
    }

    /**
     * The wiring layer no longer records MAVLink traffic itself.
     *
     * Two properties in one assertion. It is the **no-behaviour-change** guard — with the link
     * tapping inbound, a surviving `Recorder.mavIn` in `Bridge` would write every inbound message
     * twice — and it is the structural point: the router is not where recording lives any more, so
     * a new router cannot forget.
     */
    @Test
    fun noWiringLayerRecordsMavlinkTrafficItself() {
        val bridge = read("com/dimensional/mini4pro/Bridge.kt")
        assertFalse(
            "Bridge must not record inbound MAVLink: MavlinkLink already does, and a second " +
                "call writes every message twice.",
            bridge.contains("Recorder.mavIn("),
        )
        assertFalse(
            "Bridge must not record outbound MAVLink: the socket tap in MavlinkLink already does.",
            bridge.contains("Recorder.mavOutWire("),
        )
        assertTrue(
            "Bridge must hand the recorder to the link as its tap",
            bridge.contains("tap = Recorder"),
        )
    }

    /**
     * The video phase trail reaches the recorder — **the one channel still held by a convention,
     * pinned rather than fixed.**
     *
     * `VideoStreamer.eventSink` is a nullable `var` that `Bridge` sets, which is exactly the shape
     * this seam replaced everywhere else. It is *not* converted to a constructor tap, and the
     * reason is a real tension rather than an oversight: `video/` deliberately types the sink as
     * primitives so the package imports nothing from `record/`, and a non-null default would have
     * to be `Recorder.event`. Trading a stated package boundary for a nullable field is not
     * obviously the better deal, and `VideoStreamer` is an `object` with no constructor to put a
     * tap in.
     *
     * So the hole is closed the loud way instead: if `Bridge` ever stops wiring the sink, or
     * `VideoStreamer` ever stops having one, this fails. `Channel.GCS_VIDEO` is the declaration
     * that keeps it visible in the registry.
     */
    @Test
    fun theVideoPhaseTrailIsWiredToTheRecorder() {
        val bridge = read("com/dimensional/mini4pro/Bridge.kt")
        assertTrue(
            "Bridge must wire VideoStreamer's phase trail into the flight recorder. Video is the " +
                "one subsystem whose complete failure is indistinguishable from 'the operator " +
                "did not turn it on'.",
            bridge.contains("VideoStreamer.eventSink = "),
        )
        assertTrue(
            "and the thing it wires must be the recorder",
            bridge.contains("Recorder.event("),
        )
        assertTrue(
            "VideoStreamer must still have a sink to wire",
            read("com/dimensional/mini4pro/video/VideoStreamer.kt").contains("var eventSink"),
        )
    }

    /**
     * Every [Channel] declares a distinct entry kind, and every kind is one `LogEntry` really
     * produces.
     *
     * The exhaustive `when` in `Channel.entryKind` is what makes a *new* channel a compile error
     * until it names its record line. This test covers the two things a `when` cannot: that no two
     * channels quietly share a kind (which would make one of them unreadable in the file), and
     * that every kind names a real entry rather than a string somebody invented.
     */
    @Test
    fun everyChannelDeclaresAnEntryKindThatSomethingProduces() {
        val realKinds = setOf(
            LogEntry.KIND_MAV_IN, LogEntry.KIND_MAV_OUT, LogEntry.KIND_DJI_STATE,
            LogEntry.KIND_DJI_FIELD, LogEntry.KIND_DJI_WARN, LogEntry.KIND_GIMBAL,
            LogEntry.KIND_DJI_CALL, LogEntry.KIND_STICK_CMD, LogEntry.KIND_EVENT,
            LogEntry.KIND_FRAME, LogEntry.KIND_TAG,
        )
        for (channel in Channel.entries) {
            assertTrue(
                "$channel declares the kind '${channel.entryKind}', which no LogEntry produces",
                channel.entryKind in realKinds,
            )
        }

        // Two channels may share a kind only when they are genuinely the same line — the two
        // video ones are both `event`. Everything else must be distinguishable in the file.
        val byKind = Channel.entries.groupBy { it.entryKind }
        for ((kind, channels) in byKind) {
            if (channels.size == 1) continue
            assertTrue(
                "Channels $channels all write '$kind' and would be indistinguishable in the " +
                    "record. Only the video channels may share one.",
                channels.all { it == Channel.GCS_VIDEO },
            )
        }

        // The seam's own verbs must cover every channel a *transport class* crosses.
        assertEquals(
            listOf(
                Channel.GCS_MAVLINK_OUT, Channel.GCS_MAVLINK_IN, Channel.AIRCRAFT_ACTION,
                Channel.AIRCRAFT_TAG,
            ),
            Channel.TAPPED,
        )
        for (c in Channel.TAPPED) assertTrue("$c must be outward-facing", c.peer in Peer.entries)
    }

    /**
     * The gimbal — the gap this whole seam was built around — is now on the record.
     *
     * Asserted end to end through the pieces a JVM test can reach: a sample that moved past the
     * deadband produces a `gimbal` entry, and that entry renders the three axes and the age.
     * `Recorder.sampleGimbal` itself needs Android; what it decides is
     * `GimbalSample.movedFrom`, which is pinned next door in `GimbalSampleTest`.
     */
    @Test
    fun theGimbalChannelProducesARecordLine() {
        val entry = LogEntry.Gimbal(
            monoNanos = 0L, pitchDeg = -30.25, rollDeg = 0.5, yawDeg = 118.0, ageMs = 140L,
        )
        assertEquals(Channel.AIRCRAFT_GIMBAL.entryKind, entry.kind)
        assertFalse(
            "A gimbal line must not be urgent: it can arrive at 5 Hz and an fsync per line " +
                "would put a flash sync on a stream.",
            entry.urgent,
        )
        val json = JsonObject.render { entry.writeBody(it) }
        assertEquals("""{"p":-30.25,"r":0.5,"y":118,"age":140}""", json)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // source access
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * One file's source **with every comment removed**.
     *
     * Load-bearing, and learned the hard way the first time this test ran: `MavlinkLink`'s own
     * KDoc explains that it no longer exposes `var onSent`, `Bridge`'s explains that it no longer
     * calls `Recorder.mavIn`, and `SimulatorPort`'s documents the `KeyManager.getInstance()` calls
     * DJI's own `SimulatorManager` makes. All three would trip a naive text search, and a
     * structural test that a class cannot be *described* is worse than no test: the cure is to
     * delete the explanation, which is precisely the wrong incentive in a codebase whose KDoc is
     * where the reasoning lives.
     *
     * So every assertion here reads code. Not a parser — a stripper: line comments, block
     * comments, and nothing clever. `"//"` inside a string literal would be mis-stripped, and
     * nothing in this project has one on a line that matters.
     */
    private fun code(text: String): String {
        val out = StringBuilder(text.length)
        var i = 0
        while (i < text.length) {
            when {
                text.startsWith("//", i) -> {
                    val end = text.indexOf('\n', i)
                    i = if (end < 0) text.length else end
                }
                text.startsWith("/*", i) -> {
                    val end = text.indexOf("*/", i + 2)
                    i = if (end < 0) text.length else end + 2
                }
                else -> {
                    out.append(text[i])
                    i++
                }
            }
        }
        return out.toString()
    }

    /** One file's code, comments stripped. */
    private fun read(relative: String): String {
        val f = File(sourceRoot, relative)
        assertTrue("missing source file: $relative", f.isFile)
        return code(f.readText())
    }

    /** Every `.kt` under `src/main/java`, as (path relative to the root, code with no comments). */
    private fun sourceFiles(): List<Pair<String, String>> =
        sourceRoot.walkTopDown()
            .filter { it.isFile && it.extension == "kt" }
            .map {
                it.relativeTo(sourceRoot).path.replace(File.separatorChar, '/') to code(it.readText())
            }
            .toList()

    /**
     * `src/main/java`, found by walking up from the test's working directory.
     *
     * Gradle runs unit tests with the module directory as the working directory, but that is a
     * default rather than a guarantee, and this test is worthless if it silently finds no files —
     * an empty scan would pass [everyFileThatTouchesAWireIsDeclared] for the wrong reason. So the
     * walk is bounded, the result is asserted to exist, and the file count is asserted to be
     * plausible.
     */
    private val sourceRoot: File by lazy {
        var dir: File? = File(".").absoluteFile
        var found: File? = null
        repeat(6) {
            val candidate = dir?.let { File(it, "src/main/java") }
            if (found == null && candidate != null && candidate.isDirectory) found = candidate
            dir = dir?.parentFile
        }
        val root = requireNotNull(found) {
            "could not locate src/main/java from ${File(".").absolutePath}"
        }
        val count = root.walkTopDown().count { it.isFile && it.extension == "kt" }
        assertTrue(
            "found only $count Kotlin sources under $root — the scan must not pass by finding " +
                "nothing",
            count > 50,
        )
        root
    }
}
