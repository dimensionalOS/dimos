package com.dimensional.mini4pro.replay

import com.dimensional.mini4pro.command.Announcer
import com.dimensional.mini4pro.command.CommandDispatcher
import com.dimensional.mini4pro.command.CommandInterlock
import com.dimensional.mini4pro.command.FlightAction
import com.dimensional.mini4pro.command.FlightActions
import com.dimensional.mini4pro.command.ActionOutcome
import com.dimensional.mini4pro.handshake.HandshakeResponder
import com.dimensional.mini4pro.mavlink.StatusTextSink
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.StateSource
import io.dronefleet.mavlink.common.CommandAck
import io.dronefleet.mavlink.common.CommandLong
import io.dronefleet.mavlink.common.MavCmd
import io.dronefleet.mavlink.common.MavMode
import io.dronefleet.mavlink.common.MavResult
import io.dronefleet.mavlink.common.SetMode
import io.dronefleet.mavlink.util.EnumValue
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * **Publishing a recording: the rules, the seam, and the chain that keeps commands out.**
 *
 * A drawn replay is contained by the screen — the watermark, the strip and the banner are all in
 * front of the person who opened it, and `SituationHonestyTest` is the argument for why nothing it
 * produces can command anything. A **published** replay is not contained: it puts a moving
 * aircraft on QGroundControl's map and on DiMOS's bus, for people who did not open anything and
 * cannot see the phone. Everything in this file exists because of that difference.
 *
 * ## 1. The new rule: two flights never share one stream
 *
 * [ReplayAdmission.mayPublish] refuses while an aircraft is connected. Not because a replay is
 * dangerous to an aircraft — rules 1 and 2 already keep it out of the command path — but because
 * two sources on one downstream produce a stream that is **fabricated** rather than merely
 * degraded: one aircraft's position interleaved with another's at 5 Hz, same system id, same key
 * expressions, with nothing in any message saying which is which.
 *
 * ## 2. The seam: the command path cannot see a recording, by construction
 *
 * `telemetry/StateSource` splits the fan-out that `Bridge.aircraftState()` used to be. [read] is
 * switchable and feeds MAVLink and Zenoh; [StateSource.liveState] is not switchable, has no
 * parameter, and is what `guided/`, `mission/`, the dispatcher and the tag detector read.
 * [theCommandPathsReadCannotBeSwitchedToARecording] drives that with a feed installed on both
 * sinks and asserts the live read is unmoved — the structural half, in the mould of
 * `SituationHonestyTest`'s reflection over `Situation`.
 *
 * ## 3. The chain: an inbound command arriving mid-replay is refused
 *
 * This is the one the brief asked to be asserted rather than assumed, and the chain is three
 * links long: a replay on screen makes [ReplayAdmission.mayArm] refuse, so the interlock stays
 * off, so [CommandDispatcher] answers `MAV_RESULT_UNSUPPORTED`. Each link is tested in isolation
 * elsewhere; what is tested here is that they compose, against the **real** dispatcher and the
 * real responder rather than a description of them. MAVLink-out makes this concrete rather than
 * hypothetical: QGC now sees a flying aircraft and its buttons light up.
 *
 * ## Mutations killed, measured 2026-07-28
 *
 * | mutation | failures |
 * |---|---|
 * | `mayPublish` allows publishing while an aircraft is connected | 3 |
 * | `mayPublish` refuses a connected aircraft with no sentence | 3 |
 * | `mayPublish` ignores the interlock | 2 |
 * | `mayPublish` ignores an engaged manoeuvre | 2 |
 * | `mayPublish` names the interlock before the connected aircraft | 1 |
 * | `mayArm` allows arming during a replay | 5 |
 * | `StateSource.liveState` consults the replay feed | 3 |
 * | a MAVLink-only recording also answers the Zenoh sink | 1 |
 * | a Zenoh-only recording also answers the MAVLink sink | 1 |
 * | `install(null)` leaves the recording in place | 2 |
 * | a recording with neither sink on still reads as publishing | 1 |
 * | a cursor with no sample yet falls through to the aircraft | 1 |
 * | `takeSightings` re-emits what it has already handed out | 1 |
 * | a seek does not re-arm, so a forward jump delivers the burst it skipped | 1 |
 * | the sighting cursor starts at the first sighting rather than below it | 1 |
 * | `sightings` defaults a missing tag id to 0 instead of skipping the line | 1 |
 * | **`sightings` reads every line rather than only `tag` lines** | **0, then 1** |
 * | the publishing banner is the quiet one | 1 |
 * | the publishing watermark is the quiet one | 1 |
 * | the bus announcement drops the word REPLAY | 1 |
 * | the two sinks are named in the other order | 2 |
 * | **the Scene drops the publishing flag on the way to the view** | **0, then 1** |
 * | the Scene drops the publishing flag on the empty early return | 1 |
 *
 * Each row is one mutation applied alone to `src/main`, with the **whole suite** run against it
 * and `app/build/test-results/testDebugUnitTest` deleted first — a mutation that fails to compile
 * otherwise leaves the previous run's XML and reports a confident zero. Harness: `tmp/mutate.py`.
 * Suite 2253 tests on the first run, 2254 after the two fixes below. Nothing was NO-RUN.
 *
 * ### Two survived, and both were real holes
 *
 * **`sightings` reads every line rather than only `tag` lines** scored **0**. The reason is worth
 * keeping: `onlyTagLinesBecomeSightings` listed a `dji_state` and an `event`, and *both decode to
 * their own `RecordEntry` types* rather than to `Other` — so dropping the kind check killed
 * nothing, because no line in the test could reach the branch it guards. The line that can is
 * `mav_in`, which decodes to `Other` **and carries `"id"`** — the MAVLink message id. The bug that
 * would have shipped is every MAVLink line in a record becoming a detection: 21 303 of them in the
 * reference session, each claiming a tag whose number is a message id, published onto the bus. The
 * test now carries a `mav_in` and a `mav_out` line and the mutation dies.
 *
 * **The Scene drops the publishing flag** scored **0** because nothing built a `Scene` at all —
 * the flag was asserted on the `Situation` and on `ReplayPublication`, and the step between them
 * was untested. `SituationSceneTest` now asserts it on both of `SituationScene.build`'s exits,
 * including the empty early return, which is the branch every published replay starts in.
 */
class ReplayPublishTest {

    /**
     * The seam is a process-wide holder, like `ZenohBus.gimbalSource` and `Recorder.gimbalSource`
     * beside it, so a test that installs one has to put it back. Without this a later test in the
     * same JVM reads another test's recording, which is the exact failure this feature exists to
     * prevent, arriving in the test suite instead of on a bus.
     */
    @After
    fun clearSeam() {
        StateSource.install(null)
        StateSource.live = null
    }

    private val liveMarker = AircraftState(latitude = 1.0)
    private val replayedMarker = AircraftState(latitude = 2.0)

    private fun installLive() {
        StateSource.live = { liveMarker }
    }

    private fun feed(toMavlink: Boolean, toZenoh: Boolean) = StateSource.Feed(
        name = "20260728-123104.001",
        toMavlink = toMavlink,
        toZenoh = toZenoh,
        read = { replayedMarker },
    )

    // ───────────────────────────────────────────────── 1. the admission rules

    /**
     * **Rule 3.** A connected aircraft refuses publishing outright, and the sentence names the
     * hazard rather than the rule.
     */
    @Test
    fun publishingIsRefusedWhileAnAircraftIsConnected() {
        val verdict = ReplayAdmission.mayPublish(
            aircraftConnected = true, interlockEnabled = false, manoeuvreEngaged = false,
        )
        assertFalse(verdict.allowed)
        assertEquals(ReplayAdmission.REASON_AIRCRAFT_CONNECTED, verdict.reason)
        assertTrue(
            "the sentence must say what would happen, not merely that it is not allowed",
            verdict.reason!!.contains("two flights"),
        )
    }

    /**
     * The connected aircraft is checked **first**, and that ordering is the one an operator can
     * act on: with a live aircraft on the link the interlock is the state they will reach for
     * next, and being told about it would send them to switch off the wrong thing.
     */
    @Test
    fun theConnectedAircraftIsNamedBeforeTheInterlock() {
        val both = ReplayAdmission.mayPublish(
            aircraftConnected = true, interlockEnabled = true, manoeuvreEngaged = true,
        )
        assertEquals(ReplayAdmission.REASON_AIRCRAFT_CONNECTED, both.reason)
    }

    /** Rules 1 and 2, asked again here so the function is total for any caller that reaches it. */
    @Test
    fun publishingIsRefusedWhileCommandsAreLiveOrAManoeuvreIsFlying() {
        assertEquals(
            ReplayAdmission.REASON_COMMANDS_LIVE,
            ReplayAdmission.mayPublish(false, interlockEnabled = true, manoeuvreEngaged = false).reason,
        )
        assertEquals(
            ReplayAdmission.REASON_MANOEUVRE,
            ReplayAdmission.mayPublish(false, interlockEnabled = false, manoeuvreEngaged = true).reason,
        )
    }

    /** The quiet case: no aircraft, nothing armed, nothing flying. */
    @Test
    fun publishingIsAllowedWithNoAircraftAndNothingArmed() {
        val verdict = ReplayAdmission.mayPublish(false, false, false)
        assertTrue(verdict.allowed)
        assertNull("an allowed decision carries no sentence to show", verdict.reason)
    }

    /**
     * **Every refusal carries a sentence.** A refusal with no words is a switch that springs back
     * with no explanation, which trains an operator to tap it again harder.
     */
    @Test
    fun everyRefusalCarriesASentence() {
        val refusals = listOf(
            ReplayAdmission.mayPublish(true, false, false),
            ReplayAdmission.mayPublish(false, true, false),
            ReplayAdmission.mayPublish(false, false, true),
            ReplayAdmission.mayArm(replayActive = true),
            ReplayAdmission.mayReplay(interlockEnabled = true, manoeuvreEngaged = false),
        )
        for (r in refusals) {
            assertFalse(r.allowed)
            assertTrue("a refusal with no sentence: $r", !r.reason.isNullOrBlank())
        }
    }

    // ──────────────────────────────────────────────────────── 2. the seam

    /** With nothing installed both sinks read the aircraft, which is every session but this one. */
    @Test
    fun withNoRecordingInstalledEverySinkReadsTheAircraft() {
        installLive()
        assertFalse(StateSource.publishing)
        for (sink in StateSource.Sink.entries) {
            assertEquals(StateSource.Kind.LIVE, StateSource.kind(sink))
            assertEquals(liveMarker, StateSource.read(sink))
        }
    }

    /**
     * **The structural half.** A feed on *both* sinks still cannot be seen by the command path's
     * read, because that read takes no parameter and consults no feed.
     *
     * This is `SituationHonestyTest`'s argument one layer down: there, a replayed state has no
     * object to be routed into; here, the function the command path calls has no way to be handed
     * one. `guided/`, `mission/`, `CommandDispatcher`'s AMSL datum and the tag detector's arming
     * all read through it.
     */
    @Test
    fun theCommandPathsReadCannotBeSwitchedToARecording() {
        installLive()
        StateSource.install(feed(toMavlink = true, toZenoh = true))
        assertEquals(replayedMarker, StateSource.read(StateSource.Sink.MAVLINK))
        assertEquals(replayedMarker, StateSource.read(StateSource.Sink.ZENOH))
        assertEquals("the command path must still see the aircraft", liveMarker, StateSource.liveState())
    }

    /** Each sink is switched separately: one on does not turn the other on. */
    @Test
    fun theTwoSinksAreSwitchedIndependently() {
        installLive()
        StateSource.install(feed(toMavlink = true, toZenoh = false))
        assertEquals(replayedMarker, StateSource.read(StateSource.Sink.MAVLINK))
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.ZENOH))
        assertEquals(StateSource.Kind.REPLAY, StateSource.kind(StateSource.Sink.MAVLINK))
        assertEquals(StateSource.Kind.LIVE, StateSource.kind(StateSource.Sink.ZENOH))

        StateSource.install(feed(toMavlink = false, toZenoh = true))
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.MAVLINK))
        assertEquals(replayedMarker, StateSource.read(StateSource.Sink.ZENOH))
    }

    /**
     * A recording installed with **neither** sink on is a picture, and must not read as publishing
     * — that flag drives the banner, the watermark and the record's own event line.
     */
    @Test
    fun aFeedWithNeitherSinkOnIsNotPublishing() {
        installLive()
        StateSource.install(feed(toMavlink = false, toZenoh = false))
        assertFalse(StateSource.publishing)
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.MAVLINK))
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.ZENOH))

        StateSource.install(feed(toMavlink = false, toZenoh = true))
        assertTrue(StateSource.publishing)
    }

    /** Ending it restores the aircraft on both sinks, immediately and with no handshake. */
    @Test
    fun removingTheRecordingRestoresTheAircraft() {
        installLive()
        StateSource.install(feed(toMavlink = true, toZenoh = true))
        StateSource.install(null)
        assertNull(StateSource.feed())
        assertFalse(StateSource.publishing)
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.MAVLINK))
        assertEquals(liveMarker, StateSource.read(StateSource.Sink.ZENOH))
    }

    /**
     * **A cursor that has not reached a sample publishes nothing, not the last thing it had.**
     *
     * Null from the feed becomes an all-null state rather than a held one. Before the first
     * sample there is no reading, and this project's oldest rule is that no reading is null —
     * a held value wearing a fresh face is the failure `AircraftState` is null-per-field to
     * prevent.
     */
    @Test
    fun aCursorWithNoSampleYetReadsAsNothingRatherThanAHeldValue() {
        installLive()
        StateSource.install(
            StateSource.Feed("empty", toMavlink = true, toZenoh = true, read = { null })
        )
        assertEquals(AircraftState(), StateSource.read(StateSource.Sink.MAVLINK))
        assertNull(StateSource.read(StateSource.Sink.ZENOH).latitude)
    }

    // ─────────────────────────────────── 3. the chain that keeps commands out

    /**
     * **An inbound command arriving mid-replay is refused**, driven through the real dispatcher.
     *
     * The chain, asserted link by link rather than assumed:
     *
     *  1. A replay is on screen, so [ReplayAdmission.mayArm] refuses.
     *  2. So the interlock is never enabled — and this test never enables it, which is exactly
     *     what rule 2 guarantees for a UI that honours the refusal.
     *  3. So every actuating command is answered `MAV_RESULT_UNSUPPORTED` and **nothing reaches
     *     `FlightActions`**, which is the assertion that matters: an ack is a sentence, but an
     *     empty call list is the aircraft not moving.
     *
     * Driven through **both inbound shapes**, because QGC uses both and they take different paths
     * through the responder: `SET_MODE` for the Return button — the one QGC lights up the moment
     * it believes a vehicle is flying, which with MAVLink-out on is exactly what a replay makes it
     * believe — and a `COMMAND_LONG` for Takeoff.
     */
    @Test
    fun anInboundCommandDuringAReplayIsRefusedAndReachesNoAircraft() {
        // 1. A replay is on screen.
        val armVerdict = ReplayAdmission.mayArm(replayActive = true)
        assertFalse("a replay must make arming impossible", armVerdict.allowed)
        assertEquals(ReplayAdmission.REASON_REPLAYING, armVerdict.reason)

        // 2. So the interlock is off — never enabled, because the UI honoured the refusal.
        val rig = Rig()
        assertFalse(rig.interlock.enabled)

        // 3a. QGC's Return, which is a SET_MODE and is never acked — the refusal is a sentence.
        rig.responder.onMessage(setMode(PX4_AUTO_RTL), 255, 190)
        assertEquals(
            "nothing may reach the aircraft while a recording is on screen",
            emptyList<FlightAction>(),
            rig.actions.calls,
        )

        // 3b. And a COMMAND_LONG, which is acked, and refused.
        rig.responder.onMessage(takeoff(), 255, 190)
        val acks = rig.sent.filterIsInstance<CommandAck>()
        assertEquals(1, acks.size)
        assertEquals(MavResult.MAV_RESULT_UNSUPPORTED, acks.single().result().entry())
        assertEquals(emptyList<FlightAction>(), rig.actions.calls)
    }

    /**
     * And the same Return **is** honoured once the interlock is on, so the test above is measuring
     * the interlock rather than a dispatcher that refuses everything.
     *
     * A refusal test with no matching acceptance proves nothing: it passes just as well against a
     * dispatcher that has been broken into rejecting every command there is.
     */
    @Test
    fun theSameCommandIsHonouredOnceTheInterlockIsOn() {
        val rig = Rig()
        rig.interlock.enable()
        rig.responder.onMessage(setMode(PX4_AUTO_RTL), 255, 190)
        assertEquals(listOf(FlightAction.RETURN_TO_HOME), rig.actions.calls)
    }

    // ───────────────────────────────────────────── 4. what the replay says

    /** The banner names the sinks, because "on screen" and "being broadcast" are different states. */
    @Test
    fun theBannerNamesWhatIsBeingPublished() {
        val name = "20260728-123104.001"
        assertEquals(
            "REPLAY — NOT THE AIRCRAFT · $name",
            ReplayPublication.banner(name, toMavlink = false, toZenoh = false),
        )
        assertEquals(
            "REPLAY PUBLISHING → MAVLink — NOT THE AIRCRAFT · $name",
            ReplayPublication.banner(name, toMavlink = true, toZenoh = false),
        )
        assertEquals(
            "REPLAY PUBLISHING → Zenoh — NOT THE AIRCRAFT · $name",
            ReplayPublication.banner(name, toMavlink = false, toZenoh = true),
        )
        assertEquals(
            "REPLAY PUBLISHING → MAVLink + Zenoh — NOT THE AIRCRAFT · $name",
            ReplayPublication.banner(name, toMavlink = true, toZenoh = true),
        )
    }

    /** The whole-picture wash says which of the two states it is. */
    @Test
    fun theWatermarkSaysWhetherTheRecordingHasLeftThePhone() {
        assertEquals(ReplayPublication.WATERMARK_QUIET, ReplayPublication.watermark(false))
        assertEquals(ReplayPublication.WATERMARK_PUBLISHING, ReplayPublication.watermark(true))
        org.junit.Assert.assertNotEquals(
            ReplayPublication.WATERMARK_QUIET,
            ReplayPublication.WATERMARK_PUBLISHING,
        )
        assertTrue(ReplayPublication.WATERMARK_PUBLISHING.contains("PUBLISHING"))
        // Both still say the thing that matters most, so neither can be read as an aircraft.
        assertTrue(ReplayPublication.WATERMARK_QUIET.contains("NOT THE AIRCRAFT"))
        assertTrue(ReplayPublication.WATERMARK_PUBLISHING.contains("NOT THE AIRCRAFT"))
    }

    /**
     * **The sentence a subscriber sees.** It is the only thing on the bus that says the stream is
     * a recording, so it has to say so in words a consumer logging status text will keep.
     */
    @Test
    fun theBusIsToldTheStreamIsARecording() {
        val text = ReplayPublication.announcement("20260728-123104.001", true, true)
        assertTrue(text.startsWith("REPLAY"))
        assertTrue(text.contains("20260728-123104.001"))
        assertTrue(text.contains("MAVLink + Zenoh"))
        assertTrue("a subscriber must be told what it is not", text.contains("not an aircraft"))
        assertTrue(ReplayPublication.endedAnnouncement("x").contains("no longer"))
    }

    // ────────────────────────────────────────── 5. the sightings, as events

    private fun sighting(id: Int) = com.dimensional.mini4pro.vision.TagSighting.Sighting(
        tagId = id, x = 0.1, y = 0.2, z = 3.0, atNanos = 0L, pixelSize = 100.0,
        imageWidth = 1920, imageHeight = 1080,
    )

    private fun playerWithSightings(): ReplayPlayer = ReplayPlayer(
        samples = listOf(
            ReplaySample(0.0, null, null, AircraftState()),
            ReplaySample(10.0, null, null, AircraftState()),
        ),
        name = "tags",
        sightings = listOf(
            TimedSighting(1.0, sighting(1)),
            TimedSighting(2.0, sighting(2)),
            TimedSighting(8.0, sighting(8)),
        ),
    )

    /** Runs the cursor forward by [seconds] of playback, as the drawing tick does. */
    private fun ReplayPlayer.run(seconds: Double, from: Long = 0L) {
        play(from)
        var t = from
        val stepMs = 200L
        var left = seconds
        while (left > 0.0) {
            t += stepMs
            onClock(t)
            left -= stepMs / 1000.0
        }
    }

    /**
     * **A sighting leaves exactly once.** A state is a level and a sighting is an event, and
     * re-publishing the one the cursor happens to be sitting past would be a detection that never
     * happened.
     */
    @Test
    fun eachSightingIsHandedOutOnceAndOnlyOnce() {
        val p = playerWithSightings()
        p.run(2.5)
        assertEquals(listOf(1, 2), p.takeSightings().map { it.sighting.tagId })
        assertEquals(
            "asked again at the same position: nothing",
            emptyList<Int>(),
            p.takeSightings().map { it.sighting.tagId },
        )
        p.run(6.0, from = 10_000L)
        assertEquals(listOf(8), p.takeSightings().map { it.sighting.tagId })
    }

    /** A recording with no `tag` lines hands out nothing, forever, without special-casing. */
    @Test
    fun aRecordingWithNoTagLinesPublishesNothing() {
        val p = ReplayPlayer(
            listOf(
                ReplaySample(0.0, null, null, AircraftState()),
                ReplaySample(5.0, null, null, AircraftState()),
            ),
            "none",
        )
        p.run(5.0)
        assertEquals(emptyList<TimedSighting>(), p.takeSightings())
    }

    /**
     * **A seek emits nothing, in either direction.** The jump is not playback.
     *
     * Forward: the forty detections that were skipped did not happen on this run, and a
     * subscriber receiving them at once with the aircraft ten seconds further on would be reading
     * a lie about when they were seen. Backward: dragging back means "play that again", and the
     * playback that follows the drag is what replays them — not the drag itself, which would
     * deliver the whole stretch in one tick.
     */
    @Test
    fun aSeekEmitsNothingInEitherDirection() {
        val forward = playerWithSightings()
        forward.seekFraction(1.0, 0L)
        assertEquals(
            "a forward jump must not deliver the burst it skipped",
            emptyList<Int>(),
            forward.takeSightings().map { it.sighting.tagId },
        )

        val back = playerWithSightings()
        back.run(2.5)
        assertEquals(listOf(1, 2), back.takeSightings().map { it.sighting.tagId })
        back.seekFraction(0.0, 5_000L)
        assertEquals(
            "the drag itself is not an emission",
            emptyList<Int>(),
            back.takeSightings().map { it.sighting.tagId },
        )
        // …and playing from there replays them, one tick at a time, as they are reached.
        back.run(2.5, from = 6_000L)
        assertEquals(listOf(1, 2), back.takeSightings().map { it.sighting.tagId })
    }

    /** A sighting at `t = 0` is still emitted — the cursor starts *below* the first one. */
    @Test
    fun aSightingAtTheVeryStartIsNotSwallowed() {
        val p = ReplayPlayer(
            samples = listOf(
                ReplaySample(0.0, null, null, AircraftState()),
                ReplaySample(4.0, null, null, AircraftState()),
            ),
            name = "t0",
            sightings = listOf(TimedSighting(0.0, sighting(7))),
        )
        assertEquals(listOf(7), p.takeSightings().map { it.sighting.tagId })
    }

    // ──────────────────────────────────── 6. reading sightings off a record

    private fun tagLine(extra: String = "", id: String = "\"id\":7,"): String =
        """{"t":1.5,"k":"tag",$id"cx":700.0,"cy":300.0,"px":112.5,"w":1920,"h":1080,""" +
            """"ham":0,"margin":41.75,"x":-0.375,"y":0.8125,"z":3.5,"metric":false$extra}"""

    private fun readSightings(vararg lines: String) = FlightReplay.sightings(
        FlightRecordReader.read(
            lines.asSequence(),
            FlightRecordReader.Options(kinds = null),
        )
    )

    /** The whole line, back as the `Sighting` the detector produced. */
    @Test
    fun aTagLineComesBackAsTheSightingItWasWrittenFrom() {
        val s = readSightings(tagLine()).single()
        assertEquals(1.5, s.tSeconds, 1e-9)
        assertEquals(7, s.sighting.tagId)
        assertEquals(-0.375, s.sighting.x, 1e-9)
        assertEquals(0.8125, s.sighting.y, 1e-9)
        assertEquals(3.5, s.sighting.z, 1e-9)
        assertEquals(41.75, s.sighting.decisionMargin, 1e-9)
        assertEquals(1920, s.sighting.imageWidth)
        assertEquals(1080, s.sighting.imageHeight)
        assertFalse(s.sighting.metric)
        assertEquals(1_500_000_000L, s.sighting.atNanos)
    }

    /**
     * **Tolerant in both directions of the skew that is happening right now.**
     *
     * Fields that do not exist yet are read as absent, and fields added by other work are ignored
     * until this is taught to read them. Neither may break a replay: corner and solved-pose
     * fields are being added to `tag` lines while this is being written.
     */
    @Test
    fun anUnknownFieldIsIgnoredAndAMissingOneFallsBackToItsDefault() {
        val withExtra = readSightings(
            tagLine(extra = ""","corners":[1.0,2.0,3.0,4.0],"solved_pitch":12.5""")
        ).single()
        assertEquals(7, withExtra.sighting.tagId)
        assertEquals(3.5, withExtra.sighting.z, 1e-9)

        // A line from before half these fields existed: what is absent is absent, not zero-ish
        // nonsense, and the sighting still reconstructs.
        val sparse = readSightings("""{"t":2.0,"k":"tag","id":3,"metric":false}""").single()
        assertEquals(3, sparse.sighting.tagId)
        assertEquals(0.0, sparse.sighting.z, 0.0)
        assertEquals(0, sparse.sighting.imageWidth)
    }

    /**
     * **A line with no id is skipped, never defaulted.** Tag 0 is a real tag, so there is no
     * value that could stand in for "this line did not say", and a sighting of tag 0 that was
     * never seen is the false detection the whole latch exists to keep out.
     */
    @Test
    fun aTagLineWithNoIdIsSkipped() {
        assertEquals(emptyList<TimedSighting>(), readSightings(tagLine(id = "")))
        // And tag 0 itself survives, so the skip is about absence rather than about zero.
        assertEquals(1, readSightings(tagLine(id = "\"id\":0,")).size)
        assertEquals(0, readSightings(tagLine(id = "\"id\":0,")).single().sighting.tagId)
    }

    /**
     * **Nothing but `tag` lines becomes a sighting — and `mav_in` is the reason that check exists.**
     *
     * A `mav_in`/`mav_out` line carries `"id"`: the **MAVLink message id**, not a tag. It also
     * decodes to `RecordEntry.Other`, the same branch a `tag` line lands in. So a reconstruction
     * that filtered on "is it an `Other` with an id" would turn every MAVLink line in the record
     * into a detection — 21 303 of them in the reference session, each claiming a tag whose number
     * is a message id — and publish the lot onto the bus.
     *
     * `FlightLogLibrary` also filters by kind at read time, so the shipping path is belted and
     * braced. This asserts the brace, because [FlightReplay.sightings] is a pure function over any
     * record and `SUMMARY_KINDS` and `kinds = null` are both real ways to hand it one.
     *
     * **This test used to pass against a broken reconstruction.** It listed only `dji_state` and
     * `event` lines, and both decode to their own `RecordEntry` types rather than to `Other`, so
     * dropping the kind check killed nothing. A mutation run on 2026-07-28 found that; the
     * `mav_in` line below is the fix.
     */
    @Test
    fun onlyTagLinesBecomeSightings() {
        val out = readSightings(
            """{"t":0.5,"k":"dji_state","lat":1.0}""",
            // `id` here is MAVLink's HEARTBEAT (0) — a real line shape, and the trap.
            """{"t":0.7,"k":"mav_in","name":"Heartbeat","id":0,"sys":255,"comp":190}""",
            """{"t":0.9,"k":"mav_out","name":"Attitude","id":30,"sys":1,"comp":1}""",
            tagLine(),
            """{"t":2.0,"k":"event","code":"takeoff"}""",
        )
        assertEquals("only the tag line is a sighting", 1, out.size)
        assertEquals(7, out.single().sighting.tagId)
    }

    // ─────────────────────────────────────────────────────────── the rig

    /** PX4's `AUTO.RTL` custom mode, as `telemetry/Px4Mode` packs it. QGC's Return button. */
    private val PX4_AUTO_RTL = com.dimensional.mini4pro.telemetry.Px4Mode.AUTO_RTL

    private fun setMode(customMode: Long) = SetMode.builder()
        .targetSystem(1) // SET_MODE has no target_component field
        .baseMode(EnumValue.create<MavMode>(1)) // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        .customMode(customMode)
        .build()

    private fun takeoff() = CommandLong.builder()
        .targetSystem(1).targetComponent(1)
        .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
        .confirmation(0)
        .param7(5.0f)
        .build()

    /** The real dispatcher and the real responder, wired as `Bridge` wires them. */
    private class Rig {
        val sent = mutableListOf<Any>()
        val interlock = CommandInterlock()
        val actions = RecordingActions()
        val dispatcher = CommandDispatcher(
            interlock = interlock,
            announcer = Announcer(StatusTextSink { sent.add(it) }),
        ).also { it.actions = actions }
        val responder = HandshakeResponder(send = { sent.add(it) }, nowMs = { 1_000L })
            .also { dispatcher.attachTo(it) }
    }

    /** Records what it was asked. An empty call list is the aircraft not moving. */
    private class RecordingActions : FlightActions {
        val calls = mutableListOf<FlightAction>()
        override fun takeoff() = record(FlightAction.TAKEOFF)
        override fun returnToHome() = record(FlightAction.RETURN_TO_HOME)
        override fun land() = record(FlightAction.LAND)
        private fun record(a: FlightAction): ActionOutcome {
            calls.add(a)
            return ActionOutcome.Requested
        }
    }
}
