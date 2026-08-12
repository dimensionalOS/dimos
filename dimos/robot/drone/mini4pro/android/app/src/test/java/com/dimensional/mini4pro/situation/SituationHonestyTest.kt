package com.dimensional.mini4pro.situation

import com.dimensional.mini4pro.replay.ReplayAdmission
import com.dimensional.mini4pro.telemetry.AircraftState
import com.dimensional.mini4pro.telemetry.SampleAges
import com.dimensional.mini4pro.telemetry.Signal
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.lang.reflect.ParameterizedType

/**
 * **The replay safety argument, as tests.**
 *
 * A top-down view of an aircraft, painted on a phone that sits beside an armed interlock, is a
 * claim about *now*. Driving that same view from a recording produces a picture that is
 * pixel-for-pixel the shape of a live flight and is a picture of a different afternoon. If the
 * two can be confused — by an operator, or by a wire in the code — the result is somebody
 * commanding an aircraft they are not looking at. That is the worst outcome available to this
 * feature, so it is defended twice: **structurally**, by the type of what leaves the replay
 * path, and **behaviourally**, by [ReplayAdmission].
 *
 * ## 1. Structural: there is nothing for a replayed state to be routed into
 *
 * A replayed `AircraftState` is turned into a [Situation] before it leaves the replay
 * controller, and [Situation] — with everything reachable from it — is **flat value data**:
 * doubles, ints, booleans, strings, enums, lists of the same. No `AircraftState`, no engine, no
 * port, no function type, no `Context`.
 *
 * That is not a style preference. It means a replayed state cannot be handed to something that
 * commands, because by the time it is in the drawing path it *is not a state any more* — it is a
 * bag of coordinates with no method on it. The property is asserted here by reflection rather
 * than by review, so adding `val engine: GuidedStickEngine` or keeping the raw state around
 * "just for the altitude" fails the build rather than passing a code review.
 *
 * ## 2. Behavioural: live commands and a replayed picture never coexist
 *
 * [ReplayAdmission]'s two rules, in both orders of arrival. Rule 1 alone is a door you walk
 * through and lock behind you; rule 2 is what makes it hold.
 *
 * ## Measured mutation kill counts
 *
 * Mutation-checked 2026-07-27, one breakage at a time against the shipped source, run,
 * confirmed red, reverted. Counts are failing tests across `SituationHonestyTest`,
 * `SituationReadingTest` and `ReplayPlayerTest` — measured, not estimated.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `AircraftState` kept as a field of `Situation` | 1 |
 *  | a `(() -> Unit)?` callback added to `Situation` | **build fails** |
 *  | a `GuidedSituation` reachable from `OrbitMark` | 1 |
 *  | replay source rewritten to `LIVE` in `SituationReading.read` | 2 |
 *  | the source dropped on the way into the `Scene` | 2 |
 *  | `mayReplay` allowing a replay while the interlock is on | 3 |
 *  | `mayReplay` allowing a replay mid-manoeuvre | 2 |
 *  | `mayArm` allowing arming during a replay | 3 |
 *  | a refusal returned with no sentence | 4 |
 *  | the replay cursor interpolating between two samples | 3 |
 *  | the background gap counted as playback time | 2 |
 *  | the cursor running past the end of the recording | 1 |
 *
 * The callback row is the interesting one. Adding a function type to [Situation] does not merely
 * fail a test — the suite stops compiling, because the field lands in a data class whose
 * constructor every test calls. A structural property enforced at build time is the strongest
 * form this kind of rule comes in, and it is worth noting that the reflection test is the
 * *weaker* of the two mechanisms guarding it.
 */
class SituationHonestyTest {

    /**
     * Everything a drawable value is allowed to be made of.
     *
     * Deliberately short, and deliberately does **not** include `Any`, `Object`, function types,
     * or any type from `telemetry`, `guided`, `mission`, `command` or `record`. Widening it is
     * how this test would be defeated, which is why the list is spelled out rather than derived.
     */
    private val allowed: Set<Class<*>> = setOf(
        Double::class.javaPrimitiveType!!, Double::class.javaObjectType,
        Float::class.javaPrimitiveType!!, Float::class.javaObjectType,
        Int::class.javaPrimitiveType!!, Int::class.javaObjectType,
        Long::class.javaPrimitiveType!!, Long::class.javaObjectType,
        Boolean::class.javaPrimitiveType!!, Boolean::class.javaObjectType,
        String::class.java, List::class.java,
    )

    /** The situation types themselves — value carriers, so they may contain one another. */
    private val situationTypes: Set<Class<*>> = setOf(
        Situation::class.java, AircraftMark::class.java, OrbitMark::class.java,
        GotoMark::class.java, RoiMark::class.java, PlanMark::class.java,
        PlanPoint::class.java, Fix::class.java,
        // The track is history rather than now, and is held to exactly the same rule: a
        // `TrackMark` is segments of `Fix`, and a segment is a list of `Fix`. Nothing else may
        // be smuggled in behind it — a per-point `AircraftState` "just for the altitude" would
        // put a commandable object back in the drawing path through the longest-lived field
        // in the model.
        TrackMark::class.java, TrackSegment::class.java,
    )

    @Test
    fun `the drawable model is flat value data with nothing to call`() {
        val visited = HashSet<Class<*>>()
        val queue = ArrayDeque<Class<*>>()
        queue.add(Situation::class.java)
        var checked = 0
        while (queue.isNotEmpty()) {
            val type = queue.removeFirst()
            if (!visited.add(type)) continue
            for (field in type.declaredFields) {
                // `Companion`, `$stable` and friends are the compiler's, not the model's.
                if (java.lang.reflect.Modifier.isStatic(field.modifiers)) continue
                checked++
                val members = mutableListOf(field.type)
                // A List<T> is only as safe as its T, so the argument is checked too.
                (field.genericType as? ParameterizedType)?.actualTypeArguments?.forEach { arg ->
                    (arg as? Class<*>)?.let { members.add(it) }
                }
                for (member in members) {
                    if (member.isEnum) continue
                    if (member in allowed) continue
                    if (member in situationTypes) {
                        queue.add(member)
                        continue
                    }
                    throw AssertionError(
                        "${type.simpleName}.${field.name} is a ${member.name}. " +
                            "The drawable model must be flat value data: a replayed flight is " +
                            "turned into a Situation before it leaves the replay path, and this " +
                            "is what guarantees there is nothing on it to command an aircraft with.",
                    )
                }
            }
        }
        assertTrue("reflection found nothing — the test would pass vacuously", checked > 10)
        assertTrue("every situation type must be reachable from Situation", visited.size >= 7)
    }

    @Test
    fun `a replayed picture is labelled a replayed picture, all the way through`() {
        val state = AircraftState(
            fcConnected = true,
            latitude = 37.9938232,
            longitude = 23.7253477,
            yawDeg = 12.0,
            ages = SampleAges.of(mapOf(Signal.POSITION to 50L, Signal.ATTITUDE to 50L)),
        )
        val situation = SituationReading.read(SituationSource.REPLAY, state)
        assertEquals(SituationSource.REPLAY, situation.source)
        assertNotNull("a replay still draws an aircraft — that is the point", situation.aircraft)

        val scene = SituationScene.build(situation, Viewport(600.0, 400.0, 16.0))
        assertEquals(
            "the View must be told, not left to infer it from a flag elsewhere",
            SituationSource.REPLAY,
            scene.source,
        )
        assertFalse(scene.isEmpty)
    }

    @Test
    fun `live and replay are the only two answers there are`() {
        assertEquals(2, SituationSource.entries.size)
    }

    // ── ReplayAdmission ──────────────────────────────────────────────────────

    @Test
    fun `a recording may not be opened while commands are live`() {
        val d = ReplayAdmission.mayReplay(interlockEnabled = true, manoeuvreEngaged = false)
        assertFalse(d.allowed)
        assertEquals(ReplayAdmission.REASON_COMMANDS_LIVE, d.reason)
    }

    @Test
    fun `a recording may not be opened while a manoeuvre is flying`() {
        val d = ReplayAdmission.mayReplay(interlockEnabled = false, manoeuvreEngaged = true)
        assertFalse(d.allowed)
        assertEquals(ReplayAdmission.REASON_MANOEUVRE, d.reason)
    }

    @Test
    fun `with nothing commanding, a recording may be opened`() {
        val d = ReplayAdmission.mayReplay(interlockEnabled = false, manoeuvreEngaged = false)
        assertTrue(d.allowed)
        assertEquals(null, d.reason)
    }

    @Test
    fun `commands may not be armed while a recording is on screen`() {
        val d = ReplayAdmission.mayArm(replayActive = true)
        assertFalse(d.allowed)
        assertEquals(ReplayAdmission.REASON_REPLAYING, d.reason)
    }

    @Test
    fun `with no recording on screen, arming is none of this file's business`() {
        assertTrue(ReplayAdmission.mayArm(replayActive = false).allowed)
    }

    @Test
    fun `the two rules close the loop in both orders of arrival`() {
        // Armed first, then reaching for replay.
        assertFalse(ReplayAdmission.mayReplay(interlockEnabled = true, manoeuvreEngaged = false).allowed)
        // Replaying first, then reaching for the arming switch.
        assertFalse(ReplayAdmission.mayArm(replayActive = true).allowed)
    }

    @Test
    fun `every refusal carries a sentence to show the operator`() {
        val refusals = listOf(
            ReplayAdmission.mayReplay(true, false),
            ReplayAdmission.mayReplay(false, true),
            ReplayAdmission.mayArm(true),
        )
        for (d in refusals) {
            assertFalse(d.allowed)
            assertTrue("a silent refusal is the one refusal this project does not ship", !d.reason.isNullOrBlank())
        }
    }
}
