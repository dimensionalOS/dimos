package com.dimensional.mini4pro

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File

/**
 * **The cockpit's switch positions at app start, read out of the layout that sets them.**
 *
 * A control's default is a flight-behaviour decision — it is what the operator gets if they never
 * touch anything — and in this app those defaults live in `res/layout/activity_main.xml` rather
 * than in Kotlin, which puts them outside every other test in this suite. That is exactly the shape
 * a change slips through in: a one-character edit to an XML attribute, no compiler, no test, and a
 * different aircraft on the next flight. So the XML is read as text here, on the
 * `record/RecordingSeamTest` precedent (that suite walks `src/main/java` the same way, for the same
 * reason: some properties are properties of *files*, not of types).
 *
 * This is a **defaults** test and deliberately nothing more. What the switch *does* is the engine's
 * and is pinned by `GuidedAutolandTest` and `GuidedPrecisionLandTest`; what the dialog says is
 * `MainActivity`'s. Nothing here asserts that a default is safe — only that it is the one that was
 * decided, on the date it was decided, so that changing it is a deliberate act with a test to
 * update.
 *
 * ## Mutations killed, measured 2026-07-30
 *
 * Whole suite per mutant, **2644 tests**, `test-results` deleted first, confirmed red, reverted.
 *
 * | mutation | tests that failed |
 * |---|---|
 * | the full-autoland default flipped back to OFF (with its caption) | 1 |
 * | `COMMAND_INTERLOCK` flipped back to `false` (the interlock stays disarmed) | 1 |
 * | `GCS_HOST` back to `""` (a fresh install comes up silent) | 1 |
 * | `FAST_RECORD` back to `false` (5 Hz state) | 1 |
 * | `ZENOH_VIDEO` back to `false` (no camera on the bus) | 1 |
 * | `RECORD_VIDEO` back to `false` (no on-phone sidecars) | 1 |
 * | **the startup arming call deleted from `wireCommandInterlock`** | 1 |
 * | **the replay gate in `armCommands` fed a constant instead of `replay != null`** | 1 |
 * | **`videoRecord.isChecked` no longer assigned from `Recorder.videoEnabled`** | 1 |
 *
 * The value rows are 1 **by construction** — each is asserted in exactly one place here, and that
 * is all a defaults test can be. It is still the reason this file exists: the count for the first
 * row was **0** before it, because an XML attribute is not compiled, not typed and not read by any
 * other test, so a whole default could be reversed by a one-character edit with the suite green.
 * The Kotlin constants are typed and compiled but share the weakness that matters — nothing else in
 * the suite asserts a *value*, only that the plumbing reads whatever is there.
 *
 * ## The survivor, 2026-07-30
 *
 * The last row measured **0** on its first run and is the reason it is now a row at all. The test
 * asserted `armCommands` contained `"ReplayAdmission.mayArm("` — so the mutant that changed the
 * argument to a hardcoded `false`, disabling the gate while leaving the call in place, passed the
 * whole suite. A gate fed a constant is not a gate, and a test that only looks for the function
 * name cannot tell the difference. The assertion now pins the argument, and the re-measured count
 * is the 1 above.
 *
 * That is precisely the failure this project's mutation protocol exists to catch, and it is worth
 * noting where it happened: not in the flight law, but in a text-matching test written to cover an
 * Activity that no executable test can reach. The weaker the instrument, the more it needs the
 * mutant to prove it is measuring anything.
 */
class CockpitDefaultsTest {

    /**
     * **The session defaults, as one assertion per property.**
     *
     * These moved from "everything off" to "the way this aircraft is actually flown" on 2026-07-30,
     * when the airframe phone was replaced and a fresh install came up configured for nothing. The
     * values are not a judgement call — they were read out of the old phone's `shared_prefs` after
     * three autonomous missions. [CockpitDefaults] carries the reasoning and the costs.
     *
     * Asserted individually rather than as a set, so a failure names the property that moved.
     */
    @Test
    fun `a fresh install comes up configured for flight`() {
        assertEquals(
            "the bridge autostarts only if a host is saved; empty means a silent phone",
            "10.55.1.50", CockpitDefaults.GCS_HOST,
        )
        assertTrue("camera video to the GCS", CockpitDefaults.GCS_VIDEO)
        assertTrue("the Zenoh transport", CockpitDefaults.ZENOH)
        assertTrue("camera video on the bus — this is the uplink-doubling one", CockpitDefaults.ZENOH_VIDEO)
        assertTrue("AprilTag detections on the bus (metric=false travels in every id)", CockpitDefaults.ZENOH_DETECTIONS)
        assertTrue("25 Hz state recording, the rate control delay is visible at", CockpitDefaults.FAST_RECORD)
        assertTrue("on-phone video sidecars, what makes a flight reconstructable", CockpitDefaults.RECORD_VIDEO)
    }

    /**
     * **The Record-video switch is assigned from `Recorder.videoEnabled`, not just labelled.**
     *
     * A default nobody can see is worse than no default. `wireVideoRecording` reads `videoEnabled`
     * once at `onCreate`, when no recorder has started and the answer is therefore always `false`;
     * the sidecar that honours [CockpitDefaults.RECORD_VIDEO] is built later, inside
     * `Recorder.start`. So switching this default on without also driving the switch from the
     * recorder would have produced the project's characteristic bad shape: video writing to disk
     * under a control reading OFF, with the click handler's own comment — *"a control that silently
     * disagrees with the thing it controls is the bug this line exists to prevent"* — sitting two
     * hundred lines above it.
     *
     * Pinned by text for the same reason as the arming test: `MainActivity` needs a framework.
     */
    @Test
    fun `the record-video switch tracks the recorder rather than its own initial state`() {
        val label = body(mainActivity.readText(), "private fun updateVideoRecordLabel()")
        assertTrue(
            "updateVideoRecordLabel must assign videoRecord.isChecked from Recorder.videoEnabled, " +
                "or a session that starts recording shows a switch that says OFF",
            label.contains("videoRecord.isChecked = Recorder.videoEnabled"),
        )
    }

    /**
     * **The command interlock is armed at app start, and no dialog asks first.**
     *
     * Kept apart from the switches above because it is the only default here about *authority*: it
     * means QGroundControl's Return and Land buttons reach DJI from startup. Ivan asked for both
     * halves on 2026-07-30 (*"enable interlock, enable control by default … We don't need
     * validation confirmation"*), and the full argument — including what the deleted dialog was and
     * was not protecting — is in [CockpitDefaults.COMMAND_INTERLOCK].
     *
     * What this test does **not** assert is that it is safe. The two properties that carry the
     * safety are pinned where they live: `CommandInterlockTest` holds "no inbound MAVLink message
     * can enable it", and `ReplayAdmission` still refuses to arm over a loaded replay — checked at
     * startup by the same `armCommands` path as the switch, precisely so the two cannot drift.
     */
    @Test
    fun `the command interlock arms itself at startup`() {
        assertTrue(
            "startup arming is Ivan's 2026-07-30 decision — see CockpitDefaults before changing it",
            CockpitDefaults.COMMAND_INTERLOCK,
        )
    }

    /**
     * **Autoland: ON by default since 2026-07-30.**
     *
     * Ivan: *"can we make this last stretch autoland the default toggled on in the app?"* — asked
     * after thirteen recorded tag descents and the first mission that flew its own approach. Only
     * the switch's starting position moved. The option is still **taken at arm time**, behind the
     * "Arm FULL AUTOLAND?" dialog that names where the flight ends, it is still pinned into the run
     * at the arm so a mid-descent flick changes nothing, and every gate the descent applies is
     * untouched.
     *
     * The caption is asserted with it, and that is not tidiness: `MainActivity.wireTagDescent`
     * installs an `OnCheckedChangeListener` that rewrites the label, and a listener only fires on a
     * **change**. A layout whose `android:checked` and `android:text` disagreed would show an
     * operator "Autoland: OFF" over a switch that was armed to land — until the first time they
     * touched it.
     */
    @Test
    fun `the full-autoland toggle starts ON, with a caption that agrees`() {
        val block = switchBlock("fullAutoland")
        assertEquals(
            "the full-autoland default is ON (Ivan, 2026-07-30) — see this test's KDoc before changing it",
            "true", attribute(block, "android:checked"),
        )
        assertEquals(
            "the caption must agree with the checked state: the listener only fires on a change",
            "Autoland: ON", attribute(block, "android:text"),
        )
    }

    /**
     * **The startup arming is actually wired, and it goes through the gate.**
     *
     * The constant above is worth 1 kill by construction — it pins a value, which is not the same
     * as pinning a behaviour. The behaviour that matters is in `MainActivity`, which needs an
     * Android framework to run and is therefore outside every executable test in this suite. That
     * is the same blind spot the layout XML has, so it gets the same instrument: the source is read
     * as text, on the `RecordingSeamTest` precedent.
     *
     * Two facts are asserted, and the second is the safety-carrying one:
     *
     * 1. `wireCommandInterlock` consults [CockpitDefaults.COMMAND_INTERLOCK] and arms. Without this,
     *    flipping the constant would be a no-op in one direction and nothing would notice.
     * 2. **`armCommands` asks `ReplayAdmission` before enabling anything.** This is the property
     *    that made a shared arming path worth having: the switch has always refused to arm a real
     *    aircraft over a loaded replay, and adding a second caller was the moment that guarantee
     *    could have quietly grown a hole. A startup path that called `Bridge.commandInterlock
     *    .enable()` directly would pass every other test in this file.
     *
     * A text test is a weak instrument and this KDoc should say so: it pins the *shape* of two
     * lines, not their semantics, and a rename would defeat it. It is here because the alternative
     * measured 0.
     */
    @Test
    fun `startup arming is wired, and cannot skip the replay gate`() {
        val src = mainActivity.readText()

        val wiring = body(src, "private fun wireCommandInterlock()")
        assertTrue(
            "wireCommandInterlock must arm at startup behind CockpitDefaults.COMMAND_INTERLOCK",
            wiring.contains("CockpitDefaults.COMMAND_INTERLOCK") && wiring.contains("armCommands()"),
        )

        val arming = body(src, "private fun armCommands()")
        assertTrue(
            "armCommands must ask ReplayAdmission about the LOADED replay — a replay must never " +
                "arm a real aircraft, and a gate fed a constant is not a gate",
            arming.contains("ReplayAdmission.mayArm(replay != null)"),
        )
        assertTrue(
            "the enable must come after the gate, not before it",
            arming.indexOf("ReplayAdmission.mayArm(") < arming.indexOf("commandInterlock.enable()"),
        )
        assertEquals(
            "armCommands is the ONLY place that enables the interlock outside the simulator's " +
                "edge detector; a second direct caller is how the gate grows a hole",
            2, Regex("""commandInterlock\.enable\(\)""").findAll(src).count(),
        )
    }

    // ─────────────────────────────────────────────────────────────────── reading the source

    /**
     * The text of a function body, from its signature to the next top-level `    }`.
     *
     * Crude on purpose — it only has to be good enough to tell "this call is inside that function"
     * from "this call is somewhere in a 2000-line file", and a parser here would be a second
     * language implementation to maintain for one assertion.
     */
    private fun body(src: String, signature: String): String {
        val start = src.indexOf(signature)
        assertTrue("no `$signature` in MainActivity.kt", start > 0)
        val end = src.indexOf("\n    }", start)
        assertTrue("could not find the end of `$signature`", end > start)
        return src.substring(start, end)
    }

    /** `src/main/java/com/dimensional/mini4pro/MainActivity.kt`, found the same way [layout] is. */
    private val mainActivity: File by lazy {
        var dir: File? = File(".").absoluteFile
        var found: File? = null
        repeat(6) {
            val candidate = dir?.let {
                File(it, "src/main/java/com/dimensional/mini4pro/MainActivity.kt")
            }
            if (found == null && candidate != null && candidate.isFile) found = candidate
            dir = dir?.parentFile
        }
        requireNotNull(found) { "could not locate MainActivity.kt from ${File(".").absolutePath}" }
    }

    // ─────────────────────────────────────────────────────────────────── reading the layout

    /** The `<...Switch .../>` element carrying `android:id="@+id/$id"`, as text. */
    private fun switchBlock(id: String): String {
        val xml = layout.readText()
        val anchor = xml.indexOf("@+id/$id")
        assertTrue("no view with id $id in ${layout.path}", anchor > 0)
        val start = xml.lastIndexOf('<', anchor)
        val end = xml.indexOf("/>", anchor)
        assertTrue("the $id element is not self-closing; this reader assumes it is", end > start)
        return xml.substring(start, end)
    }

    /** One attribute's value, or null. Fails loudly rather than defaulting — unknown is not a value. */
    private fun attribute(block: String, name: String): String? =
        Regex("""$name\s*=\s*"([^"]*)"""").find(block)?.groupValues?.get(1)

    /**
     * `src/main/res/layout/activity_main.xml`, found by walking up from the test's working
     * directory — `RecordingSeamTest.sourceRoot`'s pattern, including its reason: gradle runs unit
     * tests with the module directory as the working directory, but that is a default rather than a
     * guarantee, and a test that silently found no file would pass for the wrong reason.
     */
    private val layout: File by lazy {
        var dir: File? = File(".").absoluteFile
        var found: File? = null
        repeat(6) {
            val candidate = dir?.let { File(it, "src/main/res/layout/activity_main.xml") }
            if (found == null && candidate != null && candidate.isFile) found = candidate
            dir = dir?.parentFile
        }
        requireNotNull(found) { "could not locate activity_main.xml from ${File(".").absolutePath}" }
    }
}
