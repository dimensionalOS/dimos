package com.dimensional.mini4pro.warn

import com.dimensional.mini4pro.record.LogEntry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The device-health decision core: severity mapping, change detection, dedup under repeated
 * identical deliveries, the rate bound, and the ordering rule that decides which sentence wins a
 * scarce token.
 *
 * Pure — no DJI, no MAVLink, no clock but the `Long` each call is handed. That is the whole point
 * of the package's shape: the part that has to be right is the part that can be tested without an
 * aircraft.
 *
 * Written to fail loudly for the landmines this module actually has:
 *
 *  - **emitting per delivery instead of per change.** DJI re-delivers the complete list every time
 *    anything in it moves; a standing overheat would then be re-announced forever. This project
 *    once put 25 lines a second into a log monitor from a per-iteration log and buried everything
 *    else in the file — that class of mistake is forbidden, and several tests here exist only to
 *    keep it forbidden.
 *  - **a warning DJI raises that never reaches the operator** — the failure that created this
 *    package. Silence is tested for as carefully as noise.
 *  - **collapsing per-component faults**, so "all four ESCs overheating" reads as one ESC.
 *  - **`UNKNOWN` treated as `NORMAL`**, i.e. an unmappable DJI code silently read as good news.
 *  - **the rate bound eating the *serious* message** while spending its tokens on chatter.
 *
 * Mutation-checked 2026-07-26, one breakage at a time, failing tests counted across the three
 * health suites (this one, `WarnStatusTextsTest`, `DeviceHealthWatchTest`), code reverted after
 * each — measured counts, not estimates:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | emit per delivery (`previous` forced null; every item re-emitted every time) | 10 |
 *  | dedup keyed on `code` alone (component/sensor dropped from `Warning.key`) | 1 |
 *  | `CHANGED` not detected (the level comparison never fires) | 1 |
 *  | `CLEARED` not detected (disappearance ignored) | 6 |
 *  | title/description folded into the change test (`previous != item`; re-wording is news) | 1 |
 *  | `NORMAL` forwarded (`mavSeverity(NORMAL)` returns INFO) | 5 |
 *  | `UNKNOWN` mapped to `null` (unmappable DJI code silently dropped) | 3 |
 *  | `WARNING` mapped to `MAV_SEVERITY_WARNING` instead of `ERROR` (no QGC modal) | 3 |
 *  | `SERIOUS_WARNING` mapped to `ERROR` instead of `CRITICAL` | 3 |
 *  | `CLEARED` sent at the item's own severity instead of INFO (a clear pops a red modal) | 1 |
 *  | rate bound removed (`take` always grants) | 5 |
 *  | rate-bound window never slides (credit never returns) | 1 |
 *  | non-forwardable events spend a token (`take` called before the severity test) | 2 |
 *  | ordering by delivery order instead of severity (chatter wins the last token) | 2 |
 *  | clears not sorted last | 1 |
 *  | suppressed events dropped instead of returned with `rateLimited` | 5 |
 *  | duplicate key within one delivery not folded (diff taken against the raw list) | 1 |
 *  | `reset()` keeps the picture (a re-subscribe re-announces nothing) | 2 |
 *  | `recordSeverity` disagreeing with `mavSeverity` (`ERROR` recorded as `warn`) | 1 |
 *  | blank-title fallback removed (`Warning.name`, in `DeviceHealth.kt`) | 1 |
 *
 * The 10 and the 6 are the two that matter: the diff *is* the feature, and a clear that never
 * arrives leaves the operator's last information saying the aircraft is still overheating.
 *
 * **Several mutants score 1, and that is the point rather than a weakness** — each names one
 * property and exactly one test pins it, so a failure says which property broke instead of
 * lighting up the whole suite. The two that a careless suite would have scored **0** on are worth
 * naming:
 *
 *  - *suppressed events dropped.* The wire is identical either way — a suppressed event is by
 *    definition not sent — so only a test that asserts on the **returned list** rather than on
 *    `announce` can see it. [suppressed_events_are_still_returned] asserts the list length first,
 *    before anything about the flags, for exactly that reason.
 *  - *non-forwardable events spend a token.* Invisible unless `NORMAL` chatter and real warnings
 *    compete for the same window, which is what
 *    [events_that_are_never_forwarded_do_not_spend_credit] constructs deliberately.
 */
class WarningMonitorTest {

    /**
     * A delivery from the device-health source, which is what every test below that does not name
     * a source is about.
     *
     * The source became an argument on 2026-07-30, when the monitor stopped being device health's
     * and became every DJI warning's; these tests were written before that and their subject is
     * unchanged, so the shorthand keeps them readable. The multi-source properties — that one
     * source's delivery says nothing about another's picture — are asserted with the full call, by
     * name, further down.
     */
    private fun WarningMonitor.onDelivery(items: List<Warning>, nowMs: Long) =
        onDelivery(WarnSource.DEVICE_HEALTH, items, nowMs)

    private companion object {
        /** A DJI code shaped like the real ones. */
        const val OVERHEAT = "0x1600A0"
        const val COMPASS = "0x160050"

        fun item(
            code: String,
            level: WarnLevel,
            title: String = "t-$code",
            comp: Int? = 0,
            sensor: Int? = 0,
        ) = Warning(
            source = WarnSource.DEVICE_HEALTH,
            code = code,
            // DJI's health manager's state word *is* the level name, which is why this source
            // makes the two look alike; `WindWarningsTest` is where they diverge.
            state = level.name,
            level = level,
            title = title,
            description = "d-$code",
            componentId = comp,
            sensorIndex = sensor,
        )
    }

    // ── Severity mapping ─────────────────────────────────────────────────────

    /**
     * The published table, pinned constant by constant.
     *
     * Every one of these is a decision about what an operator sees in QGroundControl, read out of
     * QGC's own source rather than assumed (`Vehicle.cc:3466` speaks anything ≤ NOTICE;
     * `StatusTextHandler.cc:18-26` makes anything ≤ ERROR a red modal). Changing a row changes the
     * cockpit, so the table is asserted rather than described.
     */
    @Test
    fun severity_table_is_exactly_as_documented() {
        assertNull("NORMAL is never forwarded", WarnLevel.NORMAL.mavSeverity())
        assertEquals(
            WarnLevel.MAV_SEVERITY_INFO,
            WarnLevel.NOTICE.mavSeverity(),
        )
        assertEquals(
            WarnLevel.MAV_SEVERITY_WARNING,
            WarnLevel.CAUTION.mavSeverity(),
        )
        assertEquals(
            "DJI WARNING earns QGC's red modal — the whole reason this package exists",
            WarnLevel.MAV_SEVERITY_ERROR,
            WarnLevel.WARNING.mavSeverity(),
        )
        assertEquals(
            WarnLevel.MAV_SEVERITY_CRITICAL,
            WarnLevel.SERIOUS_WARNING.mavSeverity(),
        )
        assertEquals(
            "an unmappable DJI code is not good news",
            WarnLevel.MAV_SEVERITY_WARNING,
            WarnLevel.UNKNOWN.mavSeverity(),
        )
    }

    /** The record's severity is derived from the wire's, so the two can never disagree. */
    @Test
    fun record_severity_tracks_the_wire_severity() {
        assertEquals(LogEntry.SEV_INFO, WarnLevel.NORMAL.recordSeverity())
        assertEquals(LogEntry.SEV_INFO, WarnLevel.NOTICE.recordSeverity())
        assertEquals(LogEntry.SEV_WARN, WarnLevel.CAUTION.recordSeverity())
        assertEquals(LogEntry.SEV_ERROR, WarnLevel.WARNING.recordSeverity())
        assertEquals(LogEntry.SEV_ERROR, WarnLevel.SERIOUS_WARNING.recordSeverity())
        assertEquals(LogEntry.SEV_WARN, WarnLevel.UNKNOWN.recordSeverity())
    }

    /**
     * A `NORMAL` entry is **recorded and not forwarded** — both halves, in one test, because
     * either alone is satisfied by a mutant. Dropping it entirely satisfies "not forwarded";
     * forwarding it satisfies "recorded".
     */
    @Test
    fun normal_level_is_recorded_but_never_forwarded() {
        val m = WarningMonitor()
        val events = m.onDelivery(listOf(item(OVERHEAT, WarnLevel.NORMAL)), 0)

        assertEquals("NORMAL still produces an event for the flight record", 1, events.size)
        val e = events.single()
        assertNull("…but never a STATUSTEXT", e.mavSeverity)
        assertFalse(e.announce)
        assertFalse("not forwarded by design, not by the rate bound", e.rateLimited)
        assertEquals(LogEntry.SEV_INFO, e.recordSeverity)
    }

    /** DJI could not map its own code. That must be visible, not silently swallowed. */
    @Test
    fun unknown_level_is_forwarded_at_warning() {
        val m = WarningMonitor()
        val e = m.onDelivery(listOf(item(OVERHEAT, WarnLevel.UNKNOWN)), 0).single()

        assertEquals(WarnLevel.MAV_SEVERITY_WARNING, e.mavSeverity)
        assertTrue(e.announce)
    }

    // ── Change detection ─────────────────────────────────────────────────────

    @Test
    fun a_new_code_appears() {
        val m = WarningMonitor()
        val e = m.onDelivery(listOf(item(OVERHEAT, WarnLevel.WARNING, title = "Overheating")), 0)
            .single()

        assertEquals(WarnChange.APPEARED, e.change)
        assertEquals(OVERHEAT, e.warning.code)
        assertNull(e.previousLevel)
        assertEquals("DJI: Overheating", e.text)
    }

    /**
     * The same code at a new level is a change, and it carries **both** levels.
     *
     * The previous level is on the event rather than left to be reconstructed from an earlier log
     * line, for the reason `LogEntry.Field` gives: the interesting line is one of two hundred
     * thousand, and a reader should not have to scan backwards to learn that CAUTION just became
     * SERIOUS_WARNING.
     */
    @Test
    fun the_same_code_at_a_new_level_is_a_change() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.CAUTION, title = "Overheating")), 0)

        val e = m.onDelivery(
            listOf(item(OVERHEAT, WarnLevel.SERIOUS_WARNING, title = "Overheating")), 100,
        ).single()

        assertEquals(WarnChange.CHANGED, e.change)
        assertEquals(WarnLevel.CAUTION, e.previousLevel)
        assertEquals(WarnLevel.SERIOUS_WARNING, e.warning.level)
        assertEquals(WarnLevel.MAV_SEVERITY_CRITICAL, e.mavSeverity)
        assertEquals(
            "the level word is the news, so it is in the sentence",
            "DJI SERIOUS_WARNING: Overheating", e.text,
        )
    }

    /**
     * A code disappearing is announced, at INFO.
     *
     * Both halves matter. Announcing at all, because the operator's most recent information still
     * says the aircraft is overheating and silence does not retract it. At INFO, because a clear
     * that arrived at the item's own CRITICAL would raise QGC's red modal to say the emergency was
     * over — which is exactly how an operator learns to stop trusting the modal.
     */
    @Test
    fun a_code_disappearing_is_cleared_at_info() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.SERIOUS_WARNING, title = "Overheating")), 0)

        val e = m.onDelivery(WarnSource.DEVICE_HEALTH, emptyList(), 100).single()

        assertEquals(WarnChange.CLEARED, e.change)
        assertEquals("the cleared item is carried, as last seen", OVERHEAT, e.warning.code)
        assertEquals(WarnLevel.MAV_SEVERITY_INFO, e.mavSeverity)
        assertEquals(LogEntry.SEV_INFO, e.recordSeverity)
        assertTrue(e.announce)
        assertEquals("DJI cleared: Overheating", e.text)
    }

    /** A `NORMAL` entry disappearing was never announced, so its clear is not announced either. */
    @Test
    fun clearing_a_never_forwarded_level_is_not_forwarded() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.NORMAL)), 0)

        val e = m.onDelivery(WarnSource.DEVICE_HEALTH, emptyList(), 100).single()

        assertEquals(WarnChange.CLEARED, e.change)
        assertNull("QGC was never told it appeared; telling it about the clear is noise", e.mavSeverity)
        assertFalse(e.announce)
    }

    // ── Dedup ────────────────────────────────────────────────────────────────

    /**
     * **The property this whole module turns on.** DJI hands over the complete current list every
     * time anything in it moves, and is free to re-deliver an identical one at any rate it likes.
     * Twenty identical deliveries of a standing overheat produce one event, not twenty.
     */
    @Test
    fun repeated_identical_deliveries_produce_exactly_one_event() {
        val m = WarningMonitor()
        val picture = listOf(
            item(OVERHEAT, WarnLevel.WARNING),
            item(COMPASS, WarnLevel.CAUTION),
        )

        val first = m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 0)
        assertEquals(2, first.size)

        var later = 0
        for (i in 1..20) later += m.onDelivery(picture, i * 40L).size

        assertEquals("a standing warning is news exactly once", 0, later)
    }

    /**
     * DJI rewrites titles and descriptions as its localisation tables load. A re-worded warning at
     * the same level is not news, and treating it as news would re-announce a standing overheat
     * every time DJI's string table changed under us.
     */
    @Test
    fun rewording_at_the_same_level_is_not_a_change() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.WARNING, title = "Overheat")), 0)

        val events = m.onDelivery(
            listOf(
                item(OVERHEAT, WarnLevel.WARNING, title = "Aircraft overheating")
                    .copy(description = "quite different prose")
            ),
            100,
        )

        assertEquals(0, events.size)
    }

    /**
     * Two components raising the same code are two warnings.
     *
     * DJI reports per-component faults with one code — four ESCs, two IMUs, two compasses — so a
     * key of `code` alone would collapse "all four motors are overheating" into one warning and
     * hide three of them. It would also manufacture a fake `changed` every time DJI happened to
     * order its list differently.
     */
    @Test
    fun the_same_code_on_two_components_is_two_warnings() {
        val m = WarningMonitor()

        val events = m.onDelivery(
            listOf(
                item(OVERHEAT, WarnLevel.WARNING, comp = 1),
                item(OVERHEAT, WarnLevel.WARNING, comp = 2),
            ),
            0,
        )

        assertEquals(2, events.size)
        assertEquals(setOf(1, 2), events.map { it.warning.componentId }.toSet())
        assertTrue(events.all { it.change == WarnChange.APPEARED })

        // …and one of them clearing leaves the other standing, rather than clearing both.
        val next = m.onDelivery(listOf(item(OVERHEAT, WarnLevel.WARNING, comp = 1)), 100)
        assertEquals(1, next.size)
        assertEquals(WarnChange.CLEARED, next.single().change)
        assertEquals(2, next.single().warning.componentId)
    }

    /**
     * A duplicated key inside one delivery folds to one item.
     *
     * DJI hands us a `List`, not a `Map`, so nothing stops it repeating an entry. Without the fold
     * the picture would hold one copy while the delivery held two, and the *next* delivery would
     * produce a phantom clear for an item that never went away.
     */
    @Test
    fun a_duplicated_key_in_one_delivery_folds_to_one() {
        val m = WarningMonitor()

        val first = m.onDelivery(
            listOf(
                item(OVERHEAT, WarnLevel.CAUTION),
                item(OVERHEAT, WarnLevel.CAUTION),
            ),
            0,
        )
        assertEquals(1, first.size)

        val second = m.onDelivery(listOf(item(OVERHEAT, WarnLevel.CAUTION)), 100)
        assertEquals("no phantom clear", 0, second.size)
    }

    /** After a teardown we have no grounds for the picture, so the next delivery re-announces it. */
    @Test
    fun reset_makes_the_next_delivery_a_fresh_appearance() {
        val m = WarningMonitor()
        val picture = listOf(item(OVERHEAT, WarnLevel.WARNING))
        m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 0)
        assertEquals(0, m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 100).size)

        m.reset()

        val again = m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 200).single()
        assertEquals(WarnChange.APPEARED, again.change)
        assertTrue("and the rate bound starts with full credit too", again.announce)
    }

    // ── Rate bound ───────────────────────────────────────────────────────────

    /**
     * Six forwardable events go out in a window; the seventh does not.
     *
     * The diff already stops repeats, so reaching this needs a genuinely flapping aircraft. The
     * bound is what keeps that case merely noisy instead of drowning the link.
     */
    @Test
    fun the_wire_is_bounded_at_capacity_per_window() {
        val m = WarningMonitor()
        val picture = (1..RateBound.CAPACITY + 1).map { item("code-$it", WarnLevel.CAUTION) }

        val events = m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 0)

        assertEquals(RateBound.CAPACITY + 1, events.size)
        assertEquals(RateBound.CAPACITY, events.count { it.announce })
    }

    /**
     * A suppressed event is **still returned**, marked, so the flight record and logcat keep it.
     *
     * The bound protects the operator's screen and the link. It never protects them from the
     * evidence — see this class's note on the mutant that made this test necessary.
     */
    @Test
    fun suppressed_events_are_still_returned() {
        val m = WarningMonitor()
        val picture = (1..RateBound.CAPACITY + 2).map { item("code-$it", WarnLevel.CAUTION) }

        val events = m.onDelivery(WarnSource.DEVICE_HEALTH, picture, 0)

        assertEquals("nothing is dropped", RateBound.CAPACITY + 2, events.size)
        val suppressed = events.filter { !it.announce }
        assertEquals(2, suppressed.size)
        assertTrue(
            "and the record can tell 'the bound ate it' from 'this level is never sent'",
            suppressed.all { it.rateLimited && it.mavSeverity != null },
        )
    }

    /** The window slides: once it has passed, the credit is back. */
    @Test
    fun the_window_slides() {
        val m = WarningMonitor()
        val full = (1..RateBound.CAPACITY).map { item("code-$it", WarnLevel.CAUTION) }
        assertEquals(RateBound.CAPACITY, m.onDelivery(WarnSource.DEVICE_HEALTH, full, 0).count { it.announce })

        // Inside the window: no credit.
        val blocked = m.onDelivery(full + item("late", WarnLevel.CAUTION), RateBound.WINDOW_MS - 1)
        assertFalse(blocked.single().announce)

        // Past it: credit again.
        val allowed = m.onDelivery(
            full + item("late", WarnLevel.CAUTION) + item("later", WarnLevel.CAUTION),
            RateBound.WINDOW_MS,
        )
        assertTrue(allowed.single { it.warning.code == "later" }.announce)
    }

    /**
     * Chatter that is never forwarded must not spend the credit a real warning needs.
     *
     * Six `NORMAL` entries and six clears of `NORMAL` entries pass through the core without
     * touching the bound, so the six `CAUTION` appearances in the same delivery all reach QGC.
     */
    @Test
    fun events_that_are_never_forwarded_do_not_spend_credit() {
        val m = WarningMonitor()
        val normals = (1..RateBound.CAPACITY).map { item("normal-$it", WarnLevel.NORMAL) }
        m.onDelivery(WarnSource.DEVICE_HEALTH, normals, 0)

        // The NORMALs all clear (not forwarded), and six CAUTIONs appear, in one delivery.
        val cautions = (1..RateBound.CAPACITY).map { item("caution-$it", WarnLevel.CAUTION) }
        val events = m.onDelivery(WarnSource.DEVICE_HEALTH, cautions, 10)

        assertEquals(RateBound.CAPACITY * 2, events.size)
        assertEquals(
            "every real warning got through",
            RateBound.CAPACITY, events.count { it.announce },
        )
    }

    // ── Ordering ─────────────────────────────────────────────────────────────

    /**
     * Most serious first, clears last — and the delivery order deliberately disagrees with both,
     * so a mutant that returns DJI's order fails here.
     */
    @Test
    fun events_come_back_most_serious_first_with_clears_last() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item("going", WarnLevel.CAUTION)), 0)

        val events = m.onDelivery(
            listOf(
                item("quiet", WarnLevel.NOTICE),
                item("loud", WarnLevel.SERIOUS_WARNING),
                item("middling", WarnLevel.CAUTION),
            ),
            100,
        )

        assertEquals(
            listOf("loud", "middling", "quiet", "going"),
            events.map { it.warning.code },
        )
        assertEquals(WarnChange.CLEARED, events.last().change)
    }

    /**
     * **The safety property the ordering exists for.** With one token left, the token goes to the
     * most serious change, not to whichever DJI listed first.
     *
     * The delivery lists the NOTICE ahead of the SERIOUS_WARNING precisely so that a mutant which
     * spends tokens in delivery order announces the wrong one.
     */
    @Test
    fun the_last_token_goes_to_the_most_serious_change() {
        val m = WarningMonitor()
        // Spend all but one token.
        val drain = (1 until RateBound.CAPACITY).map { item("drain-$it", WarnLevel.CAUTION) }
        assertEquals(RateBound.CAPACITY - 1, m.onDelivery(WarnSource.DEVICE_HEALTH, drain, 0).count { it.announce })

        val events = m.onDelivery(
            drain + listOf(
                item("chatter", WarnLevel.NOTICE),
                item("serious", WarnLevel.SERIOUS_WARNING),
            ),
            100,
        )

        assertEquals(2, events.size)
        assertTrue(
            "the serious one reaches the operator",
            events.single { it.warning.code == "serious" }.announce,
        )
        assertFalse(
            "the chatter is the one that waits",
            events.single { it.warning.code == "chatter" }.announce,
        )
    }

    // ── Snapshot ─────────────────────────────────────────────────────────────

    /** The picture tracks the last delivery exactly — it is what the diff is taken against. */
    @Test
    fun the_snapshot_is_the_last_delivered_picture() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.WARNING), item(COMPASS, WarnLevel.NOTICE)), 0)
        assertEquals(setOf(OVERHEAT, COMPASS), m.snapshot().map { it.code }.toSet())

        m.onDelivery(listOf(item(COMPASS, WarnLevel.NOTICE)), 100)
        assertEquals(listOf(COMPASS), m.snapshot().map { it.code })

        m.onDelivery(WarnSource.DEVICE_HEALTH, emptyList(), 200)
        assertTrue(m.snapshot().isEmpty())
    }

    // ── Retraction ───────────────────────────────────────────────────────────

    /**
     * A warning that cools back to `NORMAL` is retracted on the wire, not left standing.
     *
     * DJI keeps a recovered code **in the list** at `NORMAL` rather than dropping it, so this is
     * how the overheat most plausibly ends. Before this fix the de-escalation mapped to "not
     * forwarded" and said nothing at all — leaving the operator's newest message, and QGC's red
     * modal, still asserting the aircraft was overheating long after it had cooled. That is the
     * exact asymmetry `cleared` was written to avoid, applied to the case that reaches it by a
     * different door. Found by review, 2026-07-27.
     *
     * The retraction goes out at `INFO` for the same reason a clear does: good news must not
     * arrive looking like a new emergency.
     *
     * | mutation | tests that fail | measured |
     * |---|---|---|
     * | de-escalation returns null again (no retraction) | this test (announce, severity) | 1 |
     * | retraction sent at the *old* level's severity | this test (severity assert) | 1 |
     *
     * The guard's other branch — a de-escalation whose *previous* level was also not
     * forwardable — is deliberately unreachable: `NORMAL` is the only unforwarded level, and a
     * `NORMAL` to `NORMAL` transition is not a change, so the diff never emits one. The condition
     * stays as a cheap statement of the rule rather than as a branch anything can exercise.
     * | retraction text uses `changed` (says "NORMAL") | this test (text assert) | 1 |
     */
    @Test
    fun a_warning_that_cools_to_normal_is_retracted_on_the_wire() {
        val m = WarningMonitor()
        m.onDelivery(listOf(item(OVERHEAT, WarnLevel.WARNING, title = "Aircraft overheating")), 0)

        val cooled = m.onDelivery(
            listOf(item(OVERHEAT, WarnLevel.NORMAL, title = "Aircraft overheating")),
            1_000,
        )

        val e = cooled.single()
        assertEquals(WarnChange.CHANGED, e.change)
        assertTrue("the operator must be told it is over", e.announce)
        assertEquals(WarnLevel.MAV_SEVERITY_INFO, e.mavSeverity)
        assertTrue("reads as a clear, not as a NORMAL-level report", e.text.contains("cleared"))
        assertFalse("and it must not say NORMAL in fifty bytes", e.text.contains("NORMAL"))
    }

    // ── Unknown level names ──────────────────────────────────────────────────

    /**
     * A `WarningLevel` name we do not recognise reads as [WarnLevel.UNKNOWN], never as fine.
     *
     * This is the package's single most load-bearing honesty property and it had no test at all:
     * the whole suite passed with the fallback mutated to `NORMAL`, which would make a future
     * SDK's new level — or any name mismatch — arrive as "everything is fine" and be dropped
     * silently on the wire. The port passes `WarningLevel.name` rather than the enum precisely so
     * that an unrecognised value lands here instead of throwing inside a DJI callback.
     * Found by review, 2026-07-27.
     *
     * | mutation | tests that fail | measured |
     * |---|---|---|
     * | `?: UNKNOWN` becomes `?: NORMAL` | this test | 1 |
     * | `?: UNKNOWN` becomes `?: SERIOUS_WARNING` | this test | 1 |
     * | name match made case-insensitive/prefix | this test (`WARNIN`, `warning`) | 1 |
     */
    @Test
    fun an_unrecognised_level_name_is_unknown_and_never_normal() {
        // Every name DJI has today still resolves exactly.
        for (level in WarnLevel.entries) {
            assertEquals(level, WarnLevel.ofName(level.name))
        }

        // And everything else lands on UNKNOWN — which is forwarded, not dropped.
        for (name in listOf(null, "", "CATASTROPHIC", "warning", "WARNIN", "NORMAL ", "0")) {
            assertEquals(
                "an unrecognised level must never read as fine: $name",
                WarnLevel.UNKNOWN,
                WarnLevel.ofName(name),
            )
        }
    }
}
