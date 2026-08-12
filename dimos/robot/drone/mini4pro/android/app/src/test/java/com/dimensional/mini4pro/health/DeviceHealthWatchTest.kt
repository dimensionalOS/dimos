package com.dimensional.mini4pro.health

import com.dimensional.mini4pro.warn.RateBound
import com.dimensional.mini4pro.warn.WarnChange
import com.dimensional.mini4pro.warn.WarnEvent
import com.dimensional.mini4pro.warn.WarnLevel
import com.dimensional.mini4pro.warn.WarnSource
import com.dimensional.mini4pro.warn.Warning
import com.dimensional.mini4pro.warn.WarningBus
import com.dimensional.mini4pro.warn.WarningMonitor
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The shell: the deferred-planting cure for the silent pre-registration trap, the priming read,
 * and the fan-out to the three sinks with each one contained.
 *
 * The decisions all live in `WarningMonitor` and are tested there. What is tested here is the
 * wiring, and specifically the two ways this layer can fail **silently** — a listener that was
 * never really planted, and a warning that reached one sink but not the others.
 *
 * Mutation-checked 2026-07-26, one breakage at a time, failing tests counted across the three
 * health suites and the code reverted after each — measured counts, not estimates. The wiring
 * mutants:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `ensure()` plants without checking `unavailableReason` (the pre-registration no-op) | 1 |
 *  | `ensure()` not idempotent (a listener per 200 ms tick) | 1 |
 *  | `ensure()` marks itself planted before the check (gives up after the first refusal) | 1 |
 *  | the priming `current()` read dropped (a standing warning stays invisible) | 4 |
 *  | `current()` read before `listen()` (a change in the gap is lost) | 2 |
 *  | record/log skipped for events the wire does not get | 1 |
 *  | a throwing sink aborts the remaining sinks (the `record` try/catch removed) | 1 |
 *  | `stop()` does not reset the picture (a re-subscribe re-announces nothing) | 1 |
 *
 * The **priming** mutant is the one worth keeping notes on. It scores 4 here, but every one of
 * those tests had to be written to *not* deliver anything afterwards — a suite that always follows
 * `ensure()` with a delivery scores it 0, because a delivery is exactly what priming substitutes
 * for. The real-world case is the one with no delivery at all: an overheat already standing when
 * the bridge starts may never change again, which is precisely how this package came to exist.
 */
class DeviceHealthWatchTest {

    private class FakePort(
        var unavailable: String? = "SDK_NOT_REGISTERED",
    ) : DeviceHealthPort {
        var listenCalls = 0
        var cancelCalls = 0
        var currentCalls = 0

        /** What `current()` returns, and when it was read relative to `listen()`. */
        var standing: List<Warning>? = null
        var listenedBeforeCurrent: Boolean? = null

        var deliver: ((List<Warning>) -> Unit)? = null

        override fun unavailableReason(): String? = unavailable

        /** Set to make the plant fail, as a DJI singleton in a bad state would. */
        var listenSucceeds = true

        override fun listen(onDelivery: (List<Warning>) -> Unit): Boolean {
            listenCalls++
            if (!listenSucceeds) return false
            deliver = onDelivery
            return true
        }

        override fun cancelListen() {
            cancelCalls++
            deliver = null
        }

        override fun current(): List<Warning>? {
            currentCalls++
            if (listenedBeforeCurrent == null) listenedBeforeCurrent = listenCalls > 0
            return standing
        }
    }

    private class Sinks {
        val sent = mutableListOf<Pair<String, Int>>()
        val recorded = mutableListOf<WarnEvent>()
        val logged = mutableListOf<WarnEvent>()
        val published = mutableListOf<WarnEvent>()
        val screened = mutableListOf<WarnEvent>()
        val notes = mutableListOf<String>()
        var recordThrows = false
        var sendThrows = false
        var logThrows = false
    }

    private fun watch(port: FakePort, sinks: Sinks, now: () -> Long = { 0L }) = DeviceHealthWatch(
        port = port,
        warnings = bus(sinks, now),
        note = { sinks.notes += it },
    )

    /**
     * The owner the watch now feeds, with this file's fakes on its five sinks.
     *
     * The wiring under test moved on 2026-07-30: this class plants a listener and hands pictures
     * to `warn/WarningBus`, which owns the diff, the severities and the fan-out for **every**
     * source. The properties below are unchanged — a delivery reaches every sink, a throwing sink
     * does not take the others with it — because they were always the bus's properties; what
     * changed is that the wind warning now gets them too, for free.
     */
    private fun bus(sinks: Sinks, now: () -> Long) = WarningBus(
        qgc = { text, severity ->
            if (sinks.sendThrows) throw IllegalStateException("send")
            sinks.sent += text to severity
        },
        screen = { sinks.screened += it },
        record = {
            if (sinks.recordThrows) throw IllegalStateException("record")
            sinks.recorded += it
        },
        bus = { sinks.published += it },
        log = {
            if (sinks.logThrows) throw IllegalStateException("log")
            sinks.logged += it
        },
        note = { sinks.notes += it },
        nowMs = now,
    )

    private fun item(code: String, level: WarnLevel, title: String = "t-$code") = Warning(
        source = WarnSource.DEVICE_HEALTH,
        code = code,
        state = level.name,
        level = level,
        title = title,
        componentId = 0,
        sensorIndex = 0,
    )

    // ── The silent pre-registration trap ─────────────────────────────────────

    /**
     * **The trap.** A DJI listener planted before MSDK registration completes silently never
     * delivers — no error, no exception, no callback — and the symptom is indistinguishable from
     * an aircraft that has nothing to report. The bridge comes up about a second before
     * registration on a fresh launch, so the obvious place to subscribe is always too early.
     *
     * The cure: refuse while the SDK says it is not ready, and let `Bridge.tick()` retry every
     * 200 ms until it takes.
     */
    @Test
    fun nothing_is_planted_before_the_sdk_is_registered() {
        val port = FakePort(unavailable = "SDK_NOT_REGISTERED")
        val sinks = Sinks()
        val w = watch(port, sinks)

        // Five seconds of Bridge ticks before registration lands.
        repeat(25) { assertFalse(w.ensure()) }

        assertEquals("no listener may be planted early", 0, port.listenCalls)
        assertFalse(w.isListening)

        // Registration completes; the very next tick takes.
        port.unavailable = null
        assertTrue(w.ensure())
        assertEquals(1, port.listenCalls)
        assertTrue(w.isListening)
    }

    /**
     * `ensure()` is called five times a second for the life of a session. It must plant once.
     *
     * A listener per tick would be a leak measured in thousands, and every one of them would
     * deliver — turning the diff into nonsense as the same picture arrived N times.
     */
    @Test
    fun ensure_is_idempotent() {
        val port = FakePort(unavailable = null)
        val w = watch(port, Sinks())

        repeat(100) { assertTrue(w.ensure()) }

        assertEquals(1, port.listenCalls)
        assertEquals(1, port.currentCalls)
    }

    // ── Priming ──────────────────────────────────────────────────────────────

    /**
     * A warning that was **already standing** when we subscribed is announced without DJI ever
     * delivering anything.
     *
     * This is the real-world case that created the package: an overheat that has been showing for
     * an hour is not going to change again just because we started listening. DJI's own sample
     * pairs `getCurrentDJIDeviceHealthInfos()` with the listener for exactly this reason.
     */
    @Test
    fun a_standing_warning_is_announced_at_subscribe_time() {
        val port = FakePort(unavailable = null)
        port.standing = listOf(item("0x1600A0", WarnLevel.WARNING, "Aircraft overheating"))
        val sinks = Sinks()

        watch(port, sinks).ensure()

        assertEquals(1, sinks.sent.size)
        assertEquals("DJI: Aircraft overheating" to WarnLevel.MAV_SEVERITY_ERROR, sinks.sent.single())
        assertEquals(1, sinks.recorded.size)
        assertEquals(1, sinks.logged.size)
    }

    /**
     * The listener is planted **before** the priming read, so a change landing in the gap between
     * the two is caught by the listener rather than falling down it. A warning present in both is
     * deduped by the diff — which is what the diff is for.
     */
    @Test
    fun the_listener_is_planted_before_the_priming_read() {
        val port = FakePort(unavailable = null)
        port.standing = listOf(item("a", WarnLevel.CAUTION))
        val sinks = Sinks()

        watch(port, sinks).ensure()

        assertEquals(true, port.listenedBeforeCurrent)
        // And the picture the listener then re-delivers is not announced twice.
        port.deliver!!(listOf(item("a", WarnLevel.CAUTION)))
        assertEquals(1, sinks.sent.size)
    }

    // ── Fan-out ──────────────────────────────────────────────────────────────

    @Test
    fun a_delivery_reaches_every_sink() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()

        port.deliver!!(listOf(item("0x1600A0", WarnLevel.SERIOUS_WARNING, "Overheat")))

        assertEquals("DJI: Overheat" to WarnLevel.MAV_SEVERITY_CRITICAL, sinks.sent.single())
        assertEquals(WarnChange.APPEARED, sinks.recorded.single().change)
        assertEquals("Overheat", sinks.logged.single().warning.title)
    }

    /**
     * An event the wire never gets is still recorded and logged — both the "this level is never
     * forwarded" case and the "the rate bound suppressed it" case.
     *
     * The record is the sink whose whole purpose is surviving the session. A warning that reached
     * nobody *and* left no trace is the exact failure this package was built to end.
     */
    @Test
    fun events_the_wire_does_not_get_are_still_recorded_and_logged() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()

        // NORMAL: never forwarded by design.
        port.deliver!!(listOf(item("normal", WarnLevel.NORMAL)))
        assertTrue(sinks.sent.isEmpty())
        assertEquals(1, sinks.recorded.size)
        assertEquals(1, sinks.logged.size)

        // Then more forwardable changes than the bound allows in one window.
        val flood = (1..RateBound.CAPACITY + 3).map { item("flap-$it", WarnLevel.CAUTION) }
        port.deliver!!(flood)

        assertEquals("the wire is bounded", RateBound.CAPACITY, sinks.sent.size)
        assertEquals(
            "the record is not — the NORMAL, its clear, and every flood item",
            1 + 1 + flood.size, sinks.recorded.size,
        )
        assertEquals(sinks.recorded.size, sinks.logged.size)
    }

    /** DJI repeating itself costs nothing anywhere — not one line, not one datagram. */
    @Test
    fun repeated_deliveries_reach_no_sink() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()
        val picture = listOf(item("a", WarnLevel.WARNING), item("b", WarnLevel.CAUTION))
        port.deliver!!(picture)
        val after = Triple(sinks.sent.size, sinks.recorded.size, sinks.logged.size)

        repeat(50) { port.deliver!!(picture) }

        assertEquals(after, Triple(sinks.sent.size, sinks.recorded.size, sinks.logged.size))
    }

    // ── Containment ──────────────────────────────────────────────────────────

    /**
     * A failing sink must not take the others with it, and must not throw.
     *
     * DJI delivers on the Android main thread. An exception escaping this callback would kill the
     * process — which this project has already done once, by letting a DJI callback reach a UDP
     * send on the main thread. Nothing about reporting a warning may become a bigger problem than
     * the warning.
     */
    @Test
    fun a_throwing_recorder_does_not_stop_the_statustext() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()
        sinks.recordThrows = true

        port.deliver!!(listOf(item("a", WarnLevel.WARNING, "Overheat")))

        assertEquals("DJI: Overheat", sinks.sent.single().first)
        assertEquals(1, sinks.logged.size)
        assertTrue(sinks.notes.any { it.contains("recorder") })
    }

    @Test
    fun a_throwing_send_does_not_stop_the_record() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()
        sinks.sendThrows = true
        sinks.logThrows = true

        port.deliver!!(listOf(item("a", WarnLevel.WARNING), item("b", WarnLevel.CAUTION)))

        assertEquals("both events survived to the record", 2, sinks.recorded.size)
        assertTrue(sinks.notes.any { it.contains("STATUSTEXT") })
    }

    // ── Teardown ─────────────────────────────────────────────────────────────

    /**
     * `stop()` drops the subscription **and** this source's picture, so a later re-subscribe
     * re-announces everything DJI is currently reporting rather than silently withholding it.
     *
     * Keeping the picture across a teardown would be a claim about *now* that we no longer have
     * grounds for — and would hide exactly the standing overheat this package exists to surface.
     *
     * **The teardown is an empty delivery, not a reset of the bus**, and that difference is
     * load-bearing since the bus became every source's: resetting would silently drop the wind
     * source's picture too, and the operator would get no retraction for either. An empty picture
     * clears exactly this source's warnings, through the same diff as everything else, so a
     * standing overheat is properly *cleared* on the way out rather than vanishing.
     */
    @Test
    fun stop_drops_the_subscription_and_the_picture() {
        val port = FakePort(unavailable = null)
        val sinks = Sinks()
        val w = watch(port, sinks)
        w.ensure()
        port.deliver!!(listOf(item("a", WarnLevel.WARNING, "Overheat")))
        assertEquals(1, sinks.sent.size)

        w.stop()
        assertEquals(1, port.cancelCalls)
        assertFalse(w.isListening)
        assertTrue(w.snapshot().isEmpty())
        // The retraction the operator is owed: silence does not withdraw the last message.
        assertEquals(WarnChange.CLEARED, sinks.recorded.last().change)
        assertEquals("DJI cleared: Overheat" to WarnLevel.MAV_SEVERITY_INFO, sinks.sent.last())

        // Re-subscribing re-announces the standing warning.
        port.standing = listOf(item("a", WarnLevel.WARNING, "Overheat"))
        assertTrue(w.ensure())
        assertEquals(3, sinks.sent.size)
        assertEquals(WarnChange.APPEARED, sinks.recorded.last().change)
    }

    /** Teardown of something that was never planted is a no-op, not a crash. */
    @Test
    fun stop_before_ensure_is_harmless() {
        val port = FakePort(unavailable = "SDK_NOT_REGISTERED")
        val w = watch(port, Sinks())

        w.stop()

        assertEquals(0, port.cancelCalls)
        assertFalse(w.isListening)
    }

    /**
     * A plant that the SDK refuses must stay retryable.
     *
     * The failure this pins is the one that would have been permanent and silent: if `planted`
     * were committed before the SDK accepted the listener, a single failed plant would suppress
     * every subsequent 200 ms retry and health would be dead for the whole session with nothing
     * on the wire and nothing in the log to say so — which is precisely the silent-blindness
     * failure this package exists to end. Found by review, 2026-07-27.
     *
     * | mutation | tests that fail | measured |
     * |---|---|---|
     * | `planted = true` moved back before `port.listen` | this test (retry never re-attempts) | 1 |
     * | `ensure` returns true on a refused plant | this test (first assert) | 1 |
     * | `listen`'s failure rethrown instead of reported | this test (exception escapes) | 1 |
     */
    @Test
    fun a_refused_plant_is_retried_rather_than_remembered_as_done() {
        val port = FakePort(unavailable = null)
        port.listenSucceeds = false
        val w = watch(port, Sinks())

        assertFalse("a refused plant is not a live listener", w.ensure())
        assertFalse(w.isListening)
        assertEquals(1, port.listenCalls)

        // The tick comes round again 200 ms later, exactly as it does for the RC stick feed.
        assertFalse(w.ensure())
        assertEquals("every tick retries a refused plant", 2, port.listenCalls)

        // And when the SDK is ready, the very next tick takes it.
        port.listenSucceeds = true

        assertTrue(w.ensure())
        assertTrue(w.isListening)
        assertEquals(3, port.listenCalls)

        // Once planted, the flag does its job: no further SDK calls.
        assertTrue(w.ensure())
        assertEquals(3, port.listenCalls)
    }
}
