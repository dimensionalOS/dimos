package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.Timeout
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

/**
 * [ZenohPublisher] — the bounded queue and the one thread that stand between a Zenoh session and
 * an aircraft.
 *
 * **This is the file that matters if this feature ever hurts someone.** Everything else in
 * `zenoh/` is arithmetic that can be wrong; this is the part that can be *slow*, and slow is the
 * failure mode that reaches the setpoint loop. The three properties it exists to pin, and each is
 * mutation-checked below:
 *
 *  - **a full queue drops rather than blocks**
 *  - **a Zenoh failure never propagates into the caller**
 *  - **the publisher stops when the session is gone, rather than accumulating a backlog to
 *    replay at whoever reconnects**
 *
 * ## What is covered here, and what is only covered by the seam
 *
 * Covered: every state transition, the drop and discard accounting, the backoff, the containment
 * of a throwing transport, and — the one that needs a real thread rather than [pumpOnce] — that a
 * transport which blocks *forever* inside `put` cannot make [offer] block, which is asserted from
 * a second thread against a wall clock.
 *
 * **Not covered, and not coverable here:** that `libzenoh_jni.so` loads in a process holding MSDK
 * and OpenCV natives; that zenoh accepts the key expressions we build; that
 * `CongestionControl.BLOCK` blocks and `DROP` drops; that a session reconnects to a router that
 * came back. Those are claims about the library and the device, and only a real session on the
 * phone settles them. `ZenohSink`'s KDoc lists them; `ZenohKotlinSink` is the implementation.
 *
 * ## Mutation-checked 2026-07-27
 *
 * One breakage at a time, whole suite run, reverted after each, by `tmp/mutate.py`. **Counts are
 * failing tests across the whole 1994-test suite, measured, not estimated.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `offer` uses `queue.put` (blocks) instead of `queue.offer` | 2 |
 *  | a full queue silently returns true without enqueueing | 2 |
 *  | drops are not counted | 2 |
 *  | drops are not counted per channel | 1 |
 *  | `offer` publishes inline instead of enqueueing — no queue, no thread | **10** |
 *  | a throwing `put` is not caught in the worker | 2 |
 *  | a failed `factory.open` is not caught | 3 |
 *  | a failed open does not back off (retries on every pump) | 1 |
 *  | the backoff does not grow | 1 |
 *  | the backoff has no ceiling | 1 |
 *  | with no session, the queue is left full instead of discarded | 2 |
 *  | discards are counted as drops | 2 |
 *  | a broken session is not closed after a failed put | 2 |
 *  | a reopened session keeps the old `declared` set | **0 — equivalent, see below** |
 *  | a retired channel (no key expression) is published anyway | 1 |
 *  | QoS is not taken from the channel — everything declared DEFAULT | 1 |
 *  | `offer` accepted while stopped | 1 |
 *  | `stop` does not close the session | 1 |
 *  | `stop` drains the queue instead of abandoning it | **0 — equivalent, see below** |
 *
 * ### The three findings in that table
 *
 * **`offer` publishing inline kills 10 — by far the largest number here, and it is the mutant that
 * best describes the design.** The thread is not an optimisation, it is the safety property.
 * Removing it turns every one of the transport's failure modes into the caller's, and ten separate
 * assertions notice.
 *
 * **`queue.put` for `queue.offer` kills only 2, and that is the shape of the hazard rather than a
 * weakness in the suite.** A blocking enqueue is invisible until the consumer is slower than the
 * producer — the condition that never occurs on a bench and always occurs on the flight where a
 * subscriber wedged a `NEVER_DROP` channel. It also does not *fail* by default: **before the
 * `Timeout` rule above existed, this mutation hung the suite for twenty minutes instead of failing
 * it.** That is why the rule is there and why it is documented as load-bearing.
 *
 * **Two mutants are alive, and both are equivalent rather than uncaught.**
 *
 *  - *A reopened session keeps the old `declared` set.* `closeSink` already clears it, and every
 *    path that replaces a session goes through `closeSink` — a failed put, a stop, a broken
 *    session. So the reset inside `ensureSink` is unreachable defence and no behaviour can
 *    distinguish it. It stays, deliberately: the invariant it protects ("a fresh session has
 *    declared nothing") is one an edit could break by opening without closing, and a test cannot
 *    guard against a path that does not exist yet.
 *  - *`stop` drains the queue instead of abandoning it.* `pumpOnce`'s publish loop returns early
 *    once `running` is false, which `stop` sets first — so the added drain publishes nothing and
 *    the two are observationally identical. The *intent* still matters and is stated in `stop`'s
 *    KDoc: a stop that waited out a `NEVER_DROP` publisher would put the wedge back on the
 *    caller's thread.
 */
class ZenohPublisherTest {

    /**
     * **Every test in this file has twenty seconds, and that is a load-bearing rule rather than
     * hygiene.**
     *
     * The property under test is *"this never blocks"*, so the natural failure of the code under
     * test is a thread that never returns — and a JUnit assertion cannot fire on a thread that
     * never reaches it. Without this rule the first mutation in the table below (`queue.put` for
     * `queue.offer`) does not fail the suite; it **hangs** it, indefinitely, with no output. That
     * was measured on 2026-07-27 by trying it: the sweep produced nothing for twenty minutes and
     * had to be killed.
     *
     * A hang is a worse outcome than a failure in a way that matters beyond tidiness. On a
     * developer's machine it is an afternoon; in CI it is a timeout somebody reads as
     * infrastructure flakiness. **A test for a liveness property has to have a deadline**, or it
     * cannot distinguish "correct" from "still going".
     */
    @get:Rule
    val timeout: Timeout = Timeout.seconds(20)

    private companion object {
        val CONFIG = ZenohConfig(endpoint = "tcp/10.55.1.50:7447")
        val PAYLOAD = byteArrayOf(1, 2, 3, 4)
    }

    /** A transport that records everything and can be told to misbehave on demand. */
    private class FakeSink : ZenohSink {
        val declared = ArrayList<Pair<String, ZenohQos>>()
        val put = ArrayList<Pair<String, ByteArray>>()
        var closed = 0

        /** Throw from the next [put] call, this many times. */
        var throwOnPut = 0

        /** Block inside [put] until released. */
        var gate: CountDownLatch? = null

        override fun declare(key: String, qos: ZenohQos) {
            declared += key to qos
        }

        override fun put(key: String, payload: ByteArray) {
            gate?.await()
            if (throwOnPut > 0) {
                throwOnPut--
                throw IllegalStateException("transport is unhappy")
            }
            put += key to payload
        }

        override fun close() {
            closed++
        }
    }

    private class FakeFactory(var sink: FakeSink? = FakeSink()) : ZenohSinkFactory {
        val opened = ArrayList<ZenohConfig>()

        /** Throw from the next [open] call, this many times. */
        var throwOnOpen = 0

        override fun open(config: ZenohConfig): ZenohSink {
            opened += config
            if (throwOnOpen > 0) {
                throwOnOpen--
                throw java.io.IOException("no route to the router")
            }
            return sink ?: throw IllegalStateException("no sink configured")
        }
    }

    private var clock = 0L

    private fun publisher(
        factory: ZenohSinkFactory,
        settings: ZenohPublisher.Settings = ZenohPublisher.Settings(),
        onPhase: (ZenohPublisher.Phase, String) -> Unit = { _, _ -> },
    ) = ZenohPublisher(CONFIG, factory, settings, nowMs = { clock }, onPhase = onPhase)

    // ── 1. the happy path, so the failures below mean something ───────────────

    @Test
    fun `an offered message reaches the transport on the right key with the right QoS`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        p.start(startThread = false)
        assertTrue(p.offer(ZenohChannel.ODOM, PAYLOAD))
        p.pumpOnce()

        val sink = factory.sink!!
        assertEquals(
            listOf("dimos/drone/odom/nav_msgs.Odometry" to ZenohQos.DEFAULT),
            sink.declared,
        )
        assertEquals(1, sink.put.size)
        assertEquals("dimos/drone/odom/nav_msgs.Odometry", sink.put[0].first)
        assertTrue(PAYLOAD.contentEquals(sink.put[0].second))
        assertEquals(1L, p.counters().published)
        assertEquals(ZenohPublisher.Phase.PUBLISHING, p.counters().phase)
    }

    /**
     * `status` and `warnings` are reliable + **block**, and they are the only channels here that
     * are. Getting this wrong is silent in both directions: `NEVER_DROP` on everything wedges the
     * bus behind a slow subscriber, and *default* on either of these two loses something said
     * once — a refusal an operator needed to read, or a DJI warning whose diff will never repeat
     * it (`warn/WarningMonitor`).
     */
    @Test
    fun `the said-once channels are declared NEVER_DROP and the telemetry channels are not`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        p.start(startThread = false)
        for (ch in ZenohChannel.PUBLISHED) p.offer(ch, PAYLOAD)
        p.pumpOnce()

        val byKey = factory.sink!!.declared.toMap()
        val blocking = listOf(ZenohChannel.STATUS, ZenohChannel.WARNINGS)
        for (ch in blocking) assertEquals("$ch must block", ZenohQos.NEVER_DROP, byKey[ch.keyOrNull()!!])
        for (ch in ZenohChannel.PUBLISHED.filter { it !in blocking }) {
            assertEquals("$ch must not block", ZenohQos.DEFAULT, byKey[ch.keyOrNull()!!])
        }
    }

    /** Declared once per session, not once per message. */
    @Test
    fun `a key is declared once however many messages go on it`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        p.start(startThread = false)
        repeat(20) { p.offer(ZenohChannel.IMU, PAYLOAD) }
        p.pumpOnce()
        assertEquals(1, factory.sink!!.declared.size)
        assertEquals(20, factory.sink!!.put.size)
    }

    /** A retired channel has no key expression and must not reach a bus even if something asks. */
    @Test
    fun `a channel with no key expression is refused at the door`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        p.start(startThread = false)
        assertNull("the retired gimbal channel must have no key", ZenohChannel.GIMBAL.keyOrNull())
        assertFalse(p.offer(ZenohChannel.GIMBAL, PAYLOAD))
        p.pumpOnce()
        assertTrue(factory.sink!!.put.isEmpty())
        // Not counted as a drop either: nothing was lost, there was nowhere for it to go.
        assertEquals(0L, p.counters().dropped)
    }

    // ── 2. a full queue drops rather than blocks ──────────────────────────────

    @Test
    fun `a full queue drops and counts, and never refuses to return`() {
        val factory = FakeFactory()
        val p = publisher(factory, ZenohPublisher.Settings(queueCapacity = 4))
        p.start(startThread = false)

        // Nothing is pumped, so the queue fills and stays full.
        assertTrue(p.offer(ZenohChannel.POSE, PAYLOAD))
        assertTrue(p.offer(ZenohChannel.POSE, PAYLOAD))
        assertTrue(p.offer(ZenohChannel.POSE, PAYLOAD))
        assertTrue(p.offer(ZenohChannel.POSE, PAYLOAD))
        assertFalse("the fifth must be dropped", p.offer(ZenohChannel.POSE, PAYLOAD))
        assertFalse(p.offer(ZenohChannel.ODOM, PAYLOAD))

        assertEquals(2L, p.counters().dropped)
        assertEquals(1L, p.dropsByChannel()[ZenohChannel.POSE])
        assertEquals(1L, p.dropsByChannel()[ZenohChannel.ODOM])
        // Nothing was lost from the queue itself: the four that fitted still go out.
        p.pumpOnce()
        assertEquals(4, factory.sink!!.put.size)
    }

    /**
     * **The property, measured against a wall clock from another thread.**
     *
     * The transport is held inside `put` and never released. If [ZenohPublisher.offer] enqueued
     * with `put` instead of `offer`, or published inline, a caller would block here for the whole
     * test — which is exactly what a `NEVER_DROP` channel behind a stalled subscriber does on a
     * real bus.
     *
     * The caller is the thing under test, not the publisher: what is asserted is that a producer
     * — standing in for the guided engine's 10 Hz thread — completes a burst of offers in
     * bounded time while the transport is wedged.
     */
    @Test
    fun `a wedged transport cannot block a caller`() {
        val factory = FakeFactory()
        val gate = CountDownLatch(1)
        factory.sink!!.gate = gate
        val p = publisher(factory, ZenohPublisher.Settings(queueCapacity = 8))
        p.start()
        try {
            // Let the worker reach `put` and wedge there.
            p.offer(ZenohChannel.STATUS, PAYLOAD)
            Thread.sleep(50)

            val done = CountDownLatch(1)
            val offered = AtomicInteger()
            // A daemon, so that a mutation which makes `offer` block cannot keep the test JVM
            // alive after the assertion below has already failed. The class `Timeout` rule is the
            // other half of the same guard.
            Thread {
                repeat(200) { if (p.offer(ZenohChannel.STATUS, PAYLOAD)) offered.incrementAndGet() }
                done.countDown()
            }.apply { isDaemon = true }.start()

            assertTrue(
                "200 offers against a wedged transport must complete in well under a second",
                done.await(2, TimeUnit.SECONDS),
            )
            // Most were dropped — the queue is 8 deep and nothing is draining. That is the
            // correct outcome and the whole point: losing telemetry beats blocking a caller.
            assertTrue("some must have been dropped", p.counters().dropped > 0)
            assertTrue("the queue must not have grown", offered.get() <= 8)
        } finally {
            gate.countDown()
            p.stop()
        }
    }

    // ── 3. a transport failure never reaches the caller ───────────────────────

    @Test
    fun `a transport that throws on every put is contained and counted`() {
        val factory = FakeFactory()
        factory.sink!!.throwOnPut = 99
        val p = publisher(factory)
        p.start(startThread = false)
        // The offer itself must succeed: the caller cannot know the transport is broken and must
        // not be told by an exception.
        assertTrue(p.offer(ZenohChannel.MODE, PAYLOAD))
        p.pumpOnce()

        assertEquals(1L, p.counters().failures)
        assertEquals(0L, p.counters().published)
        assertNotNull(p.counters().lastError)
        // The session is condemned, not the message: putting the rest of a batch onto a dead
        // session is the same exception several hundred more times.
        assertEquals(1, factory.sink!!.closed)
        assertEquals(ZenohPublisher.Phase.CONNECTING, p.counters().phase)
    }

    @Test
    fun `a factory that cannot open is contained, and the caller never learns`() {
        val factory = FakeFactory()
        factory.throwOnOpen = 99
        val p = publisher(factory)
        p.start(startThread = false)
        assertTrue(p.offer(ZenohChannel.BATTERY, PAYLOAD))
        p.pumpOnce()

        assertEquals(ZenohPublisher.Phase.CONNECTING, p.counters().phase)
        assertEquals(1L, p.counters().failures)
        assertEquals(1L, p.counters().discarded)
        assertEquals(0L, p.counters().dropped)
    }

    /**
     * The distinction the status screen is built on: **`dropped` is a full queue and `discarded`
     * is no session.** One is a capacity problem, the other a network problem, and a single
     * counter would send an operator to the wrong one.
     */
    @Test
    fun `with no session the queue is discarded rather than left to fill`() {
        val factory = FakeFactory()
        factory.throwOnOpen = 99
        val p = publisher(factory, ZenohPublisher.Settings(queueCapacity = 4))
        p.start(startThread = false)
        repeat(4) { p.offer(ZenohChannel.POSE, PAYLOAD) }
        p.pumpOnce()
        assertEquals(4L, p.counters().discarded)
        assertEquals(0L, p.counters().dropped)
        // And the queue is empty again, so the next offers are accepted rather than dropped
        // against a backlog nobody can send.
        assertTrue(p.offer(ZenohChannel.POSE, PAYLOAD))
        assertEquals(0L, p.counters().dropped)
    }

    // ── 4. the backoff ───────────────────────────────────────────────────────

    @Test
    fun `a failed open backs off, doubles, and stops at the ceiling`() {
        val factory = FakeFactory()
        factory.throwOnOpen = 99
        val p = publisher(
            factory,
            ZenohPublisher.Settings(reopenDelayMs = 100, maxReopenDelayMs = 400),
        )
        p.start(startThread = false)

        clock = 0
        p.pumpOnce()
        assertEquals(1, factory.opened.size)
        // Inside the backoff: no second dial.
        clock = 99
        p.pumpOnce()
        assertEquals("must not retry inside the backoff", 1, factory.opened.size)
        clock = 100
        p.pumpOnce()
        assertEquals(2, factory.opened.size)
        // Doubled to 200.
        clock = 299
        p.pumpOnce()
        assertEquals(2, factory.opened.size)
        clock = 300
        p.pumpOnce()
        assertEquals(3, factory.opened.size)
        // 400, then held at the ceiling rather than growing to 800.
        clock = 700
        p.pumpOnce()
        assertEquals(4, factory.opened.size)
        clock = 1100
        p.pumpOnce()
        assertEquals("the ceiling must hold", 5, factory.opened.size)
    }

    @Test
    fun `a session that comes back is used, and its publishers are declared afresh`() {
        val factory = FakeFactory()
        val p = publisher(factory, ZenohPublisher.Settings(reopenDelayMs = 100))
        p.start(startThread = false)

        p.offer(ZenohChannel.IMU, PAYLOAD)
        p.pumpOnce()
        assertEquals(1, factory.sink!!.declared.size)

        // The session breaks on the next put and is closed.
        factory.sink!!.throwOnPut = 1
        p.offer(ZenohChannel.IMU, PAYLOAD)
        p.pumpOnce()
        assertEquals(1, factory.sink!!.closed)

        // A brand-new transport object, exactly as a real reopen produces.
        val second = FakeSink()
        factory.sink = second
        clock = 1_000
        p.offer(ZenohChannel.IMU, PAYLOAD)
        p.pumpOnce()
        assertEquals(
            "a reopened session has declared nothing — a kept `declared` set would publish " +
                "onto a publisher that does not exist",
            1,
            second.declared.size,
        )
        assertEquals(1, second.put.size)
        assertEquals(2L, p.counters().opens)
    }

    // ── 5. lifecycle ─────────────────────────────────────────────────────────

    @Test
    fun `nothing is accepted before start or after stop`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        assertFalse("not started", p.offer(ZenohChannel.ODOM, PAYLOAD))
        p.start(startThread = false)
        assertTrue(p.offer(ZenohChannel.ODOM, PAYLOAD))
        p.stop()
        assertFalse("stopped", p.offer(ZenohChannel.ODOM, PAYLOAD))
    }

    @Test
    fun `stop closes the session and abandons the queue rather than flushing it`() {
        val factory = FakeFactory()
        val p = publisher(factory)
        p.start(startThread = false)
        p.offer(ZenohChannel.ODOM, PAYLOAD)
        p.pumpOnce()
        // Queued, and never pumped: `stop` must not sit here draining it.
        repeat(10) { p.offer(ZenohChannel.ODOM, PAYLOAD) }
        p.stop()
        assertEquals("the session must be closed", 1, factory.sink!!.closed)
        assertEquals("only the pumped one went out", 1, factory.sink!!.put.size)
        assertEquals(ZenohPublisher.Phase.STOPPED, p.counters().phase)
    }

    @Test
    fun `a transport that throws on close does not stop the stop`() {
        val factory = FakeFactory()
        val angry = object : ZenohSink {
            override fun declare(key: String, qos: ZenohQos) = Unit
            override fun put(key: String, payload: ByteArray) = Unit
            override fun close() = throw IllegalStateException("no")
        }
        val p = ZenohPublisher(
            CONFIG,
            { angry },
            ZenohPublisher.Settings(),
            nowMs = { clock },
        )
        p.start(startThread = false)
        p.offer(ZenohChannel.ODOM, PAYLOAD)
        p.pumpOnce()
        p.stop()
        assertEquals(ZenohPublisher.Phase.STOPPED, p.counters().phase)
        assertFalse(p.isRunning)
        // The point of the factory being unused here: `angry` is a lambda-supplied sink, so a
        // throwing close must not have prevented the publisher reaching STOPPED.
        assertTrue(factory.opened.isEmpty())
    }

    /** Phase transitions are what the flight record and the status screen are built from. */
    @Test
    fun `phase changes are reported once each, in order`() {
        val factory = FakeFactory()
        val seen = ArrayList<ZenohPublisher.Phase>()
        val p = publisher(factory, onPhase = { phase, _ -> seen += phase })
        p.start(startThread = false)
        p.offer(ZenohChannel.ODOM, PAYLOAD)
        p.pumpOnce()
        // Repeated pumps on a healthy session must not re-report PUBLISHING.
        p.pumpOnce()
        p.pumpOnce()
        p.stop()
        assertEquals(
            listOf(
                ZenohPublisher.Phase.CONNECTING,
                ZenohPublisher.Phase.PUBLISHING,
                ZenohPublisher.Phase.STOPPED,
            ),
            seen,
        )
    }

    /** The endpoint the transport is handed carries the source-address pin when there is one. */
    @Test
    fun `the bind address is applied to the endpoint and nothing else is`() {
        assertEquals("tcp/10.55.1.50:7447", CONFIG.connectEndpoint)
        assertEquals(
            "tcp/10.55.1.50:7447#bind=192.168.1.9:0",
            CONFIG.copy(bindAddress = "192.168.1.9").connectEndpoint,
        )
        val factory = FakeFactory()
        val p = ZenohPublisher(
            CONFIG.copy(bindAddress = "192.168.1.9"),
            factory,
            ZenohPublisher.Settings(),
            nowMs = { clock },
        )
        p.start(startThread = false)
        p.offer(ZenohChannel.ODOM, PAYLOAD)
        p.pumpOnce()
        assertEquals("192.168.1.9", factory.opened.single().bindAddress)
    }
}
