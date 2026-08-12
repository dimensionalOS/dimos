package com.dimensional.mini4pro.zenoh

/**
 * **The seam Zenoh sits behind**, and the reason every rule above it is a JVM assertion.
 *
 * A Zenoh session is a native runtime: `io.zenoh.ZenohLoad` runs `System.loadLibrary("zenoh_jni")`
 * in a static initialiser, and the shipped `libzenoh_jni.so` is an arm64 Android binary. Nothing
 * about it can exist in a `src/test` JVM. So it goes behind an interface, exactly as
 * `record/VideoSidecar` puts a file behind `VideoSink` and `vision/TagSighting` puts OpenCV behind
 * a supplier — and for exactly the same reason, which is that the *policy* is the dangerous part
 * and the policy is what a test must be able to reach.
 *
 * ## What this seam does and does not buy
 *
 * Above it, a test can drive every one of these: the queue filling and dropping rather than
 * blocking, a session that never opens, a session that throws on the tenth publish, a session that
 * blocks forever inside `put`, a link going down mid-flight, a stop while a publish is in flight.
 * Those are the properties that can hurt someone, and they are all seam-testable.
 *
 * Below it, **nothing is covered by a JVM test and this must be said plainly**: that
 * `libzenoh_jni.so` loads in a process that already holds MSDK and OpenCV natives; that the key
 * expressions zenoh parses are the ones we wrote; that `CongestionControl.BLOCK` blocks and
 * `DROP` drops; that a `put` from a background thread is safe; that the session reconnects to a
 * router that went away. Every one of those is a claim about the library or the device, and the
 * only thing that settles them is a real session on the real phone. `ZenohKotlinSink` is the
 * implementation and the acceptance test is `tools/zenohpublish --spy` seeing our keys.
 *
 * ## The contract an implementation owes
 *
 * - [declare] and [put] are called **only from the publisher's own thread**, never from a caller's
 *   and never from two at once. An implementation therefore needs no locking of its own.
 * - Both are allowed to **block**, and one of them is expected to: `NEVER_DROP` is reliable +
 *   block by definition. That is safe precisely because of where they are called from — see
 *   [ZenohPublisher], whose entire shape exists so that a blocking transport can never reach a
 *   thread that flies an aircraft.
 * - Both are allowed to **throw**. [ZenohPublisher] contains every throw, counts it, and keeps
 *   going. An implementation that swallowed its own errors would be hiding the one signal that
 *   says the bus is broken.
 */
interface ZenohSink : AutoCloseable {

    /**
     * Declares a publisher for one key expression at one QoS, before anything is put on it.
     *
     * Declared once per session rather than per message because that is what zenoh's own API is
     * shaped for — a `Publisher` carries the QoS, and a bare `session.put` would have to restate
     * it on every message, which is one more place for `NEVER_DROP` to be forgotten.
     */
    fun declare(key: String, qos: ZenohQos)

    /**
     * Puts one payload on one previously [declare]d key.
     *
     * The payload is **one LCM-encoded message and nothing else** — no envelope, no length prefix,
     * no compression, no batching (`docs/zenoh-replay-contract.md` D-4). A subscriber feeding
     * `sample.payload.to_bytes()` straight to `lcm_decode()` gets a message or an exception, never
     * a partial one, and that is the whole of the wire contract.
     */
    fun put(key: String, payload: ByteArray)

    /** Closes the session. Called once, on the publisher's thread, and may block. */
    override fun close()
}

/**
 * Opens sessions. Separate from [ZenohSink] so that a *failure to open* is expressible — which is
 * the case that matters, because it is the normal one: the router is on hyper1, the phone is on
 * WiFi, and any given start may find neither.
 *
 * Mirrors `record/VideoSinkFactory` and for the same reason: the thing that creates the resource
 * and the resource itself fail differently and are faked differently.
 */
fun interface ZenohSinkFactory {

    /**
     * Opens a session, or throws.
     *
     * **Called only on the publisher's thread, never on a caller's**, because opening a zenoh
     * session dials TCP to a router that may not be there and `connect.timeout_ms` for a peer
     * defaults to `-1` — an infinite retry. On the UI thread that is an ANR; on the setpoint
     * thread it is an aircraft with no one flying it.
     */
    fun open(config: ZenohConfig): ZenohSink
}

/**
 * Where the bus is and what we are called on it.
 *
 * A plain value with no Android in it, so the whole of `ZenohSettings`' resolution and every test
 * above the seam can build one. `ZenohSettings.resolve` is what turns saved preferences and launch
 * extras into this.
 *
 * @param endpoint the router to dial, as a zenoh endpoint — `tcp/10.55.1.50:7447`. **We dial out
 *   and we do not listen** (`docs/zenoh-replay-contract.md` D-3): this network does not carry
 *   phone→laptop traffic in either direction that matters, and everything that has ever worked
 *   goes through hyper1. A router is also what lets several consumers attach without the phone
 *   knowing how many there are.
 * @param prefix the key prefix. `dimos/drone` is DiMOS's own `transport_topic()` shape.
 * @param bindAddress the local IPv4 to pin the outbound socket's **source address** to, appended
 *   to the endpoint as zenoh's `#bind=<ip>:0`, or null for none.
 *
 *   **This is not the equivalent of `WifiBind`, and pretending otherwise would be the exact bug it
 *   is meant to prevent.** MAVLink's socket is bound to the `android.net.Network` object, which
 *   sets the socket's fwmark and therefore selects Android's per-network route table; that is what
 *   stops telemetry leaving over LTE when WiFi fails router validation. zenoh-kotlin exposes **no
 *   socket-factory hook at all** — its configuration is a JSON5 blob and its sockets are opened in
 *   Rust — so `Network.bindSocket` cannot be applied to them. `#bind=` pins the source IP, which
 *   is strictly better than nothing and strictly weaker than a network bind: with WiFi off the
 *   default network, the route to the router lives only in the WiFi table and a source-address
 *   pin does not reach it. The only true equivalent is
 *   `ConnectivityManager.bindProcessToNetwork`, which is process-wide and would take DJI's own
 *   sockets with it, and it is deliberately not done here. This is an open item, it is measured
 *   rather than assumed, and it is written down at the one place that could pretend otherwise.
 * @param mode **`client`, and this was measured rather than chosen.** `docs/zenoh-topics.md` and
 *   `docs/zenoh-dimos-transport.md` §2.6 both say `peer`, *"matching DiMOS"*, with §6.3 noting
 *   that `client` through the relay *"may prove more reliable"* and should be measured. It was, on
 *   2026-07-27, and `peer` does not work at all in the topology this project requires — see
 *   [MODE_CLIENT].
 */
data class ZenohConfig(
    val endpoint: String,
    val prefix: String = ZenohChannel.DEFAULT_PREFIX,
    val bindAddress: String? = null,
    val mode: String = MODE_CLIENT,
) {

    /** The endpoint with any `#bind=` applied — what actually goes into the zenoh config. */
    val connectEndpoint: String
        get() = bindAddress?.let { "$endpoint#bind=$it:0" } ?: endpoint

    /**
     * The zenoh session configuration, as the JSON5 document zenoh's own `Config.fromJson5` takes.
     *
     * **Here rather than in [ZenohKotlinSink] on purpose.** This string is the whole of what we
     * tell zenoh about how to behave on a network that has already broken two transports, and it
     * is a *string* — so it can be got wrong in ways a type system will not catch and a test can.
     * Keeping it in a class with no `io.zenoh` import means `ZenohConfigTest` asserts it on the
     * JVM and `tools/zenohlive` feeds the very same characters to a real router.
     *
     * The four decisions, all from `docs/zenoh-replay-contract.md` §2:
     *
     *  - **`mode: peer`**, matching DiMOS, so the phone is an equal on the bus rather than a
     *    client of it. `client` through the relay is the alternative and §6.3 says it should be
     *    measured rather than assumed, which is why [mode] is a field.
     *  - **Connect only, never listen.** Every participant dials the router at hyper1; nobody
     *    dials anybody else, because the direction that dials *into* the phone is the blackholed
     *    one on this network (`wifi-fix.md`). The empty `listen.endpoints` is load-bearing rather
     *    than a default left alone: zenoh's own default for a peer is `tcp/[::]:0`, which opens a
     *    port nothing here can usefully reach.
     *  - **Multicast scouting off**, and gossip with it. D-2: this project's AP filters multicast
     *    between WiFi clients, DiMOS stopped trusting it for the same reason, and hyper1 also runs
     *    Home Assistant, Frigate and MQTT — a phone that scouts is a phone that can find them.
     *  - **`exit_on_failure: false`, stated rather than inherited.** For a peer this is already
     *    zenoh's default; the value that would be wrong here is the *client* default of `true`.
     *    If [mode] is ever changed to `client` on §6.3's evidence, inheriting that default would
     *    hand a library the power to terminate the process that is flying the aircraft.
     */
    fun json5(): String = """
        {
          mode: "$mode",
          connect: {
            endpoints: ["$connectEndpoint"],
            exit_on_failure: false,
          },
          listen: { endpoints: [] },
          scouting: {
            multicast: { enabled: false },
            gossip: { enabled: false },
          },
        }
    """.trimIndent()

    companion object {
        /**
         * **What the design documents specified, and what does not work here.** Kept as a
         * constant so the alternative is one word away and so this paragraph has somewhere to
         * live.
         *
         * A zenoh **peer** does not get its traffic relayed by a router. In zenoh's default
         * `peer_to_peer` routing mode a router's job between peers is *introduction*: it
         * **gossips** them to one another and they then dial each other directly. Data goes
         * peer↔peer, not peer→router→peer. So a peer that has gossip disabled and declares no
         * listen endpoints — which is exactly what D-2 and D-3 require of us — has no way to
         * learn about anybody and no way to be dialled, and it publishes into a void while
         * reporting complete success.
         *
         * **Measured on 2026-07-27, twice, on a live router**, because it is the sort of claim
         * that should not rest on reading:
         *
         * | publisher | scouting | received by `zenohpublish --spy` |
         * |---|---|---|
         * | `tools/zenohlive` (these classes), `mode: peer` | gossip + multicast off | **0 of 3885** |
         * | `tools/zenohpublish` (Python), `mode: peer`, `--no-scouting` | gossip + multicast off | **0 of 5704** |
         * | `tools/zenohpublish` (Python), `mode: peer`, default | gossip **on** | 5704 of 5704 |
         * | `tools/zenohlive` (these classes), `mode: client` | n/a | **all six channels, decoded** |
         *
         * The third row is what `docs/zenoh-replay-contract.md` §9 recorded as *"two processes via
         * the hyper1 router"* — and the fourth row is what that test was actually missing. **With
         * gossip on and both processes on one laptop, the router introduced them and they talked
         * to each other directly.** The router was never carrying the data. That is invisible when
         * both ends can dial each other, and fatal the moment one end is the phone, because
         * phone↔laptop unicast is blackholed on this network (`wifi-fix.md`) — which is the entire
         * reason there is a router.
         *
         * Turning gossip back on would not rescue `peer`, and it is worth being explicit about
         * why: the phone would learn a laptop's address and dial a route that does not exist,
         * while D-2's own reason for disabling scouting — this AP filters multicast, and hyper1
         * also hosts Home Assistant, Frigate and MQTT — would be given up for nothing.
         */
        const val MODE_PEER = "peer"

        /**
         * **What we use, and what the topology actually requires.**
         *
         * A client's traffic is routed *by the router, on its behalf*. It dials out, never
         * listens, discovers nothing, and every subscriber attached to the same router receives
         * it. That is precisely the shape `docs/zenoh-replay-contract.md` §2 describes — *"every
         * participant dials the router at hyper1; nobody dials anybody else"* — and `peer` was
         * simply the wrong word for it.
         *
         * One consequence travels with this choice and is handled in [json5]: **zenoh's
         * `exit_on_failure` defaults to `true` for a client** and `false` for a peer. Left
         * inherited, a router that is down at startup would terminate the process that is flying
         * the aircraft. It is set explicitly, and this is the mode that made that matter.
         */
        const val MODE_CLIENT = "client"
    }
}
