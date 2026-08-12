package com.dimensional.mini4pro.zenoh

import android.os.Build
import io.zenoh.Config
import io.zenoh.Session
import io.zenoh.Zenoh
import io.zenoh.bytes.ZBytes
import io.zenoh.keyexpr.KeyExpr
import io.zenoh.pubsub.Publisher
import io.zenoh.qos.CongestionControl
import io.zenoh.qos.Priority
import io.zenoh.qos.QoS
import io.zenoh.qos.Reliability

/**
 * **The real transport**: `org.eclipse.zenoh:zenoh-kotlin-android:1.9.0`, behind [ZenohSink].
 *
 * The only file in this package that imports `io.zenoh`, and the only one no JVM test can reach.
 * Everything it does is mechanical; every decision it embodies is argued somewhere else and cited
 * here.
 *
 * ## What is verified about this dependency, and what is not
 *
 * Measured from the published artifact before a line of this was written:
 *
 *  - **The version matches the bus exactly.** hyper1 runs `eclipse/zenoh:1.9.0`, DiMOS uses
 *    eclipse-zenoh 1.9.0, and `zenoh-kotlin` 1.9.0's `zenoh-jni/Cargo.toml` pins core zenoh
 *    `release/1.9.0`. Nothing here relies on zenoh's cross-version compatibility promise.
 *  - **It brings no second C++ runtime.** `readelf -d` on the shipped `arm64-v8a/libzenoh_jni.so`
 *    gives three NEEDED entries — `libdl.so`, `libm.so`, `libc.so` — because it is Rust with no
 *    C++ dependency linked. The `libc++_shared.so` collision that killed the test APK on
 *    2026-07-27 does not arise, and the `pickFirst` in `build.gradle` is untouched by it.
 *  - **It costs 13.2 MiB of APK**, measured end to end: 177,612,957 → 191,414,206 bytes, of which
 *    ~8.6 MiB is the deflated arm64 library and the rest is dex for `kotlin-reflect`, coroutines
 *    and kotlinx-serialization, which the AAR depends on.
 *  - **arm64 only, and that is a correctness requirement rather than a size one.** zenoh-kotlin
 *    PR #659 documents that every `zenoh-jni` entry point takes native pointers as `*const T`
 *    while Kotlin passes `jlong`, so on `armeabi-v7a` and `x86` every argument after the first
 *    pointer is shifted and `declarePublisher` fails outright. `abiFilters 'arm64-v8a'` already
 *    excludes them for MSDK's sake; this is a second, independent reason not to relax it.
 *
 * Not verified, and the reason [SUPPORTED_SDK] and the `tools:overrideLibrary` exist:
 *
 *  - **The AAR declares `minSdkVersion 30` and this app declares 24.** The floor is real —
 *    `libzenoh_jni.so` is built by NDK r26 against API 30 — but it applies to this library alone,
 *    so it is overridden in the manifest and enforced here instead. Below API 30 [open] refuses
 *    with a sentence rather than letting `System.loadLibrary` fail somewhere less legible.
 *  - **The library is not 16 KB page aligned** (zenoh-kotlin issue #656, open, unfixed as of
 *    1.9.0): every LOAD segment is `Align 0x1000`. On a 16 KB-page device the library will not
 *    load, and Google Play rejects apps targeting Android 15+ without it. This app is sideloaded
 *    onto one 4 KB device, so it is a liability rather than a blocker, and it is written down so
 *    it is discovered here rather than on a new phone.
 *
 * ## The socket is not bound to the WiFi network, and cannot be
 *
 * `wifi-fix.md` gotcha #2 and `docs/zenoh-replay-contract.md` D-3: an unbound socket on Android 16
 * follows the *default* network, which moves to cellular the moment WiFi fails router validation,
 * and telemetry addressed to the LAN then leaves over LTE and dies. MAVLink solves this by binding
 * its `DatagramSocket` to the `android.net.Network` object.
 *
 * **zenoh-kotlin exposes no equivalent hook.** Its configuration is a JSON5 document and its
 * sockets are opened in Rust, so there is no `Socket` for `Network.bindSocket` to reach. What is
 * available is zenoh's own endpoint parameter `#bind=<ip>:0`, which pins the local *source
 * address*; [ZenohConfig.bindAddress] carries it and `ZenohBus` supplies the WiFi interface's IPv4
 * when it has one. That is strictly better than nothing and strictly weaker than a network bind,
 * and the difference is stated in [ZenohConfig] rather than papered over. The only true equivalent
 * is `ConnectivityManager.bindProcessToNetwork`, which would take DJI's own sockets with it, and
 * it is deliberately not done.
 *
 * ## Threading
 *
 * Every method here is called from [ZenohPublisher]'s single worker thread and from nowhere else,
 * which is why there is no locking. [close] is called from that thread too, or after it has
 * ended — never underneath it.
 */
class ZenohKotlinSink private constructor(private val session: Session) : ZenohSink {

    private val publishers = HashMap<String, Publisher>()

    /**
     * Declares a publisher, translating our two QoS classes into zenoh's own vocabulary.
     *
     * DiMOS's names map onto zenoh's exactly as `zenohpubsub.py` maps them, which is what makes
     * `tools/zenohpublish`'s Python and this Kotlin the same publisher: `NEVER_DROP` is
     * `RELIABLE` + `BLOCK`, and *default* is zenoh's own publisher default of `RELIABLE` + `DROP`.
     * `LATEST_WINS` is not expressed because nothing on this bus uses it.
     *
     * `Priority.DATA` and `express = false` throughout — zenoh's defaults, and deliberately not
     * tuned. A priority scheme invented on one side of a bus is a claim the other side has not
     * agreed to.
     */
    override fun declare(key: String, qos: ZenohQos) {
        if (publishers.containsKey(key)) return
        val keyExpr = KeyExpr.tryFrom(key).getOrThrow()
        val zqos = when (qos) {
            ZenohQos.NEVER_DROP -> QoS(CongestionControl.BLOCK, Priority.DATA, express = false)
            ZenohQos.DEFAULT -> QoS(CongestionControl.DROP, Priority.DATA, express = false)
        }
        val publisher = session.declarePublisher(
            keyExpr,
            qos = zqos,
            reliability = Reliability.RELIABLE,
        ).getOrThrow()
        publishers[key] = publisher
    }

    /**
     * One LCM message, and nothing around it — D-4. `ZBytes.from(bytes)` wraps the array without
     * copying it, and the encoding is left at zenoh's default: the type is already the last
     * segment of the key expression, which is DiMOS's own convention, so restating it in a
     * content-type would be a second place for it to be wrong.
     */
    override fun put(key: String, payload: ByteArray) {
        val publisher = publishers[key] ?: throw IllegalStateException("no publisher for $key")
        publisher.put(ZBytes.from(payload)).getOrThrow()
    }

    override fun close() {
        // Publishers first, then the session. Each contained separately: one that refuses to
        // close must not leave the session open, because the next `open` would then be the second
        // live session in this process.
        for (p in publishers.values) runCatching { p.close() }
        publishers.clear()
        runCatching { session.close() }
    }

    companion object {

        /**
         * The API level `libzenoh_jni.so` is built for, from its own ELF note (`file` reports
         * *"for Android 30"*), and the `minSdkVersion` the AAR declares.
         */
        const val SUPPORTED_SDK = 30

        /**
         * Whether this device can load the native library at all.
         *
         * Checked by `ZenohBus` before the publisher is ever started, rather than here, so that
         * the refusal is one sentence in the flight record and on the status screen instead of an
         * `UnsatisfiedLinkError` from a static initialiser. Keeping the check out of [open] is
         * also what lets `tools/zenohlive` run this exact class on a desktop JVM against the JVM
         * build of the same binding — which is the only way anything below this seam gets
         * exercised for real before it reaches a phone.
         */
        fun supportedHere(): Boolean = Build.VERSION.SDK_INT >= SUPPORTED_SDK

        /**
         * Opens a session, or throws. The configuration is [ZenohConfig.json5], which is where the
         * four decisions it encodes are argued and where a test can reach them.
         */
        fun open(config: ZenohConfig): ZenohSink {
            val zenohConfig = Config.fromJson5(config.json5()).getOrThrow()
            val session = Zenoh.open(zenohConfig).getOrThrow()
            return ZenohKotlinSink(session)
        }

        /** The factory the publisher is built with in the app. */
        val FACTORY = ZenohSinkFactory { open(it) }
    }
}
