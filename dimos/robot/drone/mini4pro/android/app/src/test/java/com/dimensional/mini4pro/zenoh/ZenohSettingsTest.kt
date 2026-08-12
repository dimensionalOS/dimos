package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [ZenohSettings] — whether the bus is wanted and where it is.
 *
 * Small rules, and the reason they are tested at all is the incident `video/VideoRequest` was
 * written after: `tools/session video` passed `--ez video true` for weeks into an app that read no
 * such extra, and nothing noticed, because *"it did not appear"* and *"it was never asked for"*
 * look identical from outside. Every rule here is therefore a test rather than a habit.
 *
 * ## Mutation-checked 2026-07-27
 *
 * Whole suite run per mutation, reverted after each, by `tmp/mutate.py`. Counts are failing tests
 * across the whole 1994-test suite.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | on by default | 2 |
 *  | `--ez zenoh false` cannot turn it off (the flag only ever enables) | 1 |
 *  | the intent host does not win over the saved one | 1 |
 *  | a non-relay router raises no warning | 2 |
 *  | a blank prefix is passed through instead of defaulted | 1 |
 *  | an out-of-range port is dialled anyway | 1 |
 *  | the status line cannot say "ON, NOT RUNNING" | 1 |
 *
 * ## Mutation-checked 2026-07-28 (the video switch, added after the dropped-flag incident)
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | video survives the bus being off (`&& enabled` dropped) | 1 |
 *  | `--ez zenohVideo` is ignored (`intentVideo` dropped from the fold) | 1 |
 *
 * The incident itself — `BridgeService.start` not carrying `video` across the service intent —
 * is *not killable from here*: it lives in an Android `Intent` round-trip this JVM suite cannot
 * express. Its regression guard is the hardware check: the `"zenoh: video channel ON"` warning
 * in logcat, absent on the 10:54 run and present on the 11:00 run of 2026-07-28.
 *
 * And 2026-07-28, on the `detections` switch, against the 2218-test suite. Nothing survived:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the `detections` switch survives a bus that is off | 1 |
 *  | the `detections` switch defaults on | 1 |
 *
 * *"the router defaults to the GCS address"* is **not in this table and cannot be**, which is
 * worth a sentence because it is the rule this file exists for. [ZenohSettings.resolve] takes no
 * `gcsHost` parameter at all, so the mutation is unexpressible: making the router follow
 * telemetry would mean adding an input, not changing a branch. That is the design defending
 * itself, and it is the difference between a rule and a habit.
 */
class ZenohSettingsTest {

    @Test
    fun `off unless asked, and the launch flag works in both directions`() {
        assertFalse(ZenohSettings.resolve().enabled)
        assertFalse(ZenohSettings.resolve(savedEnabled = false).enabled)
        assertTrue(ZenohSettings.resolve(savedEnabled = true).enabled)
        // Absent leaves the saved setting alone…
        assertTrue(ZenohSettings.resolve(intentEnabled = null, savedEnabled = true).enabled)
        // …and `false` is a considered "not this session", which is what makes the flag usable
        // for ruling Zenoh out as the cause of something.
        assertFalse(ZenohSettings.resolve(intentEnabled = false, savedEnabled = true).enabled)
        assertTrue(ZenohSettings.resolve(intentEnabled = true, savedEnabled = false).enabled)
    }

    /**
     * **The router does not follow the GCS address**, and that is the one place this resolution
     * deliberately differs from video's. Video is a stream to the ground station and the relay
     * carries `:14550` and `:5600` in one process, so they share an address. A Zenoh bus is a
     * router several consumers attach to independently; pointing it at QGroundControl's laptop
     * would dial a port nothing listens on.
     */
    @Test
    fun `the default router is the relay and there is no gcsHost input at all`() {
        val plan = ZenohSettings.resolve()
        assertEquals(ZenohSettings.ROUTER_HOST, plan.host)
        assertEquals(ZenohSettings.ROUTER_PORT, plan.port)
        assertEquals(ZenohSettings.Source.DEFAULT_RELAY, plan.source)
        assertEquals("tcp/10.55.1.50:7447", plan.endpoint)
        assertNull("the relay must not warn about itself", plan.warning)
    }

    @Test
    fun `the intent host wins over the saved one, and both win over the default`() {
        assertEquals(
            ZenohSettings.Source.SAVED,
            ZenohSettings.resolve(savedHost = "10.55.1.99").source,
        )
        val both = ZenohSettings.resolve(intentHost = "10.55.1.98", savedHost = "10.55.1.99")
        assertEquals("10.55.1.98", both.host)
        assertEquals(ZenohSettings.Source.INTENT, both.source)
        // Blank is not an override.
        assertEquals(
            ZenohSettings.Source.DEFAULT_RELAY,
            ZenohSettings.resolve(intentHost = "  ", savedHost = "").source,
        )
    }

    /**
     * `wifi-fix.md`: phone→laptop unicast is blackholed by this AP in every radio combination
     * tried. An observation rather than a refusal — a different network is a legitimate thing to
     * be on — but never silence, because a wrong Zenoh address is silent for a whole session.
     */
    @Test
    fun `a router that is not the relay is named out loud`() {
        val plan = ZenohSettings.resolve(savedHost = "10.55.1.12")
        assertNotNull(plan.warning)
        assertTrue(plan.warning!!.contains("10.55.1.12"))
        assertTrue(plan.warning.contains(ZenohSettings.ROUTER_HOST))
    }

    @Test
    fun `loopback gets its own sentence, because it is a different mistake`() {
        val plan = ZenohSettings.resolve(savedHost = "127.0.0.1")
        assertNotNull(plan.warning)
        assertTrue(plan.warning!!.contains("nothing off this phone"))
    }

    @Test
    fun `a blank or nonsense prefix falls back to DiMOS's own`() {
        assertEquals(ZenohChannel.DEFAULT_PREFIX, ZenohSettings.resolve().prefix)
        assertEquals(ZenohChannel.DEFAULT_PREFIX, ZenohSettings.resolve(savedPrefix = "   ").prefix)
        // A leading or trailing slash would produce `//odom/...` or `dimos/drone//odom/...`,
        // both of which zenoh accepts and no DiMOS module subscribes to.
        assertEquals("dimos/drone2", ZenohSettings.resolve(savedPrefix = "/dimos/drone2/").prefix)
    }

    @Test
    fun `an impossible port falls back rather than being dialled`() {
        assertEquals(ZenohSettings.ROUTER_PORT, ZenohSettings.resolve(savedPort = 0).port)
        assertEquals(ZenohSettings.ROUTER_PORT, ZenohSettings.resolve(savedPort = 70_000).port)
        assertEquals(ZenohSettings.ROUTER_PORT, ZenohSettings.resolve(savedPort = -1).port)
        assertEquals(7452, ZenohSettings.resolve(savedPort = 7452).port)
    }

    /**
     * The video switch, which earned its tests the hard way: on the first hardware test
     * (2026-07-28) `--ez zenohVideo true` set the pref, the UI switch showed ON, and the bus came
     * up without the channel — `BridgeService.start` carried every field of this plan across the
     * service intent except `video`, so `onStartCommand` re-resolved it to the default. These
     * tests pin the resolve rules; the intent round-trip itself has no unit seam (no Robolectric
     * here) and is verified on hardware by the "video channel ON" logcat warning.
     */
    @Test
    fun `video is off by default, follows the pref, and the flag works in both directions`() {
        assertFalse(ZenohSettings.resolve(savedEnabled = true).video)
        assertTrue(ZenohSettings.resolve(savedEnabled = true, savedVideo = true).video)
        // Absent leaves the saved setting alone; `false` is a considered "not this session",
        // exactly as `--ez zenoh false` is — a flag that can only add bandwidth cannot be used
        // to rule the channel out as the cause of a link problem.
        assertTrue(
            ZenohSettings.resolve(savedEnabled = true, intentVideo = null, savedVideo = true).video,
        )
        assertFalse(
            ZenohSettings.resolve(savedEnabled = true, intentVideo = false, savedVideo = true).video,
        )
        assertTrue(
            ZenohSettings.resolve(savedEnabled = true, intentVideo = true, savedVideo = false).video,
        )
    }

    @Test
    fun `there is no such thing as video on a bus that is off`() {
        assertFalse(ZenohSettings.resolve(savedVideo = true).video)
        assertFalse(ZenohSettings.resolve(intentVideo = true).video)
        assertFalse(
            ZenohSettings.resolve(intentEnabled = false, savedEnabled = true, savedVideo = true).video,
        )
    }

    @Test
    fun `the plan builds the config the publisher is given`() {
        val plan = ZenohSettings.resolve(savedEnabled = true, savedPrefix = "dimos/drone2")
        val config = plan.config(bindAddress = "192.168.1.9")
        assertEquals("tcp/10.55.1.50:7447", config.endpoint)
        assertEquals("dimos/drone2", config.prefix)
        assertEquals("tcp/10.55.1.50:7447#bind=192.168.1.9:0", config.connectEndpoint)
        // **`client`, not `peer`**, and this assertion is the tripwire on a measurement rather
        // than a preference. A peer's traffic is not relayed by a router — the router only
        // gossips peers to each other so they can dial *directly*, which the phone cannot do on
        // this network. Measured 2026-07-27: 0 of 3885 messages arrived under `peer` with
        // scouting off, and every channel arrived under `client`. See `ZenohConfig.MODE_PEER`.
        assertEquals(ZenohConfig.MODE_CLIENT, config.mode)
    }

    /**
     * The session configuration is a **string**, so it can be wrong in ways no type system
     * catches. `tools/zenohlive` feeds these exact characters to a real router; this is the half
     * that runs on every build.
     */
    @Test
    fun `the session config says client, connect-only, and no scouting`() {
        val json5 = ZenohConfig("tcp/10.55.1.50:7447").json5()
        assertTrue("mode must be client", json5.contains("mode: \"client\""))
        assertTrue(json5.contains("endpoints: [\"tcp/10.55.1.50:7447\"]"))
        // Never listen: the direction that dials *into* the phone is the blackholed one, and
        // zenoh's own default for a peer would open a port nothing here can reach.
        assertTrue("must declare no listen endpoints", json5.contains("listen: { endpoints: [] }"))
        // D-2. This AP filters multicast, and hyper1 also hosts Home Assistant, Frigate and MQTT.
        assertTrue(json5.contains("multicast: { enabled: false }"))
        assertTrue(json5.contains("gossip: { enabled: false }"))
        // **Load-bearing for a client**, whose zenoh default is `true`: inherited, a router that
        // is down at startup would terminate the process that is flying the aircraft.
        assertTrue("exit_on_failure must be explicit", json5.contains("exit_on_failure: false"))
        // The bind address rides on the endpoint, not on a field of its own.
        assertTrue(
            ZenohConfig("tcp/10.55.1.50:7447", bindAddress = "192.168.1.9").json5()
                .contains("tcp/10.55.1.50:7447#bind=192.168.1.9:0"),
        )
    }

    /**
     * **"ON, NOT RUNNING" is the honest reading** when the bridge was never started or refused for
     * want of WiFi — states in which the publisher is correctly sitting in `STOPPED` with nothing
     * to say. Collapsing it into "connecting" would tell an operator something is being attempted
     * when nothing is.
     */
    @Test
    fun `the status line separates the setting from the session`() {
        val off = ZenohSettings.resolve()
        val on = ZenohSettings.resolve(savedEnabled = true)
        assertEquals("off", ZenohSettings.statusLine(off, ZenohPublisher.Phase.STOPPED))
        assertTrue(
            ZenohSettings.statusLine(on, ZenohPublisher.Phase.STOPPED).contains("NOT RUNNING"),
        )
        assertTrue(
            ZenohSettings.statusLine(on, ZenohPublisher.Phase.CONNECTING).contains("connecting"),
        )
        assertTrue(
            ZenohSettings.statusLine(on, ZenohPublisher.Phase.PUBLISHING).contains("publishing"),
        )
        // The setting turned off while a session is still up is its own state, because the switch
        // does not take effect until the bridge restarts and the screen must not imply it did.
        assertTrue(
            ZenohSettings.statusLine(off, ZenohPublisher.Phase.PUBLISHING).contains("still running"),
        )
    }

    /**
     * **The `detections` channel is off unless asked, and meaningless on a bus that is off.**
     *
     * A third switch rather than part of the first, and its cost is not bandwidth: 10 Hz of
     * 585-byte messages is 5.9 kB/s. What makes it a choice is what is in them — a pose on a
     * *fitted* focal length and an *assumed* principal point, published where other people's
     * software reads it (`docs/tag-detector.md` §7). The `&& enabled` is the same rule video has
     * and for the same reason: a channel switch left on while the bus is off would put a second
     * "why is nothing publishing" state on the screen with no way to tell it from the first.
     */
    @Test
    fun `the detections channel is off by default and dies with the bus`() {
        assertFalse(ZenohSettings.resolve().detections)
        assertFalse(ZenohSettings.resolve(savedEnabled = true).detections)
        assertTrue(ZenohSettings.resolve(savedEnabled = true, savedDetections = true).detections)
        // On a bus that is off it is meaningless, and that is enforced here rather than by every
        // caller remembering.
        assertFalse(ZenohSettings.resolve(savedEnabled = false, savedDetections = true).detections)
        // `--ez zenohDetections false` genuinely turns it off, which is what makes the flag usable
        // for ruling the channel out as the cause of something.
        assertFalse(
            ZenohSettings.resolve(
                savedEnabled = true, savedDetections = true, intentDetections = false,
            ).detections,
        )
        assertTrue(
            ZenohSettings.resolve(savedEnabled = true, intentDetections = true).detections,
        )
        // And it is independent of video in both directions.
        val plan = ZenohSettings.resolve(savedEnabled = true, savedDetections = true)
        assertFalse(plan.video)
        assertTrue(ZenohSettings.statusLine(plan, ZenohPublisher.Phase.PUBLISHING).contains("+detections"))
    }
}
