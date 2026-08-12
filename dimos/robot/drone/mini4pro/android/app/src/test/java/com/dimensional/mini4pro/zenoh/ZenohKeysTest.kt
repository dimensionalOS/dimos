package com.dimensional.mini4pro.zenoh

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * [ZenohChannel] — the key expressions and the QoS, against the contract they implement.
 *
 * **This file is one of three copies of one table**, and it exists so the three cannot drift
 * silently. The others are `docs/zenoh-replay-contract.md` §1, which is the prose a subscriber is
 * written against, and `tools/zenohpublish`'s `CATALOGUE` dict, which prints itself with `--keys`
 * for exactly this reason. Every string below was read off the contract and typed in by hand, so
 * that a change in the Kotlin has to be a change here too — and a change here that the contract
 * does not authorise is a change somebody has to justify in review.
 *
 * The expressions are literals rather than composed from the enum, deliberately. Asserting
 * `ch.keyOrNull() == "${prefix}/${ch.channel}/${ch.type}"` would pass for any prefix, any
 * separator and any ordering, which is to say it would assert nothing. A subscriber sees a string.
 *
 * ## Mutation-checked 2026-07-27
 *
 * Whole suite run per mutation, reverted after each, by `tmp/mutate.py`. Counts are failing tests
 * across the whole 1994-test suite.
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the type joined with `/` rather than `.` | 4 |
 *  | prefix and channel swapped | 4 |
 *  | the type segment dropped | 4 |
 *  | the default prefix loses the robot name (`dimos`, not `dimos/drone`) | 4 |
 *  | `status` demoted to the default QoS | 3 |
 *  | every channel promoted to NEVER_DROP | 4 |
 *  | the retired gimbal channel given a key expression | 3 |
 *
 * And 2026-07-28, on the `detections` channel, against the 2218-test suite:
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | the `detections` type becomes `vision_msgs.Detection3DArray` | 1 |
 *  | the `detections` channel is promoted to NEVER_DROP | 3 |
 *
 * The second is 3 rather than 1 because promoting a channel to `NEVER_DROP` breaks the rule below
 * as well as the catalogue: reliable + **block** may only carry messages that never repeat, and a
 * detection is followed by another 100 ms later.
 *
 * Every key-expression mutation kills 4, and three of those four are in this file: the fourth is
 * `ZenohPublisherTest`'s, which asserts the literal key a message is *put* on. That is the pairing
 * worth having — this file says what the catalogue claims, and the publisher's says what actually
 * reaches a transport, and a change has to satisfy both.
 */
class ZenohKeysTest {

    /**
     * The contract's own table, `docs/zenoh-replay-contract.md` §1, transcribed.
     *
     * `datum` is deliberately absent — deleted 2026-07-27 because `pose` and `gps_location` are
     * the same measurement expressed two ways, so a consumer recovers the origin by subtracting
     * them over every sample rather than trusting the one instant we chose to announce it.
     * Measured agreement with the deleted channel: 0.0000 m, worst residual 4.4 µm.
     */
    private val contract = listOf(
        ZenohChannel.ODOM to ("dimos/drone/odom/nav_msgs.Odometry" to ZenohQos.DEFAULT),
        ZenohChannel.POSE to ("dimos/drone/pose/geometry_msgs.PoseStamped" to ZenohQos.DEFAULT),
        ZenohChannel.GPS_LOCATION to
            ("dimos/drone/gps_location/sensor_msgs.NavSatFix" to ZenohQos.DEFAULT),
        ZenohChannel.IMU to ("dimos/drone/imu/sensor_msgs.Imu" to ZenohQos.DEFAULT),
        ZenohChannel.BATTERY to
            ("dimos/drone/battery/sensor_msgs.BatteryState" to ZenohQos.DEFAULT),
        ZenohChannel.MODE to ("dimos/drone/mode/std_msgs.String" to ZenohQos.DEFAULT),
        ZenohChannel.TF to ("dimos/drone/tf/tf2_msgs.TFMessage" to ZenohQos.DEFAULT),
        ZenohChannel.CAMERA_INFO to
            ("dimos/drone/camera_info/sensor_msgs.CameraInfo" to ZenohQos.DEFAULT),
        ZenohChannel.VIDEO to
            ("dimos/drone/video/foxglove_msgs.CompressedVideo" to ZenohQos.DEFAULT),
        ZenohChannel.DETECTIONS to
            ("dimos/drone/detections/vision_msgs.Detection3DArray" to ZenohQos.DEFAULT),
        // 2026-07-29: the tag's world-frame belief and the commanded velocity. `tag_fix`
        // is `detections`' world-frame companion, same type, same QoS argument (a fix
        // repeats ~100 ms later). `setpoint` is Ivan's "publish twist messages", stamped
        // per the bus's convention, DEFAULT because a setpoint repeats and the record is
        // the lossless copy.
        ZenohChannel.TAG_FIX to
            ("dimos/drone/tag_fix/vision_msgs.Detection3DArray" to ZenohQos.DEFAULT),
        ZenohChannel.SETPOINT to
            ("dimos/drone/setpoint/geometry_msgs.TwistStamped" to ZenohQos.DEFAULT),
        // 2026-07-30: DJI's own wind estimate, m/s, as a bare Float — Ivan's 1-D-rides-a-Float
        // rule, motivated by landing14 (9.1 m/s explained the "flyaway" a day late). DEFAULT
        // because a wind reading repeats: the next change supersedes a dropped one and the
        // record's `windSpeedDmS` line is the lossless copy.
        ZenohChannel.WIND to ("dimos/drone/wind/std_msgs.Float32" to ZenohQos.DEFAULT),
        ZenohChannel.STATUS to ("dimos/drone/status/std_msgs.String" to ZenohQos.NEVER_DROP),
        // 2026-07-30: every DJI warning, one message per change. Ivan, after landing17: "make
        // sure to pass all of these DJI warnings into both Zenoh and QGroundControl, of course."
        // `diagnostic_msgs.DiagnosticArray` because the family exists upstream in `dimos_lcm` and
        // the standing rule is to use standard types; NEVER_DROP because a warning is said once
        // and the diff that produced it is deliberately silent about what it has already said.
        ZenohChannel.WARNINGS to
            ("dimos/drone/warnings/diagnostic_msgs.DiagnosticArray" to ZenohQos.NEVER_DROP),
    )

    @Test
    fun `every published channel's key expression is the contract's, verbatim`() {
        for ((channel, expected) in contract) {
            assertEquals("$channel key", expected.first, channel.keyOrNull())
            assertEquals("$channel QoS", expected.second, channel.qos)
        }
    }

    @Test
    fun `the published set is exactly the contract's set, with nothing extra`() {
        assertEquals(contract.map { it.first }, ZenohChannel.PUBLISHED)
    }

    /**
     * The type is the **last** segment and it is joined with a dot, not a slash. That is not our
     * invention: it is what `Topic.key_expr` produces in DiMOS
     * (`dimos/protocol/pubsub/impl/zenohpubsub.py`), and a slash would put us on a key expression
     * no DiMOS module subscribes to while still being perfectly valid zenoh.
     */
    @Test
    fun `the type is the last segment and is joined with a dot`() {
        val key = ZenohChannel.ODOM.keyOrNull()!!
        val segments = key.split('/')
        assertEquals(listOf("dimos", "drone", "odom", "nav_msgs.Odometry"), segments)
        assertTrue(segments.last().contains('.'))
        assertFalse("the type must not be split across segments", segments.last().contains('/'))
    }

    @Test
    fun `the prefix is DiMOS's own transport_topic shape`() {
        // `transport_topic()` is literally `"dimos/" + name.lstrip("/")`, and our name is `drone`.
        assertEquals("dimos/drone", ZenohChannel.DEFAULT_PREFIX)
    }

    @Test
    fun `a configured prefix replaces only the prefix`() {
        assertEquals(
            "dimos/drone2/odom/nav_msgs.Odometry",
            ZenohChannel.ODOM.keyOrNull("dimos/drone2"),
        )
    }

    /**
     * The retired channel. It survives in the enum because [ZenohEmission] still tallies it — a
     * flight record carries no gimbal attitude, and *"withheld because the schema has no column"*
     * is a different statement from *"not published because the channel is retired"*. It must not
     * be able to reach a bus.
     */
    @Test
    fun `the retired gimbal channel has no key and is not published`() {
        assertNull(ZenohChannel.GIMBAL.keyOrNull())
        assertNull(ZenohChannel.GIMBAL.type)
        assertFalse(ZenohChannel.GIMBAL in ZenohChannel.PUBLISHED)
    }

    /** A `datum` channel would restate, once a second, something the data already proves. */
    @Test
    fun `there is no datum channel`() {
        assertTrue(ZenohChannel.entries.none { it.channel == "datum" })
    }

    /**
     * `NEVER_DROP` is reliable + **block**, so every channel that carries it can wedge the
     * publisher's thread behind a slow subscriber. Exactly two do, and both are channels whose
     * messages do not repeat: an operator sentence said once and lost is gone, and so is a
     * warning — `warn/WarningMonitor`'s diff deliberately never says the same thing twice, so
     * nothing later restates a dropped one. A `pose`, by contrast, is followed by another 100 ms
     * later.
     */
    @Test
    fun `only the channels whose messages never repeat may block`() {
        assertEquals(
            listOf(ZenohChannel.STATUS, ZenohChannel.WARNINGS),
            ZenohChannel.PUBLISHED.filter { it.qos == ZenohQos.NEVER_DROP },
        )
    }
}
