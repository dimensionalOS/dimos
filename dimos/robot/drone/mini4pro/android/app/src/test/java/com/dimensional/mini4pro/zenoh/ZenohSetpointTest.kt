package com.dimensional.mini4pro.zenoh

import com.dimensional.mini4pro.record.Json
import com.dimensional.mini4pro.record.SetpointFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The `setpoint` channel — **the commanded velocity as a `geometry_msgs.TwistStamped`**, held
 * to the byte against DiMOS's own encoder and to the record against the replay rule.
 *
 * The channel is Ivan's ask made concrete: *"log commands we are sending to the drone as
 * vector 3 messages to Zenoh and they should be in our recording and convertible to mem2 of
 * course because Zenoh and mem2 have to have the same result"* — and, on the type, *"we can
 * actually publish twist messages, that's the correct thing to publish"* (2026-07-29). The
 * "same result" clause is the load-bearing one, and it is what most of this file pins: the
 * wire may carry nothing the `stick_cmd` line cannot reproduce, so the encoder quantises
 * through the record's own rounding before converting, and the fixture vectors below are
 * **real landing10 lines** encoded by **DiMOS's generated Python bindings** — the far side of
 * the bus, not a second copy of our own arithmetic.
 *
 * ## Mutation campaign, 2026-07-29
 *
 * Whole suite per mutant (2545 tests), fresh `test-results` each run, reverted after each,
 * driven by `tmp/mutants.py`. **Counts are failing tests across the whole suite, measured.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | frame conversion broken: `enuFromNed` dropped in `setpointOrNull` (linear = (vn, ve, −vd)) | 3 |
 *  | yaw dropped: `angular.z = 0.0` | 2 |
 *  | yaw sign flipped: `angular.z = +toRadians(rate)` | 2 |
 *  | zeros filtered out: `setpointOrNull` returns null when all four values are 0 | 2 |
 *  | record-vs-live divergence: quantisation removed (raw doubles encoded) | 1 |
 *  | shadow commands published: `setpointReason`'s `accepted == null` gate dropped | 1 |
 *  | cadence regression: `Cadence.poseMs` back to 200 ms | 1 |
 *
 * No survivors. Two of the counts deserve a sentence. **The yaw mutants kill the landing10
 * fixture even though every landing10 yaw rate is zero**, because `−toRadians(0.0)` is `−0.0`
 * and the byte pin sees the sign bit — the fixture is doing real work at rest. **The
 * quantisation mutant kills exactly 1**, the test whose planted values carry more precision
 * than the record writes: that is the divergence the replay-indistinguishability rule exists
 * to catch, invisible to every test that plants record-precision values.
 *
 * ## Mutation, 2026-07-30 — the radian spelling
 *
 * Whole suite per mutant (2672 tests), fresh `test-results`, reverted after. **Measured.**
 *
 *  | mutation | tests that failed |
 *  |---|---|
 *  | `angular.z = −(qr / 180.0 * Math.PI)` instead of `−Math.toRadians(qr)` | 1 |
 *
 * Exactly one: [`a yaw rate the two radian spellings disagree on`], added the same day. Every
 * other yaw assertion in this file — including the landing10 byte fixture — uses a rate on
 * which the two spellings agree, so **before that test the mutation was a survivor**, and it
 * survived in the mirror rather than here: `tools/memexport` had the divide-first spelling
 * for two flights on the strength of a comment in the encoder that described `toRadians`
 * wrongly. A campaign that plants only round angles cannot see this class of defect at all.
 */
class ZenohSetpointTest {

    private fun stamp(unixMs: Long) = LcmTime.ofEpochSeconds(unixMs / 1000.0)

    private fun emit(
        vn: Double?,
        ve: Double?,
        vd: Double?,
        yr: Double?,
        accepted: Boolean? = true,
        frame: String? = SetpointFrame.NED_VELOCITY,
        unixMs: Long = 1_785_315_837_700,
    ) = ZenohEmission.setpoint(frame, vn, ve, vd, yr, accepted, stamp(unixMs))

    private fun twist(
        vn: Double,
        ve: Double,
        vd: Double,
        yr: Double,
    ): LcmTwistStamped = TwistStampedCodec.decode(emit(vn, ve, vd, yr).bytes!!)

    // ── byte identity against DiMOS's own encoder, on real landing10 lines ───

    /**
     * Three `stick_cmd` lines of `datasets/landing10/20260729-120342.001.jsonl`
     * (`started_unix_ms = 1785315822708`), each encoded by `dimos-lcm`'s generated Python
     * (`TwistStamped.lcm_encode()` — literally what DiMOS calls) from the same conversion
     * this encoder claims to implement. Byte equality here is the "Zenoh and mem2 have the
     * same result" proof at its sharpest: fingerprint, field order, quantised values, NaN
     * bit patterns and the −0.0 the yaw conversion produces at rest, all at once.
     */
    @Test
    fun `real landing10 sends encode byte-identically to DiMOS's Python`() {
        // seq 1, t=14.991982: the first DO_REPOSITION climb command (vd −1.5 is UP).
        assertEquals(
            "9cd2bcbe6cb2a7c0000000006a69c1fd29b9272f0000000c64726f6e652f776f726c6400" +
                "3f7cac083126e979bf789374bc6a7efa3ff8000000000000" +
                "7ff80000000000007ff80000000000008000000000000000",
            emit(-0.006, 0.007, -1.5, 0.0, unixMs = 1_785_315_837_700).bytes!!.toHex(),
        )
        // seq 98, t=24.692732: an all-zero setpoint — a command, on the wire, not silence.
        assertEquals(
            "9cd2bcbe6cb2a7c0000000006a69c20717e6c6560000000c64726f6e652f776f726c6400" +
                "000000000000000000000000000000008000000000000000" +
                "7ff80000000000007ff80000000000008000000000000000",
            emit(0.0, 0.0, 0.0, 0.0, unixMs = 1_785_315_847_401).bytes!!.toHex(),
        )
        // seq 106, t=64.392636: TAG_DESCENT steering toward the tag.
        assertEquals(
            "9cd2bcbe6cb2a7c0000000006a69c22f060523860000000c64726f6e652f776f726c6400" +
                "3fde978d4fdf3b643fdbc6a7ef9db22d8000000000000000" +
                "7ff80000000000007ff80000000000008000000000000000",
            emit(0.434, 0.478, 0.0, 0.0, unixMs = 1_785_315_887_101).bytes!!.toHex(),
        )
    }

    // ── the frame conversion, one axis at a time ─────────────────────────────

    /** NED north lands on ENU `linear.y`, alone — a swap flies east when told north. */
    @Test
    fun `north lands on linear y alone`() {
        val t = twist(1.5, 0.0, 0.0, 0.0)
        assertEquals(1.5, t.twist.linear.y, 0.0)
        assertEquals(0.0, t.twist.linear.x, 0.0)
    }

    @Test
    fun `east lands on linear x alone`() {
        val t = twist(0.0, 2.25, 0.0, 0.0)
        assertEquals(2.25, t.twist.linear.x, 0.0)
        assertEquals(0.0, t.twist.linear.y, 0.0)
    }

    /** The sign that ends in the ground: NED down 1.5 is ENU z −1.5. */
    @Test
    fun `down negates onto linear z`() {
        assertEquals(-1.5, twist(0.0, 0.0, 1.5, 0.0).twist.linear.z, 0.0)
    }

    /**
     * The yaw rate crosses on its own line: NED clockwise-positive **deg/s** becomes ENU
     * counterclockwise-positive **rad/s** — a negation and a unit change, both silent if
     * dropped. +90 °/s clockwise is exactly −π/2 rad/s.
     *
     * Both angles here are ones where every plausible spelling of "degrees to radians" agrees
     * (90/180 and 45/180 are exact halvings), so this test pins the *sign and the unit* and
     * nothing finer — see [`a yaw rate the two radian spellings disagree on`] for the case it
     * cannot see.
     */
    @Test
    fun `yaw rate negates and converts to radians`() {
        assertEquals(-(90.0 / 180.0 * Math.PI), twist(0.0, 0.0, 0.0, 90.0).twist.angular.z, 0.0)
        assertEquals(Math.PI / 4.0, twist(0.0, 0.0, 0.0, -45.0).twist.angular.z, 0.0)
    }

    /**
     * **The exact double `Math.toRadians` produces, pinned on a rate where the spelling
     * matters** — the golden `tools/memexport` mirrors, so that the Python's expected bytes
     * trace to the shipping encoder and not to a probe someone once ran.
     *
     * `java.lang.Math.toRadians` is a single multiply by `DEGREES_TO_RADIANS`
     * (0.017453292519943295), **not** `deg / 180.0 * PI` — it has not been that since JDK 9,
     * and the two are different doubles for some inputs. −84.54 °/s is one of them
     * (`3ff79ba74d9e3f86` here, `…85` from the divide-first form), and it is not a synthetic
     * choice: it is `stick_cmd` seq 122 of
     * `datasets/landing16/20260730-161329.001.jsonl`, t=64.035565, from the first flight this
     * project ever recorded that commanded a nonzero yaw rate. `tools/memexport` mirrored the
     * divide-first spelling on the strength of a comment in this encoder that said so, and
     * the kotlinframes cross-check duly failed on 246 of landing16's 1513 setpoints and 361
     * of landing17's 2613 — every one at byte 83 of 84, the low byte of `angular.z`.
     *
     * The whole frame is pinned rather than the one field, for the same reason the landing10
     * fixture above pins whole frames: a value assertion cannot see a fingerprint, a field
     * order or a stamp move under it.
     */
    @Test
    fun `a yaw rate the two radian spellings disagree on`() {
        // landing16 seq 122: vn 0.658, ve −2.927, vd −1.5 (climbing), yaw rate −84.54 °/s.
        assertEquals(
            "9cd2bcbe6cb2a7c0000000006a6b4e391a4920300000000c64726f6e652f776f726c6400" +
                "c0076a7ef9db22d13fe50e56041893753ff8000000000000" +
                "7ff80000000000007ff80000000000003ff79ba74d9e3f86",
            emit(0.658, -2.927, -1.5, -84.54, unixMs = 1_785_417_273_441).bytes!!.toHex(),
        )
        // The pin above is only doing work while the two spellings really differ here; if a
        // future JDK makes them agree, this fails and says so rather than going quiet.
        assertTrue(
            "deg/180*PI and Math.toRadians agree on -84.54 — the pin above no longer bites",
            -84.54 / 180.0 * Math.PI != Math.toRadians(-84.54),
        )
    }

    /** No roll or pitch rate is ever commanded — NaN, never a confident zero. */
    @Test
    fun `angular x and y are NaN`() {
        val t = twist(1.0, 0.0, 0.0, 0.0)
        assertTrue(t.twist.angular.x.isNaN())
        assertTrue(t.twist.angular.y.isNaN())
    }

    @Test
    fun `the frame is the world frame and the header seq is zero`() {
        val t = twist(1.0, 0.0, 0.0, 0.0)
        assertEquals(ZenohTelemetryEncoder.FRAME_WORLD, t.header.frameId)
        assertEquals(0, t.header.seq)
    }

    // ── the record is the source of truth ────────────────────────────────────

    /**
     * **The wire carries nothing the `stick_cmd` line cannot reproduce.** The record writes
     * velocities at 3 dp and the yaw rate at 2 dp (`Setpoint.VELOCITY_DECIMALS`, `Json.num`'s
     * half-up rule), so the encoder quantises through the same owner before converting —
     * full-precision controller output and its record rendering must produce identical
     * bytes, or the live bus and a mem2 conversion of the same flight would disagree in the
     * last bits of every message.
     */
    @Test
    fun `full-precision input encodes identically to its record rendering`() {
        assertEquals(
            emit(-0.006, 0.007, -1.5, 0.0).bytes!!.toHex(),
            emit(-0.0060499, 0.0070102, -1.4999999, 0.0049).bytes!!.toHex(),
        )
        // Half-up, Java's Math.round — not banker's rounding, which would send 0.0005 to 0.
        assertEquals(0.001, Json.roundTo(0.0005, 3), 0.0)
        assertEquals(0.002, Json.roundTo(0.0015, 3), 0.0)
        // Quantising an already-quantised value is the identity, which is what lets
        // tools/kotlinframes feed record values back through this same function.
        assertEquals(emit(0.434, 0.478, 0.0, 0.0).bytes!!.toHex(),
            emit(Json.roundTo(0.434, 3), Json.roundTo(0.478, 3), 0.0, 0.0).bytes!!.toHex())
    }

    // ── the refusals, by name ────────────────────────────────────────────────

    /**
     * A shadow-mode would-be command (`accepted = null`: never handed to the SDK) is refused
     * as [Withheld.NOT_SENT] however well-formed it is — absence on this channel must mean no
     * command flowed, and a phantom setpoint beside real motion is the lie the gate exists to
     * prevent. A **refused** send (`accepted = false`) still publishes: it *was* sent.
     */
    @Test
    fun `shadow commands are refused by name and refused sends still publish`() {
        val shadow = emit(1.0, 0.0, 0.0, 0.0, accepted = null)
        assertEquals(Withheld.NOT_SENT, shadow.reason)
        assertNull(shadow.bytes)
        val refused = emit(0.5, 0.0, 0.0, 0.0, accepted = false)
        assertEquals(Withheld.PUBLISHED, refused.reason)
        assertEquals(emit(0.5, 0.0, 0.0, 0.0, accepted = true).bytes!!.toHex(),
            refused.bytes!!.toHex())
    }

    @Test
    fun `an unconvertible frame is refused by name, never translated`() {
        val e = emit(1.0, 0.0, 0.0, 0.0, frame = SetpointFrame.BODY_VELOCITY)
        assertEquals(Withheld.SETPOINT_FRAME_UNSUPPORTED, e.reason)
        assertNull(e.bytes)
    }

    @Test
    fun `a missing or non-finite axis withholds the whole message`() {
        assertEquals(Withheld.SETPOINT_INCOMPLETE, emit(null, 0.0, 0.0, 0.0).reason)
        assertEquals(Withheld.SETPOINT_INCOMPLETE, emit(0.0, 0.0, 0.0, null).reason)
        assertEquals(Withheld.SETPOINT_INCOMPLETE, emit(Double.NaN, 0.0, 0.0, 0.0).reason)
        assertEquals(
            Withheld.SETPOINT_INCOMPLETE,
            emit(0.0, Double.POSITIVE_INFINITY, 0.0, 0.0).reason,
        )
        assertEquals(Withheld.SETPOINT_INCOMPLETE, emit(1.0, 1.0, 1.0, 1.0, frame = null).reason)
    }

    /** A zero setpoint is a command; filtering it would fake quiet where a command flowed. */
    @Test
    fun `zeros publish`() {
        assertEquals(Withheld.PUBLISHED, emit(0.0, 0.0, 0.0, 0.0).reason)
    }

    private fun ByteArray.toHex() = joinToString("") { "%02x".format(it) }
}
