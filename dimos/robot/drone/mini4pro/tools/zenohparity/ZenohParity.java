// The `tf`, `camera_info` and `video` bytes the *shipping Kotlin encoder* would publish, for a set
// of planted inputs, so `tools/memexport` can be compared against the real thing rather than
// against a second copy of the rules.
//
// The counterpart of `tools/kotlinframes`, and it exists because those three streams cannot go
// through that one. `KotlinFrames` drives `ZenohReplay` over a flight record, and a `ReplaySample`
// carries no gimbal and no video geometry — so the two newest streams are exactly the two the
// record-driven cross-check cannot reach (`replay/ReplayCoverage.TF_TREE`, `.CAMERA_GEOMETRY`).
// Planted inputs are what is left, and they are enough: both sides are pure functions of the
// numbers below, so agreeing on every one of them is agreeing on the derivation.
//
// This file is deliberately dumb. Every decision it could get wrong — the composition, the frames,
// the fitted focal length, the byte layout — is taken by `zenoh.ZenohTelemetryEncoder`, whose
// compiled classes this runs against. Nothing here re-implements anything.
//
// Java rather than Kotlin because there is no `kotlinc` in this dev shell and there is a JDK; a
// Kotlin `object` is `INSTANCE` and a Kotlin property is a getter. Build and run it with
// `tools/zenohparity/run`.
//
// Output, one JSON object per line:
//
//     {"case": "tf/nadir-30roll-20pitch", "kind": "tf", "len": 331, "hex": "..."}

import com.dimensional.mini4pro.telemetry.AircraftState;
import com.dimensional.mini4pro.telemetry.SampleAges;
import com.dimensional.mini4pro.telemetry.Signal;
import com.dimensional.mini4pro.zenoh.CameraInfoCodec;
import com.dimensional.mini4pro.zenoh.CompressedVideoCodec;
import com.dimensional.mini4pro.zenoh.GimbalEarthAttitude;
import com.dimensional.mini4pro.zenoh.LcmCompressedVideo;
import com.dimensional.mini4pro.zenoh.LcmTime;
import com.dimensional.mini4pro.zenoh.OdomDatum;
import com.dimensional.mini4pro.zenoh.TfMessageCodec;
import com.dimensional.mini4pro.zenoh.ZenohTelemetryEncoder;
import com.dimensional.mini4pro.zenoh.Gate;

import java.util.LinkedHashMap;
import java.util.Map;

public final class ZenohParity {

    /** The reference flight's own origin, so the local metres below are a real place. */
    static final double LAT = 37.9938243;
    static final double LON = 23.7252903;
    static final double TAKEOFF_ALT = 88.5;

    /** One instant, millisecond-quantised exactly as D-5 requires. */
    static final long UNIX_MS = 1785169465521L;

    public static void main(String[] args) {
        StringBuilder out = new StringBuilder();
        LcmTime stamp = LcmTime.Companion.ofEpochSeconds(UNIX_MS / 1000.0);
        OdomDatum datum = new OdomDatum(LAT, LON, TAKEOFF_ALT);

        // ── tf ────────────────────────────────────────────────────────────────
        //
        // Four attitudes: level, `memexport`'s own planted 30 roll / 20 pitch, the flight's
        // measured worst roll, and a heading past 180 so the 90-minus-yaw term cannot hide.
        double[][] attitudes = {
            {0.0, 0.0, 0.0},
            {30.0, 20.0, 118.0},
            {-31.7, 27.3, 250.0},
            {5.5, -3.25, 359.5},
        };
        // Two gimbal pitches: nadir, which is what every flight in this project has flown, and a
        // 45 that is not a special angle in any convention.
        double[] gimbalPitches = {-90.0, -45.0};

        for (double[] a : attitudes) {
            AircraftState s = state(a[0], a[1], a[2]);
            for (double pitch : gimbalPitches) {
                GimbalEarthAttitude g = new GimbalEarthAttitude(
                    0.0, pitch, a[2], GimbalEarthAttitude.Source.COMMANDED, null);
                byte[] bytes = TfMessageCodec.INSTANCE.encode(
                    ZenohTelemetryEncoder.INSTANCE.tfOrNull(s, datum, stamp, g, Gate.HELD));
                emit(out, String.format(
                    "tf/roll%.3f_pitch%.3f_yaw%.3f_gimbal%.3f", a[0], a[1], a[2], pitch),
                    "tf", bytes);
            }
            // And one with no gimbal at all: two edges, and the tree is still published.
            byte[] noCam = TfMessageCodec.INSTANCE.encode(
                ZenohTelemetryEncoder.INSTANCE.tfOrNull(s, datum, stamp, null, Gate.HELD));
            emit(out, String.format(
                "tf/roll%.3f_pitch%.3f_yaw%.3f_nogimbal", a[0], a[1], a[2]), "tf", noCam);
        }

        // ── camera_info ───────────────────────────────────────────────────────
        int[][] resolutions = {{1920, 1080}, {960, 540}, {1280, 720}, {3840, 2160}};
        for (int[] r : resolutions) {
            byte[] bytes = CameraInfoCodec.INSTANCE.encode(
                ZenohTelemetryEncoder.INSTANCE.cameraInfo(r[0], r[1], stamp));
            emit(out, String.format("camera_info/%dx%d", r[0], r[1]), "camera_info", bytes);
        }

        // ── video ─────────────────────────────────────────────────────────────
        //
        // An access unit shaped like a real one — start code, SPS, start code, IDR — and a long
        // one, because the payload sits between two strings and a length bug shows up on size.
        byte[] shortUnit = new byte[] {
            0, 0, 0, 1, 0x67, 0x42, (byte) 0xC0, 0x1F,
            0, 0, 0, 1, 0x65, (byte) 0x88, (byte) 0x84, 0x00,
        };
        byte[] longUnit = new byte[17000];
        System.arraycopy(shortUnit, 0, longUnit, 0, shortUnit.length);
        for (int i = shortUnit.length; i < longUnit.length; i++) longUnit[i] = (byte) (i % 251);

        emit(out, "video/short", "video", video(stamp, shortUnit));
        emit(out, "video/17k", "video", video(stamp, longUnit));

        System.out.print(out);
    }

    private static byte[] video(LcmTime stamp, byte[] data) {
        return CompressedVideoCodec.INSTANCE.encode(new LcmCompressedVideo(
            stamp, ZenohTelemetryEncoder.FRAME_CAMERA_OPTICAL, data, "h264"));
    }

    /**
     * 30 m east, 40 m north, 12 m up of the origin, every signal fresh, link alive.
     *
     * Every field written out, because Kotlin's default arguments are not visible from Java and
     * the alternative — reflection into `copy$default` — would be a harness that keeps compiling
     * while the thing it drives changes shape. If `AircraftState` grows a field this stops
     * building, which is the outcome to want in a cross-check.
     */
    private static AircraftState state(double roll, double pitch, double yaw) {
        Map<Signal, Long> ages = new LinkedHashMap<>();
        ages.put(Signal.POSITION, 50L);
        ages.put(Signal.ALTITUDE, 50L);
        ages.put(Signal.ATTITUDE, 50L);
        ages.put(Signal.VELOCITY, 50L);
        return new AircraftState(
            /* fcConnected        */ true,
            /* latitude           */ LAT + 40.0 / 111194.93,
            /* longitude          */ LON + 30.0 / (111194.93 * Math.cos(Math.toRadians(LAT))),
            /* relativeAltitude   */ 12.0,
            /* takeoffAltitudeAmsl*/ TAKEOFF_ALT,
            /* rollDeg            */ roll,
            /* pitchDeg           */ pitch,
            /* yawDeg             */ yaw,
            /* velocityNorth      */ 3.0,
            /* velocityEast       */ 4.0,
            /* velocityDown       */ -1.5,
            /* satelliteCount     */ 14,
            /* gpsSignalLevel     */ 5,
            /* homeLatitude       */ null,
            /* homeLongitude      */ null,
            /* homeLocationSet    */ null,
            /* isFlying           */ Boolean.TRUE,
            /* motorsOn           */ Boolean.TRUE,
            /* flightMode         */ "GPS_ATTI",
            /* notAllowMotorStart */ null,
            /* imuWarmingUp       */ null,
            /* inFailsafe         */ null,
            /* goHomeHeightM      */ null,
            /* batteryPercent     */ 62,
            /* voltageMv          */ 8371,
            /* currentMa          */ -4200,
            /* cellCount          */ 2,
            /* cellVoltagesMv     */ null,
            /* batteryTempC       */ 27.5,
            /* ages               */ SampleAges.Companion.of(ages));
    }

    private static void emit(StringBuilder out, String name, String kind, byte[] bytes) {
        out.append("{\"case\": \"").append(name).append("\", \"kind\": \"").append(kind)
            .append("\", \"len\": ").append(bytes.length).append(", \"hex\": \"");
        for (byte b : bytes) out.append(String.format("%02x", b));
        out.append("\"}\n");
    }
}
