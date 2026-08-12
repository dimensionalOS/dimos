// Dump the LCM frames the *shipping Kotlin encoder* would have published for a flight
// record, so a Python tool can be compared against the real thing rather than against a
// second copy of the rules.
//
// This file is deliberately dumb. Every decision it could get wrong -- gating, frames,
// units, quaternion convention, byte layout -- is taken by
// `com.dimensional.mini4pro.replay.ZenohReplay` and `zenoh.ZenohTelemetryEncoder`, whose
// compiled classes this runs against. It reads a record, asks them what would have gone
// out, and prints the bytes as hex. Nothing here re-implements anything, because the whole
// point of a cross-check is that the two sides were not written by the same hand.
//
// Java rather than Kotlin because there is no `kotlinc` in this dev shell and there is a
// JDK; the classes are ordinary JVM bytecode, so a Kotlin `object` is `INSTANCE` and a
// Kotlin property is a getter. Build and run it with `tools/kotlinframes/run`.
//
// Output, one JSON object per line, the same `mini4pro-zenohframes-1` shape
// `tools/zenohreplay --frames` writes:
//
//     {"format": "...", "session": "...", "started_unix_ms": ..., "datum": {...}}
//     {"t": 31.205990, "unix": 1785160432.959, "ch": "pose", "len": 92, "hex": "..."}
//
// It writes no store, decodes nothing, and has no opinion about mem2.

import com.dimensional.mini4pro.replay.FlightRecord;
import com.dimensional.mini4pro.replay.FlightRecordReader;
import com.dimensional.mini4pro.replay.FlightReplay;
import com.dimensional.mini4pro.replay.ReplaySample;
import com.dimensional.mini4pro.replay.ZenohReplay;
import com.dimensional.mini4pro.vision.TagCorners;
import com.dimensional.mini4pro.vision.TagPoseSolve;
import com.dimensional.mini4pro.vision.TagSighting;
import com.dimensional.mini4pro.zenoh.LcmTime;
import com.dimensional.mini4pro.zenoh.OdomDatum;
import com.dimensional.mini4pro.zenoh.Withheld;
import com.dimensional.mini4pro.zenoh.ZenohChannel;
import com.dimensional.mini4pro.zenoh.ZenohEmission;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import kotlin.sequences.SequencesKt;

public final class KotlinFrames {

    public static void main(String[] args) throws IOException {
        if (args.length < 2) {
            System.err.println("usage: KotlinFrames <out.jsonl> <record.jsonl> [more parts...]");
            System.exit(2);
        }
        Path out = Paths.get(args[0]);
        List<String> lines = new ArrayList<>();
        for (int i = 1; i < args.length; i++) {
            lines.addAll(Files.readAllLines(Paths.get(args[i]), StandardCharsets.UTF_8));
        }

        Iterator<String> it = lines.iterator();
        FlightRecordReader.Options options =
                new FlightRecordReader.Options(FlightRecordReader.INSTANCE.getREPLAY_KINDS(), true);
        FlightRecord record = FlightRecordReader.INSTANCE.read(SequencesKt.asSequence(it), options);
        List<ReplaySample> samples = FlightReplay.INSTANCE.samples(record, true);
        ZenohReplay.Result result =
                ZenohReplay.INSTANCE.run(samples, new ZenohReplay.Options(true));

        Long startedUnixMs = record.getHeader() == null ? null : record.getHeader().getStartedUnixMillis();
        String session = record.getHeader() == null ? null : record.getHeader().getSession();
        OdomDatum datum = result.getDatum();

        try (BufferedWriter w = Files.newBufferedWriter(out, StandardCharsets.UTF_8)) {
            StringBuilder head = new StringBuilder();
            head.append("{\"format\":\"mini4pro-zenohframes-1\"");
            head.append(",\"source\":\"kotlin\"");
            head.append(",\"session\":").append(json(session));
            head.append(",\"started_unix_ms\":").append(startedUnixMs);
            head.append(",\"samples\":").append(samples.size());
            if (datum != null) {
                head.append(",\"datum\":{\"lat\":").append(repr(datum.getLatitudeDeg()))
                        .append(",\"lon\":").append(repr(datum.getLongitudeDeg()))
                        .append(",\"alt\":").append(datum.getTakeoffAltitudeM() == null
                                ? "null" : repr(datum.getTakeoffAltitudeM()))
                        .append(",\"t\":").append(result.getDatumAtSeconds()).append("}");
            } else {
                head.append(",\"datum\":null");
            }
            head.append(",\"counts\":{");
            boolean first = true;
            for (ZenohChannel ch : ZenohChannel.values()) {
                if (!first) head.append(",");
                first = false;
                head.append(json(ch.getChannel())).append(":").append(result.get(ch).getPublished());
            }
            head.append("}}");
            w.write(head.toString());
            w.newLine();

            // The stamp a frame carries is the sample's own, from the record's header --
            // exactly what ZenohReplay handed the encoder. Recomputing it here would make
            // this file a second implementation of §5.1's rounding, which is the one thing
            // a cross-check tool must not be.
            java.util.Map<Double, Long> unixByT = new java.util.HashMap<>();
            for (ReplaySample s : samples) {
                if (s.getUnixMillis() != null) unixByT.put(s.getTSeconds(), s.getUnixMillis());
            }

            for (ZenohReplay.Frame f : result.getFrames()) {
                Long ms = unixByT.get(f.getTSeconds());
                StringBuilder b = new StringBuilder();
                b.append("{\"t\":").append(repr(f.getTSeconds()));
                b.append(",\"unix\":").append(ms == null ? "null" : repr(ms / 1000.0));
                b.append(",\"ch\":").append(json(f.getChannel().getChannel()));
                b.append(",\"len\":").append(f.getBytes().length);
                b.append(",\"hex\":\"").append(hex(f.getBytes())).append("\"}");
                w.write(b.toString());
                w.newLine();
            }

            // Detections, from the record's own `tag` lines through the SHIPPING encoder.
            //
            // ZenohReplay cannot produce these (ReplayCoverage.TAG_SIGHTINGS: a sighting is
            // not an AircraftState), so this harness rebuilds each Sighting from the line's
            // fields -- which are the Sighting's fields, written by TagTap -- and asks
            // ZenohEmission.detection for the bytes. The gates, the field mapping and the
            // byte layout all stay the encoder's; the only arithmetic done here is the
            // record's mono->wall mapping, the same floor(x + 0.5) millisecond the sample
            // stamps use. `seq` counts published messages from 1, as the live counter does.
            long detections = 0;
            if (startedUnixMs != null) {
                for (String line : lines) {
                    if (!line.contains("\"k\":\"tag\"")) continue;
                    Double t = num(line, "t");
                    Double id = num(line, "id");
                    if (t == null || id == null) continue;
                    TagSighting.Sighting s = new TagSighting.Sighting(
                            (int) (double) id,
                            val(num(line, "x")), val(num(line, "y")), val(num(line, "z")),
                            0L,                                   // atNanos: not encoded
                            val(num(line, "px")),
                            line.contains("\"metric\":true"),
                            (int) val(num(line, "ham")),
                            val(num(line, "margin")),
                            val(num(line, "cx")), val(num(line, "cy")),
                            (int) val(num(line, "w")), (int) val(num(line, "h")),
                            corners(line), solve(line));
                    long unixMs = (long) Math.floor(startedUnixMs + t * 1000.0 + 0.5);
                    LcmTime stamp = LcmTime.Companion.ofEpochSeconds(unixMs / 1000.0);
                    ZenohEmission.Emission em = ZenohEmission.INSTANCE.detection(
                            s, stamp, (int) detections + 1, true);
                    byte[] bytes = em.getBytes();
                    if (bytes == null) continue;
                    detections++;
                    StringBuilder b = new StringBuilder();
                    b.append("{\"t\":").append(repr(t));
                    b.append(",\"unix\":").append(repr(unixMs / 1000.0));
                    b.append(",\"ch\":\"detections\"");
                    b.append(",\"len\":").append(bytes.length);
                    b.append(",\"hex\":\"").append(hex(bytes)).append("\"}");
                    w.write(b.toString());
                    w.newLine();
                }
            }
            // The tag's world-frame fix, from the same `tag` lines through the SHIPPING
            // encoder -- the record's own `n`/`e` values, never a re-derivation, which is the
            // channel's founding rule (ZenohTelemetryEncoder.tagFixOrNull). A line without a
            // fix is refused by ZenohEmission.tagFixReason exactly as the live path refuses
            // it; `seq` counts published fixes from 1, as the live counter does.
            long tagFixes = 0;
            if (startedUnixMs != null) {
                for (String line : lines) {
                    if (!line.contains("\"k\":\"tag\"")) continue;
                    Double t = num(line, "t");
                    Double id = num(line, "id");
                    if (t == null || id == null) continue;
                    long unixMs = (long) Math.floor(startedUnixMs + t * 1000.0 + 0.5);
                    LcmTime stamp = LcmTime.Companion.ofEpochSeconds(unixMs / 1000.0);
                    ZenohEmission.Emission em = ZenohEmission.INSTANCE.tagFix(
                            (int) (double) id,
                            val(num(line, "margin")),
                            num(line, "n"),
                            num(line, "e"),
                            line.contains("\"fix_metric\":true"),
                            str(line, "range_src"),
                            line.contains("\"pitch_reported\":true"),
                            stamp, (int) tagFixes + 1, true);
                    byte[] bytes = em.getBytes();
                    if (bytes == null) continue;
                    tagFixes++;
                    writeFrame(w, t, unixMs, "tag_fix", bytes);
                }
            }
            System.out.printf("  %-13s %6d  (from tag lines, via ZenohEmission)%n",
                    "tag_fix", tagFixes);

            // The commanded velocity, from the record's `stick_cmd` lines through the
            // SHIPPING encoder. The gates are ZenohEmission.setpointReason's: a shadow line
            // (`accepted` absent -- never handed to the SDK) is refused as NOT_SENT, exactly
            // as the live path refuses it, and the encoder re-quantises the already-quantised
            // record values (idempotent -- Json.roundTo's own rule).
            long setpoints = 0;
            if (startedUnixMs != null) {
                for (String line : lines) {
                    if (!line.contains("\"k\":\"stick_cmd\"")) continue;
                    Double t = num(line, "t");
                    if (t == null) continue;
                    Boolean accepted = line.contains("\"accepted\":true") ? Boolean.TRUE
                            : line.contains("\"accepted\":false") ? Boolean.FALSE : null;
                    long unixMs = (long) Math.floor(startedUnixMs + t * 1000.0 + 0.5);
                    LcmTime stamp = LcmTime.Companion.ofEpochSeconds(unixMs / 1000.0);
                    ZenohEmission.Emission em = ZenohEmission.INSTANCE.setpoint(
                            str(line, "frame"),
                            num(line, "vn"),
                            num(line, "ve"),
                            num(line, "vd"),
                            num(line, "yawrate"),
                            accepted, stamp, true);
                    byte[] bytes = em.getBytes();
                    if (bytes == null) continue;
                    setpoints++;
                    writeFrame(w, t, unixMs, "setpoint", bytes);
                }
            }
            System.out.printf("  %-13s %6d  (from stick_cmd lines, via ZenohEmission)%n",
                    "setpoint", setpoints);

            // The wind reading, from the record's `dji_field` windSpeedDmS lines through the
            // SHIPPING encoder. One line = one delivery = one message (the recorder's
            // on-change dedup is the single owner of that equality, and it already ran when
            // the line was written); a value-less line -- DJI withdrawing the reading -- is
            // refused by ZenohEmission.windReason exactly as the live path refuses it. The
            // dm/s->m/s division happens inside windOrNull, the one owner, so this harness
            // hands over the line's own integer and nothing else.
            long winds = 0;
            if (startedUnixMs != null) {
                for (String line : lines) {
                    if (!line.contains("\"k\":\"dji_field\"")
                            || !line.contains("\"f\":\"windSpeedDmS\"")) {
                        continue;
                    }
                    Double t = num(line, "t");
                    if (t == null) continue;
                    String v = str(line, "v");
                    Integer dms = null;
                    if (v != null) {
                        try {
                            dms = Integer.valueOf(v);
                        } catch (NumberFormatException e) {
                            dms = null;
                        }
                    }
                    ZenohEmission.Emission em = ZenohEmission.INSTANCE.wind(dms, true);
                    byte[] bytes = em.getBytes();
                    if (bytes == null) continue;
                    winds++;
                    long unixMs = (long) Math.floor(startedUnixMs + t * 1000.0 + 0.5);
                    writeFrame(w, t, unixMs, "wind", bytes);
                }
            }
            System.out.printf("  %-13s %6d  (from dji_field windSpeedDmS lines, via ZenohEmission)%n",
                    "wind", winds);

            System.out.printf("  %-13s %6d  (from tag lines, via ZenohEmission)%n",
                    "detections", detections);
        }

        PrintStream o = System.out;
        o.printf("kotlin: %d samples, %d frames -> %s%n",
                samples.size(), result.getFrames().size(), out);
        for (ZenohChannel ch : ZenohChannel.values()) {
            ZenohReplay.ChannelResult r = result.get(ch);
            StringBuilder why = new StringBuilder();
            for (Map.Entry<Withheld, Integer> e : r.getReasons().entrySet()) {
                if (e.getKey() == Withheld.PUBLISHED) continue;
                if (why.length() > 0) why.append(", ");
                why.append(e.getKey().name().toLowerCase()).append(" ").append(e.getValue());
            }
            o.printf("  %-13s %6d  %s%n", ch.getChannel(), r.getPublished(),
                    why.length() == 0 ? "-" : why.toString());
        }
    }

    /**
     * The four corners off a `tag` line, all eight or nothing — a record older than the fields
     * (2026-07-28) reads as "corners not measured", which is the backward-compat contract.
     * They do not affect the encoder's bytes; they are reconstructed so the Sighting is the
     * Sighting, not the subset of it a byte comparison happens to need.
     */
    private static TagCorners corners(String line) {
        Double[] c = new Double[8];
        String[] keys = {"c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y"};
        for (int i = 0; i < 8; i++) {
            c[i] = num(line, keys[i]);
            if (c[i] == null) return null;
        }
        return new TagCorners(c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]);
    }

    /**
     * The raw pose solve off a `tag` line, or null on a line that predates it — which makes the
     * rebuilt Sighting produce the LEGACY message through the shipping encoder, exactly as the
     * live path would for a solve-less sighting. `e2` may be the format's `"Infinity"` string
     * (no second PnP minimum, the unambiguous case) and must survive as a real infinity: a NaN
     * there would read as a partial solve and be gated out, silently un-solving every
     * unambiguous frame.
     */
    private static TagPoseSolve solve(String line) {
        String[] keys = {"qx", "qy", "qz", "qw", "tx", "ty", "tz", "e1", "tag_m"};
        Double[] v = new Double[9];
        for (int i = 0; i < keys.length; i++) {
            v[i] = num(line, keys[i]);
            if (v[i] == null) return null;
        }
        double e2;
        if (line.contains("\"e2\":\"Infinity\"")) {
            e2 = Double.POSITIVE_INFINITY;
        } else {
            e2 = val(num(line, "e2"));
        }
        return new TagPoseSolve(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], e2, v[8]);
    }

    /** One frame line of the `mini4pro-zenohframes-1` output. */
    private static void writeFrame(BufferedWriter w, double t, long unixMs, String ch, byte[] bytes)
            throws IOException {
        StringBuilder b = new StringBuilder();
        b.append("{\"t\":").append(repr(t));
        b.append(",\"unix\":").append(repr(unixMs / 1000.0));
        b.append(",\"ch\":").append(json(ch));
        b.append(",\"len\":").append(bytes.length);
        b.append(",\"hex\":\"").append(hex(bytes)).append("\"}");
        w.write(b.toString());
        w.newLine();
    }

    /** The string value of `"key":"value"` in a compact JSON line, or null when absent. */
    private static String str(String line, String key) {
        String needle = "\"" + key + "\":\"";
        int i = line.indexOf(needle);
        if (i < 0) return null;
        int start = i + needle.length();
        int end = line.indexOf('"', start);
        if (end < 0) return null;
        return line.substring(start, end);
    }

    /** The numeric value of `"key":<number>` in a compact JSON line, or null when absent. */
    private static Double num(String line, String key) {
        String needle = "\"" + key + "\":";
        int i = line.indexOf(needle);
        if (i < 0) return null;
        int start = i + needle.length();
        int end = start;
        while (end < line.length() && "-+.eE0123456789".indexOf(line.charAt(end)) >= 0) end++;
        if (end == start) return null;
        try {
            return Double.parseDouble(line.substring(start, end));
        } catch (NumberFormatException e) {
            return null;
        }
    }

    /** Absent-as-NaN, so a missing field withholds through the encoder's own gate. */
    private static double val(Double d) {
        return d == null ? Double.NaN : d;
    }

    private static final char[] HEXITS = "0123456789abcdef".toCharArray();

    private static String hex(byte[] b) {
        char[] c = new char[b.length * 2];
        for (int i = 0; i < b.length; i++) {
            c[i * 2] = HEXITS[(b[i] >> 4) & 0xf];
            c[i * 2 + 1] = HEXITS[b[i] & 0xf];
        }
        return new String(c);
    }

    /** A double as the shortest string that reads back to the same double. */
    private static String repr(double d) {
        if (Double.isNaN(d)) return "null";
        if (Double.isInfinite(d)) return "null";
        return Double.toString(d);
    }

    private static String json(String s) {
        if (s == null) return "null";
        StringBuilder b = new StringBuilder("\"");
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '"' || c == '\\') b.append('\\').append(c);
            else if (c < 0x20) b.append(String.format("\\u%04x", (int) c));
            else b.append(c);
        }
        return b.append('"').toString();
    }
}
