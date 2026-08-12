// Run M3 Stage D's COMPILED descent law over a recorded flight and emit, as jsonl, the
// commands it would have flown -- beside the human's actual sticks, mapped through the
// shipping StickMapping -- so `compare.py` can say whether the law agrees with a person.
//
// This file is deliberately dumb, on `tools/kotlinframes`' model. Every decision it could
// get wrong is taken by compiled app classes:
//
//   - the state timeline comes from `replay.FlightReplay` (the same reconstruction every
//     replay tool uses);
//   - the latch is `vision.TagLatch` -- three sightings in two seconds, the real rule;
//   - the machine is `guided.TagDescent` + `TagDescentGuidance` -- the phase ladder, the
//     cone, the laws, byte for byte what the aircraft would run;
//   - the human's sticks go through `guided.StickMapping.rcVelocities` -- THE mapping,
//     never a second one -- and are rotated from the sticks' body frame into north/east by
//     the recorded heading (the same rotation `TagWorld.fix` uses, restated here only
//     because a frame rotation is bookkeeping, not a mapping).
//
// What this harness decides itself, stated because it is the whole of its own surface:
// the auto-arm gates use `TagDescentGuidance`'s own constants over the record's latch, fix
// age and altitude (the SDK/RC-feed/interlock gates are engagement facts a recording does
// not have); the nadir gate is implied rather than checked, because a record's `tag` line
// only carries a fix when the on-board `TagWorld.fix` already passed its own nadir gate;
// the manoeuvre timeout and the terminal idle window are not applied (this is a law
// comparison, not a watchdog test); and segments end/re-arm on the shadow-mode semantics,
// so one manual landing yields a continuous timeline.
//
// Output, one JSON object per line:
//   {"k":"header","session":...,"samples":N,"tags":N,"sticks":N}
//   {"k":"arm","t":65.71,"id":0,"height":5.2}
//   {"k":"cmd","t":65.79,"phase":"tracking","vn":..,"ve":..,"vd":..,"err_n":..,"err_e":..,
//    "height":..,"fix_age_ms":..}
//   {"k":"end","t":80.10,"why":"tag gone 10023ms"}
//   {"k":"human","t":65.80,"vn":..,"ve":..,"vd":..,"yaw_rate":..}

import com.dimensional.mini4pro.guided.RcSticks;
import com.dimensional.mini4pro.guided.StickMapping;
import com.dimensional.mini4pro.guided.StickVelocities;
import com.dimensional.mini4pro.guided.TagDescent;
import com.dimensional.mini4pro.guided.TagDescentGuidance;
import com.dimensional.mini4pro.replay.FlightRecord;
import com.dimensional.mini4pro.replay.FlightRecordReader;
import com.dimensional.mini4pro.replay.FlightReplay;
import com.dimensional.mini4pro.replay.ReplaySample;
import com.dimensional.mini4pro.telemetry.AircraftState;
import com.dimensional.mini4pro.telemetry.Geo;
import com.dimensional.mini4pro.vision.TagDetection;
import com.dimensional.mini4pro.vision.TagFix;
import com.dimensional.mini4pro.vision.TagLatch;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import kotlin.Pair;
import kotlin.sequences.SequencesKt;

public final class DescentReplay {

    /** One `tag` line's useful part: when, which id, and where the on-board fix put it. */
    private static final class Tag {
        double t;
        int id;
        double cx, cy, px;
        Double n, e, fromH;
    }

    /** One `rc_stick` line: when, and the four raw axes. */
    private static final class Stick {
        double t;
        Integer lh, lv, rh, rv;
    }

    public static void main(String[] args) throws IOException {
        if (args.length < 2) {
            System.err.println("usage: DescentReplay <out.jsonl> <record.jsonl> [more parts...]");
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

        List<Tag> tags = new ArrayList<>();
        List<Stick> sticks = new ArrayList<>();
        for (String line : lines) {
            if (line.contains("\"k\":\"tag\"")) {
                Double t = num(line, "t");
                Double id = num(line, "id");
                if (t == null || id == null) continue;
                Tag g = new Tag();
                g.t = t;
                g.id = (int) (double) id;
                g.cx = val(num(line, "cx"));
                g.cy = val(num(line, "cy"));
                g.px = val(num(line, "px"));
                g.n = num(line, "n");
                g.e = num(line, "e");
                g.fromH = num(line, "from_h");
                tags.add(g);
            } else if (line.contains("\"k\":\"rc_stick\"")) {
                Double t = num(line, "t");
                if (t == null) continue;
                Stick s = new Stick();
                s.t = t;
                s.lh = intOrNull(num(line, "lh"));
                s.lv = intOrNull(num(line, "lv"));
                s.rh = intOrNull(num(line, "rh"));
                s.rv = intOrNull(num(line, "rv"));
                sticks.add(s);
            }
        }

        String session = record.getHeader() == null ? null : record.getHeader().getSession();
        TagLatch latch = new TagLatch(TagLatch.DEFAULT_MIN_SIGHTINGS, TagLatch.DEFAULT_WINDOW_NANOS);

        try (BufferedWriter w = Files.newBufferedWriter(out, StandardCharsets.UTF_8)) {
            w.write(String.format(Locale.US,
                    "{\"k\":\"header\",\"format\":\"mini4pro-descentreplay-1\",\"session\":%s,"
                            + "\"samples\":%d,\"tags\":%d,\"sticks\":%d}",
                    session == null ? "null" : "\"" + session + "\"",
                    samples.size(), tags.size(), sticks.size()));
            w.newLine();

            // The controller's pass: samples in order, tag lines fed to the latch and the
            // newest-fix cursor as their time comes -- the recogniser's own delivery order.
            TagDescent machine = null;
            int tagIdx = 0;
            Integer latchedId = null;
            Double fixN = null, fixE = null;
            double fixT = Double.NEGATIVE_INFINITY;
            long commands = 0, arms = 0;

            for (ReplaySample sample : samples) {
                double t = sample.getTSeconds();
                while (tagIdx < tags.size() && tags.get(tagIdx).t <= t) {
                    Tag g = tags.get(tagIdx++);
                    TagFix fix = null;
                    if (g.n != null && g.e != null) {
                        fix = new TagFix(g.id, g.n, g.e, val(g.fromH), (long) (g.t * 1e9),
                                g.px, false, true);
                    }
                    latch.observe(new TagDetection(g.id, 0, g.cx, g.cy, g.px, 0.0),
                            fix, (long) (g.t * 1e9));
                    if (latch.isLatched()) latchedId = latch.latched().getTagId();
                    // The newest believed fix: id-matched against the latch, exactly as the
                    // engine ingests -- an impostor id must not steer or rejuvenate.
                    if (fix != null && latchedId != null && g.id == latchedId) {
                        fixN = g.n;
                        fixE = g.e;
                        fixT = g.t;
                    }
                }

                AircraftState st = sample.getState();
                Double lat = st.getLatitude(), lon = st.getLongitude();
                Double hLat = st.getHomeLatitude(), hLon = st.getHomeLongitude();
                Double alt = st.getRelativeAltitude();
                long fixAgeMs = (long) ((t - fixT) * 1000.0);

                if (machine == null) {
                    boolean gates = latch.isLatched() && fixN != null
                            && fixAgeMs <= TagDescentGuidance.ARM_FRESH_MS
                            && lat != null && lon != null && hLat != null && hLon != null
                            && alt != null && alt <= TagDescentGuidance.ARM_CEILING_M;
                    if (gates) {
                        machine = new TagDescent();
                        arms++;
                        w.write(String.format(Locale.US,
                                "{\"k\":\"arm\",\"t\":%.6f,\"id\":%d,\"height\":%.2f}",
                                t, latchedId, alt));
                        w.newLine();
                    }
                    continue;
                }

                if (lat == null || lon == null || hLat == null || hLon == null) {
                    machine = null;
                    w.write(String.format(Locale.US,
                            "{\"k\":\"end\",\"t\":%.6f,\"why\":\"no position\"}", t));
                    w.newLine();
                    continue;
                }
                Pair<Double, Double> ne = Geo.INSTANCE.nedMetres(hLat, hLon, lat, lon);
                TagDescent.Step step = machine.step(
                        alt, fixN - ne.getFirst(), fixE - ne.getSecond(), fixAgeMs);
                if (step instanceof TagDescent.Step.HandBack) {
                    w.write(String.format(Locale.US,
                            "{\"k\":\"end\",\"t\":%.6f,\"why\":\"tag gone %dms\"}", t, fixAgeMs));
                    w.newLine();
                    machine = null;
                    continue;
                }
                TagDescent.Step.Fly fly = (TagDescent.Step.Fly) step;
                StickVelocities v = fly.getVelocities();
                commands++;
                w.write(String.format(Locale.US,
                        "{\"k\":\"cmd\",\"t\":%.6f,\"phase\":\"%s\",\"vn\":%.4f,\"ve\":%.4f,"
                                + "\"vd\":%.4f,\"err_n\":%.3f,\"err_e\":%.3f,\"height\":%s,"
                                + "\"fix_age_ms\":%d}",
                        t, machine.getPhase().name().toLowerCase(Locale.US),
                        v.getNorth(), v.getEast(), v.getDown(),
                        fixN - ne.getFirst(), fixE - ne.getSecond(),
                        alt == null ? "null" : String.format(Locale.US, "%.2f", alt),
                        fixAgeMs));
                w.newLine();
            }

            // The human's pass: every rc_stick line through the shipping mapping, rotated
            // into north/east by the nearest sample's heading. A line with a null axis or no
            // usable heading is skipped and counted -- absence, never zero.
            long human = 0, skipped = 0;
            int cursor = 0;
            for (Stick s : sticks) {
                while (cursor + 1 < samples.size()
                        && samples.get(cursor + 1).getTSeconds() <= s.t) {
                    cursor++;
                }
                Double yaw = samples.isEmpty() ? null : samples.get(cursor).getState().getYawDeg();
                StickVelocities body = (s.lh == null || s.lv == null || s.rh == null || s.rv == null)
                        ? null
                        : StickMapping.INSTANCE.rcVelocities(new RcSticks(s.lh, s.lv, s.rh, s.rv));
                if (body == null || yaw == null) {
                    skipped++;
                    continue;
                }
                double rad = Math.toRadians(yaw);
                double vn = body.getNorth() * Math.cos(rad) - body.getEast() * Math.sin(rad);
                double ve = body.getNorth() * Math.sin(rad) + body.getEast() * Math.cos(rad);
                human++;
                w.write(String.format(Locale.US,
                        "{\"k\":\"human\",\"t\":%.6f,\"vn\":%.4f,\"ve\":%.4f,\"vd\":%.4f,"
                                + "\"yaw_rate\":%.2f}",
                        s.t, vn, ve, body.getDown(), body.getYawRateDegPerS()));
                w.newLine();
            }

            System.out.printf(Locale.US,
                    "descentreplay: %d samples, %d tag lines, %d stick lines -> %s%n",
                    samples.size(), tags.size(), sticks.size(), out);
            System.out.printf(Locale.US,
                    "  controller: %d arm(s), %d would-be commands%n", arms, commands);
            System.out.printf(Locale.US,
                    "  human: %d stick lines mapped, %d skipped%n", human, skipped);
        }
    }

    private static Integer intOrNull(Double d) {
        return d == null ? null : (int) (double) d;
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

    private static double val(Double d) {
        return d == null ? Double.NaN : d;
    }
}
