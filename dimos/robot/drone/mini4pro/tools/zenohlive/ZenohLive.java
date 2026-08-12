// Put a recorded flight onto a REAL Zenoh bus using the app's OWN publisher classes.
//
//     tools/zenohlive/run <record.jsonl> [--endpoint tcp/10.55.1.50:7447] [--speed N]
//
// Why this exists, and what it is not. `tools/zenohpublish` already puts our contract on a real
// bus — but it is Python, and what it proves is that the *bytes* and the *key expressions* are
// right. It proves nothing at all about the Kotlin that will actually run on the phone.
//
// This runs `ZenohPublisher`, `ZenohTelemetryPump`, `ZenohEmission` and `ZenohKotlinSink` —
// the shipping classes, as compiled — against `org.eclipse.zenoh:zenoh-kotlin` (the JVM build of
// the same binding the Android AAR ships). So it exercises, for real:
//
//   * `Config.fromJson5` accepting the config string `ZenohConfig.json5()` produces;
//   * `KeyExpr.tryFrom` accepting the key expressions `ZenohChannel` builds;
//   * `declarePublisher` accepting the QoS `ZenohQos` maps to;
//   * `Publisher.put` carrying the encoder's bytes;
//   * the whole bounded-queue publisher lifecycle, against a session that can really fail.
//
// What it CANNOT prove, and nothing on a desktop can: that `libzenoh_jni.so` loads in a process
// that already holds MSDK and OpenCV natives, on arm64, on Android. That is the one thing left,
// and `docs/zenoh-android-transport.md` §7 is the list.
//
// Verify the other end with the tool that already speaks this contract:
//
//     tools/zenohpublish --spy
//
import com.dimensional.mini4pro.replay.FlightRecord;
import com.dimensional.mini4pro.replay.FlightRecordReader;
import com.dimensional.mini4pro.replay.FlightReplay;
import com.dimensional.mini4pro.replay.ReplaySample;
import com.dimensional.mini4pro.zenoh.ZenohChannel;
import com.dimensional.mini4pro.zenoh.ZenohConfig;
import com.dimensional.mini4pro.zenoh.ZenohKotlinSink;
import com.dimensional.mini4pro.zenoh.ZenohPublisher;
import com.dimensional.mini4pro.zenoh.ZenohTelemetryPump;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import kotlin.sequences.SequencesKt;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;
import kotlin.jvm.functions.Function1;
import kotlin.jvm.functions.Function2;

public final class ZenohLive {

    public static void main(String[] args) throws Exception {
        String record = null;
        String endpoint = "tcp/10.55.1.50:7447";
        String prefix = "dimos/drone";
        String mode = ZenohConfig.MODE_CLIENT;
        double speed = 10.0;
        int limit = Integer.MAX_VALUE;
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--endpoint": endpoint = args[++i]; break;
                case "--prefix": prefix = args[++i]; break;
                case "--mode": mode = args[++i]; break;
                case "--speed": speed = Double.parseDouble(args[++i]); break;
                case "--limit": limit = Integer.parseInt(args[++i]); break;
                default: record = args[i];
            }
        }
        if (record == null) {
            System.err.println("usage: ZenohLive <record.jsonl> [--endpoint E] [--speed N] [--limit N]");
            System.exit(2);
        }

        FlightRecordReader.Options options =
                new FlightRecordReader.Options(FlightRecordReader.INSTANCE.getREPLAY_KINDS(), true);
        FlightRecord flight = FlightRecordReader.INSTANCE.read(
                SequencesKt.asSequence(Files.readAllLines(Path.of(record), StandardCharsets.UTF_8).iterator()),
                options);
        List<ReplaySample> samples = FlightReplay.INSTANCE.samples(flight, true);
        System.out.printf("record: %d samples, %.1f s%n", samples.size(), flight.getDurationSeconds());

        ZenohConfig config = new ZenohConfig(endpoint, prefix, null, mode);
        System.out.println("config: " + config.getConnectEndpoint() + "  mode " + config.getMode());

        final long[] mono = {0};
        ZenohPublisher publisher = new ZenohPublisher(
                config,
                ZenohKotlinSink.Companion.getFACTORY(),
                new ZenohPublisher.Settings(512, 1000L, 30000L, 100L),
                (Function0<Long>) System::currentTimeMillis,
                (Function1<String, Unit>) m -> { System.out.println("  log: " + m); return Unit.INSTANCE; },
                (Function2<ZenohPublisher.Phase, String, Unit>) (phase, why) -> {
                    System.out.println("  phase: " + phase + " — " + why);
                    return Unit.INSTANCE;
                });

        ZenohTelemetryPump pump = new ZenohTelemetryPump(
                (Function2<ZenohChannel, byte[], Boolean>) publisher::offer,
                new ZenohTelemetryPump.Cadence(200, 200, 200, 200, 1000, 1000),
                (Function1<String, Unit>) s -> { System.out.println("  say: " + s); return Unit.INSTANCE; },
                (Function1<String, Unit>) s -> { System.out.println("  " + s); return Unit.INSTANCE; });

        publisher.start(true);
        // Give the session a moment to reach the router before the first sample, so the run does
        // not spend its opening seconds discarding into a socket that is still dialling.
        Thread.sleep(1500);

        int sent = 0;
        double t0 = samples.isEmpty() ? 0 : samples.get(0).getTSeconds();
        long wallStart = System.currentTimeMillis();
        int n = 0;
        for (ReplaySample s : samples) {
            if (n++ >= limit) break;
            double dt = (s.getTSeconds() - t0) / speed;
            long due = wallStart + (long) (dt * 1000);
            long wait = due - System.currentTimeMillis();
            if (wait > 0) Thread.sleep(wait);
            mono[0] = (long) (s.getTSeconds() * 1000);
            Long unix = s.getUnixMillis();
            sent += pump.sample(s.getState(), mono[0], unix == null ? System.currentTimeMillis() : unix);
        }
        // Let the queue drain before the session goes.
        Thread.sleep(1000);

        ZenohPublisher.Counters c = publisher.counters();
        System.out.println();
        System.out.printf("offered %d, published %d, dropped %d, discarded %d, failures %d, opens %d, peak queue %d%n",
                sent, c.getPublished(), c.getDropped(), c.getDiscarded(), c.getFailures(),
                c.getOpens(), c.getPeakQueued());
        System.out.println("origin: " + pump.getDatumOrigin() + " " + pump.getDatum());
        publisher.stop();
        // Non-zero when nothing reached the bus, so a scripted run can tell.
        System.exit(c.getPublished() > 0 && c.getFailures() == 0 ? 0 : 1);
    }
}
