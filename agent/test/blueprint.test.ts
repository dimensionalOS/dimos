import assert from "node:assert/strict";
import { spawn, type ChildProcess } from "node:child_process";
import { createHash } from "node:crypto";
import { createSocket } from "node:dgram";
import { once } from "node:events";
import { writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import { test } from "node:test";
import { connect, createDecoderRegistry, type Slot } from "@dimos/sdk";
import sharp from "sharp";
import { z } from "zod";
import { MediaPool, nodeTransport } from "../src/media.js";

const statsSchema = z.object({
  robots: z.array(z.object({ id: z.string() })),
  viewers: z.number(),
  perRobot: z.record(z.string(), z.object({ subs: z.array(z.string()) })),
});
const cloudSchema = z.object({
  bytes: z.instanceof(Uint8Array),
  meta: z.object({
    count: z.number().int().positive(),
    frame: z.string().min(1),
    capture_ts: z.number().positive(),
    sha256: z.string(),
  }),
});
async function until(
  check: () => boolean | Promise<boolean>,
  label: string,
  timeout = 30000,
): Promise<void> {
  const deadline = Date.now() + timeout;
  while (!(await check())) {
    if (Date.now() > deadline) throw new Error("Timed out: " + label);
    await delay(50);
  }
}
async function stop(child: ChildProcess): Promise<void> {
  if (child.exitCode !== null || child.signalCode || !child.pid) return;
  const exited = once(child, "exit");
  child.kill("SIGINT");
  const timer = setTimeout(() => {
    try {
      process.kill(-child.pid!, "SIGKILL");
    } catch {}
  }, 10000);
  try {
    await exited;
  } finally {
    clearTimeout(timer);
  }
}

test(
  "standard Go2 blueprint streams video, full XYZ and odometry through the Node Web SDK",
  {
    timeout: 150000,
    skip: !(
      process.env.DIMCODE_TEST_PYTHON &&
      process.env.DIMCODE_TEST_DENO &&
      process.env.DIMCODE_TEST_GO2_DB
    ),
  },
  async (t) => {
    const relay = spawn(
      process.env.DIMCODE_TEST_DENO!,
      [
        "run",
        "--allow-net",
        "--allow-read",
        "../web/relay/main.ts",
        "--port",
        "0",
      ],
      { detached: true, stdio: ["ignore", "pipe", "pipe"] },
    );
    t.after(() => stop(relay));
    let relayLog = "",
      httpPort = 0;
    relay.stderr.on("data", (chunk) => {
      relayLog = (relayLog + chunk).slice(-8000);
    });
    relay.on("error", (error) => {
      relayLog += error.message;
    });
    let buffer = "";
    relay.stdout.on("data", (chunk) => {
      buffer += chunk;
      const lines = buffer.split("\n");
      buffer = lines.pop()!;
      for (const line of lines)
        if (line.startsWith('{"event":"ready"'))
          httpPort = z
            .object({ httpPort: z.number() })
            .parse(JSON.parse(line)).httpPort;
    });
    await until(() => {
      assert.equal(relay.exitCode, null, relayLog);
      return httpPort > 0;
    }, "relay startup");
    const url = "http://127.0.0.1:" + httpPort;
    const stats = async () =>
      statsSchema.parse(await (await fetch(url + "/api/stats")).json());
    const scout = createSocket("udp4");
    scout.bind(0, "127.0.0.1");
    await once(scout, "listening");
    const scoutPort = scout.address().port;
    scout.close();
    const robot = "dimcode-e2e-" + process.pid;
    const blueprint = spawn(
      process.env.DIMCODE_TEST_PYTHON!,
      ["-c", "from go2_media import main; main()"],
      {
        detached: true,
        env: {
          ...process.env,
          PYTHONPATH: resolve("test/fixtures") + ":" + resolve(".."),
          REPLAY: "true",
          REPLAY_DB: resolve(process.env.DIMCODE_TEST_GO2_DB!),
          VIEWER: "none",
          RELAY_URL: url,
          ROBOT_ID: robot,
          ZENOH_SCOUT_ADDR: "239.255.79.22:" + scoutPort,
        },
        stdio: ["ignore", "pipe", "pipe"],
      },
    );
    t.after(() => stop(blueprint));
    let blueprintLog = "";
    const record = (chunk: Buffer) => {
      blueprintLog = (blueprintLog + chunk).slice(-16000);
    };
    blueprint.stdout.on("data", record);
    blueprint.stderr.on("data", record);
    blueprint.on("error", (error) => {
      blueprintLog += error.message;
    });
    try {
      await until(
        async () => {
          assert.equal(blueprint.exitCode, null, blueprintLog);
          return (await stats()).robots.some((item) => item.id === robot);
        },
        "Go2 blueprint registration",
        90000,
      );
      assert.equal((await stats()).viewers, 0);
      assert.deepEqual((await stats()).perRobot[robot].subs, []);
      const decoders = createDecoderRegistry();
      decoders.register("dimcode-test.xyz.v1", (bytes, header) => ({
        value: cloudSchema.parse({ bytes, meta: header.meta }),
      }));
      let opens = 0;
      const pool = new MediaPool(
        nodeTransport,
        { decoders },
        (options, deps) => {
          opens++;
          return connect(options, deps);
        },
      );
      t.after(() => pool.close());
      assert.equal(opens, 0);
      const frames = new Map<number, Slot>(),
        clouds = new Map<number, Slot>(),
        poses = new Map<number, Slot>();
      const capture = async (channel: string, values: Map<number, Slot>) => {
        const lease = await pool.acquire(
          { url, robot },
          channel,
          (slot) => slot,
          () => {
            const slot = lease.current();
            if (slot) values.set(slot.seq, slot);
          },
        );
        t.after(() => lease.close());
        return lease;
      };
      const camera = await capture("color_image", frames),
        cloud = await capture("lidar", clouds),
        pose = await capture("odom", poses);
      assert.equal(opens, 1);
      await until(
        () => frames.size >= 12 && clouds.size >= 5 && poses.size >= 5,
        "video, XYZ and pose frames",
      );
      const hashes = new Set<string>();
      for (const slot of frames.values()) {
        assert(slot.value instanceof Uint8Array);
        const decoded = await sharp(slot.value)
          .raw()
          .toBuffer({ resolveWithObject: true });
        assert(decoded.info.width > 100 && decoded.info.height > 100);
        hashes.add(createHash("sha256").update(decoded.data).digest("hex"));
      }
      assert(hashes.size > 1, "video contains distinct decoded frames");
      const counts: number[] = [];
      for (const slot of clouds.values()) {
        const { bytes, meta } = cloudSchema.parse(slot.value);
        assert.equal(bytes.byteLength, meta.count * 3 * 4);
        assert.equal(
          createHash("sha256").update(bytes).digest("hex"),
          meta.sha256,
        );
        const view = new DataView(
          bytes.buffer,
          bytes.byteOffset,
          bytes.byteLength,
        );
        let nonzeroZ = 0;
        for (let i = 0; i < bytes.byteLength; i += 4) {
          const value = view.getFloat32(i, true);
          assert(Number.isFinite(value));
          if (i % 12 === 8 && value !== 0) nonzeroZ++;
        }
        assert(nonzeroZ > 0, "actual Z coordinates, not an XY projection");
        counts.push(meta.count);
      }
      for (const slot of poses.values())
        z.object({
          x: z.number(),
          y: z.number(),
          z: z.number(),
          ts: z.number().positive(),
        }).parse(slot.value);
      camera.close();
      pose.close();
      await until(
        async () => (await stats()).perRobot[robot].subs.join() === "lidar",
        "unused channels unsubscribe",
      );
      const previous = clouds.size;
      await until(
        () => clouds.size > previous,
        "point clouds continue after camera closes",
      );
      cloud.close();
      await until(async () => {
        const value = await stats();
        return value.viewers === 0 && value.perRobot[robot].subs.length === 0;
      }, "last renderer releases viewer and subscriptions");
      const report = {
        blueprint: "unitree-go2 + cockpit",
        videoFrames: frames.size,
        distinctVideoFrames: hashes.size,
        pointClouds: counts.length,
        pointsPerCloud: counts,
        odometryFrames: poses.size,
        sourceHashesMatch: true,
        finalViewers: 0,
        finalSubscriptions: [],
      };
      t.diagnostic(JSON.stringify(report));
      if (process.env.DIMCODE_TEST_REPORT)
        await writeFile(
          process.env.DIMCODE_TEST_REPORT,
          JSON.stringify(report, null, 2) + "\n",
        );
    } catch (error) {
      t.diagnostic(blueprintLog);
      t.diagnostic(relayLog);
      throw error;
    }
  },
);
