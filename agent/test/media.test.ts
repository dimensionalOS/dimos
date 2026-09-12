import assert from "node:assert/strict";
import { test } from "node:test";
import { connect, createDecoderRegistry } from "@dimos/sdk";
import {
  FakeRelayEnd,
  INFO,
  ROBOT_A,
  manifest,
  spec,
  until,
} from "../../web/sdk/src/testing/fakeRelay.ts";
import { MediaPool } from "../src/media.js";

test("generic media is lazy, shared, byte exact and released by its last consumer", async () => {
  const relay = new FakeRelayEnd();
  let opens = 0,
    loads = 0;
  const decoders = createDecoderRegistry();
  decoders.register("test.xyz.v1", (bytes, header) => ({
    value: { bytes, meta: header.meta },
  }));
  const pool = new MediaPool(
    async () => {
      loads++;
      return {
        fetchInfo: async () => INFO,
        createWebTransport: () => relay.wt,
      };
    },
    { decoders },
    (options, deps) => {
      opens++;
      return connect(options, deps);
    },
  );
  assert.equal(loads, 0);
  const first = await pool.acquire(
    { robot: ROBOT_A.id },
    "cloud",
    (slot) => slot,
    () => {},
  );
  const second = await pool.acquire(
    { robot: ROBOT_A.id },
    "cloud",
    (slot) => slot,
    () => {},
  );
  try {
    assert.equal(opens, 1);
    assert.equal(loads, 1);
    relay.push({ t: "welcome", v: INFO.v });
    relay.push({ t: "robots", robots: [ROBOT_A] });
    relay.pushManifest(
      ROBOT_A.id,
      manifest([spec({ ch: "cloud", encoding: "test.xyz.v1" })]),
    );
    await until(() => relay.subs().length === 1, "one wire subscription");
    const bytes = new Uint8Array(new Float32Array([1, 2, 3, -1, 0, 4]).buffer);
    relay.pushRaw(1, bytes, "cloud", {
      observation: "actual-selection",
      frame: "map",
    });
    await until(() => first.current() !== undefined, "decoded cloud");
    assert.deepEqual(first.current()?.value, {
      bytes,
      meta: { observation: "actual-selection", frame: "map" },
    });
    first.close();
    first.close();
    assert(second.current());
    second.close();
    assert.equal(opens, 1);
  } finally {
    first.close();
    second.close();
    pool.close();
  }
});

test("closing a pool during lazy initialization cannot create a leaked connection", async () => {
  let complete!: (value: {}) => void;
  const deps = new Promise<{}>((resolve) => {
    complete = resolve;
  });
  let opens = 0;
  const pool = new MediaPool(
    () => deps,
    {},
    (options, transport) => {
      opens++;
      return connect(options, transport);
    },
  );
  const pending = pool.acquire(
    {},
    "image",
    (slot) => slot,
    () => {},
  );
  pool.close();
  complete({});
  await assert.rejects(pending, /closed/);
  assert.equal(opens, 0);
});
