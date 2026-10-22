import { assert, assertEquals } from "@std/assert";
import { PROTOCOL_VERSION } from "@dimos/shared";
import { startRelay } from "./server.ts";
import { shardFor, withoutFlag } from "./shards.ts";

Deno.test("shardFor spreads ids over the shards and is stable", () => {
  const counts = [0, 0, 0, 0];
  for (let i = 0; i < 1000; i++) counts[shardFor(`robot-${i}`, 4)]++;
  for (const c of counts) assert(c > 150, `the shards share the ids: ${counts}`);
  assertEquals(shardFor("go2-lab", 4), shardFor("go2-lab", 4));
  assertEquals(shardFor("anything", 1), 0);
});

Deno.test("withoutFlag drops a flag in both spellings", () => {
  assertEquals(
    withoutFlag(["--port", "7780", "--shards=3", "--host", "h", "--shards", "2"], "shards"),
    ["--port", "7780", "--host", "h"],
  );
  assertEquals(withoutFlag(["--port=0", "--x"], "port"), ["--x"]);
});

Deno.test("/api/info?robot= answers with the robot's shard", async () => {
  const other = {
    httpPort: 1,
    wtUrl: "https://127.0.0.1:1",
    certHash: "aGFzaA==",
  };
  const relay = await startRelay({ port: 0, shardInfos: [other] });
  try {
    const base = `http://127.0.0.1:${relay.httpPort}`;
    const ids = Array.from({ length: 50 }, (_, i) => `r${i}`);
    const onOther = ids.find((id) => shardFor(id, 2) === 1)!;
    const onSelf = ids.find((id) => shardFor(id, 2) === 0)!;
    const info = async (query: string) => (await fetch(`${base}/api/info${query}`)).json();
    assertEquals(await info(`?robot=${onOther}`), {
      wtUrl: other.wtUrl,
      certHash: other.certHash,
      v: PROTOCOL_VERSION,
      shard: 1,
    });
    const own = await info(`?robot=${onSelf}`);
    assertEquals([own.wtUrl, own.shard], [relay.wtUrl, 0]);
    assertEquals((await info("")).shard, 0); // no robot named: this shard
    const stats = await (await fetch(`${base}/api/stats`)).json();
    assertEquals([stats.shards, stats.shardHttpPorts], [2, [relay.httpPort, 1]]);
    assertEquals(relay.shards, 2);
  } finally {
    await relay.shutdown();
  }
});
