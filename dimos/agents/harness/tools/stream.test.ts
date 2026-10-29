import assert from "node:assert/strict";
import type { JsonObject } from "@earendil-works/pi-durable";
import { test } from "node:test";
import { mkdtemp, rm, stat, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import sharp from "sharp";
import { fauxAssistantMessage, fauxToolCall, normalizeContext } from "@earendil-works/pi-ai";
import { anthropicProvider } from "@earendil-works/pi-ai/providers/anthropic";
import { stream as anthropicStream } from "@earendil-works/pi-ai/api/anthropic-messages";
import { Type } from "typebox";
import { Value } from "typebox/value";
import { ChildAgent } from "../../test/child-driver.ts";
import { dimosFixture } from "../../test/environment.ts";
import { DimosConnection } from "../dimos/connection.ts";
import { RuntimeClient } from "../dimos/client.ts";
import { until } from "../../sdk/until.ts";
import { Status } from "../../shared/protocol/status.ts";
import { FileRenderer } from "../../shared/render/render.ts";
import { streamTool } from "./lib/stream.ts";

const dimos = dimosFixture();

test(
  "the local stream tool reads fresh relay frames and exports an image for the model",
  { timeout: 120_000 },
  async (t) => {
    const system = dimos();
    if (system === null) return t.skip("no .venv in the checkout");
    const scratch = await mkdtemp(join(tmpdir(), "dimos-stream-tool-"));
    const agent = ChildAgent.start(scratch, system.env);
    try {
      const calls: JsonObject[] = [
        { op: "list" },
        { op: "read", channel: "HarnessProbe.position", mode: "latest" },
        { op: "read", channel: "HarnessProbe.position", mode: "next" },
        { op: "read", channel: "color_image", export: true },
      ];
      await agent.call({
        op: "script",
        messages: [
          ...calls.map((args, i) =>
            fauxAssistantMessage([fauxToolCall("stream_get", { ...args, timeout_s: 15 }, { id: `stream-${i}` })], {
              stopReason: "toolUse",
            }),
          ),
          fauxAssistantMessage("observed"),
        ],
      });
      const { conversation } = await agent.call<{ conversation: number }>({ op: "start" });
      await agent.call({ op: "submit", conversation, text: "look at the streams", request: "stream-read" });
      const results = await agent.call<{ isError: boolean; text: string }[]>({ op: "toolResults", conversation });
      assert.equal(results.length, 4);
      for (const result of results) assert.equal(result.isError, false, result.text);
      const listed = JSON.parse(results[0].text) as { channels: { channel: string; encoding: string }[] };
      assert.ok(listed.channels.some((spec) => spec.channel === "HarnessProbe.position"));
      assert.ok(listed.channels.some((spec) => spec.channel === "color_image" && spec.encoding === "jpeg.v1"));
      const reads = results
        .slice(1)
        .map(
          (result) =>
            JSON.parse(result.text.split("\n")[0]) as { seq: number; preview: string | null; export: string | null },
        );
      assert.ok(reads[1].seq > reads[0].seq, "next returns a newer native position frame");
      assert.match(reads[0].preview ?? "", /x:/);
      const path = reads[2].export;
      assert.ok(path !== null);
      assert.ok((await stat(path)).size > 0);
      const image = await sharp(path).metadata();
      assert.deepEqual([image.width, image.height], [64, 48]);
      assert.match(results[3].text, /sha256/, "the saved image went through the shared renderer");
      assert.equal(await agent.close(), 0);
    } finally {
      if (agent.alive) agent.kill();
      await rm(scratch, { recursive: true, force: true });
    }
  },
);

test(
  "the runtime catalog discovers, reads and removes a loaded module through one relay session",
  { timeout: 60_000 },
  async (t) => {
    const system = dimos();
    if (system === null) return t.skip("no .venv in the checkout");
    const scratch = await mkdtemp(join(tmpdir(), "dimos-live-stream-"));
    const connection = new DimosConnection({
      url: system.install.mcpUrl,
      connectTimeoutMs: 10_000,
      maxBackoffMs: 1_000,
      report: () => {},
    });
    const runtime = new RuntimeClient(connection);
    connection.start();
    try {
      await t.test("the Anthropic provider receives the discovery and read arguments", async () => {
        const model = anthropicProvider().getModels()[0];
        assert.ok(model);
        const tool = streamTool({
          runtime,
          renderer: new FileRenderer({ cacheDir: scratch, maxCacheBytes: 1024, maxSourceBytes: 1024 }),
          exportsDir: scratch,
        }).registration;
        let payload: unknown;
        const response = anthropicStream(
          model,
          normalizeContext({
            messages: [{ role: "user", content: "List streams", timestamp: 1 }],
            tools: [tool],
          }),
          {
            apiKey: "offline-test",
            onPayload(value) {
              payload = value;
              throw new Error("offline payload captured");
            },
          },
        );
        assert.match((await response.result()).errorMessage ?? "", /offline payload captured/);
        const sent = Value.Parse(
          Type.Object({
            tools: Type.Array(
              Type.Object({
                input_schema: Type.Object({
                  properties: Type.Record(Type.String(), Type.Unknown()),
                  required: Type.Array(Type.String()),
                }),
              }),
            ),
          }),
          payload,
        ).tools[0].input_schema;
        assert.deepEqual(Object.keys(sent.properties).sort(), [
          "channel",
          "export",
          "max_chars",
          "mode",
          "op",
          "timeout_s",
        ]);
        assert.deepEqual(sent.required, ["op"]);
      });
      const session = await runtime.relay();
      await until(
        runtime.streams,
        (rows) => rows.find((row) => row.channel === "color_image"),
        AbortSignal.timeout(10_000),
      );
      const phases: string[] = [];
      const stopStatus = session.status.subscribe(() => phases.push(session.status.get().transport.phase));
      t.after(stopStatus);
      const path = join(scratch, "ticker.py");
      await writeFile(
        path,
        `
from reactivex import interval
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc
from dimos.core.stream import Out

class ApiTicker(AgentModule):
    ticks: Out[int]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(interval(0.05).subscribe(lambda _: self.ticks.publish(42)))
`,
      );
      const created = await connection.callTool(
        "agent_module",
        { op: "create", name: "api_ticker", path },
        { caller: null, timeoutMs: 30_000 },
      );
      assert.equal(created.isError, false, JSON.stringify(created));
      const discovered = await until(
        runtime.streams,
        (rows) => rows.find((row) => row.channel === "ApiTicker.ticks"),
        AbortSignal.timeout(10_000),
      );
      assert.equal(discovered.source, null, "discovery arrives before another inventory read");
      const value = new Status<unknown>(null, (error) => {
        throw error;
      });
      const stop = session.subscribe(discovered.channel, ({ slot }) => {
        if (slot !== null) value.set(slot.value);
      });
      try {
        assert.equal(
          await until(value, (current) => (current === 42 ? current : undefined), AbortSignal.timeout(10_000)),
          42,
        );
        assert.equal(await runtime.relay(), session);
        assert.equal(
          runtime.streams.value.find((row) => row.channel === discovered.channel)?.source?.module,
          "ApiTicker",
        );
        const removed = await connection.callTool(
          "agent_module",
          { op: "delete", name: "api_ticker" },
          { caller: null, timeoutMs: 30_000 },
        );
        assert.equal(removed.isError, false, JSON.stringify(removed));
        await until(
          runtime.streams,
          (rows) => (rows.every((row) => row.channel !== discovered.channel) ? true : undefined),
          AbortSignal.timeout(10_000),
        );
        assert.equal(session.store.get(discovered.channel), null);
        assert.ok(phases.length > 0);
        assert.deepEqual(new Set(phases), new Set(["connected"]));
      } finally {
        stop();
      }
    } finally {
      await connection
        .callTool("agent_module", { op: "delete", name: "api_ticker" }, { caller: null, timeoutMs: 30_000 })
        .catch(() => {});
      runtime.close();
      await connection.close();
      await rm(scratch, { recursive: true, force: true });
    }
  },
);
