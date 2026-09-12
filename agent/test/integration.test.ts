import assert from "node:assert/strict";
import { test } from "node:test";
import { spawn, type ChildProcess } from "node:child_process";
import { createServer } from "node:net";
import { once } from "node:events";
import { resolve } from "node:path";
import { setTimeout as delay } from "node:timers/promises";
import {
  Client,
  StreamableHTTPClientTransport,
} from "@modelcontextprotocol/client";
import { WebTransport } from "rwebtransport";
import sharp from "sharp";
import {
  CONTROL_CHANNEL,
  PROTOCOL_VERSION,
  encodeDataFrame,
  encodeDatagram,
} from "@dimos/shared";
import { MediaPool } from "../src/media.js";
import { McpTools, toolName } from "../src/mcp.js";

async function stop(child: ChildProcess): Promise<void> {
  if (child.exitCode !== null || child.signalCode) return;
  child.kill("SIGTERM");
  await once(child, "exit");
}
async function port(): Promise<number> {
  const server = createServer();
  server.listen(0, "127.0.0.1");
  await once(server, "listening");
  const address = server.address();
  assert(address && typeof address !== "string");
  await new Promise<void>((resolve) => server.close(() => resolve()));
  return address.port;
}
test(
  "official MCP client consumes actual DimOS handlers, images and errors",
  { timeout: 30000, skip: !process.env.DIMCODE_TEST_PYTHON },
  async (t) => {
    const endpoint = await port();
    const child = spawn(
      process.env.DIMCODE_TEST_PYTHON!,
      ["test/fixtures/mcp_server.py"],
      {
        env: {
          ...process.env,
          PYTHONPATH: resolve(".."),
          DIMCODE_TEST_PORT: String(endpoint),
        },
        stdio: ["ignore", "ignore", "pipe"],
      },
    );
    t.after(() => stop(child));
    const url = "http://127.0.0.1:" + endpoint + "/mcp";
    const deadline = Date.now() + 15000;
    while (true) {
      try {
        await fetch(url, {
          method: "POST",
          body: JSON.stringify({
            jsonrpc: "2.0",
            method: "notifications/initialized",
          }),
        });
        break;
      } catch {
        if (Date.now() > deadline)
          throw new Error("MCP server startup deadline");
        await delay(50);
      }
    }
    const client = new Client({ name: "dimcode-test", version: "0.1.0" });
    t.after(() => client.close());
    await client.connect(new StreamableHTTPClientTransport(new URL(url)));
    const listing = await client.listTools();
    assert(listing.tools.some((tool) => tool.name === "image"));
    assert.deepEqual(
      (await client.callTool({ name: "echo", arguments: { value: "exact" } }))
        .content,
      [{ type: "text", text: "exact" }],
    );
    assert.equal(
      (await client.callTool({ name: "image", arguments: {} })).content[0].type,
      "image",
    );
    const failed = await client.callTool({ name: "failure", arguments: {} });
    assert.equal(failed.isError, true);
    assert.match(JSON.stringify(failed.content), /FIXTURE_FAILURE/);
    let endpoints = [{ name: "fixture", url }];
    const registry = new McpTools(async () => endpoints);
    t.after(() => registry.close());
    const tools = await registry.tools();
    assert.equal(tools.length, listing.tools.length);
    assert.notEqual(toolName("a", "tool.name"), toolName("a", "tool_name"));
    assert(
      toolName("long-endpoint".repeat(10), "long-tool".repeat(20)).length <= 64,
    );
    endpoints = [];
    assert.deepEqual(await registry.tools(), []);
    endpoints = [{ name: "fixture", url }];
    assert.equal((await registry.tools()).length, listing.tools.length);
    assert.deepEqual(
      tools.find((tool) => tool.label.endsWith("/ echo"))?.parameters,
      listing.tools.find((tool) => tool.name === "echo")?.inputSchema,
    );
  },
);

test(
  "Node Web SDK receives real JPEG over the existing Deno QUIC relay",
  { timeout: 30000, skip: !process.env.DIMCODE_TEST_DENO },
  async (t) => {
    const child = spawn(
      process.env.DIMCODE_TEST_DENO!,
      [
        "run",
        "--allow-net",
        "--allow-read",
        "../web/relay/main.ts",
        "--port",
        "0",
      ],
      { stdio: ["ignore", "pipe", "pipe"] },
    );
    t.after(() => stop(child));
    const info = await new Promise<{
      httpPort: number;
      wtUrl: string;
      certHash: string;
    }>((resolve, reject) => {
      let buffer = "";
      child.stdout.on("data", (chunk) => {
        buffer += chunk.toString();
        for (const line of buffer.split("\n").slice(0, -1)) {
          if (line.startsWith('{"event":"ready"')) resolve(JSON.parse(line));
        }
        buffer = buffer.slice(buffer.lastIndexOf("\n") + 1);
      });
      child.once("error", reject);
      child.once("exit", (code) => reject(new Error("Relay exited " + code)));
    });
    const robot = new WebTransport(info.wtUrl + "/robot", {
      serverCertificateHashes: [
        {
          algorithm: "sha-256",
          value: Uint8Array.from(Buffer.from(info.certHash, "base64")).buffer,
        },
      ],
    });
    t.after(() => robot.close());
    await robot.ready;
    const send = async (ch: string, seq: number, bytes: Uint8Array) => {
      const stream = await robot.createBidirectionalStream();
      const writer = stream.writable.getWriter();
      await writer.write(
        encodeDataFrame({ ch, seq, ts: seq, delivery: "latest" }, bytes),
      );
      await writer.close();
    };
    await send(
      CONTROL_CHANNEL,
      0,
      encodeDatagram({
        t: "hello",
        v: PROTOCOL_VERSION,
        role: "robot",
        robot: { id: "fixture", name: "Fixture", model: "test" },
        manifest: {
          version: 1,
          channels: [
            {
              ch: "camera",
              encoding: "jpeg.v1",
              delivery: "latest",
              maxHz: 30,
            },
          ],
        },
      }),
    );
    const reader = robot.datagrams.readable.getReader();
    await reader.read();
    reader.releaseLock();
    const pool = new MediaPool();
    t.after(() => pool.close());
    const lease = await pool.acquire(
      { url: "http://127.0.0.1:" + info.httpPort, robot: "fixture" },
      "camera",
      (slot) => slot,
      () => {},
    );
    t.after(() => lease.close());
    const jpeg = await sharp({
      create: { width: 64, height: 32, channels: 3, background: "#40d0a0" },
    })
      .jpeg()
      .toBuffer();
    const deadline = Date.now() + 10000;
    let seq = 1;
    while (!lease.current()) {
      if (Date.now() > deadline) throw new Error("JPEG reception deadline");
      await send("camera", seq++, jpeg);
      await delay(25);
    }
    assert.deepEqual(lease.current()?.value, new Uint8Array(jpeg));
  },
);
