import assert from "node:assert/strict";
import type { LinkStatus } from "../../shared/protocol/gateway/link.ts";
import { DimosConnection } from "./connection.ts";
import { dimosFixture } from "../../test/environment.ts";
import { join } from "node:path";
import { mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { test } from "node:test";
import { tmpdir } from "node:os";

const dimos = dimosFixture();

const SENTINEL = `
import time
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.agents.annotation import skill
from dimos.core.core import rpc


class Sentinel(AgentModule):
    """Says hello once."""

    @rpc
    def start(self) -> None:
        super().start()
        self.notify("hello")

    @skill
    def delay(self, seconds: float) -> str:
        """Return after the given number of seconds."""
        time.sleep(seconds)
        return "done"
`;

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function until(
  connection: DimosConnection,
  reached: (status: LinkStatus) => boolean,
  timeoutMs: number,
): Promise<LinkStatus> {
  const deadline = Date.now() + timeoutMs;
  while (!reached(connection.status.value) && Date.now() < deadline) await sleep(100);
  return connection.status.value;
}

test(
  "the link holds through a long call, recovers from a restart with calls in flight, and goes down when DimOS stops",
  { timeout: 300_000 },
  async (t) => {
    const system = dimos();
    if (system === null) {
      t.skip("no .venv in the checkout");
      return;
    }
    const transitions: LinkStatus[] = [];
    const connection = new DimosConnection({
      url: system.install.mcpUrl,
      connectTimeoutMs: 10_000,
      maxBackoffMs: 5_000,
      report: (error) => console.error("[connection test report]", error),
    });
    connection.status.subscribe((status) => transitions.push(status));
    connection.start();
    const scratch = mkdtempSync(join(tmpdir(), "dimos-connection-"));
    const source = join(scratch, "sentinel.py");
    writeFileSync(source, SENTINEL);
    try {
      const connected = await until(connection, (status) => status.state === "up", 30_000);
      assert.equal(connected.state, "up", "the link comes up");
      connection.retryNow();
      assert.equal(connection.status.value, connected, "a readiness signal leaves an established connection alone");
      const deployed = await connection.callTool(
        "agent_module",
        { op: "create", name: "sentinel", path: source },
        { caller: null, timeoutMs: 120_000 },
      );
      const modules = join(String(system.env.XDG_STATE_HOME), "dimos", "modules", "dimos_agent");
      assert.deepEqual(
        deployed.content,
        [{ type: "text", text: `sentinel created: running r1 from ${join(modules, "sentinel.py")}` }],
        "the sentinel deploys",
      );

      const quiet = transitions.length;
      const started = Date.now();
      const read = await connection.callTool("delay", { seconds: 3 }, { caller: null, timeoutMs: 8_000 });
      assert.ok(Date.now() - started >= 3_000, "the skill waited 3 s");
      assert.deepEqual(
        { isError: read.isError, transitions: transitions.slice(quiet) },
        { isError: false, transitions: [] },
        "the call completed and the link never left up",
      );

      const inFlight = Promise.allSettled(
        Array.from({ length: 4 }, () =>
          connection.callTool("delay", { seconds: 20 }, { caller: null, timeoutMs: 30_000 }),
        ),
      );
      await sleep(500);
      await system.stop();
      const outcomes = await inFlight;
      assert.deepEqual(
        outcomes.map((outcome) => outcome.status),
        ["rejected", "rejected", "rejected", "rejected"],
        "every call in flight failed when DimOS stopped",
      );
      await system.start();
      const restarted = await until(connection, (status) => status.state === "up", 60_000);
      assert.equal(restarted.state, "up", "the link comes back up on the new system");
      const echo = await connection.callTool("echo", { text: "back" }, { caller: null, timeoutMs: 10_000 });
      assert.deepEqual(echo.content, [{ type: "text", text: "back" }], "a call succeeds on the new system");

      await system.stop();
      const stopped = await until(connection, (status) => status.state !== "up", 5_000);
      assert.notEqual(stopped.state, "up", "the link went down when DimOS stopped with no call in flight");
    } finally {
      await connection.close();
      rmSync(scratch, { recursive: true, force: true });
    }
  },
);
