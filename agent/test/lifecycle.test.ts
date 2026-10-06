import assert from "node:assert/strict";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { test } from "node:test";
import { configSchema, type Paths } from "../src/config.js";
import { Gateway } from "../src/gateway.js";
import { Connection, reachable } from "../src/protocol.js";
import { serviceUnit } from "../src/service.js";
import { deferred, fixtureFactory } from "./fixtures/session.js";

test(
  "recover a stale socket and interrupted turn without replaying the operation",
  { timeout: 20000 },
  async (t) => {
    const root = await mkdtemp(join(tmpdir(), "dimcode-recovery-"));
    t.after(() => rm(root, { recursive: true, force: true }));
    const p: Paths = {
      config: join(root, "config"),
      state: root,
      sessions: join(root, "sessions"),
      cache: join(root, "cache"),
      socket: join(root, "gateway.sock"),
    };
    const stale = spawn(process.execPath, [
      "--input-type=module",
      "-e",
      'import {createServer} from "node:net";createServer().listen(process.argv[1],()=>process.stdout.write("ready"));',
      p.socket,
    ]);
    t.after(() => {
      stale.kill();
    });
    await once(stale.stdout, "data");
    stale.kill("SIGKILL");
    await once(stale, "exit");
    const started = deferred<void>(),
      release = deferred<void>();
    let calls = 0;
    const config = configSchema.parse({ workspace: root });
    let gateway = new Gateway(
      config,
      p,
      fixtureFactory(release.promise, () => {
        calls++;
        started.resolve();
      }),
    );
    await gateway.start();
    t.after(() => gateway.close());
    await assert.rejects(new Gateway(config, p).start(), /EADDRINUSE/);
    assert.equal(await reachable(p.socket), true);
    const client = new Connection(p.socket);
    t.after(() => client.close());
    const snapshot = await client.call({ type: "new_session" });
    await client.call({ type: "prompt", message: "wait" });
    await started.promise;
    await gateway.close();
    gateway = new Gateway(
      config,
      p,
      fixtureFactory(Promise.resolve(), () => {
        calls++;
      }),
    );
    await gateway.start();
    const resumed = new Connection(p.socket);
    t.after(() => resumed.close());
    const state = await resumed.call({
      type: "attach",
      sessionId: snapshot.sessionId,
      writable: true,
    });
    assert.equal(state.busy, false);
    assert.equal(calls, 1);
    assert(state.notices.some((notice) => notice.includes("not replayed")));
    assert(state.messages.some((message) => message.role === "user"));
    const unit = serviceUnit(
      config,
      p,
      join(root, "path with spaces", "100%.js"),
    );
    assert(
      unit.includes(
        '"' + join(root, "path with spaces", "100%%.js") + '" gateway',
      ),
    );
    assert(unit.includes('Environment="DIMCODE_HOME=' + p.config + '"'));
    assert(unit.includes("WorkingDirectory=" + root + "\n"));
    assert(!unit.includes("api_key"));
  },
);
