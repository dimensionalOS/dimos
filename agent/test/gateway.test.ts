import assert from "node:assert/strict";
import { test } from "node:test";
import { mkdtemp, mkdir, readFile, readdir, rm, stat } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { randomUUID } from "node:crypto";
import { Gateway } from "../src/gateway.js";
import { Connection, type Event } from "../src/protocol.js";
import { configSchema, type Paths } from "../src/config.js";
import { deferred, fixtureFactory } from "./fixtures/session.js";

test(
  "detached turn, ownership, deduplicated prompt and persisted resume",
  { timeout: 20000 },
  async (t) => {
    const root = await mkdtemp(join(tmpdir(), "dimcode-"));
    t.after(() => rm(root, { recursive: true, force: true }));
    const p: Paths = {
      config: join(root, "config"),
      state: root,
      cache: join(root, "cache"),
      sessions: join(root, "sessions"),
      socket: join(root, "test.sock"),
    };
    await mkdir(p.config);
    await mkdir(p.sessions);
    const release = deferred<void>(),
      started = deferred<void>();
    const config = configSchema.parse({ workspace: root });
    let gateway = new Gateway(
      config,
      p,
      fixtureFactory(release.promise, () => started.resolve()),
    );
    await gateway.start();
    t.after(() => gateway.close());
    const first = new Connection(p.socket),
      viewer = new Connection(p.socket);
    t.after(() => {
      first.close();
      viewer.close();
    });
    const snapshot = await first.call({ type: "new_session" });
    await viewer.call({
      type: "attach",
      sessionId: snapshot.sessionId,
      writable: false,
    });
    await assert.rejects(
      viewer.call({ type: "prompt", message: "forbidden" }),
      /read-only/,
    );
    // Provider credentials travel only to the input owner and Pi's credential store.
    const auth = deferred<Extract<Event, { type: "auth_prompt" }>>();
    const viewerEvents: Event[] = [];
    first.onEvent = (_seq, event) => {
      if (event.type === "auth_prompt") auth.resolve(event);
    };
    viewer.onEvent = (_seq, event) => viewerEvents.push(event);
    const login = first.call({
      type: "login",
      provider: "openai",
      authType: "api_key",
    });
    const dialog = await auth.promise;
    assert.equal(dialog.prompt.type, "secret");
    await assert.rejects(
      viewer.call({
        type: "ui_response",
        promptId: dialog.promptId,
        value: "not-owner",
      }),
      /read-only/,
    );
    const credential = "local-fixture-credential";
    await first.call({
      type: "ui_response",
      promptId: dialog.promptId,
      value: credential,
    });
    await login;
    assert(
      !viewerEvents.some(
        (event) => event.type === "auth_prompt" || event.type === "auth_info",
      ),
    );
    assert.equal((await stat(join(p.config, "auth.json"))).mode & 0o777, 0o600);
    const id = randomUUID();
    await first.call({ type: "prompt", message: "wait" }, id);
    await started.promise;
    await first.call({ type: "prompt", message: "wait" }, id);
    const closed = deferred<void>();
    first.onClose = () => closed.resolve();
    first.close();
    await closed.promise;
    const state = await viewer.call({ type: "get_state" });
    assert.equal(state.busy, true);
    assert(state.tools.some((tool) => tool.type === "tool_execution_start"));
    const finished = deferred<void>();
    viewer.onEvent = (_seq, event: Event) => {
      if (event.type === "idle") finished.resolve();
    };
    release.resolve();
    await finished.promise;
    assert.equal(
      (await viewer.call({ type: "get_state" })).messages.filter(
        (message) => message.role === "user",
      ).length,
      1,
    );
    for (const name of await readdir(p.sessions, { recursive: true })) {
      if (name.endsWith(".jsonl"))
        assert(
          !(await readFile(join(p.sessions, name), "utf8")).includes(
            credential,
          ),
        );
    }
    viewer.close();
    await gateway.close();
    gateway = new Gateway(
      config,
      p,
      fixtureFactory(Promise.resolve(), () => {}),
    );
    await gateway.start();
    const resumed = new Connection(p.socket);
    t.after(() => resumed.close());
    const simultaneous = new Connection(p.socket);
    t.after(() => simultaneous.close());
    const [restored, observed] = await Promise.all([
      resumed.call({
        type: "attach",
        sessionId: snapshot.sessionId,
        writable: true,
      }),
      simultaneous.call({
        type: "attach",
        sessionId: snapshot.sessionId,
        writable: false,
      }),
    ]);
    assert.equal(restored.sessionId, observed.sessionId);
    assert(restored.messages.some((message) => message.role === "toolResult"));
    assert.equal(restored.busy, false);
    await resumed.call({ type: "prompt", message: "wait" }, id);
    assert.equal(
      (await resumed.call({ type: "get_state" })).messages.filter(
        (message) => message.role === "user",
      ).length,
      1,
    );
    const second = new Connection(p.socket);
    t.after(() => second.close());
    const cwd = join(root, "second");
    await mkdir(cwd);
    assert.equal((await second.call({ type: "new_session", cwd })).cwd, cwd);
    const unattached = new Connection(p.socket);
    const disconnected = deferred<void>();
    unattached.onClose = () => disconnected.resolve();
    await gateway.close();
    await disconnected.promise;
    await assert.rejects(
      unattached.call({ type: "list_sessions" }),
      /disconnected/,
    );
  },
);
