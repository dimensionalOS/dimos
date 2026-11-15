import type { AgentTool } from "#dimos/agents/harness/tools/base.ts";
import type { AgentToolTable } from "#dimos/agents/harness/tools/extension.ts";
import type { Context } from "@earendil-works/chord/context";
import type { ConversationDefaults } from "#dimos/agents/harness/conversations.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { ImageProcessor } from "@earendil-works/pi-durable/tools";
import type { JobFiringDeps } from "#dimos/agents/harness/jobs/fire.ts";
import type { Models } from "@earendil-works/pi-ai";
import type { RuntimeToolSchemas } from "#dimos/agents/harness/tools/runtime-catalog.ts";
import type { Storage } from "@earendil-works/pi-durable";
import { ContextBudget } from "#dimos/agents/harness/limit/budget.ts";
import { ConversationHistory } from "#dimos/agents/harness/history/forks.ts";
import { Conversations } from "#dimos/agents/harness/conversations.ts";
import { Detacher } from "#dimos/agents/harness/background/detacher.ts";
import { DimosConnection } from "#dimos/agents/harness/dimos/connection.ts";
import { EventDrafts } from "#dimos/agents/harness/events/message.ts";
import { EventIntake } from "#dimos/agents/harness/events/intake.ts";
import { FileRenderer, modelImages } from "#dimos/agents/shared/render/render.ts";
import { Harness, createRegistry } from "@earendil-works/pi-durable";
import { InputGate } from "#dimos/agents/harness/inputs/gate.ts";
import { JobOps } from "#dimos/agents/harness/jobs/ops.ts";
import { OutputLimiter } from "#dimos/agents/harness/limit/limit.ts";
import { QueueKicker } from "#dimos/agents/harness/inputs/retry.ts";
import { RunWatcher } from "#dimos/agents/harness/dimos/runs.ts";
import { RuntimeClient } from "#dimos/agents/harness/dimos/client.ts";
import { RuntimeLoader } from "#dimos/agents/harness/dimos/loader.ts";
import { RuntimeObserver } from "#dimos/agents/harness/context/observer.ts";
import { RuntimeToolCatalog } from "#dimos/agents/harness/tools/runtime-catalog.ts";
import { SqliteStorage } from "@earendil-works/pi-durable/storage/sqlite";
import { TrajectorySender } from "#dimos/agents/harness/trajectory/sender.ts";
import { bashTool } from "#dimos/agents/harness/tools/bash.ts";
import { contextExtension } from "#dimos/agents/harness/context/extension.ts";
import { createJobTask } from "#dimos/agents/harness/jobs/task.ts";
import { createNodePhotonImages } from "@earendil-works/pi-durable/images/node";
import { editTool } from "#dimos/agents/harness/tools/edit.ts";
import { executionEnv } from "#dimos/agents/harness/env.ts";
import { jobFacts } from "#dimos/agents/harness/jobs/registry.ts";
import { backgroundTool } from "#dimos/agents/harness/tools/background.ts";
import { jobTool } from "#dimos/agents/harness/tools/job.ts";
import { jobsExtension } from "#dimos/agents/harness/jobs/extension.ts";
import { loadSkills, skillsExtension } from "#dimos/agents/harness/skills/catalog.ts";
import { mayChangeSystem, toolsExtension } from "#dimos/agents/harness/tools/extension.ts";
import { openNodeSqliteDatabase } from "@earendil-works/pi-durable/storage/sqlite/node";
import { promptExtension } from "#dimos/agents/harness/prompt.ts";
import { readTool } from "#dimos/agents/harness/tools/read.ts";
import { runtimeTools } from "#dimos/agents/harness/tools/runtime.ts";
import { writeTool } from "#dimos/agents/harness/tools/write.ts";

type AgentOptions = {
  readonly install: DimosInstall;
  readonly models: Models;
  /** `~/.local/state/dimos/agent/agent.sqlite`. */
  readonly storePath: string;
  /** `~/.local/state/dimos/agent/tool-results/`. */
  readonly toolResultsDir: string;
  /** Rendered previews; `~/.cache/dimos/agent/render/`. */
  readonly renderCacheDir: string;
  /** The bundled `harness/skills/`, then `~/.config/dimos/agent/skills/`. */
  readonly skillRoots: readonly string[];
  readonly defaults: () => ConversationDefaults;
  readonly gatewayEnv: NodeJS.ProcessEnv;
  readonly report: (error: unknown) => void;
  /** The Harness clock, which job tasks sleep on; tests move it. Wall-clock time when omitted. */
  readonly now?: () => number;
};

/**
 * The harness as the gateway uses it. Background work (events, jobs, trajectory, retries) runs inside and needs no
 * terminal. The gateway feeds a Chord cell from each status source.
 */
export type Agent = {
  /** pi-durable, for watching, aborting and configuring conversations. Inputs go through `inputs`, never `submit()`. */
  readonly harness: Harness;
  /** The store's scan of a conversation's submissions, settled ones included, which the harness does not offer. */
  readonly storage: Pick<Storage, "scanSubmissions">;
  readonly inputs: InputGate;
  readonly conversations: Conversations;
  readonly history: ConversationHistory;
  readonly runtime: RuntimeClient;
  readonly jobs: JobOps;
  readonly detacher: Detacher;
  /** The DimOS link as `status`, plus the `onEvent` and `onHarnessStatus` event streams. */
  readonly connection: DimosConnection;
  readonly loader: RuntimeLoader;
  readonly runs: RunWatcher;
  /** `$XDG_STATE_HOME/dimos`, where module records are read while DimOS is down. */
  readonly stateDir: string;
  close(context: Context): Promise<void>;
};

type ToolTableDeps = {
  readonly install: DimosInstall;
  readonly gatewayEnv: NodeJS.ProcessEnv;
  readonly images: ImageProcessor;
  /** Files under these are read whole: the skill roots. */
  readonly skillRoots: readonly string[];
  readonly renderer: FileRenderer;
  readonly connection: DimosConnection;
  readonly runtime: RuntimeClient;
  readonly exportsDir: string;
  /** The runtime tool schemas McpServer last advertised, from the stored catalog. */
  readonly schemas: RuntimeToolSchemas;
  readonly job: AgentTool<"job">;
  readonly background: AgentTool<"background">;
};

/**
 * Opens the one store and everything that runs in it, in the order recovery needs: every tool and task kind is
 * installed before `resume()`, so an interrupted turn, a replay-safe tool and a sleeping job task all find their code.
 * `Harness.open` gets `settings: { steeringMode: "all", followUpMode: "all" }` so every item queued for a boundary is
 * placed together; `Detacher.extension()` is installed before `resume()` and `toolsExtension` wraps the tools whose
 * policy allows backgrounding; `JobOps` is built once and given to the `job` tool and the gateway; RunWatcher and
 * the live event intake start beside the loader.
 */
export async function openAgent(options: AgentOptions, context: Context): Promise<Agent> {
  const { install, gatewayEnv, report } = options;
  const db = await openNodeSqliteDatabase(options.storePath);
  await db.exec("PRAGMA synchronous = FULL");
  await db.exec("PRAGMA locking_mode = EXCLUSIVE");
  const storage = await SqliteStorage.open(db);
  const registry = createRegistry();
  const harness = await Harness.open(
    storage,
    {
      models: options.models,
      registry,
      settings: { steeringMode: "all", followUpMode: "all" },
      env: executionEnv(() => options.defaults().workspace),
      onReport: report,
      now: options.now,
    },
    context,
  );
  const connection = new DimosConnection({
    url: install.mcpUrl,
    connectTimeoutMs: 30_000,
    maxBackoffMs: 30_000,
    report,
  });
  connection.start();
  const runtime = new RuntimeClient(connection, report);
  const renderer = new FileRenderer({
    cacheDir: options.renderCacheDir,
    maxCacheBytes: 128 * 2 ** 20,
    maxSourceBytes: 64 * 2 ** 20,
  });
  const budget = new ContextBudget(options.models);
  const limiter = new OutputLimiter({ dir: options.toolResultsDir, budget, defaultMaxChars: 2_000 });
  const observer = new RuntimeObserver({
    harness,
    runtime,
    jobFacts: (c) => jobFacts(harness, c),
    runsDir: install.runsDir,
    reuseForMs: 1_000,
    timeoutMs: 3_000,
    report,
  });
  const inputs = new InputGate({ harness, observer, limiter });
  const detacher = new Detacher(inputs, report);
  const firing: JobFiringDeps = { inputs, connection, skillTimeoutMs: 120_000 };
  const jobTask = createJobTask(firing);
  const jobs = new JobOps(harness, jobTask, firing);
  const catalog = await RuntimeToolCatalog.load(harness, context);
  const images = modelImages(await createNodePhotonImages());
  let table: AgentToolTable;
  const installTools = (): void => {
    table = toolTable({
      install,
      gatewayEnv,
      images,
      skillRoots: options.skillRoots,
      renderer,
      connection,
      runtime,
      exportsDir: options.toolResultsDir,
      schemas: catalog.schemas.value,
      job: jobTool(jobs),
      background: backgroundTool(detacher),
    });
    registry.install(toolsExtension(table, limiter, detacher));
  };
  registry.install(promptExtension(install));
  registry.install(skillsExtension(await loadSkills(options.skillRoots, report)));
  registry.install(contextExtension({ inputs, mayChangeSystem: (name) => mayChangeSystem(table)(name) }));
  catalog.schemas.subscribe(installTools);
  registry.install(budget.extension());
  registry.install(jobsExtension(jobTask));
  registry.install(detacher.extension());
  harness.resume();

  const runs = new RunWatcher({ dir: install.runsDir, pollMs: 5_000, report });
  runs.start();
  const loader = new RuntimeLoader({ install, env: gatewayEnv, runs, maxBackoffMs: 60_000, report });
  const intake = new EventIntake({
    connection,
    inputs,
    drafts: new EventDrafts(renderer),
    report,
  });
  const refreshCatalog = (): void => {
    catalog.refresh(connection, context).then(
      (result) => {
        if (result === "missing") loader.trigger("tools_missing");
        else loader.toolsComplete();
      },
      (error: unknown) => report(error),
    );
  };
  connection.status.subscribe((status) => {
    if (status.state === "up") {
      refreshCatalog();
    }
  });
  const stopRuntimeRefresh = connection.onHarnessStatus(refreshCatalog);
  const refreshTimer = setInterval(() => {
    if (connection.status.value.state === "up") refreshCatalog();
  }, 15_000);
  refreshTimer.unref();
  loader.status.subscribe((status) => {
    if (status.state === "loaded") {
      connection.retryNow();
      refreshCatalog();
    }
  });
  await intake.start(context);
  loader.start();
  const trajectory = new TrajectorySender({ harness, connection, runtime, batch: 50, maxBackoffMs: 60_000, report });
  trajectory.start(context);
  const kicker = new QueueKicker({ harness, inputs, backoffMs: (n) => Math.min(60_000 * 2 ** n, 1_800_000), report });
  kicker.start();
  const conversations = new Conversations({
    harness,
    runtime,
    gatewayFacts: (c) => observer.gatewayFacts(c),
    defaults: options.defaults,
    snapshotTimeoutMs: 5_000,
  });
  const history = new ConversationHistory(harness, inputs, options.models);

  return {
    harness,
    storage,
    inputs,
    conversations,
    history,
    runtime,
    jobs,
    detacher,
    connection,
    loader,
    runs,
    stateDir: install.stateDir,
    close: async (closeContext) => {
      clearInterval(refreshTimer);
      stopRuntimeRefresh();
      kicker.stop();
      await trajectory.stop();
      runtime.close();
      await intake.stop();
      await loader.close();
      runs.close();
      await harness.close(closeContext);
      await connection.close();
    },
  };
}

/** The plan's tool table. Rebuilt and installed again when McpServer's runtime tool schemas change. */
function toolTable(deps: ToolTableDeps): AgentToolTable {
  return {
    read: readTool(deps.images, deps.skillRoots),
    bash: bashTool({ install: deps.install, gatewayEnv: deps.gatewayEnv, renderer: deps.renderer }),
    edit: editTool(),
    write: writeTool(),
    ...runtimeTools(deps.schemas, {
      connection: deps.connection,
      install: deps.install,
      renderer: deps.renderer,
      runtime: deps.runtime,
      exportsDir: deps.exportsDir,
    }),
    job: deps.job,
    background: deps.background,
  };
}
