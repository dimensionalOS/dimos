import type { AgentPaths } from "#dimos/agents/gateway/config/paths.ts";
import type { DimosInstall } from "#dimos/agents/harness/dimos/install.ts";
import type { Provider } from "@earendil-works/pi-ai";
import { BACKGROUND_CONTEXT, withCancel } from "@earendil-works/chord/context";
import { ConfigStore } from "#dimos/agents/gateway/config/store.ts";
import { GatewayServer } from "#dimos/agents/gateway/server/host.ts";
import { LoginFlows } from "#dimos/agents/gateway/auth/dialog.ts";
import { ModelCatalog, openModels } from "#dimos/agents/gateway/auth/models.ts";
import { Replies } from "#dimos/agents/gateway/server/replies.ts";
import { appendFileSync, mkdirSync } from "node:fs";
import { buildInfo } from "#dimos/agents/gateway/config/build.ts";
import { dirname } from "node:path";
import { epochMs } from "#dimos/agents/shared/protocol/time.ts";
import { migrateDimcode } from "#dimos/agents/gateway/config/migrate.ts";
import { openAgent } from "#dimos/agents/harness/main.ts";

/** How long a `by_request_id` call's first outcome answers its retries. */
const REPLIES_KEPT_MS = 300_000;

export type GatewayLaunch = {
  readonly paths: AgentPaths;
  readonly install: DimosInstall;
  /** Bundled skills, `harness/skills/` next to the bundle. */
  readonly bundledSkillsDir: string;
  readonly env: NodeJS.ProcessEnv;
  /** Model providers beside pi-ai's built-in ones; the faux provider in tests, none in `dimos agent gateway`. */
  readonly providers: readonly Provider[];
};

function describe(error: unknown): string {
  return error instanceof Error ? (error.stack ?? `${error.name}: ${error.message}`) : String(error);
}

/** Appends one JSON line per failure or DimOS link change to the gateway log, and writes it to stderr for the service journal. */
function reporter(logFile: string): (error: unknown) => void {
  mkdirSync(dirname(logFile), { recursive: true, mode: 0o700 });
  return (error) => {
    const line = JSON.stringify({ at: new Date().toISOString(), message: describe(error) });
    process.stderr.write(`${line}\n`);
    try {
      appendFileSync(logFile, `${line}\n`, { mode: 0o600 });
    } catch {
      // The log is best effort; stderr already has the line.
    }
  };
}

/** Everything the server needs after the socket is bound, in the order recovery needs. */
async function openServices(launch: GatewayLaunch, report: (error: unknown) => void) {
  const { paths, install } = launch;
  const migration = await migrateDimcode(paths);
  if (migration.kind === "copied")
    process.stderr.write(`Copied ${migration.files.join(", ")} into ${paths.configDir}\n`);
  const config = await ConfigStore.open(paths.configFile);
  const models = await openModels(paths.authFile, paths.modelsFile, launch.providers, report);
  const catalog = new ModelCatalog(models);
  await catalog.refresh();
  const logins = new LoginFlows(models, catalog, report, 600_000);
  const agent = await openAgent(
    {
      install,
      models,
      storePath: paths.storeFile,
      toolResultsDir: paths.toolResultsDir,
      renderCacheDir: paths.renderCacheDir,
      skillRoots: [launch.bundledSkillsDir, paths.userSkillsDir],
      defaults: () => ({
        model: config.status.value.model,
        thinkingLevel: config.status.value.thinkingLevel,
        workspace: config.status.value.workspace === null ? install.root : config.status.value.workspace,
      }),
      gatewayEnv: launch.env,
      report,
    },
    BACKGROUND_CONTEXT,
  );
  return { agent, models, catalog, logins, config };
}

/**
 * Binds the socket, migrates `~/.dimcode` once, opens config, models, the catalog, the login flows and the agent, then
 * serves every object. Resolves after SIGTERM, SIGINT or `lifecycle.stop` closed the socket and the agent; rejects
 * after closing them when serving failed. A crash is written to the gateway log before the process exits 1, as a
 * gateway a terminal started has no stderr.
 */
export async function runGateway(launch: GatewayLaunch): Promise<void> {
  const server = await GatewayServer.bind(launch.paths.socket);
  const report = reporter(launch.paths.logFile);
  const crash = (error: unknown) => {
    report(error);
    process.exit(1);
  };
  process.on("uncaughtException", crash);
  process.on("unhandledRejection", crash);
  try {
    await serveUntilStopped(launch, server, report);
  } finally {
    process.off("uncaughtException", crash);
    process.off("unhandledRejection", crash);
  }
}

async function serveUntilStopped(
  launch: GatewayLaunch,
  server: GatewayServer,
  report: (error: unknown) => void,
): Promise<void> {
  const startedAt = epochMs(Date.now());
  const services = await openServices(launch, report).catch(async (error: unknown) => {
    report(error);
    await server.close();
    throw error;
  });
  const failure = await new Promise<{ readonly error: unknown } | null>((resolve) => {
    let stopping = false;
    let failed: { readonly error: unknown } | null = null;
    const lifetime = withCancel(BACKGROUND_CONTEXT);
    const shutdown = () => {
      if (stopping) return;
      stopping = true;
      lifetime.cancel(new Error("The gateway is stopping"));
      void server
        .close()
        .then(() => services.agent.close(BACKGROUND_CONTEXT))
        .catch(report)
        .finally(() => resolve(failed));
    };
    process.once("SIGTERM", shutdown);
    process.once("SIGINT", shutdown);
    server
      .serve({
        ...services,
        process: { build: buildInfo(), pid: process.pid, startedAt },
        replies: new Replies(REPLIES_KEPT_MS, lifetime.context),
        report,
        shutdown,
      })
      .catch((error: unknown) => {
        report(error);
        if (!stopping) failed = { error };
        shutdown();
      });
  });
  if (failure !== null) throw failure.error;
}
