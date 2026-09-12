import { mkdir } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import {
  createAgentSessionServices,
  createAgentSessionFromServices,
  type AgentSession,
  SessionManager,
  createBashToolDefinition,
} from "@earendil-works/pi-coding-agent";
import { loadConfig, type Config, type Paths } from "./config.js";
import { McpTools } from "./mcp.js";
import { renderExtension } from "./render.js";

export interface SessionHandle {
  session: AgentSession;
  close(): Promise<void>;
}
export async function openSession(
  config: Config,
  paths: Paths,
  manager: SessionManager,
  report: (message: string) => void,
): Promise<SessionHandle> {
  await mkdir(paths.config, { recursive: true, mode: 0o700 });
  await mkdir(paths.sessions, { recursive: true, mode: 0o700 });
  const mcp = new McpTools(async () => (await loadConfig(paths)).mcp, report);
  const services = await createAgentSessionServices({
    cwd: manager.getCwd(),
    agentDir: paths.config,
    resourceLoaderOptions: {
      additionalSkillPaths: [
        fileURLToPath(new URL("../skills", import.meta.url)),
      ],
      extensionFactories: [
        mcp.extension(),
        renderExtension(paths),
        (pi) => {
          pi.registerTool(
            createBashToolDefinition(manager.getCwd(), {
              spawnHook: (context) => ({
                ...context,
                env: {
                  ...context.env,
                  PATH: [
                    config.dimos && dirname(config.dimos),
                    config.python && dirname(config.python),
                    context.env.PATH,
                  ]
                    .filter(Boolean)
                    .join(":"),
                },
              }),
            }),
          );
        },
      ],
      appendSystemPrompt: [
        "You are dimcode, Dimensional's coding and robotics agent. Use DimOS CLI/public Python APIs and the advertised MCP skills. Do not create a parallel runtime or transport service. Select the intended run and endpoint explicitly; never assume the latest run is the intended one. Render actual saved results with dimcode_render. Live previews are labeled context, not historical query results.",
        "Visualize each meaningful sensor or memory operation with dimcode_render. Prefer supported types and existing DimOS visualizers; otherwise generate a self-contained SVG with inline Python from the already evaluated result and render that file. Label proposed ideas or overlays distinctly from observations. Use returned images as visual feedback when interpreting results. DimOS owns reusable visualization semantics; dimcode owns terminal-specific styling and interaction. Built-in views are only point clouds and images/SVGs. For a saved sequence, use the Dimensional skill’s finite frame index format; the model sees a contact sheet and can request an original frame index for closer inspection. Never imply a contact sheet covers every frame.",
        "Selected configuration: " + JSON.stringify(config),
      ],
    },
  });
  const { session } = await createAgentSessionFromServices({
    services,
    sessionManager: manager,
  });
  return {
    session,
    close: async () => {
      try {
        await session.extensionRunner.emit({
          type: "session_shutdown",
          reason: "quit",
        });
      } finally {
        session.dispose();
        await mcp.close();
      }
    },
  };
}
export const authPath = (paths: Paths): string =>
  join(paths.config, "auth.json");
