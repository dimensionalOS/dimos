import { homedir } from "node:os";
import { join } from "node:path";

/** Every file the gateway owns, from XDG variables with the usual fallbacks. */
export type AgentPaths = {
  /** `~/.config/dimos/agent/`. */
  readonly configDir: string;
  readonly authFile: string;
  readonly configFile: string;
  /** Optional provider overrides in pi's models.json format, such as a base URL. */
  readonly modelsFile: string;
  /** Skills the user adds; the catalog lists them after the bundled ones. */
  readonly userSkillsDir: string;
  /** `~/.local/state/dimos/agent/`. */
  readonly stateDir: string;
  readonly storeFile: string;
  readonly toolResultsDir: string;
  readonly logFile: string;
  /** `~/.cache/dimos/agent/render/`: rendered previews, pruned past 128 MiB. */
  readonly renderCacheDir: string;
  /** `$XDG_RUNTIME_DIR/dimos-agent.sock`, or `stateDir/dimos-agent.sock` without XDG_RUNTIME_DIR. */
  readonly socket: string;
  /** `~/.dimcode/` (or `$DIMCODE_HOME`), read once by the migration. */
  readonly legacyDir: string;
};

export function agentPaths(env: NodeJS.ProcessEnv): AgentPaths {
  const home = env.HOME || homedir();
  const xdg = (name: "XDG_CONFIG_HOME" | "XDG_STATE_HOME" | "XDG_CACHE_HOME", fallback: string) =>
    env[name] || join(home, fallback);
  const configDir = join(xdg("XDG_CONFIG_HOME", ".config"), "dimos", "agent");
  const stateDir = join(xdg("XDG_STATE_HOME", ".local/state"), "dimos", "agent");
  const cacheDir = join(xdg("XDG_CACHE_HOME", ".cache"), "dimos", "agent");
  return {
    configDir,
    authFile: join(configDir, "auth.json"),
    configFile: join(configDir, "config.json"),
    modelsFile: join(configDir, "models.json"),
    userSkillsDir: join(configDir, "skills"),
    stateDir,
    storeFile: join(stateDir, "agent.sqlite"),
    toolResultsDir: join(stateDir, "tool-results"),
    logFile: join(stateDir, "gateway.log"),
    renderCacheDir: join(cacheDir, "render"),
    socket: join(env.XDG_RUNTIME_DIR || stateDir, "dimos-agent.sock"),
    legacyDir: env.DIMCODE_HOME || join(home, ".dimcode"),
  };
}
