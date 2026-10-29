import type { DimosInstall } from "./dimos/install.ts";
import type { EnvTarget } from "@earendil-works/pi-durable";
import { NodeExecutionEnv } from "@earendil-works/pi-durable/env/node";
import { delimiter, dirname } from "node:path";

/** `HarnessOptions.env`: each conversation's files and processes live in its `cwd`, or the workspace when it has none. */
export function executionEnv(workspace: () => string): (target: EnvTarget) => NodeExecutionEnv {
  return (target) => new NodeExecutionEnv({ cwd: target.cwd ?? workspace() });
}

const SECRET_NAME =
  /(_API_KEY|_TOKEN|_SECRET|_TOKEN_FILE)$|^(AWS_ACCESS_KEY_ID|AWS_SECRET_ACCESS_KEY|AWS_BEARER_TOKEN_BEDROCK|GOOGLE_APPLICATION_CREDENTIALS)$/;

/**
 * bash's whole environment, passed with `inheritEnv: false`: the gateway's variables minus provider credentials
 * (names ending in _API_KEY, _TOKEN, _SECRET or _TOKEN_FILE, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY,
 * AWS_BEARER_TOKEN_BEDROCK and GOOGLE_APPLICATION_CREDENTIALS), DimOS's Python first on PATH, DIMOS_ROOT, and
 * RUST_LOG=error. Never `.env`.
 */
export function bashEnvironment(install: DimosInstall, gatewayEnv: NodeJS.ProcessEnv): Record<string, string> {
  const env: Record<string, string> = {};
  for (const [name, value] of Object.entries(gatewayEnv)) {
    if (value !== undefined && !SECRET_NAME.test(name)) env[name] = value;
  }
  const pythonDir = dirname(install.python);
  const path = (gatewayEnv["PATH"] ?? "").split(delimiter).filter((entry) => entry !== "" && entry !== pythonDir);
  env["PATH"] = [pythonDir, ...path].join(delimiter);
  env["DIMOS_ROOT"] = install.root;
  // zenoh-python's init_log_from_env_or reads RUST_LOG; without it every Dimos.connect() script prints peer warnings.
  env["RUST_LOG"] = "error";
  return env;
}
