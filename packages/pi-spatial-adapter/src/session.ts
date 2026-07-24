import { ModelRegistry, ModelRuntime, SessionManager, createAgentSession, readStoredCredential } from "@earendil-works/pi-coding-agent";
import { InMemoryCredentialStore, type Model } from "@earendil-works/pi-ai";
import { customTools, assertNoBuiltinTools, assertToolInventory } from "./tools.js";
import type { ToolDefinition } from "@earendil-works/pi-coding-agent";
import { chmodSync, closeSync, constants, existsSync, fsyncSync, lstatSync, mkdirSync, openSync, readFileSync, unlinkSync, writeSync } from "node:fs";
import { createHash } from "node:crypto";
import { basename, dirname, relative, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";

export const MODEL_PROVIDER = "openai-codex";
export const API_KEY_MODEL_PROVIDER = "openai";
export const MODEL_ID = "gpt-5.6-luna";
export const THINKING_LEVEL = "medium" as const;
export const REQUIRED_API = "openai-codex-responses";
export const API_KEY_REQUIRED_API = "openai-responses";
export const SESSION_DIR_ENV = "PI_SPATIAL_SESSION_DIR";
export const AGENT_CWD = process.env.PI_SPATIAL_AGENT_CWD ?? "/work";
export const PINNED_PI_VERSION = "0.80.10";

// The adapter is a dedicated process. Set this before Pi can create a session file.
process.umask(0o077);

export type SessionEvidenceState = "complete" | "partial" | "unavailable";

export interface SystemPromptEvidenceMetadata {
  readonly relativePath: "pi-prompt/system.txt";
  readonly byteCount: number;
  readonly sha256: string;
}
export interface InitialPromptEvidenceMetadata {
  readonly relativePath: "pi-prompt/initial.txt";
  readonly byteCount: number;
  readonly sha256: string;
}

export interface SessionEvidenceMetadata {
  /** Safe path relative to the attempt cwd; never an absolute host path. */
  readonly relativePath?: string;
  readonly persisted: boolean;
  readonly state: SessionEvidenceState;
  readonly systemPrompt?: SystemPromptEvidenceMetadata;
  readonly initialPrompt?: InitialPromptEvidenceMetadata;
}

export interface SessionAdapterHandle {
  prompt(prompt: string): Promise<unknown>;
  subscribe(listener: (event: unknown) => void): void;
  abort: () => Promise<void>;
  dispose: () => void;
  sessionEvidence: (completed: boolean) => SessionEvidenceMetadata;
}

interface SessionDirectory {
  readonly name: string;
  readonly path: string;
}

function configuredSessionDirectory(): SessionDirectory {
  const value = process.env[SESSION_DIR_ENV];
  if (value === undefined || !/^[A-Za-z0-9][A-Za-z0-9._-]*$/.test(value)) {
    throw new Error(`${SESSION_DIR_ENV} must be provided as one simple relative directory name`);
  }
  const path = resolve(process.cwd(), value);
  if (dirname(path) !== process.cwd()) throw new Error(`${SESSION_DIR_ENV} must stay beneath process.cwd()`);
  return { name: value, path };
}

function ensurePrivateSessionDirectory(directory: SessionDirectory): void {
  try {
    const stat = lstatSync(directory.path);
    if (stat.isSymbolicLink() || !stat.isDirectory()) throw new Error(`${SESSION_DIR_ENV} must be a real directory`);
    chmodSync(directory.path, 0o700);
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code !== "ENOENT") throw error;
    try {
      mkdirSync(directory.path, { mode: 0o700 });
    } catch (mkdirError) {
      if ((mkdirError as NodeJS.ErrnoException).code !== "EEXIST") throw mkdirError;
    }
    const stat = lstatSync(directory.path);
    if (stat.isSymbolicLink() || !stat.isDirectory()) throw new Error(`${SESSION_DIR_ENV} must be a real directory`);
    chmodSync(directory.path, 0o700);
  }
}

function ensurePrivatePromptDirectory(): string {
  const path = resolve(process.cwd(), "pi-prompt");
  try {
    const stat = lstatSync(path);
    if (stat.isSymbolicLink() || !stat.isDirectory()) throw new Error("pi-prompt must be a real directory");
    chmodSync(path, 0o700);
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code !== "ENOENT") throw error;
    try {
      mkdirSync(path, { mode: 0o700 });
    } catch (mkdirError) {
      if ((mkdirError as NodeJS.ErrnoException).code !== "EEXIST") throw mkdirError;
    }
    const stat = lstatSync(path);
    if (stat.isSymbolicLink() || !stat.isDirectory()) throw new Error("pi-prompt must be a real directory");
    chmodSync(path, 0o700);
  }
  return path;
}

export function retainSystemPromptEvidence(systemPrompt: string): SystemPromptEvidenceMetadata {
  const promptDirectory = ensurePrivatePromptDirectory();
  const path = resolve(promptDirectory, "system.txt");
  const bytes = Buffer.from(systemPrompt, "utf8");
  const fd = openSync(path, constants.O_WRONLY | constants.O_CREAT | constants.O_EXCL | constants.O_NOFOLLOW, 0o600);
  try {
    const written = writeSync(fd, bytes);
    if (written !== bytes.length) throw new Error("system prompt sidecar write was incomplete");
    fsyncSync(fd);
  } catch (error) {
    closeSync(fd);
    try { unlinkSync(path); } catch { /* preserve the original setup failure */ }
    throw error;
  }
  closeSync(fd);
  try {
    const directoryFd = openSync(promptDirectory, constants.O_RDONLY | constants.O_DIRECTORY | constants.O_NOFOLLOW);
    try { fsyncSync(directoryFd); } finally { closeSync(directoryFd); }
  } catch {
    // Directory fsync is best effort; the file itself was fsynced above.
  }
  return {
    relativePath: "pi-prompt/system.txt",
    byteCount: bytes.length,
    sha256: createHash("sha256").update(bytes).digest("hex"),
  };
}

export function retainInitialPromptEvidence(initialPrompt: string): InitialPromptEvidenceMetadata {
  const promptDirectory = ensurePrivatePromptDirectory();
  const path = resolve(promptDirectory, "initial.txt");
  const bytes = Buffer.from(initialPrompt, "utf8");
  const fd = openSync(path, constants.O_WRONLY | constants.O_CREAT | constants.O_EXCL | constants.O_NOFOLLOW, 0o600);
  try {
    if (writeSync(fd, bytes) !== bytes.length) throw new Error("initial prompt sidecar write was incomplete");
    fsyncSync(fd);
  } catch (error) {
    closeSync(fd);
    try { unlinkSync(path); } catch { /* preserve original setup failure */ }
    throw error;
  }
  closeSync(fd);
  return { relativePath: "pi-prompt/initial.txt", byteCount: bytes.length, sha256: createHash("sha256").update(bytes).digest("hex") };
}

export function createSessionManager(cwd: string = AGENT_CWD): SessionManager {
  const directory = configuredSessionDirectory();
  ensurePrivateSessionDirectory(directory);
  return SessionManager.create(cwd, directory.path);
}

function safeRelativeSessionFile(manager: SessionManager): string | undefined {
  const file = manager.getSessionFile();
  if (!file) return undefined;
  const sessionDirectory = resolve(manager.getSessionDir());
  const relativeToSessionDirectory = relative(sessionDirectory, resolve(file));
  const relativeSessionDirectory = relative(process.cwd(), sessionDirectory);
  if (!relativeToSessionDirectory || relativeToSessionDirectory.includes(sep) || relativeToSessionDirectory.startsWith("..") || !/^[A-Za-z0-9][A-Za-z0-9._-]*$/.test(relativeSessionDirectory) || !basename(relativeToSessionDirectory).endsWith(".jsonl")) {
    return undefined;
  }
  let stat: ReturnType<typeof lstatSync>;
  try {
    stat = lstatSync(file);
  } catch (error) {
    if ((error as NodeJS.ErrnoException).code === "ENOENT") return undefined;
    throw error;
  }
  if (!stat.isFile()) return undefined;
  return `${relativeSessionDirectory}/${basename(relativeToSessionDirectory)}`;
}

export function sessionEvidenceForManager(manager: SessionManager, completed: boolean): SessionEvidenceMetadata {
  const relativePath = safeRelativeSessionFile(manager);
  const persisted = manager.isPersisted();
  return {
    ...(relativePath ? { relativePath } : {}),
    persisted: persisted && relativePath !== undefined,
    state: !persisted || relativePath === undefined ? "unavailable" : completed ? "complete" : "partial",
  };
}

export function resolvePinnedPiCli(): string {
  const packageName = "@earendil-works/pi-coding-agent";
  let directory = dirname(fileURLToPath(import.meta.url));
  while (true) {
    const packageDirectory = resolve(directory, "node_modules", packageName);
    const packageJson = resolve(packageDirectory, "package.json");
    if (existsSync(packageJson)) {
      const metadata = JSON.parse(readFileSync(packageJson, "utf8")) as { version?: unknown; bin?: unknown };
      if (metadata.version !== PINNED_PI_VERSION) throw new Error(`expected pinned Pi ${PINNED_PI_VERSION}`);
      const bin = metadata.bin;
      const binPath = typeof bin === "object" && bin !== null && "pi" in bin && typeof bin.pi === "string" ? bin.pi : undefined;
      if (binPath !== "dist/cli.js") throw new Error("pinned Pi package has an unexpected pi bin");
      const cli = resolve(packageDirectory, binPath);
      if (!existsSync(cli)) throw new Error("pinned Pi CLI entrypoint is missing");
      return cli;
    }
    const parent = dirname(directory);
    if (parent === directory) break;
    directory = parent;
  }
  throw new Error("pinned Pi package cannot be resolved");
}

export interface PinnedPiExportCommand {
  readonly executable: string;
  readonly args: readonly [string, "--export", string, string];
  readonly packageVersion: typeof PINNED_PI_VERSION;
}

export function resolvePinnedPiExportCommand(input: string, output: string): PinnedPiExportCommand {
  return {
    executable: process.execPath,
    args: [resolvePinnedPiCli(), "--export", input, output],
    packageVersion: PINNED_PI_VERSION,
  };
}

export function modelProviderForAuthMode(authMode: AuthMode): typeof MODEL_PROVIDER | typeof API_KEY_MODEL_PROVIDER {
  return authMode === "codex-oauth" ? MODEL_PROVIDER : API_KEY_MODEL_PROVIDER;
}

export function requiredApiForAuthMode(authMode: AuthMode): typeof REQUIRED_API | typeof API_KEY_REQUIRED_API {
  return authMode === "codex-oauth" ? REQUIRED_API : API_KEY_REQUIRED_API;
}

export function resolveConfiguredModel(
  registry: ModelRegistry,
  authMode: AuthMode = "codex-oauth",
): Model<"openai-codex-responses" | "openai-responses"> {
  const provider = modelProviderForAuthMode(authMode);
  const requiredApi = requiredApiForAuthMode(authMode);
  const model = registry.find(provider, MODEL_ID);
  if (!model || model.api !== requiredApi || !model.input.includes("image") || !model.reasoning) {
    throw new Error("configured model is missing, has the wrong API, does not accept images, or does not support thinking");
  }
  return model as Model<"openai-codex-responses" | "openai-responses">;
}

export type AuthMode = "codex-oauth" | "openai-api-key";

export interface StoredAuthOptions {
  authMode: AuthMode;
  authPath?: string;
  apiKey?: string;
  modelsPath?: string;
}

export interface SessionConfig {
  thinkingLevel: typeof THINKING_LEVEL;
}

export function validateSessionConfig(config: SessionConfig): void {
  if (config.thinkingLevel !== THINKING_LEVEL) throw new Error("unsupported thinking level");
}

export async function createFreshSession(tools: readonly ToolDefinition[], options: StoredAuthOptions, config: SessionConfig, initialPrompt: string) {
  validateSessionConfig(config);
  const manager = createSessionManager();
  const initialPromptEvidence = retainInitialPromptEvidence(initialPrompt);
  let runtime: ModelRuntime;
  if (options.authMode === "codex-oauth") {
    if (!options.authPath) throw new Error("Codex OAuth auth path is required");
    const credential = readStoredCredential(MODEL_PROVIDER, options.authPath);
    if (!credential || credential.type !== "oauth") throw new Error("Codex OAuth credentials are not stored");
    runtime = await ModelRuntime.create({ authPath: options.authPath, modelsPath: options.modelsPath });
  } else {
    if (!options.apiKey) throw new Error("OpenAI API key is required");
    const credentials = new InMemoryCredentialStore();
    await credentials.modify(API_KEY_MODEL_PROVIDER, async () => ({
      type: "api_key",
      key: options.apiKey,
    }));
    runtime = await ModelRuntime.create({
      credentials,
      modelsPath: options.modelsPath,
    });
  }
  const registry = new ModelRegistry(runtime);
  await registry.refresh();
  const model = resolveConfiguredModel(registry, options.authMode);
  if (options.authMode === "codex-oauth") {
    if (!registry.isUsingOAuth(model)) throw new Error("configured model is not using Codex OAuth");
  } else {
    const status = registry.getProviderAuthStatus(API_KEY_MODEL_PROVIDER);
    if (registry.isUsingOAuth(model) || !status.configured || status.source !== "stored") {
      throw new Error("configured model is not using an OpenAI API key");
    }
  }
  const custom = customTools(tools);
  const available = custom.map((tool) => tool.name);
  assertNoBuiltinTools(available);
  if (available.length !== 3) throw new Error("exactly three custom tools are required");
  const result = await createAgentSession({
    cwd: AGENT_CWD,
    model,
    thinkingLevel: config.thinkingLevel,
    modelRuntime: runtime,
    sessionManager: manager,
    noTools: "builtin",
    tools: available,
    customTools: custom,
  });
  const active = result.session.getActiveToolNames();
  assertToolInventory(active);
  let systemPrompt: SystemPromptEvidenceMetadata;
  try {
    systemPrompt = retainSystemPromptEvidence(result.session.systemPrompt);
  } catch (error) {
    result.session.dispose();
    throw error;
  }
  let disposed = false;
  return {
    prompt: (prompt: string) => result.session.prompt(prompt),
    subscribe: (listener: (event: unknown) => void) => { result.session.subscribe((event) => listener(event)); },
    abort: () => result.session.abort(),
    dispose: () => {
      if (!disposed) {
        disposed = true;
        result.session.dispose();
      }
    },
    sessionEvidence: (completed: boolean): SessionEvidenceMetadata => {
      const evidence = sessionEvidenceForManager(manager, completed);
      return evidence.state === "unavailable" ? evidence : { ...evidence, systemPrompt, initialPrompt: initialPromptEvidence };
    },
  } satisfies SessionAdapterHandle;
}
