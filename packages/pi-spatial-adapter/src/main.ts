import { createInterface, type ReadLine } from "node:readline";
import { stdin, stdout, stderr } from "node:process";
import { encodeFrame, parseFrame, MAX_PENDING_REQUESTS, type RunStart, type SessionEvidenceFrame, type ToolReply } from "./protocol.js";
import { TOOL_NAMES, createBroker, toolDefinitions, type ToolBroker } from "./tools.js";
import { createFreshSession, type StoredAuthOptions } from "./session.js";
import type { SessionEvidenceMetadata } from "./session.js";

export interface SessionAdapter {
  prompt(prompt: string): Promise<unknown>;
  subscribe(listener: (event: unknown) => void): void;
  abort?: () => Promise<void>;
  dispose(): void;
  sessionEvidence?: (completed: boolean) => SessionEvidenceMetadata;
}
export type SessionFactory = (broker: ToolBroker, start: RunStart) => Promise<SessionAdapter>;

export function authOptionsFromEnvironment(env: NodeJS.ProcessEnv): StoredAuthOptions {
  const mode = env.PI_SPATIAL_AUTH_MODE ?? "codex-oauth";
  if (mode === "codex-oauth") {
    if (!env.PI_SPATIAL_AUTH_PATH) throw new Error("PI_SPATIAL_AUTH_PATH is required");
    return {
      authMode: mode,
      authPath: env.PI_SPATIAL_AUTH_PATH,
      modelsPath: env.PI_SPATIAL_MODELS_PATH,
    };
  }
  if (mode === "openai-api-key") {
    if (!env.OPENAI_API_KEY) throw new Error("OPENAI_API_KEY is required");
    return {
      authMode: mode,
      apiKey: env.OPENAI_API_KEY,
      modelsPath: env.PI_SPATIAL_MODELS_PATH,
    };
  }
  throw new Error("PI_SPATIAL_AUTH_MODE is invalid");
}

const defaultFactory: SessionFactory = (broker, start) => createFreshSession(
  toolDefinitions(broker, start.config.answerType, start.config.promptMode),
  // Credentials are deliberately not representable in protocol frames.
  authOptionsFromEnvironment(process.env),
  { thinkingLevel: start.config.thinkingLevel },
  start.prompt,
) as Promise<SessionAdapter>;

const CONTINUATION_PROMPT = "Continue working on the task and submit an answer when ready.";
const UNAVAILABLE_EVIDENCE: SessionEvidenceMetadata = { state: "unavailable", persisted: false };

function frameEvidence(evidence: SessionEvidenceMetadata): SessionEvidenceFrame {
  const path = evidence.relativePath;
  const safePath = path !== undefined && path.length <= 256 && /^pi-session\/[A-Za-z0-9][A-Za-z0-9._-]*\.jsonl$/.test(path)
    ? path
    : undefined;
  const prompt = evidence.systemPrompt;
  const safePrompt = evidence.state !== "unavailable" && prompt?.relativePath === "pi-prompt/system.txt" && Number.isSafeInteger(prompt.byteCount) && prompt.byteCount >= 0 && /^[a-f0-9]{64}$/.test(prompt.sha256)
    ? { relative_path: prompt.relativePath, byte_count: prompt.byteCount, sha256: prompt.sha256 }
    : undefined;
  const initial = evidence.initialPrompt;
  const safeInitial = evidence.state !== "unavailable" && initial?.relativePath === "pi-prompt/initial.txt" && Number.isSafeInteger(initial.byteCount) && initial.byteCount >= 0 && /^[a-f0-9]{64}$/.test(initial.sha256)
    ? { relative_path: initial.relativePath, byte_count: initial.byteCount, sha256: initial.sha256 }
    : undefined;
  return { state: evidence.state, persisted: evidence.persisted, ...(safePath ? { relative_path: safePath } : {}), ...(safePrompt ? { system_prompt: safePrompt } : {}), ...(safeInitial ? { initial_prompt: safeInitial } : {}) };
}

function diagnostic(output: NodeJS.WritableStream, message: string): void { output.write(`${message.replace(/[\r\n]+/g, " ").slice(0, 1024)}\n`); }
function frameType(event: unknown): string | undefined {
  if (typeof event !== "object" || event === null || Array.isArray(event)) return undefined;
  const type = (event as Record<string, unknown>).type;
  return typeof type === "string" ? type : undefined;
}

export async function runAdapter(
  input: NodeJS.ReadableStream = stdin,
  output: NodeJS.WritableStream = stdout,
  diagnostics: NodeJS.WritableStream = stderr,
  factory: SessionFactory = defaultFactory,
): Promise<void> {
  const lines: ReadLine = createInterface({ input, crlfDelay: Infinity });
  let started = false;
  let finished = false;
  let runId = "";
  let runPromise: Promise<void> | undefined;
  let broker: ToolBroker | undefined;

  const emit = (frame: Parameters<typeof encodeFrame>[0]): void => { output.write(encodeFrame(frame)); };
  const fail = (message: string): void => {
    diagnostic(diagnostics, message);
    emit({ version: 2, type: "protocol_error", error: message });
  };
  const execute = async (start: RunStart): Promise<void> => {
    const activeBroker = broker as ToolBroker;
    let session: SessionAdapter | undefined;
    let reason: "submitted" | "max_turns" | "max_tool_calls" | "timeout" | "session_error" | "protocol_error" | "pre_image_policy_violation" | "post_image_policy_violation" = "session_error";
    let evidence: SessionEvidenceMetadata = UNAVAILABLE_EVIDENCE;
    try {
      session = await factory(activeBroker, start);
      let turns = 0;
      const deadline = Date.now() + start.budget.timeoutMs;
      let budgetAbort = false;
      session.subscribe((event) => {
        const type = frameType(event);
        if (type) emit({ version: 2, type: "transcript", event: type });
        if (type === "turn_end") {
          turns += 1;
          if (turns >= start.budget.maxTurns) {
            budgetAbort = true;
            void session?.abort?.();
          }
        }
      });
      try {
        let prompt = start.prompt;
        while (true) {
          const remaining = deadline - Date.now();
          if (remaining <= 0) { reason = "timeout"; break; }
          await Promise.race([
            session.prompt(prompt),
            new Promise<never>((_, reject) => setTimeout(() => reject(new Error("run timeout")), remaining)),
          ]);
          if (activeBroker.submissionAccepted()) { reason = "submitted"; break; }
          if (start.config.promptMode === "visualization_encouraged") {
            reason = activeBroker.visualizationSatisfied()
              ? "post_image_policy_violation"
              : "pre_image_policy_violation";
            break;
          }
          if (activeBroker.toolCallCount() >= start.budget.maxToolCalls) { reason = "max_tool_calls"; break; }
          if (turns >= start.budget.maxTurns || budgetAbort) { reason = "max_turns"; break; }
          if (Date.now() >= deadline) { reason = "timeout"; break; }
          prompt = CONTINUATION_PROMPT;
          emit({ version: 2, type: "transcript", event: "continuation", delta: prompt });
        }
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : "session failed";
        diagnostic(diagnostics, errorMessage);
        const budgetExhausted = activeBroker.toolCallCount() >= start.budget.maxToolCalls || budgetAbort || turns >= start.budget.maxTurns;
        if (errorMessage === "pre_image_policy_violation" || errorMessage === "post_image_policy_violation") {
          reason = errorMessage;
        } else if (budgetExhausted && start.config.promptMode === "visualization_encouraged") {
          reason = activeBroker.visualizationSatisfied() ? "post_image_policy_violation" : "pre_image_policy_violation";
        } else if (errorMessage === "run timeout") {
          reason = "timeout";
          await session.abort?.().catch((abortError: unknown) => diagnostic(diagnostics, abortError instanceof Error ? abortError.message : "session abort failed"));
        } else if (activeBroker.toolCallCount() >= start.budget.maxToolCalls) reason = "max_tool_calls";
        else if (budgetAbort || turns >= start.budget.maxTurns) reason = "max_turns";
        else reason = "session_error";
      }
    } catch (error) {
      reason = "session_error";
      diagnostic(diagnostics, error instanceof Error ? error.message : "session setup failed");
    } finally {
      if (session) {
        try {
          session.dispose();
        } catch (error) {
          reason = "session_error";
          diagnostic(diagnostics, error instanceof Error ? error.message : "session dispose failed");
        }
        try {
          evidence = session.sessionEvidence?.(reason === "submitted") ?? UNAVAILABLE_EVIDENCE;
        } catch (error) {
          evidence = UNAVAILABLE_EVIDENCE;
          diagnostic(diagnostics, error instanceof Error ? error.message : "session evidence failed");
        }
      }
    }
    emit({ version: 2, type: "run_complete", id: runId, ok: reason === "submitted", reason, session_evidence: frameEvidence(evidence) });
  };

  try {
    for await (const line of lines) {
      const frame = parseFrame(line);
      if (frame.type === "run_start") {
        if (started) throw new Error("duplicate run_start frame");
        started = true;
        runId = frame.id;
        broker = createBroker((toolFrame) => emit(toolFrame), MAX_PENDING_REQUESTS, frame.budget.maxToolCalls, frame.config.answerType);
        emit({ version: 2, type: "run_started", id: runId, tools: TOOL_NAMES });
        runPromise = execute(frame);
        continue;
      }
      if (!started || finished || !broker) throw new Error("tool_reply received outside an active run");
      broker.reply(frame as ToolReply);
    }
    if (!started) throw new Error("missing run_start frame");
    broker?.close("host input closed before tool reply");
    await runPromise;
    finished = true;
  } catch (error) {
    const message = error instanceof Error ? error.message : "protocol failure";
    fail(message);
    lines.close();
  }
}

if (process.argv[1]?.endsWith("main.ts") || process.argv[1]?.endsWith("main.js")) void runAdapter();
