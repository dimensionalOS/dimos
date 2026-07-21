import { createInterface, type ReadLine } from "node:readline";
import { stdin, stdout, stderr } from "node:process";
import { encodeFrame, parseFrame, MAX_PENDING_REQUESTS, type RunStart, type ToolReply } from "./protocol.js";
import { TOOL_NAMES, createBroker, toolDefinitions, type ToolBroker } from "./tools.js";
import { createFreshSession } from "./session.js";

export interface SessionAdapter {
  prompt(prompt: string): Promise<unknown>;
  subscribe(listener: (event: unknown) => void): void;
  abort?: () => Promise<void>;
  dispose(): void;
}
export type SessionFactory = (broker: ToolBroker, start: RunStart) => Promise<SessionAdapter>;

const defaultFactory: SessionFactory = (broker, start) => createFreshSession(toolDefinitions(broker, start.config.answerType, start.config.promptMode), {
  // Credentials are deliberately not representable in protocol frames.
  authPath: process.env.PI_SPATIAL_AUTH_PATH ?? (() => { throw new Error("PI_SPATIAL_AUTH_PATH is required"); })(),
  modelsPath: process.env.PI_SPATIAL_MODELS_PATH,
}, { thinkingLevel: start.config.thinkingLevel }) as Promise<SessionAdapter>;

const CONTINUATION_PROMPT = "Continue working on the task and submit an answer when ready.";

function diagnostic(output: NodeJS.WritableStream, message: string): void { output.write(`${message}\n`); }
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
    const session = await factory(activeBroker, start);
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
          void session.abort?.();
        }
      }
    });
    let reason: "submitted" | "max_turns" | "max_tool_calls" | "timeout" | "session_error" | "protocol_error" | "pre_image_policy_violation" | "post_image_policy_violation" = "session_error";
    let errorMessage: string | undefined;
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
      errorMessage = (error instanceof Error ? error.message : "session failed").slice(0, 1024);
      diagnostic(diagnostics, errorMessage);
      const budgetExhausted = activeBroker.toolCallCount() >= start.budget.maxToolCalls || budgetAbort || turns >= start.budget.maxTurns;
      if (errorMessage === "pre_image_policy_violation" || errorMessage === "post_image_policy_violation") {
        reason = errorMessage;
      } else if (budgetExhausted && start.config.promptMode === "visualization_encouraged") {
        reason = activeBroker.visualizationSatisfied() ? "post_image_policy_violation" : "pre_image_policy_violation";
      } else if (errorMessage === "run timeout") {
        reason = "timeout";
        void session.abort?.();
      }
      else if (activeBroker.toolCallCount() >= start.budget.maxToolCalls) reason = "max_tool_calls";
      else if (budgetAbort || turns >= start.budget.maxTurns) reason = "max_turns";
      else reason = "session_error";
    } finally {
      session.dispose();
    }
    emit({ version: 2, type: "run_complete", id: runId, ok: reason === "submitted", reason, ...(errorMessage ? { error: errorMessage } : {}) });
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
