import { createInterface } from "node:readline";
import { stdin, stdout, stderr } from "node:process";
import {
  encodeCodePolicyFrame,
  parseCodePolicyFrame,
  type CodePolicyOutbound,
} from "./code-policy-protocol.js";
import {
  CODE_POLICY_TOOL_NAME,
  createFreshCodePolicySession,
  type CodePolicyBroker,
} from "./code-policy-session.js";
import type {
  SessionAdapterHandle,
  SessionEvidenceMetadata,
  StoredAuthOptions,
} from "./session.js";

function authOptionsFromEnvironment(env: NodeJS.ProcessEnv): StoredAuthOptions {
  const mode = env.PI_SPATIAL_AUTH_MODE ?? "codex-oauth";
  if (mode === "codex-oauth" && env.PI_SPATIAL_AUTH_PATH) {
    return {
      authMode: mode,
      authPath: env.PI_SPATIAL_AUTH_PATH,
      modelsPath: env.PI_SPATIAL_MODELS_PATH,
    };
  }
  if (mode === "openai-api-key" && env.OPENAI_API_KEY) {
    return {
      authMode: mode,
      apiKey: env.OPENAI_API_KEY,
      modelsPath: env.PI_SPATIAL_MODELS_PATH,
    };
  }
  throw new Error("Pi authentication environment is incomplete");
}

class HostBroker implements CodePolicyBroker {
  private sequence = 0;
  private readonly pending = new Map<
    string,
    { resolve: (value: string) => void; reject: (error: Error) => void }
  >();

  constructor(private readonly emit: (frame: CodePolicyOutbound) => void) {}

  request(
    tool: typeof CODE_POLICY_TOOL_NAME,
    params: { code: string; timeout_s?: number },
  ): Promise<string> {
    const id = `tool-${++this.sequence}`;
    return new Promise<string>((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.emit({ version: 1, type: "tool_call", id, tool, params });
    });
  }

  reply(id: string, ok: boolean, result?: string, error?: string): void {
    const pending = this.pending.get(id);
    if (!pending) throw new Error("unknown or duplicate code-policy tool reply");
    this.pending.delete(id);
    if (ok && result !== undefined) pending.resolve(result);
    else pending.reject(new Error(error ?? "host code-policy tool failed"));
  }

  count(): number {
    return this.sequence;
  }

  close(reason: string): void {
    for (const pending of this.pending.values()) pending.reject(new Error(reason));
    this.pending.clear();
  }
}

function eventType(event: unknown): string | undefined {
  if (typeof event !== "object" || event === null || Array.isArray(event)) return undefined;
  const type = (event as Record<string, unknown>).type;
  return typeof type === "string" ? type : undefined;
}

function evidenceFrame(evidence: SessionEvidenceMetadata) {
  return {
    state: evidence.state,
    persisted: evidence.persisted,
    ...(evidence.relativePath ? { relative_path: evidence.relativePath } : {}),
    ...(evidence.systemPrompt
      ? {
          system_prompt: {
            relative_path: evidence.systemPrompt.relativePath,
            byte_count: evidence.systemPrompt.byteCount,
            sha256: evidence.systemPrompt.sha256,
          },
        }
      : {}),
    ...(evidence.initialPrompt
      ? {
          initial_prompt: {
            relative_path: evidence.initialPrompt.relativePath,
            byte_count: evidence.initialPrompt.byteCount,
            sha256: evidence.initialPrompt.sha256,
          },
        }
      : {}),
  };
}

export async function runCodePolicyAdapter(
  input: NodeJS.ReadableStream = stdin,
  output: NodeJS.WritableStream = stdout,
  diagnostics: NodeJS.WritableStream = stderr,
): Promise<void> {
  const lines = createInterface({ input, crlfDelay: Infinity });
  const emit = (frame: CodePolicyOutbound): void => {
    output.write(encodeCodePolicyFrame(frame));
  };
  let session: SessionAdapterHandle | undefined;
  let broker: HostBroker | undefined;
  let sessionId = "";
  let activeTurn: Promise<void> | undefined;
  let closed = false;
  try {
    for await (const line of lines) {
      const frame = parseCodePolicyFrame(line);
      if (frame.type === "session_start") {
        if (session || broker) throw new Error("duplicate session_start");
        sessionId = frame.id;
        broker = new HostBroker(emit);
        session = await createFreshCodePolicySession(
          broker,
          authOptionsFromEnvironment(process.env),
          { thinkingLevel: frame.thinking_level },
          frame.initial_prompt,
        );
        session.subscribe((event) => {
          const type = eventType(event);
          if (type) emit({ version: 1, type: "transcript", event: type });
        });
        emit({ version: 1, type: "session_started", id: sessionId, tools: ["python_exec"] });
      } else if (frame.type === "prompt") {
        if (!session || !broker || activeTurn) throw new Error("prompt outside idle session");
        const before = broker.count();
        activeTurn = session
          .prompt(frame.text)
          .then((result: unknown) => {
            const finalText =
              typeof result === "string"
                ? result
                : typeof result === "object" && result !== null
                  ? JSON.stringify(result).slice(0, 16_384)
                  : "";
            emit({
              version: 1,
              type: "turn_complete",
              id: frame.id,
              policy_call_count: broker?.count() ?? before,
              final_text: finalText,
            });
          })
          .finally(() => {
            activeTurn = undefined;
          });
      } else if (frame.type === "tool_reply") {
        if (!broker) throw new Error("tool reply before session");
        broker.reply(frame.id, frame.ok, frame.result, frame.error);
      } else if (frame.type === "abort") {
        await session?.abort();
      } else {
        if (!session || !broker) throw new Error("dispose before session");
        await session.abort().catch(() => undefined);
        await activeTurn?.catch(() => undefined);
        session.dispose();
        broker.close("session disposed");
        emit({
          version: 1,
          type: "session_closed",
          id: sessionId,
          evidence: evidenceFrame(session.sessionEvidence(true)),
        });
        closed = true;
        lines.close();
      }
    }
  } catch (error) {
    const message = error instanceof Error ? error.message : "code-policy adapter failure";
    diagnostics.write(`${message.replace(/[\r\n]+/g, " ").slice(0, 1024)}\n`);
    emit({ version: 1, type: "protocol_error", error: message.slice(0, 1024) });
  } finally {
    if (!closed) {
      broker?.close("adapter input closed");
      session?.dispose();
    }
  }
}

if (process.argv[1]?.endsWith("code-policy-main.js")) {
  void runCodePolicyAdapter();
}
