import type {
  PiSessionViewModel,
  SessionEntry,
  SessionPart,
  ToolCallPart,
  ToolResultPart,
} from "./types";

export function indexEntries(
  session: PiSessionViewModel,
): ReadonlyMap<string, SessionEntry> {
  return new Map(session.entries.map((entry) => [entry.id, entry]));
}

export function ancestryFor(
  session: PiSessionViewModel,
  headId: string,
): SessionEntry[] {
  const byId = indexEntries(session);
  const ancestry: SessionEntry[] = [];
  const seen = new Set<string>();
  let current = byId.get(headId);

  while (current !== undefined) {
    if (seen.has(current.id)) {
      throw new Error("Session view model contains a cycle");
    }
    seen.add(current.id);
    ancestry.push(current);
    current =
      current.parentId === null ? undefined : byId.get(current.parentId);
  }

  return ancestry.reverse();
}

export function coalesceToolExchanges(entries: SessionEntry[]): SessionEntry[] {
  const pendingCalls = new Map<
    string,
    Array<{ call: ToolCallPart; result?: ToolResultPart }>
  >();
  const exchanges = new Map<ToolCallPart, ToolResultPart>();
  const matchedResults = new Set<ToolResultPart>();

  for (const entry of entries) {
    for (const part of entry.parts) {
      if (part.type === "tool_call") {
        const pending = pendingCalls.get(part.callId) ?? [];
        pending.push({ call: part });
        pendingCalls.set(part.callId, pending);
      } else if (part.type === "tool_result") {
        const pending = pendingCalls.get(part.callId);
        const exchange = pending?.shift();
        if (exchange !== undefined) {
          exchanges.set(exchange.call, part);
          matchedResults.add(part);
        }
      }
    }
  }

  return entries.flatMap((entry) => {
    const parts: SessionPart[] = [];
    for (const part of entry.parts) {
      if (part.type === "tool_result" && matchedResults.has(part)) {
        continue;
      }
      parts.push(part);
      if (part.type === "tool_call") {
        const result = exchanges.get(part);
        if (result !== undefined) {
          parts.push(result);
        }
      }
    }

    return parts.length === 0 ? [] : [{ ...entry, parts }];
  });
}

export function sandboxCommand(call: ToolCallPart): {
  command: string;
  options?: Record<string, unknown>;
} | null {
  if (
    call.name !== "sandbox_exec" ||
    typeof call.arguments !== "object" ||
    call.arguments === null ||
    Array.isArray(call.arguments)
  ) {
    return null;
  }
  const values = call.arguments as Record<string, unknown>;
  if (typeof values.command !== "string") {
    return null;
  }
  const options = Object.fromEntries(
    Object.entries(values).filter(([key]) => key !== "command"),
  );
  return {
    command: values.command,
    options: Object.keys(options).length > 0 ? options : undefined,
  };
}

export interface SandboxOutput {
  stdout: string;
  stderr: string;
  exitCode?: number;
}

export function parseSandboxOutput(content: string): SandboxOutput | null {
  let value: unknown;
  try {
    value = JSON.parse(content);
  } catch {
    return null;
  }
  if (typeof value !== "object" || value === null || Array.isArray(value)) {
    return null;
  }
  const record = value as Record<string, unknown>;
  if (
    typeof record.stdout !== "string" ||
    typeof record.stderr !== "string" ||
    (record.exit_code !== undefined && typeof record.exit_code !== "number")
  ) {
    return null;
  }
  return {
    stdout: record.stdout,
    stderr: record.stderr,
    exitCode:
      typeof record.exit_code === "number" ? record.exit_code : undefined,
  };
}

export function assertViewModel(value: unknown): PiSessionViewModel {
  if (
    typeof value !== "object" ||
    value === null ||
    !("schemaVersion" in value) ||
    value.schemaVersion !== "1.0" ||
    !("summary" in value) ||
    !("entries" in value) ||
    !Array.isArray(value.entries) ||
    !("defaultHeadId" in value) ||
    typeof value.defaultHeadId !== "string"
  ) {
    throw new Error("The staged session document is incompatible");
  }
  return value as PiSessionViewModel;
}
