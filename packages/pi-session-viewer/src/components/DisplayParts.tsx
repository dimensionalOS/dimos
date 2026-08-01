/*
 * Presentation patterns adapted from Vercel AI Elements 1.9.0 (Apache-2.0).
 * The implementation is repo-owned and uses immutable Pi view-model types;
 * it contains no AI SDK hooks, composer, actions, or agent runtime.
 */
import { Collapsible } from "@base-ui/react/collapsible";
import {
  BrainCircuit,
  CheckCircle2,
  ChevronDown,
  CircleAlert,
  EyeOff,
  TerminalSquare,
  Wrench,
} from "lucide-react";

import type {
  StatePart,
  ThinkingPart,
  ToolCallPart,
  ToolResultPart,
} from "../types";
import { parseSandboxOutput, sandboxCommand } from "../session";
import { Markdown } from "./Markdown";

function JsonBlock({ value }: { value: unknown }) {
  return (
    <pre className="json-block">
      <code>{JSON.stringify(value, null, 2)}</code>
    </pre>
  );
}

function hasDetails(value: unknown): boolean {
  if (value === undefined) return false;
  if (
    typeof value === "object" &&
    value !== null &&
    !Array.isArray(value) &&
    Object.keys(value).length === 0
  ) {
    return false;
  }
  return true;
}

function TerminalOutput({
  stdout,
  stderr,
  exitCode,
}: {
  stdout: string;
  stderr: string;
  exitCode?: number;
}) {
  return (
    <div className="terminal-output">
      <div className="terminal-bar">
        <span>Terminal output</span>
        {exitCode !== undefined && <span>exit {exitCode}</span>}
      </div>
      <div className="terminal-streams">
        {stdout && (
          <pre className="terminal-stream">
            <code>{stdout}</code>
          </pre>
        )}
        {stderr && (
          <div className="terminal-stderr">
            <span className="terminal-stream-label">stderr</span>
            <pre className="terminal-stream">
              <code>{stderr}</code>
            </pre>
          </div>
        )}
        {!stdout && !stderr && (
          <p className="terminal-empty">Command produced no output.</p>
        )}
      </div>
    </div>
  );
}

export function ReasoningBlock({ part }: { part: ThinkingPart }) {
  return (
    <Collapsible.Root className="reasoning">
      <Collapsible.Trigger className="disclosure-trigger">
        <span className="disclosure-label">
          {part.redacted ? <EyeOff aria-hidden /> : <BrainCircuit aria-hidden />}
          {part.redacted ? "Redacted reasoning" : "Reasoning"}
        </span>
        <ChevronDown className="disclosure-chevron" aria-hidden />
      </Collapsible.Trigger>
      <Collapsible.Panel className="disclosure-panel">
        {part.redacted ? (
          <p className="muted-copy">
            The provider retained an opaque reasoning signature.
          </p>
        ) : (
          <Markdown>{part.text}</Markdown>
        )}
      </Collapsible.Panel>
    </Collapsible.Root>
  );
}

export function ToolExchangeBlock({
  call,
  result,
}: {
  call: ToolCallPart;
  result?: ToolResultPart;
}) {
  const StatusIcon =
    result === undefined ? CircleAlert : result.isError ? CircleAlert : CheckCircle2;
  const status =
    result === undefined ? "No result retained" : result.isError ? "Failed" : "Succeeded";
  const shell = sandboxCommand(call);
  const ToolIcon = shell === null ? Wrench : TerminalSquare;
  const resultDetails = result?.details;
  const terminal =
    shell !== null && result !== undefined
      ? parseSandboxOutput(result.content)
      : null;
  return (
    <Collapsible.Root
      className={`tool-card ${result?.isError ? "tool-card-error" : ""}`}
      defaultOpen
    >
      <Collapsible.Trigger className="disclosure-trigger">
        <span
          className={`disclosure-label ${shell === null ? "" : "tool-command-label"}`}
        >
          <ToolIcon aria-hidden />
          {shell === null ? (
            <span>
              <span className="eyebrow">Tool</span>
              <strong>{call.name}</strong>
            </span>
          ) : (
            <code className="tool-command-summary">{shell.command}</code>
          )}
        </span>
        <span
          className={`tool-status ${
            result?.isError
              ? "tool-status-error"
              : result === undefined
                ? "tool-status-missing"
                : "tool-status-success"
          }`}
        >
          <StatusIcon aria-hidden />
          {status}
        </span>
        <ChevronDown className="disclosure-chevron" aria-hidden />
      </Collapsible.Trigger>
      <Collapsible.Panel className="disclosure-panel">
        {shell === null ? (
          <div className="tool-section">
            <span className="tool-section-label">Input</span>
            <JsonBlock value={call.arguments} />
          </div>
        ) : (
          shell.options !== undefined && (
            <div className="tool-section">
              <span className="tool-section-label">Options</span>
              <JsonBlock value={shell.options} />
            </div>
          )
        )}
        <div className="tool-section">
          <span className="tool-section-label">Output</span>
          {result === undefined ? (
            <p className="muted-copy">No matching result in this session path.</p>
          ) : terminal !== null ? (
            <TerminalOutput {...terminal} />
          ) : result.content ? (
            <pre className="tool-output">
              <code>{result.content}</code>
            </pre>
          ) : (
            <p className="muted-copy">No textual result.</p>
          )}
        </div>
        {hasDetails(resultDetails) && (
          <div className="tool-section">
            <span className="tool-section-label">Details</span>
            <JsonBlock value={resultDetails} />
          </div>
        )}
        <span className="technical-id">Call {call.callId}</span>
      </Collapsible.Panel>
    </Collapsible.Root>
  );
}

export function ToolResultBlock({ part }: { part: ToolResultPart }) {
  const StatusIcon = part.isError ? CircleAlert : CheckCircle2;
  return (
    <Collapsible.Root
      className={`tool-card ${part.isError ? "tool-card-error" : ""}`}
      defaultOpen
    >
      <Collapsible.Trigger className="disclosure-trigger">
        <span className="disclosure-label">
          <StatusIcon aria-hidden />
          <span>
            <span className="eyebrow">Unmatched tool result</span>
            <strong>{part.name}</strong>
          </span>
        </span>
        <span
          className={`tool-status ${
            part.isError ? "tool-status-error" : "tool-status-success"
          }`}
        >
          <StatusIcon aria-hidden />
          {part.isError ? "Failed" : "Succeeded"}
        </span>
        <ChevronDown className="disclosure-chevron" aria-hidden />
      </Collapsible.Trigger>
      <Collapsible.Panel className="disclosure-panel">
        <div className="tool-section">
          <span className="tool-section-label">Output</span>
          {part.content ? (
            <pre className="tool-output">
              <code>{part.content}</code>
            </pre>
          ) : (
            <p className="muted-copy">No textual result.</p>
          )}
        </div>
        {hasDetails(part.details) && (
          <div className="tool-section">
            <span className="tool-section-label">Details</span>
            <JsonBlock value={part.details} />
          </div>
        )}
        <span className="technical-id">Call {part.callId}</span>
      </Collapsible.Panel>
    </Collapsible.Root>
  );
}

export function StateBlock({ part }: { part: StatePart }) {
  return (
    <div className="state-change">
      <TerminalSquare aria-hidden />
      <span>{part.label}</span>
      <strong>{part.value}</strong>
    </div>
  );
}
