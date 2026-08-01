import {
  Bot,
  CircleEllipsis,
  GitBranch,
  TerminalSquare,
  UserRound,
  Wrench,
} from "lucide-react";
import type { CSSProperties } from "react";

import { sandboxCommand } from "../session";
import type { SessionEntry } from "../types";

const roleIcon = {
  user: UserRound,
  assistant: Bot,
  tool: Wrench,
  custom: CircleEllipsis,
  bash: TerminalSquare,
  state: GitBranch,
};

export function BranchRail({
  entries,
  pathIds,
  selectedId,
  onSelect,
}: {
  entries: SessionEntry[];
  pathIds: ReadonlySet<string>;
  selectedId: string;
  onSelect: (id: string) => void;
}) {
  const commandsByCallId = new Map<string, string>();
  for (const entry of entries) {
    for (const part of entry.parts) {
      if (part.type === "tool_call") {
        const shell = sandboxCommand(part);
        if (shell !== null) {
          commandsByCallId.set(part.callId, shell.command);
        }
      }
    }
  }

  return (
    <nav className="branch-rail" aria-label="Session branches">
      <div className="rail-heading">
        <span>
          <GitBranch aria-hidden />
          Session tree
        </span>
        <span className="count-badge">{entries.length}</span>
      </div>
      <ol className="branch-list">
        {entries.map((entry) => {
          const Icon = roleIcon[entry.role];
          const active = pathIds.has(entry.id);
          const selected = entry.id === selectedId;
          const shellResult = entry.parts.find(
            (part) =>
              part.type === "tool_result" &&
              part.name === "sandbox_exec" &&
              commandsByCallId.has(part.callId),
          );
          const command =
            shellResult?.type === "tool_result"
              ? commandsByCallId.get(shellResult.callId)
              : undefined;
          const title = command ?? entry.title;
          const preview =
            command !== undefined && shellResult?.type === "tool_result"
              ? shellResult.isError
                ? "Failed"
                : "Succeeded"
              : entry.preview || entry.type;
          return (
            <li key={entry.id}>
              <button
                className={`branch-entry ${active ? "branch-active" : ""} ${
                  selected ? "branch-selected" : ""
                }`}
                style={
                  {
                    "--entry-depth": Math.min(entry.depth, 8),
                  } as CSSProperties
                }
                type="button"
                onClick={() => onSelect(entry.id)}
                aria-current={selected ? "true" : undefined}
                title={command ?? (entry.preview || entry.title)}
              >
                <Icon aria-hidden />
                <span>
                  <strong className={command === undefined ? undefined : "branch-command"}>
                    {title}
                  </strong>
                  <small>{preview}</small>
                </span>
              </button>
            </li>
          );
        })}
      </ol>
    </nav>
  );
}
