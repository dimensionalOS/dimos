import {
  Bot,
  CircleEllipsis,
  ImageOff,
  Info,
  TerminalSquare,
  UserRound,
  Wrench,
} from "lucide-react";

import type { SessionEntry } from "../types";
import {
  ReasoningBlock,
  StateBlock,
  ToolExchangeBlock,
  ToolResultBlock,
} from "./DisplayParts";
import { Markdown } from "./Markdown";

const roleIcon = {
  user: UserRound,
  assistant: Bot,
  tool: Wrench,
  custom: CircleEllipsis,
  bash: TerminalSquare,
  state: Info,
};

export function EntryCard({
  entry,
  selected,
  onInspect,
}: {
  entry: SessionEntry;
  selected: boolean;
  onInspect: (entry: SessionEntry) => void;
}) {
  const RoleIcon = roleIcon[entry.role];
  return (
    <article
      className={`entry-card entry-${entry.role} ${selected ? "entry-selected" : ""}`}
      aria-labelledby={`entry-${entry.id}`}
    >
      <header className="entry-header">
        <span className="entry-role">
          <RoleIcon aria-hidden />
          <span id={`entry-${entry.id}`}>{entry.title}</span>
        </span>
        <button
          className="inspect-button"
          type="button"
          onClick={() => onInspect(entry)}
          aria-label={`Inspect ${entry.title} entry`}
        >
          Details
        </button>
      </header>
      <div className="entry-content">
        {entry.parts.map((part, index) => {
          const key = `${entry.id}-${part.type}-${index}`;
          switch (part.type) {
            case "text":
              return <Markdown key={key}>{part.text}</Markdown>;
            case "thinking":
              return <ReasoningBlock key={key} part={part} />;
            case "tool_call": {
              const next = entry.parts[index + 1];
              const result =
                next?.type === "tool_result" && next.callId === part.callId
                  ? next
                  : undefined;
              return <ToolExchangeBlock key={key} call={part} result={result} />;
            }
            case "tool_result": {
              const previous = entry.parts[index - 1];
              if (
                previous?.type === "tool_call" &&
                previous.callId === part.callId
              ) {
                return null;
              }
              return <ToolResultBlock key={key} part={part} />;
            }
            case "state":
              return <StateBlock key={key} part={part} />;
            case "image":
              return (
                <div className="image-omitted" key={key}>
                  <ImageOff aria-hidden />
                  Embedded image omitted from review
                  {part.mimeType ? ` (${part.mimeType})` : ""}
                </div>
              );
          }
        })}
      </div>
      <footer className="entry-footer">
        <time dateTime={entry.timestamp}>{entry.timestamp}</time>
        {entry.model && <span>{entry.model}</span>}
        {entry.usage?.totalTokens !== undefined && (
          <span>{entry.usage.totalTokens.toLocaleString()} tokens</span>
        )}
        {entry.stopReason && <span>{entry.stopReason}</span>}
      </footer>
    </article>
  );
}
