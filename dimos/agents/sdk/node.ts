import type { TerminalId } from "#dimos/agents/shared/protocol/auth.ts";
import type { WebSocketLike } from "@earendil-works/chord";
import { WebSocket } from "ws";
import { createConnection } from "node:net";
import { randomUUID } from "node:crypto";

/**
 * A fresh socket per attempt, with one terminal identity across reconnects. The path is dialed as given: a `ws+unix:`
 * URL would percent-encode spaces and non-ASCII, split at `:` and reject `#`.
 */
export function unixSocket({
  path,
  label,
  terminal,
}: {
  readonly path: string;
  readonly label: string;
  readonly terminal?: TerminalId;
}): () => WebSocketLike {
  const identity = terminal ?? randomUUID();
  const url = `ws://localhost/?terminal=${identity}&label=${encodeURIComponent(label)}`;
  return () => new WebSocket(url, { createConnection: () => createConnection(path) });
}
