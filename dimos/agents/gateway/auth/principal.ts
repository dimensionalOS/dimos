import type { TerminalId } from "#dimos/agents/shared/protocol/auth.ts";
import type { IncomingMessage } from "node:http";

/** A page's access token; each token carries its grant. */
export type PageToken = string & { readonly __brand: "PageToken" };

/**
 * Who a link acts as, stamped once per connection by the accepting side; a client never names its own. Terminals on
 * the 0600 socket are owners; the file mode is the authentication. A page's grant comes from its token.
 */
export type Principal =
  | { readonly kind: "gateway" }
  | { readonly kind: "terminal"; readonly terminal: TerminalId; readonly label: string }
  | {
      readonly kind: "page";
      readonly terminal: TerminalId;
      readonly label: string;
      readonly grant: "owner" | "viewer";
    };

const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i;

const LABEL = /^[A-Za-z0-9_-]{1,32}$/;

/** Stable per client process, not per connection, so a reconnect reaches the same objects. */
export function principalKey(principal: Principal): string {
  switch (principal.kind) {
    case "gateway":
      return "gateway";
    case "terminal":
      return `terminal:${principal.terminal}`;
    case "page":
      return `page:${principal.terminal}:${principal.grant}`;
  }
}

/** `?terminal=<uuid>&label=<word>` on the socket; on the web listener also `token=`; undefined refuses the upgrade. */
export function principalOf(
  request: IncomingMessage,
  listener: "socket" | "web",
  grants: ReadonlyMap<PageToken, "owner" | "viewer">,
): Principal | undefined {
  if (request.url === undefined) return undefined;
  const url = URL.parse(request.url, "http://gateway");
  if (url === null) return undefined;
  const { searchParams } = url;
  const terminal = searchParams.get("terminal");
  const label = searchParams.get("label");
  if (terminal === null || !UUID.test(terminal) || label === null || !LABEL.test(label)) return undefined;
  if (listener === "socket") return { kind: "terminal", terminal: terminal as TerminalId, label };
  const token = searchParams.get("token");
  if (token === null) return undefined;
  const grant = grants.get(token as PageToken);
  if (grant === undefined) return undefined;
  return { kind: "page", terminal: terminal as TerminalId, label, grant };
}

/** The maximum access a principal gets on served objects. */
export function grantOf(principal: Principal): "owner" | "viewer" {
  return principal.kind === "page" ? principal.grant : "owner";
}
