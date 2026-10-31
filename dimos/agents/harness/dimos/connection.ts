import type { CallToolResult, McpFetch } from "@earendil-works/pi-mcp";
import type { ConversationId, JsonObject } from "@earendil-works/pi-durable";
import type { HarnessEvent } from "../../shared/protocol/runtime/event.ts";
import type { ImageContent, TextContent } from "@earendil-works/pi-ai";
import type { LinkStatus } from "../../shared/protocol/gateway/link.ts";
import type { Live } from "../../shared/protocol/status.ts";
import {
  McpClient,
  McpConnectionClosedError,
  McpHttpError,
  McpSessionExpiredError,
  StreamableHttpTransport,
  toLlmContent,
} from "@earendil-works/pi-mcp";
import { Status } from "../../shared/protocol/status.ts";
import { WireError } from "./wire/base.ts";
import { describeLink } from "../../shared/protocol/gateway/link.ts";
import { epochMs } from "../../shared/protocol/time.ts";
import { harnessEventDecoder } from "./wire/event.ts";

/** A tool McpServer advertises, as `tools/list` describes it. */
export type RemoteTool = {
  readonly name: string;
  readonly description: string;
  readonly inputSchema: JsonObject;
};

type CallOptions = {
  /** The conversation the call acts for; sent as `_meta["dimos/conversation"]`. Null for the gateway's own calls. */
  readonly caller: ConversationId | null;
  readonly timeoutMs: number;
  readonly signal?: AbortSignal;
};

/** A `tools/call` result: model content from `toLlmContent`, and `structuredContent`, which `toLlmContent` drops. */
type ToolCallOutcome = {
  readonly content: readonly (TextContent | ImageContent)[];
  readonly structured: unknown;
  readonly isError: boolean;
};

/** Thrown at once by a call while the link is not up, instead of waiting for a timeout. */
export class DimosUnavailable extends Error {
  readonly status: LinkStatus;

  constructor(status: LinkStatus) {
    super(`DimOS is not reachable: ${describeLink(status)}`);
    this.status = status;
  }
}

type ConnectionOptions = {
  readonly url: URL;
  /** How long the MCP handshake and opening the event stream may take; a server that accepts the connection and never answers otherwise holds an attempt for undici's 300 s. */
  readonly connectTimeoutMs: number;
  readonly maxBackoffMs: number;
  /** Link transitions, and errors that do not fail a call: undecodable events, late responses. */
  readonly report: (error: unknown) => void;
};

/** The client an attempt is connecting, and when the attempt is over; `close()` aborts the first and waits for the second. */
type Attempt = { readonly client: McpClient; readonly settled: Promise<void> };

function isCallToolResult(value: unknown): value is CallToolResult {
  return typeof value === "object" && value !== null && Array.isArray((value as { content?: unknown }).content);
}

/** An HTTP failure the transport will not retry on its own, so the client is dead until replaced. */
function endsStream(error: Error): boolean {
  if (error instanceof McpSessionExpiredError) return true;
  return (
    error instanceof McpHttpError &&
    error.status < 500 &&
    error.status !== 408 &&
    error.status !== 429 &&
    error.status !== 405
  );
}

/** A call failure that means the client no longer works, not that the call was wrong. */
function breaksLink(error: unknown): boolean {
  return (
    error instanceof McpConnectionClosedError || error instanceof McpSessionExpiredError || error instanceof TypeError
  );
}

/**
 * The transport reopens a dropped stream by itself, so this is how the connection learns DimOS stopped or restarted.
 * `opened` settles with the first GET: resolved once its stream is open, rejected if it could not be opened.
 */
function watchEventStream(ended: (error: unknown) => void): { fetch: McpFetch; opened: Promise<void> } {
  let resolveOpened: () => void = () => {};
  let rejectOpened: (error: unknown) => void = () => {};
  const opened = new Promise<void>((resolve, reject) => {
    resolveOpened = resolve;
    rejectOpened = reject;
  });
  opened.catch(() => undefined);
  const lost = (error: unknown): void => {
    rejectOpened(error);
    ended(error);
  };
  const watched: McpFetch = async (input, init) => {
    if (init?.method !== "GET") return fetch(input, init);
    let response: Response;
    try {
      response = await fetch(input, init);
    } catch (error) {
      lost(error);
      throw error;
    }
    if (!response.ok || response.body === null) {
      rejectOpened(new Error(`DimOS did not open the event stream: HTTP ${response.status}`));
      return response;
    }
    resolveOpened();
    const relay = new TransformStream<Uint8Array, Uint8Array>();
    response.body.pipeTo(relay.writable).then(() => lost(new Error("DimOS closed the event stream")), lost);
    return new Response(relay.readable, {
      status: response.status,
      statusText: response.statusText,
      headers: response.headers,
    });
  };
  return { fetch: watched, opened };
}

/**
 * Owns one pi-mcp client at a time and replaces it when it stops working. A client is replaced whole, never
 * reconnected, and a failure reported for a client that is no longer current changes nothing: its replacement is
 * already on its way.
 */
export class DimosConnection {
  /** The client calls go through; null while connecting or down. */
  private current: McpClient | null = null;
  /** The attempt that brought `current` up; a link that drops soon after continues its backoff from here. */
  private currentAttempt = 0;
  private readonly link: Status<LinkStatus>;
  readonly status: Live<LinkStatus>;
  private readonly options: ConnectionOptions;
  private readonly eventListeners = new Set<(event: HarnessEvent) => void>();
  private readonly harnessStatusListeners = new Set<() => void>();
  private attemptInFlight: Attempt | null = null;
  private retryTimer: NodeJS.Timeout | null = null;
  private closed = false;
  private markClosed: () => void = () => {};
  /** Settles when `close()` is called, so an attempt mid-handshake returns at once instead of waiting out its deadline. */
  private readonly closing = new Promise<"closed">((resolve) => {
    this.markClosed = () => resolve("closed");
  });
  /** What the log last said about the link; only a change is reported, never a retry. */
  private reported: "up" | "down" | null = null;

  constructor(options: ConnectionOptions) {
    this.options = options;
    this.link = new Status<LinkStatus>({ state: "connecting", attempt: 0, lastError: null }, options.report);
    this.status = this.link;
  }

  /**
   * Every `dimos/harness_status` notification, which HarnessRuntime sends after a module op, a subscription change and
   * the coordinator's `on_system_modules`. It carries no payload the gateway trusts; the gateway re-reads inventory.
   */
  onHarnessStatus(listener: () => void): () => void {
    this.harnessStatusListeners.add(listener);
    return () => this.harnessStatusListeners.delete(listener);
  }

  /** Every decoded `dimos/harness_event`, from the GET stream or a POST response stream. */
  onEvent(listener: (event: HarnessEvent) => void): () => void {
    this.eventListeners.add(listener);
    return () => this.eventListeners.delete(listener);
  }

  start(): void {
    this.connect(0, null);
  }

  /** Runtime readiness makes a pending reconnect worth trying immediately. */
  retryNow(): void {
    const status = this.link.value;
    if (this.closed || status.state !== "down" || this.attemptInFlight !== null) return;
    if (this.retryTimer !== null) clearTimeout(this.retryTimer);
    this.retryTimer = null;
    this.connect(0, status.error);
  }

  /**
   * `tools/call` with `_meta` through `client.request`, since `callTool()` cannot send `_meta`. A failure that breaks
   * the link is reported as DimOS unreachable, which it is from that moment: the client is being replaced.
   */
  async callTool(name: string, args: JsonObject, options: CallOptions): Promise<ToolCallOutcome> {
    const client = await this.clientWithin(options.timeoutMs, options.signal);
    try {
      const raw = await client.request<unknown>(
        "tools/call",
        { name, arguments: args, _meta: options.caller === null ? {} : { "dimos/conversation": options.caller } },
        // No progress token: McpServer sends every progress frame to every open stream, so a token cannot name a call.
        { timeoutMs: options.timeoutMs, signal: options.signal },
      );
      if (!isCallToolResult(raw)) throw new WireError("tools/call result", ["content is not an array"]);
      return { content: toLlmContent(raw), structured: raw.structuredContent, isError: raw.isError === true };
    } catch (error) {
      if (!breaksLink(error)) throw error;
      this.rebuild(client, error);
      throw new DimosUnavailable(this.link.value);
    }
  }

  async listTools(signal?: AbortSignal): Promise<readonly RemoteTool[]> {
    const client = await this.clientWithin(30_000, signal);
    try {
      const tools = await client.listTools({ signal });
      return tools.map((tool) => ({
        name: tool.name,
        description: tool.description ?? "",
        inputSchema: tool.inputSchema as JsonObject,
      }));
    } catch (error) {
      if (!breaksLink(error)) throw error;
      this.rebuild(client, error);
      throw new DimosUnavailable(this.link.value);
    }
  }

  async close(): Promise<void> {
    this.closed = true;
    this.markClosed();
    if (this.retryTimer !== null) clearTimeout(this.retryTimer);
    this.retryTimer = null;
    const client = this.current;
    this.current = null;
    if (client !== null) await client.close();
    const attempt = this.attemptInFlight;
    if (attempt !== null) {
      await attempt.client.close().catch(() => undefined);
      await attempt.settled;
    }
  }

  /** The client once the link is up; waits out an attempt in flight, within `timeoutMs`; throws at once while down. */
  private async clientWithin(timeoutMs: number, signal: AbortSignal | undefined): Promise<McpClient> {
    if (this.current !== null) return this.current;
    if (this.closed || this.link.value.state !== "connecting") throw new DimosUnavailable(this.link.value);
    await new Promise<void>((resolve) => {
      const timer = setTimeout(finish, timeoutMs);
      const unsubscribe = this.link.subscribe((status) => {
        if (status.state !== "connecting") finish();
      });
      signal?.addEventListener("abort", finish, { once: true });
      function finish(): void {
        clearTimeout(timer);
        unsubscribe();
        signal?.removeEventListener("abort", finish);
        resolve();
      }
    });
    const client = this.clientNow();
    if (client === null) throw new DimosUnavailable(this.link.value);
    return client;
  }

  /** Read after an await, where the compiler still holds the narrowing from before it. */
  private clientNow(): McpClient | null {
    return this.current;
  }

  private setStatus(status: LinkStatus): void {
    this.announce(status);
    this.link.set(status);
  }

  private announce(status: LinkStatus): void {
    switch (status.state) {
      case "up":
        if (this.reported !== "up") this.options.report(`DimOS link up: ${status.server}`);
        this.reported = "up";
        return;
      case "connecting":
        if (this.reported !== "up") return;
        this.options.report(`DimOS link down: ${status.lastError}`);
        this.reported = "down";
        return;
      case "down":
        if (this.reported !== "down") this.options.report(`DimOS link down: ${status.error}`);
        this.reported = "down";
        return;
    }
  }

  private connect(attempt: number, lastError: unknown): void {
    this.setStatus({ state: "connecting", attempt, lastError: lastError === null ? null : String(lastError) });
    const client = new McpClient({ name: "dimos-agent", version: "1" });
    this.attemptInFlight = {
      client,
      settled: this.attempt(client, attempt).finally(() => {
        this.attemptInFlight = null;
      }),
    };
  }

  private async attempt(client: McpClient, attempt: number): Promise<void> {
    const watcher = watchEventStream((error) => this.rebuild(client, error));
    const transport = new StreamableHttpTransport({
      url: this.options.url,
      fetch: watcher.fetch,
      reconnect: { maxRetries: Number.POSITIVE_INFINITY },
    });
    client.onNotification("dimos/harness_status", () => {
      if (this.current !== client) return;
      for (const listener of this.harnessStatusListeners) {
        try {
          listener();
        } catch (error) {
          this.options.report(error);
        }
      }
    });
    client.onNotification("dimos/harness_event", (params) => {
      if (this.current !== client) return;
      let event: HarnessEvent;
      try {
        event = harnessEventDecoder.decode(params);
      } catch (error) {
        this.options.report(error);
        return;
      }
      for (const listener of this.eventListeners) {
        try {
          listener(event);
        } catch (error) {
          this.options.report(error);
        }
      }
    });
    client.onError((error) => {
      if (this.current !== client) return;
      if (endsStream(error)) this.rebuild(client, error);
      else this.options.report(error);
    });
    const ready = client.connect(transport).then(async (initialized) => {
      await watcher.opened;
      return initialized;
    });
    let deadline: NodeJS.Timeout | undefined;
    const expiry = new Promise<never>((_, reject) => {
      deadline = setTimeout(
        () =>
          reject(
            new Error(`the MCP handshake and event stream did not open within ${this.options.connectTimeoutMs} ms`),
          ),
        this.options.connectTimeoutMs,
      );
    });
    try {
      const initialized = await Promise.race([ready, expiry, this.closing]);
      if (this.closed || initialized === "closed") {
        await client.close();
        return;
      }
      this.current = client;
      this.currentAttempt = attempt;
      this.setStatus({ state: "up", since: epochMs(Date.now()), server: initialized.serverInfo.name });
    } catch (error) {
      await client.close().catch(() => undefined);
      if (this.closed) return;
      this.retryAfterBackoff(attempt, error);
    } finally {
      clearTimeout(deadline);
    }
  }

  private retryAfterBackoff(attempt: number, error: unknown): void {
    const delay = Math.min(1_000 * 2 ** attempt, this.options.maxBackoffMs);
    this.setStatus({ state: "down", error: String(error), retryAt: epochMs(Date.now() + delay) });
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.connect(attempt + 1, error);
    }, delay);
  }

  /**
   * Drops `client` if it is still current and connects again: at once from attempt 0 if the link stayed up at least
   * `maxBackoffMs`, otherwise after the backoff, as a failed attempt.
   */
  private rebuild(client: McpClient, error: unknown): void {
    const status = this.link.value;
    if (this.closed || this.current !== client || status.state !== "up") return;
    this.current = null;
    client.close().catch(() => undefined);
    if (Date.now() - status.since < this.options.maxBackoffMs) this.retryAfterBackoff(this.currentAttempt, error);
    else this.connect(0, error);
  }
}
