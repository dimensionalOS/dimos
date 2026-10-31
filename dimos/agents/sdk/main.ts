import type { Host, WebSocketLike } from "@earendil-works/chord";
import type { EpochMs } from "../shared/protocol/time.ts";
import type { Refusal } from "../shared/protocol/outcome.ts";
import type { Live } from "../shared/protocol/status.ts";
import { createHost, webSocketLink } from "@earendil-works/chord";
import { epochMs } from "../shared/protocol/time.ts";
import { RefusalError } from "./failures.ts";
import { Status } from "../shared/protocol/status.ts";
import { until } from "./until.ts";

type GatewayPrincipal = { readonly kind: "gateway" };

export type Connection =
  | { readonly state: "connecting"; readonly attempt: number; readonly since: EpochMs }
  | { readonly state: "live"; readonly since: EpochMs }
  | { readonly state: "reconnecting"; readonly attempt: number; readonly since: EpochMs; readonly lastError: string }
  | { readonly state: "refused"; readonly refusal: Refusal };

export type ConnectOptions = {
  readonly backoffMs?: { readonly min: number; readonly max: number };
  readonly readyTimeoutMs?: number;
  readonly now?: () => EpochMs;
};

export type Client = {
  readonly host: Host<GatewayPrincipal>;
  readonly connection: Live<Connection>;
  /** Rejects when the link will not return, including an explicit close. */
  readonly ended: Promise<never>;
  whenLive(): Promise<void>;
  close(): Promise<void>;
};

export async function connect(open: () => WebSocketLike, options: ConnectOptions = {}): Promise<Client> {
  const now = options.now ?? (() => epochMs(Date.now()));
  const backoff = options.backoffMs ?? { min: 250, max: 5_000 };
  const lifetime = new AbortController();
  const ended = new Promise<never>((_, reject) => {
    lifetime.signal.addEventListener("abort", () => reject(lifetime.signal.reason), { once: true });
  });
  ended.catch(() => {});
  let droppedAt = now();
  let everLive = false;
  let attempt = 1;
  let socket: WebSocketLike | null = null;
  let timer: ReturnType<typeof setTimeout> | undefined;
  const connection = new Status<Connection>({ state: "connecting", attempt, since: droppedAt }, (error) => {
    throw error;
  });
  const host = createHost<GatewayPrincipal>({
    name: "client",
    self: { kind: "gateway" },
    kind: "client",
    readyTimeout: options.readyTimeoutMs ?? 5_000,
  });

  const finish = (refusal: Refusal): void => {
    if (lifetime.signal.aborted) return;
    lifetime.abort(new RefusalError(refusal));
    clearTimeout(timer);
    socket?.close();
    connection.set({ state: "refused", refusal });
  };
  const whenLive = async (): Promise<void> => {
    lifetime.signal.throwIfAborted();
    await until(connection, (value) => (value.state === "live" ? true : undefined), lifetime.signal);
  };
  const close = async (): Promise<void> => {
    finish({ code: "unknown_outcome", since: now() });
    await host.close();
  };
  const retry = (lastError: string): void => {
    if (lifetime.signal.aborted) return;
    if (connection.value.state === "live") droppedAt = now();
    attempt++;
    connection.set(
      everLive
        ? { state: "reconnecting", attempt, since: droppedAt, lastError }
        : { state: "connecting", attempt, since: droppedAt },
    );
    timer = setTimeout(dial, Math.min(backoff.max, backoff.min * 2 ** (attempt - 2)));
  };
  const dial = (): void => {
    if (lifetime.signal.aborted) return;
    let current: WebSocketLike;
    try {
      current = open();
    } catch (error) {
      retry(String(error));
      return;
    }
    socket = current;
    let settled = false;
    current.addEventListener("open", () => {
      if (settled) return;
      settled = true;
      if (lifetime.signal.aborted) {
        current.close();
        return;
      }
      const link = webSocketLink(current);
      link.onClose((reason) => (reason === "stopping" ? finish({ code: "stopping" }) : retry(reason)));
      host.connect(link, {
        principal: { kind: "gateway" },
        onProtocolMismatch: () => finish({ code: "other_build", gatewayBuild: null }),
      });
      everLive = true;
      attempt = 1;
      connection.set({ state: "live", since: now() });
    });
    const failed = (): void => {
      if (settled) return;
      settled = true;
      current.close();
      retry("the socket closed before it opened");
    };
    current.addEventListener("error", failed);
    current.addEventListener("close", failed);
  };
  dial();
  try {
    await whenLive();
    return { host, connection, ended, whenLive, close };
  } catch (error) {
    await close();
    throw error;
  }
}
