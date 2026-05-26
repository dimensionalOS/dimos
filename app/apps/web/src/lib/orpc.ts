import { type AppRouterClient, createORPCClient, RPCLink } from "@robomoo/api/client";

// HTTP oRPC client. In the browser it hits the current origin — the Caddy
// gateway proxies /rpc/* to the server, so it's same-origin (auth cookies are
// first-party). During SSR there's no window, so we reach the server directly
// over the internal network via RPC_INTERNAL_URL.
const link = new RPCLink({
  url:
    typeof window === "undefined"
      ? `${process.env.RPC_INTERNAL_URL ?? "http://localhost:4471"}/rpc`
      : `${window.location.origin}/rpc`,
  fetch(url, options) {
    return fetch(url, { ...options, credentials: "include" });
  },
});

export const rpcClient: AppRouterClient = createORPCClient(link);
