// Server-side surface. apps/server mounts the router via the fetch adapter.
export { ORPCError, os } from "@orpc/server";
export { RPCHandler } from "@orpc/server/fetch";
export { type ApiContext, type ApiSession, buildContext } from "./context";
export { type AppRouter, appRouter, groupScans } from "./routers/app.router";
