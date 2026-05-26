// Server-side surface. apps/server mounts the router via the fetch adapter.
export { ORPCError, os } from "@orpc/server";
export { RPCHandler } from "@orpc/server/fetch";
export { appRouter, type AppRouter } from "./routers/app.router";
export { buildContext, type ApiContext, type ApiSession } from "./context";
