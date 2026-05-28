import { ORPCError, os } from "@orpc/server";
import type { ApiContext } from "./context";

const base = os.$context<ApiContext>();

export const publicProcedure = base;

// Requires a session. After this middleware the handler's `context.session`
// is narrowed to non-null.
export const protectedProcedure = base.use(({ context, next }) => {
  if (!context.session) {
    throw new ORPCError("UNAUTHORIZED", { message: "Sign in required" });
  }
  return next({ context: { session: context.session } });
});
