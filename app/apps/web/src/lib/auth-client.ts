import { createAuthClient } from "better-auth/react";

// Same-origin in the browser — the gateway proxies /api/auth/* to the server.
export const authClient = createAuthClient({
  baseURL: typeof window === "undefined" ? undefined : window.location.origin,
});

export const { signIn, signUp, signOut, useSession } = authClient;
