import type { Database } from "@robomoo/db";

export interface ApiSession {
  user: { id: string; name: string };
}

// Built fresh per request on the server. `presignGet` turns an S3 object key
// into a time-limited GET URL; `readObject` reads an object's bytes back (for
// small artifacts we inline as same-origin data URIs). Injected so the api
// package never imports the storage client directly.
export interface ApiContext {
  db: Database;
  session: ApiSession | null;
  presignGet: (key: string, expiresIn?: number) => Promise<string>;
  readObject: (key: string) => Promise<ArrayBuffer>;
  // Forward a command to the remote dimos agent (URL + token kept server-side).
  sendAgentCommand: (text: string) => Promise<void>;
}

export function buildContext(ctx: ApiContext): ApiContext {
  return ctx;
}
