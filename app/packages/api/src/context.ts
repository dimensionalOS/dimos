import type { Database } from "@robomoo/db";

export interface ApiSession {
  user: { id: string; name: string };
}

// Built fresh per request on the server. `presignGet` turns an S3 object key
// into a time-limited GET URL — injected so the api package never imports the
// storage client directly.
export interface ApiContext {
  db: Database;
  session: ApiSession | null;
  presignGet: (key: string) => Promise<string>;
}

export function buildContext(ctx: ApiContext): ApiContext {
  return ctx;
}
