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
  // Forward a command to the remote dimos agent (URL + token kept server-side;
  // the URL respects the DB override set in the Console).
  sendAgentCommand: (text: string) => Promise<void>;
  // Server env defaults for the dimos agent, surfaced so settings.get can show
  // the effective endpoint + whether a token is configured (the DB override
  // wins over agentEnvUrl). The token value itself is never exposed.
  agentEnvUrl: string | null;
  agentTokenConfigured: boolean;
}

export function buildContext(ctx: ApiContext): ApiContext {
  return ctx;
}
