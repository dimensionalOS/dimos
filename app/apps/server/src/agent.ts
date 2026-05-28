import { type Database, settings } from "@robomoo/db";
import { eq } from "drizzle-orm";
import { env } from "./env";

const SINGLETON = "singleton";

export interface AgentConfig {
  url?: string;
  token?: string;
}

// Resolve the dimos agent endpoint at command time: a DB override (set from the
// Console UI) wins over the DIMOS_AGENT_URL env so a different robot can be
// targeted without a redeploy. The bearer token always comes from the env.
export async function resolveAgentConfig(db: Database): Promise<AgentConfig> {
  const [row] = await db
    .select()
    .from(settings)
    .where(eq(settings.id, SINGLETON));
  return {
    url: row?.dimosAgentUrl ?? env.DIMOS_AGENT_URL,
    token: env.DIMOS_AGENT_TOKEN,
  };
}

// Forward a natural-language command to the dimos agent's token-guarded
// /agent-command endpoint (reached over a tunnel). The URL + token never reach
// the browser. dimos publishes the text to its `/human_input` topic, so the
// agent runs it exactly like `dimos agent-send`.
export async function forwardAgentCommand(
  text: string,
  cfg: AgentConfig,
): Promise<void> {
  if (!cfg.url || !cfg.token) {
    throw new Error(
      "agent control not configured — set the robot URL in Console (and DIMOS_AGENT_TOKEN on the server)",
    );
  }
  const res = await fetch(`${cfg.url.replace(/\/$/, "")}/agent-command`, {
    method: "POST",
    headers: {
      "content-type": "application/json",
      authorization: `Bearer ${cfg.token}`,
    },
    body: JSON.stringify({ text }),
  });
  if (!res.ok) {
    throw new Error(`agent endpoint returned ${res.status}`);
  }
}
