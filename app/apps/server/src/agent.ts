import { env } from "./env";

// Forward a natural-language command to the dimos agent's token-guarded
// /agent-command endpoint (reached over a tunnel). The URL + token live here on
// the server and never reach the browser. dimos publishes the text to its
// `/human_input` topic, so the agent runs it exactly like `dimos agent-send`.
export async function sendAgentCommand(text: string): Promise<void> {
  if (!env.DIMOS_AGENT_URL || !env.DIMOS_AGENT_TOKEN) {
    throw new Error("agent control not configured (DIMOS_AGENT_URL / DIMOS_AGENT_TOKEN)");
  }
  const res = await fetch(`${env.DIMOS_AGENT_URL.replace(/\/$/, "")}/agent-command`, {
    method: "POST",
    headers: {
      "content-type": "application/json",
      authorization: `Bearer ${env.DIMOS_AGENT_TOKEN}`,
    },
    body: JSON.stringify({ text }),
  });
  if (!res.ok) {
    throw new Error(`agent endpoint returned ${res.status}`);
  }
}
