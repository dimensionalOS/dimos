import { agents, type Database } from "@robomoo/db";
import { eq } from "drizzle-orm";

// The ERC-8004 `agentURI` document. `register(agentURI)` on the Identity
// Registry points at this URL; it describes the agent (name, capabilities,
// services, where to hire it) so anyone resolving the on-chain identity can
// discover what the agent is and does.
export async function handleAgentCard(
  db: Database,
  slug: string,
  baseUrl: string,
): Promise<Response> {
  const [row] = await db
    .select()
    .from(agents)
    .where(eq(agents.slug, slug))
    .limit(1);
  if (!row) return new Response("unknown agent", { status: 404 });

  const card = {
    name: row.name,
    description: row.description,
    tagline: row.tagline,
    image: row.emoji ?? null,
    type: "robot-agent",
    provider: "robomoo",
    chain: row.chain,
    capabilities: row.capabilities,
    services: row.services,
    url: `${baseUrl.replace(/\/$/, "")}/agents/${row.slug}`,
    cardUrl: `${baseUrl.replace(/\/$/, "")}/api/agents/${row.slug}/card.json`,
    ...(row.agentId ? { agentId: row.agentId } : {}),
    ...(row.agentWallet ? { agentWallet: row.agentWallet } : {}),
  };
  return Response.json(card);
}
