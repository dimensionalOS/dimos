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

  const base = baseUrl.replace(/\/$/, "");
  const card = {
    name: row.name,
    description: row.description,
    tagline: row.tagline,
    // Stable identifier the on-chain `image` resolves through — backed by the
    // agent's uploaded avatar (object storage) once set, 404 until then. The
    // card is served live, so the photo can be added without re-registering.
    image: `${base}/api/agents/${row.slug}/avatar`,
    emoji: row.emoji ?? null,
    type: "robot-agent",
    provider: "robomoo",
    chain: row.chain,
    capabilities: row.capabilities,
    services: row.services,
    url: `${base}/agents/${row.slug}`,
    cardUrl: `${base}/api/agents/${row.slug}/card.json`,
    ...(row.agentId ? { agentId: row.agentId } : {}),
    ...(row.agentWallet ? { agentWallet: row.agentWallet } : {}),
  };
  return Response.json(card);
}

// Stable avatar endpoint referenced by the agent card's `image`. Redirects to a
// presigned URL of the agent's uploaded avatar; returns 404 until one is set.
export async function handleAgentAvatar(
  db: Database,
  slug: string,
  presignGet: (key: string, expiresIn?: number) => Promise<string>,
): Promise<Response> {
  const [row] = await db
    .select({ avatarKey: agents.avatarKey })
    .from(agents)
    .where(eq(agents.slug, slug))
    .limit(1);
  if (!row?.avatarKey) return new Response("no avatar", { status: 404 });
  const url = await presignGet(row.avatarKey, 6 * 60 * 60);
  return Response.redirect(url, 302);
}
