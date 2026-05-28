import {
  boolean,
  doublePrecision,
  jsonb,
  pgTable,
  text,
  timestamp,
} from "drizzle-orm/pg-core";

// A service a marketplace agent offers (a hireable task). `priceUsd` is the
// quoted price; `key` is the stable identifier the hire flow passes to a job.
export interface AgentService {
  key: string;
  name: string;
  desc: string;
  priceUsd: number;
  durationHint?: string;
}

// A hireable robot agent in the robomoo marketplace. The marketplace doubles as
// the ERC-8004 registry surface: once an agent is registered on-chain its
// `agentId` (the Identity Registry NFT id) + bound `agentWallet` are stored here
// so the UI can show a verified identity + read live reputation. Fictional
// "coming soon" agents fill out the registry and carry no real on-chain identity.
export const agents = pgTable("agents", {
  id: text("id").primaryKey(),
  slug: text("slug").notNull().unique(),
  name: text("name").notNull(),
  tagline: text("tagline").notNull(),
  description: text("description").notNull(),
  // Optional object-storage avatar; null agents fall back to `emoji`.
  avatarKey: text("avatar_key"),
  emoji: text("emoji"),
  services: jsonb("services").$type<AgentService[]>().notNull().default([]),
  basePriceUsd: doublePrecision("base_price_usd").notNull().default(0),
  // RoboDoc (the real Go2 scan pipeline) is the one live, hireable agent.
  isReal: boolean("is_real").notNull().default(false),
  // "live" → hireable now; "coming_soon" → listed but disabled.
  status: text("status").notNull().default("coming_soon"),
  // CAIP-2 chain id the agent's identity lives on (Ethereum Sepolia by default).
  chain: text("chain").notNull().default("eip155:11155111"),
  // ERC-8004 Identity Registry NFT id, set after `register()` confirms.
  agentId: text("agent_id"),
  // Wallet bound to the identity via `setAgentWallet` (pays/receives).
  agentWallet: text("agent_wallet"),
  // Tx hash of the on-chain `register()` call (for an etherscan link).
  registerTx: text("register_tx"),
  capabilities: jsonb("capabilities").$type<string[]>().notNull().default([]),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});
