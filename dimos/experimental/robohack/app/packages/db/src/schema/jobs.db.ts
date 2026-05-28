import {
  boolean,
  doublePrecision,
  integer,
  pgTable,
  text,
  timestamp,
} from "drizzle-orm/pg-core";

// One hire of a marketplace agent. A job ties together everything a single
// booking produces: the dispatched command, the scan `run` it generated, the
// `splatId` 3D-tour deliverable, the (mock or x402) payment, and the on-chain
// ERC-8004 reputation feedback. This is what lets results be pinned to a job
// instead of pooled into one global gallery.
export const jobs = pgTable("jobs", {
  id: text("id").primaryKey(),
  agentSlug: text("agent_slug").notNull(),
  // Wallet that hired the agent (from the browser wallet connection).
  requesterAddr: text("requester_addr"),
  // Optional better-auth user, if the hirer was signed in.
  requesterUserId: text("requester_user_id"),
  // Which AgentService.key was hired.
  service: text("service").notNull(),
  // booked | dispatched | scanning | reconstructing | done | failed | cancelled
  status: text("status").notNull().default("booked"),
  priceUsd: doublePrecision("price_usd").notNull().default(0),
  paid: boolean("paid").notNull().default(false),
  // "mock" (default) | "x402"
  paymentMode: text("payment_mode").notNull().default("mock"),
  paymentTx: text("payment_tx"),
  // The natural-language command forwarded to the DimOS agent on dispatch.
  command: text("command"),
  // The scan sweep this job produced — links to frames.run (claim-latest).
  run: text("run"),
  // The 3D-tour deliverable — links to splats.id.
  splatId: text("splat_id"),
  // 1..5 rating the requester gave on completion.
  rating: integer("rating"),
  // Tx hash of the ERC-8004 giveFeedback() call.
  feedbackTx: text("feedback_tx"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  dispatchedAt: timestamp("dispatched_at"),
  completedAt: timestamp("completed_at"),
});
