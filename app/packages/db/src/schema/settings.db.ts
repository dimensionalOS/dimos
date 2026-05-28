import { pgTable, text, timestamp } from "drizzle-orm/pg-core";

// Single-row runtime config (id is always "singleton"). Holds the operator-
// overridable dimos agent command URL: when set it takes precedence over the
// DIMOS_AGENT_URL env, so a different robot can be targeted from the Console UI
// without a redeploy. The bearer token stays in the server env (not editable).
export const settings = pgTable("settings", {
  id: text("id").primaryKey(),
  dimosAgentUrl: text("dimos_agent_url"),
  updatedAt: timestamp("updated_at").notNull().defaultNow(),
});
