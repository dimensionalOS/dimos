CREATE TABLE "settings" (
	"id" text PRIMARY KEY NOT NULL,
	"dimos_agent_url" text,
	"updated_at" timestamp DEFAULT now() NOT NULL
);
