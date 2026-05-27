CREATE TABLE "trajectories" (
	"id" text PRIMARY KEY NOT NULL,
	"points_key" text NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
ALTER TABLE "frames" ADD COLUMN "embedding" jsonb;