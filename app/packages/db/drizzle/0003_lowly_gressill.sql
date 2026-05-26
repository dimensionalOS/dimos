CREATE TABLE "splats" (
	"id" text PRIMARY KEY NOT NULL,
	"splat_key" text NOT NULL,
	"name" text,
	"format" text NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL
);
