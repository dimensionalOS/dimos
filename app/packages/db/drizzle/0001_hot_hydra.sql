CREATE TABLE "frames" (
	"id" text PRIMARY KEY NOT NULL,
	"image_key" text NOT NULL,
	"note" text,
	"created_at" timestamp DEFAULT now() NOT NULL
);
