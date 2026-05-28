CREATE TABLE "maps" (
	"id" text PRIMARY KEY NOT NULL,
	"image_key" text NOT NULL,
	"resolution" double precision NOT NULL,
	"origin_x" double precision NOT NULL,
	"origin_y" double precision NOT NULL,
	"width" integer NOT NULL,
	"height" integer NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
ALTER TABLE "frames" ADD COLUMN "label" text;--> statement-breakpoint
ALTER TABLE "frames" ADD COLUMN "pose_x" double precision;--> statement-breakpoint
ALTER TABLE "frames" ADD COLUMN "pose_y" double precision;