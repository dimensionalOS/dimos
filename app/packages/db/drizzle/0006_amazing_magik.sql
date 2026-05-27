CREATE TABLE "frame_analyses" (
	"id" text PRIMARY KEY NOT NULL,
	"frame_id" text NOT NULL,
	"model" text,
	"description" text,
	"summary" text,
	"texts" text,
	"created_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE "frame_analysis_objects" (
	"id" text PRIMARY KEY NOT NULL,
	"analysis_id" text NOT NULL,
	"idx" integer NOT NULL,
	"query" text,
	"label" text,
	"xy_norm_x" double precision,
	"xy_norm_y" double precision,
	"hw_norm_w" double precision,
	"hw_norm_h" double precision,
	"mask_area" integer,
	"crop_key" text,
	"mask_key" text
);
--> statement-breakpoint
ALTER TABLE "frame_analyses" ADD CONSTRAINT "frame_analyses_frame_id_frames_id_fk" FOREIGN KEY ("frame_id") REFERENCES "public"."frames"("id") ON DELETE cascade ON UPDATE no action;--> statement-breakpoint
ALTER TABLE "frame_analysis_objects" ADD CONSTRAINT "frame_analysis_objects_analysis_id_frame_analyses_id_fk" FOREIGN KEY ("analysis_id") REFERENCES "public"."frame_analyses"("id") ON DELETE cascade ON UPDATE no action;