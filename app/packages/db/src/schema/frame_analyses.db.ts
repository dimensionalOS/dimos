import {
  doublePrecision,
  integer,
  pgTable,
  text,
  timestamp,
} from "drizzle-orm/pg-core";
import { frames } from "./frames.db";

// Per-frame VLM analysis run (Gemma scene reasoning + Falcon-Perception
// detections). A single frame can have many analyses — every trigger (auto on
// ingest, manual "Analyze room", per-image re-run) appends a new row so prompt
// tweaks stay debuggable. The web defaults to displaying the newest.
//
// `description` is the Gemma scene paragraph, `summary` is the final Gemma
// summary that combines scene + Falcon counts, `texts` is a JSON array of
// auxiliary strings (raw initial JSON, falcon `raw` lines, anything else worth
// keeping). `model` records the Gemma snapshot id used so we can compare runs.
export const frameAnalyses = pgTable("frame_analyses", {
  id: text("id").primaryKey(),
  frameId: text("frame_id")
    .notNull()
    .references(() => frames.id, { onDelete: "cascade" }),
  model: text("model"),
  description: text("description"),
  summary: text("summary"),
  texts: text("texts"), // JSON string — Postgres jsonb works too, kept text for write-shape parity
  createdAt: timestamp("created_at").notNull().defaultNow(),
});

// One row per Falcon detection inside an analysis. `xyNorm` / `hwNorm` are the
// 0..1 normalized center + size of the bounding box (multiply by image width/
// height to render). `maskArea` is the boolean pixel count of the segmentation
// mask. `cropKey` / `maskKey` are S3 object keys: the cropped object PNG and a
// single-object alpha mask PNG, uploaded by mlxvlm alongside the JSON callback.
// Both may be null when the detection had no usable mask/crop.
export const frameAnalysisObjects = pgTable("frame_analysis_objects", {
  id: text("id").primaryKey(),
  analysisId: text("analysis_id")
    .notNull()
    .references(() => frameAnalyses.id, { onDelete: "cascade" }),
  idx: integer("idx").notNull(),
  query: text("query"),
  label: text("label"),
  xyNormX: doublePrecision("xy_norm_x"),
  xyNormY: doublePrecision("xy_norm_y"),
  hwNormW: doublePrecision("hw_norm_w"),
  hwNormH: doublePrecision("hw_norm_h"),
  maskArea: integer("mask_area"),
  cropKey: text("crop_key"),
  maskKey: text("mask_key"),
});
