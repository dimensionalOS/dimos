import type { Static } from "typebox";
import { Type } from "typebox";

export type Sha256 = string & { readonly __brand: "Sha256" };

export const Sha256Schema = Type.Unsafe<Sha256>(Type.String({ pattern: "^[0-9a-f]{64}$" }));

/** A file and the hash it had when rendered. Sources are never modified. */
const FileRefSchema = Type.Object({ path: Type.String(), sha256: Sha256Schema });

export type FileRef = Static<typeof FileRefSchema>;

const ClipFrameSchema = Type.Object({
  /** The frame's zero-based index in the frame index. */
  index: Type.Integer({ minimum: 0 }),
  timestamp: Type.Number(),
  source: FileRefSchema,
  png: FileRefSchema,
});

/** About 10 Hz, at most 120 frames over at most 60 seconds, at source timing. The terminal loops `frames`; never the GIF. */
const ClipSchema = Type.Object({
  frames: Type.Array(ClipFrameSchema, { maxItems: 120 }),
  gif: Type.Union([FileRefSchema, Type.Null()]),
});

const RenderedViewSchema = Type.Union([
  Type.Object({ kind: Type.Literal("image"), label: Type.String(), source: FileRefSchema, preview: FileRefSchema }),
  Type.Object({
    kind: Type.Literal("points"),
    label: Type.String(),
    /** The cloud JSON, which the terminal's spatial view reads to rotate it. */
    source: FileRefSchema,
    /** The painted projection the model receives. */
    preview: FileRefSchema,
    points: Type.Integer({ minimum: 1 }),
    frame: Type.Union([Type.String(), Type.Null()]),
  }),
  Type.Object({
    kind: Type.Literal("clip"),
    label: Type.String(),
    /** The frame index. */
    source: FileRefSchema,
    /** The timestamped contact sheet the model receives. */
    sheet: FileRefSchema,
    clip: ClipSchema,
  }),
]);

export type RenderedView = Static<typeof RenderedViewSchema>;

/**
 * The views, and so the images the model receives, of one tool result or input. Every result stays in context until
 * compaction, so the cap bounds what one result costs on every later request; six fit the terminal's gallery.
 */
export const MAX_VIEWS = 6;

/**
 * What a tool card or an event line shows: at most MAX_VIEWS views, renderable files past the limit listed by path,
 * and files that failed. Stored in tool result details and input entries; the type is derived from the schema.
 */
export const RenderDetailsSchema = Type.Object({
  title: Type.String(),
  views: Type.Array(RenderedViewSchema, { maxItems: MAX_VIEWS }),
  listed: Type.Array(Type.String()),
  failed: Type.Array(Type.Object({ path: Type.String(), error: Type.String() })),
});

export type RenderDetails = Static<typeof RenderDetailsSchema>;
