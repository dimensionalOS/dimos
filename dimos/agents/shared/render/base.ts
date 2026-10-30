import type { ModelImage } from "./lib/model-image.ts";
import type { PreviewCache, SourceFile } from "./lib/artifacts.ts";
import type { RenderProgress } from "./types.ts";
import type { RenderedView } from "../protocol/media.ts";

type RenderContext = {
  readonly cache: PreviewCache;
  /** The cap on every file a renderer reads, the frames of a clip included. */
  readonly maxSourceBytes: number;
  readonly onProgress: ((progress: RenderProgress) => void) | null;
};

type RenderedPart<K extends RenderedView["kind"]> = {
  readonly view: Extract<RenderedView, { readonly kind: K }>;
  /** What the model receives for this view; the view's previews are the terminal's PNGs. */
  readonly image: ModelImage;
  /** What the model reads after the view's label, path and hash: size, point count, frame times. */
  readonly caption: string;
};

/** One renderable format. image.ts, points.ts and clip.ts each implement it; render.ts tries them in order. */
export interface Renderer<K extends RenderedView["kind"]> {
  /** What a file of this format is called in the notice for a file past the view limit, e.g. "point cloud". */
  readonly noun: string;
  /** Whether the file is in this format at all; `render` validates the rest and throws with the reason. */
  accepts(file: SourceFile): Promise<boolean>;
  /** The size the notice for a file past the view limit states, e.g. `1280x720` or `48,213 points`. */
  measure(file: SourceFile): Promise<string>;
  render(file: SourceFile, label: string, context: RenderContext): Promise<RenderedPart<K>>;
}
