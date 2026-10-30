import type { Component } from "@earendil-works/pi-tui";
import type { RenderedView } from "../../../shared/protocol/media.ts";

/**
 * The terminal's view of one rendered view kind. image.ts, points.ts and clip.ts implement it, matching shared/render/'s
 * renderers. A panel loads its files itself and calls the `changed` it was built with when it needs a redraw.
 */
export interface ViewPanel<K extends RenderedView["kind"]> extends Component {
  readonly view: Extract<RenderedView, { readonly kind: K }>;
  /** Keys while the panel has focus: arrows rotate a cloud, `+`/`-` zoom; images and clips take none. */
  handleInput?(data: string): void;
  dispose(): void;
}
