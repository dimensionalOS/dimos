import type { EntryId } from "@earendil-works/pi-durable";
import type { Static } from "typebox";
import { Type } from "typebox";

/**
 * Image bytes never ride a transcript. The gateway's projection empties every image block's `data` and keeps its
 * `mimeType`; a renderer fetches the few it draws with `conversation.image({ at })`, which answers the stored block in
 * its own format.
 */
export const ImageAddressSchema = Type.Object({
  entry: Type.Unsafe<EntryId>(Type.Integer({ minimum: 1 })),
  /** The message's index in the entry's `model`. */
  message: Type.Integer({ minimum: 0 }),
  /** The block's index in that message's content. */
  block: Type.Integer({ minimum: 0 }),
});

/** Where one image block sits in its conversation. */
export type ImageAddress = Static<typeof ImageAddressSchema>;

/** An image a transcript item refers to: what a renderer needs to draw a placeholder and fetch the bytes. */
export type ImageRef = { readonly at: ImageAddress; readonly mimeType: string };
