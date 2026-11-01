/** How a stream is drawn: a picture of one of three kinds, or a rate and a one-line value. */
export type Picture = "image" | "cloud" | "grid";

const PICTURES: ReadonlyMap<string, Picture> = new Map([
  ["jpeg.v1", "image"],
  ["video.webrtc.v1", "image"],
  ["voxels.zlib.v1", "cloud"],
  ["costmap.zlib.v1", "grid"],
]);

/** Choose a preview from the advertised encoding, including channels without Python type metadata. */
export function pictureOf(encoding: string): Picture | null {
  const picture = PICTURES.get(encoding);
  return picture === undefined ? null : picture;
}
