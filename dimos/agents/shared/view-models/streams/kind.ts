/** How a stream is drawn: a picture of one of three kinds, or a rate and a one-line value. */
export type Picture = "image" | "cloud" | "grid";

/** Choose a preview from the selected topic profile and trusted message type. */
export function pictureOf(profile: string, type?: string): Picture | null {
  if (type === "OccupancyGrid") return "grid";
  if (profile === "image.jpeg.balanced.v1") return "image";
  if (profile === "cloud.q16.balanced.v1") return "cloud";
  return null;
}
