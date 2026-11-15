import sharp from "sharp";
import type { Renderer } from "#dimos/agents/shared/render/base.ts";
import { Compile } from "typebox/compile";
import { Type } from "typebox";
import { firstProblem } from "#dimos/agents/shared/render/lib/schema.ts";
import { modelImage } from "#dimos/agents/shared/render/lib/model-image.ts";

type Vec3 = readonly [number, number, number];

/** Point-cloud JSON: `{points: [[x,y,z]], colors?, frame?, timestamp?, selectedIndices?}`, 1 to 1,000,000 finite points. */
export type Cloud = {
  readonly points: readonly Vec3[];
  /** 0-255, one per point. */
  readonly colors: readonly Vec3[] | null;
  readonly frame: string | null;
  readonly timestamp: number | null;
  readonly selectedIndices: readonly number[] | null;
};

/** Bounds that fit one or more clouds, seen from `yaw` at `zoom`. Clouds painted with one camera share scale and height. */
export type Camera = {
  readonly center: Vec3;
  /** The longest side of the bounds; the projection fits it to the image. */
  readonly extent: number;
  /** The lowest z and the z range, which set height brightness. */
  readonly floor: number;
  readonly height: number;
  /** Radians about z. */
  readonly yaw: number;
  /** 1 fits the bounds. */
  readonly zoom: number;
};

type Size = { readonly width: number; readonly height: number };

const Channel = Type.Integer({ minimum: 0, maximum: 255 });

const CloudJson = Compile(
  Type.Object({
    points: Type.Array(Type.Tuple([Type.Number(), Type.Number(), Type.Number()]), { minItems: 1, maxItems: 1_000_000 }),
    colors: Type.Optional(Type.Array(Type.Tuple([Channel, Channel, Channel]))),
    frame: Type.Optional(Type.String()),
    timestamp: Type.Optional(Type.Number()),
    selectedIndices: Type.Optional(Type.Array(Type.Integer({ minimum: 0 }))),
  }),
);

const CloudShape = Compile(Type.Object({ points: Type.Array(Type.Unknown()) }));

/** Throws with the first problem when the JSON is not a valid cloud. Moved from dimcode points.ts. */
export function parseCloud(json: unknown): Cloud {
  if (!CloudJson.Check(json)) throw new Error(`not a point cloud: ${firstProblem(CloudJson, json)}`);
  const { points, colors = null, frame = null, timestamp = null, selectedIndices = null } = json;
  if (colors !== null && colors.length !== points.length) {
    throw new Error(`${colors.length} colors for ${points.length} points`);
  }
  const outside = selectedIndices?.find((index) => index >= points.length);
  if (outside !== undefined) throw new Error(`selected index ${outside} is outside the ${points.length} points`);
  return { points, colors, frame, timestamp, selectedIndices };
}

/** Fits the bounds of every cloud given, looking from the default angle. */
export function fitCamera(...clouds: readonly Cloud[]): Camera {
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  for (const cloud of clouds) {
    for (const point of cloud.points) {
      for (let axis = 0; axis < 3; axis++) {
        low[axis] = Math.min(low[axis], point[axis]);
        high[axis] = Math.max(high[axis], point[axis]);
      }
    }
  }
  return {
    center: [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, (low[2] + high[2]) / 2],
    extent: Math.max(high[0] - low[0], high[1] - low[1], high[2] - low[2], 0.001),
    floor: low[2],
    height: Math.max(high[2] - low[2], 0.001),
    yaw: -0.58,
    zoom: 1,
  };
}

/** Where `point` lands in an image of `size`; the spatial view's Braille fallback projects with it too. */
export function projectPoint(point: Vec3, camera: Camera, size: Size): [number, number] {
  const x = point[0] - camera.center[0];
  const y = point[1] - camera.center[1];
  const z = point[2] - camera.center[2];
  const cos = Math.cos(camera.yaw);
  const sin = Math.sin(camera.yaw);
  // A cube of side `extent` projects up to (1 + √½) * extent tall at any yaw; fitting that keeps one scale as the view
  // rotates.
  const scale = (Math.min(size.width / 1.65, size.height / (1 + Math.SQRT1_2)) / camera.extent) * camera.zoom;
  return [
    size.width / 2 + (x * cos - y * sin) * scale,
    size.height / 2 + (x * sin + y * cos) * scale * 0.5 - z * scale,
  ];
}

/**
 * A PNG of the cloud from `camera`, decimated to at most 20,000 points with at most 20,000 selected ones drawn over
 * them, projected into the top `height - 48` pixels above a legend. The spatial view repaints with it as it rotates.
 */
export async function paintCloud(cloud: Cloud, camera: Camera, size: Size): Promise<Buffer> {
  const area = { width: size.width, height: size.height - 48 };
  const at = (point: Vec3): string =>
    projectPoint(point, camera, area)
      .map((n) => n.toFixed(1))
      .join(" ");
  const selected = new Set(cloud.selectedIndices);
  const step = Math.max(1, Math.ceil(cloud.points.length / 20_000));
  const circles: string[] = [];
  const circle = (i: number, radius: number, color: string, opacity: number): void => {
    const [x, y] = projectPoint(cloud.points[i], camera, area);
    circles.push(
      `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${radius}" fill="${color}" opacity="${opacity}"/>`,
    );
  };
  for (let i = 0; i < cloud.points.length; i += step) {
    if (selected.has(i)) continue;
    const elevation = (cloud.points[i][2] - camera.floor) / camera.height;
    const color = cloud.colors !== null ? `rgb(${cloud.colors[i].join(",")})` : "#6ae5cf";
    circle(i, 1, color, cloud.selectedIndices !== null ? 0.09 : cloud.colors !== null ? 0.9 : 0.12 + elevation * 0.75);
  }
  const highlighted = [...selected];
  const highlightStep = Math.max(1, Math.ceil(highlighted.length / 20_000));
  for (let n = 0; n < highlighted.length; n += highlightStep) circle(highlighted[n], 1.8, "#f5c078", 0.9);
  const grid: string[] = [];
  for (let i = -3; i <= 3; i++) {
    for (const axis of [0, 1]) {
      const a = [...camera.center];
      const b = [...camera.center];
      a[axis] += (i * camera.extent) / 6;
      b[axis] = a[axis];
      a[1 - axis] -= camera.extent / 2;
      b[1 - axis] += camera.extent / 2;
      grid.push(`<path d="M${at([a[0], a[1], camera.floor])}L${at([b[0], b[1], camera.floor])}" stroke="#284047"/>`);
    }
  }
  const brightness =
    cloud.colors !== null
      ? "source RGB"
      : `height brightness z=${camera.floor.toFixed(2)}..${(camera.floor + camera.height).toFixed(2)}`;
  const legend = `${cloud.points.length.toLocaleString()} points | display 1/${step} | ${brightness} | ${selected.size} selected`;
  const axes = size.height - 83;
  const svg =
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size.width}" height="${size.height}">` +
    `<rect width="${size.width}" height="${size.height}" fill="#101b21"/>${grid.join("")}${circles.join("")}` +
    `<path d="M30 ${axes}h25M30 ${axes}v-25M30 ${axes}l-14 12" stroke="#8ba4a8"/>` +
    `<g fill="#8ba4a8" font-family="monospace" font-size="12"><text x="59" y="${axes + 5}">x</text>` +
    `<text x="26" y="${axes - 31}">z</text><text x="7" y="${axes + 24}">y</text>` +
    `<text x="16" y="${size.height - 23}">${legend}</text></g></svg>`;
  return sharp(Buffer.from(svg)).removeAlpha().png().toBuffer();
}

const count = new Intl.NumberFormat("en-US");

export const pointsRenderer: Renderer<"points"> = {
  noun: "point cloud",
  accepts: async (file) => CloudShape.Check(file.json),
  measure: async (file) => `${count.format(parseCloud(file.json).points.length)} points`,
  render: async (file, label, context) => {
    const cloud = parseCloud(file.json);
    const png = await paintCloud(cloud, fitCamera(cloud), { width: 900, height: 448 });
    const timestamp = cloud.timestamp === null ? "" : `, timestamp ${cloud.timestamp}`;
    return {
      view: {
        kind: "points",
        label,
        source: { path: file.path, sha256: file.sha256 },
        preview: await context.cache.put(png, "png"),
        points: cloud.points.length,
        frame: cloud.frame,
      },
      image: await modelImage(png, "graphic"),
      caption: `${cloud.points.length} points, frame ${cloud.frame ?? "unspecified"}${timestamp}, painted projection`,
    };
  },
};
