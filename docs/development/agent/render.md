# Media rendering

`dimos/agents/shared/render/` turns the files a tool call or an event produced, images, point-cloud JSON and frame indexes, into what the
model receives and what the terminal shows. A `Rendering` is caption text for the model, one image per view, and
`RenderDetails` whose views name the source and the preview files with their hashes. The harness renders the files
`bash` wrote, the runtime tools returned and an event exported; the terminal reads the previews back by hash, paints
them into cells, rotates clouds and paints live stream frames. The entry is `FileRenderer` in [render.ts](/dimos/agents/shared/render/render.ts).
sharp is a native addon, so this folder runs in Node, in the gateway and in the terminal.

Source: [dimos/agents/shared/render/](/dimos/agents/shared/render/). TypeScript examples use `#dimos/agents/...` package subpaths.
Save temporary example files there and run them from `dimos/agents` with `node --import=./test/web-sdk.mjs shared/render/demo_example.ts`.
Remove the temporary file with `rm shared/render/demo_example.ts` before running `npm run typecheck` or `npm test`.
Those checks treat saved examples as package code and may reject their imports.

## Built on sharp

[sharp](https://sharp.pixelplumbing.com) is a Node binding to libvips. A call starts a pipeline with `sharp(input)`,
chains operations and runs it with an output such as `.png().toBuffer()`. This folder uses:

- Four inputs: an encoded file sharp reads (PNG, JPEG, WebP, GIF, TIFF, AVIF, SVG), raw pixel bytes with
  `{ raw: { width, height, channels } }`, a blank canvas with `{ create }` that `composite()` places tiles on, and PNGs
  joined as animated frames for a GIF.
- `metadata()` reads the format, size, orientation and page count without decoding pixels.
- `rotate()` with no angle turns an image upright by its EXIF orientation, and `resize()` with `fit: "inside"` fits it
  in a box.
- Outputs `.png()`, `.jpeg()`, `.gif()` and `.raw()`, the last being the RGB bytes the terminal turns into cells.

sharp decodes, resizes and encodes. This folder decides the size and format the model receives, the PNG the terminal
shows, the hash every file reference carries, how a cloud is projected and which frames of a clip are kept.

## Quickstart

Render one file as `bash` does after a command wrote it, print the text the model reads, and read each preview back
through `readVerified` as the terminal's views do. For a 64x48 PNG the text is
`plot.png · /home/you/plot.png · sha256 HASH · png 64x48`. The snippets under Usage continue from this `rendering`.

```ts
import { FileRenderer } from "#dimos/agents/shared/render/render.ts";
import { join, resolve } from "node:path";
import { mkdtemp } from "node:fs/promises";
import { readVerified } from "#dimos/agents/shared/render/files.ts";
import { tmpdir } from "node:os";

const renderer = new FileRenderer({
    cacheDir: await mkdtemp(join(tmpdir(), "render-")),
    maxCacheBytes: 128 * 2 ** 20,
    maxSourceBytes: 64 * 2 ** 20,
});
const path = resolve(process.argv[2] ?? "plot.png");
const rendering = await renderer.render([{ path, sha256: null }], "plot");
if (rendering === null) throw new Error(`nothing renderable at ${path}`);
console.log(rendering.text);
for (const view of rendering.details.views) {
    const preview = await readVerified(view.kind === "clip" ? view.sheet : view.preview);
    console.log(`${view.label}: ${view.kind}, ${preview.length}-byte PNG preview`);
}
```

## Usage

**Show the previews in a terminal.** `overviewOf` tiles a rendering's previews into one picture, as the terminal's
gallery does, and `cellPixels` samples it into RGB bytes with two samples per row, which the terminal paints as colored
cells. `pngOf` and `rowSlices` prepare a picture for terminals that draw PNGs.

```ts
import { cellPixels, overviewOf } from "#dimos/agents/shared/render/raster.ts";

const previews = await Promise.all(
    rendering.details.views.map((view) => readVerified(view.kind === "clip" ? view.sheet : view.preview)),
);
const rgb = await cellPixels(await overviewOf(previews), 80, 24);
console.log(`${rgb.length} RGB bytes for 80 columns by 24 rows`);
```

**Paint a point cloud from another angle.** `parseCloud` reads the cloud JSON a `points` view names as its `source`,
`fitCamera` frames it, and `paintCloud` draws it. The terminal's spatial view turns `yaw` by 45 degrees per key press
and paints again with the same camera, so the scale and the height colors hold.

```ts
import { fitCamera, paintCloud, parseCloud } from "#dimos/agents/shared/render/points.ts";
import { readFile, writeFile } from "node:fs/promises";

const cloud = parseCloud(JSON.parse(await readFile("cloud.json", "utf8")));
const fitted = fitCamera(cloud);
const turned = { ...fitted, yaw: fitted.yaw + Math.PI / 4 };
await writeFile("cloud.png", await paintCloud(cloud, turned, { width: 900, height: 448 }));
```

**Paint a live frame.** The sidebar paints each stream frame at the size of its card: a camera JPEG with `imageFrame`, a
cloud with `cloudFrame` through a top-down camera, and an occupancy grid with `gridFrame`. Each returns a PNG.

```ts
import { cloudFrame, topDownCamera } from "#dimos/agents/shared/render/frames.ts";

const xyz = new Float32Array([0, 0, 0, 1, 0, 0.5, 0, 1, 1]);
const frame = await cloudFrame(xyz, topDownCamera(xyz), { width: 320, height: 200 });
console.log(`${frame.length}-byte PNG`);
```

## API

| Export                                                      | File                                             | What it does                                                                                                                                                                                                                                        |
| ----------------------------------------------------------- | ------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `FileRenderer({ cacheDir, maxCacheBytes, maxSourceBytes })` | [render.ts](/dimos/agents/shared/render/render.ts)                           | `render(files, title, onProgress?)` renders every renderable file of a call or an event as one group and returns a `Rendering`, or null when nothing was renderable. Each file is `{ path, sha256 }`, with the hash when the producer recorded one. |
| `Rendering`, `RenderProgress`                               | [types.ts](/dimos/agents/shared/render/types.ts)                             | The `text`, `images` and `details` a rendering adds to a tool result or an input, and the `done`, `total` and `latest` a clip reports while its frames render.                                                                                      |
| `readVerified(ref)`                                         | [files.ts](/dimos/agents/shared/render/files.ts)                             | A file's bytes, when they still hash to `ref.sha256`. Every terminal view reads through it.                                                                                                                                                         |
| `parseCloud`, `fitCamera`, `paintCloud`, `projectPoint`     | [points.ts](/dimos/agents/shared/render/points.ts)                           | A cloud from its JSON, a camera that fits it, the painted PNG, and the pixel a point lands on.                                                                                                                                                      |
| `imageFrame`, `cloudFrame`, `gridFrame`, `topDownCamera`    | [frames.ts](/dimos/agents/shared/render/frames.ts)                           | A live frame as a PNG at a card's size, from a JPEG, an xyz array or occupancy cells.                                                                                                                                                               |
| `pngOf`, `cellPixels`, `rowSlices`, `overviewOf`            | [raster.ts](/dimos/agents/shared/render/raster.ts)                           | A picture as a PNG, as RGB cell samples, as one PNG per terminal row, and several previews as one mosaic.                                                                                                                                           |
| `modelImages(undecodable)`                                  | [render.ts](/dimos/agents/shared/render/render.ts)                           | The image processor for pi-durable's `read`, which prepares a file the way the renderers do and hands a format sharp cannot decode to `undecodable`.                                                                                                |
| `Renderer<K>`                                               | [base.ts](/dimos/agents/shared/render/base.ts)                               | One format: `accepts`, `measure` and `render`. [image.ts](/dimos/agents/shared/render/image.ts), [points.ts](/dimos/agents/shared/render/points.ts) and [clip.ts](/dimos/agents/shared/render/clip.ts) implement it, and render.ts tries them clip, points, image.                                                                      |
| `FileRef`, `RenderedView`, `RenderDetails`, `MAX_VIEWS`     | [shared/protocol/media.ts](/dimos/agents/shared/protocol/media.ts) | What a rendering records, because the records cross to the terminal: a path with its hash, one view told apart by `kind`, and the details with at most six views.                                                                                   |

Three facts every caller relies on:

- The model receives a file as it is when it is JPEG, PNG, WebP or GIF, upright, a single frame, within 1600x1200 and at
  most 4.5 MiB as base64. Anything else is turned upright and fitted into that box, a photo as JPEG and a graphic as
  PNG. The terminal's preview is always a PNG fitted into the same box.
- Sources are read and left as they are. A preview lives in the cache as `<sha256>.png` or `.gif`, so a picture rendered
  twice is stored once, and a source whose bytes changed since its producer hashed it fails to render.
- A rendering holds at most `MAX_VIEWS` views, six. Each renderable file past them is listed by path with a notice
  telling the model to `read` it.
