import type { FrameBox } from "#dimos/agents/shared/render/frames.ts";
import type { FrameRecord } from "#dimos/agents/sdk/streams/base.ts";
import { cloudFrame, gridFrame, imageFrame, topDownCamera } from "#dimos/agents/shared/render/frames.ts";

type PictureRecord = Extract<FrameRecord, { readonly kind: "image" | "cloud" | "grid" }>;

export class FramePainter {
  private readonly box: FrameBox;

  constructor(box: FrameBox) {
    this.box = box;
  }

  paint(record: PictureRecord): Promise<Buffer> {
    switch (record.kind) {
      case "image":
        return imageFrame(record.jpeg, this.box);
      case "cloud":
        return cloudFrame(record.xyz, topDownCamera(record.xyz), this.box);
      case "grid":
        return gridFrame(record.cells, record.width, record.height, this.box);
    }
  }
}
