import type { FileRenderer } from "../../shared/render/render.ts";
import type { HarnessEvent } from "../../shared/protocol/runtime/event.ts";
import type { InputDraft } from "../inputs/gate.ts";

/** How an event reads to the model: its notice and data as text, and the files `notify` exported as images. */
export class EventDrafts {
  private readonly renderer: FileRenderer;

  constructor(renderer: FileRenderer) {
    this.renderer = renderer;
  }

  async forEvent(event: HarnessEvent): Promise<InputDraft> {
    const lines = [`[DimOS event] ${event.module}: ${event.text}`];
    if (Object.keys(event.data).length > 0) lines.push(`Data: ${JSON.stringify(event.data)}`);
    if (event.suppressed > 0)
      lines.push(`${event.suppressed} earlier notices from this module were dropped by its rate limit.`);
    lines.push("This message comes from an agent module running in DimOS, not from the user.");
    const rendering =
      event.exports.length === 0
        ? null
        : await this.renderer.render(
            event.exports.map((item) => ({ path: item.path, sha256: item.sha256 })),
            `${event.module} notice`,
          );
    if (rendering !== null) lines.push(rendering.text);
    return {
      source: { kind: "event", event },
      text: lines.join("\n"),
      images: rendering?.images ?? [],
      render: rendering?.details ?? null,
      whenBusy: "followUp",
    };
  }
}
