import type { Component } from "@earendil-works/pi-tui";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { StreamCard } from "#dimos/agents/shared/view-models/streams/cards.ts";
import { Container, Text, TruncatedText } from "@earendil-works/pi-tui";
import { TerminalImage } from "#dimos/agents/frontends/tui/lib/terminal-image.ts";
import { accent, levelColor, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { duration } from "#dimos/agents/shared/view-models/time.ts";

const ARROWS = { in: "↓", out: "↑", inout: "↕" } as const;

const INDENT = "    ";

/** The title row's rate: the samples per second this terminal counted, and how long the stream has been silent. */
export function rateText(card: StreamCard, now: EpochMs): string {
  if (card.rate === null || card.level === null) return muted("no samples");
  const hz = `${card.rate.hz.toFixed(1)} Hz`;
  if (card.level === "ok" || card.rate.lastAt === null) return hz;
  return levelColor(card.level, `${hz} · silent ${duration(now - card.rate.lastAt)}`);
}

/**
 * One stream: a title row with its rate, then a picture slot or the newest value, indented under the title.
 */
export class StreamCardView implements Component {
  private readonly slot = new Container();
  private readonly imageId: number;
  private readonly columns: number;
  private readonly rows: number;
  private readonly changed: () => void;
  private image: TerminalImage | null = null;
  private value: string | null = null;
  private framed = false;
  private card: StreamCard | null = null;
  private now: EpochMs | null = null;

  constructor(imageId: number, columns: number, rows: number, changed: () => void) {
    this.imageId = imageId;
    this.columns = columns;
    this.rows = rows;
    this.changed = changed;
  }

  update(card: StreamCard, now: EpochMs): void {
    if (card.picture === null && this.framed) {
      this.framed = false;
      this.image?.close();
      this.image = null;
      this.slot.clear();
    }
    this.card = card;
    this.now = now;
  }

  setFrame(png: Buffer): void {
    this.framed = true;
    this.value = null;
    if (this.image !== null) {
      this.image.setPng(png);
      this.slot.invalidate();
      return;
    }
    this.image = new TerminalImage(
      png,
      { maxWidthCells: this.columns, maxHeightCells: this.rows },
      () => {
        this.slot.invalidate();
        this.changed();
      },
      this.imageId,
    );
    this.slot.clear();
    this.slot.addChild(this.image);
  }

  setValue(text: string): void {
    if (text === this.value) return;
    this.value = text;
    this.framed = false;
    this.image?.close();
    this.image = null;
    this.slot.clear();
    this.slot.addChild(new TruncatedText(muted(clean(text)), 0, 0));
  }

  setError(text: string): void {
    this.framed = false;
    this.image?.close();
    this.image = null;
    this.slot.clear();
    this.slot.addChild(new TruncatedText(muted(clean(text)), 0, 0));
  }

  render(width: number, selected = false, expanded = true, source?: string): string[] {
    const card = this.card;
    const now = this.now;
    const arrow = card?.direction === null || card === null ? "·" : ARROWS[card.direction];
    const Title = source === undefined ? TruncatedText : Text;
    const title =
      card === null || now === null
        ? []
        : new Title(
            ` ${selected ? accent("›") : " "}${expanded ? "▾" : "▸"} ${source === undefined ? `${arrow} ${clean(card.title)}  ${muted(clean(card.type))}` : clean(source)}  ${rateText(card, now)}`,
            0,
            0,
          ).render(width);
    if (!expanded) return title;
    const inner = Math.max(1, width - INDENT.length);
    return [...title, ...this.slot.render(inner).map((line) => INDENT + line)];
  }

  invalidate(): void {
    this.slot.invalidate();
  }

  dispose(): void {
    this.image?.close();
    this.image = null;
  }
}
