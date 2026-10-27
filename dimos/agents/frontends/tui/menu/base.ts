import type { Ref } from "@earendil-works/chord";
import { InventoryContract } from "../../../shared/protocol/inventory/contract.ts";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { EpochMs } from "../../../shared/protocol/time.ts";
import type { Level } from "../../../shared/view-models/levels.ts";
import type { Ui } from "../session.ts";

/** What Enter opens on a row: a pane drawn from served state when it renders, closed by Esc. */
export type MenuDetail = {
  readonly title: string;
  lines(): readonly string[];
  close(): Promise<void>;
};

/** One key a row answers to, besides Enter. */
export type MenuKey = { readonly key: string; readonly label: string; run(): Promise<void> };

/** One row of a /menu tab: its text, its level, its keys, and what Enter does. */
export type MenuRow = {
  readonly text: string;
  readonly level: Level;
  readonly keys: readonly MenuKey[];
  /** Enter: a detail pane, or an action that leaves /menu (null after it ran); none when the row has nothing more. */
  readonly open: (() => MenuDetail | null) | null;
};

/** What a tab reads when it renders. */
export type TabContext = {
  readonly ui: Ui;
  readonly now: EpochMs;
  /** The inventory object held by the terminal. */
  readonly inventory: Ref<typeof InventoryContract> | null;
  /** The conversation the chat shows, to mark it. */
  readonly current: ConversationId | null;
  /** Shows a conversation in the chat and closes /menu. */
  attach(conversation: ConversationId): void;
};

/** One tab of /menu. Every tab reads served state when it renders; none pulls. */
export interface MenuTab {
  readonly title: string;
  rows(context: TabContext): readonly MenuRow[];
}
