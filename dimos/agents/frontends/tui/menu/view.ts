import { owner, refusalOfFailure } from "#dimos/agents/sdk/failures.ts";
import type { Component, TUI } from "@earendil-works/pi-tui";
import type { ConversationId } from "@earendil-works/pi-durable";
import type { EpochMs } from "#dimos/agents/shared/protocol/time.ts";
import type { Profile } from "#dimos/agents/shared/protocol/profiles.ts";
import type { MenuDetail, MenuRow, MenuTab, TabContext } from "#dimos/agents/frontends/tui/menu/base.ts";
import type { Ui } from "#dimos/agents/frontends/tui/session.ts";
import { ProfileEditor } from "#dimos/agents/frontends/tui/menu/lib/profile-editor.ts";
import { accent, levelColor, muted } from "#dimos/agents/frontends/tui/style.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";
import { actionLine } from "#dimos/agents/shared/view-models/actions.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { conversationsTab } from "#dimos/agents/frontends/tui/menu/conversations.ts";
import { connectionSegment } from "#dimos/agents/shared/view-models/connection.ts";
import { linkSegments, runSegment } from "#dimos/agents/shared/view-models/gateway/link.ts";
import { jobsHeading, jobsTab } from "#dimos/agents/frontends/tui/menu/jobs.ts";
import { listen } from "#dimos/agents/sdk/state.ts";
import {
  decodeKittyPrintable,
  matchesKey,
  truncateToWidth,
  visibleWidth,
  wrapTextWithAnsi,
} from "@earendil-works/pi-tui";
import { modulesTab } from "#dimos/agents/frontends/tui/menu/modules.ts";

const TABS: readonly (MenuTab | null)[] = [null, conversationsTab, modulesTab, jobsTab];
const TITLES = TABS.map((tab) => tab?.title ?? "Profiles");
const WIDE = 100;

type Action = { label: string; run(): void | Promise<void> };
type Item = { kind: "profile"; id: string } | { kind: "module"; name: string } | { kind: "job"; name: string };
type ItemScreen = { kind: "item"; item: Item; selected: number; confirmDelete?: boolean };
type Screen =
  | { kind: "list"; selectProfile?: string }
  | ItemScreen
  | { kind: "detail"; detail: MenuDetail; parent: ItemScreen; scroll: number };

export type MenuMemory = { tab: number; selections: number[]; filters: string[] };
export function newMenuMemory(): MenuMemory {
  return { tab: 0, selections: [0, 0, 0, 0], filters: ["", "", "", ""] };
}

type MenuViewOptions = {
  readonly ui: Ui;
  readonly tui: TUI;
  readonly current: ConversationId | null;
  readonly attach: (conversation: ConversationId) => void;
  readonly create: (profile: string) => void;
  readonly close: () => void;
  readonly requestRender: () => void;
  readonly height: () => number;
  readonly memory: MenuMemory;
};

export class MenuView implements Component {
  private readonly stop: () => void;
  private readonly options: MenuViewOptions;
  private screen: Screen = { kind: "list" };
  private editor: ProfileEditor | null = null;

  constructor(options: MenuViewOptions) {
    this.options = options;
    const { ui } = options;
    this.stop = listen(
      [
        ui.gateway.link,
        ui.gateway.load,
        ui.gateway.runs,
        ui.gateway.conversations,
        ui.gateway.profiles,
        ui.gateway.modules,
        ui.gateway.jobs,
        ui.actions,
        ui.connection,
      ],
      () => options.requestRender(),
    );
  }

  handleInput(data: string): void {
    if (this.editor !== null) {
      this.editor.handleInput(data);
      this.options.requestRender();
      return;
    }
    this.selectSavedProfile();
    const screen = this.screen;
    if (matchesKey(data, "escape")) {
      if (screen.kind === "detail") this.screen = screen.parent;
      else if (screen.kind === "item")
        this.screen =
          screen.item.kind === "profile" ? { kind: "list", selectProfile: screen.item.id } : { kind: "list" };
      else this.options.close();
    } else if (screen.kind === "detail") {
      if (matchesKey(data, "up")) screen.scroll = Math.max(0, screen.scroll - 1);
      if (matchesKey(data, "down")) screen.scroll++;
    } else if (screen.kind === "item") {
      const actions = this.actions(screen);
      screen.selected = Math.min(screen.selected, Math.max(0, actions.length - 1));
      if (matchesKey(data, "up") || matchesKey(data, "down"))
        screen.selected = Math.max(
          0,
          Math.min(actions.length - 1, screen.selected + (matchesKey(data, "down") ? 1 : -1)),
        );
      else if (matchesKey(data, "enter")) this.run(actions[screen.selected]);
    } else if (matchesKey(data, "left") || matchesKey(data, "right")) {
      const direction = matchesKey(data, "right") ? 1 : -1;
      this.options.memory.tab = (this.tab + direction + TABS.length) % TABS.length;
    } else if (matchesKey(data, "up") || matchesKey(data, "down")) {
      this.selected = Math.max(
        0,
        Math.min(this.items().length - 1, this.selected + (matchesKey(data, "down") ? 1 : -1)),
      );
    } else if (matchesKey(data, "enter")) this.open();
    else {
      const character = decodeKittyPrintable(data) ?? data;
      if (matchesKey(data, "backspace") || matchesKey(data, "ctrl+h"))
        this.filter = [...this.filter].slice(0, -1).join("");
      else if ([...character].length === 1 && character >= " " && character !== "\x7f") this.filter += character;
      else return;
      this.selected = 0;
    }
    if (screen.kind === "list") delete screen.selectProfile;
    this.options.requestRender();
  }

  render(width: number): string[] {
    if (this.editor !== null) return this.editor.render(width, this.options.height());
    const tabs = TITLES.map((title, index) => (index === this.tab ? accent(`[${title}]`) : muted(` ${title} `))).join(
      " ",
    );
    const { ui } = this.options;
    const runs = ui.gateway.runs.value;
    const run = runSegment(runs);
    const status = [
      connectionSegment(ui.connection.value),
      ...linkSegments(ui.gateway.link.value, ui.gateway.load.value, runs, Date.now() as EpochMs),
      ...(run === null ? [] : [run]),
    ];
    const statusLine = status.map((segment) => levelColor(segment.level, clean(segment.text))).join(muted(" · "));
    const lines = [tabs];
    const withFooter = (footer: readonly string[]): string[] => {
      const height = Math.max(0, this.options.height());
      const bottom = footer.slice(Math.max(0, footer.length - height));
      const bodyHeight = height - bottom.length;
      const body = lines.slice(0, bodyHeight);
      while (body.length < bodyHeight) body.push("");
      return [...body, ...bottom].map((line) => truncateToWidth(line, width));
    };
    const newest = ui.actions.value.at(-1);
    const observation = newest === undefined ? "" : clean(actionLine(newest));
    const screen = this.screen;
    if (screen.kind === "detail") {
      const detailLines = screen.detail.lines().flatMap((line) => wrapTextWithAnsi(clean(line), width));
      const available = Math.max(1, this.options.height() - 6);
      screen.scroll = Math.min(screen.scroll, Math.max(0, detailLines.length - available));
      lines.push(
        "",
        accent(clean(`Jobs › ${screen.detail.title}`)),
        ...detailLines.slice(screen.scroll, screen.scroll + available),
      );
      return withFooter([observation, muted("↑/↓ scroll · Esc back"), statusLine]);
    }
    if (screen.kind === "item") {
      const profile = screen.item.kind === "profile" ? this.profile(screen.item.id) : undefined;
      const row = this.row(screen.item);
      const definition =
        profile === undefined ? (row === undefined ? ["This item is gone."] : [row.text]) : this.preview(profile);
      const actions = this.actions(screen);
      screen.selected = Math.min(screen.selected, Math.max(0, actions.length - 1));
      const title = screen.item.kind === "profile" ? (profile?.name ?? screen.item.id) : screen.item.name;
      const available = Math.max(0, this.options.height() - actions.length - 8);
      lines.push(
        "",
        accent(clean(`${TITLES[this.tab]} › ${title}`)),
        ...definition.map((line) => truncateToWidth(clean(line), width)).slice(0, available),
        "",
        ...actions.map((action, index) =>
          index === screen.selected ? accent(`› ${clean(action.label)}`) : `  ${clean(action.label)}`,
        ),
      );
      return withFooter([observation, muted("↑/↓ select action · Enter run · Esc back"), statusLine]);
    }
    this.selectSavedProfile();
    const items = this.items();
    this.selected = Math.min(this.selected, Math.max(0, items.length - 1));
    const footer = [
      observation,
      muted("←/→ sections · ↑/↓ select · Enter open · type to filter · Esc back"),
      statusLine,
    ];
    const available = Math.max(1, this.options.height() - lines.length - footer.length - (this.tab === 3 ? 2 : 1));
    const start = Math.max(0, Math.min(this.selected - Math.floor(available / 2), items.length - available));
    const list =
      items.length === 0
        ? [muted("  No matches")]
        : items
            .slice(start, start + available)
            .map(
              (item, offset) =>
                (start + offset === this.selected ? accent("› ") : "  ") + levelColor(item.level, clean(item.text)),
            );
    const profile = this.tab === 0 && width >= WIDE ? this.filteredProfiles()[this.selected - 1] : undefined;
    lines.push(muted(`Search: ${this.filter || "(type to filter)"}`));
    if (this.tab === 3) lines.push(muted(jobsHeading(Date.now() as EpochMs)));
    if (profile !== undefined) {
      const left = Math.floor(width * 0.4);
      const right = width - left - 3;
      const details = this.preview(profile).flatMap((line) => wrapTextWithAnsi(clean(line), right));
      for (let i = 0; i < Math.min(available, Math.max(list.length, details.length)); i++) {
        const l = truncateToWidth(list[i] ?? "", left);
        lines.push(l + " ".repeat(Math.max(0, left - visibleWidth(l))) + " │ " + (details[i] ?? ""));
      }
    } else lines.push(...list);
    return withFooter(footer);
  }

  invalidate(): void {}
  dispose(): void {
    this.stop();
  }
  private get tab(): number {
    return this.options.memory.tab;
  }
  private get selected(): number {
    return this.options.memory.selections[this.tab] ?? 0;
  }
  private set selected(value: number) {
    this.options.memory.selections[this.tab] = value;
  }
  private get filter(): string {
    return this.options.memory.filters[this.tab] ?? "";
  }
  private set filter(value: string) {
    this.options.memory.filters[this.tab] = value;
  }

  private context(): TabContext {
    return {
      ui: this.options.ui,
      now: Date.now() as EpochMs,
      current: this.options.current,
      attach: this.options.attach,
    };
  }
  private profiles(): readonly Profile[] {
    return this.options.ui.gateway.profiles.value.profiles;
  }
  private filteredProfiles(): readonly Profile[] {
    const filter = this.filter.toLowerCase();
    return this.profiles().filter((profile) =>
      `${profile.name} ${profile.description} ${profile.role}`.toLowerCase().includes(filter),
    );
  }
  private selectSavedProfile(): void {
    const screen = this.screen;
    if (screen.kind !== "list" || screen.selectProfile === undefined) return;
    const index = this.filteredProfiles().findIndex((profile) => profile.id === screen.selectProfile);
    if (index < 0) return;
    this.selected = index + 1;
  }
  private profile(id: string): Profile | undefined {
    return this.profiles().find((profile) => profile.id === id);
  }
  private items(): readonly MenuRow[] {
    if (this.tab === 0) {
      const catalog = this.options.ui.gateway.profiles.value;
      const current = this.options.ui.gateway.conversations.value.find((row) => row.id === this.options.current);
      const currentProfile = current && (current.profile?.id ?? "general");
      return [
        { text: "+ Create profile", level: "ok", actions: [], open: null },
        ...this.filteredProfiles().map(
          (profile): MenuRow => ({
            text: `${profile.name}${profile.id === (catalog.defaultProfile ?? "general") ? " · default" : ""}${profile.id === currentProfile ? " · current" : ""}`,
            level: "ok",
            actions: [],
            open: null,
          }),
        ),
      ];
    }
    const tab = TABS[this.tab];
    return (tab?.rows(this.context()) ?? []).filter((row) =>
      `${row.text} ${row.searchText ?? ""}`.toLowerCase().includes(this.filter.toLowerCase()),
    );
  }
  private open(): void {
    this.selected = Math.min(this.selected, Math.max(0, this.items().length - 1));
    let item: Item;
    if (this.tab === 0) {
      if (this.selected === 0) {
        this.edit(null);
        return;
      }
      const profile = this.filteredProfiles()[this.selected - 1];
      if (profile === undefined) return;
      item = { kind: "profile", id: profile.id };
    } else {
      const row = this.items()[this.selected];
      if (row === undefined) return;
      if (this.tab === 1) {
        this.run({
          label: "Resume conversation",
          run: () => {
            row.open?.();
          },
        });
        return;
      }
      if (row.searchText === undefined) return;
      item = { kind: this.tab === 2 ? "module" : "job", name: row.searchText };
    }
    this.screen = { kind: "item", item, selected: 0 };
  }
  private row(item: Item): MenuRow | undefined {
    if (item.kind !== "profile") {
      const tab = item.kind === "module" ? modulesTab : jobsTab;
      return tab.rows(this.context()).find((row) => row.searchText === item.name);
    }
    return undefined;
  }
  private actions(screen: ItemScreen): Action[] {
    if (screen.item.kind === "profile") {
      const profile = this.profile(screen.item.id);
      if (profile === undefined) return [];
      if (screen.confirmDelete)
        return [
          {
            label: "Cancel",
            run: () => {
              screen.confirmDelete = false;
              screen.selected = 0;
            },
          },
          {
            label: `Confirm delete ${profile.name}`,
            run: async () => {
              const result = await this.options.ui.actions.observe(
                `delete ${profile.name}`,
                owner(this.options.ui.gateway).deleteProfile({ profile: profile.id }),
              );
              if (result.ok && this.screen === screen) this.screen = { kind: "list" };
            },
          },
        ];
      const previous = this.options.ui.gateway.conversations.value
        .filter((row) => (row.profile?.id ?? "general") === profile.id)
        .sort((a, b) => b.updatedAt - a.updatedAt)[0];
      const actions: Action[] = [{ label: "Start new conversation", run: () => this.options.create(profile.id) }];
      if (previous !== undefined)
        actions.push({
          label: `Resume latest conversation #${previous.id}`,
          run: () => this.options.attach(previous.id),
        });
      if (profile.id !== "general") actions.push({ label: "Edit profile", run: () => this.edit(profile) });
      actions.push({
        label: "Set as default",
        run: async () => {
          await this.options.ui.actions.observe(
            `default ${profile.name}`,
            owner(this.options.ui.gateway).setDefaultProfile({ profile: profile.id }),
          );
        },
      });
      if (profile.id !== "general")
        actions.push({
          label: "Delete profile",
          run: () => {
            screen.confirmDelete = true;
            screen.selected = 0;
          },
        });
      return actions;
    }
    const row = this.row(screen.item);
    if (row === undefined) return [];
    const actions: Action[] = [];
    if (screen.item.kind === "job" && row.open !== null)
      actions.push({
        label: "View history",
        run: () => {
          const detail = row.open?.();
          if (detail) this.screen = { kind: "detail", detail, parent: screen, scroll: 0 };
        },
      });
    return [
      ...actions,
      ...row.actions.map((action) => ({ ...action, label: action.label[0]?.toUpperCase() + action.label.slice(1) })),
    ];
  }
  private preview(profile: Profile): string[] {
    return [
      `${profile.name} · ${profile.description}`,
      `Role: ${profile.role || "unspecified"}`,
      `Expected blueprints: ${profile.blueprints.join(", ") || "none"}`,
      `Always-loaded skills: ${profile.alwaysLoadedSkills.join(", ") || "none"}`,
      `Available skills: ${profile.availableSkills.join(", ") || "none"}`,
    ];
  }
  private edit(profile: Profile | null): void {
    const editor = new ProfileEditor({
      tui: this.options.tui,
      profile,
      save: (value) => {
        void Promise.resolve()
          .then(() =>
            this.options.ui.actions.observe(
              "save profile",
              owner(this.options.ui.gateway).saveProfile({ profile: value }),
            ),
          )
          .then((result) => {
            if (this.editor !== editor) return;
            if (result.ok) {
              this.editor = null;
              if (profile === null) {
                this.screen = { kind: "list", selectProfile: result.value.id };
                this.filter = "";
                this.selectSavedProfile();
              }
            } else editor.setError(refusalText(result.refusal));
          })
          .catch((error) => {
            if (this.editor === editor) editor.setError(refusalText(refusalOfFailure(error, Date.now() as EpochMs)));
          })
          .finally(() => this.options.requestRender());
      },
      cancel: () => {
        this.editor = null;
      },
    });
    this.editor = editor;
    void Promise.resolve()
      .then(() => owner(this.options.ui.gateway).profileOptions({}))
      .then((result) => {
        if (result.ok) editor.setOptions(result.value);
        else editor.setOptionsError(refusalText(result.refusal));
      })
      .catch((error) => editor.setOptionsError(refusalText(refusalOfFailure(error, Date.now() as EpochMs))))
      .finally(() => this.options.requestRender());
  }
  private run(action: Action | undefined): void {
    if (action === undefined) return;
    Promise.resolve()
      .then(() => action.run())
      .catch((error) => this.options.ui.actions.refuse(action.label, refusalOfFailure(error, Date.now() as EpochMs)))
      .finally(() => this.options.requestRender());
  }
}
