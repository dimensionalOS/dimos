import { Input, SelectList, matchesKey, truncateToWidth, wrapTextWithAnsi } from "@earendil-works/pi-tui";
import { getSelectListTheme } from "@earendil-works/pi-coding-agent";
import { accent, amber, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";

type Choice = { name: string; description: string };
type Catalog = { choices: readonly Choice[]; notice: string };

export class ProfileChoices {
  private readonly options: Catalog & {
    title: string;
    hint: string;
    selected: readonly string[];
    done: (selected: string[]) => void;
  };
  private catalog: Catalog;
  private readonly selected: Set<string>;
  private readonly search = new Input({ prompt: "Search: ", placeholder: "Type to filter", placeholderStyle: muted });
  private doneFocused = false;
  private list: SelectList;
  private rows = 8;

  constructor(options: ProfileChoices["options"]) {
    this.options = options;
    this.catalog = options;
    this.selected = new Set(options.selected);
    this.list = new SelectList([], this.rows, getSelectListTheme());
    this.rebuild();
  }

  setChoices(catalog: Catalog): void {
    this.catalog = catalog;
    this.rebuild();
  }

  handleInput(data: string): void {
    if (matchesKey(data, "escape")) {
      this.options.done([...this.selected]);
      return;
    }
    if (matchesKey(data, "tab") || matchesKey(data, "shift+tab")) {
      this.doneFocused = !this.doneFocused;
      return;
    }
    if (matchesKey(data, "enter") || data === " ") {
      if (this.doneFocused) this.options.done([...this.selected]);
      else this.toggle();
    } else if (matchesKey(data, "up") || matchesKey(data, "down")) {
      this.doneFocused = false;
      this.list.handleInput(data);
    } else {
      this.doneFocused = false;
      this.search.handleInput(data);
      this.rebuild(false);
    }
  }

  render(width: number, height: number): string[] {
    this.search.focused = !this.doneFocused;
    const header = [
      accent(this.options.title),
      ...wrapTextWithAnsi(muted(this.options.hint), width),
      "",
      ...this.search.render(width),
      muted(`${this.selected.size} selected`),
      ...wrapTextWithAnsi(this.catalog.notice ? amber(clean(this.catalog.notice)) : "", width).slice(0, 2),
    ];
    const item = this.list.getSelectedItem();
    const description = item?.description ?? "";
    const footer = [
      ...wrapTextWithAnsi(muted(clean(description)), width).slice(0, 2),
      "",
      this.doneFocused ? accent("[ Done ]") : muted("[ Done ]"),
      muted("Enter / Space toggle · Esc back with selections"),
      muted("Type to filter · ↑/↓ move · Tab Done"),
    ];
    const rows = Math.max(1, height - header.length - footer.length - 1);
    if (rows !== this.rows) {
      this.rows = rows;
      this.rebuild();
    }
    const list =
      item === null
        ? [muted(this.catalog.choices.length === 0 ? "  No choices available" : "  No matching choices")]
        : this.list.render(width);
    const body = [...list];
    while (body.length < this.rows + 1) body.push("");
    return [...header, ...body, ...footer].slice(0, height).map((line) => truncateToWidth(line, width));
  }

  private filtered(): readonly Choice[] {
    const query = this.search.getValue().toLowerCase();
    return this.catalog.choices.filter((choice) =>
      `${choice.name} ${choice.description}`.toLowerCase().includes(query),
    );
  }

  private toggle(): void {
    const item = this.list.getSelectedItem();
    if (item === null) return;
    if (this.selected.has(item.value)) this.selected.delete(item.value);
    else this.selected.add(item.value);
    this.rebuild();
  }

  private rebuild(preserve = true): void {
    const current = preserve ? this.list.getSelectedItem()?.value : undefined;
    const items = this.filtered().map((choice) => ({
      value: choice.name,
      label: `${this.selected.has(choice.name) ? "[x]" : "[ ]"} ${clean(choice.name)}`,
      description: clean(choice.description),
    }));
    this.list = new SelectList(items, this.rows, getSelectListTheme());
    this.list.setSelectedIndex(
      Math.max(
        0,
        items.findIndex((item) => item.value === current),
      ),
    );
  }
}
