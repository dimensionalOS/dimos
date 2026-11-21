import type { TUI } from "@earendil-works/pi-tui";
import { CURSOR_MARKER, Editor, Input, matchesKey, truncateToWidth, wrapTextWithAnsi } from "@earendil-works/pi-tui";
import { wordWrapLine } from "@earendil-works/pi-tui/dist/components/editor.js";
import { getSelectListTheme } from "@earendil-works/pi-coding-agent";
import type { Profile, ProfileDraft, ProfileOptions } from "#dimos/agents/shared/protocol/profiles.ts";
import { ProfileChoices } from "#dimos/agents/frontends/tui/menu/lib/profile-choices.ts";
import { accent, muted, red } from "#dimos/agents/frontends/tui/style.ts";
import { clean, cleanMultiline } from "#dimos/agents/shared/view-models/text.ts";

type ChoiceField = "blueprints" | "availableSkills" | "alwaysLoadedSkills";
const FOCUS = [
  "name",
  "description",
  "role",
  "blueprints",
  "availableSkills",
  "alwaysLoadedSkills",
  "cancel",
  "save",
] as const;
const LABELS = {
  blueprints: "Blueprints",
  availableSkills: "Available skills",
  alwaysLoadedSkills: "Always loaded skills",
};
type ChoicesState = { kind: "loading" } | { kind: "ready"; value: ProfileOptions } | { kind: "failed"; error: string };

export class ProfileEditor {
  private readonly options: {
    tui: TUI;
    profile: Profile | null;
    save: (profile: ProfileDraft) => void;
    cancel: () => void;
  };
  private readonly name = new Input({ prompt: "  ", placeholder: "Give this profile a name", placeholderStyle: muted });
  private readonly description = new Input({
    prompt: "  ",
    placeholder: "A short description",
    placeholderStyle: muted,
  });
  private readonly role: Editor;
  private readonly selections: Pick<ProfileDraft, ChoiceField>;
  private choices: ChoicesState = { kind: "loading" };
  private picker: { field: ChoiceField; view: ProfileChoices } | null = null;
  private focused = 0;
  private scroll = 0;
  private roleWidth = 58;
  private nameError = "";
  private error = "";
  private saving = false;

  constructor(options: ProfileEditor["options"]) {
    this.options = options;
    const profile = options.profile;
    this.name.setValue(profile?.name ?? "");
    this.name.handleInput("\x05");
    this.description.setValue(profile?.description ?? "");
    this.description.handleInput("\x05");
    this.name.onSubmit = () => {
      this.focused = 1;
    };
    this.description.onSubmit = () => {
      this.focused = 2;
    };
    this.role = new Editor(options.tui, { borderColor: muted, selectList: getSelectListTheme() }, { paddingX: 1 });
    this.role.setText(profile?.role ?? "");
    this.selections = {
      blueprints: [...(profile?.blueprints ?? [])],
      availableSkills: [...(profile?.availableSkills ?? [])],
      alwaysLoadedSkills: [...(profile?.alwaysLoadedSkills ?? [])],
    };
  }

  setOptions(options: ProfileOptions): void {
    this.choices = { kind: "ready", value: options };
    this.refreshPicker();
  }

  setOptionsError(error: string): void {
    this.choices = { kind: "failed", error };
    this.refreshPicker();
  }

  setError(error: string): void {
    this.error = error;
    this.saving = false;
  }

  handleInput(data: string): void {
    if (this.picker !== null) {
      this.picker.view.handleInput(data);
      return;
    }
    if (matchesKey(data, "escape")) {
      this.options.cancel();
      return;
    }
    if (this.saving) return;
    if (matchesKey(data, "tab") || matchesKey(data, "shift+tab")) {
      this.focused = (this.focused + (matchesKey(data, "tab") ? 1 : -1) + FOCUS.length) % FOCUS.length;
      return;
    }
    if (matchesKey(data, "up") || matchesKey(data, "down")) {
      if (FOCUS[this.focused] === "role") {
        const { line, col } = this.role.getCursor();
        const lines = this.role.getLines();
        const chunks = wordWrapLine(lines[line] ?? "", Math.max(1, this.roleWidth - 2 * this.role.getPaddingX()));
        const atBoundary = matchesKey(data, "up")
          ? line === 0 && col < (chunks[1]?.startIndex ?? Infinity)
          : line === lines.length - 1 && col >= (chunks.at(-1)?.startIndex ?? 0);
        if (!atBoundary) {
          this.role.handleInput(data);
          return;
        }
      }
      this.focused = Math.max(0, Math.min(FOCUS.length - 1, this.focused + (matchesKey(data, "down") ? 1 : -1)));
      return;
    }
    if (matchesKey(data, "ctrl+s")) {
      this.submit();
      return;
    }
    const focused = FOCUS[this.focused];
    switch (focused) {
      case "name":
        this.name.handleInput(data);
        if (this.name.getValue().trim()) this.nameError = "";
        break;
      case "description":
        this.description.handleInput(data);
        break;
      case "role":
        this.role.handleInput(matchesKey(data, "enter") ? "\n" : data);
        break;
      case "blueprints":
      case "availableSkills":
      case "alwaysLoadedSkills":
        if (matchesKey(data, "enter") || data === " ") this.openPicker(focused);
        break;
      case "cancel":
        if (matchesKey(data, "enter") || data === " ") this.options.cancel();
        break;
      case "save":
        if (matchesKey(data, "enter") || data === " ") this.submit();
        break;
    }
  }

  render(width: number, height: number): string[] {
    if (this.picker !== null) return this.picker.view.render(width, height);
    const focused = FOCUS[this.focused];
    const label = (key: typeof focused, text: string) => (key === focused ? accent(`› ${text}`) : `  ${text}`);
    this.name.focused = focused === "name";
    this.description.focused = focused === "description";
    this.role.focused = focused === "role";
    this.role.borderColor = this.role.focused ? accent : muted;
    const roleWidth = Math.max(3, width - 2);
    this.roleWidth = roleWidth;
    const roleRows = height >= 30 ? 5 : 3;
    const roleLines = this.role.focused
      ? this.role.render(roleWidth).slice(1, -1)
      : wrapTextWithAnsi(
          cleanMultiline(this.role.getExpandedText()) || muted("Describe this agent's role and instructions."),
          roleWidth - 2,
        ).map((line) => ` ${line}`);
    const cursor = roleLines.findIndex((line) => line.includes(CURSOR_MARKER));
    const roleStart = Math.max(0, cursor - roleRows + 1);
    const roleVisible = roleLines.slice(roleStart, roleStart + roleRows);
    while (roleVisible.length < roleRows) roleVisible.push("");
    const border = (more: boolean, direction: string) =>
      this.role.borderColor(
        more ? `${direction} more ${"─".repeat(Math.max(0, roleWidth - 7))}` : "─".repeat(roleWidth),
      );
    const sections = [
      [
        label("name", "Name *"),
        ...(this.name.focused
          ? this.name.render(width)
          : [`  ${clean(this.name.getValue()) || muted("Give this profile a name")}`]),
        ...(this.nameError ? [`  ${red(this.nameError)}`] : []),
      ],
      [
        label("description", "Description"),
        ...(this.description.focused
          ? this.description.render(width)
          : [`  ${clean(this.description.getValue()) || muted("A short description")}`]),
      ],
      [
        label("role", "Role"),
        ...[border(roleStart > 0, "↑"), ...roleVisible, border(roleStart + roleRows < roleLines.length, "↓")].map(
          (line) => `  ${line}`,
        ),
      ],
      ...(["blueprints", "availableSkills", "alwaysLoadedSkills"] as const).map((field) => [
        label(field, LABELS[field]),
        `  ${focused === field ? accent("[ Choose… ]") : muted("[ Choose… ]")} ${this.selectionText(field)}`,
      ]),
    ];
    const title = this.options.profile === null ? "Create profile" : "Edit profile";
    const button = this.saving ? "Saving…" : this.options.profile === null ? "Create profile" : "Save changes";
    const header = [accent(title), muted("Name is required. All other fields are optional."), ""];
    const footer = [
      ...wrapTextWithAnsi(this.error ? red(clean(this.error)) : "", width).slice(0, 2),
      `  ${focused === "cancel" ? accent("[ Cancel ]") : muted("[ Cancel ]")}  ${focused === "save" ? accent(`[ ${button} ]`) : muted(`[ ${button} ]`)}`,
      muted("↑/↓ fields · Tab / Shift+Tab · Ctrl+S save · Esc cancel"),
      muted(
        focused === "role"
          ? "↑/↓ text lines, then fields at ends · Enter newline"
          : focused === "name" || focused === "description"
            ? "Enter next field"
            : "Enter opens choices or activates a button",
      ),
    ];
    const body = sections.flat();
    const available = Math.max(1, height - header.length - footer.length);
    const selectedSection = Math.min(this.focused, sections.length - 1);
    const start = sections.slice(0, selectedSection).flat().length;
    const end = start + (sections[selectedSection]?.length ?? 0);
    this.scroll = Math.min(this.scroll, Math.max(0, body.length - available));
    if (start < this.scroll) this.scroll = start;
    if (end > this.scroll + available) this.scroll = Math.max(0, end - available);
    return [...header, ...body.slice(this.scroll, this.scroll + available), ...footer]
      .slice(0, height)
      .map((line) => truncateToWidth(line, width));
  }

  private selectionText(field: ChoiceField): string {
    const values = this.selections[field];
    return values.length === 0 ? muted("None selected") : clean(`${values.length} selected · ${values.join(", ")}`);
  }

  private pickerData(field: ChoiceField) {
    const options = this.choices.kind === "ready" ? this.choices.value : null;
    const catalog =
      field === "blueprints"
        ? (options?.blueprints ?? []).map((name) => ({ name, description: "" }))
        : (options?.skills ?? []);
    const choices = new Map(catalog.map((choice) => [choice.name, { ...choice }]));
    const saved =
      field === "blueprints"
        ? this.selections.blueprints
        : [...this.selections.availableSkills, ...this.selections.alwaysLoadedSkills];
    for (const name of saved)
      if (!choices.has(name)) choices.set(name, { name, description: "Saved selection · not in current catalog" });
    const other = field === "availableSkills" ? "alwaysLoadedSkills" : "availableSkills";
    if (field !== "blueprints")
      for (const name of this.selections[other]) {
        const choice = choices.get(name);
        if (choice) choice.description = `${LABELS[other]} · choosing moves it here. ${choice.description}`;
      }
    const notice =
      this.choices.kind === "loading"
        ? "Loading choices…"
        : this.choices.kind === "failed"
          ? `Choices unavailable. ${this.choices.error}`
          : field === "blueprints" && options?.blueprintsError
            ? `Blueprints unavailable. ${options.blueprintsError}`
            : "";
    return { choices: [...choices.values()], notice };
  }

  private refreshPicker(): void {
    if (this.picker !== null) this.picker.view.setChoices(this.pickerData(this.picker.field));
  }

  private openPicker(field: ChoiceField): void {
    this.picker = {
      field,
      view: new ProfileChoices({
        title: LABELS[field],
        ...this.pickerData(field),
        selected: this.selections[field],
        hint:
          field === "blueprints"
            ? "Choose the blueprints this profile expects."
            : field === "availableSkills"
              ? "Available when needed. Selecting moves skills out of always loaded."
              : "Loaded before the first response. Selecting moves skills out of available.",
        done: (selected) => {
          this.selections[field] = selected;
          if (field !== "blueprints") {
            const other = field === "availableSkills" ? "alwaysLoadedSkills" : "availableSkills";
            this.selections[other] = this.selections[other].filter((name) => !selected.includes(name));
          }
          this.picker = null;
        },
      }),
    };
  }

  private submit(): void {
    const name = this.name.getValue().trim();
    if (!name) {
      this.nameError = "Enter a name.";
      this.focused = 0;
      return;
    }
    this.error = "";
    this.saving = true;
    this.options.save({
      ...(this.options.profile === null ? {} : { id: this.options.profile.id }),
      name,
      description: this.description.getValue(),
      role: this.role.getExpandedText(),
      ...this.selections,
    });
  }
}
