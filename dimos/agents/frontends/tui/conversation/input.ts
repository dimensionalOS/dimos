import type { AutocompleteSuggestions, Component, TUI } from "@earendil-works/pi-tui";
import { CombinedAutocompleteProvider, Editor, Input, matchesKey, truncateToWidth } from "@earendil-works/pi-tui";
import { accent } from "#dimos/agents/frontends/tui/style.ts";
import { getSelectListTheme } from "@earendil-works/pi-coding-agent";

/** A one-line input for dialogs; masked, it drops its undo and kill-ring history on every mode change. */
export class ChatInput implements Component {
  private readonly placeholder: string;
  private editor: Input;
  private masked = false;
  focused = false;
  onSubmit?: (value: string) => void;
  onEscape?: () => void;

  constructor(placeholder = "") {
    this.placeholder = placeholder;
    this.editor = new Input({ prompt: "› ", placeholder });
  }

  get secret(): boolean {
    return this.masked;
  }

  set secret(value: boolean) {
    if (value === this.masked) return;
    this.masked = value;
    this.editor = new Input({ prompt: "› ", placeholder: this.placeholder });
  }

  getValue(): string {
    return this.editor.getValue();
  }

  setValue(value: string): void {
    this.editor.setValue(value);
  }

  invalidate(): void {
    this.editor.invalidate();
  }

  handleInput(data: string): void {
    this.editor.focused = this.focused;
    this.editor.onSubmit = (value) => this.onSubmit?.(value);
    this.editor.onEscape = () => this.onEscape?.();
    this.editor.handleInput(data);
  }

  render(width: number): string[] {
    return this.secret
      ? [truncateToWidth(`credential › ${"*".repeat(this.getValue().length)}`, width)]
      : this.editor.render(width);
  }
}

class FileAutocomplete extends CombinedAutocompleteProvider {
  override async getSuggestions(
    lines: string[],
    cursorLine: number,
    cursorCol: number,
    options: Parameters<CombinedAutocompleteProvider["getSuggestions"]>[3],
  ): Promise<AutocompleteSuggestions | null> {
    const beforeCursor = (lines[cursorLine] ?? "").slice(0, cursorCol);
    const prefix = /(?:^|[\s([{<`])(@(?:"[^"]*"?|[^\s"()[\]{}<>`]*))$/u.exec(beforeCursor)?.[1];
    if (prefix === undefined) return super.getSuggestions(lines, cursorLine, cursorCol, options);

    // Quoted normal-path lookup works without the fd dependency of pi-tui's fuzzy @ lookup.
    const quoted = prefix.startsWith('@"');
    if (quoted && prefix.length > 2 && prefix.endsWith('"')) return null;
    const query = `"${prefix.slice(quoted ? 2 : 1)}`;
    const suggestions = await super.getSuggestions([query], 0, query.length, { ...options, force: true });
    if (suggestions === null) return null;
    return {
      prefix,
      items: suggestions.items.map((item) => {
        const path = item.value.slice(1, -1);
        return { ...item, value: quoted || /[\s"'=()[\]{}<>`]/u.test(path) ? item.value : path };
      }),
    };
  }
}

/**
 * pi-tui's Editor with slash-command and file autocomplete, an Esc hook and a masked secret mode in which keys go
 * to a disposable ChatInput. Moved from dimcode input.ts.
 */
export class PromptEditor extends Editor {
  private readonly commands: readonly string[];
  private readonly credential = new ChatInput();
  secret = false;
  onEscape?: () => void;

  constructor(tui: TUI, commands: readonly string[], cwd: string) {
    super(tui, { selectList: getSelectListTheme(), borderColor: accent }, { paddingX: 1 });
    this.commands = commands;
    this.setWorkspace(cwd);
  }

  /** File completion follows the attached conversation's directory. */
  setWorkspace(cwd: string): void {
    this.setAutocompleteProvider(
      new FileAutocomplete(
        this.commands.map((name) => ({ name })),
        cwd,
      ),
    );
  }

  getValue(): string {
    return this.secret ? this.credential.getValue() : this.getExpandedText();
  }

  setValue(value: string): void {
    this.credential.secret = this.secret;
    this.credential.setValue(value);
    if (!this.secret) this.setText(value);
  }

  override handleInput(data: string): void {
    if (matchesKey(data, "escape") && !this.isShowingAutocomplete()) {
      this.onEscape?.();
      return;
    }
    if (this.secret) {
      this.credential.onSubmit = (value) => this.onSubmit?.(value);
      this.credential.handleInput(data);
    } else super.handleInput(data);
  }

  override render(width: number): string[] {
    return this.secret ? this.credential.render(width) : super.render(width);
  }
}
