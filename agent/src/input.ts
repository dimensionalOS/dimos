import {
  CombinedAutocompleteProvider,
  Editor,
  Input,
  matchesKey,
  truncateToWidth,
  type Component,
  type TUI,
} from "@earendil-works/pi-tui";
import { getSelectListTheme } from "@earendil-works/pi-coding-agent";
import { accent } from "./terminal-style.js";

export const commands = [
  "help",
  "new",
  "sessions",
  "resume",
  "models",
  "model",
  "login",
  "logout",
  "abort",
  "steer",
  "follow",
  "reload",
  "image",
  "inspect",
  "view",
  "panel",
  "expand",
  "exit",
];

/** Pi owns editing, multiline paste, history and file/command completion. */
export class PromptEditor extends Editor {
  private credential = new ChatInput();
  secret = false;
  onEscape?: () => void;
  constructor(tui: TUI) {
    super(
      tui,
      { selectList: getSelectListTheme(), borderColor: accent },
      { paddingX: 1 },
    );
  }
  setWorkspace(cwd: string): void {
    this.setAutocompleteProvider(
      new CombinedAutocompleteProvider(
        commands.map((name) => ({ name })),
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
    if (this.secret) return this.credential.render(width);
    return super.render(width);
  }
}

export class ChatInput implements Component {
  private editor = new Input({
    prompt: "dimcode › ",
    placeholder: "Ask about your robot or app…",
  });
  private masked = false;
  focused = false;
  onSubmit?: (value: string) => void;
  onEscape?: () => void;
  get secret(): boolean {
    return this.masked;
  }
  set secret(value: boolean) {
    if (value === this.masked) return;
    this.masked = value;
    // Discard the credential editor and its undo/kill-ring history on mode change.
    this.editor = new Input({ prompt: "dimcode › " });
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
      ? [
          truncateToWidth(
            "credential › " + "*".repeat(this.getValue().length),
            width,
          ),
        ]
      : this.editor.render(width);
  }
}
