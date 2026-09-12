import { Input, truncateToWidth, type Component } from "@earendil-works/pi-tui";

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
