import type { AuthEvent } from "@earendil-works/pi-ai";
import type { Component, Focusable, TUI } from "@earendil-works/pi-tui";
import type { DialogId, DialogPrompt, LoginState } from "../../shared/protocol/auth.ts";
import type { Ui } from "./session.ts";
import { ChatInput } from "./conversation/input.ts";
import { ExtensionSelectorComponent } from "@earendil-works/pi-coding-agent";
import { Text, truncateToWidth } from "@earendil-works/pi-tui";
import { accent, muted } from "./style.ts";
import { clean } from "../../shared/view-models/text.ts";

type OpenPrompt = {
  readonly dialog: DialogId;
  readonly view: Component;
  readonly input: ChatInput | null;
  dispose(): void;
};

/** An auth_url, device_code, info or progress event as the lines above the prompt. */
function infoLines(info: AuthEvent): readonly string[] {
  switch (info.type) {
    case "auth_url":
      return [info.url, info.instructions ?? "Open this link to sign in."];
    case "device_code":
      return [`Open ${info.verificationUri} and enter ${info.userCode}`];
    case "info":
    case "progress":
      return [info.message];
  }
}

/**
 * The login prompt in the editor slot, drawn from this terminal's `loginFlow` member when it renders: a list for
 * select, a masked line for secret and manual_code, a line for text, and the newest progress lines above it, so a
 * device code stays visible. An answer is `gateway.answerLogin`; Esc answers null, which cancels the flow. Chat and
 * setup both show it.
 */
export class LoginPanel implements Component, Focusable {
  private readonly tui: TUI;
  private readonly ui: Ui;
  private open: OpenPrompt | null = null;
  /** The dialog this panel already answered, so the prompt is not drawn again while the gateway moves on. */
  private answered: DialogId | null = null;
  private hasFocus = false;

  constructor(tui: TUI, ui: Ui) {
    this.tui = tui;
    this.ui = ui;
  }

  get focused(): boolean {
    return this.hasFocus;
  }

  set focused(value: boolean) {
    this.hasFocus = value;
    if (this.open !== null && this.open.input !== null) this.open.input.focused = value;
  }

  render(width: number): string[] {
    const state = this.ui.gateway.loginFlow.value;
    this.sync(state);
    if (state.phase !== "running") return [];
    const lines = [truncateToWidth(accent(` Sign in to ${clean(state.provider)}`), width)];
    for (const info of state.info)
      for (const line of infoLines(info)) lines.push(...new Text(muted(clean(line)), 1, 0).render(width));
    if (this.open === null) lines.push(truncateToWidth(muted(" Waiting for approval…"), width));
    else lines.push(...this.open.view.render(width));
    return lines;
  }

  invalidate(): void {
    this.open?.view.invalidate();
  }

  handleInput(data: string): void {
    this.open?.view.handleInput?.(data);
    this.tui.requestRender();
  }

  dispose(): void {
    this.open?.dispose();
    this.open = null;
  }

  private sync(state: LoginState): void {
    const prompt = state.phase === "running" ? state.prompt : null;
    const wanted = prompt === null || prompt.dialog === this.answered ? null : prompt;
    if (wanted?.dialog === this.open?.dialog) return;
    this.open?.dispose();
    this.open = wanted === null ? null : this.promptView(wanted.dialog, wanted.prompt);
  }

  private reply(dialog: DialogId, value: string | null): void {
    this.answered = dialog;
    this.open?.dispose();
    this.open = null;
    if (this.ui.gateway.variant !== "owner") {
      this.ui.actions.refuse("answer the login prompt", { code: "viewing", conversation: null });
      return;
    }
    void this.ui.actions.observe("answer the login prompt", this.ui.gateway.answerLogin({ dialog, value }));
    this.tui.requestRender();
  }

  private promptView(dialog: DialogId, prompt: DialogPrompt): OpenPrompt {
    if (prompt.type === "select") {
      const labels = prompt.options.map(
        (option) => option.label + (option.description === undefined ? "" : ` · ${option.description}`),
      );
      const selector = new ExtensionSelectorComponent(
        prompt.message,
        labels,
        (label) => this.reply(dialog, prompt.options[labels.indexOf(label)]?.id ?? null),
        () => this.reply(dialog, null),
        { tui: this.tui },
      );
      return { dialog, view: selector, input: null, dispose: () => selector.dispose() };
    }
    const input = new ChatInput(prompt.placeholder ?? "");
    input.secret = prompt.type === "secret" || prompt.type === "manual_code";
    input.focused = this.hasFocus;
    input.onSubmit = (value) => this.reply(dialog, value);
    input.onEscape = () => this.reply(dialog, null);
    const message = new Text(accent(clean(prompt.message)), 1, 0);
    const hint = new Text(muted("Enter continue · Esc cancel"), 1, 0);
    const view: Component = {
      render: (width) => [...message.render(width), ...input.render(width), ...hint.render(width)],
      invalidate: () => input.invalidate(),
      handleInput: (data) => input.handleInput(data),
    };
    return {
      dialog,
      view,
      input,
      dispose: () => {
        input.secret = false;
        input.setValue("");
      },
    };
  }
}
