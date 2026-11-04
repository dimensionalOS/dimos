import { owner } from "#dimos/agents/sdk/failures.ts";
import type { Component } from "@earendil-works/pi-tui";
import type { LoginState, ModelChoice } from "#dimos/agents/shared/protocol/auth.ts";
import type { Outcome } from "#dimos/agents/shared/protocol/outcome.ts";
import type { Ui } from "#dimos/agents/frontends/tui/session.ts";
import {
  Container,
  ProcessTerminal,
  Spacer,
  Text,
  TuiMainScreen,
  isKeyRelease,
  matchesKey,
} from "@earendil-works/pi-tui";
import {
  DynamicBorder,
  ExtensionSelectorComponent,
  OAuthSelectorComponent,
  ThemeSelectorComponent,
  initTheme,
} from "@earendil-works/pi-coding-agent";
import { LoginPanel } from "#dimos/agents/frontends/tui/dialogs.ts";
import { accent, muted } from "#dimos/agents/frontends/tui/style.ts";
import { refusalText } from "#dimos/agents/shared/view-models/outcome.ts";
import { until } from "#dimos/agents/sdk/until.ts";

type SetupResult = { readonly kind: "done" } | { readonly kind: "cancelled" };

class Cancelled extends Error {}

type Step = { readonly view: Component; readonly focus: Component; dispose(): void };

type LoginEnd = Extract<LoginState, { readonly phase: "done" | "failed" }>;

const MODELS_WAIT_MS = 5_000;

/** One screen of setup at a time on the main screen; Ctrl-C cancels whichever step is open. */
class SetupScreen {
  readonly tui = new TuiMainScreen(new ProcessTerminal());
  private readonly body = new Container();
  private cancel: (() => void) | null = null;
  /** Set by Ctrl-C, so a login the gateway ends with an error still reads as a cancelled setup. */
  cancelled = false;

  constructor(theme: string) {
    initTheme(theme, false);
    this.tui.addChild(new DynamicBorder(accent));
    this.tui.addChild(
      new Text(
        `${accent("dimos agent setup")}\n${muted("A model provider, its login and a default model. Run /login later to change them.")}`,
        1,
        1,
      ),
    );
    this.tui.addChild(this.body);
    this.tui.addInputListener((data) => {
      if (!matchesKey(data, "ctrl+c") || isKeyRelease(data)) return undefined;
      this.cancelled = true;
      this.cancel?.();
      return { consume: true };
    });
    this.tui.start();
  }

  preview(theme: string): void {
    initTheme(theme, false);
    this.tui.invalidate();
    this.tui.requestRender();
  }

  /** Shows the step `open` builds and resolves with what it finishes with; a cancelled step rejects with Cancelled. */
  ask<T>(open: (finish: (value: T | null) => void) => Step): Promise<T> {
    return new Promise<T>((resolve, reject) => {
      let step: Step | null = null;
      let finished = false;
      const finish = (value: T | null) => {
        if (finished) return;
        finished = true;
        this.cancel = null;
        step?.dispose();
        this.body.clear();
        this.tui.requestRender();
        if (value === null) reject(new Cancelled("Setup cancelled"));
        else resolve(value);
      };
      this.cancel = () => finish(null);
      step = open(finish);
      this.body.clear();
      this.body.addChild(new Spacer(1));
      this.body.addChild(step.view);
      this.tui.setFocus(step.focus);
      this.tui.requestRender();
    });
  }

  close(): void {
    this.tui.stop();
  }
}

async function settled<T>(call: Promise<Outcome<T>>): Promise<T> {
  const outcome = await call;
  if (!outcome.ok) throw new Error(refusalText(outcome.refusal));
  return outcome.value;
}

function select(
  screen: SetupScreen,
  title: string,
  options: readonly { readonly id: string; readonly label: string }[],
): Promise<string> {
  return screen.ask<string>((finish) => {
    const labels = options.map((option) => option.label);
    const view = new ExtensionSelectorComponent(
      title,
      labels,
      (label) => finish(options[labels.indexOf(label)]?.id ?? null),
      () => finish(null),
      {},
    );
    return { view, focus: view, dispose: () => view.dispose() };
  });
}

/**
 * Starts `login` and shows this terminal's prompts until its flow is done or failed. The flow is a member, not the
 * call's result, so the panel shows each prompt and the device code as the gateway publishes them.
 */
async function login(ui: Ui, screen: SetupScreen, provider: string, method: "api_key" | "oauth"): Promise<void> {
  const flow = ui.gateway.loginFlow;
  const before = flow.value;
  const panel = new LoginPanel(screen.tui, ui);
  const stop = flow.subscribe(() => screen.tui.requestRender());
  try {
    const end = await screen.ask<LoginEnd | { readonly phase: "refused"; readonly text: string }>((finish) => {
      void until(flow, (state) =>
        state !== before && (state.phase === "done" || state.phase === "failed") ? state : undefined,
      ).then(finish);
      void ui.actions
        .observe(`log in to ${provider}`, owner(ui.gateway).login({ provider, method }))
        .then((outcome) => {
          if (!outcome.ok) finish({ phase: "refused", text: refusalText(outcome.refusal) });
        });
      return { view: panel, focus: panel, dispose: () => panel.dispose() };
    });
    if (end.phase === "failed") throw new Error(`Login to ${provider} failed: ${end.error}`);
    if (end.phase === "refused") throw new Error(end.text);
  } catch (error) {
    const open = flow.value;
    if (open.phase === "running" && open.prompt !== null)
      await ui.actions.observe(
        "answer the login prompt",
        owner(ui.gateway).answerLogin({ dialog: open.prompt.dialog, value: null }),
      );
    throw error;
  } finally {
    stop();
  }
}

/** The provider's usable models once the accounts member lists them; a login reaches `accounts` after its flow ends. */
async function availableModels(ui: Ui, provider: string): Promise<readonly ModelChoice[]> {
  try {
    return await until(
      ui.gateway.accounts,
      (accounts) => {
        const usable = accounts.models.filter((model) => model.provider === provider && model.available);
        return usable.length > 0 ? usable : undefined;
      },
      AbortSignal.timeout(MODELS_WAIT_MS),
    );
  } catch {
    throw new Error(`No models are available for ${provider}`);
  }
}

/** First run: a theme, a provider and its login, and a default model, each one gateway action. */
export async function runSetup(ui: Ui): Promise<SetupResult> {
  const current = ui.gateway.settings.value.theme ?? "dark";
  const screen = new SetupScreen(current);
  try {
    const theme = await screen.ask<string>((finish) => {
      const view = new ThemeSelectorComponent(
        current,
        finish,
        () => finish(null),
        (name) => screen.preview(name),
      );
      return { view, focus: view.getSelectList(), dispose: () => {} };
    });
    await settled(ui.actions.observe("change settings", owner(ui.gateway).updateSettings({ theme })));
    const providers = ui.gateway.accounts.value.providers;
    const chosen = await screen.ask<{ readonly provider: string; readonly method: "api_key" | "oauth" }>((finish) => {
      const view = new OAuthSelectorComponent(
        "login",
        providers.map((choice) => ({
          id: choice.provider,
          name: choice.name,
          authType: choice.method,
          subscription: choice.subscription,
          ...(choice.status === null ? {} : { status: choice.status }),
        })),
        (provider, method) => finish({ provider, method }),
        () => finish(null),
      );
      return { view, focus: view, dispose: () => {} };
    });
    const configured = providers.some(
      (choice) => choice.provider === chosen.provider && choice.method === chosen.method && choice.status !== null,
    );
    if (!configured) await login(ui, screen, chosen.provider, chosen.method);
    const models = await availableModels(ui, chosen.provider);
    const modelId = await select(
      screen,
      "Default model for new conversations",
      models.map((model) => ({
        id: model.id,
        label: `${model.id}${model.name === model.id ? "" : ` · ${model.name}`}`,
      })),
    );
    await settled(
      ui.actions.observe(
        "change settings",
        owner(ui.gateway).updateSettings({
          model: { provider: chosen.provider, modelId },
          thinkingLevel: null,
        }),
      ),
    );
    return { kind: "done" };
  } catch (error) {
    if (error instanceof Cancelled || screen.cancelled) return { kind: "cancelled" };
    throw error;
  } finally {
    screen.close();
  }
}
