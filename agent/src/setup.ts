import { access } from "node:fs/promises";
import { join, resolve } from "node:path";
import { stdin, stdout } from "node:process";
import {
  ModelRuntime,
  SettingsManager,
  getSelectListTheme,
  initTheme,
} from "@earendil-works/pi-coding-agent";
import {
  Container,
  ProcessTerminal,
  SelectList,
  Text,
  TuiMainScreen,
} from "@earendil-works/pi-tui";
import type { AuthInteraction, AuthPrompt } from "@earendil-works/pi-ai";
import { loadConfig, saveConfig, type Paths } from "./config.js";
import { ChatInput } from "./input.js";
import { service } from "./service.js";

/** Pi's login callbacks, with credentials confined to a disposable masked input. */
function setupUI(): AuthInteraction & { close(): void } {
  if (!stdin.isTTY)
    throw new Error(
      "Setup requires a terminal. For automation use dimcode setup --provider NAME --key-env VAR.",
    );
  initTheme("dark", false);
  const tui = new TuiMainScreen(new ProcessTerminal());
  const body = new Container(),
    info = new Text("", 1, 1);
  tui.addChild(
    new Text("\x1b[1;36mdimcode\x1b[0m · Set up your Dimensional agent", 1, 1),
  );
  tui.addChild(info);
  tui.addChild(body);
  tui.addChild(
    new Text("\x1b[2m↑ ↓ choose · Enter continue · Esc cancel\x1b[0m", 1, 1),
  );
  const abort = new AbortController();
  tui.addInputListener((data) => {
    if (data === "\x03") {
      abort.abort();
      return { consume: true };
    }
    return undefined;
  });
  tui.start();
  return {
    signal: abort.signal,
    prompt: async (question: AuthPrompt) => {
      abort.signal.throwIfAborted();
      question.signal?.throwIfAborted();
      body.clear();
      body.addChild(new Text(question.message, 1, 1));
      return new Promise<string>((resolve, reject) => {
        const cancel = () => finish(undefined);
        const input = new ChatInput(
          question.type === "select" ? "" : question.placeholder,
        );
        const finish = (value: string | undefined) => {
          abort.signal.removeEventListener("abort", cancel);
          question.signal?.removeEventListener("abort", cancel);
          input.secret = false;
          input.setValue("");
          body.clear();
          tui.requestRender();
          if (value === undefined)
            reject(
              new Error("Setup cancelled. Run dimcode setup to continue."),
            );
          else resolve(value);
        };
        abort.signal.addEventListener("abort", cancel, { once: true });
        question.signal?.addEventListener("abort", cancel, { once: true });
        if (question.type === "select") {
          const select = new SelectList(
            question.options.map((item) => ({
              value: item.id,
              label: item.label,
              description: item.description,
            })),
            10,
            getSelectListTheme(),
          );
          select.onSelect = (item) => finish(item.value);
          select.onCancel = cancel;
          body.addChild(select);
          tui.setFocus(select);
        } else {
          input.secret =
            question.type === "secret" || question.type === "manual_code";
          input.onSubmit = finish;
          input.onEscape = cancel;
          body.addChild(input);
          tui.setFocus(input);
        }
        tui.requestRender();
      });
    },
    notify: (event) => {
      info.setText(
        event.type === "auth_url"
          ? event.url +
              "\n" +
              (event.instructions ?? "Open this link to sign in.")
          : event.type === "device_code"
            ? event.verificationUri + " · " + event.userCode
            : event.message,
      );
      tui.requestRender();
    },
    close: () => tui.stop(),
  };
}
export async function needsSetup(paths: Paths): Promise<boolean> {
  try {
    await access(join(paths.config, "config.json"));
    return false;
  } catch (error) {
    if (error instanceof Error && "code" in error && error.code === "ENOENT")
      return true;
    throw error;
  }
}
export interface SetupOptions {
  provider?: string;
  oauth?: boolean;
  keyEnv?: string;
  workspace?: string;
}
export async function setup(
  paths: Paths,
  options: SetupOptions = {},
  interaction?: AuthInteraction,
): Promise<boolean> {
  const config = await loadConfig(paths);
  if (options.workspace) config.workspace = resolve(options.workspace);
  const runtime = await ModelRuntime.create({
    authPath: join(paths.config, "auth.json"),
    modelsPath: join(paths.config, "models.json"),
  });
  const wizard = !options.provider && !options.keyEnv && !options.oauth;
  const ui = interaction ?? (options.keyEnv ? undefined : setupUI());
  const ask = (question: AuthPrompt) => {
    if (!ui) throw new Error("Interactive setup requires a terminal");
    return ui.prompt(question);
  };
  const choose = (
    message: string,
    items: readonly { id: string; label: string; description?: string }[],
  ) => ask({ type: "select", message, options: items });
  let atLogin = false;
  try {
    const providers = runtime
      .getProviders()
      .filter(
        (p) => p.auth.apiKey?.login || (p.id !== "anthropic" && p.auth.oauth),
      );
    const preferred = ["openai", "openai-codex", "anthropic"];
    providers.sort(
      (a, b) =>
        (preferred.indexOf(a.id) + 1 || 99) -
        (preferred.indexOf(b.id) + 1 || 99),
    );
    const providerId =
      options.provider ??
      (wizard
        ? await choose(
            "1. Choose your model provider",
            providers.map((p) => ({
              id: p.id,
              label:
                p.id === "openai-codex"
                  ? "ChatGPT subscription (sign in)"
                  : p.name,
              description:
                p.id === "anthropic"
                  ? "Claude API key"
                  : p.id === "openai"
                    ? "OpenAI API key"
                    : p.id,
            })),
          )
        : "openai");
    const provider = runtime.getProvider(providerId);
    if (!provider) throw new Error("Unknown provider: " + providerId);
    if (providerId === "anthropic" && options.oauth)
      throw new Error("Use an Anthropic API key for dimcode.");
    const methods = [
      ...(wizard && runtime.hasConfiguredAuth(providerId)
        ? [{ id: "existing", label: "Use configured credentials" }]
        : []),
      ...(provider.auth.apiKey?.login
        ? [{ id: "api_key", label: provider.auth.apiKey.name }]
        : []),
      ...(providerId !== "anthropic" && provider.auth.oauth
        ? [{ id: "oauth", label: provider.auth.oauth.name }]
        : []),
    ];
    const method = options.keyEnv
      ? "api_key"
      : options.oauth
        ? "oauth"
        : wizard && methods.length > 1
          ? await choose("Sign in", methods)
          : methods[0]?.id;
    if (!method) throw new Error("Provider has no supported login method");
    if (method !== "existing") {
      if (options.keyEnv) {
        const key = process.env[options.keyEnv];
        if (!key)
          throw new Error("Requested credential environment variable is empty");
        await runtime.login(providerId, "api_key", {
          prompt: async () => key,
          notify: () => {},
        });
      } else {
        if (method !== "api_key" && method !== "oauth")
          throw new Error("Unknown login method");
        await runtime.login(providerId, method, ui!);
      }
    }
    const models = await runtime.getAvailable(providerId);
    if (!models.length)
      throw new Error("No available models for " + providerId);
    const settings = SettingsManager.create(config.workspace, paths.config);
    const currentModel =
      settings.getDefaultProvider() === providerId
        ? settings.getDefaultModel()
        : undefined;
    const ordered = [...models].sort(
      (a, b) => Number(b.id === currentModel) - Number(a.id === currentModel),
    );
    const model = wizard
      ? await choose(
          "2. Choose a model",
          ordered.map((m) => ({ id: m.id, label: m.id, description: m.name })),
        )
      : ordered[0].id;
    if (wizard) {
      if (process.platform === "linux")
        atLogin =
          (await choose("3. Start the gateway at login?", [
            { id: "no", label: "Start when I open dimcode" },
            {
              id: "yes",
              label: "Start at login",
              description:
                "Keep dimcode available in the background; no robot starts",
            },
          ])) === "yes";
    }
    settings.setDefaultModelAndProvider(providerId, model);
    await settings.flush();
    await saveConfig(paths, config);
  } finally {
    if (ui && "close" in ui && typeof ui.close === "function") ui.close();
  }
  if (atLogin) await service("install", config, paths, process.argv[1]);
  stdout.write(
    wizard ? "Opening agent-led DimOS setup…\n" : "Provider configured.\n",
  );
  return wizard;
}
