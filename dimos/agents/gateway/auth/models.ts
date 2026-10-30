import type { Accounts, ModelChoice, ProviderChoice } from "../../shared/protocol/auth.ts";
import type { Models, Provider } from "@earendil-works/pi-ai";
import type { Live } from "../../shared/protocol/status.ts";
import type { Outcome } from "../../shared/protocol/outcome.ts";
import { InMemoryModelsStore } from "@earendil-works/pi-ai";
import { ModelRuntime } from "@earendil-works/pi-coding-agent";
import { Status } from "../../shared/protocol/status.ts";
import { done, refused } from "../../shared/protocol/outcome.ts";

/**
 * Every built-in pi-ai provider, then `extra` (the faux provider in tests), with credentials in `authFile` through
 * pi-coding-agent's AuthStorage, the store that wrote dimcode's auth.json. `modelsFile`, when it exists, overrides
 * providers in pi's models.json format. A malformed one leaves the providers as built in and goes to `report`. No
 * catalog download, and the catalog stays in memory.
 */
export async function openModels(
  authFile: string,
  modelsFile: string,
  extra: readonly Provider[],
  report: (error: unknown) => void,
): Promise<Models> {
  const models = await ModelRuntime.create({
    authPath: authFile,
    modelsPath: modelsFile,
    modelsStore: new InMemoryModelsStore(),
  });
  // pi folds a malformed models.json in with credential and catalog failures, so none of them stops the gateway.
  const error = models.getError();
  if (error !== undefined) report(new Error(error));
  for (const provider of extra) models.registerNativeProvider(provider);
  return models;
}

/** Login methods a terminal offers: an interactive API key, or OAuth except Anthropic's. */
function methodsOf(provider: Provider): readonly ProviderChoice["method"][] {
  return [
    ...(provider.auth.apiKey?.login === undefined ? [] : (["api_key"] as const)),
    ...(provider.auth.oauth === undefined || provider.id === "anthropic" ? [] : (["oauth"] as const)),
  ];
}

const PREFERRED = ["openai", "openai-codex", "anthropic"];

function preference(provider: Provider): number {
  const index = PREFERRED.indexOf(provider.id);
  return index < 0 ? PREFERRED.length : index;
}

/** Every provider login a terminal may start, preferred providers first. */
export async function providerChoices(models: Models): Promise<readonly ProviderChoice[]> {
  const providers = models
    .getProviders()
    .filter((provider) => methodsOf(provider).length > 0)
    .sort((a, b) => preference(a) - preference(b));
  const choices = await Promise.all(
    providers.map(async (provider) => {
      const status = (await models.checkAuth(provider.id).catch(() => undefined)) ?? null;
      return methodsOf(provider).map(
        (method): ProviderChoice => ({
          provider: provider.id,
          name: provider.id === "openai-codex" ? "ChatGPT subscription" : provider.name,
          method,
          subscription: method === "oauth" && provider.auth.oauth?.isSubscription !== false,
          status,
        }),
      );
    }),
  );
  return choices.flat();
}

/** Chat models of the providers a terminal may log in to, and of providers configured without a login, available ones first. */
export async function modelChoices(models: Models): Promise<readonly ModelChoice[]> {
  const available = new Set((await models.getAvailable()).map((model) => `${model.provider}\u0000${model.id}`));
  const offered = models.getProviders().filter((provider) => methodsOf(provider).length > 0);
  const choices = models
    .getModels()
    .filter(
      (model) =>
        offered.some((provider) => provider.id === model.provider) ||
        available.has(`${model.provider}\u0000${model.id}`),
    )
    .map(
      (model): ModelChoice => ({
        provider: model.provider,
        id: model.id,
        name: model.name,
        available: available.has(`${model.provider}\u0000${model.id}`),
        reasoning: model.reasoning,
        contextWindow: model.contextWindow,
      }),
    );
  return choices.sort((a, b) => Number(b.available) - Number(a.available));
}

/** The providers and models as of the last login or logout: the `accounts` member. */
export class ModelCatalog {
  private readonly accounts = new Status<Accounts>({ providers: [], models: [] }, (error) => {
    throw error;
  });
  readonly status: Live<Accounts> = this.accounts;
  private readonly models: Models;

  constructor(models: Models) {
    this.models = models;
  }

  async refresh(): Promise<void> {
    const [providers, models] = await Promise.all([providerChoices(this.models), modelChoices(this.models)]);
    this.accounts.set({ providers, models });
  }

  async logout(provider: string): Promise<Outcome<null>> {
    if (this.models.getProvider(provider) === undefined)
      return refused({ code: "not_found", what: "provider", name: provider });
    await this.models.logout(provider);
    await this.refresh();
    return done(null);
  }
}
