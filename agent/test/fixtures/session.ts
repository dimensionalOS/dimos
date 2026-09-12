import {
  createAgentSessionServices,
  createAgentSessionFromServices,
  type ExtensionFactory,
} from "@earendil-works/pi-coding-agent";
import {
  createAssistantMessageEventStream,
  type AssistantMessage,
} from "@earendil-works/pi-ai";
import { Type } from "typebox";
import type { SessionFactory } from "../../src/gateway.js";

export function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((done) => {
    resolve = done;
  });
  return { promise, resolve };
}
export function fixtureFactory(
  wait: Promise<void>,
  started: () => void,
): SessionFactory {
  return async (_config, paths, manager) => {
    const extension: ExtensionFactory = (pi) => {
      pi.registerTool({
        name: "wait_for_test",
        label: "Wait",
        description: "Deterministic test operation",
        parameters: Type.Object({}),
        execute: async (_id, _args, signal) => {
          started();
          await Promise.race([
            wait,
            new Promise<void>((_, reject) =>
              signal?.addEventListener(
                "abort",
                () => reject(new Error("aborted")),
                { once: true },
              ),
            ),
          ]);
          return {
            content: [{ type: "text", text: "operation finished" }],
            details: {},
          };
        },
      });
      pi.registerProvider("dimcode-fixture", {
        baseUrl: "http://localhost.invalid",
        apiKey: "nonsecret-fixture",
        api: "openai-completions",
        models: [
          {
            id: "fixture",
            name: "fixture",
            reasoning: false,
            input: ["text"],
            cost: { input: 0, output: 0, cacheRead: 0, cacheWrite: 0 },
            contextWindow: 32000,
            maxTokens: 100,
          },
        ],
        streamSimple: (model, context) => {
          const stream = createAssistantMessageEventStream();
          queueMicrotask(() => {
            const done = context.messages.at(-1)?.role === "toolResult";
            const message: AssistantMessage = {
              role: "assistant",
              api: model.api,
              provider: model.provider,
              model: model.id,
              content: done
                ? [{ type: "text", text: "done" }]
                : [
                    {
                      type: "toolCall",
                      id: "call-" + context.messages.length,
                      name: "wait_for_test",
                      arguments: {},
                    },
                  ],
              stopReason: done ? "stop" : "toolUse",
              timestamp: Date.now(),
              usage: {
                input: 0,
                output: 0,
                cacheRead: 0,
                cacheWrite: 0,
                totalTokens: 0,
                cost: {
                  input: 0,
                  output: 0,
                  cacheRead: 0,
                  cacheWrite: 0,
                  total: 0,
                },
              },
            };
            stream.push({ type: "start", partial: message });
            stream.push({
              type: "done",
              reason: done ? "stop" : "toolUse",
              message,
            });
            stream.end();
          });
          return stream;
        },
      });
    };
    const services = await createAgentSessionServices({
      cwd: manager.getCwd(),
      agentDir: paths.config,
      resourceLoaderOptions: {
        extensionFactories: [extension],
        noExtensions: true,
        noSkills: true,
      },
    });
    const { session } = await createAgentSessionFromServices({
      services,
      sessionManager: manager,
      model: services.modelRuntime.getModel("dimcode-fixture", "fixture"),
    });
    await session.bindExtensions({ mode: "rpc" });
    return {
      session,
      close: async () => {
        session.dispose();
      },
    };
  };
}
