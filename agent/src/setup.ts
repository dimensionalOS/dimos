import { mkdir } from "node:fs/promises";
import { join } from "node:path";
import { createInterface } from "node:readline/promises";
import { stdin, stdout } from "node:process";
import { ModelRuntime } from "@earendil-works/pi-coding-agent";
import type { AuthPrompt } from "@earendil-works/pi-ai";
import { loadConfig, saveConfig, type Paths } from "./config.js";

export async function prompt(prompt: AuthPrompt): Promise<string> {
  if (!stdin.isTTY)
    throw new Error(
      "Interactive login requires a terminal; use setup --key-env for an environment API key.",
    );
  if (prompt.type === "select")
    stdout.write(
      prompt.options
        .map((option) => option.id + ": " + option.label)
        .join("\n") + "\n",
    );
  if (prompt.type !== "secret" && prompt.type !== "manual_code") {
    const rl = createInterface({ input: stdin, output: stdout });
    try {
      return await rl.question(prompt.message + " ");
    } finally {
      rl.close();
    }
  }
  stdout.write(prompt.message + " ");
  const wasRaw = stdin.isRaw;
  stdin.setRawMode(true);
  stdin.resume();
  try {
    return await new Promise<string>((resolve, reject) => {
      let value = "";
      const onData = (bytes: Buffer) => {
        for (const character of bytes.toString("utf8")) {
          if (character === "\x03" || character === "\x1b") {
            cleanup();
            reject(new Error("Cancelled"));
            return;
          }
          if (character === "\r" || character === "\n") {
            cleanup();
            resolve(value);
            return;
          }
          if (character === "\x7f" || character === "\b")
            value = value.slice(0, -1);
          else if (character >= " ") value += character;
        }
      };
      const cleanup = () => {
        stdin.off("data", onData);
        stdout.write("\n");
      };
      stdin.on("data", onData);
    });
  } finally {
    stdin.setRawMode(wasRaw);
    stdin.pause();
  }
}
export async function setup(
  paths: Paths,
  options: {
    provider?: string;
    oauth?: boolean;
    keyEnv?: string;
    workspace?: string;
  },
): Promise<void> {
  const config = await loadConfig(paths);
  if (options.workspace) config.workspace = options.workspace;
  await saveConfig(paths, config);
  await mkdir(paths.config, { recursive: true, mode: 0o700 });
  const runtime = await ModelRuntime.create({
    authPath: join(paths.config, "auth.json"),
    modelsPath: join(paths.config, "models.json"),
  });
  const provider = options.provider ?? "openai";
  if (provider === "anthropic" && options.oauth)
    throw new Error("Use an Anthropic API key for dimcode.");
  if (options.keyEnv) {
    const key = process.env[options.keyEnv];
    if (!key)
      throw new Error("Requested credential environment variable is empty");
    await runtime.login(provider, "api_key", {
      prompt: async () => key,
      notify: () => {},
    });
  } else {
    await runtime.login(provider, options.oauth ? "oauth" : "api_key", {
      prompt,
      notify: (event) => {
        stdout.write(
          (event.type === "auth_url"
            ? event.url
            : event.type === "device_code"
              ? event.verificationUri + " code: " + event.userCode
              : event.message) + "\n",
        );
      },
    });
  }
  stdout.write(
    "dimcode configured. Run dimcode to chat, or dimcode service install to start it at login.\n",
  );
}
