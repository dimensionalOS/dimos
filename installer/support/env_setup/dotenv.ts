#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../dax.ts"

import { dimosEnvVars } from "../constants.ts"
import { addGitIgnorePatterns, getProjectDirectory } from "../misc.ts"
import * as p from "../prompt_tools.ts"

export async function setupDotenv(projectPath: string, envPath: string): Promise<boolean> {
    let envExists = false
    try {
        const st = await Deno.stat(envPath)
        envExists = st.isFile
    } catch {
        envExists = false
    }

    if (!envExists) {
        console.log(`Dimos involves setting several project specific environment variables.\nWe highly recommend having these in a git-ignored ${p.highlight(`.env`)} file.\n`)
        if (!p.askYesNo(`I don't see a ${p.highlight(`.env`)} file, can I create one for you?`)) {
            console.log("- Okay, I'll assume you will manage env vars yourself:")
            for (const [name, value] of Object.entries(dimosEnvVars)) {
                console.log(`  ${name}=${value}`)
            }
            // skip rest of phase (no .envrc if they won't even make a .env)
            return false
        }
        await Deno.writeTextFile(envPath, "")
        await addGitIgnorePatterns(projectPath, ["/.env"], { comment: "Added by dimos setup" })
    }

    let envText = ""
    try {
        envText = await Deno.readTextFile(envPath)
    } catch {
        envText = ""
    }
    const existingVarsInDotEnv = new Set(
        envText
            .split("\n")
            .map((line) => line.trim())
            .map((line) => line.match(/^(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=/)?.[1])
            .filter((name) => !!name),
    )

    const missingEnvVars: string[] = []
    for (const [name, value] of Object.entries(dimosEnvVars)) {
        if (!existingVarsInDotEnv.has(name)) {
            missingEnvVars.push(`${name}=${value}`)
        }
    }

    if (missingEnvVars.length > 0) {
        const needsTrailingNewline = envText.length > 0 && !envText.endsWith("\n")
        const additions = (needsTrailingNewline ? "\n" : "") + missingEnvVars.join("\n") + "\n"
        await Deno.writeTextFile(envPath, envText + additions)
        p.boringLog(`- appended ${missingEnvVars.length} env var(s) to .env`)
    } else {
        p.boringLog("- all required env vars already exist in .env")
    }

    return true
}
