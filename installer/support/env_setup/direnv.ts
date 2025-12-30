#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../dax.ts"

import * as p from "../prompt_tools.ts"

export async function setupDirenv(envrcPath: string) {
    if (!await $.commandExists("direnv")) {
        p.boringLog("- direnv not detected; skipping .envrc setup")
        p.subHeader(`- In the future don't forget to: ${p.highlight(`source ${Deno.env.get("VIRTUAL_ENV")||"venv"}/bin/activate`)}\n  (each time you create a new terminal and cd to the project)`)
        return
    }

    let envrcText = ""
    let envrcExists = false
    try {
        const st = await Deno.stat(envrcPath)
        envrcExists = st.isFile
    } catch {
        envrcExists = false
    }

    if (!envrcExists) {
        console.log(`direnv detected but no .envrc found.`)
        if (!p.askYesNo("Create one to auto-activate the virtual environment?")) {
            p.boringLog("- skipping .envrc creation")
            return
        }
        await Deno.writeTextFile(envrcPath, envrcText)
        p.boringLog("- created .envrc")
    } else {
        envrcText = await Deno.readTextFile(envrcPath)
    }

    const hasVenvActivation = envrcText.match(/(^|;)\s*(source|\.)\s+.*[v]?env.*\/bin\/activate/i)
    if (!hasVenvActivation) {
        const addVenvActivation = p.askYesNo("It looks like there is a .envrc file, but I don't see a python virtual environment activation in there. Is it okay if I add a python virtual env activation to the .envrc?")
        if (addVenvActivation) {
            const block = [
                "for venv in venv .venv env; do",
                "  if [[ -f \"$venv/bin/activate\" ]]; then",
                "    . \"$venv/bin/activate\"",
                "    break",
                "  fi",
                "done",
            ].join("\n")
            const needsNewline = envrcText.length > 0 && !envrcText.endsWith("\n")
            envrcText = envrcText + (needsNewline ? "\n" : "") + block + "\n"
            await Deno.writeTextFile(envrcPath, envrcText)
            p.boringLog("- added venv activation to .envrc")
        }
    }

    const hasDotenv = /\bdotenv_if_exists\b/.test(envrcText)
    if (!hasDotenv) {
        console.log(`I don't see ${p.highlight(`dotenv_if_exists`)} in .envrc.`)
        if (p.askYesNo(`Can I add it so the .env file is loaded automatically?`)) {
            const needsNewline = envrcText.length > 0 && !envrcText.endsWith("\n")
            envrcText = envrcText + (needsNewline ? "\n" : "") + "dotenv_if_exists\n"
            await Deno.writeTextFile(envrcPath, envrcText)
            p.boringLog("- added dotenv_if_exists to .envrc")
        }
    }
}
