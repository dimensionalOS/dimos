#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.js"

import { RenderLogo } from "../support/dimos_banner.js"
import { getToolCheckResults, type ToolResult } from "../support/get_tool_check_results.ts"
import { activateVenv, getVenvDirsAt } from "../support/venv.js"
import { dependencyListHumanNames, dependencyListAptPackages, discordUrl } from "../support/constants.ts"
import { mentionSystemDependencies, parseVersion, isVersionAtLeast, detectPythonCommand, ensureGitAndLfs, ensurePortAudio, ensurePython, aptInstall, getProjectDirectory, addGitIgnorePatterns } from "../support/misc.ts"
import * as p from "../support/prompt_tools.js"

//
// Phase 3: Pip install dimos
//
export async function phase3() {
    p.clearScreen()
    p.header("Phase 3: Pip installing dimos")
    const res = await $$`pip install 'dimos @ git+https://github.com/dimensionalOS/dimos.git'`.printCommand()
    if (res.code != 0) {
        console.log(``)
        p.error(`Failed to pip install dimos 😕\nPlease message us in our discord and we'll help you get it installed!:\n    ${discordUrl}`)
        Deno.exit(1)
    }
}
