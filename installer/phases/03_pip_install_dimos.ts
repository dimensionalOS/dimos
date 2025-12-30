#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import { RenderLogo } from "../support/dimos_banner.ts"
import { getToolCheckResults, type ToolResult } from "../support/get_tool_check_results.ts"
import { activateVenv, getVenvDirsAt } from "../support/venv.ts"
import { dependencyListHumanNames, dependencyListAptPackages, discordUrl } from "../support/constants.ts"
import { mentionSystemDependencies, parseVersion, isVersionAtLeast, detectPythonCommand, ensureGitAndLfs, ensurePortAudio, ensurePython, aptInstall, getProjectDirectory, addGitIgnorePatterns } from "../support/misc.ts"
import * as p from "../support/prompt_tools.ts"

export async function phase3() {
    p.clearScreen()
    p.header("Next Phase: Pip Installing Dimos")
    const res = await $$`pip install 'dimos @ git+https://github.com/dimensionalOS/dimos.git'`.printCommand()
    if (res.code != 0) {
        console.log(``)
        p.error(`Failed to pip install dimos 😕\nPlease message us in our discord and we'll help you get it installed!:\n    ${p.highlight(discordUrl)}`)
        Deno.exit(1)
    }
}
