#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import { RenderLogo } from "../support/dimos_banner.ts"
import { getToolCheckResults, type ToolResult } from "../support/get_tool_check_results.ts"
import { activateVenv } from "../support/venv.ts"
import { dependencyListHumanNames, dependencyListAptPackages } from "../support/constants.ts"
import { mentionSystemDependencies, parseVersion, isVersionAtLeast, detectPythonCommand, ensureGitAndLfs, ensurePortAudio, ensurePython } from "../support/misc.ts"
import * as p from "../support/prompt_tools.ts"

// NOTE: this is basically only user-interactive (if not in an interactive environment, skip this phase)
export async function phase0() : Promise<Record<string, ToolResult>> {
    const logo = new RenderLogo({
        glitchyness: 0.35,
        stickyness: 18,
        fps: 30,
        waveStrength: 12,
        waveSpeed: 0.12,
        waveFreq: 0.07,
        scrollable: true,
    })

    logo.log("- checking system")

    const systemAnalysis = await getToolCheckResults()
    for (const [key, {name, exists, version, note}] of Object.entries(systemAnalysis)) {
        // sleep so user can actually read whats happening before clearing the screen
        await new Promise(r=>setTimeout(r,500))
        if (!exists) {
            logo.log(`- ❌ ${name||key} ${note||""}`)
        } else {
            logo.log(`- ✅ ${name}: ${version} ${note||""}`)
        }
    }

    await new Promise(r=>setTimeout(r,500))
    logo.stop()
    clearScreen()
    return systemAnalysis
}
