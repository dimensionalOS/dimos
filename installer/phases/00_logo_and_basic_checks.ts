#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import { RenderLogo } from "../support/dimos_banner.ts"
import { getToolCheckResults } from "../support/get_tool_check_results.ts"
import { getProjectToml } from "../support/misc.ts"
import * as p from "../support/prompt_tools.ts"

// NOTE: this is basically only user-interactive (if not in an interactive environment, skip this phase)
export async function phase0() {
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
    const timeout = 500
    // const timeout = 50
    for (const [key, {name, exists, version, note}] of Object.entries(systemAnalysis)) {
        // sleep so user can actually read whats happening before clearing the screen
        await new Promise(r=>setTimeout(r,timeout))
        if (!exists) {
            logo.log(`- ❌ ${name||key} ${note||""}`)
        } else {
            logo.log(`- ✅ ${name}: ${version} ${note||""}`)
        }
    }
    let tomlPromise = getProjectToml()
    await new Promise(r=>setTimeout(r,timeout))
    logo.stop()
    p.clearScreen()

    const tomlData = await tomlPromise
    console.log(`tomlData.project is:`,tomlData.project)
    const features = Object.keys(tomlData.project["optional-dependencies"]).filter(each=>!["cpu"].includes(each))
    const selectedFeatures = await p.pickMany("Which features do you want?", { options: features })
    return [systemAnalysis, selectedFeatures]
}


if (import.meta.main) {
    const out = await phase0()
    console.log(`out is:`,out)
}
