#!/usr/bin/env -S deno run --allow-all --no-lock
import { getToolCheckResults, type ToolResult } from "../support/get_tool_check_results.ts"
import { dependencyListHumanNames, dependencyListAptPackages, dependenciesNixNames } from "../support/constants.ts"
import { getSystemDeps, aptInstall } from "../support/misc.ts"
import * as p from "../support/prompt_tools.ts"
import { $ } from "../support/dax.ts";

// NOTE: skip this phase if system dependencies already exist (e.g. docker, or nix environment)
export async function phase1(systemAnalysis: Record<string, ToolResult> | null, selectedFeatures) {
    p.clearScreen()
    p.header("Next Phase: System Dependency Install")
    if (!systemAnalysis) {
        systemAnalysis = await getToolCheckResults()
    }
    const deps = await getSystemDeps(selectedFeatures) // null = core deps (as opposed to a feature name)
    mentionSystemDependencies()

    let toolsWereAutoInstalled = false
    // TODO: later check the debian version (make sure not older than whatever ubuntu's 20.04 debian version is)
    const osInfo = systemAnalysis.os
    if (osInfo.name === "debianBased") {
        p.boringLog("Detected Debian-based OS")
        const installDeps = p.confirm("Install these system dependencies for you via apt-get? (NOTE: sudo may prompt for a password)")
        if (installDeps) {
            p.boringLog("- this may take a few minutes...")
            // this will throw if some packages fail to install
            try {
                await aptInstall(deps.aptDeps)
                toolsWereAutoInstalled = true
            } catch (error) {
                p.error(error.message)
            }
        } else {
            console.log("- skipping automatic installation.")
            const proceed = p.confirm("Proceed to the next step without installing system dependencies?")
            if (!proceed) {
                console.log("- ❌ Please install the listed dependencies and rerun.")
                Deno.exit(1)
            }
        }
    }

    if (!toolsWereAutoInstalled) {
        p.confirm("I can't confirm that all those tools are installed\nPress enter to continue anyway, or CTRL+C to cancel and install them yourself")
    }
}


function mentionSystemDependencies() {
    console.log("- we will need the following system dependencies:")
    // TODO: clean up this filter, many (probably most) of the deps are not cli commands. This intentionally fails on python because we need a specific version of python
    // later all of these should have versions
    const missingDeps = dependencyListHumanNames.filter(each=>!$.commandExists(each))
    for (const dep of missingDeps) {
        console.log(`  • ${p.highlight(dep)}`)
    }
}
