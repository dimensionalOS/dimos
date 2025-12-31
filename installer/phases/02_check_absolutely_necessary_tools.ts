#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import { activateVenv, getVenvDirsAt } from "../support/venv.ts"
import { discordUrl } from "../support/constants.ts"
import { ensureGitAndLfs, ensurePortAudio, ensurePython, aptInstall, getProjectDirectory, addGitIgnorePatterns } from "../support/misc.ts"
import * as p from "../support/prompt_tools.ts"

// NOTE: this part always gets run regardless of nix/docker/manual install
export async function phase2(systemAnalysis, selectedFeatures) {
    p.clearScreen()
    p.header("Next Phase: Check Install of Vital System Dependencies")
    try {
        const [hasIfconfig, hasRoute, hasSysctl, pythonCmd, ...rest] = await Promise.all([
            $.commandExists("ifconfig"),
            $.commandExists("route"),
            $.commandExists("sysctl"),
            ensurePython(),
            ensureGitAndLfs(),
            ensurePortAudio(),
        ])
        if (!hasIfconfig || !hasRoute || !hasSysctl) {
            console.log("- ifconfig, route, and sysctl are required for the installer to function")
            console.log("- Please install these system dependencies and re-run this command from the terminal")
            Deno.exit(1)
        }
        if (selectedFeatures.includes("cuda")) {
            if (!systemAnalysis.cuda.exists) {
                p.error("you selected the CUDA feature but I don't see CUDA support in your system")
            }
        }
        await ensureVenvActive(pythonCmd)
    } catch (error) {
        console.log(``)
        console.log(``)
        p.error(`One of the vital dependencies was missing or had versioning issues`)
        p.error(`    error: ${error?.message}`)
        p.error(`Message us in the discord if you're having trouble: ${p.highlight(discordUrl)}`)
        if (p.askYesNo(`It is NOT recommended to continue. Would you like to stop the setup? [y=exit, n=continue]`)) {
            Deno.exit(1)
        }
    }
}

const defaultVenvName = "venv"
async function ensureVenvActive(pythonCmd) {
    // NOTE: this function is here (rather than in support) because it depends on context (right version of python already being installed/checked)

    const activeVenv = Deno.env.get("VIRTUAL_ENV")
    // case 1: venv already active
    if (activeVenv) {
        p.boringLog(`- detected active virtual environment: ${activeVenv}`)
        return activeVenv
    }
    p.clearScreen()
    let projectDirectory = await getProjectDirectory()
    const possibleVenvDirs = getVenvDirsAt(projectDirectory)
    // case 2: venv exist but is not active
    if (possibleVenvDirs.length == 1) {
        activateVenv(possibleVenvDirs[0])
    // case 3: multiple venvs exist but none are active
    } else if (possibleVenvDirs.length > 1) {
        console.log(`- multiple python virtual environments found`)
        console.log(`- Dimos needs to be installed to a python virtual environment`)
        const chosenVenvDir = await p.pickOne(`Choose a virtual environment to activate:`, {options: possibleVenvDirs})
        activateVenv(chosenVenvDir)
    // case 4: no venvs exist
    } else {
        console.log(`- Dimos needs to be installed to a python virtual environment`)
        if (!p.confirm("Can I setup a Python virtual environment for you?")) {
            throw Error("- ❌ A virtual environment is required to install dimos. Please set one up then rerun this command.")
        }
        const venvDir = `${projectDirectory}/${defaultVenvName}`
        p.boringLog(`- creating virtual environment at ${venvDir}`)
        const venvRes = await $$`${pythonCmd} -m venv ${venvDir}`
        if (venvRes.code !== 0) {
            throw Error("- ❌ Failed to create virtual environment. Please create one manually and rerun this command.")
        }
        addGitIgnorePatterns(projectDirectory, [ `/${defaultVenvName}` ], { comment: "Added by dimos setup" })
        activateVenv(venvDir)
        p.boringLog("- ✅ virtual environment activated")
        return venvDir
    }
    // regardless of the conditional case this should end up as a valid path
    return Deno.env.get("VIRTUAL_ENV")
}
