#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../dax.js"
import * as p from "../prompt_tools.js"
import { aptInstall } from "../misc.ts"
import { dependencyListAptPackages } from "../constants.ts"

const aptPackages = [
    "build-essential",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-wheel",
    "libgl1",
    "libglew-dev",
    "libosmesa6-dev",
    "patchelf",
]

const extraPackages = aptPackages.filter((pkg) => !dependencyListAptPackages.includes(pkg))

async function maybeInstallAptDeps(packages: string[]) {
    if (packages.length === 0) return false
    if (!await $.commandExists("apt-get")) {
        p.boringLog("- apt-get not available; please install these system dependencies manually")
        return false
    }
    const installDeps = p.confirm("Detected apt-get. Install required system dependencies automatically? (sudo may prompt for a password)")
    if (!installDeps) return false

    try {
        await aptInstall(packages)
        return true
    } catch (error) {
        p.error(error.message || "Failed to install some system dependencies.")
        return false
    }
}

export async function setupSimFeature({ assumeSysDepsInstalled = false }: { assumeSysDepsInstalled?: boolean } = {}) {
    p.clearScreen()
    p.header("Optional Feature: Simulation")

    if (!assumeSysDepsInstalled) {
        console.log("- Likely system dependencies needed for MuJoCo/OpenGL headless sim:")
        for (const pkg of extraPackages) console.log(`  • ${pkg}`)
        if (extraPackages.length > 0) {
            const installed = await maybeInstallAptDeps(extraPackages)
            const proceed = installed || p.confirm("Proceed to pip installation (continue even if some system deps may be missing)?")
            if (!proceed) {
                p.error("Please install the listed system dependencies, then rerun this feature installer.")
                return
            }
        } else {
            p.boringLog("- No additional system dependencies beyond the core set.")
        }
    }

    const res = await $$`pip install "dimos[sim]"`
    if (res.code !== 0) {
        p.error("pip install dimos[sim] failed. Please ensure GL/headless deps are installed and try again.")
        p.error("If issues persist, reinstall system deps or consult MuJoCo/OpenGL setup docs.")
        return
    }
    p.boringLog("- dimos[sim] installed successfully")
}
