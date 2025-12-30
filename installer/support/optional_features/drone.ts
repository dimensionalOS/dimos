#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../dax.ts"
import * as p from "../prompt_tools.ts"
import { aptInstall } from "../misc.ts"

const aptPackages = [
    "build-essential",
    "python3-dev",
    "python3-pip",
    "python3-setuptools",
    "python3-wheel",
    "libxml2-dev",
    "libxslt1-dev",
]

async function maybeInstallAptDeps(packages: string[]) {
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

export async function setupDroneFeature({ assumeSysDepsInstalled = false }: { assumeSysDepsInstalled?: boolean } = {}) {
    p.clearScreen()
    p.header("Optional Feature: Drone / MAVLink")

    if (!assumeSysDepsInstalled) {
        console.log("- Likely system dependencies needed for pymavlink:")
        for (const pkg of aptPackages) console.log(`  • ${pkg}`)
        const installed = await maybeInstallAptDeps(aptPackages)
        const proceed = installed || p.confirm("Proceed to pip installation (continue even if some system deps may be missing)?")
        if (!proceed) {
            p.error("Please install the listed system dependencies, then rerun this feature installer.")
            return
        }
    }

    const res = await $$`pip install "dimos[drone]"`
    if (res.code !== 0) {
        p.error("pip install dimos[drone] failed. Please ensure system deps are installed and try again.")
        p.error("If issues persist, reinstall system deps or install pymavlink with verbose logs.")
        return
    }
    p.boringLog("- dimos[drone] installed successfully")
}
