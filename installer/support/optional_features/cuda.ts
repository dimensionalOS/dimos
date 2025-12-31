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
    "pkg-config",
    "libopenblas-dev",
    "liblapack-dev",
    "gfortran",
    "libjpeg-dev",
    "libpng-dev",
    "zlib1g-dev",
    "libeigen3-dev",
]

const extraPackages = aptPackages.filter((pkg) => !dependencyListAptPackages.includes(pkg))

const cudaReminder = [
    "- This feature expects NVIDIA drivers + CUDA 12.x toolkit to already be installed.",
    "- Install the official NVIDIA packages for your distro (not the older `nvidia-cuda-toolkit`).",
    "- Ensure PATH and LD_LIBRARY_PATH include CUDA binaries and libraries.",
]

async function maybeInstallAptDeps(packages: string[]) {
    if (packages.length === 0) return false
    if (!await $.commandExists("apt-get")) {
        p.boringLog("- apt-get not available; please install these system dependencies manually")
        return false
    }
    const installDeps = p.confirm("Detected apt-get. Install common build/runtime deps automatically? (sudo may prompt for a password)")
    if (!installDeps) return false

    try {
        await aptInstall(packages)
        return true
    } catch (error) {
        p.error(error.message || "Failed to install some system dependencies.")
        return false
    }
}

export async function setupCudaFeature({ assumeSysDepsInstalled = false }: { assumeSysDepsInstalled?: boolean } = {}) {
    p.clearScreen()
    p.header("Optional Feature: CUDA / GPU stack")

    for (const line of cudaReminder) {
        p.warning(line)
    }

    if (!assumeSysDepsInstalled) {
        console.log("- Likely system dependencies needed for CUDA builds (xformers, mmcv, detectron2, etc.):")
        for (const pkg of extraPackages) console.log(`  • ${pkg}`)
        if (extraPackages.length > 0) {
            const installed = await maybeInstallAptDeps(extraPackages)
            const proceed = installed || p.confirm("Proceed to pip installation (continue even if some system deps may be missing)?")
            if (!proceed) {
                p.error("Please install the listed system dependencies (and CUDA toolkit), then rerun this feature installer.")
                return
            }
        } else {
            p.boringLog("- No additional system dependencies beyond the core set.")
        }
    }

    const res = await $$`pip install "dimos[cuda]"`
    if (res.code !== 0) {
        p.error("pip install dimos[cuda] failed. Please confirm CUDA 12.x drivers/toolkit are installed and try again.")
        p.error("If issues persist, reinstall system deps or build the failing packages from source with verbose logs.")
        return
    }
    p.boringLog("- dimos[cuda] installed successfully")
}
