#!/usr/bin/env -S deno run --allow-all --no-lock
import { phase0 } from "./phases/00_logo_and_basic_checks.ts"
import { phase1 } from "./phases/01_all_system_dependencies.ts"
import { phase2 } from "./phases/02_check_absolutely_necessary_tools.ts"
import { phase3 } from "./phases/03_pip_install_dimos.ts"

if (import.meta.main) {
    // logo and inital check
    const systemAnalysis = await phase0()
    // try to install the full suite of system dependencies (or tell user what is needed)
    await phase1(systemAnalysis)
    // ensure that critical tools are available (python is correct version, venv is active, git lfs, etc)
    await phase2()
    // phase 3 - pip install dimos
    await phase3()
    // FIXME: phase 4 - envrc setup (dimos env vars)
    // FIXME: phase 5 - test dimos was installed correctly
    // FIXME: phase 6 - ask about extras (sim, cuda, etc)
}
