#!/usr/bin/env -S deno run --allow-all --no-lock
import { phase0 } from "./phases/00_logo_and_basic_checks.ts"
import { phase1 } from "./phases/01_all_system_dependencies.ts"
import { phase2 } from "./phases/02_check_absolutely_necessary_tools.ts"
import { phase3 } from "./phases/03_pip_install_dimos.ts"
import { phase4 } from "./phases/04_dimos_check.ts"
import { phase5 } from "./phases/05_env_setup.ts"

if (import.meta.main) {
    const [systemAnalysis, selectedFeatures] = await phase0() // logo and initial check
    await phase1(systemAnalysis, selectedFeatures)            // try to install the full suite of system dependencies (or tell user what is needed)
    await phase2(systemAnalysis, selectedFeatures)            // ensure that critical tools are available (python is correct version, venv is active, git lfs, etc)
    await phase3(systemAnalysis, selectedFeatures)            // pip install dimos
    await phase4(systemAnalysis, selectedFeatures)            // test dimos was installed correctly
    await phase5(systemAnalysis, selectedFeatures)            // use dimos/python to setup everything else (networking, .env, .envrc, etc)
    // FIXME: phase 6 - ask about extras (sim, cuda, etc)
}
