#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.js"

import { dimosEnvVars } from "../support/constants.ts"
import { addGitIgnorePatterns, getProjectDirectory } from "../support/misc.ts"
import * as p from "../support/prompt_tools.js"

export async function phase4() {
    p.clearScreen()
    p.header("Next Phase: Dimos Check")

    // FIXME: test the dimos cli by running `dimos --version`
    // FIXME: test the dimos pip module by importing dimos in a python runtime
    // if there's an error, report it, give the standard message about messaging in the discord
}
