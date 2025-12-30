#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import { getProjectDirectory } from "../support/misc.ts"
import { setupDirenv } from "../support/env_setup/direnv.ts"
import { setupDotenv } from "../support/env_setup/dotenv.ts"
import * as p from "../support/prompt_tools.ts"

export async function phase5() {
    p.clearScreen()
    p.header("Next Phase: Environment configuration")

    const projectPath = await getProjectDirectory()
    const envPath = `${projectPath}/.env`
    const envrcPath = `${projectPath}/.envrc`

    const hasDotenv = await setupDotenv(projectPath, envPath)
    if (!hasDotenv) return

    await setupDirenv(envrcPath)
}
