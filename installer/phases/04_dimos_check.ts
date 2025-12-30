#!/usr/bin/env -S deno run --allow-all --no-lock
import { $$ } from "../support/dax.ts"

import { discordUrl } from "../support/constants.ts"
import * as p from "../support/prompt_tools.ts"

export async function phase4() {
    p.clearScreen()
    p.header("Next Phase: Dimos Check")

    const bail = (msg: string) => {
        console.log("")
        p.error(msg)
        p.error(`Please message us in our discord and we'll help you get it installed:\n    ${discordUrl}`)
        Deno.exit(1)
    }

    const checks: Array<{ label: string; run: () => Promise<{ code: number }> }> = [
        {
            label: "dimos --version",
            run: () => $$`dimos --version`,
        },
        {
            label: "import dimos (python)",
            run: () => $$`python -c "import dimos;"`,
        },
    ]

    let passed = 0
    for (const check of checks) {
        const res = await check.run()
        if (res.code !== 0) {
            bail(`Failed to run ${check.label} 😕`)
        }
        passed++
        p.boringLog(`- ${check.label} succeeded`)
    }

    p.boringLog(`- ${passed} Dimos checks passed`)
}
