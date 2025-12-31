#!/usr/bin/env -S deno run --allow-all --no-lock
import { $, $$ } from "../support/dax.ts"

import * as p from "../support/prompt_tools.ts"

export async function phase3(systemAnalysis, selectedFeatures) {
    p.clearScreen()
    p.header("Next Phase: Pip Installing Dimos")
    let selectedFeaturesString = ""
    if (selectedFeatures.length > 0) {
        selectedFeaturesString = `[${selectedFeatures.join(",")}]`
    }
    const packageName = `dimos${selectedFeaturesString} @ git+https://github.com/dimensionalOS/dimos.git`
    const res = await $$`pip install ${packageName}`.printCommand()
    if (res.code != 0) {
        console.log(``)
        p.error(`Failed to pip install dimos 😕\nPlease message us in our discord and we'll help you get it installed!:\n    ${p.highlight(discordUrl)}`)
        Deno.exit(1)
    }
}
