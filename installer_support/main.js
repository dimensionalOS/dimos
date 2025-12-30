#!/usr/bin/env -S deno run --allow-all --no-lock
import { RenderLogo } from "./dimos_banner.js"
import { getToolCheckResults } from "./get_tool_check_results.ts"
import $ from "https://esm.sh/@jsr/david__dax@0.43.2/mod.ts"
const $$ = (...args)=>$(...args).noThrow()
// await $$`false`
// (await $$`false`).code
// await $$`false`.text("stderr")
// await $$`false`.text("combined")
// await $$`echo`.stdinText("yes\n")

const logo = new RenderLogo({
    glitchyness: 0.35,
    stickyness: 18,
    fps: 30,
    waveStrength: 12,
    waveSpeed: 0.12,
    waveFreq: 0.07,
    scrollable: true,
})


logo.log("- checking system")

const tooling = await getToolCheckResults()

for (const [key, {name, exists, version, note}] of Object.entries(tooling)) {
    // sleep so user can actually read whats happening before clearing the screen
    await new Promise(r=>setTimeout(r,300))
    if (!exists) {
        logo.log(`- ❌ ${name||key} ${note||""}`)
    } else {
        logo.log(`- ✅ ${name}: ${version} ${note||""}`)
    }
}
