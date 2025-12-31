#!/usr/bin/env -S deno run --allow-all --no-lock
import $ from "https://esm.sh/@jsr/david__dax@0.43.2/mod.ts"
import { FileSystem, glob } from "https://deno.land/x/quickr@0.8.13/main/file_system.js"
const $$ = (...args)=>$(...args).noThrow()

/**
 * Like Promise.all, but with backpressure: runs at most `maxConcurrent` async tasks at once.
 *
 * @template T
 * @param {Array<() => Promise<T>>} fns - async task factories (each call starts the task)
 * @param {number} maxConcurrent - max number of in-flight tasks
 * @returns {Promise<T[]>} results in the same order as `fns`
 */
export async function promiseAllWithConcurrency(fns, maxConcurrent) {
    if (!Array.isArray(fns)) throw new TypeError("fns must be an array");
    if (!Number.isFinite(maxConcurrent) || maxConcurrent <= 0) {
        throw new TypeError("maxConcurrent must be a finite number > 0");
    }
    const n = fns.length;
    if (n === 0) return [];

    const results = new Array(n);
    let nextIndex = 0;

    // Run one worker "lane" that keeps pulling tasks until exhausted.
    async function worker() {
        while (true) {
            const i = nextIndex++;
            if (i >= n) return;

            const fn = fns[i];
            if (typeof fn !== "function") {
                throw new TypeError(`fns[${i}] is not a function`);
            }

            // If this rejects, the whole function rejects (like Promise.all),
            // but in-flight tasks may still be running (same as typical concurrency pools).
            try {
                results[i] = await fn();
            } catch (error) {
                if (error instanceof Error) {
                    results[i] = error
                }
                console.error(`index ${i} hit an error: ${error.stack}`)
            }
        }
    }

    const lanes = Math.min(maxConcurrent, n);
    await Promise.all(Array.from({ length: lanes }, () => worker()));
    return results;
}

export async function executeClaudeNamedPrompts(prompts, {maxConcurrentPrompts=5, extraArgs=[], logDir="./.claude/", startFinishLog=true}={}) {
    if (!$.commandExistsSync("claude")) {
        throw Error(`This code requires claude. Yeah yeah cringe I know but thats how it is`)
    }
    return await promiseAllWithConcurrency(
        Object.entries(prompts).map(([name, prompt])=>async ()=>{
            startFinishLog && console.log(`starting: ${name}`)
            await FileSystem.write({path:`${logDir}/${name}.log`, data: "", overwrite: true})
            let output = await $$`claude --allowedTools "Edit,Write,Read,Bash" -p ${prompt} > ${$.path(`${logDir}/${name}.log`,)}`
            startFinishLog && console.log(`finished: ${name}`)
            return output
        }), maxConcurrentPrompts
    )
}
