#!/usr/bin/env -S deno run --allow-all --no-lock
import {
    FileSystem,
    glob,
} from "https://deno.land/x/quickr@0.8.13/main/file_system.js";
import * as Toml from "https://esm.sh/smol-toml@1.6.0";
import $ from "https://esm.sh/@jsr/david__dax@0.43.2/mod.ts";
import { executeClaudeNamedPrompts } from "./support/claude.ts";
import { intersection } from "https://esm.sh/gh/jeff-hykin/good-js@1.18.2.0/source/flattened/intersection.js";
const $$ = (...args) => $(...args).noThrow();
// await $$`false`
// (await $$`false`).code
// await $$`false`.text("stderr")
// await $$`false`.text("combined")
// await $$`echo`.stdinText("yes\n")

if (!$.commandExistsSync("claude")) {
    console.log(
        `This generator requires claude. Yeah yeah cringe I know but thats the only way to do it.`
    );
}
const pyproject = Toml.parse(
    await FileSystem.read(`${FileSystem.thisFolder}/../../pyproject.toml`)
);

// get all python deps
let dependencies = pyproject.project.dependencies;
for (const [key, deps] of Object.entries(
    pyproject.project["optional-dependencies"]
)) {
    dependencies.push(...deps);
}

const projectDir = await $`git rev-parse --show-toplevel`.text();

// FIXME: add a filter for already existing ones
const depLists = [
    "apt_dependencies",
    "brew_dependencies",
    "nix_dependencies"
]
const requiredKeys = ["package", ...depLists]
const missing = []
const namesAndPrompts = dependencies
    .map((requirement) => {
        const name = requirement.replace(/[=>,;].+/, "").toLowerCase();
        return [name, requirement];
    })
    .filter(([name, requirement]) => {
        let obj
        try {
            obj = JSON.parse(
                FileSystem.sync.read(
                    `${projectDir}/installer/dep_database/${name}.json`
                )
            )
        } catch (error) {
            missing.push(name)
            return true
        }
        const overlap = [
            ...intersection(new Set(Object.keys(obj)), new Set(requiredKeys)),
        ]
        if (overlap.length == requiredKeys.length) {
            if (depLists.every((each) => obj[each] instanceof Array)) {
                // if all the keys/values are correct, then don't make a prompt for it
                return false;
            }
        }
        return true;
    })
    .map(([name, each]) => {
        return [
            name,
            `list all apt-get dependencies, nix, and brew dependencies for the ${each} pip module. The result should be a json object with the following ${requiredKeys
                .map(JSON.stringify)
                .join(" ")} and optionally "description", "notes". These (${depLists
                .map(JSON.stringify)
                .join(
                    " "
                )}) should be list of strings. Store that resulting json inside ./installer/dep_database/${name}.json`,
        ]
    })

if (Deno.args.length > 0 || Deno.args.includes("--dry-run")) {
    for (const each of namesAndPrompts.map(each=>each[0])) {
        if (missing.includes(each)) {
            console.log(`missing           : ${each}.json`)
        } else {
            console.log(`needs modification: ${each}.json`)
        }
    }
    const total = namesAndPrompts.length
    console.log("total:",total)
    console.debug(`missing:`,missing.length)
    console.debug(`need modification:`,total-missing.length)
} else {
    await executeClaudeNamedPrompts(Object.fromEntries(namesAndPrompts))
}
