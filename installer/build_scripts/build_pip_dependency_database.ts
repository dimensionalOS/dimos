#!/usr/bin/env -S deno run --allow-all --no-lock
import { FileSystem, glob } from "https://deno.land/x/quickr@0.8.13/main/file_system.js"

FileSystem.pwd = `${FileSystem.thisFolder}/../dep_database`
const bigObject = {}
for (const each of await glob(`*.json`)) {
    const name = each.replace(/\.json$/,"")
    try {
        bigObject[name] = JSON.parse(await FileSystem.read(each))
    } catch (error) {
        console.log(`${name} had an error:`, error)
    }
}
await FileSystem.write({path:`${FileSystem.thisFolder}/../support/pip_dependency_database.ts`, data: `export default ${JSON.stringify(bigObject,0,2)}`, overwrite: true})
