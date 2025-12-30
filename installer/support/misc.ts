import { $, $$ } from "./dax.js"
import { activateVenv, getVenvDirsAt } from "./venv.js"
import { dependencyListHumanNames } from "./constants.ts"
import * as p from "./prompt_tools.js"

export function mentionSystemDependencies() {
    console.log("- we will need the following system dependencies:")
    for (const dep of dependencyListHumanNames) {
        console.log(`  • ${p.highlight(dep)}`)
    }
}

export function parseVersion(text) {
    const match = text.match(/\b(\d+(?:\.\d+)+)\b/)
    return match?.[1]
}

export function isVersionAtLeast(found, required) {
    const foundParts = found.split(".").map(Number)
    const requiredParts = required.split(".").map(Number)
    const length = Math.max(foundParts.length, requiredParts.length)
    for (let i = 0; i < length; i++) {
        const f = foundParts[i] ?? 0
        const r = requiredParts[i] ?? 0
        if (f > r) return true
        if (f < r) return false
    }
    return true
}

export async function detectPythonCommand() {
    if (await $.commandExists("python3")) return "python3"
    if (await $.commandExists("python")) return "python"
    return null
}

export async function ensureGitAndLfs() {
    if (!await $.commandExists("git")) {
        throw Error("- ❌ git is required. Please install git and rerun.")
    }
    const gitLfsRes = await $$`git lfs version`
    if (gitLfsRes.code !== 0) {
        throw Error("- ❌ git-lfs is required. Please install git-lfs and rerun.")
    }
}

export async function ensurePortAudio() {
    p.boringLog("Checking if portaudio is available")
    const portAudioRes = await $$`pkg-config --modversion portaudio-2.0`.printCommand()
    if (portAudioRes.code !== 0) {
        throw Error("- ❌ portaudio is required. Please install portaudio and rerun.")
    }
}

export async function ensurePython() {
    const pythonCmd = await detectPythonCommand()
    if (!pythonCmd) {
        throw Error("- ❌ Python 3.10+ is required but was not found.")
    }
    const versionRes = await $$`${pythonCmd} --version`
    const versionText = (versionRes.stdout ?? versionRes.stderr ?? "").trim()
    const parsed = parseVersion(versionText)
    if (!parsed || !isVersionAtLeast(parsed, "3.10.0")) {
        throw Error(`- ❌ Python 3.10+ required. Detected: ${parsed ?? "unknown"}`)
    }
    return pythonCmd
}

let projectDirectory
export async function getProjectDirectory() {
    if (!projectDirectory) {
        console.log(`Dimos needs to be installed to a project (not just a global install)`)
        if (p.askYesNo("Are you currently in a project directory?")) {
            projectDirectory = Deno.cwd()
        } else {
            throw Error(`- ❌ Please create a project directory and rerun this command from there.`)
        }
    }
    return projectDirectory
}

let alreadyCalledAptGetUpdate = false
export async function aptInstall(packageNames) {
    // apt-get update if needed
    if (!alreadyCalledAptGetUpdate) {
        const updateRes = await $$`sudo apt-get update`.printCommand()
        if (updateRes.code !== 0) {
            throw Error(`sudo apt-get update failed: ${updateRes.code}`)
        }
        alreadyCalledAptGetUpdate = true
    }
    let failedPackages = []
    for (const eachAptPackage of dependencyListAptPackages) {
        const res = await $$`dpkg -s ${eachAptPackage}`
        if (res.code === 0) {
            console.log(`- ✅ looks like ${p.highlight(eachAptPackage)} is already installed`)
            continue
        } else {
            p.subHeader(`- installing ${p.highlight(eachAptPackage)}`)
            const installRes = await $$`sudo apt-get install -y ${eachAptPackage}`.printCommand()
            if (installRes.code !== 0) {
                failedPackages.push(eachAptPackage)
            }
        }
    }
    if (failedPackages.length > 0) {
        throw Error(`apt-get install failed for: ${failedPackages.join(" ")}\nTry to install them yourself with ${failedPackages.map(each => `    sudo apt-get install -y ${each}`).join("\n")}`)
    }
}


/**
 * Add ignore patterns to an existing .gitignore in `projectPath`, if not already present.
 *
 * - Does nothing if `${projectPath}/.gitignore` does not exist.
 * - Adds each pattern as its own line.
 * - Skips patterns that already exist (exact line match after trimming).
 * - Optionally groups new entries under a comment header.
 */
export async function addGitIgnorePatterns(
    projectPath: string,
    patterns: string[],
    opts: { comment?: string } = {},
): Promise<{ updated: boolean; added: string[]; alreadyPresent: string[]; ignoreDidNotExist: boolean }> {
    const gitignorePath = `${projectPath.replace(/\/+$/, "")}/.gitignore`

    // If .gitignore doesn't exist, do nothing.
    try {
        const st = await Deno.stat(gitignorePath)
        if (!st.isFile) return { updated: false, added: [], alreadyPresent: [], ignoreDidNotExist: true }
    } catch {
        return { updated: false, added: [], alreadyPresent: patterns.slice(), ignoreDidNotExist: false }
    }

    const original = await Deno.readTextFile(gitignorePath)

    // Normalize to '\n' for easier handling, preserve content otherwise.
    const hasTrailingNewline = original.endsWith("\n") || original.endsWith("\r\n")
    const text = original.replace(/\r\n/g, "\n")

    const existingLines = text.split("\n")
    const existingSet = new Set(existingLines.map((l) => l.trim()).filter((l) => l.length > 0))

    const cleanedPatterns = patterns
        .map((p) => p.trim())
        .filter((p) => p.length > 0)

    const added: string[] = []
    const alreadyPresent: string[] = []

    for (const p of cleanedPatterns) {
        if (existingSet.has(p)) alreadyPresent.push(p)
        else added.push(p)
    }

    if (added.length === 0) {
        return { updated: false, added: [], alreadyPresent, ignoreDidNotExist: false }
    }

    const newLines: string[] = []
    // Ensure file ends with exactly one blank line before our group (nice-ish formatting).
    const needsNewline = !hasTrailingNewline && text.length > 0
    if (needsNewline) newLines.push("")

    // If the file doesn't already end with a newline (after normalization), we already inserted "" above.
    // If it ends with newline but not with a blank line, add a blank line before our header/entries.
    const endsWithBlankLine = existingLines.length > 0 && existingLines[existingLines.length - 1].trim() === ""
    if (hasTrailingNewline && !endsWithBlankLine) newLines.push("")

    if (added.length > 0 && opts.comment && opts.comment.trim().length > 0) {
        const header = opts.comment.trim().startsWith("#")
            ? opts.comment.trim()
            : `# ${opts.comment.trim()}`
        // add to top
        newLines.unshift(header)
    }

    newLines.push(...added)

    const updatedText = text + newLines.join("\n") + "\n"
    await Deno.writeTextFile(gitignorePath, updatedText)

    return { updated: true, added, alreadyPresent, ignoreDidNotExist: false }
}
