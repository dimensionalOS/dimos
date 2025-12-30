#!/usr/bin/env -S deno run --allow-run --allow-env --allow-read
/**
 * Parallel tool/version checker using dax.
 *
 * Changes in this version:
 * - ToolResult now includes `name`
 * - OS "tool" name is dynamic and comes from a fixed exported union/list:
 *   "macos" | "windows" | "debianBased" | "archBased" | "fedora" | "unknownLinux" | "unknown"
 * - On Linux:
 *   - report "debianBased" ONLY if `apt-get` exists
 *   - else "archBased" if `pacman` exists
 *   - else "fedora" if `dnf` exists
 *   - else "unknownLinux"
 */

import $ from "https://esm.sh/@jsr/david__dax@0.43.2/mod.ts";

export type ToolResult = {
    /** Stable identifier for the check (for OS this is dynamic from OS_NAMES). */
    name: string;
    exists: boolean;
    version?: string;
    raw?: string;
    note?: string;
};

/** All possible OS "names" reported by this script. */
export const OS_NAMES = [
    "macos",
    "windows",
    "debianBased",
    "archBased",
    "fedora",
    "unknownLinux",
    "unknown",
] as const;

export type OSName = typeof OS_NAMES[number];

function extractDigitsDots(s: string): string | undefined {
    const m = s.match(/\b(\d+(?:\.\d+)+)\b/);
    return m?.[1];
}

async function runFirstLine(
    command: ReturnType<typeof $>
): Promise<{ code: number; line: string; combined: string }> {
    const res = await command.quiet().noThrow();
    const combined = `${res.stdout ?? ""}\n${res.stderr ?? ""}`.trim();
    const line =
        combined
            .split(/\r?\n/)
            .find((l) => l.trim().length > 0)
            ?.trim() ?? "";
    return { code: res.code, line, combined };
}

async function getVersionFromCommand(
    name: string,
    command: ReturnType<typeof $>,
    opts?: { allowRawIfNoDigits?: boolean; note?: string }
): Promise<[string, ToolResult]> {
    const { code, line, combined } = await runFirstLine(command);

    if (code !== 0 && !line) {
        return [
            name,
            { name, exists: true, note: "Command exists but version query failed." },
        ];
    }

    const ver = extractDigitsDots(line) ?? extractDigitsDots(combined);
    if (ver)
        return [
            name,
            { name, exists: true, version: ver, raw: line, note: opts?.note },
        ];

    return [
        name,
        {
            name,
            exists: true,
            raw: line || combined.split(/\r?\n/)[0]?.trim() || "",
            note: opts?.allowRawIfNoDigits
                ? opts?.note ?? "No digit-dot version found; showing raw output."
                : "No digit-dot version found.",
        },
    ];
}

async function detectOSName(): Promise<OSName> {
    if (Deno.build.os === "darwin") return "macos";
    if (Deno.build.os === "windows") return "windows";

    if (Deno.build.os === "linux") {
        // Per your rule: ONLY call it debianBased if apt-get exists.
        const [hasApt, hasPacman, hasDnf] = await Promise.all([
            $.commandExists("apt-get"),
            $.commandExists("pacman"),
            $.commandExists("dnf"),
        ]);

        if (hasApt) return "debianBased";
        if (hasPacman) return "archBased";
        if (hasDnf) return "fedora";
        return "unknownLinux";
    }

    return "unknown";
}

async function detectOSDetails(osName: OSName): Promise<{ version?: string; raw?: string; note?: string }> {
    if (osName === "macos") {
        const { line } = await runFirstLine($`sw_vers -productVersion`);
        return {
            version: extractDigitsDots(line) ?? line,
            raw: line,
            note: "macOS version",
        };
    }

    if (osName === "windows") {
        const { line } = await runFirstLine($`cmd /c ver`);
        return { version: extractDigitsDots(line), raw: line };
    }

    if (
        osName === "debianBased" ||
        osName === "archBased" ||
        osName === "fedora" ||
        osName === "unknownLinux"
    ) {
        // Best-effort: /etc/os-release if present
        try {
            const text = await Deno.readTextFile("/etc/os-release");
            const versionId = (text.match(/^VERSION_ID=(.*)$/m)?.[1] ?? "").replace(
                /^"|"$/g,
                ""
            );
            const pretty = (text.match(/^PRETTY_NAME=(.*)$/m)?.[1] ?? "").replace(
                /^"|"$/g,
                ""
            );
            return {
                version: extractDigitsDots(versionId) ?? (versionId || undefined),
                raw: pretty || undefined,
            };
        } catch {
            const { line } = await runFirstLine($`uname -sr`);
            return { raw: line };
        }
    }

    return {};
}

export async function getToolCheckResults(): Promise<Record<string, ToolResult>> {
    const results: Record<string, ToolResult> = {};

    // Existence checks in parallel (using dax built-in)
    const existenceEntries = await Promise.all(
        [
            "git",
            "nix",
            "docker",
            "python3",
            "python",
            "pip3",
            "pip",
            "nvcc",
            "nvidia-smi",
        ].map(async (cmd) => [cmd, await $.commandExists(cmd)] as const)
    );
    const existence = Object.fromEntries(existenceEntries) as Record<
        string,
        boolean
    >;

    // Schedule version checks (run in parallel)
    const tasks: Promise<[string, ToolResult]>[] = [];

    if (existence.git)
        tasks.push(
            getVersionFromCommand("git", $`git --version`, {
                allowRawIfNoDigits: true,
            })
        );
    else results.git = { name: "git", exists: false };

    if (existence.nix)
        tasks.push(
            getVersionFromCommand("nix", $`nix --version`, {
                allowRawIfNoDigits: true,
            })
        );
    else results.nix = { name: "nix", exists: false };

    if (existence.docker)
        tasks.push(
            getVersionFromCommand("docker", $`docker --version`, {
                allowRawIfNoDigits: true,
            })
        );
    else results.docker = { name: "docker", exists: false };

    // git lfs (git must exist)
    if (existence.git) {
        tasks.push(
            getVersionFromCommand("git_lfs", $`git lfs version`, {
                allowRawIfNoDigits: true,
            }).then(([k, v]) => {
                if (v.exists && !v.version) {
                    v.note =
                        (v.note ? v.note + " " : "") +
                        "If this says 'git: lfs is not a git command', Git LFS isn't installed.";
                }
                return [k, v] as [string, ToolResult];
            })
        );
    } else {
        results.git_lfs = {
            name: "git_lfs",
            exists: false,
            note: "git not found, so git lfs cannot be checked.",
        };
    }

    // Python: prefer python3 then python
    const pythonCmd = existence.python3
        ? "python3"
        : existence.python
            ? "python"
            : null;
    if (pythonCmd) {
        tasks.push(
            getVersionFromCommand(
                "python",
                pythonCmd === "python3" ? $`python3 --version` : $`python --version`,
                { allowRawIfNoDigits: true, note: `From ${pythonCmd}` }
            )
        );
    } else {
        results.python = { name: "python", exists: false };
    }

    // Pip: prefer `python -m pip`, else pip3, else pip
    if (pythonCmd) {
        tasks.push(
            getVersionFromCommand(
                "pip",
                pythonCmd === "python3"
                    ? $`python3 -m pip --version`
                    : $`python -m pip --version`,
                { allowRawIfNoDigits: true, note: `From ${pythonCmd} -m pip` }
            ).then(([k, v]) => {
                if (
                    v.exists &&
                    (v.raw ?? "").toLowerCase().includes("no module named pip")
                ) {
                    return [
                        k,
                        {
                            name: "pip",
                            exists: false,
                            raw: v.raw,
                            note: "Python found, but pip module not installed.",
                        },
                    ] as [string, ToolResult];
                }
                return [k, v] as [string, ToolResult];
            })
        );
    } else if (existence.pip3) {
        tasks.push(
            getVersionFromCommand("pip", $`pip3 --version`, {
                allowRawIfNoDigits: true,
                note: "From pip3",
            })
        );
    } else if (existence.pip) {
        tasks.push(
            getVersionFromCommand("pip", $`pip --version`, {
                allowRawIfNoDigits: true,
                note: "From pip",
            })
        );
    } else {
        results.pip = { name: "pip", exists: false };
    }

    // CUDA: prefer nvcc, else nvidia-smi
    if (existence.nvcc) {
        tasks.push(
            (async () => {
                const { combined } = await runFirstLine($`nvcc --version`);
                const ver =
                    combined.match(/release\s+(\d+(?:\.\d+)+)/i)?.[1] ??
                    extractDigitsDots(combined);
                const firstLine = combined.trim().split(/\r?\n/)[0]?.trim() ?? "";
                return [
                    "cuda",
                    {
                        name: "cuda",
                        exists: true,
                        version: ver,
                        raw: firstLine,
                        note: "From nvcc",
                    },
                ] as [string, ToolResult];
            })()
        );
    } else if (existence["nvidia-smi"]) {
        tasks.push(
            (async () => {
                const res = await $`nvidia-smi`.quiet().noThrow();
                const combined = `${res.stdout ?? ""}\n${res.stderr ?? ""}`.trim();
                const ver =
                    combined.match(/CUDA Version:\s*([0-9]+(?:\.[0-9]+)+)/i)?.[1] ??
                    extractDigitsDots(combined);
                const firstLine = combined.split(/\r?\n/)[0]?.trim() ?? "";
                return [
                    "cuda",
                    {
                        name: "cuda",
                        exists: res.code === 0,
                        version: ver,
                        raw: firstLine,
                        note: "From nvidia-smi",
                    },
                ] as [string, ToolResult];
            })()
        );
    } else {
        results.cuda = {
            name: "cuda",
            exists: false,
            note: "Neither nvcc nor nvidia-smi found.",
        };
    }

    // OS check (dynamic name)
    tasks.push(
        (async () => {
            const osName = await detectOSName();
            const details = await detectOSDetails(osName);
            return ["os", { name: osName, exists: true, ...details }] as [
                string,
                ToolResult
            ];
        })()
    );

    // Execute all scheduled checks in parallel
    const pairs = await Promise.all(tasks);
    for (const [k, v] of pairs) results[k] = v;

    return results;
}
