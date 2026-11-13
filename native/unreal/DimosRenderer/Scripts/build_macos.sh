#!/usr/bin/env bash
# Build a standalone Mac renderer using an existing Unreal Engine installation.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
    echo "Usage: $0 /path/to/UE_5.7 /path/to/package-output [/path/to/DimosRenderer.uproject]" >&2
    exit 2
fi
engine_dir="$(cd "$1" && pwd)"
script_dir="$(cd "$(dirname "$0")" && pwd)"
project_file="${3:-$script_dir/../DimosRenderer.uproject}"
project_dir="$(cd "$(dirname "$project_file")" && pwd)"
project_file="$project_dir/$(basename "$project_file")"
if [[ ! -f "$project_file" ]]; then
    echo "Project file does not exist: $project_file" >&2
    exit 2
fi
mkdir -p "$2"
output_dir="$(cd "$2" && pwd)"
"$engine_dir/Engine/Build/BatchFiles/Mac/Build.sh" DimosRendererEditor Mac Development "$project_file" -WaitMutex
"$engine_dir/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" "$project_file" \
    -run=pythonscript "-script=$script_dir/prepare_assets.py" -unattended -NullRHI -nosound
"$engine_dir/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
    "-project=$project_file" -noP4 -platform=Mac -clientconfig=Development \
    -build -cook -stage -pak -package -archive "-archivedirectory=$output_dir" -utf8output -unattended
