#!/usr/bin/env bash
# Build a standalone Mac renderer using an existing Unreal Engine installation.
set -euo pipefail
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 /path/to/UE_5.7 /path/to/package-output" >&2
    exit 2
fi
engine_dir="$(cd "$1" && pwd)"
project_dir="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$2"
output_dir="$(cd "$2" && pwd)"
project_file="$project_dir/DimosRenderer.uproject"
"$engine_dir/Engine/Build/BatchFiles/Mac/Build.sh" DimosRendererEditor Mac Development "$project_file" -WaitMutex
"$engine_dir/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor" "$project_file" \
    -run=pythonscript "-script=$project_dir/Scripts/prepare_assets.py" -unattended -NullRHI -nosound
"$engine_dir/Engine/Build/BatchFiles/RunUAT.sh" BuildCookRun \
    "-project=$project_file" -noP4 -platform=Mac -clientconfig=Development \
    -build -cook -stage -pak -package -archive "-archivedirectory=$output_dir" -utf8output -unattended
