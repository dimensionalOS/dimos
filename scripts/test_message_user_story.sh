#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
story_build="$PWD/build/message-codegen/user-story"
story_python="${DIMOS_CODEGEN_PYTHON:-$PWD/.venv/bin/python}"
story_fastcdr="${DIMOS_FASTCDR_PREFIX:-$PWD/build/message-codegen/install}"
"$story_python" -m dimos.message_codegen.generate \
  --package-root examples/message-codegen/user-story \
  --type story_msgs/msg/DeviceReading --python-module story_messages \
  --output "$story_build"
cmake -S "$story_build/cpp" -B "$story_build/cpp/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$story_fastcdr" \
  -DPython_EXECUTABLE="$story_python"
cmake --build "$story_build/cpp/build" -j 2
c++ -std=c++17 -I"$story_fastcdr/include" -I"$story_build/cpp" \
  examples/message-codegen/user-story/consumer.cpp "$story_fastcdr/lib/libfastcdr.a" \
  -o "$story_build/cpp-consumer"
mkdir -p "$story_build/rust/src/bin" "$story_build/evidence"
cp examples/message-codegen/user-story/consumer.rs "$story_build/rust/src/bin/story-consumer.rs"
cargo build --offline --manifest-path "$story_build/rust/Cargo.toml" --bin story-consumer
PYTHONPATH="$story_build/cpp/build:$PWD${PYTHONPATH:+:$PYTHONPATH}" \
  "$story_python" examples/message-codegen/user-story/demo_module.py --build "$story_build" \
  | tee "$story_build/evidence/terminal.txt"
