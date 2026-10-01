# C++ CDR Robot Control Example

Subscribes to `/odom` and publishes velocity commands to `/cmd_vel` using
encapsulated CDR messages over raw LCM. Run only with the virtual
[SimpleRobot](/examples/simplerobot/README.md) unless robot motion is authorized.

Install the generated CMake package and Fast CDR using the
[message package workflow](/docs/development/messages.md). Install raw LCM
separately. Configure with the installation prefixes, without fetching message
headers from another repository:

```bash
cmake -S examples/language-interop/cpp -B build/language-interop-cpp \
  -DCMAKE_PREFIX_PATH="/path/to/message-prefix;/path/to/fastcdr-prefix"
cmake --build build/language-interop-cpp --parallel 2
# Publishes velocity commands; use the virtual robot for this demonstration.
build/language-interop-cpp/robot_control
```

For the bounded Python/C++/Rust relay, schema inspection, recording and replay,
see [message-codegen](/examples/message-codegen/README.md).
