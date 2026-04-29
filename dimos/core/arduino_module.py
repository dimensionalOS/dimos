# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ArduinoModule: DimOS module for Arduino-based hardware.

An ArduinoModule generates a ``dimos_arduino.h`` header at build time,
compiles and flashes the user's Arduino sketch, then launches a generic
C++ bridge that relays structured data between the Arduino's USB serial
and the DimOS LCM bus.

Example usage::

    class MyArduinoBot(ArduinoModule):
        config: MyArduinoBotConfig
        imu_out: Out[Imu]
        motor_cmd_in: In[Twist]

See ``dimos/hardware/arduino/`` for the C headers, bridge binary, and
protocol documentation.
"""

from __future__ import annotations

from dataclasses import dataclass
import errno
import functools
import glob
import inspect
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
from typing import IO, Any, ClassVar, get_args, get_origin, get_type_hints

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Path to the arduino hardware directory (relative to this file)
_ARDUINO_HW_DIR = Path(__file__).resolve().parent.parent / "hardware" / "arduino"
_COMMON_DIR = _ARDUINO_HW_DIR / "common"
_DSP_PROTOCOL_PATH = _COMMON_DIR / "dsp_protocol.h"

# Lock file coordinating concurrent `nix build .#arduino_bridge` across
# ArduinoModule instances in the same blueprint.
_BRIDGE_BUILD_LOCK_PATH = _ARDUINO_HW_DIR / ".bridge_build.lock"


@functools.lru_cache(maxsize=1)
def _arduino_tools_bin_dir() -> Path:
    """Resolve dimos_arduino_tools via ``nix build`` and return its bin/ path.

    Uses nix rather than $PATH so the module works without ``nix develop``.
    Cached for the process lifetime; no-op after the first cold build.
    """
    logger.info("Resolving dimos_arduino_tools via nix build")
    try:
        result = subprocess.run(
            [
                "nix",
                "build",
                ".#dimos_arduino_tools",
                "--print-out-paths",
                "--no-link",
            ],
            cwd=str(_ARDUINO_HW_DIR),
            capture_output=True,
            text=True,
            timeout=600,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            "`nix` not found on PATH. ArduinoModule resolves its Arduino "
            "toolchain (arduino-cli, avrdude, qemu-system-avr) through the "
            f"flake at {_ARDUINO_HW_DIR}, so the `nix` CLI must be "
            "installed and on PATH. Install Nix (https://nixos.org/download) "
            "and re-run."
        ) from exc
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to materialize `dimos_arduino_tools` via "
            "`nix build .#dimos_arduino_tools`:\n"
            f"{result.stderr}\n{result.stdout}"
        )
    # `--print-out-paths` prints one store path per output on stdout;
    # we have a single output, so the last non-empty line is it.
    out_paths = [line for line in result.stdout.splitlines() if line.strip()]
    if not out_paths:
        raise RuntimeError(
            "`nix build .#dimos_arduino_tools --print-out-paths` returned "
            "no paths on stdout. This should never happen; please file a "
            "bug against the arduino flake."
        )
    return Path(out_paths[-1]) / "bin"


def _arduino_msgs_dir() -> Path:
    """Return the path to generated Arduino LCM message headers (share/arduino_msgs/)."""
    tools_bin = _arduino_tools_bin_dir()
    msgs_dir = tools_bin.parent / "share" / "arduino_msgs"
    if not msgs_dir.is_dir():
        raise RuntimeError(
            f"Expected Arduino message headers at {msgs_dir} but directory "
            "does not exist. Rebuild dimos_arduino_tools: "
            "nix build .#dimos_arduino_tools"
        )
    return msgs_dir


def _arduino_cli_bin() -> str:
    return str(_arduino_tools_bin_dir() / "arduino-cli")


def _avrdude_bin() -> str:
    return str(_arduino_tools_bin_dir() / "avrdude")


def _qemu_system_avr_bin() -> str:
    return str(_arduino_tools_bin_dir() / "qemu-system-avr")


@dataclass
class CTypeGenerator:
    """Override for generating C struct/encode/decode for a message type."""

    struct_create: Any  # Callable[[str], str]  — (type_name) -> C code
    encode_create: Any | None = None  # Callable[[str, str, int], str]
    decode_create: Any | None = None  # Callable[[str, str, int], str]


# Registry of known Arduino-compatible message type header paths.
#
# This list is kept in sync with two other places:
#   - dimos/hardware/arduino/cpp/main.cpp :: init_hash_registry()
#   - dimos-lcm :: generated/arduino_c_msgs/**
# `tests/test_arduino_msg_registry_sync.py` fails CI if any drift appears.
_KNOWN_TYPE_HEADERS: dict[str, str] = {
    "std_msgs.Time": "std_msgs/Time.h",
    "std_msgs.Bool": "std_msgs/Bool.h",
    "std_msgs.Int32": "std_msgs/Int32.h",
    "std_msgs.Float32": "std_msgs/Float32.h",
    "std_msgs.Float64": "std_msgs/Float64.h",
    "std_msgs.ColorRGBA": "std_msgs/ColorRGBA.h",
    "geometry_msgs.Vector3": "geometry_msgs/Vector3.h",
    "geometry_msgs.Point": "geometry_msgs/Point.h",
    "geometry_msgs.Point32": "geometry_msgs/Point32.h",
    "geometry_msgs.Quaternion": "geometry_msgs/Quaternion.h",
    "geometry_msgs.Pose": "geometry_msgs/Pose.h",
    "geometry_msgs.Pose2D": "geometry_msgs/Pose2D.h",
    "geometry_msgs.Twist": "geometry_msgs/Twist.h",
    "geometry_msgs.Accel": "geometry_msgs/Accel.h",
    "geometry_msgs.Transform": "geometry_msgs/Transform.h",
    "geometry_msgs.Wrench": "geometry_msgs/Wrench.h",
    "geometry_msgs.Inertia": "geometry_msgs/Inertia.h",
    "geometry_msgs.PoseWithCovariance": "geometry_msgs/PoseWithCovariance.h",
    "geometry_msgs.TwistWithCovariance": "geometry_msgs/TwistWithCovariance.h",
    "geometry_msgs.AccelWithCovariance": "geometry_msgs/AccelWithCovariance.h",
}


class ArduinoModuleConfig(NativeModuleConfig):
    def to_cli_args(self) -> list[str]:
        # Bridge CLI is built explicitly in start() — suppress generic emission.
        return []

    # Sketch
    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"

    # Bridge binary (generic, same for all modules)
    executable: str = "result/bin/arduino_bridge"
    build_command: str = "nix build .#arduino_bridge"
    cwd: str | None = None

    # Connection
    port: str | None = None
    baudrate: int = 115200
    auto_detect: bool = True
    auto_reconnect: bool = True
    reconnect_interval: float = 2.0

    # Virtual mode (QEMU emulator instead of real hardware)
    virtual: bool = False
    qemu_startup_timeout_s: float = 5.0

    # Flash
    auto_flash: bool = True
    flash_timeout: float = 60.0

    # Compile-time tuning (passed as -D flags to arduino-cli)
    # These override the defaults in dimos_lcm_pubsub.h / dsp_protocol.h.
    # Set to None (the default) to keep the header's built-in default.
    max_subs: int | None = None  # DIMOS_LCM_MAX_SUBS (AVR default: 4)
    max_pending: int | None = None  # DIMOS_LCM_MAX_PENDING (AVR default: 2)
    max_msg_size: int | None = None  # DIMOS_LCM_MAX_MSG_SIZE (AVR default: 64)
    max_payload: int | None = None  # DSP_MAX_PAYLOAD (AVR default: 256)

    # Arbitrary user-defined #defines emitted in dimos_arduino.h and
    # passed as -D compiler flags.  Example:
    #   arduino_defines={"MOTOR_PIN": 13, "SENSOR_THRESHOLD": 0.5}
    # becomes:
    #   #define MOTOR_PIN 13
    #   #define SENSOR_THRESHOLD 0.5f
    arduino_defines: dict[str, int | float | str | bool] = {}

    # Subclass fields to exclude from the generated #define embedding.
    # Framework fields are excluded automatically; this is for user fields
    # that shouldn't reach the sketch.
    arduino_config_exclude: frozenset[str] = frozenset()


# Framework fields that DO get embedded in the sketch header.
_ARDUINO_SKETCH_FIELDS: frozenset[str] = frozenset({"baudrate"})


# Must match #ifdef __AVR__ in dsp_protocol.h.
_AVR_DEFAULT_DSP_MAX_PAYLOAD = 256

# Mapping from ArduinoModuleConfig field names to C preprocessor macros
# for the compile-time tuning knobs.
_TUNING_FIELD_TO_DEFINE: dict[str, str] = {
    "max_subs": "DIMOS_LCM_MAX_SUBS",
    "max_pending": "DIMOS_LCM_MAX_PENDING",
    "max_msg_size": "DIMOS_LCM_MAX_MSG_SIZE",
    "max_payload": "DSP_MAX_PAYLOAD",
}


def _c_literal(name: str, val: int | float | str | bool) -> str:
    """Convert a Python value to a C literal string for ``#define``."""
    if isinstance(val, bool):
        return "1" if val else "0"
    if isinstance(val, int):
        return str(val)
    if isinstance(val, float):
        if not math.isfinite(val):
            raise ValueError(
                f"Cannot embed non-finite float for arduino_defines key {name!r} (value={val!r})"
            )
        return f"{val}f"
    if isinstance(val, str):
        return json.dumps(val)
    raise TypeError(
        f"arduino_defines key {name!r} has unsupported type "
        f"{type(val).__name__}. Use int, float, str, or bool."
    )


_AVR_FQBN_PREFIXES: tuple[str, ...] = ("arduino:avr:",)


class ArduinoModule(NativeModule):
    """Manages an Arduino board: generates header, compiles sketch, flashes,
    and runs a C++ serial↔LCM bridge.
    """

    config: ArduinoModuleConfig
    c_type_generators: ClassVar[dict[type, CTypeGenerator]] = {}

    _qemu_proc: subprocess.Popen[bytes] | None = None
    _virtual_pty: str | None = None
    _qemu_log_path: str | None = None
    _qemu_log_fd: IO[bytes] | None = None
    _bridge_bin: str | None = None

    @rpc
    def build(self) -> None:
        """Generate header, compile sketch, build bridge, and optionally flash."""
        if not self.config.virtual and self.config.auto_detect and not self.config.port:
            self.config.port = self._detect_port()
            logger.info("Auto-detected Arduino port", port=self.config.port)

        self._generate_header()
        self._ensure_core_installed()
        self._compile_sketch()
        self._build_bridge()
        self._bridge_bin = str(_ARDUINO_HW_DIR / "result" / "bin" / "arduino_bridge")

        if not self.config.virtual and self.config.auto_flash and self.config.port:
            self._flash()

    def _build_bridge(self) -> None:
        """Build the C++ bridge via nix. File-locked to handle concurrent modules."""
        import fcntl  # POSIX-only; deferred here so the module can be imported on Windows

        bridge_bin = _ARDUINO_HW_DIR / "result" / "bin" / "arduino_bridge"

        # Ensure the lock file exists (nix flake dir is always present).
        _BRIDGE_BUILD_LOCK_PATH.touch(exist_ok=True)

        with open(_BRIDGE_BUILD_LOCK_PATH, "w") as lock_fh:
            fcntl.flock(lock_fh.fileno(), fcntl.LOCK_EX)
            try:
                logger.info("Building arduino_bridge via nix flake")
                result = subprocess.run(
                    ["nix", "build", ".#arduino_bridge"],
                    cwd=str(_ARDUINO_HW_DIR),
                    capture_output=True,
                    text=True,
                    timeout=600,
                )
                if result.returncode != 0:
                    raise RuntimeError(
                        f"arduino_bridge build failed:\n{result.stderr}\n{result.stdout}"
                    )
                if not bridge_bin.exists():
                    raise RuntimeError(
                        f"arduino_bridge build succeeded but binary missing: {bridge_bin}"
                    )
                logger.info("arduino_bridge built successfully", path=str(bridge_bin))
            finally:
                fcntl.flock(lock_fh.fileno(), fcntl.LOCK_UN)

    def _resolve_topics(self) -> dict[str, str]:
        """Collect topics and validate they have the ``channel#msg_type`` shape
        the bridge requires (only LCMTransport produces this).
        """
        raw = super()._collect_topics()
        bad: list[tuple[str, str]] = []
        for stream_name, channel in raw.items():
            if "#" not in channel:
                bad.append((stream_name, channel))
        if bad:
            bad_desc = ", ".join(f"{s!r}={c!r}" for s, c in bad)
            raise RuntimeError(
                f"ArduinoModule stream(s) {bad_desc} resolved to channel "
                f"strings without a '#msg_type' suffix.  The arduino_bridge "
                f"binary needs typed channels to look up LCM fingerprint "
                f"hashes.  Declare these streams with LCMTransport (the "
                f"default) rather than pLCMTransport / SHMTransport / etc., "
                f"or remap them to use LCM."
            )
        return raw

    def _build_topic_args(self) -> list[str]:
        # The bridge uses its own CLI schema (--topic_in <id> <channel>,
        # --topic_out <id> <channel>) built in start(), so suppress the
        # generic --<stream_name> <topic> args from NativeModule.
        return []

    @rpc
    def start(self) -> None:
        """Launch the C++ bridge subprocess (and QEMU if virtual)."""
        topics = self._resolve_topics()
        topic_enum = self._build_topic_enum()

        if self.config.virtual:
            serial_port = self._start_qemu()
        else:
            serial_port = self.config.port or "/dev/ttyACM0"

        bridge_args = [
            "--serial_port",
            serial_port,
            "--baudrate",
            str(self.config.baudrate),
            "--reconnect",
            str(self.config.auto_reconnect).lower(),
            "--reconnect_interval",
            str(self.config.reconnect_interval),
        ]

        for stream_name, topic_id in topic_enum.items():
            if stream_name not in topics:
                continue
            lcm_channel = topics[stream_name]
            if stream_name in self.outputs:
                bridge_args.extend(["--topic_out", str(topic_id), lcm_channel])
            elif stream_name in self.inputs:
                bridge_args.extend(["--topic_in", str(topic_id), lcm_channel])

        if self._bridge_bin is not None:
            self.config.executable = self._bridge_bin

        # Save/restore extra_args so start()/stop() cycles don't accumulate.
        user_extra = list(self.config.extra_args)
        self.config.extra_args = user_extra + bridge_args
        try:
            super().start()
        except BaseException:
            # If the bridge itself failed to launch we still need to tear
            # down any QEMU process we just brought up.
            self._cleanup_qemu()
            raise
        finally:
            self.config.extra_args = user_extra

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            self._cleanup_qemu()

    def _cleanup_qemu(self) -> None:
        """Tear down QEMU state. Safe to call even if never started."""
        if self._qemu_proc is not None:
            try:
                if self._qemu_proc.poll() is None:
                    self._qemu_proc.terminate()
                    try:
                        self._qemu_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self._qemu_proc.kill()
                        try:
                            self._qemu_proc.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            logger.error(
                                "QEMU did not exit after SIGKILL",
                                pid=self._qemu_proc.pid,
                            )
            finally:
                self._qemu_proc = None

        if self._qemu_log_fd is not None:
            try:
                self._qemu_log_fd.close()
            except OSError:
                pass
            self._qemu_log_fd = None

        if self._qemu_log_path is not None:
            try:
                os.unlink(self._qemu_log_path)
            except FileNotFoundError:
                pass
            except OSError as exc:
                logger.warning(
                    "Failed to remove QEMU log file",
                    path=self._qemu_log_path,
                    error=str(exc),
                )
            self._qemu_log_path = None

        if self._virtual_pty is not None:
            logger.info("QEMU virtual Arduino stopped")
            self._virtual_pty = None

    @rpc
    def flash(self) -> None:
        self._flash()

    def _get_stream_types(self) -> dict[str, type]:
        hints = get_type_hints(type(self))
        result: dict[str, type] = {}
        for name, hint in hints.items():
            origin = get_origin(hint)
            if origin is In or origin is Out:
                args = get_args(hint)
                if args:
                    result[name] = args[0]
        return result

    # Topic IDs are transmitted as 2 bytes on the wire (DSP protocol)
    # with id 0 reserved for the debug channel, leaving 1..65534 usable.
    MAX_TOPICS: ClassVar[int] = 65534

    def _validate_inbound_payload_sizes(self, stream_types: dict[str, type]) -> None:
        """Reject inbound streams that exceed AVR's DSP_MAX_PAYLOAD (256B).

        Only checks AVR targets — non-AVR boards use a 1024B buffer.
        Only checks inbound (In[T]) — outbound is the Arduino's own buffer.
        """
        if not self.config.board_fqbn.startswith(_AVR_FQBN_PREFIXES):
            return

        limit = self.config.max_payload or _AVR_DEFAULT_DSP_MAX_PAYLOAD
        offenders: list[tuple[str, str, int]] = []
        for name, msg_type in stream_types.items():
            if name not in self.inputs:
                continue  # outbound — Arduino owns the encoder, not our problem
            size = _encoded_payload_size(msg_type)
            if size is None:
                continue  # custom type via c_type_generators — trust the user
            if size > limit:
                offenders.append((name, msg_type.__name__, size))
        if offenders:
            desc = "; ".join(
                f"{name!r}: {type_name}={size}B" for name, type_name, size in offenders
            )
            raise ValueError(
                f"ArduinoModule inbound stream(s) exceed the AVR "
                f"DSP_MAX_PAYLOAD limit of {limit} bytes ({desc}). The "
                f"AVR-side parser would silently drop every frame. "
                f"Either (a) split the message into smaller types, "
                f"(b) target a non-AVR board with more SRAM (e.g. "
                f"esp32:esp32:*), or (c) if you know your board has "
                f"enough SRAM, override the buffer in your sketch "
                f"via `-DDSP_MAX_PAYLOAD=<bigger>` in compile flags "
                f"and remove this check by subclassing "
                f"`_validate_inbound_payload_sizes`."
            )

    def _warn_avr_sram_pressure(self, stream_types: dict[str, type]) -> None:
        """Log a warning if the number of streams is likely to overflow AVR SRAM.

        Each stream adds a subscription entry (~9 bytes), a topic mapping
        entry, and the type's encode/decode code.  The LCM pubsub engine
        itself uses ~256 bytes (decode buffer + outbox) on AVR.  With
        Arduino Uno's 2KB total SRAM, more than ~4 streams is risky.
        """
        if not self.config.board_fqbn.startswith(_AVR_FQBN_PREFIXES):
            return
        n = len(stream_types)
        threshold = self.config.max_subs or 4
        if n > threshold:
            logger.warning(
                "AVR SRAM pressure: %d streams declared (max_subs=%d) on a "
                "board with ~2KB SRAM. Each stream adds subscriptions, type "
                "descriptors, and encode/decode buffers. Compilation may fail "
                "with 'data section exceeds available space'. Consider "
                "reducing streams, increasing max_subs (if your board has "
                "enough SRAM), or using a board with more SRAM.",
                n,
                threshold,
            )

    def _build_topic_enum(self) -> dict[str, int]:
        stream_types = self._get_stream_types()
        max_topics = 65534
        if len(stream_types) > max_topics:
            raise ValueError(
                f"{type(self).__name__} declares {len(stream_types)} streams, "
                f"but ArduinoModule supports at most {max_topics} (topic "
                f"IDs are uint16_t with 0 reserved for the debug channel). "
                f"Split the module or drop streams."
            )
        topic_enum: dict[str, int] = {}
        topic_id = 1
        for name in sorted(stream_types.keys()):
            topic_enum[name] = topic_id
            topic_id += 1
        return topic_enum

    def _detect_port(self) -> str:
        """Find port whose FQBN matches config.board_fqbn, or raise."""
        result = subprocess.run(
            [_arduino_cli_bin(), "board", "list", "--format", "json"],
            capture_output=True,
            text=True,
            timeout=10,
        )

        if result.returncode != 0:
            raise RuntimeError(f"arduino-cli board list failed: {result.stderr}")

        try:
            boards = json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"arduino-cli board list returned invalid JSON: {exc}\n"
                f"stdout was:\n{result.stdout[:4096]}"
            ) from exc

        # Accept both old (bare list) and new ({"detected_ports": [...]}) formats.
        if isinstance(boards, list):
            entries = boards
        elif isinstance(boards, dict):
            entries = boards.get("detected_ports", [])
        else:
            entries = []

        # Search for a port whose matching_boards contains our FQBN.
        for entry in entries:
            port_info = entry if isinstance(entry, dict) else {}
            address = str(port_info.get("port", {}).get("address", ""))
            matching_boards = port_info.get("matching_boards", [])
            for board in matching_boards:
                if board.get("fqbn", "") == self.config.board_fqbn:
                    return address

        raise RuntimeError(
            f"No Arduino board found matching FQBN '{self.config.board_fqbn}'. "
            f"Connected ports: {sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))}. "
            f"Run 'arduino-cli board list' to see what arduino-cli can see, "
            f"or set `port=...` explicitly on your module config."
        )

    def _generate_header(self) -> None:
        """Generate dimos_arduino.h from stream declarations + config."""
        stream_types = self._get_stream_types()
        topic_enum = self._build_topic_enum()
        self._validate_inbound_payload_sizes(stream_types)
        self._warn_avr_sram_pressure(stream_types)

        sections: list[str] = []

        sections.append(
            "/* Auto-generated by DimOS ArduinoModule — do not edit */\n"
            "#ifndef DIMOS_ARDUINO_H\n"
            "#define DIMOS_ARDUINO_H\n"
        )

        # Emit #defines for user-defined config fields + _ARDUINO_SKETCH_FIELDS.
        # Framework-internal fields (executable, virtual, etc.) are skipped.
        sections.append("/* --- Config --- */")
        framework_fields = set(ArduinoModuleConfig.model_fields)
        emit_framework = framework_fields & _ARDUINO_SKETCH_FIELDS
        ignore_fields = (framework_fields - emit_framework) | set(
            self.config.arduino_config_exclude
        )
        for field_name in self.config.__class__.model_fields:
            if field_name in ignore_fields:
                continue
            val = getattr(self.config, field_name)
            if val is None:
                continue
            c_name = f"DIMOS_{field_name.upper()}"
            if isinstance(val, bool):
                sections.append(f"#define {c_name} {'1' if val else '0'}")
            elif isinstance(val, int):
                sections.append(f"#define {c_name} {val}")
            elif isinstance(val, float):
                if not math.isfinite(val):
                    raise ValueError(
                        f"Cannot embed non-finite float for config field "
                        f"'{field_name}' (value={val!r}) in dimos_arduino.h"
                    )
                sections.append(f"#define {c_name} {val}f")
            elif isinstance(val, str):
                # json.dumps produces a valid C string literal (escapes ",
                # \, and non-printables; wraps in double quotes).
                sections.append(f"#define {c_name} {json.dumps(val)}")
            else:
                raise TypeError(
                    f"Cannot embed config field '{field_name}' of type "
                    f"{type(val).__name__} in dimos_arduino.h. Add it to "
                    f"arduino_config_exclude or convert it to str/int/float/bool."
                )
        sections.append("")

        # User-defined #defines from arduino_defines config.
        if self.config.arduino_defines:
            sections.append("/* --- User-defined constants --- */")
            for def_name, def_val in self.config.arduino_defines.items():
                if not def_name.isidentifier():
                    raise ValueError(
                        f"arduino_defines key {def_name!r} is not a valid C identifier."
                    )
                sections.append(f"#define {def_name} {_c_literal(def_name, def_val)}")
            sections.append("")

        # Topic enum (still used by bridge CLI args and backward compat)
        sections.append("/* --- Topic enum (shared with C++ bridge) --- */")
        sections.append("enum dimos_topic {")
        sections.append("    DIMOS_TOPIC_DEBUG = 0,")
        for name, tid in topic_enum.items():
            direction = "Out" if name in self.outputs else "In"
            msg_type = stream_types[name]
            sections.append(
                f"    DIMOS_TOPIC__{name.upper()} = {tid},  /* {direction}[{msg_type.__name__}] */"
            )
        sections.append("};")
        sections.append("")

        # LCM pubsub layer (must come before message headers so type
        # descriptors defined in the message headers can reference it)
        sections.append("/* --- LCM pubsub layer --- */")
        sections.append('#include "dimos_lcm_pubsub.h"')
        sections.append("")

        # Message type includes
        sections.append("/* --- Message type headers --- */")
        included_types: set[str] = set()
        for _name, msg_type in stream_types.items():
            msg_name = getattr(msg_type, "msg_name", None)
            if msg_name is None:
                msg_name = f"{msg_type.__module__}.{msg_type.__qualname__}"

            if msg_name in included_types:
                continue
            included_types.add(msg_name)

            header = _KNOWN_TYPE_HEADERS.get(msg_name)
            if header:
                sections.append(f'#include "{header}"')
            elif msg_type in self.c_type_generators:
                gen = self.c_type_generators[msg_type]
                sections.append(gen.struct_create(msg_type.__name__))
            else:
                raise TypeError(
                    f"No Arduino C header for message type '{msg_name}'. "
                    f"Either add it to arduino_msgs/ or set c_type_generators "
                    f"on your ArduinoModule subclass."
                )
        sections.append("")

        # Topic mapping table (channel name ↔ topic ID)
        # The bridge uses "topic_name#msg_type" as the LCM channel.
        # For the Arduino side, we use just the topic_name part as the
        # channel name for the subscribe API.
        try:
            topics = self._resolve_topics()
        except Exception:
            # During unit tests or when transports aren't wired yet,
            # fall back to stream names as channel names.
            topics = {name: name for name in topic_enum}
        sections.append("/* --- Topic ↔ channel mapping --- */")
        sections.append("#ifndef DIMOS_TOPIC_MAPPING_DEFINED")
        sections.append("#define DIMOS_TOPIC_MAPPING_DEFINED")
        sections.append("typedef struct {")
        sections.append("    uint16_t    topic_id;")
        sections.append("    const char *channel;")
        sections.append("} dimos_topic_mapping_t;")
        sections.append("#endif")
        sections.append(f"#define DIMOS_NUM_TOPICS {len(topic_enum)}")
        sections.append("static const dimos_topic_mapping_t _dimos_topic_map[] = {")
        for name, tid in topic_enum.items():
            if name in topics:
                # Extract channel name (before #) for the Arduino-side API
                lcm_channel = topics[name]
                channel_name = lcm_channel.split("#")[0]
            else:
                channel_name = name
            sections.append(f'    {{ {tid}, "{channel_name}" }},')
        sections.append("};")
        sections.append("")

        # Channel name constants for user convenience
        sections.append("/* --- Channel name constants --- */")
        for name in topic_enum:
            if name in topics:
                channel_name = topics[name].split("#")[0]
            else:
                channel_name = name
            sections.append(f'#define DIMOS_CHANNEL__{name.upper()} "{channel_name}"')
        sections.append("")

        # DSP protocol + LCM serial adapter
        sections.append("/* --- Serial transport + LCM integration --- */")
        sections.append('#include "dimos_lcm_serial.h"')
        sections.append("")

        # Close header guard
        sections.append("#endif /* DIMOS_ARDUINO_H */")

        # Header must live in the sketch dir — arduino-cli's preprocessor
        # ignores -I flags during its ctags pass. Wipe build dir to avoid
        # stale includes.cache causing "not declared" errors.
        sketch_dir = self._resolve_sketch_dir()
        header_path = sketch_dir / "dimos_arduino.h"
        header_path.write_text("\n".join(sections))
        build_dir = self._build_dir()
        if build_dir.exists():
            shutil.rmtree(build_dir)
        build_dir.mkdir(parents=True, exist_ok=True)
        logger.info("Generated Arduino header", path=str(header_path))

    def _resolve_sketch_dir(self) -> Path:
        subclass_file = Path(inspect.getfile(type(self)))
        base_dir = subclass_file.parent
        if self.config.cwd:
            base_dir = base_dir / self.config.cwd
        sketch_path = base_dir / self.config.sketch_path
        return sketch_path.parent

    def _build_dir(self) -> Path:
        sketch_dir = self._resolve_sketch_dir()
        return sketch_dir / "build"

    def _ensure_core_installed(self) -> None:
        """Install the arduino-cli core for board_fqbn if not already present."""
        # board_fqbn is e.g. "arduino:avr:uno" — the core id is the first
        # two colon-separated segments: "arduino:avr".
        parts = self.config.board_fqbn.split(":")
        if len(parts) < 2:
            raise RuntimeError(
                f"Invalid board_fqbn {self.config.board_fqbn!r}; "
                f"expected 'vendor:architecture:board' (e.g. 'arduino:avr:uno')"
            )
        core_id = f"{parts[0]}:{parts[1]}"

        arduino_cli = _arduino_cli_bin()

        # Cheap check first: `arduino-cli core list` prints installed cores.
        # If our core is already there, skip the install step entirely.
        list_result = subprocess.run(
            [arduino_cli, "core", "list"],
            capture_output=True,
            text=True,
            timeout=30,
        )
        if list_result.returncode == 0 and any(
            line.split()[0] == core_id for line in list_result.stdout.splitlines() if line.strip()
        ):
            return

        logger.info("Installing arduino core", core=core_id)
        install_result = subprocess.run(
            [arduino_cli, "core", "install", core_id],
            capture_output=True,
            text=True,
            timeout=600,
        )
        if install_result.returncode != 0:
            raise RuntimeError(
                f"arduino-cli core install {core_id} failed:\n"
                f"{install_result.stderr}\n{install_result.stdout}"
            )
        logger.info("Arduino core installed", core=core_id)

    def _compile_sketch(self) -> None:
        sketch_dir = self._resolve_sketch_dir()
        build_dir = self._build_dir()
        build_dir.mkdir(parents=True, exist_ok=True)

        common = str(_COMMON_DIR)
        msgs = str(_arduino_msgs_dir())
        extra_flags = f"-I{common} -I{msgs} -DF_CPU=16000000UL"
        if self.config.virtual:
            # QEMU AVR doesn't fire USART interrupts — use direct register I/O.
            extra_flags += " -DDSP_DIRECT_USART"

        # Compile-time tuning knobs from config fields.
        for field, macro in _TUNING_FIELD_TO_DEFINE.items():
            val = getattr(self.config, field)
            if val is not None:
                extra_flags += f" -D{macro}={val}"

        # User-defined #defines also passed as -D flags so they're
        # visible in all translation units, not just via the header.
        for def_name, def_val in self.config.arduino_defines.items():
            extra_flags += f" -D{def_name}={_c_literal(def_name, def_val)}"

        cmd = [
            _arduino_cli_bin(),
            "compile",
            "--fqbn",
            self.config.board_fqbn,
            "--build-property",
            f"compiler.cpp.extra_flags={extra_flags}",
            "--build-property",
            f"compiler.c.extra_flags={extra_flags}",
            "--build-path",
            str(build_dir),
            str(sketch_dir),
        ]

        logger.info("Compiling Arduino sketch", cmd=" ".join(cmd))
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Arduino sketch compilation failed:\n{result.stderr}\n{result.stdout}"
            )
        logger.info("Arduino sketch compiled successfully", build_dir=str(build_dir))

    def _start_qemu(self) -> str:
        """Launch qemu-system-avr and return the PTY path. Cleans up on failure."""
        build_dir = self._build_dir()
        sketch_name = Path(self.config.sketch_path).stem
        elf_path = build_dir / f"{sketch_name}.ino.elf"
        if not elf_path.exists():
            raise RuntimeError(f"Compiled sketch not found: {elf_path}")

        machine_map = {
            "arduino:avr:uno": "uno",
            "arduino:avr:mega": "mega",
            "arduino:avr:mega2560": "mega2560",
        }
        machine = machine_map.get(self.config.board_fqbn, "uno")

        tmp_log = tempfile.NamedTemporaryFile(
            prefix="dimos_qemu_", suffix=".log", delete=False, mode="w"
        )
        self._qemu_log_path = tmp_log.name
        tmp_log.close()

        cmd = [
            _qemu_system_avr_bin(),
            "-machine",
            machine,
            "-bios",
            str(elf_path),
            "-serial",
            "pty",
            "-monitor",
            "null",
            "-nographic",
        ]

        logger.info("Starting QEMU virtual Arduino", cmd=" ".join(cmd))
        try:
            self._qemu_log_fd = open(self._qemu_log_path, "wb")
            self._qemu_proc = subprocess.Popen(
                cmd,
                stdout=self._qemu_log_fd,
                stderr=subprocess.STDOUT,
            )

            timeout = self.config.qemu_startup_timeout_s
            deadline = time.monotonic() + timeout
            pty: str | None = None
            while time.monotonic() < deadline:
                if self._qemu_proc.poll() is not None:
                    with open(self._qemu_log_path) as f:
                        raise RuntimeError(
                            f"QEMU exited unexpectedly before announcing a PTY:\n{f.read()}"
                        )
                with open(self._qemu_log_path) as f:
                    content = f.read()
                # Match "char device redirected to /dev/pts/N" or /dev/ttysNNN
                m = re.search(
                    r"char device redirected to (/dev/(?:pts/\d+|ttys\d+))",
                    content,
                )
                if m:
                    pty = m.group(1)
                    break
                time.sleep(0.1)

            if pty is None:
                raise RuntimeError(
                    f"QEMU started but did not announce a PTY within {timeout:.1f}s. "
                    f"Increase qemu_startup_timeout_s in the module config if "
                    f"this is a loaded CI machine. Log tail:\n"
                    f"{_tail_text(self._qemu_log_path, 2048)}"
                )

            self._virtual_pty = pty
            logger.info("QEMU virtual Arduino running", pty=pty, pid=self._qemu_proc.pid)
            return pty
        except BaseException:
            self._cleanup_qemu()
            raise

    def _flash(self) -> None:
        sketch_dir = self._resolve_sketch_dir()
        build_dir = self._build_dir()
        port = self.config.port
        if not port:
            raise RuntimeError("No port configured for flashing")

        cmd = [
            _arduino_cli_bin(),
            "upload",
            "-p",
            port,
            "--fqbn",
            self.config.board_fqbn,
            "--input-dir",
            str(build_dir),
            str(sketch_dir),
        ]

        logger.info("Flashing Arduino", cmd=" ".join(cmd), port=port)
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=self.config.flash_timeout,
        )
        if result.returncode != 0:
            combined = f"{result.stderr}\n{result.stdout}"
            hint = ""
            if "Permission denied" in combined and port:
                import sys

                if sys.platform == "linux":
                    hint = (
                        f"\n\nHint: the current user cannot access {port}. "
                        f"Quick fix:\n"
                        f"  sudo chmod 666 {port}\n"
                        f"Permanent fix (requires re-login):\n"
                        f"  sudo usermod -a -G dialout $USER"
                    )
                else:
                    hint = (
                        f"\n\nHint: the current user cannot access {port}. "
                        f"Check that your user has read/write access to the "
                        f"serial device."
                    )
            raise RuntimeError(f"Arduino flash failed:\n{combined}{hint}")
        logger.info("Arduino flashed successfully", port=port)


def _encoded_payload_size(msg_type: type) -> int | None:
    """Return the full DSP wire payload size (fingerprint + data), or None if
    the type can't be introspected.
    """
    try:
        instance = msg_type()
    except Exception:
        return None
    encode = getattr(instance, "lcm_encode", None)
    if encode is None:
        return None
    try:
        encoded = encode()
    except Exception:
        return None
    return len(encoded)


def _tail_text(path: str, max_bytes: int) -> str:
    try:
        with open(path, "rb") as f:
            try:
                f.seek(-max_bytes, os.SEEK_END)
            except OSError as exc:
                if exc.errno != errno.EINVAL:
                    raise
                f.seek(0)
            return f.read().decode(errors="replace")
    except OSError:
        return ""


__all__ = [
    "ArduinoModule",
    "ArduinoModuleConfig",
    "CTypeGenerator",
]
