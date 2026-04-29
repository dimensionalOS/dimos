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

"""Unit tests for dimos.core.arduino_module.

Covers the pure/host-side logic — header generation, topic enum
assignment, the three-way registry sync, port detection with mocked
arduino-cli, and QEMU cleanup paths.  These tests do not require a real
Arduino or QEMU.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
from typing import Any
from unittest import mock

import pytest

# Captured at import time — the autouse fixture below patches the module
# attribute for every test, so tests that want to exercise the *real*
# resolver (e.g. its FileNotFoundError handling) reach it through this
# unpatched reference.
from dimos.core.arduino_module import (
    _ARDUINO_HW_DIR,
    _KNOWN_TYPE_HEADERS,
    ArduinoModule,
    ArduinoModuleConfig,
    _arduino_tools_bin_dir as _real_arduino_tools_bin_dir,
)
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist

# Fixtures / helpers


@pytest.fixture(autouse=True)
def _fake_arduino_tools_bin_dir(
    tmp_path_factory: pytest.TempPathFactory,
) -> Any:
    """Short-circuit ``_arduino_tools_bin_dir`` for every test in this file.

    Without this, every helper that shells out to ``arduino-cli`` /
    ``qemu-system-avr`` (``_detect_port``, ``_ensure_core_installed``,
    ``_compile_sketch``, ``_start_qemu``, ``_flash``) would first invoke
    the resolver, which runs ``nix build .#dimos_arduino_tools``.  The
    unit tests are *unit* tests — they have no business touching the
    Nix store — so we replace the resolver with a fixed fake ``bin/``
    directory.  Tests that mock ``subprocess.run`` will then see calls
    routed through absolute paths under this fake dir, which is fine
    because the mocks don't care what the argv's first element is.
    """
    fake_bin = tmp_path_factory.mktemp("fake_arduino_tools") / "bin"
    fake_bin.mkdir()
    with mock.patch(
        "dimos.core.arduino_module._arduino_tools_bin_dir",
        return_value=fake_bin,
    ):
        yield fake_bin


class _ExampleConfig(ArduinoModuleConfig):
    """Minimal config for tests — no auto-detect, no flash, no virtual."""

    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"
    baudrate: int = 115200
    auto_detect: bool = False
    auto_flash: bool = False
    virtual: bool = False
    port: str | None = "/dev/ttyACM0"
    # Custom config field that should end up in the generated header.
    greeting: str = 'he said "hi"'
    tick_rate_hz: int = 50


class _ExampleModule(ArduinoModule):
    config: _ExampleConfig
    twist_in: In[Twist]
    twist_echo_out: Out[Twist]


def _make_module() -> _ExampleModule:
    """Build an _ExampleModule without triggering its __init__ machinery.

    ArduinoModule subclasses pydantic Module whose real `__init__` spins
    up RPC / worker plumbing we don't need for unit tests.  We use
    `__new__` to bypass it, then install bare `In` / `Out` stubs (via
    `__new__` again) into `self.__dict__` — the `Module.inputs` /
    `Module.outputs` properties are read-only `@property`s that scan
    `__dict__` for any attribute that is an `In`/`Out` instance, so
    this is the minimum required to make those properties return the
    expected port names.
    """
    inst = _ExampleModule.__new__(_ExampleModule)
    inst.config = _ExampleConfig()
    inst.__dict__["twist_in"] = In.__new__(In)
    inst.__dict__["twist_echo_out"] = Out.__new__(Out)
    return inst


# _build_topic_enum


def test_build_topic_enum_assigns_1_based_alphabetical() -> None:
    mod = _make_module()
    enum = mod._build_topic_enum()
    # Alphabetical order, topic 0 reserved for debug.
    assert enum == {"twist_echo_out": 1, "twist_in": 2}


# _generate_header — config embedding & escaping


def _patch_sketch_and_build_dirs(mod: _ExampleModule, tmp_path: Path) -> Any:
    """Redirect ``_resolve_sketch_dir`` and ``_build_dir`` into ``tmp_path``.

    ``_generate_header`` writes the header into the sketch dir (so
    arduino-cli's sketch preprocessor can find it) and also wipes +
    recreates the build dir.  Tests need both paths diverted away from
    the real repo.
    """
    sketch_dir = tmp_path
    build_dir = tmp_path / "build"
    sketch_patch = mock.patch.object(mod, "_resolve_sketch_dir", return_value=sketch_dir)
    build_patch = mock.patch.object(mod, "_build_dir", return_value=build_dir)
    return sketch_patch, build_patch


def test_generate_header_escapes_quoted_strings(tmp_path: Path) -> None:
    """A config string containing " or \\ must not produce invalid C."""
    mod = _make_module()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        mod._generate_header()
    text = (tmp_path / "dimos_arduino.h").read_text()

    # The greeting contains an embedded double-quote.  If the header
    # generator naively interpolated it, the resulting C file would have
    # an unterminated string literal.  `json.dumps` escapes it to \".
    assert r'#define DIMOS_GREETING "he said \"hi\""' in text
    assert "#define DIMOS_BAUDRATE 115200" in text
    assert "#define DIMOS_TICK_RATE_HZ 50" in text


def test_generate_header_includes_topic_enum_and_message_header(
    tmp_path: Path,
) -> None:
    mod = _make_module()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        mod._generate_header()
    text = (tmp_path / "dimos_arduino.h").read_text()

    assert "enum dimos_topic {" in text
    assert "DIMOS_TOPIC_DEBUG = 0" in text
    assert "DIMOS_TOPIC__TWIST_ECHO_OUT = 1" in text
    assert "DIMOS_TOPIC__TWIST_IN = 2" in text
    # Twist → geometry_msgs/Twist.h
    assert '#include "geometry_msgs/Twist.h"' in text
    # LCM pubsub layer and serial adapter always pulled in.
    assert '#include "dimos_lcm_pubsub.h"' in text
    assert '#include "dimos_lcm_serial.h"' in text


def test_generate_header_rejects_non_finite_float(tmp_path: Path) -> None:
    class _NaNConfig(_ExampleConfig):
        nan_val: float = float("nan")

    mod = _make_module()
    mod.config = _NaNConfig()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        with pytest.raises(ValueError, match="non-finite"):
            mod._generate_header()


def test_generate_header_rejects_unembeddable_type(tmp_path: Path) -> None:
    class _ListConfig(_ExampleConfig):
        the_list: list[int] = [1, 2, 3]

    mod = _make_module()
    mod.config = _ListConfig()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        with pytest.raises(TypeError, match="Cannot embed config field 'the_list'"):
            mod._generate_header()


def test_generate_header_includes_arduino_defines(tmp_path: Path) -> None:
    """arduino_defines config values appear as #define in the header."""

    class _DefinesConfig(_ExampleConfig):
        arduino_defines: dict[str, int | float | str | bool] = {
            "MOTOR_PIN": 13,
            "SENSOR_THRESHOLD": 0.5,
            "ROBOT_NAME": "uno_bot",
            "ENABLE_DEBUG": True,
        }

    mod = _make_module()
    mod.config = _DefinesConfig()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        mod._generate_header()
    text = (tmp_path / "dimos_arduino.h").read_text()

    assert "#define MOTOR_PIN 13" in text
    assert "#define SENSOR_THRESHOLD 0.5f" in text
    assert '#define ROBOT_NAME "uno_bot"' in text
    assert "#define ENABLE_DEBUG 1" in text
    assert "User-defined constants" in text


def test_generate_header_rejects_invalid_define_name(tmp_path: Path) -> None:
    """arduino_defines keys must be valid C identifiers."""

    class _BadDefConfig(_ExampleConfig):
        arduino_defines: dict[str, int | float | str | bool] = {"123bad": 1}

    mod = _make_module()
    mod.config = _BadDefConfig()
    sketch_patch, build_patch = _patch_sketch_and_build_dirs(mod, tmp_path)
    with sketch_patch, build_patch:
        with pytest.raises(ValueError, match="not a valid C identifier"):
            mod._generate_header()


def test_tuning_fields_default_to_none() -> None:
    """Tuning knobs default to None (use header defaults)."""
    cfg = _ExampleConfig()
    assert cfg.max_subs is None
    assert cfg.max_pending is None
    assert cfg.max_msg_size is None
    assert cfg.max_payload is None


# _detect_port — mocked arduino-cli


def _run_result(stdout: str, returncode: int = 0) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(
        args=["arduino-cli"], returncode=returncode, stdout=stdout, stderr=""
    )


def test_detect_port_matches_fqbn() -> None:
    mod = _make_module()
    payload: dict[str, Any] = {
        "detected_ports": [
            {
                "port": {"address": "/dev/ttyACM1"},
                "matching_boards": [{"fqbn": "arduino:avr:uno"}],
            },
            {
                "port": {"address": "/dev/ttyUSB0"},
                "matching_boards": [{"fqbn": "arduino:avr:mega"}],
            },
        ]
    }
    with mock.patch(
        "dimos.core.arduino_module.subprocess.run",
        return_value=_run_result(json.dumps(payload)),
    ):
        assert mod._detect_port() == "/dev/ttyACM1"


def test_detect_port_raises_on_no_match() -> None:
    mod = _make_module()
    payload: dict[str, list[Any]] = {"detected_ports": []}
    with mock.patch(
        "dimos.core.arduino_module.subprocess.run",
        return_value=_run_result(json.dumps(payload)),
    ):
        with pytest.raises(RuntimeError, match="No Arduino board found matching FQBN"):
            mod._detect_port()


def test_detect_port_wraps_invalid_json() -> None:
    mod = _make_module()
    with mock.patch(
        "dimos.core.arduino_module.subprocess.run",
        return_value=_run_result("not-json-at-all"),
    ):
        with pytest.raises(RuntimeError, match="invalid JSON"):
            mod._detect_port()


def test_arduino_tools_bin_dir_raises_on_missing_nix() -> None:
    """The resolver surfaces a clean RuntimeError (not a bare
    FileNotFoundError) when ``nix`` itself is missing from PATH.  That is
    the only failure mode of the toolchain resolver now that
    ``arduino-cli`` / ``avrdude`` / ``qemu-system-avr`` are packaged as
    a flake output and come from a ``nix build`` rather than from
    ``$PATH``.
    """
    # Clear the ``lru_cache`` so we actually re-enter the function body.
    _real_arduino_tools_bin_dir.cache_clear()
    with mock.patch(
        "dimos.core.arduino_module.subprocess.run",
        side_effect=FileNotFoundError,
    ):
        with pytest.raises(RuntimeError, match="nix"):
            _real_arduino_tools_bin_dir()
    # Leave the cache cleared so the next test (which may have its own
    # fake_arduino_tools_bin_dir fixture) starts from a known state.
    _real_arduino_tools_bin_dir.cache_clear()


def test_detect_port_wraps_non_zero_exit() -> None:
    mod = _make_module()
    with mock.patch(
        "dimos.core.arduino_module.subprocess.run",
        return_value=subprocess.CompletedProcess(
            args=[], returncode=1, stdout="", stderr="permission denied"
        ),
    ):
        with pytest.raises(RuntimeError, match="permission denied"):
            mod._detect_port()


# _cleanup_qemu — idempotency + leak sealing


def test_cleanup_qemu_is_idempotent_on_unstarted_module() -> None:
    mod = _make_module()
    # Never started — all slots None.  Must not raise.
    mod._cleanup_qemu()
    mod._cleanup_qemu()
    assert mod._qemu_proc is None
    assert mod._qemu_log_fd is None
    assert mod._qemu_log_path is None
    assert mod._virtual_pty is None


def test_cleanup_qemu_closes_log_fd_and_removes_log_file(tmp_path: Path) -> None:
    mod = _make_module()
    log_path = tmp_path / "qemu.log"
    log_path.write_text("hi")
    mod._qemu_log_path = str(log_path)
    fd = open(log_path, "wb")
    mod._qemu_log_fd = fd
    mod._qemu_proc = None  # no process — just the fd + file

    mod._cleanup_qemu()

    assert fd.closed
    assert not log_path.exists()
    assert mod._qemu_log_fd is None
    assert mod._qemu_log_path is None


def test_cleanup_qemu_terminates_live_process() -> None:
    mod = _make_module()
    proc = mock.Mock(spec=subprocess.Popen)
    # poll() returns None while alive, then 0 after wait.
    proc.poll.side_effect = [None]
    proc.wait.return_value = 0
    mod._qemu_proc = proc

    mod._cleanup_qemu()

    proc.terminate.assert_called_once()
    proc.wait.assert_called_once_with(timeout=5)
    assert mod._qemu_proc is None


def test_cleanup_qemu_kills_on_terminate_timeout() -> None:
    mod = _make_module()
    proc = mock.Mock(spec=subprocess.Popen)
    proc.poll.side_effect = [None]
    proc.wait.side_effect = [subprocess.TimeoutExpired(cmd="qemu", timeout=5), 0]
    mod._qemu_proc = proc

    mod._cleanup_qemu()

    proc.terminate.assert_called_once()
    proc.kill.assert_called_once()
    assert proc.wait.call_count == 2
    assert mod._qemu_proc is None


# Registry sync — _KNOWN_TYPE_HEADERS vs arduino_msgs/ vs main.cpp


def _arduino_common_dir() -> Path:
    """Resolve Arduino message headers from dimos-lcm via nix.

    These headers now live in dimos-lcm (not vendored in dimos4), so we
    resolve them through `nix build` the same way ArduinoModule does at
    runtime.  Skips if nix is not available.
    """
    import shutil
    import subprocess

    if shutil.which("nix") is None:
        pytest.skip("nix not available — cannot resolve Arduino message headers")

    result = subprocess.run(
        ["nix", "build", ".#dimos_arduino_tools", "--print-out-paths", "--no-link"],
        cwd=str(_ARDUINO_HW_DIR),
        capture_output=True,
        text=True,
        timeout=120,
    )
    if result.returncode != 0:
        pytest.skip(f"nix build failed: {result.stderr[:200]}")

    out_paths = [line for line in result.stdout.splitlines() if line.strip()]
    if not out_paths:
        pytest.skip("nix build returned no paths")

    msgs_dir = Path(out_paths[-1]) / "share" / "arduino_msgs"
    if not msgs_dir.is_dir():
        pytest.skip(f"Arduino message headers not found at {msgs_dir}")

    return msgs_dir


def _main_cpp_path() -> Path:
    return _ARDUINO_HW_DIR / "cpp" / "main.cpp"


def test_registry_headers_exist_on_disk() -> None:
    common = _arduino_common_dir()
    missing = [
        (msg_name, header)
        for msg_name, header in _KNOWN_TYPE_HEADERS.items()
        if not (common / header).is_file()
    ]
    assert not missing, (
        f"Every entry in _KNOWN_TYPE_HEADERS must point to an existing "
        f"arduino_msgs header, but these are missing: {missing}"
    )


def test_registry_matches_main_cpp_hash_registry() -> None:
    """Every type in `_KNOWN_TYPE_HEADERS` must also appear in the C++
    bridge's `init_hash_registry()` and vice versa.  Either half is a
    silent wire-format bug waiting to happen."""
    main_cpp = _main_cpp_path().read_text()

    # The C++ side stores keys as "std_msgs.Time" etc.
    cpp_entries = set(re.findall(r'hash_registry\["([^"]+)"\]', main_cpp))
    py_entries = set(_KNOWN_TYPE_HEADERS.keys())

    only_in_py = py_entries - cpp_entries
    only_in_cpp = cpp_entries - py_entries

    assert not only_in_py, (
        f"These message types are in _KNOWN_TYPE_HEADERS but NOT in "
        f"main.cpp::init_hash_registry: {sorted(only_in_py)}. Add them to "
        f"dimos/hardware/arduino/cpp/main.cpp or remove from the Python registry."
    )
    assert not only_in_cpp, (
        f"These message types are in main.cpp::init_hash_registry but NOT "
        f"in _KNOWN_TYPE_HEADERS: {sorted(only_in_cpp)}. Add them to "
        f"dimos/core/arduino_module.py::_KNOWN_TYPE_HEADERS or remove from main.cpp."
    )


# _resolve_topics — validates LCM-typed channel strings

# NOTE: these tests monkey-patch ``inputs``/``outputs`` on the ``_ExampleModule``
# *instance* rather than relying on stream auto-discovery, because
# ``_make_module`` uses bypass constructors that don't set up real
# transports.  The parent ``_collect_topics`` walks transports via
# ``getattr(self, name)._transport.topic``, so we stub the whole chain.


class _FakeTransport:
    def __init__(self, topic: str) -> None:
        self.topic = topic


class _FakeStream:
    def __init__(self, topic: str) -> None:
        self._transport = _FakeTransport(topic)


def _make_module_with_topics(topics: dict[str, str]) -> _ExampleModule:
    """Build a module whose `super()._collect_topics()` returns `topics`."""
    mod = _make_module()
    # Install fake streams so NativeModule._collect_topics walks them.
    for name, topic in topics.items():
        mod.__dict__[name] = _FakeStream(topic)
    # Force `inputs` / `outputs` to report the fake names (otherwise
    # the module-level reflection sees only the two In/Out stubs from
    # _make_module).  Monkey-patch the properties on this instance.
    inputs_list = list(topics)
    mod.__class__.inputs = property(lambda self: inputs_list)  # type: ignore[method-assign,assignment]
    mod.__class__.outputs = property(lambda self: [])  # type: ignore[method-assign,assignment]
    return mod


def test_resolve_topics_accepts_typed_lcm_channels() -> None:
    mod = _make_module_with_topics({"twist_in": "twist_command#geometry_msgs.Twist"})
    try:
        resolved = mod._resolve_topics()
        assert resolved == {"twist_in": "twist_command#geometry_msgs.Twist"}
    finally:
        # Unwind the monkey-patched properties so later tests see
        # the real Module.inputs/outputs descriptors.
        del mod.__class__.inputs
        del mod.__class__.outputs


def test_resolve_topics_rejects_bare_channel_names() -> None:
    mod = _make_module_with_topics({"twist_in": "twist_command"})
    try:
        with pytest.raises(RuntimeError, match="'#msg_type' suffix"):
            mod._resolve_topics()
    finally:
        del mod.__class__.inputs
        del mod.__class__.outputs


# _validate_inbound_payload_sizes — AVR SRAM guard


# Module-scope classes for the payload-size tests.  They must live at
# module level (not inside the test functions) because
# ``_get_stream_types`` uses ``get_type_hints`` which re-evaluates the
# string annotations via ``eval(..., globals=module.__dict__, ...)``
# and can't see locals of a test function.
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance


class _BigInboundModule(ArduinoModule):
    config: _ExampleConfig
    pose_in: In[PoseWithCovariance]


class _BigOutboundModule(ArduinoModule):
    config: _ExampleConfig
    pose_out: Out[PoseWithCovariance]


class _Esp32Config(_ExampleConfig):
    board_fqbn: str = "esp32:esp32:esp32"


class _Esp32Module(ArduinoModule):
    config: _Esp32Config
    pose_in: In[PoseWithCovariance]


def test_validate_inbound_payload_sizes_passes_for_small_inbound() -> None:
    """Twist is 56 bytes on the wire (8B fingerprint + 48B data) — well under the 256 AVR limit."""
    mod = _make_module()
    # twist_in is declared as In[Twist] — 48 bytes, passes.
    mod._validate_inbound_payload_sizes(mod._get_stream_types())


def test_validate_inbound_payload_sizes_rejects_oversized_inbound() -> None:
    """PoseWithCovariance is 352 bytes on the wire (8B fingerprint + 344B data) — exceeds the 256 AVR default."""
    mod = _BigInboundModule.__new__(_BigInboundModule)
    mod.config = _ExampleConfig()
    mod.__dict__["pose_in"] = In.__new__(In)

    with pytest.raises(ValueError, match="DSP_MAX_PAYLOAD"):
        mod._validate_inbound_payload_sizes(mod._get_stream_types())


def test_validate_inbound_payload_sizes_ignores_outbound() -> None:
    """Even an oversized *outbound* stream is fine — the Arduino owns the encoder."""
    mod = _BigOutboundModule.__new__(_BigOutboundModule)
    mod.config = _ExampleConfig()
    mod.__dict__["pose_out"] = Out.__new__(Out)

    mod._validate_inbound_payload_sizes(mod._get_stream_types())  # must not raise


def test_validate_inbound_payload_sizes_skips_non_avr_board() -> None:
    """A non-AVR FQBN skips the check entirely — non-AVR gets 1024."""
    mod = _Esp32Module.__new__(_Esp32Module)
    mod.config = _Esp32Config()
    mod.__dict__["pose_in"] = In.__new__(In)

    mod._validate_inbound_payload_sizes(mod._get_stream_types())  # must not raise


def test_registry_headers_cover_all_arduino_msgs_files() -> None:
    """Every header referenced by _KNOWN_TYPE_HEADERS must exist on disk.
    Extra generated headers (from dimos-lcm codegen) and infrastructure
    headers (lcm_coretypes_arduino.h, dimos_lcm_pubsub.h) are allowed
    without registry entries since they are auto-generated dependencies,
    not user-facing message types."""
    common = _arduino_common_dir()
    on_disk = {str(p.relative_to(common)) for p in common.rglob("*.h")}
    referenced = set(_KNOWN_TYPE_HEADERS.values())
    missing = referenced - on_disk
    assert not missing, (
        f"These headers are referenced by _KNOWN_TYPE_HEADERS but missing "
        f"on disk: {sorted(missing)}"
    )
