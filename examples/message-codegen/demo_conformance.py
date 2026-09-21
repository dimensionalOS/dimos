# Copyright 2026 Dimensional Inc.
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

"""Exercise real native encoders/decoders and an independent ROS2 codec."""

import argparse
from pathlib import Path
import subprocess

from dimos_generated.demo_msgs.msg import Telemetry
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


def relay(
    executable: Path, source: Path, target: Path, mode: str = "echo"
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(executable), mode, str(source), str(target)],
        text=True,
        capture_output=True,
        check=False,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    args = parser.parse_args()
    build = args.build.resolve()
    evidence = build / "evidence"
    evidence.mkdir(exist_ok=True)
    native = {"C++": build / "cpp-relay", "Rust": build / "rust/target/debug/relay"}
    reference = get_typestore(Stores.ROS2_JAZZY)
    reference.register(get_types_from_msg(Telemetry.schema, Telemetry.msg_name))
    defaults = Telemetry().encode()
    (evidence / "defaults-python.cdr").write_bytes(defaults)
    for language, executable in native.items():
        target = evidence / f"defaults-{language}.cdr"
        result = relay(executable, evidence / "defaults-python.cdr", target, "defaults")
        assert result.returncode == 0, result.stderr
        assert target.read_bytes() == defaults
    fixture = Telemetry(
        sequence=4294967295, hops=[-2147483648, 2147483647], payload=list(range(256))
    )
    fixture.header.stamp.sec = -1
    fixture.header.stamp.nanosec = 999999999
    fixture.header.frame_id = "map"
    fixture.label = "café"
    fixture.position.x = -1.25
    for endian in (True, False):
        encoded = fixture.encode(little_endian=endian)
        (evidence / ("python-le.cdr" if endian else "python-be.cdr")).write_bytes(encoded)
        (evidence / "source.cdr").write_bytes(encoded)
        decoded = reference.deserialize_cdr(encoded, Telemetry.msg_name)
        assert decoded.header.stamp.sec == -1
        assert decoded.sequence == 4294967295
        assert decoded.label == "café"
        assert decoded.reading.temperature == 21.5
        assert decoded.position.x == -1.25
        assert list(decoded.hops) == [-2147483648, 2147483647]
        assert list(decoded.payload) == list(range(256))
        canonical = bytes(reference.serialize_cdr(decoded, Telemetry.msg_name, little_endian=True))
        assert encoded == bytes(
            reference.serialize_cdr(decoded, Telemetry.msg_name, little_endian=endian)
        )
        produced = {"Python": encoded}
        for language, executable in native.items():
            target = evidence / f"{language}.cdr"
            result = relay(
                executable, evidence / "source.cdr", target, "echo" if endian else "echo-be"
            )
            assert result.returncode == 0, result.stderr
            produced[language] = target.read_bytes()
            assert produced[language] == encoded
        for producer, payload in produced.items():
            decoded_python = Telemetry.decode(payload)
            assert decoded_python.encode() == canonical
            print(f"{'LE' if endian else 'BE'} {producer:6} -> Python: fields and bytes match")
            (evidence / "matrix.cdr").write_bytes(payload)
            for consumer, executable in native.items():
                result = relay(executable, evidence / "matrix.cdr", evidence / "decoded.cdr")
                assert result.returncode == 0, result.stderr
                assert (evidence / "decoded.cdr").read_bytes() == canonical
                print(
                    f"{'LE' if endian else 'BE'} {producer:6} -> {consumer:6}: fields and bytes match"
                )
    encoded = fixture.encode()
    malformed = {
        "truncated": encoded[:-1],
        "unsupported representation": b"\x00\x07\x00\x00" + encoded[4:],
        "trailing bytes": encoded + b"\x00",
        "missing header": b"\x00",
    }
    for reason, payload in malformed.items():
        try:
            Telemetry.decode(payload)
        except (ValueError, RuntimeError):
            pass
        else:
            raise AssertionError(f"Python accepted {reason}")
        (evidence / "invalid.cdr").write_bytes(payload)
        for language, executable in native.items():
            result = relay(executable, evidence / "invalid.cdr", evidence / "invalid-output.cdr")
            assert result.returncode == 1, (language, reason, result.stderr)
        print(f"Python/C++/Rust reject {reason}")
    print(
        "All nine encoder/decoder combinations and both input byte orders match the independent ROS2 codec."
    )


if __name__ == "__main__":
    main()
