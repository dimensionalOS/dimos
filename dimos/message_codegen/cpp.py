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

"""Generate native C++ value types and Fast CDR customization functions."""

from __future__ import annotations

from contextlib import redirect_stdout
from dataclasses import asdict
from hashlib import sha256
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from ._vendor.rosidl.rosidl_adapter.api import convert_to_idl
from ._vendor.rosidl.rosidl_pycommon.api import generate_files
from .definitions import Message


def validation(message: Message) -> list[str]:
    lines = []
    for field in message.fields:
        member = field.name
        type_ = field.type
        if type_.array_bounded:
            lines.append(
                f'if ({member}.size() > {type_.array_size}) throw std::length_error("{field.name} exceeds sequence bound");'
            )
        value = "item" if type_.is_array else member
        checks = []
        if type_.string_bound is not None:
            checks.append(
                f'if ({value}.size() > {type_.string_bound}) throw std::length_error("{field.name} exceeds string bound");'
            )
        if type_.nested:
            checks.append(f"{value}.validate();")
        if checks and type_.is_array:
            lines.append(f"for (const auto& item : {member}) {{ {' '.join(checks)} }}")
        else:
            lines.extend(checks)
    return lines


def generate(messages: tuple[Message, ...], imports: tuple[str, ...] = ()) -> str:
    lines = [
        "// Generated from ROS2 .msg definitions. Do not edit.",
        "#pragma once",
        "#include <array>",
        "#include <cstdint>",
        "#include <limits>",
        "#include <stdexcept>",
        "#include <string>",
        "#include <vector>",
        "#include <fastcdr/Cdr.h>",
        "#include <fastcdr/CdrSizeCalculator.hpp>",
        '#include "dimos_cdr.hpp"',
    ]
    lines.extend(f"#include <{module}/messages.hpp>" for module in imports)
    upstream = Path(__file__).with_name("_vendor") / "rosidl"
    for name in [
        "rosidl_runtime_c/message_initialization.h",
        "rosidl_runtime_cpp/message_initialization.hpp",
        "rosidl_runtime_cpp/bounded_vector.hpp",
    ]:
        lines.append(
            "\n".join(
                line
                for line in (upstream / "include" / name).read_text().splitlines()
                if not line.startswith("#include <rosidl_runtime")
            )
        )
    serialization = []
    guards = {}
    with TemporaryDirectory(prefix="dimos-rosidl-") as temporary:
        output = Path(temporary)
        templates = output / "templates"
        templates.mkdir()
        (templates / "idl_cdr.hpp.em").write_text(
            Path(__file__).with_name("templates").joinpath("idl_cdr.hpp.em").read_text()
        )
        (templates / "msg__cdr.hpp.em").write_text(
            (upstream / "serialization/msg__cdr.hpp.em").read_text()
        )
        for message in messages:
            with redirect_stdout(StringIO()):
                idl = convert_to_idl(
                    message.source.parent.parent.resolve(),
                    message.package,
                    Path("msg") / message.source.name,
                    output / "idl" / message.package,
                )
                arguments = output / "arguments.json"
                arguments.write_text(
                    json.dumps(
                        {
                            "package_name": message.package,
                            "idl_tuples": [str(idl.parent.parent) + ":msg/" + idl.name],
                            "output_dir": str(output / "cpp" / message.package),
                            "template_dir": str(upstream / "rosidl_generator_cpp" / "resource"),
                            "target_dependencies": [str(message.source.resolve())],
                        }
                    )
                )
                generate_files(str(arguments), {"idl__struct.hpp.em": "detail/%s__struct.hpp"})
                config = json.loads(arguments.read_text())
                config["template_dir"] = str(templates)
                arguments.write_text(json.dumps(config))
                generated = generate_files(str(arguments), {"idl_cdr.hpp.em": "detail/%s__cdr.hpp"})
                serialized = Path(generated[0]).read_text()
            headers = (output / "cpp" / message.package / "msg" / "detail").glob("*__struct.hpp")
            declaration = next(
                path.read_text()
                for path in headers
                if f"struct {message.short_name}_\n" in path.read_text()
            )
            declaration = "\n".join(
                line for line in declaration.splitlines() if not line.startswith('#include "')
            )
            # Identical dependency declarations share an include guard; conflicting
            # definitions must not be hidden by an upstream name-only guard.
            original_guard = next(
                line.split()[1] for line in declaration.splitlines() if line.startswith("#ifndef ")
            )
            identity = [
                message.name,
                [asdict(field) for field in message.fields],
                [asdict(constant) for constant in message.constants],
            ]
            guard = (
                "DIMOS_CDR_"
                + sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest().upper()
            )
            guards[message.name] = guard
            serialization.append(
                f"#ifndef {guard}_SERIALIZATION\n#define {guard}_SERIALIZATION\n{serialized}\n#endif"
            )
            declaration = declaration.replace(original_guard, guard)
            marker = f"  using Type = {message.short_name}_<ContainerAllocator>;"
            adapter = "\nvoid validate() const {\n" + "\n".join(validation(message)) + "\n}\n"
            adapter += f'static constexpr const char* msg_name = "{message.name}";'
            lines.append(declaration.replace(marker, marker + adapter))

    lines.extend(serialization)
    lines.append("namespace eprosima::fastcdr {")
    for message in messages:
        name = message.name.replace("/", "::")
        guard = guards[message.name] + "_CODEC"
        lines.extend([f"#ifndef {guard}", f"#define {guard}"])
        namespace = "::".join(message.name.split("/")[:-1]) + "::typesupport_fastrtps_cpp"
        lines.extend(
            [
                f"template<> inline size_t calculate_serialized_size(CdrSizeCalculator&, const {name}& value, size_t& alignment) {{ auto size = {namespace}::get_serialized_size(value, alignment); alignment += size; return size; }}",
                f'template<> inline void serialize(Cdr& cdr, const {name}& value) {{ if (!{namespace}::cdr_serialize(value, cdr)) throw std::invalid_argument("Invalid CDR value"); }}',
                f'template<> inline void deserialize(Cdr& cdr, {name}& value) {{ if (!{namespace}::cdr_deserialize(cdr, value)) throw std::invalid_argument("Invalid CDR data"); value.validate(); }}',
            ]
        )
        lines.append("#endif")
    lines.append("}")
    return "\n".join(lines) + "\n"
