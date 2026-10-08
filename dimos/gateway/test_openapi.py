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

"""The gateway's OpenAPI: complete, the same shapes as Desktop's (fixtures/), and the checked-in openapi.json current."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any

from fastapi.testclient import TestClient
import pytest
import yaml

from dimos.gateway import openapi
from dimos.gateway.app import ServerState, create_app
from dimos.gateway.events import Bus
from dimos.gateway.uploads import Uploads

FIXTURE = Path(__file__).parent / "fixtures" / "desktop_openapi_dimos.json"
DIMOS_YAML = Path(__file__).parents[2] / "dimos.yaml"

# where this gateway knowingly differs from Desktop's doc, and why
KNOWN_DIFFERENCES = {
    # Desktop calls `?fresh=true` itself (its Blueprints tag says so) but leaves it out of the parameter list
    "get /dimos/blueprints: parameter only here: fresh",
    # Desktop's Rust takes `after` too (its Logs tag documents it) but leaves it out of the parameter list
    "get /dimos/runs/{}/log: parameter only here: after",
    # the discovery cache's import status, new here (Desktop's doc follows this one)
    "get /dimos/blueprints.blueprints[]: field only here: importable",
    "get /dimos/blueprints.blueprints[]: field only here: import_error",
    "get /dimos/blueprints.blueprints[]: field only here: missing_module",
    # a module's docs, methods, code location and its streams' wired topics, new here in 1.7 (Desktop reads them when
    # they're there)
    *(
        f"get /dimos/blueprints/{{}}.modules[]: field only here: {field}"
        for field in ("doc", "summary", "file", "line", "rpcs", "skills")
    ),
    "get /dimos/blueprints/{}.modules[].streams[]: field only here: topic",
    # where the blueprint itself is defined, new here in 1.8
    "get /dimos/blueprints/{}: field only here: file",
    "get /dimos/blueprints/{}: field only here: line",
    # a launch's own `dimos run` arguments (POST /dimos/runs `args`), new here in 1.17
    "get /dimos/runs.launch: field only here: args",
    "post /dimos/runs: field only here: args",
    "post /dimos/runs/restart: field only here: args",
    "post /dimos/runs: body fields ['args', 'blueprint', 'overrides', 'replay'] != ['blueprint', 'overrides', 'replay']",
}


@pytest.fixture(scope="module")
def spec() -> dict[str, Any]:
    return openapi.document(openapi.spec_app())


def operations(doc: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """`method path` (path params as {}) -> operation."""
    return {
        f"{method} {re.sub(r'{[^}]+}', '{}', path)}": operation
        for path, methods in doc["paths"].items()
        for method, operation in methods.items()
    }


def resolve(doc: dict[str, Any], schema: dict[str, Any]) -> dict[str, Any]:
    """`$ref` followed, and `X | null` taken as X."""
    while True:
        if "$ref" in schema:
            schema = doc["components"]["schemas"][schema["$ref"].rsplit("/", 1)[1]]
        elif "anyOf" in schema and any(s.get("type") == "null" for s in schema["anyOf"]):
            rest = [s for s in schema["anyOf"] if s.get("type") != "null"]
            if len(rest) != 1:
                return schema
            schema = rest[0]
        else:
            return schema


def kind(schema: dict[str, Any]) -> str | None:
    if "type" in schema:
        return str(schema["type"])
    if "const" in schema or "enum" in schema:
        return type((schema.get("enum") or [schema.get("const")])[0]).__name__.replace(
            "str", "string"
        )
    return None


# Desktop documents its answers in a small notation inside backticks, e.g.
# `{ name, modules: [{ name, class, streams: [...] }], error? }`, `Launch | null`, `{ byPath: { [path]: Uploaded } }`


def parse_shape(text: str) -> Any:
    """("object", {field: (optional, shape)}), ("array", shape), ("map", shape), ("ref", Name) or None (a leaf)."""
    tokens = re.findall(r'\[[a-z_]+\]|"[^"]*"|[A-Za-z_][A-Za-z0-9_]*|[{}\[\]:,?|]', text)
    position = 0

    def take(expected: str | None = None) -> str:
        nonlocal position
        token = tokens[position]
        assert expected is None or token == expected, (text, token, expected)
        position += 1
        return token

    def value() -> Any:
        options = [atom()]
        while position < len(tokens) and tokens[position] == "|":
            take("|")
            options.append(atom())
        shapes = [option for option in options if option is not None]
        return shapes[0] if len(shapes) == 1 else None

    def atom() -> Any:
        token = take()
        if token == "{":
            if tokens[position].startswith("[") and len(tokens[position]) > 1:
                take()
                take(":")
                shape = ("map", value())
                take("}")
                return shape
            fields: dict[str, tuple[bool, Any]] = {}
            while tokens[position] != "}":
                name = take()
                optional = tokens[position] == "?"
                if optional:
                    take("?")
                fields[name] = (optional, None)
                if tokens[position] == ":":
                    take(":")
                    fields[name] = (optional, value())
                if tokens[position] == ",":
                    take(",")
            take("}")
            return ("object", fields)
        if token == "[":
            shape = ("array", value())
            take("]")
            return shape
        if token[0].isupper():
            return ("ref", token)
        return None

    shape = value()
    return shape


def desktop_shapes(contract: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """Each operation's 200 shape (None: not a JSON shape), and the named ones (`Launch`: `{...}`)."""
    named: dict[str, Any] = {}
    answers: dict[str, Any] = {}
    for key, operation in operations(contract).items():
        description = operation["responses"]["200"]["description"]
        spans = re.findall(r"`([^`]*)`", description)
        definition = re.match(r"`([A-Z][A-Za-z]*)`: `([^`]*)`", description)
        if definition:
            named[definition.group(1)] = parse_shape(definition.group(2))
        first = spans[0] if spans and description.startswith("`") else ""
        # a shape (`{...}`, `[...]`) or a named one (`Launch`); `ok`, "JSON" and prose aren't
        answers[key] = (
            parse_shape(first) if first[:1] in ("{", "[") or first[:1].isupper() else None
        )
    return answers, named


def compare_shape(
    doc: dict[str, Any], schema: dict[str, Any], shape: Any, named: dict[str, Any], where: str
) -> list[str]:
    if shape is None:
        return []
    schema = resolve(doc, schema)
    if "anyOf" in schema:
        # one of several answers (e.g. a map, or one entry): the closest one
        tries = [
            compare_shape(doc, option, shape, named, where)
            for option in schema["anyOf"]
            if option.get("type") != "null"
        ]
        return min(tries, key=len)
    if shape[0] == "ref":
        return (
            compare_shape(doc, schema, named[shape[1]], named, where) if shape[1] in named else []
        )
    if shape[0] == "array":
        if schema.get("type") != "array":
            return [f"{where}: Desktop has a list, here {kind(schema)}"]
        return compare_shape(doc, schema["items"], shape[1], named, f"{where}[]")
    if shape[0] == "map":
        if schema.get("type") != "object" or "additionalProperties" not in schema:
            return [f"{where}: Desktop has a map, here {kind(schema)}"]
        return compare_shape(doc, schema["additionalProperties"], shape[1], named, f"{where}{{}}")
    if (
        "properties" not in schema
        and isinstance(schema.get("additionalProperties"), dict)
        and len(shape[1]) == 1
    ):
        # Desktop writes a map with a placeholder key: `{ module: { field: value } }`
        [(_, inner)] = shape[1].values()
        return compare_shape(doc, schema["additionalProperties"], inner, named, f"{where}{{}}")
    properties, required = schema.get("properties", {}), set(schema.get("required", []))
    problems = [
        f"{where}: field only in Desktop's doc: {n}" for n in shape[1] if n not in properties
    ]
    problems += [f"{where}: field only here: {n}" for n in properties if n not in shape[1]]
    for name, (optional, inner) in shape[1].items():
        if name in properties:
            if optional == (name in required):
                problems.append(f"{where}.{name}: required differs (Desktop: {not optional})")
            problems += compare_shape(doc, properties[name], inner, named, f"{where}.{name}")
    return problems


def mismatches(spec: dict[str, Any], contract: dict[str, Any]) -> list[str]:
    ours, theirs = operations(spec), operations(contract)
    answers, named = desktop_shapes(contract)
    problems = [f"{key}: not served" for key in theirs if key not in ours]
    for key, desktop in theirs.items():
        mine = ours.get(key)
        if mine is None:
            continue
        for extension in ("x-agent", "x-mcp-tool", "x-family"):
            if mine.get(extension) != desktop.get(extension):
                problems.append(
                    f"{key}: {extension} {mine.get(extension)!r} != Desktop's {desktop.get(extension)!r}"
                )
        if mine["operationId"] != desktop["operationId"]:
            problems.append(
                f"{key}: operationId {mine['operationId']} != Desktop's {desktop['operationId']}"
            )
        # parameters: name, place, required, type
        params = {(p["name"], p["in"]): p for p in mine.get("parameters", [])}
        others = {(p["name"], p["in"]): p for p in desktop.get("parameters", [])}
        problems += [
            f"{key}: parameter only in Desktop's doc: {n}" for n, _ in others.keys() - params.keys()
        ]
        problems += [f"{key}: parameter only here: {n}" for n, _ in params.keys() - others.keys()]
        for at in params.keys() & others.keys():
            a, b = params[at], others[at]
            if a.get("required", False) != b.get("required", False):
                problems.append(f"{key}: parameter {at[0]} required differs")
            if kind(resolve(spec, a["schema"])) != kind(b["schema"]):
                problems.append(
                    f"{key}: parameter {at[0]} type {kind(resolve(spec, a['schema']))} != {kind(b['schema'])}"
                )
        # request body: field names, types, required
        if ("requestBody" in mine) != ("requestBody" in desktop):
            problems.append(
                f"{key}: request body only {'here' if 'requestBody' in mine else 'in Desktop'}"
            )
        elif "requestBody" in desktop:
            a = resolve(spec, mine["requestBody"]["content"]["application/json"]["schema"])
            b = desktop["requestBody"]["content"]["application/json"]["schema"]
            if a.get("properties", {}).keys() != b["properties"].keys():
                problems.append(
                    f"{key}: body fields {sorted(a.get('properties', {}))} != {sorted(b['properties'])}"
                )
            for name in a.get("properties", {}).keys() & b["properties"].keys():
                if kind(resolve(spec, a["properties"][name])) != kind(b["properties"][name]):
                    problems.append(f"{key}: body field {name} type differs")
            if set(a.get("required", [])) != set(b.get("required", [])):
                problems.append(f"{key}: body required {a.get('required')} != {b.get('required')}")
        # answer: field names, nesting, required, against Desktop's notation
        content = mine["responses"]["200"].get("content", {})
        if "application/json" in content:
            problems += compare_shape(
                spec, content["application/json"]["schema"], answers[key], named, key
            )
    return sorted(problems)


def test_the_same_api_as_desktops_doc(spec: dict[str, Any]) -> None:
    found = set(mismatches(spec, json.loads(FIXTURE.read_text())))
    assert found - KNOWN_DIFFERENCES == set()
    assert KNOWN_DIFFERENCES - found == set(), (
        "a known difference is gone: drop it from KNOWN_DIFFERENCES"
    )


def test_the_comparison_catches_a_difference(spec: dict[str, Any]) -> None:
    contract = json.loads(FIXTURE.read_text())
    contract["paths"]["/dimos/info"]["get"]["responses"]["200"]["description"] = (
        "`{ dir, colour? }`"
    )
    del contract["paths"]["/dimos/uploads"]["post"]["requestBody"]["content"]["application/json"][
        "schema"
    ]["properties"]["kind"]
    found = mismatches(spec, contract)
    assert "get /dimos/info: field only in Desktop's doc: colour" in found
    assert "get /dimos/info: field only here: inRange" in found
    assert any(problem.startswith("post /dimos/uploads: body fields") for problem in found)


def test_every_operation_is_documented(spec: dict[str, Any]) -> None:
    tags = {tag["name"] for tag in spec["tags"]}
    assert tags == {
        "blueprints",
        "runs",
        "logs",
        "global-config",
        "cloud",
        "uploads",
        "events",
        "server",
        "discovery",
        "docs",
        "extras",
        "jobs",
        "skills",
    }
    found = operations(spec)
    assert len(found) >= 27
    for key, operation in found.items():
        assert operation["summary"] and operation["description"], key
        assert len(operation["tags"]) == 1 and operation["tags"][0] in tags, key
        assert operation["x-family"] == "dimos" and isinstance(operation["x-agent"], bool), key
        ok = operation["responses"]["200"]
        assert ok["description"] and ok["content"], key
        for media in ok["content"].values():
            schema = media["schema"]
            assert (schema and schema.get("type") != "object") or schema.get("properties"), key
        for code, response in operation["responses"].items():
            if code != "200":
                assert code in {"400", "404", "409", "500"}, (key, code)
                assert response["content"]["application/json"]["schema"] == {
                    "$ref": "#/components/schemas/ErrorResponse"
                }, key
        for parameter in operation.get("parameters", []):
            assert parameter.get("description"), (key, parameter["name"])


def test_every_model_field_is_described(spec: dict[str, Any]) -> None:
    for name, schema in spec["components"]["schemas"].items():
        for field, value in schema.get("properties", {}).items():
            assert field == "type" or value.get("description"), f"{name}.{field}"


def test_events_are_documented_with_their_zenoh_keys(spec: dict[str, Any]) -> None:
    schemas = spec["components"]["schemas"]
    union = schemas["DimosEvent"]
    types = set(union["discriminator"]["mapping"])
    assert types == {
        "launch",
        "log",
        "upload",
        "uploads",
        "upload-removed",
        "cloud-login",
        "discovery",
        "job",
        "blueprints",
    }
    for event_type, ref in union["discriminator"]["mapping"].items():
        schema = schemas[ref.rsplit("/", 1)[1]]
        assert schema["x-zenoh-key"] == f"<ns>/dimos/events/{event_type}"
        assert schema["properties"]["type"]["const"] == event_type
    stream = spec["paths"]["/dimos/events"]["get"]
    assert stream["deprecated"] and "<ns>/dimos/events/<type>" in stream["description"]
    assert stream["responses"]["200"]["content"]["text/event-stream"]["schema"] == {
        "$ref": "#/components/schemas/DimosEvent"
    }


def test_served_with_the_dimos_version(tmp_path: Path) -> None:
    bus = Bus()
    app = create_app(
        ServerState(tmp_path, bus, Uploads(tmp_path, bus, None, tmp_path / "log")), background=False
    )
    with TestClient(app) as client:
        served = client.get("/dimos/openapi.json").json()
        assert client.get("/dimos/docs").status_code == 404
    assert served["info"]["version"] == openapi.API_VERSION
    assert "x-dimos-version" in served["info"]
    del served["info"]["x-dimos-version"]
    assert served == json.loads(openapi.text(openapi.document(app, runtime=False)))


def test_checked_in_openapi_is_current(spec: dict[str, Any]) -> None:
    generated = openapi.text(openapi.document(openapi.spec_app(), runtime=False))
    assert openapi.SPEC_FILE.read_text() == generated, (
        "dimos/gateway/openapi.json is stale: run `python -m dimos.gateway --write-openapi`"
    )


def test_dimos_yaml_points_at_it() -> None:
    api = yaml.safe_load(DIMOS_YAML.read_text())["api"]
    assert api["version"] == openapi.API_VERSION
    assert (DIMOS_YAML.parent / api["openapi"]).resolve() == openapi.SPEC_FILE.resolve()
    assert json.loads(openapi.SPEC_FILE.read_text())["info"]["version"] == api["version"]
