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

"""annotations.json: every robot dimos supports and what each of its blueprints is for (title, description, the settings to
decide before running it (`recommended_config`), starter picks, hidden ones).

It is generated: annotations.yaml is the hand-written part (with comments) (which robots and blueprints there are, and which settings
each should decide first, by config key, with only what reflection can't know: a nicer label, a placeholder, a docs
link, a pick of several values at once like Run it on: Robot / Replay / Simulator), and `python -m experimental.gateway.utils.robots
--write` fills in each GlobalConfig setting's type, default and choices (a Literal's values) by importing GlobalConfig.
A blueprint's own entry for a key the robot's defaults also set inherits that entry's fields.

It sits beside all_blueprints.py because it describes the same registry, and the generation test that keeps
all_blueprints.py current checks it too (`problems`), so the two can't drift:

  (a) every directory under dimos/robot is a robot's `dirs` entry (or inside one), in `excluded` with a reason, or a
      vendor folder that only holds those (dimos/robot/unitree);
  (b) every blueprint defined under a robot's dirs is listed under that robot;
  (c) every listed blueprint is registered in all_blueprints.py;
  (d) every setting names a real GlobalConfig field, or (`module_arg_problems`, which imports the blueprints) a real
      config field of a module in that blueprint;
  (e) it matches annotations.schema.json, and its types, starter ranks, recommended blueprints and settings are consistent;
  (f) each robot's `type` (a key of `types`, or null: not a robot) and `manufacturer` agree with
      where its code lives (`kind_problems`);
  (g) every setting's `docs` link resolves, its #anchor too (`link_problems`, which goes online);
  (h) annotations.json is what annotations.yaml generates (`generate`).

dimOS Desktop reads it per tag through dimos.yaml's `robots:` (no gateway needed), or resolved from the dimos gateway's
`GET /dimos/robots` (`resolved`).
"""

from __future__ import annotations

import copy
import difflib
import json
from pathlib import Path
import re
import time
from typing import Any
import urllib.error
import urllib.parse
import urllib.request

from dimos.constants import DIMOS_PROJECT_ROOT

ROBOTS_FILE = Path(__file__).parents[1] / "annotations.json"
SOURCE_FILE = Path(__file__).parents[1] / "annotations.yaml"
SCHEMA_FILE = Path(__file__).parents[1] / "annotations.schema.json"
ROBOT_ROOT = "dimos/robot"
SKIPPED_DIRS = {"__pycache__"}


def load(path: Path = ROBOTS_FILE) -> dict[str, Any]:
    result: dict[str, Any] = json.loads(path.read_text())
    return result


def blueprint_file(ref: str) -> str:
    """`dimos.robot.drone.blueprints.basic.drone_basic:drone_basic` -> `dimos/robot/drone/blueprints/basic/drone_basic.py`"""
    return ref.split(":")[0].replace(".", "/") + ".py"


def _inside(path: str, directory: str) -> bool:
    return path == directory or path.startswith(directory.rstrip("/") + "/")


def owner_of(doc: dict[str, Any], path: str) -> str | None:
    """The robot whose dirs hold `path` (the most specific dir wins)."""
    best: tuple[int, str] | None = None
    for robot_id, robot in doc["robots"].items():
        for directory in robot["dirs"]:
            if _inside(path, directory) and (best is None or len(directory) > best[0]):
                best = (len(directory), robot_id)
    return best[1] if best else None


def schema_problems(doc: dict[str, Any]) -> list[str]:
    """(e) Where it breaks annotations.schema.json."""
    import jsonschema

    validator = jsonschema.Draft202012Validator(json.loads(SCHEMA_FILE.read_text()))
    return [
        f"annotations.json{''.join(f'[{p!r}]' for p in error.absolute_path)}: {error.message}"
        for error in sorted(
            validator.iter_errors(doc), key=lambda e: list(map(str, e.absolute_path))
        )
    ]


def _robot_dirs(root: Path) -> list[str]:
    """Every directory under dimos/robot, repo-relative, parents before children."""
    found = []
    for path in sorted((root / ROBOT_ROOT).rglob("*")):
        relative = path.relative_to(root)
        if path.is_dir() and not any(
            part in SKIPPED_DIRS or part.startswith(".") for part in relative.parts
        ):
            found.append(relative.as_posix())
    return found


def directory_problems(doc: dict[str, Any], root: Path = DIMOS_PROJECT_ROOT) -> list[str]:
    """(a) Robot directories annotations.json doesn't account for, and entries naming directories that don't exist."""
    problems = []
    claimed = {d: robot_id for robot_id, robot in doc["robots"].items() for d in robot["dirs"]}
    covered = [*claimed, *doc["excluded"]]
    for directory in covered:
        if not (root / directory).is_dir():
            where = f"robots.{claimed[directory]}.dirs" if directory in claimed else "excluded"
            problems.append(f"annotations.json {where} names {directory}, which is not a directory")
    for directory in _robot_dirs(root):
        if any(_inside(directory, c) for c in covered):
            continue
        if any(_inside(c, directory) for c in covered):
            continue  # a vendor folder (dimos/robot/unitree) holding robots
        problems.append(
            f"{directory} is not in annotations.json: if it is a robot, add it to a robot's `dirs` (or add a robot: "
            f'"robots": {{"<id>": {{"name": ..., "description": ..., "dirs": ["{directory}"], "blueprints": {{}}}}}}); '
            f'if it isn\'t, add it to `excluded`: "{directory}": "<why it is not a robot>"'
        )
    return problems


def blueprint_problems(doc: dict[str, Any], registry: dict[str, str]) -> list[str]:
    """(b) and (c): blueprints the code has that annotations.json doesn't list where it should, and the reverse."""
    problems = []
    listed: dict[str, list[str]] = {}
    for robot_id, robot in doc["robots"].items():
        for name in robot["blueprints"]:
            listed.setdefault(name, []).append(robot_id)
    for name, robots in sorted(listed.items()):
        if len(robots) > 1:
            problems.append(
                f"{name} is listed under several robots ({', '.join(robots)}): keep one"
            )
        if name not in registry:
            guess = difflib.get_close_matches(name, list(registry), n=1)
            hint = f" (did you mean {guess[0]}?)" if guess else ""
            problems.append(
                f"annotations.json lists {name} under robots.{robots[0]}, but no such blueprint is registered in "
                f"dimos/robot/all_blueprints.py{hint}: fix the name or remove it"
            )
    excluded = list(doc["excluded"])
    for name, ref in sorted(registry.items()):
        path = blueprint_file(ref)
        owner = owner_of(doc, path)
        entry = '{"title": ..., "description": ...}'
        if owner is not None:
            if owner not in listed.get(name, []):
                problems.append(
                    f"blueprint {name} ({path}) is in robots.{owner}'s dirs but annotations.json doesn't list it: add "
                    f'"{name}": {entry} to robots.{owner}.blueprints (with "hidden": true if it is a test or '
                    "building block)"
                )
        elif any(_inside(path, d) for d in excluded):
            problems.append(
                f"blueprint {name} is defined in {path}, inside a directory annotations.json excludes: list it under a "
                "robot (and give that robot the directory), or move it"
            )
        elif _inside(path, ROBOT_ROOT) and name not in listed:
            problems.append(
                f"blueprint {name} ({path}) is under {ROBOT_ROOT} but no robot's dirs hold it: add that directory "
                f'to a robot\'s dirs and "{name}": {entry} to its blueprints'
            )
    return problems


def _recommended_config_of(
    robot: dict[str, Any], blueprint: dict[str, Any]
) -> list[dict[str, Any]]:
    """A blueprint's recommended settings: its own, else its robot's defaults', else none."""
    if "recommended_config" in blueprint:
        return list(blueprint["recommended_config"])
    return list(robot.get("defaults", {}).get("recommended_config", []))


def _setting_spec(doc: dict[str, Any], setting: dict[str, Any]) -> dict[str, Any]:
    """A recommended setting as written (annotations.json writes every one out)."""
    return dict(setting)


def _setting_key(spec: dict[str, Any]) -> str | None:
    """A setting's config key (a GlobalConfig field, or <module>.<field>); None for a pick (it sets several)."""
    if "global" in spec:
        return str(spec["global"])
    if "module" in spec:
        return f"{spec['module']}.{spec['field']}"
    return None


def _settings_keys(doc: dict[str, Any], settings: list[dict[str, Any]]) -> set[str]:
    """Every config key a list of recommended settings sets: each setting's own, and what each pick's choices set."""
    keys: set[str] = set()
    for setting in settings:
        spec = _setting_spec(doc, setting)
        key = _setting_key(spec)
        if key is not None:
            keys.add(key)
        for choice in spec.get("choices", []):
            keys.update(choice.get("set", {}))
    return keys


def consistency_problems(doc: dict[str, Any]) -> list[str]:
    """(d) GlobalConfig names, and (e) types, starter ranks, recommended blueprints and settings."""
    from dimos.core.global_config import GlobalConfig

    fields = GlobalConfig.model_fields
    problems = []
    ranks: dict[int, str] = {}
    for robot_id, robot in doc["robots"].items():
        if robot["type"] is not None and robot["type"] not in doc["types"]:
            problems.append(
                f"robots.{robot_id}.type is {robot['type']!r}, not one of types: {', '.join(doc['types'])} "
                "(or null: not a robot dimos drives)"
            )
        for name in robot.get("recommended", []):
            if name not in robot["blueprints"]:
                problems.append(
                    f"robots.{robot_id}.recommended: {name!r} is not one of its blueprints"
                    f"{_guess(name, robot['blueprints'])}"
                )
        for name, blueprint in robot["blueprints"].items():
            rank = blueprint.get("starter")
            if rank is not None:
                if rank in ranks:
                    problems.append(
                        f"{name} and {ranks[rank]} both have starter rank {rank}: give each its own"
                    )
                ranks[rank] = name
    for robot_id, robot in doc["robots"].items():
        containers = [(f"robots.{robot_id}.defaults", robot.get("defaults", {}))]
        containers += [
            (f"robots.{robot_id}.blueprints.{n}", b) for n, b in robot["blueprints"].items()
        ]
        for where, container in containers:
            settings = container.get("recommended_config", [])
            known = _settings_keys(doc, settings)
            for setting in settings:
                spec = _setting_spec(doc, setting)
                label = spec.get("label")
                if spec.get("streams") and spec.get("kind") != "recording":
                    problems.append(
                        f"{where}.recommended_config: {label!r}: `streams` only goes on a recording"
                    )
                if "global" in spec and spec["global"] not in fields:
                    problems.append(
                        f"{where}.recommended_config: {spec['global']!r} is not a GlobalConfig field"
                        f"{_guess(spec['global'], fields)}"
                    )
                choices = spec.get("choices", [])
                key = _setting_key(spec)
                if key is None:
                    # a pick: every choice sets values, the same keys each, real GlobalConfig fields or module ones
                    if any("set" not in choice for choice in choices):
                        problems.append(
                            f"{where}.recommended_config: {label!r} names no config value, so each choice needs a "
                            "`set` of the values it applies"
                        )
                    sets = [choice.get("set", {}) for choice in choices]
                    if len({tuple(sorted(each)) for each in sets}) > 1:
                        problems.append(
                            f"{where}.recommended_config: {label!r}'s choices set different keys: give each choice a "
                            "value for every key any of them sets, so picking one is unambiguous"
                        )
                    for each in sets:
                        for set_key in each:
                            if "." not in set_key and set_key not in fields:
                                problems.append(
                                    f"{where}.recommended_config: {label!r} sets {set_key!r}, which is not a "
                                    f"GlobalConfig field{_guess(set_key, fields)}"
                                )
                    labels = [choice["label"] for choice in choices]
                    if len(labels) != len(set(labels)):
                        problems.append(f"{where}.recommended_config: {label!r} has a choice twice")
                    if "default" in spec and spec["default"] not in labels:
                        problems.append(
                            f"{where}.recommended_config: {label!r} defaults to {spec['default']!r}, which isn't "
                            "the label of one of its choices"
                        )
                else:
                    if any("set" in choice for choice in choices):
                        problems.append(
                            f"{where}.recommended_config: {label!r} is one config value ({key}), so its choices "
                            "each have a `value`, not a `set`"
                        )
                    values = [choice["value"] for choice in choices if "value" in choice]
                    if len(values) != len({json.dumps(v) for v in values}):
                        problems.append(f"{where}.recommended_config: {label!r} has a choice twice")
                    if values and "default" in spec and spec["default"] not in values:
                        problems.append(
                            f"{where}.recommended_config: {label!r} defaults to {spec['default']!r}, "
                            "which isn't one of its choices"
                        )
                for when_key in spec.get("when", {}):
                    if when_key not in known:
                        problems.append(
                            f"{where}.recommended_config: {label!r} is shown when {when_key!r} is a value, but no "
                            "entry in this list sets it (a pick's choices or a setting of its own)"
                        )
    return problems


# folders under a robot root that group robots by kind or origin, not by who makes them
KIND_DIRS = {"manipulators", "diy"}
ROBOT_ROOTS = (ROBOT_ROOT, "dimos/experimental/robot")


def vendor_dir(directory: str) -> str | None:
    """`dimos/robot/unitree/go2` -> `dimos/robot/unitree` (the folder of the company that makes it); None for a robot
    at a robot root, in a kind folder (manipulators, diy), or outside the robot roots."""
    for root in ROBOT_ROOTS:
        if _inside(directory, root) and directory != root:
            parts = directory[len(root) + 1 :].split("/")
            if len(parts) >= 2 and parts[0] not in KIND_DIRS:
                return f"{root}/{parts[0]}"
    return None


def kind_problems(doc: dict[str, Any]) -> list[str]:
    """(f) Each robot's `type` and `manufacturer` agree with the code's layout: a manufacturer is how the robot's name
    starts; robots in one vendor folder share a manufacturer (and have one); DIY robots have none; and something with no
    code under a robot root (dimos/robot, dimos/experimental/robot) isn't a robot
    (type null)."""
    problems = []
    vendors: dict[str, list[tuple[str, Any]]] = {}
    for robot_id, robot in doc["robots"].items():
        where = f"robots.{robot_id}"
        maker, kind = robot["manufacturer"], robot["type"]
        if maker is not None and not robot["name"].startswith(maker + " "):
            problems.append(
                f"{where}.manufacturer is {maker!r}, but its name {robot['name']!r} doesn't start with it: name it "
                f'"{maker} <model>", or fix the manufacturer'
            )
        for directory in robot["dirs"]:
            vendor = vendor_dir(directory)
            if vendor is not None:
                vendors.setdefault(vendor, []).append((robot_id, maker))
            if any(_inside(directory, f"{root}/diy") for root in ROBOT_ROOTS) and maker is not None:
                problems.append(
                    f"{where}.manufacturer is {maker!r}, but it lives in {directory} (DIY): make it null"
                )
        if kind is not None and not any(
            _inside(d, root) for d in robot["dirs"] for root in ROBOT_ROOTS
        ):
            problems.append(
                f"{where}.type is {kind!r}, but none of its dirs is under {' or '.join(ROBOT_ROOTS)}: a robot's code "
                "lives there (make the type null if it isn't a robot)"
            )
    for vendor, members in sorted(vendors.items()):
        makers = {maker for _, maker in members}
        if None in makers or len(makers) > 1:
            listing = ", ".join(f"{robot_id}: {maker!r}" for robot_id, maker in members)
            problems.append(
                f"the robots in {vendor} ({listing}) are made by one company: give them the same manufacturer"
            )
    return problems


def _guess(name: str, options: Any) -> str:
    guess = difflib.get_close_matches(name, list(options), n=1)
    return f" (did you mean {guess[0]}?)" if guess else ""


def module_args(doc: dict[str, Any]) -> dict[str, set[tuple[str, str]]]:
    """Blueprint name -> the (module, field) pairs its recommended settings name (their own, and what picks set)."""
    found: dict[str, set[tuple[str, str]]] = {}
    for robot in doc["robots"].values():
        for name, blueprint in robot["blueprints"].items():
            for key in _settings_keys(doc, _recommended_config_of(robot, blueprint)):
                if "." in key:
                    module, field = key.rsplit(".", 1)
                    found.setdefault(name, set()).add((module, field))
    return found


def module_arg_problems(doc: dict[str, Any]) -> list[str]:
    """(d) Module args that aren't a real config field of a module in that blueprint (imports each such blueprint and
    asks dimos's own option parser, so `--<module>.<field>` is exactly what `dimos run` accepts)."""
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.coordination.blueprint_config.schema import normalize_option_name
    from dimos.robot.get_all_blueprints import get_blueprint_by_name

    problems = []
    for name, pairs in sorted(module_args(doc).items()):
        try:
            blueprint = get_blueprint_by_name(name)
        except Exception as error:  # a missing optional dependency, or a broken blueprint
            problems.append(f"{name}: can't import it to check its module args: {error}")
            continue
        schema = BlueprintConfigParser(blueprint)._get_schema()
        modules = sorted({t.root for t in schema.targets if t.section == "module"})
        for module, field in sorted(pairs):
            option = f"{module}.{field}"
            if any(
                t.section == "module" for t in schema.aliases.get(normalize_option_name(option), ())
            ):
                continue
            if module not in modules:
                problems.append(
                    f"{name}: module arg {option}: the blueprint has no module {module!r} "
                    f"(its modules: {', '.join(modules)})"
                )
            else:
                fields = {".".join(t.path) for t in schema.targets if t.root == module}
                problems.append(
                    f"{name}: module arg {option}: {module} has no config field {field!r}{_guess(field, fields)}"
                )
    return problems


# some docs hosts refuse a plain urllib client
BROWSER_USER_AGENT = (
    "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) "
    "Chrome/130.0.0.0 Safari/537.36"
)


def doc_links(doc: dict[str, Any]) -> dict[str, list[str]]:
    """Each `docs` link in annotations.json -> where it is (`robots.<id>...recommended_config`)."""
    found: dict[str, list[str]] = {}
    for robot_id, robot in doc["robots"].items():
        lists = [
            (f"robots.{robot_id}.defaults", robot.get("defaults", {}).get("recommended_config", []))
        ]
        lists += [
            (f"robots.{robot_id}.blueprints.{name}", blueprint.get("recommended_config", []))
            for name, blueprint in robot["blueprints"].items()
        ]
        for where, settings in lists:
            for setting in settings:
                if "docs" in setting and f"{where}.recommended_config" not in found.get(
                    setting["docs"], []
                ):
                    found.setdefault(setting["docs"], []).append(f"{where}.recommended_config")
    return found


def _fetch(url: str, timeout: float) -> tuple[int | None, str, str]:
    """(HTTP status after redirects, or None when it didn't answer; the body of a GET, or ""; the error)."""
    needs_body = bool(urllib.parse.urlsplit(url).fragment)
    for method in ("GET",) if needs_body else ("HEAD", "GET"):
        request = urllib.request.Request(
            url, method=method, headers={"User-Agent": BROWSER_USER_AGENT}
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                body = response.read().decode("utf-8", "replace") if method == "GET" else ""
                return response.status, body, ""
        except urllib.error.HTTPError as error:
            if method == "HEAD":
                continue  # a host that refuses HEAD may still answer GET
            return error.code, "", f"HTTP {error.code}"
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            return None, "", str(getattr(error, "reason", error))
    return None, "", "no answer"


def _has_anchor(html: str, anchor: str) -> bool:
    quoted = re.escape(anchor)
    return (
        re.search(rf"""\b(?:id|name)=(?:"{quoted}"|'{quoted}'|{quoted}(?=[\s>]))""", html)
        is not None
    )


def link_problems(
    doc: dict[str, Any], timeout: float = 15.0, attempts: int = 3, backoff: float = 2.0
) -> list[str]:
    """(g) `docs` links that don't answer 2xx/3xx (following redirects), or whose #anchor isn't on the page. A host
    that doesn't answer, answers 5xx or 429 is asked again (`attempts` in all); a 4xx is final."""
    problems = []
    for url, wheres in sorted(doc_links(doc).items()):
        status, body, error = None, "", ""
        for attempt in range(attempts):
            status, body, error = _fetch(url, timeout)
            if status is not None and (status < 500 and status != 429):
                break
            if attempt + 1 < attempts:
                time.sleep(backoff * 2**attempt)
        where = ", ".join(f"annotations.json {w}" for w in wheres)
        anchor = urllib.parse.unquote(urllib.parse.urlsplit(url).fragment)
        if status is None or status >= 400:
            problems.append(f"{where}: {url} doesn't resolve ({error}): fix the link or remove it")
        elif anchor and not _has_anchor(body, anchor):
            problems.append(
                f"{where}: {url} loads, but has no #{anchor} on it (a renamed heading?): fix the anchor"
            )
    return problems


def problems(
    doc: dict[str, Any], registry: dict[str, str], root: Path = DIMOS_PROJECT_ROOT
) -> list[str]:
    """Everything wrong with annotations.json, except module args (`module_arg_problems` imports blueprints) and docs links
    (`link_problems` goes online)."""
    found = schema_problems(doc)
    if found:
        return found  # the other checks assume its shape
    return [
        *directory_problems(doc, root),
        *blueprint_problems(doc, registry),
        *consistency_problems(doc),
        *kind_problems(doc),
    ]


def _resolved_arg(arg_id: str, arg: dict[str, Any]) -> dict[str, Any]:
    """An arg with its `id`, `key` (the GlobalConfig field, `--key value` before `run`, or `<module>.<field>`, after
    the blueprint name), `scope` and `docs` (None without one)."""
    if "global" in arg:
        key, scope = arg["global"], "global"
    else:
        key, scope = f"{arg['module']}.{arg['field']}", "module"
    return {
        "id": arg_id,
        "key": key,
        "scope": scope,
        "kind": "text",
        "required": False,
        "docs": None,
        **copy.deepcopy(arg),
    }


def _resolved_setting(doc: dict[str, Any], index: int, setting: dict[str, Any]) -> dict[str, Any]:
    """A recommended setting as clients read it: a config value's setting resolved like an arg (`id`, `key`, `scope`,
    `kind`, `required`, `docs`), or a pick (`kind` "pick", `key` and `scope` None, `default` the label of the choice
    to start on); both with `choices` (None for a free value) and `when` (None: always shown)."""
    spec = _setting_spec(doc, setting)
    key = _setting_key(spec)
    if key is None:
        choices = copy.deepcopy(spec["choices"])
        return {
            "id": f"pick-{index}",
            "key": None,
            "scope": None,
            "kind": "pick",
            "label": spec["label"],
            "required": False,
            "docs": spec.get("docs"),
            "default": spec.get("default", choices[0]["label"]),
            "choices": choices,
            "when": copy.deepcopy(spec.get("when")),
        }
    return {
        **_resolved_arg(key, spec),
        "choices": copy.deepcopy(spec.get("choices")),
        "when": copy.deepcopy(spec.get("when")),
    }


def resolved(doc: dict[str, Any], registry: dict[str, str] | None = None) -> dict[str, Any]:
    """annotations.json with every blueprint's defaults applied: its robot, `starter`, `hidden` and `recommended_config`
    (each setting resolved, `_resolved_setting`). With the registry: each blueprint's `registered`, and `unlisted`, the
    registered blueprints no robot lists."""
    out = copy.deepcopy(doc)
    out.pop("$schema", None)
    listed = set()
    for robot_id, robot in out["robots"].items():
        for name, blueprint in robot["blueprints"].items():
            listed.add(name)
            source = doc["robots"][robot_id]
            blueprint["robot"] = robot_id
            blueprint["recommended_config"] = [
                _resolved_setting(doc, index, setting)
                for index, setting in enumerate(
                    _recommended_config_of(source, source["blueprints"][name])
                )
            ]
            blueprint.setdefault("starter", None)
            blueprint.setdefault("hidden", False)
            if registry is not None:
                blueprint["registered"] = name in registry
        robot.pop("defaults", None)
        robot.setdefault("recommended", [])
    if registry is not None:
        out["unlisted"] = sorted(set(registry) - listed)
    return out


# ───────────────────────── generating annotations.json from annotations.yaml ─────────────────────────


def load_source(path: Path = SOURCE_FILE) -> dict[str, Any]:
    import yaml

    result: dict[str, Any] = yaml.safe_load(path.read_text())
    return result


def _jsonable(value: Any) -> Any:
    """A default as JSON (a Path as text, an enum as its value); None for what JSON can't say."""
    import enum

    if isinstance(value, enum.Enum):
        value = value.value
    if isinstance(value, Path):
        return str(value)
    try:
        json.dumps(value)
    except TypeError:
        return None
    return value


def reflected(field_name: str) -> dict[str, Any]:
    """What GlobalConfig says about one field: its `type` (string, boolean, integer, number or json), whether it is
    `nullable`, its `default`, a `kind` for its input (bool, number or text) and, for a Literal or an Enum, its
    `choices` (each value labelled as itself)."""
    import enum
    import types
    import typing

    from dimos.core.global_config import GlobalConfig

    field = GlobalConfig.model_fields[field_name]
    annotation = field.annotation
    parts = (
        list(typing.get_args(annotation))
        if typing.get_origin(annotation) in (typing.Union, types.UnionType)
        else [annotation]
    )
    nullable = type(None) in parts
    main = [part for part in parts if part is not type(None)]
    one = main[0] if len(main) == 1 else None
    choices: list[Any] | None = None
    if one is not None and typing.get_origin(one) is typing.Literal:
        choices = list(typing.get_args(one))
        one = type(choices[0]) if choices else str
    elif isinstance(one, type) and issubclass(one, enum.Enum):
        choices = [member.value for member in one]
        one = type(choices[0]) if choices else str
    kinds = {bool: "boolean", int: "integer", float: "number", str: "string", Path: "string"}
    type_name = kinds.get(one, "json") if isinstance(one, type) else "json"
    out: dict[str, Any] = {
        "type": type_name,
        "nullable": nullable,
        "default": _jsonable(field.default) if not field.is_required() else None,
        "kind": {"boolean": "bool", "integer": "number", "number": "number"}.get(type_name, "text"),
    }
    if choices is not None:
        out["choices"] = [{"value": value, "label": str(value)} for value in choices]
    if field.description:
        out["description"] = field.description
    return out


def _humanized(key: str) -> str:
    """robot_ip -> Robot ip; spothighlevel.ip -> Ip (a label for a setting the source doesn't name)."""
    words = key.rsplit(".", 1)[-1].replace("_", " ")
    return words[:1].upper() + words[1:]


def _generated_setting(
    entry: dict[str, Any],
    inherited: dict[str, Any] | None,
    keys: set[str],
    catalog: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """One source entry written out: GlobalConfig's reflection, then the robot defaults' entry for the same key (on a
    blueprint's own list: its `when` only if this list sets every key it names, `keys`), then the entry itself; a pick
    (no key) as written."""
    key = _setting_key(entry)
    if key is None:
        return copy.deepcopy(entry)
    inherited = dict(inherited or {})
    if "when" in inherited and not set(inherited["when"]) <= keys:
        del inherited["when"]
    base: dict[str, Any] = {}
    if "global" in entry:
        if catalog is not None:
            base = copy.deepcopy(catalog.get(entry["global"], {}))
        else:
            from dimos.core.global_config import GlobalConfig

            if entry["global"] in GlobalConfig.model_fields:
                base = reflected(entry["global"])
    merged = {**base, **copy.deepcopy(inherited), **copy.deepcopy(entry)}
    merged.setdefault("label", _humanized(key))
    ordered = {k: merged.pop(k) for k in ("global", "module", "field", "label") if k in merged}
    return {**ordered, **merged}


def generate(source: dict[str, Any]) -> dict[str, Any]:
    """annotations.json from annotations.yaml: every recommended setting written out (`_generated_setting`)."""
    about = (
        "Generated from annotations.yaml by `python -m experimental.gateway.utils.robots --write` (don't edit it; edit annotations.yaml): "
        "every robot dimos supports and its blueprints, and the settings to decide before running each, with each "
        "GlobalConfig setting's type, default and choices read from GlobalConfig. Served by the dimos gateway at "
        "GET /dimos/robots; read per tag through dimos.yaml's `robots:`."
    )
    out = {"$schema": "./annotations.schema.json", "about": about, **copy.deepcopy(source)}
    # annotations.yaml's generated catalog of GlobalConfig fields (the reflection, kept in the file); annotations.json leaves it out
    catalog = out.pop("global_config", None)
    for robot in out["robots"].values():
        defaults = robot.get("defaults", {}).get("recommended_config", [])
        by_key = {_setting_key(entry): entry for entry in defaults if _setting_key(entry)}
        if defaults:
            keys = _settings_keys(out, defaults)
            robot["defaults"]["recommended_config"] = [
                _generated_setting(e, None, keys, catalog) for e in defaults
            ]
        for blueprint in robot["blueprints"].values():
            if "recommended_config" in blueprint:
                own = blueprint["recommended_config"]
                keys = _settings_keys(out, own)
                blueprint["recommended_config"] = [
                    _generated_setting(e, by_key.get(_setting_key(e) or ""), keys, catalog)
                    for e in own
                ]
    return out


# ───────────────────────── keeping annotations.yaml current (like all_blueprints.py) ─────────────────────────
#
# annotations.yaml is hand-edited, with comments, so it is never re-dumped: `updated_source` edits its text in place, only in
# the parts it owns. The GlobalConfig catalog after GENERATED_MARK is rewritten whole; a blueprint the registry has in a
# robot's dirs but the file doesn't list is added as a stub block (TODO title and description) at the end of that
# robot's `blueprints:`; a listed blueprint the registry no longer has is removed with its block. Everything else (hand
# fields, comments, a robot directory nobody accounts for, a setting naming a field GlobalConfig no longer has) is left
# for a person, and `problems` says what to do.

GENERATED_MARK = "# ── generated below this line by `python -m experimental.gateway.utils.robots --write`, from GlobalConfig: don't edit it ──"
_ROBOT_INDENT, _FIELD_INDENT, _BLUEPRINT_INDENT = 2, 4, 6


def _indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def _is_code(line: str) -> bool:
    """A line that holds YAML (not blank, not only a comment)."""
    stripped = line.strip()
    return bool(stripped) and not stripped.startswith("#")


def _block_end(lines: list[str], start: int, indent: int) -> int:
    """The index after the block that starts at `start`: the next YAML line indented `indent` or less (trailing blank
    and comment lines stay outside it)."""
    end = start + 1
    last = start
    while end < len(lines):
        if _is_code(lines[end]):
            if _indent(lines[end]) <= indent:
                break
            last = end
        end += 1
    return last + 1


def _catalog_text() -> str:
    """The GlobalConfig catalog block: every field with its reflected type, default, choices and description."""
    import yaml

    from dimos.core.global_config import GlobalConfig

    catalog = {name: reflected(name) for name in GlobalConfig.model_fields}
    body = yaml.safe_dump(
        {"global_config": catalog}, sort_keys=False, allow_unicode=True, width=120
    )
    return f"{GENERATED_MARK}\n{body}"


def updated_source(text: str, registry: dict[str, str]) -> str:
    """annotations.yaml's text with its generated parts current (see above); the same text when nothing changed."""
    import yaml

    head = text.split(GENERATED_MARK, 1)[0].rstrip("\n") + "\n"
    source = yaml.safe_load(head) or {}
    lines = head.splitlines(keepends=True)
    robots_doc = source.get("robots", {})
    listed = {name for robot in robots_doc.values() for name in robot.get("blueprints", {})}
    # remove listed blueprints the registry doesn't have (bottom up, so indices stay good)
    removals: list[tuple[int, int]] = []
    robot_at: dict[str, int] = {}
    for index, line in enumerate(lines):
        if _is_code(line) and _indent(line) == _ROBOT_INDENT and line.strip().endswith(":"):
            robot_at[line.strip()[:-1]] = index
    for robot_id, robot in robots_doc.items():
        start = robot_at.get(robot_id)
        if start is None:
            continue
        end = _block_end(lines, start, _ROBOT_INDENT)
        for index in range(start, end):
            line = lines[index]
            name = line.strip()[:-1]
            if (
                _is_code(line)
                and _indent(line) == _BLUEPRINT_INDENT
                and line.strip().endswith(":")
                and name in robot.get("blueprints", {})
                and name not in registry
            ):
                removals.append((index, _block_end(lines, index, _BLUEPRINT_INDENT)))
    for start, end in sorted(removals, reverse=True):
        del lines[start:end]
    # add blueprints a robot's dirs hold that no robot lists
    missing: dict[str, list[str]] = {}
    for name, ref in sorted(registry.items()):
        owner = owner_of(source, blueprint_file(ref)) if "robots" in source else None
        if owner is not None and name not in listed:
            missing.setdefault(owner, []).append(name)
    robot_at = {}
    for index, line in enumerate(lines):
        if _is_code(line) and _indent(line) == _ROBOT_INDENT and line.strip().endswith(":"):
            robot_at[line.strip()[:-1]] = index
    for robot_id in sorted(missing, key=lambda r: robot_at.get(r, 0), reverse=True):
        start = robot_at.get(robot_id)
        if start is None:
            continue
        end = _block_end(lines, start, _ROBOT_INDENT)
        stubs = "".join(
            f"{' ' * _BLUEPRINT_INDENT}{name}:\n"
            f"{' ' * (_BLUEPRINT_INDENT + 2)}title: 'TODO: a short name for {name}'\n"
            f"{' ' * (_BLUEPRINT_INDENT + 2)}description: 'TODO: what it does and what it needs'\n"
            for name in missing[robot_id]
        )
        if "blueprints" in robots_doc.get(robot_id, {}) and robots_doc[robot_id]["blueprints"]:
            lines.insert(end, stubs)
        else:
            lines.insert(end, f"{' ' * _FIELD_INDENT}blueprints:\n{stubs}")
    return "".join(lines) + "\n" + _catalog_text()


def write(path: Path = ROBOTS_FILE, source: Path = SOURCE_FILE) -> None:
    """Bring annotations.yaml's generated parts current, then write annotations.json from it."""
    from dimos.robot.all_blueprints import all_blueprints

    text = source.read_text()
    updated = updated_source(text, all_blueprints)
    if updated != text:
        source.write_text(updated)
    path.write_text(json.dumps(generate(load_source(source)), indent=2, ensure_ascii=False) + "\n")


def stale_problems(doc: dict[str, Any] | None = None) -> list[str]:
    """(h) annotations.yaml's generated parts or annotations.json aren't current."""
    from dimos.robot.all_blueprints import all_blueprints

    text = SOURCE_FILE.read_text()
    found = []
    if updated_source(text, all_blueprints) != text:
        found.append(
            "experimental/gateway/annotations.yaml is out of date (a blueprint added or removed, or GlobalConfig changed): run "
            "`python -m experimental.gateway.utils.robots --write`, then fill in any TODO it added"
        )
    if (doc if doc is not None else load()) != generate(load_source()):
        found.append(
            "experimental/gateway/annotations.json is stale: it is generated from annotations.yaml; run "
            "`python -m experimental.gateway.utils.robots --write`"
        )
    return found


if __name__ == "__main__":
    import sys

    if "--write" in sys.argv[1:]:
        write()
        print(f"wrote {SOURCE_FILE.name} (its generated parts) and {ROBOTS_FILE.name}")
    else:
        found = stale_problems()
        print("\n".join(found) or "annotations.yaml and annotations.json are current")
        raise SystemExit(1 if found else 0)
