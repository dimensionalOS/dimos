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

"""robots.json: every robot dimos supports and what each of its blueprints is for (title, description, tags, the
modes it runs in and the essential args of each, the app to install for it, starter picks, hidden ones).

It sits beside all_blueprints.py because it describes the same registry, and the generation test that keeps
all_blueprints.py current checks it too (`problems`), so the two can't drift:

  (a) every directory under dimos/robot is a robot's `dirs` entry (or inside one), in `excluded` with a reason, or a
      vendor folder that only holds those (dimos/robot/unitree);
  (b) every blueprint defined under a robot's dirs is listed under that robot;
  (c) every listed blueprint is registered in all_blueprints.py;
  (d) every arg names a real GlobalConfig field, or (`module_arg_problems`, which imports the blueprints) a real
      config field of a module in that blueprint;
  (e) it matches robots.schema.json, and its tags, groups, starter ranks and recommended blueprints are consistent;
  (f) each robot's `type` (dog, wheeled, humanoid, arm, drone, or null: not a robot) and `manufacturer` agree with
      where its code lives (`kind_problems`);
  (g) every arg's `docs` link resolves, its #anchor too (`link_problems`, which goes online).

dimOS Desktop reads it per tag through dimos.yaml's `robots:` (no server needed), or resolved from the dimos server's
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

ROBOTS_FILE = Path(__file__).parent / "robots.json"
SCHEMA_FILE = Path(__file__).parent / "robots.schema.json"
ROBOT_ROOT = "dimos/robot"
# tags that come from a blueprint's modes, never written on it
MODE_TAGS = ("replay", "sim")
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
    """(e) Where it breaks robots.schema.json."""
    import jsonschema  # type: ignore[import-untyped]

    validator = jsonschema.Draft202012Validator(json.loads(SCHEMA_FILE.read_text()))
    return [
        f"robots.json{''.join(f'[{p!r}]' for p in error.absolute_path)}: {error.message}"
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
    """(a) Robot directories robots.json doesn't account for, and entries naming directories that don't exist."""
    problems = []
    claimed = {d: robot_id for robot_id, robot in doc["robots"].items() for d in robot["dirs"]}
    covered = [*claimed, *doc["excluded"]]
    for directory in covered:
        if not (root / directory).is_dir():
            where = f"robots.{claimed[directory]}.dirs" if directory in claimed else "excluded"
            problems.append(f"robots.json {where} names {directory}, which is not a directory")
    for directory in _robot_dirs(root):
        if any(_inside(directory, c) for c in covered):
            continue
        if any(_inside(c, directory) for c in covered):
            continue  # a vendor folder (dimos/robot/unitree) holding robots
        problems.append(
            f"{directory} is not in robots.json: if it is a robot, add it to a robot's `dirs` (or add a robot: "
            f'"robots": {{"<id>": {{"name": ..., "description": ..., "dirs": ["{directory}"], "blueprints": {{}}}}}}); '
            f'if it isn\'t, add it to `excluded`: "{directory}": "<why it is not a robot>"'
        )
    return problems


def blueprint_problems(doc: dict[str, Any], registry: dict[str, str]) -> list[str]:
    """(b) and (c): blueprints the code has that robots.json doesn't list where it should, and the reverse."""
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
                f"robots.json lists {name} under robots.{robots[0]}, but no such blueprint is registered in "
                f"dimos/robot/all_blueprints.py{hint}: fix the name or remove it"
            )
    excluded = list(doc["excluded"])
    for name, ref in sorted(registry.items()):
        path = blueprint_file(ref)
        owner = owner_of(doc, path)
        entry = '{"title": ..., "description": ..., "tags": [...]}'
        if owner is not None:
            if owner not in listed.get(name, []):
                problems.append(
                    f"blueprint {name} ({path}) is in robots.{owner}'s dirs but robots.json doesn't list it: add "
                    f'"{name}": {entry} to robots.{owner}.blueprints (with "hidden": true if it is a test or '
                    "building block)"
                )
        elif any(_inside(path, d) for d in excluded):
            problems.append(
                f"blueprint {name} is defined in {path}, inside a directory robots.json excludes: list it under a "
                "robot (and give that robot the directory), or move it"
            )
        elif _inside(path, ROBOT_ROOT) and name not in listed:
            problems.append(
                f"blueprint {name} ({path}) is under {ROBOT_ROOT} but no robot's dirs hold it: add that directory "
                f'to a robot\'s dirs and "{name}": {entry} to its blueprints'
            )
    return problems


def _modes_of(robot: dict[str, Any], blueprint: dict[str, Any]) -> dict[str, Any]:
    modes: dict[str, Any] = (
        blueprint.get("modes") or robot.get("defaults", {}).get("modes") or {"robot": {}}
    )
    return modes


def _recommended_config_of(
    robot: dict[str, Any], blueprint: dict[str, Any]
) -> list[dict[str, Any]]:
    """A blueprint's recommended settings: its own, else its robot's defaults', else none."""
    if "recommended_config" in blueprint:
        return list(blueprint["recommended_config"])
    return list(robot.get("defaults", {}).get("recommended_config", []))


def _setting_spec(doc: dict[str, Any], setting: dict[str, Any]) -> dict[str, Any]:
    """A recommended setting as an arg: the top-level arg it names (with its own label, choices and default on top),
    or itself."""
    if "arg" in setting:
        rest = {k: v for k, v in setting.items() if k != "arg"}
        return {**doc["args"].get(setting["arg"], {}), **rest}
    return dict(setting)


def _each_mode(doc: dict[str, Any]) -> list[tuple[str, str, str, dict[str, Any]]]:
    """(robot id, blueprint name, mode id, mode) for every blueprint's effective modes."""
    return [
        (robot_id, name, mode_id, mode)
        for robot_id, robot in doc["robots"].items()
        for name, blueprint in robot["blueprints"].items()
        for mode_id, mode in _modes_of(robot, blueprint).items()
    ]


def consistency_problems(doc: dict[str, Any]) -> list[str]:
    """(d) GlobalConfig names, and (e) tags, groups, starter ranks, recommended blueprints and settings."""
    from dimos.core.global_config import GlobalConfig

    fields = GlobalConfig.model_fields
    problems = []
    tags = doc["tags"]
    for tag in MODE_TAGS:
        if tag not in tags:
            problems.append(
                f"robots.json tags must define {tag} (it comes from a blueprint's {tag} mode)"
            )
    ranks: dict[int, str] = {}
    for robot_id, robot in doc["robots"].items():
        if "group" in robot and robot["group"] not in doc["groups"]:
            problems.append(
                f"robots.{robot_id}.group is {robot['group']!r}, not one of groups: {', '.join(doc['groups'])}"
            )
        for name in robot.get("recommended", []):
            if name not in robot["blueprints"]:
                problems.append(
                    f"robots.{robot_id}.recommended: {name!r} is not one of its blueprints"
                    f"{_guess(name, robot['blueprints'])}"
                )
        for name, blueprint in robot["blueprints"].items():
            for tag in blueprint["tags"]:
                if tag in MODE_TAGS:
                    problems.append(
                        f"robots.{robot_id}.blueprints.{name}: drop the {tag!r} tag, it comes from having a "
                        f"{tag!r} mode"
                    )
                elif tag not in tags:
                    problems.append(
                        f"robots.{robot_id}.blueprints.{name}: unknown tag {tag!r} (tags: {', '.join(tags)}; "
                        "add it to the top-level `tags` with a label and description if it is a new one)"
                    )
            rank = blueprint.get("starter")
            if rank is not None:
                if rank in ranks:
                    problems.append(
                        f"{name} and {ranks[rank]} both have starter rank {rank}: give each its own"
                    )
                ranks[rank] = name
    for arg_id, arg in doc["args"].items():
        if "global" in arg and arg["global"] not in fields:
            problems.append(
                f"args.{arg_id}: {arg['global']!r} is not a GlobalConfig field{_guess(arg['global'], fields)}"
            )
        if arg.get("streams") and arg.get("kind") != "recording":
            problems.append(f"args.{arg_id}: `streams` only goes on a recording arg")
    used = set()
    for robot_id, robot in doc["robots"].items():
        containers = [(f"robots.{robot_id}.defaults", robot.get("defaults", {}))]
        containers += [
            (f"robots.{robot_id}.blueprints.{n}", b) for n, b in robot["blueprints"].items()
        ]
        for where, container in containers:
            for mode_id, mode in container.get("modes", {}).items():
                for key in mode.get("set", {}):
                    if key not in fields:
                        problems.append(
                            f"{where} {mode_id} mode sets {key!r}, which is not a GlobalConfig field"
                            f"{_guess(key, fields)}"
                        )
                for arg_id in mode.get("args", []):
                    used.add(arg_id)
                    if arg_id not in doc["args"]:
                        problems.append(
                            f"{where} {mode_id} mode takes arg {arg_id!r}, which isn't defined in the top-level "
                            f"`args`{_guess(arg_id, doc['args'])}"
                        )
            for setting in container.get("recommended_config", []):
                if "arg" in setting:
                    used.add(setting["arg"])
                    if setting["arg"] not in doc["args"]:
                        problems.append(
                            f"{where}.recommended_config names arg {setting['arg']!r}, which isn't defined in the "
                            f"top-level `args`{_guess(setting['arg'], doc['args'])}"
                        )
                        continue
                spec = _setting_spec(doc, setting)
                if "global" in spec and spec["global"] not in fields:
                    problems.append(
                        f"{where}.recommended_config: {spec['global']!r} is not a GlobalConfig field"
                        f"{_guess(spec['global'], fields)}"
                    )
                values = [choice["value"] for choice in spec.get("choices", [])]
                if len(values) != len({json.dumps(v) for v in values}):
                    problems.append(
                        f"{where}.recommended_config: {spec['label']!r} has a choice twice"
                    )
                if values and "default" in spec and spec["default"] not in values:
                    problems.append(
                        f"{where}.recommended_config: {spec['label']!r} defaults to {spec['default']!r}, "
                        "which isn't one of its choices"
                    )
    for arg_id in doc["args"]:
        if arg_id not in used:
            problems.append(f"args.{arg_id} is defined but no mode takes it: remove it")
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
    starts; robots in one vendor folder share a manufacturer (and have one); DIY robots have none; arms are the `arms`
    group's robots; and something with no code under a robot root (dimos/robot, dimos/experimental/robot) isn't a robot
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
        in_arms = robot.get("group") == "arms"
        if kind == "arm" and not in_arms:
            problems.append(f'{where}.type is "arm": put it in the arms group ("group": "arms")')
        if in_arms and kind not in ("arm", None):
            problems.append(
                f'{where} is in the arms group, so its type is "arm" (or null if it isn\'t a robot)'
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
    """Blueprint name -> the (module, field) pairs its modes' args and its recommended settings name."""
    found: dict[str, set[tuple[str, str]]] = {}
    for _, name, _, mode in _each_mode(doc):
        for arg_id in mode.get("args", []):
            arg = doc["args"].get(arg_id, {})
            if "module" in arg:
                found.setdefault(name, set()).add((arg["module"], arg["field"]))
    for robot in doc["robots"].values():
        for name, blueprint in robot["blueprints"].items():
            for setting in _recommended_config_of(robot, blueprint):
                spec = _setting_spec(doc, setting)
                if "module" in spec:
                    found.setdefault(name, set()).add((spec["module"], spec["field"]))
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
    """Each `docs` link in robots.json -> where it is (`args.<id>`)."""
    found: dict[str, list[str]] = {}
    for arg_id, arg in doc["args"].items():
        if "docs" in arg:
            found.setdefault(arg["docs"], []).append(f"args.{arg_id}")
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
        where = ", ".join(f"robots.json {w}.docs" for w in wheres)
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
    """Everything wrong with robots.json, except module args (`module_arg_problems` imports blueprints) and docs links
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


def resolved(doc: dict[str, Any], registry: dict[str, str] | None = None) -> dict[str, Any]:
    """robots.json with every blueprint's defaults applied: its modes (each arg inlined, with `id`, `key` and `scope`), its tags
    including replay/sim from its modes, its robot, recommended app, `starter`, `hidden` and `recommended_config` (each
    setting as a resolved arg, with `choices` or None). With the registry: each
    blueprint's `registered`, and `unlisted`, the registered blueprints no robot lists."""
    out = copy.deepcopy(doc)
    out.pop("$schema", None)
    out.pop("args", None)  # inlined into each mode
    listed = set()
    for robot_id, robot in out["robots"].items():
        for name, blueprint in robot["blueprints"].items():
            listed.add(name)
            modes = _modes_of(doc["robots"][robot_id], doc["robots"][robot_id]["blueprints"][name])
            blueprint["modes"] = {
                mode_id: {
                    "set": copy.deepcopy(mode.get("set", {})),
                    "args": [_resolved_arg(a, doc["args"][a]) for a in mode.get("args", [])],
                }
                for mode_id, mode in modes.items()
            }
            blueprint["tags"] = [
                tag
                for tag in doc["tags"]
                if tag in blueprint["tags"] or (tag in MODE_TAGS and tag in modes)
            ]
            blueprint["robot"] = robot_id
            blueprint["recommended_config"] = [
                {
                    **_resolved_arg(
                        setting.get("arg")
                        or setting.get("global")
                        or f"{setting['module']}.{setting['field']}",
                        _setting_spec(doc, setting),
                    ),
                    "choices": copy.deepcopy(_setting_spec(doc, setting).get("choices")),
                }
                for setting in _recommended_config_of(
                    doc["robots"][robot_id], doc["robots"][robot_id]["blueprints"][name]
                )
            ]
            if "recommended_app" not in blueprint:
                blueprint["recommended_app"] = copy.deepcopy(robot.get("recommended_app"))
            blueprint.setdefault("starter", None)
            blueprint.setdefault("hidden", False)
            if registry is not None:
                blueprint["registered"] = name in registry
        robot.pop("defaults", None)
        robot.setdefault("group", None)
        robot.setdefault("recommended_app", None)
        robot.setdefault("recommended", [])
    if registry is not None:
        out["unlisted"] = sorted(set(registry) - listed)
    return out
