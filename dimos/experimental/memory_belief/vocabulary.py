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

"""Naming what the detector was asked about, so a "no" differs from a "never asked".

A closed list cannot see what it was not given. That cost is only acceptable
because the list travels with the record --
:attr:`~dimos.experimental.memory_belief.types.BeliefObservation.vocabulary` -- so a later
"is there a humidifier here" answers OUT_OF_VOCABULARY rather than a confident
and wrong "no".

LVIS is used rather than a hand-written list because it is a published object
vocabulary with a long tail of household and office items, and a hand-maintained
list drifts.
"""

from __future__ import annotations

import re


def _clean(name: str) -> str:
    """LVIS names carry parenthetical disambiguators and slash-separated
    synonyms: ``monitor_(computer_equipment)/television_monitor``. The first
    synonym, de-underscored and stripped of the parenthetical, is what a person
    would say and what the text encoder handles best."""
    primary = str(name).split("/")[0]
    primary = re.sub(r"_?\(.*?\)", "", primary)
    return primary.replace("_", " ").strip()


def lvis_names() -> tuple[str, ...]:
    """The 1,203 LVIS categories as readable primary names.

    Returned in the dataset's own order so an index means the same thing across
    runs. Duplicates after cleaning are dropped, keeping first occurrence.
    """
    import importlib.util

    import yaml

    spec = importlib.util.find_spec("ultralytics")
    if spec is None or not spec.submodule_search_locations:
        raise RuntimeError("ultralytics is not installed; LVIS names unavailable")
    root = spec.submodule_search_locations[0]
    path = f"{root}/cfg/datasets/lvis.yaml"

    with open(path) as handle:
        names = yaml.safe_load(handle)["names"]

    seen: set[str] = set()
    out: list[str] = []
    for key in sorted(names, key=int):
        cleaned = _clean(names[key])
        if cleaned and cleaned not in seen:
            seen.add(cleaned)
            out.append(cleaned)
    return tuple(out)


def from_file(path: str) -> tuple[str, ...]:
    """One term per line. Blank lines and ``#`` comments ignored.

    Exists so a deployment can narrow or extend the list without editing code;
    whatever it returns is what gets recorded on every observation, so the file
    is part of the evidence rather than a setting.
    """
    with open(path) as handle:
        lines = [line.split("#")[0].strip() for line in handle]
    return tuple(dict.fromkeys(line for line in lines if line))


def resolve(spec: str | None) -> tuple[str, ...] | None:
    """Turn a CLI value into a vocabulary, or ``None`` for open-vocabulary.

    ``None`` is not "no vocabulary chosen" -- it is the claim that the run
    covered everything, and it is recorded as such.
    """
    if spec is None or spec.lower() in {"none", "open", "prompt-free"}:
        return None
    if spec.lower() == "lvis":
        return lvis_names()
    return from_file(spec)
