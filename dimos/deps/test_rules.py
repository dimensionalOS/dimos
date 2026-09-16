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

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.global_config import GlobalConfig
from dimos.deps.predicates import evaluate, fields
from dimos.deps.rules import DEFAULTS, GLOBAL_RULES, SCENARIOS

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"


def test_defaults_match_global_config() -> None:
    constructed = GlobalConfig.model_construct()
    for name, value in DEFAULTS.items():
        if name in GlobalConfig.PLANNING_PROPERTIES:
            assert getattr(constructed, name) == value, name
        else:
            assert GlobalConfig.model_fields[name].default == value, name


def test_rule_paths_exist() -> None:
    for rule in GLOBAL_RULES:
        for root in rule.roots:
            assert (DIMOS_DIR / root).is_file(), root


def test_predicates_only_read_known_fields() -> None:
    known = set(DEFAULTS)
    for rule in GLOBAL_RULES:
        assert fields(rule.when) <= known, rule.name
    for scenario in SCENARIOS.values():
        assert fields(scenario.when) <= known, scenario.name
        assert set(scenario.overrides) <= known


def test_scenarios_and_rules_are_off_by_default() -> None:
    for scenario in SCENARIOS.values():
        assert evaluate(scenario.when, DEFAULTS) is False, scenario.name
        assert evaluate(scenario.when, {**DEFAULTS, **scenario.overrides}) is True, scenario.name
    for rule in GLOBAL_RULES:
        assert evaluate(rule.when, DEFAULTS) is False, rule.name
