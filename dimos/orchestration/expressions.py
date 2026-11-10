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

"""CEL expressions over a machine's params, vars, inputs and event."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
import logging
import re

from cel_expr_python import cel
from pydantic import JsonValue

logger = logging.getLogger(__name__)

_MAP = cel.Type.Map(cel.Type.STRING, cel.Type.DYN)
_EVENT_FIELDS = {
    "event.reason": cel.Type.STRING,
    "event.result": cel.Type.DYN,
    "event.msg": _MAP,
    "event.data": _MAP,
}
_COMPILE_ERROR = re.compile(
    r"ERROR: <input>:\d+:(\d+): (.*?)(?: \(in container ''\))?$", re.MULTILINE
)
_FAILED = (cel.Type.ERROR, cel.Type.UNKNOWN)

Scope = dict[str, JsonValue]


class ExpressionError(Exception):
    """An expression that fails to compile or evaluate."""


def kind_of(value: JsonValue) -> str:
    """The CEL kind of a TOML value, such as int, double or list."""
    return _kind(_cel_type(value))


def _cel_type(value: JsonValue) -> cel.Type:
    if isinstance(value, bool):
        return cel.Type.BOOL
    if isinstance(value, int):
        return cel.Type.INT
    if isinstance(value, float):
        return cel.Type.DOUBLE
    if isinstance(value, str):
        return cel.Type.STRING
    if isinstance(value, list):
        return cel.Type.List(cel.Type.DYN)
    if isinstance(value, dict):
        return _MAP
    return cel.Type.DYN


def _kind(cel_type: cel.Type) -> str:
    return cel_type.name().split("<", 1)[0].lower()


class Expression:
    """A compiled CEL expression."""

    def __init__(self, source: str, compiled: cel.Expression) -> None:
        self.source = source
        self.kind = _kind(compiled.return_type())
        self._compiled = compiled

    def evaluate(self, scope: Scope) -> JsonValue:
        """Evaluate against a scope, raising ExpressionError on any failure."""
        value = self._compiled.eval(data=scope)
        if value.type() in _FAILED:
            raise ExpressionError(f"{self.source!r}: {value.plain_value()}")
        return value.plain_value()

    def test(self, scope: Scope) -> bool:
        """Evaluate as a guard, counting a failure as false."""
        try:
            result = self.evaluate(scope)
        except ExpressionError as error:
            logger.warning("Guard failed, counting it as false: %s", error)
            return False
        if not isinstance(result, bool):
            logger.warning("Guard %r gave %r, counting it as false", self.source, result)
            return False
        return result

    def assign(self, scope: Scope, current: JsonValue) -> JsonValue:
        """Evaluate a var's new value, keeping the current one on a failure or a type change."""
        try:
            result = self.evaluate(scope)
        except ExpressionError as error:
            logger.warning("Set failed, keeping %r: %s", current, error)
            return current
        if type(result) is not type(current):
            logger.warning(
                "Set %r gave %s for a var of type %s, keeping %r",
                self.source,
                kind_of(result),
                kind_of(current),
                current,
            )
            return current
        return result


class Environment:
    """The names a machine's expressions can use, and a cache of compiled expressions."""

    def __init__(
        self,
        params: Mapping[str, JsonValue],
        vars: Mapping[str, JsonValue],
        inputs: Iterable[str],
    ) -> None:
        variables = {f"params.{name}": _cel_type(value) for name, value in params.items()}
        variables |= {f"vars.{name}": _cel_type(value) for name, value in vars.items()}
        variables |= {f"inputs.{name}": _MAP for name in inputs}
        variables |= _EVENT_FIELDS
        self._env = cel.NewEnv(variables=variables)
        self._params = {f"params.{name}": value for name, value in params.items()}
        self._compiled: dict[str, Expression] = {}

    def compile(self, source: str) -> Expression:
        """Compile and type-check an expression, raising ExpressionError if it is invalid."""
        if source not in self._compiled:
            try:
                compiled = self._env.compile(source)
            except RuntimeError as error:
                problems = [
                    f"{message} at column {column}"
                    for column, message in _COMPILE_ERROR.findall(str(error))
                ]
                raise ExpressionError(", ".join(problems) or str(error)) from error
            self._compiled[source] = Expression(source, compiled)
        return self._compiled[source]

    def scope(
        self,
        vars: Mapping[str, JsonValue],
        inputs: Mapping[str, JsonValue],
        event: Mapping[str, JsonValue],
    ) -> Scope:
        """Flatten the vars, the latest inputs and the event's fields into the names expressions use."""
        scope: Scope = dict(self._params)
        scope |= {f"vars.{name}": value for name, value in vars.items()}
        scope |= {f"inputs.{name}": value for name, value in inputs.items()}
        scope |= {f"event.{name}": value for name, value in event.items()}
        return scope
