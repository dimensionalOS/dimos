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

"""Static import scanner shared by the dependency catalog and codebase checks.

Every import statement in a file is classified by *when* it runs:

- ``EAGER``: at import time of the module, unconditionally (module level,
  class bodies, ``else`` branches of ``TYPE_CHECKING`` guards, ...).
- ``OPTIONAL``: at import time but tolerated when it fails: inside a ``try``
  whose handler catches ``ImportError`` (or a broader exception), or under an
  availability flag such as ``if DDS_AVAILABLE:`` that such a ``try`` assigned.
- ``LAZY``: inside a function, method or lambda body.
- ``TYPE_ONLY``: inside ``if TYPE_CHECKING:``.
- ``MAIN_ONLY``: inside ``if __name__ == "__main__":``.

Eager sites under a module-level ``if`` that reads ``global_config`` carry a
predicate (see :mod:`dimos.deps.predicates`), so a blueprint's composition can
be evaluated per configuration scenario without executing anything.

The scanner also reads two kinds of literals: ``Requires(...)`` declarations
(:mod:`dimos.deps.requires`) and registry selections such as
``adapter_type="xarm"``, whose value may be a constant, a name bound to a
constant (module level, a parameter default or a local assignment) or a
conditional expression over constants. Anything else is reported as an error
instead of being interpreted.

Nothing here imports the runtime framework; the scanner only parses sources.
"""

from __future__ import annotations

import ast
from collections.abc import Callable, Iterator
from dataclasses import dataclass
import enum
from fnmatch import fnmatch
from pathlib import Path
from typing import cast

from dimos.deps.predicates import Predicate, simplify
from dimos.deps.requires import Requires

IMPORT_ERROR_NAMES = frozenset({"ImportError", "ModuleNotFoundError", "Exception", "BaseException"})
NATIVE_CONFIG_BASE = "NativeModuleConfig"
REQUIRES_NAME = "Requires"
TASK_CONFIG_NAME = "TaskConfig"
SELECTOR_KEYWORDS = {"adapter_type": "adapter", "type": "task"}
"""Call keyword -> registry family; ``type`` only counts on ``TaskConfig(...)``."""
Bindings = dict[str, list[tuple[str, Predicate | None]]]
TRY_NODE_TYPES: tuple[type[ast.AST], ...] = (
    (ast.Try, ast.TryStar) if hasattr(ast, "TryStar") else (ast.Try,)
)


class ImportKind(enum.Enum):
    EAGER = "eager"
    OPTIONAL = "optional"
    LAZY = "lazy"
    TYPE_ONLY = "type_only"
    MAIN_ONLY = "main_only"


@dataclass(frozen=True)
class ImportSite:
    """One import statement (one entry per imported module for ``import a, b``)."""

    module: str
    """Absolute dotted module: ``x.y`` for ``import x.y`` and ``from x.y import z``."""
    names: tuple[str, ...]
    """Imported names of a ``from`` import; empty for ``import x``."""
    kind: ImportKind
    condition: Predicate | None
    """Configuration predicate guarding an eager site, when it could be derived."""
    lineno: int

    @property
    def top_level(self) -> str:
        return self.module.split(".", 1)[0]


@dataclass(frozen=True)
class ManifestRef:
    """A registry selection the file makes, such as ``adapter_type="xarm"``."""

    family: str
    name: str
    condition: Predicate | None
    lineno: int


@dataclass(frozen=True)
class Declaration:
    """One ``Requires(...)`` literal; ``owner`` is the class name or ``""`` at module scope."""

    owner: str
    lineno: int
    requires: Requires


@dataclass(frozen=True)
class FileScan:
    path: Path
    module: str
    imports: tuple[ImportSite, ...]
    manifest_refs: tuple[ManifestRef, ...]
    native_executables: tuple[str, ...]
    """Executable basenames of ``NativeModuleConfig`` subclasses defined in the file."""
    declarations: tuple[Declaration, ...]
    errors: tuple[tuple[int, str], ...]
    """``(lineno, message)`` for declarations and selections that are not literals."""


# Files that are never part of a runtime closure and are not checked on their own.
EXCLUDED_PATTERNS: tuple[str, ...] = (
    "test_*.py",
    "*/test_*.py",
    "conftest.py",
    "*/conftest.py",
    "tool_*.py",
    "*/tool_*.py",
    "*test_support*.py",
    "*/benchmark/*",
    "models/Detic/*",
    "*/node_modules/*",
    "e2e_tests/*",
    "codebase_checks/*",
    "web/dimos_interface/src/*",
)


def is_excluded_from_check(rel_path: str) -> bool:
    """Whether a path relative to ``dimos/`` is outside the dependency audit."""
    return any(fnmatch(rel_path, pattern) for pattern in EXCLUDED_PATTERNS)


def module_name_for(path: Path, project_root: Path) -> str:
    return ".".join(path.resolve().relative_to(project_root.resolve()).with_suffix("").parts)


def is_test_file(path: Path) -> bool:
    return path.name.startswith("test_") or path.name == "conftest.py"


def iter_source_files(root: Path, exclude: Callable[[Path], bool] | None = None) -> Iterator[Path]:
    """Python files under ``root`` in sorted order, skipping caches and node_modules."""
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts or "node_modules" in path.parts:
            continue
        if exclude is not None and exclude(path):
            continue
        yield path


def scan_file(
    path: Path,
    *,
    project_root: Path,
    package: str = "dimos",
    config_name: str = "global_config",
) -> FileScan:
    """Parse one source file and classify every import it contains."""
    module = module_name_for(path, project_root)
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    scanner = _Scanner(module, package, config_name)
    scanner.scan(tree)
    declarations, errors = _declarations(tree)
    return FileScan(
        path=path,
        module=module,
        imports=tuple(scanner.sites),
        manifest_refs=tuple(scanner.refs),
        native_executables=tuple(_native_executables(tree)),
        declarations=tuple(declarations),
        errors=tuple(sorted({*errors, *scanner.errors})),
    )


def resolve_internal(site: ImportSite, *, project_root: Path, package: str) -> tuple[Path, ...]:
    """Source files executed by an import of a first-party module.

    The package has no ``__init__.py`` files, so directories execute nothing:
    ``import pkg.a.b`` runs ``pkg/a/b.py`` when it is a file, and
    ``from pkg.a import x`` runs ``pkg/a.py`` when that is a file, otherwise
    ``pkg/a/x.py`` for every imported name that is a file.
    """
    if site.top_level != package:
        return ()
    parts = site.module.split(".")
    executed: list[Path] = []
    for depth in range(1, len(parts) + 1):
        candidate = project_root.joinpath(*parts[:depth]).with_suffix(".py")
        if candidate.is_file():
            executed.append(candidate)
    module_dir = project_root.joinpath(*parts)
    if not executed or not executed[-1].with_suffix("").parts == module_dir.parts:
        if module_dir.is_dir():
            for name in site.names:
                child = module_dir / f"{name}.py"
                if child.is_file():
                    executed.append(child)
    return tuple(executed)


def _is_type_checking(test: ast.expr) -> bool:
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    return isinstance(test, ast.Attribute) and test.attr == "TYPE_CHECKING"


def _is_main_guard(test: ast.expr) -> bool:
    if not isinstance(test, ast.Compare) or len(test.ops) != 1:
        return False
    if not isinstance(test.ops[0], ast.Eq):
        return False
    sides = (test.left, test.comparators[0])
    has_name = any(isinstance(side, ast.Name) and side.id == "__name__" for side in sides)
    has_main = any(isinstance(side, ast.Constant) and side.value == "__main__" for side in sides)
    return has_name and has_main


def _catches_import_error(handlers: list[ast.ExceptHandler]) -> bool:
    for handler in handlers:
        if handler.type is None:
            return True
        types = handler.type.elts if isinstance(handler.type, ast.Tuple) else [handler.type]
        for node in types:
            name = node.id if isinstance(node, ast.Name) else getattr(node, "attr", "")
            if name in IMPORT_ERROR_NAMES:
                return True
    return False


def _as_try(node: ast.AST) -> ast.Try | None:
    """``try`` and ``try*`` statements share the same fields; view either as ``ast.Try``."""
    if isinstance(node, TRY_NODE_TYPES):
        return cast("ast.Try", node)
    return None


def _constant_targets(statements: list[ast.stmt]) -> Iterator[str]:
    """Names bound to a boolean constant by simple assignments."""
    for statement in statements:
        if isinstance(statement, ast.Assign) and isinstance(statement.value, ast.Constant):
            if isinstance(statement.value.value, bool):
                for target in statement.targets:
                    if isinstance(target, ast.Name):
                        yield target.id


def _availability_flags(body: list[ast.stmt]) -> set[str]:
    """Module-level flags set by import-guarding ``try`` statements."""
    flags: set[str] = set()
    for statement in body:
        try_node = _as_try(statement)
        if try_node is not None and _catches_import_error(try_node.handlers):
            flags.update(_constant_targets(try_node.body))
            for handler in try_node.handlers:
                flags.update(_constant_targets(handler.body))
    return flags


class _Scanner:
    def __init__(self, module: str, package: str, config_name: str) -> None:
        self.module = module
        self.package = package
        self.config_name = config_name
        self.sites: list[ImportSite] = []
        self.refs: list[ManifestRef] = []
        self.errors: list[tuple[int, str]] = []
        self.flags: set[str] = set()
        self.aliases: dict[str, Predicate] = {}
        self.bindings: Bindings = {}
        """Module-level names bound to string constants, with the condition of each binding."""
        self._pending: list[tuple[str, ast.expr, Predicate | None, Bindings | None]] = []

    def scan(self, tree: ast.Module) -> None:
        self.flags = _availability_flags(tree.body)
        for statement in tree.body:
            if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
                target = statement.targets[0]
                if isinstance(target, ast.Name):
                    predicate = self._predicate(statement.value)
                    if predicate is not None:
                        self.aliases[target.id] = predicate
        self._walk(tree.body, ImportKind.EAGER, None, None)
        # Selections are resolved last so a function may use a name bound below it.
        for family, value, condition, scope in self._pending:
            self._resolve_selection(family, value, condition, scope)

    def _walk(
        self,
        statements: list[ast.stmt],
        kind: ImportKind,
        condition: Predicate | None,
        scope: Bindings | None,
    ) -> None:
        for statement in statements:
            self._visit(statement, kind, condition, scope)

    def _visit(
        self,
        node: ast.stmt,
        kind: ImportKind,
        condition: Predicate | None,
        scope: Bindings | None,
    ) -> None:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            self._record(node, kind, condition)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            self._walk(node.body, ImportKind.LAZY, None, self._local_bindings(node))
        elif isinstance(node, ast.ClassDef):
            self._walk(node.body, kind, condition, scope)
        elif isinstance(node, ast.If):
            self._visit_if(node, kind, condition, scope)
        elif (try_node := _as_try(node)) is not None:
            self._visit_try(try_node, kind, condition, scope)
        elif any(isinstance(child, ast.stmt) for child in ast.iter_child_nodes(node)):
            for child in ast.iter_child_nodes(node):
                if isinstance(child, ast.stmt):
                    self._visit(child, kind, condition, scope)
                elif isinstance(child, (ast.match_case, ast.ExceptHandler)):
                    self._walk(child.body, kind, condition, scope)
        else:
            self._visit_simple(node, kind, condition, scope)

    def _visit_simple(
        self,
        node: ast.stmt,
        kind: ImportKind,
        condition: Predicate | None,
        scope: Bindings | None,
    ) -> None:
        """A statement without nested statements: record bindings and selections."""
        if scope is None and kind is ImportKind.EAGER:
            target, value = _assigned(node)
            if target is not None and value is not None:
                constants = self._string_constants(value)
                if constants is not None:
                    self.bindings.setdefault(target, []).extend(
                        (text, _conjoin_optional(condition, when)) for text, when in constants
                    )
        for call in ast.walk(node):
            if not isinstance(call, ast.Call):
                continue
            func = call.func
            callee = func.id if isinstance(func, ast.Name) else getattr(func, "attr", "")
            for keyword in call.keywords:
                family = SELECTOR_KEYWORDS.get(keyword.arg or "")
                if family is None or (family == "task" and callee != TASK_CONFIG_NAME):
                    continue
                self._pending.append((family, keyword.value, condition, scope))

    def _resolve_selection(
        self, family: str, value: ast.expr, condition: Predicate | None, scope: Bindings | None
    ) -> None:
        if isinstance(value, ast.Name) and scope is not None and scope.get(value.id) == []:
            return  # a parameter without a default: the caller selects, as a keyword
        constants = self._string_constants(value, scope)
        if constants is None:
            self.errors.append(
                (
                    value.lineno,
                    f"{family} selection is not a literal; use a string constant, a name bound "
                    "to one, or a parameter the caller passes as a keyword",
                )
            )
            return
        for text, when in constants:
            self.refs.append(
                ManifestRef(family, text, _conjoin_optional(condition, when), value.lineno)
            )

    def _local_bindings(self, function: ast.FunctionDef | ast.AsyncFunctionDef) -> Bindings:
        """Strings bound in a function: parameter defaults and local assignments."""
        bindings: Bindings = {}
        arguments = function.args
        positional = arguments.posonlyargs + arguments.args
        for argument in (*positional, *arguments.kwonlyargs):
            bindings[argument.arg] = []  # provided by the caller unless a default says otherwise
        defaulted = positional[len(positional) - len(arguments.defaults) :]
        defaults = [
            *zip(defaulted, arguments.defaults, strict=True),
            *zip(arguments.kwonlyargs, arguments.kw_defaults, strict=True),
        ]
        for argument, default in defaults:
            constants = self._string_constants(default) if default is not None else None
            if constants is not None:
                bindings[argument.arg] = constants
        for statement in function.body:
            target, value = _assigned(statement)
            if target is None or value is None:
                continue
            constants = self._string_constants(value, bindings)
            if constants is not None:
                bindings[target] = constants
        return bindings

    def _string_constants(
        self, node: ast.expr, scope: Bindings | None = None
    ) -> list[tuple[str, Predicate | None]] | None:
        """The string values an expression can take, each with its condition."""
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return [(node.value, None)]
        if isinstance(node, ast.Name):
            bound = (scope or {}).get(node.id)
            if bound is None:
                bound = self.bindings.get(node.id)
            return list(bound) if bound else None
        if isinstance(node, ast.IfExp):
            body = self._string_constants(node.body, scope)
            orelse = self._string_constants(node.orelse, scope)
            if body is None or orelse is None:
                return None
            predicate = self._predicate(node.test)
            if predicate is None:
                return body + orelse
            return [(text, _conjoin_optional(when, predicate)) for text, when in body] + [
                (text, _conjoin_optional(when, ["not", predicate])) for text, when in orelse
            ]
        return None

    def _visit_if(
        self, node: ast.If, kind: ImportKind, condition: Predicate | None, scope: Bindings | None
    ) -> None:
        if _is_type_checking(node.test):
            self._walk(node.body, ImportKind.TYPE_ONLY, None, scope)
            self._walk(node.orelse, kind, condition, scope)
            return
        if _is_main_guard(node.test):
            self._walk(node.body, ImportKind.MAIN_ONLY, None, scope)
            self._walk(node.orelse, kind, condition, scope)
            return
        flag = self._flag_test(node.test)
        if flag is not None:
            positive_kind = ImportKind.OPTIONAL if kind is ImportKind.EAGER else kind
            body_kind, else_kind = (positive_kind, kind) if flag else (kind, positive_kind)
            self._walk(node.body, body_kind, condition, scope)
            self._walk(node.orelse, else_kind, condition, scope)
            return
        predicate = self._predicate(node.test)
        if predicate is None or kind is not ImportKind.EAGER:
            self._walk(node.body, kind, condition, scope)
            self._walk(node.orelse, kind, condition, scope)
            return
        self._walk(node.body, kind, _conjoin(condition, predicate), scope)
        self._walk(node.orelse, kind, _conjoin(condition, ["not", predicate]), scope)

    def _visit_try(
        self, node: ast.Try, kind: ImportKind, condition: Predicate | None, scope: Bindings | None
    ) -> None:
        guarded = _catches_import_error(node.handlers)
        inner_kind = ImportKind.OPTIONAL if guarded and kind is ImportKind.EAGER else kind
        self._walk(node.body, inner_kind, condition, scope)
        for handler in node.handlers:
            self._walk(handler.body, inner_kind, condition, scope)
        self._walk(node.orelse, kind, condition, scope)
        self._walk(node.finalbody, kind, condition, scope)

    def _flag_test(self, test: ast.expr) -> bool | None:
        """True for ``if FLAG:``, False for ``if not FLAG:``, None otherwise."""
        if isinstance(test, ast.Name) and test.id in self.flags:
            return True
        if isinstance(test, ast.UnaryOp) and isinstance(test.op, ast.Not):
            operand = test.operand
            if isinstance(operand, ast.Name) and operand.id in self.flags:
                return False
        return None

    def _record(
        self, node: ast.Import | ast.ImportFrom, kind: ImportKind, condition: Predicate | None
    ) -> None:
        if isinstance(node, ast.Import):
            for alias in node.names:
                self.sites.append(ImportSite(alias.name, (), kind, condition, node.lineno))
            return
        module = node.module or ""
        if node.level:
            parts = self.module.split(".")
            anchor = parts[: len(parts) - node.level]
            module = ".".join([*anchor, *([module] if module else [])])
        names = tuple(alias.name for alias in node.names)
        self.sites.append(ImportSite(module, names, kind, condition, node.lineno))

    def _config_field(self, node: ast.expr) -> str | None:
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == self.config_name:
                return node.attr
        return None

    def _predicate(self, node: ast.expr) -> Predicate | None:
        field = self._config_field(node)
        if field is not None:
            return ["truthy", field]
        if isinstance(node, ast.Name):
            return self.aliases.get(node.id)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == "bool" and len(node.args) == 1 and not node.keywords:
                return self._predicate(node.args[0])
            return None
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
            inner = self._predicate(node.operand)
            return None if inner is None else ["not", inner]
        if isinstance(node, ast.BoolOp):
            children = [self._predicate(value) for value in node.values]
            if any(child is None for child in children):
                return None
            operator = "all" if isinstance(node.op, ast.And) else "any"
            return simplify([operator, *children])
        if isinstance(node, ast.Compare) and len(node.ops) == 1:
            return self._compare(node.left, node.ops[0], node.comparators[0])
        return None

    def _compare(self, left: ast.expr, op: ast.cmpop, right: ast.expr) -> Predicate | None:
        field = self._config_field(left)
        if field is None:
            field = self._config_field(right)
            if field is None or not isinstance(op, (ast.Eq, ast.NotEq)):
                return None
            left, right = right, left
        if isinstance(op, (ast.Eq, ast.NotEq)) and isinstance(right, ast.Constant):
            operator = "eq" if isinstance(op, ast.Eq) else "ne"
            return [operator, field, right.value]
        if isinstance(op, (ast.In, ast.NotIn)) and isinstance(
            right, (ast.Tuple, ast.List, ast.Set)
        ):
            if not all(isinstance(element, ast.Constant) for element in right.elts):
                return None
            values = [element.value for element in right.elts if isinstance(element, ast.Constant)]
            predicate: Predicate = ["in", field, values]
            return predicate if isinstance(op, ast.In) else ["not", predicate]
        return None


def _conjoin(condition: Predicate | None, predicate: Predicate) -> Predicate:
    if condition is None:
        return simplify(predicate)
    return simplify(["all", condition, predicate])


def _conjoin_optional(condition: Predicate | None, predicate: Predicate | None) -> Predicate | None:
    if predicate is None:
        return condition
    return _conjoin(condition, predicate)


def _assigned(node: ast.stmt) -> tuple[str | None, ast.expr | None]:
    """Target name and value of a simple assignment to one name."""
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
        target = node.targets[0]
        if isinstance(target, ast.Name):
            return target.id, node.value
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        return node.target.id, node.value
    return None, None


def _declarations(tree: ast.Module) -> tuple[list[Declaration], list[tuple[int, str]]]:
    """``Requires(...)`` literals at module or class scope, and every misplaced one."""
    declarations: list[Declaration] = []
    errors: list[tuple[int, str]] = []
    accepted: set[int] = set()
    scopes: list[tuple[str, list[ast.stmt]]] = [("", tree.body)]
    scopes += [(node.name, node.body) for node in tree.body if isinstance(node, ast.ClassDef)]
    for owner, body in scopes:
        for statement in body:
            _target, value = _assigned(statement)
            if not isinstance(value, ast.Call) or not _is_requires(value.func):
                continue
            accepted.add(id(value))
            try:
                declarations.append(Declaration(owner, value.lineno, _literal_requires(value)))
            except ValueError as error:
                errors.append((value.lineno, str(error)))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not _is_requires(node.func) or id(node) in accepted:
            continue
        if any(keyword.arg is None for keyword in node.keywords):
            continue  # ``Requires(**values)`` builds one at run time; a declaration never does
        errors.append(
            (node.lineno, "Requires(...) must be a literal assigned at module or class scope")
        )
    return declarations, errors


def _is_requires(func: ast.expr) -> bool:
    return isinstance(func, ast.Name) and func.id == REQUIRES_NAME


def _literal_requires(call: ast.Call) -> Requires:
    if call.args:
        raise ValueError("Requires(...) takes keyword arguments only")
    values: dict[str, object] = {}
    for keyword in call.keywords:
        if keyword.arg is None:
            raise ValueError("Requires(...) does not accept ** arguments")
        try:
            literal = ast.literal_eval(keyword.value)
        except ValueError:
            raise ValueError(f"Requires({keyword.arg}=...) is not a literal") from None
        if keyword.arg == "selectors":
            if not isinstance(literal, dict) or not all(
                isinstance(k, str) and isinstance(v, str) for k, v in literal.items()
            ):
                raise ValueError("Requires(selectors=...) must map strings to strings")
            values[keyword.arg] = dict(literal)
        elif keyword.arg in Requires.__dataclass_fields__:
            if not isinstance(literal, (list, tuple)) or not all(
                isinstance(item, str) for item in literal
            ):
                raise ValueError(f"Requires({keyword.arg}=...) must be a tuple of strings")
            values[keyword.arg] = tuple(literal)
        else:
            raise ValueError(f"Requires() has no field {keyword.arg!r}")
    return Requires(**values)  # type: ignore[arg-type]


def _last_string(node: ast.expr) -> str | None:
    """The string constant that appears last in source order within ``node``."""
    constants = [
        child
        for child in ast.walk(node)
        if isinstance(child, ast.Constant) and isinstance(child.value, str)
    ]
    if not constants:
        return None
    last = max(constants, key=lambda constant: (constant.lineno, constant.col_offset))
    return str(last.value)


def _native_executables(tree: ast.Module) -> Iterator[str]:
    for node in tree.body:
        if not isinstance(node, ast.ClassDef):
            continue
        bases = [
            base.id if isinstance(base, ast.Name) else getattr(base, "attr", "")
            for base in node.bases
        ]
        if NATIVE_CONFIG_BASE not in bases:
            continue
        for statement in node.body:
            target: ast.expr | None = None
            value: ast.expr | None = None
            if isinstance(statement, ast.AnnAssign):
                target, value = statement.target, statement.value
            elif isinstance(statement, ast.Assign) and len(statement.targets) == 1:
                target, value = statement.targets[0], statement.value
            if isinstance(target, ast.Name) and target.id == "executable" and value is not None:
                path = _last_string(value)
                if path:
                    yield path.rsplit("/", 1)[-1]
