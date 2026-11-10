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

"""Caller-driven hierarchical state selection and task execution."""

from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import Enum, auto
import math
from types import MappingProxyType
from typing import Generic, TypeVar

C = TypeVar("C")
MAX_COMPLETION_TRANSITIONS = 5


class Status(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class Trigger(Enum):
    TICK = auto()
    EVENT = auto()
    COMPLETED = auto()
    SUCCESS = auto()
    FAILURE = auto()


class Completion(Enum):
    ANY = auto()
    ALL = auto()


class Task(Generic[C]):
    """Fresh instance per activation. Exit also runs when enter raises or fails."""

    def enter(self, ctx: C) -> Status:
        return Status.RUNNING

    def tick(self, ctx: C, dt: float) -> Status:
        return Status.RUNNING

    def exit(self, ctx: C) -> None:
        pass


@dataclass(frozen=True)
class Transition(Generic[C]):
    """Ordered transition to a named state or a terminal status.

    Attributes:
        event: Required only for EVENT triggers. Events carry a name, without payload.
        condition: Optional synchronous context predicate.
    """

    trigger: Trigger
    target: str | Status
    condition: Callable[[C], bool] | None = None
    event: str | None = None


@dataclass(frozen=True)
class State(Generic[C]):
    """Immutable definition; tasks are zero-argument factories, never instances.

    Attributes:
        name: Unique within the tree, also used as a transition target.
        enter_if: Selection predicate, not a continuous guard.
        children: Tried in order with backtracking until a leaf can be selected.
        completion: ANY succeeds on one task; ALL waits for every task. Failure wins.
    """

    name: str
    enter_if: Callable[[C], bool] | None = None
    tasks: tuple[Callable[[], Task[C]], ...] = ()
    children: tuple["State[C]", ...] = ()
    transitions: tuple[Transition[C], ...] = ()
    completion: Completion = Completion.ANY

    def __post_init__(self) -> None:
        object.__setattr__(self, "tasks", tuple(self.tasks))
        object.__setattr__(self, "children", tuple(self.children))
        object.__setattr__(self, "transitions", tuple(self.transitions))


@dataclass(frozen=True)
class StateTree(Generic[C]):
    """Validated reusable definition. Factories and predicates must not share task state."""

    root: State[C]
    _paths: Mapping[str, tuple[State[C], ...]] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        paths: dict[str, tuple[State[C], ...]] = {}
        seen: set[int] = set()

        def visit(state: State[C], parents: tuple[State[C], ...]) -> None:
            if id(state) in seen:
                raise ValueError("State nodes cannot be shared or cyclic")
            seen.add(id(state))
            if not state.name or state.name in paths:
                raise ValueError(f"State names must be nonempty and unique: {state.name!r}")
            if not isinstance(state.completion, Completion):
                raise ValueError(f"Invalid completion policy on {state.name!r}")
            paths[state.name] = (*parents, state)
            for child in state.children:
                visit(child, paths[state.name])

        visit(self.root, ())
        for path in paths.values():
            for transition in path[-1].transitions:
                if not isinstance(transition.trigger, Trigger):
                    raise ValueError("Invalid transition trigger")
                if isinstance(transition.target, str):
                    if transition.target not in paths:
                        raise ValueError(f"Unknown target: {transition.target!r}")
                elif transition.target not in (Status.SUCCEEDED, Status.FAILED):
                    raise ValueError("Transition status targets must be terminal")
                if transition.trigger is Trigger.EVENT:
                    if not transition.event:
                        raise ValueError("EVENT transitions require a nonempty event name")
                elif transition.event is not None:
                    raise ValueError("Only EVENT transitions can specify an event name")
        object.__setattr__(self, "_paths", MappingProxyType(paths))

    def path_to(self, name: str) -> tuple[State[C], ...]:
        """Return the root-to-state ancestry, without selecting children."""
        return self._paths[name]


@dataclass
class _TaskInstance(Generic[C]):
    task: Task[C]
    status: Status = Status.RUNNING


@dataclass
class _Frame(Generic[C]):
    state: State[C]
    tasks: list[_TaskInstance[C]] = field(default_factory=list)

    def completed(self) -> Status | None:
        statuses = [instance.status for instance in self.tasks]
        if Status.FAILED in statuses:
            return Status.FAILED
        if statuses and (
            all(status is Status.SUCCEEDED for status in statuses)
            if self.state.completion is Completion.ALL
            else Status.SUCCEEDED in statuses
        ):
            return Status.SUCCEEDED
        return None


class Runtime(Generic[C]):
    """Single-owner synchronous execution with caller-supplied context and clock.

    Serialize all access on one thread or application loop. Execution callbacks may
    enqueue events, but may not recursively start, tick or stop this runtime.
    """

    def __init__(self, tree: StateTree[C], context: C) -> None:
        self._tree = tree
        self._context = context
        self._frames: list[_Frame[C]] = []
        self._events: list[str] = []
        self._status: Status | None = None
        self._executing = False

    @property
    def status(self) -> Status | None:
        """None before start or after explicit stop; terminal results persist until restart."""
        return self._status

    @property
    def active_path(self) -> tuple[str, ...]:
        return tuple(frame.state.name for frame in self._frames)

    def start(self) -> Status:
        """Select and enter root; restart a stopped/terminal runtime with fresh tasks."""
        if self._status is Status.RUNNING:
            raise RuntimeError("StateTree is already running")
        with self._operation():
            self._events.clear()
            self._status = Status.RUNNING
            path = self._select(self._tree.root)
            if path is None:
                self._finish(Status.FAILED)
            else:
                self._activate(path, 0)
                self._settle()
            return self._result()

    def tick(self, dt: float) -> Status:
        """Tick tasks, take at most one event/tick transition, then settle completion.

        Snapshot and consume queued events at entry. Events emitted by any callback
        wait until the next external tick. Terminal ticks return the stored result.
        """
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and nonnegative")
        if self._status is None:
            raise RuntimeError("StateTree must be started before ticking")
        with self._operation():
            if self._status is not Status.RUNNING:
                return self._result()
            events, self._events = self._events, []
            failed = False
            for frame in self._frames:
                for instance in frame.tasks:
                    if instance.status is Status.RUNNING:
                        instance.status = self._task_status(instance.task.tick(self._context, dt))
                        if instance.status is Status.FAILED:
                            failed = True
                            break
                if failed:
                    break
            taskless = not any(frame.tasks for frame in self._frames)
            transitioned = self._conditional_transition(events)
            self._settle(taskless=taskless and not transitioned)
            return self._result()

    def send_event(self, name: str) -> None:
        """Queue an event while running, including from execution callbacks."""
        if not name:
            raise ValueError("Event name must be nonempty")
        if self._status is not Status.RUNNING:
            raise RuntimeError("StateTree must be running to receive events")
        self._events.append(name)

    def stop(self) -> None:
        """Exit every entered task once, discard events and reset status to None."""
        with self._operation():
            self._events.clear()
            self._exit_to(0)
            self._events.clear()
            self._status = None

    @contextmanager
    def _operation(self) -> Iterator[None]:
        if self._executing:
            raise RuntimeError("StateTree execution is not reentrant")
        self._executing = True
        try:
            yield
        except BaseException as error:
            self._status = Status.FAILED
            self._events.clear()
            try:
                self._exit_to(0)
            except BaseException as cleanup_error:
                raise error from cleanup_error
            raise
        finally:
            self._executing = False

    def _result(self) -> Status:
        assert self._status is not None
        return self._status

    @staticmethod
    def _task_status(status: Status) -> Status:
        if not isinstance(status, Status):
            raise TypeError("Task enter/tick must return a Status")
        return status

    def _select(self, state: State[C]) -> tuple[State[C], ...] | None:
        if state.enter_if is not None and not state.enter_if(self._context):
            return None
        if not state.children:
            return (state,)
        for child in state.children:
            selected = self._select(child)
            if selected is not None:
                return (state, *selected)
        return None

    def _target_path(
        self, target: str, completed: int | None
    ) -> tuple[tuple[State[C], ...], int] | None:
        ancestors = self._tree.path_to(target)
        retained = 0
        for frame, ancestor in zip(self._frames, ancestors[:-1], strict=False):
            if frame.state is not ancestor:
                break
            retained += 1
        if completed is not None:
            retained = min(retained, completed)
        for ancestor in ancestors[retained:-1]:
            if ancestor.enter_if is not None and not ancestor.enter_if(self._context):
                return None
        selected = self._select(ancestors[-1])
        if selected is None:
            return None
        return ((*ancestors[:-1], *selected), retained)

    def _activate(self, path: tuple[State[C], ...], retained: int) -> None:
        self._exit_to(retained)
        for state in path[retained:]:
            frame = _Frame(state)
            self._frames.append(frame)
            for factory in state.tasks:
                instance = _TaskInstance(factory())
                frame.tasks.append(instance)
                instance.status = self._task_status(instance.task.enter(self._context))
                if instance.status is Status.FAILED:
                    return

    def _exit_to(self, retained: int) -> None:
        error: BaseException | None = None
        while len(self._frames) > retained:
            frame = self._frames.pop()
            while frame.tasks:
                instance = frame.tasks.pop()
                try:
                    instance.task.exit(self._context)
                except BaseException as exit_error:
                    if error is None:
                        error = exit_error

        if error is not None:
            raise error

    def _finish(self, status: Status) -> None:
        self._exit_to(0)
        self._events.clear()
        self._status = status

    def _take(self, transition: Transition[C], completed: int | None = None) -> bool:
        if transition.condition is not None and not transition.condition(self._context):
            return False
        if isinstance(transition.target, Status):
            self._finish(transition.target)
            return True
        selection = self._target_path(transition.target, completed)
        if selection is None:
            return False
        self._activate(*selection)
        return True

    def _conditional_transition(self, events: list[str]) -> bool:
        for frame in reversed(self._frames):
            for transition in frame.state.transitions:
                if (
                    transition.trigger is Trigger.TICK
                    or (transition.trigger is Trigger.EVENT and transition.event in events)
                ) and self._take(transition):
                    return True
        return False

    def _settle(self, taskless: bool = False) -> None:
        for iteration in range(MAX_COMPLETION_TRANSITIONS + 1):
            if self._status is not Status.RUNNING:
                return
            completed = next(
                (
                    (index, status)
                    for index, frame in enumerate(self._frames)
                    if (status := frame.completed()) is not None
                ),
                None,
            )
            if completed is None and taskless:
                completed = (len(self._frames) - 1, Status.SUCCEEDED)
            taskless = False
            if completed is None:
                return
            if iteration == MAX_COMPLETION_TRANSITIONS:
                self._finish(Status.FAILED)
                return
            index, status = completed
            triggers = (
                Trigger.COMPLETED,
                Trigger.SUCCESS if status is Status.SUCCEEDED else Trigger.FAILURE,
            )
            accepted = False
            for frame in reversed(self._frames[: index + 1]):
                for transition in frame.state.transitions:
                    if transition.trigger in triggers and self._take(transition, index):
                        accepted = True
                        break
                if accepted:
                    break
            if not accepted:
                path = self._select(self._tree.root)
                if path is None:
                    self._finish(Status.FAILED)
                else:
                    self._activate(path, 0)
