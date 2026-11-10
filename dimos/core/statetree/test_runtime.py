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

from dataclasses import dataclass, field
from functools import partial

import pytest

from dimos.core.statetree.runtime import (
    Completion,
    Runtime,
    State,
    StateTree,
    Status,
    Task,
    Transition,
    Trigger,
)


@dataclass
class Context:
    trace: list[str] = field(default_factory=list)
    entered: dict[str, Status] = field(default_factory=dict)
    ticked: dict[str, Status] = field(default_factory=dict)
    gates: dict[str, bool] = field(default_factory=dict)
    exit_errors: set[str] = field(default_factory=set)


class Probe(Task[Context]):
    def __init__(self, name):
        self.name = name

    def enter(self, ctx):
        ctx.trace.append(f"enter {self.name}")
        return ctx.entered.get(self.name, Status.RUNNING)

    def tick(self, ctx, dt):
        ctx.trace.append(f"tick {self.name} {dt}")
        return ctx.ticked.get(self.name, Status.RUNNING)

    def exit(self, ctx):
        ctx.trace.append(f"exit {self.name}")
        if self.name in ctx.exit_errors:
            raise ValueError(f"exit {self.name}")


def probe(name):
    return partial(Probe, name)


def test_ordered_selection_backtracks_and_stop_reverses_entry():
    ctx = Context()
    tree = StateTree(
        State(
            "root",
            tasks=(probe("root"),),
            children=(
                State("dead_end", children=(State("blocked", enter_if=lambda c: False),)),
                State("branch", tasks=(probe("first"), probe("second"))),
            ),
        )
    )
    runtime = Runtime(tree, ctx)
    assert runtime.start() is Status.RUNNING
    assert runtime.active_path == ("root", "branch")
    runtime.tick(0.25)
    runtime.stop()
    runtime.stop()
    assert ctx.trace == [
        "enter root",
        "enter first",
        "enter second",
        "tick root 0.25",
        "tick first 0.25",
        "tick second 0.25",
        "exit second",
        "exit first",
        "exit root",
    ]
    assert runtime.status is None


def test_sibling_transition_retains_ancestor_without_rechecking_its_condition():
    ctx = Context(gates={"root": True})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                enter_if=lambda c: c.gates["root"],
                tasks=(probe("root"),),
                children=(
                    State(
                        "a",
                        tasks=(probe("a"),),
                        transitions=(Transition(Trigger.EVENT, "b", event="next"),),
                    ),
                    State("b", tasks=(probe("b"),)),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    ctx.gates["root"] = False
    runtime.send_event("next")
    runtime.tick(0)
    assert runtime.active_path == ("root", "b")
    assert ctx.trace == ["enter root", "enter a", "tick root 0", "tick a 0", "exit a", "enter b"]
    runtime.stop()


@pytest.mark.parametrize("target", ["a", "root"])
def test_self_and_ancestor_targets_reenter_the_target(target):
    ctx = Context()
    runtime = Runtime(
        StateTree(
            State(
                "root",
                tasks=(probe("root"),),
                children=(
                    State(
                        "a",
                        tasks=(probe("a"),),
                        transitions=(Transition(Trigger.EVENT, target, event="again"),),
                    ),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    runtime.send_event("again")
    runtime.tick(0)
    runtime.stop()
    expected = (
        ["exit a", "enter a"] if target == "a" else ["exit a", "exit root", "enter root", "enter a"]
    )
    assert ctx.trace[4:-2] == expected


def test_completed_parent_is_reentered_when_transitioning_to_its_descendant():
    ctx = Context(ticked={"parent": Status.SUCCEEDED})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                children=(
                    State(
                        "parent",
                        tasks=(probe("parent"),),
                        children=(
                            State("a", tasks=(probe("a"),)),
                            State("b", tasks=(probe("b"),)),
                        ),
                        transitions=(Transition(Trigger.SUCCESS, "b"),),
                    ),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    runtime.tick(0)
    assert runtime.active_path == ("root", "parent", "b")
    assert ctx.trace == [
        "enter parent",
        "enter a",
        "tick parent 0",
        "tick a 0",
        "exit a",
        "exit parent",
        "enter parent",
        "enter b",
    ]
    runtime.stop()


def test_failed_target_selection_tries_next_transition_and_deepest_wins():
    ctx = Context()
    runtime = Runtime(
        StateTree(
            State(
                "root",
                transitions=(Transition(Trigger.EVENT, Status.FAILED, event="go"),),
                children=(
                    State(
                        "a",
                        tasks=(probe("a"),),
                        transitions=(
                            Transition(Trigger.EVENT, "blocked", event="go"),
                            Transition(Trigger.EVENT, "b", event="go"),
                        ),
                    ),
                    State("blocked", enter_if=lambda c: False),
                    State("b", tasks=(probe("b"),)),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    runtime.send_event("go")
    assert runtime.tick(0) is Status.RUNNING
    assert runtime.active_path == ("root", "b")
    runtime.stop()


@pytest.mark.parametrize(
    "policy,result", [(Completion.ANY, Status.SUCCEEDED), (Completion.ALL, Status.RUNNING)]
)
def test_any_all_and_completed_tasks_stop_ticking(policy, result):
    ctx = Context(ticked={"a": Status.SUCCEEDED})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                tasks=(probe("a"), probe("b")),
                completion=policy,
                transitions=(Transition(Trigger.SUCCESS, Status.SUCCEEDED),),
            )
        ),
        ctx,
    )
    runtime.start()
    assert runtime.tick(1) is result
    ctx.ticked["b"] = Status.SUCCEEDED
    assert runtime.tick(1) is Status.SUCCEEDED
    assert ctx.trace.count("tick a 1") == 1
    runtime.stop()


def test_failure_wins_stops_later_ticks_and_parent_completion_wins():
    ctx = Context(ticked={"success": Status.SUCCEEDED, "failure": Status.FAILED})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                tasks=(probe("success"), probe("failure"), probe("later")),
                children=(
                    State(
                        "child",
                        tasks=(probe("child"),),
                        transitions=(Transition(Trigger.COMPLETED, Status.SUCCEEDED),),
                    ),
                ),
                transitions=(Transition(Trigger.FAILURE, Status.FAILED),),
            )
        ),
        ctx,
    )
    runtime.start()
    assert runtime.tick(0) is Status.FAILED
    assert ctx.trace == [
        "enter success",
        "enter failure",
        "enter later",
        "enter child",
        "tick success 0",
        "tick failure 0",
        "exit child",
        "exit later",
        "exit failure",
        "exit success",
    ]


def test_event_snapshot_precedes_callbacks_and_consumes_unmatched_events():
    ctx = Context()

    class Emit(Task[Context]):
        def tick(self, ctx, dt):
            runtime.send_event("next")
            return Status.RUNNING

    runtime = Runtime(
        StateTree(
            State(
                "root",
                children=(
                    State(
                        "a",
                        tasks=(Emit,),
                        transitions=(Transition(Trigger.EVENT, "b", event="next"),),
                    ),
                    State(
                        "b",
                        tasks=(probe("b"),),
                        transitions=(Transition(Trigger.EVENT, Status.FAILED, event="unused"),),
                    ),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    runtime.send_event("unused")
    runtime.tick(0)
    assert runtime.active_path == ("root", "a")
    runtime.tick(0)
    assert runtime.active_path == ("root", "b")
    assert runtime.tick(0) is Status.RUNNING
    runtime.stop()


def test_only_one_conditional_transition_and_it_precedes_completion():
    ctx = Context(ticked={"a": Status.FAILED})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                children=(
                    State(
                        "a",
                        tasks=(probe("a"),),
                        transitions=(
                            Transition(Trigger.TICK, "b"),
                            Transition(Trigger.FAILURE, Status.FAILED),
                        ),
                    ),
                    State(
                        "b",
                        tasks=(probe("b"),),
                        transitions=(Transition(Trigger.TICK, Status.SUCCEEDED),),
                    ),
                ),
            )
        ),
        ctx,
    )
    runtime.start()
    assert runtime.tick(0) is Status.RUNNING
    assert runtime.active_path == ("root", "b")
    assert runtime.tick(0) is Status.SUCCEEDED


def test_taskless_path_completes_on_tick_once_and_structural_parent_does_not_complete():
    ctx = Context()
    selections = []

    def selected(ctx):
        selections.append("root")
        return True

    runtime = Runtime(StateTree(State("root", enter_if=selected, children=(State("leaf"),))), ctx)
    assert runtime.start() is Status.RUNNING
    assert selections == ["root"]
    assert runtime.tick(0) is Status.RUNNING
    assert selections == ["root", "root"]
    runtime.stop()
    runtime = Runtime(
        StateTree(
            State(
                "root",
                children=(State("leaf", tasks=(probe("leaf"),)),),
                transitions=(Transition(Trigger.SUCCESS, Status.FAILED),),
            )
        ),
        ctx,
    )
    runtime.start()
    assert runtime.tick(0) is Status.RUNNING
    runtime.stop()


def test_immediate_completion_chain_terminates_and_cleans_up():
    ctx = Context(entered={"instant": Status.SUCCEEDED})
    runtime = Runtime(StateTree(State("root", tasks=(probe("instant"),))), ctx)
    assert runtime.start() is Status.FAILED
    assert runtime.active_path == ()
    assert ctx.trace == ["enter instant", "exit instant"] * 6


def test_partial_entry_failure_halts_entry_and_resolves_failure_transition():
    ctx = Context(entered={"bad": Status.FAILED})
    runtime = Runtime(
        StateTree(
            State(
                "root",
                tasks=(probe("first"), probe("bad"), probe("never")),
                children=(State("child", tasks=(probe("child"),)),),
                transitions=(Transition(Trigger.FAILURE, Status.FAILED),),
            )
        ),
        ctx,
    )
    assert runtime.start() is Status.FAILED
    assert ctx.trace == ["enter first", "enter bad", "exit bad", "exit first"]


@pytest.mark.parametrize("phase", ["factory", "enter", "tick", "condition"])
def test_callback_exception_preserved_and_all_entered_tasks_exit_once(phase):
    ctx = Context(exit_errors={"first", "bad"})

    class Broken(Probe):
        def enter(self, ctx):
            if phase == "enter":
                raise LookupError("original")
            return super().enter(ctx)

        def tick(self, ctx, dt):
            if phase == "tick":
                raise LookupError("original")
            return super().tick(ctx, dt)

    def factory():
        if phase == "factory":
            raise LookupError("original")
        return Broken("bad")

    def condition(ctx):
        raise LookupError("original")

    runtime = Runtime(
        StateTree(
            State(
                "root",
                tasks=(probe("first"), factory),
                transitions=(Transition(Trigger.TICK, Status.SUCCEEDED, condition=condition),),
            )
        ),
        ctx,
    )
    with pytest.raises(LookupError, match="original"):
        runtime.start()
        runtime.tick(0)
    runtime.stop()
    assert runtime.active_path == ()
    assert ctx.trace.count("exit first") == 1
    assert ctx.trace.count("exit bad") == (0 if phase == "factory" else 1)


def test_exit_exception_still_releases_every_task_and_is_not_repeated():
    ctx = Context(exit_errors={"first", "second"})
    runtime = Runtime(StateTree(State("root", tasks=(probe("first"), probe("second")))), ctx)
    runtime.start()
    with pytest.raises(ValueError, match="exit second"):
        runtime.stop()
    assert runtime.status is Status.FAILED
    runtime.stop()
    assert ctx.trace == ["enter first", "enter second", "exit second", "exit first"]


def test_shared_definition_gives_independent_task_instances_and_restart():
    class Counter(Task[list[int]]):
        def __init__(self):
            self.count = 0

        def tick(self, ctx, dt):
            self.count += 1
            ctx.append(self.count)
            return Status.RUNNING

    tree = StateTree(State("root", tasks=(Counter,)))
    a, b = [], []
    first, second = Runtime(tree, a), Runtime(tree, b)
    first.start()
    second.start()
    first.tick(0)
    first.tick(0)
    second.tick(0)
    first.stop()
    first.start()
    first.tick(0)
    first.stop()
    second.stop()
    assert a == [1, 2, 1]
    assert b == [1]


def test_lifecycle_boundaries_terminal_result_and_bad_delta():
    runtime = Runtime(
        StateTree(State("root", transitions=(Transition(Trigger.TICK, Status.SUCCEEDED),))),
        Context(),
    )
    runtime.stop()
    with pytest.raises(RuntimeError, match="started"):
        runtime.tick(0)
    with pytest.raises(RuntimeError, match="running"):
        runtime.send_event("go")
    runtime.start()
    with pytest.raises(RuntimeError, match="already running"):
        runtime.start()
    assert runtime.status is Status.RUNNING
    for dt in (-1, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="dt"):
            runtime.tick(dt)
    assert runtime.tick(0) is Status.SUCCEEDED
    assert runtime.tick(0) is Status.SUCCEEDED
    assert runtime.active_path == ()
    assert runtime.start() is Status.RUNNING
    runtime.stop()


def test_reentrant_execution_fails_and_unwinds_the_callback():
    ctx = Context()

    class Recursive(Probe):
        def tick(self, ctx, dt):
            runtime.stop()
            return Status.RUNNING

    runtime = Runtime(StateTree(State("root", tasks=(partial(Recursive, "recursive"),))), ctx)
    runtime.start()
    with pytest.raises(RuntimeError, match="not reentrant"):
        runtime.tick(0)
    assert runtime.status is Status.FAILED
    assert ctx.trace == ["enter recursive", "exit recursive"]


@pytest.mark.parametrize(
    "root,message",
    [
        (State("root", children=(State("same"), State("same"))), "unique"),
        (State("root", transitions=(Transition(Trigger.TICK, "missing"),)), "Unknown target"),
        (State("root", transitions=(Transition(Trigger.EVENT, "root"),)), "event name"),
        (State("root", transitions=(Transition(Trigger.TICK, Status.RUNNING),)), "terminal"),
    ],
)
def test_rejects_definitions_that_cannot_execute(root, message):
    with pytest.raises(ValueError, match=message):
        StateTree(root)
