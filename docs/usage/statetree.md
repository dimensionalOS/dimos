# StateTree

A StateTree gives a [module](/docs/usage/modules.md) hierarchical behavior logic. The module defines states, tasks, and transitions. A component ticks the tree on the module loop and hands every task the module as its context.

- `State`, `Transition`, `Task`, and `StateTree` define a tree. `Runtime` executes one. Both live in `dimos/core/statetree/runtime.py`.
- `StateTreeComponent` in `dimos/core/statetree/component.py` runs a tree for a module.

## A module with a StateTree

Enter a `StateTreeComponent` inside `main()`. It starts the tree with the module and stops it with the module.

```python
from collections.abc import AsyncIterator
import time

from dimos.core.module import Module
from dimos.core.statetree.component import StateTreeComponent
from dimos.core.statetree.runtime import State, StateTree, Status, Task, Transition, Trigger


class Count(Task["Counter"]):
    def tick(self, ctx: "Counter", dt: float) -> Status:
        ctx.count += 1
        return Status.SUCCEEDED if ctx.count == 3 else Status.RUNNING


def counting_tree() -> StateTree["Counter"]:
    return StateTree(
        State(
            "counting",
            tasks=(Count,),
            transitions=(Transition(Trigger.SUCCESS, Status.SUCCEEDED),),
        )
    )


class Counter(Module):
    count = 0

    async def main(self) -> AsyncIterator[None]:
        async with StateTreeComponent(
            counting_tree(), self, tick_rate=100.0, on_status_changed=print
        ) as self.behavior:
            yield


counter = Counter()
counter.start()
time.sleep(0.2)
counter.stop()
print(counter.count)
```

```results
Status.RUNNING
Status.SUCCEEDED
None
3
```

`StateTreeComponent(tree, context, tick_rate=10.0, on_status_changed=None)` runs everything on the loop that started it. Tasks, predicates, async `@rpc` methods, and `handle_*` stream handlers therefore share module state without locks.

- `start()` selects the initial states and begins ticking. It restarts a tree that has finished. `stop()` exits every active task. `async with` calls both.
- `send_event(name)` queues an event for the next tick. It is the one method that is safe from any thread. An event sent while the tree is not running is dropped.
- `status` is `Status.RUNNING`, `Status.SUCCEEDED`, `Status.FAILED`, or `None` when stopped. `active_path` lists the active state names from root to leaf.
- `on_status_changed` is called on the loop with each new status.
- Ticking ends when the tree succeeds or fails. A task that raises fails the tree, and the exception is logged.

Stream input reaches the tree through the module. A handler stores the message on the module and sends an event. Tasks publish on the module's `Out` streams.

[`dimos/core/statetree/example/module.py`](/dimos/core/statetree/example/module.py#L125) is a complete module. It waits for dispatch, drives a route by publishing `cmd_vel`, pauses when an obstacle is reported, and resumes. Run it in a worker process:

```bash
uv run python -m dimos.core.statetree.example.demo_statetree
```

## Defining a tree

`StateTree(root)` validates a tree of immutable `State` definitions. Each state has a unique name, ordered children, task factories, transitions, an optional `enter_if(context)` predicate, and a `Completion` policy. One definition can back many runtimes.

`Task` provides `enter(context)`, `tick(context, dt)`, and `exit(context)`. The first two return a `Status`, and their defaults keep running. Each entry in `tasks` is a zero-argument factory, usually the task class. The runtime calls it on every activation, so task fields hold per-activation data. Shared services and observations live on the context.

`Transition(trigger, target, condition=None, event=None)` targets a state name, `Status.SUCCEEDED`, or `Status.FAILED`. The triggers are `TICK`, `EVENT`, `SUCCESS`, `FAILURE`, and `COMPLETED`. Only `EVENT` takes an event name, and it requires one. A condition is a synchronous context predicate.

## Selection and task lifecycle

Selection checks entry predicates and tries children in declaration order. A branch whose children cannot select a leaf fails selection, and the next sibling is tried. A failed initial selection returns `FAILED` without entering tasks.

Tasks enter from root to leaf in declaration order and tick in the same order. A failed task stops later entry or tick callbacks in that operation. Completed tasks keep their result and no longer tick. Exits run in reverse order, including for a task whose entry failed or raised.

`Completion.ANY` is the default. One successful task completes a state, and failure wins over success. `Completion.ALL` waits until every task in the state succeeds. States without tasks do not complete on their own. An active path with no tasks at all completes its leaf on a tick. If several states complete, the rootmost wins and interrupts its descendants.

## Transitions and events

Each tick snapshots queued events before task callbacks. After the task ticks, event and tick transitions are searched from leaf to root in declaration order. The first eligible transition with a selectable target wins. At most one event or tick transition runs per tick. The snapshot is consumed even when nothing matches. Events sent by callbacks wait for the next tick. Events carry a name and no payload.

Completion is resolved after event and tick transitions. Completion transitions are searched from the completed state upward. Without an eligible transition the runtime selects from the root again. New tasks that complete on entry can cause immediate transitions. After five completion transitions in one tick, another immediate completion fails the tree.

A transition between siblings keeps their common running ancestors. A self or ancestor target reenters that target. Retained ancestors do not recheck entry predicates, so `enter_if` is a selection condition and not a continuous guard. Target selection finishes before old tasks exit.

## Using the runtime directly

`Runtime(tree, context)` is the caller-driven core that the component wraps. It has no clock, thread, or transport, which makes it the right surface for deterministic tests. All access must come from one thread.

```python
from dataclasses import dataclass

from dimos.core.statetree.runtime import Runtime, State, StateTree, Status, Task, Transition, Trigger


@dataclass
class Journey:
    distance: float = 0.0


class Drive(Task[Journey]):
    def tick(self, ctx: Journey, dt: float) -> Status:
        ctx.distance += dt
        return Status.SUCCEEDED if ctx.distance >= 2.0 else Status.RUNNING


tree = StateTree(
    State(
        "journey",
        children=(
            State(
                "idle",
                tasks=(Task,),
                transitions=(Transition(Trigger.EVENT, "drive", event="dispatch"),),
            ),
            State(
                "drive",
                tasks=(Drive,),
                transitions=(Transition(Trigger.SUCCESS, Status.SUCCEEDED),),
            ),
        ),
    )
)

journey = Journey()
runtime = Runtime(tree, journey)
runtime.start()
print(runtime.active_path)
runtime.send_event("dispatch")
runtime.tick(0.0)
print(runtime.active_path)
print(runtime.tick(2.0), journey.distance)
```

```results
('journey', 'idle')
('journey', 'drive')
Status.SUCCEEDED 2.0
```

`start()` rejects a running runtime. After `stop()` or a terminal result it selects again with fresh tasks and no events. A tick after a terminal result returns the stored result. A tick before start raises `RuntimeError`. Deltas must be finite and nonnegative. Callbacks may send events but may not start, tick, or stop the runtime. A callback exception marks the runtime failed, exits every entered task once, and propagates.
