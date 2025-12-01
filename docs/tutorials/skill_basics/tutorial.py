# Copyright 2025 Dimensional Inc.
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

# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "marimo>=0.17.0",
#     "pyzmq",
# ]
# ///

import marimo

__generated_with = "0.18.0"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo

    return (mo,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    # Building your first DimOS skill, part 1

    In this tutorial, we'll build a simple skill that allows your robot to make greetings.

    We'll assume that you've skimmed the Quickstart and installed DimOS, but we won't require the simulator-related packages.

    **TODO**: Add link to installation instructions, or if they end up making a simple install script, just have a cell with that
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Setup
    """)
    return


@app.cell
def _():
    import inspect
    import time

    from dimos.core.blueprints import autoconnect
    from docs.tutorials.skill_basics.greeter import Greeter, RobotCapabilities

    return Greeter, RobotCapabilities, autoconnect, inspect, time


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 1: Define the skill (and its containing module)

    Before jumping into how to define skills, let's first establish some background: What even are skills; how do they enter the picture?

    On the DimOS framework, programming a robot involves composing [*modules*](../../concepts/modules.md). These might be modules that endow the robot with some sort of skill or capability -- e.g. perception or navigation capabilities -- or 'agentic' modules that orchestrate the use of certain capabilities.

    At a high level, then, skills are capabilities for your robot. But at a more prosaic level, they are just methods on a `Module` that have been wrapped with a special decorator, the `@skill` decorator. (As we'll see in the next tutorial, this allows them to be invoked by LLM agents as tool calls.)

    <!-- Citation: dimos/core/module.py:77 - ModuleBase inherits from SkillContainer -->
    <!-- Citation: dimos/core/module.py:85 - rpc_calls list declaration -->

    So, to define a skill for greeting people, we need to define a module that'll house the greeting skill method, as well as the method itself:
    """)
    return


@app.cell(hide_code=True)
def _(Greeter, inspect, mo):
    mo.ui.code_editor(inspect.getsource(Greeter), language="python", disabled=True)
    return


@app.cell
def _(mo):
    mo.md("""
    There are two things to explain here: (i) the declaration of the module's dependencies in `rpc_calls` and (ii) the `@skill` decorator.

    ### Dependency injection

    Notice how `Greeter` declares its dependencies in the `rpc_calls` list:

    - `rpc_calls` declares what methods this module needs from other modules
    - while `get_rpc_calls()` retrieves the actual method references at runtime

    This is *dependency injection*: `Greeter` doesn't import `RobotCapabilities` directly.
    Instead, the dependencies are supplied at runtime, when the modules are wired up with `autoconnect` (more on this later).
    <!-- Citation: dimos/core/module.py:268-275 - get_rpc_calls() retrieves from _bound_rpc_calls -->
    <!-- Citation: dimos/protocol/skill/skill.py:65-113 - @skill decorator implementation -->

    ### `@skill`

    The `@skill()` decorator transforms a method into an agent-callable tool — generating a JSON schema from your signature, tracking execution state, and running in background threads.

    For simple skills, `@skill()` with no arguments works fine (as in `Greeter.greet`). For streaming or background data, you'd use parameters like `stream` and `reducer` — see the [Skills concept guide](../../concepts/skills.md) and the `@skill`-related docstrings for details.

    /// tip
    Your docstring becomes the tool description LLMs see — write it for an LLM audience.
    ///
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### RobotCapabilities

    We assumed that there was a `RobotCapabilities` module that encapsulated the lower-level robot capabilities that `Greeter` builds upon. This is basically a mock robot that logs when its methods are called. The main thing to note about it is the `@rpc` decorator -- more on this shortly.
    """)
    return


@app.cell(hide_code=True)
def _(RobotCapabilities, inspect, mo):
    mo.ui.code_editor(inspect.getsource(RobotCapabilities), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    #### Why does `RobotCapabilities` use `@rpc` for `speak`?

    Whether to use `@rpc` or `@skill` depends on what you want the method to be used for.

    - `@rpc` methods are for module-to-module communication via `get_rpc_calls()`
    - `@skill` methods are for invocation by agents; they come with additional infrastructure that we'll see shortly.

    That is, we're using `@rpc` for `speak` since we aren't trying to expose it to agents -- since it's more of a lower-level capability that's used by the higher-level skills.

    <!-- TODO: If we end up going with a notebook foramt,
    explain why we needed to import from a separate file, instead of just defining it in the notebook, since it relates to Dask x multiprocessing
    -->

    <!-- **[TODO: Add skillspy stuff -- e.g. a skillspy widget next to UI for skills invocation in a notebook]** -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 3: Combine Module blueprints with `autoconnect`

    Now that we've defined the constituent modules of our system, now that we have defined blueprints for each of them, we can reduce them down to a combined blueprint with `autoconnect`.

    (We can also optionally override the default global configuration, as is done here with `n_dask_workers`.)

    <!-- Citation: dimos/core/blueprints.py:180-217 - autoconnect() -->
    <!-- YM: Autoconnect is basically a mconcat -->
    """)
    return


@app.cell
def _(Greeter, RobotCapabilities, autoconnect):
    blueprint_set = autoconnect(
        RobotCapabilities.blueprint(),  # Provides speak
        Greeter.blueprint(),  # Requires RobotCapabilities.speak
    ).global_config(n_dask_workers=1)
    return (blueprint_set,)


@app.cell
def _(mo):
    mo.md(r"""
    ## Step 4: Build and run the blueprint

    And then we can build and run the combined blueprint.
    """)
    return


@app.cell
def _(blueprint_set):
    dimos = blueprint_set.build()
    return (dimos,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    The `build` method wires up the modules, deploys them to worker(s), and starts them;
    it returns a `ModuleCoordinator` instance that manages the deployed modules.

    <!-- TODO: Add link to API reference for ModuleCoordinator -->

    ### Dependency injection, redux

    It's worth pausing to reflect on the wiring up of modules.

    Recall that `Greeter` had declared it needs certain dependencies.
    When we build the blueprint, the blueprint system

    - checks what dependencies the various modules require; e.g., that `Greeter` needs `RobotCapabilities.speak`
    - and wires up the modules so these dependencies are supplied.
    <!-- Citation: dimos/core/blueprints.py - _connect_rpc_methods -->
    <!-- Citation: dimos/core/blueprints.py:203 - instance.set_rpc_method() binds RPC calls -->

    This, in other words, is the runtime dependency injection we had alluded to.

    ### Use the `loop` method for long-running applications

    After `build()`, the system is already running. For long-running applications (e.g. an honest-to-goodness robot),
    use the `loop()` method to keep the process alive:

    ```python
    dimos.loop()
    ```

    This sleeps indefinitely until interrupted (Ctrl+C / SIGINT), whereupon it calls `stop()` to shut down gracefully.

    We won't need to do that in this tutorial, though -- we'll just call `stop()` at the end.
    """)
    return


@app.cell
def _(mo):
    mo.md("""
    ## Step 4: Try calling the skills

    Now that we have our running system, let's invoke our greeting skill!

    /// note
    In the following, we'll peer beneath the hood and use lower-level APIs that you'd typically only use when testing or debugging.
    ///

    First, we'll get our greeter module instance:
    """)
    return


@app.cell
def _(Greeter, dimos):
    greeter = dimos.get_instance(Greeter)
    print(f"✅ Got greeter instance: {greeter}")
    return (greeter,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    Then we'll setup the `SkillCoordinator`. This is a lower-level API that you don't typically need to use; we're just using it to give you more intuition for what's happening under the hood.
    """)
    return


@app.cell
def _(greeter):
    from dimos.protocol.skill.coordinator import SkillCoordinator

    skill_coordinator = SkillCoordinator()
    skill_coordinator.start()

    # Register our greeter's skills with the coordinator
    skill_coordinator.register_skills(greeter)
    print("📋 SkillCoordinator ready with greeter's skills")
    return (skill_coordinator,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    At this point, you might wonder, why not just call `greeter.greet()` directly?

    Answer: because
    * (i) we want to invoke skills the way that LLM agents would, as preparation for the next tutorial
    * and (ii) LLM agents don't call Python methods; instead, they make *tool calls* that get routed through the SkillCoordinator.

    The SkillCoordinator
    * executes skills
    * monitor skills; for instance tracking when skills start, stream updates, complete, or error
    * and handles communication between agents and skills.

    By using `skill_coordinator.call_skill()` here, we're following the pattern an LLM agent will use in part 2.
    <!-- TODO: Add link to part 2 -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Let's invoke some skills!

    Run the cell below to see your robot greet different people:
    """)
    return


@app.cell
def _(skill_coordinator):
    # Call with no arguments (uses default "friend")
    skill_coordinator.call_skill(
        call_id="greeting-1",  # Unique ID for this specific invocation
        skill_name="greet",
        args={},  # Empty args → uses default
    )

    # Call with a specific name
    skill_coordinator.call_skill(
        call_id="greeting-2",
        skill_name="greet",
        args={"args": {"name": "Alice"}},  # Pass name as keyword argument
    )

    print("Skills invoked!")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Monitoring skill execution

    The SkillCoordinator tracks the state of every skill invocation. Let's check what happened:
    """)
    return


@app.cell
def _(skill_coordinator, time):
    from dimos.utils.cli.skillspy.skillspy import format_duration

    # Wait a moment for skills to complete
    time.sleep(0.3)

    # Generate a snapshot of all skill states
    snapshot = skill_coordinator.generate_snapshot(clear=False)

    print("Skill Execution Summary:")
    print("-" * 40)
    for call_id, state in snapshot.items():
        print(f"• {call_id}: {state.name} → {state.state.name}")
        print(f"  Duration: {format_duration(state.duration())}s")
        print(f"  Messages: {state.msg_count}")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    Notice the output above:
    - `[Skill] Greeter.greet executing` indicates the greeting skill started
    - `[Skill] RobotCapabilities.speak called` shows the robot "speaking"
    - Each invocation has a unique `call_id` for tracking
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    Congratulations! You've just built and deployed your first DimOS skill.

    ### Key takeaways

    Let's recap:

    **Blueprints allow you to declaratively specify and combine modules**, with runtime dependency injection.

    **Skills are methods with superpowers** — The `@skill` decorator transforms regular methods into agent-callable tools with built-in execution tracking.

    **Skill invocations are tracked** — the `call_id` lets you monitor multiple concurrent executions.

    ### What's next?

    In part 2 of this tutorial, you'll see how LLM agents use this exact same pattern to invoke skills as *tool calls*. The agent will decide when to greet, who to greet, and orchestrate complex behaviors by combining multiple skills!
    """)
    return


@app.cell
def _(dimos, skill_coordinator):
    # Gracefully shut down / release resources
    skill_coordinator.stop()
    dimos.stop()
    return


if __name__ == "__main__":
    app.run()
