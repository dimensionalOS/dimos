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
#     "python-dotenv",
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
    # Building your first DimOS skill, part 2: Adding an LLM agent

    In [part 1](../skill_basics/tutorial.py), you built a greeter skill and invoked it manually via `SkillCoordinator.call_skill()`.
    This gave you a foundation for understanding how skills work under the hood.

    In part 2, you'll wire up your greeter to an **LLM agent**.
    With this, you can command the agent to call the greeter skill -- to greet -- by simply asking it to "say hello to Alice".

    <!-- Citation: dimos/agents2/agent.py:163-213 - Agent class with LLM integration, SkillCoordinator -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Prerequisites

    - Ideally you'd have at least skimmed [the previous tutorial](../skill_basics/tutorial.py); but the main gist should still be understandable even if not
    - OpenAI API key set in your environment (`OPENAI_API_KEY`)
    - Same Python environment as part 1

    /// tip | API key setup
    Make sure there's a `.env` file in your project root with:
    ```
    OPENAI_API_KEY=your-key-here
    ```
    ///
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

    from dotenv import load_dotenv

    load_dotenv()  # Load API keys from .env file

    from dimos.agents2.agent import LlmAgent, llm_agent
    from dimos.core.blueprints import autoconnect
    from docs.tutorials.skill_with_agent.greeter import (
        Greeter,
        GreeterForAgents,
        RobotCapabilities,
    )

    return (
        GreeterForAgents,
        LlmAgent,
        RobotCapabilities,
        autoconnect,
        inspect,
        llm_agent,
    )


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 1: Enable agent auto-registration

    When you compose a skill module with `llm_agent()`, the framework needs to discover which skills to expose.
    In part 1, we did this manually by calling `skill_coordinator.register_skills(greeter)`.
    For agent composition, modules declare themselves as skill providers through a hook method.

    ### The auto-registration hook

    Here's our `GreeterForAgents`, which extends `Greeter` from part 1 with one additional method:
    """)
    return


@app.cell
def _(GreeterForAgents, inspect, mo):
    mo.ui.code_editor(inspect.getsource(GreeterForAgents), language="python")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    The method name `set_LlmAgent_register_skills` follows a naming convention that DimOS uses to discover skill providers.
    When you call `.build()` on a blueprint containing both this module and `llm_agent()`, this hook is called to register your skills.

    /// note | The naming convention for `set_`-prefixed methods
    Suppose you have a method like `set_Mod_some_method` on module `A`.
    The blueprint system will try looking for a Module in the combined blueprint named `Mod` with a method named `some_method`.
    If it finds such a Module and method, the blueprint system
    will call the original method with the matched method; i.e.,
    in this case, it will call `<instance of A>.set_Mod_some_method(<instance of Mod>.some_method)`.
    ///

    The method body is boilerplate. It does basically the same thing we did with `skill_coordinator.register_skills(greeter)` in the previous tutorial; i.e., it wires up the registration callback.

    <!-- Citation: dimos/core/blueprints.py:396-405 - Convention: methods starting with set_ are called with matching RPC references -->

    ### The SkillModule shortcut

    Since this hook is so common, there's a convenience class that adds it for you:

    ```python
    from dimos.core.skill_module import SkillModule

    class MySkills(SkillModule):
        @skill()
        def do_something(self) -> str:
            ...
    ```

    `SkillModule` is just `Module` plus the `set_LlmAgent_register_skills` method shown above.


    You might want to use the explicit pattern

    * when extending an existing class (like we're doing here)
    * when you have more than one module that subclasses `LlmAgent` (e.g. in a multi-agent setup)
    * or when you need custom serialization

    but otherwise, you can just subclass `SkillModule`.

    <!-- Citation: dimos/core/skill_module.py:20-26 - SkillModule adds set_LlmAgent_register_skills -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 2: Compose with llm_agent

    Now we'll build a blueprint that wires together:

    - `RobotCapabilities` - the low-level mock robot
    - `GreeterForAgents` - our skill module with auto-registration
    - `llm_agent()` - creates an agent that can reason and call skills

    <!-- Citation: dimos/agents2/agent.py:591-673 - LlmAgent automatically starts processing loop on startup -->
    """)
    return


@app.cell
def _(GreeterForAgents, RobotCapabilities, autoconnect, llm_agent):
    # Combine the blueprints
    blueprint = autoconnect(
        RobotCapabilities.blueprint(),  # Low-level capabilities
        GreeterForAgents.blueprint(),  # Our skill (now agent-enabled)
        llm_agent(
            system_prompt="You are a friendly robot that can greet people. Use the greet skill when asked to say hello to someone."
        ),
    ).global_config(n_dask_workers=1)

    # Build the combined blueprint
    dimos = blueprint.build()
    print("System built and running!")
    return (dimos,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### What just happened?

    - `autoconnect()` combined the individual blueprints
    - `.build()` wired everything together:
      - Deployed modules to workers
      - Called `set_LlmAgent_register_skills` on `GreeterForAgents`, registering its skills with the agent
      - Started the agent's processing loop

    (See [the Blueprints concept](../../concepts/blueprints.md) for more on the blueprint system.)
    <!-- Citation: dimos/agents2/agent.py:680 - llm_agent = LlmAgent.blueprint -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 3: Interact with the agent

    ### In the notebook (interactive)

    For interactive exploration, we can get the agent instance and call `query()` directly.

    This is the same pattern as part 1, except that now we aren't the ones calling the skill; instead, it is *the agent* that decides which skill to invoke.

    <!-- Citation: dimos/agents2/agent.py:559-563 - Agent.query() runs agent_loop() -->
    """)
    return


@app.cell
def _(LlmAgent, dimos):
    agent = dimos.get_instance(LlmAgent)
    print(f"Got agent instance: {agent}")
    return (agent,)


@app.cell
def _(agent):
    # Ask the agent to greet someone
    response = agent.query("Say hello to Bob")
    print(response)
    return


@app.cell
def _(agent):
    # Try another greeting
    response2 = agent.query("Can you greet Alice?")
    print(response2)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    Notice that you didn't have to specify which skill to call or how to call it.
    The agent:

    1. Received your natural language request
    2. Looked at its available tools (including `greet`)
    3. Decided to call `greet` with the appropriate name
    4. Executed the skill and returned the result

    <!-- Citation: dimos/agents2/agent.py:329-341 - execute_tool_calls routes to coordinator.call_skill -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### An alternative terminal-based workflow

    A notebook is helpful for prototyping and experimenting,
    but often you would want to do things in the terminal, instead of a notebook.

    DimOS provides TUI helpers to facilitate this workflow: you can run the agent as a standalone script and interact with it via TUIs like `dimos.agents2.cli.human.humancli`.

    **In a terminal pane: Run the agent system**
    ```bash
    python docs/tutorials/skill_with_agent/cli.py
    ```

    **In another terminal pane: Send messages to the agent**
    ```bash
    python -m dimos.agents2.cli.human.humancli
    ```
    This opens up a TUI that you can use to send messages.

    **Optionally, in yet another terminal pane: Use `agentspy` to monitor agent activity**
    ```bash
    python -m dimos.utils.cli.agentspy.agentspy
    ```

    This opens a TUI that shows all messages flowing through the agent in real-time:

    - **Human** (green): Messages you send
    - **Agent** (yellow): LLM responses and tool calls
    - **Tool** (red): Skill execution results
    - **System** (red): System prompts

    Useful for debugging or improving your understanding of what's happening under the hood.
    Press `q` to quit, `c` to clear.

    <!-- Citation: dimos/utils/cli/agentspy/agentspy.py:102-115 - Message type color coding -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## What's happening, under the hood?

    Let's trace the flow from your message to the skill execution:

    ```
    User input ("Say hello to Bob")
           |
           v
       LlmAgent
           |-- discovers tools from registered skills
           |-- LLM decides: call greet(name="Bob")
           v
     SkillCoordinator
           |
           v
     GreeterForAgents.greet("Bob")
           |
           v
     RobotCapabilities.speak("Hello, Bob!")
           |
           v
       Result flows back to agent
           |
           v
       Agent responds to user
    ```

    <!-- Citation: dimos/agents2/agent.py:577-578 - get_tools() returns coordinator.get_tools() -->
    <!-- Citation: dimos/protocol/skill/coordinator.py:639-695 - SkillCoordinator.call_skill() creates SkillState, invokes skill -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### The key stages

    1. **Skill discovery**: When the system starts, `llm_agent` finds modules with `set_LlmAgent_register_skills` and calls that method.
       Your `GreeterForAgents` registers its `greet` skill.

    2. **Tool schema generation**: The agent converts `@skill` methods into tool schemas that the LLM understands.


       /// tip | Write descriptive docstrings!
       Your skill's docstring becomes the tool description.


       That's why you want the docstrings to be descriptive (and optimized for LLMs), if you are using an LLM agent.
       ///


    4. **Agent reasoning**: When you type "say hello to Bob", the agent considers available tools and decides to call `greet` with `name="Bob"`.

    5. **Skill execution**: The agent's tool call goes through `SkillCoordinator`, which invokes your skill method and returns the result.

    6. **Response**: The agent receives the skill result and formulates a natural language response.

    <!-- Citation: dimos/protocol/skill/schema.py:63-103 - function_to_schema extracts docstring -->
    <!-- Citation: dimos/protocol/skill/skill.py:262 - @skill decorator calls function_to_schema -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Hands-on exploration

    Want to dig deeper? Try these:

    - Add `print()` statements in `GreeterForAgents.greet()` to see when it's called
    - Inspect the skill registry: `dimos.get_instance(LlmAgent).get_tools()`
    - Run with `DIMOS_LOG_LEVEL=DEBUG` to see the full message flow
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Key takeaways

    1. **Auto-registration via convention**: The `set_LlmAgent_register_skills` method lets the framework wire your skills to agents automatically.

    2. **Natural language to tool calls**: Agents convert natural language requests into skill invocations, using your docstrings to understand what each skill does.

    3. **Composition with the blueprint system**: Combine skill modules, `llm_agent()`, and `human_input()` declaratively; DimOS handles the wiring.
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## What's next

    - **Add more skills**: Try adding a `farewell` skill and see how the agent uses both.
    - **Stream progress**: For long-running skills, use `stream=Stream.call_agent` to send updates to the agent as work progresses.
    - **Explore real robot skills**: Check out `dimos/agents2/skills/navigation.py` for examples of navigation skills.
    - **Multi-agent systems**: See the [multi-agent tutorial](../multi_agent/tutorial.md).

    ## See also

    - [Agents concept guide](../../concepts/agent.md)
    - [Blueprints concept guide](../../concepts/blueprints.md)
    """)
    return


@app.cell
def _(dimos):
    # Gracefully shut down / release resources
    dimos.stop()
    return


if __name__ == "__main__":
    app.run()
