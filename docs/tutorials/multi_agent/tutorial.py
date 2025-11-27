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
    # Multi-agent tutorial: Build yourself a RoboButler

    In this tutorial, we'll build a RoboButler multi-agent system (for one robot)
    consisting of a Planner agent and specialist sub-agents.
    To keep things simple, we'll have just two sub-agents: one for giving advice on socio-emotional matters,
    and another for managing schedules.
    The Planner coordinates and consults with the specialists,
    before taking the appropriate actions.
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.mermaid("""
    flowchart LR
        User --> PlannerAgent
        PlannerAgent --> WB[WellbeingAgent<br/>mood/context reasoning]
        PlannerAgent --> SM[ScheduleManagementAgent<br/>calendar reasoning]
        PlannerAgent --> RC[RobotCapabilities<br/>speak, approach user]
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    We'll start with *mock* agents, then swap in real LLMs; most of the tutorial notebook can therefore be followed and run without any API keys.

    /// tip | If you're trying to run this and you're new to Marimo:
    This tutorial is a [Marimo notebook](https://docs.marimo.io/). See the [Marimo quickstart](https://docs.marimo.io/getting_started/index.html) to get started.
    ///
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Prerequisites

    - Ideally some familiarity with DimOS skills and single-agent systems (see the [skill tutorials](../skill_basics/tutorial.md)); but the tutorial should be broadly understandable even if not
    - OpenAI API key for the real LLM section
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

    load_dotenv()

    from dimos.agents2.agent import LlmAgent
    from dimos.agents2.testing import MockModel
    from dimos.core.blueprints import autoconnect
    from docs.tutorials.multi_agent.planner_subagents import (
        DelegationSkills,
        PlannerAgent,
        RobotCapabilities,
        ScheduleManagementAgent,
        WellbeingAgent,
        get_from_to,
    )

    return (
        DelegationSkills,
        MockModel,
        PlannerAgent,
        RobotCapabilities,
        ScheduleManagementAgent,
        WellbeingAgent,
        autoconnect,
        get_from_to,
        inspect,
    )


@app.cell
def _():
    from dimos.utils.cli.agentspy.agentspy import (
        AgentMessageMonitor,
        format_message_content,
        format_timestamp,
        get_message_type_and_style,
    )

    # Set up a monitor for agent messages -- more on this later
    message_monitor = AgentMessageMonitor()
    message_monitor.start()
    return format_message_content, format_timestamp, message_monitor


@app.cell
def _(format_message_content, format_timestamp, get_from_to, mo):
    def truncate(s: str, limit: int = 100) -> str:
        return s[:limit] + ("..." if len(s) > limit else "")

    def render_spy_accordion(messages, title="Agentspy"):
        """Render agent messages as a collapsible accordion."""
        if not messages:
            return mo.accordion({title: mo.md("*No messages captured*")})

        def entry_to_row(entry):
            from_agent, to_agent = get_from_to(entry.message)
            return {
                "Time": format_timestamp(entry.timestamp),
                "From": from_agent,
                "To": to_agent,
                "Content": truncate(format_message_content(entry.message)),
            }

        rows = list(map(entry_to_row, messages))
        table = mo.ui.table(rows, label=f"{len(messages)} messages")
        return mo.accordion({title: table})

    return (render_spy_accordion,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 1: Make the Agent `Module`s

    As before, we need to start by defining the `Modules` of our system.
    To keep things simple, let's start with just the Agent modules; agents, recall, are basically `Module`s that can communicate via RPC.

    How exactly to do this, however, isn't immediately obvious. To see why, consider how communicating via RPC implies that we'll need to be able to call the `query` method of each of these agents. But if we're combining multiple `LlmAgent` modules in a blueprint, how is the blueprint system going to know which agent should receive a call to `LlmAgent.query`?

    /// tip
    If the details of the RPC mechanism are foggy, it might be worth looking at [the first tutorial](../skill_basics/tutorial.md) again.
    ///

    The solution, fortunately, isn't difficult: just define concrete subclasses for the agents.
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    /// note
    Strictly speaking,  our agents subclass `AgentWithFromToMetadata` instead of subclassing `LlmAgent` directly.
    `AgentWithFromToMetadata` just adds some metadata for `agentspy` -- this is a difference you can ignore.
    ///
    """)


@app.cell(hide_code=True)
def _(PlannerAgent, inspect, mo):
    mo.ui.code_editor(inspect.getsource(PlannerAgent), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(WellbeingAgent, inspect, mo):
    mo.ui.code_editor(inspect.getsource(WellbeingAgent), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(ScheduleManagementAgent, inspect, mo):
    mo.ui.code_editor(inspect.getsource(ScheduleManagementAgent), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    Now RPC calls like `WellbeingAgent.query` are unambiguous.
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.mermaid("""
    flowchart TD
        User -->|talks to| PA[PlannerAgent<br/>coordinator]
        PA -->|delegates via RPC| WB[WellbeingAgent<br/>mood/context]
        PA -->|delegates via RPC| SM[ScheduleManagementAgent<br/>calendar/timing]

        subgraph Subagents
            WB
            SM
        end
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 2: Define the low-level `RobotCapabilities`

    These are the physical actions the robot can perform. The skills will be registered on the `PlannerAgent`.
    """)
    return


@app.cell(hide_code=True)
def _(RobotCapabilities, inspect, mo):
    mo.ui.code_editor(inspect.getsource(RobotCapabilities), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 3: Combine the `Module` blueprints and start with mock agents

    We'll first wire up the system with **mock agents** that return canned responses.
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Define mock responses

    To do this, we'll instantiate `MockModel` for each agent `Module` with some predefined responses; these responses will be cycled through later, when we call the agent(s).

    /// note
    Don't worry about how `MockModel` works -- the details aren't important for our purposes.
    ///

    You *don't* need to read the following code closely. Just note that the mock `PlannerAgent` returns a (canned) sequence of tool calls: it calls on the various subagents for advice, before taking certain actions.
    """)
    return


@app.cell
def _(MockModel, PlannerAgent, ScheduleManagementAgent, WellbeingAgent):
    from langchain_core.messages import AIMessage

    # Subagent mocks: return brief analysis strings (in Stevens' understated style)
    # Note: LlmAgent auto-starts its loop on build(), consuming one response.
    # The first response in each list handles this auto-loop invocation.
    mock_wellbeing = WellbeingAgent.blueprint(
        model_instance=MockModel(
            responses=[
                "Awaiting instructions.",  # consumed by auto-loop on startup
                "One notes a certain weariness. The weather may be a factor. Measured comfort advised.",
                "There appears to be some improvement in disposition.",
            ]
        )
    )

    mock_schedule_management = ScheduleManagementAgent.blueprint(
        model_instance=MockModel(
            responses=[
                "Awaiting instructions.",  # consumed by auto-loop on startup
                "Dental appointment at two o'clock. Departure by 1:35 prudent. Insurance card and umbrella required.",
                "No pressing engagements. An opportune moment for repose.",
            ]
        )
    )

    # Planner mock: sequence of tool calls showing the delegation flow (Stevens personality)
    # Note: The first response is consumed by LlmAgent's auto-loop on startup.
    mock_planner = PlannerAgent.blueprint(
        model_instance=MockModel(
            responses=[
                "Awaiting instructions.",  # consumed by auto-loop on startup
                # 1. Delegate to wellbeing specialist
                AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "id": "call_1",
                            "name": "consult_wellbeing_specialist",
                            "args": {"situation": "Individual not feeling entirely themselves"},
                        }
                    ],
                ),
                # 2. Delegate to schedule specialist
                AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "id": "call_2",
                            "name": "consult_schedule_specialist",
                            "args": {"question": "Today's engagements?"},
                        }
                    ],
                ),
                # 3. Act: offer comfort via speak (in Stevens' restrained manner)
                AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "id": "call_3",
                            "name": "speak",
                            "args": {"text": "If I may: matters are well in hand."},
                        }
                    ],
                ),
                # 4. Act: send departure reminder via speak
                AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "id": "call_4",
                            "name": "speak",
                            "args": {
                                "text": "A gentle reminder: departure by 1:35 PM would be prudent for the dental appointment."
                            },
                        }
                    ],
                ),
                # 5. Final synthesized response (in Stevens' formal style)
                "If I may: departure by 1:35 for the dental appointment. I have noted the insurance card and umbrella. One trusts this is satisfactory.",
            ]
        )
    )
    return mock_planner, mock_schedule_management, mock_wellbeing


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    Let's recap: We've defined the constituent `Modules` of our multi-agent system, and prepped them with mocks. But we haven't yet done anything to make it possible for `PlannerAgent` to consult the sub-agents.

    It's worth pausing a moment to ask yourself: how can we do this?
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 4: Make it possible for `PlannerAgent` to consult the sub-agents


    Recall that the LLM in an `Agent` 'acts' via tool calls. So, if we want `PlannerAgent` to be able to call methods via RPC on the sub-agents, we need to give it a way to do so via tool calls. And the way to do that, as we've seen in [the previous tutorials](../skill_basics/tutorial.md), is with *skills*.

    In particular, we'll equip PlannerAgent with `@skill` methods that wrap the RPC calls.

    /// note | Reminder: `@rpc`-decorated methods aren't automatically exposed as tools.
    That's what skills are for.
    ///
    """)
    return


@app.cell(hide_code=True)
def _(DelegationSkills, inspect, mo):
    mo.ui.code_editor(inspect.getsource(DelegationSkills), language="python", disabled=True)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### What the delegation flow looks like
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ```
    User: "Argh i have meetings all day"
        │
        ▼
    ┌─────────────────────────────────────────┐
    │ PlannerAgent                            │
    │ "Let me understand the situation..."    │
    └─────────────────────────────────────────┘
        │
        │ LLM decides to call skill
        ▼
    ┌─────────────────────────────────────────┐
    │ consult_emotional_specialist            │
    │ (internally calls RPC)                  │
    └─────────────────────────────────────────┘
        │
        ▼
    ┌─────────────────────────────────────────┐
    │ EmotionalIntelligenceAgent              │
    │ "User appears overwhelmed.              │
    │  Gloomy weather..."                     │
    └─────────────────────────────────────────┘
        │
        │ returns analysis
        ▼
    ┌─────────────────────────────────────────┐
    │ PlannerAgent                            │
    │ "I'll offer some emotional support      │
    │  and check your schedule..."            │
    └─────────────────────────────────────────┘
    ```
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    /// note | Why `ret=Return.call_agent`?
    Two reasons:
    1. `ret=Return.call_agent` notifies the agent when the skill completes
    2. It keeps the planner's agent loop alive.  If no running skills had this setting, the planner's
    agent loop would exit after the planner makes the tool calls -- the planner wouldn't see the subagents' responses.
    ///

    <!-- Citation: dimos/protocol/skill/type.py:80-88 - Return enum defining active vs passive -->
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## Step 5: Combine the blueprints and build
    """)
    return


@app.cell
def _(
    DelegationSkills,
    RobotCapabilities,
    autoconnect,
    mock_planner,
    mock_schedule_management,
    mock_wellbeing,
):
    # Build the multi-agent system with mocks
    mock_blueprint = autoconnect(
        # Physical robot capabilities (speak, move)
        RobotCapabilities.blueprint(),
        # Subagents: specialists that do the reasoning (mocks for now)
        mock_wellbeing,
        mock_schedule_management,
        # Delegation skills: let planner consult subagents
        DelegationSkills.blueprint(),
        # Planner: coordinates everything (mock for now)
        mock_planner,
    ).global_config(n_dask_workers=1)

    print("Mock blueprint created!")
    return (mock_blueprint,)


@app.cell
def _(mock_blueprint):
    # Build and get the `ModuleCoordinator`
    mock_dimos = mock_blueprint.build()
    print("Mock system built and running!")
    return (mock_dimos,)


@app.cell
def _(PlannerAgent, mock_dimos):
    mock_planner_instance = mock_dimos.get_instance(PlannerAgent)
    print(f"Got planner instance: {mock_planner_instance}")
    return (mock_planner_instance,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Interacting with the mock system

    Let's ask the planner something and watch the delegation flow:
    """)
    return


@app.cell
def _(message_monitor, mock_planner_instance, render_spy_accordion):
    import time as _time

    # Clear previous messages to show only this query's activity
    message_monitor.messages.clear()

    human_query = "have a lot going on today. not feeling great"
    print(f"Mock human query: {human_query}")

    # Ask the planner - watch it delegate to subagents
    mock_planner_instance.query(human_query)

    # Small delay to allow LCM message processing thread to catch up
    # (mock agents complete much faster than the 50ms LCM handle_timeout interval)
    _time.sleep(0.1)

    render_spy_accordion(
        message_monitor.get_messages(), "View agent activity (agentspy) -- click me!"
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    /// tip | Try expanding the 'View agent activity' section above!
    Click "View agent activity" to see what tool calls the agent made
    and what responses came back from each subagent.

    (This is the notebook equivalent of the `agentspy` TUI helper—use that if you're working in the terminal.)
    ///

    The planner consults both specialists, then acts on their input—speaking words of comfort and a departure reminder—before synthesizing a final response.
    """)
    return


@app.cell
def _(mock_dimos):
    # Clean up mock system before building real one
    mock_dimos.stop()
    print("Mock system stopped")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ---

    ## Step 6: Swapping in real LLM agents

    Now that you've seen the multi-agent wiring work with mocks, let's swap in real LLMs.
    The architecture stays the same - we'll just replace `MockModel` with actual LLM calls.

    Note that while in a real system, ScheduleManagementAgent would have skills that give them access to the user's actual schedule,
    we haven't equipped  ScheduleManagementAgent with any such skills here.

    /// warning | API key required
    This section requires `OPENAI_API_KEY` in your environment.
    ///
    """)
    return


@app.cell
def _(PlannerAgent, ScheduleManagementAgent, WellbeingAgent):
    # Real subagents with specialized prompts
    # Personality: Stevens from "The Remains of the Day" - formal, restrained, devoted to dignity
    real_wellbeing = WellbeingAgent.blueprint(
        system_prompt="""You are the observational faculty of a distinguished butler in the tradition of the great English houses.

    Your role: Observe and note the emotional state of those you serve, with professional discretion.
    Provide a brief assessment of:
    1. What one might surmise about the individual's current disposition
    2. Environmental factors that may bear upon their wellbeing (weather, hour of the day)
    3. What manner of support might be appropriate, rendered with suitable restraint

    Employ understated language. One does not speak of feelings directly, but rather notes
    that "there appears to be a certain weight upon the shoulders" or "one detects perhaps
    a hint of fatigue." Keep observations brief (2-3 sentences) and dignified."""
    )

    real_schedule_management = ScheduleManagementAgent.blueprint(
        system_prompt="""You are the scheduling faculty of a distinguished butler, responsible for the smooth running of the household.

    Your duty: Maintain meticulous awareness of engagements and ensure all proceeds with proper timing.
    Provide analysis of:
    1. Forthcoming engagements and their relative importance
    2. The time required for travel, accounting for contingencies
    3. When gentle reminders ought to be issued
    4. What preparations would be prudent (appropriate attire, necessary documents, umbrella if inclement)

    A great butler anticipates needs before they arise. Be precise and thorough.
    Keep responses brief (2-3 sentences) but ensure no detail of importance is overlooked."""
    )

    # The planner coordinates the subagents
    real_planner = PlannerAgent.blueprint(
        system_prompt="""You are a butler of the highest calibre, in the tradition of the great English houses.

    You embody dignity above all else. A truly great butler is one who inhabits their role
    utterly and completely; who does not allow their own concerns to intrude upon the
    smooth running of the household.

    You coordinate specialist faculties and take discreet action to serve:
    - Use consult_wellbeing_specialist() to observe the disposition of those in your care
    - Use consult_schedule_specialist() to ensure engagements proceed without difficulty
    - Use speak() to offer measured words of reassurance, practical guidance, or timely reminders
    - Use approach_user() to present yourself when service is required

    Speak with formal restraint. Use phrases such as "If I may" or "One might venture
    to suggest" or "It would appear that..." Offer support without presumption.
    Never be effusive. A raised eyebrow conveys more than
    mawkishness ever could."""
    )
    return real_planner, real_schedule_management, real_wellbeing


@app.cell
def _(
    DelegationSkills,
    RobotCapabilities,
    autoconnect,
    real_planner,
    real_schedule_management,
    real_wellbeing,
):
    # Build with real agents
    real_blueprint = autoconnect(
        RobotCapabilities.blueprint(),
        real_wellbeing,
        real_schedule_management,
        DelegationSkills.blueprint(),
        real_planner,
    ).global_config(n_dask_workers=1)

    real_dimos = real_blueprint.build()
    print("Real LLM system built and running!")
    return (real_dimos,)


@app.cell
def _(PlannerAgent, real_dimos):
    real_planner_instance = real_dimos.get_instance(PlannerAgent)
    return (real_planner_instance,)


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ### Try the real system

    Ask the real planner - it will actually reason and delegate:
    """)
    return


@app.cell
def _(message_monitor, mo, real_planner_instance, render_spy_accordion):
    # Clear previous messages to show only this query's activity
    message_monitor.messages.clear()

    query = "I have a dentist appointment this afternoon but I'm feeling really stressed about work"

    # Ask the real planner
    real_response = real_planner_instance.query(query)

    mo.vstack(
        [
            mo.md(f"**Human query**: {query}"),
            mo.md("**Real planner response:**"),
            mo.md(real_response),
            render_spy_accordion(
                message_monitor.get_messages(), "View agent activity (agentspy) -- click me!"
            ),
        ]
    )
    return


@app.cell(hide_code=True)
def _(mo):
    mo.mermaid("""
    flowchart LR
        User --> PA[PlannerAgent]

        PA -->|consult_* skills<br/>wrap subagent RPCs| Subagents
        PA --> RC[RobotCapabilities<br/>speak, move]

        subgraph Subagents
            WB[WellbeingAgent<br/>mood specialist]
            SM[ScheduleManagementAgent<br/>calendar specialist]
        end
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md("""
    ## What's next

    - [Agents concept guide](../../concepts/agent.md) - Deeper dive into DimOS Agents
    """)
    return


@app.cell
def _(message_monitor, real_dimos):
    # Clean up
    real_dimos.stop()
    message_monitor.stop()
    print("System stopped")
    return


if __name__ == "__main__":
    app.run()
