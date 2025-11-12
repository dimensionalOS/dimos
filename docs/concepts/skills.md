# Skills

## What is a Skill?

A Skill in DimOS is the **bridge between AI reasoning and robot capabilities** - a reusable, composable unit of robot behavior that turns high-level intent ("wave hello") into low-level execution (joint trajectories, motor commands). Skills are what enable you to tell a robot **what** to do rather than **how** to do it.

Every Module in DimOS can expose skills - methods decorated with `@skill` that become callable by AI agents. Think of skills as the robot's vocabulary: a catalog of actions that agents can discover, reason about, and invoke.
<!-- Citation: dimos/core/module.py:77 - ModuleBase inherits from SkillContainer -->
<!-- Citation: notes/ARCHITECTURE.md:89 - "ANY module can expose skills via the @skill decorator" -->

```python
from dimos.core import Module
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Return

class NavigationModule(Module):
    @skill(ret=Return.call_agent)
    def navigate_to(self, location: str) -> str:
        """Navigate to a named location like 'kitchen' or 'front door'."""
        pose = self.get_rpc_calls("SpatialMemory.query")(location)
        self.get_rpc_calls("Navigation.set_goal")(pose)
        return f"Navigating to {location}"
```

<!-- Citation: dimos/protocol/skill/skill.py:65-113 - Full @skill decorator definition with all parameters -->
<!-- Note: NavigationModule is illustrative pattern based on dimos/robot/unitree_webrtc/rosnav.py:29-100 -->

Instead of telling a robot "set wheel velocities to [0.5, 0, 0.3] for 2.5 seconds," you tell it "navigate to the kitchen." The skill encapsulates the implementation details while exposing semantic meaning that both humans and LLMs understand.

## Why Skills Matter

### High-Level Control instead of Low-Level Commands

Skills allow you to work with high-level, semantic operations rather than motor commands:

```python
# Without skills: manually control motors
robot.set_joint_velocity(3, 0.5)
robot.wait(2.0)
robot.set_joint_velocity(3, -0.5)

# With skills: semantic actions
robot.wave_hello()
```

<!-- Note: Code example is illustrative, not from codebase -->

This isn't just convenience - it's a fundamentally different way of working with robots that aligns with how people naturally think about behavior.

### Safety Through Constrained Action Space

Skills create a safety boundary between AI reasoning and robot hardware. Agents can only invoke tested, validated behaviors - they cannot issue arbitrary motor commands:

```python
# Agent can call these skills
navigate_to("kitchen")    # ✓ Safe, tested behavior
wave_hello()             # ✓ Safe, tested behavior
dance_routine_1()        # ✓ Safe, tested behavior

# Agent CANNOT do this
set_motor_torque(motor_id=3, torque=100)  # ✗ Not exposed as skill
```

<!-- Interpretation: Safety through constrained action space is a design principle - not a single code reference -->
<!-- Note: Code example is illustrative, demonstrating concept rather than actual implementation -->

This constrained action space enables agents to operate autonomously while maintaining safety guarantees.

### Cross-Platform Portability

Write skills once, run them on different robots. The same `pick_up()` skill works on a quadruped robot, humanoid robot, or robotic arm - the implementation adapts to the hardware:

```python
@skill()
def pick_up(self, object_name: str) -> str:
    """Pick up an object - adapts to robot capabilities."""
    # Implementation uses platform-specific hardware
    # But skill interface stays consistent
    return f"Picked up {object_name}"
```

<!-- Note: pick_up example is illustrative, showing the pattern rather than actual implementation -->

### Composability for Complex Behaviors

Skills compose naturally. Agents can chain multiple skills to create complex behaviors:

```python
# Agent reasons about this sequence
navigate_to("kitchen")
detect_objects()
grasp("coffee_pot")
navigate_to("living_room")
pour("coffee", "mug")
```

<!-- Interpretation: Skill composability is a design pattern observed across system - not a single code reference -->
<!-- Note: Coffee-making sequence is illustrative, demonstrating skill composition concept -->

Each skill is independently testable and reusable. Agents compose them into task-specific sequences based on high-level goals.

### Observable and Debuggable

Every skill execution creates an audit trail. You can trace exactly what the agent decided to do, when it did it, and what the result was:

```markdown
[12:45:03] Agent called: navigate_to(location="kitchen")
[12:45:03] Skill state: pending → started
[12:45:15] Skill state: started → completed
[12:45:15] Result: "Arrived at kitchen"
```

This observability is crucial for debugging agent behavior and building trust in autonomous systems.
<!-- Citation: https://news.ycombinator.com/item?id=45457631 - "Observability & debugging" listed as core user need -->
<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum defines message protocol for state tracking -->
<!-- Note: Execution log example is illustrative, showing concept of audit trail -->

## Skills in Context

Understanding how skills fit into the larger system helps clarify their role in DimOS.

### Skills ↔ Agents

Agents see skills as **function-callable tools** compatible with OpenAI's function calling API:

```python
# Agent's view of a skill
{
    "name": "navigate_to",
    "description": "Navigate to a named location",
    "parameters": {
        "type": "object",
        "properties": {
            "location": {"type": "string", "description": "Target location"}
        },
        "required": ["location"]
    }
}
```

<!-- Citation: dimos/protocol/skill/schema.py - function_to_schema() converts Python functions to OpenAI schema -->
<!-- Citation: dimos/skills/skills.py:165 - pydantic_function_tool() for OpenAI function calling format -->
<!-- Citation: skills.py:159 -->
<!-- Note: JSON schema example is illustrative, showing OpenAI function calling format -->

The agent's LLM reasons about which skills to invoke and in what order. Skills are semantic primitives that LLMs understand - "navigate," "grasp," "detect" - rather than low-level commands.

### Skills ↔ Modules

Skills are methods on [Modules](./modules.md). This makes them naturally distributed:

```python
class RobotControlModule(Module):
    # Module provides execution context
    rpc_calls = ["Hardware.send_command"]

    @skill()
    def emergency_stop(self) -> str:
        """Immediately halt all robot motion."""
        # Skill has access to Module's RPC, streams, config
        self.get_rpc_calls("Hardware.send_command")("STOP")
        return "Emergency stop executed"
```

<!-- Citation: dimos/core/module.py:77 - ModuleBase inherits SkillContainer, making all Modules skill-capable -->
<!-- Citation: dimos/core/module.py:77 - Skills are methods on Module classes (which inherit from ModuleBase → SkillContainer), making them naturally distributed -->
<!-- Note: RobotControlModule example is illustrative, demonstrating module-skill relationship -->

The Module abstraction provides:

- Distributed execution across Dask workers
- Stream communication for sensor data
- RPC for inter-module calls
- Lifecycle management (start/stop)

<!-- Citation: dimos/core/module.py:85-99 - Modules provide rpc_calls, streams (In[T]/Out[T]), and lifecycle methods (start/stop) -->

Skills leverage all these Module capabilities while exposing a clean interface to agents.

### Skills ↔ Hardware

Skills abstract hardware differences behind common interfaces. When deploying to different robots, you swap the hardware-specific skills module but keep the same agent code:

```python
# Platform-specific implementation
class UnitreeGo2Skills(Module):
    @skill()
    def stand_up(self) -> str:
        """Stand up from prone position."""
        # Unitree Go2 uses API ID 1004
        self.connection.send(StandUp)  # api_id=1004
        return "Standing up"

# Platform-specific implementation
class UnitreeG1Skills(Module):
    @skill()
    def stand_up(self) -> str:
        """Stand up from prone position."""
        # G1 humanoid uses different command sequence
        self.connection.send(G1StandSequence)
        return "Standing up"
```

<!-- Citation: dimos/robot/unitree_webrtc/unitree_skills.py:43-45 - StandUp skill with api_id=1004 for Unitree Go2 -->
<!-- Citation: dimos/robot/unitree_webrtc/unitree_skills.py - UNITREE_WEBRTC_CONTROLS list defines 32 platform-specific skills -->
<!-- Note: UnitreeGo2Skills and UnitreeG1Skills examples are illustrative, showing cross-platform abstraction pattern -->

Both expose `stand_up()` as a skill, but with platform-specific implementations. Agents call the same skill name across different robots.

### A Complete Example

Here's how everything connects:

```python
# User gives command
"Go to the kitchen and tell me what you see"

# Agent Module receives query, reasons with LLM
agent.process_query(user_input)

# Agent decides to call skills in sequence
1. agent calls skill: navigate_to("kitchen")
   └─> NavigationModule.navigate_to() executes
       └─> Uses RPC to query SpatialMemory
       └─> Uses RPC to command Navigation stack
       └─> Returns "Arrived at kitchen"

2. agent calls skill: detect_objects()
   └─> PerceptionModule.detect_objects() executes
       └─> Reads from camera stream
       └─> Runs detection model
       └─> Returns "Found: [cup, plate, refrigerator]"

3. agent synthesizes response
   └─> "I've arrived at the kitchen. I can see a cup, plate, and refrigerator."
```

This shows the neurosymbolic orchestration pattern where agents handle high-level reasoning while skills handle execution.

<!-- Note: Complete example is illustrative, demonstrating the full integration pattern -->

## How Skills Work

Let's look at the mechanics of how skills actually work in DimOS.

### Every Module Can Have Skills

Any Module can expose skills via the `@skill` decorator.

<!-- Citation: dimos/core/module.py:77 - "class ModuleBase(Configurable[ModuleConfig], SkillContainer, Resource)" - Every Module automatically inherits from SkillContainer -->
<!-- Citation: notes/ARCHITECTURE.md:89 - "ANY module can expose skills via the @skill decorator" -->

Modules inherit skill capabilities through this chain: Module extends DaskModule, which extends ModuleBase, which extends SkillContainer.

<!-- Citation: dimos/core/module.py:351,278,77 - Module = DaskModule → DaskModule(ModuleBase) → ModuleBase(..., SkillContainer, ...) -->

Every Module gets:

- Thread pool for concurrent skill execution (max 50 workers)
- Message protocol for distributed execution
- Skill discovery and registration
- No special setup required

<!-- Citation: dimos/protocol/skill/skill.py:149-153 - SkillContainer class definition with _skill_thread_pool and _skill_transport -->
<!-- Citation: dimos/protocol/skill/skill.py:126-128 - ThreadPoolExecutor(max_workers=50, thread_name_prefix='skill_worker') -->
<!-- Citation: dimos/protocol/skill/skill.py:178-240 - SkillContainer provides call_skill(), skills(), dynamic_skills() methods -->

### The @skill Decorator Pattern

Define skills with a decorator on Module methods:

```python
from dimos.core import Module
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Return, Stream, Reducer

class PerceptionModule(Module):
    @skill(
        ret=Return.call_agent,    # Notify agent when complete
        stream=Stream.passive,     # Support streaming updates
        reducer=Reducer.latest,    # Keep only latest value
        hide_skill=False          # Visible to agents
    )
    def detect_objects(self) -> str:
        """Detect all visible objects in current view."""
        detections = self._run_detector()
        return f"Found {len(detections)} objects"
```

<!-- Citation: dimos/protocol/skill/skill.py:65-70 - @skill decorator signature with all parameters -->
<!-- Citation: dimos/protocol/skill/skill.py:96-105 - SkillConfig creation with reducer, stream, ret, output, schema, hide_skill -->
<!-- Note: PerceptionModule example is illustrative, demonstrating all decorator parameters -->

The decorator wraps your method with:

- Message protocol for distributed execution
- State tracking (pending → started → completed)
- Result aggregation for streaming skills
- Automatic function schema generation for LLMs

<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum: pending, start, stream, reduced_stream, ret, error -->
<!-- Citation: dimos/protocol/skill/skill.py:103 - function_to_schema(f) generates OpenAI-compatible schema -->
<!-- Citation: dimos/protocol/skill/skill.py:191-227 - State machine implementation: start → [stream]* → ret/error -->

### Skills as Distributed Methods

Skills execute as methods on distributed Modules. This means they have full access to:

```python
from dimos.core import Module, In
from dimos.protocol.skill.skill import skill
from dimos.msgs.geometry_msgs import PoseStamped

class NavigationSkills(Module):
    # Declare dependencies
    rpc_calls = ["SpatialMemory.query", "Navigation.set_goal"]

    # Declare streams
    odom: In[PoseStamped] = None

    @skill()
    def navigate_with_text(self, query: str) -> str:
        """Navigate using natural language location description."""
        # Access RPC methods
        pose = self.get_rpc_calls("SpatialMemory.query")(query)

        # Access streams
        current_position = self.odom.latest()

        # Call other modules
        self.get_rpc_calls("Navigation.set_goal")(pose)

        return f"Navigating from {current_position} to {query}"
```

<!-- Note: NavigationSkills example pattern based on dimos/robot/unitree_webrtc/rosnav.py:29-100 -->
<!-- Citation: dimos/core/module.py:85 - rpc_calls list for declaring dependencies -->

Skills compose Module capabilities - RPC calls to other modules, stream subscriptions, hardware access - into agent-callable actions.

### Message Protocol and Execution Flow

When an agent calls a skill, here's what happens:

```
1. Agent decides to call skill based on LLM reasoning
2. SkillCoordinator creates SkillState tracking object
3. Skill executes in background thread pool
4. Skill publishes messages: start → [stream]* → ret/error
5. Coordinator aggregates results (using Reducer if streaming)
6. Agent receives final result and continues reasoning
```

<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum: pending=0, start=1, stream=2, reduced_stream=3, ret=4, error=5 -->
<!-- Citation: dimos/protocol/skill/skill.py:191-227 - State machine implementation: start → [stream]* → ret/error -->
<!-- Citation: dimos/protocol/skill/skill.py:191,213,227 - Message protocol contracts: initialization (start), termination (ret/error), streaming -->
<!-- Note: Execution flow diagram is illustrative, representing the message protocol -->

The message protocol enables:

- **Non-blocking execution** - Agents don't wait, skills run concurrently
- **Progress monitoring** - Long operations stream updates
- **Distributed deployment** - Skills run on different machines/processes
- **Audit trails** - Every execution is traceable

<!-- Citation: dimos/protocol/skill/skill.py:126-128 - Thread pool for non-blocking execution -->
<!-- Citation: dimos/protocol/skill/skill.py:191-227 - Message-based communication enables decoupling and distributed support -->
<!-- Citation: notes/ARCHITECTURE.md:25-26 - Distributed actor model -->

<!-- TODO: Move the following to a tutorial/how-to -->
## Defining Your First Skill

Here's a simple skill to get started:

```python
from dimos.core import Module
from dimos.protocol.skill.skill import skill

class MyRobotSkills(Module):
    @skill()
    def greet(self, name: str) -> str:
        """Greet a person by name."""
        # Your implementation here
        print(f"Hello, {name}!")
        return f"Greeted {name}"
```

<!-- Citation: dimos/agents2/skills/demo_calculator_skill.py:19-38 - DemoCalculatorSkill with sum_numbers method -->
<!-- Note: MyRobotSkills example is simplified for tutorial purposes, based on the calculator pattern -->

Note that `@skill()` uses smart defaults that work for most cases:

- `ret=Return.call_agent` - Results sent back to agent (most common)
- `stream=Stream.none` - No streaming
- `reducer=Reducer.latest` - For streaming, keep latest value

Most skills work great with just `@skill()`!

Key points:

- Inherit from `Module` to get skill capabilities automatically
- Use `@skill()` decorator on methods (defaults work for most cases)
- Include docstring - agents see this as the function description
- Return string for LLM consumption

<!-- Citation: dimos/core/module.py:77 - Module inherits SkillContainer automatically -->
<!-- Citation: dimos/protocol/skill/skill.py:65-70 - @skill decorator with ret parameter -->
<!-- Citation: dimos/protocol/skill/skill.py:103 - function_to_schema extracts docstring for schema -->
<!-- Interpretation: Skills return strings for LLM compatibility - design pattern, not enforced by code -->

Use it in a system (see [Modules](modules.md) for blueprint details):

```python
from dimos.core.blueprints import autoconnect
from dimos.agents2.agent import llm_agent

# Compose agent with your skills
blueprint = autoconnect(
    MyRobotSkills(),
    llm_agent(system_prompt="You are a friendly robot.")
)

coordinator = blueprint.build()
coordinator.start()
coordinator.start_all_modules()  # Start the agent and skills
```

Now an agent can call your `greet` skill naturally: "Say hello to Alice."

<!-- Note: Blueprint usage example is illustrative, showing typical integration pattern -->
<!-- Citation: dimos/core/module.py:246-251 - Module.blueprint property returns create_module_blueprint partial -->

## Skill Configuration Options

Most skills work great with `@skill()`, but here are options for advanced cases:

### Return Behavior

Control when the agent is notified:

```python
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Return

@skill(ret=Return.call_agent)  # Default - most common
def notify_immediately(self) -> str:
    """Agent receives callback when complete."""
    return "done"

@skill(ret=Return.passive)
def agent_must_poll(self) -> str:
    """Agent queries for result."""
    return "done"

@skill(ret=Return.none)
def fire_and_forget(self):
    """Execute without notifying agent."""
    pass
```

<!-- Citation: dimos/protocol/skill/type.py:43-52 - Return enum: none=0, passive=1, call_agent=2, callback=3 -->
<!-- Citation: dimos/protocol/skill/type.py:44-51 - Documentation for each Return mode -->
<!-- Note: Code examples demonstrate the three main Return modes (callback not shown as it's not implemented) -->

Use `Return.call_agent` (default) for most skills. Use `Return.passive` when the agent should poll for results. Use `Return.none` for fire-and-forget actions that don't affect agent reasoning.

### Streaming Results

Skills can be generators that yield intermediate results for long-running operations:

```python
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Stream, Reducer

@skill(
    stream=Stream.call_agent,
    reducer=Reducer.latest
)
def explore_environment(self):
    """Explore while streaming progress updates."""
    for waypoint in self.waypoints:
        self.navigate(waypoint)
        yield f"Explored waypoint {waypoint}"  # Streams to agent

    return "Exploration complete"  # Final return value
```

<!-- Citation: dimos/protocol/skill/type.py:34-41 - Stream enum: none=0, passive=1, call_agent=2 -->
<!-- Citation: dimos/protocol/skill/type.py:266-269 - Reducer class with sum, latest, all reducers -->
<!-- Note: explore_environment example is illustrative, demonstrating streaming pattern -->

The `Reducer` controls how streaming values are aggregated:

- `Reducer.latest` - Keep only most recent (default)
- `Reducer.sum` - Accumulate numeric values
- `Reducer.all` - Collect all values as list

<!-- Citation: dimos/protocol/skill/type.py:266-269 - Reducer class attributes match these options -->
<!-- Citation: dimos/protocol/skill/skill.py:66 - reducer parameter defaults to Reducer.latest in @skill decorator -->

### Skill Visibility

Use `hide_skill=True` for internal helper skills or low-level operations that agents shouldn't directly invoke:

```python
from dimos.protocol.skill.skill import skill

@skill(hide_skill=True)
def internal_calibration(self):
    """Internal skill, not exposed to agents."""
    pass
```

<!-- Citation: dimos/protocol/skill/skill.py:70 - "hide_skill: bool = False" parameter in decorator -->
<!-- Citation: dimos/protocol/skill/skill.py:104 - hide_skill stored in SkillConfig -->
<!-- Note: internal_calibration example is illustrative, demonstrating hide_skill usage -->

**Note on SkillModule**: You may see `SkillModule` used in the codebase as an alternative to `Module`. `SkillModule` is a thin wrapper around `Module` that adds automatic skill registration with `llm_agent`. Both work - most skill-providing modules in the codebase use `SkillModule` for convenience, but `Module` works equally well. Use whichever you prefer.

## Common Skill Patterns

Skills typically coordinate actions across modules using RPC calls (see [Modules](modules.md)):

### Navigation Skills

Skills that move the robot:

```python
from dimos.protocol.skill.skill import skill

@skill()
def navigate_to_coordinates(self, x: float, y: float) -> str:
    """Navigate to specific coordinates."""
    pose = Pose(position=Point(x, y, 0))
    self.get_rpc_calls("Navigation.set_goal")(pose)
    return f"Navigating to ({x}, {y})"

@skill()
def tag_location(self, label: str) -> str:
    """Tag current position with a label for future reference."""
    self.get_rpc_calls("SpatialMemory.tag_location")(
        label, self.current_pose
    )
    return f"Tagged location as '{label}'"
```

### Perception Skills

Skills that query what the robot sees (return strings for agent consumption or structured data for programmatic use):

```python
from dimos.protocol.skill.skill import skill

@skill()
def list_visible_objects(self) -> str:
    """Return list of currently visible objects."""
    detections = self.get_rpc_calls("Detector.get_detections")()
    objects = [d.label for d in detections]
    return f"Visible: {', '.join(objects)}"

@skill()
def find_object_location(self, name: str) -> str:
    """Search memory for where object was last seen."""
    results = self.get_rpc_calls("SpatialMemory.query_by_text")(name)
    if results:
        return f"Last saw {name} at {results[0].pose}"
    return f"Never seen {name}"
```

### Compound Skills

Skills that orchestrate multiple operations:

```python
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Stream

@skill(stream=Stream.call_agent)
def search_for_object(self, target: str):
    """Explore environment until target is found."""
    for waypoint in self._exploration_waypoints():
        yield f"Searching at waypoint {waypoint}"
        self.navigate(waypoint)

        objects = self.detect_objects()
        if target in objects:
            return f"Found {target}!"

    return f"Could not find {target}"
```

<!-- Note: All examples in this section are illustrative, demonstrating common patterns based on system architecture -->

## Best Practices

### Design Principles

**Make skills focused** - Each skill should do one thing well. Instead of `fetch_and_deliver_object()`, create separate `navigate_to_object()`, `grasp_object()`, and `deliver_to_location()` skills that agents can compose.
<!-- Interpretation: Best practice based on composability principle and user research on multi-agent pipelines (notes/user-research-summary.md:17) -->

**Return meaningful strings** - LLMs consume your return values. "Navigated to kitchen in 12 seconds" helps agent reasoning more than "ok".
<!-- Interpretation: String returns are a design choice for LLM compatibility - best practice, not enforced by code -->

**Use streaming for long operations** - If a skill takes more than a few seconds, stream progress updates so agents and users know what's happening:
<!-- Citation: dimos/protocol/skill/type.py:34-41 - Stream enum supports streaming updates -->
<!-- Citation: dimos/protocol/skill/skill.py:200-210 - Generator pattern with stream message publishing -->

```python
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Stream, Reducer

@skill(stream=Stream.call_agent, reducer=Reducer.latest)
def long_operation(self):
    for step in self.steps:
        yield f"Progress: {step.percentage}%"
    return "Complete"
```

<!-- Note: Streaming example is illustrative, demonstrating the pattern -->

### Implementation

**Write descriptive docstrings** - The docstring becomes the function description that LLMs see. Make it clear and include parameter descriptions:
<!-- Citation: dimos/protocol/skill/skill.py:103 - function_to_schema(f) extracts docstring for OpenAI schema -->
<!-- Citation: dimos/protocol/skill/skill.py:110 - wrapper.__doc__ = f.__doc__ preserves docstring -->

```python
@skill()
def navigate_to(self, location: str) -> str:
    """Navigate to a named location in the environment.

    Args:
        location: Location name like 'kitchen', 'bedroom', or 'front door'

    Returns:
        Status message indicating success or failure
    """
```

<!-- Note: navigate_to docstring example is illustrative, demonstrating best practice -->

**Handle errors gracefully** - Skills should catch exceptions and return meaningful error messages that give agents context to try alternative approaches:
<!-- Citation: dimos/protocol/skill/skill.py:215-227 - Exception handling publishes MsgType.error with traceback -->

```python
@skill()
def navigate_to(self, location: str) -> str:
    try:
        pose = self.query_location(location)
        self.navigate(pose)
        return f"Arrived at {location}"
    except LocationNotFoundError:
        return f"Location '{location}' not found in memory"
    except NavigationError as e:
        return f"Navigation failed: {e}"
```

<!-- Note: Error handling example is illustrative, demonstrating best practice for graceful degradation -->

**Use RPC for module communication** - When skills need capabilities from other modules, declare dependencies in the `rpc_calls` list and use `get_rpc_calls()` to access them. This makes dependencies explicit and supports distributed execution.

### Integration

**Test skills independently** - Skills are Module methods. Test them in isolation before integrating with agents. This makes debugging easier and ensures skills work correctly before agents start composing them into complex behaviors.
<!-- Interpretation: Best practice recommendation, not directly evidenced in codebase -->

## Two Skill Systems

DimOS has two parallel skill implementations serving the same purpose - bridging AI reasoning with robot capabilities:

**Legacy (AbstractSkill)** - Pydantic-based class definitions in `dimos/skills/skills.py`, used with BaseAgent. Maintained for backward compatibility.

**Modern (@skill decorator)** - Decorator-based method definitions in `dimos/protocol/skill/`, used with llm_agent. Better integrated with the Module architecture and supports distributed execution.

<!-- Citation: notes/ARCHITECTURE.md:66-69 - Two skill systems documented -->
<!-- Citation: dimos/skills/skills.py:258-261 - AbstractSkill inherits from BaseModel (Pydantic) - legacy system -->
<!-- Citation: dimos/protocol/skill/skill.py:65-113,149-247 - @skill decorator and SkillContainer (modern system) -->

**For new code, use the modern @skill system** shown in this document.

## Summary

Skills bridge the semantic gap between what users want ("go to the kitchen") and what robots understand (velocity commands, joint angles). They enable high-level semantic control, safety through constrained actions, cross-platform portability, composable behaviors, and observable execution.
<!-- Interpretation: Summary synthesizes concepts explained throughout document with primary source citations -->

Every Module can expose skills via the `@skill` decorator. Agents discover these skills automatically and invoke them as function calls. Skills execute in background threads with message-based coordination for distributed execution.
<!-- Citation: dimos/core/module.py:77 - All Modules inherit SkillContainer -->
<!-- Citation: dimos/protocol/skill/type.py:90-96 - Message protocol with MsgType enum -->

Start with focused skills, clear docstrings, and meaningful return values. The same skill definitions work across different robot platforms - just swap the hardware modules underneath.

This skill-based abstraction lets you tell robots what to do, not how to do it.
<!-- user research: people like high-level apis for this kind of thing: https://news.ycombinator.com/item?id=44146459 -->

## Related Concepts

- **[Agent](agent.md)** - LLM-based reasoning systems that invoke skills
- **[Modules](modules.md)** - The distributed actors that provide skills
- **Blueprint** - Declarative system composition for connecting skills and agents

<!-- Citation: docs/concepts/agent.md - Agent documentation exists -->
<!-- Citation: docs/concepts/modules.md - Modules documentation exists -->
<!-- Note: Blueprint documentation not yet written but is a core concept -->
