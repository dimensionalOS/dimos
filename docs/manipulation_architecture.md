# Manipulation Architecture

Complete guide to the manipulation pipeline architecture in Dimos.

## Overview

The manipulation system follows a three-layer architecture:

```
┌─────────────────────────────────────────────────────────┐
│  Agent Layer (LLM + Human Interface)                    │
│  - Natural language understanding                       │
│  - User interaction                                     │
└─────────────────────────────────────────────────────────┘
                          ↓ uses
┌─────────────────────────────────────────────────────────┐
│  Skills Layer (High-level behaviors)                    │
│  - ManipulationSkillContainer                           │
│  - MoveAxisModule                                       │
│  - RobotCapabilities                                    │
└─────────────────────────────────────────────────────────┘
                          ↓ uses RPC
┌─────────────────────────────────────────────────────────┐
│  Hardware Layer (Low-level control)                     │
│  - xarm_driver / piper_driver                           │
│  - manipulation_module (Drake planning)                 │
│  - joint_trajectory_controller                          │
└─────────────────────────────────────────────────────────┘
```

## File Structure

```
dimos/
├── manipulation/
│   ├── modules/
│   │   ├── move_axis.py              # Directional movement (RPC methods)
│   │   └── robot_capabilities.py     # Robot capabilities (speak, etc.)
│   ├── manipulation_module.py         # Core Drake-based motion planning
│   └── manipulation_blueprints.py     # Hardware-only blueprints
│
└── agents2/
    └── skills/
        ├── manipulation_skill_container.py  # High-level skills
        └── manipulation.py                   # Complete system blueprints
```

## Layer Details

### 1. Hardware Layer

Located in: `dimos/manipulation/`

#### manipulation_module.py
Core manipulation with Drake-based motion planning.

**Key RPC Methods:**
- `get_ee_pose()` → [x, y, z, roll, pitch, yaw]
- `move_to_pose(x, y, z, roll, pitch, yaw)` → bool
- `get_current_joints()` → list[float]
- `move_to_joints(joints)` → bool
- `cancel()` → None
- `get_state()` → str

#### manipulation_blueprints.py
Hardware-only blueprints combining driver + planner + controller:
- `xarm6_manipulation`
- `xarm7_manipulation`
- `piper_manipulation`
- `xarm6_planner_only`

**Example:**
```python
xarm7_manipulation = autoconnect(
    xarm_driver(...),
    manipulation_module(...),
    joint_trajectory_controller(...),
).transports({...})
```

### 2. Skills Layer

Located in: `dimos/manipulation/modules/` and `dimos/agents2/skills/`

#### MoveAxisModule (modules/move_axis.py)
Provides directional movement by wrapping ManipulationModule RPC methods.

**RPC Dependencies:**
- `ManipulationModule.get_ee_pose`
- `ManipulationModule.move_to_pose`

**RPC Methods:**
- `move_up(distance: float)` → bool
- `move_down(distance: float)` → bool
- `move_left(distance: float)` → bool
- `move_right(distance: float)` → bool
- `move_forward(distance: float)` → bool
- `move_backward(distance: float)` → bool

**Implementation Pattern:**
```python
@rpc
def move_up(self, distance: float) -> bool:
    current_pose = self._get_ee_pose()  # RPC call
    x, y, z, roll, pitch, yaw = current_pose
    new_z = z + distance
    return self._move_to_pose(x, y, new_z, roll, pitch, yaw)  # RPC call
```

#### RobotCapabilities (modules/robot_capabilities.py)
Basic robot capabilities, extensible for future features.

**RPC Methods:**
- `speak(text: str)` → str

#### ManipulationSkillContainer (agents2/skills/manipulation_skill_container.py)
High-level skills exposed to LLM agents.

**RPC Dependencies:**
```python
rpc_calls = [
    "MoveAxisModule.move_up",
    "MoveAxisModule.move_down",
    "MoveAxisModule.move_left",
    "MoveAxisModule.move_right",
    "MoveAxisModule.move_forward",
    "MoveAxisModule.move_backward",
    "ManipulationModule.get_ee_pose",
    "ManipulationModule.move_to_pose",
    "ManipulationModule.cancel",
    "RobotCapabilities.speak",
]
```

**Skills (exposed to LLM):**
- `move_arm_up(distance=0.05)` - Move arm upward
- `move_arm_down(distance=0.05)` - Move arm downward
- `move_arm_left(distance=0.05)` - Move arm left
- `move_arm_right(distance=0.05)` - Move arm right
- `move_arm_forward(distance=0.05)` - Move arm forward
- `move_arm_backward(distance=0.05)` - Move arm backward
- `move_arm_to_pose(x, y, z, roll, pitch, yaw)` - Move to specific pose
- `get_arm_position()` - Get current position
- `stop_arm_movement()` - Cancel movement
- `greet(name)` - Greet someone by name

### 3. Agent Layer

Located in: `dimos/agents2/`

#### LLM Agent
- Understands natural language commands
- Selects appropriate skills
- Provides conversational interface

#### Human Input
- CLI interface for user interaction
- Publishes to `/human_input` LCM topic

## Blueprints

### Available Blueprints

Located in: `dimos/agents2/skills/manipulation.py`

#### 1. `manipulation_skills`
Skills + agent, no hardware. For testing or composing with hardware.

```python
manipulation_skills = autoconnect(
    RobotCapabilities.blueprint(),
    MoveAxisModule.blueprint(),
    ManipulationSkillContainer.blueprint(),
    llm_agent(...),
    human_input(),
)
```

#### 2. `greetings_skill_blueprint`
Minimal example with just greeting capability.

```python
greetings_skill_blueprint = autoconnect(
    RobotCapabilities.blueprint(),
    ManipulationSkillContainer.blueprint(),
    llm_agent(...),
    human_input(),
)
```

#### 3. `xarm7_manipulation_agent` ⭐
**Complete system** - hardware + skills + agent.

```python
xarm7_manipulation_agent = autoconnect(
    xarm7_manipulation,      # Hardware layer
    manipulation_skills,     # Skills + agent
)
```

**Full component breakdown:**
```
xarm7_manipulation_agent
├─ Hardware Layer (from xarm7_manipulation)
│  ├─ xarm_driver                    # Hardware interface
│  ├─ manipulation_module            # Drake motion planning
│  └─ joint_trajectory_controller    # Trajectory execution
│
├─ Skills Layer (from manipulation_skills)
│  ├─ RobotCapabilities              # speak()
│  ├─ MoveAxisModule                 # Directional movement
│  └─ ManipulationSkillContainer     # High-level skills
│
└─ Agent Layer (from manipulation_skills)
   ├─ LLM Agent                      # Natural language
   └─ Human Input                    # CLI interface
```

## Usage

### CLI Commands

```bash
# Hardware only (no agent)
dimos run xarm7-manipulation

# Greetings skill (testing/demo)
dimos run greetings

# Complete system with agent
dimos run xarm7-manipulation-agent
```

### Example Interaction

```
User: "Move the arm up 10 centimeters"
Agent: → calls move_arm_up(0.10)
MoveAxisModule: → calls ManipulationModule.get_ee_pose()
MoveAxisModule: → calls ManipulationModule.move_to_pose(x, y, z+0.10, roll, pitch, yaw)
ManipulationModule: → plans trajectory with Drake
ManipulationModule: → sends to joint_trajectory_controller
joint_trajectory_controller: → executes trajectory
xarm_driver: → sends commands to hardware

User: "My name is Alice"
Agent: → calls greet("Alice")
ManipulationSkillContainer: → calls RobotCapabilities.speak("Hello, Alice!")
```

## RPC Connection Flow

How `autoconnect()` wires RPC methods:

1. **Declaration**: Skill modules declare needed RPC calls
   ```python
   rpc_calls = ["MoveAxisModule.move_up"]
   ```

2. **Auto-connection**: `autoconnect()` finds matching RPC methods and creates setters
   ```python
   @rpc
   def set_MoveAxisModule_move_up(self, callable: RpcCall):
       self._move_up = callable
   ```

3. **Usage**: Skills call RPC methods as if they were local
   ```python
   result = self._move_up(0.05)
   ```

## Key Design Patterns

### 1. RPC Method Wrapping
Lower-level modules expose RPC methods that higher-level modules wrap:
```
ManipulationModule.get_ee_pose()  [Low-level]
         ↓ used by
MoveAxisModule.move_up()          [Mid-level]
         ↓ used by
ManipulationSkillContainer.move_arm_up()  [High-level]
```

### 2. Blueprint Composition
Blueprints can be composed for clean separation:
```python
# Hardware-only blueprint
xarm7_manipulation = autoconnect(driver, planner, controller)

# Skills-only blueprint
manipulation_skills = autoconnect(capabilities, move_axis, skill_container, agent)

# Complete system
xarm7_manipulation_agent = autoconnect(
    xarm7_manipulation,    # Hardware
    manipulation_skills,   # Skills + Agent
)
```

### 3. Skill Module Pattern
Skills use `@skill()` decorator and RPC dependencies:
```python
class ManipulationSkillContainer(SkillModule):
    rpc_calls = ["MoveAxisModule.move_up"]
    
    @skill()
    def move_arm_up(self, distance: float = 0.05) -> str:
        move_up = self.get_rpc_calls("MoveAxisModule.move_up")
        success = move_up(distance)
        return "Success" if success else "Failed"
```

## Environment Setup

Set `OPENAI_API_KEY` for LLM agent:

```bash
# Option 1: In .env file
OPENAI_API_KEY=sk-your-api-key-here

# Option 2: Export in shell
export OPENAI_API_KEY=sk-your-api-key-here

# Option 3: Add to ~/.bashrc
echo 'export OPENAI_API_KEY=sk-your-api-key-here' >> ~/.bashrc
source ~/.bashrc
```

The key is automatically loaded by `GlobalConfig` via pydantic-settings.

## Adding New Skills

To add a new manipulation skill:

1. **If needed, add RPC method to a core module** (e.g., `MoveAxisModule`)
2. **Add skill to `ManipulationSkillContainer`**:
   ```python
   @skill()
   def my_new_skill(self, param: str) -> str:
       """Skill description for LLM."""
       # Use RPC calls
       result = self.get_rpc_calls("SomeModule.some_method")(param)
       return f"Completed: {result}"
   ```
3. **Update RPC dependencies** if calling new modules

That's it! The skill is automatically available to the LLM agent.

