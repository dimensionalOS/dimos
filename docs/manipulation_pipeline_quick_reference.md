# Manipulation Pipeline Quick Reference

## File Architecture Summary

```
dimos/
├── agents2/
│   └── skills/
│       ├── navigation.py              # ✅ Reference implementation
│       └── manipulation_example.py    # 🆕 Example for manipulation
│
├── manipulation/
│   ├── manipulation_module.py         # ✅ Core manipulation module
│   ├── manipulation_skills.py         # ✅ Basic low-level skills
│   ├── manipulation_interface.py      # ✅ History/constraints interface
│   ├── planning/                      # ✅ Motion planning stack
│   └── control/                       # ✅ Control stack
│
└── robot/
    └── [your_robot]/
        └── [robot]_agent.py           # 🆕 Your robot deployment
```

## Key Design Patterns

### 1. Skill Container Pattern

**Location**: `agents2/skills/manipulation_example.py`

```python
class ManipulationSkillContainer(SkillModule):
    # Inherit from SkillModule
    # Use @skill() decorator for agent-callable methods
    # Use get_rpc_calls() to access module RPCs
    # Subscribe to streams (camera, object detection)
    # Return human-readable strings
```

### 2. Module Integration

**Location**: `manipulation/manipulation_module.py`

```python
class ManipulationModule(Module):
    # Implements core manipulation logic
    # Provides RPC methods:
    #   - move_to_pose(x, y, z, roll, pitch, yaw)
    #   - move_to_joints(joints)
    #   - get_state()
    #   - cancel()
```

### 3. Agent Deployment

**Location**: `robot/[your_robot]/[robot]_agent.py`

```python
def deploy(dimos: DimosCluster, ip: str):
    # 1. Deploy hardware modules
    robot = robot_detector.deploy(dimos, ip)

    # 2. Deploy manipulation modules
    manip_module = dimos.deploy(ManipulationModule, ...)

    # 3. Deploy perception modules
    object_stream = dimos.deploy(ObjectDetectionStream, ...)

    # 4. Deploy skill container
    manip_skills = dimos.deploy(
        ManipulationSkillContainer,
        manip_module,
        object_stream,
        ...
    )
    manip_skills.start()

    # 5. Deploy agent
    agent = agents2.deploy(
        dimos,
        "You are controlling a robot arm",
        skill_containers=[..., manip_skills],
    )
```

## Comparison: Navigation vs Manipulation

| Component | Navigation | Manipulation |
|-----------|-----------|--------------|
| **Skill Container** | `NavigationSkillContainer` | `ManipulationSkillContainer` |
| **Core Module** | `BehaviorTreeNavigator` | `ManipulationModule` |
| **High-Level Skill** | `navigate_with_text()` | `manipulate_with_text()` |
| **Perception** | SpatialMemory, ObjectTracking | ObjectDetectionStream, Detection3D |
| **Planning** | Path planning (A*, RRT) | Motion planning (RRT*, IK) |
| **State** | NavigationState | ManipulationState |

## Implementation Checklist

### Phase 1: Foundation ✅
- [x] Review existing `manipulation/manipulation_module.py`
- [x] Review existing `manipulation/manipulation_skills.py`
- [x] Review `manipulation/planning/` stack
- [x] Review `manipulation/control/` stack

### Phase 2: Skill Container 🆕
- [ ] Create `agents2/skills/manipulation.py` (use `manipulation_example.py` as reference)
- [ ] Implement `manipulate_with_text()` skill
- [ ] Implement `move_arm_to_object()` skill
- [ ] Implement `grasp_object()` skill
- [ ] Implement `place_object()` skill
- [ ] Add object detection integration
- [ ] Add error handling and recovery

### Phase 3: Integration 🆕
- [ ] Create robot-specific agent deployment file
- [ ] Wire ManipulationModule → ManipulationSkillContainer
- [ ] Wire ObjectDetectionStream → ManipulationSkillContainer
- [ ] Wire Camera → ObjectDetectionStream
- [ ] Test end-to-end: "pick up the red cup"

### Phase 4: Advanced Features 🆕
- [ ] Add visual servoing integration
- [ ] Add constraint management
- [ ] Add manipulation memory/history
- [ ] Add error recovery and retry logic

## Key Skills to Implement

### High-Level Skills (Natural Language)

1. **`manipulate_with_text(query: str)`**
   - Parse natural language: "pick up the red cup"
   - Detect object in view
   - Execute manipulation sequence
   - Return status

2. **`move_arm_to_object(object_query: str)`**
   - Find object by description
   - Calculate approach pose
   - Move arm to approach position

3. **`grasp_object(object_id: int | str)`**
   - Execute complete grasp sequence
   - Pre-grasp → Open gripper → Grasp → Close gripper

4. **`place_object(location: str)`**
   - Find target location
   - Execute place sequence

### Low-Level Skills (State/Control)

5. **`get_arm_state()`**
   - Query current manipulation state

6. **`cancel_arm_motion()`**
   - Cancel active motion

7. **`open_gripper()` / `close_gripper()`**
   - Gripper control

## RPC Methods Needed

From `ManipulationModule`:
- `move_to_pose(x, y, z, roll, pitch, yaw) -> bool`
- `move_to_joints(joints: list[float]) -> bool`
- `get_state() -> ManipulationState`
- `get_state_name() -> str`
- `get_error() -> str`
- `cancel() -> bool`
- `reset() -> bool`
- `get_current_joints() -> list[float]`
- `get_ee_pose() -> tuple[float, ...]`

## Streams to Subscribe

1. **`color_image: In[Image]`**
   - Latest camera image for visual querying

2. **`object_detections: In[dict[str, Any]]`**
   - ObjectDetectionStream output with detected objects

## Next Steps

1. **Copy example**: Use `manipulation_example.py` as starting point
2. **Adapt to your needs**: Modify skills based on your robot capabilities
3. **Test incrementally**: Start with simple skills, add complexity
4. **Integrate**: Wire modules together in robot deployment file
5. **Iterate**: Refine based on agent behavior

## References

- **Design Guide**: `docs/manipulation_agentic_pipeline_design.md`
- **Navigation Reference**: `agents2/skills/navigation.py`
- **G1 Agent Example**: `robot/unitree/g1/g1agent.py`
- **Manipulation Module**: `manipulation/manipulation_module.py`
- **Example Implementation**: `agents2/skills/manipulation_example.py`
