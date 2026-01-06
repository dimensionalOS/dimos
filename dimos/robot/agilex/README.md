# DIMOS Manipulator Robot Development Guide

This guide explains how to create robot classes, integrate agents, and use the DIMOS module system with LCM transport.

## Table of Contents
1. [Robot Class Architecture](#robot-class-architecture)
2. [Module System & LCM Transport](#module-system--lcm-transport)
3. [Agent Integration](#agent-integration)
4. [Complete Example](#complete-example)

## Robot Class Architecture

### Basic Robot Class Structure

A DIMOS robot class should follow this pattern:

```python
from typing import Optional, List
from dimos import core
from dimos.types.robot_capabilities import RobotCapability

class YourRobot:
    """Your robot implementation."""

    def __init__(self, robot_capabilities: Optional[List[RobotCapability]] = None):
        # Core components
        self.dimos = None
        self.modules = {}
        self.skill_library = SkillLibrary()

        # Define capabilities
        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    async def start(self):
        """Start the robot modules."""
        # Initialize DIMOS with worker count
        self.dimos = core.start(2)  # Number of workers needed

        # Deploy modules
        # ... (see Module System section)

    def stop(self):
        """Stop all modules and clean up."""
        # Stop modules
        # Close DIMOS
        if self.dimos:
            self.dimos.close()
```

### Key Components Explained

1. **Initialization**: Store references to modules, skills, and capabilities
2. **Async Start**: Modules must be deployed asynchronously
3. **Proper Cleanup**: Always stop modules before closing DIMOS

## Module System & LCM Transport

### Understanding DIMOS Modules

Modules are the building blocks of DIMOS robots. They:
- Process data streams (inputs)
- Produce outputs
- Can be connected together
- Communicate via LCM (Lightweight Communications and Marshalling)

### Deploying a Module

```python
# Deploy a camera module
self.camera = self.dimos.deploy(
    ZEDModule,                    # Module class
    camera_id=0,                  # Module parameters
    resolution="HD720",
    depth_mode="NEURAL",
    fps=30,
    publish_rate=30.0,
    frame_id="camera_frame"
)
```

### Setting Up LCM Transport

LCM transport enables inter-module communication:

```python
# Enable LCM auto-configuration
from dimos.protocol import pubsub
pubsub.lcm.autoconf()

# Configure output transport
self.camera.color_image.transport = core.LCMTransport(
    "/camera/color_image",        # Topic name
    Image                         # Message type
)
self.camera.depth_image.transport = core.LCMTransport(
    "/camera/depth_image",
    Image
)
```

### Connecting Modules

Connect module outputs to inputs:

```python
# Connect manipulation module to camera outputs
self.manipulation.rgb_image.connect(self.camera.color_image)
self.manipulation.depth_image.connect(self.camera.depth_image)
self.manipulation.camera_info.connect(self.camera.camera_info)
```

### Module Communication Pattern

```
┌──────────────┐  LCM    ┌────────────────┐  LCM    ┌──────────────┐
│   Camera     │────────▶│  Manipulation  │────────▶│ Visualization│
│   Module     │ Messages│     Module     │ Messages│    Output    │
└──────────────┘         └────────────────┘         └──────────────┘
     ▲                          ▲
     │                          │
     └──────────────────────────┘
          Direct Connection via RPC call
```

## Agent Integration

### Setting Up Agent with Robot

The run file pattern for agent integration:

```python
#!/usr/bin/env python3
import asyncio
import reactivex as rx
from dimos.agents_deprecated.claude_agent import ClaudeAgent
from dimos.web.robot_web_interface import RobotWebInterface

def main():
    # 1. Create and start robot
    robot = YourRobot()
    asyncio.run(robot.start())

    # 2. Set up skills
    skills = robot.get_skills()
    skills.add(YourSkill)
    skills.create_instance("YourSkill", robot=robot)

    # 3. Set up reactive streams
    agent_response_subject = rx.subject.Subject()
    agent_response_stream = agent_response_subject.pipe(ops.share())

    # 4. Create web interface
    web_interface = RobotWebInterface(
        port=5555,
        text_streams={"agent_responses": agent_response_stream},
        audio_subject=rx.subject.Subject()
    )

    # 5. Create agent
    agent = ClaudeAgent(
        dev_name="your_agent",
        input_query_stream=web_interface.query_stream,
        skills=skills,
        system_query="Your system prompt here",
        model_name="claude-3-5-haiku-latest"
    )

    # 6. Connect agent responses
    agent.get_response_observable().subscribe(
        lambda x: agent_response_subject.on_next(x)
    )

    # 7. Run interface
    web_interface.run()
```

### Key Integration Points

1. **Reactive Streams**: Use RxPy for event-driven communication
2. **Web Interface**: Provides user input/output
3. **Agent**: Processes natural language and executes skills
4. **Skills**: Define robot capabilities as executable actions

## Complete Example

### Step 1: Create Robot Class (`my_robot.py`)

```python
import asyncio
from typing import Optional, List
from dimos import core
from dimos.hardware.camera import CameraModule
from dimos.manipulation.module import ManipulationModule
from dimos.skills.skills import SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos_lcm.sensor_msgs import Image, CameraInfo
from dimos.protocol import pubsub

class MyRobot:
    def __init__(self, robot_capabilities: Optional[List[RobotCapability]] = None):
        self.dimos = None
        self.camera = None
        self.manipulation = None
        self.skill_library = SkillLibrary()

        self.capabilities = robot_capabilities or [
            RobotCapability.VISION,
            RobotCapability.MANIPULATION,
        ]

    async def start(self):
        # Start DIMOS
        self.dimos = core.start(2)

        # Enable LCM
        pubsub.lcm.autoconf()

        # Deploy camera
        self.camera = self.dimos.deploy(
            CameraModule,
            camera_id=0,
            fps=30
        )

        # Configure camera LCM
        self.camera.color_image.transport = core.LCMTransport("/camera/rgb", Image)
        self.camera.depth_image.transport = core.LCMTransport("/camera/depth", Image)
        self.camera.camera_info.transport = core.LCMTransport("/camera/info", CameraInfo)

        # Deploy manipulation
        self.manipulation = self.dimos.deploy(ManipulationModule)

        # Connect modules
        self.manipulation.rgb_image.connect(self.camera.color_image)
        self.manipulation.depth_image.connect(self.camera.depth_image)
        self.manipulation.camera_info.connect(self.camera.camera_info)

        # Configure manipulation output
        self.manipulation.viz_image.transport = core.LCMTransport("/viz/output", Image)

        # Start modules
        self.camera.start()
        self.manipulation.start()

        await asyncio.sleep(2)  # Allow initialization

    def get_skills(self):
        return self.skill_library

    def stop(self):
        if self.manipulation:
            self.manipulation.stop()
        if self.camera:
            self.camera.stop()
        if self.dimos:
            self.dimos.close()
```

### Step 2: Create Run Script (`run.py`)

```python
#!/usr/bin/env python3
import asyncio
import os
from my_robot import MyRobot
from dimos.agents_deprecated.claude_agent import ClaudeAgent
from dimos.skills.basic import BasicSkill
from dimos.web.robot_web_interface import RobotWebInterface
import reactivex as rx
import reactivex.operators as ops

SYSTEM_PROMPT = """You are a helpful robot assistant."""

def main():
    # Check API key
    if not os.getenv("ANTHROPIC_API_KEY"):
        print("Please set ANTHROPIC_API_KEY")
        return

    # Create robot
    robot = MyRobot()

    try:
        # Start robot
        asyncio.run(robot.start())

        # Set up skills
        skills = robot.get_skills()
        skills.add(BasicSkill)
        skills.create_instance("BasicSkill", robot=robot)

        # Set up streams
        agent_response_subject = rx.subject.Subject()
        agent_response_stream = agent_response_subject.pipe(ops.share())

        # Create web interface
        web_interface = RobotWebInterface(
            port=5555,
            text_streams={"agent_responses": agent_response_stream}
        )

        # Create agent
        agent = ClaudeAgent(
            dev_name="my_agent",
            input_query_stream=web_interface.query_stream,
            skills=skills,
            system_query=SYSTEM_PROMPT
        )

        # Connect responses
        agent.get_response_observable().subscribe(
            lambda x: agent_response_subject.on_next(x)
        )

        print("Robot ready at http://localhost:5555")

        # Run
        web_interface.run()

    finally:
        robot.stop()

if __name__ == "__main__":
    main()
```

### Step 3: Define Skills (`skills.py`)

```python
from dimos.skills import Skill, skill

@skill(
    description="Perform a basic action",
    parameters={
        "action": "The action to perform"
    }
)
class BasicSkill(Skill):
    def __init__(self, robot):
        self.robot = robot

    def run(self, action: str):
        # Implement skill logic
        return f"Performed: {action}"
```

## Best Practices

1. **Module Lifecycle**: Always start DIMOS before deploying modules
2. **LCM Topics**: Use descriptive topic names with namespaces
3. **Error Handling**: Wrap module operations in try-except blocks
4. **Resource Cleanup**: Ensure proper cleanup in stop() methods
5. **Async Operations**: Use asyncio for non-blocking operations
6. **Stream Management**: Use RxPy for reactive programming patterns

## Debugging Tips

1. **Check Module Status**: Print module.io().result() to see connections
2. **Monitor LCM**: Use Foxglove to visualize LCM messages
3. **Log Everything**: Use dimos.utils.logging_config.setup_logger()
4. **Test Modules Independently**: Deploy and test one module at a time

## Common Issues

1. **"Module not started"**: Ensure start() is called after deployment
2. **"No data received"**: Check LCM transport configuration
3. **"Connection failed"**: Verify input/output types match
4. **"Cleanup errors"**: Stop modules before closing DIMOS
