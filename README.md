![Screenshot 2025-02-18 at 16-31-22 DimOS Terminal](/assets/dimos_terminal.png)

<div align="center">
  <table>
    <tr>
      <td width="80%">
        <img src="./assets/dimos_interface.gif" alt="dimOS interface" width="100%">
        <p align="center"><em>A simple two-shot PlanningAgent</em></p>
      </td>
      <td width="20%">
        <img src="./assets/simple_demo_small.gif" alt="3rd person POV" width="100%">
        <p align="center"><em>3rd person POV</em></p>
      </td>
    </tr>
  </table>
</div>

# The Dimensional Framework
*The universal framework for AI-native generalist robotics*

## What is Dimensional?

Dimensional is an open-source framework for building intelligent robots. DimOS allows off-the-shelf robots to call tools/functions and read sensor/state data directly. It is both a conceptual approach compatible with many programming languages and well as a high performance python-first library that works well with (and without) [ROS](https://www.ros.org/). The framework comes with a rich set of integrations including spatial reasoners, planners, simulators (mujoco, Isaac Sim, etc.), robot state/action primitives, and more.

The result: the best toolkit for creating symbolic action execution on platforms like the Unitree G1, Unitree Go2 and Boston Dynamics Spot.

<!-- ## DIMOS x Unitree Go2 (OUT OF DATE)

We are shipping a first look at the DIMOS x Unitree Go2 integration, allowing for off-the-shelf Agents() to "call" Unitree ROS2 Nodes and WebRTC action primitives, including:

- Navigation control primitives (move, reverse, spinLeft, spinRight, etc.)
- WebRTC control primitives (FrontPounce, FrontFlip, FrontJump, etc.)
- Camera feeds (image_raw, compressed_image, etc.)
- IMU data
- State information
- Lidar / PointCloud primitives
- Any other generic Unitree ROS2 topics

-->

<!--
FIXME: add a list of features based on what we've actually documented

### Features

- **DimOS Modules**
    -
- **DimOS Agents**
  - Agent() classes with planning, spatial reasoning, and Robot.Skill() function calling abilities.
  - Integrate with any off-the-shelf hosted or local model: OpenAIAgent, ClaudeAgent, GeminiAgent 🚧, DeepSeekAgent 🚧, HuggingFaceRemoteAgent, HuggingFaceLocalAgent, etc.
  - Modular agent architecture for easy extensibility and chaining of Agent output -> Subagents input.
  - Agent spatial / language memory for location grounded reasoning and recall.

- **DimOS Infrastructure**
  - A reactive data streaming architecture using RxPY to manage real-time video (or other sensor input), outbound commands, and inbound robot state between the DimOS interface, Agents, and ROS2.
  - Robot Command Queue to handle complex multi-step actions to Robot.
  - Simulation bindings (Genesis, Isaacsim, etc.) to test your agentive application before deploying to a physical robot.

- **DimOS Interface / Development Tools**
  - Local development interface to control your robot, orchestrate agents, visualize camera/lidar streams, and debug your dimensional agentive application.
-->

## How can I start using DimOS?

### Installation

- **MacOS**: MacOS support is currently alpha. You can install the same as Linux, but know that you WILL get flakey behavior while running dimos.
- **Linux**:
  1. Get a quick start with the [dimos-template](https://github.com/dimensionalOS/dimos-template) and follow the instructions in the README.
  2. If you have an existing python project, cd to it then run this: `sh <(curl -fsSL "https://raw.githubusercontent.com/dimensionalOS/dimos/refs/heads/main/bin/install")`

DimOS supports Docker, Nix, and normal Python Venv installations.

### Usage

#### Test Dimensional with a replay UnitreeGo2 stream (no robot required)
```bash
dimos --replay run unitree-go2
```

#### Test Dimensional with a simulated UnitreeGo2 in MuJoCo (no robot required)
```bash
pip install -e '.[sim]'
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
dimos --simulation run unitree-go2
```

#### Test Dimensional with a real UnitreeGo2 over WebRTC
```bash
export ROBOT_IP=192.168.X.XXX # Add the robot IP address
dimos run unitree-go2
```

#### Test Dimensional with a real UnitreeGo2 running Agents
*OpenAI / Alibaba keys required*
```bash
export ROBOT_IP=192.168.X.XXX # Add the robot IP address
dimos run unitree-go2-agentic
```

---

### Agent API keys

Full functionality will require API keys for the following:

Requirements:
- OpenAI API key (required for all LLMAgents due to OpenAIEmbeddings)
- Claude API key (required for ClaudeAgent)
- Alibaba API key (required for Navigation skills)

These keys can be added to your .env file or exported as environment variables.
```
export OPENAI_API_KEY=<your private key>
export CLAUDE_API_KEY=<your private key>
export ALIBABA_API_KEY=<your private key>
```

### ROS2 Unitree Go2 SDK Installation

#### System Requirements
- Ubuntu 22.04
- ROS2 Distros: Iron, Humble, Rolling

See [Unitree Go2 ROS2 SDK](https://github.com/dimensionalOS/go2_ros2_sdk) for additional installation instructions.

```bash
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/dimensionalOS/go2_ros2_sdk.git src
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs

sudo apt install python3-pip clang portaudio19-dev
cd src
pip install -r requirements.txt
cd ..

# Ensure clean python install before running
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Run the test application

#### ROS2 Terminal:
```bash
# Change path to your Go2 ROS2 SDK installation
source /ros2_ws/install/setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash

export ROBOT_IP="robot_ip" #for muliple robots, just split by ,
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py

```

#### Python Terminal:
```bash
# Change path to your Go2 ROS2 SDK installation
source /ros2_ws/install/setup.bash
python tests/run.py
```

#### DimOS Interface:
```bash
cd dimos/web/dimos_interface
yarn install
yarn dev # you may need to run sudo if previously built via Docker
```

<!-- ## Building

### Simple DimOS Application (OUT OF DATE)

```python
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.agents_deprecated.agent import OpenAIAgent

# Initialize robot
robot = UnitreeGo2(
    ip=robot_ip,
    ros_control=UnitreeROSControl(),
    skills=MyUnitreeSkills()
)

# Initialize agent
agent = OpenAIAgent(
    dev_name="UnitreeExecutionAgent",
    input_video_stream=robot.get_ros_video_stream(),
    skills=robot.get_skills(),
    system_query="Jump when you see a human! Front flip when you see a dog!",
    model_name="gpt-4o"
)

while True: # keep process running
    time.sleep(1)
```


### DimOS Application with Agent chaining (OUT OF DATE)

Let's build a simple DimOS application with Agent chaining. We define a ```planner``` as a ```PlanningAgent``` that takes in user input to devise a complex multi-step plan. This plan is passed step-by-step to an ```executor``` agent that can queue ```AbstractRobotSkill``` commands to the ```ROSCommandQueue```.

Our reactive Pub/Sub data streaming architecture allows for chaining of ```Agent_0 -> Agent_1 -> ... -> Agent_n``` via the ```input_query_stream``` parameter in each which takes an ```Observable``` input from the previous Agent in the chain.

**Via this method you can chain together any number of Agents() to create complex dimensional applications.**

```python

web_interface = RobotWebInterface(port=5555)

robot = UnitreeGo2(ip=robot_ip,
                  ros_control=UnitreeROSControl(),
                  skills=MyUnitreeSkills())

# Initialize master planning agent
planner = PlanningAgent(
            dev_name="UnitreePlanningAgent",
            input_query_stream=web_interface.query_stream, # Takes user input from dimOS interface
            skills=robot.get_skills(),
            model_name="gpt-4o",
        )

# Initialize execution agent
executor = OpenAIAgent(
            dev_name="UnitreeExecutionAgent",
            input_query_stream=planner.get_response_observable(), # Takes planner output as input
            skills=robot.get_skills(),
            model_name="gpt-4o",
            system_query="""
            You are a robot execution agent that can execute tasks on a virtual
            robot. ONLY OUTPUT THE SKILLS TO EXECUTE.
            """
        )

while True: # keep process running
  time.sleep(1)
```

### Calling Action Primitives (OUT OF DATE)

Call action primitives directly from ```Robot()``` for prototyping and testing.

```python
robot = UnitreeGo2(ip=robot_ip,)

# Call a Unitree WebRTC action primitive
robot.webrtc_req(api_id=1016) # "Hello" command

# Call a ROS2 action primitive
robot.move(distance=1.0, speed=0.5)
```
 -->
<!--
### Creating Custom Skills (non-unitree specific)

#### Create basic custom skills by inheriting from ```AbstractRobotSkill``` and implementing the ```__call__``` method.

```python
class Move(AbstractRobotSkill):
    distance: float = Field(...,description="Distance to reverse in meters")
    def __init__(self, robot: Optional[Robot] = None, **data):
        super().__init__(robot=robot, **data)
    def __call__(self):
        super().__call__()
        return self._robot.move(distance=self.distance)
```

#### Chain together skills to create recursive skill trees

```python
class JumpAndFlip(AbstractRobotSkill):
    def __init__(self, robot: Optional[Robot] = None, **data):
        super().__init__(robot=robot, **data)
    def __call__(self):
        super().__call__()
        jump = Jump(robot=self._robot)
        flip = Flip(robot=self._robot)
        return (jump() and flip())
```

### Integrating Skills with Agents: Single Skills and Skill Libraries

DimOS agents, such as `OpenAIAgent`, can be endowed with capabilities through two primary mechanisms: by providing them with individual skill classes or with comprehensive `SkillLibrary` instances. This design offers flexibility in how robot functionalities are defined and managed within your agent-based applications.

**Agent's `skills` Parameter**

The `skills` parameter in an agent's constructor is key to this integration:

1.  **A Single Skill Class**: This approach is suitable for skills that are relatively self-contained or have straightforward initialization requirements.
    *   You pass the skill *class itself* (e.g., `GreeterSkill`) directly to the agent's `skills` parameter.
    *   The agent then takes on the responsibility of instantiating this skill when it's invoked. This typically involves the agent providing necessary context to the skill's constructor (`__init__`), such as a `Robot` instance (or any other private instance variable) if the skill requires it.

2.  **A `SkillLibrary` Instance**: This is the preferred method for managing a collection of skills, especially when skills have dependencies, require specific configurations, or need to share parameters.
    *   You first define your custom skill library by inheriting from `SkillLibrary`. Then, you create and configure an *instance* of this library (e.g., `my_lib = EntertainmentSkills(robot=robot_instance)`).
    *   This pre-configured `SkillLibrary` instance is then passed to the agent's `skills` parameter. The library itself manages the lifecycle and provision of its contained skills.

**Examples:**

#### 1. Using a Single Skill Class with an Agent

First, define your skill. For instance, a `GreeterSkill` that can deliver a configurable greeting:

```python
class GreeterSkill(AbstractSkill):
    """Greats the user with a friendly message.""" # Gives the agent better context for understanding (the more detailed the better).

    greeting: str = Field(..., description="The greating message to display.") # The field needed for the calling of the function. Your agent will also pull from the description here to gain better context.

    def __init__(self, greeting_message: Optional[str] = None, **data):
        super().__init__(**data)
        if greeting_message:
            self.greeting = greeting_message
        # Any additional skill-specific initialization can go here

    def __call__(self):
        super().__call__() # Call parent's method if it contains base logic
        # Implement the logic for the skill
        print(self.greeting)
        return f"Greeting delivered: '{self.greeting}'"
```

Next, register this skill *class* directly with your agent. The agent can then instantiate it, potentially with specific configurations if your agent or skill supports it (e.g., via default parameters or a more advanced setup).

```python
agent = OpenAIAgent(
    dev_name="GreetingBot",
    system_query="You are a polite bot. If a user asks for a greeting, use your GreeterSkill.",
    skills=GreeterSkill,  # Pass the GreeterSkill CLASS
    # The agent will instantiate GreeterSkill.
    # If the skill had required __init__ args not provided by the agent automatically,
    # this direct class passing might be insufficient without further agent logic
    # or by passing a pre-configured instance (see SkillLibrary example).
    # For simple skills like GreeterSkill with defaults or optional args, this works well.
    model_name="gpt-4o"
)
```
In this setup, when the `GreetingBot` agent decides to use the `GreeterSkill`, it will instantiate it. If the `GreeterSkill` were to be instantiated by the agent with a specific `greeting_message`, the agent's design would need to support passing such parameters during skill instantiation.

#### 2. Using a `SkillLibrary` Instance with an Agent

Define the SkillLibrary and any skills it will manage in its collection:
```python
class MovementSkillsLibrary(SkillLibrary):
    """A specialized skill library containing movement and navigation related skills."""

    def __init__(self, robot=None):
        super().__init__()
        self._robot = robot

    def initialize_skills(self, robot=None):
        """Initialize all movement skills with the robot instance."""
        if robot:
            self._robot = robot

        if not self._robot:
            raise ValueError("Robot instance is required to initialize skills")

        # Initialize with all movement-related skills
        self.add(Navigate(robot=self._robot))
        self.add(NavigateToGoal(robot=self._robot))
        self.add(FollowHuman(robot=self._robot))
        self.add(NavigateToObject(robot=self._robot))
        self.add(GetPose(robot=self._robot))  # Position tracking skill
```

Note the addision of initialized skills added to this collection above.

Proceed to use this skill library in an Agent:

Finally, in your main application code:
```python
# 1. Create an instance of your custom skill library, configured with the robot
my_movement_skills = MovementSkillsLibrary(robot=robot_instance)

# 2. Pass this library INSTANCE to the agent
performing_agent = OpenAIAgent(
    dev_name="ShowBot",
    system_query="You are a show robot. Use your skills as directed.",
    skills=my_movement_skills,  # Pass the configured SkillLibrary INSTANCE
    model_name="gpt-4o"
)
```

### Unitree Test Files
- **`tests/run_go2_ros.py`**: Tests `UnitreeROSControl(ROSControl)` initialization in `UnitreeGo2(Robot)` via direct function calls `robot.move()` and `robot.webrtc_req()`
- **`tests/simple_agent_test.py`**: Tests a simple zero-shot class `OpenAIAgent` example
- **`tests/unitree/test_webrtc_queue.py`**: Tests `ROSCommandQueue` via a 20 back-to-back WebRTC requests to the robot
- **`tests/test_planning_agent_web_interface.py`**: Tests a simple two-stage `PlanningAgent` chained to an `ExecutionAgent` with backend FastAPI interface.
- **`tests/test_unitree_agent_queries_fastapi.py`**: Tests a zero-shot `ExecutionAgent` with backend FastAPI interface.
-->

## Documentation

<!-- FIXME: -->

## Contributing

We welcome contributions! See our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Huge thanks to!
- The Roboverse Community and their unitree-specific help. Check out their [Discord](https://discord.gg/HEXNMCNhEh).
- @abizovnuralem for his work on the [Unitree Go2 ROS2 SDK](https://github.com/abizovnuralem/go2_ros2_sdk) we integrate with for DimOS.
- @legion1581 for his work on the [Unitree Go2 WebRTC Connect](https://github.com/legion1581/go2_webrtc_connect) from which we've pulled the ```Go2WebRTCConnection``` class and other types for seamless WebRTC-only integration with DimOS.
- @tfoldi for the webrtc_req integration via Unitree Go2 ROS2 SDK, which allows for seamless usage of Unitree WebRTC control primitives with DimOS.

## Contact

- GitHub Issues: For bug reports and feature requests
- Email: [build@dimensionalOS.com](mailto:build@dimensionalOS.com)

## Known Issues
<!-- FIXME: update this section -->
<!-- - Agent() failure to execute Nav2 action primitives (move, reverse, spinLeft, spinRight) is almost always due to the internal ROS2 collision avoidance, which will sometimes incorrectly display obstacles or be overly sensitive. Look for ```[behavior_server]: Collision Ahead - Exiting DriveOnHeading``` in the ROS logs. Reccomend restarting ROS2 or moving robot from objects to resolve.
- ```docker-compose up --build``` does not fully initialize the ROS2 environment due to ```std::bad_alloc``` errors. This will occur during continuous docker development if the ```docker-compose down``` is not run consistently before rebuilding and/or you are on a machine with less RAM, as ROS is very memory intensive. Reccomend running to clear your docker cache/images/containers with ```docker system prune``` and rebuild. -->
