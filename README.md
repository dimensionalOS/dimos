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

<!-- TODO: Review / improve this description -->
## What is Dimensional?

Dimensional is an open-source framework for building agentive generalist robots. DimOS allows off-the-shelf Agents to call tools/functions and read sensor/state data directly from ROS.

The framework enables neurosymbolic orchestration of Agents as generalized spatial reasoners/planners and Robot state/action primitives as functions.

The result: cross-embodied *"Dimensional Applications"* exceptional at generalization and robust at symbolic action execution.

<!-- TODO: Review / improve Features -->
### Features

- **DimOS Agents**
  - Agent() classes with planning, spatial reasoning, and Robot.Skill() function calling abilities.
  - Integrate with any off-the-shelf hosted or local model: OpenAIAgent, ClaudeAgent, GeminiAgent 🚧, DeepSeekAgent 🚧, HuggingFaceRemoteAgent, HuggingFaceLocalAgent, etc.
  - Modular agent architecture for easy extensibility and chaining of Agent output --> Subagents input.
  - Agent spatial / language memory for location grounded reasoning and recall.

- **DimOS Infrastructure**
  - A reactive data streaming architecture using RxPY to manage real-time video (or other sensor input), outbound commands, and inbound robot state between the DimOS interface, Agents, and ROS2.
  - Robot Command Queue to handle complex multi-step actions to Robot.
  - Simulation bindings (Genesis, Isaacsim, etc.) to test your agentive application before deploying to a physical robot.

- **DimOS Interface / Development Tools**
  - Local development interface to control your robot, orchestrate agents, visualize camera/lidar streams, and debug your dimensional agentive application.

---

## Installation

```bash
# TODO: Ideally, when this is released, this should be as simple as
# pip install dimos
# I've commented out the installation instructions below
# because I feel like the *user*-oriented instructions
# should be simpler. But if we do want to just stick with these instructions, I can un-comment them (or maybe move them to a separate file and link to it).
```
<!-- TODO: Add whatever other system deps are needed for users (as opposed to contributors) -->
<!-- TODO: Move detailed installation to development.md or similar
## Python Installation
Tested on Ubuntu 22.04/24.04

```bash
sudo apt install python3-venv

# Clone the repository
git clone --branch dev --single-branch https://github.com/dimensionalOS/dimos.git
cd dimos

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

sudo apt install portaudio19-dev python3-pyaudio

# Install LFS
sudo apt install git-lfs
git lfs install

# Install torch and torchvision if not already installed
# Example CUDA 11.7, Pytorch 2.0.1 (replace with your required pytorch version if different)
pip install torch==2.0.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

#### Install dependencies
```bash
# CPU only (reccomended to attempt first)
pip install -e .[cpu,dev]

# CUDA install
pip install -e .[cuda,dev]

# Copy and configure environment variables
cp default.env .env
```

#### Test the install
```bash
pytest -s dimos/
```
-->

<!-- TODO: Move test commands to docs
#### Test Dimensional with a replay UnitreeGo2 stream (no robot required)
```bash
CONNECTION_TYPE=replay python dimos/robot/unitree_webrtc/unitree_go2.py
```

#### Test Dimensional with a simulated UnitreeGo2 in MuJoCo (no robot required)
```bash
pip install -e .[sim]
export DISPLAY=:1 # Or DISPLAY=:0 if getting GLFW/OpenGL X11 errors
CONNECTION_TYPE=mujoco python dimos/robot/unitree_webrtc/unitree_go2.py
```

#### Test Dimensional with a real UnitreeGo2 over WebRTC
```bash
export ROBOT_IP=192.168.X.XXX # Add the robot IP address
python dimos/robot/unitree_webrtc/unitree_go2.py
```

#### Test Dimensional with a real UnitreeGo2 running Agents
*OpenAI / Alibaba keys required*
```bash
export ROBOT_IP=192.168.X.XXX # Add the robot IP address
python dimos/robot/unitree_webrtc/run_agents2.py
```
-->

## Quickstart

Get started in minutes with our [Quickstart](./docs/quickstart.md): build an agentic robot that can make greetings!

<!-- TODO: Verify/update test file paths (some may have moved)
### Unitree Test Files
- **`tests/run_go2_ros.py`**: Tests `UnitreeROSControl(ROSControl)` initialization in `UnitreeGo2(Robot)` via direct function calls `robot.move()` and `robot.webrtc_req()`
- **`tests/simple_agent_test.py`**: Tests a simple zero-shot class `OpenAIAgent` example
- **`tests/unitree/test_webrtc_queue.py`**: Tests `ROSCommandQueue` via a 20 back-to-back WebRTC requests to the robot
- **`tests/test_planning_agent_web_interface.py`**: Tests a simple two-stage `PlanningAgent` chained to an `ExecutionAgent` with backend FastAPI interface.
- **`tests/test_unitree_agent_queries_fastapi.py`**: Tests a zero-shot `ExecutionAgent` with backend FastAPI interface.
-->

## Documentation

For detailed documentation, please visit our [documentation site](#) (Coming Soon)

## Contributing

We welcome contributions! See our [Bounty List](https://docs.google.com/spreadsheets/d/1tzYTPvhO7Lou21cU6avSWTQOhACl5H8trSvhtYtsk8U/edit?usp=sharing) for open requests for contributions. If you would like to suggest a feature or sponsor a bounty, open an issue.

<!-- TODO: Add a CONTRIBUTING.md -->

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
- Agent() failure to execute Nav2 action primitives (move, reverse, spinLeft, spinRight) is almost always due to the internal ROS2 collision avoidance, which will sometimes incorrectly display obstacles or be overly sensitive. Look for ```[behavior_server]: Collision Ahead - Exiting DriveOnHeading``` in the ROS logs. Reccomend restarting ROS2 or moving robot from objects to resolve.
- ```docker-compose up --build``` does not fully initialize the ROS2 environment due to ```std::bad_alloc``` errors. This will occur during continuous docker development if the ```docker-compose down``` is not run consistently before rebuilding and/or you are on a machine with less RAM, as ROS is very memory intensive. Reccomend running to clear your docker cache/images/containers with ```docker system prune``` and rebuild.
