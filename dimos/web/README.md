# DimOS Robot Web Interface

A streamlined interface for controlling and interacting with robots through DimOS.

## Setup

First, create an `.env` file in the root dimos directory with your configuration:

```bash
# Example .env file
OPENAI_API_KEY=sk-your-openai-api-key
ROBOT_IP=192.168.x.x
CONN_TYPE=webrtc
WEBRTC_SERVER_HOST=0.0.0.0
WEBRTC_SERVER_PORT=9991
DISPLAY=:0
```

## Unitree Go2 Example

Running a full stack for Unitree Go2 requires three components:

### 1. Start ROS2 Robot Driver

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/your_ros_workspace/install/setup.bash

# Launch robot driver
ros2 launch go2_robot_sdk robot.launch.py
```

### 2. Start DimOS Backend

```bash
# In a new terminal, source your Python environment
source venv/bin/activate  # Or your environment

# Install requirements
pip install -r requirements.txt

# Source ROS workspace (needed for robot communication)
source /opt/ros/humble/setup.bash
source ~/your_ros_workspace/install/setup.bash

# Run the server with Robot() and Agent() initialization
python tests/test_unitree_agent_queries_fastapi.py
```

### 3. Start Frontend

**Install yarn if not already installed**

```bash
npm install -g yarn
```

**Then install dependencies and start the development server**

```bash
# In a new terminal
cd dimos/web/dimos-interface

# Install dependencies (first time only)
yarn install

# Start development server
yarn dev
```

The frontend will be available at http://localhost:3000

## Using the Interface

1. Access the web terminal at http://localhost:3000
2. Type commands to control your robot:
   - `unitree command <your instruction>` - Send a command to the robot
   - `unitree status` - Check connection status
   - `unitree start_stream` - Start the video stream
   - `unitree stop_stream` - Stop the video stream

## Integrating DimOS with the DimOS-interface

### Unitree Go2 Example

```python
from dimos.agents.agent import OpenAIAgent
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.web.robot_web_interface import RobotWebInterface

robot_ip = os.getenv("ROBOT_IP")

# Initialize robot
logger.info("Initializing Unitree Robot")        
robot = UnitreeGo2(ip=robot_ip,
                    connection_method=connection_method,
                    output_dir=output_dir)

# Set up video stream
logger.info("Starting video stream")
video_stream = robot.get_ros_video_stream()

# Create FastAPI server with video stream
logger.info("Initializing FastAPI server")
streams = {"unitree_video": video_stream}
web_interface = RobotWebInterface(port=5555, **streams)

# Initialize agent with robot skills
skills_instance = MyUnitreeSkills(robot=robot)

agent = OpenAIAgent(
    dev_name="UnitreeQueryPerceptionAgent",
    input_query_stream=web_interface.query_stream,
    output_dir=output_dir,
    skills=skills_instance,
)

web_interface.run()
```

## Architecture

- **Backend**: FastAPI server runs on port 5555
- **Frontend**: Web application runs on port 3000
