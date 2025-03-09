# Unitree API Server

This is a minimal FastAPI server implementation that provides API endpoints for the terminal frontend.

## Quick Start

```bash
# Navigate to the api directory
cd api

# Install minimal requirements
pip install -r requirements.txt

# Run the server
python unitree_server.py
```

The server will start on `http://localhost:5555`.

## Integration with Frontend

1. Start the API server as described above
2. In another terminal, run the frontend from the root directory:
   ```bash
   cd ..  # Navigate to root directory (if you're in api/)
   yarn dev
   ```
3. Use the `unitree` command in the terminal interface:
   - `unitree status` - Check the API status
   - `unitree command <text>` - Send a command to the API

## Integration with DIMOS Agents

See DimOS Documentation for more info. 

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

## API Endpoints

- **GET /unitree/status**: Check the status of the Unitree API
- **POST /unitree/command**: Send a command to the Unitree API

## How It Works

The frontend and backend are separate applications:

1. The Svelte frontend runs on port 3000 via Vite
2. The FastAPI backend runs on port 5555
3. Vite's development server proxies requests from `/unitree/*` to the FastAPI server
4. The `unitree` command in the terminal interface sends requests to these endpoints

This architecture allows the frontend and backend to be developed and run independently. 