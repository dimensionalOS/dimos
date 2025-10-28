# Dimensional Modules

The DimOS Module system enables distributed, multiprocess robotics applications using Dask for compute distribution and LCM (Lightweight Communications and Marshalling) for high-performance IPC.

## Core Concepts

### 1. Module Definition
Modules are Python classes that inherit from `dimos.core.Module` and define inputs, outputs, and RPC methods:

```python
from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import Vector3

class MyModule(Module):
    # Declare inputs/outputs as class attributes initialized to None
    data_in: In[Vector3] = None  
    data_out: Out[Vector3] = None

    def __init__():
        # Call parent Module init
        super().__init__()

    @rpc
    def remote_method(self, param):
        """Methods decorated with @rpc can be called remotely"""
        return param * 2
```

### 2. Module Deployment
Modules are deployed across Dask workers using the `dimos.deploy()` method:

```python
from dimos import core

# Start Dask cluster with N workers
dimos = core.start(4)

# Deploying modules allows for passing initialization parameters.
# In this case param1 and param2 are passed into Module init
module = dimos.deploy(Module, param1="value1", param2=123)
```

### 3. Stream Connections
Modules communicate via reactive streams using LCM transport:

```python
# Configure LCM transport for outputs
module1.data_out.transport = core.LCMTransport("/topic_name", MessageType)

# Connect module inputs to outputs
module2.data_in.connect(module1.data_out)

# Access the underlying Observable stream
stream = module1.data_out.observable()
stream.subscribe(lambda msg: print(f"Received: {msg}"))
```

### 4. Module Lifecycle
```python
# Start modules to begin processing
module.start()  # Calls the @rpc start() method if defined

# Inspect module I/O configuration  
print(module.io().result())  # Shows inputs, outputs, and RPC methods

# Clean shutdown
dimos.shutdown()
```

## Real-World Example: Robot Control System

```python
# Connection module wraps robot hardware/simulation
connection = dimos.deploy(ConnectionModule, ip=robot_ip)
connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
connection.video.transport = core.LCMTransport("/video", Image)

# Perception module processes sensor data
perception = dimos.deploy(PersonTrackingStream, camera_intrinsics=[...])
perception.video.connect(connection.video)
perception.tracking_data.transport = core.pLCMTransport("/person_tracking")

# Start processing
connection.start()
perception.start()

# Enable tracking via RPC
perception.enable_tracking()

# Get latest tracking data
data = perception.get_tracking_data()
```

## LCM Transport Configuration

```python
# Standard LCM transport for simple types like lidar
connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)

# Pickle-based transport for complex Python objects / dictionaries
connection.tracking_data.transport = core.pLCMTransport("/person_tracking")

# Auto-configure LCM system buffers (required in containers)
from dimos.protocol import pubsub
pubsub.lcm.autoconf()
```

This architecture enables building complex robotic systems as composable, distributed modules that communicate efficiently via streams and RPC, scaling from single machines to clusters.

# Dimensional Install 
## Python Installation (Ubuntu 22.04)

```bash
sudo apt install python3-venv

# Clone the repository (dev branch, no submodules)
git clone -b dev https://github.com/dimensionalOS/dimos.git
cd dimos

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

sudo apt install portaudio19-dev python3-pyaudio

# Install torch and torchvision if not already installed
# Example CUDA 11.7, Pytorch 2.0.1 (replace with your required pytorch version if different)
pip install torch==2.0.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Install dependencies
```bash
# CPU only (reccomended to attempt first)
pip install .[cpu,dev]

# CUDA install
pip install .[cuda,dev]

# Copy and configure environment variables
cp default.env .env
```

### Test install 
```bash 
# Run standard tests
pytest -s dimos/

# Test modules functionality
pytest -s -m module dimos/

# Test LCM communication
pytest -s -m lcm dimos/
```

# Unitree Go2 Quickstart

To quickly test the modules system, you can run the Unitree Go2 multiprocess example directly:

```bash
# Make sure you have the required environment variables set
export ROBOT_IP=<your_robot_ip>

# Run the multiprocess Unitree Go2 example
python dimos/robot/unitree_webrtc/multiprocess/unitree_go2.py
```


