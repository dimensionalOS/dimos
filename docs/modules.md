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

# Deploy modules with initialization parameters
my_module = dimos.deploy(MyModule, param1="value1", param2=123)
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
dimos.close()
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

## Multiprocess Application Design  

1. @lesh can you add considerations here for best practices, issues, etc. One for example I will add is handling CUDA properly in multiprocess which is WIP. 

