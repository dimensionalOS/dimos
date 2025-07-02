## DimOS Modules & Capability Protocol System

New plug-and-play modules and capability protocols for general Robots. 

### 1  Concepts

We use Python `typing.Protocol` for declaring **capabilities**. Because every protocol is annotated with `@runtime_checkable`, you can safely inspect a connection object at runtime:

```python
if isinstance(conn, Move):
    conn.move(Vector(0.2, 0, 0), duration=1.0)
```

This powers the `REQUIRES` filtering inside `@robot_capability` – modules only attach when the underlying `conn` implements every required protocol.

| Term | Meaning |
|------|---------|
| **Capability** | A lightweight `Protocol` (e.g. `Move`, `Lidar`, `Video`) implemented by a _connection interface_ (WebRTC, ROS, Gazebo, …). |
| **Module** | A self-contained feature (planner, mapper, perception, …) that declares the capabilities it needs and is attached to the robot at runtime. |
| **robot_module** | Decorator / helper marking a Module class as a DIMOS module. Adds a default `setup(robot)` hook and registers the name. |
| **robot_capability** | Decorator for `Robot` subclasses. Auto-instantiates the listed modules after `__init__` if the underlying connection provides the required capabilities. |

### 2  Writing a Module

```python
from dimos.robot.module_utils import robot_module
from dimos.robot.capabilities import Video, Lidar

@robot_module
class 3DObjectDetection:
    REQUIRES = (Video,Lidar)

    def setup(self, robot):
        self.robot = robot          # backlink
        self.video_stream = robot.video_stream()
        self.lidar_stream = robot.lidar_stream()
        # … run detection on incoming frames …
```

### 3  Building a Simple Robot with a Module

```python
from dimos.robot.module_utils import robot_capability
from dimos.robot.robot import Robot
from dimos.robot.unitree_webrtc.connection import WebRTCRobot
from mypkg.perception import ObjectDetection

@robot_capability(ObjectDetection)
class MyGo2(Robot):
    def __init__(self, ip: str):
        conn = WebRTCRobot(ip=ip)
        super().__init__(connection_interface=conn)
```

Instantiation:

```python
robot = MyGo2(ip="192.168.1.10")
obj_det = robot.get_module(ObjectDetection)
```

### 4  Capability Protocols & Writing a Connection Interface

All capabilities live in `dimos/robot/capabilities.py`.  They are plain `Protocol` classes decorated with `@runtime_checkable`, so any object that *structurally* matches the method signatures automatically fulfils the capability.

Key protocols (excerpt):
```python
@runtime_checkable
class Move(Protocol):
    def move(self, velocity: Vector, duration: float = 0.0) -> bool: ...
    def stop(self) -> bool: ...

@runtime_checkable
class Video(Protocol):
    def video_stream(self, **kwargs) -> Optional[Observable]: ...
```

You wire those into a transport / driver class via the lightweight `@implements` decorator:

```python
from dimos.robot.capabilities import implements, Move, Video, Connection
from dimos.types.vector import Vector
from reactivex.subject import Subject

@implements(Move, Video)
class SimConnection(Connection):
    ip = "sim://localhost"

    def connect(self):
        print("connected to simulator")

    # --- Move capability
    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        print(f"moving {velocity} for {duration}s")
        return True

    def stop(self):
        print("stopping")
        return True

    # --- Video capability
    def video_stream(self):
        return Subject()  # push fake frames here

    # Optional: satisfy minimal Connection protocol
    def disconnect(self):
        print("disconnect")
```

Because `SimConnection` advertises `Move` and `Video`, any module that declares `REQUIRES = (Move, Video)` will auto-attach when the robot is built with this connection.

---

### 5  Switching Connection Back-Ends

Because modules specify capabilities, you can plug different connection providers:

```python
if use_sim:
    conn = SimConnection()
else:
    conn = WebRTCRobot(ip="192.168.1.10")

robot = MyGo2(conn)  # same robot & modules, new backend
```

The framework aborts early if a provider lacks a required capability.

### 6  Quick Test

`tests/run_robot_clean.py` demonstrates a minimal stack:

```
python tests/run_robot_clean.py
```

Basic agent and movement commands work but still some small bugs with the new UnitreeWebRTCConnection interface as we abstract everthing out of what used to be in Robot.py. 
