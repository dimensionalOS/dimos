# DimOS Graph-of-Modules Multiprocess Runtime

Documentation on new parallel **LCM / graph-of-modules** architecture. This is a necessary backbone for our AI-first agentive robotics operating system. 

## 1. Conceptual Layers

```
┌────────────────────────────┐
│  YAML configuration file   │  (declarative – what the system should run)
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│  Robot class (orchestrator)│  builds the graph, loads modules, wires edges
└────────────┬───────────────┘
             ▼
┌────────────────────────────┐
│ ConnectionModule           │  the single hardware interface
│ – implements capability    │  protocols (Move, Video, Lidar, …)
│ – exposes Rx-streams as    │  Out ports (LCM topics)             ▲
└────────────▲───────────────┘                                     │ capability
             │ consumes In ports (movecmd, etc.)                   │ protocols
             │                                                     │
┌────────────┴───────────────┐             ┌───────────────────────┴─────────────┐
│  "Logic Modules"           │ …graph…     │  LCM transport layer                │
│  (AstarPlanner, VFH, …)    │────────────▶│  (local/remote processes)           │
└────────────────────────────┘             └──────────────────────────────────────┘
```

### Where Capability Protocols Live

* **ConnectionModule** is the *only* object that talks to the robot hardware, so **it implements everything**: `Move`, `Video`, `Lidar`, `Odometry`, …  
  (You can still keep a tiny `BaseConnection` ABC for type-checking.)
* `Robot` **does not** implement protocols; it simply forwards calls (`robot.move → connection.move`) so existing non-LCM code keeps working.
* Logic modules **declare** `REQUIRES` (e.g. `Move, Lidar`) exactly as they do today – the decorator/runtime will satisfy those by pointing them at the ConnectionModule instance.

This gives you:

```python
if has_capability(robot.conn, Lidar):
    # works in classic monolith
if has_capability(connection_module, Lidar):
    # works in pure-LCM graph (module talks directly)
```

## 2. YAML-based Configuration

`robot_capability` already accepts per-module kwargs; you can externalise them:

```yaml
# unitree.yaml
modules:
  spatialmemory:
    collection_name: spatial_mem_test
    new_memory: false
    db_path: ~/maps/chroma
  astarplanner:
    conservativism: 10
  vfhpurepursuitplanner:
    max_linear_vel: 0.6
    lookahead_distance: 1.2
  wavefrontfrontierexplorer:
    min_frontier_size: 20
```

Usage in classic mode:

```python
robot = UnitreeGo2(ip="192.168.1.100", config_file="unitree.yaml")
```

Usage in LCM graph:

```python
dimos.deploy(AstarPlanner, config_file="unitree.yaml")   # planner reads its section
```

Because the decorator already merges `module_config_file` → `module_config`, both runtimes can share the same YAML.

## 3. LCM Graph Example (Simplified)

```python
# --- nodes -------------------------------------------------
conn   = dimos.deploy(ConnectionModule, ip="ROBOT_IP")
mapper = dimos.deploy(Map, voxel_size=0.5)
gplan  = dimos.deploy(AstarPlanner, config_file="unitree.yaml")
lplan  = dimos.deploy(VFHPurePursuitPlanner)

# --- wiring (edges) ---------------------------------------
# raw sensors
mapper.lidar    .connect(conn.lidar)    # Out ➜ In
gplan.target    .connect(ctrl.plancmd)

# capability lambdas for modules that expect callables
lplan.get_costmap  = conn.get_local_costmap
lplan.get_robot_pos= conn.get_pos
gplan.get_costmap  = mapper.costmap
gplan.get_robot_pos= conn.get_pos
lplan.movecmd      .connect(conn.movecmd)
```

LCM topics are assigned simply by attaching a `Transport`:

```python
conn.lidar .transport = core.LCMTransport("/lidar",  LidarMessage)
conn.odom  .transport = core.LCMTransport("/odom",   Odometry)
gplan.path .transport = core.LCMTransport("/gpath",  Path)
```

The same module class therefore works in-process (RxPY only) and across processes (LCM).

## 4. Example Module Class with IN/OUT Ports

```python
class FollowPerson(Module):
    target    : In[Vector3]      = None      # from perception
    movecmd   : Out[Vector3]     = None      # to Connection

    async def start(self):
        self.target.pipe(
            ops.map(lambda v: self._simple_controller(v))
        ).subscribe(self.movecmd.publish)

    def _simple_controller(self, v: Vector3) -> Vector3:
        # ...compute desired velocity...
        return Vector3([0.2, 0.0, 0.0])
```

A YAML fragment could instantiate and wire it:

```yaml
modules:
  followperson:
    _type: FollowPerson
    inputs:
      target: "/person_pos"     # subscribe to LCM topic
    outputs:
      movecmd: "/follow_vel"    # publish velocity
    params:
      kp: 1.2
```

## 5. Initialisation Flow Summary

1. **Robot class** loads YAML → list of module specs.  
2. It instantiates **ConnectionModule** first (provides capabilities).  
3. Dependency resolver orders remaining modules, injects dependencies.  
4. For each module:  
    a. If it has `In/Out` attributes, Robot wires streams / LCM transports.  
    b. Calls `setup(robot)` (current pattern) then `start()` (async) if it inherits from `Module`.  
5. Classic helper functions (`navigate_path_local`) continue to work because Robot exposes `robot.local_planner`, etc.

## Why This Blends Cleanly with Future LCM

• Capability protocols stay exactly where hardware lives (ConnectionModule).  
• Logic modules stay ignorant of transport; they just receive Observables or call injected lambdas.  
• Switching to multi-process is a deployment concern: attach `LCMTransport` to an Out stream and it "just works".  
• YAML gives ops/field engineers a single file to tweak parameters without changing code.

Feel free to adjust naming (`ConnectionModule`, `Robot`, etc.); the pattern above will remain valid. 

**7. Full Implementation Example**

```python
@robot_capability(AstarPlanner, VFHPurePursuitPlanner)
class UnitreeGo2(Robot):
    def __init__(self, ip: str):
        super().__init__(UnitreeWebRTCConnection(ip))

robot = UnitreeGo2(ip="192.168.1.100")
print("modules:", list(robot._modules))
```
The decorator auto-instantiates the planners; nothing else is required.

---
**8. ConnectionModule with Capability Protocols**

```python
@implements(Move, Lidar, Odometry, Video)
class ConnectionModule(UnitreeWebRTCConnection, Module):
    movecmd : In [Vector]        = None
    lidar   : Out[LidarMessage]  = None
    odom    : Out[Odometry]      = None
    video   : Out[VideoMessage]  = None

    async def start(self):
        self.lidar_stream().subscribe(self.lidar.publish)
        self.odom_stream() .subscribe(self.odom.publish)
        self.video_stream().subscribe(self.video.publish)
        self.movecmd.subscribe(self.move)   # respond to velocity commands
```
This single class now both **implements** the runtime capability protocols *and* exposes LCM/Rx streams for the graph runtime. Any logic module that declares `REQUIRES = (Move, Lidar)` will be wired against this instance automatically. 