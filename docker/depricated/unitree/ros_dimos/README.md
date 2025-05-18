# Unitree Go2 ROS + DIMOS Movement Agents Docker Setup

This README explains how to run the Unitree Go2 ROS nodes with DIMOS integration using Docker.

## Prerequisites

- Docker and Docker Compose installed
- A Unitree Go2 robot accessible on your network
- The robot's IP address
- Python requirements installed (see root directory's requirements.txt)

## Configuration

1. Set environment variables in .env:
   ```bash
      ROBOT_IP=
      CONN_TYPE=webrtc
      WEBRTC_SERVER_HOST=0.0.0.0
      WEBRTC_SERVER_PORT=9991
      DISPLAY=:0
      ROS_OUTPUT_DIR=/app/assets/output/ros
   ```

2. Or run with environment variables in command line docker-compose:
   ```bash
   ROBOT_IP=192.168.9.140 CONN_TYPE=webrtc docker compose -f docker/unitree/ros_dimos/docker-compose.yml up --build
   ```

## Usage

To run the ROS nodes with DIMOS:

```bash
xhost +local:root # If running locally and desire RVIZ GUI
ROBOT_IP=<ROBOT_IP> CONN_TYPE=<webrtc/cyclonedds> docker compose -f docker/unitree/ros_dimos/docker-compose.yml up --build
```

Where:
- `<ROBOT_IP>` is your Go2's IP address
- `<webrtc/cyclonedds>` choose either:
  - `webrtc`: For WebRTC video streaming connection
  - `cyclonedds`: For DDS communication

The containers will build and start, establishing connection with your Go2 robot and opening RVIZ. The DIMOS integration will start 10 seconds after ROS to ensure proper initialization.

Note: You can run this command from any directory since the docker-compose.yml file handles all relative paths internally.

## Process Management

The setup uses supervisord to manage both ROS and DIMOS processes. To check process status or view logs when inside the container:

```bash
# Get a shell in the container
docker compose -f docker/unitree/ros_dimos/docker-compose.yml exec unitree_ros_dimos bash

# View process status
supervisorctl status

# View logs
supervisorctl tail ros2    # ROS2 logs
supervisorctl tail dimos   # DIMOS logs
supervisorctl tail -f ros2 # Follow ROS2 logs
```

## Known Issues

1. ROS2 doesn't have time to initialize before DIMOS starts, so the DIMOS logs will show successful aioice.ice:Connection followed by aiortc.exceptions.InvalidStateError. 

This is currently solved by hardcoding a delay between ros2 and DIMOS start in supervisord.conf. 

```ini
[lifecycle_manager-18] [INFO] [1740128988.350926960] [lifecycle_manager_navigation]: Managed nodes are active
[lifecycle_manager-18] [INFO] [1740128988.350965828] [lifecycle_manager_navigation]: Creating bond timer...
[go2_driver_node-3] INFO:scripts.webrtc_driver:Connection state is connecting
[go2_driver_node-3] INFO:aioice.ice:Connection(1) Discovered peer reflexive candidate Candidate(3hokvTUH7e 1 udp 2130706431 192.168.9.140 37384 typ prflx)
[go2_driver_node-3] INFO:aioice.ice:Connection(1) Check CandidatePair(('192.168.9.155', 33483) -> ('192.168.9.140', 37384)) State.WAITING -> State.IN_PROGRESS
[go2_driver_node-3] [INFO] [1740128990.171453153] [go2_driver_node]: Move
[go2_driver_node-3] INFO:scripts.webrtc_driver:Receiving video
[go2_driver_node-3] ERROR:asyncio:Task exception was never retrieved
[go2_driver_node-3] future: <Task finished name='Task-4' coro=<RobotBaseNode.run() done, defined at /ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/go2_robot_sdk/go2_driver_node.py:625> exception=InvalidStateError()>
[go2_driver_node-3] Traceback (most recent call last):
[go2_driver_node-3]   File "/ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/go2_robot_sdk/go2_driver_node.py", line 634, in run
[go2_driver_node-3]     self.joy_cmd(robot_num)
[go2_driver_node-3]   File "/ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/go2_robot_sdk/go2_driver_node.py", line 320, in joy_cmd
[go2_driver_node-3]     self.conn[robot_num].data_channel.send(
[go2_driver_node-3]   File "/usr/local/lib/python3.10/dist-packages/aiortc/rtcdatachannel.py", line 182, in send
[go2_driver_node-3]     raise InvalidStateError
[go2_driver_node-3] aiortc.exceptions.InvalidStateError
[go2_driver_node-3] Exception in thread Thread-1 (_spin):
[go2_driver_node-3] Traceback (most recent call last):
[go2_driver_node-3]   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
[go2_driver_node-3]     self.run()
[go2_driver_node-3]   File "/usr/lib/python3.10/threading.py", line 953, in run
[go2_driver_node-3]     self._target(*self._args, **self._kwargs)
[go2_driver_node-3]   File "/ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/go2_robot_sdk/go2_driver_node.py", line 646, in _spin
[go2_driver_node-3]     rclpy.spin_once(node)
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 203, in spin_once
[go2_driver_node-3]     executor = get_global_executor() if executor is None else executor
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 106, in get_global_executor
[go2_driver_node-3]     __executor = SingleThreadedExecutor()
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 721, in __init__
[go2_driver_node-3]     super().__init__(context=context)
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 172, in __init__
[go2_driver_node-3]     self._guard = GuardCondition(
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/guard_condition.py", line 23, in __init__
[go2_driver_node-3]     with self._context.handle:
[go2_driver_node-3] AttributeError: __enter__
[go2_driver_node-3] Exception ignored in: <function Executor.__del__ at 0x79dfdedc2c20>
[go2_driver_node-3] Traceback (most recent call last):
[go2_driver_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 243, in __del__
[go2_driver_node-3]     if self._sigint_gc is not None:
[go2_driver_node-3] AttributeError: 'SingleThreadedExecutor' object has no attribute '_sigint_gc'
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-16' coro=<RTCIceTransport._monitor() done, defined at /usr/local/lib/python3.10/dist-packages/aiortc/rtcicetransport.py:344> wait_for=<Future pending cb=[shield.<locals>._outer_done_callback() at /usr/lib/python3.10/asyncio/tasks.py:864, Task.task_wakeup()]>>
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-17' coro=<Connection.check_start() done, defined at /ros2_ws/install/go2_robot_sdk/share/go2_robot_sdk/external_lib/aioice/ice.py:789> wait_for=<Future pending cb=[Task.task_wakeup()]>>
[go2_driver_node-3] Exception ignored in: <coroutine object Go2Connection.on_track at 0x79dffafa85f0>
[go2_driver_node-3] Traceback (most recent call last):
[go2_driver_node-3]   File "/ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/scripts/webrtc_driver.py", line 229, in on_track
[go2_driver_node-3]     frame = await track.recv()
[go2_driver_node-3]   File "/usr/local/lib/python3.10/dist-packages/aiortc/rtcrtpreceiver.py", line 203, in recv
[go2_driver_node-3]     frame = await self._queue.get()
[go2_driver_node-3]   File "/usr/lib/python3.10/asyncio/queues.py", line 161, in get
[go2_driver_node-3]     getter.cancel()  # Just in case getter is not done yet.
[go2_driver_node-3]   File "/usr/lib/python3.10/asyncio/base_events.py", line 753, in call_soon
[go2_driver_node-3]     self._check_closed()
[go2_driver_node-3]   File "/usr/lib/python3.10/asyncio/base_events.py", line 515, in _check_closed
[go2_driver_node-3]     raise RuntimeError('Event loop is closed')
[go2_driver_node-3] RuntimeError: Event loop is closed
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-20' coro=<Go2Connection.on_track() done, defined at /ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/scripts/webrtc_driver.py:223> wait_for=<Future cancelled> cb=[AsyncIOEventEmitter._emit_run.<locals>.callback() at /usr/local/lib/python3.10/dist-packages/pyee/asyncio.py:95]>
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-21' coro=<RTCPeerConnection.__connect() done, defined at /usr/local/lib/python3.10/dist-packages/aiortc/rtcpeerconnection.py:1008> wait_for=<Future pending cb=[Task.task_wakeup()]>>
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-2' coro=<spin() running at /ros2_ws/install/go2_robot_sdk/lib/python3.10/site-packages/go2_robot_sdk/go2_driver_node.py:655> wait_for=<Future pending cb=[Task.task_wakeup()]>>
[go2_driver_node-3] ERROR:asyncio:Task was destroyed but it is pending!
[go2_driver_node-3] task: <Task pending name='Task-15' coro=<RTCPeerConnection.__connect() running at /usr/local/lib/python3.10/dist-packages/aiortc/rtcpeerconnection.py:1016> wait_for=<Future pending cb=[Task.task_wakeup()]>>
[INFO] [go2_driver_node-3]: process has finished cleanly [pid 120]
```


2. If you encounter the error `unitree_ros_dimos-1  | exec /entrypoint.sh: no such file or directory`, this can be caused by:
   - Incorrect file permissions
   - Windows-style line endings (CRLF) in the entrypoint script

   To fix:
   1. Ensure the entrypoint script has execute permissions:
      ```bash
      chmod +x /path/to/dimos/docker/unitree/ros_dimos/entrypoint.sh
      ```
   
   2. If using Windows, convert line endings to Unix format (LF):
      ```bash
      # Using dos2unix
      dos2unix /path/to/dimos/docker/unitree/ros_dimos/entrypoint.sh
      
      # Or using sed
      sed -i 's/\r$//' /path/to/dimos/docker/unitree/ros_dimos/entrypoint.sh
      ```

2. If DIMOS fails to start, check:
   - The ROS nodes are fully initialized (wait a few seconds)
   - The environment variables are properly set
   - The Python path includes the dimos directory
   - The logs using supervisorctl for specific error messages 