#!/usr/bin/env python3
"""MuJoCo drone simulation with interactive 3D viewer + DimOS Nightwatch.

Opens a MuJoCo window with a Skydio X2 quadrotor in a city scene.
DimOS agent (Nightwatch) runs alongside — give natural language commands
at localhost:5555 and watch the drone move in the 3D viewer.

Usage:
    .venv/bin/python dimos/robot/drone/mujoco_sim.py
"""

import json
import math
import select
import sys
import termios
import threading
import time
import tty

import mujoco
import mujoco.viewer
import numpy as np

from dimos.robot.drone.sim import GPS_ORIGIN_LAT, GPS_ORIGIN_LON, EARTH_R, _local_to_gps
from dimos.simulation.mujoco.policy import DroneController
from dimos.utils.data import get_data
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3


class DimOSBridge:
    """Bridges MuJoCo sim ↔ DimOS: publishes state, receives agent commands."""

    def __init__(self, controller: DroneController) -> None:
        self.controller = controller
        self._odom_pub = None
        self._gps_pub = None
        self._status_pub = None
        self._cmd_sub = None
        self._agent_thread = None
        self._running = False

    def start(self) -> None:
        import socket as _socket

        from dimos.robot.drone.mujoco_skill_proxy import CMD_PORT

        self._cmd_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._cmd_sock.bind(("127.0.0.1", CMD_PORT))
        self._cmd_sock.setblocking(False)

        self._running = True
        self._agent_thread = threading.Thread(target=self._start_agent, daemon=True)
        self._agent_thread.start()

        self._cmd_thread = threading.Thread(target=self._cmd_listen_loop, daemon=True)
        self._cmd_thread.start()

        print(f"[DimOS] Bridge started — listening on UDP {CMD_PORT}, agent at http://localhost:5555")

    def _start_agent(self) -> None:
        try:
            from dimos.agents.agent import agent as make_agent
            from dimos.agents.web_human_input import web_input
            from dimos.core.blueprints import autoconnect
            from dimos.robot.drone.blueprints.sim.drone_sim import DRONE_SIM_SYSTEM_PROMPT as DRONE_SYSTEM_PROMPT
            from dimos.robot.drone.mujoco_skill_proxy import MuJocoSkillProxy
            from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

            bp = autoconnect(
                MuJocoSkillProxy.blueprint(),
                WebsocketVisModule.blueprint(),
                make_agent(system_prompt=DRONE_SYSTEM_PROMPT, model="gpt-4o"),
                web_input(),
            )
            bp.build().loop()
        except Exception as e:
            import traceback
            print(f"[DimOS] Agent startup error: {e}")
            traceback.print_exc()

    def _cmd_listen_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._cmd_sock.recvfrom(4096)
                cmd = json.loads(data.decode())
                self._handle_cmd(cmd)
            except BlockingIOError:
                time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def _handle_cmd(self, cmd: dict) -> None:
        t = cmd.get("type")

        if t == "velocity":
            vx = float(cmd.get("vx", 0))
            vy = float(cmd.get("vy", 0))
            vz = float(cmd.get("vz", 0))
            yaw = float(cmd.get("yaw_rate", 0))
            dur = float(cmd.get("duration", 0))
            self.controller.set_velocity(vx, vy, vz, yaw)
            print(f"[Agent] velocity: vx={vx} vy={vy} vz={vz} yaw={yaw} dur={dur}s")
            if dur > 0:
                threading.Timer(dur, lambda: self.controller.set_velocity(0, 0, 0, 0)).start()

        elif t == "takeoff":
            alt = float(cmd.get("altitude", 3.0))
            self.controller.set_velocity(0, 0, 1.5, 0)
            print(f"[Agent] takeoff to {alt}m")

        elif t == "land":
            self.controller.set_velocity(0, 0, -0.5, 0)
            print("[Agent] landing")

        elif t == "arm":
            print("[Agent] armed")

        elif t == "disarm":
            self.controller.set_velocity(0, 0, 0, 0)
            print("[Agent] disarmed")

        elif t == "set_mode":
            print(f"[Agent] mode → {cmd.get('mode')}")

        elif t == "fly_to":
            lat = float(cmd.get("lat", 0))
            lon = float(cmd.get("lon", 0))
            dx = math.radians(lat - GPS_ORIGIN_LAT) * EARTH_R
            dy = -(math.radians(lon - GPS_ORIGIN_LON) * EARTH_R * math.cos(math.radians(GPS_ORIGIN_LAT)))
            pos = self.controller.data.xpos[self.controller.body_id]
            ex, ey = dx - pos[0], dy - pos[1]
            dist = math.sqrt(ex**2 + ey**2)
            if dist > 0.5:
                speed = min(2.0, dist)
                self.controller.set_velocity(speed * ex / dist, speed * ey / dist, 0, 0)
            print(f"[Agent] fly_to {lat:.6f}, {lon:.6f}")

    def publish_state(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if self._odom_pub is None:
            return

        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "x2")
        pos = data.xpos[body_id]
        quat_mj = data.xquat[body_id]
        vel = data.cvel[body_id]

        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        now = time.time()

        pose = PoseStamped(
            position=Vector3(x, y, z),
            orientation=Quaternion(float(quat_mj[1]), float(quat_mj[2]), float(quat_mj[3]), float(quat_mj[0])),
            frame_id="world",
            ts=now,
        )
        self._odom_pub.publish(pose)

        lat, lon = _local_to_gps(x, y)
        self._gps_pub.publish(LatLon(lat=lat, lon=lon))

        siny = 2.0 * (quat_mj[0] * quat_mj[3] + quat_mj[1] * quat_mj[2])
        cosy = 1.0 - 2.0 * (quat_mj[2] ** 2 + quat_mj[3] ** 2)
        heading_deg = math.degrees(math.atan2(siny, cosy)) % 360

        from dimos_lcm.std_msgs import String
        status = {
            "armed": True, "mode": "GUIDED", "altitude": z,
            "heading": heading_deg, "lat": lat, "lon": lon,
            "vx": float(vel[3]), "vy": float(vel[4]), "vz": float(vel[5]),
            "simulator": "mujoco", "ts": now,
        }
        self._status_pub.publish(String(json.dumps(status)))

    def stop(self) -> None:
        self._running = False


def main() -> None:
    scene_path = str(get_data("mujoco_sim") / "city_scene.xml")
    print(f"Loading model: {scene_path}")

    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)  # compute xquat from qpos before controller reads it

    controller = DroneController(model, data)

    bridge = DimOSBridge(controller)
    bridge.start()

    armed = True
    mode = "GUIDED"
    step_count = 0
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "x2")

    # Keyframe already at z=3 hover — no takeoff needed, just hold position
    controller.set_velocity(0, 0, 0, 0)
    print(">> Hovering at 3.0m — ready for commands")

    print("""
╔═══════════════════════════════════════════════╗
║   MuJoCo Drone + DimOS Nightwatch             ║
╠═══════════════════════════════════════════════╣
║  3D Viewer: MuJoCo window                     ║
║  AI Agent:  http://localhost:5555              ║
║                                               ║
║  Terminal:  t=takeoff l=land w/s/a/d/q/e/r/f  ║
║            x=stop  ESC=quit                   ║
╚═══════════════════════════════════════════════╝
    """)

    def physics_step(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        if armed and mode == "GUIDED":
            d.ctrl[:] = controller.compute_control()
        elif armed:
            d.ctrl[:] = controller.hover_thrust
        else:
            d.ctrl[:] = 0.0

    mujoco.set_mjcb_control(physics_step)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -20.0
        viewer.cam.lookat[:] = [0, 0, 1.5]

        # Terminal keyboard input (only works in a real TTY)
        use_keyboard = sys.stdin.isatty()
        old_settings = None
        if use_keyboard:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        try:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                step_count += 1
                viewer.sync()

                viewer.cam.lookat[:] = data.xpos[body_id]

                # Publish to DimOS at ~30Hz
                if step_count % max(1, int(1.0 / model.opt.timestep / 30)) == 0:
                    bridge.publish_state(model, data)

                # Terminal keyboard
                if use_keyboard and select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    cmds = {
                        't': (lambda: (controller.set_velocity(0, 0, 1.5, 0), print(">> Takeoff"))),
                        'l': (lambda: (controller.set_velocity(0, 0, -0.5, 0), print(">> Landing"))),
                        'w': (lambda: (controller.set_velocity(2.0, 0, 0, 0), print(">> Forward"))),
                        's': (lambda: (controller.set_velocity(-2.0, 0, 0, 0), print(">> Backward"))),
                        'a': (lambda: (controller.set_velocity(0, 0, 0, -1.0), print(">> Yaw left"))),
                        'd': (lambda: (controller.set_velocity(0, 0, 0, 1.0), print(">> Yaw right"))),
                        'q': (lambda: (controller.set_velocity(0, -2.0, 0, 0), print(">> Strafe left"))),
                        'e': (lambda: (controller.set_velocity(0, 2.0, 0, 0), print(">> Strafe right"))),
                        'r': (lambda: (controller.set_velocity(0, 0, 1.0, 0), print(">> Alt up"))),
                        'f': (lambda: (controller.set_velocity(0, 0, -1.0, 0), print(">> Alt down"))),
                        'x': (lambda: (controller.set_velocity(0, 0, 0, 0), print(">> Stop"))),
                    }
                    if key == '\x1b':
                        break
                    elif key in cmds:
                        cmds[key]()

                time.sleep(max(0, model.opt.timestep - 0.001))

        finally:
            if old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            mujoco.set_mjcb_control(None)
            bridge.stop()
            print("Simulation stopped.")


if __name__ == "__main__":
    main()
