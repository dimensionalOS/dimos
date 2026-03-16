"""Blueprint for the MuJoCo drone simulation.

Usage:
    from dimos.robot.drone.blueprints.sim.drone_sim import drone_sim_agentic
    drone_sim_agentic().build().loop()
"""

from dimos.agents.agent import agent
from dimos.agents.web_human_input import web_input
from dimos.core.blueprints import Blueprint, autoconnect
from dimos.robot.drone.sim import SimulatedDroneConnection
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

DRONE_SIM_SYSTEM_PROMPT = """\
You are controlling a simulated Skydio X2 quadrotor drone in a MuJoCo city environment.
You have access to these drone control skills:
- move(x, y, z, duration): Move with velocity (m/s). x=forward, y=right, z=up.
- move_with_yaw(vx, vy, vz, yaw_rate, duration): Move with velocity AND yaw rotation.
- takeoff(altitude): Takeoff to altitude in meters.
- land(): Land the drone.
- arm() / disarm(): Arm or disarm motors.
- set_mode(mode): Set flight mode (GUIDED, LAND, RTL, STABILIZE).
- fly_to(lat, lon, alt): Fly to GPS coordinates.

The drone is already hovering at 3m altitude, armed in GUIDED mode."""


def drone_sim_agentic(
    scene: str = "city",
    system_prompt: str = DRONE_SIM_SYSTEM_PROMPT,
    model: str = "gpt-4o",
    headless: bool = False,
) -> Blueprint:
    """Simulated drone with Nightwatch agent.

    Args:
        scene: Scene name under data/mujoco_sim/ (e.g. "city" loads city_scene.xml)
        system_prompt: Agent system prompt
        model: LLM model name
        headless: Skip camera rendering for faster simulation
    """
    return autoconnect(
        SimulatedDroneConnection.blueprint(
            scene=scene,
            headless=headless,
        ),
        WebsocketVisModule.blueprint(),
        agent(system_prompt=system_prompt, model=model),
        web_input(),
    )
