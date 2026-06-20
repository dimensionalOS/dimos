"""Agentic XLeRobot blueprint — LLM agent control via MCP.

Composes on top of xlerobot_basic and adds MCP server/client
for agent-driven control of the XLeRobot. Includes all skills:
base movement, dual-arm manipulation, head, grippers, and vision.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.xlerobot.blueprints.basic.xlerobot_basic import xlerobot_basic

XLEROBOT_SYSTEM_PROMPT = """\
You are controlling an XLeRobot — a 3-wheel omni-directional mobile robot with dual 6-DOF arms and a 2-DOF head.

# AVAILABLE SKILLS

## Base Movement
- move_base(x, y, theta, duration): Drive the base. x=forward m/s, y=left m/s, theta=rotation deg/s.
- stop_base(): Emergency stop all wheel motors.

## Arms & Manipulation
- move_arm(arm, joint_positions, duration): Move "left" or "right" arm. joint_positions is a JSON string mapping joint names to target values.
  Left arm joints: left_arm_shoulder_pan, left_arm_shoulder_lift, left_arm_elbow_flex, left_arm_wrist_flex, left_arm_wrist_roll, left_arm_gripper
  Right arm joints: right_arm_shoulder_pan, right_arm_shoulder_lift, right_arm_elbow_flex, right_arm_wrist_flex, right_arm_wrist_roll, right_arm_gripper
- open_gripper(arm): Open gripper on "left" or "right" arm.
- close_gripper(arm): Close gripper on "left" or "right" arm.
- home_arms(): Move both arms and head to zero position.
- get_joint_positions(): Get all current joint angles as JSON.

## Head
- move_head(pan, tilt): Point the head camera.

## Vision
- observe(): Get the latest camera frame.

When given a task, plan movements and confirm results with observe()."""

xlerobot_agentic = autoconnect(
    xlerobot_basic,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=XLEROBOT_SYSTEM_PROMPT),
    WebInput.blueprint(),
)

__all__ = ["XLEROBOT_SYSTEM_PROMPT", "xlerobot_agentic"]
