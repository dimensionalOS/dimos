import threading
import time
import logging

from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from reactivex import operators as ops
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

from dimos.robot.robot import Robot
from dimos.utils.logging_config import setup_logger
from dimos.robot.global_planner.vector import VectorLike, to_vector, Vector
from dimos.robot.global_planner.costmap import Costmap
from dimos.robot.global_planner.path import Path
from dimos.robot.global_planner.algo import astar


logger = setup_logger("dimos.robot.unitree.global_planner", level=logging.DEBUG)


@dataclass
class Planner(ABC):
    robot: Robot

    @abstractmethod
    def plan(self, goal: VectorLike) -> Path: ...

    # actually we might want to rewrite this into rxpy
    def walk_loop(self, path: Path) -> bool:
        # pop the next goal from the path
        local_goal = path.head()
        print("path head", local_goal)
        result = self.robot.navigate_to_goal_local(
            local_goal.to_list(), is_robot_frame=False
        )

        if not result:
            # do we need to re-plan here?
            logger.warning("Failed to navigate to the local goal.")
            return False

        # get the rest of the path (btw here we can globally replan also)
        tail = path.tail()
        print("path tail", tail)
        if not tail:
            logger.info("Reached the goal.")
            return True

        # continue walking down the rest of the path
        # does python support tail calling haha?
        self.walk_loop(tail)

    def set_goal(self, goal: VectorLike):
        goal = to_vector(goal).to_2d()
        path = self.plan(goal)
        if not path:
            print("NO PATH FOUND")
            logger.warning("No path found to the goal.")
            return False

        return self.walk_loop(path)


def transform_to_euler(msg: TransformStamped) -> [Vector, Vector]:
    q = msg.transform.rotation
    rotation = R.from_quat([q.x, q.y, q.z, q.w])
    return [
        Vector(msg.transform.translation).to_2d(),
        Vector(rotation.as_euler("zyx", degrees=False)),
    ]


class AstarPlanner(Planner):
    def __init__(self, robot, algo_opts={}):
        super().__init__(robot)

        self.algo_opts = algo_opts
        base_link = self.robot.ros_control.transform("base_link")
        self.position = base_link.pipe(ops.map(transform_to_euler)).subscribe(
            self.newpos
        )

        self.costmap_thread()

    def newpos(self, pos):
        self.pos = pos

    def costmap_thread(self):
        def costmap_loop():
            while True:
                time.sleep(1)
                # this is a bit dumb tbh, we should have a stream :/
                costmap_msg = self.robot.ros_control.get_global_costmap()
                if costmap_msg is None:
                    continue
                self.costmap = Costmap.from_msg(costmap_msg).smudge(
                    preserve_unknown=True
                )

        threading.Thread(target=costmap_loop, daemon=True).start()

    def plan(self, goal: VectorLike) -> Path:
        time.sleep(3)
        return astar(self.costmap, goal, self.pos[0], **self.algo_opts)
