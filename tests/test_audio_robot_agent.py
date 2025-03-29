from dimos.utils.threadpool import get_scheduler
import os
from dimos.robot.unitree.unitree_go2 import UnitreeGo2
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
from dimos.agents.agent import OpenAIAgent
from dimos.stream.audio.pipelines import tts, stt
from dimos.stream.audio.utils import keepalive


def main():
    stt_node = stt()
    tts_node = tts()

    robot = UnitreeGo2(
        ip=os.getenv("ROBOT_IP"),
        ros_control=UnitreeROSControl(),
        skills=MyUnitreeSkills(),
    )

    # Initialize agent with main thread pool scheduler
    agent = OpenAIAgent(
        dev_name="UnitreeExecutionAgent",
        input_query_stream=stt_node.emit_text(),
        system_query="You are a helpful robot named daneel that does my bidding",
        pool_scheduler=get_scheduler(),
        skills=robot.get_skills(),
    )

    tts_node.consume_text(agent.get_response_observable())

    # Keep the main thread alive
    keepalive()


if __name__ == "__main__":
    main()
