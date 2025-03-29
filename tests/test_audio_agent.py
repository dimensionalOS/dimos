from dimos.stream.audio.utils import keepalive
from dimos.stream.audio.pipelines import tts, stt
from dimos.utils.threadpool import get_scheduler
from dimos.agents.agent import OpenAIAgent


def main():

    stt_node = stt()

    agent = OpenAIAgent(
        dev_name="UnitreeExecutionAgent",
        input_query_stream=stt_node.emit_text(),
        system_query="You are a helpful robot named daneel that does my bidding",
        pool_scheduler=get_scheduler(),
    )

    tts_node = tts()
    tts_node.consume_text(agent.get_response_observable())

    # Keep the main thread alive
    keepalive()


if __name__ == "__main__":
    main()
