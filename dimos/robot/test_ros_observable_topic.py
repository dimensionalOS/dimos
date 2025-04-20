#!/usr/bin/env python3
import threading
import time
from nav_msgs import msg

from dimos.robot.ros_observable_topic import ROSObservableTopicAbility
from dimos.utils.logging_config import setup_logger


class MockROSNode:
    def __init__(self):
        self.logger = setup_logger("ROS")

        self.sub_id_cnt = 0
        self.subs = {}

    def _get_sub_id(self):
        sub_id = self.sub_id_cnt
        self.sub_id_cnt += 1
        return sub_id

    def create_subscription(self, msg_type, topic_name, callback, qos):
        # Mock implementation of ROS subscription

        sub_id = self._get_sub_id()
        stop_event = threading.Event()
        self.subs[sub_id] = stop_event
        self.logger.info(f"Subscribed {topic_name} subid {sub_id}")

        # Create message simulation thread
        def simulate_messages():
            message_count = 0
            while not stop_event.is_set():
                message_count += 1
                time.sleep(0.1)  # 20Hz default publication rate
                callback(message_count)
            # cleanup
            self.subs.pop(sub_id)

        thread = threading.Thread(target=simulate_messages, daemon=True)
        thread.start()
        return sub_id

    def destroy_subscription(self, subscription):
        if subscription in self.subs:
            self.subs[subscription].set()
            self.logger.info(f"Destroyed subscription: {subscription}")
        else:
            self.logger.info(f"Unknown subscription: {subscription}")


class MockRobot(ROSObservableTopicAbility):
    def __init__(self):
        self.logger = setup_logger("ROBOT")
        # Initialize the mock ROS node
        self._node = MockROSNode()


# This test verifies a bunch of basics:
#
# 1. that the system creates a single ROS sub for multiple reactivex subs
# 2. that the system creates a single ROS sub for multiple observers
# 3. that the system unsubscribes from ROS when observers are disposed
# 4. that the system replays the last message to new observers,
#    before the new ROS sub starts producing
def test_parallel_and_cleanup():
    robot = MockRobot()
    received_messages = []

    obs1 = robot.topic("/odom", msg.Odometry)
    print(f"Created subscription: {obs1}")

    subscription1 = obs1.subscribe(lambda x: received_messages.append(x + 2))
    subscription2 = obs1.subscribe(lambda x: received_messages.append(x + 3))

    obs2 = robot.topic("/odom", msg.Odometry)
    subscription3 = obs2.subscribe(lambda x: received_messages.append(x + 5))

    time.sleep(0.25)

    # We have 2 messages and 3 subscribers
    assert len(received_messages) == 6, "Should have received exactly 6 messages"

    #        [1, 1, 1, 2, 2, 2] +
    #        [2, 3, 5, 2, 3, 5]
    #        =
    for i in [3, 4, 6, 4, 5, 7]:
        assert i in received_messages, (
            f"Expected {i} in received messages, got {received_messages}"
        )

    # ensure that ROS end has only a single subscription
    assert len(robot._node.subs) == 1, (
        f"Expected 1 subscription, got {len(robot._node.subs)}: {robot._node.subs}"
    )

    subscription1.dispose()
    subscription2.dispose()
    subscription3.dispose()

    # Make sure that ros end was unsubscribed, thread terminated
    time.sleep(0.1)
    assert not robot._node.subs, f"Expected empty subs dict, got: {robot._node.subs}"

    # Ensure we replay the last message
    second_received = []
    second_sub = obs1.subscribe(lambda x: second_received.append(x))

    time.sleep(0.075)
    # we immediately receive the stored topic message
    assert len(second_received) == 1

    # now that sub is hot, we wait for a second one
    time.sleep(0.2)

    # we expect 2, 1 since first message was preserved from a previous ros topic sub
    # second one is the first message of the second ros topic sub
    assert second_received == [2, 1, 2]

    print(f"Second subscription immediately received {len(second_received)} message(s)")

    second_sub.dispose()

    time.sleep(0.1)
    assert not robot._node.subs, f"Expected empty subs dict, got: {robot._node.subs}"

    print("Test completed successfully")


# here we test parallel subs and slow observers hogging our topic
# we expect slow observers to skip messages by default
#
# ROS thread ─► ReplaySubject─► observe_on(pool) ─► backpressure.latest ─► sub1 (fast)
#                          ├──► observe_on(pool) ─► backpressure.latest ─► sub2 (slow)
#                          └──► observe_on(pool) ─► backpressure.latest ─► sub3 (slower)
def test_parallel_and_hog():
    robot = MockRobot()

    obs1 = robot.topic("/odom", msg.Odometry)
    obs2 = robot.topic("/odom", msg.Odometry)

    subscriber1_messages = []
    subscriber2_messages = []
    subscriber3_messages = []

    subscription1 = obs1.subscribe(lambda x: subscriber1_messages.append(x))
    subscription2 = obs1.subscribe(
        lambda x: time.sleep(0.15) or subscriber2_messages.append(x)
    )
    subscription3 = obs2.subscribe(
        lambda x: time.sleep(0.25) or subscriber3_messages.append(x)
    )

    assert len(robot._node.subs) == 1

    time.sleep(2)

    subscription1.dispose()
    subscription2.dispose()
    subscription3.dispose()

    print("Subscriber 1 messages:", len(subscriber1_messages), subscriber1_messages)
    print("Subscriber 2 messages:", len(subscriber2_messages), subscriber2_messages)
    print("Subscriber 3 messages:", len(subscriber3_messages), subscriber3_messages)

    assert len(subscriber1_messages) == 19
    assert len(subscriber2_messages) == 12
    assert len(subscriber3_messages) == 7

    assert subscriber2_messages[1] != [2]
    assert subscriber3_messages[1] != [2]

    time.sleep(0.1)

    assert robot._node.subs == {}


if __name__ == "__main__":
    test_parallel_and_cleanup()
    test_parallel_and_hog()



    # so skills can stream into the thought process.
