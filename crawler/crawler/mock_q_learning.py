import time

import rclpy
import torch  # type: ignore
import torch.random  # type: ignore
from crawler_msgs.msg import (  # type: ignore
    Action,
    QLearningInternalState,
    QLearningParameters,  # type: ignore
    StateReward,
)
from numpy import random
from rclpy.node import Node
from std_msgs.msg import Empty  # type: ignore


class MockQLearningNode(Node):
    def __init__(self):
        super().__init__("crawler_mock_q_learning")

        self.running = False

        # start subscriber
        self.create_subscription(
            QLearningParameters, "/crawler/rl/q_learning/start", self.start, 5
        )

        # subscribers/publishers
        self.internals_publisher = self.create_publisher(
            QLearningInternalState, "/crawler/rl/q_learning/internals", 5
        )
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, 5)
        self.create_subscription(
            StateReward, "/crawler/rl/state_reward", self.receive_state_reward, 5
        )
        self.action_publisher = self.create_publisher(Action, "/crawler/rl/action", 5)

    def start(self, parameters):
        # parameters
        self.arm_states = parameters.arm_states
        self.hand_states = parameters.hand_states
        self.arm_step = parameters.arm_step
        self.hand_step = parameters.hand_step
        self.learning_rate = parameters.learning_rate
        self.explor_rate = parameters.explor_rate
        self.explor_decay_rate = parameters.explor_decay_rate
        self.max_explor_rate = parameters.max_explor_rate
        self.min_explor_rate = parameters.min_explor_rate
        self.discount_factor = parameters.discount_factor

        # q table
        self.q_table = torch.rand(self.arm_states, self.hand_states, 4)

        # send start signal to environment
        self.create_publisher(Empty, "/crawler/rl/start", 5).publish(Empty())

        self.running = True
        self.publish_internals()

    def stop(self, _):
        self.get_logger().info("Stopping mock Q-learning node")
        self.running = False

    def receive_state_reward(self, _):
        if not self.running:
            return

        # randomly change a value in the Q-table
        row = random.randint(0, self.arm_states - 1)
        col = random.randint(0, self.hand_states - 1)
        action = random.randint(0, 3)
        self.q_table[row][col][action] = random.randint(0, 100) / 100.0

        self.publish_internals()

        time.sleep(1)
        self.publish_action()

    def publish_action(self):
        action = Action()
        action.move_arm = random.randint(-5, 5)
        action.move_hand = random.randint(-5, 5)
        self.action_publisher.publish(action)

    def publish_internals(self):
        msg = QLearningInternalState()
        msg.arm_states = self.arm_states
        msg.hand_states = self.hand_states
        msg.arm_step = self.arm_step
        msg.hand_step = self.hand_step
        msg.learning_rate = self.learning_rate
        msg.explor_rate = self.explor_rate
        msg.explor_decay_factor = self.explor_decay_factor
        msg.min_explor_rate = self.min_explor_rate
        msg.discount_factor = self.discount_factor

        msg.q_table_rows = []
        for i_arm in range(self.arm_states):
            for i_hand in range(self.hand_states):
                msg.q_table_rows.append(f"{i_arm}x{i_hand}")

        msg.q_table_cols = []
        for i_action in range(4):
            msg.q_table_cols.append(f"action {i_action}")

        msg.q_table_values = self.q_table.flatten().tolist()

        self.internals_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MockQLearningNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
