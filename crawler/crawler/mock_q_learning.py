import random
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import torch
import torch.random
from crawler_msgs.msg import StateReward, Action, QLearningInternalState  # type: ignore


class MockQLearningNode(Node):
    def __init__(self):
        super().__init__("crawler_mock_q_learning")

        # parameters

        self.declare_parameter("arm_states", 3)
        self.arm_states = self.get_parameter("arm_states").get_parameter_value().integer_value

        self.declare_parameter("hand_states", 3)
        self.hand_states = self.get_parameter("hand_states").get_parameter_value().integer_value

        self.declare_parameter("arm_step", 200)
        self.arm_step = self.get_parameter("arm_step").get_parameter_value().integer_value

        self.declare_parameter("hand_step", 200)
        self.hand_step = self.get_parameter("hand_step").get_parameter_value().integer_value

        self.declare_parameter("learning_rate", 0.5)
        self.learning_rate = self.get_parameter("learning_rate").get_parameter_value().double_value

        self.declare_parameter("explor_rate", 1.0)
        self.explor_rate = self.get_parameter("explor_rate").get_parameter_value().double_value

        self.declare_parameter("explor_decay_rate", 0.05)
        self.explor_decay_rate = self.get_parameter("explor_decay_rate").get_parameter_value().double_value

        self.declare_parameter("max_explor_rate", 0.5)
        self.max_explor_rate = self.get_parameter("max_explor_rate").get_parameter_value().double_value

        self.declare_parameter("min_explor_rate", 0.01)
        self.min_explor_rate = self.get_parameter("min_explor_rate").get_parameter_value().double_value

        self.declare_parameter("discount_factor", 0.99)
        self.discount_factor = self.get_parameter("discount_factor").get_parameter_value().double_value

        # q table
        self.q_table = torch.rand(self.arm_states, self.hand_states, 4)

        # subscribers/publishers
        self.internals_publisher = self.create_publisher(QLearningInternalState, "/crawler/rl/q_learning/internals", 5)
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, 5)
        self.create_subscription(StateReward, "/crawler/rl/state_reward", self.receive_state_reward, 5)
        self.action_publisher = self.create_publisher(Action, "/crawler/rl/action", 5)

        self.create_publisher(Empty, "/crawler/rl/start", 5).publish(Empty())

        self.publish_internals()

    def stop(self, _):
        self.get_logger().info("Shutting down mock Q-learning node")
        self.destroy_node()
    
    def receive_state_reward(self, _):
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
        action.move_arm = random.randint(0, self.arm_states)
        action.move_hand = random.randint(0, self.hand_states)
        self.action_publisher.publish(action)

    def publish_internals(self):
        msg = QLearningInternalState()
        msg.arm_states = self.arm_states
        msg.hand_states = self.hand_states
        msg.arm_step = self.arm_step
        msg.hand_step = self.hand_step
        msg.learning_rate = self.learning_rate
        msg.explor_rate = self.explor_rate
        msg.explor_decay_rate = self.explor_decay_rate
        msg.max_explor_rate = self.max_explor_rate
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
