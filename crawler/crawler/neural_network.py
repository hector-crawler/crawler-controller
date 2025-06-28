import time
from datetime import datetime

import rclpy
import torch
from crawler_msgs.msg import (  # type: ignore
    Action,
    QLearningInternalState,
    StateReward,
)
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from torch import nn, optim

from .motors import Arm, Hand
from .move import Move

ARM_MOTOR_RANGE = Arm.max_limit - Arm.min_limit
HAND_MOTOR_RANGE = Hand.max_limit - Hand.min_limit


class NN(nn.Module):
    def __init__(self, inner_size: int) -> None:
        super().__init__()

        inputs = 3
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(inputs, inner_size),
            nn.ReLU(),
            nn.Linear(inner_size, inner_size),
            nn.ReLU(),
            nn.Linear(inner_size, len(Move)),
        )

    def forward(self, x):
        return self.linear_relu_stack(nn.Flatten()(x))


class NeuralNetworkNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_neural_network")

        # Parameters regarding the motors
        self.declare_parameter("arm_states", 3)
        self.arm_states = (
            self.get_parameter("arm_states").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_states", 3)
        self.hand_states = (
            self.get_parameter("hand_states").get_parameter_value().integer_value
        )
        self.declare_parameter("arm_step", 200)
        self.arm_step = (
            self.get_parameter("arm_step").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_step", 200)
        self.hand_step = (
            self.get_parameter("hand_step").get_parameter_value().integer_value
        )

        # Parameters regarding the Neural Network
        self.declare_parameter("inner_layer_width", 100)
        self.inner_layer_width = (
            self.get_parameter("inner_layer_width").get_parameter_value().integer_value
        )
        self.declare_parameter("learning_rate", 0.5)
        self.learning_rate = (
            self.get_parameter("learning_rate").get_parameter_value().double_value
        )

        self.get_logger().info(f"""
NN parameters:
    Learning rate: {self.learning_rate}
""")

        queue_len = 5
        self.internal_state_publisher = self.create_publisher(
            QLearningInternalState, "/crawler/rl/q_learning/internals", queue_len
        )
        self.create_timer(1.0, self.publish_internal_state)

        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, queue_len)
        self.create_subscription(
            StateReward, "/crawler/rl/state_reward", self.get_reward, queue_len
        )

        self.create_subscription(
            Int32, "/crawler/arm/position", self.set_arm_pos, queue_len
        )
        self.create_subscription(
            Int32, "/crawler/hand/position", self.set_hand_pos, queue_len
        )

        self.action_publisher = self.create_publisher(
            Action, "/crawler/rl/action", queue_len
        )

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using {device} device for the neural network")
        self.model = NN(inner_size=self.inner_layer_width).to(device)
        self.get_logger().info(f"Model: {self.model}")
        # Maybe use optim.SGD?
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)

        self.moves_count = len(Move)
        self.last_move = Move.ARM_UP
        self.curr_arm_state = 0
        self.curr_hand_state = 0
        self.last_arm_state = 0
        self.last_hand_state = 0

        time.sleep(0.5)
        self.create_publisher(Empty, "/crawler/rl/start", queue_len).publish(Empty())

    def publish_internal_state(self) -> None:
        msg = QLearningInternalState()
        msg.param_a = self.arm_states
        msg.param_b = self.hand_states
        msg.timestamp = datetime.now().isoformat()
        self.internal_state_publisher.publish(msg)

    def stop(self, _) -> None:
        self.get_logger().info("Shutting down Neural Network node")
        self.destroy_node()

    def set_arm_pos(self, msg) -> None:
        self.curr_arm_state = int(msg.data * self.arm_states / ARM_MOTOR_RANGE)

    def set_hand_pos(self, msg) -> None:
        self.curr_hand_state = int(msg.data * self.hand_states / HAND_MOTOR_RANGE)

    def get_reward(self, msg) -> None:
        self.learn(msg.data)
        self.last_move = self.pick_move()
        self.send_move(self.last_move)

    def send_move(self, m: Move) -> None:
        match m:
            case Move.ARM_UP:
                self.action_publisher.publish(Action(self.arm_step, 0))
            case Move.ARM_DOWN:
                self.action_publisher.publish(Action(-self.arm_step, 0))
            case Move.HAND_UP:
                self.action_publisher.publish(Action(0, self.hand_step))
            case Move.HAND_DOWN:
                self.action_publisher.publish(Action(0, -self.hand_step))

    def pick_move(self) -> Move:
        self.last_outputs = self.model(
            [self.curr_arm_state, self.curr_hand_state, self.last_move.value]
        )
        return Move(torch.argmax(self.last_outputs))

    def learn(self, rw: StateReward) -> None:
        nn.CrossEntropyLoss()(self.last_outputs, rw.reward).backward()
        self.optimizer.step()
        self.optimizer.zero_grad()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeuralNetworkNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
