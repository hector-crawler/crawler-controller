import time
from datetime import datetime

import numpy as np
import rclpy
from crawler_msgs.msg import (  # type: ignore
    Action,
    QLearningInternalState,
    StateReward,
)
from keras import layers, model
from numpy import random as rand
from rclpy.node import Node
from std_msgs.msg import Empty, Int32

from .motors import Arm, Hand
from .move import Move

ARM_MOTOR_RANGE = Arm.max_limit - Arm.min_limit
HAND_MOTOR_RANGE = Hand.max_limit - Hand.min_limit


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
        self.declare_parameter("inner_layer_count", 1)
        self.inner_layer_count = (
            self.get_parameter("inner_layer_count").get_parameter_value().integer_value
        )

        self.declare_parameter("learning_rate", 0.5)
        self.learning_rate: float = (
            self.get_parameter("learning_rate").get_parameter_value().double_value
        )
        self.declare_parameter("explor_rate", 1.0)
        self.explor_rate: float = (
            self.get_parameter("explor_rate").get_parameter_value().double_value
        )
        self.declare_parameter("explor_decay_factor", 0.99)
        self.explor_decay_factor: float = (
            self.get_parameter("explor_decay_factor").get_parameter_value().double_value
        )
        self.declare_parameter("min_explor_rate", 0.01)
        self.min_explor_rate: float = (
            self.get_parameter("min_explor_rate").get_parameter_value().double_value
        )
        self.declare_parameter("discount_factor", 0.95)
        self.discount_factor: float = (
            self.get_parameter("discount_factor").get_parameter_value().double_value
        )

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

        self.moves_count = len(Move)
        self.last_move = Move.ARM_UP
        self.curr_arm_state = 0
        self.curr_hand_state = 0
        self.last_arm_state = 0
        self.last_hand_state = 0

        self.model = model.Sequential()
        self.model.add(
            layers.InputLayer(batch_input_shape=(self.arm_states, self.hand_states))
        )
        for _ in range(self.inner_layer_count):
            self.model.add(layers.Dense(self.inner_layer_width, activation="relu"))
        self.model.add(layers.Dense(self.moves_count, activation="linear"))
        self.model.compile(loss="mse", optimizer="adam", metrics=["mae"])

        self.last_predicts: np.ndarray = np.array([])

        self.get_logger().info(
            f"""
NN parameters:
    Arm states = {self.arm_states}
    Hand states = {self.hand_states}
    No. of Moves = {self.moves_count}

    Exploration rate = {self.explor_rate}
    Exploration decay factor = {self.explor_decay_factor}
    Min exploration rate = {self.min_explor_rate}
"""
        )

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
        if rand.random() < self.explor_rate:
            something_new = rand.choice(np.array(Move))
            self.get_logger().info(f"Randomly selected move {something_new}")
            return something_new

        self.last_predicts = model.predict((self.curr_arm_state, self.curr_hand_state))
        move = np.argmax(self.last_predicts)
        self.get_logger().info(f"Selected move {move}")
        return Move(move)

    def learn(self, rw: StateReward) -> None:
        target = rw.reward + self.discount_factor * np.max(self.last_predicts)
        target_vector = self.last_predicts[0]
        target_vector[self.last_move] = target
        model.fit(
            (self.curr_arm_state, self.curr_hand_state),
            target_vector.reshape(-1, self.moves_count),
            epochs=1,
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeuralNetworkNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
