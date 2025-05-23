import datetime
import random as rand
from enum import Enum

import rclpy
import torch
from crawler_msgs.msg import (  # type: ignore
    Action,
    QLearningInternalState,
    StateReward,
)
from rclpy.node import Node
from std_msgs.msg import Empty, Int32 

from .motors import Arm, Hand

ARM_MOTOR_RANGE = Arm.max_limit - Arm.min_limit
HAND_MOTOR_RANGE = Hand.max_limit - Hand.min_limit


class Move(Enum):
    ARM_UP = 1
    ARM_DOWN = 2
    HAND_UP = 3
    HAND_DOWN = 4


class QLearningNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

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

        # Parameters regarding the Q-Learning
        self.declare_parameter("learning_rate", 0.5)
        self.learning_rate = (
            self.get_parameter("learning_rate").get_parameter_value().double_value
        )
        self.declare_parameter("explor_rate", 1.0)
        self.explor_rate = (
            self.get_parameter("explor_rate").get_parameter_value().double_value
        )
        self.declare_parameter("explor_decay_rate", 0.05)
        self.explor_decay_rate = (
            self.get_parameter("explor_decay_rate").get_parameter_value().double_value
        )
        self.declare_parameter("max_explor_rate", 0.5)
        self.max_explor_rate = (
            self.get_parameter("max_explor_rate").get_parameter_value().double_value
        )
        self.declare_parameter("min_explor_rate", 0.01)
        self.min_explor_rate = (
            self.get_parameter("min_explor_rate").get_parameter_value().double_value
        )
        self.declare_parameter("discount_factor", 0.99)
        self.discount_factor = (
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

        # We might also want to use torch.rand() for initialization.
        self.q_table = torch.zeros(
            # At this point we might also think about adding another dimension for self.last_move
            [self.arm_states, self.hand_states, self.moves_count]
            # We might also want to investigate changing the dtype parameter for our usecase.
            # https://docs.pytorch.org/docs/stable/tensor_attributes.html#torch.dtype
        )

        self.get_logger().info(
            f"""
Q-learning parameters:
    Arm states = {self.arm_states}
    Hand states = {self.hand_states}
    No. of Moves = {self.moves_count}

    Exploration rate = {self.explor_rate}
    Exploration decay rate = {self.explor_decay_rate}
    Min exploration rate = {self.min_explor_rate}
    Max exploration rate = {self.max_explor_rate}
"""
        )

        self.create_publisher(Empty, "/crawler/rl/start", queue_len).publish(Empty())

    def set_arm_pos(self, msg) -> None:
        self.curr_arm_state = int(msg.data * self.arm_states / ARM_MOTOR_RANGE)

    def set_hand_pos(self, msg) -> None:
        self.curr_hand_state = int(msg.data * self.hand_states / HAND_MOTOR_RANGE)

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
        if rand.uniform(0, 1) < self.explor_rate:
            something_new = rand.choice(list(Move))
            return something_new

        pool = self.q_table[self.curr_arm_state][self.curr_hand_state]
        move_idx = torch.argmax(pool).item() + 1
        move = Move(move_idx)
        return move

    def get_reward(self, msg) -> None:
        reward = msg.data
        self.learn(reward)
        m = self.pick_move()
        self.last_move = m
        self.send_move(m)

    def learn(self, rw: StateReward) -> None:
        idx = [
            self.last_arm_state,
            self.last_hand_state,
            self.last_move.value - 1,
        ]
        predicted_value = self.q_table[idx]
        target_value = (
            rw.reward
            + self.discount_factor
            * self.q_table[self.curr_arm_state, self.curr_hand_state].max()
        )
        self.q_table[idx] = predicted_value + self.learning_rate * (
            target_value - predicted_value
        )
        self.last_arm_state = self.curr_arm_state
        self.last_hand_state = self.curr_hand_state

    def publish_internal_state(self) -> None:
        msg = QLearningInternalState()
        msg.param_a = self.arm_states
        msg.param_b = self.hand_states
        msg.timestamp = datetime.datetime.now().isoformat()
        self.internal_state_publisher.publish(msg)

    def stop(self, _) -> None:
        self.get_logger().info("Shutting down Q-learning node")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
