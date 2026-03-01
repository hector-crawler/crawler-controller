from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from crawler_msgs.msg import (
    Action,
    StateReward,
)
from numpy import random as rand
from rclpy.node import Node
from std_msgs.msg import Empty, Int32

from crawler.move import MOVES_COUNT, Move, MoveMode

QUEUE_LEN = 5


class AlgorithmNode(ABC, Node):
    def __init__(self, node_name, pretty_name) -> None:
        super().__init__(node_name)
        self.pretty_name = pretty_name

        # parameters for arm/hand limits
        self.declare_parameter("arm_min_limit", 900)
        self.arm_min_limit = (
            self.get_parameter("arm_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("arm_max_limit", 1500)
        self.arm_max_limit = (
            self.get_parameter("arm_max_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_min_limit", 2900)
        self.hand_min_limit = (
            self.get_parameter("hand_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_max_limit", 3700)
        self.hand_max_limit = (
            self.get_parameter("hand_max_limit").get_parameter_value().integer_value
        )

        self.create_subscription(
            Int32, "/crawler/arm/position", self.receive_arm_pos, QUEUE_LEN
        )
        self.create_subscription(
            Int32, "/crawler/hand/position", self.receive_hand_pos, QUEUE_LEN
        )

        self.action_publisher = self.create_publisher(
            Action, "/crawler/rl/action", QUEUE_LEN
        )

        self.running = False
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, QUEUE_LEN)
        self.create_subscription(
            StateReward, "/crawler/rl/state_reward", self.get_reward, QUEUE_LEN
        )

        self.last_move = Move.ARM_UP
        self.curr_arm_state = 0
        self.curr_hand_state = 0
        self.last_arm_state = 0
        self.last_hand_state = 0

        self.running = False
        self.move_is_exploration = False
        self.create_timer(0.2, self.publish_internal_state)

    def start(self, params):
        # parameters
        self.arm_states = params.arm_states
        self.hand_states = params.hand_states
        self.arm_step = params.arm_step
        self.hand_step = params.hand_step

        self.learning_rate = params.learning_rate
        self.explor_rate = params.explor_rate
        self.explor_decay_factor = params.explor_decay_factor
        self.min_explor_rate = params.min_explor_rate
        self.discount_factor = params.discount_factor

        self.get_logger().info(
            f"""
RL Algorithm parameters:
    Arm states = {self.arm_states}
    Hand states = {self.hand_states}
    No. of Moves = {MOVES_COUNT}

    Learning rate = {self.learning_rate}
    Exploration rate = {self.explor_rate}
    Exploration decay factor = {self.explor_decay_factor}
    Min exploration rate = {self.min_explor_rate}
    Discount factor = {self.discount_factor}
"""
        )

        self.move_mode = MoveMode.AUTOMATIC
        if params.initial_move_mode_wait:
            self.move_mode = MoveMode.USER_WAIT
        self.waiting_for_user_move = False

        self.running = True
        self.create_publisher(Empty, "/crawler/rl/start", QUEUE_LEN).publish(Empty())

    def set_move_mode(self, msg) -> None:
        if msg.data not in [mode.name for mode in MoveMode]:
            self.get_logger().error(f"Unknown move mode {msg.data}!")
            return

        self.move_mode = MoveMode[msg.data]
        self.get_logger().info(f"Set move mode to {self.move_mode}")
        if self.waiting_for_user_move:
            self.pick_move_and_send()
            self.waiting_for_user_move = False
        self.publish_internal_state()

    def receive_arm_pos(self, msg) -> None:
        if not self.running:
            return
        discrete_value = int(
            (msg.data - self.arm_min_limit)
            / (self.arm_max_limit - self.arm_min_limit)
            * self.arm_states
        )
        self.curr_arm_state = min(max(discrete_value, 0), self.arm_states - 1)

    def receive_hand_pos(self, msg) -> None:
        if not self.running:
            return
        discrete_value = int(
            (msg.data - self.hand_min_limit)
            / (self.hand_max_limit - self.hand_min_limit)
            * self.hand_states
        )
        self.curr_hand_state = min(max(discrete_value, 0), self.hand_states - 1)

    def send_move(self, m: Move) -> None:
        act = Action()
        match m:
            case Move.ARM_UP:
                act.move_arm = self.arm_step
                act.move_hand = 0
            case Move.ARM_DOWN:
                act.move_arm = -self.arm_step
                act.move_hand = 0
            case Move.HAND_UP:
                act.move_arm = 0
                act.move_hand = self.hand_step
            case Move.HAND_DOWN:
                act.move_arm = 0
                act.move_hand = -self.hand_step
        self.action_publisher.publish(act)

    def pick_move_and_send(self) -> None:
        if not self.running:
            return
        move = self.pick_move()
        if move is not None:
            self.last_move = move
            self.send_move(self.last_move)

    def pick_move(self) -> Optional[Move]:
        match self.move_mode:
            case MoveMode.USER_WAIT:
                self.waiting_for_user_move = True
            case MoveMode.USER_ARM_UP:
                self.move_mode = MoveMode.USER_WAIT
                return Move.ARM_UP
            case MoveMode.USER_ARM_DOWN:
                self.move_mode = MoveMode.USER_WAIT
                return Move.ARM_DOWN
            case MoveMode.USER_HAND_UP:
                self.move_mode = MoveMode.USER_WAIT
                return Move.HAND_UP
            case MoveMode.USER_HAND_DOWN:
                self.move_mode = MoveMode.USER_WAIT
                return Move.HAND_DOWN
            case MoveMode.USER_STEP:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_via_algo()
            case MoveMode.USER_STEP_EXPLORATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploration()
            case MoveMode.USER_STEP_EXPLOITATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploitation()
            case MoveMode.AUTOMATIC:
                return self.pick_move_via_algo()
            case move_mode:
                self.get_logger().error(f"Unknown move mode {move_mode}!")

    def pick_move_via_algo(self) -> Move:
        if rand.random() < self.explor_rate:
            return self.pick_move_exploration()
        return self.pick_move_exploitation()

    def pick_move_exploration(self) -> Move:
        something_new = rand.choice(np.array(list(Move)))
        self.get_logger().info(f"Randomly selected move {something_new}")
        self.move_is_exploration = True
        return something_new

    def get_reward(self, msg) -> None:
        self.learn(msg.data)
        self.pick_move_and_send()

    def stop(self, _) -> None:
        if not self.running:
            self.get_logger().info(f"{self.pretty_name} node is not running!")
            return
        self.running = False
        self.get_logger().info(f"Stopping {self.pretty_name} node")

    def update_explor_rate(self) -> None:
        self.explor_rate *= self.explor_decay_factor
        self.explor_rate = max(self.min_explor_rate, self.explor_rate)

    def attach_common_params(self, msg):
        msg.arm_states = self.arm_states
        msg.hand_states = self.hand_states
        msg.arm_step = self.arm_step
        msg.hand_step = self.hand_step
        msg.learning_rate = self.learning_rate
        msg.explor_rate = self.explor_rate
        msg.explor_decay_factor = self.explor_decay_factor
        msg.min_explor_rate = self.min_explor_rate
        msg.discount_factor = self.discount_factor

        msg.move_is_exploration = self.move_is_exploration

        msg.move_mode = self.move_mode.name
        msg.waiting_for_user_move = self.waiting_for_user_move

        return msg

    @abstractmethod
    def learn(self, rw: StateReward) -> None:
        pass

    @abstractmethod
    def pick_move_exploitation(self) -> Move:
        pass

    @abstractmethod
    def publish_internal_state(self) -> None:
        pass
