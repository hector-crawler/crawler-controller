from typing import Optional
import numpy as np
import rclpy
from crawler_msgs.msg import (
    Action,  # type: ignore
    QLearningInternalState,  # type: ignore
    QLearningParameters,  # type: ignore
    StateReward,  # type: ignore
)
from numpy import random as rand
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String  # type: ignore
from enum import Enum

from .move import MOVES_COUNT, Move


def sigmoid(x: float) -> float:
    return 1 / (1 + np.exp(-x))

class MoveMode(Enum):
    USER_WAIT = 0
    USER_ARM_UP = 1
    USER_ARM_DOWN = 2
    USER_HAND_UP = 3
    USER_HAND_DOWN = 4
    USER_STEP = 5
    USER_STEP_EXPLORATION = 6
    USER_STEP_EXPLOITATION = 7
    AUTOMATIC = 8


class QLearningNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

        queue_len = 5

        # parameters for arm/hand limits
        self.declare_parameter("arm_min_limit", 900)
        self.arm_min_limit = (
            self.get_parameter("arm_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("arm_max_limit", 1500)
        self.arm_max_limit = (
            self.get_parameter("arm_max_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_min_limit", 2500)
        self.hand_min_limit = (
            self.get_parameter("hand_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_max_limit", 3300)
        self.hand_max_limit = (
            self.get_parameter("hand_max_limit").get_parameter_value().integer_value
        )

        # start subscriber
        self.running = False
        self.create_subscription(
            QLearningParameters, "/crawler/rl/q_learning/start", self.start, queue_len
        )

        self.internals_publisher = self.create_publisher(
            QLearningInternalState, "/crawler/rl/q_learning/internals", queue_len
        )
        self.move_is_exploration = False
        self.create_timer(0.2, self.publish_internal_state)

        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, queue_len)
        self.create_subscription(
            StateReward, "/crawler/rl/state_reward", self.get_reward, queue_len
        )

        self.create_subscription(
            Int32, "/crawler/arm/position", self.receive_arm_pos, queue_len
        )
        self.create_subscription(
            Int32, "/crawler/hand/position", self.receive_hand_pos, queue_len
        )

        self.create_subscription(
            String, "/crawler/rl/q_learning/set_move_mode", self.set_move_mode, queue_len
        )

        self.action_publisher = self.create_publisher(
            Action, "/crawler/rl/action", queue_len
        )

        self.last_move = Move.ARM_UP
        self.curr_arm_state = 0
        self.curr_hand_state = 0
        self.last_arm_state = 0
        self.last_hand_state = 0

    def start(self, parameters):
        if self.running:
            self.get_logger().info("Q-learning node is already running!")
            return

        # parameters
        self.arm_states = parameters.arm_states
        self.hand_states = parameters.hand_states
        self.arm_step = parameters.arm_step
        self.hand_step = parameters.hand_step
        self.learning_rate = parameters.learning_rate
        self.explor_rate = parameters.explor_rate
        self.explor_decay_factor = parameters.explor_decay_factor
        self.min_explor_rate = parameters.min_explor_rate
        self.discount_factor = parameters.discount_factor
        
        # q table
        if len(parameters.initial_q_table_values) > 0:
            self.q_table = np.array(parameters.initial_q_table_values).reshape(self.arm_states, self.hand_states, MOVES_COUNT)
            self.get_logger().info("Initialized Q-table from parameters")
        else:
            # At this point we might also think about adding another dimension for self.last_move
            self.q_table = np.ones([self.arm_states, self.hand_states, MOVES_COUNT])
            self.get_logger().info("Initialized Q-table with ones")

        # move mode
        self.move_mode = MoveMode.USER_WAIT if parameters.initial_move_mode_wait else MoveMode.AUTOMATIC
        self.waiting_for_user_move = False

        self.get_logger().info(
            f"""
Q-learning parameters:
    Arm states = {self.arm_states}
    Hand states = {self.hand_states}
    No. of Moves = {MOVES_COUNT}
    Q-Table size = {self.arm_states}x{self.hand_states}x{MOVES_COUNT} = {self.arm_states * self.hand_states * MOVES_COUNT} cells

    Exploration rate = {self.explor_rate}
    Exploration decay factor = {self.explor_decay_factor}
    Min exploration rate = {self.min_explor_rate}
    Discount factor = {self.discount_factor}
"""
        )

        self.running = True
        self.create_publisher(Empty, "/crawler/rl/start", 5).publish(Empty())
    
    def set_move_mode(self, msg) -> None:
        if msg.data in [mode.name for mode in MoveMode]:
            self.move_mode = MoveMode[msg.data]
            self.get_logger().info(f"Set move mode to {self.move_mode}")
            if self.waiting_for_user_move:
                self.pick_move_and_send()
                self.waiting_for_user_move = False
            self.publish_internal_state()
        else:
            self.get_logger().error(f"Unknown move mode {msg.data}!")

    def receive_arm_pos(self, msg) -> None:
        if not self.running:
            return
        self.curr_arm_state = int(
            (msg.data - self.arm_min_limit)
            / (self.arm_max_limit - self.arm_min_limit)
            * self.arm_states
        )

    def receive_hand_pos(self, msg) -> None:
        if not self.running:
            return
        self.curr_hand_state = int(
            (msg.data - self.hand_min_limit)
            / (self.hand_max_limit - self.hand_min_limit)
            * self.hand_states
        )

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
                return self.pick_move_via_q_learning()
            case MoveMode.USER_STEP_EXPLORATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploration()
            case MoveMode.USER_STEP_EXPLOITATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploitation()
            case MoveMode.AUTOMATIC:
                return self.pick_move_via_q_learning()
            case _:
                self.get_logger().error(f"Unknown move mode {move_mode}!")
    
    def pick_move_via_q_learning(self) -> Move:
        if rand.random() < self.explor_rate:
            return self.pick_move_exploration()
        else:
            return self.pick_move_exploitation()
    
    def pick_move_exploration(self) -> Move:
        something_new = rand.choice(np.array(Move))
        self.get_logger().info(f"Randomly selected move {something_new}")
        self.move_is_exploration = True
        return something_new

    def pick_move_exploitation(self) -> Move:
        pool = self.q_table[self.curr_arm_state][self.curr_hand_state]
        move_idx = np.argmax(pool).item()
        move = Move(move_idx)
        self.get_logger().info(f"Selected move {move}")
        self.move_is_exploration = False
        return move

    def get_reward(self, msg: StateReward) -> None:
        self.learn(msg)
        self.pick_move_and_send()

    def learn(self, rw: StateReward) -> None:
        idx = (self.last_arm_state, self.last_hand_state, self.last_move.value)

        value_of_next_action = self.q_table[
            self.curr_arm_state, self.curr_hand_state
        ].max()
        reward = sigmoid(rw.reward)
        quality_of_action = (value_of_next_action * self.discount_factor + reward) / 2
        difference = quality_of_action - self.q_table[idx]
        self.q_table[idx] += difference * self.learning_rate

        self.last_arm_state = self.curr_arm_state
        self.last_hand_state = self.curr_hand_state

        self.explor_rate *= self.explor_decay_factor
        self.explor_rate = max(self.min_explor_rate, self.explor_rate)

    def publish_internal_state(self) -> None:
        if not self.running:
            return

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
        for i_action in range(MOVES_COUNT):
            match Move(i_action):
                case Move.ARM_UP:
                    msg.q_table_cols.append("arm up")
                case Move.ARM_DOWN:
                    msg.q_table_cols.append("arm down")
                case Move.HAND_UP:
                    msg.q_table_cols.append("hand up")
                case Move.HAND_DOWN:
                    msg.q_table_cols.append("hand down")
                case _:
                    msg.q_table_cols.append("(unknown)")

        msg.q_table_values = self.q_table.flatten().tolist()

        msg.move_is_exploration = self.move_is_exploration

        msg.move_mode = self.move_mode.name
        msg.waiting_for_user_move = self.waiting_for_user_move

        self.internals_publisher.publish(msg)

    def stop(self, _) -> None:
        if not self.running:
            self.get_logger().info("Q-learning node is not running!")
            return
        self.running = False
        self.get_logger().info("Stopping Q-learning node")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
