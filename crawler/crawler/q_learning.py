import numpy as np
import rclpy
from crawler_msgs.msg import (  # type: ignore
    Action,
    QLearningInternalState,
    QLearningParameters,
    StateReward,
)
from numpy import random as rand
from rclpy.node import Node
from std_msgs.msg import Empty, Int32  # type: ignore

from .motors import ARM_RANGE, HAND_RANGE, ARM_MIN_LIMIT, HAND_MIN_LIMIT
from .move import MOVES_COUNT, Move


class QLearningNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

        queue_len = 5

        # start subscriber
        self.running = False
        self.create_subscription(
            QLearningParameters, "/crawler/rl/q_learning/start", self.start, queue_len
        )

        self.internals_publisher = self.create_publisher(
            QLearningInternalState, "/crawler/rl/q_learning/internals", queue_len
        )
        self.create_timer(1.0, self.publish_internal_state)

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

        # At this point we might also think about adding another dimension for self.last_move
        self.q_table = np.ones([self.arm_states, self.hand_states, MOVES_COUNT])

        self.running = True
        self.create_publisher(Empty, "/crawler/rl/start", 5).publish(Empty())

    def receive_arm_pos(self, msg) -> None:
        if not self.running:
            return
        self.curr_arm_state = int((msg.data - ARM_MIN_LIMIT) * self.arm_states / ARM_RANGE)

    def receive_hand_pos(self, msg) -> None:
        if not self.running:
            return
        self.curr_hand_state = int((msg.data - HAND_MIN_LIMIT) * self.hand_states / HAND_RANGE)

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

    def pick_move(self) -> Move:
        if rand.random() < self.explor_rate:
            something_new = rand.choice(np.array(Move))
            self.get_logger().info(f"Randomly selected move {something_new}")
            return something_new

        pool = self.q_table[self.curr_arm_state][self.curr_hand_state]
        move_idx = np.argmax(pool).item()
        move = Move(move_idx)
        self.get_logger().info(f"Selected move {move}")
        return move

    def get_reward(self, msg: StateReward) -> None:
        self.learn(msg)
        self.last_move = self.pick_move()
        self.send_move(self.last_move)

    def learn(self, rw: StateReward) -> None:
        idx = (self.last_arm_state, self.last_hand_state, self.last_move.value)
        predicted_value = self.q_table[idx]
        target_value = (
            self.q_table[self.curr_arm_state, self.curr_hand_state].max()
            * self.discount_factor
            + rw.reward
        )
        self.q_table[idx] = predicted_value + self.learning_rate * (
            target_value - predicted_value
        )
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
            msg.q_table_cols.append(f"action {i_action}")

        msg.q_table_values = self.q_table.flatten().tolist()

        self.internals_publisher.publish(msg)

    def stop(self, _) -> None:
        if not self.running:
            self.get_logger().info("Q-learning node is not running!")
            return

        self.get_logger().info("Shutting down Q-learning node")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
