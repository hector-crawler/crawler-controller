import datetime
import random as rand
from enum import Enum

import rclpy
import torch
from crawler_msgs.msg import (
    Action, # type: ignore
    QLearningParameters, # type: ignore
    QLearningInternalState, # type: ignore
    StateReward, # type: ignore
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

        self.arm_states = 0
        self.hand_states = 0
        self.arm_step = 0
        self.hand_step = 0
        self.learning_rate = 0.0
        self.explor_rate = 0.0
        self.explor_decay_rate = 0.0
        self.max_explor_rate = 0.0
        self.min_explor_rate = 0.0
        self.discount_factor = 0.0

        # start subscriber
        self.running = False
        self.create_subscription(QLearningParameters, "/crawler/rl/q_learning/start", self.start, 5)

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

        """ # We might also want to use torch.rand() for initialization.
        self.q_table = torch.zeros(
            # At this point we might also think about adding another dimension for self.last_move
            [self.arm_states, self.hand_states, self.moves_count]
            # We might also want to investigate changing the dtype parameter for our usecase.
            # https://docs.pytorch.org/docs/stable/tensor_attributes.html#torch.dtype
        ) """

        self.q_table = torch.rand(self.arm_states, self.hand_states, 4)
    
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
        self.explor_decay_rate = parameters.explor_decay_rate
        self.max_explor_rate = parameters.max_explor_rate
        self.min_explor_rate = parameters.min_explor_rate
        self.discount_factor = parameters.discount_factor

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

        self.running = True
        self.create_publisher(Empty, "/crawler/rl/start", 5).publish(Empty())

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
        reward = msg.reward
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
        if not self.running:
            return

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
        
        # msg.timestamp = datetime.datetime.now().isoformat() # todo: reintroduce this?
        self.internal_state_publisher.publish(msg)

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
