from typing import Optional

import numpy as np
import rclpy
from crawler_msgs.msg import (  # type: ignore
    Action,
    NNInternalState,
    NNParameters,
    StateReward,
    String,
)
from keras import layers, models  # type: ignore
from keras.optimizers import Adam  # type: ignore
from numpy import random as rand
from rclpy.node import Node
from std_msgs.msg import Empty, Int32  # type: ignore

from .move import MOVES_COUNT, Move, MoveMode

QUEUE_LEN = 5


class NeuralNetworkNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_neural_network")

        self.internals_publisher = self.create_publisher(
            NNInternalState, "/crawler/rl/nn/internals", QUEUE_LEN
        )
        self.move_is_exploration = False
        self.create_timer(0.2, self.publish_internal_state)

        # start subscriber
        self.running = False
        self.create_subscription(
            NNParameters, "/crawler/rl/nn/start", self.start, QUEUE_LEN
        )
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, QUEUE_LEN)
        self.create_subscription(
            StateReward, "/crawler/rl/state_reward", self.get_reward, QUEUE_LEN
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

        self.create_subscription(
            String,
            "/crawler/rl/nn/set_move_mode",
            self.set_move_mode,
            QUEUE_LEN,
        )

        self.last_move = Move.ARM_UP
        self.curr_arm_state = 0
        self.curr_hand_state = 0
        self.last_arm_state = 0
        self.last_hand_state = 0

    def start(self, params):
        if self.running:
            self.get_logger().info("Neural Network node is already running!")
            return

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

        self.inner_layer_width = params.inner_layer_width
        self.inner_layer_count = params.inner_layer_count
        self.hidden_activation = params.hidden_activation
        self.output_activation = params.output_activation

        self.model = models.Sequential()
        motors = 2
        self.model.add(layers.InputLayer(input_shape=(motors,)))
        for _ in range(self.hidden_count):
            self.model.add(
                layers.Dense(self.hidden_width, activation=self.hidden_activation)
            )
        self.model.add(layers.Dense(MOVES_COUNT, activation=self.output_activation))

        self.optimizer: Adam = Adam(learning_rate=self.learning_rate)
        self.model.compile(optimizer=self.optimizer)
        # Alternative call with some defaults from https://www.baeldung.com/cs/reinforcement-learning-neural-network:
        # self.model.compile(loss="mse", optimizer=self.optimizer, metrics=["mae"])

        self.last_predicts: np.ndarray = np.array([])

        self.move_mode = (
            MoveMode.USER_WAIT if params.initial_move_mode_wait else MoveMode.AUTOMATIC
        )
        self.waiting_for_user_move = False

        self.get_logger().info(
            f"""
NN parameters:
    Arm states = {self.arm_states}
    Hand states = {self.hand_states}
    No. of Moves = {MOVES_COUNT}

    Hidden layers = {self.hidden_count}
    Hidden layer width = {self.hidden_width}
    Hidden layer activation = {self.hidden_activation}
    Output layer activation = {self.output_activation}

    Learning rate = {self.learning_rate}
    Exploration rate = {self.explor_rate}
    Exploration decay factor = {self.explor_decay_factor}
    Min exploration rate = {self.min_explor_rate}
    Discount factor = {self.discount_factor}
"""
        )

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
        self.curr_arm_state = min(max(discrete_value, 0), len(self.q_table) - 1)

    def receive_hand_pos(self, msg) -> None:
        if not self.running:
            return
        discrete_value = int(
            (msg.data - self.hand_min_limit)
            / (self.hand_max_limit - self.hand_min_limit)
            * self.hand_states
        )
        self.curr_hand_state = min(
            max(discrete_value, 0), len(self.q_table[self.curr_arm_state]) - 1
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
                return self.pick_move_via_nn()
            case MoveMode.USER_STEP_EXPLORATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploration()
            case MoveMode.USER_STEP_EXPLOITATION:
                self.move_mode = MoveMode.USER_WAIT
                return self.pick_move_exploitation()
            case MoveMode.AUTOMATIC:
                return self.pick_move_via_nn()
            case move_mode:
                self.get_logger().error(f"Unknown move mode {move_mode}!")

    def pick_move_via_nn(self) -> Move:
        if rand.random() < self.explor_rate:
            return self.pick_move_exploration()
        return self.pick_move_exploitation()

    def pick_move_exploration(self) -> Move:
        something_new = rand.choice(np.array(list(Move)))
        self.get_logger().info(f"Randomly selected move {something_new}")
        self.move_is_exploration = True
        return something_new

    def pick_move_exploitation(self) -> Move:
        arr = np.array([[self.curr_arm_state, self.curr_hand_state]])
        self.last_predicts = self.model.predict(arr)
        move = np.argmax(self.last_predicts)
        self.get_logger().info(f"Selected move {move}")
        self.move_is_exploration = False
        return Move(move)

    def get_reward(self, msg) -> None:
        self.learn(msg.data)
        self.pick_move_and_send()

    def learn(self, rw: StateReward) -> None:
        target = self.discount_factor * np.max(self.last_predicts) + rw.reward
        target_vector = self.last_predicts[0]
        target_vector[self.last_move.value] = target
        arr = np.array([[self.curr_arm_state, self.curr_hand_state]])
        self.model.fit(arr, target_vector.reshape(-1, MOVES_COUNT))

        self.explor_rate *= self.explor_decay_factor
        self.explor_rate = max(self.min_explor_rate, self.explor_rate)

    def publish_internal_state(self) -> None:
        if not self.running:
            return

        msg = NNInternalState()
        msg.arm_states = self.arm_states
        msg.hand_states = self.hand_states
        msg.arm_step = self.arm_step
        msg.hand_step = self.hand_step
        msg.learning_rate = self.learning_rate
        msg.explor_rate = self.explor_rate
        msg.explor_decay_factor = self.explor_decay_factor
        msg.min_explor_rate = self.min_explor_rate
        msg.discount_factor = self.discount_factor

        msg.hidden_count = self.hidden_count
        msg.hidden_width = self.hidden_width
        msg.hidden_activation = self.hidden_activation
        msg.output_activation = self.output_activation
        msg.last_predicts = self.last_predicts.flatten()

        self.internals_publisher.publish(msg)

    def stop(self, _) -> None:
        if not self.running:
            self.get_logger().info("Neural Network node is not running!")
            return
        self.running = False
        self.get_logger().info("Stopping Neural Network node")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeuralNetworkNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
