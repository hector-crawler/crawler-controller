import time

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

from .motors import ARM_RANGE, HAND_RANGE
from .move import MOVES_COUNT, Move, MoveMode

QUEUE_LEN = 5


class NeuralNetworkNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_neural_network")

        self.internals_publisher = self.create_publisher(
            NNInternalState, "/crawler/rl/nn/internals", QUEUE_LEN
        )
        self.create_timer(1.0, self.publish_internal_state)

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
            "/crawler/rl/q_learning/set_move_mode",
            self.set_move_mode,
            QUEUE_LEN,
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

        
        self.inner_layer_width = parameters.inner_layer_width
        self.inner_layer_count = parameters.inner_layer_count
        self.hidden_activation = parameters.hidden_activation
        self.output_activation = parameters.output_activation

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

        time.sleep(0.5)

        # move mode
        self.move_mode = (
            MoveMode.USER_WAIT
            if parameters.initial_move_mode_wait
            else MoveMode.AUTOMATIC
        )
        self.waiting_for_user_move = False

        self.running = True
        self.create_publisher(Empty, "/crawler/rl/start", QUEUE_LEN).publish(Empty())

    def publish_internal_state(self) -> None:
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
        self.get_logger().info("Shutting down Neural Network node")
        self.destroy_node()

    def receive_arm_pos(self, msg) -> None:
        self.curr_arm_state = int(msg.data * self.arm_states / ARM_RANGE)

    def receive_hand_pos(self, msg) -> None:
        self.curr_hand_state = int(msg.data * self.hand_states / HAND_RANGE)

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

    def pick_move_exploration(self) -> Move:
        arr = np.array([[self.curr_arm_state, self.curr_hand_state]])
        self.last_predicts = self.model.predict(arr)
        move = np.argmax(self.last_predicts)
        self.get_logger().info(f"Selected move {move}")
        return Move(move)

    def pick_move_exploitation(self) -> Move:
        something_new = rand.choice(np.array(list(Move)))
        self.get_logger().info(f"Randomly selected move {something_new}")
        return something_new

    def pick_move(self) -> Move:
        if rand.random() < self.explor_rate:
            return self.pick_move_exploration()
        return self.pick_movr_exploitation()

    def learn(self, rw: StateReward) -> None:
        target = self.discount_factor * np.max(self.last_predicts) + rw.reward
        target_vector = self.last_predicts[0]
        target_vector[self.last_move.value] = target
        arr = np.array([[self.curr_arm_state, self.curr_hand_state]])
        self.model.fit(arr, target_vector.reshape(-1, MOVES_COUNT))

        self.explor_rate *= self.explor_decay_factor
        self.explor_rate = max(self.min_explor_rate, self.explor_rate)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeuralNetworkNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
