import numpy as np
import rclpy
from crawler_msgs.msg import (
    NNInternalState,
    NNParameters,
    StateReward,
)
from keras import layers, models
from keras.optimizers import Adam
from std_msgs.msg import String

from crawler.algo_commons import QUEUE_LEN, AlgorithmNode
from crawler.move import MOVES_COUNT, Move


class NeuralNetworkNode(AlgorithmNode):
    def __init__(self) -> None:
        super().__init__("crawler_neural_network", "Neural Network")

        self.internals_publisher = self.create_publisher(
            NNInternalState, "/crawler/rl/nn/internals", QUEUE_LEN
        )

        self.create_subscription(
            NNParameters, "/crawler/rl/nn/start", self.start, QUEUE_LEN
        )
        self.create_subscription(
            String,
            "/crawler/rl/nn/set_move_mode",
            self.set_move_mode,
            QUEUE_LEN,
        )

    def start(self, params):
        if self.running:
            self.get_logger().info("Neural Network node is already running!")
            return

        super().start(params)

        self.hidden_width = params.hidden_width
        self.hidden_count = params.hidden_count
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

        self.get_logger().info(
            f"""
NN parameters:
    Hidden layers = {self.hidden_count}
    Hidden layer width = {self.hidden_width}
    Hidden layer activation = {self.hidden_activation}
    Output layer activation = {self.output_activation}
"""
        )

    def pick_move_exploitation(self) -> Move:
        arr = np.array([[self.curr_arm_state, self.curr_hand_state]])
        self.last_predicts = self.model.predict(arr)
        move = np.argmax(self.last_predicts)
        self.get_logger().info(f"Selected move {move}")
        self.move_is_exploration = False
        return Move(move)

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
        msg.last_predicts = self.last_predicts.flatten().tolist()

        self.internals_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NeuralNetworkNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
