import numpy as np
import rclpy
from crawler_msgs.msg import (
    QLearningInternalState,
    QLearningParameters,
    StateReward,
)
from std_msgs.msg import String

from crawler.algo_commons import QUEUE_LEN, AlgorithmNode
from crawler.move import MOVES_COUNT, Move


def sigmoid(x: float) -> float:
    return 1 / (1 + np.exp(-x))


class QLearningNode(AlgorithmNode):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning", "Q-Learning")

        self.create_subscription(
            QLearningParameters, "/crawler/rl/q_learning/start", self.start, QUEUE_LEN
        )

        self.internals_publisher = self.create_publisher(
            QLearningInternalState, "/crawler/rl/q_learning/internals", QUEUE_LEN
        )

        self.create_subscription(
            String,
            "/crawler/rl/q_learning/set_move_mode",
            self.set_move_mode,
            QUEUE_LEN,
        )

    def start(self, params):
        if self.running:
            self.get_logger().info("Q-learning node is already running!")
            return

        super().start(params)

        # q table
        if len(params.initial_q_table_values) > 0:
            self.q_table = np.array(params.initial_q_table_values).reshape(
                self.arm_states, self.hand_states, MOVES_COUNT
            )
            self.get_logger().info("Initialized Q-table from parameters")
        else:
            # At this point we might also think about adding another dimension for self.last_move
            self.q_table = np.full(
                [self.arm_states, self.hand_states, MOVES_COUNT], 0.5
            )
            self.get_logger().info("Initialized Q-table with 0.5es")

    def pick_move_exploitation(self) -> Move:
        pool = self.q_table[self.curr_arm_state][self.curr_hand_state]
        move_idx = np.argmax(pool).item()
        move = Move(move_idx)
        self.get_logger().info(f"Selected move {move}")
        self.move_is_exploration = False
        return move

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

        self.update_explor_rate()

    def publish_internal_state(self) -> None:
        if not self.running:
            return

        msg = QLearningInternalState()
        msg = self.attach_common_params(msg)

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

        self.internals_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
