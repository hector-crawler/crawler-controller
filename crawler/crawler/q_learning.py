import random as rand
from enum import Enum

from rclpy.node import Node
from std_msgs.msg import Empty, Int32

MSG_QUEUE_LEN = 5


class Move(Enum):
    initial = 0
    ARM_UP = 1
    ARM_DOWN = 2
    HAND_UP = 3
    HAND_DOWN = 4


class Brain(Node):
    def __init__(self) -> None:
        super().__init__("crawler_brain")

        self.running = False
        self.algo = Algo()

        self.create_subscription(
            Empty, "/crawler/brain/start", self.start, MSG_QUEUE_LEN
        )
        self.create_subscription(Empty, "/crawler/brain/stop", self.stop, MSG_QUEUE_LEN)

        self.arm_publisher = self.create_publisher(
            Int32, "/crawler/arm/move", MSG_QUEUE_LEN
        )
        self.hand_publisher = self.create_publisher(
            Int32, "/crawler/hand/move", MSG_QUEUE_LEN
        )

    def start(self, _) -> None:
        print("Starting automatic control")
        self.running = True

        self.train()
        self.run()

        print("Exited automatic control")

    def train(self) -> None:
        print("Starting training")

        episodes = 10
        for _ in range(episodes):
            if not self.running:
                return

            move = self.algo.pick_movement()
            self.do_move(move)
            self.algo.learn()

    def run(self) -> None:
        print("Starting to simply move about")

        while self.running:
            move = self.algo.pick_movement()
            self.do_move(move)

    def do_move(self, move: Move) -> None:
        print(f"Doing move: {move}")

        motor_steps = 250

        match move:
            case Move.ARM_UP:
                self.arm_publisher.publish(Int32(motor_steps))
            case Move.ARM_DOWN:
                self.arm_publisher.publish(Int32(-motor_steps))
            case Move.HAND_UP:
                self.hand_publisher.publish(Int32(motor_steps))
            case Move.HAND_DOWN:
                self.hand_publisher.publish(Int32(-motor_steps))
            case _:
                raise Exception(f"Got an unrecognized movement command: {move}")

    def stop(self, _) -> None:
        print("Stopping automatic control")
        self.running = False


class QLearning(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

        # Picking random value here for now,
        # have to look into this more closely
        arm_possibilities = 2
        hand_possibilities = 2

        self.q_table = [[[0.0] * arm_possibilities] * hand_possibilities] * len(Move)

        self.learning_rate = 0.5
        self.discount_factor = 0.99
        self.exploration_rate = 1.0
        self.exploration_decay_rate = 0.01
        self.max_exploration_rate = 0.5
        self.min_exploration_rate = 0.01

        self.last_move = Move.initial

        self.arm_pos = 0
        self.hand_pos = 0
        self.left_encoder_val = 0
        self.right_encoder_val = 0
        self.left_encoder_change = 0
        self.right_encoder_change = 0

        self.create_subscription(
            Int32, "/crawler/arm/position", self.set_arm_pos, MSG_QUEUE_LEN
        )
        self.create_subscription(
            Int32, "/crawler/hand/position", self.set_hand_pos, MSG_QUEUE_LEN
        )

        self.create_subscription(
            Int32,
            "/crawler/left_encoder/position",
            self.set_left_encoder_val,
            MSG_QUEUE_LEN,
        )
        self.create_subscription(
            Int32,
            "/crawler/right_encoder/position",
            self.set_right_encoder_val,
            MSG_QUEUE_LEN,
        )

    def learn(self) -> None:
        pass

    def pick_movement(self) -> Move:
        if rand.uniform(0, 1) < self.exploration_rate:
            something_new = Move(rand.randint(1, len(Move)))
            self.last_move = something_new
            return something_new

        column = self.q_table[self.arm_pos][self.hand_pos]
        move = Move(max(column))
        self.last_move = move
        return move

    def set_arm_pos(self, msg) -> None:
        self.arm_pos = msg.data

    def set_hand_pos(self, msg) -> None:
        self.hand_pos = msg.data

    def set_left_encoder_val(self, msg) -> None:
        self.left_encoder_change = msg.data - self.left_encoder_val
        self.left_encoder_val = msg.data

    def set_right_encoder_val(self, msg) -> None:
        self.right_encoder_change = msg.data - self.right_encoder_val
        self.right_encoder_val = msg.data


def Algo() -> QLearning:
    return QLearning()
