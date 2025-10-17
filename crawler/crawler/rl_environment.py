import random
import time
from enum import Enum
from typing import List

import rclpy
from crawler_msgs.msg import Action, RLEnvironmentInternals, StateReward  # type: ignore
from rclpy.node import Node
from std_msgs.msg import Empty, Int32  # type: ignore


class LoopState(Enum):
    STOPPED = 0
    WAITING = 1
    EXECUTING = 2


class RLEnvironmentNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_rl_environment")

        # parameters: motor start positions
        self.declare_parameter("arm_start_position", 1200)
        self.arm_start_position = (
            self.get_parameter("arm_start_position").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_start_position", 3000)
        self.hand_start_position = (
            self.get_parameter("hand_start_position")
            .get_parameter_value()
            .integer_value
        )

        queue_len = 5
        # handle /crawler/rl/state_reward, /crawler/rl/action
        self.state_reward_publisher = self.create_publisher(
            StateReward, "/crawler/rl/state_reward", queue_len
        )
        self.create_subscription(
            Action,
            "/crawler/rl/action",
            lambda msg: self.execute_action(msg),
            queue_len,
        )

        # observe environment (arm, hand, encoders)
        self.arm_position = 0
        self.create_subscription(
            Int32,
            "/crawler/arm/position",
            lambda msg: self.update_arm_position(msg.data),  # type: ignore
            queue_len,
        )
        self.hand_position = 0
        self.create_subscription(
            Int32,
            "/crawler/hand/position",
            lambda msg: self.update_hand_position(msg.data),  # type: ignore
            queue_len,
        )
        self.left_encoder_position = 0
        self.create_subscription(
            Int32,
            "/crawler/left_encoder/position",
            lambda msg: self.update_left_encoder_position(msg.data),  # type: ignore
            queue_len,
        )
        self.right_encoder_position = 0
        self.create_subscription(
            Int32,
            "/crawler/right_encoder/position",
            lambda msg: self.update_right_encoder_position(msg.data),  # type: ignore
            queue_len,
        )

        self.last_left_encoder_position = 0
        self.last_right_encoder_position = 0
        self.initial_encoder_positions = 0
        self.standstill_since = 0

        # handle actions (move arm, hand)
        self.arm_publisher = self.create_publisher(
            Int32, "/crawler/arm/move", queue_len
        )
        self.hand_publisher = self.create_publisher(
            Int32, "/crawler/hand/move", queue_len
        )

        # handle /crawler/rl/start, /crawler/rl/stop
        self.create_subscription(Empty, "/crawler/rl/start", self.start_rl, queue_len)
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop_rl, queue_len)

        # handle /crawler/rl/internals
        self.latest_state_reward = StateReward(
            arm_position=0, hand_position=0, reward=0.0
        )
        self.internals_publisher = self.create_publisher(
            RLEnvironmentInternals, "/crawler/rl/internals", queue_len
        )
        self.reset()

    def update_arm_position(self, position: int) -> None:
        self.arm_position = position

    def update_hand_position(self, position: int) -> None:
        self.hand_position = position

    def update_left_encoder_position(self, position: int) -> None:
        self.left_encoder_position = position

    def update_right_encoder_position(self, position: int) -> None:
        self.right_encoder_position = position

    # RL loop:
    # - receive at /crawler/rl/start from algorithm node
    # - send first state to /crawler/rl/state_reward
    # - repeat until stopped via /crawler/rl/stop:
    #   - receive action at /crawler/rl/action and execute it
    #   - send next state and reward to /crawler/rl/state_reward

    def publish_state_reward(self) -> None:
        left_progress = self.left_encoder_position - self.last_left_encoder_position
        right_progress = self.right_encoder_position - self.last_right_encoder_position
        reward = left_progress + right_progress

        self.standstill_since = self.standstill_since + 1 if reward == 0 else 0
        if self.standstill_since > 3:
            reward -= min(100, int(1.2**self.standstill_since))

        # calculate reward by adding difference in encoder positions
        self.last_left_encoder_position = self.left_encoder_position
        self.last_right_encoder_position = self.right_encoder_position

        # calculate progress
        self.progress.append(
            self.left_encoder_position
            + self.right_encoder_position
            - self.initial_encoder_positions
        )

        # send state and reward
        msg = StateReward()
        msg.arm_position = self.arm_position
        msg.hand_position = self.hand_position
        msg.reward = float(reward)
        self.state_reward_publisher.publish(msg)
        self.get_logger().info(
            f"Publishing state: arm_position={msg.arm_position}, hand_position={msg.hand_position}, reward={msg.reward}"
        )
        self.latest_state_reward = msg
        self.loop_state = LoopState.WAITING
        self.publish_internals()

    def start_rl(self, _) -> None:
        self.get_logger().info(f"Received request to start RL, loop state: {self.loop_state}")
        if self.loop_state != LoopState.STOPPED:
            raise Exception("Attempted to start RL, but it is already running!")
        self.get_logger().info("Starting RL")

        self.initial_encoder_positions = (
            self.left_encoder_position + self.right_encoder_position
        )
        self.standstill_since = 0

        # set motors to starting position
        self.arm_publisher.publish(Int32(data=self.arm_start_position))
        self.hand_publisher.publish(Int32(data=self.hand_start_position))

        time.sleep(0.5)  # TODO: Wait gracefully for motors to stop
        self.publish_state_reward()

    def execute_action(self, msg: Action) -> None:
        if self.loop_state == LoopState.STOPPED:
            self.get_logger().info(
                "Attempted to execute action, but RL is not running!"
            )
            return

        move_arm = msg.move_arm
        move_hand = msg.move_hand
        self.latest_action = msg

        self.arm_publisher.publish(Int32(data=move_arm))
        self.hand_publisher.publish(Int32(data=move_hand))
        self.get_logger().info(
            f"Executing action: move_arm={move_arm}, move_hand={move_hand}"
        )
        self.loop_state = LoopState.EXECUTING
        self.publish_internals()

        self.publish_state_reward_delayed(1.0)

    def publish_state_reward_delayed(self, delay: float) -> None:
        callback_id = random.randint(0, 1_000_000)
        self.current_state_reward_callback_id = callback_id
        self.publish_state_reward_timer = self.create_timer(delay, lambda: self.publish_state_reward_delayed_callback(callback_id))

    def publish_state_reward_delayed_callback(self, callback_id: int):
        if self.loop_state != LoopState.EXECUTING or self.current_state_reward_callback_id != callback_id:
            return
        self.publish_state_reward()
        self.publish_state_reward_timer.destroy()

    def publish_internals(self) -> None:
        msg = RLEnvironmentInternals()
        msg.loop_state = self.loop_state.value
        msg.latest_state_reward = self.latest_state_reward
        msg.latest_action = self.latest_action
        msg.progress = self.progress
        self.internals_publisher.publish(msg)

    def stop_rl(self, _) -> None:
        self.reset()
        self.publish_internals()
        self.get_logger().info("Stopping RL")

    def reset(self) -> None:
        self.loop_state = LoopState.STOPPED
        self.latest_action = Action(move_arm=0, move_hand=0)
        self.initial_encoder_positions = 0
        self.progress: List[int] = []


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RLEnvironmentNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
