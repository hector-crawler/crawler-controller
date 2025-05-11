import rclpy
from rclpy.node import Node
from crawler_msgs.msg import StateReward, Action # type: ignore
from std_msgs.msg import Empty, Int32


class RLEnvironmentNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_rl_environment")

        # handle /crawler/rl/state_reward, /crawler/rl/action
        self.state_reward_publisher = self.create_publisher(StateReward, "/crawler/rl/state_reward", 5)
        self.create_subscription(Action, "/crawler/rl/action", lambda msg : [msg.move_hand, msg.move_arm], 5) # type: ignore

        # observe environment (arm, hand, encoders)
        self.arm_position = 0
        self.create_subscription(Int32, "/crawler/arm/position", lambda msg : self.update_arm_position(msg.data), 5) # type: ignore
        self.hand_position = 0
        self.create_subscription(Int32, "/crawler/hand/position", lambda msg : self.update_hand_position(msg.data), 5) # type: ignore
        self.left_encoder_position = 0
        self.create_subscription(Int32, "/crawler/left_encoder/position", lambda msg : self.update_left_encoder_position(msg.data), 5) # type: ignore
        self.right_encoder_position = 0
        self.create_subscription(Int32, "/crawler/right_encoder/position", lambda msg : self.update_right_encoder_position(msg.data), 5) # type: ignore

        self.last_left_encoder_position = 0
        self.last_right_encoder_position = 0

        # handle actions (move arm, hand)
        self.arm_publisher = self.create_publisher(Int32, "/crawler/arm/move", 5)
        self.hand_publisher = self.create_publisher(Int32, "/crawler/hand/move", 5)

        # start RL
        self.create_subscription(Empty, "/crawler/rl/start", self.start_rl, 5)
        self.create_subscription(Empty, "/crawler/rl/stop", self.stop_rl, 5)

        # todo: perform RL cycle
    
    def update_arm_position(self, position: int) -> None:
        self.arm_position = position

    def update_hand_position(self, position: int) -> None:
        self.hand_position = position
    
    def update_left_encoder_position(self, position: int) -> None:
        self.left_encoder_position = position
    
    def update_right_encoder_position(self, position: int) -> None:
        self.right_encoder_position = position

    def execute_action(self, move_arm: int, move_hand: int) -> None:
        self.arm_publisher.publish(Int32(data=move_arm))
        self.hand_publisher.publish(Int32(data=move_hand))
        self.get_logger().info(f"Executed action: move_arm={move_arm}, move_hand={move_hand}")

    def publish_state_reward(self) -> None:
        msg = StateReward()
        msg.arm_position = self.arm_position
        msg.hand_position = self.hand_position
        msg.reward = self.calculate_reward()
        self.state_reward_publisher.publish(msg)
        self.get_logger().info(f"Published state: {msg.arm_position}, {msg.hand_position}, reward: {msg.reward}")

    def calculate_reward(self) -> int:
        # calculate reward by adding difference in encoder positions
        reward = (self.left_encoder_position - self.last_left_encoder_position) + (self.right_encoder_position - self.last_right_encoder_position)
        self.last_left_encoder_position = self.left_encoder_position
        self.last_right_encoder_position = self.right_encoder_position
        return reward
    
    def start_rl(self, _) -> None:
        self.rl_running = True
        self.get_logger().info(f"Starting RL")

    def stop_rl(self, _) -> None:
        self.rl_running = False
        self.get_logger().info(f"Stopping RL")



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RLEnvironmentNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()