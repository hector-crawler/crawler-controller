import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from crawler_msgs.msg import QLearningInternalState # type: ignore
import datetime

class QLearningNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

        self.declare_parameter("param_a", 0)
        self.param_a = self.get_parameter("param_a").get_parameter_value().integer_value
        self.declare_parameter("param_b", 0)
        self.param_b = self.get_parameter("param_b").get_parameter_value().integer_value
        self.get_logger().info(f"Q-learning parameters: paramA={self.param_a}, paramB={self.param_b}")

        self.internal_state_publisher = self.create_publisher(QLearningInternalState, "/crawler/rl/q_learning/internals", 5)
        self.create_timer(1.0, self.publish_internal_state)

        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, 5)

        # ...
    
    def publish_internal_state(self) -> None:
        msg = QLearningInternalState()
        msg.param_a = self.param_a
        msg.param_b = self.param_b
        msg.timestamp = datetime.datetime.now().isoformat()
        self.internal_state_publisher.publish(msg)

    def stop(self, _):
        self.get_logger().info("Shutting down Q-learning node")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()