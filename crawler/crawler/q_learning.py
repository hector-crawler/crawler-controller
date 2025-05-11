import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class QLearningNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_q_learning")

        self.declare_parameter("paramA", 0)
        self.declare_parameter("paramB", 0)

        self.paramA = self.get_parameter("paramA").get_parameter_value().integer_value
        self.paramB = self.get_parameter("paramB").get_parameter_value().integer_value
        
        self.get_logger().info(f"Q-learning parameters: paramA={self.paramA}, paramB={self.paramB}")

        self.create_subscription(Empty, "/crawler/rl/stop", self.stop, 5)

        # ...

    def stop(self, _):
        self.get_logger().info("Shutting down Q-learning node")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QLearningNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()