import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TestSubscriber(Node):

    def __init__(self):
        super().__init__("test_subscriber")
        self.subscription = self.create_subscription(String, "test_topic", self.listener_callback, 10)
        self.subscription
    
    def listener_callback(self, msg):
        self.get_logger().info(f"Test: {msg}")


def main(args=None):
    print("Starting crawler_test")
    rclpy.init(args=args)
    test_subscriber = TestSubscriber()
    rclpy.spin(test_subscriber)


if __name__ == '__main__':
    main()
