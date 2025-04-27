import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import importlib


class MotorsNode(Node):
    def __init__(self):
        super().__init__("crawler_motors")

        self.motors = Motors()

        self.create_subscription(Int32, "/crawler/arm/move", self.move_arm, 5)
        self.arm_publisher = self.create_publisher(Int32, "/crawler/arm/position", 5)
        self.create_subscription(Int32, "/crawler/hand/move", self.move_hand, 5)
        self.hand_publisher = self.create_publisher(Int32, "/crawler/hand/position", 5)

    def move_arm(self, msg):
        step = msg.data
        result = self.motors.move_arm(step)
        self.get_logger().info(f"Moved arm by {step} to {result}")
        self.arm_publisher.publish(Int32(data=result))

    def move_hand(self, msg):
        step = msg.data
        result = self.motors.move_hand(step)
        self.get_logger().info(f"Moved hand by {step} to {result}")
        self.hand_publisher.publish(Int32(data=result))


def Motors():
    return MockMotors() if os.environ.get("CRAWLER_ENV") == "dev" else PhysicalMotors()


class PhysicalMotors:
    def __init__(self):
        raise NotImplementedError("Physical motors not implemented yet")


class MockMotors:
    def __init__(self):
        self.hand_position = 50
        self.arm_position = 50

    def move_arm(self, step):
        self.arm_position = max(0, min(100, self.arm_position + step))
        return self.arm_position

    def move_hand(self, step):
        self.hand_position = max(0, min(100, self.hand_position + step))
        return self.hand_position


def main(args=None):
    rclpy.init(args=args)

    motors = MotorsNode()
    rclpy.spin(motors)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
