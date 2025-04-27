import os
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class EncodersNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_encoders")

        self.encoders = Encoders(self.publish_left_encoder, self.publish_right_encoder)

        queue_len = 5
        self.left_encoder_publisher = self.create_publisher(
            Int32, "/crawler/left_encoder/position", queue_len
        )
        self.right_encoder_publisher = self.create_publisher(
            Int32, "/crawler/right_encoder/position", queue_len
        )

        # only for mocked encoders
        self.create_subscription(
            Int32,
            "/crawler/left_encoder/mock",
            lambda msg: self.encoders.mock_left_encoder_position(msg.data),
            queue_len,
        )
        self.create_subscription(
            Int32,
            "/crawler/right_encoder/mock",
            lambda msg: self.encoders.mock_right_encoder_position(msg.data),
            queue_len,
        )

    def publish_left_encoder(self, position: int) -> None:
        self.get_logger().info(f"Left encoder position: {position}")
        self.left_encoder_publisher.publish(Int32(data=position))

    def publish_right_encoder(self, position: int) -> None:
        self.get_logger().info(f"Right encoder position: {position}")
        self.right_encoder_publisher.publish(Int32(data=position))


def Encoders(left_encoder_callback: Callable, right_encoder_callback: Callable):
    return (
        MockEncoders(left_encoder_callback, right_encoder_callback)
        if os.environ.get("CRAWLER_ENV") == "dev"
        else PhysicalEncoders()
    )


class PhysicalEncoders:
    def __init__(self) -> None:
        raise NotImplementedError("Physical encoders not implemented yet")


class MockEncoders:
    def __init__(
        self,
        left_encoder_callback: Callable[[int], None],
        right_encoder_callback: Callable[[int], None],
    ) -> None:
        self.left_encoder_callback = left_encoder_callback
        self.right_encoder_callback = right_encoder_callback

    def mock_left_encoder_position(self, position: int) -> None:
        self.left_encoder_callback(position)

    def mock_right_encoder_position(self, position: int) -> None:
        self.right_encoder_callback(position)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EncodersNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
