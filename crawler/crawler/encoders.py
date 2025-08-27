import os
from typing import Callable

import gpiozero  # type: ignore
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # type: ignore


class EncodersNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_encoders")

        self.declare_parameter("left_encoder_pin_a", 20)
        left_encoder_pin_a = (
            self.get_parameter("left_encoder_pin_a").get_parameter_value().integer_value
        )
        self.declare_parameter("left_encoder_pin_b", 16)
        left_encoder_pin_b = (
            self.get_parameter("left_encoder_pin_b").get_parameter_value().integer_value
        )
        self.declare_parameter("right_encoder_pin_a", 27)
        right_encoder_pin_a = (
            self.get_parameter("right_encoder_pin_a")
            .get_parameter_value()
            .integer_value
        )
        self.declare_parameter("right_encoder_pin_b", 17)
        right_encoder_pin_b = (
            self.get_parameter("right_encoder_pin_b")
            .get_parameter_value()
            .integer_value
        )

        self.encoders = Encoders(
            left_encoder_pin_a,
            left_encoder_pin_b,
            right_encoder_pin_a,
            right_encoder_pin_b,
            self.publish_left_encoder,
            self.publish_right_encoder,
        )

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
            lambda msg: self.encoders.mock_left_encoder_position(msg.data),  # type: ignore
            queue_len,
        )
        self.create_subscription(
            Int32,
            "/crawler/right_encoder/mock",
            lambda msg: self.encoders.mock_right_encoder_position(msg.data),  # type: ignore
            queue_len,
        )

    def publish_left_encoder(self, position: int) -> None:
        self.get_logger().info(f"Left encoder position: {position}")
        self.left_encoder_publisher.publish(Int32(data=position))

    def publish_right_encoder(self, position: int) -> None:
        # The value of the right encoder needs to be negated first to account for its physical orientation.
        position *= -1
        self.get_logger().info(f"Right encoder position: {position}")
        self.right_encoder_publisher.publish(Int32(data=position))


def Encoders(
    left_encoder_pin_a: int,
    left_encoder_pin_b: int,
    right_encoder_pin_a: int,
    right_encoder_pin_b: int,
    left_encoder_callback: Callable[[int], None],
    right_encoder_callback: Callable[[int], None],
):
    return (
        MockEncoders(left_encoder_callback, right_encoder_callback)
        if os.environ.get("CRAWLER_ENV") == "dev"
        else PhysicalEncoders(
            left_encoder_pin_a,
            left_encoder_pin_b,
            right_encoder_pin_a,
            right_encoder_pin_b,
            left_encoder_callback,
            right_encoder_callback,
        )
    )


class PhysicalEncoders:
    def __init__(
        self,
        left_encoder_pin_a,
        left_encoder_pin_b,
        right_encoder_pin_a,
        right_encoder_pin_b,
        left_encoder_callback,
        right_encoder_callback,
    ):
        PhysicalEncoder(left_encoder_pin_a, left_encoder_pin_b, left_encoder_callback)
        PhysicalEncoder(
            right_encoder_pin_a, right_encoder_pin_b, right_encoder_callback
        )


class PhysicalEncoder:
    def __init__(self, pin_a: int, pin_b: int, callback: Callable[[int], None]) -> None:
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.callback = callback

        self.encoder = gpiozero.RotaryEncoder(pin_a, pin_b, max_steps=0)
        self.encoder.when_rotated = lambda: self.callback(self.encoder.steps)


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
