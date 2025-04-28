import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
import gpiozero


class Blinker(Node):
    def __init__(self) -> None:
        super().__init__("crawler_blinker")

        self.declare_parameter("led_pin", 21)
        self.led_pin = self.get_parameter("led_pin").get_parameter_value().integer_value

        self.led = Led(self.led_pin)

        self.create_subscription(Empty, "/crawler/blinker/toggle", self.toggle, 5)
        self.create_subscription(Bool, "/crawler/blinker/write", self.write, 5)
        self.state_publisher = self.create_publisher(Bool, "/crawler/blinker/state", 5)

    def toggle(self, _) -> None:
        state = not self.led.read_state()
        self.led.set_state(state)
        self.get_logger().info(
            f"Toggled LED to be {'on' if state else 'off'} (GPIO pin {self.led_pin})"
        )
        self.publish_state(state)

    def write(self, msg) -> None:
        state = msg.data
        self.led.set_state(state)
        self.get_logger().info(
            f"Turned LED {'on' if state else 'off'} (GPIO pin {self.led_pin})"
        )
        self.publish_state(state)

    def publish_state(self, state: bool) -> None:
        self.state_publisher.publish(Bool(data=state))


def Led(led_pin: int):
    return MockLed() if os.environ.get("CRAWLER_ENV") == "dev" else PhysicalLed(led_pin)


class PhysicalLed:
    def __init__(self, pin: int) -> None:
        self.led = gpiozero.LED(pin)

    def read_state(self) -> bool:
        return self.led.value == 1

    def set_state(self, state: bool) -> None:
        self.led.value = 1 if state else 0


class MockLed:
    state = False

    def read_state(self) -> bool:
        return self.state

    def set_state(self, state: bool) -> None:
        self.state = state


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Blinker())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
