import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool
import importlib


class Blinker(Node):
    def __init__(self):
        super().__init__("crawler_blinker")

        self.declare_parameter("led_pin", 40)
        self.led_pin = self.get_parameter("led_pin").get_parameter_value().integer_value

        self.led = Led(self.led_pin)

        self.create_subscription(Empty, "/crawler/blinker/toggle", self.toggle, 5)
        self.create_subscription(Bool, "/crawler/blinker/write", self.write, 5)
        self.state_publisher = self.create_publisher(Bool, "/crawler/blinker/state", 5)

    def toggle(self, _):
        state = not self.led.read_state()
        self.led.set_state(state)
        self.get_logger().info(f"Toggled LED to be {'on' if state else 'off'} (GPIO pin {self.led_pin})")
        self.publish_state(state)
    
    def write(self, msg):
        state = msg.data
        self.led.set_state(state)
        self.get_logger().info(f"Turned LED {'on' if state else 'off'} (GPIO pin {self.led_pin})")
        self.publish_state(state)

    def publish_state(self, state):
        self.state_publisher.publish(Bool(data=state))


def Led(led_pin):
    return MockLed() if os.environ.get("CRAWLER_ENV") == "dev" else PhysicalLed(led_pin)


class PhysicalLed():
    def __init__(self, pin):
        self.pin = pin
        self.GPIO = importlib.import_module("RPi.GPIO")
        self.GPIO.setwarnings(False)
        self.GPIO.setmode(self.GPIO.BOARD)
        self.GPIO.setup(pin, self.GPIO.OUT)

    def read_state(self):
        return self.GPIO.input(self.pin)
    
    def set_state(self, state):
        self.GPIO.output(self.pin, state)


class MockLed():
    state = False

    def read_state(self):
        return self.state
    
    def set_state(self, state):
        self.state = state


def main(args=None):
    rclpy.init(args=args)

    blinker = Blinker()
    rclpy.spin(blinker)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
