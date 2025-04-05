import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool

import RPi.GPIO as GPIO


class Blinker(Node):
    def __init__(self):
        super().__init__("crawler_blinker")

        self.declare_parameter("led_pin", 40)
        self.led_pin = self.get_parameter("led_pin").get_parameter_value().integer_value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)

        self.create_subscription(Empty, "crawler_blinker_toggle", self.toggle, 5)
        self.create_subscription(Bool, "crawler_blinker_write", self.write, 5)
        self.state_publisher = self.create_publisher(Bool, "crawler_blinker_state", 5)

    def toggle(self, _):
        state = not GPIO.input(self.led_pin)
        GPIO.output(self.led_pin, state)
        self.get_logger().info(f"Toggled LED to be {'on' if state else 'off'} (GPIO pin {self.led_pin})")
        self.publish_state(state)
    
    def write(self, msg):
        state = msg.data
        GPIO.output(self.led_pin, state)
        self.get_logger().info(f"Turned LED {'on' if state else 'off'} (GPIO pin {self.led_pin})")
        self.publish_state(state)

    def publish_state(self, state):
        self.state_publisher.publish(Bool(data=state))


def main(args=None):
    rclpy.init(args=args)

    blinker = Blinker()
    rclpy.spin(blinker)
    
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
