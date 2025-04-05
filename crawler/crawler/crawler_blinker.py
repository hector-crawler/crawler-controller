import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

import RPi.GPIO as GPIO


class BlinkerSubscriber(Node):
    def __init__(self):
        super().__init__("crawler_blinker_subscriber")

        self.declare_parameter("led_pin", 40)
        self.led_pin = self.get_parameter("led_pin").get_parameter_value().integer_value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)

        self.subscription = self.create_subscription(Empty, "crawler_blinker_toggle", self.toggle, 5)

    def toggle(self, _):
        GPIO.output(self.led_pin, not GPIO.input(self.led_pin))
        self.get_logger().info(f"Toggled LED (GPIO pin {self.led_pin})")


def main(args=None):
    rclpy.init(args=args)

    blinker_subscriber = BlinkerSubscriber()
    rclpy.spin(blinker_subscriber)
    
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
