import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

import RPi.GPIO as GPIO

#TODO make this an argument
led_pin = 40

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(led_pin, GPIO.OUT)

        queue_length = 5
        self.subscription = self.create_subscription(
            Empty, "led_blinker", self.listener_callback, queue_length
        )

    def listener_callback(self, msg):
        GPIO.output(led_pin, not GPIO.input(led_pin))
        self.get_logger().info(f"Blinking pin {led_pin}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
