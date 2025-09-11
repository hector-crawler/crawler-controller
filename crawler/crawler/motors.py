import os
from dataclasses import dataclass

import dynamixel_sdk as dxl  # type: ignore
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32  # type: ignore

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"

SHUTDOWN_MEM_ADDR = 63
TORQUE_MEM_ADDR = 64
TORQUE_ENABLE = 1
HW_ERR_STAT_MEM_ADDR = 70
PROFILE_VELOCITY_MEM_ADDR = 112
GOAL_POS_MEM_ADDR = 116
MOVING_MEM_ADDR = 122
PRESENT_POS_MEM_ADDR = 132


@dataclass
class MotorData:
    name: str
    id: int
    min_limit: int
    max_limit: int
    velocity: int


class MotorsNode(Node):
    def __init__(self):
        super().__init__("crawler_motors")

        # arm parameters
        self.declare_parameter("arm_id", 6)
        arm_id = self.get_parameter("arm_id").get_parameter_value().integer_value
        self.declare_parameter("arm_min_limit", 900)
        arm_min_limit = (
            self.get_parameter("arm_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("arm_max_limit", 1500)
        arm_max_limit = (
            self.get_parameter("arm_max_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("arm_speed", 10000)
        arm_speed = self.get_parameter("arm_speed").get_parameter_value().integer_value
        self.arm = MotorData("Arm", arm_id, arm_min_limit, arm_max_limit, arm_speed)

        # hand parameters
        self.declare_parameter("hand_id", 7)
        hand_id = self.get_parameter("hand_id").get_parameter_value().integer_value
        self.declare_parameter("hand_min_limit", 2500)
        hand_min_limit = (
            self.get_parameter("hand_min_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_max_limit", 3300)
        hand_max_limit = (
            self.get_parameter("hand_max_limit").get_parameter_value().integer_value
        )
        self.declare_parameter("hand_speed", 10000)
        hand_speed = (
            self.get_parameter("hand_speed").get_parameter_value().integer_value
        )
        self.hand = MotorData(
            "Hand", hand_id, hand_min_limit, hand_max_limit, hand_speed
        )

        queue_len = 5
        self.create_subscription(Int32, "/crawler/arm/move", self.move_arm, queue_len)
        self.arm_publisher = self.create_publisher(
            Int32, "/crawler/arm/position", queue_len
        )
        self.create_subscription(Int32, "/crawler/hand/move", self.move_hand, queue_len)
        self.hand_publisher = self.create_publisher(
            Int32, "/crawler/hand/position", queue_len
        )

        self.port_handler = dxl.PortHandler(DEVICENAME)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

        if self.port_handler.openPort() == -1:
            raise Exception("Failed to open the port")

        if self.port_handler.setBaudRate(BAUDRATE) == -1:
            raise Exception("Failed to set baudrate")

        for motor in [self.arm, self.hand]:
            self.setup_motor(motor)

        self.moving_status_publisher = self.create_publisher(
            Bool, "/crawler/motors/moving", queue_len
        )
        interval_seconds = 0.1
        self.create_timer(interval_seconds, self.update_moving_status, autostart=True)

        interval_seconds = 0.5
        self.create_timer(interval_seconds, self.update_motor_positions, autostart=True)

    def move_arm(self, msg):
        step = msg.data
        self.move_motor(self.arm, step)
        self.get_logger().info(f"Moved arm by {step}")

    def move_hand(self, msg):
        step = msg.data
        self.move_motor(self.hand, step)
        self.get_logger().info(f"Moved hand by {step}")

    def publish_arm_position(self, position: int) -> None:
        self.get_logger().info(f"Arm position: {position}")
        self.arm_publisher.publish(Int32(data=position))

    def publish_hand_position(self, position: int) -> None:
        self.get_logger().info(f"Hand position: {position}")
        self.hand_publisher.publish(Int32(data=position))

    def handle_errors(self, m: MotorData, comm_res: int, err: int, msg: str) -> None:
        if comm_res != dxl.COMM_SUCCESS:
            self.get_logger().error(f"COMMUNICATION FAIL: {msg} for motor {m.name}")
        elif err != 0:
            self.get_logger().error(
                f"""ERROR: {msg} for motor {m.name}: {self.packet_handler.getRxPacketError(err)}"""
            )

    def setup_motor(self, motor: MotorData) -> None:
        comm_result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor.id, PROFILE_VELOCITY_MEM_ADDR, motor.velocity
        )
        self.handle_error(motor, comm_result, error, "Setting velocity")

        comm_result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor.id, TORQUE_MEM_ADDR, TORQUE_ENABLE
        )
        self.handle_error(motor, comm_result, error, "Enabling torque")

    def update_motor_positions(self) -> None:
        self.arm_position = self.read_motor_position(self.arm)
        self.hand_position = self.read_motor_position(self.hand)
        self.publish_arm_position(self.arm_position)
        self.publish_hand_position(self.hand_position)
        self.get_logger().info(
            f"Updated arm and hand position: {self.arm_position} {self.hand_position}"
        )

    def read_motor_position(self, motor: MotorData) -> int:
        pos, comm_result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor.id, PRESENT_POS_MEM_ADDR
        )
        self.handle_error(motor, comm_result, error, "Reading position")
        return pos

    def move_motor(self, motor: MotorData, step: int) -> int:
        current_position = self.read_motor_position(motor)
        desired_position = max(
            motor.min_limit, min(current_position + step, motor.max_limit)
        )

        comm_result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor.id, GOAL_POS_MEM_ADDR, desired_position
        )
        self.handle_error(motor, comm_result, error, "Moving")
        return desired_position

    def is_moving(self, motor) -> bool:
        is_moving, comm_result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor.id, MOVING_MEM_ADDR
        )
        self.handle_error(motor, comm_result, error, "Reading Movement")
        return is_moving == 1

    def update_moving_status(self):
        hand_moving = self.is_moving(self.hand)
        arm_moving = self.is_moving(self.arm)
        self.moving_status_publisher.publish(Bool(data=hand_moving or arm_moving))


class MockMotorsNode(Node):
    def __init__(self) -> None:
        super().__init__("crawler_motors")

        self.arm_position = 1000
        self.arm_publisher = self.create_publisher(Int32, "/crawler/arm/position", 5)
        self.create_subscription(
            Int32,
            "/crawler/arm/move",
            lambda msg: self.move_arm(msg.data),  # type: ignore
            5,
        )

        self.hand_position = 1000
        self.hand_publisher = self.create_publisher(Int32, "/crawler/hand/position", 5)
        self.create_subscription(
            Int32,
            "/crawler/hand/move",
            lambda msg: self.move_hand(msg.data),  # type: ignore
            5,
        )

    def move_arm(self, step: int) -> None:
        self.arm_position = max(600, min(1200, self.arm_position + step))
        self.arm_publisher.publish(Int32(data=self.arm_position))

    def move_hand(self, step: int) -> None:
        self.hand_position = max(1500, min(2100, self.hand_position + step))
        self.hand_publisher.publish(Int32(data=self.hand_position))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(
        MotorsNode() if os.environ.get("CRAWLER_ENV") != "dev" else MockMotorsNode()
    )
    rclpy.shutdown()


if __name__ == "__main__":
    main()
