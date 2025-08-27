import os
from dataclasses import dataclass

import dynamixel_sdk as dxl  # type: ignore
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # type: ignore

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"

TORQUE_MEM_ADDR = 64
TORQUE_ENABLE = 1
PROFILE_VELOCITY_MEM_ADDR = 112
PROFILE_VELOCITY = 10000
GOAL_POS_MEM_ADDR = 116
PRESENT_POS_MEM_ADDR = 132


@dataclass
class MotorData:
    name: str
    id: int
    min_limit: int
    max_limit: int


ARM_MIN_LIMIT = 900
HAND_MIN_LIMIT = 2500
Arm = MotorData("Arm", 6, 900, 1500)
Hand = MotorData("Hand", 7, 2500, 3300)
# todo: make the motor IDs and limits a node parameter

ARM_RANGE = Arm.max_limit - Arm.min_limit
HAND_RANGE = Hand.max_limit - Hand.min_limit

class MotorsNode(Node):
    def __init__(self):
        super().__init__("crawler_motors")

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

        for motor in [Arm, Hand]:
            self.setup_motor(motor)

        self.create_timer(0.5, self.update_motor_positions, autostart=True)

    def move_arm(self, msg):
        step = msg.data
        self.move_motor(Arm, step)
        self.get_logger().info(f"Moved arm by {step}")

    def move_hand(self, msg):
        step = msg.data
        self.move_motor(Hand, step)
        self.get_logger().info(f"Moved hand by {step}")

    def publish_arm_position(self, position: int) -> None:
        self.get_logger().info(f"Arm position: {position}")
        self.arm_publisher.publish(Int32(data=position))

    def publish_hand_position(self, position: int) -> None:
        self.get_logger().info(f"Hand position: {position}")
        self.hand_publisher.publish(Int32(data=position))

    def setup_motor(self, motor: MotorData) -> None:
        comm_result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor.id, PROFILE_VELOCITY_MEM_ADDR, PROFILE_VELOCITY
        )
        if comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to set profile velocity for {motor.name}")
        elif error != 0:
            self.get_logger().error(
                f"Error occurred while setting profile velocity for {motor.name}:"  # error occured here, should probably log error and/or comm_result
                + self.packet_handler.getRxPacketError(error)
                + str(comm_result)
            )

        comm_result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor.id, TORQUE_MEM_ADDR, TORQUE_ENABLE
        )
        if comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque for {motor.name}")
        elif error != 0:
            self.get_logger().error(
                f"Error occurred while enabling torque for {motor.name}:"  # error occured here, should probably log error and/or comm_result
                + self.packet_handler.getRxPacketError(error)
                + str(comm_result)
            )

    def update_motor_positions(self) -> None:
        self.arm_position = self.read_motor_position(Arm)
        self.hand_position = self.read_motor_position(Hand)
        self.publish_arm_position(self.arm_position)
        self.publish_hand_position(self.hand_position)
        self.get_logger().info(
            f"Updated arm and hand position: {self.arm_position} {self.hand_position}"
        )

    def read_motor_position(self, motor: MotorData) -> int:
        pos, comm_result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor.id, PRESENT_POS_MEM_ADDR
        )
        if comm_result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to read position of {motor.name}")
        elif error != 0:
            self.get_logger().error(f"""
Error occurred while reading position of motor {motor.name}
Error code: {self.packet_handler.getRxPacketError(error)}""")
            data, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, 63
            )
            self.get_logger().fatal(f"SHUTDOWN: {data}")
            data, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, 70
            )
            self.get_logger().fatal(f"ERROR: {data}")

        return pos

    def move_motor(self, motor: MotorData, step: int) -> int:
        current_position = self.read_motor_position(motor)
        desired_position = max(
            motor.min_limit, min(current_position + step, motor.max_limit)
        )

        comm_result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor.id, GOAL_POS_MEM_ADDR, desired_position
        )
        if comm_result != dxl.COMM_SUCCESS:
            self.get_logger().fatal(
                f"Failed to communicate with {motor.name}, result: {comm_result}"
            )
        elif error != 0:
            data, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, 63
            )
            self.get_logger().fatal(f"SHUTDOWN: {data}")
            data, comm_result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor.id, 70
            )
            self.get_logger().fatal(f"ERROR: {data}")

            self.get_logger().fatal(f"Failed to move {motor.name}, error: {error}")

        return desired_position


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
