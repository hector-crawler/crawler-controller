import os

import dynamixel_sdk as dxl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"

TORQUE_OFF = 0
TORQUE_ON = 1
TORQUE_MEM_ADDR = 64
GOAL_POS_MEM_ADDR = 116
PRESENT_POS_MEM_ADDR = 132


class MotorData:
    def __init__(self, name: str, id: int, min_limit: int, max_limit: int) -> None:
        self.name = name
        self.id = id
        self.min_limit = min_limit
        self.max_limit = max_limit


Arm = MotorData(
    "Arm",
    1,
    1500,
    2500,
)

Hand = MotorData(
    "Hand",
    2,
    1000,
    2000,
)


class MotorsNode(Node):
    def __init__(self):
        super().__init__("crawler_motors")

        self.motors = Motors()

        queue_len = 5
        self.create_subscription(Int32, "/crawler/arm/move", self.move_arm, queue_len)
        self.arm_publisher = self.create_publisher(
            Int32, "/crawler/arm/position", queue_len
        )
        self.create_subscription(Int32, "/crawler/hand/move", self.move_hand, queue_len)
        self.hand_publisher = self.create_publisher(
            Int32, "/crawler/hand/position", queue_len
        )

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
    #return MockMotors() if os.environ.get("CRAWLER_ENV") == "dev" else PhysicalMotors()
    return MockMotors()


class PhysicalMotors:
    def __init__(self):
        self.port_handler = dxl.PortHandler(DEVICENAME)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

        self.last_encoder_position = None

        fail = -1
        if self.port_handler.openPort() == fail:
            raise Exception("Failed to open the port")
        print("Port opened")

        if self.port_handler.setBaudRate(BAUDRATE) == fail:
            raise Exception("Failed to set baudrate")
        print("Baudrate set to", BAUDRATE)

        for motor in [Arm, Hand]:
            self.setup_motor(motor)

    def setup_motor(self, motor: MotorData) -> None:
        comm_result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor.id, TORQUE_MEM_ADDR, TORQUE_ON
        )
        if comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to enable torque for {motor.name}")
            dxl.error_pub.publish(f"Not able to properly enable motor {motor.id}")
        elif error != 0:
            print(
                f"Error occurred while enabling torque for {motor.name}:",
                self.packet_handler.getRxPacketError(error),
            )
            dxl.error_pub.publish(f"Not able to properly enable motor {motor.id}")
        else:
            print(f"{motor.name} motor is now stiff (torque enabled).")

    def read_motor_position(self, motor: MotorData) -> int:
        pos, comm_result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor.id, PRESENT_POS_MEM_ADDR
        )
        if comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to read position of {motor.name}")
            dxl.error_pub.publish(f"Not able to read position of motor {motor.id}")
        elif error != 0:
            print(
                f"Error occurred while reading position of motor {motor.name}",
                self.packet_handler.getRxPacketError(error),
            )
            dxl.error_pub.publish(
                f"Not able to properly read position of motor {motor.id}"
            )
        else:
            print(f"Got position {pos} from motor {motor.name}")
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
            msg = f"Failed to communicate with {motor.name}, result: {comm_result}"
            print(msg)
            dxl.error_pub.publish(msg)
        elif error != 0:
            msg = f"Failed to move {motor.name}, error: {error}"
            print(msg)
            dxl.error_pub.publish(msg)
        else:
            print(f"Moved {motor.name} by {step} to {desired_position}")

        return desired_position

    def move_arm(self, step: int) -> int:
        return self.move_motor(Arm, step)

    def move_hand(self, step: int) -> int:
        return self.move_motor(Hand, step)


class MockMotors:
    def __init__(self) -> None:
        self.hand_position = 50
        self.arm_position = 50

    def move_arm(self, step: int) -> int:
        self.arm_position = max(0, min(100, self.arm_position + step))
        return self.arm_position

    def move_hand(self, step: int) -> int:
        self.hand_position = max(0, min(100, self.hand_position + step))
        return self.hand_position


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotorsNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
