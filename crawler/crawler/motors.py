import dynamixel_sdk as dxl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

PROTOCOL_VERSION = 2.0
DXL_ID_1, DXL_ID_2 = 1, 2
BAUDRATE = 57600
DEVICENAME = "/dev/ttyUSB0"
TORQUE_ENABLE, TORQUE_DISABLE = 1, 0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116

MOTOR_LIMITS = {
    DXL_ID_1: {"min": 1500, "max": 2500},
    DXL_ID_2: {"min": 1000, "max": 2000},
}

MOTORS_STEP = 250


class MotorsListener(Node):
    def __init__(self):
        super().__init__("motor_control_subscriber")

        self.create_subscription(
            Bool,
            "crawler_hand_move",
            (lambda self, msg: self.move_motor(DXL_ID_1, msg.data)),
            5,
        )
        self.create_subscription(
            Bool,
            "crawler_arm_move",
            (lambda self, msg: self.move_motor(DXL_ID_2, msg.data)),
            5,
        )

        self.port_handler = dxl.PortHandler(DEVICENAME)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

        self.setup_motors()

    def setup_motors(self):
        self.last_encoder_position = None

        if self.port_handler.openPort():
            print("Port opened")
        else:
            raise Exception("Failed to open the port")

        if self.port_handler.setBaudRate(BAUDRATE):
            print("Baudrate set to", BAUDRATE)
        else:
            raise Exception("Failed to set baudrate")

        for motor_id in [DXL_ID_1, DXL_ID_2]:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Failed to enable torque for Motor {motor_id}")
                dxl.error_pub.publish("Not able to properly enable motors")
            elif dxl_error != 0:
                print(
                    f"Error occurred while enabling torque for Motor {motor_id}:",
                    self.packet_handler.getRxPacketError(dxl_error),
                )
                dxl.error_pub.publish("Not able to properly enable motors")
            else:
                print(f"Motor {motor_id} is now stiff (torque enabled).")

    def move_motor(self, id: int, up: bool):
        current_position = self.read_motor_position(id)
        if up:
            desired_position = current_position + MOTORS_STEP
        else:
            desired_position = current_position - MOTORS_STEP

        min_limit = MOTOR_LIMITS[id]["min"]
        max_limit = MOTOR_LIMITS[id]["max"]

        desired_position = max(min_limit, min(desired_position, max_limit))

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, id, ADDR_GOAL_POSITION, desired_position
        )


def main(args=None):
    rclpy.init(args=args)
    motors_node = MotorsListener()
    rclpy.spin(motors_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
