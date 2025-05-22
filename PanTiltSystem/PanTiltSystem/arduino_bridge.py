import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Int8
import serial
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        # Publishers
        self.publisher_ = self.create_publisher(Int32, "/data", 10)
        self.error_pub = self.create_publisher(Int8, "/arduino_error_pub", 10)

        # Subscriptions
        self.cmd_sub = self.create_subscription(
            String, "/arduino_cmd", self.send_command, 10
        )
        self.error_sub = self.create_subscription(
            Int8, "/marker_error", self.get_error, 10
        )

        # Serial connection
        self.ser = serial.Serial("/dev/ttyACM0", 9600)
        time.sleep(2)

        self.timer = self.create_timer(0.5, self.read_serial)

    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            self.get_logger().info(f"Serial line: {line}")
            if line:
                try:
                    msg = Int32()
                    msg.data = int(line)
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().warn(f"Invalid int received: '{line}'")
        else:
            self.get_logger().info(f"No messages from Arduino")

    def send_command(self, msg: String):
        command = msg.data.strip()
        if command:
            self.get_logger().info(f"Sending to Arduino: {command}")
            self.ser.write((command + "\n").encode())

    def get_error(self, msg: Int8):
        self.get_logger().info(f"Camera error: {msg.data}")
        self.send_error(msg)

    def send_error(self, msg: Int8):
        value = max(0, min(msg.data, 100))
        byte_val = bytes([value])
        self.ser.write(byte_val)
        self.get_logger().info(f"Sent error code to Arduino: {byte_val}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()
