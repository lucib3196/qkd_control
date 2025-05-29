import sys
import rclpy
import threading
from enum import Enum
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import Float32, Float32MultiArray, Header,String
from serial import SerialException
from thorlabs_elliptec import ELLx, ELLError
from custom_interfaces.msg import PanTiltMsg, ArucoMsg, PanTiltError, HalfWavePlate
from rclpy.executors import MultiThreadedExecutor  # type: ignore

class HWPState(Enum):
    MANUAL = "manual"
    BASIS_ENCODING = "basis_encoding"
    CALIBRATION = "calibration"
    SWEEP = "sweep"
    IDLE = "idle"

class HWPController(Node):
    def __init__(self):
        super().__init__("hwp_controller")
        self.get_logger().set_level(LoggingSeverity.WARN)

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("state", "sweep")
        self.declare_parameter("manual_control", "hwp/manual/angle_control")
        self.declare_parameter("calibration", "hwp/calibrate")

        # Get current mode
        self.mode = HWPState(
            self.get_parameter("state").get_parameter_value().string_value
        )

        self.running = True
        self.mode_lock = threading.Lock()

        # Publisher for status
        self.status_pub = self.create_publisher(String, "/hwp/status", 10)

        # Subscription for manual angle changes
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("manual_control").get_parameter_value().string_value,
            self.manual_angle_change,
            10,
        )

        # Serial connection
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.initialize_halfwaveplate()

        # Start mode handler
        self.mode_thread = threading.Thread(target=self.handle_mode)
        self.mode_thread.start()

        # Angle subscription
        self.create_subscription(Float32, "hwp/angle", self.angle_callback, 10)

    def initialize_halfwaveplate(self):
        self.get_logger().info(f"Opening HWP on port {self.port}")
        try:
            self.stage = ELLx(serial_port=self.port)
        except SerialException as e:
            self.get_logger().error(f"Could not open {self.port}: {e}")
            sys.exit(1)

        try:
            self.get_logger().info("Homing stage...")
            self.stage.home(blocking=True)
            self.get_logger().info("Homing complete.")
        except ELLError as e:
            self.get_logger().error(f"Elliptec error during home: {e}")
            sys.exit(2)

    def manual_angle_change(self, msg: Float32MultiArray):
        with self.mode_lock:
            if self.mode != HWPState.MANUAL:
                self.get_logger().warn(
                    f"Error: Mode is set to {self.mode}, must be 'manual'"
                )
                return
        for angle in msg.data:
            self.angle_change(angle)

    def calibrate(self):
        self.get_logger().info("Calibrating (setting angle to 0.0)")
        self.angle_change(0.0)

    def sweep(self):
        angle_ranges = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0]
        for angle in angle_ranges:
            self.angle_change(angle)

    def angle_callback(self, msg: Float32):
        if self.mode == HWPState.MANUAL:
            self.angle_change(msg.data)

    def angle_change(self, angle):
        if not 0.0 <= angle <= 360.0:
            self.get_logger().warn(f"Angle {angle} out of range [0.0, 360.0]")

        self.get_logger().info(f"Moving to angle {angle}")
        try:
            self.stage.move_absolute(angle, blocking=True)
            self.publish_status(angle, f"set_angle {angle}")
        except ELLError as e:
            self.get_logger().error(f"Elliptec move error: {e}")

    def handle_mode(self):
        while self.running:
            with self.mode_lock:
                if self.mode == HWPState.SWEEP:
                    self.sweep()
                elif self.mode == HWPState.CALIBRATION:
                    self.calibrate()
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_status(self, angle: float, command: str):
        msg = HalfWavePlate()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "hwp_base"

        msg.state = self.mode.value
        msg.angle = angle
        msg.last_command = command
        msg.is_moving = False  # Update if motion tracking is implemented

        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.mode_thread.join()
        try:
            self.stage.close()
            self.get_logger().info("Stage connection closed.")
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HWPController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
