import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from sensor_msgs.msg import JointState  # type: ignore
from std_msgs.msg import Header, Int32, String, Float32  # type: ignore
import math
from enum import Enum
import time
import threading
from .PID import PIDController
import numpy as np
# Set Up for GPIO
from gpiozero import Device, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

Device.pin_factory = PiGPIOFactory()


class PanTiltMode(Enum):
    IDLE = "idle"
    TRACK = "track"
    SCOUT = "scout"


class PanTiltSystem(Node):
    """A ROS2 Node for Pan and Tilt System that receives commands and moves accordingly."""

    def __init__(self):
        super().__init__("pan_tilt_system")

        self.get_logger().info("Setting up Pan and Tilt System")

        # Create a publisher to publish the angles of the pan and tilt system
        self.angle_pub = self.create_publisher(
            msg_type=JointState, topic="/pan_tilt_angles", qos_profile=10
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize servos
        self.servos = {
            "pan": {
                "gpio": 17,
                "servo": None,
                "angle": 0,
                "angle_range": [-90, 90],
                "pid": [0.2, 0, 0.2],
                "controller": None,
            },
            "tilt": {
                "gpio": 27,
                "servo": None,
                "angle": 0,
                "angle_range": [-90, 90],
                "pid": [0.1, 0.0, 0.2],
                "controller": None,
            },
        }
        self.initialize_servos()

        # Define the mode of the system
        self.running = True
        self.mode = PanTiltMode.SCOUT
        self.mode_lock = threading.Lock()
        self.mode_thread = threading.Thread(target=self.handle_mode)
        self.mode_thread.start()

        # Create subscriptions
        self.create_subscription(
            Int32,
            "/angle_command/pan_angle",
            lambda msg: self.update_servo("pan", msg.data),
            10,
        )
        self.create_subscription(
            Int32,
            "/angle_command/tilt_angle",
            lambda msg: self.update_servo("tilt", msg.data),
            10,
        )
        self.create_subscription(String, "/pan_tilt_mode", self.mode_callback, 10)
        self.create_subscription(
            Float32,
            "/error/pan",
            lambda msg: self.pid_control(servo_name="pan", error=msg.data),
            10,
        )
        self.create_subscription(
            Float32,
            "/error/tilt",
            lambda msg: self.pid_control(servo_name="tilt", error=msg.data),
            10,
        )

    def initialize_servos(self):
        for name, config in self.servos.items():
            try:
                servo = AngularServo(
                    config["gpio"],
                    min_angle=config["angle_range"][0],
                    max_angle=config["angle_range"][1],
                )
                servo.angle = config["angle"]
                config["servo"] = servo
                self.get_logger().info(
                    f"Initialized {name} servo on GPIO {config['gpio']}"
                )
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize {name} servo: {e}")

            try:
                controller = PIDController(
                    Kp=config["pid"][0],
                    Ki=config["pid"][1],
                    Kd=config["pid"][2],
                )
                self.get_logger().info(
                    "Initialized {} PID Controller (Ki, Kp, Kd) {}".format(
                        name, "\n".join(str(x) for x in config.get("pid", []))
                    )
                )
                config["controller"] = controller
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to initialize controller for {name} servo: {e}"
                )
        return self

    def timer_callback(self):
        """Method that is periodically called by the timer."""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["pan_joint", "tilt_joint"]

        joint_state.position = [
            self.servos["pan"].get("angle"),
            self.servos["tilt"].get("angle"),
        ]

        # Explicitly define that these values are empty
        joint_state.velocity = []
        joint_state.effort = []

        self.angle_pub.publish(joint_state)

    def update_servo(self, servo_name: str, angle: float):
        if servo_name in self.servos and self.servos[servo_name]["servo"]:
            self.get_logger().info(f"Setting {servo_name} angle to {angle}")
            self.servos[servo_name]["servo"].angle = angle
            self.servos[servo_name]["angle"] = angle
        else:
            self.get_logger().warn(f"Servo {servo_name} not found or not initialized")

    def pid_control(self, servo_name: str, error: float):
        with self.mode_lock:
            if self.mode != PanTiltMode.TRACK:
                return  # Exit the callback if not in TRACK mode

        self.get_logger().info("Error Detected Updating")
        self.get_logger().info(
            f"""
            Servo: {servo_name} Current Error: {error}
            """
        )
        current_angle = self.servos[servo_name].get('angle', 0)
        correction = self.servos[servo_name]['controller'].compute(error, 0.1)
        new_angle = np.clip(
            current_angle - correction,
            self.servos[servo_name]['angle_range'][0],
            self.servos[servo_name]['angle_range'][1]
        )
        
        self.get_logger().info(
            f"""
            Servo: {servo_name} New Angle: {new_angle}
            """
        )
        self.update_servo(servo_name, new_angle)


    def mode_callback(self, msg):
        new_mode = msg.data.lower().strip()
        try:
            with self.mode_lock:
                self.mode = PanTiltMode(new_mode)
            self.get_logger().info(f"Mode set to {self.mode}")
        except ValueError:
            self.get_logger().warn(f"Unknown Mode: {new_mode}. Resuming {self.mode}")

    def handle_mode(self):
        while self.running:
            with self.mode_lock:
                current_mode = self.mode
            if current_mode == PanTiltMode.IDLE:
                self.idle()
            elif current_mode == PanTiltMode.TRACK:
                self.track()
            elif current_mode == PanTiltMode.SCOUT:
                self.scout()
            time.sleep(0.1)

    def idle(self):
        self.get_logger().info("Entering Idle Mode")
        self.update_servo("pan", 0)
        self.update_servo("tilt", 0)
        time.sleep(1)

    def track(self):
        self.get_logger().info("Entering Track Mode")

    def scout(self):
        self.get_logger().info("Entering Scout Mode")
        try:
            self.update_servo("pan", -90)
            time.sleep(2)
            self.update_servo("pan", 0)
            time.sleep(2)
            self.update_servo("pan", 90)
            time.sleep(2)
        except Exception as e:
            self.get_logger().warn(f"Scout Mode failed: {e}")

    def destroy_node(self):
        self.running = False
        self.mode_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    pan_tilt_system = PanTiltSystem()
    executor = MultiThreadedExecutor()
    executor.add_node(pan_tilt_system)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pan_tilt_system.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
