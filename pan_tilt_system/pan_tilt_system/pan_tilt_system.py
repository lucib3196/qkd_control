import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from sensor_msgs.msg import JointState  # type: ignore
from std_msgs.msg import Header, Int32, String, Float32  # type: ignore
import math
from time import time
import threading
import numpy as np
from enum import Enum
from custom_interfaces.msg import PanTiltMsg, ArucoMsg, PanTiltError
from .PID import PIDController

# GPIO setup for AngularServo
from gpiozero import Device, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory

# Attempt to use pigpio pin factory
try:
    Device.pin_factory = PiGPIOFactory
except Exception:
    print(f"Could not change Pin Factory using default {Device.pin_factory}")


class PanTiltMode(Enum):
    IDLE = "idle"
    TRACK = "track"
    SCOUT = "scout"
    MANUAL = "manual"


class PanTiltSystem(Node):
    """A ROS2 Node for Pan and Tilt System that receives commands and moves accordingly."""

    def __init__(self, servo_configs: dict[str, dict]):
        super().__init__("pan_tilt_system")
        self.get_logger().info("Setting up Pan and Tilt System")

        # Injected servo configurations
        self.servos = servo_configs
        self.initialize_servos()

        # Node state
        self.running = True
        self.mode = PanTiltMode.TRACK
        self.mode_lock = threading.Lock()
        self.mode_thread = threading.Thread(target=self.handle_mode)
        self.mode_thread.start()

        # Publishers
        self.angle_pub = self.create_publisher(PanTiltMsg, "/pan_tilt_angles", 10)
        self.error_pub = self.create_publisher(PanTiltError, "/pan_tilt_error", 10)

        # Timer for publishing angles
        self.create_timer(0.1, self.angle_callback)

        # Subscriptions
        self.create_subscription(ArucoMsg, "/aruco_data", self.pid_control, 10)
        self.create_subscription(Int32, "/angle_command/pan_angle",
                                 lambda msg: self.update_servo("pan", msg.data), 10)
        self.create_subscription(Int32, "/angle_command/tilt_angle", 
                                 lambda msg: self.update_servo("tilt", msg.data), 10)
        self.create_subscription(String, "/pan_tilt_mode", self.mode_callback, 10)
        

    def initialize_servos(self):
        for name, config in self.servos.items():
            # Initialize AngularServo
            try:
                servo = AngularServo(
                    config["gpio"],
                    min_angle=config["angle_range"][0],
                    max_angle=config["angle_range"][1],
                )
                servo.angle = config.get("angle", 0)
                config["servo"] = servo
                self.get_logger().info(f"Initialized {name} servo on GPIO {config['gpio']}")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize {name} servo: {e}")

            # Initialize PID controller
            try:
                kp, ki, kd = config.get("pid", (0, 0, 0))
                controller = PIDController(Kp=kp, Ki=ki, Kd=kd)
                config["controller"] = controller
                self.get_logger().info(
                    f"Initialized {name} PID Controller (Kp={kp}, Ki={ki}, Kd={kd})"
                )
            except Exception as e:
                self.get_logger().warn(f"Failed to init controller for {name}: {e}")
        return self

    def angle_callback(self):
        msg = PanTiltMsg()
        msg.pan_angle = float(self.servos["pan"]["angle"])
        msg.tilt_angle = float(self.servos["tilt"]["angle"])
        self.angle_pub.publish(msg)
        self.get_logger().info(f"Published pan: {msg.pan_angle}, tilt: {msg.tilt_angle}")

    def update_servo(self, servo_name: str, angle: float):
        config = self.servos.get(servo_name)
        if config and config.get("servo"):
            self.get_logger().info(f"Setting {servo_name} angle to {angle}")
            config["servo"].angle = angle
            config["angle"] = angle
        else:
            self.get_logger().warn(f"Servo {servo_name} not found or not initialized")

    def pid_control(self, msg: ArucoMsg):
        with self.mode_lock:
            if self.mode != PanTiltMode.TRACK:
                return
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        nano = msg.detection_time.nanosec
        pan_error = np.degrees(np.arctan2(x, z))
        tilt_error = np.degrees(np.arctan(y, z))
        elapsed = (nano - self.last_marker_position_time) if hasattr(self, 'last_marker_position_time') else nano
        self.pid_control_angle('pan', pan_error, elapsed)
        self.pid_control_angle('tilt', tilt_error, elapsed)
        self.last_marker_position_time = nano
        err_msg = PanTiltError(pan_error=pan_error, tilt_error=tilt_error,
                               total_error=np.hypot(pan_error, tilt_error))
        self.error_pub.publish(err_msg)

    def pid_control_angle(self, servo_name: str, error: float, elapsed: float):
        with self.mode_lock:
            if self.mode != PanTiltMode.TRACK:
                return
        cfg = self.servos[servo_name]
        correction = cfg['controller'].compute(error, elapsed)
        new_angle = np.clip(
            cfg.get('angle', 0) - correction,
            cfg['angle_range'][0],
            cfg['angle_range'][1]
        )
        self.get_logger().info(f"Servo {servo_name} New Angle: {new_angle}")
        self.update_servo(servo_name, new_angle)

    def mode_callback(self, msg: String):
        try:
            with self.mode_lock:
                self.mode = PanTiltMode(msg.data.lower().strip())
            self.get_logger().info(f"Mode set to {self.mode}")
        except ValueError:
            self.get_logger().warn(f"Unknown Mode: {msg.data}")

    def handle_mode(self):
        while self.running:
            with self.mode_lock:
                mode = self.mode
            if mode == PanTiltMode.IDLE:
                self.idle()
            elif mode == PanTiltMode.TRACK:
                self.track()
            elif mode == PanTiltMode.SCOUT:
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
            for angle in (-90, 0, 90):
                self.update_servo("pan", angle)
                time.sleep(2)
        except Exception as e:
            self.get_logger().warn(f"Scout Mode failed: {e}")

    def destroy_node(self):
        self.running = False
        self.mode_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # Define servo configurations externally
    servo_configs = {
        "pan": {
            "gpio": 17,
            "angle": 0,
            "angle_range": [-90, 90],
            "pid": [0.2, 0, 0.2],
        },
        "tilt": {
            "gpio": 27,
            "angle": 0,
            "angle_range": [-90, 90],
            "pid": [0.1, 0, 0.2],
        }
    }
    pan_tilt_system = PanTiltSystem(servo_configs)
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
