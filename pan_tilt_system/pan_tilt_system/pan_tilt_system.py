import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from sensor_msgs.msg import JointState  # type: ignore
from std_msgs.msg import Header, Int32, String, Float32  # type: ignore
import math
from custom_interfaces.msg import PanTiltMsg, ArucoMsg
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
    MANUaL ="manual"


class PanTiltSystem(Node):
    """A ROS2 Node for Pan and Tilt System that receives commands and moves accordingly."""

    def __init__(self):
        super().__init__("pan_tilt_system")

        self.get_logger().info("Setting up Pan and Tilt System")

        # Servo Configuration
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
        self.mode = PanTiltMode.TRACK
        self.mode_lock = threading.Lock()
        self.mode_thread = threading.Thread(target=self.handle_mode)
        self.mode_thread.start()
        
        ## Section for services and subscriptios
        # Create a publisher to publish the angles of the pan and tilt system
        self.angle_pub = self.create_publisher(
            msg_type=PanTiltMsg, topic="/pan_tilt_angles", qos_profile=10
        )
        timer_period = 0.1  # seconds
        self.angle_timer = self.create_timer(timer_period, self.angle_callback)
        
        # Main Subscription will get the aruco marker data for angle adjustments
        self.last_marker_position_time = None # This will be updated as the tracking goes
        self.create_subscription(
            ArucoMsg,
            "/aruco_data",
            self.pid_control, 
            10
        )
        
        ### Additional Subscriptions not yet complete
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

    def angle_callback(self):
        """Method that is periodically called by the timer, to publish the angles of the servos"""
        pantilt_angles = PanTiltMsg()
        pantilt_angles.pan_angle=self.servos["pan"].get("angle")
        pantilt_angles.tilt_angle=self.servos["tilt"].get("angle"),
        self.angle_pub.publish(pantilt_angles)


    def update_servo(self, servo_name: str, angle: float):
        if servo_name in self.servos and self.servos[servo_name]["servo"]:
            self.get_logger().info(f"Setting {servo_name} angle to {angle}")
            self.servos[servo_name]["servo"].angle = angle
            self.servos[servo_name]["angle"] = angle
        else:
            self.get_logger().warn(f"Servo {servo_name} not found or not initialized")
    
    def pid_control(self,msg:ArucoMsg):
        with self.mode_lock:
            if self.mode != PanTiltMode.TRACK:
                return  # Exit the callback if not in TRACK mode
        self.get_logger().info("Got ArucoData Need to Update Position")
        
        x = msg.pose.position.x
        y=msg.pose.position.y
        z=msg.pose.position.z
        detection_time = msg.detection_time.sec
        detection_time_nano = msg.detection_time.nanosec
        
        # Apply the pid control
        pan_error = np.degrees(np.arctan2(x, z))
        tilt_error = np.degrees(np.arctan(y,z))
        
        if self.last_marker_position_time:
            elapsed_time = detection_time_nano-self.last_marker_position_time
        else:
            elapsed_time = detection_time_nano
        
        self.pid_control_angle('pan',pan_error,elapsed_time)
        self.pid_control_angle('tilt',tilt_error,elapsed_time)
        
        self.last_marker_position_time = detection_time_nano
        
    def pid_control_angle(self, servo_name:str, error:float, elapsed_time:float):
        "Helper function meant to just calculate error, adjust the angles and clipd"
        with self.mode_lock:
            if self.mode != PanTiltMode.TRACK:
                return  # Exit the callback if not in TRACK mode
        current_angle = self.servos[servo_name].get('angle', 0)
        correction = self.servos[servo_name]['controller'].compute(error, elapsed_time)
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
