import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import Header, Int32, String, Float32, Int8  # type: ignore
import math
import random


class Camera(Node):
    """A ROS2 Node that does the camera."""

    def __init__(self):
        super().__init__("camera")

        # Create publishers which will publish the data
        self.pan_error_pub = self.create_publisher(
            msg_type=Float32, topic="/error/pan", qos_profile=1
        )
        self.tilt_error_pub = self.create_publisher(
            msg_type=Float32, topic="/error/tilt", qos_profile=1
        )

        self.total_error = self.create_publisher(
            msg_type=Int8, topic="/marker_error", qos_profile=1
        )

        timer_period: float = 0.1
        self.timer = self.create_timer(timer_period, self.error_callback)

    def error_callback(self):
        pan_error = Float32()
        pan_error.data = random.uniform(0, 50)
        tilt_error = Float32()
        tilt_error.data = random.uniform(0, 50)
        total_error = Int8()
        total_error.data = math.floor(random.uniform(0, 100))

        self.total_error.publish(total_error)
        self.pan_error_pub.publish(pan_error)
        self.tilt_error_pub.publish(tilt_error)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        camera_node = Camera()

        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
