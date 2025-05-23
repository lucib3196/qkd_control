import cv2
import rclpy
from picamera2 import Picamera2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import io

from aruco_detection.aruco_detection import track_and_render_marker

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        
        # Initialize the Pi Camera
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration())
        except Exception as e:
            self.get_logger().warn(f"Error Intializing Camera {e}")
        
        # Bridge for converting OpenCV images to ROS 2 images
        self.bridge = CvBridge()

        # Publisher
        self.topic_name = "/pan_tilt_images/image_raw"
        self.queue_size = 10
        self.image_pub = self.create_publisher(Image, self.topic_name, self.queue_size)

        # Timer
        self.timer_period = 0.05  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.image_count = 0
        self.get_logger().info("Initialized the Camera Node")
        
        
    # Call Back Function that will handle the publishing of the images
    def timer_callback(self):
        
        self.picam2.start()
        buffers, metadata = self.picam2.capture_buffers(["main"])
        buffer = buffers[0]
        image_array = self.picam2.helpers.make_array(
            buffer, self.picam2.camera_configuration()["main"]
        )
        image_bgr = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            self.image_pub.publish(ros_image)
            self.image_count += 1
            self.get_logger().info(f"Published image #{self.image_count}")
        except Exception as e:
            self.get_logger().warn(f"Error {e}")

        
        


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()