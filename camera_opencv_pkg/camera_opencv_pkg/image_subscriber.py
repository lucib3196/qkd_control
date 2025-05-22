import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics



class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.img_sub = self.create_subscription(
            msg_type=Image,
            topic = "/camera/image_raw",
            callback = self.listener_callback,
            qos_profile=10
        )
        self.get_logger().info('Initialized Bridge')
        
        self.bridge = CvBridge()
        
        self.get_logger().info("Initializing the Camera")    
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration()
            picam2.start(config)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMX500 or Picamera2: {e}")
            sys.exit(1)

            
        # Intialize Picamera2
        try:
            self.get_logger().info("Initializing the PiCamera")  
            self.picam2.start(self.config, show_preview=False)  # show_preview=False, since we use OpenCV for display
        except Exception as e:
            self.get_logger().error(f"Failed to start Picamera2: {e}")
            sys.exit(1)
        
        # Intialize the publisher
        try:
            self.raw_image_publisher = self.create_publisher(Image, "camera/raw_image",10)
            self.detection_image_publisher = self.create_publisher(Image, "camera/detection_image",10)
        except Exception as e:
            self.get_logger().error(f"Failed to create publishers: {e}")
            sys.exit(1)
        # Timer to process frames at the specified inference rate
        
        try:
            timer_period = 1
            self.timer = self.create_timer(timer_period, self.timer_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to create timer: {e}")
            sys.exit(1)

        self.get_logger().info("raspberrypi_ai_camera_ros2 Node has been started.")
        
    def timer_callback(self):
        """Callback function to process and publish frames at each timer tick."""
        try:
            if not rclpy.ok():
                self.get_logger().warn("ROS2 is shutting down. Skipping frame processing.")
                return

            request = self.picam2.capture_request()
            if request is None:
                self.get_logger().debug("No request available.")
                return

            # Check if the image data exists
            with MappedArray(request, 'main') as m:
                raw_image = m.array.copy()
                if raw_image is None or raw_image.size == 0:
                    self.get_logger().warn('No image data received from the camera.')
                    request.release()
                    return  # Skip the image

                raw_image_bgr = cv2.cvtColor(raw_image, cv2.COLOR_RGB2BGR)

            self.frame_counter += 1
            

            # Publish the raw image
            try:
                raw_image_msg = self.bridge.cv2_to_imgmsg(raw_image_bgr, encoding='bgr8')
                self.get_logger().debug(f'This is the raw image type {type(raw_image_msg)}, contents: {raw_image_bgr.shape}')
                
                current_time = self.get_clock().now()
                raw_image_msg.header.stamp = current_time.to_msg()
                raw_image_msg.header.frame_id = self.frame_id

                self.raw_image_publisher.publish(raw_image_msg)
                self.get_logger().debug(f'Published raw image {self.frame_counter}')
            except Exception as e:
                self.get_logger().warn(f'Could not publish image: {e}')
        except Exception as e:
            self.get_logger().warn(f'An error has occurred: {e}')



    def destroy_node(self):
        self.get_logger().info('Shutting down raspberry pi')
        try:
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f'Error destorying OpenCv windows {e}')
        try:
            self.picam2.stop()
        except Exception as e:
            self.get_logger().error(f'Could not stop the pi camera {e}')
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()