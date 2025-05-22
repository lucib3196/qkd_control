import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __int__(self):
        super().__init__("camera_publisher")
        
       
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imagmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Camera not returning frame.")
            
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher("camera_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()