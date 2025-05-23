# Standard library imports
import io
import threading
from time import sleep, time

# Third-party imports
import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse
from geometry_msgs.msg import Point, Pose, Quaternion
from builtin_interfaces.msg import Time
from picamera2 import Picamera2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Local application imports
from .utils import draw_center_frame
from aruco_detection.aruco_detection import find_aruco_marker
from custom_interfaces.msg import ArucoMsg


class ArucoDectectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        # Aruco Detection Parameters 
        # ArUco marker parameters
        self.ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_1000
        self.MARKER_LENGTH = 0.033 # meters
        self.MARKER_ID = 5     # ID of the marker to track
        self.TRACK_POINT = False # Whether to track an offset point
        self.marker_point = np.array([0.1, 0, 0, 1]).reshape(4, 1)  # Offset of the point to track in meters
        
        
        # Intialize the CV Bridge
        self.bridge = CvBridge()
        # Intialize the subscribtion to the image
        self.frame_buffer = None
        self.subscription = self.create_subscription(
            Image,
            '/pan_tilt_images/image_raw',
            self.listener_callback,
            10
        )
        
        # Create a subscription to send the marker data
        self.aruco_pub = self.create_publisher(
            msg_type = ArucoMsg,
            topic="/aruco_detection",
            qos_profile=1)
        
        self.start_time = time()
        
    def listener_callback(self, msg):
        try:
            self.get_logger().info(f'Received an Image')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert to image to process this may be redundant need to check
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if ret:
                self.frame_buffer = jpeg.tobytes()
                if self.frame_buffer is not None:
                    nparr = np.frombuffer(self.frame_buffer, dtype=np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    draw_center_frame(frame, (480,480))
                    
                    # Marker detection using the predefined ArUco dictionary
                    aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT_TYPE)
                    parameters = cv2.aruco.DetectorParameters()
                    marker_array = find_aruco_marker(frame, aruco_dict, parameters)
                    if marker_array:
                        self.get_logger().info("Found Marker Doing Detection")
                        for marker, marker_id in marker_array:
                            self.get_logger().info("Processing Markers")
                            # Intialize
                            aruco_data = ArucoMsg()
                            aruco_data.id = marker_id
                            
                            # Get the Pose
                            aruco_data.pose = Pose()
                            aruco_data.pose.position = Point(x=1.0, y=0.0, z=2.0)
                            aruco_data.pose.orientation = Quaternion(x=0.1, y=0.1, z=0.3, w=1.0)
                            # Get the time
                            current_time = time.time()
                            elapsed_time = current_time - self.start_time
                            aruco_data.detection_time = Time()
                            aruco_data.detection_time.sec = int(elapsed_time)
                            aruco_data.detection_time.nanosec = int((elapsed_time - int(elapsed_time)) * 1e9)
                            
                            aruco_data.size = self.MARKER_LENGTH

                            # Set the dictionary type
                            aruco_data.dictionary_type = str(self.ARUCO_DICT_TYPE)

                            # Publish the message
                            self.publisher_.publish(aruco_data)

                        
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
        aruco_detector = ArucoDectectionNode()
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()