from fastapi import FastAPI
from fastapi.responses import StreamingResponse,HTMLResponse
import uvicorn
import cv2
import rclpy
from picamera2 import Picamera2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import io
from time import sleep, time
import asyncio
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np

from .utils import draw_center_frame
from aruco_detection.aruco_detection import find_aruco_marker



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
                            # transformation_matrix = track_and_render_marker(
                            #         frame,
                            #         marker,
                            #         marker_id,
                            #         camera_matrix=stream_instace.camera_matrix,
                            #         distortion_coefficient=stream_instace.camera_dist,
                            #         marker_length=MARKER_LENGTH
                            #     )
                            # # Optionally compute an offset point transformation if tracking a point
                            # if TRACK_POINT:
                            #     T0point = transformation_matrix @ marker_point
                            #     tvec = T0point[:-1, -1]
                            # else:
                            #     tvec = transformation_matrix[:-1, -1]
                                
                            # x, y, z = tvec
                            self.get_logger().info("Processing Markers")

                        
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