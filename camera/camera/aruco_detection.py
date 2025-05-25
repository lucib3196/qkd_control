# ───── Standard Library ─────
from time import time
import threading
from collections import deque

# ───── Third-Party Libraries ─────
import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion  # type: ignore
from builtin_interfaces.msg import Time  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import CompressedImage  # type: ignore
from cv_bridge import CvBridge  # type: ignore
import rclpy  # type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore
from concurrent.futures import ThreadPoolExecutor

# ───── Local Modules ─────
from .utils import draw_center_frame
from aruco_detection.aruco_detection import find_aruco_marker, track_and_render_marker  # type: ignore
from custom_interfaces.msg import ArucoMsg  # type: ignore
from .camera_settings import ArucoParameters, calibrated_camera
from .utils.camera_calibration_utils import CalibratedCamera


class ArucoDectectionNode(Node):
    def __init__(self, aruco_params: ArucoParameters, calibrated_camera: CalibratedCamera, max_queue: int = 5, target_fps: float = 10):
        super().__init__('aruco_detection')

        self.aruco_params = aruco_params
        self.calibrated_camera = calibrated_camera
        self.bridge = CvBridge()
        self.frame_buffer = None
        self.frame_queue = deque(maxlen=max_queue)
        self.start_time = time()
        self.shutdown_flag = False
        self.lock = threading.Lock()
        self.thread_pool = ThreadPoolExecutor(max_workers=4)

        # ROS 2 image subscription
        self.subscription = self.create_subscription(
            CompressedImage,
            '/pan_tilt_images/image_raw',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data
        )

        # ROS 2 marker publisher
        self.publisher_ = self.create_publisher(
            ArucoMsg,
            "/aruco_detection",
            qos_profile=10
        )
        self.proccess_image_pub = self.create_publisher(
            CompressedImage,
            "/pan_tilt_images/processed",
            qos_profile=10
        )

    def listener_callback(self, msg: CompressedImage):
        if self.shutdown_flag:
            return

        try:
            self.get_logger().info("📷 Received image message.")

            # Safely decode the compressed image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Validate image shape before proceeding
            if cv_image is None:
                self.get_logger().warn("⚠️ cv_image is None after conversion.")
                return

            if len(cv_image.shape) != 3 or cv_image.shape[0] == 0 or cv_image.shape[1] == 0:
                self.get_logger().warn(f"⚠️ Invalid image shape: {cv_image.shape}")
                return

            # Submit to thread pool
            self.thread_pool.submit(self._process_frame, cv_image)

        except Exception as e:
            self.get_logger().warn(f"❌ Error converting image: {e}")


    def _process_frame(self, cv_image):
        # self.get_logger().info("Processing Image")
        try:
            if cv_image is None or len(cv_image.shape) != 3:
                self.get_logger().warn("Invalid or empty cv_image.")
            

            h, w = cv_image.shape[:2]
            draw_center_frame(cv_image, (int(w / 2), int(h / 2)))

            # Marker Detection and intiliaziatiuon
            default_dict = cv2.aruco.DICT_5X5_1000
            dict_id = getattr(cv2.aruco, self.aruco_params.aruco_dict_type, default_dict)
            if self.aruco_params.aruco_dict_type is None:
                self.get_logger().warn(f"Could not initialize the given aruco dict type it is either none or other: {self.aruco_params.aruco_dict_type} will default to {default_dict}")

            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            parameters = cv2.aruco.DetectorParameters()
            marker_array = find_aruco_marker(cv_image, aruco_dict, parameters)

            if marker_array:
                self.get_logger().info("Found marker(s), starting detection.")

                for marker, marker_id in marker_array:
                    self.get_logger().info(f"Processing marker ID: {marker_id}")
                    transformation_matrix = track_and_render_marker(
                        cv_image,
                        marker,
                        marker_id,
                        camera_matrix=self.calibrated_camera.camera_matrix,
                        distortion_coefficient=self.calibrated_camera.camera_dist,
                        marker_length=self.aruco_params.marker_length
                    )
                    

                    if transformation_matrix is None:
                        self.get_logger().warn("⚠️ Failed to compute transformation matrix.")

                    if self.aruco_params.track_point:
                        T0point = transformation_matrix @ self.aruco_params.marker_point
                        if T0point.shape[0] >= 3:
                            tvec = T0point[:3, -1].flatten()
                        else:
                            self.get_logger().warn("Invalid transformation matrix for track point.")

                    else:
                        tvec = transformation_matrix[:3, -1].flatten()

                    if len(tvec) != 3:
                        self.get_logger().warn("⚠️ Invalid translation vector.")

                    x, y, z = tvec

                    

                    # Send the message for aruco data
                    msg = ArucoMsg()
                    msg.id = int(marker_id)
                    msg.pose = Pose(
                        position=Point(x=x, y=y, z=z),
                        orientation=Quaternion(x=0.1, y=0.1, z=0.3, w=1.0)  # Replace with real orientation if available
                    )


                    elapsed_time = time() - self.start_time
                    msg.detection_time = Time(
                        sec=int(elapsed_time),
                        nanosec=int((elapsed_time - int(elapsed_time)) * 1e9)
                    )

                    msg.size = self.aruco_params.marker_length
                    msg.dictionary_type = self.aruco_params.aruco_dict_type
                    
                    self.publisher_.publish(msg)
                   
            # Will send the frame back
            ros_image =  self.bridge.cv2_to_compressed_imgmsg(
                cv_image,
                dst_format='jpg'               # <-- use 'jpg' or 'jpeg'
                )
            # Publish the processed image
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera'
            self.proccess_image_pub.publish(ros_image)

        except Exception as e:
            self.get_logger().warn(f"Error during frame processing: {e}")


def main(args=None):
    node =None
    try:
        rclpy.init(args=args)
        aruco_params = ArucoParameters()
        node = ArucoDectectionNode(aruco_params, calibrated_camera=calibrated_camera)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
