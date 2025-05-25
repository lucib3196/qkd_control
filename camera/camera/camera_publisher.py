# ───────────────────────────────────────────────────────────────
# 📦 Standard Library
import io

# ───────────────────────────────────────────────────────────────
# 🧪 Third-Party Libraries
import cv2
from picamera2 import Picamera2
from libcamera import controls  # type: ignore
from cv_bridge import CvBridge  # type: ignore

# ───────────────────────────────────────────────────────────────
# 🤖 ROS 2
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import CompressedImage  # type: ignore

# ───────────────────────────────────────────────────────────────
# 🧠 Local Modules
from .utils.camera_calibration_utils import CalibratedCamera
from .camera_settings import calibrated_camera, camera_config




class CameraPublisher(Node):
    def __init__(
        self,
        calibrated_camera: CalibratedCamera,
        framerate: int = 90,
        brightness: float = 0.0,
        contrast: float = 0.8,
        saturation: float = 1.3,
    ):
        super().__init__("camera_publisher")

        # (we override sensor resolution to 2304×1296 for streaming)
        self.width = calibrated_camera.frame_width
        self.height = calibrated_camera.frame_height
        self.framerate = framerate

        # Calibration data (for undistort later)
        self.calibrated_camera = calibrated_camera
        self.camera_matrix = calibrated_camera.new_cam_mtx
        self.camera_dist = calibrated_camera.camera_dist

        # Initialize Picamera2
        try:
            self.picam2 = Picamera2()
            frame_us = int(1_000_000 / self.framerate)

            # Module 3 (IMX708) optimized video config:
            video_config = self.picam2.create_video_configuration(
                main={
                    "size": (self.width, self.height),
                    "format": "YUV420",
                },
                controls={
                    # Autofocus
                    "AfMode": controls.AfModeEnum.Continuous,
                    "AfSpeed": controls.AfSpeedEnum.Fast,

                    # Auto‐exposure & framerate cap (→ max 1e6/framerate μs)
                    "AeEnable": True,
                    "FrameDurationLimits": (frame_us, frame_us),

                    # Gain & noise reduction
                    "AnalogueGain": 1.0,
                    "NoiseReductionMode": 1,

                    # Color balance
                    "AwbEnable": True,
                    "AwbMode": controls.AwbModeEnum.Auto,

                    # User tweaks
                    "Brightness": float(brightness),
                    "Contrast":   float(contrast),
                    "Saturation": float(saturation),
                },
                buffer_count=6,  # larger buffer for smoother streaming
            )

            self.picam2.configure(video_config)
            self.picam2.start()
            self.get_logger().info(
                f"Camera initialized at {self.width}×{self.height} @ {self.framerate} FPS"
            )

        except Exception as e:
            self.get_logger().warn(f"Error initializing camera: {e}")

        # Bridge & ROS publisher
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            CompressedImage, "/pan_tilt_images/image_raw", 10
        )

        # Spin at framerate
        self.timer = self.create_timer(1.0 / self.framerate, self.timer_callback)
        self.shutdown_flag = False
        self.image_count=0
        
    # Call Back Function that will handle the publishing of the images
    def timer_callback(self):
        
        buffers, metadata = self.picam2.capture_buffers(["main"])
        buffer = buffers[0]
        image_array = self.picam2.helpers.make_array(
            buffer, self.picam2.camera_configuration()["main"]
        )
        bgr = cv2.cvtColor(image_array, cv2.COLOR_YUV2BGR_I420)
        try:
            ros_image =  self.bridge.cv2_to_compressed_imgmsg(
                bgr,
                dst_format='jpg'               # <-- use 'jpg' or 'jpeg'
            )
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera'
            self.image_pub.publish(ros_image)
            self.image_count += 1
            self.get_logger().info(f"Published image #{self.image_count}")
        except Exception as e:
            self.get_logger().warn(f"Error {e}")

    def destroy_node(self):
        if hasattr(self, 'picam2'):
            self.picam2.stop()
        super().destroy_node()
        

def main(args=None):
    """
    Main entry point for the camera publisher node.
    Initializes ROS2, loads camera settings, and starts publishing frames.
    Ensures all resources are safely destroyed on shutdown.
    """


    camera_publisher = None

    try:
        rclpy.init(args=args)

        camera_publisher = CameraPublisher(
            calibrated_camera=calibrated_camera,
            framerate=camera_config["framerate"],
            brightness=camera_config["brightness"],
            contrast=camera_config["contrast"],
            saturation=camera_config["saturation"]
        )

        rclpy.spin(camera_publisher)

    except KeyboardInterrupt:
        print("🔌 KeyboardInterrupt received. Shutting down...")
    except Exception as e:
        print(f"❌ Error starting camera publisher: {e}")
    finally:
        # Gracefully destroy node if it was created
        if camera_publisher is not None:
            camera_publisher.destroy_node()

        # Only shutdown if ROS is still running
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()