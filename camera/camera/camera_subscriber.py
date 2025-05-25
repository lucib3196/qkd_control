import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import cv2
from fastapi import FastAPI, Request
from fastapi.responses import StreamingResponse, HTMLResponse
import uvicorn
import asyncio
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import CompressedImage # type: ignore
from cv_bridge import CvBridge # type: ignore
from uvicorn import Config, Server
from .utils import draw_center_frame
from rclpy.qos import qos_profile_sensor_data # type: ignore

class VideoFrameHandler:
    def __init__(self, node_logger, max_queue: int = 5, target_fps: float = 10):
        self.logger = node_logger
        self.bridge = CvBridge()
        self.frame_buffer = None
        self.frame_queue = deque(maxlen=max_queue)
        self.lock = threading.Lock()
        self.shutdown_flag = False
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        self.min_interval = 1.0 / target_fps

    def on_image_msg(self, msg: CompressedImage):
        if self.shutdown_flag:
            return
        try:
            self.logger.info("📷 Received image message.")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.thread_pool.submit(self._process_frame, cv_image)
        except Exception as e:
            self.logger.warn(f"❌ Error converting image: {e}")

    def _process_frame(self, cv_image):
        try:
            h, w = cv_image.shape[:2]
           
            ### The following are debug statements for camera uncomment if testing leave commented for performance
             # self.logger.info(f"📐 Processing frame with size: width={w}, height={h}")
            # # Save raw frame for debug
            # timestamp = int(time.time() * 1000)
            # debug_image_path =f"frame_debug_{timestamp}.png"
            # success = cv2.imwrite(debug_image_path, cv_image)
            # if success:
            #     self.logger.info(f"🖼️ Saved raw frame to: {debug_image_path}")
            # else:
            #     self.logger.warn("⚠️ Failed to save raw frame for debugging.")

            # Draw something to verify position
            
            # Actual Processing


            # Encode as JPEG
            ret, jpeg = cv2.imencode(
                '.jpg', cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, 80]  # Remove OPTIMIZE for now
            )
            if not ret:
                self.logger.warn("⚠️ JPEG encoding failed.")
                return

            data = jpeg.tobytes()
            with self.lock:
                self.frame_buffer = data
                self.frame_queue.append(data)
        except Exception as e:
            self.logger.warn(f"❌ Error processing frame: {e}")


    async def generate_frames(self):
        last_time = 0.0
        while not self.shutdown_flag:
            current = time.time()
            if self.frame_buffer and (current - last_time) >= self.min_interval:
                with self.lock:
                    frame = self.frame_buffer
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n'
                )
                last_time = current
            await asyncio.sleep(self.min_interval)
    def stop(self):
        self.shutdown_flag = True
        self.thread_pool.shutdown(wait=False)
        self.logger.info("🛑 VideoFrameHandler shutdown complete.")


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.frame_handler = VideoFrameHandler(self.get_logger())

        self.subscription = self.create_subscription(
            CompressedImage,
            "/pan_tilt_images/processed",
            self.frame_handler.on_image_msg,
            qos_profile=qos_profile_sensor_data
        )

        self.app = FastAPI()
        self._setup_routes()

    def _setup_routes(self):
        @self.app.get('/', response_class=HTMLResponse)
        def index():
            return HTMLResponse(content="""
            <html><head><title>Face Tracking Stream</title></head>
            <body><h1>Face Tracking</h1>
              <img src=\"/video_feed\" />
            </body></html>""")

        @self.app.get(
            '/video_feed',
            response_class=StreamingResponse,
            responses={200: {'content': {'multipart/x-mixed-replace; boundary=frame': {}}}}
        )
        async def video_feed(request: Request):
            return StreamingResponse(
                self.frame_handler.generate_frames(),
                media_type='multipart/x-mixed-replace; boundary=frame',
                headers={
                    'Cache-Control': 'no-cache',
                    'Pragma': 'no-cache',
                    'Expires': '0'
                }
            )


def main():
    rclpy.init()
    node = CameraSubscriber()

    config = Config(
        app=node.app,
        host="0.0.0.0",
        port=8001,
        log_level="warning",
        reload=False,
    )
    server = Server(config)
    api_thread = threading.Thread(target=server.run, daemon=True)
    api_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⌨️ KeyboardInterrupt received. Initiating shutdown...")
    finally:
        node.frame_handler.stop()
        node.destroy_node()
        server.should_exit = True
        api_thread.join()
        rclpy.shutdown()
        node.get_logger().info("✅ Server and ROS2 node shutdown complete.")


if __name__ == '__main__':
    main()
