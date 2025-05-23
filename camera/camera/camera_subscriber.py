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

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.frame_buffer = None
        self.subscription = self.create_subscription(
            Image,
            '/pan_tilt_images/image_raw',
            self.listener_callback,
            10
        )
        
        self.lock = threading.Lock()
        self.app = FastAPI()
        self.setup_routes()
        
    def listener_callback(self, msg):
        try:
            self.get_logger().info(f'Received an Image')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if ret:
                self.frame_buffer = jpeg.tobytes()
                
                if self.frame_buffer is not None:
                    nparr = np.frombuffer(self.frame_buffer, dtype=np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    draw_center_frame(frame, (640,480))
            
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                    self.frame_buffer = jpeg.tobytes()
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
            
    def setup_routes(self):
        @self.app.get('/video_feed')
        def video_feed():
            return StreamingResponse(self.generate_frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    def generate_frames(self):
        while True:
            with self.lock:
                if self.frame_buffer is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + self.frame_buffer + b'\r\n')
            sleep(0.033)  # ~30 FPS


def main():
    rclpy.init()
    video_streamer = CameraSubscriber()

    # Start FastAPI server in a separate thread
    api_thread = threading.Thread(target=uvicorn.run, args=(video_streamer.app,), kwargs={"host": "0.0.0.0", "port": 8000})
    api_thread.start()

    try:
        rclpy.spin(video_streamer)
    except KeyboardInterrupt:
        pass
    finally:
        video_streamer.destroy_node()
        rclpy.shutdown()
        api_thread.join()

if __name__ == '__main__':
    main()