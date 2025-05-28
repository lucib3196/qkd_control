import asyncio
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import cv2
import requests
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool
import uvicorn
from uvicorn import Config, Server

import rclpy  # type: ignore
from rclpy.logging import LoggingSeverity  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore

from sensor_msgs.msg import CompressedImage  # type: ignore
from cv_bridge import CvBridge  # type: ignore
from custom_interfaces.msg import PanTiltMsg, ArucoMsg


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


## Pydantic Classes
# Aruco Pydantic Class
class Point(BaseModel):
    x: float
    y: float
    z: float
class Quaternion(BaseModel):
    x: float
    y: float
    z: float
    w: float
class Pose(BaseModel):
    position: Point
    orientation: Quaternion
class Time(BaseModel):
    sec: int
    nanosec: int
class PydanticArucoMsg(BaseModel):
    pose: Pose
    detection_time: Time
    size: float
    dictionary_type: str

    @classmethod
    def from_ros_msg(cls, msg):
        return cls(
            pose=Pose(
                position=Point(
                    x=msg.pose.position.x,
                    y=msg.pose.position.y,
                    z=msg.pose.position.z
                ),
                orientation=Quaternion(
                    x=msg.pose.orientation.x,
                    y=msg.pose.orientation.y,
                    z=msg.pose.orientation.z,
                    w=msg.pose.orientation.w
                )
            ),
            detection_time=Time(
                sec=msg.detection_time.sec,
                nanosec=msg.detection_time.nanosec
            ),
            size=msg.size,
            dictionary_type=msg.dictionary_type
        )
        
class PydanticServoAngles(BaseModel):
    pan_angle:float
    tilt_angle:float

    @classmethod
    def from_ros_msg(cls,msg):
        return cls(
            pan_angle=msg.pan_angle,
            tilt_angle=msg.tilt_angle
        )

class Backend(Node):
    def __init__(self):
        super().__init__("backend")
        self.get_logger().set_level(LoggingSeverity.WARN)

        self.frame_handler = VideoFrameHandler(self.get_logger())

        # Video feed sub
        self.subscription = self.create_subscription(
            CompressedImage,
            "/pan_tilt_images/processed",
            self.frame_handler.on_image_msg,
            qos_profile=qos_profile_sensor_data
        )
        # Subscription to data we want
        self.aruco_sub = self.create_subscription(msg_type=ArucoMsg, topic="/aruco_detection", callback=self.aruco_cb,qos_profile=1)
        self.pantilt_sub = self.create_subscription(msg_type=PanTiltMsg, topic="/pan_tilt_angles", callback=self.pantilt_cb,qos_profile=1)

        # Most recent data
        self.latest_aruco: PydanticArucoMsg|None=None
        self.latest_pan_tilt: PydanticArucoMsg|None = None

        self.app = FastAPI()
        self.setup_routes()
    
    def aruco_cb(self,msg:ArucoMsg):
        self.latest_aruco = PydanticArucoMsg.from_ros_msg(msg)
    def pantilt_cb(self,msg:PanTiltMsg):
        self.latest_pan_tilt = PydanticServoAngles.from_ros_msg(msg)

    def setup_routes(self):
        self.app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173"],  # Replace "*" with your frontend URL for production
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )   
        @self.app.get('/', response_class=HTMLResponse)
        def index():
            return HTMLResponse(content="""
            <html><head><title>Face Tracking Stream</title></head>
            <body><h1>Tracking</h1>
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
        @self.app.post("/aruco_detection",response_model=Optional[PydanticArucoMsg])
        async def get_aruco():
            if self.latest_aruco:
                # self.get_logger().warn(f"{self.latest_aruco}")
                return self.latest_aruco.model_dump()
            else:
                return None
        
        @self.app.post("/pan_tilt_angles",response_model=Optional[PydanticServoAngles])
        async def get_pantilt():
            if self.latest_pan_tilt:
                return self.latest_pan_tilt
            else:
                return None
    
    
    # def publish_data(self,payload,url):
    #     response = requests.post(url, json=payload)
    #     if response.status_code != 200:
    #         self.get_logger().error('Failed to publish to FastAPI endpoint: %s')
        
    def start_api(self,host: str = "0.0.0.0", port: int = 8000):
        config = Config(
        app=self.app,
        host=host,
        port=port,
        log_level="warning",
        reload=False,
    )
        server = Server(config)
        api_thread = threading.Thread(target=server.run, daemon=True)
        api_thread.start()
        return api_thread
    def destroy_node(self):
        self.frame_handler.stop()
        super().destroy_node()
        
    

    
def main(args=None):
    rclpy.init(args=args)
    web_service_node = Backend()
    api_thread = web_service_node.start_api()
    try:
        rclpy.spin(web_service_node)
    except KeyboardInterrupt:
        web_service_node.get_logger("Shutting Down")
    finally:
        web_service_node.destroy_node()
        rclpy.shutdown()
        api_thread.join()


if __name__ == '__main__':
    main()