import rclpy
from rclpy.node import Node
from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel
from std_msgs.msg import String
import requests
import threading
from custom_interfaces.msg import PanTiltMsg, ArucoMsg
from typing import Literal
from typing import Optional
from fastapi.middleware.cors import CORSMiddleware


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],  # Replace "*" with your frontend URL for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class Message(BaseModel):
    data: str
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

@app.post("/endpoint")
async def receive_data(data: Message):
    # print(data.data)
    return {"message": "Data received"}

# In memory 
latest_arucodata: Optional[PydanticArucoMsg]=None
@app.post("/arucodata")
async def aruco_data(data:PydanticArucoMsg):
    global latest_arucodata
    latest_arucodata = data
    print("Received ArUco data:", data.model_dump())  # Add this
    return {'message': "Data Received"}

@app.get("/pantiltangle", response_model=Optional[PydanticArucoMsg])
async def get_latest_arucodata():
    return latest_arucodata

# In memory 
latest_pantiltangle: Optional[PydanticServoAngles]=None
@app.post("/pantiltangle")
async def aruco_data(data:PydanticServoAngles):
    global latest_pantiltangle
    latest_pantiltangle = data
    print("Received PanTiltAngles data:", data.model_dump())  # Add this
    return {'message': "Data Received"}

@app.get("/pantiltangle", response_model=Optional[PydanticServoAngles])
async def get_latest_arucodata():
    return latest_pantiltangle





class FastAPINode(Node):
    def __init__(self):
        super().__init__("fastapi_node")
        
        self.publisher_ = self.create_publisher(String, 'web_service_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscription to data we want
        self.aruco_sub = self.create_subscription(msg_type=ArucoMsg, topic="/aruco_data", callback=self.aruco_callback,qos_profile=1)
        self.pantilt_sub = self.create_subscription(msg_type=PanTiltMsg, topic="/pan_tilt_angles", callback=self.pantilt_callback,qos_profile=1)
        
    def aruco_callback(self,msg: ArucoMsg):
        data = PydanticArucoMsg.from_ros_msg(msg)
        self.get_logger().info('Published message: "%s"' % data)
        self.publish_arucodata(data)
        
    def publish_arucodata(self,data:PydanticArucoMsg):
        url = "http://localhost:8000/arucodata"
        payload = data.model_dump()
        response = requests.post(url, json=payload)
        if response.status_code != 200:
            self.get_logger().error('Failed to publish to FastAPI endpoint: %s')

    def pantilt_callback(self,msg: PanTiltMsg):
        data = PydanticServoAngles.from_ros_msg(msg)
        self.get_logger().info('Published message: "%s"' % data)
        self.publish_pantilt(data)
        
    def publish_pantilt(self,data:PydanticServoAngles):
        url = "http://localhost:8000/pantiltangle"
        payload = data.model_dump()
        response = requests.post(url, json=payload)
        if response.status_code != 200:
            self.get_logger().error('Failed to publish to FastAPI endpoint: %s')


    def timer_callback(self):
        message = "Hello World!"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Published message: "%s"' % msg.data)
        self.publish_to_fastapi(msg.data)
        
    def publish_to_fastapi(self, data):
        url = "http://localhost:8000/endpoint"
        payload = {"data": data}
        response = requests.post(url, json=payload)
        if response.status_code != 200:
            self.get_logger().error('Failed to publish to FastAPI endpoint: %s', response.text)
        
    
def main(args=None):
    rclpy.init(args=args)
    web_service_node = FastAPINode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(web_service_node)
    #rclpy.spin(web_service_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    uvicorn.run(app, host="localhost", port=8000, log_level="info")
    
    web_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
