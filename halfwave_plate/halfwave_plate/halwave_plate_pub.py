import sys 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from rclpy.logging import LoggingSeverity

class HWPPub(Node):
    def __init__(self):
        super().__init__("hwp_pub")
        self.get_logger().set_level(LoggingSeverity.WARN)

        self.angle_pub = self.create_publisher(
            Float32,
            "/hwp/angle",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_angle
        )

    def publish_angle(self):
        angle_ranges = [0,45,90,135,180,270]
        for angle in angle_ranges:
            # Create a message and publish it
            msg = Float32()
            msg.data = float(angle)  # Example angle value
            self.angle_pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = HWPPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()