import sys 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
from serial import SerialException
from thorlabs_elliptec import ELLx,ELLError
from rclpy.logging import LoggingSeverity
from custom_interfaces.msg import PanTiltError

class HWPController(Node):
    def __init__(self):
        super().__init__("hwp_controller")
        self.get_logger().set_level(LoggingSeverity.WARN)

        self.declare_parameter("port","/dev/ttyUSB0")
        port =self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info(f"Openingin HWP on port {port}")

        try:
            self.state = ELLx(serial_port =port)
        except SerialException as e:
            self.get_logger().error(f"Could not open {port} {e}")
            sys.exit(1)

        try: 
            self.get_logger().info("Homing")
            self.state.home(blocking=True)
            self.get_logger().info("Homed to 0")
        except ELLError as e:
            self.get_logger().error(f"Elliptec error during home {e}")
            sys.exit(2)
            
        self.create_subscription(
            Float32, 
            "hwp/angle",
            self.angle_callback,
            10
        )
        
        
        
    def angle_callback(self,msg:Float32):
        angle = msg.data
        if not 0.0 <=angle <=360.0:
            self.get_logger().warn(f"Angle {angle} out of range [0,360]")

        self.get_logger().info(f"Moving to angle {angle}")

        try:
            self.state.move_absolute(angle,blocking=True)
        except ELLError as e:
            self.get_logger().error(f"Elliptec move error {e}")

    def destroy_node(self):
        try:
            self.state.close()
            self.get_logger().info("Stage Closed")
        except Exception:
            pass
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = HWPController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
