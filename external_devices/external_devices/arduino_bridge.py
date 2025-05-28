import rclpy
from rclpy.node import Node
import serial 

class ArduinoBridgeNode(Node):
    def __init__(self): 
        super().__init__('arduino_bridge_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0') #CHECK THE SERIAL PORT TO MATCH THE ACTUAL
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info(f"Connected to Arduno on {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port. Check port number.")
            exit(1)

        self.timer = self.create_timer(2.0, self.turn_on_laser)

    def turn_on_laser(self):
        self.get_logger().info("Turning on laser...")
        self.ser.write(b'1') #send a char '1' thru the serial to arduino so arduino serial reads high
        
        # turn off after 1 second 
        self.create_timer(1.0, self.turn_off_laser) 
    
    def turn_off_laser(self):
        self.get_logger().info("Turning off laser....")
        self.ser.write(b'0') #send 0 to turn it off


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main() 
        
        