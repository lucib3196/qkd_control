# arduino_bridge_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.serial_port = self.get_parameter('serial_port') \
                               .get_parameter_value().string_value
        
        try: #try to connect
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info(f"Connected to arduino in {self.serial_port}")
        except serial.SerialException:  #failed to connect
            self.get_logger().error("Couldnt open serial port ")
            exit(1)
       
        # state tracking are lasers on or off? 
        self.active = False
        self.treshold = 10      # ADJUST THIS VALUE FOR ERORR TRESHOLD

        self.create_subscription(
            Float32,
            'pan_tilt_error',
            self.error_callback,
            10
        )


    def send_cmd(self, cmd: str):
        """ helper to send messgaes"""
        line = (cmd + '\n').encode()
        self.ser.write(line)
        self.get_logger().debug(f" > {cmd}")

    def error_callback(self, msg: Float32):
        err = msg.data

       # self.get_logger().info(f"Getting message {err}")

            #TURN ON IF ERROR IS UNDER THRESHOLD
        if err <= self.treshold and not self.active:
            self.get_logger().info(f"Error {err:.3f} <= {self.treshold}: LASERS ON")
            self.send_cmd("redon") #turn on red laser
            self.send_cmd("greenon") #turnh on  green laser
            self.active = True
            
        elif err > self.treshold and self.active:
            self.get_logger().info(f"Error {err:.3f} > {self.treshold}: LASERS OFF")
            self.send_cmd("redoff") #turn off red laser
            self.send_cmd("greenoff") #turn off  green laser
            self.active = False

                # else do nothing
        
    def destroy(self):
        try: 
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try: 
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()