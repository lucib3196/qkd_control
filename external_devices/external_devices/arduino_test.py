import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        # # Subscriptions
        # self.cmd_sub = self.create_subscriptions(
        #     Float32,"/arduino_cmd", self.send_command,10
        # )

        # Serial connection
        self.ser = serial.Serial("/dev/ttyACM0", 9600)
        time.sleep(2)
        

        self.timer = self.create_timer(0.5, self.send_command)
        self.timer2 = self.create_timer(0.5, self.read_serial)


    def send_command(self):
        value=3.14
        messages = f"{value}\n"
        self.get_logger().info(f"Serial line: {messages}")
        self.ser.write(messages.encode())
    
    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode()
            self.get_logger().info(f"Serial line: {line}")
            if line:
                try:
                    msg = Float32()
                    msg.data = float(line)
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().warn(f"Invalid int received: '{line}'")
        else:
            self.get_logger().info(f"No messages from Arduino")



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


    


