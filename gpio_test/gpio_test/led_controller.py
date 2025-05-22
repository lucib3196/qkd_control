import rclpy
from rclpy.node import Node
from gpiozero import LED
from time import sleep
from std_msgs.msg import Bool


class LEDController(Node):
    "Simple test to check gpio zero integration with ros2"
    
    def __init__(self):
        super().__init__("led_controller")
        
        self.get_logger().info("Initializing the LED Node")
        # Connecte to gpi pin 17
        self.led = LED(17)
        
        # Create a subscription for toggling the led on and off
        self.led_sub = self.create_subscription(
            Bool, 
            "/led_toggle",
            self.listener_callback,
            10
        )
        
    def listener_callback(self,msg):
        if msg.data:
            self.led.on()
            self.get_logger().info('LED turned ON')
        else:
            self.led.off()
            self.get_logger().info('LED turned OFF')
            
            
            
def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDController()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()