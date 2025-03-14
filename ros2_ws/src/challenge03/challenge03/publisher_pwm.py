""""
Ingesitos Co. 2025

COntrolador de pwm en ROS

"""

# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#from rcl_interfaces.msg import SetParametersResult

#Class Definition
class PWM_Publisher(Node):
    def __init__(self):
        super().__init__('publisher_pwm')

        #Create a publisher and timer for the signal
        self.pwm_publisher = self.create_publisher(Float32, 'cmd_pwm', 10)    ## CHECK FOR THE NAME OF THE TOPIC
        self.timer = self.create_timer(1, self.timer_cb)
        
        #Create a messages and variables to be used
        self.pwm_msg = Float32()
        #self.start_time = self.get_clock().now()

        self.get_logger().info("PWM publisher from ROS started succesfully... \U0001F680")
    

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        pwm_value = Float32()
        pwm_value.data = 0.0
        
        self.pwm_publisher.publish(pwm_value)
        #self.get_logger().info(f"Data published: {self.signal_msg}")

# ---- MAIN ---- #
def main(args=None):
    rclpy.init(args=args)

    pwm_node = PWM_Publisher()

    try:
        rclpy.spin(pwm_node)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_node.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()