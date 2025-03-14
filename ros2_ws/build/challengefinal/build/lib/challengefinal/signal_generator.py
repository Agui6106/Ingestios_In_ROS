"""
Jose Albeto Aguilar Sanchez - A01735612

Multiple signal types for the controller
(Signal Generator SP_GEN- Publciator)
"""

# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('InputSignal')
        # PArameter for signal type
        self.declare_parameter('signal_type', 'sqre')
        self.declare_parameter('amplitude', 6.0)
        self.declare_parameter('omega', 0.2)
        self.declare_parameter('timer_period', 0.1)
        # Retrieve  wave parameters
        self.signal_type = self.get_parameter('signal_type').value
        self.amplitude = self.get_parameter('amplitude').value
        self.omega  = self.get_parameter('omega').value
        self.timer_period = self.get_parameter('timer_period').value #seconds

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)    ## CHECK FOR THE NAME OF THE TOPIC
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        #Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info("SetPoint Sin Wave Node Started successfully \U0001F680")

    # - PARAMETERS -#
    def parameter_callback(self, params):
        for param in params:
            # Check Signal type
            if param.name == "signal_type":
                if (param.value != 'sin') and (param.value != 'step') and (param.value != 'sqre'):
                    self.get_logger().warn("Invalid signal_type! It must be 'sin', 'step' or 'sqre'.")
                    return SetParametersResult(successful=False, reason="Invalid signal_type")
                else:   
                    self.signal_type = param.value
                    self.get_logger().info(f"Signal Type ssuccessfully changed to: {self.signal_type}")
            
            # Check Amplitude
            if param.name == "amplitude":
                if param.value < -15.0 or param.value > 15.0:
                    self.get_logger().warn("Invalid amplitude! It must be between -15.0 and 15.0!!!")
                    return SetParametersResult(successful=False, reason="Invalid amplitude")
                else:
                    self.amplitude = param.value
                    self.get_logger().info(f"Amplitude successfully changed to: {self.amplitude}")

            # Check Omega
            if param.name == "omega":
                if param.value <= 0:
                    self.get_logger().warn("Invalid omega! It must be a positive value!!!")
                    return SetParametersResult(successful=False, reason="Invalid omega")
                else:
                    self.omega = param.value
                    self.get_logger().info(f"Omega successfully changed to: {self.omega}")
            
            # Check Timer Period
            if param.name == "timer_period":
                if param.value <= 0:
                    self.get_logger().warn("Invalid timer_period! It must be a positive value!!!")
                    return SetParametersResult(successful=False, reason="Invalid timer_period")
                else:
                    self.timer_period = param.value
                    self.timer.timer_period = self.timer_period
                    self.get_logger().info(f"Timer Period successfully changed to: {self.timer_period}")
                    
        return SetParametersResult
    

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        #Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        
        # Check signal type
        if self.signal_type == 'sin':    
            # Generate SINE WAVE signal
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)

        elif self.signal_type == 'step': 
            # Generate  STEP WAVE signal
            self.signal_msg.data = self.amplitude

        elif self.signal_type == 'sqre':
            # Generate SQUARE WAVE signal
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        
        self.signal_publisher.publish(self.signal_msg)
        #self.get_logger().info(f"Data published: {self.signal_msg}")

# ---- MAIN ---- #
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()