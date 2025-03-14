"""
Jose Alberto Aguilar Sanchez - A01735612

Controaldor PID

"""

# Imports of ROS
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32 
# Imports
#import numpy as np

# ---- ROS NODE ---- #
class Processing_signal_Node(Node): 
    def __init__(self): 
        super().__init__('processs') 
        self.get_logger().info("The PID Controller Node has succesfully initialized...")

        # -- PARAMETERS -- #
        # DECLARE
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 18.0)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('T', 0.01) # Periodo de muestreo

        # CALL
        self.kp_gain = self.get_parameter('kp').value
        self.ki_gain = self.get_parameter('ki').value
        self.kd_gain = self.get_parameter('kd').value
        self.T = self.get_parameter('T').value

        # Coeficientes discretos
        self.K1 = self.kp_gain + (self.T * self.ki_gain) + (self.kd_gain / self.T)
        self.K2 = -self.kp_gain - 2.0 * (self.kd_gain / self.T)
        self.K3 = self.kd_gain / self.T

        # -- COMMS -- #
        # Subscribtores
        self.signal_sub = self.create_subscription(Float32, '/set_point', self.set_point_Callback,10) 
        self.motor_retro_sub = self.create_subscription(Float32, '/motor_output_y', self.motor_retro_Callback,10) 
        # Publicador
        self.motor_out_pub = self.create_publisher(Float32, '/motor_input_u', 10)
        # Timer de operacion Asincorona (Bucle)
        self.operation_time = self.create_timer(self.T,self.publish_mtr_result)
        
        # -- INITIAL DATA -- #
        self.motor_retro = 0.0  # from dc_motor.py
        self.motor_ref = 0.0    # From set_point.py
        self.errors = [0,0,0]   # Array of erros
        self.outputs = [0,0]    # Array of outputs

    # Data save
    def set_point_Callback(self, msg):
        # Read Ref
        self.motor_ref = msg.data
        print(f'Received data: {self.motor_ref}')
    
    def motor_retro_Callback(self, msg):
        # Read Retro
        self.motor_retro = msg.data

    # Public process result
    def publish_mtr_result(self): 
        # Enviamos los datos procesados
        proc_msg = Float32()
        proc_msg.data = self.controller_PID(self.motor_ref, self.motor_retro)
        self.motor_out_pub.publish(proc_msg)
        self.get_logger().info(f'PID Controller data: {proc_msg.data}')

    # ---- FUNCTIONS ---- #
    def controller_PID(self, ref, retro):
        self.errors[0] = ref - retro
        # Controlador PID en su forma Z
        u = self.K1 * self.errors[0] + self.K2 * self.errors[1] + self.K3 * self.errors[2] + self.outputs[1]

        # Update data
        self.errors[2] = self.errors[1]
        self.errors[1] = self.errors[0]
        self.outputs[1] = u

        return u
    
# ---- MAIN ---- # 
def main(args=None): 
    rclpy.init(args=args) 
    minimal_subscriber  = Processing_signal_Node() 

    try: 
        rclpy.spin(minimal_subscriber) 

    except KeyboardInterrupt: 
        print("Signal processor stopped due to user interrumpt") 

    finally: 
        if rclpy.ok():  # Ensure shutdown is only called once 
            rclpy.shutdown() 
        minimal_subscriber.destroy_node() 

#Execute Node 
if __name__ == '__main__': 
    main() 