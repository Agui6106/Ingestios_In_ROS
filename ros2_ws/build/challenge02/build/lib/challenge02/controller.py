"""
Jose Alberto Aguilar Sanchez - A01735612

Controaldor PID

"""

# Imports of ROS
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32 
from rcl_interfaces.msg import SetParametersResult
# Servicio
from custom_interfaces.srv import SetProcessBool

# ---- ROS NODE ---- #
class Processing_signal_Node(Node): 
    def __init__(self): 
        super().__init__('processs') 
        self.get_logger().info("The PID Controller Node has succesfully initialized...")

        # -- PARAMETERS -- #
        # DECLARE
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('T', 0.01) # Periodo de muestreo
        # kp = 2.588
        # ki = 5.125
        # kd = 0.0128

        self.T = self.get_parameter('T').value

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # -- COMMS -- #
        # Subscribtores
        self.signal_sub = self.create_subscription(Float32, 'set_point', self.set_point_Callback,10) 
        self.motor_retro_sub = self.create_subscription(Float32, 'motor_output_y', self.motor_retro_Callback,10) 
        # Publicador
        self.motor_out_pub = self.create_publisher(Float32, 'motor_input_u', 10)
        # Timer de operacion Asincorona (Bucle)
        self.operation_time = self.create_timer(self.T,self.publish_mtr_result)
        # Server
        self.system_running = False

        self.control_service = self.create_service(SetProcessBool, 'EnableController', self.service_callback)
        #while not self.client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')

        # Enviamos request (solicitud) para ejeuctar al servidor
        #request_msg = SetProcessBool.Request()
        #request_msg.enable = True
        #self.send_request(request_msg)
        
        # -- INITIAL DATA -- #
        self.motor_retro = 0.0  # from dc_motor.py
        self.motor_ref = 0.0    # From set_point.py
        self.errors = [0,0,0]   # Array of erros
        self.outputs = [0,0]    # Array of outputs

    # - DATA SAVE - #
    def set_point_Callback(self, msg):
        # Read Ref
        self.motor_ref = msg.data
    
    def motor_retro_Callback(self, msg):
        # Read Retro
        self.motor_retro = msg.data

    # - PARAMETERS -#
    def parameter_callback(self, params):
        for param in params:
            # - Check user inputs - #
            # Kp
            if param.name == "kp":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid Kp_gain! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="System PID Gain Kp cannot be negative")
                else:
                    self.param_Kp = param.value  # Update internal variable
                    self.get_logger().info(f"Kp_gain updated to {self.param_Kp}")

            # Ki
            if param.name == "ki":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid Ki_gain! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="System PID Gain Ki cannot be negative")
                else:
                    self.param_Ki = param.value  # Update internal variable
                    self.get_logger().info(f"Ki_gain updated to {self.param_Ki}")

            # Kd
            if param.name == "kd":
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid Kd_gain! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="System PID Gain Kd cannot be negative")
                else:
                    self.param_Kd = param.value  # Update internal variable
                    self.get_logger().info(f"Kd_gain updated to {self.param_Kd}")

        return SetParametersResult

    # - SERVICE - #
    def service_callback(self, request, response):
        # Activacion (Request)
        if request.enable:
            self.system_running = True
            self.get_logger().info('PID Controller Started!!')
            response.success = True
            response.message = 'PID Controller Started'
        # Desactivacion
        else:
            # (Response)
            self.system_running = False
            self.get_logger().info('PID Controller Stopped!!')
            response.success = True
            response.message = 'PID Controller Stopped'
        
        return response

    # - PUBLIC REUSLTS - #
    def publish_mtr_result(self): 
        # Verifica que se haya encendido la simulacion
        if self.system_running == False:
            return
        
        # Enviamos los datos procesados
        proc_msg = Float32()
        proc_msg.data = self.controller_PID(self.motor_ref, self.motor_retro)
        self.motor_out_pub.publish(proc_msg)
        #self.get_logger().info(f'PID Controller data: {proc_msg.data}')

    # ---- FUNCTIONS ---- #
    def controller_PID(self, ref, retro):
        # CALL
        self.kp_gain = self.get_parameter('kp').value
        self.ki_gain = self.get_parameter('ki').value
        self.kd_gain = self.get_parameter('kd').value
        
        self.errors[0] = ref - retro
        # Controlador PID en su forma Z

        # Coeficientes discretos
        self.K1 = self.kp_gain + (self.T * self.ki_gain) + (self.kd_gain / self.T)
        self.K2 = -self.kp_gain - 2.0 * (self.kd_gain / self.T)
        self.K3 = self.kd_gain / self.T

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