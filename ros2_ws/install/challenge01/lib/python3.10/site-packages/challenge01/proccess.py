"""
Jose Alberto Aguilar Sanchez - A01735612
17 de Febrero de 2025

Basic signal processor (Subscriber)

"""

# Imports of ROS
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32 
# Imports
import numpy as np

# ---- ROS COMUNICATIONS ---- #
class Processing_signal_Node(Node): 
    def __init__(self): 
        super().__init__('processs') 
        print("The Signal Porcessor Node has succesfully initialized...")

        # Subscribtores a signal y time
        self.signal_sub = self.create_subscription(Float32, '/signal', self.signal_callback,10) 
        self.time_sub = self.create_subscription(Float32, '/time', self.time_callback,10) 
        #No unusedvariable warning 
        #self.signal_sub 

        # Publicador de la señal procesaada
        self.final_signal_pub = self.create_publisher(Float32, '/proc_signal', 10)

        # Timer de operacion sincrona
        self.operation_time = self.create_timer(0.1,self.publish_result)

        # Initial data
        self.time = 0.0
        self.signal_data = 0.0
        #self.i = 0.0

    # Guardamos los datos obtenidos
    def signal_callback(self, msg):
        self.signal_data = msg.data
        print(f'Received data: {self.signal_data}')
    
    def time_callback(self, msg):
        self.time = msg.data

    # Publicamos en el topico la signal procesada
    def publish_result(self): 
        # Recuperamos la processed signal
        proc_signal = signal_process(self.signal_data)
        self.get_logger().info(f'Processed signal data: {proc_signal}')

        # Enviamos los datos procesados
        proc_msg = Float32()
        proc_msg.data = proc_signal
        self.final_signal_pub.publish(proc_msg)

# ---- FUNCTIONS ---- #
def signal_process(data):
    # Filter design
    amp = 0.5
    offset = 1.0
    dephase = np.pi
    
    # User inputs
    # cos = raiz(1-sin²x)
    # (data * np.cos(dephase)) + (raiz(1-data²)*sen(dephase))
    proc = (data * np.cos(dephase)) + np.sqrt(1 - np.square(data))  * np.sin(dephase)
    new_signal = amp * proc + offset
    return new_signal

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