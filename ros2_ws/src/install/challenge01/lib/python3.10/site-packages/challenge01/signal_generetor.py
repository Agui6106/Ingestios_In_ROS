"""
Jose Alberto Aguilar Sanchez - A01735612
17 de Febrero de 2025

Basic signal generetor (Publisher)

"""

# Imports of ROS
import rclpy 
from rclpy.node import Node     # Libreria Nodo de ROS
from std_msgs.msg import Float32 # Libreria detipos de mensajes de ROS
# Imports
import numpy as np

# ---- ROS COMUNICATIONS ---- #
class Signal_Gen_Node(Node): 
    def __init__(self): 
        super().__init__('signal_generetor') 
        print("The Signal Generetor Node has succesfully initialized...")
        
        # Creaion de timer de ROS
        self.timer1 = self.create_timer(0.1,self.timer_cb)
        self.iterador = 0 # Tiempo inicial a publciar

        # Creacion de topicos a publciar
        self.signal_pub = self.create_publisher(Float32, '/signal', 10) 
        self.timer_pub = self.create_publisher(Float32, '/time', 10) 

        # Tiempo incial
        self.iterador = 0.0
        
    #Timer Callback 
    def timer_cb(self): 
        # - Creacion de signal - #
        signal_msg = Float32()
        signal_2_send = signal(self.iterador)
        # Envio
        signal_msg.data = float(signal_2_send)
        self.signal_pub.publish(signal_msg)

        # - Timer - #
        times = Float32()
        times.data = float(self.iterador)
        self.timer_pub.publish(times)

        # - LOGGER - #
        self.get_logger().info(f'Generated signal data: {signal_msg.data}')

        self.iterador += 0.1
 
# ---- FUNCTIONS ---- #
def signal(x):
    return np.sin(x)
 
# ---- MAIN ---- #
def main(args=None): 
    rclpy.init(args=args) 
    # Inicializo una estancia de la clase MyNode
    my_node  = Signal_Gen_Node() 

    try: 
        # while(1) de ROS:
        rclpy.spin(my_node) 

    except KeyboardInterrupt:
        print("Stopping signal generation due to user interrupt")

    finally: 
        if rclpy.ok():  # Ensure shutdown is only called once 
            rclpy.shutdown() 

        my_node.destroy_node() 

#Execute Node (MOST BASIC)
if __name__ == '__main__': 
    main() 