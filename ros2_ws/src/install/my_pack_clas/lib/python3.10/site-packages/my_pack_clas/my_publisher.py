# Imports 
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32 

#Class Definition 
class MyNode(Node): 
    def __init__(self): 
        super().__init__('my_publisher') 
        # Create a ROS publisher to the topic signl (Como ya heredamos de Node lollamamos comoself)
        self.signal_pub = self.create_publisher(Float32, 'signal', 10) 
        # Create a ros timer with period of 1.0 seconds
        self.timer1 = self.create_timer(1.0,self.timer1_cv)
        print("The node has succesfully initialized :D")

    #Timer Callback 
    def timer1_cv(self): 
        print("Im in a timer callback!!")
        # Tipo demnesaje de ros(Sobre)
        signal_msg = Float32()
        # Informacionamandar por el sobre (DEBE DE SER DELTIPO QUE SE DECLARO ARRIBA!!!)
        signal_msg.data = 77.0
        # Mandar la informacion
        self.signal_pub.publish(signal_msg)
 
#Main 
def main(args=None): 
    rclpy.init(args=args) 
    # Inicializo una estancia de la clase MyNode
    my_node  = MyNode() 
    # While(1) de ROS. Until keyboard interrumpt
    rclpy.spin(my_node)

    # Terminamos los procesos y limpiamos como dios manda
    rclpy.shutdown()
    my_node.destroy_node()

#Execute Node (MOST BASIC)
if __name__ == '__main__': 
    main() 