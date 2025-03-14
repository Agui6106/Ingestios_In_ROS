# Imports 
import rclpy 
from rclpy.node import Node     # Libreria Nodo de ROS
from std_msgs.msg import String # Libreria detipos de mensajes de ROS

#Class Definition heredada de un Nodo de ros
class MinimalPublisher(Node): 
    # Contrsuctor para inicializar el nodo
    def __init__(self): 
        # Definicion del nombre del nodo inicializador
        super().__init__('talker_node') 
        # Variable local miembro de la clase. Crea un publisher de ros
        #                    (tipo de mensaje, nombre del topico, QS)
        self.publisher = self.create_publisher(String, 'chatter', 10) 
        timer_period = 0.5 #seconds 
        # Timer de ROS - Cuando se termine dicho periodo ejecuta la funcion 
        #                              (timepo a esperar, funcion a ejecutar)
        self.timer = self.create_timer(timer_period, self.timer_cb) 

        self.i = 0 

    #Timer Callback 
    def timer_cb(self): 
        # Tipo de mensaje (String de ROS)
        msg = String() 
        # Variable a enviar del tipo d emensaje 
        msg.data = 'Hello World: %d' %self.i 
        # Publico mi mensjae de ROS
        self.publisher.publish(msg) 
        # Print de ROS para ver en la terminal cuando se ejecute esta funcion del nodo
        self.get_logger().info('Publising: "%s"' % msg.data) 

        self.i +=1 

#Main 
def main(args=None): 
    # Inicializar elnodo con argumentos
    rclpy.init(args=args) 
    # Se crea un objeto de la clase publicadora.
    minimal_publisher  = MinimalPublisher() 

    try: 
        # while(1) de ROS:
        rclpy.spin(minimal_publisher) 

    except KeyboardInterrupt:
        pass 

    finally: 
        if rclpy.ok():  # Ensure shutdown is only called once 
            rclpy.shutdown() 

        minimal_publisher.destroy_node() 

#Execute Node (MOST BASIC)
if __name__ == '__main__': 
    main() 