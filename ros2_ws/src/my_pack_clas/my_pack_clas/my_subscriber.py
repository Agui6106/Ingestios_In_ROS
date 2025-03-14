import rclpy 
from rclpy.node import Node 
from std_msgs.msg import Float32 

# Pueden ser los publicadores y subscriptores que quieras
class MinimalSubscriber(Node): 
    def __init__(self): 
        super().__init__('my_subscriber') 

        # Creamos un subscritor
        #Cuando rcibaun mensaje tipo FLoat32 en signal llama a lsitner callback pasandole el mensaje coo paramaetro con QA de 10
        self.signal_sub = self.create_subscription(Float32, 'signal', self.listener_callback,10) 
        self.signal_sub #No unusedvariable warning 

        # Creamos un publicador que devuelve la señal procesaada
        self.process_pub = self.create_publisher(Float32, 'processed_signal', 10)

        # Iteador
        self.i = 0.0

    # Cundo reciba el mensaje pasaemlopor herencia a esta funcion :0
    def listener_callback(self, msg): 
        print(msg)
        #self.get_logger().info('I heard "%s"' % msg.data) 

        # Procesamos la señalrecibida
        processed_signal =Float32()
        processed_signal.data = msg.data + self.i
        # Regresmaos la informacion
        self.process_pub.publish(processed_signal)
        
        # Inrementamos el iterador 1 unidades
        self.i += 1.0

#Main 
def main(args=None): 
    rclpy.init(args=args) 
    minimal_subscriber  = MinimalSubscriber() 

    try: 
        rclpy.spin(minimal_subscriber) 

    except KeyboardInterrupt: 
        pass 

    finally: 
        if rclpy.ok():  # Ensure shutdown is only called once 
            rclpy.shutdown() 
        minimal_subscriber.destroy_node() 

#Execute Node 
if __name__ == '__main__': 
    main() 