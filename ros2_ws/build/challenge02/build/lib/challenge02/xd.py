import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        
        # Declarar parámetros
        self.declare_parameter('signal_type', 'step')  # "step" o "square"
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('period', 2.0)  # Solo para la señal cuadrada
        
        # Obtener parámetros
        self.signal_type = self.get_parameter('signal_type').value
        self.amplitude = self.get_parameter('amplitude').value
        self.period = self.get_parameter('period').value
        
        # Publicador de la señal
        self.signal_pub = self.create_publisher(Float32, 'reference_signal', 10)
        
        # Configurar el timer con un período de muestreo de 20ms
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.start_time = time.time()

        self.get_logger().info('Signal Generator Node Started 🚀')

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time
        msg = Float32()
        
        if self.signal_type == 'step':
            msg.data = self.amplitude  # La señal escalón se mantiene constante
        elif self.signal_type == 'square':
            msg.data = self.amplitude if (elapsed_time % self.period) < (self.period / 2) else 0.0
        else:
            msg.data = 0.0  # Tipo de señal no reconocido

        self.signal_pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
