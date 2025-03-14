from launch import LaunchDescription
from launch_ros.actions import Node
# Libreria para pdoer encontrar la carpeta share
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del archivo de parametros
    config = os.path.join(
        get_package_share_directory('challenge02'),
        'config',
        'params.yaml'
    )

    # Nodo 1 (Generador)
    generator_node = Node(
            name = 'set_point_node',
            package='challenge02', 
            executable='set_point', 
            emulate_tty=True,
            output='screen') 

    # Nodo 2 (Procesador)
    contoller_node = Node( 
            name = 'controller_node',
            package='challenge02', 
            executable='controller', 
            emulate_tty=True,
            output='screen',
            parameters=[config]) 
    
    # Nodo 3 (Motor Plant)
    motor_node = Node( 
            name = 'dc_motor_node',
            package='challenge02', 
            executable='dc_motor', 
            emulate_tty=True,
            output='screen') 
    
    # RQT GRAPH
    rqt_graph = Node(name='rqt_graph',
                       package='rqt_graph',
                       executable='rqt_graph',
                       emulate_tty=True,
                       output='screen',
                       )
    
    l_d = LaunchDescription([generator_node, contoller_node, motor_node, rqt_graph])

    return l_d