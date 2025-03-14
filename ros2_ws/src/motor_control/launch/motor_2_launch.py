from launch import LaunchDescription
from launch_ros.actions import Node
# Libreria para pdoer encontrar la carpeta share
from ament_index_python.packages import get_package_share_directory
import os

"""
Launch file con namesapces (Grupos)- Se da un namespace individual a cada nodo.
Ya configurado para el Yamcha
"""
def generate_launch_description():
    # Obtener la ruta del archivo de parametros
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
    )

    # Crear los nodos
    motor_node_1 = Node(name="motor_sys_1",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True, # Habilitar los nodos en una terminal
                       output='screen',
                       namespace="group1",
                       parameters=[config] # Lecutra de la lista de parametros
                       )
    
    sp_node_1 = Node(name="sp_gen_1",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace="group1"
                       )
    
    motor_node_2 = Node(name="motor_sys_2",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace="group2"
                       )
    
    sp_node_2 = Node(name="sp_gen_2",
                       package='motor_control',
                       executable='set_point',
                       emulate_tty=True,
                       output='screen',
                       namespace="group2"
                       )
    
    l_d = LaunchDescription([motor_node_1, sp_node_1, motor_node_2, sp_node_2])

    return l_d