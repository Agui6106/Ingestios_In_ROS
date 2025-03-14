"""
Archivo de inicializacion de varios nodos en ROS
"""

from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 

    # Nodo 1 (Generador)
    node1 = Node( 
            package='challenge01', 
            executable='signal_generetor', 
            output='screen') 

    # Nodo 2 (Procesador)
    node2 = Node( 
            package='challenge01', 
            executable='proccess', 
            output='screen') 
    
    # RQT
    rqt_node = Node(name = 'rqt_plot',
                    package='rqt_plot',
                    executable='rqt_plot',
                    arguments=['/signal/data', 
                               '/proc_signal/data'])
    
    # RQT Graph
    rqt_graph = Node( 
            package='rqt_graph', 
            executable='rqt_graph', 
            output='screen') 

    ld = LaunchDescription([node1, node2, rqt_node, rqt_graph]) 

    return ld 