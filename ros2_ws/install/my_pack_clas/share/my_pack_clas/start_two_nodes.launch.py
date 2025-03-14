"""
Archivo de inicializacion de varios nodos en ROS
"""

from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 

    # Nodo 1
    node1 = Node( 
            package='my_pack_clas', 
            executable='my_publisher', 
            output='screen') 

    # Nodo 2
    node2 = Node( 
            package='my_pack_clas', 
            executable='my_subscriber', 
            output='screen') 

    ld = LaunchDescription([node1, node2]) 

    return ld 