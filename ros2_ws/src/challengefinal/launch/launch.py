from launch import LaunchDescription
from launch_ros.actions import Node
# Libreria para pdoer encontrar la carpeta share
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del archivo de parametros
    config = os.path.join(
        get_package_share_directory('challengefinal'),
        'config',
        'params.yaml'
    )

    # Nodo 1 (Generador)
    generator_node = Node(
            name = 'InputSignal',
            package='challengefinal', 
            executable='signal_generator', 
            emulate_tty=True,
            output='screen') 
    
    # RQT GRAPH
    rqt_graph = Node(name='rqt_graph',
                       package='rqt_graph',
                       executable='rqt_graph',
                       emulate_tty=True,
                       output='screen',
                       )

    # PlotterJuggler
    plotter_juggler = Node(name='PlotterJuggler',
                       package='plotjuggler',
                       executable='plotjuggler',
                       emulate_tty=True,
                       output='screen',
                       )
    
    l_d = LaunchDescription([generator_node, rqt_graph, plotter_juggler])

    return l_d