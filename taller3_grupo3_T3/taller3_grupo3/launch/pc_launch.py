"""
pc_launch.py  —  Corre en tu PC / Laptop (NO en la RPi)
─────────────────────────────────────────────────────────
Lanza únicamente el vision_node, que necesita procesar imágenes
con OpenCV y requiere la potencia de la PC.

Configuración previa (en la PC, misma red Wi-Fi que la RPi):
  export ROS_DOMAIN_ID=42          # mismo valor que en la RPi
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

Uso:
  ros2 launch taller3_grupo3 pc_launch.py
  ros2 launch taller3_grupo3 pc_launch.py show_window:=false  # sin ventana
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='Mostrar ventana de debug de OpenCV (true/false)'
        ),

        Node(
            package='taller3_grupo3',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'show_window': LaunchConfiguration('show_window')
            }]
        ),
    ])
