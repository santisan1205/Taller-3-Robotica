"""
pc_launch.py  —  Corre en tu PC / Laptop
──────────────────────────────────────────
Lanza el nodo de visión que corre en la PC.
La PC debe estar en la misma red Wi-Fi que la Raspberry Pi y tener
configurado el mismo ROS_DOMAIN_ID.

Uso:
  # En la PC, antes de lanzar:
  export ROS_DOMAIN_ID=42          # mismo valor que en la RPi
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

  ros2 launch taller3_grupo3 pc_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Mostrar ventana de debug de OpenCV'
    )

    return LaunchDescription([
        show_window_arg,

        # ── Nodo de Visión + OpenCV ────────────────────────────────────────
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
