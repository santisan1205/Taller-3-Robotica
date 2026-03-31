"""
pi_launch.py  —  Corre en la Raspberry Pi 5
────────────────────────────────────────────
Lanza los nodos que deben ejecutarse en la RPi:
  • robot_core_node      → motores + odometría
  • camera_node          → captura y publica imágenes comprimidas
  • manipulator_core_node → controla el elevador/servo
  • robot_fsm_node       → máquina de estados autónoma

Uso:
  ros2 launch taller3_grupo3 pi_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ── Argumentos configurables ──────────────────────────────────────────
    fps_arg     = DeclareLaunchArgument('fps',    default_value='15')
    width_arg   = DeclareLaunchArgument('width',  default_value='640')
    height_arg  = DeclareLaunchArgument('height', default_value='480')

    return LaunchDescription([
        fps_arg, width_arg, height_arg,

        # ── Nodo 1: Control de motores + Odometría ────────────────────────
        Node(
            package='taller3_grupo3',
            executable='robot_core',
            name='robot_core_node',
            output='screen'
        ),

        # ── Nodo 2: Cámara RPi ────────────────────────────────────────────
        Node(
            package='taller3_grupo3',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'fps':          LaunchConfiguration('fps'),
                'width':        LaunchConfiguration('width'),
                'height':       LaunchConfiguration('height'),
                'jpeg_quality': 80,
                'camera_index': 0
            }]
        ),

        # ── Nodo 3: Control del elevador ──────────────────────────────────
        Node(
            package='taller3_grupo3',
            executable='manipulator_core',
            name='manipulator_core_node',
            output='screen'
        ),

        # ── Nodo 4: Máquina de estados autónoma ───────────────────────────
        Node(
            package='taller3_grupo3',
            executable='robot_fsm',
            name='robot_fsm_node',
            output='screen'
        ),

        # ── Nodo 5: Teleoperación (útil para pruebas manuales) ────────────
        # Comenta este bloque cuando quieras solo modo autónomo.
        Node(
            package='taller3_grupo3',
            executable='robot_teleop',
            name='robot_teleop_node',
            output='screen',
            prefix='xterm -e'  # Abre en terminal separada en RPi con escritorio
        ),
    ])
