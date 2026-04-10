"""
robot_launch.py  —  Launch file completo Taller 2 + Taller 3
──────────────────────────────────────────────────────────────
Contiene TODOS los nodos del paquete. Los del Taller 2 se conservan
intactos y los del Taller 3 se agregan.

Nodos del Taller 2 (conservados):
  • robot_core_node       — motores + odometría
  • robot_player_node     — reproducción de trayectorias
  • joy_node              — leer joystick/control
  • teleop_twist_joy_node — traducir botones a cmd_vel

Nodos nuevos Taller 3:
  • camera_node           — captura cámara RPi y publica JPEG
  • manipulator_core_node — controla servo del elevador via GPIO
  • robot_fsm_node        — máquina de estados autónoma

Nota: vision_node DEBE correr en la PC, no aquí.
Usa pc_launch.py en tu laptop para lanzarlo.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ════════════════════════════════════════
        #  NODOS TALLER 2 — sin cambios
        # ════════════════════════════════════════

        # Control de motores DC + publicación de odometría
        Node(
            package='taller3_grupo3',
            executable='robot_core',
            name='robot_core_node',
            output='screen'
        ),

        # Piloto automático — reproducción de trayectorias grabadas
        Node(
            package='taller3_grupo3',
            executable='robot_player',
            name='robot_player_node',
            output='screen'
        ),

        # Driver del joystick / control Bluetooth (Xbox, PS4, etc.)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),

        # Traductor de ejes del joystick a mensajes Twist en /cmd_vel
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'require_enable_button': False,
                'axis_linear.x':        1,      # Palanca Izq — Adelante/Atrás
                'axis_angular.yaw':     0,      # Palanca Izq — Giro
                'scale_linear.x':       0.5,    # Vel máx lineal (m/s)
                'scale_angular.yaw':    1.0     # Vel máx angular (rad/s)
            }]
        ),

        # ════════════════════════════════════════
        #  NODOS NUEVOS TALLER 3
        # ════════════════════════════════════════

        # Cámara RPi → publica /camera/image_compressed
        Node(
            package='taller3_grupo3',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'fps':          15,
                'width':        640,
                'height':       480,
                'jpeg_quality': 80,
                'camera_index': 0
            }]
        ),

        # Servo del mecanismo de elevación → suscrito a /lift_cmd
        Node(
            package='taller3_grupo3',
            executable='manipulator_core',
            name='manipulator_core_node',
            output='screen'
        ),

        # Máquina de estados autónoma — coordina visión + movimiento + elevador
        Node(
            package='taller3_grupo3',
            executable='robot_fsm',
            name='robot_fsm_node',
            output='screen'
        ),

    ])
