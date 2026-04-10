"""
robot_fsm.py  —  NUEVO Taller 3 — Máquina de Estados Autónoma
──────────────────────────────────────────────────────────────
Coordina la secuencia autónoma completa de clasificación:

  BUSCANDO → CENTRANDO → ACERCANDO → RECOGIENDO → DEPOSITANDO → LISTO

Tópicos suscritos:
  /detected_object  [std_msgs/String]   — JSON del vision_node
  /odom             [nav_msgs/Odometry] — posición del robot

Tópicos publicados:
  /cmd_vel   [geometry_msgs/Twist]  — mueve las ruedas
  /lift_cmd  [std_msgs/String]      — controla el elevador

┌──────────────────────────────────────────────────────────────┐
│  DIAGRAMA DE ESTADOS                                          │
│                                                               │
│  BUSCANDO──(objeto visto)──►CENTRANDO──(centrado)──►ACERCANDO│
│      ▲                                                   │    │
│      │                                               (cerca)  │
│      │                                                   ▼    │
│  LISTO◄──(vuelve)──DEPOSITANDO◄──RECOGIENDO               │  │
└──────────────────────────────────────────────────────────────┘
"""

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from enum import Enum, auto


class Estado(Enum):
    BUSCANDO    = auto()
    CENTRANDO   = auto()
    ACERCANDO   = auto()
    RECOGIENDO  = auto()
    DEPOSITANDO = auto()
    LISTO       = auto()


# Parámetros de control — ajusta según tu robot y el área de prueba
ANCHO_FRAME    = 640      # px, debe coincidir con camera_node
TOLERANCIA_PX  = 40       # px de margen para considerar objeto centrado
AREA_CERCA     = 18000    # px², objeto lo suficientemente cerca para recoger
VEL_GIRO       = 0.35     # rad/s al buscar
VEL_GIRO_FINO  = 0.15     # rad/s al centrar
VEL_AVANCE     = 0.15     # m/s al acercarse
T_DEPOSITO     = 3.0      # s avanzando hacia zona de depósito
T_ESPERA_LIFT  = 1.5      # s esperando que el elevador suba/baje


class RobotFSM(Node):
    def __init__(self):
        super().__init__('robot_fsm_node')

        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',  10)
        self.lift_pub = self.create_publisher(String, '/lift_cmd', 10)

        self.sub_vision = self.create_subscription(
            String,   '/detected_object', self.vision_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom',            self.odom_callback,   10
        )

        self.estado       = Estado.BUSCANDO
        self.deteccion    = {'color': 'none', 'shape': 'unknown',
                             'cx': 0, 'cy': 0, 'area': 0.0}
        self.timer_inicio = None
        self.x = self.y = self.theta = 0.0

        # FSM corre a 10 Hz
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info("RobotFSM iniciado — Estado: BUSCANDO")

    # ── Callbacks ────────────────────────────────────────────────────────
    def vision_callback(self, msg: String):
        try:
            self.deteccion = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ── Paso principal ───────────────────────────────────────────────────
    def step(self):
        color = self.deteccion.get('color', 'none')
        area  = self.deteccion.get('area',  0.0)
        cx    = self.deteccion.get('cx',    0)
        cx_c  = ANCHO_FRAME // 2

        if self.estado == Estado.BUSCANDO:
            if color != 'none' and area > 500:
                self.get_logger().info(f"Objeto '{color}' detectado → CENTRANDO")
                self._detener()
                self.estado = Estado.CENTRANDO
            else:
                self._vel(0.0, VEL_GIRO)

        elif self.estado == Estado.CENTRANDO:
            if color == 'none':
                self.get_logger().info("Objeto perdido → BUSCANDO")
                self.estado = Estado.BUSCANDO
                return
            error = cx - cx_c
            if abs(error) <= TOLERANCIA_PX:
                self.get_logger().info("Centrado → ACERCANDO")
                self._detener()
                self.estado = Estado.ACERCANDO
            else:
                self._vel(0.0, -float(error) / cx_c * VEL_GIRO_FINO)

        elif self.estado == Estado.ACERCANDO:
            if color == 'none':
                self.get_logger().info("Objeto perdido → BUSCANDO")
                self.estado = Estado.BUSCANDO
                return
            if area >= AREA_CERCA:
                self.get_logger().info(f"Objeto cerca (área={area:.0f}px²) → RECOGIENDO")
                self._detener()
                self._lift('DOWN')
                self.estado       = Estado.RECOGIENDO
                self.timer_inicio = self.get_clock().now()
            else:
                error = cx - cx_c
                self._vel(VEL_AVANCE, -float(error) / cx_c * VEL_GIRO_FINO)

        elif self.estado == Estado.RECOGIENDO:
            dt = self._dt()
            if dt < T_ESPERA_LIFT:
                pass                    # Esperar que baje
            elif dt < T_ESPERA_LIFT * 2:
                self._lift('UP')        # Subir con el objeto
            else:
                self.get_logger().info("Objeto recogido → DEPOSITANDO")
                self.estado       = Estado.DEPOSITANDO
                self.timer_inicio = self.get_clock().now()

        elif self.estado == Estado.DEPOSITANDO:
            if self._dt() < T_DEPOSITO:
                self._vel(VEL_AVANCE, 0.0)
            else:
                self._detener()
                self._lift('DOWN')
                self.get_logger().info(
                    f"Objeto '{self.deteccion.get('color','?')}' depositado → LISTO"
                )
                self.estado       = Estado.LISTO
                self.timer_inicio = self.get_clock().now()

        elif self.estado == Estado.LISTO:
            if self._dt() > 2.0:
                self._lift('STOP')
                self.get_logger().info("Ciclo completo → BUSCANDO")
                self.estado       = Estado.BUSCANDO
                self.timer_inicio = None

    # ── Helpers ──────────────────────────────────────────────────────────
    def _vel(self, lineal: float, angular: float):
        msg = Twist()
        msg.linear.x  = lineal
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _detener(self):
        self._vel(0.0, 0.0)

    def _lift(self, cmd: str):
        m = String()
        m.data = cmd
        self.lift_pub.publish(m)

    def _dt(self) -> float:
        if self.timer_inicio is None:
            return 0.0
        return (self.get_clock().now() - self.timer_inicio).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = RobotFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
