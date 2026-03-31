"""
robot_fsm.py  —  Máquina de Estados del Robot (FSM)
─────────────────────────────────────────────────────
Coordina la secuencia autónoma completa:
  BUSCANDO → CENTRANDO → ACERCANDO → RECOGIENDO → DEPOSITANDO → LISTO

Tópicos suscritos:
  /detected_object  [std_msgs/String]  — JSON del vision_node
  /odom             [nav_msgs/Odometry]

Tópicos publicados:
  /cmd_vel          [geometry_msgs/Twist]  — mueve las ruedas
  /lift_cmd         [std_msgs/String]      — controla el elevador

┌─────────────────────────────────────────────────────────────────────┐
│  DIAGRAMA DE ESTADOS                                                 │
│                                                                      │
│  BUSCANDO ──(objeto visto)──► CENTRANDO ──(centrado)──► ACERCANDO   │
│      ▲                                                      │        │
│      │                                                  (cerca)      │
│      │                                                      ▼        │
│  LISTO ◄──(vuelve)── DEPOSITANDO ◄──── RECOGIENDO          │        │
│                                   ↑                         │        │
│                            (subió elevador)                 │        │
└─────────────────────────────────────────────────────────────────────┘

Parámetros importantes:
  cx_centro       : columna del centroide que se considera "centrado"
  tolerancia_px   : margen de píxeles para considerar objeto centrado
  area_cerca      : área mínima (px²) para considerar el objeto "cerca"
  vel_giro        : velocidad angular al buscar/centrar (rad/s)
  vel_avance      : velocidad lineal al acercarse (m/s)
  t_deposito      : segundos que avanza hacia la zona de depósito
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
    BUSCANDO    = auto()   # Gira sobre su eje buscando un objeto
    CENTRANDO   = auto()   # Ajusta yaw para que el centroide quede al centro
    ACERCANDO   = auto()   # Avanza hacia el objeto
    RECOGIENDO  = auto()   # Baja el elevador, espera, sube
    DEPOSITANDO = auto()   # Se mueve a la zona de depósito
    LISTO       = auto()   # Ciclo completado, espera nuevo objeto


# ── Parámetros de control (ajusta según tu robot) ─────────────────────────
ANCHO_FRAME     = 640     # px — debe coincidir con camera_node
TOLERANCIA_PX   = 40      # px — margen para "objeto centrado"
AREA_CERCA      = 18000   # px² — objeto suficientemente cerca para recoger
VEL_GIRO        = 0.35    # rad/s al buscar
VEL_AVANCE      = 0.15    # m/s al acercarse
VEL_GIRO_FINO   = 0.15    # rad/s al centrar (más lento)
T_DEPOSITO      = 3.0     # s avanzando hacia la zona de depósito
T_ESPERA_LIFT   = 1.5     # s esperando que el elevador termine


class RobotFSM(Node):
    def __init__(self):
        super().__init__('robot_fsm_node')

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',   10)
        self.lift_pub = self.create_publisher(String, '/lift_cmd',  10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.sub_vision = self.create_subscription(
            String, '/detected_object', self.vision_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # ── Estado interno ────────────────────────────────────────────────
        self.estado        = Estado.BUSCANDO
        self.deteccion     = {'color': 'none', 'shape': 'unknown',
                              'cx': 0, 'cy': 0, 'area': 0.0}
        self.timer_inicio  = None  # para estados con temporización

        # ── Timer principal de la FSM (10 Hz) ────────────────────────────
        self.timer = self.create_timer(0.1, self.step)

        self.get_logger().info("RobotFSM iniciado. Estado: BUSCANDO")

    # ── Callbacks de datos ────────────────────────────────────────────────
    def vision_callback(self, msg: String):
        try:
            self.deteccion = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("JSON inválido en /detected_object")

    def odom_callback(self, msg: Odometry):
        # Guardado para uso futuro (navegación precisa al depósito)
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        q          = msg.pose.pose.orientation
        siny_cosp  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    # ── Paso de la FSM ────────────────────────────────────────────────────
    def step(self):
        """Se llama cada 100 ms. Decide qué hacer según el estado actual."""

        color_detectado = self.deteccion.get('color', 'none')
        area            = self.deteccion.get('area',  0.0)
        cx              = self.deteccion.get('cx',    0)
        cx_centro       = ANCHO_FRAME // 2

        # ─────────────────────────────────────────────────────────────────
        if self.estado == Estado.BUSCANDO:
            if color_detectado != 'none' and area > 500:
                self.get_logger().info(
                    f"Objeto {color_detectado} detectado. Pasando a CENTRANDO."
                )
                self._detener()
                self.estado = Estado.CENTRANDO
            else:
                # Girar lentamente sobre el eje buscando colores
                self._publicar_vel(0.0, VEL_GIRO)

        # ─────────────────────────────────────────────────────────────────
        elif self.estado == Estado.CENTRANDO:
            if color_detectado == 'none':
                self.get_logger().info("Objeto perdido. Volviendo a BUSCANDO.")
                self.estado = Estado.BUSCANDO
                return

            error_px = cx - cx_centro
            if abs(error_px) <= TOLERANCIA_PX:
                self.get_logger().info("Objeto centrado. Pasando a ACERCANDO.")
                self._detener()
                self.estado = Estado.ACERCANDO
            else:
                # Proporcional simple: error positivo → objeto a la derecha
                # → girar a la derecha (angular negativo en ROS)
                w = -float(error_px) / cx_centro * VEL_GIRO_FINO
                self._publicar_vel(0.0, w)

        # ─────────────────────────────────────────────────────────────────
        elif self.estado == Estado.ACERCANDO:
            if color_detectado == 'none':
                self.get_logger().info("Objeto perdido. Volviendo a BUSCANDO.")
                self.estado = Estado.BUSCANDO
                return

            if area >= AREA_CERCA:
                self.get_logger().info(
                    f"Objeto cerca (área={area:.0f}px²). Pasando a RECOGIENDO."
                )
                self._detener()
                self.estado     = Estado.RECOGIENDO
                self.timer_inicio = self.get_clock().now()
                # Bajar el elevador
                self._enviar_lift('DOWN')
            else:
                # Avanzar y corregir ligero descentrado al mismo tiempo
                error_px = cx - cx_centro
                w = -float(error_px) / cx_centro * VEL_GIRO_FINO
                self._publicar_vel(VEL_AVANCE, w)

        # ─────────────────────────────────────────────────────────────────
        elif self.estado == Estado.RECOGIENDO:
            # Esperar T_ESPERA_LIFT para que el elevador baje completamente,
            # luego subir
            dt = self._segundos_en_estado()
            if dt < T_ESPERA_LIFT:
                pass  # Esperar (elevador bajando)
            elif dt < T_ESPERA_LIFT * 2:
                self._enviar_lift('UP')  # Subir con el objeto
            else:
                self.get_logger().info("Objeto recogido. Pasando a DEPOSITANDO.")
                self.estado       = Estado.DEPOSITANDO
                self.timer_inicio = self.get_clock().now()

        # ─────────────────────────────────────────────────────────────────
        elif self.estado == Estado.DEPOSITANDO:
            dt = self._segundos_en_estado()
            if dt < T_DEPOSITO:
                # Avanzar hacia la zona de depósito (hard-coded por ahora)
                # En un sistema más avanzado usarías odometría o un marcador
                self._publicar_vel(VEL_AVANCE, 0.0)
            else:
                self._detener()
                self._enviar_lift('DOWN')   # Depositar
                self.get_logger().info(
                    f"Objeto {self.deteccion.get('color','?')} "
                    "depositado. Estado: LISTO"
                )
                self.estado = Estado.LISTO

        # ─────────────────────────────────────────────────────────────────
        elif self.estado == Estado.LISTO:
            # Esperar 2 segundos y volver a buscar (permite múltiples objetos)
            if self.timer_inicio is None:
                self.timer_inicio = self.get_clock().now()
            elif self._segundos_en_estado() > 2.0:
                self.get_logger().info("Listo. Volviendo a BUSCANDO.")
                self._enviar_lift('STOP')
                self.estado       = Estado.BUSCANDO
                self.timer_inicio = None

    # ── Métodos auxiliares ────────────────────────────────────────────────
    def _publicar_vel(self, lineal: float, angular: float):
        msg = Twist()
        msg.linear.x  = lineal
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _detener(self):
        self._publicar_vel(0.0, 0.0)

    def _enviar_lift(self, comando: str):
        msg      = String()
        msg.data = comando
        self.lift_pub.publish(msg)

    def _segundos_en_estado(self) -> float:
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
