"""
manipulator_core.py  —  NUEVO Taller 3 — Corre en la Raspberry Pi 5
─────────────────────────────────────────────────────────────────────
Controla el mecanismo de elevación del montacargas (o garra del brazo)
mediante un servo conectado al GPIO 18 (PWM hardware de la RPi 5).

Tópico suscrito:
  /lift_cmd  [std_msgs/String]  — "UP" | "DOWN" | "STOP"

Hardware esperado:
  Servo de posición (SG90 / MG996R) en GPIO 18.
  Si usas motor DC + L298N, cambia AngularServo por Motor y
  ajusta los pines según tu PCB.

Pines:
  SERVO_PIN = 18  → señal PWM del servo de elevación
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

try:
    from gpiozero import AngularServo
    GPIO_DISPONIBLE = True
except (ImportError, RuntimeError):
    GPIO_DISPONIBLE = False


SERVO_PIN        = 18
TIEMPO_MOVIMIENTO = 1.0   # segundos que tarda el servo en llegar


class ManipulatorCore(Node):
    def __init__(self):
        super().__init__('manipulator_core_node')

        if GPIO_DISPONIBLE:
            self.servo = AngularServo(
                SERVO_PIN,
                min_angle=-90,
                max_angle=90,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025
            )
            self.get_logger().info(f"Servo de elevación en GPIO {SERVO_PIN} listo.")
        else:
            self.servo = None
            self.get_logger().warn(
                "GPIO no disponible — modo simulación. "
                "Los comandos aparecerán en el log pero no moverán hardware."
            )

        self.sub = self.create_subscription(
            String, '/lift_cmd', self.lift_callback, 10
        )
        self.posicion_actual = ''
        self.get_logger().info(
            "ManipulatorCore listo. Esperando comandos en /lift_cmd."
        )

    def lift_callback(self, msg: String):
        comando = msg.data.strip().upper()
        if comando == self.posicion_actual:
            return

        self.get_logger().info(f"Elevador: {comando}")

        if comando == 'DOWN':
            self._mover(-90.0)
        elif comando == 'UP':
            self._mover(90.0)
        elif comando == 'STOP':
            self._mover(0.0)
        else:
            self.get_logger().warn(
                f"Comando desconocido '{comando}'. Usa UP, DOWN o STOP."
            )
            return

        self.posicion_actual = comando

    def _mover(self, grados: float):
        if self.servo is not None:
            self.servo.angle = grados
            time.sleep(TIEMPO_MOVIMIENTO)

    def destroy_node(self):
        if self.servo is not None:
            self.servo.detach()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
