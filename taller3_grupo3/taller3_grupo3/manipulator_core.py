"""
manipulator_core.py  —  Corre en la Raspberry Pi 5
────────────────────────────────────────────────────
Controla el mecanismo de elevación del montacargas (o la garra del brazo)
a través de los pines GPIO de la RPi usando gpiozero.

Tópico suscrito:
  /lift_cmd  [std_msgs/String]  — "UP" | "DOWN" | "STOP"

El hardware esperado es:
  • Un servo de posición (ej. SG90 / MG996R) conectado a GPIO 18 (PWM).
  • Si usas un motor DC + driver L298N para el elevador, cambia
    'Servo' por 'Motor' y ajusta los pines según tu PCB.

Pines por defecto (ajusta a tu diseño):
  SERVO_PIN = 18   → señal PWM del servo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# ── Importación condicional de gpiozero ───────────────────────────────────
# Permite correr el nodo en una PC para pruebas (sin GPIO real).
try:
    from gpiozero import Servo, AngularServo
    GPIO_DISPONIBLE = True
except (ImportError, RuntimeError):
    GPIO_DISPONIBLE = False


# ── Constantes de posición del servo ─────────────────────────────────────
SERVO_PIN      = 18   # Pin BCM del servo de elevación
POS_ARRIBA     = 1.0  # Valor máximo gpiozero  (+90°)
POS_ABAJO      = -1.0 # Valor mínimo gpiozero  (-90°)
POS_MEDIO      = 0.0  # Posición neutral

# Tiempo de espera para que el servo llegue a la posición
TIEMPO_MOVIMIENTO = 1.0   # segundos


class ManipulatorCore(Node):
    def __init__(self):
        super().__init__('manipulator_core_node')

        # ── Inicializar servo ─────────────────────────────────────────────
        if GPIO_DISPONIBLE:
            # min_pulse_width y max_pulse_width en segundos (ajusta a tu servo)
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
                "GPIO no disponible (modo simulación). "
                "Los comandos se mostrarán en el log pero no moverán hardware."
            )

        # ── Subscriber ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            String,
            '/lift_cmd',
            self.lift_callback,
            10
        )

        self.posicion_actual = 'STOP'
        self.get_logger().info("ManipulatorCore listo. Esperando comandos en /lift_cmd.")

    # ── Callback ──────────────────────────────────────────────────────────
    def lift_callback(self, msg: String):
        comando = msg.data.strip().upper()

        if comando == self.posicion_actual:
            return  # Evitar re-enviar el mismo comando

        self.get_logger().info(f"Comando recibido: {comando}")

        if comando == 'DOWN':
            self._mover_servo(POS_ABAJO)
            self.get_logger().info("Elevador BAJANDO — listo para recoger objeto.")

        elif comando == 'UP':
            self._mover_servo(POS_ARRIBA)
            self.get_logger().info("Elevador SUBIENDO — objeto recogido.")

        elif comando == 'STOP':
            self._mover_servo(POS_MEDIO)
            self.get_logger().info("Elevador en posición NEUTRAL.")

        else:
            self.get_logger().warn(
                f"Comando desconocido: '{comando}'. "
                "Usa 'UP', 'DOWN' o 'STOP'."
            )
            return

        self.posicion_actual = comando

    def _mover_servo(self, angulo: float):
        """Envía la posición al servo y espera a que llegue."""
        if self.servo is not None:
            # AngularServo acepta ángulos en grados (-90 a 90)
            grados = angulo * 90.0
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
