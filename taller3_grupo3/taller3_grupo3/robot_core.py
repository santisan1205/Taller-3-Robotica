"""
robot_core.py  —  Corre en la Raspberry Pi 5
──────────────────────────────────────────────
Controla los motores de tracción diferencial y publica la odometría.
Sin cambios estructurales respecto al Taller 2, excepto:
  • Se eliminó la referencia al servicio de reproducción de trayectorias.
  • Se ajustó el log de arranque.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

try:
    from gpiozero import Motor, DigitalInputDevice
    GPIO_DISPONIBLE = True
except (ImportError, RuntimeError):
    GPIO_DISPONIBLE = False


class RobotCore(Node):
    def __init__(self):
        super().__init__('robot_core_node')

        # ── Motores ───────────────────────────────────────────────────────
        if GPIO_DISPONIBLE:
            self.motor_izq = Motor(forward=5,  backward=6,  enable=12)
            self.motor_der = Motor(forward=23, backward=24, enable=13)
        else:
            self.motor_izq = None
            self.motor_der = None
            self.get_logger().warn("GPIO no disponible — modo simulación.")

        # ── Encoders ──────────────────────────────────────────────────────
        if GPIO_DISPONIBLE:
            self.enc_izq_a = DigitalInputDevice(17)
            self.enc_izq_b = DigitalInputDevice(27)
            self.enc_der_a = DigitalInputDevice(22)
            self.enc_der_b = DigitalInputDevice(25)

            self.enc_izq_a.when_activated = self.contar_tick_izq
            self.enc_der_a.when_activated = self.contar_tick_der

        self.ticks_izq = 0
        self.ticks_der = 0

        # ── Parámetros físicos ────────────────────────────────────────────
        self.R               = 0.06    # Radio rueda (m)
        self.L               = 0.209   # Distancia entre ruedas (m)
        self.TICKS_POR_VUELTA = 374.0  # Reducción motores

        # ── Odometría ─────────────────────────────────────────────────────
        self.x         = 0.0
        self.y         = 0.0
        self.theta     = 0.0
        self.last_time = self.get_clock().now()

        # ── ROS 2 I/O ─────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer    = self.create_timer(0.05, self.publicar_odometria)  # 20 Hz

        self.get_logger().info(
            "RobotCore iniciado. Motores y encoders listos. "
            "Escuchando /cmd_vel."
        )

    # ── Encoders ──────────────────────────────────────────────────────────
    def contar_tick_izq(self):
        if GPIO_DISPONIBLE and self.enc_izq_b.is_active:
            self.ticks_izq -= 1
        else:
            self.ticks_izq += 1

    def contar_tick_der(self):
        if GPIO_DISPONIBLE and self.enc_der_b.is_active:
            self.ticks_der += 1
        else:
            self.ticks_der -= 1

    # ── Control de motores ────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_izq = v + (w * self.L / 2.0)
        v_der = v - (w * self.L / 2.0)

        pwm_izq = max(-1.0, min(1.0, v_izq / 0.5))
        pwm_der = max(-1.0, min(1.0, v_der / 0.5))

        if self.motor_izq:
            self.motor_izq.value = pwm_izq
            self.motor_der.value = pwm_der

    # ── Odometría ─────────────────────────────────────────────────────────
    def publicar_odometria(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        dist_izq = 2.0 * math.pi * self.R * (self.ticks_izq / self.TICKS_POR_VUELTA)
        dist_der = 2.0 * math.pi * self.R * (self.ticks_der / self.TICKS_POR_VUELTA)
        self.ticks_izq = 0
        self.ticks_der = 0

        d_centro = (dist_izq + dist_der) / 2.0
        d_theta  = (dist_der - dist_izq) / self.L

        self.x     += d_centro * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_centro * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        odom = Odometry()
        odom.header.stamp          = current_time.to_msg()
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        if dt > 0:
            odom.twist.twist.linear.x  = d_centro / dt
            odom.twist.twist.angular.z = d_theta  / dt

        self.odom_pub.publish(odom)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = RobotCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
