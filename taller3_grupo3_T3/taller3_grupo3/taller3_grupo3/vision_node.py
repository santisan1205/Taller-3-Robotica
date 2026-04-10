"""
vision_node.py  —  NUEVO Taller 3 — Corre en la PC (NO en la RPi)
──────────────────────────────────────────────────────────────────
Recibe los frames comprimidos, aplica visión por computadora con
OpenCV y publica el objeto detectado (color + forma + centroide).

Tópico suscrito:
  /camera/image_compressed   [sensor_msgs/CompressedImage]

Tópico publicado:
  /detected_object           [std_msgs/String]  — JSON:
      {
        "color": "green" | "blue" | "red" | "none",
        "shape": "cube"  | "sphere" | "unknown",
        "cx":    <int>,   # columna del centroide en píxeles
        "cy":    <int>,   # fila del centroide en píxeles
        "area":  <float>  # área del contorno en px²
      }

Rangos HSV (ajustar con hsv_calibrator.py si los colores no detectan bien):
  Verde → H: 40-80,   S: 50-255, V: 50-255
  Azul  → H: 100-130, S: 80-255, V: 50-255
  Rojo  → H: 0-10 y 170-180 (el rojo envuelve el 0° en HSV)

Discriminación cubo vs esfera por circularidad:
  C = 4π·Área / Perímetro²
  C ≈ 1.0  → esfera / cilindro (circular en 2D)
  C < 0.75 → cubo (cuadrado / polígono)
"""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np


# Rangos de color en espacio HSV — ajusta con hsv_calibrator.py
COLOR_RANGES = {
    'green': [
        (np.array([40,  50,  50]), np.array([80,  255, 255]))
    ],
    'blue': [
        (np.array([100, 80,  50]), np.array([130, 255, 255]))
    ],
    'red': [
        (np.array([0,   80,  50]), np.array([10,  255, 255])),
        (np.array([170, 80,  50]), np.array([180, 255, 255]))
    ],
}

MIN_AREA         = 500    # px² mínimos para contorno válido
CIRCULARITY_THR  = 0.75   # >= esfera, < cubo


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('show_window', True)
        self.show_window = self.get_parameter('show_window').value

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image_compressed',
            self.imagen_callback,
            10
        )
        self.pub = self.create_publisher(String, '/detected_object', 10)

        self.get_logger().info(
            "VisionNode listo. Esperando frames en /camera/image_compressed..."
        )

    def imagen_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mejor    = {'color': 'none', 'shape': 'unknown', 'cx': 0, 'cy': 0, 'area': 0.0}
        max_area = 0.0

        for color_name, rangos in COLOR_RANGES.items():
            mascara = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (low, high) in rangos:
                mascara |= cv2.inRange(hsv, low, high)

            kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN,  kernel)
            mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

            contornos, _ = cv2.findContours(
                mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contornos:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA:
                    continue

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                perimetro    = cv2.arcLength(cnt, True)
                circularidad = (4 * np.pi * area / (perimetro ** 2)
                                if perimetro > 0 else 0)
                forma = 'sphere' if circularidad >= CIRCULARITY_THR else 'cube'

                if area > max_area:
                    max_area = area
                    mejor = {'color': color_name, 'shape': forma,
                             'cx': cx, 'cy': cy, 'area': float(area)}

                if self.show_window:
                    cv2.drawContours(frame, [cnt], -1, (255, 255, 255), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 0), -1)
                    cv2.putText(frame, f"{color_name}/{forma}",
                                (cx - 40, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        msg_out      = String()
        msg_out.data = json.dumps(mejor)
        self.pub.publish(msg_out)

        if mejor['color'] != 'none':
            self.get_logger().info(
                f"Detectado: {mejor['color']}/{mejor['shape']} "
                f"centroide=({mejor['cx']},{mejor['cy']}) "
                f"área={mejor['area']:.0f}px²"
            )

        if self.show_window:
            cv2.imshow('Vision Node - Debug', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
