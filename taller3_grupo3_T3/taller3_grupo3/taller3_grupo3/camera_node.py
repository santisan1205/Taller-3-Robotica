"""
camera_node.py  —  NUEVO Taller 3 — Corre en la Raspberry Pi 5
───────────────────────────────────────────────────────────────
Captura frames del módulo de cámara RPi (5 MP, interfaz CSI) y los
publica comprimidos en JPEG hacia la PC mediante ROS 2.

Por qué CompressedImage y no Image cruda:
  Imagen cruda 640x480 RGB  → ~900 KB por frame → ~90 MB/s a 30 fps
  JPEG comprimida al 80 %   →  ~20 KB por frame →  ~2 MB/s a 30 fps
  Esto es crítico en una red Wi-Fi doméstica.

Tópico publicado:
  /camera/image_compressed  [sensor_msgs/CompressedImage]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parámetros configurables desde la línea de comandos
        self.declare_parameter('fps',           15)
        self.declare_parameter('width',         640)
        self.declare_parameter('height',        480)
        self.declare_parameter('jpeg_quality',  80)
        self.declare_parameter('camera_index',  0)   # /dev/video0

        fps          = self.get_parameter('fps').value
        width        = self.get_parameter('width').value
        height       = self.get_parameter('height').value
        self.quality = self.get_parameter('jpeg_quality').value
        cam_idx      = self.get_parameter('camera_index').value

        # Publisher
        self.pub = self.create_publisher(
            CompressedImage, '/camera/image_compressed', 10
        )

        # Abrir cámara (OpenCV funciona tanto en RPi OS como en Ubuntu)
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        if not self.cap.isOpened():
            self.get_logger().error(
                f"No se pudo abrir la cámara en índice {cam_idx}. "
                "Verifica que el módulo CSI está conectado y habilitado."
            )
            raise RuntimeError("Cámara no disponible")

        self.timer = self.create_timer(1.0 / fps, self.capturar_y_publicar)
        self.get_logger().info(
            f"CameraNode listo: {width}x{height} @ {fps} fps — "
            "publicando en /camera/image_compressed"
        )

    def capturar_y_publicar(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame vacío — reintentando...")
            return

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        ok, buffer = cv2.imencode('.jpg', frame, encode_params)
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.format          = 'jpeg'
        msg.data            = buffer.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
        rclpy.spin(node)
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
