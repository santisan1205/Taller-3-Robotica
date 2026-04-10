"""
hsv_calibrator.py  —  Script auxiliar de calibración (sin ROS 2)
──────────────────────────────────────────────────────────────────
Herramienta interactiva con sliders para encontrar los rangos HSV
correctos para tus cubos bajo las condiciones de luz del laboratorio.

Uso (en cualquier PC con cámara):
  python3 hsv_calibrator.py          # usa /dev/video0
  python3 hsv_calibrator.py 1        # usa /dev/video1

Pasos:
  1. Coloca el objeto frente a la cámara.
  2. Mueve los sliders hasta que SOLO el objeto aparezca en blanco.
  3. Presiona 'q' — se imprimen los valores listos para copiar
     en COLOR_RANGES dentro de vision_node.py.

Requiere solo: pip install opencv-python
"""

import cv2
import numpy as np
import sys


def nada(_):
    pass


def main():
    camara_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(camara_idx)

    if not cap.isOpened():
        print(f"Error: no se pudo abrir la cámara {camara_idx}.")
        return

    cv2.namedWindow('Calibrador HSV', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Calibrador HSV', 400, 300)
    cv2.createTrackbar('H min', 'Calibrador HSV',   0, 179, nada)
    cv2.createTrackbar('H max', 'Calibrador HSV', 179, 179, nada)
    cv2.createTrackbar('S min', 'Calibrador HSV',  50, 255, nada)
    cv2.createTrackbar('S max', 'Calibrador HSV', 255, 255, nada)
    cv2.createTrackbar('V min', 'Calibrador HSV',  50, 255, nada)
    cv2.createTrackbar('V max', 'Calibrador HSV', 255, 255, nada)

    print("Ajusta los sliders. Presiona 'q' para salir y ver los valores.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos('H min', 'Calibrador HSV')
        h_max = cv2.getTrackbarPos('H max', 'Calibrador HSV')
        s_min = cv2.getTrackbarPos('S min', 'Calibrador HSV')
        s_max = cv2.getTrackbarPos('S max', 'Calibrador HSV')
        v_min = cv2.getTrackbarPos('V min', 'Calibrador HSV')
        v_max = cv2.getTrackbarPos('V max', 'Calibrador HSV')

        mask     = cv2.inRange(hsv,
                               np.array([h_min, s_min, v_min]),
                               np.array([h_max, s_max, v_max]))
        resultado = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('Original',  frame)
        cv2.imshow('Máscara',   mask)
        cv2.imshow('Resultado', resultado)
        cv2.imshow('Calibrador HSV', np.zeros((10, 400, 3), dtype=np.uint8))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\n── VALORES CALIBRADOS ───────────────────────────────")
    print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
    print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
    print("Copia estos valores a COLOR_RANGES en vision_node.py")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
