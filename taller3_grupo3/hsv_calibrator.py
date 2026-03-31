"""
hsv_calibrator.py  —  Script de calibración (NO es un nodo ROS 2)
──────────────────────────────────────────────────────────────────
Herramienta interactiva para encontrar los rangos HSV correctos
para tus objetos físicos bajo las condiciones de luz del laboratorio.

Cómo usar:
  1. Conecta una cámara a tu PC.
  2. Coloca el objeto frente a ella.
  3. Ejecuta:  python3 hsv_calibrator.py
  4. Ajusta los sliders hasta que SOLO el objeto de interés aparezca
     en blanco en la máscara.
  5. Copia los valores H/S/V mínimos y máximos a vision_node.py.

NO necesita ROS 2 instalado. Solo requiere opencv-python:
  pip install opencv-python
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
        print(f"Error: no se pudo abrir la cámara en índice {camara_idx}.")
        return

    # ── Ventana de sliders ──────────────────────────────────────────────
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

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos('H min', 'Calibrador HSV')
        h_max = cv2.getTrackbarPos('H max', 'Calibrador HSV')
        s_min = cv2.getTrackbarPos('S min', 'Calibrador HSV')
        s_max = cv2.getTrackbarPos('S max', 'Calibrador HSV')
        v_min = cv2.getTrackbarPos('V min', 'Calibrador HSV')
        v_max = cv2.getTrackbarPos('V max', 'Calibrador HSV')

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask  = cv2.inRange(hsv, lower, upper)

        # Resultado: frame original con la máscara aplicada
        resultado = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('Original',       frame)
        cv2.imshow('Máscara',        mask)
        cv2.imshow('Resultado',      resultado)
        cv2.imshow('Calibrador HSV', np.zeros((10, 400, 3), dtype=np.uint8))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("\n── VALORES CALIBRADOS ──────────────────────────────")
    print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
    print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
    print("Copia estos valores a COLOR_RANGES en vision_node.py")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
