import cv2
import mediapipe as mp
import socket
import time
import math
import numpy as np

# ================== CONFIG ==================
HOST = '3.149.222.108'
PORT = 4062
BUFFER_SIZE = 1024

CAMERA_INDEX = 0
SEND_DELAY = 0.05   # 50 Hz aprox

# ============================================

# -------- TCP --------
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((HOST, PORT))
print("Conectado al servidor")

# -------- MediaPipe --------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.6
)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(CAMERA_INDEX)

# ---------- Estados ----------
pot_value = 0
ejeX = 2047
ejeY = 2047

reproducir =0
pausa = 0
adelante = 0
atras = 0
boton = 0
detener = 0

prev_reproducir = 0
prev_pausa = 0
prev_adelante = 0
prev_atras = 0
prev_boton = 0
prev_detener = 0

cursor_activo = False
ref_x = 0
ref_y = 0
# ---------- Utilidades ----------
def fingers_up(lm, handedness):
    fingers = []

    # Pulgar
    if handedness == "Right":
        fingers.append(1 if lm[4].x < lm[3].x else 0)
    else:
        fingers.append(1 if lm[4].x > lm[3].x else 0)

    # Otros dedos
    for tip in [8, 12, 16, 20]:
        fingers.append(1 if lm[tip].y < lm[tip - 2].y else 0)

    return fingers

def pinch_distance(lm):
    x1, y1 = lm[4].x, lm[4].y
    x2, y2 = lm[8].x, lm[8].y
    return math.hypot(x2 - x1, y2 - y1)

def send(msg):
    server.send(bytes(msg, 'utf-8'))

# ================== LOOP ==================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        # Reset estados discretos
        reproducir = 0
        pausa = 0
        adelante = 0
        atras = 0
        boton = 0
        detener = 0

        left_fingers = None
        right_fingers = None
        lm_left = None
        lm_right = None

        if result.multi_hand_landmarks:
            for i, hand in enumerate(result.multi_hand_landmarks):
                handedness = result.multi_handedness[i].classification[0].label
                lm = hand.landmark
                fingers = fingers_up(lm, handedness)

                if handedness == "Left":
                    left_fingers = fingers
                    lm_left = lm
                else:
                    right_fingers = fingers
                    lm_right = lm

                mp_draw.draw_landmarks(
                    frame, hand, mp_hands.HAND_CONNECTIONS
                )

        # ======= MANO IZQUIERDA =======
        if left_fingers:
            # Play / pausa
            if left_fingers == [0,1,0,0,0]:
                reproducir = 1

            elif left_fingers == [0,1,1,0,0]:
                pausa = 1

            # Siguiente
            elif left_fingers == [0,1,1,1,0]:
                adelante = 1

            # Anterior
            elif left_fingers == [0,0,0,0,1]:
                atras = 1

            # Stop / Click
            elif left_fingers == [0,0,0,0,0]:
                boton = 1

        # ======= MANO DERECHA =======
        dx = 0
        dy = 0
        
        if right_fingers and lm_right:
            
            if right_fingers == [0,0,0,0,0]:
                detener = 1

            # -------- Volumen (pellizco) --------
            if lm_right:

                pinch = pinch_distance(lm_right)

                palm = math.hypot(
                    lm_right[0].x - lm_right[12].x,
                    lm_right[0].y - lm_right[12].y
                )

                if palm > 0:
                    ratio = pinch / palm

                    # SOLO cuando realmente hay pellizco
                    if ratio < 0.6:
                        pot_value = int(np.interp(ratio, [0.05, 0.6], [0, 4095]))
                        pot_value = max(0, min(4095, pot_value))

            # -------- Cursor relativo --------
            if right_fingers == [0,1,0,0,0]:

                x = lm_right[8].x
                y = lm_right[8].y

                if not cursor_activo:
                    # Primer frame del gesto
                    ref_x = x
                    ref_y = y
                    cursor_activo = True

                dx = x - ref_x
                dy = y - ref_y
                
            else:
                cursor_activo = False

            # Escalado (ajústalo si quieres)
            SENS = 3000

            ejeX = int(2047 + dx * SENS)
            ejeY = int(2047 + dy * SENS)

            ejeX = max(0, min(4095, ejeX))
            ejeY = max(0, min(4095, ejeY))

        else:
            cursor_activo = False
            ejeX = 2047
            ejeY = 2047

        # ======= ENVÍO TCP =======
        send(f"<pot>{pot_value}\n")
        send(f"<ejeX>{ejeX}\n")
        send(f"<ejeY>{ejeY}\n")
        
        if reproducir != prev_reproducir:
            send(f"<reproducir>{reproducir}\n")
            prev_reproducir = reproducir

        if pausa != prev_pausa:
            send(f"<pausa>{pausa}\n")
            prev_pausa = pausa

        if adelante != prev_adelante:
            send(f"<adelante>{adelante}\n")
            prev_adelante = adelante

        if atras != prev_atras:
            send(f"<atras>{atras}\n")
            prev_atras = atras

        if boton != prev_boton:
            send(f"<boton>{boton}\n")
            prev_boton = boton

        if detener != prev_detener:
            send(f"<detener>{detener}\n")
            prev_detener = detener


        cv2.imshow("Gesture Sender", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

        time.sleep(SEND_DELAY)

except KeyboardInterrupt:
    pass

finally:
    cap.release()
    cv2.destroyAllWindows()
    server.close()
    print("Conexión cerrada")
