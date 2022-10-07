# TechVidvan hand Gesture Recognizer

# importación de librerias
import cv2
import math
import numpy as np
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model
import numpy as np
#Importamos libreria de Socket
import socket


import time
import struct

from cv2_acquire_visualize_module import *
from face_recognition_module import *

contador=0 # Variable para contar frames en gestos
contadorFinger=0 # Variable para contar frames para funcion de dedo
flag=0 # Bandera numerica por gesto
HOST = "192.168.119.15"  # Dirección IP del Servidor
PORT = 65432  # Puerto para escuchar

# Inicializacion de camara
video_capture = init_camera()

# Se carga el modelo pre entrendado de reconocimiento de manos
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils

# Se carga el modelo de reconocimiento de rostro
known_faces_encodings, known_faces_names = load_known_faces_and_encode(KNOWN_FACES_DIR)

# Inicializacion de posiciones del dedo indice
x2=0
y2=0

# Variables para reconocimiento facial
lastPublication = 0.0
PUBLISH_TIME   = 1     # seconds

# Codigo de reconocimiento facial como medida de seguridad
while (True):
    # Se obtiene la imagen
    rgb_frame, scaled_rgb_frame = acquire_image(video_capture)
    # Se obtienen los rostros de la imagen
    face_locations, face_encodings = extract_faces_and_encode(scaled_rgb_frame)
    # Reconocimiento de la persona
    face_names = find_face_matches(face_encodings, known_faces_encodings, known_faces_names)
    if np.abs(time.time()-lastPublication) > PUBLISH_TIME:
        try:
            print(face_names)
        except (keyboardInterrupt):
            break
        except Exception as e:
            print(e)
        lastPublication = time.time()

    # Se muestra la imagen
    show_frame (rgb_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Se cierra la cámara
        break

    if face_names == ['Unknown']:
        break

# Se destruyen la ventana correspondiente a la cámara
video_capture.release()
cv2.destroyAllWindows()

# Codigo necesario para mandar indicaciones por medio de gestos
if face_names == ['Unknown']: # Si se reconoce un rostro
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Intento de conexión por Socket a la IP y puerto inicializado
        s.connect((HOST, PORT))
        # Inicialización mediapipe
        mpHands = mp.solutions.hands
        hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        mpDraw = mp.solutions.drawing_utils

        # Se carga el modelo preentrenado de reconocimiento de gestos
        model = load_model('mp_hand_gesture')

        # Se cargan los nombres de las clases
        f = open('gesture.names', 'r')
        classNames = f.read().split('\n')
        f.close()
        print(classNames)

        # Se inicializa la webcam
        cap = cv2.VideoCapture(0)

        # Incializacion de bandera para activar funcion de seguimiento de dedo
        finger = False

        while True:
            # Lectura de cada frame de la camara
            _, frame = cap.read()
            x, y, c = frame.shape

            # Inversion de la cámara
            frame = cv2.flip(frame, 1)
            framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Se obtienen los puntos claves de la mano detectada
            result = hands.process(framergb)

            # Inicializacion de variables para acumular los puntos y el nombre del gesto de la clase
            className = ''
            lmList = []

            # Procesamiento de las marcas
            if result.multi_hand_landmarks: # Si se encuentra una mano
                landmarks = []
                for handslms in result.multi_hand_landmarks: # Por cada mano
                    for id, lm in enumerate(handslms.landmark): # Por cada marca
                        # Se obtienen las coordenadas
                        lmx = int(lm.x * x) 
                        lmy = int(lm.y * y)
                        h, w, c = frame.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        lmList.append([id, cx, cy]) 
                        landmarks.append([lmx, lmy])

                    # Si se requiere mostras las marcas en el frame, descomentar la siguiente linea
                    # mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

                    # Si se encontraron marcas
                    if lmList != []:
                        # Obtener las coordenadas de la punta del dedo indice [8]
                        x2, y2 = lmList[8][1], lmList[8][2]
                        #cv2.circle(frame, (x2, y2), 10, (255, 0, 0), cv2.FILLED) 
                        # Mapeo de pixeles a grados 
                        fingerMap = round(np.interp(x2, [0, 600], [-90, 90]),2)
                        # Conversion de grados a radianes
                        radians = math.radians(fingerMap) 
                        #print(x2)
                        #print(fingerMap)
                        #print(radians)

                    # Predicción de Getso
                    prediction = model.predict([landmarks])
                    # Obtención del ID de clase
                    classID = np.argmax(prediction)
                    # Obtención del nombre de la clase
                    className = classNames[classID]

            # Mostrar nombre de clase en frame
            cv2.putText(frame, className, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0,0,255), 2, cv2.LINE_AA)

            # Mostrar frame
            cv2.imshow("Output", frame)
            
            # Algoritmo para reconocimiento de cada gesto
            # Se deben detectar 20 veces consecutivas el mismo gesto para que pueda ser enviado al Locobot
            # Las condiciones comparan el nombre de la clase obtenida con la predicción
            # 'contador' es la variable para contar 20 veces el mismo gesto
            # 'flag' nos permite reiniciar el valor a 0 si se detectar otro gesto
            if className == "okay" :
                if(flag!=1):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        # Envio de datos por medio de Socket
                        s.send(b"okayy")
                        # Recepción de datos
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=1
            
            # El proceso se repite con los demás gestos
            if className == "thumbs up" :
                if(flag!=2):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"thumbs up")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=2
            if className == "peace" :
                if(flag!=3):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"peace")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=3
            if className == "rock" :
                if(flag!=4):
                    contador=0
                else:
                    contador+=1
                    # En 'rock' el contador solo tiene que llegar a 15, dandole una mayor prioridad
                    if(contador==15):
                        # 'rock' activa o desactiva la bandera de seguimiento de dedo
                        if finger:
                            finger = False
                        else:
                            finger = True
                            s.sendall(b"rockk")
                            datad = s.recv(1024)
                            print("Ahora seguire tu dedo")
                flag=4
            if className == "fist" :
                if(flag!=5):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"fistt")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=5
            if className == "call me" :
                if(flag!=6):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"call me")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=6
            if className == "live long" :
                if(flag!=7):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"live long")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=7
            if className == "thumbs down" :
                if(flag!=8):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"thumbs down")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=8
            if className == "smile" :
                if(flag!=9):
                    contador=0
                else:
                    contador+=1
                    if(contador==20):
                        s.send(b"smile")
                        datad = s.recv(1024)
                        print(f"Received {datad!r}")
                flag=9
            
            # Algoritmo de seguimiento de dedo
            if finger: # Si la bandera está activa
                contadorFinger +=1 # Contador de dedo
                if(contadorFinger == 8): # después de 8 frames 
                    if lmList != []: # Si se encontró una mano
                        # Se enpaqueta radianes y se manda por medio del socket
                        bradians = struct.pack("f", radians)
                        print(bradians)
                        s.sendall(bradians)
                        # Se reinicia el contador
                        datad = s.recv(1024)
                    contadorFinger=0

            # Si se oprime la tecla 'q' se sale
            if cv2.waitKey(1) == ord('q'):
                break
else: #En caso de que no se reconozca el rostro
    print("Ni te topo vato")
# Liberación de camara
cap.release()
cv2.destroyAllWindows()