
"""
Proyecto Final - Visión para robots - Laboratorio de Sistemas Embebidos
Autores:
Sergio Alonso Saldaña Millán
Laura Patricia Carrasco Molina
Benjamín Gutiérrez Padilla
Manuel Amadeo Villarreal

Programa para recibir datos en LoCoBot y realizar el movimiento correspondiente
05 de mayo de 2022
"""
#importación de librerías
import socket
import math
import struct
from interbotix_xs_modules.locobot import InterbotixLocobotXS

#definición de host y port para comunicación con computadora externa
flag=0
HOST = "192.168.119.15"  # Direccion IP de Locobot(Server)
PORT = 65432  # Puerto de Conexion para Socket 

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addre = s.accept()
    with conn:
        print(f"Connected by {addre}")

        #inicialización de LoCoBot
        locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")
        print("Voy a home")

        #Posicion de seguridad inicial
        locobot.arm.go_to_home_pose()
        locobot.arm.go_to_sleep_pose()
        locobot.arm.set_ee_pose_components(x=0.3, z=0.2) 

        #Bandera de seguridad
        movement = 0

        while True:

            data = conn.recv(1024)
            tamanio=len(data)

            #Cuando el dato es nulo
            if tamanio==0:
                print("Gracias por Utilizar el Gesture Controlled Robot")

            #Para recibir el valor de radianes y llevar el robot a esa posición
            elif tamanio<=4:
                datad=struct.unpack('f',data)
                locobot.arm.set_single_joint_position("waist", datad[0])

            #Para recibir el gesto y hacer posteriormente el movimiento correspondiente
            elif(tamanio<12):
                datad=data.decode("utf-8")

            #Elección de gesto:

            #Si recibe un 'peace', realiza trayectoria en x y z
            if (datad == 'peace'):
                print(f"Estoy mandando un peace")
                movement = movement - 1
                if(movement == 0):
                    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.15)
                else:
                    movement = 0

            #Si recibe un 'okayy', va a posición de seguridad       
            if (datad == 'okayy') :
                print(f"Estoy mandando un okay")
                locobot.arm.set_ee_pose_components(x=0.3, z=0.2) 

            #Si recibe un 'thumbs up', gira hasta llegar a -90° 
            if (datad == 'thumbs up'):
                print(f"Estoy mandando un thumbs up")
                locobot.arm.set_single_joint_position("waist", -math.pi/2.0)

            #Si recibe un 'thumbs down', realiza un pitch 
            if (datad == 'thumbs down'):
                print(f"Estoy mandando un thumbs down")
                locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)

            #Si recibe un 'smile', "regresa" el pitch 
            if (datad == 'smile'):
                print(f"Estoy mandando un smile")
                locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)

            if (datad == 'rockk'):
                print(f"Estoy mandando un rock")
                print(datad)

            #Si recibe un 'fistt', cierra el gripper 
            if (datad == 'fistt'):
                print(f"Estoy mandando un fist")
                locobot.gripper.close()

            #Si recibe un 'call me', realiza trayectoria en x y z
            if (datad == 'call me'):
                print(f"Estoy mandando un call me")
                movement = movement + 1
                if(movement == 1):
                    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.15)
                else:
                    movement = 1

            #Si recibe un 'live long', abre el gripper
            if (datad == 'live long'):
                print(f"Estoy mandando un live long")
                locobot.gripper.open()

            #Si no recibe dato, se termina el while
            if not data:
                break
            conn.sendall(data)

    #Cuando termina el programa, regresa posición de sleep
    locobot.gripper.close()
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()
      