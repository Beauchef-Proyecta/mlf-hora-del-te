import serial
import time
from mk2robot import MK2Robot

import sys
import tty
import termios
import os

# En este archivo se combina lo de where_to_raspi.py del repo mlf-base-dev,
# branch uwu con extras para mover el robot con el teclado. No lo he probado
# conectado al robot aun :c

#BLOQUE SERIAL
ser = serial
try:
    #ser = serial.Serial("COM5", 115200, timeout=1) #ojito con la ruta
    ser = serial.Serial("/dev/USB0", 115200, timeout=1)
    serial_port = "Open"
    print("The port is available")

except serial.serialutil.SerialException:
    print("The port is at use")
    ser.close()
    ser.open()
#BLOQUE SERIAL

robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot.update_pose(0, 0, 90)
[X_pos, Y_pos, Z_pos] = robot.get_joint_positions()


def base_ang_ik(q0):
    time.sleep(0.015)
    angleData = str(int(abs(q0)))
    ser.write(('&1:' + angleData).encode())
    time.sleep(0.02)

def L1_ang_ik(q1):
    #time.sleep(0.5)
    angleData2 = str(int(abs(q1)))
    ser.write(('&2:' + angleData2).encode())
    time.sleep(0.03)

def L2_ang_ik(q2):
    # time.sleep(0.5)
    angleData3 = str(int(abs(q2)))
    ser.write(('&3:' + angleData3).encode())
    time.sleep(0.03)


#Definicion de funcion que recupera la posición del slider y publica el angulo en la ID del servo
def base_ang(q0_ang):
    time.sleep(0.015)
    ser.write(('&1:' + str(int(q0_ang))).encode())
    time.sleep(0.02)

def L1_ang(q1_ang):
    ser.write(('&2:' + str(int(q1_ang))).encode())
    time.sleep(0.03)

def L2_ang(q2_ang):
    # time.sleep(0.5)
    ser.write(('&3:' + str(int(q2_ang))).encode())
    time.sleep(0.03)

def eff_ang(q3_ang):
    ser.write(('&4:' + str(int(q3_ang))).encode())
    time.sleep(0.03)


# Funciones para controlar el robot con el teclado
def refresh_terminal(q, dq, go_back_n=15):
    sys.stdout.write("\033[%iA"%go_back_n) # go up n lines
    print(" Controles: ")
    print("q0+, q0-: q, a     dq0+, dq0-: Q, A")
    print("q1+, q1-: w, s     dq1+, dq1-: W, S")
    print("q2+, q2-: e, d     dq2+, dq2-: E, D")
    print("q3+, q3-: r, f     dq3+, dq3-: R, F")
    print("")
    print("")
    print("Posiciones:      Velocidades:")
    print("q0: %3i          dq0: %2i"%(q[0], dq[0]))
    print("q1: %3i          dq1: %2i"%(q[1], dq[1]))
    print("q2: %3i          dq2: %2i"%(q[2], dq[2]))
    print("q3: %3i          dq3: %2i"%(q[3], dq[3]))
    print("")
    print("Presionar una sola tecla a la vez")
    print("ESC para salir")

def getKey():
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            b = os.read(sys.stdin.fileno(), 3).decode()
            if len(b)==1:
                return b
            else:
                return ' '
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def ajustar_parametros(q,dq,key):
    # Cambiar posiciones
    if key=='q':
        q[0] += dq[0]
    elif key=='a':
        q[0] -= dq[0]
    elif key=='w':
        q[1] += dq[1]
    elif key=='s':
        q[1] -= dq[1]
    elif key=='e':
        q[2] += dq[2]
    elif key=='d':
        q[2] -= dq[2]
    elif key=='r':
        q[3] += dq[3]
    elif key=='f':
        q[3] -= dq[3]
    # Cambiar velocidad
    elif key=='Q':
        dq[0] += 1
    elif key=='A':
        dq[0] -= 1
    elif key=='W':
        dq[1] += 1
    elif key=='S':
        dq[1] -= 1
    elif key=='E':
        dq[2] += 1
    elif key=='D':
        dq[2] -= 1
    elif key=='R':
        dq[3] += 1
    elif key=='F':
        dq[3] -= 1

    # Corregir posiciones imposibles
    for i in range(4):
        if q[i] < 0  : q[i]=0
        if q[i] > 180: q[i]=180
        if dq[i]< 1  : dq[i]=1
    return q, dq

def move_robot(q):
    base_ang(q[0])
    L1_ang(q[1])
    L2_ang(q[2])
    #eff_ang(q[3]) # Para cuando haya un efector

if __name__ == "__main__":
    go_back_n = 15
    # Condiciones iniciales
    q = [90, 90, 90, 90] # Home
    dq = [1, 1, 1, 1] # Velocidades iniciales

    # Esto tiene que ver con la cantidad de lineas que se usan
    for i in range(go_back_n):
        print("")
    refresh_terminal(q,dq, go_back_n)

    while True:
        key=getKey() # Leer entrada de teclado
        if ord(key)==27: # si se apreta ESC
            break

        q, dq = ajustar_parametros(q, dq, key)
        refresh_terminal(q,dq)
        move_robot(q)

    print("FIN :D")
