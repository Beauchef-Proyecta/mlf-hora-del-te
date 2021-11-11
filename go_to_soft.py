import serial
import time
import sys

sys.path.append('/home/pi/mlf/core')
from serial_control import SerialControl
from mk2robot import MK2Robot
# from core.serial_control import SerialControl #for pc
# from core.mk2robot import MK2Robot #for pc

#BLOQUE SERIAL
ser = serial
try:
    #ser = serial.Serial("COM5", 115200, timeout=1) #ojito con la ruta
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
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

def move_robot(pos0, goal, time=1):
    # Ajustar estos params en fn del tiempo
    n_pasos = 10
    a = 0.95
    sleep_time = 0.2

    # Posiciones intermedias
    pos = [pos_i for pos_i in pos0]

    # Ajustar posiciones de forma suave
    for _ in range(n_pasos):
        for i in range(3):
            pos[i] = a*goal[i] + (1-a)*pos[i]
            time.sleep(sleep_time)
        base_ang(pos[0])
        L1_ang(pos[1])
        L2_ang(pos[2])

while True:
    selector= input("ang or home? ")

    if selector=="XYZ":
        Xval = int(input("X: "))
        Yval = int(input("Y: "))
        Zval = int(input("Z: "))
        sol = robot.inverse_kinematics(Xval,Yval,Zval)
        base_ang_ik(90 + sol[0])
        L1_ang_ik(-90 + sol[1])
        L2_ang_ik(180 + sol[1] + sol[2])
        time.sleep(0.4)

    elif selector=="ang":
        q0val = int(input("q0: "))
        q1val = int(input("q1: "))
        q2val = int(input("q2: "))
        move_robot([q0val, q1val, q2val])

    elif selector=="home":
        base_ang(90)
        L1_ang(90)
        L2_ang(90)
