import serial
import time
from mk2robot import MK2Robot
#BLOQUE SERIAL
ser = serial
try:
    ser = serial.Serial("COM5", 115200, timeout=1) #ojito con la ruta
    #ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
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

while True:
    selector= input("ang or XYZ or home? ")

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
        if 0<=q0val<=180:
            base_ang(q0val)
        L1_ang(q1val)
        L2_ang(q2val)
    elif selector=="home":
        base_ang(90)
        L1_ang(90)
        L2_ang(90)