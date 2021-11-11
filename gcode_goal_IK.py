import re
import time
import serial
import numpy as np
from mk2robot import MK2Robot


robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot.update_pose(0, 0, 90)
[X_pos, Y_pos, Z_pos] = robot.get_joint_positions()

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

def goal(array):
    arr=array
    x_sol=arr[0,:]
    y_sol=arr[1,:]
    z_sol=arr[2,:]
    for i in range(len(arr[0,:])):
        sol = robot.inverse_kinematics(x_sol[i], y_sol[i], z_sol[i])
        base_ang_ik(90 + sol[0])
        L1_ang_ik(-90 + sol[1])
        L2_ang_ik(180 + sol[1] + sol[2])
        time.sleep(0.4)

with open('test_gcode.gcode') as gcode:
    X_list = np.array([])
    Y_list = np.array([])
    Z_list = np.array([])
    for line in gcode:
        line = line.strip()
        coord = re.findall(r'[XY].?\d+.\d+', line)
        coord_Z=re.findall(r'[YZ].?\d+.\d+', line)
        if coord:
            X_list = np.append(X_list, float(coord[0][1:]))
            Y_list = np.append(Y_list, float(coord[1][1:]))
        if coord_Z:
            Z_list = np.append(Z_list, float(coord_Z[1][1:]))


    goal_XYZ=np.array([X_list,Y_list,Z_list])
    goal(goal_XYZ)


