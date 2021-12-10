import sys
import time
import numpy as np
# Objetivos:
# Usar cinemática inversa
# Agregar Gripper


on_raspi = False
use_IK = False

if on_raspi:
    sys.path.append('/home/pi/mlf/core')
    from serial_control import SerialControl
    from mk2robot import MK2Robot
    # from core.serial_control import SerialControl #for pc
    # from core.mk2robot import MK2Robot #for pc

    robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
    robot_serial = SerialControl()
    # robot_serial = SerialControl("COM5") #for pc
    robot_serial.open_serial()
else:
    import matplotlib.pyplot as plt

from gotosoft_fn import gosoft

current_pose = [45, 90, 90, 90] # Posicion inicial
pos_lst = []
pos_lst.append([45, 95, 90, 90])
pos_lst.append([50, 85, 90, 90])
pos_lst.append([55, 80, 90, 90])
pos_lst.append([60, 75, 90, 90])
pos_lst.append([65, 70, 90, 90])
pos_lst.append([70, 65, 90, 90])
n_steps = 1
try:
    for pos in pos_lst:
        print(current_pose, " ->  ", end='')
        current_pose, targets = gosoft(current_pose, pos, n_steps, on_raspi, use_IK)
        time.sleep(2)
        print(current_pose)
        #print(targets)




except KeyboardInterrupt:
    robot_serial.close_serial()
    pass
