import sys
import time
import numpy as np
# Objetivos:
# Usar cinematica inversa
# Agregar Gripper


on_raspi = True
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

from hora_del_te_fns import gosoft

current_pose = [45, 90, 90, 90] # Posicion inicial
pos_lst = []
pos_lst.append([40, 90, 90, 90])
pos_lst.append([50, 90, 90, 90])
pos_lst.append([50, 80, 90, 90])
pos_lst.append([50, 90, 90, 90])
pos_lst.append([50, 90, 80, 90])
pos_lst.append([50, 90, 90, 90])
pos_lst.append([50, 80, 90, 90])
pos_lst.append([45, 90, 90, 90])

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
if on_raspi:
    robot_serial.close_serial()
