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

#from hora_del_te_fns import gosoft
def gosoft(current_pose, goal, n_pasos, on_raspi, use_IK):
    # Ajustar estos params en fn del tiempo
    if use_IK:
        goal[0] = min([max([goal[0], 0]), 90])   # Definir límites
        goal[1] = min([max([goal[1], 40]), 150]) # Definir límites
        goal[2] = min([max([goal[2], 60]), 120]) # Definir límites
        goal[3] = min([max([goal[3], 60]), 120]) # Definir límites


    else:
        goal[0] = min([max([goal[0], 0]), 90])
        goal[1] = min([max([goal[1], 40]), 150])
        goal[2] = min([max([goal[2], 60]), 120])
        goal[3] = min([max([goal[3], 60]), 120]) # Definir límites
    #print("Goal:", goal)

    targets1 = [current_pose[0]]
    targets2 = [current_pose[1]]
    targets3 = [current_pose[2]]
    targets4 = [current_pose[3]]


    for i in range(n_pasos):
        target1 = int((current_pose[0]+goal[0])/2 +((current_pose[0]-goal[0])/2)*np.cos((i+1)*np.pi/n_pasos))
        target2 = int((current_pose[1]+goal[1])/2 +((current_pose[1]-goal[1])/2)*np.cos((i+1)*np.pi/n_pasos))
        target3 = int((current_pose[2]+goal[2])/2 +((current_pose[2]-goal[2])/2)*np.cos((i+1)*np.pi/n_pasos))
        target4 = int((current_pose[3]+goal[3])/2 +((current_pose[3]-goal[3])/2)*np.cos((i+1)*np.pi/n_pasos))
        if use_IK:
            (target1, target2, target3) = robot.inverse_kinematics(target1, target2, target3)

        targets1.append(target1)
        targets2.append(target2)
        targets3.append(target3)
        targets4.append(target4)
        if on_raspi:
            #sleep_time = 0.5
            robot_serial.write_servo(1, target1)
            #time.sleep(sleep_time)
            robot_serial.write_servo(2, target2)
            #time.sleep(sleep_time)
            robot_serial.write_servo(3, target3)
            #time.sleep(sleep_time)
            robot_serial.write_servo(4, target4)
            time.sleep(1.1)
        else:
            time.sleep(0.05)

    return goal, (targets1, targets2, targets3, targets4)



current_pose = [45, 90, 90, 90] # Posicion inicial
pos_lst = []
pos_lst.append([20, 90, 90, 90])
pos_lst.append([70, 90, 90, 90])
pos_lst.append([45, 60, 90, 90])
pos_lst.append([45, 100, 90, 90])
pos_lst.append([45, 90, 70, 90])
pos_lst.append([45, 90, 100, 90])
pos_lst.append([45, 80, 90, 90])
pos_lst.append([45, 90, 90, 90])

n_steps = 1
try:
    for pos in pos_lst:
        print(current_pose, " ->  ", end='')
        current_pose, targets = gosoft(current_pose, pos, n_steps, on_raspi, use_IK)
        print(current_pose)
        time.sleep(2)

        #print(targets)




except KeyboardInterrupt:
    robot_serial.close_serial()
    pass
if on_raspi:
    robot_serial.close_serial()
