import sys
import time
import numpy as np
# Objetivos:
# Usar cinemática inversa
# Agregar Gripper


on_raspi = False
use_IK = True

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
    print("Goal:", goal)

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


current_pose = [45, 90, 90, 90]
try:
    while True:
        if use_IK:
            q0val = int(input("X: "))
            q1val = int(input("Y: "))
            q2val = int(input("Z: "))
            q3val = int(input("Eff: "))
            n_steps = int(input("n: "))
        else:
            q0val = int(input("q0: "))
            q1val = int(input("q1: "))
            q2val = int(input("q2: "))
            q3val = int(input("Eff: "))
            n_steps = int(input("n: "))
        current_pose, targets = gosoft(current_pose, [q0val, q1val, q2val, q3val], n_steps, on_raspi, use_IK)
        if not on_raspi:
            for i in range(4):
                plt.figure()
                plt.plot(np.arange(len(targets[i])), targets[i])
                plt.title("Q%i"%i)
                plt.savefig("Q%i"%i)
            break


except KeyboardInterrupt:
    robot_serial.close_serial()
    pass
