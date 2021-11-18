import sys

sys.path.append('/home/pi/mlf/core')
from serial_control import SerialControl
from mk2robot import MK2Robot
# from core.serial_control import SerialControl #for pc
# from core.mk2robot import MK2Robot #for pc
import time
import numpy as np

robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot_serial = SerialControl()
# robot_serial = SerialControl("COM5") #for pc
robot_serial.open_serial()

def gosoft(current_pose, goal, n_pasos):
    # Ajustar estos params en fn del tiempo
    sleep_time = 1
    sleep_small_time = 0.5
    goal[0] = min([max([goal[0], 0]), 90])
    goal[1] = min([max([goal[1], 40]), 150])
    goal[2] = min([max([goal[2], 60]), 120])
    print("Goal:", goal)

    for i in range(n_pasos):
        for j in range(3):
            target = int((current_pose[j]+goal[j])/2 +((current_pose[j]-goal[j])/2)*np.cos((i+1)*np.pi/n_pasos))
            robot_serial.write_servo(j+1, target)
            time.sleep(sleep_small_time)

        #time.sleep(sleep_time)
    print("final: ", current_pose)
    return goal


current_pose = [45, 90, 90]
try:
    while True:
        q0val = int(input("q0: "))
        q1val = int(input("q1: "))
        q2val = int(input("q2: "))
        n_steps = int(input("n: "))
        current_pose = gosoft(current_pose, [q0val, q1val, q2val], n_steps)


except KeyboardInterrupt:
    robot_serial.close_serial()
    pass
