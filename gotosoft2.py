import sys

sys.path.append('/home/pi/mlf/core')
from serial_control import SerialControl
from mk2robot import MK2Robot
# from core.serial_control import SerialControl #for pc
# from core.mk2robot import MK2Robot #for pc
import time

robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot_serial = SerialControl()
# robot_serial = SerialControl("COM5") #for pc
robot_serial.open_serial()

def gosoft(current_pose, goal):
    # Ajustar estos params en fn del tiempo
    n_pasos = 3
    a = 0.5
    sleep_time = 0.5

     for _ in range(n_pasos):
        for i in range(3):
            #print(goal[i])
            #print(current_pose[i])
            current_pose[i] = a*goal[i] + (1-a)*current_pose[i]
        robot_serial.write_servo(1, int(current_pose[0]))
        robot_serial.write_servo(2, int(current_pose[1]))
        robot_serial.write_servo(3, int(current_pose[2]))
        print(int(current_pose[0]), int(current_pose[1]), int(current_pose[2]))
        time.sleep(sleep_time)
    print("final: ", current_pose)
    return current_pose


current_pose = [45, 90, 90]
try:
    while True:
        selector = input("ang or XYZ or home? ")

        if selector == "XYZ":
            Xval = int(input("X: "))
            Yval = int(input("Y: "))
            Zval = int(input("Z: "))
            q0, q1, q2 = robot.inverse_kinematics(Xval, Yval, Zval)
            robot_serial.write_servo(1, 45 + q0)
            robot_serial.write_servo(2, 90 - q1)
            robot_serial.write_servo(3, q2 + q1)
            print(f"q0 = {q0}")
            print(f"q1 = {90-q1}")
            print(f"q2 = {q1+q2}")
            time.sleep(0.4)

        elif selector == "ang":
            q0val = int(input("q0: "))
            q1val = int(input("q1: "))
            q2val = int(input("q2: "))
            current_pose = gosoft(current_pose, [q0val, q1val, q2val])
            #robot_serial.write_servo(1, q0val)
            #robot_serial.write_servo(2, q1val)
            #robot_serial.write_servo(3, q2val)
        elif selector == "home":
            robot_serial.write_servo(1, 45)
            robot_serial.write_servo(2, 90)
            robot_serial.write_servo(3, 90)
        else:
            pass


except KeyboardInterrupt:
    robot_serial.close_serial()
    pass