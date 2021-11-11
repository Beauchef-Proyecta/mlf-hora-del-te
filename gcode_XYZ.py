import re
import numpy as np

def goal(array):
    print("x",array[0,:])
    print("y",array[1, :])
    print("z",array[2, :])

with open('test_gcode.gcode') as gcode:
    X_list = np.array([])
    Y_list = np.array([])
    Z_list = np.array([])
    for line in gcode:
        line = line.strip()
        coord = re.findall(r'[XY].?\d+.\d+', line) #falta ver como hacerlo con XYZ de una
        coord_Z=re.findall(r'[YZ].?\d+.\d+', line)
        if coord:
            X_list = np.append(X_list, float(coord[0][1:]))
            Y_list = np.append(Y_list, float(coord[1][1:]))
        if coord_Z:
            Z_list = np.append(Z_list, float(coord_Z[1][1:]))


    goal_XYZ=np.array([X_list,Y_list,Z_list])
    goal(goal_XYZ) #Do something with positions array


