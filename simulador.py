import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import Slider, Button
from mpl_toolkits.mplot3d import Axes3D #pa que no tire dramas con el '3d'
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

def base_ang(q0):
    time.sleep(0.015)
    angleData = str(int(q0+90))
    ser.write(('&1:' + angleData).encode())
    time.sleep(0.02)

def L1_ang(q1):
    #time.sleep(0.5)
    angleData2 = str(abs(int(q1_slider.val-90)))
    ser.write(('&2:' + angleData2).encode())
    time.sleep(0.03)

def L2_ang(q2):
    # time.sleep(0.5)
    angleData3 = str(abs(int(q2)))
    ser.write(('&3:' + angleData3).encode())
    time.sleep(0.03)

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

def update(val):
    # This function is called ny time a slider value changes
    robot.update_pose(q0_slider.val, q1_slider.val, q2_slider.val)
    [X_pos, Y_pos, Z_pos] = robot.get_joint_positions()
    plot_robot(X_pos, Y_pos, Z_pos)
    fig.canvas.draw_idle()

#Fucnion para actualizar posicion del robot real
def update2(val):
    base_ang(q0_slider.val)
    L1_ang(q1_slider.val)
    L2_ang(q2_slider.val+q1_slider.val)


def update_IK(val):
    x_req=x_req_slider.val
    y_req=y_req_slider.val
    z_req=z_req_slider.val
    sol=robot.inverse_kinematics(x_req,y_req,z_req)
    robot.update_pose(sol[0], sol[1],180+sol[1]+sol[2])
    #q0_slider.set_val(sol[0])
    #q1_slider.set_val(sol[1])
    #q2_slider.set_val(sol[2])
    [X_pos, Y_pos, Z_pos] = robot.get_joint_positions()
    plot_robot(X_pos, Y_pos, Z_pos)
    fig.canvas.draw_idle()

def GO_IK(val):
    x_req=x_req_slider.val
    y_req=y_req_slider.val
    z_req=z_req_slider.val
    sol=robot.inverse_kinematics(x_req,y_req,z_req)
    print('q0')
    print(sol[0])
    print('q1')
    print(sol[1])
    print('q2')
    print(sol[2])
    base_ang_ik(90+sol[0])
    L1_ang_ik(-90+sol[1])
    L2_ang_ik(180+sol[1]+sol[2])
    robot.update_pose(sol[0], sol[1],180+sol[1]+sol[2])
    [X_pos, Y_pos, Z_pos] = robot.get_joint_positions()
    plot_robot(X_pos, Y_pos, Z_pos)
    fig.canvas.draw_idle()

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


def home(val):
    q0_slider.set_val(0)
    q1_slider.set_val(0)
    q2_slider.set_val(90)
    x_req_slider.reset()
    y_req_slider.reset()
    z_req_slider.reset()
    base_ang(0)
    L1_ang(90)
    L2_ang(90)
    time.sleep(0.1)


def plot_robot(X_pos, Y_pos, Z_pos):
    
    # Clear figure
    ax.clear()

    # Plot the data
    ax.scatter(0, 0, 0, zdir='z', s=30)                                     # Origin
    ax.plot([0,X_pos[0]],[0,Y_pos[0]],[0,Z_pos[0]])                         # L0
    ax.plot([X_pos[0],X_pos[1]],[Y_pos[0],Y_pos[1]],[Z_pos[0],Z_pos[1]])    # L1
    ax.plot([X_pos[1],X_pos[2]],[Y_pos[1],Y_pos[2]],[Z_pos[1],Z_pos[2]])    # L2
    ax.plot([X_pos[2],X_pos[3]],[Y_pos[2],Y_pos[3]],[Z_pos[2],Z_pos[3]])    # L3
    ax.scatter(X_pos, Y_pos, Z_pos, zdir='z', s=20)                         # Joints

    # Make it prettier
    ax.set_ylabel('Y [mm]')
    ax.set_xlabel('X [mm]')
    ax.set_zlabel('Z [mm]')

    # Set axis limits
    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(0, 300)



""" 2. The actual script """

# Spawn a robot!
robot = MK2Robot(link_lengths=[55, 39, 135, 147, 66.3])
robot.update_pose(0, 0, 90)
[X_pos, Y_pos, Z_pos] = robot.get_joint_positions()

# Create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the robot for the first time
plot_robot(X_pos, Y_pos, Z_pos)

axcolor = 'lightgoldenrodyellow'
ax.margins(x=0)

# Adjust the main plot to make room for the sliders
plt.subplots_adjust(bottom=0.4)

# Make horizontal sliders
axq0 = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor=axcolor)
q0_slider = Slider(
    ax=axq0,
    label='q0 [º]',
    valmin=-90,
    valmax=90,
    valinit=0,
)

axq1 = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)
q1_slider = Slider(
    ax=axq1,
    label='q1 [º]',
    valmin=-90,
    valmax=30,
    valinit=0,
)

axq2 = plt.axes([0.1, 0.2, 0.8, 0.03], facecolor=axcolor)
q2_slider = Slider(
    ax=axq2,
    label='q2 [º]',
    valmin=40,
    valmax=110,
    valinit=90,
)

ax_x_req = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
x_req_slider = Slider(
    ax=ax_x_req,
    label='x [mm]',
    valmin=160,
    valmax=300,
    valinit=213.3,
)

ax_y_req = plt.axes([0.1, 0.3, 0.8, 0.03], facecolor=axcolor)
y_req_slider = Slider(
    ax=ax_y_req,
    label='y [mm]',
    valmin=-500,
    valmax=500,
    valinit=0,
)

ax_z_req = plt.axes([0.1, 0.35, 0.8, 0.03], facecolor=axcolor)
z_req_slider = Slider(
    ax=ax_z_req,
    label='z [mm]',
    valmin=0,
    valmax=300,
    valinit=229,
)


ax3 = plt.axes([0.3, 0, 0.3, 0.1], facecolor=axcolor)
ax4 = plt.axes([0, 0, 0.3, 0.1], facecolor=axcolor)
ax_IK = plt.axes([0.6, 0, 0.2, 0.1], facecolor=axcolor)
button=plt.Button(ax3,label='update real pose', image=None, color='0.85', hovercolor='0.95')
home_butt=plt.Button(ax4,label='Home', image=None, color='0.85', hovercolor='0.95')
IK_butt=plt.Button(ax_IK,label='Go IK', image=None, color='0.85', hovercolor='0.95')
# Add event handler for every slider
q0_slider.on_changed(update)
q1_slider.on_changed(update)
q2_slider.on_changed(update)
x_req_slider.on_changed(update_IK)
y_req_slider.on_changed(update_IK)
z_req_slider.on_changed(update_IK)

button.on_clicked(update2)
home_butt.on_clicked(home)
IK_butt.on_clicked(GO_IK)
#IK_butt.on_clicked(goal)
# Now we are ready to go
plt.show()



