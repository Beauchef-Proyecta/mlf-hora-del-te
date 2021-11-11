import serial
import time
import matplotlib.pyplot as plt
from matplotlib.pyplot import Slider, Button
from mk2robot import MK2Robot

#BLOQUE SERIAL
ser = serial
try:
    ser = serial.Serial("COM3", 115200, timeout=1) #ojito con la ruta
    #ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
    serial_port = "Open"
    print("The port is available")

except serial.serialutil.SerialException:
    print("The port is at use")
    ser.close()
    ser.open()
#BLOQUE SERIAL

def global_FK(ang, q0=False, q1=False, q2=False, q3=False):
    if q0:
        angleData = str(int(abs(2*ang)))
        ser.write(('&1:' + angleData).encode())
        time.sleep(0.02)
    elif q1:
        angleData = str(int(abs(ang)))
        ser.write(('&2:' + angleData).encode())
        time.sleep(0.02)
    elif q2:
        angleData = str(int(abs(ang)))
        ser.write(('&3:' + angleData).encode())
        time.sleep(0.02)
    elif q3:
        angleData = str(int(abs(ang)))
        ser.write(('&4:' + angleData).encode())
        time.sleep(0.02)


def GO_IK(goal = False):
    x_req=x_req_slider.val
    y_req=y_req_slider.val
    z_req=z_req_slider.val
    sol=robot.inverse_kinematics(x_req,y_req,z_req)
    #print('q0')
    #print(sol[0])
    #print('q1')
    #print(sol[1])
    #print('q2')
    #print(sol[2])
    if goal:
        global_FK(sol[0]+90, q0 = True)
        global_FK(sol[1], q1 = True)
        global_FK(sol[2], q2 = True)
    robot.update_pose(sol[0], sol[1],sol[2])
    [X_pos, Y_pos, Z_pos] = robot.get_joint_positions()
    plot_robot(X_pos, Y_pos, Z_pos)
    fig.canvas.draw_idle()


def update(go=False):
    # This function is called ny time a slider value changes
    if go:
        global_FK(q0_slider.val,q0=True)
        global_FK(q1_slider.val,q1=True)
        global_FK(q2_slider.val + q1_slider.val,q2=True)
    robot.update_pose(q0_slider.val, q1_slider.val, q2_slider.val)
    [X_pos, Y_pos, Z_pos] = robot.get_joint_positions()
    plot_robot(X_pos, Y_pos, Z_pos)
    fig.canvas.draw_idle()
def home(val):
    q0_slider.set_val(0)
    q1_slider.set_val(0)
    q2_slider.set_val(90)
    x_req_slider.reset()
    y_req_slider.reset()
    z_req_slider.reset()
    global_FK(90,q0=True)
    global_FK(90, q1=True)
    global_FK(90, q2=True)

def plot_robot(X_pos, Y_pos, Z_pos):
    # Clear figure
    ax.clear()

    # Plot the data
    ax.scatter(0, 0, 0, zdir='z', s=30)  # Origin
    ax.plot([0, X_pos[0]], [0, Y_pos[0]], [0, Z_pos[0]])  # L0
    ax.plot([X_pos[0], X_pos[1]], [Y_pos[0], Y_pos[1]], [Z_pos[0], Z_pos[1]])  # L1
    ax.plot([X_pos[1], X_pos[2]], [Y_pos[1], Y_pos[2]], [Z_pos[1], Z_pos[2]])  # L2
    ax.plot([X_pos[2], X_pos[3]], [Y_pos[2], Y_pos[3]], [Z_pos[2], Z_pos[3]])  # L3
    ax.scatter(X_pos, Y_pos, Z_pos, zdir='z', s=20)  # Joints

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
    valmin=0,
    valmax=110,
    valinit=90,
)

ax_x_req = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
x_req_slider = Slider(
    ax=ax_x_req,
    label='x [mm]',
    valmin=150,
    valmax=300,
    valinit=213.3,
)

ax_y_req = plt.axes([0.1, 0.3, 0.8, 0.03], facecolor=axcolor)
y_req_slider = Slider(
    ax=ax_y_req,
    label='y [mm]',
    valmin=-160,
    valmax=160,
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
button = plt.Button(ax3, label='update real pose', image=None, color='0.85', hovercolor='0.95')
home_butt = plt.Button(ax4, label='Home', image=None, color='0.85', hovercolor='0.95')
IK_butt = plt.Button(ax_IK, label='Go IK', image=None, color='0.85', hovercolor='0.95')
# Add event handler for every slider
q0_slider.on_changed(update)
q1_slider.on_changed(update)
q2_slider.on_changed(update)
x_req_slider.on_changed(GO_IK)
y_req_slider.on_changed(GO_IK)
z_req_slider.on_changed(GO_IK)
button.on_clicked(lambda x: update(True))
home_butt.on_clicked(home)
IK_butt.on_clicked(lambda x: GO_IK(True))
# Now we are ready to go
plt.show()