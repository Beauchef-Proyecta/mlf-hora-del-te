from finder_IK import find_q0q1q2
import time
import serial

# BLOQUE SERIAL
ser = serial
try:
    # ser = serial.Serial("COM5", 115200, timeout=1) #ojito con la ruta
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
    serial_port = "Open"
    print("The port is available")

except serial.serialutil.SerialException:
    print("The port is at use")
    ser.close()
    ser.open()


# BLOQUE SERIAL


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


while True:
    selector = input("ang or XYZ or home? ")

    if selector == "XYZ":
        Xval = int(input("X: "))
        Yval = int(input("Y: "))
        Zval = int(input("Z: "))
        sol = find_q0q1q2(Xval, Yval, Zval)
        global_FK(90 + sol[0], q0=True)
        global_FK(sol[1], q1=True)
        global_FK(180 + sol[1] + sol[2], q2=True)
        time.sleep(0.4)

    elif selector == "ang":
        q0val = int(input("q0: "))
        q1val = int(input("q1: "))
        q2val = int(input("q2: "))
        global_FK(q0val, q0=True)
        global_FK(q1val, q1=True)
        global_FK(q2val, q2=True)
