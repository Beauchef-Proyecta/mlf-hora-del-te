import cv2
import numpy as np

cap = cv2.VideoCapture(2)
cap.open(0, cv2.CAP_DSHOW)

kernel_size = (2,2)
while(True):
    ret, frame = cap.read()
    frame = cv2.blur(frame, kernel_size)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('foto.png', frame)
        break

cap.release()
cv2.destroyAllWindows()
