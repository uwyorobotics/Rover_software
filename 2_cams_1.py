#!/usr/bin/env python3
#Script to read two cameras and display them using open cv

#init the video capture

import cv2
import sys
import time

try:
    left_camera = cv2.VideoCapture(0)
except:
    print(f'Couldnt open camera 0')
    exit()

try:
    right_camera = cv2.VideoCapture(1)
except:
    print(f' couldnt open camera 1')
    exit()


if not left_camera.isOpened():
    print('no left camera, exiting')
    exit()

if not right_camera.isOpened():
    print('no right camera, exiting')
    exit()

while True:
    Lret,Lframe = left_camera.read()
    Rret,Rframe = left_camera.read()

    cv2.imshow(Lframe)
    cv2.imshow(Rframe)


    if cv2.waitKey(1) == ord('q'):
        break

Lframe.release()
Rframe.release()
cv2.destroyAllwindows()
