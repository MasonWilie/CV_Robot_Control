import cv2
import cv2.aruco as aruco
from picamera import PiCamera
import picamera.array
import numpy as np
import time
import os.path
import math
import smbus
import struct
import classes

#desired_x = input("Enter the desired x coordinate: ")
#desired_y = input("Enter the desired y coordinate: ")

x = 0.762
y = 0.914

nav = classes.navigation()
nav.go_to(x, y)
print("Done..")
