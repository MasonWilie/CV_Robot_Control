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

ft_to_m = 0.305

#desired_x = float(input("Enter the desired x coordinate: "))
#desired_y = float(input("Enter the desired y coordinate: "))

x = 4 * ft_to_m
y = -3 * ft_to_m

nav = classes.navigation()
nav.go_to(x, y)
print("Done..")
