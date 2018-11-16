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
import smbus
import struct
import os

bus = smbus.SMBus(1)
address = 0x04

aruco_dict=aruco.Dictionary_get(aruco.DICT_4X4_50)


parameters = aruco.DetectorParameters_create()
parameters.adaptiveThreshWinSizeMin =3
parameters.adaptiveThreshWinSizeMax=23
parameters.adaptiveThreshWinSizeStep=10
parameters.adaptiveThreshConstant=7
parameters.polygonalApproxAccuracyRate=0.05
parameters.minDistanceToBorder=3
parameters.cornerRefinementMethod = 0
parameters.cornerRefinementWinSize=5
parameters.cornerRefinementMaxIterations=30
parameters.cornerRefinementMinAccuracy=0.1
parameters.perspectiveRemovePixelPerCell=8
parameters.perspectiveRemoveIgnoredMarginPerCell=0.13
parameters.maxErroneousBitsInBorderRate=0.04
parameters.minOtsuStdDev=5.0
parameters.errorCorrectionRate=0.6


class navigation:
    
    def __init__(self, filename="settings.txt"):
        self.setup_file = filename
        self.robot = the_robot()
        self.setup(filename)
    
    def setup(self, filename):
        settings = {}
        file = open(filename)
        for line in file:
            split_line = line.split('=', 1)
            set_key = split_line[0]
            set_def = split_line[1]
            settings[set_key] = set_def
        file.close()
            
        camera_res_x = int(settings["camera_res_x"])
        camera_res_y = int(settings["camera_res_y"])
        focal_length_x = float(settings["Focal_length_x_mm"])
        focal_length_y = float(settings["Focal_length_y_mm"])
        field_of_view = float(settings["field_of_view_rad"])
        camera_sens_w = float(settings["camera_sensor_width_mm"])
        camera_sens_h = float(settings["camera_sensor_height_mm"])
        beacon_0_id = int(settings["beacon_0_id"])
        beacon_0_x= float(settings["beacon_0_x"])
        beacon_0_y=float(settings["beacon_0_y"])
        beacon_1_id=int(settings["beacon_1_id"])
        beacon_1_x=float(settings["beacon_1_x"])
        beacon_1_y=float(settings["beacon_1_y"])
        beacon_0_width_mm=int(settings["beacon_0_width_mm"])
        beacon_1_width_mm=int(settings["beacon_1_width_mm"])
        
        self.camera = full_camera(camera_res_x, camera_res_y, focal_length_x, focal_length_y,
                                  camera_sens_w, camera_sens_h, field_of_view)
        self.beacon_0 = beacon(beacon_0_x, beacon_0_y, beacon_0_id, beacon_0_width_mm)
        self.beacon_1 = beacon(beacon_1_x, beacon_1_y, beacon_1_id, beacon_1_width_mm)

    def go_to(self, x, y):
        self.find_current_position()
        self.find_current_angle()
        self.calculate_trajectory()
        


    def find_current_position(self):
        self.robot.start_sweep() # Making the robot spin so you can look for the beacons
        
        self.beacon_0, self.beacon_1 = self.camera.find_beacon_locations(); # Finding the beacons in the picture (don't have to be in the same frame)
        
        self.robot.stop_sweep() # Stopping the robot from turning
        
        self.beacon_0 = self.camera.distance_to_beacon(beacon_0) # Calculating the distance to beacon 0
        self.beacon_1 = self.camera.distance_to_beacon(beacon_1) # Calculating the distance to beacon 1
        
        self.robot.calculate_position(self.beacon_0, self.beacon_1) # Calculating the position of the robot
        
        
    def calculate_position(self):
        dist_between_beacons = math.sqrt( (self.beacon_0.x - self.beacon_1.x)**2 + (self.beacon_0.y - self.beacon_1.y)**2 )
        
        A = law_of_cos(self.beacon_0.distance, self.beacon_1.distance, dist_between_beacons)
        
        if A > (math.pi / 2): # If the angle is obtuse, need to use the right triangle under the line instead
            A = math.pi - A
            offset_y = self.beacon_1.distance * math.cos(A)
            offset_x = self.beacon_1.distance * math.sin(A)
            self.x = self.beacon_1.x - offset_x
            self.y = self.beacon_1.y - offset_y
        else:
            offset_y = self.beacon_1.distance * math.cos(A)
            offset_x = self.beacon_1.distance * math.sin(A)
            self.x = self.beacon_1.x - offset_x
            self.y = self.beacon_1.y + offset_y
            


    def find_current_angle(self):
        if (self.beacon_0.time_found > self.beacon_1.time_found):
            angle_beacon = self.beacon_0
        else:
            angle_beacon = self.beacon_1
        
        beacon_point_on_screen = (angle_beacon.corner_1_x + angle_beacon.corner_2_x) / 2
        distance_to_center_pix = (self.camera.res_x / 2) - beacon_point_on_screen
            
        L = self.camera.res_x / (2 * math.tan(self.camera.fov / 2)) # Intermediate variable for organizational purposes
        angle_ref_to_beacon = math.atan(abs(distance_to_center_pix) / L)
        
        angle_to_beacon = math.atan( (angle_beacon.y - self.robot.y) / (angle_beacon.x - self.robot.x) ) 
        
        if distance_to_center_pix > 0:
            self.robot.angle = angle_to_beacon + angle_ref_to_beacon
        else:
            self.robot.angle = angle_to_beacon - angle_ref_to_beacon
            
    def calculate_trajectory():
        pass
        
class beacon:
    
    def __init__(self, x_pos, y_pos, beacon_id, width=150):
        self.x = x_pos
        self.y = y_pos
        self.id = beacon_id
        self.edge_width_mm = width
        self.found = False
        
        
        self.corners_updated = False
        self.distance = 9999
    
    # Setting the locations of the top right corner (1) and the bottom right corner (2)
    def set_corners(self, corners): 
        self.corners_updated = True
        
        self.corner_1_x = corners[0][1][0]
        self.corner_1_y = corners[0][1][1]
        self.corner_2_x = corners[0][2][0]
        self.corner_2_y = corners[0][2][1]
    
    def found_set(self):
        self.found = True
        self.time_found = time.clock()
        
        
    



class full_camera:
    
    # Initializes the camera with more parameters than the regular pi camera class so that you can use it throughout the program
    def __init__(self, resolution_x, resolution_y, focal_length_x_mm, focal_length_y_mm, cam_sensor_width, cam_sensor_height, cam_FOV):
        self.res_x = resolution_x
        self.res_y = resolution_y
        self.F_x = focal_length_x_mm
        self.F_y = focal_length_y_mm
        self.sensor_width = cam_sensor_width
        self.sensor_height = cam_sensor_height
        self.fov = cam_FOV
        
        try:
            self.pi_camera = PiCamera()
            self.pi_camera.resolution = (self.res_x, self.res_y)
        except:
            print("Failed to connect to the camera... quitting")
        
    # Calculates the distance the aruco beacon is based on the image recieved  
    def distance_to_beacon(self, beacon):
        if not beacon.corners_updated:
            print("Corners were not updated before calculating distance.\n")
            return 9999
        
        edge_length_pix = math.sqrt( (beacon.corner_1_x- beacon.corner_2_x)**2 + (beacon.corner_1_y - beacon.corner_2_y)**2 ) # Calculating the length of the aruco beacon edge in pixels
        
        
        beacon.distance = (beacon.edge_width_mm * self.F_x * self.res_x) / (self.sensor_width * edge_length_pix) # Calculates the distance to the beacon
        
        return beacon



    # Determies the locations of the beacns
    def find_beacon_corners(self, beacon_0, beacon_1):
        
        while not beacon_0.found or not beacon_1.found: # Wait until both beacons are found
            image = picamera.array.PiRGBArray(self.pi_camera)
            self.pi_camera.capture(image, format='rgb')
            image_data = image.array
            gray = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
        
            corners, ids, rejected_image_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
            try:
                len(ids)
            except:
                continue
            
            for i in range(0, len(ids)):
                if ids[i] == beacon_0.id and not beacon_0.found:
                    beacon_0.set_corners(corners[i])
                    beacon_0.found_set()
                elif ids[i] == beacon_1.id and not beacon_1.found:
                    beacon_1.set_corners(corners[i])
                    beacon_1.found_set()
                    
        return beacon_0, beacon_1
        






class the_robot:
    
    def __init__(self):
        self.x = 999
        self.y = 999
        self.angle = 999
        pass
            
    def turn_angle(self, angle_to_turn_rad):
        pass
    
    def start_sweep(self):
        data = [0, 0]
        self.send_data(data)
        print("Robot sweeping...\n")
        return
    
    def stop_sweep(self, data):
        data = [1, 0]
        self.send_data(data)
        print("Stopping robot...\n")
        pass
    
        
    
    def send_data(self):
        sent = False
        while not sent:
            try:
                bus.write_block_data(address, 1, data)
            except OSError as e:
                if e.errno == 121:
                    print("Communication failure... retrying...")
                    continue
            sent = True
        return
    
    
    
def law_of_cos(a, b, c):
    
    A = math.acos((-(a**2) + b**2 + c**2) / (2 * b * c))
    
    return A