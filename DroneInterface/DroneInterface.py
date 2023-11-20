from djitellopy import tello
import cv2
from time import sleep
import numpy as np

class Drone:
    def __init__(self):
        self.front_camera = True
        self.landed = True
        
        
        self.drone = tello.Tello()
        self.drone.connect()
        self.set_camera_forward()
        self.drone.streamon()
        #self.drone.reboot() # emergency only
    
    def change_camera(self):
        if self.front_camera:
            try:
                self.drone.send_command_with_return("downvision 1")
            except tello.TelloException:
                print("Not done with previous command")
                return    
            self.front_camera = False
        else:
            try:
                self.drone.send_command_with_return("downvision 0")
            except tello.TelloException:
                return
            self.front_camera = True
            
    def set_camera_down(self):
        try:
            self.drone.send_command_with_return("downvision 1")
        except tello.TelloException:
            print("Not done with previous command")
            return
        self.front_camera = False
        
    def set_camera_forward(self):
        try:
            self.drone.send_command_with_return("downvision 0")
        except tello.TelloException:
            print("Not done with previous command")
            return
        self.front_camera = True
            
    def get_image(self):
        try:
            img = self.drone.get_frame_read().frame
        except tello.TelloException:
            return np.ones((1280,720,3), dtype=np.uint8) if self.front_camera else np.ones((720,960,3), dtype=np.uint8)
        if self.front_camera:
            img = cv2.resize(img, (1280,720))
        return img
    
    def get_battery(self):
        return self.drone.get_battery()
    
    def take_off(self):
        try:
            self.drone.takeoff()
        except tello.TelloException:
            print("Not done with previous command")
            return
        self.landed = False
    
    def land(self):
        try:
            self.drone.land()
        except tello.TelloException:
            print("Not done with previous command")
            return
        self.landed = True
    
    def rotate(self, degrees:int):
        try:
            if degrees < 0:
                self.drone.rotate_counter_clockwise(degrees*-1)
            else:
                self.drone.rotate_clockwise(degrees)
        except tello.TelloException:
            print("Not done with previous command")
            return
    
    def disconnect(self):
        self.drone.end()