#import module for tello:
from djitellopy import tello

#import module for time:
import time

#import opencv python module:
import cv2

#Global Variable
global img

import pygame
# Initialize Pygame
pygame.init()

# Set the dimensions of the window (adjust as needed)
screen = pygame.display.set_mode((640, 480))


def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 80
    liftSpeed = 80
    moveSpeed = 85
    rotationSpeed = 100

    keys = pygame.key.get_pressed()

    if keys[pygame.K_LEFT]:
        lr = -speed
    elif keys[pygame.K_RIGHT]:
        lr = speed

    if keys[pygame.K_UP]:
        fb = moveSpeed
    elif keys[pygame.K_DOWN]:
        fb = -moveSpeed

    if keys[pygame.K_w]:
        ud = liftSpeed
    elif keys[pygame.K_s]:
        ud = -liftSpeed

    if keys[pygame.K_d]:
        yv = rotationSpeed
    elif keys[pygame.K_a]:
        yv = -rotationSpeed

    if keys[pygame.K_q]:
        Drone.land()
        time.sleep(3)
    elif keys[pygame.K_e]:
        Drone.takeoff()

    if keys[pygame.K_z]:
        # Replace `img` with your actual image source from the drone
        cv2.imwrite(f"tellopy/Resources/Images/{time.time()}.jpg", img)
        time.sleep(0.3)
        
    if keys[pygame.K_m]:
        Drone.send_command_with_return("downvision 0")
    
    if keys[pygame.K_n]:
        Drone.send_command_with_return("downvision 1")

    return [lr, fb, ud, yv]




#Start Connection With Drone
Drone = tello.Tello()
Drone.connect()
#Drone.send_command_with_return("downvision 1")
#Get Battery Info
print(Drone.get_battery())


#Start Camera Display Stream
Drone.streamon()
while True:
    #Get The Return Value And Stored It On Variable:
    keyValues = getKeyboardInput() #Get The Return Value And Stored It On Variable
    #Control The Drone:
    Drone.send_rc_control(keyValues[0],keyValues[1],keyValues[2],keyValues[3]) 
    #Get Frame From Drone Camera Camera 
    img = Drone.get_frame_read().frame
    img = cv2.resize(img, (1080,720))
    #Show The Frame
    cv2.imshow("DroneCapture", img)
    cv2.waitKey(1)