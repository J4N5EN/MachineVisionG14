import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep, time
from pynput import keyboard

from DroneInterface.DroneInterface import Drone
from ImageProcessor.ImageProcessor import ArucoProcessor


################ Initialize the drone
tello = Drone()
################

################ Variables for state management
search = False
running = True
pad_found = False
search_id = 0
down_analysis = False
time_out_tries = 0
################

################ Load calibration data for image processing
calibration_data_forward = np.load('TestingStuff/drone_front.npz')
camera_matrix = calibration_data_forward['camera_matrix']
dist_coeff = calibration_data_forward['dist_coeff']
image_processor = ArucoProcessor(camera_matrix, dist_coeff, HFOV=90, marker_size=0.15)
calibration_data_down = np.load('TestingStuff/drone_down.npz')
camera_matrix_down = calibration_data_down['camera_matrix']
dist_coeff_down = calibration_data_down['dist_coeff']
image_processor_down = ArucoProcessor(camera_matrix_down, dist_coeff_down, 46.6, marker_size=0.15)
################

################ Key mapping and listeners
def on_key_press(key):
    try:
        key = key.char
    except AttributeError:
        if key == keyboard.Key.esc:
            stop_running()
            return False

    if key in key_states:
        key_states[key] = True
    elif key in key_functions:
        key_functions[key]()

def on_key_release(key):
    try:
        key = key.char
    except AttributeError:
        return False if key == keyboard.Key.esc else None

    if key in key_states:
        print(f"Key released: {key}")
        key_states[key] = False

def stop_running():
    global running
    running = False

def set_search_id(id):
    global search_id
    search_id = id
    print("Search id set to:", search_id)

def toggle_search(state):
    global search, pad_found
    search = state
    pad_found = False if state else pad_found
    
def switch_test_mode(state):
    global down_analysis
    down_analysis = state

key_states = {k: False for k in ["w", "a"]}
key_functions = {
    "r": tello.set_camera_down,
    "f": tello.set_camera_forward,
    "z": tello.land,
    "x": tello.take_off,
    "w": lambda: print("Not bound"),
    "a": lambda: print("Not bound"),
    "s": lambda: toggle_search(True),
    "d": lambda: toggle_search(False),
    "q": lambda: tello.rotate(-40),
    "e": lambda: tello.rotate(40),
    "1": lambda: set_search_id(1),
    "2": lambda: set_search_id(2),
    "3": lambda: set_search_id(3),
    "0": lambda: set_search_id(0),
    "y": lambda: switch_test_mode(True),
    "u": lambda: switch_test_mode(False)
}
listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release)
listener.start()
################


# Function to process key inputs
def process_key_inputs():
    for key, pressed in key_states.items():
        if pressed and key in key_functions:
            key_functions[key]()

# Function for searching after the landing pad
def search_for_pad(image):
    global pad_found, last_call_search
    current_time = time()
    if not pad_found and (current_time - last_call_search > 3):
        tello.rotate(40)
        last_call_search = current_time
    
    process_image_and_take_action(image)

# Function to display and process image
def display_and_process_image(image):
    global pad_found, last_call, last_call_search, last_call_down
    cv2.imshow('Tello Live', image)
    
    if cv2.waitKey(1) & 0xFF == 27:  # Check for 'Esc' key
        stop_running()

    """if down_analysis:
        height = tello.drone.get_height()
        if height < 5:
            try:
                tello.drone.go_xyz_speed(0, 0, 20, 10)
            except djitellopy.tello.TelloException:
                pass
            sleep(2)

        for xyz in image_processor_down.get_xyz_to_pads_down(image):
            if time() - last_call_down > 3:
                x, y, z = xyz
                x = int(x*100)
                y = int(y*100)
                print(x,y)
                try:
                    tello.drone.go_xyz_speed(x, y, 0, 10)
                except djitellopy.tello.TelloException:
                    pass
                sleep(0.4)
                last_call_down = time()"""
    
    if search:
        search_for_pad(image)

# Function to process image and take action
def process_image_and_take_action(image):
    global pad_found, last_call, time_out_tries
    pad_info, xyz_to_pads = image_processor.process_image(image)
    for info, xyz in zip(pad_info, xyz_to_pads):
        if info[0] != search_id:
            continue
        pad_found = True
        degrees = info[1]
        print(info)
        current_time = time()
        if current_time - last_call > 0.25:
            last_call = current_time
            tello.rotate(int(degrees))
            time_out_tries += 1
            if -2 < degrees <= 2 or time_out_tries > 30:
                time_out_tries = 0
                go_to_pad(info)

def go_to_pad(info):
    distance = int((((info[2]*100)**2 - (100**2))**0.5)*0.92)
    tello.drone.move_forward(distance)
    sleep(3)
    tello.land()
    sleep(3)
        



# Main loop
last_call = time()
last_call_search = time()
last_call_down = time()
while running:
    process_key_inputs()
    
    image = tello.get_image()
    if image is not None:
        display_and_process_image(image)

cv2.destroyAllWindows()
tello.disconnect()
listener.stop()