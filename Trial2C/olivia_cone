"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################


import sys
import cv2 as cv
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils


########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()


MIN_CONTOUR_AREA = 1000
RED = ((175, 172, 109), (179, 209, 255))  # The HSV range for the color red

BLUE = ((110, 187, 162), (113, 233, 255))
CROP_FLOOR = ((50, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Declare any global variables here
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0
contour_color = ""
last_color = ""

current_state = "APPROACH"
last_angle = 0 

queue = []

########################################################################################
# Functions
########################################################################################
def update_contour():
    
    global contour_center, contour_color, contour_area
    contour = None

    contour_color = ""

    contour_center = None

    contour_area = 0
    image = rc.camera.get_color_image()
    image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])


    if image is None:
        contour_center = None
        contour_area = None
    else:
        red_contours = rc_utils.find_contours(image, RED[0], RED[1])
        red_contour = rc_utils.get_largest_contour(red_contours, MIN_CONTOUR_AREA)

        blue_contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
        blue_contour = rc_utils.get_largest_contour(blue_contours, MIN_CONTOUR_AREA)
        contour = None

        red_area = 0 
        blue_area = 0 
        if red_contour is not None:
            red_area = rc_utils.get_contour_area(red_contour)
        
        if blue_contour is not None:
            blue_area = rc_utils.get_contour_area(blue_contour)

        if red_area > blue_area:
            contour = red_contour
            contour_color = "RED"
        else:
            contour = blue_contour 
            contour_color = "BLUE"
        
        if contour is not None:
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            rc_utils.draw_contour(image, contour)

            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0 
        rc.display.show_color_image(image)







# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed 
    global angle

    # Initialize variables
    speed = 0
    angle = 0


    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)
    
    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5) # Remove 'pass' and write your source code for the start() function here

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed, angle, current_state, last_color, last_angle, contour_center, contour_area, contour_color

    update_contour()
    depth = rc.camera.get_depth_image()
    distance = 0 
    speed = 0.6

    if contour_center is not None:
        distance = depth[contour_center[0], contour_center[1]]


    if current_state == "APPROACH":
        if contour_center is not None:
            setpoint = rc.camera.get_width()//2
            present_value = contour_center[1]
            if contour_color == "RED" and distance<100 and distance != 0:
                present_value += 250
                last_color = "RED"
            elif contour_color == "BLUE" and distance <100 and distance != 0:
                present_value -= 250
                last_color = "BLUE"
            
            kp = -0.006
            error = setpoint - present_value
            angle = kp*error 
            angle = rc_utils.clamp(angle, -1, 1)
            last_angle = angle

        else:
            current_state = "SEARCH"
        if distance < 65 and distance != 0:
            current_state = "TURN"

    elif current_state == "TURN":
        if len(queue) == 0:
            if last_color == "BLUE" and last_angle >-0.3:
                angle = -0.3
            elif last_color == "RED" and last_angle <0.3:
                angle = 0.3
            else:
                angle = last_angle
            queue.append([0.9, 1, angle])
    elif current_state == "SEARCH":
        speed = 0.4
        if last_color == "BLUE":
            angle = 1
        else:
            angle = -1
        if contour_center is not None:
            current_state = "APPROACH"
    if queue:
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            queue.pop(0)
            current_state = "SEARCH"
    

    print(f"{current_state=}, {contour_color=}")

    rc.drive.set_speed_angle(speed, angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
