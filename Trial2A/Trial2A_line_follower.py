"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_f.py

Title: Lab F - Line Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR moves forward at full speed
- When the left trigger is pressed, the RACECAR, moves backwards at full speed
- The angle of the RACECAR should only be controlled by the center of the line contour
- The RACECAR sees the color RED as the highest priority, then GREEN, then BLUE
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import csv

import yaml

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
from PIL import Image

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car (480x640)
# CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR = ((310, 0), (rc.camera.get_height() - 45, rc.camera.get_width()))
CROP_MID = ((260, 0), (310, rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
# BLUE = ((90, 100, 80), (140, 230, 240))  # The HSV range for the color blue
BLUE = ((90, 100, 100), (120, 255, 255))
GREEN = ((30, 100, 100), (80, 255, 255))  # The HSV range for the color green
RED = ((165, 50, 50), (10, 255, 255))  # The HSV range for the color red

# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (RED, GREEN, BLUE)

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
indx = 0
last_error = 0
error = 0

with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)
    BLUE = (tuple(config['Camera']['BLUE_lower']), tuple(config['Camera']['BLUE_upper']))
    kp = config['PID']['kp']
    kd = config['PID']['kd']
    OFFSET = config['Camera']['OFFSET']

# OFFSET = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour(save = 'False'):
    global indx

    image = rc.camera.get_color_image()
    image_lower = image.copy()
    image_upper = image.copy()

    if image_lower is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image_lower = rc_utils.crop(image_lower, CROP_FLOOR[0], CROP_FLOOR[1])
        image_upper = rc_utils.crop(image_upper, CROP_MID[0], CROP_MID[1])

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        max_contour_lower = []
        max_contour_upper = []
        # contours_list = []
        # for color in COLOR_PRIORITY:
        contours_upper = rc_utils.find_contours(image_upper, BLUE[0], BLUE[1])
        contours_lower = rc_utils.find_contours(image_lower, BLUE[0], BLUE[1])
        # contours_upper = rc_utils.find_contours(image_upper, color[0], color[1])
        # contours_lower = rc_utils.find_contours(image_lower, color[0], color[1])
        # contours_list.extend(contours)
        for contour in contours_lower:
            if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                if len(max_contour_lower) == 0:
                    max_contour_lower = contour
                elif cv.contourArea(contour) > cv.contourArea(max_contour_lower):
                    max_contour_lower = contour
            # if len(max_contour_lower) > 0:
            #     break
        for contour in contours_upper:
            if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                if len(max_contour_upper) == 0:
                    max_contour_upper = contour
                elif cv.contourArea(contour) > cv.contourArea(max_contour_upper):
                    max_contour_upper = contour
            # if len(max_contour_upper) > 0:
            #     break

            # contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
            # contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        contour_center_lower = None
        if len(max_contour_lower) > 0:
            contour_center_lower = rc_utils.get_contour_center(max_contour_lower)
            contour_center_lower = (contour_center_lower[0] + CROP_MID[0][0], contour_center_lower[1] + OFFSET)
            if contour_center_lower[1] < 1:
                contour_center_lower = (contour_center_lower[0], 0)
            contour_area = rc_utils.get_contour_area(max_contour_lower)

            rc_utils.draw_contour(image, max_contour_lower, color=(255, 0, 0))
            rc_utils.draw_circle(image, contour_center_lower, color=(150, 0, 150))
          

        contour_center_upper = None
        if len(max_contour_upper) > 0:
            contour_center_upper = rc_utils.get_contour_center(max_contour_upper)
            contour_center_upper = (contour_center_upper[0] + CROP_FLOOR[0][0], contour_center_upper[1] + OFFSET)
            if contour_center_upper[1] < 1:
                contour_center_upper = (contour_center_upper[0], 0)
            contour_area = rc_utils.get_contour_area(max_contour_upper)

            rc_utils.draw_contour(image, max_contour_upper, color=(0, 255, 0))
            rc_utils.draw_circle(image, contour_center_upper, color=(150, 150, 0))

        # Display the image_lower to the screen
        # rc.display.show_color_image(image)

        if save:
            # cv.imwrite('photo_upper_' + str(indx) + '.png', image_upper)
            # cv.imwrite('photo_lower_' + str(indx) + '.png', image_lower)
            cv.imwrite('photo_' + str(indx) + '.png', image_lower)

        indx += 1

        return contour_center_lower, contour_center_upper

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
    rc.set_update_slow_time(0.5)

    data = ['Speed', 'Angle', 'Error']

    with open('log.csv', mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)

    f.close()

    # Print start message
    print(
        ">> Trial 2A - line following\n"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global last_error
    global error
    global kp
    global kd
    
    rc.drive.set_max_speed(1)

    # Search for contours in the current color image
    contour_center_lower, contour_center_upper = update_contour(False)

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.


    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    # print(contour_center_lower, contour_center_upper)
    if contour_center_lower is not None and contour_center_upper is not None:
        contour_center = (contour_center_lower[0] * 0.65 + contour_center_upper[0] * 0.35, contour_center_lower[1] * 0.6 + contour_center_upper[1] * 0.4)
        setpoint = rc.camera.get_width() // 2
        
        present_value = contour_center[1]
        error = setpoint - present_value
        angle = kp * error + kd * (error - last_error)/rc.get_delta_time()
        angle = rc_utils.clamp(angle, -1, 1)
        last_error = error
    else:
        angle = 0
    speed = 1
        
    if rc.controller.was_pressed(rc.controller.Button.B):
        kp += 0.0005
    if rc.controller.was_pressed(rc.controller.Button.A):
        kp -= 0.0005
    if rc.controller.was_pressed(rc.controller.Button.Y):
        kd += 0.00002
    if rc.controller.was_pressed(rc.controller.Button.X):
        kd -= 0.00002
    
    # rc.drive.set_max_speed(0.2)
        # print(angle_factor)

    # Use the triggers to control the car's speed
    # rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # if rt > 0.2:
    #    speed = rt

    rc.drive.set_speed_angle(speed, angle)

    data = [speed, angle, error]

    with open('log.csv', mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)

    # print("SPEED: ", speed)
    # print("ANGLE: ", angle)
    # print("ERROR: ", error)

    # Print the center and area of the largest contour when B is held down
    # if rc.controller.is_down(rc.controller.Button.B):
    #     if contour_center is None:
    #         print("No contour found")
    #     else:
    #         print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    # if rc.camera.get_color_image() is None:
    #     # If no image is found, print all X's and don't display an image
    #     print("X" * 10 + " (No image) " + "X" * 10)
    # else:
    #     # If an image is found but no contour is found, print all dashes
    #     if contour_center is None:
    #         print("-" * 32 + " : area = " + str(contour_area))

    #     # Otherwise, print a line of dashes with a | indicating the contour x-position
    #     else:
    #         s = ["-"] * 32
    #         s[int(contour_center[1] / 20)] = "|"
    #         print("".join(s) + " : area = " + str(contour_area))

    update_contour(True)

    # print("SPEED: ", speed)
    # print("ANGLE: ", angle)
    # print("ERROR: ", error)

    # data = [speed, angle, error]

    # with open('log.csv', mode='a', newline='', encoding='utf-8') as f:
    #     writer = csv.writer(f)
    #     writer.writerow(data)

    # f.close()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
