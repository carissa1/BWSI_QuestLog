"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_g.py

Title: Lab H - Cone Slalom

Author: [PLACEHOLDER] << [Write your name or team name here]

"""

########################################################################################
# Imports
########################################################################################

import math
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

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 2000

# TODO Part 1: Determine the HSV color threshold pairs for RED and BLUE
RED_LOWER = ('Red', (0, 29, 29), (12, 255, 255)) # 50, 50), (5
RED_UPPER = ('Red', (170, 50, 50), (179, 255, 255))
GREEN = ('Green', (28, 100, 28), (90, 255, 255)) 
COLOR_LST = [RED_LOWER, GREEN]

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
queue = []
first_turn = True
turning = 0
two_cones = False
two_cones_time = 0

STATES = {1: "APPROACH", 2: "REVERSE", 3: "RIGHT", 4: "LEFT", 5: "STOP", 6: "CONE_RIGHT", 7: "CONE_LEFT"}
current_state = STATES[1]

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        max_contour = None
        cone_color = None

        for color in COLOR_LST:
            contours = rc_utils.find_contours(image, color[1], color[2])
            for contour in contours:
                area = cv.contourArea(contour)
                if area > MIN_CONTOUR_AREA:
                    if max_contour is None or area > cv.contourArea(max_contour):
                        max_contour = contour
                        cone_color = color[0]
        # rc.display.show_color_image(image)

        if max_contour is not None:
            max_contour_center = rc_utils.get_contour_center(max_contour)
            cv.circle(image, (max_contour_center[1], max_contour_center[0]), 6, (0, 0, 0), -1)
            rc_utils.draw_contour(image, max_contour)
            # rc.display.show_color_image(image)
            return max_contour_center, cone_color

        return None, None

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global queue
    global angle_dir
    queue = []
    angle_dir = 0

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab H - Cone Slalom\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global current_state
    global queue
    global angle_dir
    global turning
    global two_cones
    global two_cones_time

    # Search for contours in the current color image
    contour_center, cone_color = update_contour()

    rc.drive.set_max_speed(1)

    # Get LIDAR
    scan = rc.lidar.get_samples()
    LIDAR_angle, LIDAR_dist = rc_utils.get_lidar_closest_point(scan, (270, 90)) # 270, 90
    back_angle, back_distance = rc_utils.get_lidar_closest_point(scan, (80, 290))
    print(LIDAR_angle, LIDAR_dist)
    print("BACK DIST: ", back_distance, back_angle)

    if cone_color == 'Red':
        angle_dir = 1
    elif cone_color == 'Green':
        angle_dir = -1

    print("ANGLE: ", angle_dir)
    print("TURNING: ", turning)
    SPEED = 1
    BACK_ANGLE_DIST = 85
    FRONT_DIST = 92
    # cones are like 180 apart
    # CONE IN FRONT
    if abs(LIDAR_angle - 0) < 30 or abs(LIDAR_angle - 360) < 30:        # relatively straight towards cone
        if abs(LIDAR_angle - 0) < 10 or abs(LIDAR_angle - 360) < 10:    # extremely straight towards cone
            turning = 0                                                     # reset turning bc recentered
            if LIDAR_dist > FRONT_DIST:                                      # far away --> approach cone   
                speed = SPEED
                angle = 0
            else:                                                           # close to cone --> go around cone
                speed = SPEED
                angle = angle_dir 
        elif turning != 0:                                              # already turning --> continue turning
            speed = SPEED
            angle = turning
        else:                                                           # not extremely straight
            if LIDAR_dist > FRONT_DIST:                                      # not extremely straight but far away --> keep straight
                speed = SPEED
                angle = 0
            else:                                                           # closer --> go around cone
                speed = SPEED
                angle = angle_dir
    # CONE BEHIND
    elif back_distance < BACK_ANGLE_DIST and back_angle > 220:          # cone behind and left --> turn left to recenter
        print("BEHIND LEFT")
        turning = -0.8
        speed = SPEED
        angle = turning
    elif back_distance < BACK_ANGLE_DIST and back_angle < 160:          # cone behind and right --> turn right to recenter
        print("BEHIND RIGHT")
        turning = 0.8
        speed = SPEED
        angle = turning
    # CONTINUING TO TURN
    elif turning != 0:                                                  # still turning, not recentered yet
        speed = SPEED
        angle = turning
    else:                                                               # otherwise just go straight but slow
        speed = 1
        angle = 0
    
    if LIDAR_dist > 900:                                                # go straight but slow if walls too far or doesn't pick up any walls
        speed = 1
        angle = 0
    
    print("SPEED, ANGLE", speed, angle)
    print("COLOR: ", cone_color)

    # Set the speed and angle of the RACECAR after calculations have been complete
    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # # Print a line of ascii text denoting the contour area and x-position
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


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
