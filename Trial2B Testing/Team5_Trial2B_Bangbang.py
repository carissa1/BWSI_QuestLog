"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: This script provides the RACECAR with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the RACECAR should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The RACECAR may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a 
Python file in the RACECAR sim. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
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

# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
queue = []
turning = 0

STATES = {1: "APPROACH", 2: "REVERSE", 3: "RIGHT", 4: "LEFT", 5: "STOP"}
current_state = STATES[1]

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Appends the correct instructions to make a right turn to the queue
def turnRight():
    global queue
    queue.append([0.1, 0.5, 1])

# [FUNCTION] Appends the correct instructions to make a left turn to the queue
def turnLeft():
    global queue
    queue.append([0.1, 0.5, -1])

# [FUNCTION] Appends the correct instructions to go straight to the queue
def goStraight(time, dir):
    global queue
    queue.append([time, 0.5*dir, 0])

# [FUNCTION] Clears the queue to stop all actions
def stopNow():
    global queue
    queue.clear()

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global queue
    queue = []

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab I - Wall Follower\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle"
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

    rc.drive.set_max_speed(1)

    # Get LIDAR
    scan = rc.lidar.get_samples()
    LIDAR_angle_left, LIDAR_dist_left = rc_utils.get_lidar_closest_point(scan, (30, 50))
    LIDAR_angle_right, LIDAR_dist_right = rc_utils.get_lidar_closest_point(scan, (310, 330))
    print("LEFT", LIDAR_angle_left, LIDAR_dist_left)
    print("RIGHT", LIDAR_angle_right, LIDAR_dist_right)

    # if LIDAR_dist_left < 50 or LIDAR_dist_right < 50:
    #     speed = 0.7
    # else:
    speed = 1

    if abs(LIDAR_dist_right - LIDAR_dist_left) < 20:
        print("Forward")
        angle = 0
    else:
        if LIDAR_dist_left < LIDAR_dist_right:
            angle = -0.7
        else:
            angle = 0.7
    
    print("SPEED, ANGLE", speed, angle)

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
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32 
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
