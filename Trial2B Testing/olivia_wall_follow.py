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

# Declare any global variables here

angle = 0 
speed = 1.0


########################################################################################
# Functions
########################################################################################

def update_lidar():
    global left_distance, right_distance, for_left_distance, for_right_distance, forward_distance, leftside, rightside
    scan = rc.lidar.get_samples()
    forward_distance = scan[0]
    left_distance = scan[945]
    right_distance = scan[135]


    leftside = scan[1040]
    rightside = scan[40]


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    

    rc.drive.set_max_speed(1)

    speed = 1
    angle = 0
    rc.drive.set_speed_angle(speed, angle)

      # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed, angle, left_distance, right_distance, forward_distance, for_left_distance, for_right_distance, leftside, rightside
    update_lidar()
    if (for_right_distance == 0 or right_distance == 0) and leftside < 80:
        print("forced right")
            angle = 1
    elif (for_left_distance == 0 or left_distance == 0) and rightside < 80:
        print("forced left")
        angle = -1
    else:
        error = left_distance - right_distance
    
    
        kp = -0.01435
    
        angle = kp * error
        angle = rc_utils.clamp(angle, -1, 1)
        

    rc.drive.set_speed_angle(speed, angle)
    

    # Remove 'pass' and write your source code for the update() function here


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
