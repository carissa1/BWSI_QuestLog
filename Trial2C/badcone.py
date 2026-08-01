
import sys
import math
from turtle import right

import cv2 as cv
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

rc = racecar_core.create_racecar()
queue = []
# Declare any global variables here


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed, angle, cur_state, pass_timer, seen_abeam

    speed = 0.2
    angle = 0

    rc.drive.set_speed_angle(speed, angle)
    rc.set_update_slow_time(0.5)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  


def update():
    global queue
    scan = rc.lidar.get_samples()
    left_dist = rc_utils.get_lidar_closest_point(scan, -100, -80)
    right_dist = rc_utils.get_lidar_closest_point(scan, 80, 100)

    if len(queue) > 0:
           speed = queue[0][1]
           angle = queue[0][2]
           queue[0][0] -= rc.get_delta_time()
           if queue[0][0] <= 0:
               queue.pop(0)
    if len(queue) == 0 and left_dist < 100:
        green_cone()
    elif len(queue) == 0 and right_dist <100:
        red_cone()
    else:
        speed = 0.3
        angle = 0 

    print(f"Left: {left_dist}, Right: {right_dist}")
    rc.drive.set_speed_angle(speed, angle)
    

def green_cone():
    global queue 
    queue.append([0.6, 0.2, 0])
    queue.append([0.5, 0.2, 1])

def red_cone():
    global queue 
    queue.append([0.6, 0.2, 0])
    queue.append([0.5, 0.2, -1])


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



