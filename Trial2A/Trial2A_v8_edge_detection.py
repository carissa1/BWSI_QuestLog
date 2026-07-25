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
from matplotlib import pyplot as plt
import numpy as np
import csv
from scipy.interpolate import splprep, splev
import yaml

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
MIN_CONTOUR_AREA = 100

# A crop window for the floor directly in front of the car
# 480 x 320
# CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR = ((230, 0), (rc.camera.get_height() - 45, rc.camera.get_width())) 
# CROP_CEILING = ((0, 0), (100, rc.camera.get_width()))

CROP_HEIGHT = rc.camera.get_height() // 6
CROP_WIDTH = rc.camera.get_width() // 1
ROWS = rc.camera.get_height() // CROP_HEIGHT
COLS = rc.camera.get_width() // CROP_WIDTH

LOOK_AHEAD = 100

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

filtered_error = 0
ALPHA = 1 # 0.3

last_angle = 0.0
MAX_ANGLE_DELTA = 0.4

with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)
    BLUE = (tuple(config['Camera']['BLUE_lower']), tuple(config['Camera']['BLUE_upper']))
    kp = config['PID']['kp']
    kd = config['PID']['kd']
    kp_far = config['PID']['kp_far']
    OFFSET = config['Camera']['OFFSET']

# OFFSET = 0

########################################################################################
# Functions
########################################################################################


def update_contour():
    img = rc.camera.get_color_image()
    # img = cv.imread('Straight.png')
    img = rc_utils.crop(img, CROP_FLOOR[0], CROP_FLOOR[1])
    # cv.imwrite('Straight_out.png')
    # img = rc_utils.crop(img, img.)
    # TODO: try different exposures
    # img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    img_fixed = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    # cv.imwrite('Trial2C_Straight.png', img_fixed)
    blurred = cv.GaussianBlur(img_fixed, (5, 5), 1)
    # cv.imwrite('Trial2C_blurred.png', blurred)
    # blue_mask = cv.inRange(blurred, (75, 130, 162), (95, 139, 169)) # real
    blue_mask = cv.inRange(blurred, (0, 115, 162), (95, 139, 255)) # simulation
    # cv.imwrite('Trial2C_straight_Blue_mask.png', blue_mask)
    masked = cv.bitwise_and(img_fixed, img_fixed, mask=blue_mask)
    cv.imwrite('Trial2C_straight_Blue.png', masked)
    # gray = cv.cvtColor(blue_mask, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(masked,100,200)
    # plt.subplot(121),plt.imshow(blue_mask,cmap = 'gray')
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    # plt.savefig('Trial2C_edges.png')
    # cv.imwrite('Trial2C_edges.png', edges)

    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # get all outer contours (only edges)
    cv.drawContours(img, contours, -1, (0, 255, 0), 2)
    # cv.imwrite('Trial2C_contours.png', img)

    largest_contour_center = None
    if len(contours) > 0:
        largest_contour = max(contours, key=cv.contourArea)

        # Get thin line in the middle of the contours
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv.drawContours(mask, largest_contour, -1, 255, -1)
        masked_gray = cv.cvtColor(masked, cv.COLOR_BGR2GRAY)
        thin = cv.ximgproc.thinning(masked_gray, thinningType=cv.ximgproc.THINNING_GUOHALL)
        cv.imwrite('Trial2C_thin.png', thin)

        # Find middle line contour
        middle_contours = cv.findContours(thin, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        middle_contours = middle_contours[0] if len(middle_contours) == 2 else middle_contours[1]
        middle_contour = max(middle_contours, key=cv.contourArea)

        # Get intersection with look ahead distance
        circle_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        h, w = img.shape[:2]
        center_coords = (w // 2, h)
        cv.circle(circle_mask, center_coords, LOOK_AHEAD, 255, 1)
        line_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv.drawContours(line_mask, middle_contour, -1, 255, -1)
        cv.imwrite('Trial2C_circle.png', circle_mask)
        cv.imwrite('Trial2C_line.png', line_mask)
        intersection_mask = cv.bitwise_and(circle_mask, line_mask)
        intersection_points = np.argwhere(intersection_mask == 255)

        rc_utils.draw_contour(img, middle_contour, color=(0, 255, 0))

        if len(intersection_points) != 0:
            target_point = (intersection_points[0][0], max(0, intersection_points[0][1] + OFFSET))
            rc_utils.draw_circle(img, target_point, color=(150, 150, 0))
        else:
            target_point = rc_utils.get_contour_center(largest_contour)
            target_point = (target_point[0], max(0, target_point[1] + OFFSET))
            rc_utils.draw_circle(img, target_point, color=(255, 0, 0))

        print(target_point)
    else:
        target_point = None

    rc.display.show_color_image(img)

    # largest_contour = (0,0)

    return target_point

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every 5 seconds
    rc.set_update_slow_time(1)

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
    global last_angle
    global filtered_error
    
    rc.drive.set_max_speed(0.8)

    # Search for contours in the current color image
    contour_center = update_contour()

    # Get contour center
    # NEAR_WEIGHT = 0.6
    # FAR_WEIGHT = 0.4
    setpoint = rc.camera.get_width() // 2

    if contour_center is not None:
        raw_error = setpoint - contour_center[1]
    # elif contour_center_upper is not None:
    #     raw_error = setpoint - contour_center_upper[1]
    else:
        raw_error = last_error
    
    # Low-pass filter
    # filtered_error = ALPHA * raw_error + (1 - ALPHA) * filtered_error
    # angle = kp * filtered_error + kd * (filtered_error - last_error) / rc.get_delta_time()
    angle = kp * raw_error

    # angle = rc_utils.clamp(angle, -0.7, 0.7)
    angle = rc_utils.clamp(angle, -1, 1)

    # Slew-rate limit: cap how much angle can change in one frame
    # angle = rc_utils.clamp(angle, last_angle - MAX_ANGLE_DELTA, last_angle + MAX_ANGLE_DELTA)
    # last_angle = angle

    # last_error = filtered_error
    # error = filtered_error

    speed = 0.3
        
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
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    if rt > 0.2:
        angle = 0.6
    if lt > 0.2:
        angle = -0.6
    #    speed = rt

    rc.drive.set_speed_angle(speed, angle)

    data = [speed, angle, raw_error]

    with open('log.csv', mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)

    print("SPEED: ", speed)
    print("ANGLE: ", angle)
    print("ERROR: ", raw_error)

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

    # update_contour(True)
    # Canny_Test()

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
