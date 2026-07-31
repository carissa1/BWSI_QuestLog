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

import math
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

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 1000
LOOK_AHEAD = 350 

# A crop window for the floor directly in front of the car
# CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))
CROP_FLOOR = ((250, 0), (rc.camera.get_height(), rc.camera.get_width()))

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
ALPHA = 0.3

prev_angle = 0.0
MAX_ANGLE_DELTA = 0.4

last_hash = 0
last_image = None

with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)
    BLUE = (tuple(config['Camera']['BLUE_lower']), tuple(config['Camera']['BLUE_upper']))
    kp = config['PID']['kp']
    kd = config['PID']['kd']
    OFFSET = config['Camera']['OFFSET']

def is_new_frame(image):
    global last_hash
    if image is None:
        return False
    current_hash = hash(image.tobytes())   # or hashlib.md5(image.tobytes()).digest()
    if current_hash == last_hash:
        return False
    last_hash = current_hash
    return True

def is_new_frame2(image):
    global last_image
    if last_image is not None and np.array_equal(image, last_image):
        return False
    last_image = image
    return True

# OFFSET = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour(image, save = False):
    # global contour_center
    global contour_area
    global indx

    # save = False

    # image = cv.imread('test_img_13.png')

    for i in range(1, 14):
        image = cv.imread('test_img_' + str(i) + '.png')
        # image = cv.imread('IMG_492' + str(i) + '.JPG')
        # image = cv.imread('test_img_1.png')
        target_point = None
        target_angle = None

        if image is None:
            contour_center = None
            contour_area = 0
        else:
            # Crop the image to the floor directly in front of the car
            # image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

            if save:
                cv.imwrite(str(indx) + '_photo.png', image)

            # Change to hsv
            hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
            blurred_hsv = cv.blur(hsv, (10, 10))
            # blurred_img = cv.cvtColor(blurred_hsv, cv.COLOR_HSV2BGR)
            # cv.imwrite('img_blur.png', blurred_hsv)

            y = 140
            x = 440
            hsv_pixel = hsv[y, x]
            # print(hsv_pixel)
            # 178 24 100
            # 101 130 100
            # 102 129 100

            # Fix glare holes by finding small holes
            # kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (15, 15))

            # Use blue mask and glare mask to get the line
            glare_mask = cv.inRange(blurred_hsv, (0, 0, 240), (179, 40, 255))  
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (80, 80))
            glare_mask = cv.dilate(glare_mask, kernel, iterations=1)
            # cv.imwrite('img_glare.png', glare_mask)
            color_mask = cv.inRange(blurred_hsv, BLUE[0], BLUE[1])
            # cv.imwrite('img_color.png', color_mask)
            mask = cv.bitwise_and(color_mask, cv.bitwise_not(glare_mask))
            # mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
            result = cv.bitwise_and(image, image, mask=mask)
            cv.imwrite('img_mask.png', result)

            max_contour = []
            # contours_list = []
            # for color in COLOR_PRIORITY:
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            # contours = rc_utils.find_contours(image, color[0], color[1])
            # contours_list.extend(contours)
            for contour in contours:
                # print(cv.contourArea(contour))
                if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                    if len(max_contour) == 0:
                        max_contour = contour
                    elif cv.contourArea(contour) > cv.contourArea(max_contour):
                        max_contour = contour
                # if len(max_contour) > 0:
                #     break

            # contours = rc_utils.find_contours(image, BLUE[0], BLUE[1])
            # contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if len(max_contour) > 0:
                # rc_utils.draw_circle(result, contour_center)
                contour_center = rc_utils.get_contour_center(max_contour)
                contour_center = (contour_center[0], contour_center[1] + OFFSET)
                if contour_center[1] < 1:
                    contour_center = (contour_center[0], 0)

                image_shape = hsv.shape

                # last_contour_center = contour

                mask = np.zeros(image.shape[:2], np.uint8)
                cv.drawContours(mask, [max_contour], -1, 255, -1)
                ys = []
                xs = []
                for y in range(mask.shape[0]):
                    cols = np.where(mask[y] > 0)[0]

                    if len(cols) > 5:
                        left = cols[0]
                        right = cols[-1]

                        xs.append((left + right) / 2)
                        ys.append(y)
                
                idx = np.argmin(ys)
                highest_point = (max(0, int(ys[idx]) + OFFSET), max(0, int(xs[idx])))
                # highest_point = tuple(max_contour[max_contour[:, :, 1].argmin()][0])
                # highest_point = (highest_point[1], highest_point[0])
                rc_utils.draw_circle(image, highest_point)
                target_point = (int(highest_point[0] * 0.8 + contour_center[0] * 0.2), int(highest_point[1] * 0.8 + contour_center[1] * 0.2))
                # row, col

                # Get target angle
                target_angle = math.atan2(image_shape[0] - target_point[0], target_point[1] - (image_shape[1] // 2)) * 180 / math.pi
                # print(target_point)
                # print(contour_center)
                target_angle = 90 - target_angle
                print("TARGET ANGLE: ", target_angle)

                # pts = np.array(max_contour).reshape(-1, 2) # (x, y)
                # x_coords = pts[:, 0]
                # y_coords = pts[:, 1]

                # coeffs = np.polyfit(xs, ys, 3) 
                # x_plot = np.linspace(min(xs), max(xs), num=100).astype(int) # smooth out x values
                # y_plot = np.polyval(coeffs, x_plot).astype(int) 
                # curve_points = np.stack((x_plot, y_plot), axis=-1).reshape((-1, 1, 2))

                # if curve_points is not None:
                #     # print('max points')
                #     # print(image.shape)
                #     # print(np.min(x_coords), np.max(x_coords))
                #     # print(np.min(y_coords), np.max(y_coords))
                #     top_indx = np.argmin(curve_points[:, 0, 1])
                #     top_of_curve = curve_points[top_indx, 0]
                #     col, row = top_of_curve
                #     # rc_utils.draw_circle(image, top_of_curve)
                #     # avg_x, avg_y = np.mean(intersection_points, axis=0)
                #     # print(max(intersection_points, key=lambda pt: pt[0])[0])
                #     # contour_center = (int(max(intersection_points, key=lambda pt: pt[0])[0]), max(0, int(avg_y) + OFFSET))
                #     highest_point = (max(0, int(row)), max(0, int(col) + OFFSET))
                #     rc_utils.draw_circle(image, highest_point)
                #     target_point = (int(highest_point[0] * 0.8 + contour_center[0] * 0.2), int(highest_point[1] * 0.8 + contour_center[1] * 0.2))
                # else:
                #     target_point = contour_center

                rc_utils.draw_contour(image, max_contour)
                rc.display.show_text(str(round(target_angle, 2)))
                # rc_utils.draw_contour(image, middle_contour)
                cv.putText(image, str(target_angle), (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2, cv.LINE_AA)
                cv.putText(image, str(target_point), (100, 170), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2, cv.LINE_AA)
                # rc_utils.draw_circle(image, contour_center)
                rc_utils.draw_circle(image, target_point)
                cv.line(image, target_point[::-1], (image_shape[1] // 2, image_shape[0]), 255, 30)
                
                # cv.polylines(image, [curve_points], isClosed=False, color=(0, 0, 255), thickness=3)
                # cv.ellipse(image, center_coords, (width, height), 0, 0, 360, 255, 10)
                # rc_utils.draw_contour(result, max_contour_hsv)

            # Display the image to the screen
            # rc_utils.draw_circle(result, (y, x))
            # rc.display.show_color_image(image)
            # cv.imwrite('img_output.png', result)

            if save:
                cv.imwrite(str(indx) + '_result.png', image)

            # cv.imwrite('IMG_492' + str(i) + 'FIXED.JPG', image)
            cv.imwrite('test_img_' + str(i) + '_FIXED.png', image)
            # cv.imwrite('test_img_13_FIXED.png', image)

            indx += 1

    return target_angle

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every 0.5 seconds
    rc.set_update_slow_time(0.1)

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
    global prev_angle
    global filtered_error
    
    rc.drive.set_max_speed(1)

    image = rc.camera.get_color_image()

    if image is None:
        print("No camera image")
        return

    fp = int(image.sum())              # cheap whole-frame fingerprint
    same = np.array_equal(image, last_image) if last_image is not None else False
    print(f"sum: {fp}   equal_to_last: {same}")

    if not is_new_frame(image):
        # frame is identical at every byte so do nothing
        return

    if not is_new_frame2(image):
        return

    # Search for contours in the current color image
    target_angle = update_contour(image)

    # Get contour center
    if target_angle is not None:
        # setpoint = rc.camera.get_width() // 2
        # present_value = contour_center[1]

        # raw_error = setpoint - present_value

        setpoint = 0
        raw_error = target_angle - setpoint

        # Low-pass filter
        filtered_error = ALPHA * raw_error + (1 - ALPHA) * filtered_error
        filtered_error = raw_error

        angle = kp * filtered_error + kd * (filtered_error - last_error) / rc.get_delta_time()
        # angle = rc_utils.clamp(angle, -0.7, 0.7)

        # Slew-rate limit: cap how much angle can change in one frame
        # angle = rc_utils.clamp(angle, prev_angle - MAX_ANGLE_DELTA, prev_angle + MAX_ANGLE_DELTA)
        # prev_angle = angle

        last_error = filtered_error
        error = filtered_error

    speed = 0.5
    speed = rc_utils.remap_range(abs(angle), 0, 1, 0.6, 0.3, saturate=True)
        
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
    # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    if rt > 0.2:
       speed = rt

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

    image = rc.camera.get_color_image()

    # update_contour(image, True)

    print("SPEED: ", speed)
    print("ANGLE: ", angle)
    print("ERROR: ", error)

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
