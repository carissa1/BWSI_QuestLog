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

Environment: Test your code using the level "Neo Labs > Lab I: Wall Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys
import math
import csv

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

SCAN_WINDOW = 30
BLIND_WINDOW = 10
CRITICAL_DIST = 0
LOOK_AHEAD_DIST = 500

KP_WEIGHT = 0.006
KP_ANGLE = 0.04

# ---- State carried between frames -------------------------------------------------------
prev_error = 0
last_speed = 0
last_angle = 0
filtered_error = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global prev_error, last_speed, last_angle
    prev_error = 0
    last_speed = 0
    last_angle = 0
    rc.drive.set_speed_angle(0, 0)
    data = ['Speed', 'Angle', 'Error', 'lidar_left', 'lidar_right', 'weight_left', 'weight_right']

    with open('log_wall.csv', mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)



# def is_valid(dist):
    
#     return dist is not None and dist > MIN_VALID_DIST


# def get_wall_reading(scan, side_angle, front_angle, theta_deg):
#     b = rc_utils.get_lidar_average_distance(scan, side_angle, WINDOW_ANGLE)
#     a = rc_utils.get_lidar_average_distance(scan, front_angle, WINDOW_ANGLE)

#     if not is_valid(a) or not is_valid(b):
#         return None, None

#     theta = math.radians(theta_deg)
#     alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))

#     current_dist = b * math.cos(alpha)
#     predicted_dist = current_dist + LOOKAHEAD * math.sin(alpha)

#     return current_dist, predicted_dist

def get_lidar_dist(scan):
    LEN_SCAN = 720 # 1080
    # print(scan)
    new_scan = [200]*LEN_SCAN

    # Find farthest point on right side
    max_right = 0
    max_right_indx = 0
    max_right_angle = 0
    for i in range(BLIND_WINDOW, LEN_SCAN//4):
        if scan[i] > max_right:
            max_right = scan[i]
            max_right_indx = i
            max_right_angle = i * 360 / LEN_SCAN
        if scan[i] < 1:
            max_right = LOOK_AHEAD_DIST
            max_right_indx = i
            max_right_angle = i * 360 / LEN_SCAN
    
    # Get average distance within a window
    window_left = SCAN_WINDOW // 2
    window_right = SCAN_WINDOW // 2
    if max_right_indx + window_right > LEN_SCAN:
        window_right = LEN_SCAN - max_right_indx
        window_left += SCAN_WINDOW // 2 - (LEN_SCAN - max_right_indx)
    elif max_right_indx - window_left < 0:
        window_right += max_right_indx
        window_left -= max_right_indx
    avg_right_dist = 0
    for i in range(max_right_indx - window_left, max_right_indx + window_right):
        avg_right_dist += scan[i]
    avg_right_dist /= window_left + window_right
    # avg_right_dist = rc_utils.get_lidar_average_distance(scan, max_right_angle, SCAN_WINDOW)

    # print(avg_right_dist)
    if avg_right_dist > LOOK_AHEAD_DIST:
        avg_right_dist = LOOK_AHEAD_DIST

    # Find farthest point on left side
    max_left = 0
    max_left_indx = 0
    max_left_angle = 0
    for i in range(3*LEN_SCAN//4, LEN_SCAN -  BLIND_WINDOW): # Right side 
        if scan[i] > max_left:
            max_left = scan[i]
            max_left_indx = i
            max_left_angle = i * 360 / LEN_SCAN
        if scan[i] < 1:
            max_left = LOOK_AHEAD_DIST
            max_left_indx = i
            max_left_angle = i * 360 / LEN_SCAN
    
    # Get average distance within a window
    window_left = SCAN_WINDOW // 2
    window_right = SCAN_WINDOW // 2
    if max_left_indx + window_right > LEN_SCAN:
        window_right = LEN_SCAN - max_left_indx
        window_left += SCAN_WINDOW // 2 - (LEN_SCAN - max_left_indx)
    if max_left_indx - window_left < 0:
        window_right += max_left_indx
        window_left -= max_left_indx
    avg_left_dist = 0
    for i in range(max_left_indx - window_left, max_left_indx + window_right):
        avg_left_dist += scan[i]
    avg_left_dist /= window_left + window_right

    # print(avg_left_dist)
    if avg_left_dist > LOOK_AHEAD_DIST:
        avg_left_dist = LOOK_AHEAD_DIST

    # avg_left_dist = rc_utils.get_lidar_average_distance(scan, max_left_angle, SCAN_WINDOW)

    print("   RIGHT: ", avg_right_dist, max_right_angle)
    print("   LEFT: ", avg_left_dist, max_left_angle)
    print("   WINDOW: ", window_left, window_right)
    f = open('scan.txt', 'w')
    f.write(str(scan))

    return avg_right_dist, max_right_angle, avg_left_dist, max_left_angle

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global prev_error, last_speed, last_angle, filtered_error

    scan = rc.lidar.get_samples()

    rc.drive.set_max_speed(1)

    # right_dist, right_pred = get_wall_reading(scan, RIGHT_SIDE_ANGLE, RIGHT_FRONT_ANGLE, DELTA_ANGLE)
    # left_dist, left_pred = get_wall_reading(scan, LEFT_SIDE_ANGLE, LEFT_FRONT_ANGLE, DELTA_ANGLE)

    right_dist, right_angle, left_dist, left_angle = get_lidar_dist(scan)

    front_angle, front_dist = rc_utils.get_lidar_closest_point(scan, (330, 30))
    print("   FRONT_DIST: ", front_dist, front_angle)

    # have_right = right_dist is not None
    # have_left = left_dist is not None

    # EATS Algorithm
    if front_dist > CRITICAL_DIST:
        dist_diff = right_dist - left_dist
        delta_weight = abs(dist_diff * KP_WEIGHT)
        delta_weight = rc_utils.clamp(delta_weight, 0, 0.4)

        weight_R = 0.5
        weight_L = 0.5
        if right_dist < left_dist:
            weight_R -= delta_weight
            weight_L += delta_weight
        elif right_dist > left_dist:
            weight_R += delta_weight
            weight_L -= delta_weight

        weight_L = rc_utils.clamp(weight_L, 0, 1)
        weight_R = rc_utils.clamp(weight_R, 0, 1)

        print("WEIGHT: ", delta_weight, weight_R, weight_L)
        # error = (right_angle * right_dist - (360 - left_angle) * left_dist)/ (left_dist + right_dist)
        # error = right_dist * weight_R - left_dist * weight_L
        error = right_angle * weight_R - (360 - left_angle)* weight_L
        angle = error * KP_ANGLE

    else:
        error = 10
        if front_angle > 180: # turn right
            angle = 0.2
        else: # turn left
            angle = -0.2
    

    # ---- Pick a steering error based on which wall(s) are visible this frame ----
    # if have_right and have_left:
    #     error = (right_pred - left_pred)+(right_dist - left_dist) / 2
    #     filtered_error = alpha * error + (1-alpha) * filtered_error
    #     kp, kd = KP_CENTER, KD_CENTER
    # else:
    #     filtered_error = 0  # no walls in sight at all -- go straight until we pick one back up
    #     kp, kd = KP, KD

    # angle = kp * filtered_error + kd * (filtered_error - prev_error)
    # prev_error = filtered_error
   
    # front_dist = rc_utils.get_lidar_average_distance(scan, FRONT_ANGLE, FRONT_WINDOW_ANGLE)
    # if not is_valid(front_dist):
    #     front_dist = SLOW_DOWN_DIST * 2  # no reading -> assume the way ahead is clear

    # if front_dist < CRITICAL_FRONT_DIST:
    #     right_room = right_dist if have_right else float("inf")
    #     left_room = left_dist if have_left else float("inf")
    #     angle = 0.6 if right_room > left_room else -0.6
    #     speed = 0.2
    # else:
    angle = rc_utils.clamp(angle, -1, 1)


    # Slow down approaching corners/dead-ends or while turning sharply; speed back
    # up in open, straight corridors.
    # speed_for_clearance = rc_utils.remap_range(
    #     front_dist, CRITICAL_FRONT_DIST, SLOW_DOWN_DIST, MIN_SPEED, MAX_SPEED, True
    # )
    # speed_for_turn = rc_utils.remap_range(abs(angle), 0, 1, MAX_SPEED, MIN_SPEED, True)
    # speed = rc_utils.clamp(min(speed_for_clearance, speed_for_turn), MIN_SPEED, MAX_SPEED)
    # speed = rc_utils.remap_range(abs(angle), 0, 1, 0.8, 0.4, saturate=True)
    speed = 0.3

    # if right_dist is None:
    #     angle = 0.65
    # elif left_dist is None:
    #     angle = -0.65
    # elif right_dist - left_dist > 60:
    #     angle = 0.65
    
    # rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # if rt > 0.2:
    #     angle = 0.6
    # if lt > 0.2:
    #     angle = -0.6

    last_speed, last_angle = speed, angle
    rc.drive.set_speed_angle(speed, angle)
    print("Speed: ", speed, " Angle:", angle)
    # print(f"right={right_dist}  left={left_dist} front={front_dist}")

    data = [speed, angle, error, left_dist, right_dist]

    with open('log_wall.csv', mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)

def update_slow():
    # print(f"speed={last_speed}  angle={last_angle}  err={prev_error:.1f}")
    pass

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()