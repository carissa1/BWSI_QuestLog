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

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()


WINDOW_ANGLE = 25            # half-width (deg) averaged per reading -> smooths noise & small gaps
DELTA_ANGLE = 50            # deg between each side's "side ray" and its "front-diagonal ray"

RIGHT_SIDE_ANGLE = 90
RIGHT_FRONT_ANGLE = RIGHT_SIDE_ANGLE - DELTA_ANGLE      # 45 deg
LEFT_SIDE_ANGLE = 270
LEFT_FRONT_ANGLE = LEFT_SIDE_ANGLE + DELTA_ANGLE        # 315 deg

FRONT_ANGLE = 0
FRONT_WINDOW_ANGLE = 30

LOOKAHEAD = 250              # how far ahead we predict our distance from each wall   
MIN_VALID_DIST = 1          # readings at/below this count as "no wall there"

MAX_SPEED = 1.0
MIN_SPEED = 0.7
SLOW_DOWN_DIST = 0      # start slowing down once front clearance drops below this
CRITICAL_FRONT_DIST = 20    # below this, drop the PD math and force an emergency turn        
KP_CENTER, KD_CENTER = 0.015, 0.002   # gains when centering between two walls
KP = 0.01
KD = 0.00

# ---- State carried between frames -------------------------------------------------------
prev_error = 0
last_speed = 0
last_angle = 0


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


def is_valid(dist):
    
    return dist is not None and dist > MIN_VALID_DIST


def get_wall_reading(scan, side_angle, front_angle, theta_deg):
    b = rc_utils.get_lidar_average_distance(scan, side_angle, WINDOW_ANGLE)
    a = rc_utils.get_lidar_average_distance(scan, front_angle, WINDOW_ANGLE)

    if not is_valid(a) or not is_valid(b):
        return None, None

    theta = math.radians(theta_deg)
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))

    current_dist = b * math.cos(alpha)
    predicted_dist = current_dist + LOOKAHEAD * math.sin(alpha)

    return current_dist, predicted_dist


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global prev_error, last_speed, last_angle

    scan = rc.lidar.get_samples()

    rc.drive.set_max_speed(1)

    right_dist, right_pred = get_wall_reading(scan, RIGHT_SIDE_ANGLE, RIGHT_FRONT_ANGLE, DELTA_ANGLE)
    left_dist, left_pred = get_wall_reading(scan, LEFT_SIDE_ANGLE, LEFT_FRONT_ANGLE, DELTA_ANGLE)

    have_right = right_pred is not None
    have_left = left_pred is not None

    # ---- Pick a steering error based on which wall(s) are visible this frame ----
    if have_right and have_left:

        error = ((right_pred - left_pred)+(right_dist - left_dist)) / 2
        kp, kd = KP_CENTER, KD_CENTER
    else:
        error = 0  # no walls in sight at all -- go straight until we pick one back up
        kp, kd = KP, KD

    angle = kp * error + kd * (error - prev_error)
    prev_error = error

   
    front_dist = rc_utils.get_lidar_average_distance(scan, FRONT_ANGLE, FRONT_WINDOW_ANGLE)
    if not is_valid(front_dist):
        front_dist = SLOW_DOWN_DIST * 2  # no reading -> assume the way ahead is clear

    if front_dist < CRITICAL_FRONT_DIST:
       
        right_room = right_dist if have_right else float("inf")
        left_room = left_dist if have_left else float("inf")
        angle = 1 if right_room > left_room else -1
        speed = 1.0
    else:
        angle = rc_utils.clamp(angle, -1, 1)
        # Slow down approaching corners/dead-ends or while turning sharply; speed back
        # up in open, straight corridors.
        speed_for_clearance = rc_utils.remap_range(
            front_dist, CRITICAL_FRONT_DIST, SLOW_DOWN_DIST, MIN_SPEED, MAX_SPEED, True
        )
        speed_for_turn = rc_utils.remap_range(abs(angle), 0, 1, MAX_SPEED, MIN_SPEED, True)
        speed = rc_utils.clamp(min(speed_for_clearance, speed_for_turn), MIN_SPEED, MAX_SPEED)

    if right_dist is None:
        angle = 0.6
    elif left_dist is None:
        angle = -0.6
    elif right_dist - left_dist > 70:
        angle = 0.6
    

    last_speed, last_angle = speed, angle
    rc.drive.set_speed_angle(speed, angle)
    print("Speed: ", speed, " Angle:", angle)
    print(f"right={right_dist}  left={left_dist} front={front_dist}")





def update_slow():
    print(f"speed={last_speed}  angle={last_angle}  err={prev_error:.1f}")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
