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
sys.path.insert(0, '../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()


WINDOW_ANGLE = 6            # half-width (deg) averaged per reading -> smooths noise & small gaps
DELTA_ANGLE = 45            # deg between each side's "side ray" and its "front-diagonal ray"

RIGHT_SIDE_ANGLE = 90
RIGHT_FRONT_ANGLE = RIGHT_SIDE_ANGLE - DELTA_ANGLE      # 45 deg
LEFT_SIDE_ANGLE = 270
LEFT_FRONT_ANGLE = LEFT_SIDE_ANGLE + DELTA_ANGLE        # 315 deg

FRONT_ANGLE = 0
FRONT_WINDOW_ANGLE = 12

LOOKAHEAD = 60              # how far ahead we predict our distance from each wall
TARGET_WALL_DIST = 60       # desired distance from a single wall when only one is visible
MIN_VALID_DIST = 1          # readings at/below this count as "no wall there"

MAX_SPEED = 1.0
MIN_SPEED = 0.25
SLOW_DOWN_DIST = 150        # start slowing down once front clearance drops below this
CRITICAL_FRONT_DIST = 45    # below this, drop the PD math and force an emergency turn

KP, KD = 0.012, 0.02                 # gains when following a single wall
KP_CENTER, KD_CENTER = 0.01, 0.015   # gains when centering between two walls

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

    right_dist, right_pred = get_wall_reading(scan, RIGHT_SIDE_ANGLE, RIGHT_FRONT_ANGLE, DELTA_ANGLE)
    left_dist, left_pred = get_wall_reading(scan, LEFT_SIDE_ANGLE, LEFT_FRONT_ANGLE, DELTA_ANGLE)

    have_right = right_pred is not None
    have_left = left_pred is not None

    # ---- Pick a steering error based on which wall(s) are visible this frame ----
    if have_right and have_left:
        # Both walls: aim for the center of the corridor. Using the DIFFERENCE between
        # the two sides (rather than an absolute target distance) automatically adapts
        # to narrow or wide corridors without needing separate tuning for each.
        error = (right_pred - left_pred) / 2
        kp, kd = KP_CENTER, KD_CENTER
    elif have_right:
        error = right_pred - TARGET_WALL_DIST       # too far from right wall -> steer right (+)
        kp, kd = KP, KD
    elif have_left:
        error = -(left_pred - TARGET_WALL_DIST)      # too far from left wall -> steer left (-)
        kp, kd = KP, KD
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
        angle = 1.0 if right_room > left_room else -1.0
        speed = -0.2
    else:
        angle = rc_utils.clamp(angle, -1, 1)
        # Slow down approaching corners/dead-ends or while turning sharply; speed back
        # up in open, straight corridors.
        speed_for_clearance = rc_utils.remap_range(
            front_dist, CRITICAL_FRONT_DIST, SLOW_DOWN_DIST, MIN_SPEED, MAX_SPEED, True
        )
        speed_for_turn = rc_utils.remap_range(abs(angle), 0, 1, MAX_SPEED, MIN_SPEED, True)
        speed = rc_utils.clamp(min(speed_for_clearance, speed_for_turn), MIN_SPEED, MAX_SPEED)

    last_speed, last_angle = speed, angle
    rc.drive.set_speed_angle(speed, angle)



def update_slow():
    print(f"speed={last_speed:.2f}  angle={last_angle:.2f}  err={prev_error:.1f}")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()