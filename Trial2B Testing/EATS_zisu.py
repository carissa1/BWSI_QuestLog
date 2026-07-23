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

WINDOW = 80
RAY_WINDOW = 30
KP = 0.01
MIN_VALID_DIST = 1
blindspot = 5
right_max_dist = 0
left_max_dist = 0
angle = 0
speed = 1

def start ():
    rc.drive.set_speed_angle(0, 0)
    data = ['Speed', 'Angle', 'Error', 'lidar_left', 'lidar_right']

def is_valid(dist):
    
    return dist is not None and dist > MIN_VALID_DIST

SAMPLES_PER_DEGREE = 720 / 360

def get_angle_range(scan, start_deg, end_deg):
    start_idx = int(start_deg * SAMPLES_PER_DEGREE)
    end_idx = int(end_deg * SAMPLES_PER_DEGREE)
    return scan[start_idx:end_idx]

def get_dist_angle (scan, window, window_start_deg):
    idx = window.argmax()
    angle_deg = window_start_deg + idx / SAMPLES_PER_DEGREE
    max_dist = rc_utils.get_lidar_average_distance(scan, angle_deg, RAY_WINDOW)
    return max_dist, angle_deg


def update():
    scan = rc.lidar.get_samples()
    right_window = get_angle_range(scan, blindspot, WINDOW)
    left_window = get_angle_range(scan, 360  - WINDOW, 360 - blindspot)
    right_max_dist, right_angle = get_dist_angle(right_window, 0)
    left_max_dist, left_angle = get_dist_angle(left_window, 360 - WINDOW)
    left_wt = 360 - left_angle
    total_dist = right_max_dist + left_max_dist
    target_angle = (right_angle * right_max_dist - left_wt * left_max_dist)/total_dist
    angle = target_angle * KP
    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.4, saturate=True)
    rc.drive.set_speed_angle(speed, angle)






