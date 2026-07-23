import sys
import math
import csv
# If this file is nested inside a folder in the labs folder, the relative path should be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()
WINDOW = 110
RAY_WINDOW = 20
KP = 0.01
MIN_VALID_DIST = 1
RANGE = 200
speed = 1
ROBOT_HALF_WIDTH = 10
SAMPLES_PER_DEGREE = 1080 / 360

def start ():
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)
    data = ['Speed', 'Angle', 'Error', 'lidar_left', 'lidar_right']
   
def is_valid(dist):
    return dist is not None and dist > MIN_VALID_DIST
    
def get_angle_range(scan, start_deg, end_deg):
    start_idx = int(start_deg * SAMPLES_PER_DEGREE)
    end_idx = int(end_deg * SAMPLES_PER_DEGREE)
    return scan[start_idx:end_idx]

def get_dist_angle (scan, window, window_start_deg):
    if len(window) != 0:
        idx = window.argmax()
        angle_deg = window_start_deg + idx / SAMPLES_PER_DEGREE
        max_dist = rc_utils.get_lidar_average_distance(scan, angle_deg, RAY_WINDOW)
        if max_dist > RANGE:
            max_dist = RANGE
        return max_dist, angle_deg
    
def update():
    global WINDOW
    scan = rc.lidar.get_samples()
    right_window = get_angle_range(scan, 0, WINDOW)
    left_window = get_angle_range(scan, 360 - WINDOW, 360)
    right_max_dist, right_angle = get_dist_angle(scan, right_window, 0)
    left_max_dist, left_angle = get_dist_angle(scan, left_window, 360 - WINDOW)
    left_wt = (360 - left_angle) + 90
    right_wt = right_angle + 90
    left_dist = left_max_dist - ROBOT_HALF_WIDTH / math.cos(360-left_angle)
    right_dist = right_max_dist - ROBOT_HALF_WIDTH / math.cos(360-right_angle)
    total_dist = right_dist + left_dist
    target_angle = (right_wt * right_dist - left_wt * left_dist)/total_dist
    target_angle = (right_wt * right_max_dist - left_wt * left_max_dist)/total_dist
    angle = target_angle * KP
    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.25, saturate=True)
    print(f"{right_angle=}, {left_angle=}, {right_max_dist=}, {left_max_dist=} {target_angle=}")
    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()





