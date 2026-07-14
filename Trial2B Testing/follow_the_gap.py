"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs




File Name: lab_g.py




Title: Lab G - Autonomous Parking




Author: [PLACEHOLDER] << [Write your name or team name here]




Purpose: This script provides the RACECAR with the ability to autonomously detect an orange
cone and then drive and park 30cm away from the cone. Complete the lines of code under the
#TODO indicators to complete the lab.




Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR detects the orange cone using its color camera, and can navigate to the cone
and park using its color camera and LIDAR sensors.
- The RACECAR should operate on a state machine with multiple states. There should not be
a terminal state. If there is no cone in the environment, the program should not crash.




Environment: Test your code using the level "Neo Labs > Lab G: Cone Parking".
Click on the screen to move the orange cone around the screen.
"""




########################################################################################
# Imports
########################################################################################




import sys
import cv2 as cv
import numpy as np
import math




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
MIN_CONTOUR_AREA = 30




# TODO Part 1: Determine the HSV color threshold pairs for ORANGE
ORANGE = ((10, 50, 50), (20, 255, 255))  # The HSV range for the color ORANGE




# Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
last_error = 0
time = 0
isRunning = False


STATES = {1: "APPROACH", 2: "REVERSE", 3: "RIGHT", 4: "LEFT", 5: "STOP", 6: "SEARCH"}
current_state = STATES[1]


# >> Constants
# Adjustable minimum distance that defines open space in the LIDAR scan.
OPEN_DISTANCE_THRESHOLD = 50.0
USE_TRACKING_THRESHOLD = 200.0


# Treat invalid or out-of-range readings as very far away.
MAX_LIDAR_DISTANCE = 1000.0


# Proportional controller gains.
TURN_KP = 0.01
TURN_KD = 0.0


# Side-wall avoidance at the nominal wheel angles (-45 and +45 degrees).
SIDE_WALL_THRESHOLD = 0.0
SIDE_WALL_KP = 1.0


# Car cannot turn in place, so enforce a minimum forward speed.
MIN_SPEED = 0.1
MAX_SPEED = 0.75


# Limit the scan to the 180 degrees in front of the car.
FRONT_ARC_DEGREES = 180.0


target_angle = 0.0
last_target_angle = 0.0
target_distance = 0.0
open_run_length = 0
wall_error = 0.0
wall_average_distance = 0.0
max_lidar_distance = 0.0
left_wall_distance = 0.0
right_wall_distance = 0.0




########################################################################################
# Functions
########################################################################################




def _front_samples(scan):
    """Return the front 180 degrees of the scan in left-to-right order.




    The LIDAR samples start at the front of the car and proceed clockwise.
    We re-order the front half so the list runs from left side, through forward,
    to right side.
    """
    sample_count = len(scan)
    half_count = sample_count // 4




    left_side = list(range(sample_count - half_count, sample_count))
    right_side = list(range(0, half_count + 1))
    ordered_indices = left_side + right_side




    front_samples = []
    sample_span = max(1, len(ordered_indices) - 1)




    for ordered_index, sample_index in enumerate(ordered_indices):
        # Map the ordered front arc to angles from -90 to +90 degrees.
        sample_angle = -90.0 + 180.0 * ordered_index / sample_span
        front_samples.append((sample_angle, scan[sample_index]))




    return front_samples




def _clean_distance(distance):
    if distance is None or not math.isfinite(distance) or distance <= 0:
        return MAX_LIDAR_DISTANCE
    return distance







def _nearest_sample_distance(samples, target_angle):
    closest_angle, closest_distance = min(
        samples,
        key=lambda sample: abs(sample[0] - target_angle),
    )
    return _clean_distance(closest_distance)








def _largest_open_run(samples, threshold):
    """Find the largest continuous run where distance is greater than threshold."""
    best_run = []
    current_run = []
    found_open = False
    use_tracking = False


    for sample_angle, sample_distance in samples:
        sample_distance = _clean_distance(sample_distance)
        if sample_distance > USE_TRACKING_THRESHOLD:
            use_tracking = True
        if sample_distance > threshold:
            found_open = True
            current_run.append((sample_angle, sample_distance))
        else:
            if len(current_run) > len(best_run):
                best_run = current_run
            current_run = []




    if len(current_run) > len(best_run):
        best_run = current_run


    # if len(best_run) == 0:
        #     # Fallback: no sample exceeded the threshold, so use the single farthest sample.
        #     best_index = max(range(len(samples)), key=lambda i: samples[i][1])
        #     best_run = [samples[best_index]]


    if len(best_run) != 0 and use_tracking:
        run_angles = [sample[0] for sample in best_run]
        run_distances = [sample[1] for sample in best_run]


        average_angle = sum(run_angles) / len(run_angles)
        average_distance = sum(run_distances) / len(run_distances)
    else:
        average_angle = 0.0
        average_distance = 0.0

    print("angle: ", average_angle)
    print("use tracking: ", use_tracking)

    return average_angle, average_distance, len(best_run), found_open

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle




    global target_angle
    global last_target_angle
    global target_distance
    global open_run_length
    global wall_error
    global wall_average_distance
    global max_lidar_distance
    global left_wall_distance
    global right_wall_distance




    target_angle = 0.0
    last_target_angle = 0.0
    target_distance = 0.0
    open_run_length = 0
    wall_error = 0.0
    wall_average_distance = 0.0
    max_lidar_distance = 0.0
    left_wall_distance = 0.0
    right_wall_distance = 0.0


    # Initialize variables
    speed = 0
    angle = 0




    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)




    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)




    # Print start message
    print(
        ">> Lab G - Autonomous Parking\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )








# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global speed
    global angle
    global current_state
    global last_error
    global time
    global isRunning




    global target_angle
    global last_target_angle
    global target_distance
    global open_run_length
    global wall_error
    global wall_average_distance
    global max_lidar_distance
    global left_wall_distance
    global right_wall_distance


    rc.drive.set_max_speed(0.6)  




    # Get LIDAR
    scan = rc.lidar.get_samples()
    LIDAR_angle, LIDAR_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))


    front_samples = _front_samples(scan)
    cleaned_scan = [_clean_distance(distance) for distance in scan]
    max_lidar_distance = max(cleaned_scan)
    left_wall_distance = _nearest_sample_distance(front_samples, -45.0)
    right_wall_distance = _nearest_sample_distance(front_samples, 45.0)


    target_angle, target_distance, open_run_length, found_open = _largest_open_run(
        front_samples, OPEN_DISTANCE_THRESHOLD
    )
    delta = rc.get_delta_time()
    deriv = target_angle - (last_target_angle) * delta
    last_target_angle = target_angle




    # Proportional steering toward the center of the largest open region.
    angle = TURN_KP * target_angle + TURN_KD * deriv


    # If the wall is too close at the wheel-side beams (-45 and +45 degrees),
    # bias the steering away from that wall.
    if left_wall_distance < SIDE_WALL_THRESHOLD:
        angle += SIDE_WALL_KP * (SIDE_WALL_THRESHOLD - left_wall_distance) / SIDE_WALL_THRESHOLD
    if right_wall_distance < SIDE_WALL_THRESHOLD:
        angle -= SIDE_WALL_KP * (SIDE_WALL_THRESHOLD - right_wall_distance) / SIDE_WALL_THRESHOLD


    angle = rc_utils.clamp(angle, -1.0, 1.0)


    # Find the closest distance in the 90 degree window behind the car
    # _, back_distance = rc_utils.get_lidar_closest_point(scan, (135, 225))
    # Find the closest distance in the 90 degree window in front of the car
    # _, front_distance = rc_utils.get_lidar_closest_point(scan, (315, 45))


    dt = rc.get_delta_time()


    # TODO Part 3: Park the car 30cm away from the closest orange cone.
    # You may use a state machine and a combination of sensors (color camera,
    # or LIDAR to do so). Depth camera is not allowed at this time to match the
    # physical RACECAR Neo.
    print("front dist:", LIDAR_dist)
    


    rc.drive.set_max_speed(0.4)
    setpoint = 100 # CHANGE 250
    present_value = LIDAR_dist
    error = present_value - setpoint

    kp = 0.005
    kd = 0.0

    speed = kp*error + kd*(error - last_error)/dt
    last_error = error

    speed = rc_utils.clamp(speed, -1, 1)

    print("speed: ", speed)
    print("angle: ", angle)


    if isRunning:
        time += dt
    # print("time", time)


    # Set the speed and angle of the RACECAR after calculations have been complete
    rc.drive.set_speed_angle(speed, angle)




# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass




########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################




if __name__ == "__main__":
   rc.set_start_update(start, update, update_slow)
   rc.go()



