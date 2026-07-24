import sys
import math

from geometry_msgs.msg import Vector3Stamped

# If this file is nested inside a folder in the labs folder,
# the relative path should be "../../library"
sys.path.insert(1, "../../library")

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
ROBOT_HALF_WIDTH = 10
SAMPLES_PER_DEGREE = 1080 / 360

# added these to store attitude values
roll = 0.0
pitch = 0.0
yaw = 0.0
attitude_received = False

# added this to keep the ROS2 subscription alive
attitude_subscription = None


########################################################################################
# Attitude subscriber
########################################################################################

# added this to receive roll, pitch, and yaw from /attitude
def attitude_callback(msg):
    global roll, pitch, yaw, attitude_received

    # Vector3Stamped uses x, y, and z:
    # x = roll, y = pitch, z = yaw
    roll = msg.vector.x
    pitch = msg.vector.y
    yaw = msg.vector.z

    attitude_received = True


########################################################################################
# RACECAR functions
########################################################################################

def start():
    global attitude_subscription

    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)

    # added this to use racecar_core's existing ROS2 node
    if attitude_subscription is None:
        racecar_node = rc._RacecarReal__rate_node

        attitude_subscription = racecar_node.create_subscription(
            Vector3Stamped,
            "/attitude",
            attitude_callback,
            10
        )

    print("Wall follower started")
    print("Listening to /attitude")


def is_valid(dist):
    return dist is not None and dist > MIN_VALID_DIST


def get_angle_range(scan, start_deg, end_deg):
    start_idx = int(start_deg * SAMPLES_PER_DEGREE)
    end_idx = int(end_deg * SAMPLES_PER_DEGREE)

    return scan[start_idx:end_idx]


def get_dist_angle(scan, window, window_start_deg):
    if len(window) != 0:
        idx = window.argmax()

        angle_deg = (
            window_start_deg
            + idx / SAMPLES_PER_DEGREE
        )

        max_dist = rc_utils.get_lidar_average_distance(
            scan,
            angle_deg,
            RAY_WINDOW
        )

        if max_dist > RANGE:
            max_dist = RANGE

        return max_dist, angle_deg

    return 0.0, window_start_deg


def update():
    global roll, pitch, yaw, attitude_received

    scan = rc.lidar.get_samples()

    right_window = get_angle_range(
        scan,
        0,
        WINDOW
    )

    left_window = get_angle_range(
        scan,
        360 - WINDOW,
        360
    )

    right_max_dist, right_angle = get_dist_angle(
        scan,
        right_window,
        0
    )

    left_max_dist, left_angle = get_dist_angle(
        scan,
        left_window,
        360 - WINDOW
    )

    left_wt = (360 - left_angle) + 90
    right_wt = right_angle + 90

    # convert degrees to radians before using math.cos
    left_angle_radians = math.radians(360 - left_angle)
    right_angle_radians = math.radians(right_angle)

    left_cos = math.cos(left_angle_radians)
    right_cos = math.cos(right_angle_radians)

    # prevent division by zero
    if abs(left_cos) < 0.01:
        left_cos = 0.01

    if abs(right_cos) < 0.01:
        right_cos = 0.01

    left_dist = (
        left_max_dist
        - ROBOT_HALF_WIDTH / abs(left_cos)
    )

    right_dist = (
        right_max_dist
        - ROBOT_HALF_WIDTH / abs(right_cos)
    )

    total_dist = right_dist + left_dist

    if total_dist <= 0:
        rc.drive.set_speed_angle(0, 0)
        return

    target_angle = (
        right_wt * right_dist
        - left_wt * left_dist
    ) / total_dist

    angle = target_angle * KP
    angle = rc_utils.clamp(angle, -1, 1)

    speed = rc_utils.remap_range(
        abs(angle),
        0,
        1,
        1,
        0.25,
        saturate=True
    )

    # added this to display and use the attitude values
    if attitude_received:
        roll_degrees = math.degrees(roll)
        pitch_degrees = math.degrees(pitch)
        yaw_degrees = math.degrees(yaw)

        # slow down if the car is tilting
        if (
            abs(roll_degrees) > 15
            or abs(pitch_degrees) > 15
        ):
            speed = min(speed, 0.35)

        # stop if the car is tilted too far
        if (
            abs(roll_degrees) > 35
            or abs(pitch_degrees) > 35
        ):
            speed = 0
            angle = 0

        print(
            f"Roll: {roll_degrees:.1f}, "
            f"Pitch: {pitch_degrees:.1f}, "
            f"Yaw: {yaw_degrees:.1f}, "
            f"Left: {left_max_dist:.1f}, "
            f"Right: {right_max_dist:.1f}, "
            f"Angle: {angle:.2f}"
        )

    else:
        print(
            "Waiting for attitude data | "
            f"Left: {left_max_dist:.1f}, "
            f"Right: {right_max_dist:.1f}, "
            f"Angle: {angle:.2f}"
        )

    rc.drive.set_speed_angle(speed, angle)


def update_slow():
    pass


########################################################################################
# Main
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(
        start,
        update,
        update_slow
    )

    rc.go()
