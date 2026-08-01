import sys
import math
import csv
import cv2 as cv

import yaml

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Variables for wall follower
WINDOW = 100 # 100
RAY_WINDOW = 2 # 2
KP = 0.01 # 0.011
MIN_VALID_DIST = 1
RANGE = 125 # 125
right_max_dist = 0
left_max_dist = 0
angle = 0
speed = 1
ROBOT_HALF_WIDTH = 0

# Variables for line follower
MIN_CONTOUR_AREA = 700
CROP_FLOOR = ((250, 0), (rc.camera.get_height(), rc.camera.get_width()))
speed = 0.0             # The current speed of the car
angle = 0.0             # The current angle of the car's wheels
contour_center = None   # The (pixel row, pixel column) of contour
contour_area = 0        # The area of contour
indx = 0                # indx number of photos (for debugging)
last_error = 0
error = 0
filtered_error = 0      # low pass filter
prev_angle = 0.0
last_hash = 0
last_image = None

# Set tunable constants through config
with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)
    BLUE = (tuple(config['Camera']['BLUE_lower']), tuple(config['Camera']['BLUE_upper']))
    kp = -config['PID']['kp']
    kd = -config['PID']['kd']
    OFFSET = config['Camera']['OFFSET']
    EDGE_THRESHOLD = config['Camera']['EDGE_THRESHOLD'] # threshold for how close bounding box needs to be to edge
    ALPHA = config['PID']['ALPHA']
    MAX_ANGLE_DELTA = config['PID']['MAX_ANGLE_DELTA']

def is_valid(dist):
    # checks if distance from wall is above a minimum valid distance
    return dist is not None and dist > MIN_VALID_DIST

SAMPLES_PER_DEGREE = 720 / 360 # 1080 for real car

# Wall follower
def get_angle_range(scan, start_deg, end_deg):
    # get indices for range of angles
    start_idx = int(start_deg * SAMPLES_PER_DEGREE)
    end_idx = int(end_deg * SAMPLES_PER_DEGREE)
    return scan[start_idx:end_idx]

def get_dist_angle (scan, window, window_start_deg):
    # get farthest distance and its angle within a window
    if len(window) != 0:
        idx = window.argmax()
        angle_deg = window_start_deg + idx / SAMPLES_PER_DEGREE
        max_dist = rc_utils.get_lidar_average_distance(scan, angle_deg, RAY_WINDOW)
        if max_dist > RANGE:
            max_dist = RANGE
        return max_dist, angle_deg

# Line follower
def update_contour(save = False):
    global contour_area
    global indx

    image = rc.camera.get_color_image()

    # for i in range(1, 16): # TESTINGx
    # image = cv.imread('test_img_' + str(i) + '.png')
    # image = cv.imread('IMG_492' + str(i) + '.JPG')
    # image = cv.imread('test_img_1.png')
    target_point = None
    target_angle = None

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        if save:
            cv.imwrite(str(indx) + '_photo.png', image)

        # Change to hsv
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        blurred_hsv = cv.blur(hsv, (10, 10))

        # Use blue mask and glare mask to find the line
        glare_mask = cv.inRange(blurred_hsv, (0, 0, 240), (179, 40, 255))  
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (80, 80))
        glare_mask = cv.dilate(glare_mask, kernel, iterations=1)
        # cv.imwrite('img_glare.png', glare_mask)
        color_mask = cv.inRange(blurred_hsv, BLUE[0], BLUE[1])
        # cv.imwrite('img_color.png', color_mask)
        mask = cv.bitwise_and(color_mask, cv.bitwise_not(glare_mask))
        result = cv.bitwise_and(image, image, mask=mask)
        # cv.imwrite('img_mask.png', result)

        # Get the maximum contour
        max_contour = []
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        img_h, img_w = image.shape[:2]
        for contour in contours:
            # Check if the contour is touching two edges (creating a line)
            if cv.contourArea(contour) > MIN_CONTOUR_AREA:
                x, y, cw, ch = cv.boundingRect(contour)
                # print(x, y, cw, ch)
                # print(x + cw, y + ch, img_w, img_h)
                num_edges = 0
                if x <= EDGE_THRESHOLD:
                    num_edges += 1
                if y <= EDGE_THRESHOLD:
                    num_edges += 1
                if x + cw >= img_w - EDGE_THRESHOLD:
                    num_edges += 1
                if y + ch >= img_h - EDGE_THRESHOLD:
                    num_edges += 1
                cv.rectangle(image, (x, y), (x+cw, y+ch), (0, 0, 255), 2) 
                # print(num_edges)
                if num_edges >= 2:
                    if len(max_contour) == 0:
                        max_contour = contour
                    elif cv.contourArea(contour) > cv.contourArea(max_contour):
                        max_contour = contour

        # Find contour center
        contour_center = None
        if len(max_contour) > 0:
            contour_center = rc_utils.get_contour_center(max_contour)
            contour_center = (contour_center[0], contour_center[1] + OFFSET)
            if contour_center[1] < 1:
                contour_center = (contour_center[0], 0)

            # print(contour_center)

            rc_utils.draw_contour(image, max_contour)
            rc_utils.draw_circle(image, contour_center)
            if save:
                cv.imwrite(str(indx) + '_result.png', image)
            indx += 1
    return contour_center

def start ():
    rc.drive.set_speed_angle(0, 0)
    rc.drive.set_max_speed(1)
    data = ['Speed', 'Angle', 'Error', 'lidar_left', 'lidar_right']
    with open('log_wall.csv', mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(data)

def update():
    global WINDOW
    global speed
    global angle
    global last_error
    global error
    global kp
    global kd
    global prev_angle
    global filtered_error

    contour_center = update_contour()
    print(contour_center)
    contour_center = None
    scan = rc.lidar.get_samples()
    if contour_center is not None: # found a line
        setpoint = rc.camera.get_width() // 2
        present_value = contour_center[1]

        raw_error = setpoint - present_value

        # Low-pass filter
        filtered_error = ALPHA * raw_error + (1 - ALPHA) * filtered_error

        angle = kp * filtered_error + kd * (filtered_error - last_error) / rc.get_delta_time()
        angle = rc_utils.clamp(angle, -1, 1)

        # Slew-rate limit: cap how much angle can change in one frame
        angle = rc_utils.clamp(angle, prev_angle - MAX_ANGLE_DELTA, prev_angle + MAX_ANGLE_DELTA)
        prev_angle = angle

        last_error = filtered_error
        error = filtered_error
        speed = rc_utils.remap_range(abs(angle), 0, 1, 0.6, 0.3, saturate=True) # speed control
    else: # Otherwise, wall follow
        # get farthest point on left and right in window
        right_window = get_angle_range(scan, 0, WINDOW)
        left_window = get_angle_range(scan, 360 - WINDOW, 360)
        right_max_dist, right_angle = get_dist_angle(scan, right_window, 0)
        left_max_dist, left_angle = get_dist_angle(scan, left_window, 360 - WINDOW)

        # change angles to be within 90 and 180 degrees for better ratios
        left_wt = (360 - left_angle) + 90
        right_wt = right_angle + 90

        # correct left and right distance to deal with robot width
        left_dist = left_max_dist - ROBOT_HALF_WIDTH / math.cos(360-left_angle)
        right_dist = right_max_dist - ROBOT_HALF_WIDTH / math.cos(360-right_angle)
        total_dist = right_dist + left_dist

        # get target angle
        target_angle = (right_wt * right_dist - left_wt * left_dist)/total_dist
        target_angle = (right_wt * right_max_dist - left_wt * left_max_dist)/total_dist
        angle = target_angle * KP
        angle = rc_utils.clamp(angle, -1, 1)
        # speed controller
        speed = rc_utils.remap_range(abs(angle), 0, 1, 0.7, 0, saturate=True)
        #speed = rc_utils.remap_range(abs(total_dist), 0, RANGE*2, 0, 0.7, saturate=True)


        print(f"{right_angle=}, {left_angle=}, {right_max_dist=}, {left_max_dist=} {target_angle=}")
        print(f"{speed=}, f{angle=}")

    # Crash protection
    closest_left_dist, left_angle = rc_utils.get_lidar_closest_point(scan, (240, 360))
    closest_right_dist, right_angle = rc_utils.get_lidar_closest_point(scan, (0, 120))

    if closest_left_dist < 20 and closest_right_dist < 20 and left_angle > 340 and right_angle < 320:
        angle = 0
        speed = -1
    elif closest_left_dist < 20:
        speed = -1
    elif closest_right_dist < 20:
        speed = -1

    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    update_contour(True)

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()