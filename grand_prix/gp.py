import sys
import math
import csv
import cv2 as cv
import numpy as np

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

# Variables for follow-the-longest-distance
LONGEST_FOV_DEG = 180   # total forward field of view to search (centered on 0deg)
RAY_WIDTH_DEG = 20      # total angular width considered for each candidate ray, to reduce noise
STEER_GAIN = 1.0        # multiplier on normalized target angle before clamping
TIE_TOL_FRAC = 0.02     # fraction of RANGE treated as "tied for farthest"
MIN_VALID_DIST = 1      # lidar readings at/below this are treated as invalid, not real obstacles
RANGE = 400             # cap on any single lidar reading, and the "clear" fill-in value
angle = 0
speed = 1

# Variables for line follower
MIN_CONTOUR_AREA = 99999999
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
marker_num = -1

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

SAMPLES_PER_DEGREE = 1080 / 360 # 1080 for real car

def smooth(arr, half_window):
    # simple moving-average smoothing, used to reduce noise before picking a farthest ray
    if half_window <= 0:
        return arr
    kernel = np.ones(2 * half_window + 1) / (2 * half_window + 1)
    return np.convolve(arr, kernel, mode='same')

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
    global speed
    global angle
    global last_error
    global error
    global kp
    global kd
    global prev_angle
    global filtered_error
    global marker_num

    contour_center = update_contour()
    #print(contour_center)
    #contour_center = None
    scan = rc.lidar.get_samples()
    # image = rc.camera.get_color_image()
    # markers = rc_utils.get_ar_markers(image) # marker_type=cv.aruco.DICT_5X5_250
    # if len(markers) >= 1:
    #     markers_num = markers[0].get_id()
    #     print(markers_num)
    if contour_center is not None: # found a line
        print("LINE")
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
    else: # Otherwise, steer toward the direction of longest lidar distance
        half_fov_idx = int(LONGEST_FOV_DEG / 2 * SAMPLES_PER_DEGREE)
        center_idx = half_fov_idx
        n = len(scan)

        # Build a forward-centered window, wrapping across the 0/360 boundary
        start_idx = (0 - half_fov_idx) % n
        idxs = [(start_idx + i) % n for i in range(2 * half_fov_idx + 1)]
        fov = np.array([scan[i] for i in idxs], dtype=float)

        # Treat invalid/no-return readings as "clear" rather than "obstacle at 0"
        valid_fov = np.where(fov > MIN_VALID_DIST, fov, RANGE)
        valid_fov = np.clip(valid_fov, 0, RANGE)

        # Smooth to avoid chasing a single noisy spike -- each candidate ray now
        # represents the average over a full RAY_WIDTH_DEG-wide window
        ray_half_idx = int(RAY_WIDTH_DEG / 2 * SAMPLES_PER_DEGREE)
        smoothed = smooth(valid_fov, ray_half_idx)

        # Find the farthest direction. Many rays can tie (e.g. capped at RANGE in
        # open space), so among all near-max candidates pick the one closest to
        # straight ahead -- keeps the car driving straight in open areas instead
        # of drifting toward whichever tied index happens to come first.
        max_dist = float(np.max(smoothed))
        tie_tol = TIE_TOL_FRAC * RANGE
        candidate_idxs = np.where(smoothed >= max_dist - tie_tol)[0]
        best_idx = int(candidate_idxs[np.argmin(np.abs(candidate_idxs - center_idx))])

        target_deg = (best_idx - center_idx) / SAMPLES_PER_DEGREE  # signed, 0 = straight ahead
        angle = rc_utils.clamp((target_deg / (LONGEST_FOV_DEG / 2)) * STEER_GAIN, -1, 1)

        # Slew-rate limit: cap how much angle can change in one frame
        angle = rc_utils.clamp(angle, prev_angle - MAX_ANGLE_DELTA, prev_angle + MAX_ANGLE_DELTA)
        prev_angle = angle

        chosen_dist = float(valid_fov[best_idx])
        speed_from_angle = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.2, saturate=True)
        speed_from_dist = rc_utils.remap_range(chosen_dist, 20, RANGE, 0.2, 1, saturate=True)
        speed = min(speed_from_angle, speed_from_dist)  # slow for whichever is more conservative

        print(f"{best_idx=}, {target_deg=}, {chosen_dist=}, {angle=}")
    print(f"{speed=}, f{angle=}")

    # Crash protection
    # closest_left_dist, left_angle = rc_utils.get_lidar_closest_point(scan, (240, 360))
    # closest_right_dist, right_angle = rc_utils.get_lidar_closest_point(scan, (0, 120))

    # if closest_left_dist < 20 and closest_right_dist < 20 and left_angle > 340 and right_angle < 320:
    #     angle = 0
    #     speed = -1
    # elif closest_left_dist < 20:
    #     speed = -1
    # elif closest_right_dist < 20:
    #     speed = -1
    # if max(scan)

    if max(scan) < 10:
        speed = -1
    

    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    update_contour(True)

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
