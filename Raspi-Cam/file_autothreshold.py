import numpy as np
import cv2
import matplotlib.pyplot as plot
import scipy.signal as sig
from utils import calculate_center, calculate_error, calculate_direction
import test_files

size_x = 860
size_y = 540

step_size = 20

ratio = 4
kernel_size = 3

dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

frame = cv2.imread(test_files.data_tlt1)
frame = cv2.resize(frame, (size_x, size_y))

threshold_list = []
min_error_upper = 99999999
min_error_lower = 99999999
min_error_upper_threshold = 0
min_error_lower_threshold = 0

for lowThreshold in range(0, 255, step_size):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src = cv2.Canny(grey, lowThreshold, lowThreshold * ratio, apertureSize=kernel_size)
    src_processed = cv2.morphologyEx(src, cv2.MORPH_CLOSE, dilate_kernel, iterations=1)
    crop_img = []
    img_sum_av = []
    img_sum_bi = []

    for i in range(1, 3):
        crop_y_start = int(((i-1) / 2) * size_y)
        crop_y_end = int((i / 2) * size_y)
        sliced_img = src_processed[crop_y_start:crop_y_end, 0:size_x]
        crop_img.append(sliced_img)
        img_sum_av.append(sig.savgol_filter(np.sum(sliced_img, axis=0), 101, 3))
    center_upper = calculate_center(img_sum_av[0])
    center_lower = calculate_center(img_sum_av[1])
    error_upper = calculate_error(img_sum_av[0], center_upper)
    error_lower = calculate_error(img_sum_av[1], center_lower)
    display_graph = False
    if (center_upper is not 0) and (center_lower is not 0):
        if (error_upper > 0) and (error_lower > 0):
            display_graph = True
            threshold_list.append(lowThreshold)
            if min_error_upper > error_upper:
                min_error_upper = error_upper
                min_error_upper_threshold = lowThreshold
            if min_error_lower > error_lower:
                min_error_lower = error_lower
                min_error_lower_threshold = lowThreshold
    if display_graph is True:
        center_upper = calculate_center(img_sum_av[0])
        center_lower = calculate_center(img_sum_av[1])

        direction_upper, direction_lower = calculate_direction([center_upper, center_lower], img_sum_av[0].size)

        mark_value = []

        plot.figure()
        plot.subplot(3, 1, 1)
        plot.title("threshold = {}".format(lowThreshold))
        plot.imshow(src_processed)
        plot.subplot(3, 1, 2)
        plot.title("Direction = {}".format(direction_upper))
        plot.plot(img_sum_av[0], "-D", markevery=[center_upper])
        plot.subplot(3, 1, 3)
        plot.title("Direction = {}".format(direction_lower))
        plot.plot(img_sum_av[1], "-D", markevery=[center_lower])
        plot.subplots_adjust(top=0.85)
        plot.show()
# print("Ideal threshold for upper side: {}".format(min_error_upper_threshold))
# print("Ideal threshold for lower side: {}".format(min_error_lower_threshold))
