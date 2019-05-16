import numpy as np
import cv2
import matplotlib.pyplot as plot
import scipy.signal as sig
from utils import calculate_center
import test_files

size_x = 860
size_y = 540

lowThreshold = 120
ratio = 4
kernel_size = 3

dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

frame = cv2.imread(test_files.data_tr1)
frame = cv2.resize(frame, (size_x, size_y))

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

mark_value = []

plot.figure()
plot.imshow(src_processed)

plot.figure()
plot.subplot(2, 1, 1)
plot.plot(img_sum_av[0], "-D", markevery=[calculate_center(img_sum_av[0])])
plot.subplot(2, 1, 2)
plot.plot(img_sum_av[1], "-D", markevery=[calculate_center(img_sum_av[1])])
plot.show()

cv2.waitKey(0)
