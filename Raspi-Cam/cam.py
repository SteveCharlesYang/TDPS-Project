import numpy as np
import cv2
import matplotlib.pyplot as plot
import scipy.signal as sig
from utils import calculate_center, calculate_direction

size_x = 640
size_y = 480

lowThreshold = 50
ratio = 2
kernel_size = 3

dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

cap = cv2.VideoCapture(2)

plot.figure()
plot.ion()

counter = 0

while True:
    counter = counter + 1
    if counter is 16:
        counter = 0
    ret, frame = cap.read()
    frame = cv2.resize(frame, (size_x, size_y))
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    src = cv2.Canny(grey, lowThreshold, lowThreshold * ratio, apertureSize=kernel_size)
    src_processed = cv2.morphologyEx(src, cv2.MORPH_CLOSE, dilate_kernel, iterations=1)
    crop_img = []
    img_sum_av = []

    cv2.imshow('frame2', src)

    if counter - 8 is 0:
        print("a")
        for i in range(1, 3):
            crop_y_start = int(((i - 1) / 2) * size_y)
            crop_y_end = int((i / 2) * size_y)
            sliced_img = src_processed[crop_y_start:crop_y_end, 0:size_x]
            crop_img.append(sliced_img)
            img_sum_av.append(sig.savgol_filter(np.sum(sliced_img, axis=0), 101, 3))
            # img_sum_av.append(np.sum(sliced_img, axis=0))

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

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# when everything done , release the capture
cap.release()
cv2.destroyAllWindows()
