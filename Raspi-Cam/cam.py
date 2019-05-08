import numpy as np
import cv2
import matplotlib.pyplot as plot
import matplotlib.mlab as mlab
import scipy.signal as sig

size_x = 640
size_y = 480

lowThreshold = 90
ratio = 2
kernel_size = 3

judge_value = 4000

ignore_gap = 150

interval_center = 0

cap = cv2.VideoCapture(0)

plot.figure()
plot.ion()

counter = 0

while True:
    counter = counter + 1
    if counter is 16:
        counter = 0
    # capture frame-by-frame
    ret, frame = cap.read()

    # our operation on the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # src = cv2.Canny(grey, lowThreshold, lowThreshold * ratio)
    src = cv2.Canny(grey, lowThreshold, lowThreshold * ratio, apertureSize=kernel_size)
    crop_img = []
    img_sum_av = []
    img_sum_bi = []

    cv2.imshow('frame2', src)

    if counter - 8 is 0:
        print("a")
        for i in range(1, 3):
            crop_y_start = int(((i-1) / 2) * size_y)
            crop_y_end = int((i / 2) * size_y)
            sliced_img = src[crop_y_start:crop_y_end, 0:size_x]
            crop_img.append(sliced_img)
            img_sum_av.append(sig.savgol_filter(np.sum(sliced_img, axis=0), 101, 3))
            img_sum_bi.append(np.where(sig.savgol_filter(np.sum(sliced_img, axis=0), 101, 3) > judge_value, 1, 0))

        mark_value = []

#     img_sum_bi_gradient = np.gradient(img_sum_bi)
#     change_points_up = np.where(img_sum_bi_gradient > 0)[0]
#     change_points_down = np.where(img_sum_bi_gradient < 0)[0]
#     for i in range(0, len(change_points_up)):
#         interval_value = change_points_down[i] - change_points_up[i]
#         if interval_value > ignore_gap:
#             interval_center = interval_value/2 + change_points_up[i]
#             mark_value.append(change_points_up[i])
#             mark_value.append(change_points_down[i])

        plot.cla()
        plot.subplot(2, 1, 1)
        plot.plot(img_sum_av[0])
        plot.subplot(2, 1, 2)
        plot.plot(img_sum_av[1])
        plot.show()
        # plot.figure()
        # plot.subplot(2, 1, 1)
        # plot.plot(img_sum_bi[0])
        # plot.subplot(2, 1, 2)
        # plot.plot(img_sum_bi[1])
        # plot.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# when everything done , release the capture
cap.release()
cv2.destroyAllWindows()
