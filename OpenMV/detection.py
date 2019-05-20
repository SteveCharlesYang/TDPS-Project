# Untitled - By: admin - 周六 五月 18 2019

import sensor, image, time
import cv2
import numpy as np
from skimage import measure
import matplotlib.pyplot as plt
from pyb import USB_VCP()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()
#蓝红绿HSV值
hsv_low_blue = [100,80,80]
hsv_high_blue = [124,180,140]
hsv_low_red = [0,80,80]
hsv_high_red = [10,180,140]
hsv_low_green = [35,80,80]
hsv_high_green =[77,180,140]
#计算某种颜色在图片内的比例
def calculateRateOfColor(img,lower_hsv,upper_hsv,rate):
    kernel = np.ones((17,17),np.uint8)
    HSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(HSV, lowerb=lower_hsv,upperb=upper_hsv)#得到分界明显的图片
    h,w = img.shape()
    numOfColor = 0
    for i in range(h):
        for j in range(w):
            if img.item(i,j,0)> lower_hsv[0] and img.item(i,j,0)>upper_hsv[0]:
                if img.item(i,j,1)> lower_hsv[1] and img.item(i,j,0)>upper_hsv[1]:
                    if img.item(i,j,2)> lower_hsv[2] and img.item(i,j,0)>upper_hsv[2]:
                        numOfColor += 1
    rate = numOfColor/(h*w)
#在图片中找到颜色的位置的中心点 如果在大图片中心横坐标正负100像素内则算在中线内
def calculateRange(image, lower_color, upper_color):
    blur_image = cv2.GaussianBlur(image, (5, 5), 0)
    # 转换HSV颜色空间
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    kernel_size = (20,20)
    lower = np.array(lower_color, np.uint8)
    upper = np.array(upper_color, np.uint8)
    # 得到二值图像，在lower~upper范围内的值为255，不在的值为0
    mask = cv2.inRange(image, lower, upper)
    # 进行腐蚀和膨胀
    if kernel_size:
        kernel=cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
        dilated = cv2.dilate(mask, kernel)
        eroded = cv2.erode(dilated, kernel)
    img = eroded
    # 找连通域
    labels = measure.label(img, connectivity=2)
    pros = measure.regionprops(labels)
    #色块中心坐标
    central = props[0].centroid
    #w为图片横坐标
    h,w = image.shape()
    if central >(w*0.5-100):
        if central <(w*0.5+100):
            return 1
        else:
            return 0
while(True):
    clock.tick()
    img = sensor.snapshot()
    numb = 0
    rateOfBlue = 0
    rateOfRed = 0
    rateOfGreen = 0
    #查看图片中是否存在该种颜色（占图片中颜色5%）
    calculateRateOfColor(img,hsv_low_red,hsv_high_red,rateOfRed)
    calculateRateOfColor(img,hsv_low_green,hsv_high_green,rateOfGreen)
    calculateRateOfColor(img,hsv_low_blue,hsv_high_blue,rateOfBlue)
    #numb 值为1 所需颜色出现在图片中线  为0不在中线
    if rateOfRed > 0.05：
        numb = calculateRange(img,img,hsv_low_red,hsv_high_red)
    elif rateOfBlue > 0.05:
        numb = calculateRange(img,img,hsv_low_blue,hsv_high_blue)
    elif rateOfGreen > 0.05:
        numb = calculateRange(img,img,hsv_low_green,hsv_high_green)
    else:
        numb = 0
    #图片中有颜色返回True 无该颜色返回False
    if numb == 1:
        USB_VCP.send(True)
    else:
        USB_VCP.send(False)


    print(clock.fps())
