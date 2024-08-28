# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

# --------------------------------导入外部文件中的包和模块减↓↓↓----------------------------
import sensor, image, time, display  # 导入摄像头传感器，图像，显示器相关包
from pyb import UART, Pin, Timer  # 从pyb包中导入Pin模块
from pid import PID
import math, time

detect_states = {
    "NO_BLACK": 0,  # 没有检测到黑线
    "BLACK_LINE_DETECTED": 1,  # 车在射门的角度范围内，且检测到黑线距离过近
    "CAR_TO_LEFT": 2,  # 车在球门右侧（需左转）
    "CAR_TO_RIGHT": 3,  # 车在球门的左侧（需右转）

}
uart = UART(3, 115200)
# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
black = (0, 43, -20, 8, 8, 26)  # 黑色
green = (30, 100, -55, -20, -10, 71)  # 绿色
# black_threshold = [black]
mixed_thresholds = [
    black, green
]
# ===============================各个外设初始化↓↓↓========================================
# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()  # 初始化摄像头
sensor.set_hmirror(False)  # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565)  # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)  # 设置图像大小 128*160
# sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)  # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)  # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)  # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪


# -----------------------------自定义函数↓↓↓-------------------------------------
# 在传入色块对象中找到面积最大的色块返回
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.w() * blob.h() > max_size:
            max_blob = blob
            max_size = blob.w() * blob.h()
    return max_blob

# 对（球门）色块旋转角度进行去噪
def rotation_deg_filtered(blob):
    degree = [0] * 5
    max_count = 0
    most_common = degree[0]
    deg_max_err = 10

    for i in range(5):
        deg = blob.rotation_deg()
        degree[i] = deg
        time.sleep_ms(5)

    for i in range(5):
        count = 0
        for j in range(5):
            if i == j:
                continue
            if abs(degree[i] - degree[j]) <= deg_max_err:
                count += 1
        if count > max_count:
            max_count = count
            most_common = degree[i]
    return most_common

#确认球门前的黑线
def find_black_line(img):
    black_line_flag = 0
    green_blobs = img.find_blobs([green], pixels_threshold=10, area_threshold=10, merge=True)

    if green_blobs:
        green_blob_max = find_max(green_blobs)
        img.draw_rectangle(green_blob_max.rect(), color=(255, 0, 0))  # 对绿区域画红框
        deg_filtered = rotation_deg_filtered(green_blob_max)
        print('deg:', deg_filtered)
        if deg_filtered <= 150 and deg_filtered >= 100:
            # 车在球门左侧
            black_line_flag = 2  # 车在球门左侧

        elif deg_filtered < 100 and deg_filtered >= 20:
            # 在右侧
            black_line_flag = 3

        else:
            black_blobs = img.find_blobs([black], area_threshold=10, merge=True, margin = 1)
            if black_blobs:
                black_blob_max = find_max(black_blobs)
                img.draw_line(black_blob_max.major_axis_line(), color=(0, 0, 255))
                print('elongation:', black_blob_max.elongation(), 'compactness:', black_blob_max.compactness())
                #print('cy', black_blob_max.cy(),'y', black_blob_max.y())
                coordinates = black_blob_max.major_axis_line()  #(x1,y1,x2,y2)
                if black_blob_max.elongation() >= 0.9 and black_blob_max.cy() >=130 :
                    print('coordinates', coordinates)
                    img.draw_line(black_blob_max.major_axis_line(), color=(0, 255, 0))
                    black_line_flag = 1
                elif 0.7<= black_blob_max.elongation() <0.9  and max(coordinates[1],coordinates[3]) >=140:
                    print('coordinates', coordinates)
                    img.draw_line(black_blob_max.major_axis_line(), color=(0, 255, 0))
                    black_line_flag = 1
                else:
                    black_line_flag = 0

    return black_line_flag


# ================================= 程序主循环 ==========================================

while (True):
    img = sensor.snapshot()
    black_line_flag = find_black_line(img)
    print('------FLAG------', black_line_flag)
    time.sleep_ms(100)
