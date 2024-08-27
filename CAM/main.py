# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

# --------------------------------导入外部文件中的包和模块减↓↓↓----------------------------
import sensor, image,time,display    # 导入摄像头传感器，图像，显示器相关包
from pyb import UART,Pin,Timer
from enum import Enum
from pid import PID # 从pyb包中导入Pin模块
from LQ_Module import motor     # 从LQ_Module文件中导入motor

uart = UART(3, 115200)
# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
red_threshold = [
    (15, 65, 37, 59, 11, 46)   #红色，请在实际使用场景中采集
]
green_threshold = [
    (0, 82, 60, -20, -18, 49)   #绿色，请在实际使用场景中采集
]

speed_L = 0         # 左轮速度暂存全局变量（各电机的实际速度值：基准±偏差值）
speed_R = 0         # 右轮速度暂存全局变量
speed_B = 0         # 后轮速度暂存全局变量

# ------------------------------定义各状态以及相关变量↓↓↓--------------------------------------
ROTATE_SPEED = 900
FORWARD_SPEED = 1200

RED_THRESHOLD = 60
CLOSE_RED_THRESHOLD = 85

IMG_WIDTH = 125
IMG_HEIGHT = 140

CENTRE_THRESHOLD = 15

class State(Enum):
    INITIAL = 0
    SEARCHING = 1
    MOVING_FORWARD = 2
    ADJUSTING_LEFT = 3
    ADJUSTING_RIGHT = 4
    DRIBBLING = 5
    SHOOTING = 6

state = State.INITIAL

#===============================各个外设初始化↓↓↓========================================

# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(True)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 设置图像大小 128*160
sensor.skip_frames(time = 2000)     # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)         # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)     # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪

# ================================= 程序主循环 ==========================================

def find_ball(img):
    red_blobs = img.find_blobs(red_threshold,pixels_threshold=100, area_threshold=100, merge=True)
    
    if red_blobs:
        img.draw_rectangle(red_blobs[0].rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(red_blobs[0].cx(), red_blobs[0].cy(),color=(0, 0, 255))  # 根据色块位置在中心画蓝色十字
    
    return red_blobs
    
def find_goal(img):
    green_blobs = img.find_blobs(green_threshold,pixels_threshold=100, area_threshold=100, merge=True)
    
    if green_blobs:
        img.draw_rectangle(green_blobs[0].rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(green_blobs[0].cx(), green_blobs[0].cy(),color=(0, 0, 255))  # 根据色块位置在中心画蓝色十字   
    
    return green_blobs

def find_tyre(img):
    return True

while(True):
        img = sensor.snapshot()             # 获取一帧图像
        red_blobs = find_ball(img)
        green_blobs = find_goal(img)
        # find_tyre(img)
        
        if (state == State.INITIAL):
            pass
        elif (state == State.SEARCHING):
            speed_L = -900
            speed_R = -900
            speed_B = -900
            if red_blobs:
                state = State.MOVING_FORWARD


        print(speed_L,speed_R,speed_B) # 串行终端打印，偏差和最终电机输出值

        # 放在 while() 里！
        data = [int(speed_L), int(speed_R), speed_B]

        uart.write(str(data) + '\n')
        time.sleep_ms(50)

