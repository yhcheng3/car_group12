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
BACKWARD_SPEED = -1200

RED_THRESHOLD = 60
CLOSE_RED_THRESHOLD = 85

IMG_WIDTH = 125
IMG_HEIGHT = 140

CENTRE_THRESHOLD = 15

states = {
    "INITIAL": 0,
    "SEARCHING": 1,
    "FORWARD": 2,
    "BACKWARD": 3,           # 后退  
    "ADJUSTING_LEFT": 4,
    "ADJUSTING_RIGHT": 5,    # 绕球顺时针旋转，瞄准球门
    "DRIBBLING": 6,          # 带球边前行，边瞄准球门
    "DRIBBLING_ENEMY": 7,    # 带球边前行，边避开敌方
    "SHOOTING": 8            # 射门（非推进球门）
}
    
prev_state = None
state = states["INITIAL"]

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
        
        if (state == states["INITIAL"]):
            speed_L = 0
            speed_R = 0
            speed_B = 0
        elif (state == states["SEARCHING"]):
            speed_L = -900
            speed_R = -900
            speed_B = -900
            
            if red_blobs:
                state = states["FORWARD"]
        elif (state == states["FORWARD"]):
            pass
        elif (state == states["BACKWARD"]):
            pass
        elif (state == states["ADJUSTING_LEFT"]):
            pass
        elif (state == states["ADJUSTING_RIGHT"]):
            pass
        elif (state == states["DRIBBLING"]):
            pass
        elif (state == states["DRIBBLING_ENEMY"]):
            pass
        elif (state == states["SHOOTING"]):
            pass
        else:
            pass
        
        print(speed_L,speed_R,speed_B) # 串行终端打印，偏差和最终电机输出值
        
        # 注意避免状态前后跳变死循环（如：DRIBBLING->DRIBBLING_ENEMY->DRIBBLING）。否则，电机不会输出
        if (prev_state != state):
            # 状态变化，不输出给电机，减少延时
            prev_state = state
        else:
            data = [int(speed_L), int(speed_R), speed_B]
            uart.write(str(data) + '\n')
            time.sleep_ms(50)

