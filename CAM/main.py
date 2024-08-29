# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

# --------------------------------导入外部文件中的包和模块减↓↓↓----------------------------
import sensor, image, time, display    # 导入摄像头传感器，图像，显示器相关包
from pyb import UART, Pin, Timer
from pid import PID                    # 从pyb包中导入Pin模块
from LQ_Module import motor            # 从LQ_Module文件中导入motor

uart = UART(3, 115200)

# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
red_threshold = [
    (0, 100, 28, 127, -12, 49)         # 红色
]
green_threshold = [
    (30, 100, -55, -20, -10, 71)       # 绿色
]
black_threshold = [
    (0, 43, -20, 8, 8, 26)              #黑色
]

speed_L = 0                            # 左轮速度暂存全局变量（各电机的实际速度值：基准±偏差值）
speed_R = 0                            # 右轮速度暂存全局变量
speed_B = 0                            # 后轮速度暂存全局变量

# ------------------------------定义各状态以及相关变量↓↓↓--------------------------------------

RED_WIDTH_TOL = 50                     # 红球宽度容差

SHOOT_PAUSE_1 = 1
SHOOT_PAUSE_2 = 1

states = {
    "INITIAL": 0,                   # 弃用
    "SEARCHING": 1,                 # 寻找红球
    "SEARCHING_LEFT": 2,            # 向左原地旋转，对准红球
    "SEARCHING_RIGHT": 3,           # 向右原地旋转，对准红球
    "FORWARD": 4,                   # 前进（离球太远，或三点一线推着球到球门）
    "BACKWARD": 5,                  # 后退
    "SEARCHING_GREEN": 6,           # 绕球公转，寻找球门
    "ADJUSTING_GREEN": 7,           # 绕球公转，瞄准球门
    "DRIBBLING_FORWARD": 8,
    "DRIBBLING": 9,                 # 带球边前行，边瞄准球门
    # "DRIBBLING_ENEMY": 9,           # 带球边前行，边避开敌方
    "SHOOTING": 10                  # 射门（非推进球门）
}

blackline_states = {
    "BLACK_LINE_SAFE": 0,                  # 没有检测到黑线
    "BLACK_LINE_DETECTED": 1,       # 车在射门的角度范围内，且检测到黑线距离过近
    "CAR_TO_LEFT": 2,               # 车在球门右侧（需左转）
    "CAR_TO_RIGHT": 3,              # 车在球门的左侧（需右转）
}

prev_state = states["SEARCHING"]
state = states["SEARCHING"]

shoot_cnt = 0                       # 射门条件满足计数
shoot_tick = 0                      # 射门计时
shoot_length = 70                   # 射门持续时间

searching_tick = 0                  # 寻找红球计时
searching_flag = 0                  # 寻找红球标志

adjusting_tick = 0                  # 瞄准球门计时 (SEARCHING_GREEN)
adjusting_flag = 0                  # 瞄准球门标志

red_blob = None
green_blob = None

#===============================各个外设初始化↓↓↓========================================

# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(False)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 设置图像大小 128*160
sensor.skip_frames(time = 2000)     # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)         # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)     # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪

IMG_WIDTH = sensor.width()
IMG_HEIGHT = sensor.height()

# ================================= 程序主循环 ==========================================

def sign(x):
    return (x > 0) - (x < 0)

def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob.w() * blob.h() > max_size:
            max_blob = blob
            max_size = blob.w() * blob.h()
    return max_blob

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

def find_ball(img):
    red_blobs = img.find_blobs(red_threshold,pixels_threshold=10, area_threshold=10, merge=True)

    if red_blobs:
        img.draw_rectangle(red_blob.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(red_blob.cx(), red_blob.cy(),color=(0, 0, 255))  # 根据色块位置在中心画蓝色十字

    return red_blobs

def find_goal(img):
    green_blobs = img.find_blobs(green_threshold,pixels_threshold=100, area_threshold=100, merge=True)

    if green_blobs:
        img.draw_rectangle(green_blob.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(green_blob.cx(), green_blob.cy(),color=(0, 0, 255))  # 根据色块位置在中心画蓝色十字

    return green_blobs

def find_black_line(img):
    black_line_flag = 0
    green_blobs = img.find_blobs(green_threshold, pixels_threshold=10, area_threshold=10, merge=True)

    if green_blobs:
        green_blob_max = find_max(green_blobs)
        img.draw_rectangle(green_blob_max.rect(), color=(255, 0, 0))  # 对绿区域画红框
        deg_filtered = rotation_deg_filtered(green_blob_max)
        print('deg:', deg_filtered)
        if deg_filtered <= 150 and deg_filtered >= 100:
            # 车在球门左侧
            black_line_flag = 2  # 车在球门左侧

        elif deg_filtered < 100 and deg_filtered >= 30:
            # 在右侧
            black_line_flag = 3

        else:
            black_blobs = img.find_blobs(black_threshold, area_threshold=10, merge=True, margin = 1)
            if black_blobs:
                black_blob_max = find_max(black_blobs)
                img.draw_line(black_blob_max.major_axis_line(), color=(0, 0, 255))
                print('elongation:', black_blob_max.elongation(), 'compactness:', black_blob_max.compactness())
                #print('cy', black_blob_max.cy(),'y', black_blob_max.y())
                coordinates = black_blob_max.major_axis_line()  #(x1,y1,x2,y2)
                if black_blob_max.elongation() >= 0.9 and black_blob_max.cy() >=130 :
                    img.draw_line(black_blob_max.major_axis_line(), color=(0, 255, 0))
                    black_line_flag = 1
                elif 0.7<= black_blob_max.elongation() <0.9  and max(coordinates[1],coordinates[3]) >=140:
                    img.draw_line(black_blob_max.major_axis_line(), color=(0, 255, 0))
                    black_line_flag = 1
                else:
                    black_line_flag = 0

    return black_line_flag

def state_switch(red_blobs, green_blobs, black_line):
    global prev_state, state, shoot_cnt, shoot_tick, searching_tick, searching_flag, adjusting_tick, adjusting_flag
    
    global red_blob, green_blob

    is_shoot = False

    prev_state = state

#    if black_line == blackline_states["CAR_TO_LEFT"]:
#        state = states["SEARCHING_RIGHT"] #向右转，优先小车远离球门

#    elif black_line == blackline_states["CAR_TO_RIGHT"]:
#        state = states["SEARCHING_LEFT"]

    if red_blobs:
        red_blob = find_max(red_blobs)
        
    if green_blobs:
        green_blob = find_max(green_blobs)
    
    if black_line == blackline_states["BLACK_LINE_DETECTED"]:
        state = states["BACKWARD"]

    else:
        if state == states["SHOOTING"]:
            state = states["SHOOTING"]

        elif not red_blobs:
            state = states["SEARCHING"]
        else:
            if 0 <= (red_blob).cx() <= (IMG_WIDTH/2 - RED_WIDTH_TOL):
                state = states["SEARCHING_LEFT"]

            elif (IMG_WIDTH/2 + RED_WIDTH_TOL) < (red_blob).cx():
                state = states["SEARCHING_RIGHT"]

            else:
                # 找到红球，且在中间
                if not green_blobs:
                    if (red_blob).w() <= 50:
                        state = states["FORWARD"]

                    elif (red_blob).w() > 70:
                        state = states["BACKWARD"]
                    else:
                        state = states["SEARCHING_GREEN"]

                else:
                    # 红球在中间，且找到球门，但球可能很远
                    # if black_line == blackline_states["CAR_TO_LEFT"]:
                    #     state = states["SEARCHING_RIGHT"] #向右转，优先小车远离球门
                    # elif black_line == blackline_states["CAR_TO_RIGHT"]:
                    #     state = states["SEARCHING_LEFT"]
                    if black_line == blackline_states["BLACK_LINE_DETECTED"]:
                        state = states["BACKWARD"]
                    else:
                        if abs((green_blob).cx() - IMG_WIDTH/2) < 25:
                            # 球门在正中间
                            if ((green_blob).w() > 90 and
                                (green_blob.rotation_deg() >= 150 or green_blob.rotation_deg() <= 30)):

                                if shoot_cnt >= 5:
                                    state = states["SHOOTING"]
                                else:
                                    state = states["DRIBBLING_FORWARD"]
                                    is_shoot = True
                            else:
                                state = states["DRIBBLING_FORWARD"]

                        elif abs((green_blob).cx() - IMG_WIDTH/2) < 40:
                            # 球门靠近中间
                            # if ((green_blob).w() > 90 and
                            #     (green_blob.rotation_deg() >= 150 or green_blob.rotation_deg() <= 30)):
                            #     state = states["SHOOTING"]
                            # else:
                            state = states["DRIBBLING"]

                        else:
                            # 球门不在中间
                            state = states["ADJUSTING_GREEN"]
    if is_shoot == True:
        shoot_cnt += 1
    else:
        shoot_cnt = 0

    # 射门状态计时
    if state != states["SHOOTING"]:
        shoot_tick = 0

    # 搜索红球时碰到白墙，切换搜索方向
    if state == states["SEARCHING"]:
        if searching_tick >= 300:
            searching_flag = 1 - searching_flag
            searching_tick = 0
        else:
            searching_tick += 1
    else:
        searching_tick = 0

    # 搜索球门时碰到白墙，切换搜索方向
#    if (state == states["SEARCHING_GREEN"] or state == states["BACKWARD"] or
#       state == states["FORWARD"] or state == states["SEARCHING_LEFT"] or state == states["SEARCHING_RIGHT"]):
#        if adjusting_tick >= 300:
#            adjusting_flag = 1 - adjusting_flag
#            adjusting_tick = 0
#        else:
#            adjusting_tick += 1
#    else:
#        adjusting_tick = 0

while (True):
        img = sensor.snapshot()             # 获取一帧图像
        red_blobs = find_ball(img)
        green_blobs = find_goal(img)
        black_line = find_black_line(img)

        state_switch(red_blobs, green_blobs, black_line)

        # 先变状态，后定义行为
        # speed赋值放else里面，减少延时
        if (state == states["SEARCHING"]):
            # 原地旋转，寻找红球
            if searching_flag == 0:
                speed_L = -600
                speed_R = -600
                speed_B = -600
            elif searching_flag == 1:
                speed_L = 600
                speed_R = 600
                speed_B = 600

        elif (state == states["SEARCHING_LEFT"]):
            # 无论红球远近，向左原地旋转，对准红球，使其在中间
            speed_L = -600
            speed_R = -600
            speed_B = -600

        elif (state == states["SEARCHING_RIGHT"]):
            # 无论红球远近，向右原地旋转，对准红球，使其在中间
            speed_L = 600
            speed_R = 600
            speed_B = 600

        elif (state == states["FORWARD"]):
            # 红球在中间，离球远，且还没找到球门；前进
            speed_L = 2500
            speed_R = -2500
            speed_B = 0

        elif (state == states["BACKWARD"]):
            # 红球在中间，球太近，且还没找到球门；后退
            speed_L = -1000
            speed_R = 1000
            speed_B = 0

        elif (state == states["SEARCHING_GREEN"]):
            # 红球在中间，球距离刚好；还没找到球门
            # if adjusting_flag == 0:
            speed_L = 0
            speed_R = 0
            speed_B = 2300
#            else:
#                speed_L = 0
#                speed_R = 0
#                speed_B = 4000

        elif (state == states["ADJUSTING_GREEN"]):
            # 红球在中间，球距离刚好；找到球门，公转
            speed_L = 0
            speed_R = 0
            speed_B = sign((green_blob).cx()-(red_blob).cx()) * 4000

        elif (state == states["DRIBBLING_FORWARD"]):
            # 红球在中间，球距离刚好；找到球门，球门在中间，前进
            speed_L = 2500
            speed_R = -2500
            speed_B = 0

        elif (state == states["DRIBBLING"]):
            # 红球在中间，球距离刚好；找到球门，球门足够靠中心
            speed_L = 2500
            speed_R = -2500
            speed_B = -20*((green_blob).cx()-(red_blob).cx())

        elif (state == states["SHOOTING"]):
            # 终结状态；必须经过DRIBBLING状态，才能进入SHOOTING状态
            # 有球，球够近；球靠中心；有球门；球门足够靠中心；球门没偏，不会射到门框；没敌人
            shoot_tick += 1

            if 0 <= shoot_tick < SHOOT_PAUSE_1:
                if shoot_tick == 0:
                    if green_blobs:
                        if (green_blob.w()) > 90:
                            shoot_length = 2
                        else:
                            shoot_length = 5
                    else:
                        shoot_length = 50
                speed_L = 0
                speed_R = 0
                speed_B = 0

            elif shoot_tick < SHOOT_PAUSE_1 + shoot_length:
                speed_L = 6000
                speed_R = -6000
                speed_B = 0

            elif shoot_tick < SHOOT_PAUSE_1 + shoot_length + SHOOT_PAUSE_2:
                speed_L = 0
                speed_R = 0
                speed_B = 0

            elif shoot_tick < SHOOT_PAUSE_1 + 2*shoot_length + SHOOT_PAUSE_2:
                speed_L = -6000
                speed_R = 6000
                speed_B = 0

            else:
                state = states["SEARCHING"]
                shoot_tick = 0

        else:
            prev_state = states["SEARCHING"]
            state = states["SEARCHING"]

        print("Speed: ", speed_L,speed_R,speed_B) # 串行终端打印，偏差和最终电机输出值
        for key, value in states.items():
            if value == state:
                print("State: ", key)
                break

        print(IMG_WIDTH, IMG_HEIGHT)

        # 注意避免状态前后跳变死循环（如：DRIBBLING->DRIBBLING_ENEMY->DRIBBLING）。否则，电机不会输出
        # TODO: increase efficiency by setting isChanged flag, in lieu of prev_state
        if (prev_state != state):
            # 状态变化，不输出给电机，减少延时
            prev_state = state
        else:
            data = [int(speed_L), int(speed_R), int(speed_B)]
            uart.write(str(data) + '\n')
            time.sleep_ms(50)

