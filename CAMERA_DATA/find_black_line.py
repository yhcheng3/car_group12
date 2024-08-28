# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

# --------------------------------导入外部文件中的包和模块减↓↓↓----------------------------
import sensor, image,time,display    # 导入摄像头传感器，图像，显示器相关包
from pyb import UART,Pin,Timer       # 从pyb包中导入Pin模块
from pid import PID
import math,time


uart = UART(3, 115200)
# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
black= (0, 60, -15, 3, -6, 21)  #黑色
green= (30,100,-55,-20,-10,71) #绿色
#black_threshold = [black]
mixed_thresholds = [
    black, green
]
#===============================各个外设初始化↓↓↓========================================
# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(False)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 设置图像大小 128*160
#sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)     # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)         # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)     # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪

# -----------------------------自定义函数↓↓↓-------------------------------------
## 在传入色块对象中找到面积最大的色块返回
def find_max(blobs):
   max_size=0
   for blob in blobs:
       if blob[2]*blob[3] > max_size:
           max_blob=blob
           max_size = blob[2]*blob[3]
   return max_blob

def find_black_line(img):
    black_line_flag = 0
    blobs = img.find_blobs(mixed_thresholds,pixels_threshold=10, area_threshold=10, merge=True)
    if blobs:
        for b in blobs:
            if b.code() == 3: #同时看到绿色和黑色
                img.draw_rectangle(b.rect(), color=(255, 0, 0))#对大区域画红框
                area = b.rect()
                black_blobs = img.find_blobs([black], roi = area, pixels_threshold=10, area_threshold=10, merge=True) #在同时包含绿色和黑色的区域内找黑线
                if black_blobs:
                    black_line = find_max(black_blobs)
                    img.draw_rectangle(black_line.rect(), color=(0, 255, 0))#对黑线画绿框
                    print(black_line.cx(), black_line.cy())
                    black_line_flag = 1
    return black_line_flag
# ================================= 程序主循环 ==========================================

while(True):
    img = sensor.snapshot()
    black_line_flag = find_black_line(img)
    print(black_line_flag)
    time.sleep_ms(50)
