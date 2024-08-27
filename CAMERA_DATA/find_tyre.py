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

uart = UART(3, 115200)
# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
tyre_threshold = [
    (15, 65, 37, 59, 11, 46),()   #红色，请在实际使用场景中采集
]
#===============================各个外设初始化↓↓↓========================================
# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(True)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 设置图像大小 128*160
sensor.skip_frames(time = 2000)     # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)         # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)     # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪

# -----------------------------自定义函数↓↓↓-------------------------------------
## 在传入色块对象中找到面积最大的色块返回
#def find_max(blobs):
#    max_size=0
#    for blob in blobs:
#        if blob[2]*blob[3] > max_size:
#            max_blob=blob
#            max_size = blob[2]*blob[3]
#    return max_blob


def find_tyre(color_threshold):
    img = sensor.snapshot()
    circles = img.find_circles(threshold = 3500, x_margin = 10, y_margin = 10, r_margin = 10,r_min = 2, r_max = 100, r_step = 2)
    for c in circles:
        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
        area = (c.x() - c.r(), c.y() - c.r(), 2 * c.r(), 2 * c.r())# area为识别到的圆的区域，即圆的外接矩形框
        statistics = img.get_statistics(roi=area)  # 像素颜色统计
        print(statistics)

# ================================= 程序主循环 ==========================================

while(True):
    find_tyre(tyre_threshold)