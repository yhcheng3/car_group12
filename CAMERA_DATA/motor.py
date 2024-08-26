"""
# 调用示例,
注意：需了解 所用管脚pin_pwm为定时器timer几的通道chl几

# 从自定义模块LQ_Motor中导入两个自定义电机对象
import LQ_Motor
from LQ_Motor import motor,motor_brushless   # 不使用此句则需要在对象前加 模块名. LQ_Motor.motor(...)

#参数(self, timer, chl, freq, pin_pwm, pin_io)

motor_L = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")


motor_L.run_percent(cnt)   # PWM 百分比 计
motor_L.run(cnt)           # 实际脉宽24000满

#LQ 无刷驱动 在50Hz 下1~2ms范围2000~4000, 算上驱动死区可控范围约在 2240 ~3800

M1= motor_brushless(4, 1, 50, "P7")  #参数(self, timer, chl, freq, pin_pwm)
M1.run(2300)

"""
import pyb,time,sensor
from pyb import UART, Pin, Timer,ExtInt


uart = UART(3, 115200)


# --------------------------------设置初始变量参数↓↓↓--------------------------------------
# 设置要寻找的线的阈值的阈值（色块法）
red_threshold = [
                 (0, 100, 28, 127, -12, 49)#红色，
]
green_threshold = [
                 (30,100,-55,-20,-10,71) #绿色
]

min_speed = 800   # 最小速度（电机的死区，低于这个值电机不会启动）
speed = 1200        # 基准速度（控制整体前进速度，小车运行时的基本速度）

speed_L = 0         # 左轮速度暂存全局变量（各电机的实际速度值：基准±巡线偏差值）
speed_R = 0         # 右轮速度暂存全局变量
speed_B = 0         # 后轮速度暂存全局变量

turn_factor = 20
err_thres = 8

#======各个外设初始化↓↓↓==========================
#

# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(True)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 设置图像大小 128*160
sensor.skip_frames(time = 2000)     # 跳过前两秒图像后，关闭自动增益白平衡
sensor.set_auto_gain(False)         # 必须关闭自动曝光才能进行相对稳定的颜色跟踪
sensor.set_auto_whitebal(False)     # 必须关闭自动白平衡才能进行相对稳定的颜色跟踪

# -----------------------------自定义函数↓↓↓-------------------------------------
# 在传入色块对象中找到面积最大的色块返回
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:  #w*h
            blob_max = blob
            max_size = blob[2]*blob[3]
    return blob_max
# -----------------------------自定义函数↓↓↓-------------------------------------
#任务1:小车找到绿色球门并且进入
def task_one(color_threshold):
    img = sensor.snapshot()             # 获取一帧图像
    # 使用img.find_blobs()函数获取图像中的各个色块,blobs即为获取到的色块对象，roi为感兴区域[x,y,w,h]，即只在这个范围内查找
    blob = img.find_blobs(color_threshold,merge=True)
    global speed_B, speed_L, speed_R #全局变量
    if blob:                   # 找到追踪目标
        blob_max = find_max(blob)  # 提取面积最大的一个颜色色块blob
        img.draw_rectangle(blob_max.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(blob_max.cx(), blob_max.cy(),color=(0, 0, 255))  # 根据色块位置在质心画蓝色十字
        x_error = blob_max.cx()-img.width()/2                       # 计算色块中心偏差x_error
        y_error = blob_max.cy()-img.height()/2
        if y_error < -20:
            #距离较远，直行靠近
            speed_L = speed
            speed_R = speed
            speed_B = 0
        else:
            speed_L = speed + x_error * turn_factor            # 控制电机转速进行循迹，乘以放大系数，系数越大转向越迅速
            speed_R = -speed + x_error * turn_factor           # 基准速度+偏差

            if x_error > err_thres:                           # 当偏差超过这个值，后轮才会辅助转向
                speed_B = min_speed + x_error * turn_factor    # 控制后轮电机转速协助转弯，乘以放大系数，系数越大转向越迅速
            elif x_error < -err_thres:
                speed_B = -min_speed + x_error * turn_factor    # 控制后轮电机转速协助转弯
            else:
                speed_B = 0
            #print(x_error, speed_L,speed_R,speed_B) # 串行终端打印，偏差和最终电机输出
    else:
        #rotate to look for target
        speed_L = min_speed
        speed_R = min_speed
        speed_B = min_speed

        return


# ================== 程序主循环 =======================
while True:
    task_one(green_threshold) #找球门
    data = [speed_L,speed_R,speed_B]
    uart.write(str(data)+'\n')
    #print(data)
    time.sleep_ms(300)

