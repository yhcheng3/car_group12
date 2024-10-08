"""
调用示例,
注意：需了解 所用管脚pin_pwm为定时器timer几的通道chl几

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

err_thres = 20       # 用于判断是否需要后轮转向
kick_thres = 90     # 用于判断是否直行踢球

#======各个外设初始化↓↓↓==========================
#

# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()                      # 初始化摄像头
sensor.set_hmirror(False)            # 镜像（如果视觉模块倒着安装，则开启这个镜像）
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

# 对（球门）色块旋转角度进行去噪
def rotation_deg_filtered(blob):
    degree= [0] * 5
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


# -----------------------------自定义函数↓↓↓-------------------------------------
#任务1:小车检测到红色小球，靠近小球，往任意方向踢均可；
def task_one(img,color_threshold):
    # 使用img.find_blobs()函数获取图像中的各个色块,blobs即为获取到的色块对象，roi为感兴区域[x,y,w,h]，即只在这个范围内查找
    blob = img.find_blobs(color_threshold,merge=True)
    global speed_B, speed_L, speed_R #全局变量
    if blob:                   # 找到追踪目标
        blob_max = find_max(blob)  # 提取面积最大的一个颜色色块blob
        img.draw_rectangle(blob_max.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(blob_max.cx(), blob_max.cy(),color=(0, 0, 255))  # 根据色块位置在质心画蓝色十字
        x_error = blob_max.cx()-img.width()/2                       # 计算色块中心偏差x_error
        
        if abs(x_error) < err_thres and blob_max.w() > kick_thres:  #直行踢球
            speed_L = speed * 2
            speed_R = -speed * 2
            speed_B = 0
        else:   #左右转向找球
            if x_error > err_thres:       # 当偏差超过这个值，后轮才会辅助转向
                # 左轮带动小车向右转
                speed_L = speed
                speed_R = -min_speed -100
                speed_B = min_speed 
            elif x_error < -err_thres:
                #右轮转
                speed_L = min_speed +100 
                speed_R = -speed
                speed_B = -min_speed 
            else:
                #直行
                speed_B = 0
                speed_L = speed
                speed_R = -speed
            print(x_error) # 串行终端打印，偏差和最终电机输出
    else:
        #rotate to look for target
        speed_L = min_speed
        speed_R = min_speed
        speed_B = min_speed

    print(speed_L, speed_R, speed_B)  # 串行终端打印最终电机输出

    return


#任务2:小车能检测到绿色球门，小车把自己送进球门
def task_two(img,color_threshold):
    blob = img.find_blobs(color_threshold,area_threshold = 80, merge=True)
    global speed_B, speed_L, speed_R #全局变量
    if blob:                   # 找到追踪目标
        blob_max = find_max(blob)  # 提取面积最大的一个颜色色块blob
        img.draw_rectangle(blob_max.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
        img.draw_cross(blob_max.cx(), blob_max.cy(),color=(0, 0, 255))  # 根据色块位置在质心画蓝色十字
        x_error = blob_max.cx()-img.width()/2                       # 计算色块中心偏差x_error
        y_error = blob_max.cy()-img.height()/2
        print(x_error)
        if blob_max.w() > 123 and blob_max.h() > 130 and blob_max.compactness() > 0.8:
            #进入球门，为避免冲撞停止运动
            speed_B = 0
            speed_L = 0
            speed_R = 0
        else:
            deg_filtered = rotation_deg_filtered(blob_max)
            if deg_filtered <= 150 and deg_filtered >= 100:
                #车在球门左侧
                uart.write(str([int(3000),int(3000),int(3000)])+'\n')
                time.sleep_ms(50)
                uart.write(str([int(3000),int(-3000),int(0)])+'\n')
                time.sleep_ms(50)
                uart.write(str([int(-3000),int(-3000),int(-3000)])+'\n')
                time.sleep_ms(50)

            elif deg_filtered < 100 and deg_filtered >= 20:
                #在右侧
                uart.write(str([int(-3000),int(-3000),int(-3000)])+'\n')
                time.sleep_ms(50)
                uart.write(str([int(3000),int(-3000),int(0)])+'\n')
                time.sleep_ms(50)
                uart.write(str([int(3000),int(3000),int(3000)])+'\n')
                time.sleep_ms(50)
            else:
                if x_error < -err_thres:  # 当偏差超过这个值，后轮才会辅助转向
                    #右轮带动小车向左转
                    speed_L = min_speed +100 
                    speed_R = -speed
                    speed_B = -min_speed 
                elif x_error > err_thres: 
                    #左轮转
                    speed_L = speed
                    speed_R = -min_speed -100
                    speed_B = min_speed 
                else:
                    speed_B = 0
                    speed_L = speed
                    speed_R = -speed
            print('rotation:', deg_filtered)
    else:
         #rotate to look for target
        speed_L = min_speed
        speed_R = min_speed
        speed_B = min_speed
    print(speed_L, speed_R, speed_B)  # 串行终端打印最终电机输出

    return

# ================== 程序主循环 =======================
while True:
    img = sensor.snapshot()             # 获取一帧图像
    task1_flag = 0
    task2_flag = 1
    if task1_flag:
        task_one(img,red_threshold) #踢球
    if task2_flag:
        task_two(img,green_threshold) #找球门
    data = [int(speed_L),int(speed_R),int(speed_B)]
    uart.write(str(data)+'\n')
    print(data)
    time.sleep_ms(50)
