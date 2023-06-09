import sensor, image, time, ustruct
from pid import PID
from pyb import UART, LED
import pyb
import math

# LED(1).on()
# LED(2).on()
# LED(3).on()

THRESHOLD = (0, 25, -128, 127, -128, 127)  # Grayscale threshold for dark things...
THRESHOLD2 = (0, 30, -128, 127, -128, 127)  # Grayscale threshold for dark things...

crop_bottom = 15  # 裁剪底部30行像素
rho_pid = PID(p=0.17, i=0, d=0)  # y=ax+b b截距
theta_pid = PID(p=0.001, i=0, d=0)  # a斜率

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)  # 80x60 (4,800 pixels)
sensor.skip_frames(time=2000)
clock = time.clock()
uart = pyb.UART(3, 115200)  # p4/p5 are UART3
uart.init(115200, bits=8, parity=None, stop=1)

# 识别区域·1 11111111111111111111111111111111111111111
roi1 = [(0, 0, 80, 20),  # 上 x y w h
        (0, 20, 80, 40)]  # 下

tx = bytearray([0x7B, 0x00, 0x00, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x6A, 0x7D])

def data_bytes(data):
    highByte = (data >> 8) & 0xFF
    lowByte = data & 0xFF
    return highByte, lowByte

while (True):
    clock.tick()
    flag = 0

    img = sensor.snapshot().binary([THRESHOLD])  # 线设置为白色，其他为黑色
    img.dilate(3)  # 对二值图像进行膨胀操作

    blobs = img.find_blobs([(255, 255)], area_threshold=20, pixels_threshold=5, merge=True)
    if blobs:
        for blob in blobs:
            if blob.w() < 10 or blob.h() < 10 or blob.area() < 20:
                img.draw_rectangle(blob.rect(), color=(0, 0, 0), fill=True)



    line_1 = img.get_regression([(100, 100)], roi=roi1[1], robust=True)
    line_2 = img.get_regression([(100, 100)], roi=roi1[0], robust=False)

    for rec in roi1:
        img.draw_rectangle(rec, color=(255, 0, 0))  # 绘制出roi区域


    if(line_2):
        img.draw_line(line_2.line(), color=(0,255,0))  # 画出蓝色直线
    else:
        pass

    if line_1 and line_2:
        angle_diff = abs(line_1.theta() - line_2.theta())
        img.draw_string(img.width() - 20, img.height() - 10, str(angle_diff), color=(255, 255, 255), scale=1.2)



    # 检测左转、右转和直行
    if (line_1):
        rho_err = abs(line_1.rho()) - img.width() / 2  # 直线偏移距离
        if line_1.theta() > 90:  # 直线角度
            theta_err = line_1.theta() - 180
        else:
            theta_err = line_1.theta()
        img.draw_line(line_1.line(), color=(0,0,255))  # 画出蓝色直线

        if line_1.magnitude() > 8:  # 线性回归效果好进行下一步，否则不进行
            rho_output = rho_pid.get_pid(rho_err, 1)
            theta_output = theta_pid.get_pid(theta_err, 1)
            xw = rho_output + theta_output
        else:
            xw = 0.0
    else:
        xw = -404.0

    tx[5], tx[6] = data_bytes(int(xw * 1000))
    tx[3], tx[4] = data_bytes(int(flag * 1000))
    uart.write(tx)
