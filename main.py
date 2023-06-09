import sensor, image, time, ustruct
from pid import PID
from pyb import UART, LED
import pyb
import time
from pyb import Pin, Timer

# 50kHz pin6 timer2 channel1
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(10) # 控制亮度 0~100
#LED(1).on()
#LED(2).on()
#LED(3).on()
THRESHOLD = (0, 20, -128, 127, -128, 127)  # Grayscale threshold for dark things...
crop_bottom = 18  # 裁剪底部30行像素
rho_pid = PID(p=0.17, i=0, d=0)  # y=ax+b b截距
theta_pid = PID(p=0.001, i=0, d=0)  # a斜率

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)  # 80x60 (4,800 pixels)
sensor.set_windowing((0, 0, 80, 60 - crop_bottom))  # 裁剪底部像素
sensor.skip_frames(time=2000)
clock = time.clock()
uart = pyb.UART(3, 115200)  # p4/p5 are UART3
uart.init(115200, bits=8, parity=None, stop=1)

# 识别区域·1 11111111111111111111111111111111111111111
roi1 = [( 2,22,10, 7),  # 左 x y w h
        (16,22,10, 7),
        (67,22,10, 7),  # 右
        (53,22,10, 7),
        (35, 0,10,15)]  # 上

tx = bytearray([0x7B, 0x00, 0x00, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x6A, 0x7D])

def data_bytes(data):
    highByte = (data >> 8) & 0xFF
    lowByte = data & 0xFF
    return highByte, lowByte

while (True):
    clock.tick()
    flag = 0

    img = sensor.snapshot().binary([THRESHOLD])  # 线设置为白色，其他为黑色
    img.erode(2)
    img.dilate(5)  # 对二值图像进行膨胀操作

    line = img.get_regression([(100, 100)], robust=True)  # 返回直线蓝色

    for rec in roi1:
        img.draw_rectangle(rec, color=(255, 0, 0))  # 绘制出roi区域

    # 检测左转、右转和直行
    if (line):
        #if line.magnitude() > 8:
        if True:
            if img.find_blobs([(0, 30, 127, -128, 127, -128)], roi=roi1[4]):  # up
                LED(3).off()
                #pass
            else:
                flag = 3
                LED(3).on()
            if (img.find_blobs([(0, 30, 127, -128, 127, -128)], roi=roi1[0]) or img.find_blobs([(0, 30, 127, -128, 127, -128)], roi=roi1[1])):  # left
                LED(1).off()
                #pass
            else:
                flag = 1
                LED(1).on()
            if (img.find_blobs([(0, 30, 127, -128, 127, -128)], roi=roi1[2]) or img.find_blobs([(0, 30, 127, -128, 127, -128)], roi=roi1[3])):  # right
                LED(2).off()
                #pass
            else:
                flag = 2
                LED(2).on()
        else:
            pass

        rho_err = abs(line.rho()) - img.width() / 2  # 直线偏移距离
        if line.theta() > 90:  # 直线角度
            theta_err = line.theta() - 180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color=(0,0,255))  # 画出蓝色直线

        if line.magnitude() > 8:  # 线性回归效果好进行下一步，否则不进行
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
