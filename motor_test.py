#coding=utf-8
import RPi.GPIO as GPIO
import time

# 定义电机
LEFT = 0
RIGHT = 1
# 定义引脚
PWMA = 12
AIN1 = 4
AIN2 = 3
STBY = 17
PWMB = 13
BIN1 = 27
BIN2 = 22

# 设置 GPIO 模式
GPIO.setmode(GPIO.BCM)

# 设置引脚模式
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

# 设置 PWM
pwm_a = GPIO.PWM(PWMA, 100)  # 设置 PWM 频率为 100Hz
pwm_b = GPIO.PWM(PWMB, 100)  # 设置 PWM 频率为 100Hz

# 正转
def forward(mt, speed):
    GPIO.output(STBY, GPIO.HIGH)
    if mt == LEFT:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        pwm_a.start(speed)
    elif mt == RIGHT:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
        pwm_b.start(speed)

# 反转
def backward(mt, speed):
    GPIO.output(STBY, GPIO.HIGH)
    if mt == LEFT:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        pwm_a.start(speed)
    elif mt == RIGHT:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
        pwm_b.start(speed)

# 停止
def stop(mt):
    GPIO.output(STBY, GPIO.LOW)
    if mt == LEFT:
        pwm_a.stop()
    elif mt == RIGHT:
        pwm_b.stop()

# 初始化 PWM
pwm_a.start(0)
pwm_b.start(0)

forward(LEFT, 50)  # 左电机正转，速度50
forward(RIGHT, 50)  # 右电机正转，速度50
time.sleep(2)
GPIO.cleanup()
# # 测试代码
# try:
#     while True:
#         forward(LEFT, 50)  # 左电机正转，速度50
#         forward(RIGHT, 50)  # 右电机正转，速度50
#         time.sleep(2)

#         backward(LEFT, 50)  # 左电机反转，速度50
#         backward(RIGHT, 50)  # 右电机反转，速度50
#         time.sleep(2)

#         stop(LEFT)  # 停止左电机
#         stop(RIGHT)  # 停止右电机
#         time.sleep(2)

# except KeyboardInterrupt:
#     # 清理 GPIO 引脚
#     GPIO.cleanup()
