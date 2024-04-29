import RPi.GPIO as GPIO
import time

# 编码器引脚定义
ENCODER_L = 23  # 编码器采集引脚 每路2个 共4个
DIRECTION_L = 24
ENCODER_R = 25
DIRECTION_R = 8

Velocity_L = 0
Velocity_R = 0


def control():
    global Velocity_L, Velocity_R
    Velocity_Left = Velocity_L
    Velocity_L = 0
    Velocity_Right = Velocity_R
    Velocity_R = 0
    print("Left Encoder:", Velocity_Left)
    print("Right Encoder:", Velocity_Right)


def setup():
    global Velocity_L, Velocity_R
    GPIO.setmode(GPIO.BCM)  # 设置为BCM编码方式
    GPIO.setup(ENCODER_L, GPIO.IN)  # 编码器引脚
    GPIO.setup(DIRECTION_L, GPIO.IN)  # 编码器引脚
    GPIO.setup(ENCODER_R, GPIO.IN)  # 编码器引脚
    GPIO.setup(DIRECTION_R, GPIO.IN)  # 编码器引脚

def loop():
    global Velocity_L, Velocity_R
    while True:
        print("ENCODER_L",GPIO.input(ENCODER_L),GPIO.input(ENCODER_R),"ENCODER_R")
        print("DIRECTION_L",GPIO.input(DIRECTION_L),GPIO.input(DIRECTION_R),"DIRECTION_R")
        # 读取编码器数据
        if GPIO.input(ENCODER_L) == GPIO.LOW:
            if GPIO.input(DIRECTION_L) == GPIO.LOW:
                Velocity_L -= 1
            else:
                Velocity_L += 1
        else:
            if GPIO.input(DIRECTION_L) == GPIO.LOW:
                Velocity_L += 1
            else:
                Velocity_L -= 1

        if GPIO.input(ENCODER_R) == GPIO.LOW:
            if GPIO.input(DIRECTION_R) == GPIO.LOW:
                Velocity_R += 1
            else:
                Velocity_R -= 1
        else:
            if GPIO.input(DIRECTION_R) == GPIO.LOW:
                Velocity_R -= 1
            else:
                Velocity_R += 1

        # 控制间隔
        time.sleep(0.05)

        # 执行控制函数
        control()


if __name__ == "__main__":
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        GPIO.cleanup()
