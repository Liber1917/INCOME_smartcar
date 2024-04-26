import threading
import time
import os
import cv2
import numpy as np
import RPi.GPIO as GPIO

class CameraCapture(threading.Thread):
    def __init__(self, device="/dev/video0", width=640, height=480):
        super().__init__()
        # 尝试使用 Gstreamer 进行硬件加速
        gst_str = f"v4l2src device={device} ! video/x-raw, width={width}, height={height}, format=(string)YUY2 ! videoconvert ! appsink"
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            print("无法打开摄像头")
            return
        
        # 设置窗口大小
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera', width, height)

    def run(self):
        try:
            while True:
                # 读取摄像头画面
                ret, frame = self.cap.read()

                # 检查是否成功读取画面
                if not ret:
                    print("无法读取画面")
                    break

                # 在窗口中显示画面
                cv2.imshow('Camera', frame)

                # 等待按键事件，如果按下 'q' 键则退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            # 释放摄像头
            self.cap.release()
            # 关闭所有窗口
            cv2.destroyAllWindows()

class MotorController(threading.Thread):
    def __init__(self, left_pins, right_pins, pwm_frequency=100):
        super().__init__()
        # 定义电机
        self.LEFT = 0
        self.RIGHT = 1
        
        # 初始化引脚
        self.PWMA, self.AIN1, self.AIN2, self.STBY = left_pins
        self.PWMB, self.BIN1, self.BIN2, _ = right_pins
        
        # 设置 GPIO 模式
        GPIO.setmode(GPIO.BCM)
        
        # 设置引脚模式
        for pin in (self.PWMA, self.AIN1, self.AIN2, self.STBY, self.PWMB, self.BIN1, self.BIN2):
            GPIO.setup(pin, GPIO.OUT)
        
        # 设置 PWM
        self.pwm_a = GPIO.PWM(self.PWMA, pwm_frequency)
        self.pwm_b = GPIO.PWM(self.PWMB, pwm_frequency)
        
        # 初始化 PWM
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def run(self):
        try:
            while True:
                # 测试代码
                self.forward(self.LEFT, 50)  # 左电机正转，速度50
                self.forward(self.RIGHT, 50)  # 右电机正转，速度50
                time.sleep(2)

                self.backward(self.LEFT, 50)  # 左电机反转，速度50
                self.backward(self.RIGHT, 50)  # 右电机反转，速度50
                time.sleep(2)

                self.stop(self.LEFT)  # 停止左电机
                self.stop(self.RIGHT)  # 停止右电机
                time.sleep(2)

        except KeyboardInterrupt:
            # 清理 GPIO 引脚
            self.cleanup()

    # 正转
    def forward(self, mt, speed):
        GPIO.output(self.STBY, GPIO.HIGH)
        if mt == self.LEFT:
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            self.pwm_a.start(speed)
        elif mt == self.RIGHT:
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            self.pwm_b.start(speed)
    
    # 反转
    def backward(self, mt, speed):
        GPIO.output(self.STBY, GPIO.HIGH)
        if mt == self.LEFT:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            self.pwm_a.start(speed)
        elif mt == self.RIGHT:
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            self.pwm_b.start(speed)
    
    # 停止
    def stop(self, mt):
        GPIO.output(self.STBY, GPIO.LOW)
        if mt == self.LEFT:
            self.pwm_a.stop()
        elif mt == self.RIGHT:
            self.pwm_b.stop()
    
    # 清理 GPIO 引脚
    def cleanup(self):
        GPIO.cleanup()


class LaneDetection:
    def __init__(self, folder_path):
        self.folder_path = folder_path
        self.error_values = []

    def getFileList(self, dir, Filelist, ext=None):
        newDir = dir
        if os.path.isfile(dir):
            if ext is None:
                Filelist.append(dir)
            else:
                if ext in dir[-len(ext):]:
                    Filelist.append(dir)
        elif os.path.isdir(dir):
            for s in os.listdir(dir):
                newDir = os.path.join(dir, s)
                self.getFileList(newDir, Filelist, ext)

    def mid(self, follow, mask):
        halfWidth = follow.shape[1] // 2
        half = halfWidth
        for y in range(follow.shape[0] - 1, -1, -1):
            if (mask[y][max(0, half - halfWidth):half] == np.zeros_like(mask[y][max(0, half - halfWidth):half])).all():
                left = max(0, half - halfWidth)
            else:
                left = np.average(np.where(mask[y][0:half] == 255))
            if (mask[y][half:min(follow.shape[1], half + halfWidth)] == np.zeros_like(mask[y][half:min(follow.shape[1], half + halfWidth)])).all():
                right = min(follow.shape[1], half + halfWidth)
            else:
                right = np.average(np.where(mask[y][half:follow.shape[1]] == 255)) + half

            mid = (left + right) // 2
            half = int(mid)
            follow[y, int(mid)] = 255

            if y == 360:
                self.mid_output = int(mid)

        cv2.circle(follow, (self.mid_output, 360), 5, 255, -1)

        error = follow.shape[1] // 2 - self.mid_output
        self.error_values.append(error)  # 添加error值到列表中

        return follow, error

    def process_images(self):
        jpg_files = []
        self.getFileList(self.folder_path, jpg_files, ext=".jpg")
        print("所有.jpg文件路径：")
        for file_path in jpg_files:
            print(file_path)
            image = cv2.imread(file_path)
            gray = cv2.cv2tColor(image, cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

            follow = np.zeros_like(mask)
            follow, error = self.mid(follow, mask)

            cv2.imshow("Follow", follow)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

# if __name__ == "__main__":
#     my_folder = "./cv2/images"
#     lane_detection = LaneDetection(my_folder)
#     lane_detection.process_images()
#     print("Error values:", lane_detection.error_values)
if __name__ == "__main__":
    # 创建摄像头捕获线程并启动
    camera_thread = CameraCapture()
    camera_thread.start()

    # 创建电机控制线程并启动
    left_pins = (12, 4, 3, 17)
    right_pins = (13, 27, 22, 17)
    motor_controller_thread = MotorController(left_pins, right_pins)
    motor_controller_thread.start()
