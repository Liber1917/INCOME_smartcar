import threading
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

class LaneDetector:
    def __init__(self, device=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            print("无法打开摄像头")
            raise ValueError("无法打开摄像头")

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("无法读取画面")
            return None
        return frame

    def detect_lane(self):
        frame = self.read_frame()
        if frame is None:
            return None, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
        ret, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)  

        follow = np.zeros_like(mask)  
        follow, error = self._mid(follow, mask)  

        return follow, error

    def release(self):
        self.cap.release()

    def _mid(self, follow, mask):
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
                mid_output = int(mid)

        cv2.circle(follow, (mid_output, 360), 5, 255, -1)  

        error = follow.shape[1] // 2 - mid_output  

        return follow, error  

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

        # PID参数
        self.Kp = 0.1
        self.Ki = 0
        self.Kd = 0
        self.prev_error = 0
        self.integral = 0

    def run(self):
        try:
            while True:
                # 测试代码
                self.forward(50)  # 前进，速度50
                time.sleep(2)

                self.backward(50)  # 后退，速度50
                time.sleep(2)

                self.stop()  # 停止
                time.sleep(2)

        except KeyboardInterrupt:
            # 清理 GPIO 引脚
            self.cleanup()

    # 正转
    def forward(self, speed):
        lane_detector = LaneDetector()  # 创建车道检测器实例
        follow, error = lane_detector.detect_lane()  # 检测车道
        if error is None:
            return
        
        # PID控制
        pid_output = self.pid_control(error)
        
        left_speed = speed + pid_output
        right_speed = speed - pid_output
        
        GPIO.output(self.STBY, GPIO.HIGH)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)
        self.pwm_a.start(left_speed)
        self.pwm_b.start(right_speed)
    
    # 反转
    def backward(self, speed):
        lane_detector = LaneDetector()  # 创建车道检测器实例
        follow, error = lane_detector.detect_lane()  # 检测车道
        if error is None:
            return
        
        # PID控制
        pid_output = self.pid_control(error)
        
        left_speed = speed + pid_output
        right_speed = speed - pid_output
        
        GPIO.output(self.STBY, GPIO.HIGH)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)
        self.pwm_a.start(left_speed)
        self.pwm_b.start(right_speed)
    
    # 停止
    def stop(self):
        GPIO.output(self.STBY, GPIO.LOW)
        self.pwm_a.stop()
        self.pwm_b.stop()
    
    # 清理 GPIO 引脚
    def cleanup(self):
        GPIO.cleanup()

    # PID控制
    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

if __name__ == "__main__":
    try:
        # 创建电机控制线程并启动
        left_pins = (12, 4, 3, 17)
        right_pins = (13, 27, 22, 17)
        motor_controller_thread = MotorController(left_pins, right_pins)
        motor_controller_thread.start()
    except Exception as e:
        print(f"发生错误：{e}")
    except KeyboardInterrupt:
        motor_controller_thread.cleanup()
        GPIO.cleanup()
