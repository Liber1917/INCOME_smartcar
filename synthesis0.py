import threading
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# 全局变量用于存储线检测结果和错误值
global follow, error
follow = 0
error = 0

class LaneDetectorThread(threading.Thread):
    def __init__(self):
        super().__init__()

    def run(self):
        global follow, error
        lane_detector = LaneDetector()  # 创建车道检测器实例
        while True:
            frame = lane_detector.read_frame()  # 读取图像帧
            if frame is None:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            follow, error = lane_detector.mid(mask)  # 执行线检测
            # follow, error = fit_lane_center(mask)


class LaneDetector:
    def __init__(self, device=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device)
        #cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
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

    def mid(self, mask):
        follow = np.zeros_like(mask)
        halfWidth = follow.shape[1] // 2
        halfWidth = halfWidth*2
        half = halfWidth  
        rev_mid =None
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

            if y == 300:  
                mid_output = int(mid)
            if y == 380:
                rev_mid = int(mid)
        print("mid",mid_output-rev_mid)
        cv2.circle(follow, (mid_output, 300), 5, 255, -1)  
        cv2.circle(follow, (mid_output, 380), 5, 255, -1)

        error = follow.shape[1] // 2 - 1*mid_output -0*rev_mid  
        if(abs(mid_output-rev_mid)/80 > 1):
            error =error*1.2
        
        # 在这里进行线拟合，并在图像中显示拟合结果
        cv2.imshow("Fitted Lines", follow)
        cv2.waitKey(1)  # 等待1毫秒，允许图像显示

        return follow, error  



class MotorController(threading.Thread):
    def __init__(self, left_pins, right_pins, pwm_frequency=100):
        super().__init__()
        self.error_history = []
        self.error_lock = threading.Lock()  # 创建锁对象
        # 初始化PWM频率
        self.pwm_frequency = pwm_frequency
        # 初始化PID参数等
        self.Kp = 0.15
        self.Ki = 0
        self.Kd = 0
        self.prev_error = 0
        self.integral = 0

        # 引脚定义
        self.left_pins = left_pins
        self.right_pins = right_pins

        # 设置 GPIO 模式
        GPIO.setmode(GPIO.BCM)

        # 设置引脚模式
        GPIO.setup(self.left_pins, GPIO.OUT)
        GPIO.setup(self.right_pins, GPIO.OUT)

        # 设置 PWM
        self.pwm_left = GPIO.PWM(self.left_pins[0], self.pwm_frequency)  
        self.pwm_right = GPIO.PWM(self.right_pins[0], self.pwm_frequency)  

        # 初始化 PWM
        self.pwm_left.start(0)
        self.pwm_right.start(0)

    def run(self):
        global follow, error
        try:
            while True:
                # 获取线检测结果和错误值
                if follow is None or error is None:
                    time.sleep(0.01)  # 等待线检测结果
                    continue
                # 添加当前错误值到历史记录
                # 使用锁保护共享资源
                if(abs(error))>110:
                    error=1.1*error*abs(error)/90
                error = max(min(1.3*error,260),-260)
                print("error=",error)
                with self.error_lock:
                    self.error_history.append(error)
                    # 限制历史记录长度
                    if len(self.error_history) > 3:
                        self.error_history = self.error_history[-3:]
                

                # 计算历史平均错误值
                avg_error = sum(self.error_history) / len(self.error_history)

                # 根据错误值进行差速控制
                
                pid_output = 0.3*self.pid_control(avg_error)
                left_speed = 30 - pid_output
                right_speed = 30 + pid_output
                
                # 将速度限制在合理范围内
                left_speed = 0.65*max(min(left_speed, 150), -160)
                right_speed = 0.65*max(min(right_speed, 150), -160)

                print(right_speed,left_speed,right_speed-left_speed)

                # 控制电机
                GPIO.output(self.left_pins[1], GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
                GPIO.output(self.left_pins[2], GPIO.LOW if left_speed >= 0 else GPIO.HIGH)
                GPIO.output(self.right_pins[1], GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
                GPIO.output(self.right_pins[2], GPIO.LOW if right_speed >= 0 else GPIO.HIGH)

                self.pwm_left.ChangeDutyCycle(abs(left_speed))  
                self.pwm_right.ChangeDutyCycle(abs(right_speed))  

                time.sleep(0.005)  # 控制频率
        except KeyboardInterrupt:
            self.cleanup()

    def cleanup(self):
        GPIO.cleanup()

    def pid_control(self, error):
        self.integral += error
        self.integral = max(min(self.integral,100),-100)
        derivative = error - self.prev_error
        # derivative = max(min(derivative,180),-180)
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

if __name__ == "__main__":
    try:
        # 创建线检测线程并启动
        lane_detector_thread = LaneDetectorThread()
        lane_detector_thread.start()

        # 创建电机控制线程并启动
        time.sleep(4)
        left_pins = (12, 4, 3, 17)  # BCD引脚定义
        right_pins = (13, 27, 22, 17)  # BCD引脚定义
        motor_controller_thread = MotorController(left_pins, right_pins)
        motor_controller_thread.start()

        while True:
            # 主线程持续运行
            pass
    except Exception as e:
        print(f"发生错误：{e}")
    finally:
        GPIO.cleanup()  # 最终清理GPIO引脚
        cv2.destroyAllWindows()

