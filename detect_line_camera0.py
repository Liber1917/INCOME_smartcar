import cv2
import time

class Camera:
    def __init__(self, device="/dev/video0", width=640, height=480):
        # Gstreamer字符串用于硬件加速
        gst_str = f"v4l2src device={device} ! video/x-raw, width={width}, height={height}, format=(string)YUY2 ! videoconvert ! appsink"
        # 打开摄像头
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        
        # 检查摄像头是否成功打开
        if not self.cap.isOpened():
            print("无法打开摄像头")
            raise ValueError("无法打开摄像头")
        
        # 设置窗口大小
        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera', width, height)

    def start_capture(self):
        prev_time = time.time()
        frames = 0
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

                # 计算帧率
                frames += 1
                curr_time = time.time()
                if curr_time - prev_time >= 1.0:  # 每秒更新一次帧率
                    fps = frames / (curr_time - prev_time)
                    print(f"帧率: {fps:.2f}")
                    prev_time = curr_time
                    frames = 0

                # 等待按键事件，如果按下 'q' 键则退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            # 释放摄像头
            self.cap.release()
            # 关闭所有窗口
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        # 创建摄像头实例并开始捕获
        camera = Camera()
        camera.start_capture()
    except Exception as e:
        print(f"发生错误：{e}")
