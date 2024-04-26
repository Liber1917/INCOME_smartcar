#coding=utf-8
import cv2

def camera():
    # 打开摄像头
    # 尝试开Gstreamer进行硬件加速
    # cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    gst_str = "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, format=(string)YUY2 ! videoconvert ! appsink"
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头")
        return
        # 设置窗口大小
    cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Camera', 640, 480)  # 设置窗口大小为 640x480
    
    try:
        while True:
            # 读取摄像头画面
            ret, frame = cap.read()

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
        cap.release()
        # 关闭所有窗口
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera()
