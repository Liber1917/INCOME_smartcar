import cv2
import numpy as np

class Camera:
    def __init__(self, device=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            print("无法打开摄像头")
            raise ValueError("无法打开摄像头")

        # cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("无法读取画面")
            return None
        return frame

    def release(self):
        self.cap.release()
        # cv2.destroyAllWindows()

# 中线拟合
def mid(follow, mask):
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

if __name__ == "__main__":
    try:
        camera = Camera()  # 创建摄像头实例
        while True:
            frame = camera.read_frame()  # 读取摄像头画面
            if frame is None:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
            ret, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)  

            follow = np.zeros_like(mask)  
            follow, error = mid(follow, mask)  

            # cv2.imshow('Camera', frame)
            # cv2.waitKey(1)

            # # 在摄像头窗口中绘制拟合出的中线
            # cv2.imshow('Camera', follow)
            # cv2.waitKey(1)
    except Exception as e:
        print(f"发生错误：{e}")
    finally:
        camera.release()  # 释放摄像头资源
