import cv2 as cv
import os
import numpy as np

# 遍历文件夹函数
def getFileList(dir, Filelist, ext=None):
    """
    获取文件夹及其子文件夹中文件列表
    输入 dir：文件夹根目录
    输入 ext: 扩展名
    返回： 文件路径列表
    """
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
            getFileList(newDir, Filelist, ext)

# 中线拟合
def mid(follow, mask):
    halfWidth = follow.shape[1] // 2
    half = halfWidth  # 从下往上扫描赛道,最下端取图片中线为分割线
    for y in range(follow.shape[0] - 1, -1, -1):
        # V2改动:加入分割线左右各半张图片的宽度作为约束,减小邻近赛道的干扰
        if (mask[y][max(0, half - halfWidth):half] == np.zeros_like(mask[y][max(0, half - halfWidth):half])).all():  # 分割线左端无赛道
            left = max(0, half - halfWidth)  # 取图片左边界
        else:
            left = np.average(np.where(mask[y][0:half] == 255))  # 计算分割线左端平均位置
        if (mask[y][half:min(follow.shape[1], half + halfWidth)] == np.zeros_like(mask[y][half:min(follow.shape[1], half + halfWidth)])).all():  # 分割线右端无赛道
            right = min(follow.shape[1], half + halfWidth)  # 取图片右边界
        else:
            right = np.average(np.where(mask[y][half:follow.shape[1]] == 255)) + half  # 计算分割线右端平均位置

        mid = (left + right) // 2  # 计算拟合中点
        half = int(mid)  # 递归,从下往上确定分割线
        follow[y, int(mid)] = 255  # 画出拟合中线

        if y == 360:  # 设置指定提取中点的纵轴位置
            mid_output = int(mid)

    cv.circle(follow, (mid_output, 360), 5, 255, -1)  # opencv为(x,y),画出指定提取中点

    error = follow.shape[1] // 2 - mid_output  # 计算图片中点与指定提取中点的误差

    return follow, error  # error为正数右转,为负数左转

# 测试案例
if __name__ == "__main__":
    my_folder = "./CV/images"  # 文件夹路径
    jpg_files = []  # 存储.jpg文件路径的列表
    getFileList(my_folder, jpg_files, ext=".jpg")
    print("所有.jpg文件路径：")
    for file_path in jpg_files:
        print(file_path)
        image = cv.imread(file_path)  # 读取图片
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # 转换为灰度图
        ret, mask = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)  # 二值化处理

        # cv.namedWindow("Mask", cv.WINDOW_NORMAL)  # 创建可调整大小的窗口
        # cv.resizeWindow("Mask", 680, 680)  # 设定窗口大小
        # cv.imshow("Mask", mask)  # 显示二值化后的结果
        # cv.waitKey(0)
        # cv.destroyAllWindows()

        follow = np.zeros_like(mask)  # 创建一个和mask相同大小的空白图像
        follow, error = mid(follow, mask)  # 进行中线拟合

        # cv.namedWindow("Follow", cv.WINDOW_NORMAL)  # 创建可调整大小的窗口
        # cv.resizeWindow("Follow", 680, 680)  # 设定窗口大小
        # cv.imshow("Follow", follow)  # 显示拟合结果
        # cv.waitKey(0)
        # cv.destroyAllWindows()
