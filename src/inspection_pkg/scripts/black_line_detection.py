import cv2
import numpy as np


def detect_black_line(image):
    # 将图像转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 对灰度图进行阈值处理，将黑色区域提取出来
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # 使用Canny边缘检测算法检测边缘
    edges = cv2.Canny(thresh, 50, 150)

    # 使用Hough直线检测算法检测直线
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=10)

    # 如果检测到直线，则认为检测到黑色线条
    return lines is not None
    