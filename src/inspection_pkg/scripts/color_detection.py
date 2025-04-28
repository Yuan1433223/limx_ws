import cv2
import numpy as np


def detect_colors(image):
    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    detected_colors = []

    # 定义红色的HSV范围（由于红色在HSV空间中不连续，需要两个范围）
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 定义绿色的HSV范围
    lower_green = np.array([35, 120, 70])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # 定义蓝色的HSV范围
    lower_blue = np.array([100, 120, 70])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # 查找每种颜色的轮廓
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 检查是否检测到红色物体
    if len(contours_red) > 0:
        detected_colors.append('red')

    # 检查是否检测到绿色物体
    if len(contours_green) > 0:
        detected_colors.append('green')

    # 检查是否检测到蓝色物体
    if len(contours_blue) > 0:
        detected_colors.append('blue')

    return detected_colors
    