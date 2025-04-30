import os
import sys
import cv2
import time
import math
import threading
import numpy as np
from collections import deque
# 导入TRON1机器人运动控制接口（根据motion_controller.py）
from inspection_pkg.motion_library.motion_controller import MotionControlle

# -------------------- 全局配置 --------------------
# 机器人IP（根据实际环境修改）
ROBOT_IP = "10.192.1.2"
# 颜色目标配置（红/绿/蓝）
COLOR_TARGETS = ['red', 'green', 'blue']
# 检测距离阈值（30cm）
TARGET_DISTANCE = 0.3  # 米
# 障碍物类型（箱子/椅子/桌子/楼梯）
OBSTACLE_TYPES = ['box', 'chair', 'table', 'stair']
# 状态机定义
STATE_PATROL = 0       # 正常巡检
STATE_AVOID_OBSTACLE = 1  # 避障中
STATE_APPROACH_TARGET = 2  # 接近目标
STATE_INSPECT_TARGET = 3   # 绕物检测
STATE_FEEDBACK_FAULT = 4   # 反馈病害

# -------------------- 机器人与传感器初始化 --------------------
# 初始化运动控制器（基于motion_controller.py）
controller = MotionController(robot_ip=ROBOT_IP)
controller.stand()  # 初始站立

# 模拟深度相机（实际需替换为真实深度相机接口）
class DepthCamera:
    def get_depth(self, x, y):
        """模拟获取(x,y)像素处的深度（单位：米）"""
        return np.random.uniform(0.2, 2.0)  # 实际应通过相机SDK获取

depth_cam = DepthCamera()

# -------------------- 视觉处理增强 --------------------
def getAreaMaxContour(contours, area_min=10):
    """获取最大面积轮廓（优化鲁棒性）"""
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        area = cv2.contourArea(c)
        if area > contour_area_max and area >= area_min:
            contour_area_max = area
            area_max_contour = c
    return area_max_contour, contour_area_max

def detect_obstacles(img):
    """检测障碍物（箱子/椅子/桌子/楼梯）
    返回：障碍物类型列表，每个元素为 (类型, 中心坐标, 距离)
    """
    # 实际应使用目标检测模型（如YOLO），此处模拟
    obstacles = []
    # 模拟楼梯检测（通过颜色+形状）
    stair_contour, _ = getAreaMaxContour(cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2], 500)
    if stair_contour is not None:
        x, y, w, h = cv2.boundingRect(stair_contour)
        obstacles.append(('stair', (x+w//2, y+h//2), depth_cam.get_depth(x+w//2, y+h//2)))
    # 模拟其他障碍物（颜色区分）
    for obj_type in ['box', 'chair', 'table']:
        if np.random.random() < 0.3:  # 随机模拟障碍物出现
            x, y = np.random.randint(100, 500, 2)
            obstacles.append((obj_type, (x, y), depth_cam.get_depth(x, y)))
    return obstacles

def detect_color_target(img, target_color):
    """检测指定颜色的目标物体
    返回：(中心坐标x, 中心坐标y, 距离)
    """
    # 颜色空间转换（LAB更稳定）
    lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    # 颜色阈值（需根据实际校准lab_data）
    lower = np.array([30, 15, 15]) if target_color == 'red' else \
            np.array([50, -15, 15]) if target_color == 'green' else \
            np.array([30, -15, -15])  # 示例值，需实际校准
    upper = np.array([80, 40, 40]) if target_color == 'red' else \
            np.array([90, 15, 40]) if target_color == 'green' else \
            np.array([80, 15, 15])
    mask = cv2.inRange(lab_img, lower, upper)
    # 形态学处理
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    max_contour, _ = getAreaMaxContour(contours, area_min=200)
    if max_contour is None:
        return None
    # 计算中心坐标
    M = cv2.moments(max_contour)
    cx = int(M['m10']/M['m00']) if M['m00'] != 0 else 0
    cy = int(M['m01']/M['m00']) if M['m00'] != 0 else 0
    # 获取深度距离
    distance = depth_cam.get_depth(cx, cy)
    return (cx, cy, distance)

def detect_black_line(img):
    """检测蓝色物体上的黑色线条（病害）
    返回：线条中心坐标（若存在）
    """
    # 灰度化+二值化
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    # 轮廓检测
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    line_contour, _ = getAreaMaxContour(contours, area_min=50)
    if line_contour is None:
        return None
    # 计算线条中心
    M = cv2.moments(line_contour)
    cx = int(M['m10']/M['m00']) if M['m00'] != 0 else 0
    cy = int(M['m01']/M['m00']) if M['m00'] != 0 else 0
    return (cx, cy)

# -------------------- 运动控制状态机 --------------------
class PatrolStateMachine:
    def __init__(self):
        self.current_state = STATE_PATROL
        self.target_color = None       # 当前要检测的颜色目标
        self.fault_detected = False    # 是否检测到病害
        self.obstacle_queue = deque()  # 待避障的障碍物队列
        self.inspect_step = 0          # 绕物检测步骤

    def update_state(self, obstacles, color_targets, fault):
        """状态转移逻辑"""
        # 优先级：病害反馈 > 绕物检测 > 避障 > 正常巡检
        if fault:
            self.current_state = STATE_FEEDBACK_FAULT
            return
        if self.current_state == STATE_INSPECT_TARGET:
            if self.inspect_step >= 360:  # 完成360度检测
                self.current_state = STATE_PATROL
            return
        if obstacles:
            self.obstacle_queue.extend(obstacles)
            self.current_state = STATE_AVOID_OBSTACLE
            return
        if color_targets:
            self.target_color = color_targets[0]  # 选择最近的目标
            self.current_state = STATE_APPROACH_TARGET
            return
        self.current_state = STATE_PATROL

    def execute_state(self, img):
        """执行当前状态的动作"""
        if self.current_state == STATE_PATROL:
            self._patrol()
        elif self.current_state == STATE_AVOID_OBSTACLE:
            self._avoid_obstacle()
        elif self.current_state == STATE_APPROACH_TARGET:
            self._approach_target(img)
        elif self.current_state == STATE_INSPECT_TARGET:
            self._inspect_target()
        elif self.current_state == STATE_FEEDBACK_FAULT:
            self._feedback_fault()

    def _patrol(self):
        """正常巡检：直线前进"""
        controller.move_forward(step_length=0.1)
        time.sleep(0.5)

    def _avoid_obstacle(self):
        """避障逻辑：绕过最近的障碍物"""
        obstacle = self.obstacle_queue.popleft()
        # 简单避障策略：向左平移0.3米
        controller.move_left(step_length=0.3)
        time.sleep(1.0)
        # 重新前进
        controller.move_forward(step_length=0.5)
        time.sleep(1.0)
        # 向右平移回原路径
        controller.move_right(step_length=0.3)
        time.sleep(1.0)

    def _approach_target(self, img):
        """接近目标：调整位置至30cm距离"""
        target_info = detect_color_target(img, self.target_color)
        if not target_info:
            return
        cx, cy, distance = target_info
        # 水平偏移调整（左右平移）
        if cx < 300:  # 目标在左侧
            controller.move_left(step_length=0.05)
        elif cx > 340:  # 目标在右侧
            controller.move_right(step_length=0.05)
        # 前后距离调整
        if distance > TARGET_DISTANCE + 0.1:  # 太远，前进
            controller.move_forward(step_length=0.05)
        elif distance < TARGET_DISTANCE - 0.1:  # 太近，后退
            controller.move_back(step_length=0.05)
        else:  # 到达指定距离，进入检测状态
            self.current_state = STATE_INSPECT_TARGET
            self.inspect_step = 0

    def _inspect_target(self):
        """绕物检测：旋转360度"""
        controller.turn_right(angle=5)  # 每次旋转5度
        self.inspect_step += 5
        time.sleep(0.5)
        # 检测是否有病害（黑线）
        if self.target_color == 'blue':  # 仅蓝色物体需要检测病害
            line_pos = detect_black_line(camera_img)  # 需传入当前图像
            if line_pos:
                self.fault_detected = True
                self.current_state = STATE_FEEDBACK_FAULT

    def _feedback_fault(self):
        """反馈病害坐标"""
        # 假设获取机器人当前位姿（需结合定位模块）
        robot_x, robot_y = 0.0, 0.0  # 实际应通过定位系统获取
        # 计算病害世界坐标（相机外参+深度）
        # 此处为模拟，实际需根据相机标定参数转换
        fault_world_x = robot_x + 0.3 * math.cos(math.radians(controller.current_angle))
        fault_world_y = robot_y + 0.3 * math.sin(math.radians(controller.current_angle))
        print(f"发现病害！坐标：({fault_world_x:.2f}, {fault_world_y:.2f})")
        # 反馈后恢复巡检
        self.fault_detected = False
        self.current_state = STATE_PATROL

# -------------------- 主程序 --------------------
if __name__ == "__main__":
    # 初始化状态机
    state_machine = PatrolStateMachine()
    # 初始化相机（实际需替换为TRON1的相机接口）
    camera = cv2.VideoCapture(0)
    # 主循环
    try:
        while True:
            ret, img = camera.read()
            if not ret:
                continue
            # 步骤1：检测障碍物
            obstacles = detect_obstacles(img)
            # 步骤2：检测颜色目标（红/绿/蓝）
            color_targets = []
            for color in COLOR_TARGETS:
                target = detect_color_target(img, color)
                if target:
                    color_targets.append((color, target[2]))  # (颜色, 距离)
            # 按距离排序，优先接近最近的目标
            color_targets.sort(key=lambda x: x[1])
            # 步骤3：检测病害（仅当目标是蓝色时）
            current_target_color = state_machine.target_color
            fault = False
            if current_target_color == 'blue':
                fault = (detect_black_line(img) is not None)
            # 步骤4：状态机更新
            state_machine.update_state(obstacles, color_targets, fault)
            # 步骤5：执行当前状态动作
            state_machine.execute_state(img)
            # 退出条件
            if cv2.waitKey(1) == 27:
                break
    finally:
        controller.shutdown()  # 安全关闭机器人连接
        camera.release()
        cv2.destroyAllWindows()