import cv2
import numpy as np
import rospy


def avoid_obstacles(robot, depth_image):
    # 障碍物检测距离阈值（单位：米）
    obstacle_distance_threshold = 1.0
    # 转换深度图像为合适的数据类型
    depth_image = np.array(depth_image, dtype=np.float32) / 1000.0  # 假设深度值单位为毫米，转换为米

    # 获取图像中心区域作为主要检测区域
    height, width = depth_image.shape
    center_x = width // 2
    center_y = height // 2
    detection_area_width = int(width * 0.3)
    detection_area_height = int(height * 0.3)
    detection_area = depth_image[
                     center_y - detection_area_height // 2:center_y + detection_area_height // 2,
                     center_x - detection_area_width // 2:center_x + detection_area_width // 2
                     ]

    # 计算检测区域内的平均深度
    average_depth = np.mean(detection_area)

    # 检查是否存在障碍物
    if np.isfinite(average_depth) and average_depth < obstacle_distance_threshold:
        rospy.loginfo("检测到障碍物，开始避障")
        # 假设障碍物在右侧
        if np.mean(depth_image[:, center_x:]) < np.mean(depth_image[:, :center_x]):
            # 左转
            robot.turn(-0.5, 0.3)
            rospy.loginfo("障碍物在右侧，机器人左转")
        else:
            # 右转
            robot.turn(0.5, 0.3)
            rospy.loginfo("障碍物在左侧，机器人右转")
        # 前进一段距离以绕过障碍物
        robot.move_forward(0.2, 2.0)
    else:
        # 没有检测到障碍物，继续前进
        robot.move_forward(0.2, 0.1)
    