import rospy
import limxsdk
from limxsdk.datatypes import RobotCmd, ImuData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os

# 添加其他模块的搜索路径
sys.path.append(os.path.join(os.path.dirname(__file__)))

from color_detection import detect_colors
from obstacle_avoidance import avoid_obstacles
from black_line_detection import detect_black_line


class InspectionController:
    def __init__(self):
        rospy.init_node('inspection_controller', anonymous=True)
        # 初始化机器人
        self.robot = limxsdk.Robot(limxsdk.RobotType.PointFoot)
        self.robot_ip = rospy.get_param('~robot_ip', '10.192.1.2')
        try:
            self.robot.connect(self.robot_ip)
            rospy.loginfo(f"成功连接到机器人，IP: {self.robot_ip}")
        except Exception as e:
            rospy.logerr(f"无法连接到机器人: {e}")
            sys.exit(1)

        # 初始化CV桥
        self.bridge = CvBridge()
        # 订阅相机话题
        self.camera_sub = rospy.Subscriber('/depth_camera/color_image', Image, self.camera_callback)

    def camera_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 调用颜色检测函数
            detected_colors = detect_colors(cv_image)
            if detected_colors:
                rospy.loginfo(f"检测到颜色: {detected_colors}")

            # 调用黑色线条检测函数
            if detect_black_line(cv_image):
                rospy.loginfo("检测到黑色线条，停止机器人")
                self.robot.stop()

            # 调用避障函数
            avoid_obstacles(self.robot, cv_image)

        except Exception as e:
            rospy.logerr(f"处理相机图像时出错: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = InspectionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 确保在程序退出时断开机器人连接
        if hasattr(controller, 'robot'):
            controller.robot.disconnect()
    