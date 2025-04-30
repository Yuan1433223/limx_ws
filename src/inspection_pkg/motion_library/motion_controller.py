import limxsdk.robot.Rate as Rate
import limxsdk.robot.Robot as Robot
from limxsdk.datatypes import RobotCmd, ImuData, RobotState

class MotionController:
    def __init__(self, robot_ip="10.192.1.2"):
        # 初始化机器人实例（单例模式）
        self.robot = Robot(Robot.RobotType.PointFoot)
        self.robot_ip = robot_ip
        self.robot.init(robot_ip)  # 初始化连接
        self.motor_num = self.robot.getMotorNumber()  # 获取电机数量
        self.rate = Rate(100)  # 创建控制频率对象（100Hz）

        # 定义关节索引（根据文档定义）
        self.JOINT_INDEX = {
            'L_ABAD': 0, 'L_HIP': 1, 'L_KNEE': 2,
            'R_ABAD': 3, 'R_HIP': 4, 'R_KNEE': 5
        }

        # 站立初始位置（默认零位，可根据机械结构调整）
        self.STAND_POS = [0.0, 0.8, -1.6,  # 左下肢关节角度（弧度）
                          0.0, 0.8, -1.6]  # 右下肢关节角度（弧度）

    def _send_cmd(self, cmd: RobotCmd):
        """内部方法：发送控制命令"""
        self.robot.publishRobotCmd(cmd)
        self.rate.sleep()

    def _initialize_joints(self, pos: list):
        """内部方法：初始化关节位置"""
        cmd = RobotCmd(self.motor_num)
        cmd.mode = [0] * self.motor_num  # 位置控制模式
        cmd.q = pos.copy()
        for _ in range(100):  # 持续发送1秒初始化命令
            self._send_cmd(cmd)

    # 1. 站立功能
    def stand(self):
        """使机器人进入稳定站立姿态"""
        cmd = RobotCmd(self.motor_num)
        cmd.mode = [0] * self.motor_num
        cmd.q = self.STAND_POS
        for _ in range(200):  # 持续2秒稳定站立
            self._send_cmd(cmd)

    # 2. 原地踏步功能
    def stepping(self, steps=3, step_height=0.1):
        """原地踏步运动（默认3步，步高0.1弧度）"""
        stand_pos = self.STAND_POS.copy()
        step_pos = stand_pos.copy()
        
        for _ in range(steps):
            # 左下肢抬起
            step_pos[self.JOINT_INDEX['L_KNEE']] = -1.5  # 抬腿动作
            cmd = RobotCmd(self.motor_num, q=step_pos)
            for _ in range(50): self._send_cmd(cmd)
            
            # 恢复站立位
            cmd.q = stand_pos
            for _ in range(50): self._send_cmd(cmd)
            
            # 右下肢抬起（对称动作）
            step_pos[self.JOINT_INDEX['R_KNEE']] = -1.5
            cmd = RobotCmd(self.motor_num, q=step_pos)
            for _ in range(50): self._send_cmd(cmd)
            
            cmd.q = stand_pos
            for _ in range(50): self._send_cmd(cmd)

    # 3. 前进一步功能（0.2米步长）
    def move_forward(self, step_length=0.2):
        """向前移动单步（默认0.2米）"""
        # 摆动腿动作（右下肢前摆）
        swing_pos = self.STAND_POS.copy()
        swing_pos[self.JOINT_INDEX['R_HIP']] = 1.0  # 髋关节前摆
        swing_pos[self.JOINT_INDEX['R_KNEE']] = -1.5  # 膝关节弯曲
        
        # 支撑腿保持
        support_pos = self.STAND_POS.copy()
        
        # 执行摆动动作
        cmd = RobotCmd(self.motor_num, q=swing_pos)
        for _ in range(100): self._send_cmd(cmd)
        
        # 切换支撑腿
        cmd.q = support_pos
        for _ in range(100): self._send_cmd(cmd)

    # 4. 后退一步功能
    def move_back(self, step_length=0.2):
        """向后移动单步（默认0.2米）"""
        swing_pos = self.STAND_POS.copy()
        swing_pos[self.JOINT_INDEX['L_HIP']] = -1.0  # 髋关节后摆
        swing_pos[self.JOINT_INDEX['L_KNEE']] = -1.5
        
        cmd = RobotCmd(self.motor_num, q=swing_pos)
        for _ in range(100): self._send_cmd(cmd)
        
        cmd.q = self.STAND_POS
        for _ in range(100): self._send_cmd(cmd)

    # 5. 向左平移一步
    def move_left(self, step_length=0.15):
        """向左横向移动单步（默认0.15米）"""
        lateral_pos = self.STAND_POS.copy()
        lateral_pos[self.JOINT_INDEX['L_ABAD']] = 0.3  # 左髋关节外展
        lateral_pos[self.JOINT_INDEX['R_ABAD']] = -0.3  # 右髋关节内收
        
        cmd = RobotCmd(self.motor_num, q=lateral_pos)
        for _ in range(100): self._send_cmd(cmd)
        
        cmd.q = self.STAND_POS
        for _ in range(100): self._send_cmd(cmd)

    # 6. 向右平移一步
    def move_right(self, step_length=0.15):
        """向右横向移动单步（默认0.15米）"""
        lateral_pos = self.STAND_POS.copy()
        lateral_pos[self.JOINT_INDEX['R_ABAD']] = 0.3 
        lateral_pos[self.JOINT_INDEX['L_ABAD']] = -0.3 
        
        cmd = RobotCmd(self.motor_num, q=lateral_pos)
        for _ in range(100): self._send_cmd(cmd)
        
        cmd.q = self.STAND_POS
        for _ in range(100): self._send_cmd(cmd)

    # 7. 向左旋转
    def turn_left(self, angle=30):  # 角度单位：度
        """向左旋转指定角度（默认30度）"""
        rad_angle = angle * 3.14159 / 180
        rotate_pos = self.STAND_POS.copy()
        rotate_pos[self.JOINT_INDEX['L_HIP']] = 0.5  # 左髋外旋
        rotate_pos[self.JOINT_INDEX['R_HIP']] = -0.5  # 右髋内旋
        
        cmd = RobotCmd(self.motor_num, q=rotate_pos)
        for _ in range(int(angle/3 * 100)):  # 按角度计算循环次数
            self._send_cmd(cmd)
        
        cmd.q = self.STAND_POS
        for _ in range(100): self._send_cmd(cmd)

    # 8. 向右旋转
    def turn_right(self, angle=30):
        """向右旋转指定角度（默认30度）"""
        rad_angle = angle * 3.14159 / 180
        rotate_pos = self.STAND_POS.copy()
        rotate_pos[self.JOINT_INDEX['R_HIP']] = 0.5 
        rotate_pos[self.JOINT_INDEX['L_HIP']] = -0.5 
        
        cmd = RobotCmd(self.motor_num, q=rotate_pos)
        for _ in range(int(angle/3 * 100)):
            self._send_cmd(cmd)
        
        cmd.q = self.STAND_POS
        for _ in range(100): self._send_cmd(cmd)

    # 9. 上楼梯功能（简化实现，需结合传感器反馈）
    def up_stair(self, step_height=0.15):
        """上楼梯动作（默认台阶高度0.15米）"""
        # 1. 摆腿准备
        self.move_forward(step_length=0.1)
        
        # 2. 抬腿上阶（模拟膝关节抬高）
        stair_pos = self.STAND_POS.copy()
        stair_pos[self.JOINT_INDEX['L_KNEE']] = -1.3  # 左膝抬高
        cmd = RobotCmd(self.motor_num, q=stair_pos)
        for _ in range(150): self._send_cmd(cmd)
        
        # 3. 重心前移
        stair_pos[self.JOINT_INDEX['L_HIP']] = 1.2 
        for _ in range(100): self._send_cmd(cmd)
        
        # 4. 恢复双腿支撑
        self.stand()

    def shutdown(self):
        """安全关闭连接"""
        self.robot.disconnect()

# 使用示例
if __name__ == "__main__":
    controller = MotionController(robot_ip="10.192.1.2")
    
    # 初始化站立
    controller.stand()
    
    # 执行运动序列
    controller.stepping(steps=2)
    controller.move_forward()
    controller.move_left()
    controller.turn_right(angle=45)
    controller.up_stair()
    
    # 关闭连接
    controller.shutdown()