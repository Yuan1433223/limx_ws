# robot_monitor_enhanced.py
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import threading
import time
from datetime import datetime, timedelta
import matplotlib.dates as mdates
from typing import Optional

class RobotMonitor:
    """增强版机器人监控器 - 基于官方API的实时数据可视化"""
    
    def __init__(self, robot_controller, data_logger, max_points: int = 200):
        self.robot = robot_controller
        self.data_logger = data_logger
        self.max_points = max_points
        
        # 数据存储
        self.timestamps = deque(maxlen=max_points)
        
        # IMU数据
        self.euler_roll = deque(maxlen=max_points)
        self.euler_pitch = deque(maxlen=max_points)
        self.euler_yaw = deque(maxlen=max_points)
        self.acc_x = deque(maxlen=max_points)
        self.acc_y = deque(maxlen=max_points)
        self.acc_z = deque(maxlen=max_points)
        
        # 里程计数据
        self.pose_x = deque(maxlen=max_points)
        self.pose_y = deque(maxlen=max_points)
        self.linear_x = deque(maxlen=max_points)
        self.linear_y = deque(maxlen=max_points)
        
        # 系统数据
        self.battery_levels = deque(maxlen=max_points)
        self.command_success_rate = deque(maxlen=max_points)
        
        # 绘图相关
        self.fig = None
        self.axes = None
        self.lines = []
        self.animation = None
        self.monitoring = False
        
        # 配置matplotlib
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    
    def setup_plots(self):
        """设置多子图绘制界面"""
        self.fig = plt.figure(figsize=(16, 12))
        self.fig.suptitle('TRON机器人实时监控面板', fontsize=16, fontweight='bold')
        
        # 创建子图布局 (3行3列)
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # 1. 姿态角度 (欧拉角)
        self.ax_euler = self.fig.add_subplot(gs[0, 0])
        self.ax_euler.set_title('姿态角度 (度)', fontweight='bold')
        self.ax_euler.set_ylabel('角度 (°)')
        self.ax_euler.legend(['Roll', 'Pitch', 'Yaw'])
        self.ax_euler.grid(True, alpha=0.3)
        
        # 2. 加速度
        self.ax_acc = self.fig.add_subplot(gs[0, 1])
        self.ax_acc.set_title('加速度', fontweight='bold')
        self.ax_acc.set_ylabel('加速度 (m/s²)')
        self.ax_acc.legend(['X轴', 'Y轴', 'Z轴'])
        self.ax_acc.grid(True, alpha=0.3)
        
        # 3. 电池电量
        self.ax_battery = self.fig.add_subplot(gs[0, 2])
        self.ax_battery.set_title('电池电量', fontweight='bold')
        self.ax_battery.set_ylabel('电量 (%)')
        self.ax_battery.set_ylim(0, 100)
        self.ax_battery.grid(True, alpha=0.3)
        
        # 4. 位置轨迹 (X-Y平面)
        self.ax_trajectory = self.fig.add_subplot(gs[1, 0])
        self.ax_trajectory.set_title('移动轨迹 (X-Y平面)', fontweight='bold')
        self.ax_trajectory.set_xlabel('X (m)')
        self.ax_trajectory.set_ylabel('Y (m)')
        self.ax_trajectory.grid(True, alpha=0.3)
        self.ax_trajectory.axis('equal')
        
        # 5. 线速度
        self.ax_velocity = self.fig.add_subplot(gs[1, 1])
        self.ax_velocity.set_title('线速度', fontweight='bold')
        self.ax_velocity.set_ylabel('速度 (m/s)')
        self.ax_velocity.legend(['X轴', 'Y轴'])
        self.ax_velocity.grid(True, alpha=0.3)
        
        # 6. 命令成功率
        self.ax_success_rate = self.fig.add_subplot(gs[1, 2])
        self.ax_success_rate.set_title('命令成功率', fontweight='bold')
        self.ax_success_rate.set_ylabel('成功率 (%)')
        self.ax_success_rate.set_ylim(0, 100)
        self.ax_success_rate.grid(True, alpha=0.3)
        
        # 7. 数据统计面板
        self.ax_stats = self.fig.add_subplot(gs[2, :])
        self.ax_stats.set_title('实时统计信息', fontweight='bold')
        self.ax_stats.axis('off')
        
        # 初始化线条
        self.lines = []
        
        # 姿态角线条
        euler_colors = ['red', 'green', 'blue']
        for color in euler_colors:
            line, = self.ax_euler.plot([], [], color=color, linewidth=2)
            self.lines.append(line)
        
        # 加速度线条
        acc_colors = ['red', 'green', 'blue']
        for color in acc_colors:
            line, = self.ax_acc.plot([], [], color=color, linewidth=2)
            self.lines.append(line)
        
        # 电池线条
        battery_line, = self.ax_battery.plot([], [], color='orange', linewidth=3)
        self.lines.append(battery_line)
        
        # 轨迹线条
        trajectory_line, = self.ax_trajectory.plot([], [], 'b-', linewidth=2, alpha=0.7)
        trajectory_scatter = self.ax_trajectory.scatter([], [], c='red', s=50, zorder=5)
        self.lines.extend([trajectory_line, trajectory_scatter])
        
        # 速度线条
        vel_colors = ['purple', 'orange']
        for color in vel_colors:
            line, = self.ax_velocity.plot([], [], color=color, linewidth=2)
            self.lines.append(line)
        
        # 成功率线条
        success_line, = self.ax_success_rate.plot([], [], color='green', linewidth=3)
        self.lines.append(success_line)
        
        return self.fig
    
    def update_plots(self, frame):
        """更新所有图表"""
        try:
            # 获取最新数据
            self.collect_latest_data()
            
            if len(self.timestamps) == 0:
                return self.lines
            
            # 转换为列表
            times = list(self.timestamps)
            
            line_idx = 0
            
            # 更新姿态角
            euler_data = [list(self.euler_roll), list(self.euler_pitch), list(self.euler_yaw)]
            for i, data in enumerate(euler_data):
                if data:
                    self.lines[line_idx].set_data(times, data)
                    self.update_axis_limits(self.ax_euler, times, euler_data)
                line_idx += 1
            
            # 更新加速度
            acc_data = [list(self.acc_x), list(self.acc_y), list(self.acc_z)]
            for i, data in enumerate(acc_data):
                if data:
                    self.lines[line_idx].set_data(times, data)
                    self.update_axis_limits(self.ax_acc, times, acc_data)
                line_idx += 1
            
            # 更新电池
            if self.battery_levels:
                battery_data = list(self.battery_levels)
                self.lines[line_idx].set_data(times, battery_data)
                self.update_axis_limits(self.ax_battery, times, [battery_data])
            line_idx += 1
            
            # 更新轨迹
            if self.pose_x and self.pose_y:
                x_pos = list(self.pose_x)
                y_pos = list(self.pose_y)
                
                # 轨迹线
                self.lines[line_idx].set_data(x_pos, y_pos)
                
                # 当前位置点
                if x_pos and y_pos:
                    self.lines[line_idx + 1].set_offsets([[x_pos[-1], y_pos[-1]]])
                
                # 更新轨迹图范围
                if x_pos and y_pos:
                    margin = 0.5
                    x_min, x_max = min(x_pos) - margin, max(x_pos) + margin
                    y_min, y_max = min(y_pos) - margin, max(y_pos) + margin
                    self.ax_trajectory.set_xlim(x_min, x_max)
                    self.ax_trajectory.set_ylim(y_min, y_max)
            
            line_idx += 2
            
            # 更新速度
            vel_data = [list(self.linear_x), list(self.linear_y)]
            for i, data in enumerate(vel_data):
                if data:
                    self.lines[line_idx].set_data(times, data)
                    self.update_axis_limits(self.ax_velocity, times, vel_data)
                line_idx += 1
            
            # 更新成功率
            if self.command_success_rate:
                success_data = list(self.command_success_rate)
                self.lines[line_idx].set_data(times, success_data)
                self.update_axis_limits(self.ax_success_rate, times, [success_data])
            
            # 更新统计信息
            self.update_stats_panel()
            
        except Exception as e:
            print(f"更新图表时出错: {e}")
        
        return self.lines
    
    def collect_latest_data(self):
        """收集最新数据"""
        try:
            current_time = time.time()
            
            # 获取IMU数据
            imu_data = self.robot.get_imu_data()
            if imu_data:
                self.timestamps.append(current_time)
                
                euler = imu_data.get('euler', [0, 0, 0])
                self.euler_roll.append(euler[0])
                self.euler_pitch.append(euler[1])
                self.euler_yaw.append(euler[2])
                
                acc = imu_data.get('acc', [0, 0, 0])
                self.acc_x.append(acc[0])
                self.acc_y.append(acc[1])
                self.acc_z.append(acc[2])
            
            # 获取里程计数据
            odom_data = self.robot.get_odom_data()
            if odom_data:
                pose = odom_data.get('pose_position', [0, 0, 0])
                self.pose_x.append(pose[0])
                self.pose_y.append(pose[1])
                
                linear = odom_data.get('twist_linear', [0, 0, 0])
                self.linear_x.append(linear[0])
                self.linear_y.append(linear[1])
            
            # 获取电池数据
            robot_info = self.robot.get_robot_info()
            if robot_info:
                battery = robot_info.get('battery', 0)
                self.battery_levels.append(battery)
            
            # 获取命令成功率
            if self.data_logger:
                stats = self.data_logger.get_data_statistics()
                total_cmds = stats['total_commands']
                if total_cmds > 0:
                    success_rate = stats['success_commands'] / total_cmds * 100
                    self.command_success_rate.append(success_rate)
            
        except Exception as e:
            print(f"收集数据时出错: {e}")
    
    def update_axis_limits(self, ax, times, data_sets):
        """自动更新坐标轴范围"""
        try:
            if times:
                ax.set_xlim(times[0], times[-1])
            
            all_values = []
            for data_set in data_sets:
                if data_set:
                    all_values.extend(data_set)
            
            if all_values:
                data_min, data_max = min(all_values), max(all_values)
                margin = (data_max - data_min) * 0.1 or 1
                ax.set_ylim(data_min - margin, data_max + margin)
        except:
            pass
    
    def update_stats_panel(self):
        """更新统计信息面板"""
        try:
            self.ax_stats.clear()
            self.ax_stats.axis('off')
            
            # 获取统计数据
            stats = self.data_logger.get_data_statistics() if self.data_logger else {}
            robot_info = self.robot.get_robot_info()
            
            # 创建统计文本
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            runtime = stats.get('runtime_seconds', 0)
            runtime_str = f"{int(runtime//3600):02d}:{int((runtime%3600)//60):02d}:{int(runtime%60):02d}"
            
            stats_text = f"""
当前时间: {current_time}    运行时间: {runtime_str}    连接状态: {'✅ 已连接' if self.robot.is_connected else '❌ 未连接'}

命令统计: 总数 {stats.get('total_commands', 0)} | 成功 {stats.get('success_commands', 0)} | 失败 {stats.get('failed_commands', 0)} | 成功率 {(stats.get('success_commands', 0) / max(stats.get('total_commands', 1), 1) * 100):.1f}%

传感器状态: IMU {'✅' if self.robot.imu_enabled else '❌'} | 里程计 {'✅' if self.robot.odom_enabled else '❌'} | 楼梯模式 {'✅' if self.robot.stair_mode_enabled else '❌'}

电池电量: {robot_info.get('battery', 0) if robot_info else 0}%    软件版本: {robot_info.get('sw_version', '未知') if robot_info else '未知'}

数据记录: 基本信息 {len(self.data_logger.robot_info_data) if self.data_logger else 0} | IMU {len(self.data_logger.imu_data) if self.data_logger else 0} | 里程计 {len(self.data_logger.odom_data) if self.data_logger else 0} | 命令 {len(self.data_logger.command_logs) if self.data_logger else 0}
            """.strip()
            
            self.ax_stats.text(0.05, 0.5, stats_text, transform=self.ax_stats.transAxes, 
                             fontsize=11, verticalalignment='center', fontfamily='monospace',
                             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
                             
        except Exception as e:
            print(f"更新统计面板时出错: {e}")
    
    def start_monitoring(self):
        """开始实时监控"""
        if not self.robot.is_connected:
            print("❌ 机器人未连接，无法开始监控")
            return None
        
        self.monitoring = True
        
        # 启用传感器
        self.robot.enable_imu(True)
        self.robot.enable_odom(True)
        
        # 设置图表
        self.setup_plots()
        
        # 启动动画
        self.animation = FuncAnimation(
            self.fig, self.update_plots,
            interval=500, blit=False, cache_frame_data=False
        )
        
        print("🔥 实时监控已启动")
        plt.show()
        
        return self.animation
    
    def stop_monitoring(self):
        """停止监控"""
        self.monitoring = False
        if self.animation:
            self.animation.event_source.stop()
        plt.close('all')
        print("⏹️ 实时监控已停止")
