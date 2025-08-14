# robot_data_logger.py (修复版)
import os
import time
import threading
import json
import pandas as pd
from datetime import datetime, timedelta
from collections import defaultdict, deque
from typing import Dict, Any, Optional
import logging


class RobotDataLogger:
    """机器人数据记录器 - 基于官方API的完整数据保存系统"""

    def __init__(self, robot_controller, log_dir: str = "logs"):
        self.robot = robot_controller
        self.log_dir = log_dir
        self.ensure_log_directory()

        # 数据存储
        self.robot_info_data = deque(maxlen=1000)
        self.imu_data = deque(maxlen=1000)
        self.odom_data = deque(maxlen=1000)
        self.command_logs = deque(maxlen=1000)
        self.error_logs = deque(maxlen=1000)
        self.performance_data = deque(maxlen=1000)

        # 保存控制
        self.save_interval = 5.0  # 5秒保存一次
        self.logging_active = False
        self.save_thread = None
        self.last_save_time = time.time()

        # 数据统计
        self.data_stats = {
            'total_commands': 0,
            'success_commands': 0,
            'failed_commands': 0,
            'session_start': datetime.now(),
            'last_battery_level': 0,
            'total_distance': 0.0
        }

        self.setup_logging()

    def ensure_log_directory(self):
        """确保日志目录存在"""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            print(f"创建日志目录: {self.log_dir}")

    def setup_logging(self):
        """设置日志系统 - 修复路径问题"""
        try:
            # 确保日志目录存在
            self.ensure_log_directory()

            # 创建日志文件路径
            log_file = os.path.join(self.log_dir, f"robot_monitor_{datetime.now().strftime('%Y%m%d')}.log")

            # 创建logger
            self.logger = logging.getLogger('RobotDataLogger')
            self.logger.setLevel(logging.INFO)

            # 避免重复添加handler
            if not self.logger.handlers:
                # 文件handler
                file_handler = logging.FileHandler(log_file, encoding='utf-8')
                file_handler.setLevel(logging.INFO)

                # 控制台handler
                console_handler = logging.StreamHandler()
                console_handler.setLevel(logging.INFO)

                # 格式化器
                formatter = logging.Formatter(
                    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
                )
                file_handler.setFormatter(formatter)
                console_handler.setFormatter(formatter)

                # 添加handlers
                self.logger.addHandler(file_handler)
                self.logger.addHandler(console_handler)

            print(f"✅ 日志系统初始化成功: {log_file}")

        except Exception as e:
            print(f"❌ 日志系统初始化失败: {e}")
            # 创建一个基本的logger作为备份
            self.logger = logging.getLogger('RobotDataLogger')

    def start_logging(self):
        """开始数据记录"""
        if self.logging_active:
            return

        self.logging_active = True
        self.data_stats['session_start'] = datetime.now()

        # 注册机器人事件回调
        self.register_robot_callbacks()

        # 启动保存线程
        self.save_thread = threading.Thread(target=self._save_loop, daemon=True)
        self.save_thread.start()

        # 启动状态监控线程
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

        self.logger.info("数据记录已启动")
        print("🔥 数据记录系统已启动 - 每5秒自动保存")

    def stop_logging(self):
        """停止数据记录"""
        self.logging_active = False
        self.save_all_data()  # 最后保存一次
        self.logger.info("数据记录已停止")
        print("⏹️ 数据记录系统已停止")

    def register_robot_callbacks(self):
        """注册机器人事件回调"""
        # 重写机器人的消息处理方法来记录数据
        if hasattr(self.robot, 'handle_notify_message'):
            original_handle_notify = self.robot.handle_notify_message

            def enhanced_notify_handler(data):
                self.log_notify_message(data)
                return original_handle_notify(data)

            self.robot.handle_notify_message = enhanced_notify_handler

        if hasattr(self.robot, 'handle_response_message'):
            original_handle_response = self.robot.handle_response_message

            def enhanced_response_handler(data):
                self.log_response_message(data)
                return original_handle_response(data)

            self.robot.handle_response_message = enhanced_response_handler

    def log_notify_message(self, data: Dict[str, Any]):
        """记录通知消息"""
        try:
            title = data.get('title', '')
            timestamp = datetime.now()

            if title == 'notify_robot_info':
                # 机器人基本信息
                info_data = data.get('data', {})
                record = {
                    'timestamp': timestamp,
                    'accid': info_data.get('accid', ''),
                    'sw_version': info_data.get('sw_version', ''),
                    'battery': info_data.get('battery', 0),
                    'imu_status': info_data.get('imu', ''),
                    'motor_status': info_data.get('motor', ''),
                    'camera_status': info_data.get('camera', '')
                }
                self.robot_info_data.append(record)
                self.data_stats['last_battery_level'] = record['battery']

            elif title == 'notify_imu':
                # IMU数据
                imu_info = data.get('data', {})
                record = {
                    'timestamp': timestamp,
                    'euler_roll': imu_info.get('euler', [0, 0, 0])[0],
                    'euler_pitch': imu_info.get('euler', [0, 0, 0])[1],
                    'euler_yaw': imu_info.get('euler', [0, 0, 0])[2],
                    'acc_x': imu_info.get('acc', [0, 0, 0])[0],
                    'acc_y': imu_info.get('acc', [0, 0, 0])[1],
                    'acc_z': imu_info.get('acc', [0, 0, 0])[2],
                    'gyro_x': imu_info.get('gyro', [0, 0, 0])[0],
                    'gyro_y': imu_info.get('gyro', [0, 0, 0])[1],
                    'gyro_z': imu_info.get('gyro', [0, 0, 0])[2],
                    'quat_w': imu_info.get('quat', [0, 0, 0, 0])[0],
                    'quat_x': imu_info.get('quat', [0, 0, 0, 0])[1],
                    'quat_y': imu_info.get('quat', [0, 0, 0, 0])[2],
                    'quat_z': imu_info.get('quat', [0, 0, 0, 0])[3]
                }
                self.imu_data.append(record)

            elif title == 'notify_odom':
                # 里程计数据
                odom_info = data.get('data', {})
                record = {
                    'timestamp': timestamp,
                    'pose_x': odom_info.get('pose_position', [0, 0, 0])[0],
                    'pose_y': odom_info.get('pose_position', [0, 0, 0])[1],
                    'pose_z': odom_info.get('pose_position', [0, 0, 0])[2],
                    'orient_x': odom_info.get('pose_orientation', [0, 0, 0, 0])[0],
                    'orient_y': odom_info.get('pose_orientation', [0, 0, 0, 0])[1],
                    'orient_z': odom_info.get('pose_orientation', [0, 0, 0, 0])[2],
                    'orient_w': odom_info.get('pose_orientation', [0, 0, 0, 0])[3],
                    'linear_x': odom_info.get('twist_linear', [0, 0, 0])[0],
                    'linear_y': odom_info.get('twist_linear', [0, 0, 0])[1],
                    'linear_z': odom_info.get('twist_linear', [0, 0, 0])[2],
                    'angular_x': odom_info.get('twist_angular', [0, 0, 0])[0],
                    'angular_y': odom_info.get('twist_angular', [0, 0, 0])[1],
                    'angular_z': odom_info.get('twist_angular', [0, 0, 0])[2]
                }
                self.odom_data.append(record)

            elif 'fail' in data.get('data', {}).get('result', ''):
                # 错误通知
                error_record = {
                    'timestamp': timestamp,
                    'error_type': title,
                    'result': data.get('data', {}).get('result', ''),
                    'raw_data': json.dumps(data, ensure_ascii=False)
                }
                self.error_logs.append(error_record)
                self.data_stats['failed_commands'] += 1

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"记录通知消息时出错: {e}")
            else:
                print(f"记录通知消息时出错: {e}")

    def log_response_message(self, data: Dict[str, Any]):
        """记录响应消息"""
        try:
            timestamp = datetime.now()
            title = data.get('title', '')
            result = data.get('data', {}).get('result', '')

            command_record = {
                'timestamp': timestamp,
                'command': title,
                'result': result,
                'guid': data.get('guid', ''),
                'success': result == 'success',
                'raw_data': json.dumps(data, ensure_ascii=False)
            }

            self.command_logs.append(command_record)
            self.data_stats['total_commands'] += 1

            if result == 'success':
                self.data_stats['success_commands'] += 1
            else:
                self.data_stats['failed_commands'] += 1

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"记录响应消息时出错: {e}")
            else:
                print(f"记录响应消息时出错: {e}")

    def _monitor_loop(self):
        """监控循环 - 每秒在终端显示数据"""
        while self.logging_active:
            try:
                self.print_terminal_status()
                self.log_performance_data()
                time.sleep(1)
            except Exception as e:
                if hasattr(self, 'logger'):
                    self.logger.error(f"监控循环错误: {e}")
                else:
                    print(f"监控循环错误: {e}")
                break

    def print_terminal_status(self):
        """在终端打印实时状态"""
        try:
            current_time = datetime.now().strftime("%H:%M:%S")

            # 获取最新数据
            robot_info = {}
            battery = 0

            # 从机器人控制器获取基本信息
            if hasattr(self.robot, 'robot_info') and self.robot.robot_info:
                robot_info = self.robot.robot_info
                battery = robot_info.get('battery', 0)

            # 获取传感器数据
            imu_data = {}
            odom_data = {}
            if hasattr(self.robot, 'imu_data') and self.robot.imu_data:
                imu_data = self.robot.imu_data
            if hasattr(self.robot, 'odom_data') and self.robot.odom_data:
                odom_data = self.robot.odom_data

            # 终端状态显示
            print(f"\n{'=' * 80}")
            print(f"🤖 TRON机器人实时监控 [{current_time}]")
            print(f"{'=' * 80}")

            # 基本状态
            print(f"🔋 电池电量: {battery}% | 🔗 连接状态: {'✅' if self.robot.is_connected else '❌'}")
            print(
                f"📊 总命令: {self.data_stats['total_commands']} | ✅ 成功: {self.data_stats['success_commands']} | ❌ 失败: {self.data_stats['failed_commands']}")

            # 传感器状态
            sensors_status = []
            if hasattr(self.robot, 'imu_enabled') and self.robot.imu_enabled:
                sensors_status.append("📡IMU")
            if hasattr(self.robot, 'odom_enabled') and self.robot.odom_enabled:
                sensors_status.append("🗺️里程计")

            print(f"🔧 传感器: {' | '.join(sensors_status) if sensors_status else '无'}")

            # IMU数据显示
            if imu_data:
                euler = imu_data.get('euler', [0, 0, 0])
                acc = imu_data.get('acc', [0, 0, 0])
                print(f"🧭 姿态角: Roll={euler[0]:.2f}° Pitch={euler[1]:.2f}° Yaw={euler[2]:.2f}°")
                print(f"🏃 加速度: X={acc[0]:.3f} Y={acc[1]:.3f} Z={acc[2]:.3f} m/s²")

            # 里程计数据显示
            if odom_data:
                pos = odom_data.get('pose_position', [0, 0, 0])
                linear = odom_data.get('twist_linear', [0, 0, 0])
                print(f"📍 位置: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f} m")
                print(f"🏃‍♂️ 速度: X={linear[0]:.3f} Y={linear[1]:.3f} Z={linear[2]:.3f} m/s")

            # 数据统计
            runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
            print(f"⏱️ 运行时间: {int(runtime // 3600):02d}:{int((runtime % 3600) // 60):02d}:{int(runtime % 60):02d}")
            print(
                f"💾 数据条数: 基本信息({len(self.robot_info_data)}) IMU({len(self.imu_data)}) 里程计({len(self.odom_data)})")

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"终端状态显示错误: {e}")
            else:
                print(f"终端状态显示错误: {e}")

    def log_performance_data(self):
        """记录性能数据"""
        try:
            timestamp = datetime.now()

            # 计算成功率
            total_cmds = self.data_stats['total_commands']
            success_rate = (self.data_stats['success_commands'] / total_cmds * 100) if total_cmds > 0 else 0

            # 计算运行时间
            runtime = (timestamp - self.data_stats['session_start']).total_seconds()

            perf_record = {
                'timestamp': timestamp,
                'total_commands': total_cmds,
                'success_commands': self.data_stats['success_commands'],
                'failed_commands': self.data_stats['failed_commands'],
                'success_rate': success_rate,
                'runtime_seconds': runtime,
                'battery_level': self.data_stats['last_battery_level'],
                'robot_connected': self.robot.is_connected,
            }

            self.performance_data.append(perf_record)

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"记录性能数据时出错: {e}")
            else:
                print(f"记录性能数据时出错: {e}")

    def _save_loop(self):
        """保存循环"""
        while self.logging_active:
            try:
                time.sleep(self.save_interval)
                if time.time() - self.last_save_time >= self.save_interval:
                    self.save_all_data()
                    self.last_save_time = time.time()
            except Exception as e:
                if hasattr(self, 'logger'):
                    self.logger.error(f"保存循环错误: {e}")
                else:
                    print(f"保存循环错误: {e}")
                break

    def save_all_data(self):
        """保存所有数据到xlsx文件"""
        try:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

            # 1. 保存机器人基本信息
            if self.robot_info_data:
                df_robot_info = pd.DataFrame(list(self.robot_info_data))
                robot_info_file = os.path.join(self.log_dir, f"robot_info_{timestamp_str}.xlsx")
                df_robot_info.to_excel(robot_info_file, index=False, engine='openpyxl')

            # 2. 保存IMU数据
            if self.imu_data:
                df_imu = pd.DataFrame(list(self.imu_data))
                imu_file = os.path.join(self.log_dir, f"imu_data_{timestamp_str}.xlsx")
                df_imu.to_excel(imu_file, index=False, engine='openpyxl')

            # 3. 保存里程计数据
            if self.odom_data:
                df_odom = pd.DataFrame(list(self.odom_data))
                odom_file = os.path.join(self.log_dir, f"odom_data_{timestamp_str}.xlsx")
                df_odom.to_excel(odom_file, index=False, engine='openpyxl')

            # 4. 保存命令日志
            if self.command_logs:
                df_commands = pd.DataFrame(list(self.command_logs))
                commands_file = os.path.join(self.log_dir, f"commands_{timestamp_str}.xlsx")
                df_commands.to_excel(commands_file, index=False, engine='openpyxl')

            # 5. 保存错误日志
            if self.error_logs:
                df_errors = pd.DataFrame(list(self.error_logs))
                errors_file = os.path.join(self.log_dir, f"errors_{timestamp_str}.xlsx")
                df_errors.to_excel(errors_file, index=False, engine='openpyxl')

            # 6. 保存性能数据
            if self.performance_data:
                df_performance = pd.DataFrame(list(self.performance_data))
                performance_file = os.path.join(self.log_dir, f"performance_{timestamp_str}.xlsx")
                df_performance.to_excel(performance_file, index=False, engine='openpyxl')

            # 7. 保存汇总报告
            self.save_summary_report(timestamp_str)

            print(f"💾 [{datetime.now().strftime('%H:%M:%S')}] 数据已保存到 {self.log_dir}/")

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"保存数据时出错: {e}")
            else:
                print(f"保存数据时出错: {e}")

    def save_summary_report(self, timestamp_str: str):
        """保存汇总报告"""
        try:
            runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
            success_rate = (self.data_stats['success_commands'] / self.data_stats['total_commands'] * 100) if \
            self.data_stats['total_commands'] > 0 else 0

            summary_data = {
                '指标': [
                    '会话开始时间', '会话运行时间(秒)', '总命令数', '成功命令数',
                    '失败命令数', '成功率(%)', '最后电池电量(%)', '基本信息记录数',
                    'IMU记录数', '里程计记录数', '错误记录数', '性能记录数'
                ],
                '数值': [
                    self.data_stats['session_start'].strftime('%Y-%m-%d %H:%M:%S'),
                    f"{runtime:.1f}",
                    self.data_stats['total_commands'],
                    self.data_stats['success_commands'],
                    self.data_stats['failed_commands'],
                    f"{success_rate:.1f}",
                    self.data_stats['last_battery_level'],
                    len(self.robot_info_data),
                    len(self.imu_data),
                    len(self.odom_data),
                    len(self.error_logs),
                    len(self.performance_data)
                ]
            }

            df_summary = pd.DataFrame(summary_data)
            summary_file = os.path.join(self.log_dir, f"summary_report_{timestamp_str}.xlsx")
            df_summary.to_excel(summary_file, index=False, engine='openpyxl')

        except Exception as e:
            if hasattr(self, 'logger'):
                self.logger.error(f"保存汇总报告时出错: {e}")
            else:
                print(f"保存汇总报告时出错: {e}")

    def get_data_statistics(self) -> Dict[str, Any]:
        """获取数据统计信息"""
        runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
        return {
            **self.data_stats,
            'runtime_seconds': runtime,
            'data_counts': {
                'robot_info': len(self.robot_info_data),
                'imu': len(self.imu_data),
                'odom': len(self.odom_data),
                'commands': len(self.command_logs),
                'errors': len(self.error_logs),
                'performance': len(self.performance_data)
            }
        }

# # robot_data_logger.py
# import os
# import time
# import threading
# import json
# import pandas as pd
# from datetime import datetime, timedelta
# from collections import defaultdict, deque
# from typing import Dict, Any, Optional
# import logging
#
# class RobotDataLogger:
#     """机器人数据记录器 - 基于官方API的完整数据保存系统"""
#
#     def __init__(self, robot_controller, log_dir: str = "logs"):
#         self.robot = robot_controller
#         self.log_dir = log_dir
#         self.ensure_log_directory()
#
#         # 数据存储
#         self.robot_info_data = deque(maxlen=1000)  # 机器人基本信息
#         self.imu_data = deque(maxlen=1000)         # IMU数据
#         self.odom_data = deque(maxlen=1000)        # 里程计数据
#         self.command_logs = deque(maxlen=1000)     # 命令日志
#         self.error_logs = deque(maxlen=1000)       # 错误日志
#         self.performance_data = deque(maxlen=1000) # 性能数据
#
#         # 保存控制
#         self.save_interval = 5.0  # 5秒保存一次
#         self.logging_active = False
#         self.save_thread = None
#         self.last_save_time = time.time()
#
#         # 数据统计
#         self.data_stats = {
#             'total_commands': 0,
#             'success_commands': 0,
#             'failed_commands': 0,
#             'session_start': datetime.now(),
#             'last_battery_level': 0,
#             'total_distance': 0.0
#         }
#
#         self.setup_logging()
#
#     def ensure_log_directory(self):
#         """确保日志目录存在"""
#         if not os.path.exists(self.log_dir):
#             os.makedirs(self.log_dir)
#             print(f"创建日志目录: {self.log_dir}")
#
#     def setup_logging(self):
#         """设置日志系统"""
#         log_file = os.path.join(self.log_dir, f"robot_monitor_{datetime.now().strftime('%Y%m%d')}.log")
#
#         # 避免重复配置
#         if not logging.getLogger().handlers:
#             logging.basicConfig(
#                 level=logging.INFO,
#                 format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
#                 handlers=[
#                     logging.FileHandler(log_file, encoding='utf-8'),
#                     logging.StreamHandler()
#                 ]
#             )
#         self.logger = logging.getLogger('RobotDataLogger')
#
#     # def setup_logging(self):
#     #     """设置日志系统"""
#     #     log_file = os.path.join(self.log_dir, f"robot_monitor_{datetime.now().strftime('%Y%m%d')}.log")
#     #     logging.basicConfig(
#     #         level=logging.INFO,
#     #         format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
#     #         handlers=[
#     #             logging.FileHandler(log_file, encoding='utf-8'),
#     #             logging.StreamHandler()
#     #         ]
#     #     )
#     #     self.logger = logging.getLogger('RobotDataLogger')
#
#     def start_logging(self):
#         """开始数据记录"""
#         if self.logging_active:
#             return
#
#         self.logging_active = True
#         self.data_stats['session_start'] = datetime.now()
#
#         # 注册机器人事件回调
#         self.register_robot_callbacks()
#
#         # 启动保存线程
#         self.save_thread = threading.Thread(target=self._save_loop, daemon=True)
#         self.save_thread.start()
#
#         # 启动状态监控线程
#         self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
#         self.monitor_thread.start()
#
#         self.logger.info("数据记录已启动")
#         print("🔥 数据记录系统已启动 - 每5秒自动保存")
#
#     def stop_logging(self):
#         """停止数据记录"""
#         self.logging_active = False
#         self.save_all_data()  # 最后保存一次
#         self.logger.info("数据记录已停止")
#         print("⏹️ 数据记录系统已停止")
#
#     def register_robot_callbacks(self):
#         """注册机器人事件回调"""
#         # 重写机器人的消息处理方法来记录数据
#         original_handle_notify = self.robot.handle_notify_message
#         original_handle_response = self.robot.handle_response_message
#
#         def enhanced_notify_handler(data):
#             self.log_notify_message(data)
#             return original_handle_notify(data)
#
#         def enhanced_response_handler(data):
#             self.log_response_message(data)
#             return original_handle_response(data)
#
#         self.robot.handle_notify_message = enhanced_notify_handler
#         self.robot.handle_response_message = enhanced_response_handler
#
#     def log_notify_message(self, data: Dict[str, Any]):
#         """记录通知消息"""
#         try:
#             title = data.get('title', '')
#             timestamp = datetime.now()
#
#             if title == 'notify_robot_info':
#                 # 机器人基本信息
#                 info_data = data.get('data', {})
#                 record = {
#                     'timestamp': timestamp,
#                     'accid': info_data.get('accid', ''),
#                     'sw_version': info_data.get('sw_version', ''),
#                     'battery': info_data.get('battery', 0),
#                     'imu_status': info_data.get('imu', ''),
#                     'motor_status': info_data.get('motor', ''),
#                     'camera_status': info_data.get('camera', '')
#                 }
#                 self.robot_info_data.append(record)
#                 self.data_stats['last_battery_level'] = record['battery']
#
#             elif title == 'notify_imu':
#                 # IMU数据
#                 imu_info = data.get('data', {})
#                 record = {
#                     'timestamp': timestamp,
#                     'euler_roll': imu_info.get('euler', [0,0,0])[0],
#                     'euler_pitch': imu_info.get('euler', [0,0,0])[1],
#                     'euler_yaw': imu_info.get('euler', [0,0,0])[2],
#                     'acc_x': imu_info.get('acc', [0,0,0])[0],
#                     'acc_y': imu_info.get('acc', [0,0,0])[1],
#                     'acc_z': imu_info.get('acc', [0,0,0])[2],
#                     'gyro_x': imu_info.get('gyro', [0,0,0])[0],
#                     'gyro_y': imu_info.get('gyro', [0,0,0])[1],
#                     'gyro_z': imu_info.get('gyro', [0,0,0])[2],
#                     'quat_w': imu_info.get('quat', [0,0,0,0])[0],
#                     'quat_x': imu_info.get('quat', [0,0,0,0])[1],
#                     'quat_y': imu_info.get('quat', [0,0,0,0])[2],
#                     'quat_z': imu_info.get('quat', [0,0,0,0])[3]
#                 }
#                 self.imu_data.append(record)
#
#             elif title == 'notify_odom':
#                 # 里程计数据
#                 odom_info = data.get('data', {})
#                 record = {
#                     'timestamp': timestamp,
#                     'pose_x': odom_info.get('pose_position', [0,0,0])[0],
#                     'pose_y': odom_info.get('pose_position', [0,0,0])[1],
#                     'pose_z': odom_info.get('pose_position', [0,0,0])[2],
#                     'orient_x': odom_info.get('pose_orientation', [0,0,0,0])[0],
#                     'orient_y': odom_info.get('pose_orientation', [0,0,0,0])[1],
#                     'orient_z': odom_info.get('pose_orientation', [0,0,0,0])[2],
#                     'orient_w': odom_info.get('pose_orientation', [0,0,0,0])[3],
#                     'linear_x': odom_info.get('twist_linear', [0,0,0])[0],
#                     'linear_y': odom_info.get('twist_linear', [0,0,0])[1],
#                     'linear_z': odom_info.get('twist_linear', [0,0,0])[2],
#                     'angular_x': odom_info.get('twist_angular', [0,0,0])[0],
#                     'angular_y': odom_info.get('twist_angular', [0,0,0])[1],
#                     'angular_z': odom_info.get('twist_angular', [0,0,0])[2]
#                 }
#                 self.odom_data.append(record)
#
#             elif title.startswith('notify_') and 'fail' in data.get('data', {}).get('result', ''):
#                 # 错误通知
#                 error_record = {
#                     'timestamp': timestamp,
#                     'error_type': title,
#                     'result': data.get('data', {}).get('result', ''),
#                     'raw_data': json.dumps(data, ensure_ascii=False)
#                 }
#                 self.error_logs.append(error_record)
#                 self.data_stats['failed_commands'] += 1
#
#         except Exception as e:
#             self.logger.error(f"记录通知消息时出错: {e}")
#
#     def log_response_message(self, data: Dict[str, Any]):
#         """记录响应消息"""
#         try:
#             timestamp = datetime.now()
#             title = data.get('title', '')
#             result = data.get('data', {}).get('result', '')
#
#             command_record = {
#                 'timestamp': timestamp,
#                 'command': title,
#                 'result': result,
#                 'guid': data.get('guid', ''),
#                 'success': result == 'success',
#                 'raw_data': json.dumps(data, ensure_ascii=False)
#             }
#
#             self.command_logs.append(command_record)
#             self.data_stats['total_commands'] += 1
#
#             if result == 'success':
#                 self.data_stats['success_commands'] += 1
#             else:
#                 self.data_stats['failed_commands'] += 1
#
#         except Exception as e:
#             self.logger.error(f"记录响应消息时出错: {e}")
#
#     def log_performance_data(self):
#         """记录性能数据"""
#         try:
#             timestamp = datetime.now()
#
#             # 计算成功率
#             total_cmds = self.data_stats['total_commands']
#             success_rate = (self.data_stats['success_commands'] / total_cmds * 100) if total_cmds > 0 else 0
#
#             # 计算运行时间
#             runtime = (timestamp - self.data_stats['session_start']).total_seconds()
#
#             perf_record = {
#                 'timestamp': timestamp,
#                 'total_commands': total_cmds,
#                 'success_commands': self.data_stats['success_commands'],
#                 'failed_commands': self.data_stats['failed_commands'],
#                 'success_rate': success_rate,
#                 'runtime_seconds': runtime,
#                 'battery_level': self.data_stats['last_battery_level'],
#                 'robot_connected': self.robot.is_connected,
#                 'imu_enabled': self.robot.imu_enabled,
#                 'odom_enabled': self.robot.odom_enabled,
#                 'stair_mode': self.robot.stair_mode_enabled
#             }
#
#             self.performance_data.append(perf_record)
#
#         except Exception as e:
#             self.logger.error(f"记录性能数据时出错: {e}")
#
#     def _monitor_loop(self):
#         """监控循环 - 每秒在终端显示数据"""
#         while self.logging_active:
#             try:
#                 self.print_terminal_status()
#                 self.log_performance_data()
#                 time.sleep(1)
#             except Exception as e:
#                 self.logger.error(f"监控循环错误: {e}")
#                 break
#
#     def print_terminal_status(self):
#         """在终端打印实时状态"""
#         try:
#             # 清除控制台（可选）
#             # os.system('cls' if os.name == 'nt' else 'clear')
#
#             current_time = datetime.now().strftime("%H:%M:%S")
#
#             # 获取最新数据
#             robot_info = self.robot.get_robot_info()
#             battery = robot_info.get('battery', 0) if robot_info else 0
#
#             imu_data = self.robot.get_imu_data()
#             odom_data = self.robot.get_odom_data()
#
#             # 终端状态显示
#             print(f"\n{'='*80}")
#             print(f"🤖 TRON机器人实时监控 [{current_time}]")
#             print(f"{'='*80}")
#
#             # 基本状态
#             print(f"🔋 电池电量: {battery}% | 🔗 连接状态: {'✅' if self.robot.is_connected else '❌'}")
#             print(f"📊 总命令: {self.data_stats['total_commands']} | ✅ 成功: {self.data_stats['success_commands']} | ❌ 失败: {self.data_stats['failed_commands']}")
#
#             # 传感器状态
#             sensors_status = []
#             if self.robot.imu_enabled:
#                 sensors_status.append("📡IMU")
#             if self.robot.odom_enabled:
#                 sensors_status.append("🗺️里程计")
#             if self.robot.stair_mode_enabled:
#                 sensors_status.append("🪜楼梯模式")
#
#             print(f"🔧 传感器: {' | '.join(sensors_status) if sensors_status else '无'}")
#
#             # IMU数据显示
#             if imu_data:
#                 euler = imu_data.get('euler', [0,0,0])
#                 acc = imu_data.get('acc', [0,0,0])
#                 print(f"🧭 姿态角: Roll={euler[0]:.2f}° Pitch={euler[1]:.2f}° Yaw={euler[2]:.2f}°")
#                 print(f"🏃 加速度: X={acc[0]:.3f} Y={acc[1]:.3f} Z={acc[2]:.3f} m/s²")
#
#             # 里程计数据显示
#             if odom_data:
#                 pos = odom_data.get('pose_position', [0,0,0])
#                 linear = odom_data.get('twist_linear', [0,0,0])
#                 print(f"📍 位置: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f} m")
#                 print(f"🏃‍♂️ 速度: X={linear[0]:.3f} Y={linear[1]:.3f} Z={linear[2]:.3f} m/s")
#
#             # 数据统计
#             runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
#             print(f"⏱️ 运行时间: {int(runtime//3600):02d}:{int((runtime%3600)//60):02d}:{int(runtime%60):02d}")
#             print(f"💾 数据条数: 基本信息({len(self.robot_info_data)}) IMU({len(self.imu_data)}) 里程计({len(self.odom_data)})")
#
#         except Exception as e:
#             self.logger.error(f"终端状态显示错误: {e}")
#
#     def _save_loop(self):
#         """保存循环"""
#         while self.logging_active:
#             try:
#                 time.sleep(self.save_interval)
#                 if time.time() - self.last_save_time >= self.save_interval:
#                     self.save_all_data()
#                     self.last_save_time = time.time()
#             except Exception as e:
#                 self.logger.error(f"保存循环错误: {e}")
#                 break
#
#     def save_all_data(self):
#         """保存所有数据到xlsx文件"""
#         try:
#             timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
#
#             # 1. 保存机器人基本信息
#             if self.robot_info_data:
#                 df_robot_info = pd.DataFrame(list(self.robot_info_data))
#                 robot_info_file = os.path.join(self.log_dir, f"robot_info_{timestamp_str}.xlsx")
#                 df_robot_info.to_excel(robot_info_file, index=False, engine='openpyxl')
#
#             # 2. 保存IMU数据
#             if self.imu_data:
#                 df_imu = pd.DataFrame(list(self.imu_data))
#                 imu_file = os.path.join(self.log_dir, f"imu_data_{timestamp_str}.xlsx")
#                 df_imu.to_excel(imu_file, index=False, engine='openpyxl')
#
#             # 3. 保存里程计数据
#             if self.odom_data:
#                 df_odom = pd.DataFrame(list(self.odom_data))
#                 odom_file = os.path.join(self.log_dir, f"odom_data_{timestamp_str}.xlsx")
#                 df_odom.to_excel(odom_file, index=False, engine='openpyxl')
#
#             # 4. 保存命令日志
#             if self.command_logs:
#                 df_commands = pd.DataFrame(list(self.command_logs))
#                 commands_file = os.path.join(self.log_dir, f"commands_{timestamp_str}.xlsx")
#                 df_commands.to_excel(commands_file, index=False, engine='openpyxl')
#
#             # 5. 保存错误日志
#             if self.error_logs:
#                 df_errors = pd.DataFrame(list(self.error_logs))
#                 errors_file = os.path.join(self.log_dir, f"errors_{timestamp_str}.xlsx")
#                 df_errors.to_excel(errors_file, index=False, engine='openpyxl')
#
#             # 6. 保存性能数据
#             if self.performance_data:
#                 df_performance = pd.DataFrame(list(self.performance_data))
#                 performance_file = os.path.join(self.log_dir, f"performance_{timestamp_str}.xlsx")
#                 df_performance.to_excel(performance_file, index=False, engine='openpyxl')
#
#             # 7. 保存汇总报告
#             self.save_summary_report(timestamp_str)
#
#             print(f"💾 [{datetime.now().strftime('%H:%M:%S')}] 数据已保存到 {self.log_dir}/")
#
#         except Exception as e:
#             self.logger.error(f"保存数据时出错: {e}")
#
#     def save_summary_report(self, timestamp_str: str):
#         """保存汇总报告"""
#         try:
#             runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
#             success_rate = (self.data_stats['success_commands'] / self.data_stats['total_commands'] * 100) if self.data_stats['total_commands'] > 0 else 0
#
#             summary_data = {
#                 '指标': [
#                     '会话开始时间', '会话运行时间(秒)', '总命令数', '成功命令数',
#                     '失败命令数', '成功率(%)', '最后电池电量(%)', '基本信息记录数',
#                     'IMU记录数', '里程计记录数', '错误记录数', '性能记录数'
#                 ],
#                 '数值': [
#                     self.data_stats['session_start'].strftime('%Y-%m-%d %H:%M:%S'),
#                     f"{runtime:.1f}",
#                     self.data_stats['total_commands'],
#                     self.data_stats['success_commands'],
#                     self.data_stats['failed_commands'],
#                     f"{success_rate:.1f}",
#                     self.data_stats['last_battery_level'],
#                     len(self.robot_info_data),
#                     len(self.imu_data),
#                     len(self.odom_data),
#                     len(self.error_logs),
#                     len(self.performance_data)
#                 ]
#             }
#
#             df_summary = pd.DataFrame(summary_data)
#             summary_file = os.path.join(self.log_dir, f"summary_report_{timestamp_str}.xlsx")
#             df_summary.to_excel(summary_file, index=False, engine='openpyxl')
#
#         except Exception as e:
#             self.logger.error(f"保存汇总报告时出错: {e}")
#
#     def get_data_statistics(self) -> Dict[str, Any]:
#         """获取数据统计信息"""
#         runtime = (datetime.now() - self.data_stats['session_start']).total_seconds()
#         return {
#             **self.data_stats,
#             'runtime_seconds': runtime,
#             'data_counts': {
#                 'robot_info': len(self.robot_info_data),
#                 'imu': len(self.imu_data),
#                 'odom': len(self.odom_data),
#                 'commands': len(self.command_logs),
#                 'errors': len(self.error_logs),
#                 'performance': len(self.performance_data)
#             }
#         }
