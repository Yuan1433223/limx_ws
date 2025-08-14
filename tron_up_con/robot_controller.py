import os
import json
import uuid
import threading
import time
import websocket
import logging
import queue
from datetime import datetime
from typing import Dict, Any, Callable, Optional, List
from enum import Enum

class RobotController:
    def __init__(self, accid: str, host: str = "10.192.1.2", port: int = 5000):
        self.accid = accid
        self.ws_url = f"ws://{host}:{port}"
        self.ws_client = None
        self.is_connected = False
        self.should_exit = False
        
        # 消息处理
        self.response_callbacks = {}
        self.robot_status = {}
        self.robot_info = {}  # 新增：机器人基本信息
        
        # 命令确认机制（仅适用于有响应的命令）
        self.pending_commands = {}
        self.command_timeout = 5.0
        self.max_retries = 3
        
        # 连续运动控制（针对twist命令的特殊处理）
        self.twist_thread = None
        self.twist_running = False
        self.current_twist = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.twist_interval = 0.1  # 100ms发送一次twist命令
        
        # 状态监控
        self.imu_enabled = False
        self.odom_enabled = False
        self.stair_mode_enabled = False
        self.current_light_effect = 0
        
        self.setup_logging()

    def setup_logging(self):
        """设置日志系统"""
        # 确保logs目录存在
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            print(f"创建日志目录: {log_dir}")

        # 避免重复配置，检查是否已经配置过
        if not logging.getLogger().handlers:
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                handlers=[
                    logging.FileHandler('logs/robot_control.log', encoding='utf-8'),
                    logging.StreamHandler()
                ]
            )
        self.logger = logging.getLogger(__name__)
    # def setup_logging(self):
    #     """设置日志系统"""
    #     logging.basicConfig(
    #         level=logging.INFO,
    #         format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    #         handlers=[
    #             logging.FileHandler('logs/robot_control.log'),
    #             logging.StreamHandler()
    #         ]
    #     )
    #     self.logger = logging.getLogger(__name__)

    def connect(self) -> bool:
        """连接到机器人"""
        try:
            self.ws_client = websocket.WebSocketApp(
                self.ws_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_close=self.on_close,
                on_error=self.on_error
            )

            self.ws_thread = threading.Thread(target=self.ws_client.run_forever, daemon=True)
            self.ws_thread.start()

            # 等待连接建立
            timeout = 10.0
            start_time = time.time()
            while not self.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)

            if self.is_connected:
                self.start_command_monitor()
                
            return self.is_connected
        except Exception as e:
            self.logger.error(f"连接失败: {e}")
            return False

    def start_command_monitor(self):
        """启动命令监控线程"""
        def monitor_loop():
            while self.is_connected and not self.should_exit:
                try:
                    current_time = time.time()
                    expired_commands = []
                    
                    for guid, cmd_info in self.pending_commands.items():
                        if current_time - cmd_info['timestamp'] > self.command_timeout:
                            expired_commands.append(guid)
                    
                    for guid in expired_commands:
                        cmd_info = self.pending_commands.pop(guid, None)
                        if cmd_info and cmd_info['retries'] < self.max_retries:
                            self.retry_command(cmd_info)
                        else:
                            self.logger.warning(f"命令超时: {cmd_info['title'] if cmd_info else guid}")
                    
                    time.sleep(1)
                except Exception as e:
                    self.logger.error(f"命令监控错误: {e}")
                    break
        
        monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        monitor_thread.start()

    def retry_command(self, cmd_info):
        """重试命令"""
        cmd_info['retries'] += 1
        cmd_info['timestamp'] = time.time()
        self.logger.info(f"重试命令: {cmd_info['title']} (第{cmd_info['retries']}次)")
        
        guid = str(uuid.uuid4())
        cmd_info['guid'] = guid
        self.pending_commands[guid] = cmd_info
        
        if cmd_info.get('callback'):
            self.response_callbacks[guid] = cmd_info['callback']
        
        message = {
            "accid": self.accid,
            "title": cmd_info['title'],
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": cmd_info['data']
        }
        
        if self.ws_client and self.is_connected:
            self.ws_client.send(json.dumps(message))

    def on_open(self, ws):
        """WebSocket连接打开回调"""
        self.is_connected = True
        self.logger.info("机器人连接成功")

    def on_message(self, ws, message):
        """WebSocket消息接收回调 - 基于官方协议"""
        try:
            data = json.loads(message)
            title = data.get('title', '')
            
            # 处理响应消息
            if title.startswith('response_'):
                self.handle_response_message(data)
            
            # 处理通知消息
            elif title.startswith('notify_'):
                self.handle_notify_message(data)
            
            self.logger.info(f"收到消息: {data}")

        except json.JSONDecodeError as e:
            self.logger.error(f"JSON解析错误: {e}")

    def handle_response_message(self, data):
        """处理响应消息"""
        guid = data.get('guid')
        if guid:
            # 移除待处理命令
            if guid in self.pending_commands:
                self.pending_commands.pop(guid)
            
            # 执行回调
            if guid in self.response_callbacks:
                callback = self.response_callbacks.pop(guid)
                callback(data)

    def handle_notify_message(self, data):
        """处理通知消息"""
        title = data.get('title', '')
        data_content = data.get('data', {})
        
        if title == 'notify_robot_info':
            # 机器人基本信息（每秒推送）
            self.robot_info.update(data_content)
            self.logger.debug(f"机器人状态更新: 电池{data_content.get('battery', 0)}%")
            
        elif title == 'notify_imu':
            # IMU数据推送
            self.robot_status['imu'] = data_content
            
        elif title == 'notify_odom':
            # 里程计数据推送
            self.robot_status['odom'] = data_content
            
        elif title == 'notify_twist':
            # twist命令失败通知
            result = data_content.get('result', '')
            if result.startswith('fail_'):
                self.logger.error(f"运动控制失败: {result}")
                
        elif title == 'notify_invalid_request':
            # 非法指令通知
            self.logger.error(f"收到非法指令通知: {data_content}")

    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭回调"""
        self.is_connected = False
        self.logger.info("机器人连接已断开")
        # 停止连续运动
        self.stop_continuous_twist()

    def on_error(self, ws, error):
        """WebSocket错误回调"""
        self.logger.error(f"WebSocket错误: {error}")

    def send_request(self, title: str, data: Optional[Dict[str, Any]] = None,
                     callback: Optional[Callable] = None, expect_response: bool = True) -> str:
        """发送WebSocket请求"""
        if data is None:
            data = {}

        guid = str(uuid.uuid4())
        message = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "data": data
        }

        # 只有期望响应的命令才加入待处理列表
        if expect_response:
            cmd_info = {
                'guid': guid,
                'title': title,
                'data': data,
                'timestamp': time.time(),
                'retries': 0,
                'callback': callback
            }
            self.pending_commands[guid] = cmd_info

        if callback and expect_response:
            self.response_callbacks[guid] = callback

        message_str = json.dumps(message)

        if self.ws_client and self.is_connected:
            try:
                self.ws_client.send(message_str)
                self.logger.info(f"发送命令: {title} - {data}")
                return guid
            except Exception as e:
                self.logger.error(f"发送命令失败: {e}")
                if expect_response and guid in self.pending_commands:
                    self.pending_commands.pop(guid)
                return ""
        else:
            self.logger.warning("WebSocket未连接")
            return ""

    # ========== 基础控制命令（基于官方API） ==========
    
    def stand(self, callback=None):
        """站立模式"""
        return self.send_request("request_stand_mode", callback=callback)

    def walk(self, callback=None):
        """行走模式"""
        return self.send_request("request_walk_mode", callback=callback)

    def sit(self, callback=None):
        """蹲下"""
        return self.send_request("request_sitdown", callback=callback)

    def emergency_stop(self, callback=None):
        """紧急停止"""
        return self.send_request("request_emgy_stop", callback=callback)

    # ========== 运动控制（特殊处理） ==========
    
    def twist(self, x=0.0, y=0.0, z=0.0):
        """运动控制 - 基于官方协议（无响应）"""
        data = {"x": x, "y": y, "z": z}
        return self.send_request("request_twist", data, expect_response=False)

    def start_continuous_twist(self, x=0.0, y=0.0, z=0.0):
        """开始连续运动控制"""
        self.current_twist = {"x": x, "y": y, "z": z}
        
        if not self.twist_running:
            self.twist_running = True
            self.twist_thread = threading.Thread(target=self._continuous_twist_loop, daemon=True)
            self.twist_thread.start()
            self.logger.info(f"开始连续运动: X={x}, Y={y}, Z={z}")

    def update_continuous_twist(self, x=0.0, y=0.0, z=0.0):
        """更新连续运动参数"""
        self.current_twist = {"x": x, "y": y, "z": z}
        self.logger.debug(f"更新运动参数: X={x}, Y={y}, Z={z}")

    def stop_continuous_twist(self):
        """停止连续运动"""
        if self.twist_running:
            self.twist_running = False
            # 发送停止命令
            self.twist(0, 0, 0)
            self.logger.info("停止连续运动")

    def _continuous_twist_loop(self):
        """连续运动循环"""
        while self.twist_running and self.is_connected:
            try:
                x = self.current_twist["x"]
                y = self.current_twist["y"] 
                z = self.current_twist["z"]
                
                # 只有在有运动指令时才发送
                if x != 0 or y != 0 or z != 0:
                    self.twist(x, y, z)
                
                time.sleep(self.twist_interval)
            except Exception as e:
                self.logger.error(f"连续运动循环错误: {e}")
                break

    # ========== 新增功能（基于官方API） ==========
    
    def adjust_height(self, direction: int, callback=None):
        """调整机器人身高
        Args:
            direction: 1表示升高，-1表示降低（每次5cm）
        """
        data = {"direction": direction}
        return self.send_request("request_base_height", data, callback)

    def set_stair_mode(self, enable: bool, callback=None):
        """设置楼梯模式"""
        data = {"enable": enable}
        def on_response(response_data):
            if response_data.get('data', {}).get('result') == 'success':
                self.stair_mode_enabled = enable
            if callback:
                callback(response_data)
        
        return self.send_request("request_stair_mode", data, on_response)

    def enable_imu(self, enable: bool, callback=None):
        """开启/关闭IMU数据推送"""
        data = {"enable": enable}
        def on_response(response_data):
            if response_data.get('data', {}).get('result') == 'success':
                self.imu_enabled = enable
            if callback:
                callback(response_data)
        
        return self.send_request("request_enable_imu", data, on_response)

    def enable_odom(self, enable: bool, callback=None):
        """开启/关闭里程计数据推送"""
        data = {"enable": enable}
        def on_response(response_data):
            if response_data.get('data', {}).get('result') == 'success':
                self.odom_enabled = enable
            if callback:
                callback(response_data)
        
        return self.send_request("request_enable_odom", data, on_response)

    def set_light_effect(self, effect: int, callback=None):
        """设置灯光效果
        Args:
            effect: 1-21，对应不同的灯光效果
        """
        data = {"effect": effect}
        def on_response(response_data):
            if response_data.get('data', {}).get('result') == 'success':
                self.current_light_effect = effect
            if callback:
                callback(response_data)
        
        return self.send_request("request_light_effect", data, on_response)

    # ========== 状态查询 ==========
    
    def get_robot_info(self) -> Dict:
        """获取机器人基本信息"""
        return self.robot_info.copy()

    def get_battery_level(self) -> int:
        """获取电池电量"""
        return self.robot_info.get('battery', 0)

    def get_imu_data(self) -> Dict:
        """获取最新IMU数据"""
        return self.robot_status.get('imu', {})

    def get_odom_data(self) -> Dict:
        """获取最新里程计数据"""
        return self.robot_status.get('odom', {})

    def is_stair_mode_enabled(self) -> bool:
        """检查楼梯模式是否启用"""
        return self.stair_mode_enabled

    # ========== 预设动作序列 ==========
    
    def execute_forward_walk(self, duration: float = 3.0):
        """执行前进行走"""
        def sequence():
            self.stand()
            time.sleep(1)
            self.walk()
            time.sleep(0.5)
            self.start_continuous_twist(x=1.0)
            time.sleep(duration)
            self.stop_continuous_twist()
        
        thread = threading.Thread(target=sequence, daemon=True)
        thread.start()
        return thread

    def execute_turn_around(self, direction: str = "left"):
        """执行转身
        Args:
            direction: "left" 或 "right"
        """
        z_val = 1.0 if direction == "left" else -1.0
        
        def sequence():
            self.walk()
            time.sleep(0.5)
            self.start_continuous_twist(z=z_val)
            time.sleep(2.0)  # 转身2秒
            self.stop_continuous_twist()
        
        thread = threading.Thread(target=sequence, daemon=True)
        thread.start()
        return thread

    def disconnect(self):
        """断开连接"""
        self.should_exit = True
        self.stop_continuous_twist()
        
        if self.ws_client:
            self.ws_client.close()
        self.is_connected = False
