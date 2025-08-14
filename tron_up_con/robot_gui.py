import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from datetime import datetime
import threading
import json
import time


class RobotGUIComplete:
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.root = tk.Tk()

        # 灯光效果映射
        self.light_effects = {
            1: "静态红色", 2: "静态绿色", 3: "静态蓝色", 4: "静态青色",
            5: "静态紫色", 6: "静态黄色", 7: "静态白色",
            8: "低频闪烁红色", 9: "低频闪烁绿色", 10: "低频闪烁蓝色",
            11: "低频闪烁青色", 12: "低频闪烁紫色", 13: "低频闪烁黄色", 14: "低频闪烁白色",
            15: "高频闪烁红色", 16: "高频闪烁绿色", 17: "高频闪烁蓝色",
            18: "高频闪烁青色", 19: "高频闪烁紫色", 20: "高频闪烁黄色", 21: "高频闪烁白色"
        }

        self.setup_ui()
        self.start_status_monitor()

    # 未加入nlp版本setup_ui
    # def setup_ui(self):
    #     """设置完整的用户界面"""
    #     self.root.title("TRON机器人控制台 - 官方API版")
    #     self.root.geometry("1400x900")
    #
    #     # 创建笔记本控件
    #     self.notebook = ttk.Notebook(self.root)
    #     self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    #
    #     # 基础控制页面
    #     self.setup_basic_control_tab()
    #
    #     # 高级控制页面
    #     self.setup_advanced_control_tab()
    #
    #     # 连续运动页面
    #     self.setup_continuous_motion_tab()
    #
    #     # 传感器数据页面
    #     self.setup_sensor_data_tab()
    #
    #     # 状态监控页面
    #     self.setup_status_monitor_tab()

    # nlp版本setup_ui
    def setup_ui(self):
        """设置完整的用户界面"""
        self.root.title("TRON机器人控制台 - 官方API版")
        self.root.geometry("1400x900")

        # 创建笔记本控件
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 基础控制页面
        self.setup_basic_control_tab()

        # 高级控制页面
        self.setup_advanced_control_tab()

        # 连续运动页面
        self.setup_continuous_motion_tab()

        # 传感器数据页面
        self.setup_sensor_data_tab()

        # 状态监控页面
        self.setup_status_monitor_tab()

        # 添加NLP语音控制标签页
        self.setup_nlp_control_tab()

    # 加入nlp后的新增模块
    def setup_nlp_control_tab(self):
        """NLP语音控制标签页"""
        nlp_frame = ttk.Frame(self.notebook)
        self.notebook.add(nlp_frame, text="语音控制")

        # 状态显示
        status_frame = ttk.LabelFrame(nlp_frame, text="语音控制状态")
        status_frame.pack(fill=tk.X, pady=5)

        self.nlp_status_label = ttk.Label(status_frame, text="未初始化", foreground="orange")
        self.nlp_status_label.pack(side=tk.LEFT, padx=10, pady=5)

        # 控制按钮
        control_frame = ttk.Frame(nlp_frame)
        control_frame.pack(fill=tk.X, pady=5)

        self.nlp_start_btn = ttk.Button(
            control_frame,
            text="启动语音控制",
            command=self.start_nlp_control
        )
        self.nlp_start_btn.pack(side=tk.LEFT, padx=5, pady=5)

        ttk.Button(
            control_frame,
            text="停止语音控制",
            command=self.stop_nlp_control
        ).pack(side=tk.LEFT, padx=5, pady=5)

        ttk.Button(
            control_frame,
            text="测试语音识别",
            command=self.test_voice_recognition
        ).pack(side=tk.LEFT, padx=5, pady=5)

        ttk.Button(
            control_frame,
            text="初始化NLP",
            command=self.init_nlp_engine
        ).pack(side=tk.LEFT, padx=5, pady=5)

        # 语音指令历史
        log_frame = ttk.LabelFrame(nlp_frame, text="语音指令历史")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.nlp_log_text = tk.Text(log_frame, height=15, wrap=tk.WORD)
        log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.nlp_log_text.yview)
        self.nlp_log_text.configure(yscrollcommand=log_scrollbar.set)

        self.nlp_log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 初始化NLP引擎 - 修正导入错误
        self.nlp_engine = None

    def init_nlp_engine(self):
        """初始化NLP引擎"""
        try:
            from nlp_module.nlp_controller import NLPController

            self.nlp_engine = NLPController(self.robot)

            # 设置回调函数
            self.nlp_engine.on_status_changed = self.update_nlp_status
            self.nlp_engine.on_speech_recognized = self.log_speech_input
            self.nlp_engine.on_command_received = self.log_command_result

            if self.nlp_engine.initialize():
                self.nlp_status_label.config(text="已初始化", foreground="green")
                self.log_to_nlp("NLP引擎初始化成功")
            else:
                self.nlp_status_label.config(text="初始化失败", foreground="red")
                self.log_to_nlp("NLP引擎初始化失败")

        except Exception as e:
            self.nlp_status_label.config(text="初始化错误", foreground="red")
            self.log_to_nlp(f"NLP引擎初始化错误: {e}")

    def start_nlp_control(self):
        """启动语音控制"""
        if not self.nlp_engine:
            self.log_to_nlp("请先初始化NLP引擎")
            return

        self.nlp_engine.start_voice_control()
        self.nlp_status_label.config(text="正在监听...", foreground="green")
        self.log_to_nlp("语音控制已启动，请说出指令")

    def stop_nlp_control(self):
        """停止语音控制"""
        if self.nlp_engine:
            self.nlp_engine.stop_voice_control()
        self.nlp_status_label.config(text="已停止", foreground="red")
        self.log_to_nlp("语音控制已停止")

    def test_voice_recognition(self):
        """测试语音识别"""
        if not self.nlp_engine:
            self.log_to_nlp("请先初始化NLP引擎")
            return

        self.log_to_nlp("开始语音识别测试，请说话...")

        def test_thread():
            result = self.nlp_engine.test_voice_recognition()
            if result:
                self.log_to_nlp(f"识别结果: {result}")
            else:
                self.log_to_nlp("语音识别测试失败")

        threading.Thread(target=test_thread, daemon=True).start()

    def update_nlp_status(self, status: str):
        """更新NLP状态"""
        self.nlp_status_label.config(text=status)

    def log_speech_input(self, speech_text: str):
        """记录语音输入"""
        self.log_to_nlp(f"🎤 语音输入: {speech_text}")

    def log_command_result(self, command: dict):
        """记录命令结果"""
        if "error" in command:
            self.log_to_nlp(f"❌ 命令错误: {command.get('error', '未知错误')}")
        else:
            action = command.get("action", "未知")
            self.log_to_nlp(f"✅ 执行命令: {action}")

    def log_to_nlp(self, message: str):
        """添加日志到NLP标签页"""
        timestamp = time.strftime("%H:%M:%S")
        self.nlp_log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.nlp_log_text.see(tk.END)  # 滚动到最后
    #

    def setup_basic_control_tab(self):
        """基础控制标签页"""
        basic_frame = ttk.Frame(self.notebook)
        self.notebook.add(basic_frame, text="基础控制")

        # 连接状态
        status_frame = ttk.LabelFrame(basic_frame, text="连接状态")
        status_frame.pack(fill=tk.X, pady=5)

        self.status_label = ttk.Label(status_frame, text="未连接", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10, pady=5)

        self.connect_btn = ttk.Button(status_frame, text="连接", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.RIGHT, padx=10, pady=5)

        # 基础动作控制
        action_frame = ttk.LabelFrame(basic_frame, text="基础动作")
        action_frame.pack(fill=tk.X, pady=5)

        actions = [
            ("站立", self.cmd_stand, "green"),
            ("行走", self.cmd_walk, "blue"),
            ("蹲下", self.cmd_sit, "orange"),
            ("紧急停止", self.cmd_emergency_stop, "red")
        ]

        for i, (text, command, color) in enumerate(actions):
            btn = tk.Button(action_frame, text=text, command=command,
                            bg=color, fg="white", width=10, height=2)
            btn.grid(row=0, column=i, padx=5, pady=5)

        # 楼梯模式控制 - 修复了开关功能
        stair_frame = ttk.LabelFrame(basic_frame, text="楼梯模式控制")
        stair_frame.pack(fill=tk.X, pady=5)

        self.stair_status_label = ttk.Label(stair_frame, text="楼梯模式: 未知")
        self.stair_status_label.pack(side=tk.LEFT, padx=10, pady=5)

        ttk.Button(stair_frame, text="启用楼梯模式",
                   command=lambda: self.cmd_stair_mode(True)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(stair_frame, text="关闭楼梯模式",
                   command=lambda: self.cmd_stair_mode(False)).pack(side=tk.LEFT, padx=5, pady=5)

        # 身高调整
        height_frame = ttk.LabelFrame(basic_frame, text="身高调整")
        height_frame.pack(fill=tk.X, pady=5)

        ttk.Button(height_frame, text="升高 (+5cm)",
                   command=lambda: self.cmd_adjust_height(1)).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(height_frame, text="降低 (-5cm)",
                   command=lambda: self.cmd_adjust_height(-1)).pack(side=tk.LEFT, padx=5, pady=5)

        # 操作日志
        log_frame = ttk.LabelFrame(basic_frame, text="操作日志")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = tk.Text(log_frame, height=15, wrap=tk.WORD)
        log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)

        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def setup_advanced_control_tab(self):
        """高级控制标签页"""
        advanced_frame = ttk.Frame(self.notebook)
        self.notebook.add(advanced_frame, text="高级控制")

        # 灯光效果控制
        light_frame = ttk.LabelFrame(advanced_frame, text="灯光效果控制")
        light_frame.pack(fill=tk.X, pady=5)

        ttk.Label(light_frame, text="选择灯光效果:").pack(side=tk.LEFT, padx=5)

        self.light_effect_var = tk.IntVar(value=1)
        light_combo = ttk.Combobox(light_frame, textvariable=self.light_effect_var, width=20)
        light_combo['values'] = [f"{k}: {v}" for k, v in self.light_effects.items()]
        light_combo.pack(side=tk.LEFT, padx=5)

        ttk.Button(light_frame, text="设置灯效",
                   command=self.cmd_set_light_effect).pack(side=tk.LEFT, padx=5)

        # 传感器控制
        sensor_frame = ttk.LabelFrame(advanced_frame, text="传感器控制")
        sensor_frame.pack(fill=tk.X, pady=5)

        # IMU控制
        imu_frame = ttk.Frame(sensor_frame)
        imu_frame.pack(fill=tk.X, pady=2)

        ttk.Label(imu_frame, text="IMU数据:").pack(side=tk.LEFT, padx=5)
        self.imu_status_label = ttk.Label(imu_frame, text="关闭")
        self.imu_status_label.pack(side=tk.LEFT, padx=5)

        ttk.Button(imu_frame, text="开启IMU",
                   command=lambda: self.cmd_enable_imu(True)).pack(side=tk.LEFT, padx=5)
        ttk.Button(imu_frame, text="关闭IMU",
                   command=lambda: self.cmd_enable_imu(False)).pack(side=tk.LEFT, padx=5)

        # 里程计控制
        odom_frame = ttk.Frame(sensor_frame)
        odom_frame.pack(fill=tk.X, pady=2)

        ttk.Label(odom_frame, text="里程计:").pack(side=tk.LEFT, padx=5)
        self.odom_status_label = ttk.Label(odom_frame, text="关闭")
        self.odom_status_label.pack(side=tk.LEFT, padx=5)

        ttk.Button(odom_frame, text="开启里程计",
                   command=lambda: self.cmd_enable_odom(True)).pack(side=tk.LEFT, padx=5)
        ttk.Button(odom_frame, text="关闭里程计",
                   command=lambda: self.cmd_enable_odom(False)).pack(side=tk.LEFT, padx=5)

        # 预设动作
        preset_frame = ttk.LabelFrame(advanced_frame, text="预设动作")
        preset_frame.pack(fill=tk.X, pady=5)

        presets = [
            ("前进3秒", lambda: self.robot.execute_forward_walk(3.0)),
            ("左转", lambda: self.robot.execute_turn_around("left")),
            ("右转", lambda: self.robot.execute_turn_around("right"))
        ]

        for i, (text, command) in enumerate(presets):
            ttk.Button(preset_frame, text=text, command=command).grid(row=0, column=i, padx=5, pady=5)

    def setup_continuous_motion_tab(self):
        """连续运动控制标签页"""
        motion_frame = ttk.Frame(self.notebook)
        self.notebook.add(motion_frame, text="连续运动")

        # 运动控制说明
        info_frame = ttk.LabelFrame(motion_frame, text="使用说明")
        info_frame.pack(fill=tk.X, pady=5)

        info_text = """
基于官方API特性，twist命令无响应确认，因此采用连续发送模式：
- 开始连续运动后，系统会按100ms间隔持续发送运动指令
- 可实时调整运动参数，无需重新启动
- 停止后会自动发送停止指令(0,0,0)
        """
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack(padx=10, pady=5)

        # XYZ控制
        control_frame = ttk.LabelFrame(motion_frame, text="运动参数控制")
        control_frame.pack(fill=tk.X, pady=5)

        self.x_var = tk.DoubleVar()
        self.y_var = tk.DoubleVar()
        self.z_var = tk.DoubleVar()

        # X轴控制 - 修复语法错误
        x_frame = ttk.Frame(control_frame)
        x_frame.pack(fill=tk.X, pady=2)
        ttk.Label(x_frame, text="X轴(前后):", width=15).pack(side=tk.LEFT)
        ttk.Scale(x_frame, from_=-2.0, to=2.0, variable=self.x_var,
                  orient=tk.HORIZONTAL, length=300, command=self.update_motion).pack(side=tk.LEFT)
        ttk.Label(x_frame, textvariable=self.x_var).pack(side=tk.LEFT, padx=5)

        # Y轴控制 - 修复语法错误
        y_frame = ttk.Frame(control_frame)
        y_frame.pack(fill=tk.X, pady=2)
        ttk.Label(y_frame, text="Y轴(左右):", width=15).pack(side=tk.LEFT)
        ttk.Scale(y_frame, from_=-2.0, to=2.0, variable=self.y_var,
                  orient=tk.HORIZONTAL, length=300, command=self.update_motion).pack(side=tk.LEFT)
        ttk.Label(y_frame, textvariable=self.y_var).pack(side=tk.LEFT, padx=5)

        # Z轴控制 - 修复语法错误
        z_frame = ttk.Frame(control_frame)
        z_frame.pack(fill=tk.X, pady=2)
        ttk.Label(z_frame, text="Z轴(旋转):", width=15).pack(side=tk.LEFT)
        ttk.Scale(z_frame, from_=-2.0, to=2.0, variable=self.z_var,
                  orient=tk.HORIZONTAL, length=300, command=self.update_motion).pack(side=tk.LEFT)
        ttk.Label(z_frame, textvariable=self.z_var).pack(side=tk.LEFT, padx=5)

        # 控制按钮
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(pady=10)

        self.motion_status_label = ttk.Label(button_frame, text="运动状态: 停止")
        self.motion_status_label.pack(side=tk.LEFT, padx=10)

        ttk.Button(button_frame, text="开始连续运动",
                   command=self.start_continuous_motion).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="停止运动",
                   command=self.stop_continuous_motion).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="重置参数",
                   command=self.reset_motion_params).pack(side=tk.LEFT, padx=5)

    def setup_sensor_data_tab(self):
        """传感器数据标签页"""
        sensor_frame = ttk.Frame(self.notebook)
        self.notebook.add(sensor_frame, text="传感器数据")

        # IMU数据显示
        imu_frame = ttk.LabelFrame(sensor_frame, text="IMU数据")
        imu_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.imu_text = tk.Text(imu_frame, height=10, wrap=tk.WORD)
        imu_scrollbar = ttk.Scrollbar(imu_frame, orient=tk.VERTICAL, command=self.imu_text.yview)
        self.imu_text.configure(yscrollcommand=imu_scrollbar.set)

        self.imu_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        imu_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 里程计数据显示
        odom_frame = ttk.LabelFrame(sensor_frame, text="里程计数据")
        odom_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.odom_text = tk.Text(odom_frame, height=10, wrap=tk.WORD)
        odom_scrollbar = ttk.Scrollbar(odom_frame, orient=tk.VERTICAL, command=self.odom_text.yview)
        self.odom_text.configure(yscrollcommand=odom_scrollbar.set)

        self.odom_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        odom_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    def setup_status_monitor_tab(self):
        """状态监控标签页"""
        status_frame = ttk.Frame(self.notebook)
        self.notebook.add(status_frame, text="状态监控")

        # 机器人基本信息
        info_frame = ttk.LabelFrame(status_frame, text="机器人基本信息")
        info_frame.pack(fill=tk.X, pady=5)

        self.battery_label = ttk.Label(info_frame, text="电池: 未知")
        self.battery_label.pack(side=tk.LEFT, padx=10, pady=5)

        self.version_label = ttk.Label(info_frame, text="版本: 未知")
        self.version_label.pack(side=tk.LEFT, padx=10, pady=5)

        # 部件状态
        component_frame = ttk.LabelFrame(status_frame, text="部件状态")
        component_frame.pack(fill=tk.X, pady=5)

        self.imu_component_label = ttk.Label(component_frame, text="IMU: 未知")
        self.imu_component_label.pack(side=tk.LEFT, padx=10, pady=5)

        self.motor_component_label = ttk.Label(component_frame, text="电机: 未知")
        self.motor_component_label.pack(side=tk.LEFT, padx=10, pady=5)

        self.camera_component_label = ttk.Label(component_frame, text="摄像头: 未知")
        self.camera_component_label.pack(side=tk.LEFT, padx=10, pady=5)

        # 详细状态信息
        detail_frame = ttk.LabelFrame(status_frame, text="详细状态")
        detail_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.status_text = tk.Text(detail_frame, wrap=tk.WORD)
        status_scrollbar = ttk.Scrollbar(detail_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=status_scrollbar.set)

        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # ========== 控制命令方法 ==========

    def toggle_connection(self):
        """切换连接状态"""
        if not self.robot.is_connected:
            if self.robot.connect():
                self.log_message("连接成功", "SUCCESS")
            else:
                messagebox.showerror("错误", "连接失败")
        else:
            self.robot.disconnect()
            self.log_message("已断开连接", "INFO")

    def cmd_stand(self):
        """站立命令"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                self.log_message("站立命令执行成功", "SUCCESS")
            else:
                self.log_message(f"站立命令失败: {result}", "ERROR")

        self.robot.stand(callback=on_response)
        self.log_message("发送站立命令", "INFO")

    def cmd_walk(self):
        """行走命令"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                self.log_message("行走模式启用成功", "SUCCESS")
            else:
                self.log_message(f"行走模式启用失败: {result}", "ERROR")

        self.robot.walk(callback=on_response)
        self.log_message("发送行走命令", "INFO")

    def cmd_sit(self):
        """蹲下命令"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                self.log_message("蹲下命令执行成功", "SUCCESS")
            else:
                self.log_message(f"蹲下命令失败: {result}", "ERROR")

        self.robot.sit(callback=on_response)
        self.log_message("发送蹲下命令", "INFO")

    def cmd_emergency_stop(self):
        """紧急停止命令"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                self.log_message("紧急停止执行成功", "SUCCESS")
            else:
                self.log_message(f"紧急停止失败: {result}", "ERROR")

        # 同时停止连续运动
        self.robot.stop_continuous_twist()
        self.robot.emergency_stop(callback=on_response)
        self.log_message("发送紧急停止命令", "WARNING")

    def cmd_stair_mode(self, enable: bool):
        """楼梯模式控制 - 修复版"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                status = "启用" if enable else "关闭"
                self.log_message(f"楼梯模式{status}成功", "SUCCESS")
                self.stair_status_label.config(text=f"楼梯模式: {status}")
            else:
                self.log_message(f"楼梯模式操作失败: {result}", "ERROR")

        self.robot.set_stair_mode(enable, callback=on_response)
        action = "启用" if enable else "关闭"
        self.log_message(f"发送{action}楼梯模式命令", "INFO")

    def cmd_adjust_height(self, direction: int):
        """身高调整命令"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                action = "升高" if direction > 0 else "降低"
                self.log_message(f"身高{action}成功", "SUCCESS")
            else:
                self.log_message(f"身高调整失败: {result}", "ERROR")

        self.robot.adjust_height(direction, callback=on_response)
        action = "升高" if direction > 0 else "降低"
        self.log_message(f"发送{action}身高命令", "INFO")

    def cmd_set_light_effect(self):
        """设置灯光效果"""
        effect = self.light_effect_var.get()

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                effect_name = self.light_effects.get(effect, f"效果{effect}")
                self.log_message(f"灯光效果设置成功: {effect_name}", "SUCCESS")
            else:
                self.log_message(f"灯光效果设置失败: {result}", "ERROR")

        self.robot.set_light_effect(effect, callback=on_response)
        effect_name = self.light_effects.get(effect, f"效果{effect}")
        self.log_message(f"设置灯光效果: {effect_name}", "INFO")

    def cmd_enable_imu(self, enable: bool):
        """IMU控制"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                status = "开启" if enable else "关闭"
                self.log_message(f"IMU数据{status}成功", "SUCCESS")
                self.imu_status_label.config(text=status)
            else:
                self.log_message(f"IMU操作失败: {result}", "ERROR")

        self.robot.enable_imu(enable, callback=on_response)
        action = "开启" if enable else "关闭"
        self.log_message(f"发送{action}IMU命令", "INFO")

    def cmd_enable_odom(self, enable: bool):
        """里程计控制"""

        def on_response(data):
            result = data.get('data', {}).get('result', '')
            if result == 'success':
                status = "开启" if enable else "关闭"
                self.log_message(f"里程计{status}成功", "SUCCESS")
                self.odom_status_label.config(text=status)
            else:
                self.log_message(f"里程计操作失败: {result}", "ERROR")

        self.robot.enable_odom(enable, callback=on_response)
        action = "开启" if enable else "关闭"
        self.log_message(f"发送{action}里程计命令", "INFO")

    # ========== 连续运动控制 ==========

    def start_continuous_motion(self):
        """开始连续运动"""
        x = self.x_var.get()
        y = self.y_var.get()
        z = self.z_var.get()

        self.robot.start_continuous_twist(x, y, z)
        self.motion_status_label.config(text="运动状态: 运行中")
        self.log_message(f"开始连续运动: X={x:.2f}, Y={y:.2f}, Z={z:.2f}", "INFO")

    def stop_continuous_motion(self):
        """停止连续运动"""
        self.robot.stop_continuous_twist()
        self.motion_status_label.config(text="运动状态: 停止")
        self.log_message("停止连续运动", "INFO")

    def update_motion(self, value=None):
        """实时更新运动参数"""
        if self.robot.twist_running:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            self.robot.update_continuous_twist(x, y, z)

    def reset_motion_params(self):
        """重置运动参数"""
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        self.z_var.set(0.0)
        if self.robot.twist_running:
            self.robot.update_continuous_twist(0, 0, 0)
        self.log_message("运动参数已重置", "INFO")

    # ========== 状态监控 ==========

    def start_status_monitor(self):
        """启动状态监控"""

        def monitor_loop():
            while True:
                try:
                    self.update_connection_status()
                    self.update_robot_info()
                    self.update_sensor_data()
                    time.sleep(1)
                except:
                    break

        monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        monitor_thread.start()

    def update_connection_status(self):
        """更新连接状态"""
        if self.robot.is_connected:
            self.status_label.config(text="已连接", foreground="green")
            self.connect_btn.config(text="断开")
        else:
            self.status_label.config(text="未连接", foreground="red")
            self.connect_btn.config(text="连接")

    def update_robot_info(self):
        """更新机器人基本信息"""
        robot_info = self.robot.get_robot_info()
        if robot_info:
            battery = robot_info.get('battery', 0)
            self.battery_label.config(text=f"电池: {battery}%")

            version = robot_info.get('sw_version', '未知')
            self.version_label.config(text=f"版本: {version}")

            # 更新部件状态
            imu_status = robot_info.get('imu', '未知')
            self.imu_component_label.config(text=f"IMU: {imu_status}")

            motor_status = robot_info.get('motor', '未知')
            self.motor_component_label.config(text=f"电机: {motor_status}")

            camera_status = robot_info.get('camera', '未知')
            self.camera_component_label.config(text=f"摄像头: {camera_status}")

            # 更新详细状态
            status_info = json.dumps(robot_info, indent=2, ensure_ascii=False)
            self.status_text.delete(1.0, tk.END)
            self.status_text.insert(1.0, status_info)

    def update_sensor_data(self):
        """更新传感器数据"""
        # 更新IMU数据
        imu_data = self.robot.get_imu_data()
        if imu_data:
            imu_info = json.dumps(imu_data, indent=2, ensure_ascii=False)
            self.imu_text.delete(1.0, tk.END)
            self.imu_text.insert(1.0, imu_info)

        # 更新里程计数据
        odom_data = self.robot.get_odom_data()
        if odom_data:
            odom_info = json.dumps(odom_data, indent=2, ensure_ascii=False)
            self.odom_text.delete(1.0, tk.END)
            self.odom_text.insert(1.0, odom_info)

    def log_message(self, message: str, level: str = "INFO"):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        color_map = {"INFO": "black", "SUCCESS": "green", "WARNING": "orange", "ERROR": "red"}
        color = color_map.get(level, "black")

        self.log_text.insert(tk.END, f"[{timestamp}] [{level}] {message}\n")

        # 设置颜色
        if level != "INFO":
            start_index = self.log_text.index(f"end-2c linestart")
            end_index = self.log_text.index(f"end-1c")
            self.log_text.tag_add(level, start_index, end_index)
            self.log_text.tag_config(level, foreground=color)

        self.log_text.see(tk.END)

    def run(self):
        """运行GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        """窗口关闭处理"""
        if self.robot.is_connected:
            self.robot.disconnect()
        self.root.destroy()
