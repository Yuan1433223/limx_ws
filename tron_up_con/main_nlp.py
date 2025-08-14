#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import signal

# 导入本地模块 - 使用正确的类名
from robot_controller import RobotController
from robot_config import RobotConfig
from robot_gui import RobotGUIComplete
from robot_monitor import RobotMonitor
from robot_data_logger import RobotDataLogger


def create_directories():
    """创建必要的目录"""
    dirs = ['logs']
    for dir_name in dirs:
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
            print(f"创建目录: {dir_name}")


def signal_handler(signum, frame):
    """信号处理器 - 优雅退出"""
    print("\n🛑 接收到退出信号，正在安全退出...")
    sys.exit(0)


def run_enhanced_monitor(robot):
    """运行增强版监控模式"""
    try:
        # 创建数据记录器
        data_logger = RobotDataLogger(robot)

        # 创建监控器 - 使用正确的构造函数参数
        monitor = RobotMonitor(robot, data_logger)

        # 启动数据记录
        data_logger.start_logging()

        print("📊 增强数据记录系统已启动")
        print("💾 数据将每5秒自动保存到logs文件夹")
        print("🖥️ 终端将每秒显示实时状态")
        print("📈 即将启动可视化监控界面...")

        try:
            # 启动可视化监控（阻塞式）
            monitor.start_monitoring()
        except KeyboardInterrupt:
            print("\n🛑 用户中断监控")
        finally:
            # 清理资源
            data_logger.stop_logging()
            monitor.stop_monitoring()

        return True

    except Exception as e:
        print(f"❌ 增强监控启动失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_basic_monitor(robot):
    """运行基础监控模式（兼容原有功能）"""
    try:
        # 创建基础监控器 - 需要创建一个空的data_logger来满足构造函数
        data_logger = RobotDataLogger(robot)
        monitor = RobotMonitor(robot, data_logger)

        try:
            monitor.start_monitoring()
        except KeyboardInterrupt:
            print("监控被用户中断")
        finally:
            monitor.stop_monitoring()

        return True
    except Exception as e:
        print(f"❌ 基础监控启动失败: {e}")
        return False


# main.py 中新增NLP模式处理 语音版
def run_nlp_mode(robot):
    """运行NLP交互模式（语音版）"""
    print("🎤 启动语音NLP控制模式...")
    print("📋 支持指令: 站立、坐下、前进、后退、左转、右转、停止等")
    print("🔊 请确保麦克风已连接并工作正常")

    if not robot.connect():
        print("❌ 机器人连接失败，无法启动NLP模式")
        return False

    try:
        # 导入NLP控制器 - 使用我们设计的模块
        from nlp_module.nlp_controller import NLPController

        print("🧠 正在初始化NLP引擎...")

        # 初始化NLP控制器
        nlp_controller = NLPController(robot)

        # 设置回调函数用于终端输出
        def log_status(status):
            print(f"📊 状态: {status}")

        def log_speech(speech):
            print(f"🎤 识别: {speech}")

        def log_command(command):
            if "error" in command:
                print(f"❌ 错误: {command.get('error', '未知错误')}")
            else:
                action = command.get("action", "未知")
                print(f"✅ 执行: {action}")

        # 绑定回调函数
        nlp_controller.on_status_changed = log_status
        nlp_controller.on_speech_recognized = log_speech
        nlp_controller.on_command_received = log_command

        # 初始化NLP引擎
        if not nlp_controller.initialize():
            print("❌ NLP引擎初始化失败")
            return False

        print("✅ NLP引擎初始化成功")
        print("\n🎯 控制选项:")
        print("  1. 输入 'start' 开始语音控制")
        print("  2. 输入 'stop' 停止语音控制")
        print("  3. 输入 'test' 测试语音识别")
        print("  4. 直接输入文字指令")
        print("  5. 输入 'quit' 退出")
        print("\n等待您的指令...")

        while True:
            try:
                user_input = input("\n指令> ").strip().lower()

                if user_input == 'quit':
                    print("👋 正在退出NLP模式...")
                    break

                elif user_input == 'start':
                    print("🎤 启动语音控制，请说话...")
                    nlp_controller.start_voice_control()

                elif user_input == 'stop':
                    print("⏹️ 停止语音控制")
                    nlp_controller.stop_voice_control()

                elif user_input == 'test':
                    print("🧪 开始语音识别测试，请说话...")
                    result = nlp_controller.test_voice_recognition()
                    if result:
                        print(f"✅ 测试成功，识别结果: {result}")
                    else:
                        print("❌ 语音识别测试失败")

                elif user_input == 'status':
                    status = nlp_controller.get_status()
                    print("📊 当前状态:")
                    print(f"  - 语音控制: {'启动' if status['is_active'] else '停止'}")
                    print(f"  - 处理中: {'是' if status['is_processing'] else '否'}")
                    print(f"  - 模型状态: {'已加载' if status['model_loaded'] else '未加载'}")
                    print(f"  - 监听状态: {'监听中' if status['listening'] else '未监听'}")
                    print(f"  - 机器人连接: {'已连接' if status['robot_connected'] else '未连接'}")

                elif user_input == 'help':
                    print("\n📖 帮助信息:")
                    print("  控制指令:")
                    print("    start  - 开始语音控制")
                    print("    stop   - 停止语音控制")
                    print("    test   - 测试语音识别")
                    print("    status - 查看系统状态")
                    print("    quit   - 退出程序")
                    print("\n  支持的语音指令:")
                    print("    基础动作: 站立、坐下、行走、停止")
                    print("    运动控制: 前进、后退、左转、右转")
                    print("    身高调整: 升高、降低")
                    print("    灯光控制: 红灯、绿灯、蓝灯、闪烁")
                    print("    特殊功能: 楼梯模式、开启传感器等")

                elif user_input:
                    # 处理文字指令
                    print(f"📝 处理文字指令: {user_input}")
                    command = nlp_controller.process_text_command(user_input)
                    # 命令结果会通过回调函数显示

                else:
                    print("❓ 请输入有效指令，输入 'help' 查看帮助")

            except KeyboardInterrupt:
                print("\n🛑 检测到中断信号...")
                break
            except EOFError:
                print("\n🛑 输入结束...")
                break
            except Exception as e:
                print(f"❌ 处理指令时出错: {e}")

        return True

    except ImportError as e:
        print(f"❌ NLP模块导入失败: {e}")
        print("💡 请确保已正确安装NLP模块的所有依赖")
        print("   运行: pip install torch transformers SpeechRecognition pyaudio")
        return False
    except Exception as e:
        print(f"❌ NLP模式启动失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        try:
            if 'nlp_controller' in locals():
                nlp_controller.stop_voice_control()
                print("🔧 NLP控制器已清理")
        except:
            pass
        if robot.is_connected:
            robot.disconnect()
            print("🔌 机器人连接已断开")


def main():
    """主程序入口"""
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 解析命令行参数
    parser = argparse.ArgumentParser(description="TRON机器人控制系统")
    parser.add_argument('--mode', choices=['cli', 'gui', 'monitor', 'enhanced-monitor', 'nlp'],
                        default='gui',
                        help='运行模式 (cli: 命令行, gui: 图形界面, monitor: 基础监控, enhanced-monitor: 增强监控, nlp: 语音控制)')
    parser.add_argument('--config', default='robot_config.ini',
                        help='配置文件路径')
    args = parser.parse_args()

    # 创建必要目录
    create_directories()

    # 加载配置
    try:
        config = RobotConfig(args.config)
        print(f"配置文件加载成功: {args.config}")
    except Exception as e:
        print(f"配置文件加载失败: {e}")
        return 1

    # 创建机器人控制器 - 使用正确的类名和参数
    try:
        robot = RobotController(
            accid=config.get('CONNECTION', 'accid'),
            host=config.get('CONNECTION', 'host'),
            port=int(config.get('CONNECTION', 'port'))
        )
        print(f"机器人控制器创建成功")
    except Exception as e:
        print(f"机器人控制器创建失败: {e}")
        return 1

    # 根据模式运行
    try:
        if args.mode == 'cli':
            print("启动命令行模式...")
            if robot.connect():
                # 简单的命令行接口
                print("机器人已连接，输入 'help' 查看可用命令，输入 'quit' 退出")
                while True:
                    try:
                        cmd = input("robot> ").strip().lower()
                        if cmd == 'quit':
                            break
                        elif cmd == 'help':
                            print("可用命令: stand, walk, sit, stop, status, quit")
                        elif cmd == 'stand':
                            robot.stand()
                            print("发送站立命令")
                        elif cmd == 'walk':
                            robot.walk()
                            print("发送行走命令")
                        elif cmd == 'sit':
                            robot.sit()
                            print("发送蹲下命令")
                        elif cmd == 'stop':
                            robot.emergency_stop()
                            print("发送紧急停止命令")
                        elif cmd == 'status':
                            info = robot.get_robot_info()
                            print(f"机器人状态: {info}")
                        else:
                            print("未知命令，输入 'help' 查看可用命令")
                    except KeyboardInterrupt:
                        break
                    except EOFError:
                        break
            else:
                print("连接失败，无法启动命令行模式")
                return 1

        elif args.mode == 'gui':
            print("启动图形界面模式...")
            # 使用正确的类名
            gui = RobotGUIComplete(robot)
            gui.run()

        elif args.mode == 'monitor':
            print("启动基础数据监控模式...")
            if robot.connect():
                success = run_basic_monitor(robot)
                if not success:
                    print("❌ 基础监控启动失败")
                    return 1
                robot.disconnect()
            else:
                print("连接失败，无法启动监控模式")
                return 1

        elif args.mode == 'enhanced-monitor':
            print("🚀 启动增强数据监控模式...")
            if robot.connect():
                success = run_enhanced_monitor(robot)
                if not success:
                    print("⬇️ 尝试基础监控模式...")
                    success = run_basic_monitor(robot)
                    if not success:
                        print("❌ 所有监控模式都启动失败")
                        return 1

                robot.disconnect()
            else:
                print("连接失败，无法启动增强监控模式")
                return 1

        # 在main函数的模式判断中添加
        elif args.mode == 'nlp':
            print("启动NLP交互模式...")
            run_nlp_mode(robot)

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if robot.is_connected:
            robot.disconnect()

    print("程序退出")
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
