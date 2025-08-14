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


def main():
    """主程序入口"""
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 解析命令行参数
    parser = argparse.ArgumentParser(description="TRON机器人控制系统")
    parser.add_argument('--mode', choices=['cli', 'gui', 'monitor', 'enhanced-monitor'],
                        default='gui', help='运行模式')
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
