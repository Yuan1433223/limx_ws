import configparser
import os
from typing import Union


class RobotConfig:
    def __init__(self, config_file: str = "robot_config.ini"):
        self.config_file = config_file
        self.config = configparser.ConfigParser()
        self.load_config()

    def load_config(self):
        """加载配置文件"""
        if os.path.exists(self.config_file):
            self.config.read(self.config_file, encoding='utf-8')
        else:
            self.create_default_config()

    def create_default_config(self):
        """创建默认配置"""
        self.config['CONNECTION'] = {
            'host': '10.192.1.2',
            'port': '5000',
                'accid': 'WF_TRON1A_235'
        }

        self.config['CONTROL'] = {
            'default_speed': '1.0',
            'max_twist_value': '1.0',
            'command_timeout': '5.0'
        }

        self.config['LOGGING'] = {
            'level': 'INFO',
            'file': 'logs/robot_control.log'
        }

        self.save_config()

    def save_config(self):
        """保存配置文件"""
        with open(self.config_file, 'w', encoding='utf-8') as f:
            self.config.write(f)

    def get(self, section: str, key: str, fallback: Union[str, None] = None) -> str:
        """获取配置值"""
        return self.config.get(section, key, fallback=fallback)

    def set(self, section: str, key: str, value: str):
        """设置配置值"""
        if section not in self.config:
            self.config.add_section(section)
        self.config.set(section, key, value)
