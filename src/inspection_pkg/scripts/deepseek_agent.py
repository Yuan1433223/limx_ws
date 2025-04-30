import speech_recognition as sr
import rospy
from limxsdk.datatypes import RobotCmd
from openai import OpenAI

from deepseek_prompt import robot_order_template


class DeepSeekAgent:
    def __init__(self, api_key, api_url):
        self.api_key = api_key
        self.api_url = api_url
        self.robot = None
        self.client = OpenAI(api_key=self.api_key, base_url=self.api_url)

    def set_robot(self, robot):
        self.robot = robot

    def listen_for_command(self):
        r = sr.Recognizer()
        with sr.Microphone() as source:
            rospy.loginfo("正在聆听，请说话...")
            audio = r.listen(source)
        try:
            text = r.recognize_google(audio, language='zh-CN')
            rospy.loginfo(f"识别到语音指令: {text}")
            return text
        except sr.UnknownValueError:
            rospy.logerr("无法识别语音")
            return None
        except sr.RequestError as e:
            rospy.logerr(f"请求错误; {e}")
            return None

    def call_deepseek(self, user_prompt, question):
        system_prompt = """你是一个机器人指令转换助手，需要将人类自然语言（普通话）指令转换为机器人可执行的命令。输出格式为一个列表，列表中的元素为指令字符串，如 ['move_forward(0.2, 2.0)', 'turn(0.5, 0.3)']。只输出命令列表，不添加额外的解释。

当接收到机器人的反馈信息时，你需要根据反馈信息判断下一步的操作，并输出相应的命令列表。
如果不需要进一步操作，输出空列表 []。
"""
	user_prompt = robot_order_template + question

        try:
            response = self.client.chat.completions.create(
                model="deepseek-chat",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=1024,
                temperature=0.7,
                stream=False
            )
            commands = response.choices[0].message.content
            return eval(commands)
        except Exception as e:
            rospy.logerr(f"调用DeepSeek API出错: {e}")
            return []

    def execute_commands(self, commands):
        if self.robot is None:
            rospy.logerr("机器人未设置")
            return
        for command in commands:
            try:
                exec(f"self.robot.{command}")
                rospy.loginfo(f"执行命令: {command}")
            except Exception as e:
                rospy.logerr(f"执行命令 {command} 出错: {e}")

    def run(self):
        while not rospy.is_shutdown():
            command_text = self.listen_for_command()
            if command_text:
                commands = self.call_deepseek(command_text)
                self.execute_commands(commands)
    