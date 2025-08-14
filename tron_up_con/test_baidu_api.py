from aip import AipSpeech
import pyaudio
import wave
import threading
import time
import logging

# 配置日志，方便调试
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 百度语音识别配置（请替换为你的真实信息）
APP_ID = '119601598'
API_KEY = 'hx5nlUBTcu8UcNoNhzXGITE5'
SECRET_KEY = 'Ym6jMiJhUZMRPcoMVDNrd05J7Zyu2T0W'

# 音频参数（百度API要求：16000Hz，单声道，16位深）
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
RECORD_SECONDS = 5  # 检测到指令后录制5秒语音


class BaiduSpeechRecognizer:
    def __init__(self):
        # 初始化百度客户端
        self.client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
        self.is_listening = False  # 是否处于监听状态
        self.is_recording = False  # 是否正在录制目标语音
        self.audio_frame_buffer = []  # 音频帧缓冲区

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """音频回调函数，用于实时缓存音频数据"""
        if self.is_listening or self.is_recording:
            self.audio_frame_buffer.append(in_data)
        return (in_data, pyaudio.paContinue)

    def _detect_start_command(self, audio_data):
        """检测是否包含“开始服务”指令"""
        try:
            # 调用百度API识别短语音
            result = self.client.asr(audio_data, 'wav', RATE, {'dev_pid': 1537})
            if result['err_no'] == 0:
                recognized_text = result['result'][0]
                logger.info(f"检测到语音: {recognized_text}")
                return "开始服务" in recognized_text
            return False
        except Exception as e:
            logger.error(f"指令检测出错: {e}")
            return False

    def _save_wav(self, frames, filename="temp_recording.wav"):
        """将音频帧保存为WAV文件（用于调试）"""
        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(pyaudio.PyAudio().get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        return filename

    def start_listening_loop(self):
        """启动主监听循环"""
        self.is_listening = True
        p = pyaudio.PyAudio()

        # 打开音频流
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
            stream_callback=self._audio_callback
        )

        logger.info("开始监听... 请说“开始服务”以启动录音")

        try:
            # 指令检测循环（每次检测3秒内的音频）
            while self.is_listening:
                # 清空缓冲区，准备接收新音频
                self.audio_frame_buffer = []
                time.sleep(3)  # 每3秒检测一次

                if not self.audio_frame_buffer:
                    continue

                # 将缓存的音频转换为百度API需要的格式
                audio_data = b''.join(self.audio_frame_buffer)

                # 检测是否包含启动指令
                if self._detect_start_command(audio_data):
                    logger.info("检测到“开始服务”指令，准备录音...")
                    self.is_listening = False  # 暂停指令监听
                    self.is_recording = True  # 开始录制目标语音

                    # 录制指定时长的语音
                    self.audio_frame_buffer = []  # 清空缓冲区
                    logger.info(f"开始录音，请在{RECORD_SECONDS}秒内说话...")
                    time.sleep(RECORD_SECONDS)

                    # 停止录音
                    self.is_recording = False
                    logger.info("录音结束，正在识别...")

                    # 处理录制的音频
                    if self.audio_frame_buffer:
                        recording_data = b''.join(self.audio_frame_buffer)
                        # 保存录音（可选，用于调试）
                        self._save_wav(self.audio_frame_buffer)

                        # 调用百度API识别
                        result = self.client.asr(
                            recording_data, 'wav', RATE, {'dev_pid': 1537}
                        )

                        # 解析识别结果
                        if result['err_no'] == 0:
                            recognized_text = result['result'][0]
                            logger.info(f"\n识别成功！你说的是：{recognized_text}")
                        else:
                            logger.error(f"识别失败: {result['err_msg']}")
                    else:
                        logger.warning("未录制到任何音频")

                    # 结束流程
                    self.is_listening = False
                    break

        except KeyboardInterrupt:
            logger.info("用户中断程序")
        except Exception as e:
            logger.error(f"程序出错: {e}")
        finally:
            # 清理资源
            stream.stop_stream()
            stream.close()
            p.terminate()
            logger.info("程序结束")


if __name__ == "__main__":
    # 确保安装必要依赖
    try:
        import pyaudio
    except ImportError:
        logger.error("未检测到pyaudio库，请先安装：pip install pyaudio")
        exit(1)

    recognizer = BaiduSpeechRecognizer()
    recognizer.start_listening_loop()
