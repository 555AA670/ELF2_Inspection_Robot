#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 导入 sherpa_onnx
try:
    import sherpa_onnx
except ImportError:
    print("请先安装 sherpa_onnx: pip3 install sherpa-onnx")
    sys.exit(1)

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_agent_node')
        
        # 配置 Piper 模型
        self.model_dir = "/root/vits-piper-zh_CN-huayan-medium"
        self.model_file = os.path.join(self.model_dir, "zh_CN-huayan-medium.onnx")
        self.tokens_file = os.path.join(self.model_dir, "tokens.txt")
        self.data_dir = os.path.join(self.model_dir, "espeak-ng-data")
        
        if not os.path.exists(self.model_file):
            self.get_logger().error(f"找不到模型文件: {self.model_file}，请确保模型已解压到 /root 目录下")
            sys.exit(1)
            
        self.get_logger().info("正在初始化 Piper 语音合成大模型...")
        
        # 初始化 TTS 引擎
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=self.model_file,
                    tokens=self.tokens_file,
                    data_dir=self.data_dir,
                ),
                provider="cpu",
                debug=False,
                num_threads=2,
            ),
            max_num_sentences=1,
        )
        self.tts = sherpa_onnx.OfflineTts(tts_config)
        self.get_logger().info("✅ 语音合成引擎初始化完成！等待接收大模型文本...")
        
        # 订阅大模型的最终输出
        self.subscription = self.create_subscription(
            String,
            '/tts/text',
            self.llm_callback,
            10
        )
        
        # 为了不阻塞 ROS spin，使用独立线程播放音频
        self.play_queue = []
        self.play_thread = None
        self.is_playing = False

    def llm_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return
            
        # 过滤掉不需要朗读的内部标签（比如 <final>）
        text = text.replace("<final>", "").replace("</final>", "").strip()
        if not text:
            return
            
        self.get_logger().info(f"收到文本准备朗读: {text}")
        
        # 启动后台播放线程（防止阻塞回调函数）
        self.play_queue.append(text)
        if self.play_thread is None or not self.play_thread.is_alive():
            self.is_playing = True
            self.play_thread = threading.Thread(target=self.process_play_queue)
            self.play_thread.start()
            
    def process_play_queue(self):
        while self.play_queue:
            full_text = self.play_queue.pop(0)
            
            # 将整段文本按标点符号切分成小句，实现“伪流式”播放，极大地减少首字等待时间
            import re
            sentences = re.split(r'([。！？；,，])', full_text)
            sentences.append("")
            sentences = ["".join(i) for i in zip(sentences[0::2], sentences[1::2])]
            
            # 打开底层硬件声卡管道
            player = subprocess.Popen(
                ["aplay", "-D", "plughw:rockchipnau8822", "-f", "FLOAT_LE", "-r", "22050", "-c", "1"],
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL
            )
            
            for sentence in sentences:
                sentence = sentence.strip()
                if not sentence:
                    continue
                    
                # 开始合成
                audio = self.tts.generate(sentence, sid=0, speed=1.0)
                if audio is None or len(audio.samples) == 0:
                    continue
                    
                # 直接将生成的音频推入管道，边生成边播
                audio_data = np.array(audio.samples, dtype=np.float32)
                try:
                    player.stdin.write(audio_data.tobytes())
                    player.stdin.flush()
                except Exception as e:
                    self.get_logger().error(f"写入音频数据失败: {e}")
                    break
                    
            # 播放完毕，关闭管道
            if player and player.stdin:
                player.stdin.close()
                player.wait()
                
        self.is_playing = False

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
