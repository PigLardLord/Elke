# audio/wakeword/elke_listener.py

### elke_voice/wake_listener_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pvporcupine
import pyaudio
import struct
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory
import os
import threading
import numpy as np

class WakeWordListener(Node):
    def __init__(self):
        super().__init__('wake_listener')
        self.publisher_ = self.create_publisher(String, '/wake_word_detected', 10)
        load_dotenv(dotenv_path=os.path.expanduser("~/.env_elke"))
        self.ACCESS_KEY = os.getenv("PORCUPINE_KEY")
        self.running = True
        self.thread = threading.Thread(target=self.detect_wake_word)
        self.thread.start()
    
    def detect_wake_word(self):
        model_path = os.path.join(
            get_package_share_directory('audio'),
            'resources',
            'porcupine_params_it.pv'
        )

        keyword_path = os.path.join(
            get_package_share_directory('audio'),
            'resources',
            'hey-elke_it_raspberry-pi_v3_0_0.ppn'
        )

        self.porcupine = pvporcupine.create(
            access_key=self.ACCESS_KEY,
            keyword_paths=[keyword_path],
            model_path=model_path
        )

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length)

        self.get_logger().info('🔉 Wake word listener started...')

        while self.running:
            pcm = self.stream.read(self.porcupine.frame_length, exception_on_overflow=False)
            pcm = np.frombuffer(pcm, dtype=np.int16)
            result = self.porcupine.process(pcm)
            if result >= 0:
                self.publisher_.publish(String(data='hey elke'))

        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        self.porcupine.delete()

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
