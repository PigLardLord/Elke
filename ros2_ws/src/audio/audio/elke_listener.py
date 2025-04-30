# audio/wakeword/elke_listener.py

### elke_voice/wake_listener_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pvporcupine
import pyaudio
import struct

class WakeWordListener(Node):
    def __init__(self):
        super().__init__('wake_listener')
        self.publisher_ = self.create_publisher(String, '/wake_word_detected', 10)

        self.porcupine = pvporcupine.create(keywords=['hey-elke_it_raspberry-pi_v3_0_0'])
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length)

        self.get_logger().info('🔉 Wake word listener started...')
        self.timer = self.create_timer(0.1, self.listen_loop)

    def listen_loop(self):
        pcm = self.stream.read(self.porcupine.frame_length, exception_on_overflow=False)
        pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)
        keyword_index = self.porcupine.process(pcm)

        if keyword_index >= 0:
            msg = String()
            msg.data = 'hey elke'
            self.publisher_.publish(msg)
            self.get_logger().info('🎤 Wake word detected!')


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
