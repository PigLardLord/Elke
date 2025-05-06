import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import queue
import sounddevice as sd
from faster_whisper import WhisperModel
import threading
import numpy as np

class WhisperSTTNode(Node):
    def __init__(self):
        super().__init__('stt_whisper')
        self.active = False
        self.msg_pub = self.create_publisher(String, '/speech_input', 10)
        self.activate_sub = self.create_subscription(Bool, '/activate_listening_mode', self.activate_cb, 10)
        self.deactivate_sub = self.create_subscription(Bool, '/deactivate_listening_mode', self.deactivate_cb, 10)

        self.audio_queue = queue.Queue()
        self.model = WhisperModel("base", compute_type="int8")
        self.stream = None
        self.thread = None

    def activate_cb(self, msg):
        if msg.data and not self.active:
            self.get_logger().info('🎙️ Whisper STT activated.')
            self.active = True
            self.thread = threading.Thread(target=self.record)
            self.thread.start()

    def deactivate_cb(self, msg):
        if msg.data and self.active:
            self.get_logger().info('🔇 Whisper STT deactivated.')
            self.active = False
            if self.stream:
                self.stream.stop()
                self.stream.close()
                self.stream = None

    def callback_audio(self, indata, frames, time, status):
        if self.active:
            self.audio_queue.put(bytes(indata))

    def record(self):
        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1, callback=self.callback_audio)
        self.stream.start()
        buffer = b''
        while self.active:
            try:
                buffer += self.audio_queue.get(timeout=1)
                if len(buffer) > 16000 * 5:  # ~5s
                    audio_data = np.frombuffer(buffer, np.int16).astype(np.float32) / 32768.0  # normalize to [-1.0, 1.0]
                    segments, _ = self.model.transcribe(audio_data, beam_size=1, language="it")
                    for seg in segments:
                        msg = String()
                        msg.data = seg.text.strip()
                        self.msg_pub.publish(msg)
                        self.get_logger().info(f'📝 Recognized: {msg.data}')
                    buffer = b''
            except queue.Empty:
                continue
    
def main(args=None):
    rclpy.init(args=args)
    node = WhisperSTTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
