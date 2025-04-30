
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from threading import Timer

class VoiceSessionManager(Node):
    def __init__(self):
        super().__init__('voice_session_manager')
        self.active = False
        self.wake_sub = self.create_subscription(String, '/wake_word_detected', self.wake_cb, 10)
        self.speech_sub = self.create_subscription(String, '/speech_input', self.reset_timer, 10)
        self.answer_sub = self.create_subscription(String, '/ai_answer', self.reset_timer, 10)
        self.action_sub = self.create_subscription(String, '/ai_action', self.reset_timer, 10)

        self.activate_pub = self.create_publisher(Bool, '/activate_listening_mode', 10)
        self.deactivate_pub = self.create_publisher(Bool, '/deactivate_listening_mode', 10)

        self.timeout_sec = 120  # 2 min
        self.timer = None

    def wake_cb(self, msg):
        if msg.data.lower() == 'hey elke' and not self.active:
            self.get_logger().info('🟢 Activating voice session')
            self.active = True
            self.activate_pub.publish(Bool(data=True))
            self.start_timer()

    def start_timer(self):
        if self.timer:
            self.timer.cancel()
        self.timer = Timer(self.timeout_sec, self.expire_session)
        self.timer.start()

    def reset_timer(self, msg):
        if self.active:
            self.get_logger().info('⏱️ Voice session activity, resetting timer')
            self.start_timer()

    def expire_session(self):
        self.get_logger().info('🔴 Voice session timed out. Returning to idle.')
        self.deactivate_pub.publish(Bool(data=True))
        self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = VoiceSessionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
