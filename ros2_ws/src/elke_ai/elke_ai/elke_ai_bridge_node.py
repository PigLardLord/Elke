import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re

class ElkeAIBridge(Node):
    def __init__(self):
        super().__init__('elke_ai_bridge')

        self.subscription = self.create_subscription(
            String,
            'speech_input',
            self.on_speech_input,
            10
        )

        self.pub_thinking = self.create_publisher(String, 'ai_thinking', 10)
        self.pub_answer = self.create_publisher(String, 'ai_answer', 10)
        self.pub_action = self.create_publisher(String, 'ai_action', 10)
        self.pub_update = self.create_publisher(String, 'ai_update', 10)

    def on_speech_input(self, msg):
        user_text = msg.data.strip()
        response = self.query_ollama(user_text)
        self.process_response(response)

    def query_ollama(self, user_text: str) -> str:
        full_prompt = f"<|user|>\n{user_text}\n"
        result = subprocess.run(
            ["ollama", "run", "elke-gemma"],
            input=full_prompt.encode(),
            stdout=subprocess.PIPE
        )
        return result.stdout.decode()

    def process_response(self, text: str):
        def extract(tag):
            match = re.search(f"<{tag}>(.*?)</{tag}>", text, re.DOTALL)
            return match.group(1).strip() if match else ""

        self.pub_thinking.publish(String(data=extract("thinking")))
        self.pub_answer.publish(String(data=extract("answer")))
        action = extract("action")
        if action:
            self.pub_action.publish(String(data=action))
        update = extract("update")
        if update:
            self.pub_update.publish(String(data=update))

def main(args=None):
    rclpy.init(args=args)
    node = ElkeAIBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
