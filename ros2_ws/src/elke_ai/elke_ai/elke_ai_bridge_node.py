import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class ElkeAIBridge(Node):

    def __init__(self):
        super().__init__('elke_ai_bridge')

        # Subscriber per input vocale trascritto
        self.subscription = self.create_subscription(
            String,
            '/speech_input',
            self.speech_callback,
            10)

        # Publisher per le risposte e azioni
        self.answer_publisher = self.create_publisher(String, '/ai_answer', 10)
        self.update_publisher = self.create_publisher(String, '/ai_update', 10)
        self.action_publisher = self.create_publisher(String, '/ai_action', 10)
        self.thinking_publisher = self.create_publisher(String, '/ai_thinking', 10)

        # Buffer di conversazione
        self.conversation_history = []
        self.max_history = 10  # massimo 10 scambi

    def speech_callback(self, msg):
        user_input = msg.data.strip()

        if not user_input:
            return  # Ignora messaggi vuoti

        self.get_logger().info(f'🗣️ Ricevuto input: {user_input}')

        # Invia subito segnale di "thinking..."
        self.thinking_publisher.publish(String(data="Sto pensando..."))

        # Costruisci il prompt con la cronologia della conversazione
        prompt = self.build_conversation_prompt(user_input)

        # Chiedi a Ollama
        try:
            response = requests.post(
                "http://localhost:11434/api/generate",
                json={
                    "model": "elke-gemma",
                    "prompt": prompt,
                    "stream": False
                }
            )

            response.raise_for_status()
            full_response = response.json().get('response', '')

            self.parse_and_publish_response(full_response)

            # Aggiorna la conversazione
            self.update_conversation(user_input, full_response)

        except Exception as e:
            self.get_logger().error(f"Errore durante la richiesta a Ollama: {e}")

    def build_conversation_prompt(self, latest_input):
        """
        Costruisce il prompt da inviare a Ollama concatenando la cronologia.
        """
        history = ""
        for exchange in self.conversation_history:
            history += f"User: {exchange['user']}\nElke: {exchange['elke']}\n"

        history += f"User: {latest_input}\nElke:"
        return history

    def parse_and_publish_response(self, response):
        """
        Parsifica il testo di risposta in <answer>, <update> e <action> e li pubblica.
        """
        import re

        answer_match = re.search(r"<answer>(.*?)</answer>", response, re.DOTALL)
        update_match = re.search(r"<update>(.*?)</update>", response, re.DOTALL)
        action_match = re.search(r"<action>(.*?)</action>", response, re.DOTALL)

        if answer_match:
            answer = answer_match.group(1).strip()
            if answer:
                self.answer_publisher.publish(String(data=answer))

        if update_match:
            update = update_match.group(1).strip()
            if update:
                self.update_publisher.publish(String(data=update))

        if action_match:
            action = action_match.group(1).strip()
            if action:
                self.action_publisher.publish(String(data=action))

    def update_conversation(self, user_input, full_response):
        """
        Aggiorna la cronologia con l'ultima domanda e risposta.
        """
        answer = self.extract_answer(full_response)

        if answer:
            self.conversation_history.append({
                'user': user_input,
                'elke': answer
            })

        # Mantieni solo gli ultimi N scambi
        if len(self.conversation_history) > self.max_history:
            self.conversation_history.pop(0)

    def extract_answer(self, full_response):
        """
        Estrae solo il testo dentro <answer> per la cronologia.
        """
        import re
        match = re.search(r"<answer>(.*?)</answer>", full_response, re.DOTALL)
        if match:
            return match.group(1).strip()
        return None

def main(args=None):
    rclpy.init(args=args)
    node = ElkeAIBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

