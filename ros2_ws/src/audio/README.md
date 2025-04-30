# audio

This ROS 2 package provides the audio interface for the ELKE robot, including:

- 🎤 Wake word detection using [Porcupine](https://picovoice.ai/)
- 🧠 Speech-to-text using [Faster-Whisper](https://github.com/guillaumekln/faster-whisper)
- 🕒 Voice session management with automatic timeout

## Nodes

- `elke_listener`: Listens for the wake word "hey elke" and triggers session start
- `stt_whisper_node`: Performs real-time speech recognition and publishes recognized text
- `voice_session_manager`: Manages session lifecycle and toggles listening modes

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)** license.  
See the [LICENSE](./LICENSE) file for details.

## Disclaimer

This software is provided "as is", **without any warranty of any kind**, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement.  
In no event shall the authors be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from the use of this software.

**Use this software at your own risk.**
