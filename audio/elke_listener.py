# audio/wakeword/elke_listener.py

import pvporcupine
import pyaudio
import struct
import os

def main():
    keyword_path = os.path.join(os.path.dirname(__file__), "elke_raspberry-pi.ppn")

    porcupine = pvporcupine.create(
        keyword_paths=["./elke_raspberry-pi.ppn"]
    ) 
      
    pa = pyaudio.PyAudio()

    stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length
    )

    print("🎤 In ascolto... Pronuncia 'Elke' per attivare il robot.")

    try:
        while True:
            pcm = stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)

            if porcupine.process(pcm) >= 0:
                print("🔔 Wake word 'Elke' rilevata!")
                # Qui potrai avviare la pipeline STT o pubblicare un messaggio ROS
    except KeyboardInterrupt:
        print("🛑 Interrotto manualmente.")
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()
        porcupine.delete()

if __name__ == "__main__":
    main()
