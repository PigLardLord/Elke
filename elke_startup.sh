#!/bin/bash

LOG_FILE=~/elke_startup.log
echo "===== 🚀 Avvio ELKE: $(date) =====" >> "$LOG_FILE"

# 1. Attesa del server Ollama
echo "🔁 Aspettando che Ollama sia pronto..." | tee -a "$LOG_FILE"
until curl -s http://localhost:11434 > /dev/null; do
  sleep 2
done
echo "✅ Ollama è attivo." | tee -a "$LOG_FILE"

# 2. Crea il modello se non esiste
if ! ollama list | grep -q "elke-gemma"; then
  echo "📦 Creo il modello elke-gemma..." | tee -a "$LOG_FILE"
  if ! ollama create elke-gemma -f /home/master/AI_models/Modelfile >> "$LOG_FILE" 2>&1; then
    echo "❌ Errore nella creazione del modello!" | tee -a "$LOG_FILE"
    exit 1
  fi
else
  echo "📦 Modello elke-gemma già presente." | tee -a "$LOG_FILE"
fi

# 3. Controlla se ollama run è già attivo
if pgrep -f "ollama run elke-gemma" > /dev/null; then
  echo "⚠️ Il modello elke-gemma è già in esecuzione." | tee -a "$LOG_FILE"
else
  echo "💬 Avvio del modello elke-gemma..." | tee -a "$LOG_FILE"
  nohup ollama run elke-gemma --format json --system "<idle>" >> "$LOG_FILE" 2>&1 &
fi

# 4. Avvio nodo ROS 2
echo "🧠 Avvio nodo ROS AI bridge..." | tee -a "$LOG_FILE"
source /opt/ros/jazzy/setup.bash
source ~/Elke/ros2_ws/install/setup.bash
ros2 run elke-ai elke_ai_bridge >> "$LOG_FILE" 2>&1
