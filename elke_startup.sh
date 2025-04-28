#!/bin/bash

LOG_FILE=~/elke_startup.log
echo "===== 🚀 Avvio ELKE: $(date) =====" >> "$LOG_FILE"

# 🔁 Attendi che Ollama sia disponibile
echo "🔁 Aspettando che Ollama sia pronto..." | tee -a "$LOG_FILE"
until curl -s http://localhost:11434 > /dev/null; do
  sleep 2
done
echo "✅ Ollama è attivo." | tee -a "$LOG_FILE"

# 🧠 Crea il Modelfile temporaneo con blocco <update> dentro SYSTEM
create_combined_modelfile() {
  COMBINED_FILE=/tmp/Modelfile_combined
  TXT_FILE="/home/master/AI_Models/updates.txt"
  ORIGINAL_FILE="/home/master/Elke/AI/Modelfile"

  # ⚠️ Controllo file esistenti
  if [[ ! -f "$ORIGINAL_FILE" ]]; then
    echo "❌ File Modelfile non trovato: $ORIGINAL_FILE" | tee -a "$LOG_FILE"
    exit 1
  fi

  if [[ ! -f "$TXT_FILE" ]]; then
    echo "⚠️ Nessun file updates.txt trovato, procedo senza memoria persistente." | tee -a "$LOG_FILE"
  fi

  inside_system=0
  > "$COMBINED_FILE"

  while IFS= read -r line || [[ -n "$line" ]]; do
    if [[ "$line" =~ ^SYSTEM\ *\"\"\"$ ]]; then
      inside_system=1
      echo "$line" >> "$COMBINED_FILE"
      continue
    fi

    if [[ $inside_system -eq 1 && "$line" == '"""' ]]; then
      # Inserimento blocco <update> prima della chiusura
      if [[ -s "$TXT_FILE" ]]; then
        echo "" >> "$COMBINED_FILE"
        echo "<update>" >> "$COMBINED_FILE"
        while IFS= read -r upline || [[ -n "$upline" ]]; do
          [[ -z "$upline" ]] && continue
          echo "$upline" >> "$COMBINED_FILE"
        done < "$TXT_FILE"
        echo "</update>" >> "$COMBINED_FILE"
      fi
      inside_system=0
    fi

    echo "$line" >> "$COMBINED_FILE"
  done < "$ORIGINAL_FILE"

  echo "$COMBINED_FILE"
}

# ♻️ Elimina il modello se già esiste
if ollama list | grep -q "elke-gemma"; then
  echo "♻️ Elimino modello elke-gemma per aggiornare memoria..." | tee -a "$LOG_FILE"
  ollama rm elke-gemma >> "$LOG_FILE" 2>&1
fi

# 📦 Ricrea il modello sempre con la memoria aggiornata
echo "📦 Ricreo modello elke-gemma con memoria aggiornata..." | tee -a "$LOG_FILE"
COMBINED_FILE=$(create_combined_modelfile)
if ! ollama create elke-gemma -f "$COMBINED_FILE" >> "$LOG_FILE" 2>&1; then
  echo "❌ Errore nella creazione del modello!" | tee -a "$LOG_FILE"
  exit 1
fi

# 💬 (Ri)avvia sempre il modello dopo averlo rigenerato
if pgrep -f "ollama run elke-gemma" > /dev/null; then
  echo "🛑 Arresto del modello elke-gemma in esecuzione..." | tee -a "$LOG_FILE"
  pkill -f "ollama run elke-gemma"
  sleep 2
fi

echo "💬 Avvio del modello elke-gemma con memoria aggiornata..." | tee -a "$LOG_FILE"
nohup ollama run elke-gemma --format json --system "<idle>" >> "$LOG_FILE" 2>&1 &

# 🧠 Avvia nodo ROS 2
echo "🚀 Avvio nodo ROS AI bridge..." | tee -a "$LOG_FILE"
source /opt/ros/jazzy/setup.bash
source ~/Elke/ros2_ws/install/setup.bash
ros2 run elke-ai elke_ai_bridge >> "$LOG_FILE" 2>&1
