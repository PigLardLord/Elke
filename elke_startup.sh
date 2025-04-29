#!/bin/bash

LOG_FILE="/home/master/elke_startup.log"

log() {
  echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

log "===== 🚀 Starting ELKE Startup Script ====="

# 🔁 Wait for Ollama to be ready
log "🔁 Waiting for Ollama to be ready..."
until curl -s http://localhost:11434 > /dev/null; do
  sleep 2
done
log "✅ Ollama is ready."

# 🧠 Create combined Modelfile including <update> memory block
create_combined_modelfile() {
  COMBINED_FILE="/tmp/Modelfile_combined"
  TXT_FILE="/home/master/AI_Models/updates.txt"
  ORIGINAL_FILE="/home/master/Elke/AI/Modelfile"

  if [[ ! -f "$ORIGINAL_FILE" ]]; then
    log "❌ Original Modelfile not found: $ORIGINAL_FILE"
    exit 1
  fi

  if [[ ! -f "$TXT_FILE" ]]; then
    log "⚠️ No updates.txt file found, proceeding without persistent memory."
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

# ♻️ Remove model if it already exists
if ollama list | grep -q "elke-gemma"; then
  log "♻️ Removing existing model elke-gemma..."
  ollama rm elke-gemma >> "$LOG_FILE" 2>&1 || log "⚠️ Failed to remove model (probably did not exist)."
fi

# 📦 Recreate the model with updated memory
log "📦 Recreating elke-gemma model with updated memory..."
COMBINED_FILE=$(create_combined_modelfile)
if ! ollama create elke-gemma -f "$COMBINED_FILE" >> "$LOG_FILE" 2>&1; then
  log "❌ Error creating the model!"
  exit 1
fi

# 🛑 Stop any existing ollama run instance
if pgrep -f "ollama run elke-gemma" > /dev/null; then
  log "🛑 Stopping previous instance of elke-gemma..."
  pkill -f "ollama run elke-gemma"
  sleep 2
fi

# 💬 Start new ollama run instance in background
log "💬 Starting ollama run elke-gemma..."
nohup ollama run elke-gemma --format json --system "<idle>" >> "$LOG_FILE" 2>&1 &
sleep 5

log "✅ ELKE Startup Script completed."
