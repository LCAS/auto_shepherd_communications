#!/usr/bin/env bash
set -euo pipefail

PAIR_A=/dev/ttyVserver          # ← the two names you’ll map in docker‑compose
PAIR_B=/dev/ttyVrobot
PIDFILE=/tmp/socat-ttyV.pid    # store PID so we can detect/reuse it next attach

# 1. Make sure socat is installed on the host
if ! command -v socat &>/dev/null; then
  echo "[pre-attach] Installing socat ..."
  sudo apt-get update -qq
  sudo apt-get install -y socat
fi

# 2. If the pair isn’t running, (re)launch it
if [[ -f "$PIDFILE" ]] && ps -p "$(cat "$PIDFILE")" &>/dev/null; then
  echo "[pre-attach] socat already running (pid $(cat "$PIDFILE"))."
else
  echo "[pre-attach] Starting virtual null‑modem $PAIR_A <-> $PAIR_B ..."
  # Kill any stale socat that might still hold the link names
  sudo pkill -f "PTY,link=$PAIR_A" || true
  sudo pkill -f "PTY,link=$PAIR_B" || true

  sudo socat -d -d \
        PTY,link="$PAIR_A",raw,echo=0 \
        PTY,link="$PAIR_B",raw,echo=0 \
        &>/tmp/socat-ttyV.log  &
  echo $! | sudo tee "$PIDFILE" >/dev/null
  sleep 0.5                     # allow nodes to appear

  # 3. Relax permissions so containers can open them as a non‑root user
  sudo chmod 666 "$PAIR_A" "$PAIR_B"
  echo "[pre-attach] Devices ready."
fi
